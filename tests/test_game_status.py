"""Status and runtime outage checks without importing main's hardware side effects."""

import ast
import math
import subprocess
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

from lib.game_status import GameStatus, ImuPause, ProgressHealth


class Display:
    def __init__(self):
        self.components = {}
        self.state = None

    def component(self, source, health, error=""):
        self.components[source] = (health, error)

    def update(self, *state):
        self.state = state


class GameStatusTests(unittest.TestCase):
    def setUp(self):
        self.now = 0
        self.scans = 0
        self.cleared = 0
        self.display = Display()
        lidar = SimpleNamespace(get_scan_generation=lambda: self.scans,
                                clear_imu_yaw=self.clear_prior)
        self.status = GameStatus(self.display, lidar, "STRIKER", clock=lambda: self.now)
        self.camera = SimpleNamespace(capture_count=0, infer_count=0, inference_error=None,
                                      recording_info={})
        self.health = {"imu_recovery_generation": 0, "imu_healthy": True,
                       "error": "", "fault_source": ""}
        self.status.attach_hardware(SimpleNamespace(health=lambda: self.health))
        self.status.attach_camera(self.camera)

    def clear_prior(self):
        self.cleared += 1

    def fresh(self):
        self.scans += 1
        self.camera.capture_count += 1
        self.camera.infer_count += 1

    def test_runtime_disconnects_are_independent_of_run(self):
        self.status.poll()
        self.assertEqual(self.display.components["LIDAR"][0], "?")
        self.fresh()
        self.status.update("STRIKER", True)
        self.assertTrue(self.status.poll())
        self.now = 1.01
        self.assertFalse(self.status.poll())
        self.assertEqual(self.display.components["LIDAR"][0], "!")
        self.assertEqual(self.display.components["CAMERA"][0], "!")
        self.assertEqual(self.display.state[:3], ("STRIKER", True, "RUNNING"))
        self.status.update("GOALIE", False)
        self.status.poll()
        self.assertEqual(self.display.state[:3], ("GOALIE", False, "PAUSED"))
        self.fresh()
        self.assertTrue(self.status.poll())
        self.assertEqual(self.display.components["LIDAR"], ("+", ""))

    def test_capture_alive_inference_dead(self):
        self.fresh()
        self.status.poll()
        self.now = 2
        self.camera.capture_count += 1
        self.assertFalse(self.status.poll())
        self.assertIn("INFERENCE", self.display.components["CAMERA"][1])
        self.camera.infer_count += 1
        self.camera.inference_error = "Hailo failure"
        self.assertFalse(self.status.poll())
        self.assertEqual(self.display.components["CAMERA"][1], "Hailo failure")

    def test_imu_loss_uses_paused_run_state(self):
        pause = ImuPause(0)
        self.assertTrue(pause.update(True, self.health))
        self.health.update(imu_healthy=False, imu_recovery_generation=1)
        run = pause.update(True, self.health)
        self.assertFalse(run)
        self.status.update("DEFENCE", run)
        self.status.poll()
        self.assertEqual(self.display.state[:3], ("DEFENCE", False, "PAUSED"))
        self.assertEqual(self.cleared, 1)
        self.health["imu_healthy"] = True
        self.assertFalse(pause.update(True, self.health))
        pause.rezeroed(True, self.health)  # Operator has not paused yet.
        self.assertFalse(pause.update(True, self.health))
        self.assertFalse(pause.update(False, self.health))
        pause.rezeroed(False, self.health)
        self.assertTrue(pause.update(True, self.health))

    def test_new_outage_during_rezero_requires_another_pause(self):
        pause = ImuPause(0)
        self.health["imu_recovery_generation"] = 1
        self.assertFalse(pause.update(True, self.health))  # Outage between polls.
        self.health["imu_recovery_generation"] = 2
        pause.rezeroed(False, self.health)
        self.assertFalse(pause.update(True, self.health))

    def test_startup_and_other_errors(self):
        self.status.update("GOALIE", False, "STARTING", "FIRST POSE")
        self.status.poll()
        self.assertEqual(self.display.state[2:], ("STARTING", "FIRST POSE"))
        self.status.report("OTHER", "unexpected exception")
        self.camera.recording_info = {"recording_error": "disk full"}
        self.status.attach_recording(SimpleNamespace(
            game_writer=SimpleNamespace(error=OSError("game disk full")),
            detection_writer=SimpleNamespace(error=OSError("detection disk full"))))
        self.health.update(error="Motor 26: no response", fault_source="MOTOR")
        self.status.poll()
        for source in ("OTHER", "RECORDING", "MOTOR", "GAME LOG", "DETECTION LOG"):
            self.assertEqual(self.display.components[source][0], "!")

    def test_startup_grace_is_not_renewed_by_polling(self):
        tracker = ProgressHealth(0)
        self.assertEqual(tracker.update(0, 0.5), "?")
        self.assertEqual(tracker.update(0, 1), "!")
        self.assertEqual(tracker.update(1, 1.1), "+")

    def test_native_display_protocol(self):
        root = Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory() as directory:
            executable = str(Path(directory) / "display-test")
            subprocess.run([
                "g++", "-std=c++17", "-O2", "-Wall", "-Wextra", "-Werror", "-pthread",
                "-I", str(root / "lib"), str(root / "tests/status_display_native.cpp"),
                str(root / "lib/status_display.cpp"), str(root / "lib/linux_wire.cpp"),
                "-o", executable,
            ], check=True)
            subprocess.run([executable], check=True, timeout=10)


class MainCameraFallbackTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Execute the real camera/possession/peer section without hardware startup.
        source = ast.parse((Path(__file__).resolve().parents[1] / "main.py").read_text())
        running = next(node for node in ast.walk(source) if isinstance(node, ast.If)
                       and isinstance(node.test, ast.Name) and node.test.id == "run"
                       and any(isinstance(child, ast.Assign)
                               and isinstance(child.value, ast.Call)
                               and isinstance(child.value.func, ast.Attribute)
                               and child.value.func.attr == "get_scene_measurement"
                               for child in node.body))
        start = next(i for i, node in enumerate(running.body)
                     if isinstance(node, ast.Assign) and isinstance(node.value, ast.Call)
                     and isinstance(node.value.func, ast.Attribute)
                     and node.value.func.attr == "get_scene_measurement")
        end = next(i for i, node in enumerate(running.body)
                   if isinstance(node, ast.If) and "bot_mode" in ast.unparse(node.test))
        cls.section = compile(ast.Module(body=running.body[start:end], type_ignores=[]),
                              "main-camera-section", "exec")

    def scene(self, *, captured=False, peer_ball=None, last_update=0, ready=False, yaw=0):
        peer = SimpleNamespace(send=lambda _: None, receive=lambda: peer_ball)
        values = {
            "camera": SimpleNamespace(get_scene_measurement=lambda: (5, 0, 999, [(0, 900)], True)),
            "status": SimpleNamespace(camera_ready=ready), "last_camera_frame_id": 4,
            "last_camera_bot_positions": [(999, 999)], "x_pos": 100, "y_pos": 200,
            "yaw": yaw, "math": math, "time": SimpleNamespace(time=lambda: 10),
            "last_ball_x": 300, "last_ball_y": 400, "ball_dx": 10, "ball_dy": 0,
            "last_ball_update": last_update, "BALL_TIMEOUT": 0.5,
            "break_beam": SimpleNamespace(read=lambda: captured), "peer": peer,
            "bot_mode": SimpleNamespace(name="STRIKER"),
            "classify_camera_bot_positions": lambda bots, *args, **kwargs: ([], bots),
        }
        exec(self.section, values)  # noqa: S102 - execute local main.py statements with fake inputs
        return values

    def test_disconnected_camera_discards_stale_scene(self):
        values = self.scene()
        self.assertIsNone(values["ball_x"])
        self.assertIsNone(values["ball_y"])
        self.assertEqual(values["enemy_bot_positions"], [])
        self.assertFalse(values["lined_up"])

    def test_disconnected_camera_uses_peer_ball(self):
        values = self.scene(peer_ball={"ball_x": 700, "ball_y": 800})
        self.assertEqual((values["ball_x"], values["ball_y"]), (700, 800))

    def test_break_beam_possession_takes_precedence(self):
        values = self.scene(captured=True, peer_ball={"ball_x": 700, "ball_y": 800})
        self.assertTrue(values["ball_captured"])
        self.assertEqual((values["ball_x"], values["ball_y"]), (200, 200))

    def test_existing_short_extrapolation_is_preserved(self):
        values = self.scene(last_update=9.8)
        self.assertAlmostEqual(values["ball_x"], 302)
        self.assertEqual(values["ball_y"], 400)

    def test_fresh_camera_scene_is_used(self):
        values = self.scene(ready=True)
        self.assertEqual((values["ball_x"], values["ball_y"]), (1099, 200))
        self.assertEqual(values["enemy_bot_positions"], [(1000, 200)])
        self.assertTrue(values["lined_up"])

    def test_camera_scene_is_rotated_by_robot_yaw(self):
        values = self.scene(ready=True, yaw=90)
        self.assertAlmostEqual(values["ball_x"], 100)
        self.assertAlmostEqual(values["ball_y"], 1199)
        self.assertAlmostEqual(values["enemy_bot_positions"][0][0], 100)
        self.assertAlmostEqual(values["enemy_bot_positions"][0][1], 1100)
