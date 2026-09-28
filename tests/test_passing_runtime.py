"""Fake-device runner checks: real passing code, no native/GPIO/network access."""

import sys
import time
import unittest
from contextlib import ExitStack
from types import ModuleType, SimpleNamespace
from unittest.mock import patch

import lib
from lib.passing import PROTOCOL
from tests.passing import run_passing_test


class PassingRuntimeTests(unittest.TestCase):
    def run_fake(self, camera_failure=False):
        self.events = []
        self.commands = []
        self.packets = []
        self.references = []
        self.hardware_options = {}
        pose = (500, 910, 90, 0.9)
        sequence = 0
        scene_count = 0

        def scene():
            nonlocal scene_count
            scene_count += 1
            return {"frame_id": scene_count, "timestamp_s": time.monotonic() - 0.01,
                    "bots": [{"bearing_deg": -90, "distance_mm": 1000, "confidence": 0.9}]}

        def receive():
            nonlocal sequence
            sequence += 1
            return {"received_at": time.monotonic(), "message": {
                "protocol": PROTOCOL, "session": "test", "sequence": sequence,
                "bot_id": "b", "pose": (1500, 910, 180), "has_ball": False,
                "observation_age_s": 0.01, "gyro_deg_s": 0,
            }}

        hardware = SimpleNamespace(
            health=lambda: {"imu_healthy": True, "imu_recovery_generation": 1},
            set_startup_yaw=self.references.append,
            get_yaw=lambda: 45, get_gyro_z_deg_s=lambda: 0,
            move=lambda *args, **kwargs: self.commands.append((args, kwargs)),
            stop=lambda: self.events.append("motors"),
        )
        def start_camera():
            if camera_failure:
                raise RuntimeError("fake camera startup failed")
        camera = SimpleNamespace(start=start_camera, start_stream=start_camera,
                                 get_scene_snapshot=scene, inference_error=None,
                                 stop=lambda: self.events.append("camera"))
        peer = SimpleNamespace(start=lambda: None, bot_id="a", receive_snapshot=receive,
                               send=lambda value: self.packets.append(dict(value)),
                               stop=lambda: self.events.append("peer"))
        beam = SimpleNamespace(read=lambda: True,
                               switch=SimpleNamespace(deinit=lambda: self.events.append("beam")))
        lidar = SimpleNamespace(
            init=lambda *_: None, start_coordinates=lambda *_, **__: None,
            shutdown=lambda: self.events.append("lidar"), set_imu_yaw=lambda *_: None,
            get_pose=lambda: pose,
            get_deskew_status=lambda: {"accepted": True, "scan_time_s": time.monotonic() - 0.01},
            get_scan_history=lambda t: [{"time_s": t, "received_s": t, "points": []}],
            fusion_context=lambda _, t, __: {"reason": "ok", "pose": pose,
                                              "points": [(3, -950, t, 30, 400), (-3, -950, t, 30, 400)]},
        )
        def create_hardware(**options):
            self.hardware_options.update(options)
            return hardware
        modules = {}
        attrs = {
            "lib.break_beam": {"Breakbeam": lambda *_: beam},
            "lib.camera": {"Camera": lambda *_, **__: camera},
            "lib.communication": {"Peer": lambda *_, **__: peer},
            "lib.config": {"load_config": lambda: SimpleNamespace(
                i2c_addresses=[1, 2, 3, 4, 5], break_beam_pin=16, lidar_port="fake")},
            "lib.hardware_test_utils": {"create_hardware": create_hardware,
                                         "set_startup_yaw": lambda _: 10},
            "lib.localisation_motion": {
                "configure_motion": lambda *_, **__: None, "feed_timed_motion": lambda *_: None,
                "close_motion_capture": lambda *_: self.events.append("motion"),
                "MotionCapture": lambda *_: None},
        }
        for name, fields in attrs.items():
            module = ModuleType(name)
            module.__dict__.update(fields)
            modules[name] = module
        with ExitStack() as stack:
            stack.enter_context(patch.dict(sys.modules, modules))
            stack.enter_context(patch.object(lib, "lidar", lidar, create=True))
            run_passing_test(session="test", startup_heading=30, duration=0.045)

    def test_rotation_frame_conversion_and_cleanup(self):
        self.run_fake()
        self.assertEqual(self.references, [40])
        self.assertTrue(self.hardware_options["use_pcb"])
        self.assertTrue(self.hardware_options["kicker"])
        self.assertGreaterEqual(len(self.commands), 2)
        for args, kwargs in self.commands:
            self.assertEqual(args[:2], (0, 0))
            # Desired field heading 0, current field yaw 90, IMU yaw 45:
            # native command must be -45, not the field heading 0.
            self.assertAlmostEqual(args[2], -45)
            self.assertEqual(args[4], 1)
            self.assertFalse(kwargs["kick"])
        self.assertEqual(self.events, ["motors", "camera", "peer", "beam", "motion", "lidar"])
        self.assertTrue(all(p["intent"] for p in self.packets))

    def test_initialisation_failure_still_stops_all_open_devices(self):
        with self.assertRaisesRegex(RuntimeError, "camera startup"):
            self.run_fake(camera_failure=True)
        self.assertEqual(self.commands, [])
        self.assertEqual(self.events, ["motors", "camera", "beam", "motion", "lidar"])

    def test_invalid_arguments_do_not_import_hardware(self):
        for kwargs in [{"session": ""}, {"session": "ok", "duration": -1},
                       {"session": "ok", "startup_heading": float("nan")}]:
            with self.assertRaises(ValueError):
                run_passing_test(**kwargs)
