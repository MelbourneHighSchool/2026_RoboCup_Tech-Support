"""Offline tests. Build lib.hardware_controller first with lib/setup.py."""

import ast
import json
import math
import subprocess
import tempfile
import unittest
from collections import deque
from pathlib import Path
from types import SimpleNamespace

from lib.hardware_controller import HardwareController

USE_PCB = False

ROOT = Path(__file__).resolve().parents[1]


class HardwareControllerTests(unittest.TestCase):
    def test_native_motor_protocol_and_lifecycle(self):
        with tempfile.TemporaryDirectory() as directory:
            executable = str(Path(directory) / "hardware-test")
            objects = []
            for name in ("sh2", "shtp", "sh2_SensorValue", "sh2_util"):
                obj = str(Path(directory) / f"{name}.o")
                subprocess.run(
                    ["gcc", "-std=c11", "-O2", "-c", str(ROOT / "lib/imu" / f"{name}.c"),
                     "-o", obj], check=True,
                )
                objects.append(obj)
            subprocess.run(
                ["g++", "-std=c++17", "-O2", "-Wall", "-Wextra", "-Werror",
                 "-pthread", "-I", str(ROOT / "lib"),
                 *[str(ROOT / path) for path in (
                     "tests/hardware_controller_native.cpp",
                     "lib/hardware_controller.cpp", "lib/PowerfulBLDCdriver.cpp",
                     "lib/linux_wire.cpp", "lib/linux_kicker.cpp", "lib/status_display.cpp",
                     "lib/imu/linux_bno08x.cpp",
                 )], *objects, "-o", executable],
                check=True,
            )
            subprocess.run([executable], check=True, timeout=30)

    def test_binding_rejects_bad_calibration_before_opening_bus(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "calibration.json"

            def create(addresses, **kwargs):
                return HardwareController.from_i2c_addresses(
                    addresses, 50, 100, 1000, 3,
                    calibration_file=str(path), i2c_device="/nonexistent/i2c-test",
                    **kwargs, use_pcb=USE_PCB)

            with self.assertRaises(FileNotFoundError):
                create([25, 26, 27, 28])
            path.write_text(json.dumps({"motors": [
                {"address": address, "elecangleoffset": 123, "sincoscentre": 2048}
                for address in [28, 26, 25, 27]
            ]}))
            with self.assertRaisesRegex(ValueError, "Missing calibration"):
                create([25, 26, 27, 29])
            with self.assertRaisesRegex(ValueError, "duplicate"):
                create([25, 26, 27, 27])
            with self.assertRaisesRegex(ValueError, "four wheels"):
                create([25])
            for settings in ({"imu_address": 25}, {"imu_address": 0},
                             {"imu_report_interval_ms": 0}, {"imu_report_interval_ms": 1001}):
                with self.assertRaisesRegex(ValueError, "IMU"):
                    create([25, 26, 27, 28], **settings)
            for settings in ({"drive_motor_current_limit": -1.0},
                             {"dribbler_motor_current_limit": float("inf")},
                             {"kick_pulse_length": -0.01}, {"kick_cooldown": float("nan")}):
                with self.assertRaises(ValueError):
                    create([25, 26, 27, 28], **settings)
            for settings in ({"motor_hz": 0}, {"pcb_hz": 201},
                             {"imu_poll_hz": float("nan")}, {"motor_hz": float("inf")}):
                with self.assertRaisesRegex(ValueError, "Polling rate"):
                    create([25, 26, 27, 28], **settings)
            # Reordered calibration is accepted by address and reaches device open.
            for pin in (-2, 28):
                with self.assertRaisesRegex(ValueError, "kicker|Kicker"):
                    create([25, 26, 27, 28], kicker_pin=pin)
            with self.assertRaisesRegex(RuntimeError, "Open /nonexistent/i2c-test"):
                create([25, 26, 27, 28])
            path.write_text('{"motors": [{"address": 25}]}')
            with self.assertRaises(KeyError):
                create([25, 26, 27, 28])

    def test_native_imu_api(self):
        # Inspect the built binding without opening hardware.
        for method in ("get_raw_imu_yaw", "set_startup_yaw", "get_yaw",
                       "get_gyro_z_deg_s", "get_latest_quaternion", "imu_update_count"):
            self.assertTrue(hasattr(HardwareController, method))
        self.assertNotIn("yaw:", HardwareController.move.__doc__)
        self.assertIn("dribbler:", HardwareController.move.__doc__)
        self.assertIn("kick: bool = False", HardwareController.move.__doc__)
        constructor_doc = HardwareController.from_i2c_addresses.__doc__
        self.assertRegex(constructor_doc, r"drive_motor_current_limit: [^=]+ = 8\.0")
        self.assertRegex(constructor_doc, r"kick_pulse_length: [^=]+ = 0\.02")
        self.assertIn("use_pcb: bool = False", constructor_doc)

    def test_main_yaw_reference_and_lidar_prior(self):
        # Execute the actual startup helpers without importing main's hardware side effects.
        source = ast.parse((ROOT / "main.py").read_text())
        names = {"RollingYawSampler", "capture_startup_yaw", "feed_imu_yaw_prior"}
        helpers = ast.Module(body=[node for node in source.body
                                   if isinstance(node, (ast.FunctionDef, ast.ClassDef))
                                   and node.name in names], type_ignores=[])
        priors = []
        namespace = {
            "math": math, "deque": deque, "STARTUP_YAW_SAMPLE_COUNT": 2,
            "STARTUP_YAW_SAMPLE_INTERVAL": 0.02, "enter_pressed": lambda: False,
            "time": SimpleNamespace(sleep=lambda _: None),
            "lidar": SimpleNamespace(set_imu_yaw=priors.append, clear_imu_yaw=lambda: None),
        }
        exec(compile(helpers, str(ROOT / "main.py"), "exec"), namespace)  # noqa: S102 - local source helpers only
        class FakeHardware:
            def __init__(self):
                self.count = 0
                self.raw = iter([None, 179, -179])
                self.relative = iter([None, 75])

            @property
            def imu_update_count(self):
                self.count += 1
                return self.count

            def get_raw_imu_yaw(self):
                return next(self.raw)

            def get_yaw(self):
                return next(self.relative)

            def health(self):
                return {"imu_healthy": True, "imu_recovery_generation": 0}

        hardware = FakeHardware()
        reference = namespace["capture_startup_yaw"](hardware)
        self.assertAlmostEqual(abs(reference), 180)
        namespace["feed_imu_yaw_prior"](hardware)
        self.assertEqual(priors, [])
        namespace["feed_imu_yaw_prior"](hardware)
        self.assertEqual(priors, [75])  # Already relative: no second sign/offset conversion.

    def test_main_passes_kick_to_native_controller(self):
        source = ast.parse((ROOT / "main.py").read_text())
        calls = [node for node in ast.walk(source) if isinstance(node, ast.Call)]
        moves = [node for node in calls if isinstance(node.func, ast.Attribute)
                 and node.func.attr == "move"]
        self.assertTrue(any(keyword.arg == "kick" and isinstance(keyword.value, ast.Name)
                            and keyword.value.id == "kick"
                            for move in moves for keyword in move.keywords))
        self.assertFalse(any(isinstance(node.func, ast.Name) and node.func.id == "Kicker"
                             for node in calls))


if __name__ == "__main__":
    unittest.main()
