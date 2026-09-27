"""Exercise real camera publication methods without Pi/Hailo imports."""

import ast
import copy
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np


class CameraSceneTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        source = ast.parse((Path(__file__).resolve().parents[1] / "lib/camera.py").read_text())
        camera = next(n for n in source.body if isinstance(n, ast.ClassDef) and n.name == "Camera")
        camera.body = [n for n in camera.body if isinstance(n, ast.FunctionDef)
                       and n.name in {"_infer_loop", "get_scene_snapshot", "get_scene_measurement"}]
        scope = {"copy": copy, "time": time, "np": np,
                 "cv2": SimpleNamespace(cvtColor=lambda frame, _: frame, COLOR_BGR2RGB=0)}
        exec(compile(ast.Module(body=[camera], type_ignores=[]), "camera-publication", "exec"), scope)  # noqa: S102
        cls.camera_type = scope["Camera"]

    def test_atomic_metadata_matches_inference_input_without_diagnostic_images(self):
        camera = self.camera_type()
        camera._measurement_lock = threading.Lock()
        camera._latest_lock = threading.Lock()
        camera._infer_stop = threading.Event()
        camera._is_shutting_down = False
        camera._latest_seq = 1
        camera._latest_buf = np.zeros((4, 4, 3), dtype=np.uint8)
        camera._latest_sensor_timestamp_ns = 1_000_000
        camera._latest_exposure_monotonic = 1.0
        camera._latest_capture_monotonic = 1.1
        camera._frame_id = 0
        camera.detection_callback = None
        camera.diagnostics_enabled = False
        camera._scene_snapshot = None
        self.assertIsNone(camera.get_scene_snapshot())

        def detect(_frame):
            # A newer capture arrives while the current frame is being inferred.
            camera._latest_exposure_monotonic = 2.0
            camera._latest_seq = 2
            camera._infer_stop.set()
            return None, [{"confidence": 0.9}]

        camera._detect_scene = detect
        camera._polar_from_detection = lambda *_args, **_kwargs: (30, 1000)
        camera._infer_loop()
        snapshot = camera.get_scene_snapshot()
        self.assertEqual(snapshot["timestamp_s"], 1.0)
        self.assertEqual(snapshot["frame_id"], 1)
        self.assertEqual(snapshot["bots"], [{"bearing_deg": 30, "distance_mm": 1000,
                                             "confidence": 0.9}])
        self.assertNotIn("frame", snapshot)
        self.assertFalse(hasattr(camera, "_diagnostic_snapshot"))
        self.assertEqual(camera.get_scene_measurement(), (1, None, None, [(30, 1000)]))
        snapshot["bots"][0]["distance_mm"] = 10
        self.assertEqual(camera.get_scene_snapshot()["bots"][0]["distance_mm"], 1000)
        camera._is_shutting_down = True
        self.assertIsNone(camera.get_scene_snapshot())
        self.assertEqual(camera.get_scene_measurement(), (1, None, None, []))
