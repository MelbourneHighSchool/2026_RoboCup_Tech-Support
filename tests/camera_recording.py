import importlib
import sys
import threading
import types
import unittest.mock

import cv2
import numpy as np
import pytest


def _import_camera_without_picamera_hardware():
    picamera2 = types.ModuleType("picamera2")
    picamera2.Picamera2 = object
    encoders = types.ModuleType("picamera2.encoders")
    encoders.H264Encoder = object
    encoders.JpegEncoder = object
    outputs = types.ModuleType("picamera2.outputs")
    outputs.FileOutput = object
    outputs.PyavOutput = object
    request = types.ModuleType("picamera2.request")
    request.MappedArray = object
    modules = {
        "cv2": cv2,
        "picamera2": picamera2,
        "picamera2.encoders": encoders,
        "picamera2.outputs": outputs,
        "picamera2.request": request,
    }
    with unittest.mock.patch.dict(sys.modules, modules):
        sys.modules.pop("lib.camera", None)
        return importlib.import_module("lib.camera")


def _make_camera_for_infer(camera_module):
    camera = camera_module.Camera.__new__(camera_module.Camera)
    camera.camera_bearing_offset_deg = 270.0
    camera._infer_stop = threading.Event()
    camera._is_shutting_down = False
    camera._latest_lock = threading.Lock()
    camera._latest_seq = 42
    camera._latest_buf = np.zeros((8, 8, 3), dtype=np.uint8)
    camera._latest_sensor_timestamp_ns = 1_500_000_000
    camera._first_sensor_timestamp_ns = 1_000_000_000
    camera._video_start_elapsed_s = 2.0
    camera.session_epoch_monotonic = 10.0
    camera._measurement_lock = threading.Lock()
    camera._last_detection = None
    camera._bearing = None
    camera._distance = None
    camera._bot_measurements = []
    camera._frame_id = 0
    camera.distance_calibration = {
        "model": {
            "coefficients": [1.0, 0.0],
            "radial_pixel_range": [0.0, 1000.0],
        }
    }
    camera._distance_calibration_warning_logged = True
    camera.bot_distance_calibration = {
        "model": {"coefficients": [2.0, 10.0], "radial_pixel_range": [0.0, 1000.0]}
    }
    return camera


def inference_metadata_identifies_exact_source_capture():
    camera_module = _import_camera_without_picamera_hardware()
    camera = _make_camera_for_infer(camera_module)
    detection = {
        "bbox": (1, 2, 3, 4),
        "centre": (2.5, 4.0),
        "point": (2.5, 4.0),
        "radial_pixels": 1.0,
        "confidence": 0.75,
        "polygon": None,
    }
    camera._detect_scene = lambda _frame: (detection, [detection])
    events = []

    def collect(event):
        events.append(event)
        camera._infer_stop.set()

    camera.detection_callback = collect
    camera._infer_loop()

    assert events[0]["capture_sequence"] == 42
    assert events[0]["sensor_timestamp_ns"] == 1_500_000_000
    assert events[0]["video_time_s"] == pytest.approx(0.5)
    assert events[0]["elapsed_s"] == pytest.approx(2.5)
    assert events[0]["detection"] is detection
    assert events[0]["bots"] == [detection]


def inference_bottom_centre_detection_feeds_radial_pixels():
    camera_module = _import_camera_without_picamera_hardware()
    det = camera_module._detection_dict_from_xyxy(
        (100.0, 50.0, 140.0, 200.0),
        0.9,
        frame_width=640,
        frame_height=640,
        point="bottom_centre",
    )
    assert det["centre"] == (120.0, 125.0)
    assert det["point"] == (120.0, 200.0)
    assert det["radial_pixels"] == pytest.approx(
        np.hypot(120.0 - 320.0, 200.0 - 320.0)
    )


def inference_detect_scene_keeps_best_ball_and_all_bots():
    camera_module = _import_camera_without_picamera_hardware()
    camera = camera_module.Camera.__new__(camera_module.Camera)

    class FakeModel:
        ball_class_id = 0
        bot_class_id = 1

        def predict(self, _frame):
            return [
                {"xyxy": (10, 10, 20, 20), "confidence": 0.4, "class_id": 0},
                {"xyxy": (30, 30, 50, 50), "confidence": 0.8, "class_id": 0},
                {"xyxy": (100, 100, 140, 180), "confidence": 0.7, "class_id": 1},
                {"xyxy": (200, 200, 240, 280), "confidence": 0.6, "class_id": 1},
            ]

    camera.ball_model = FakeModel()
    frame = np.zeros((640, 640, 3), dtype=np.uint8)
    ball, bots = camera._detect_scene(frame)
    assert ball["confidence"] == pytest.approx(0.8)
    assert ball["centre"] == (40.0, 40.0)
    assert len(bots) == 2
    assert bots[0]["point"] == (120.0, 140.0)
    assert bots[1]["point"] == (220.0, 240.0)
    assert bots[0]["radial_pixels"] == pytest.approx(np.hypot(120 - 320, 140 - 320))


def inference_scene_measurement_returns_ball_and_bots_atomically():
    camera_module = _import_camera_without_picamera_hardware()
    camera = _make_camera_for_infer(camera_module)
    ball = {
        "bbox": (310, 310, 20, 20),
        "centre": (320.0, 320.0),
        "point": (320.0, 320.0),
        "radial_pixels": 0.0,
        "confidence": 0.9,
        "polygon": None,
    }
    bots = [
        {
            "bbox": (400, 300, 40, 80),
            "centre": (420.0, 340.0),
            "point": (420.0, 380.0),
            "radial_pixels": float(np.hypot(420.0 - 320.0, 380.0 - 320.0)),
            "confidence": 0.8,
            "polygon": None,
        }
    ]
    camera._detect_scene = lambda _frame: (ball, bots)
    camera.detection_callback = lambda _event: camera._infer_stop.set()
    camera._infer_loop()

    frame_id, bearing, distance, bot_measurements = camera.get_scene_measurement()
    assert frame_id == 1
    assert bearing is not None
    assert distance == pytest.approx(0.0)
    assert len(bot_measurements) == 1
    bot_bearing, bot_distance = bot_measurements[0]
    assert bot_bearing is not None
    assert bot_distance == pytest.approx(2 * bots[0]["radial_pixels"] + 10)
    # Legacy API still returns ball-only.
    legacy_id, legacy_bearing, legacy_distance = camera.get_measurement()
    assert legacy_id == frame_id
    assert legacy_bearing == bearing
    assert legacy_distance == distance


def inference_missing_bot_calibration_does_not_use_ball_fit():
    camera_module = _import_camera_without_picamera_hardware()
    camera = _make_camera_for_infer(camera_module)
    camera.bot_distance_calibration = None
    det = camera_module._detection_dict_from_xyxy((1, 1, 4, 4), .9, 8, 8)
    assert camera._polar_from_detection(det, 8, 8)[1] is not None
    bearing, distance = camera._polar_from_detection(det, 8, 8, target="bot")
    assert bearing is not None
    assert distance is None


def inference_classify_camera_bot_positions_filters_self_and_teammate():
    # Avoid importing main.py (hardware deps). Keep this in sync with
    # main.classify_camera_bot_positions.
    def classify_camera_bot_positions(
        camera_bot_positions,
        self_xy,
        peer_xy=None,
        match_mm=100,
    ):
        friendly_matches = []
        enemy_bot_positions = []
        self_x, self_y = self_xy
        for bot_x, bot_y in camera_bot_positions:
            if np.hypot(bot_x - self_x, bot_y - self_y) <= match_mm:
                continue
            if (
                peer_xy is not None
                and np.hypot(bot_x - peer_xy[0], bot_y - peer_xy[1]) <= match_mm
            ):
                friendly_matches.append((bot_x, bot_y))
                continue
            enemy_bot_positions.append((bot_x, bot_y))
        return friendly_matches, enemy_bot_positions

    self_xy = (1000.0, 900.0)
    peer_xy = (1400.0, 900.0)
    camera_bots = [
        (1005.0, 902.0),  # self
        (1395.0, 905.0),  # teammate
        (1800.0, 1000.0),  # enemy
        (500.0, 400.0),  # enemy
    ]
    friendly, enemies = classify_camera_bot_positions(
        camera_bots, self_xy, peer_xy=peer_xy, match_mm=100
    )
    assert friendly == [(1395.0, 905.0)]
    assert enemies == [(1800.0, 1000.0), (500.0, 400.0)]

    # Without a peer, near-peer detections become enemies.
    _friendly, enemies_no_peer = classify_camera_bot_positions(
        camera_bots, self_xy, peer_xy=None, match_mm=100
    )
    assert (1395.0, 905.0) in enemies_no_peer
    assert (1005.0, 902.0) not in enemies_no_peer


def inference_diagnostics_keep_source_pixels_and_all_boxes():
    camera_module = _import_camera_without_picamera_hardware()
    camera = _make_camera_for_infer(camera_module)
    camera.diagnostics_enabled = True
    camera._latest_capture_monotonic = 123.0
    camera._latest_buf[:] = [10, 80, 220]
    source = camera._latest_buf.copy()
    ball = camera_module._detection_dict_from_xyxy((1, 1, 4, 4), .9, 8, 8)
    bot = camera_module._detection_dict_from_xyxy((4, 4, 7, 7), .8, 8, 8, point="bottom_centre")

    def detect(rgb):
        np.testing.assert_array_equal(rgb, source[..., ::-1])
        return ball, [bot]

    camera._detect_scene = detect
    camera.detection_callback = lambda _event: camera._infer_stop.set()
    camera._infer_loop()
    snapshot = camera.get_diagnostic_snapshot()
    assert snapshot["frame_id"] == 1
    assert snapshot["capture_sequence"] == 42
    assert snapshot["timestamp"] == 123.0
    assert snapshot["ball"] == ball and snapshot["bots"] == [bot]
    np.testing.assert_array_equal(snapshot["frame"], source)
    snapshot["frame"][:] = 0
    snapshot["bots"].clear()
    again = camera.get_diagnostic_snapshot()
    np.testing.assert_array_equal(again["frame"], source)
    assert len(again["bots"]) == 1


@pytest.mark.parametrize("offset,forward,right", [
    (270, (0, 4), (4, 0)),
    (180, (4, 0), (8, 4)),
])
def test_camera_mount_bearings(offset, forward, right):
    module = _import_camera_without_picamera_hardware()
    camera = _make_camera_for_infer(module)
    camera.camera_bearing_offset_deg = offset
    for point, expected in ((forward, 0), (right, 90)):
        detection = {"centre": point, "radial_pixels": 4}
        for target in ("ball", "bot"):
            bearing, _ = camera._polar_from_detection(detection, 8, 8, target=target)
            assert bearing % 360 == pytest.approx(expected)


def test_camera_offset_config(tmp_path):
    from lib.config import load_camera_bearing_offset

    path = tmp_path / "config.txt"
    assert load_camera_bearing_offset(path) == 270
    path.write_text("# Old configuration\ni2c_addresses=28,32,31,30\n")
    assert load_camera_bearing_offset(path) == 270
    path.write_text("camera_bearing_offset_deg=180 # image up\n")
    assert load_camera_bearing_offset(path) == 180
    for value in ("nan", "inf", "up"):
        path.write_text(f"camera_bearing_offset_deg={value}\n")
        with pytest.raises(ValueError, match="camera_bearing_offset_deg"):
            load_camera_bearing_offset(path)
