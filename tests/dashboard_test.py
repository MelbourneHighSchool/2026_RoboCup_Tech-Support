"""Dashboard regressions without Raspberry Pi devices or a Hailo runtime."""

import copy
import http.client
import importlib.util
import json
import sys
import threading
import time
import types
from pathlib import Path

import cv2
import numpy as np
import pytest

from calibration.ball_distance import calculate_ball_bearing_deg
from calibration.dashboard import (
    Dashboard,
    Lease,
    discover_models,
    encode,
    line_thresholds,
    number,
    pixel_values,
    scene,
)
from calibration.dashboard_hardware import Hardware, target_command
from calibration_dashboard import DashboardServer
from lib.hailo_ball import DEFAULT_CLASS_NAMES, _load_metadata
from lib.localisation_service import (
    LidarVelocityEstimator,
    LocalisationSession,
    capture_startup_yaw,
    feed_imu_yaw_prior,
)

USE_PCB = False

ROOT = Path(__file__).resolve().parents[1]


class FakeHardware:
    def __init__(self, *_args, **_kwargs):
        self.calls = []
        self.mode = "idle"
        self.stops = 0
        self.pcb_stops = 0

    def submit(self, action, data, cancel):
        self.calls.append((action, data, cancel))
        self.mode = "queued " + action

    def snapshot(self):
        return {"mode": self.mode, "localisation": None}

    def stop_drive(self):
        self.stops += 1

    def request_stop_localisation(self):
        self.calls.append(("stop_localise", {}, None))
        self.mode = "idle"

    def request_stop_pcb(self):
        self.pcb_stops += 1

    def close(self):
        pass


@pytest.fixture
def dashboard(tmp_path):
    app = Dashboard(tmp_path, hardware_factory=FakeHardware, start=False)
    yield app
    app.close()


def detection(radius=5):
    return {"bbox": (2, 2, 4, 4), "centre": (4.0, 4.0), "point": (4.0, 4.0),
            "radial_pixels": float(radius), "confidence": 0.9, "polygon": None}


def install_snapshot(app, radius=5):
    app.sample_resolution = [16, 16]
    app.latest = {"frame": np.full((16, 16, 3), (15, 80, 220), dtype=np.uint8),
                  "ball": detection(radius), "bots": [detection(8)], "frame_id": 42,
                  "timestamp": time.monotonic()}


def wait_until(predicate):
    deadline = time.monotonic() + 3
    while not predicate():
        if time.monotonic() > deadline:
            raise AssertionError("Worker did not reach expected state")
        time.sleep(0.01)


def test_line_thresholds_save_reload_and_backup(dashboard):
    token = dashboard.command("claim", {}, None)["token"]
    with pytest.raises(PermissionError):
        dashboard.command("save_line_thresholds", {"black": 10, "white": 200}, None)
    dashboard.command("save_line_thresholds", {"black": 20, "white": 210}, token)
    # Reversed polarity is supported; green remains between thresholds.
    dashboard.command("save_line_thresholds", {"black": 220, "white": 30}, token)
    path = dashboard.root / "line_sensor_calibration.json"
    assert json.loads(path.read_text()) == {"black": 220, "white": 30,
                                            "sensor_radius_mm": 75, "sensor_count": 32}
    assert list((dashboard.root / "calibration_backups").glob("line_sensor_calibration-*.json"))
    restored = Dashboard(dashboard.root, hardware_factory=FakeHardware, start=False)
    try:
        assert restored.state()["line_thresholds"] == {"black": 220, "white": 30}
    finally:
        restored.close()
    for value in ({"black": -1, "white": 200}, {"black": 10, "white": 256},
                  {"black": 20, "white": 20}, {"black": 1.5, "white": 200},
                  {"black": True, "white": 200}, {"black": "nan", "white": 200}):
        with pytest.raises((ValueError, TypeError)):
            line_thresholds(value)


def test_pcb_start_stop_does_not_require_arming(dashboard):
    token = dashboard.command("claim", {}, None)["token"]
    dashboard.command("pcb_start", {}, token)
    assert dashboard.hardware.calls[-1][0] == "pcb_start"
    assert not dashboard.lease.armed
    dashboard.command("pcb_stop", {}, token)
    assert dashboard.hardware.pcb_stops == 1


@pytest.mark.parametrize("fail", [False, True])
def test_pcb_monitor_reads_without_motor_setup_and_invalidates(tmp_path, monkeypatch, fail):
    calls = []

    class Reader:
        def read_sensors(self):
            calls.append(time.monotonic())
            if fail and len(calls) > 1:
                raise OSError("PCB disconnected")
            return list(range(32))

    monkeypatch.setitem(sys.modules, "lib.hardware_controller", types.SimpleNamespace(PcbSensorReader=Reader))
    hardware = Hardware(tmp_path, lambda *_a, **_kw: None)
    try:
        hardware.submit("pcb_start", {}, threading.Event())
        wait_until(lambda: hardware.snapshot().get("pcb", {}).get("valid"))
        sample = hardware.snapshot()["pcb"]
        assert sample["readings"] == list(range(32))
        assert sample["age_s"] >= 0
        sample["readings"][0] = 255
        assert hardware.snapshot()["pcb"]["readings"][0] == 0
        assert hardware.controller is None and hardware.session is None
        with pytest.raises(ValueError, match="already running"):
            hardware.submit("manual_start", {}, threading.Event())
        if fail:
            wait_until(lambda: hardware.snapshot()["mode"] == "stopped")
            assert "PCB disconnected" in hardware.snapshot()["error"]
        else:
            hardware.request_stop_pcb()
            wait_until(lambda: hardware.snapshot()["mode"] == "idle")
        assert not hardware.snapshot()["pcb"]["valid"]
    finally:
        hardware.close()


def test_lease_expiry_invalidates_old_arm_and_token():
    now = [10.0]
    lease = Lease(clock=lambda: now[0])
    token = lease.claim()
    lease.arm(token)
    old_cancel = lease.cancel
    with pytest.raises(ValueError, match="Another"):
        lease.claim()
    now[0] += 1.5
    lease.heartbeat(token)
    now[0] += 1.9
    assert not lease.expire()
    now[0] += .2
    assert lease.expire() and old_cancel.is_set() and not lease.armed
    with pytest.raises(PermissionError):
        lease.heartbeat(token)
    new_token = lease.claim()
    assert new_token != token
    lease.arm(new_token)
    assert lease.cancel is not old_cancel and not lease.cancel.is_set()


def test_stop_available_to_viewers_cancels_queued_motion(dashboard):
    token = dashboard.command("claim", {}, None)["token"]
    dashboard.command("arm", {}, token)
    dashboard.command("drive", {"speed": 5000, "target": [500, 500], "addresses": [28, 32, 31, 30]}, token)
    event = dashboard.lease.cancel
    dashboard.command("stop", {}, None)
    assert event.is_set() and not dashboard.lease.armed
    worker = threading.Thread(target=dashboard._jobs)
    worker.start()
    wait_until(lambda: dashboard.events[-1]["message"].startswith("drive:"))
    assert not dashboard.hardware.calls
    dashboard.closing.set()
    worker.join(1)


def test_localisation_stop_requires_control_and_reaches_hardware(dashboard):
    with pytest.raises(PermissionError):
        dashboard.command("stop_localise", {}, None)
    token = dashboard.command("claim", {}, None)["token"]
    assert dashboard.command("stop_localise", {}, token) == {}
    assert dashboard.hardware.calls[-1][0] == "stop_localise"


@pytest.mark.parametrize("value", [0, 500, 1000, 1001, 5000])
def test_drive_speed_accepts_requested_range(value, dashboard):
    token = dashboard.command("claim", {}, None)["token"]
    dashboard.command("arm", {}, token)
    assert dashboard.command("drive", {"speed": value, "target": [100, 100], "addresses": [28, 32, 31, 30]}, token)["queued"]


@pytest.mark.parametrize("value", [-1, 5001, float("nan"), float("inf"), True, "bad"])
def test_speed_rejects_invalid_values(value):
    with pytest.raises((ValueError, TypeError)):
        number(value, 0, 5000, "speed")


def test_rpm_limit_and_arrival():
    _, requested, _, effective, arrived = target_command([0, 0, 0], [2000, 0], 5000)
    assert requested == 5000 and effective < requested and not arrived
    assert target_command([0, 0, 0], [2, 2], 500)[4]
    assert target_command([0, 0, 0], [2000, 0], 0)[1] == 0


@pytest.mark.parametrize("yaw,target_yaw,arrived", [(0, 90, False), (88, 90, True), (-179, 179, True)])
def test_target_waits_for_yaw_at_destination(yaw, target_yaw, arrived):
    _, speed, rotation, _, done = target_command([100, 100, yaw], [102, 102], 500, target_yaw)
    assert speed == 0
    assert rotation == target_yaw
    assert done is arrived


def test_target_rotates_while_translating():
    _, speed, rotation, _, arrived = target_command([0, 0, 0], [1000, 0], 500, 90)
    assert speed == 500 and rotation == 90 and not arrived


@pytest.mark.parametrize("value", [-181, 361, float("nan"), float("inf"), True, "bad"])
def test_drive_rejects_invalid_target_yaw(value, dashboard):
    token = dashboard.command("claim", {}, None)["token"]
    dashboard.command("arm", {}, token)
    with pytest.raises((ValueError, TypeError)):
        dashboard.command("drive", {"speed": 500, "target": [100, 100], "target_yaw": value,
                                    "addresses": [28, 32, 31, 30]}, token)


def test_pixel_picker_and_lossless_frozen_frame(dashboard):
    install_snapshot(dashboard)
    original = dashboard.latest["frame"].copy()
    frozen = dashboard.freeze()
    dashboard.latest["frame"][:] = 0
    frame = dashboard.frozen_frame(frozen["id"])
    np.testing.assert_array_equal(frame, original)
    decoded = cv2.imdecode(np.frombuffer(encode(frame, ".png"), dtype=np.uint8), cv2.IMREAD_COLOR)
    np.testing.assert_array_equal(frame, decoded)
    assert pixel_values(frame, 4, 4)["rgb"] == [220, 80, 15]
    assert pixel_values(frame, 4, 4)["hsv"] == cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)[4, 4].tolist()
    with pytest.raises(ValueError):
        pixel_values(frame, 16, 0)
    for _ in range(9):
        dashboard.freeze()
    with pytest.raises(ValueError, match="expired"):
        dashboard.frozen_frame(frozen["id"])


def test_scene_does_not_paint_source_frame(dashboard, monkeypatch):
    monkeypatch.setattr("calibration.dashboard.load_camera_bearing_offset", lambda: 123.0)
    install_snapshot(dashboard)
    snap = dashboard.latest
    original = snap["frame"].copy()
    overlay, results = scene(snap["frame"], snap["ball"], snap["bots"], None)
    assert [d["label"] for d in results] == ["Ball", "Bot"]
    assert [d["bearing"] for d in results] == [
        calculate_ball_bearing_deg(*d.get("point", d["centre"]), *snap["frame"].shape[1::-1]) + 123.0
        for d in (snap["ball"], snap["bots"][0])
    ]
    assert all(d["distance"] is None for d in results)
    assert not np.array_equal(overlay, original)
    np.testing.assert_array_equal(snap["frame"], original)



def test_ball_samples_fit_save_reload_and_stale_rejection(dashboard):
    class Camera:
        def reload_distance_calibration(self, path):
            self.loaded = json.loads(Path(path).read_text())
    dashboard.camera = Camera()
    for radius, distance in [(2, 100), (5, 250), (8, 400)]:
        install_snapshot(dashboard, radius)
        dashboard._edit("sample", {"distance": distance, "captured": radius == 2})
    dashboard._edit("fit", {})
    assert dashboard.fit["selected_degree"] == 1
    dashboard._edit("save_ball", {})
    assert dashboard.camera.loaded["resolution"] == [16, 16]
    assert dashboard.camera.loaded["capture_calibration"]["sample_count"] == 1
    assert dashboard.calibration["samples"] == dashboard.samples
    dashboard._edit("save_ball", {})
    assert list((dashboard.root / "calibration_backups").glob("*.json"))
    dashboard.latest["timestamp"] -= 1
    with pytest.raises(ValueError, match="fresh"):
        dashboard._edit("sample", {"distance": 500})
    dashboard.latest["timestamp"] = time.monotonic()
    dashboard.latest["ball"] = None
    with pytest.raises(ValueError):
        dashboard._edit("sample", {"distance": 500})
    dashboard.samples.append(copy.deepcopy(dashboard.samples[0]))
    with pytest.raises(ValueError, match="distinct"):
        dashboard._edit("save_ball", {})


def test_bot_calibration_is_separate_from_ball(dashboard):
    class Camera:
        def reload_distance_calibration(self, path, *, target):
            self.target = target
            self.loaded = json.loads(Path(path).read_text())

    dashboard.camera = Camera()
    install_snapshot(dashboard)
    dashboard._edit("sample", {"distance": 100})
    original_ball_samples = copy.deepcopy(dashboard.samples)
    for radius, distance in [(2, 200), (5, 500), (8, 800)]:
        install_snapshot(dashboard)
        dashboard.latest["bots"] = [detection(radius)]
        dashboard.latest["ball"] = None
        frozen = dashboard.freeze(for_bots=True)
        dashboard._edit("sample", {"target": "bot", "distance": distance, "captured": True,
                                   "frozen_id": frozen["id"], "bot_index": 0})
    dashboard._edit("save_ball", {"target": "bot"})
    assert dashboard.camera.target == "bot"
    assert dashboard.camera.loaded["capture_calibration"]["sample_count"] == 0
    assert (dashboard.root / "bot_distance_calibration.json").exists()
    assert not (dashboard.root / "ball_distance_calibration.json").exists()
    assert dashboard.samples == original_ball_samples
    assert dashboard.calibration is None
    assert dashboard.bot_calibration["samples"] == dashboard.bot_samples
    assert dashboard.state()["bot_samples"] == dashboard.bot_samples
    dashboard.latest["bots"] *= 2
    with pytest.raises(ValueError, match="select a bot"):
        dashboard._edit("sample", {"target": "bot", "distance": 500})
    dashboard._edit("clear_samples", {"target": "bot"})
    assert dashboard.bot_samples == []
    assert dashboard.samples == original_ball_samples


def test_bot_sample_uses_selected_frozen_detection_despite_live_changes(dashboard):
    install_snapshot(dashboard)
    dashboard.latest["bots"] = [detection(1), detection(9)]
    dashboard.latest["bots"][1].update(centre=(12, 10), bbox=(9, 8, 6, 4))
    frozen = dashboard.freeze(for_bots=True)
    assert [bot["radial_pixels"] for bot in frozen["bots"]] == [1, 9]
    # Both the live detections and response may change without changing the saved source.
    dashboard.latest["bots"][1]["radial_pixels"] = 3
    dashboard.latest["bots"].reverse()
    dashboard.latest["timestamp"] -= 2
    frozen["bots"][1]["radial_pixels"] = 100
    dashboard._edit("sample", {"target": "bot", "distance": 750,
                               "frozen_id": frozen["id"], "bot_index": 1})
    assert dashboard.bot_samples == [{"distance_mm": 750, "radial_pixels": 9,
                                      "centre_x": 12, "centre_y": 10,
                                      "bounding_box_area": 24, "captured": False}]
    assert dashboard.samples == []


def test_bot_selection_requires_fresh_capture_and_valid_frozen_selection(dashboard):
    install_snapshot(dashboard)
    dashboard.latest["timestamp"] -= 1
    with pytest.raises(ValueError, match="fresh"):
        dashboard.freeze(for_bots=True)
    install_snapshot(dashboard)
    dashboard.latest["bots"] = []
    with pytest.raises(ValueError, match="No detected bots"):
        dashboard.freeze(for_bots=True)
    install_snapshot(dashboard)
    frozen = dashboard.freeze(for_bots=True)
    data = {"target": "bot", "distance": 500, "frozen_id": frozen["id"]}
    for index in (None, -1, 1, True, "0"):
        with pytest.raises(ValueError, match="Select a bot"):
            dashboard._edit("sample", {**data, "bot_index": index})
    dashboard.sample_resolution = [32, 32]
    with pytest.raises(ValueError, match="resolution changed"):
        dashboard._edit("sample", {**data, "bot_index": 0})
    dashboard.sample_resolution = [16, 16]
    for _ in range(8):
        dashboard.freeze()
    with pytest.raises(ValueError, match="Freeze a fresh frame"):
        dashboard._edit("sample", {**data, "bot_index": 0})
    pixel_frame = dashboard.freeze()
    with pytest.raises(ValueError, match="Freeze a fresh frame"):
        dashboard._edit("sample", {**data, "frozen_id": pixel_frame["id"], "bot_index": 0})
    assert dashboard.bot_samples == []


def test_scene_uses_separate_distance_models(dashboard):
    install_snapshot(dashboard)
    ball_fit = {"model": {"coefficients": [10], "radial_pixel_range": [0, 100]}}
    bot_fit = {"model": {"coefficients": [20], "radial_pixel_range": [0, 100]}}
    snap = dashboard.latest
    _, results = scene(snap["frame"], snap["ball"], snap["bots"], ball_fit, bot_fit)
    assert [d["distance"] for d in results] == [10, 20]
    _, results = scene(snap["frame"], snap["ball"], snap["bots"], ball_fit)
    assert [d["distance"] for d in results] == [10, None]


@pytest.fixture
def movement(monkeypatch):
    driver = types.ModuleType("steelbar_powerful_bldc_driver")
    driver.PowerfulBLDCDriver = FakeMotor
    bus = types.ModuleType("legacy.i2c_bus")
    lock = threading.RLock()
    bus.get_shared_i2c_lock = lambda: lock
    bus.get_shared_i2c_bus = lambda: object()
    monkeypatch.setitem(sys.modules, "steelbar_powerful_bldc_driver", driver)
    monkeypatch.setitem(sys.modules, "legacy.i2c_bus", bus)
    spec = importlib.util.spec_from_file_location("dashboard_test_movement", ROOT / "legacy/movement.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setitem(sys.modules, "legacy.movement", module)
    return module


class FakeMotor:
    def __init__(self, *_args):
        self.calls = []
        self.finished = True
        self.fail_stop = False

    def __getattr__(self, name):
        def method(*args):
            self.calls.append((name, args))
            if self.fail_stop and name == "set_torque":
                raise OSError("disconnected")
            if name == "get_firmware_version":
                return 3
            if name == "is_calibration_finished":
                return self.finished
            if name.startswith("get_calibration_"):
                return 1234
            return 0
        return method


def test_four_motor_calibration_setup(movement):
    _motors, count, _ = movement.get_motors_for_calibration([28, 32, 31, 30])
    assert count == 4


@pytest.mark.parametrize("mode", ["cancel", "timeout", "failure", "success"])
def test_motor_calibration_cleanup_and_atomic_save(mode, movement, tmp_path):
    path = tmp_path / "motors.json"
    path.write_text('{"old":true}')
    motors = [FakeMotor(), FakeMotor()]
    cancel = threading.Event()
    if mode in ("cancel", "timeout"):
        motors[0].finished = False
    if mode == "failure":
        motors[0].fail_stop = True
    def progress(_event):
        if mode == "cancel":
            cancel.set()
    kwargs = {"calibration_file": str(path), "cancel_event": cancel, "progress": progress,
              "timeout_s": 0 if mode == "timeout" else 1}
    if mode == "success":
        result = movement.calibrate_motors(motors, 2, [28, 32], **kwargs)
        assert json.loads(path.read_text()) == result
    else:
        with pytest.raises((InterruptedError, TimeoutError, movement.MotorCommunicationError)):
            movement.calibrate_motors(motors, 2, [28, 32], **kwargs)
        assert json.loads(path.read_text()) == {"old": True}
    for motor in motors:
        assert ("configure_command_mode", (2,)) in motor.calls
        assert ("set_torque", (0,)) in motor.calls


class FakeLidar:
    def __init__(self):
        self.generation = 1
        self.ok = True
        self.predictions = []

    def get_coordinates_info(self):
        return (500, 600, 20, .9, self.ok)

    def predict_odometry(self, *args):
        self.predictions.append(args)

    def set_imu_yaw(self, _yaw):
        pass

    def get_scan_generation(self):
        return self.generation

    get_mcl_update_count = get_scan_generation

    def get_scan_count(self):
        return 180

    def scan_updates_enabled(self):
        return True

    def get_last_scan_correction(self):
        return (1, 500, 600, 20, 501, 601, 21, 1.4, 1, True)

    def get_recovery_status(self):
        return (1, 1, 0, .1, True)

    def get_scan_list(self):
        return [(0, 500, 20)]

    def shutdown(self):
        pass


class FakeIMU:
    imu_update_count = 1

    def __init__(self):
        self._running = True
        self.moves = []
    def get_yaw(self):
        return 10

    def get_gyro_z_deg_s(self):
        return 0

    def close(self):
        self.stop()

    def stop(self):
        self._running = False

    def move(self, *args, **kwargs):
        self.moves.append((args, kwargs))

    def get_measured_body_velocity_mm_s(self, _yaw):
        return 0.0, 0.0


def test_localisation_uses_native_relative_yaw_directly():
    hardware = FakeIMU()
    samples = iter((179.0, -179.0))
    hardware.get_raw_imu_yaw = lambda: next(samples)
    hardware.set_startup_yaw = lambda yaw: setattr(hardware, "startup_yaw", yaw)
    startup = capture_startup_yaw(hardware, sample_count=2, sample_interval=0)
    assert abs(abs(startup) - 180.0) < 1e-6
    assert hardware.startup_yaw == startup
    lidar = FakeLidar()
    lidar.prior = None
    lidar.set_imu_yaw = lambda yaw: setattr(lidar, "prior", yaw)
    feed_imu_yaw_prior(lidar, hardware, startup)
    assert lidar.prior == 10


def test_localisation_predicts_without_fix_and_flags_stale_data():
    lidar, imu = FakeLidar(), FakeIMU()
    now = [time.monotonic()]
    session = LocalisationSession(lidar, imu, 10, LidarVelocityEstimator(), clock=lambda: now[0], use_pcb=USE_PCB)
    lidar.ok = False
    assert not session.tick()["pose"][4]
    assert len(lidar.predictions) == 1 and lidar.predictions[0][:2] == (0, 0)
    lidar.ok = True
    assert session.tick()["fresh"]
    now[0] += .6
    assert not session.tick()["fresh"]


def test_pcb_follows_measured_prediction_but_does_not_renew_lidar_freshness():
    lidar, imu = FakeLidar(), FakeIMU()
    now = [time.monotonic()]
    corrections = []
    imu.get_measured_body_velocity_mm_s = lambda yaw: (123, -45)
    imu.get_gyro_z_deg_s = lambda: 100

    class Feed:
        def update(self, lidar_module, hardware):
            assert hardware is imu
            assert lidar_module.predictions[-1][:3] == (123, -45, 100)
            corrections.append(len(lidar_module.predictions))

    session = LocalisationSession(lidar, imu, 10, LidarVelocityEstimator(),
                                  clock=lambda: now[0], line_feed=Feed(), use_pcb=True)
    assert session.tick()["fresh"]
    now[0] += 0.51
    imu.imu_update_count += 1
    state = session.tick()
    assert corrections == [1, 2]
    assert not state["fresh"]  # Dashboard driving still stops after .5 s without LIDAR.
    assert state["mcl_age_s"] >= 0.5


@pytest.fixture
def rotation_session():
    lidar, imu = FakeLidar(), FakeIMU()
    lidar.mcl_updates = 1
    lidar.scans_enabled = True
    lidar.get_mcl_update_count = lambda: lidar.mcl_updates
    lidar.scan_updates_enabled = lambda: lidar.scans_enabled
    now = [time.monotonic()]
    session = LocalisationSession(lidar, imu, 10, LidarVelocityEstimator(), clock=lambda: now[0], use_pcb=USE_PCB)

    def tick(dt=0, *, scan=True, imu_report=True):
        now[0] += dt
        if scan:
            lidar.generation += 1
        if imu_report:
            imu.imu_update_count += 1
        return session.tick()

    assert tick()["fresh"]
    return lidar, imu, tick


def test_rotation_requires_fresh_corrections(rotation_session):
    lidar, _imu, tick = rotation_session
    assert tick(.1)["fresh"]
    state = tick(.5)
    assert not state["fresh"] and not state["rotation_prediction"]
    lidar.mcl_updates += 1
    assert tick(.1)["fresh"]


@pytest.mark.parametrize("failure", ["scan", "imu"])
def test_rotation_preserves_sensor_timeouts(rotation_session, failure):
    lidar, _imu, tick = rotation_session
    lidar.mcl_updates += 1
    state = tick(.6, scan=failure != "scan", imu_report=failure != "imu")
    assert not state["fresh"]


def test_legacy_gate_cannot_extend_freshness(rotation_session):
    lidar, _imu, tick = rotation_session
    lidar.scans_enabled = False
    assert tick(.1)["fresh"]
    assert not tick(.5)["fresh"]


def test_stop_localisation_releases_session_and_clears_diagnostics(tmp_path):
    notifications = []
    hardware = Hardware(tmp_path, lambda text, **_kwargs: notifications.append(text))
    session = types.SimpleNamespace(close_calls=0)
    session.close = lambda: setattr(session, "close_calls", session.close_calls + 1)
    try:
        with hardware.lock:
            hardware.session = session
            hardware.status.update(mode="monitoring", localisation={"pose": [1, 2, 3]})
            hardware.trail.append([1, 2, 3])
        hardware.request_stop_localisation()
        wait_until(lambda: hardware.snapshot()["mode"] == "idle")
        snapshot = hardware.snapshot()
        assert session.close_calls == 1
        assert snapshot["localisation"] is None
        assert snapshot["trajectory"] == []
        assert "Localisation stopped" in notifications
    finally:
        hardware.close()


def test_drive_abort_and_repeated_sessions(tmp_path):
    logs = []
    hardware = Hardware(tmp_path, lambda text, **_kwargs: logs.append(text))
    try:
        lidar, imu = FakeLidar(), FakeIMU()
        hardware.session = LocalisationSession(lidar, imu, 10, LidarVelocityEstimator(), use_pcb=USE_PCB)
        hardware.pause = types.SimpleNamespace(read=lambda: False, switch=types.SimpleNamespace(deinit=lambda: None))
        for _ in range(2):
            controller = imu
            event = threading.Event()
            with hardware.lock:
                hardware.controller = controller
                hardware.motion_cancel = event
                hardware.target = [1500, 600]
                hardware.status["mode"] = "driving"
            event.set()
            wait_until(lambda: hardware.controller is None and hardware.session is None)
            assert not controller._running
            # Aborted target must not resume if a new fix arrives.
            assert hardware.target is None
            if hardware.session is None:
                imu = FakeIMU()
                hardware.session = LocalisationSession(lidar, imu, 10, LidarVelocityEstimator(), use_pcb=USE_PCB)
    finally:
        hardware.close()


def test_http_origin_control_and_shared_previews(dashboard):
    install_snapshot(dashboard)
    dashboard.streams = {"camera": encode(dashboard.latest["frame"])}
    dashboard.stream_sequence = 1
    server = DashboardServer(("127.0.0.1", 0), dashboard)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    host, port = server.server_address
    origin = f"http://{host}:{port}"
    def post(action, headers, data=None):
        connection = http.client.HTTPConnection(host, port, timeout=2)
        connection.request("POST", "/api/" + action, json.dumps(data or {}),
                           {"Content-Type":"application/json", **headers})
        response = connection.getresponse()
        result = response.status, json.loads(response.read())
        connection.close()
        return result
    streams = []
    try:
        assert post("claim", {"Origin":"http://elsewhere"})[0] == 403
        status, result = post("claim", {"Origin":origin})
        assert status == 200
        assert post("heartbeat", {"Origin":origin, "X-Control-Token":result["token"]})[0] == 200
        assert post("stop", {"Origin":origin})[0] == 200
        for _ in range(2):
            connection = http.client.HTTPConnection(host, port, timeout=2)
            connection.request("GET", "/stream.mjpg?view=camera")
            response = connection.getresponse()
            assert response.status == 200
            assert b"FRAME" in response.read(30)
            streams.append(connection)
        # Neither client consumes further output; API calls still complete.
        assert post("freeze", {"Origin":origin})[0] == 200
        status, frozen = post("freeze", {"Origin":origin}, {"target": "bot"})
        assert status == 200
        assert frozen["bots"][0]["radial_pixels"] == 8
        connection = http.client.HTTPConnection(host, port, timeout=2)
        connection.request("GET", "/frozen.png?id=" + frozen["id"])
        response = connection.getresponse()
        assert response.status == 200
        frame = cv2.imdecode(np.frombuffer(response.read(), dtype=np.uint8), cv2.IMREAD_COLOR)
        connection.close()
        np.testing.assert_array_equal(frame, dashboard.latest["frame"])
        assert dashboard.stream_sequence == 1
    finally:
        dashboard.closing.set()
        with dashboard.condition:
            dashboard.condition.notify_all()
        for connection in streams:
            connection.close()
        server.shutdown()
        server.server_close()
        thread.join(1)


@pytest.mark.parametrize("cause", ["pose", "pause", "stale"])
def test_hardware_aborts_on_pose_pause_or_staleness(cause, tmp_path):
    hardware = Hardware(tmp_path, lambda *_args, **_kwargs: None)
    try:
        lidar, imu = FakeLidar(), FakeIMU()
        session = LocalisationSession(lidar, imu, 10, LidarVelocityEstimator(), use_pcb=USE_PCB)
        session.tick()
        if cause == "pose":
            lidar.ok = False
        if cause == "stale":
            session.last_scan_time -= 2
        hardware.pause = types.SimpleNamespace(read=lambda: cause == "pause", switch=types.SimpleNamespace(deinit=lambda: None))
        controller = imu
        with hardware.lock:
            hardware.session = session
            hardware.controller = controller
            hardware.motion_cancel = threading.Event()
            hardware.target = [1500, 600]
            hardware.status["mode"] = "driving"
        wait_until(lambda: hardware.controller is None and hardware.session is None)
        assert hardware.motion_cancel.is_set()
        assert not controller._running
        assert hardware.target is None
    finally:
        hardware.close()


def test_repeated_drive_reuses_localisation_controller(monkeypatch, tmp_path):
    config = types.ModuleType("lib.config")
    config.load_config = lambda: types.SimpleNamespace(pause_switch_pin=None)
    switch = types.ModuleType("lib.switch")
    switch.Switch = lambda _pin: types.SimpleNamespace(read=lambda: False, switch=types.SimpleNamespace(deinit=lambda: None))
    monkeypatch.setitem(sys.modules, "lib.config", config)
    monkeypatch.setitem(sys.modules, "lib.switch", switch)
    requested = [28, 32, 31, 30]
    (tmp_path / "calibration_data.json").write_text(json.dumps({"motors": [
        {"address": address, "elecangleoffset": 1, "sincoscentre": 2} for address in requested
    ]}))
    hardware = Hardware(tmp_path, lambda *_args, **_kwargs: None)
    # Stop the worker so this test can exercise startup/stop deterministically.
    hardware.closing.set()
    hardware.thread.join(1)
    try:
        session = LocalisationSession(FakeLidar(), FakeIMU(), 10, LidarVelocityEstimator(), use_pcb=USE_PCB)
        session.tick()
        hardware.session = session
        controllers = []
        for _ in range(2):
            cancel = threading.Event()
            hardware._drive({"addresses": requested, "target": [1500, 600], "speed": 500}, cancel)
            controllers.append(hardware.controller)
            hardware.stop_drive()
            assert hardware.controller is None and hardware.target is None
        assert controllers[0] is controllers[1] is session.imu
        assert session.imu._running
    finally:
        hardware.close()


def test_camera_pipeline_shared_and_latest_buffer_bounded(tmp_path):
    class FakeCamera:
        instances = 0
        def __init__(self, **_kwargs):
            FakeCamera.instances += 1
            self.resolution = (16, 16)
            self.inference_error = None
            self.capture_count = self.infer_count = 0
            self.closed = False

        def start(self):
            pass

        def get_diagnostic_snapshot(self):
            self.capture_count += 1
            self.infer_count += 1
            return {"frame": np.zeros((16, 16, 3), dtype=np.uint8), "ball": detection(), "bots": [],
                    "frame_id": self.infer_count, "timestamp": time.monotonic()}

        def stop(self):
            self.closed = True

    app = Dashboard(tmp_path, camera_factory=FakeCamera, hardware_factory=FakeHardware)
    try:
        wait_until(lambda: app.stream_sequence >= 2)
        for _ in range(20):
            assert app.state()["camera"]["status"] == "running"
        assert FakeCamera.instances == 1
        assert set(app.streams) == {"camera", "raw"}
        assert app.latest["frame_id"] <= app.camera.infer_count
        camera = app.camera
    finally:
        app.close()
    assert camera.closed


def test_model_discovery_and_live_switch_restart_only_camera(tmp_path):
    for model_id, size in (("n", 640), ("s", 800)):
        folder = tmp_path / f"open-soccer-detect-{model_id}_hailo_model"
        folder.mkdir()
        (folder / "model.hef").write_bytes(b"compiled")
        (folder / "metadata.yaml").write_text(f"imgsz: [{size}, {size}]\n")
    incomplete = tmp_path / "open-soccer-detect-m_hailo_model"
    incomplete.mkdir()
    (incomplete / "metadata.yaml").write_text("imgsz: 960\n")
    assert [(model["id"], model["input_size"]) for model in discover_models(tmp_path)] == [
        ("n", [640, 640]),
        ("s", [800, 800]),
    ]

    camera_instances = []

    class FakeCamera:

        def __init__(self, **kwargs):
            self.gain_controls = []
            self.picam2 = types.SimpleNamespace(
                camera_controls={"AnalogueGain": (1.0, 16.0, 1.0)},
                set_controls=self.gain_controls.append,
            )
            self.model_path = kwargs["ball_model_path"]
            self.resolution = (640, 640)
            self.inference_error = None
            self.capture_count = self.infer_count = 0
            self.closed = False
            camera_instances.append(self)

        def start(self):
            pass

        def get_diagnostic_snapshot(self):
            return None

        def stop(self):
            self.closed = True

    app = Dashboard(tmp_path, camera_factory=FakeCamera, hardware_factory=FakeHardware)
    try:
        wait_until(lambda: app.active_model == "n")
        app.hardware.mode = "monitoring"
        token = app.command("claim", {}, None)["token"]
        assert camera_instances[0].gain_controls == [{"AnalogueGain": 10.0}]
        app.command("analogue_gain", {"gain": "4.5"}, token)
        wait_until(lambda: app.analogue_gain == 4.5)
        assert camera_instances[0].gain_controls[-1] == {"AnalogueGain": 4.5}
        for invalid in (0, 17, "nan", "inf", "invalid"):
            with pytest.raises(ValueError):
                app._edit("analogue_gain", {"gain": invalid})
        assert app.analogue_gain == 4.5
        app.command("select_model", {"model": "s"}, token)
        wait_until(lambda: app.active_model == "s" and len(camera_instances) == 2)
        assert camera_instances[0].closed
        assert camera_instances[1].gain_controls == [{"AnalogueGain": 4.5}]
        assert app.state()["camera"]["analogue_gain_range"] == [1.0, 16.0]
        assert camera_instances[1].model_path.endswith("open-soccer-detect-s_hailo_model")
        assert app.hardware.mode == "monitoring"
        public_models = app.state()["camera"]["models"]
        assert public_models[1] == {"id": "s", "label": "Small", "input_size": [800, 800]}
        assert "path" not in public_models[1]
        with pytest.raises(ValueError, match="not available"):
            app._edit("select_model", {"model": "m"})
    finally:
        app.close()


def test_model_metadata_accepts_serialized_yaml_and_ignores_scalars(tmp_path):
    serialized = tmp_path / "open-soccer-detect-n_hailo_model"
    serialized.mkdir()
    (serialized / "model.hef").write_bytes(b"compiled")
    (serialized / "metadata.yaml").write_text(
        json.dumps("imgsz: [736, 736]\nnames: {0: Ball, 1: Bot}\n")
    )

    scalar = tmp_path / "open-soccer-detect-s_hailo_model"
    scalar.mkdir()
    (scalar / "model.hef").write_bytes(b"compiled")
    (scalar / "metadata.yaml").write_text("metadata unavailable\n")

    models = discover_models(tmp_path)
    assert [(model["id"], model["input_size"]) for model in models] == [
        ("n", [736, 736]),
        ("s", None),
    ]
    assert _load_metadata(serialized)["imgsz"] == [736, 736]
    assert _load_metadata(scalar) == {
        "names": DEFAULT_CLASS_NAMES,
        "imgsz": 640,
        "task": "detect",
    }


def test_watchdog_disarms_if_localisation_worker_stalls(dashboard):
    token = dashboard.command("claim", {}, None)["token"]
    dashboard.command("arm", {}, token)
    dashboard.hardware.snapshot = lambda: {"mode": "driving", "localisation": {"timestamp": time.monotonic() - 1}}
    thread = threading.Thread(target=dashboard._watchdog)
    thread.start()
    wait_until(lambda: dashboard.lease.cancel.is_set())
    assert not dashboard.lease.armed and dashboard.hardware.stops > 0
    dashboard.closing.set()
    thread.join(1)


def test_kicker_requires_arming_and_stop_cancels_request(dashboard):
    token = dashboard.command("claim", {}, None)["token"]
    with pytest.raises(ValueError, match="Arm"):
        dashboard.command("kick", {}, token)
    dashboard.command("arm", {}, token)
    cancel = dashboard.lease.cancel
    assert dashboard.command("kick", {}, token) == {"queued": True}
    dashboard.command("stop", {}, None)
    assert cancel.is_set()


def test_kicker_pulse_restores_input_and_checks_pause(monkeypatch, tmp_path):
    calls = []
    pin = types.SimpleNamespace(
        switch_to_input=lambda **kw: calls.append(("input", kw)),
        switch_to_output=lambda **kw: calls.append(("output", kw)),
        deinit=lambda: calls.append(("close", {})),
    )
    monkeypatch.setitem(sys.modules, "digitalio", types.SimpleNamespace(
        DigitalInOut=lambda _pin: pin, Pull=types.SimpleNamespace(DOWN="down")))
    monkeypatch.setitem(sys.modules, "lib.config", types.SimpleNamespace(
        load_config=lambda: types.SimpleNamespace(kicker_pin=21)))
    monkeypatch.setitem(sys.modules, "lib.switch", types.SimpleNamespace(Switch=None))
    hardware = Hardware(tmp_path, lambda *_a, **_kw: None)
    hardware.closing.set()
    hardware.thread.join(1)
    hardware.pause = types.SimpleNamespace(read=lambda: True)
    with pytest.raises(ValueError, match="pause"):
        hardware._kick(threading.Event())
    assert calls == []
    hardware.pause.read = lambda: False
    hardware._kick(threading.Event())
    assert [call[0] for call in calls] == ["input", "output", "input", "close"]
    assert calls[1][1] == {"value": True}
    with pytest.raises(ValueError, match="cooling"):
        hardware._kick(threading.Event())


def test_manual_api_validates_speed_and_requires_arming(dashboard):
    token = dashboard.command("claim", {}, None)["token"]
    with pytest.raises(ValueError, match="Arm"):
        dashboard.command("manual_input", {"forward": 100, "right": 0}, token)
    dashboard.command("arm", {}, token)
    received = []
    dashboard.hardware.update_manual = lambda *values: received.append(values)
    dashboard.command("manual_input", {"forward": 300, "right": -400}, token)
    assert received == [(300, -400)]
    for forward, right in [(1000, 1000), ("nan", 0), (0, "inf"), (True, 0)]:
        with pytest.raises((ValueError, TypeError)):
            dashboard.command("manual_input", {"forward": forward, "right": right}, token)


def test_manual_direction_release_timeout_and_pause(tmp_path):
    hardware = Hardware(tmp_path, lambda *_a, **_kw: None)
    hardware.closing.set()
    hardware.thread.join(1)
    imu = FakeIMU()
    hardware.controller = imu
    hardware.pause = types.SimpleNamespace(read=lambda: False)
    hardware.motion_cancel = threading.Event()
    hardware.status["mode"] = "manual"
    hardware.update_manual(0, 300)
    hardware._tick_manual()
    assert imu.moves[-1][0] == (100, 300, 10, 0)
    hardware.update_manual(0, 0)
    hardware._tick_manual()
    assert imu.moves[-1][0][1] == 0
    hardware.manual = (300, 0, time.monotonic() - 1)
    with pytest.raises(InterruptedError, match="timed out"):
        hardware._tick_manual()
    hardware.update_manual(300, 0)
    hardware.pause.read = lambda: True
    with pytest.raises(InterruptedError, match="pause"):
        hardware._tick_manual()
    hardware.stop_drive()
    assert not imu._running
    assert hardware.manual is None
    with pytest.raises(ValueError, match="Start manual"):
        hardware.update_manual(300, 0)
