"""Deadline pacing and real native localisation-worker concurrency regressions."""

import json
import subprocess
import sys
import threading
import time

import pytest
from lib.hardware_controller import MotionSource

from lib import lidar
from lib.localisation_motion import close_motion_capture as finish_motion_capture
from lib.localisation_motion import configure_motion
from lib.loop_timing import RateLimiter
from lib.recording_session import RecordingSession
from lib.replay_localisation import replay


def wait_for(predicate, timeout=2):
    deadline = time.monotonic() + timeout
    while not predicate():
        if time.monotonic() >= deadline:
            pytest.fail("Native worker did not reach the expected state")
        time.sleep(0.002)


def sample(epoch=1, vx=100):
    stamp = time.monotonic() - 0.001
    history = [(stamp - 0.05 + i * 0.005, 0) for i in range(11)]
    return {"vx": vx, "vy": 0, "timestamp_s": stamp, "read_span_s": 0.001,
            "yaw": history, "gyro": history, "epoch": epoch}


def test_rate_limiter_paces_and_skips_slow_iteration():
    now = [10.0]
    sleeps = []

    def sleep(duration):
        sleeps.append(duration)
        now[0] += duration

    limiter = RateLimiter(150, clock=lambda: now[0], sleep=sleep)
    limiter.wait()
    for _ in range(10):
        now[0] += 0.001
        limiter.wait()
    assert now[0] == pytest.approx(10 + 10 / 150)
    assert len(sleeps) == 10
    now[0] += 0.05
    limiter.wait()
    assert limiter.missed_deadlines >= 6
    assert 0 < sleeps[-1] <= 1 / 150
    assert limiter.diagnostics()["logic_work_max_ms"] == pytest.approx(50)


def test_async_checkpoint_does_not_wait_for_disk(tmp_path, monkeypatch):
    session = RecordingSession(tmp_path / "recording", resolution=(640, 640), requested_fps=90)
    entered, release = threading.Event(), threading.Event()
    original = session._write_metadata_locked

    def slow_write():
        entered.set()
        assert release.wait(2)
        original()

    monkeypatch.setattr(session, "_write_metadata_locked", slow_write)
    try:
        session.request_checkpoint({"marker": 1})
        assert entered.wait(1)
        started = time.monotonic()
        for i in range(100):
            session.request_checkpoint({"marker": i})
        assert time.monotonic() - started < 0.1
    finally:
        release.set()
        session.close()
    metadata = json.loads(session.metadata_path.read_text())
    assert metadata["finalized"]
    assert metadata["marker"] == 99
    assert session.checkpoint_error is None


def test_native_prediction_runs_without_python_feeding_localisation():
    source = lidar.test_create_motion_source()
    assert isinstance(source, MotionSource)
    lidar.start_coordinates(2430, 1820, motion_source=source, prediction_hz=100)
    try:
        for _ in range(12):
            lidar.test_publish_motion(source, sample())
            time.sleep(0.01)
        wait_for(lambda: lidar.get_prediction_count() >= 8)
        assert lidar.get_pose_snapshot()["age_s"] < 0.05
        assert lidar.get_worker_diagnostics()["error"] == ""
        with pytest.raises(RuntimeError, match="Manual motion"):
            lidar.feed_motion(sample())
        with pytest.raises(RuntimeError, match="consumer"):
            lidar.test_claim_motion_source(source)
        # Closing hardware first wakes no Python callback and invalidates the pose.
        lidar.test_close_motion_source(source)
        wait_for(lambda: lidar.get_pose_snapshot()["timestamp_s"] == 0)
    finally:
        lidar.shutdown()
    assert not lidar.autonomous_motion_enabled()


def test_scan_stall_keeps_publication_reads_responsive_and_counts_overflow():
    source = lidar.test_create_motion_source()
    lidar.start_coordinates(2430, 1820, motion_source=source)
    try:
        for _ in range(4):
            lidar.test_publish_motion(source, sample())
            time.sleep(0.01)
        before = lidar.get_worker_diagnostics()["ticks"]
        lidar.test_delay_next_scan(200)
        stamp = time.monotonic() - 0.02
        lidar.test_publish_scan([[0, 1000, 63, True, stamp]], stamp)
        # The 200 ms delay is under the actual particle mutex.
        time.sleep(0.025)
        started = time.monotonic()
        for _ in range(200):
            lidar.get_pose_snapshot()
            lidar.get_pose()
            lidar.get_coordinates_info()
            lidar.get_mcl_update_count()
            lidar.set_imu_yaw(0)
            lidar.clear_line_readings()
        assert time.monotonic() - started < 0.1
        assert lidar.get_worker_diagnostics()["ticks"] <= before + 3
        for _ in range(80):
            lidar.test_publish_motion(source, sample())
        assert source.dropped > 0
        wait_for(lambda: lidar.get_worker_diagnostics()["missed_deadlines"] > 0)
        wait_for(lambda: source.depth == 0)
        assert lidar.get_worker_diagnostics()["error"] == ""
    finally:
        lidar.shutdown()
    # Stopping localisation releases consumer ownership while hardware remains alive.
    lidar.start_coordinates(2430, 1820, motion_source=source)
    lidar.shutdown()


def test_manual_lidar_does_not_require_hardware_import():
    subprocess.run(
        [sys.executable, "-c", "from lib import lidar; lidar.start_coordinates(2430,1820); lidar.shutdown()"],
        check=True, capture_output=True, timeout=5,
    )


def test_autonomous_capture_is_ordered_and_replayable(tmp_path, monkeypatch):
    path = tmp_path / "motion.jsonl"
    monkeypatch.setenv("SOCCER_LOCALISATION_RECORD", str(path))
    source = lidar.test_create_motion_source()
    lidar.start_coordinates(2430, 1820, motion_source=source)
    try:
        configure_motion(lidar)
        for _ in range(8):
            lidar.test_publish_motion(source, sample())
            time.sleep(0.01)
        stamp = time.monotonic() - 0.02
        lidar.test_publish_scan([[0, 1000, 63, True, stamp]], stamp)
        wait_for(lambda: lidar.get_deskew_status()["reason"] == "insufficient_observations")
        finish_motion_capture(lidar)
    finally:
        finish_motion_capture(lidar)
        lidar.shutdown()
    events = [json.loads(line) for line in path.read_text().splitlines()]
    motions = [event for event in events if event["type"] == "motion"]
    assert len(motions) == 8
    assert any(event["type"] == "scan" for event in events)
    assert [event["sample"]["timestamp_s"] for event in motions] == sorted(
        event["sample"]["timestamp_s"] for event in motions)
    result = replay(path, tmp_path / "replay", "full", plot_every=0)
    assert result["capture_complete"]
    assert result["dropped_events"] == result["dropped_motion"] == result["dropped_scans"] == 0
