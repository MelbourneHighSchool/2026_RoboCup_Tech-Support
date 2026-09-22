"""Hardware-free replay, capture and timestamped caller integration checks."""

import csv
import json
import math
from pathlib import Path
from types import SimpleNamespace

import pytest

from lib.localisation_motion import MotionCapture, _captures, record_diagnostic_event
from lib.localisation_service import LidarVelocityEstimator, predict_odometry
from lib.replay_localisation import replay


def capture_fixture(path):
    events = [{"type": "header", "version": 1, "pitch": [2430, 1820], "mount": [0, 0, 0],
               "mode": "full", "motion_noise": 0.30, "seed": 42, "use_pcb": False}]
    for i in range(8):
        stamp = 1000 + i * 0.02
        events.append({"type": "motion", "recorded_s": stamp + 0.002,
                       "sample": {"timestamp_s": stamp, "read_span_s": 0.001, "vx": 0, "vy": 0,
                                  "yaw": [[stamp, 0]], "gyro": [[stamp, 0]], "epoch": 1}})
        if i >= 5:
            midpoint = stamp - 0.05
            points = [[angle, 400 / -math.sin(math.radians(angle)), 63, True,
                       midpoint - 0.03 + index * 0.002]
                      for index, angle in enumerate(range(241, 302, 2))]
            events.append({"type": "scan", "recorded_s": stamp + 0.002,
                           "received_s": stamp, "time_s": midpoint, "points": points})
    events.append({"type": "footer", "dropped_events": 0})
    path.write_text("\n".join(json.dumps(event) for event in events) + "\n")


def read_rows(path):
    with Path(path).open() as stream:
        rows = list(csv.DictReader(stream))
    for row in rows:
        row.pop("processing_ms")  # Wall CPU time is deliberately not deterministic.
    return rows


def test_identical_replays_are_deterministic_and_render_plots(tmp_path):
    path = tmp_path / "capture.jsonl"
    capture_fixture(path)
    first = replay(path, tmp_path / "one", "full", plot_every=1)
    second = replay(path, tmp_path / "two", "full", plot_every=0)
    assert first == second
    assert first["capture_complete"] and first["reasons"] == {"accepted": 3}
    assert read_rows(tmp_path / "one/full.csv") == read_rows(tmp_path / "two/full.csv")
    plots = list((tmp_path / "one").glob("*.svg"))
    assert len(plots) == 3
    assert "Forward x (mm)" in plots[0].read_text()
    for mode in ("off", "rotation"):
        result = replay(path, tmp_path / mode, mode, plot_every=0)
        assert result["reasons"] == first["reasons"]
        # All modes agree on genuinely stationary scans, including particle state.
        assert read_rows(tmp_path / mode / f"{mode}.csv") == read_rows(tmp_path / "two/full.csv")


def test_capture_closes_with_header_footer_and_does_not_overwrite(tmp_path):
    path = tmp_path / "new-run" / "motion.jsonl"
    capture = MotionCapture(path, {"mode": "full"})
    lidar = object()
    _captures[id(lidar)] = capture
    capture.record({"type": "motion", "sample": {"vx": 1}})
    record_diagnostic_event(lidar, "command", phase="forward", speed_mm_s=500)
    with pytest.raises(ValueError, match="reserved"):
        record_diagnostic_event(lidar, "scan")
    _captures.pop(id(lidar)).close()
    records = [json.loads(line) for line in path.read_text().splitlines()]
    assert [row["type"] for row in records] == ["header", "motion", "command", "footer"]
    assert records[2]["phase"] == "forward" and records[2]["speed_mm_s"] == 500
    assert records[-1]["dropped_events"] == 0
    with pytest.raises(FileExistsError):
        MotionCapture(path, {})


def test_service_feeds_acquisition_times_instead_of_poll_times():
    sample = {"vx": 123, "vy": -50, "timestamp_s": 9.98, "read_span_s": 0.003,
              "yaw": [(9.97, 10)], "gyro": [(9.97, 30)], "epoch": 7}
    fed = []
    hardware = SimpleNamespace(get_localisation_sample=lambda: sample,
                               get_measured_body_velocity_mm_s=lambda _: pytest.fail("Duplicate QDR read"),
                               get_gyro_z_deg_s=lambda: 30)
    lidar = SimpleNamespace(feed_motion=lambda value: fed.append(value))
    now, diagnostics = predict_odometry(lidar, hardware, hardware, 0, LidarVelocityEstimator(),
                                       10, 9.9, clock=lambda: 10)
    assert now == 10 and fed == [sample]
    assert diagnostics.fed_vx == 123 and diagnostics.fed_vy == -50


def test_incomplete_capture_and_drops_are_reported(tmp_path):
    path = tmp_path / "capture.jsonl"
    capture_fixture(path)
    lines = path.read_text().splitlines()[:-1]
    lines.append(json.dumps({"type": "health", "dropped_scans": 2, "dropped_events": 1}))
    path.write_text("\n".join(lines) + "\n")
    result = replay(path, tmp_path / "out", "full", plot_every=0)
    assert not result["capture_complete"]
    assert result["dropped_scans"] == 2 and result["dropped_events"] == 1
