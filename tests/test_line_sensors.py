"""Threshold classification, disabled defaults, and floor-colour localisation."""

import json
import subprocess
import time
from pathlib import Path
from types import SimpleNamespace

import pytest

from lib.line_sensors import LineSensorFeed, classify_readings, line_thresholds

USE_PCB = False


def test_disabled_feed_does_not_load_calibration_or_touch_hardware(tmp_path):
    feed = LineSensorFeed(tmp_path / "missing.json", use_pcb=USE_PCB)
    assert feed.error is None
    feed.update(object(), object())


def test_native_floor_colour_scoring(tmp_path):
    root = Path(__file__).resolve().parents[1]
    binary = tmp_path / "localisation-lines"
    subprocess.run(["g++", "-std=c++11", "-Wall", "-Wextra", "-Werror", "-O2", "-pthread",
                    str(root / "tests/localisation_lines_native.cpp"), "-o", str(binary)], check=True)
    subprocess.run([str(binary)], check=True, timeout=60)


@pytest.mark.parametrize(("black", "white", "readings", "expected"), [
    (64, 192, [0, 64, 65, 191, 192, 255], ["black", "black", "green", "green", "white", "white"]),
    (192, 64, [0, 64, 65, 191, 192, 255], ["white", "white", "green", "green", "black", "black"]),
])
def test_classify_boundaries_and_polarity(black, white, readings, expected):
    samples = readings + [100] * 26
    assert classify_readings(samples, line_thresholds({"black": black, "white": white})) == expected + ["green"] * 26


@pytest.mark.parametrize("readings", [[1] * 31, [1] * 33, [-1] * 32, [256] * 32, [True] * 32, [1.5] * 32])
def test_bad_readings_rejected(readings):
    with pytest.raises(ValueError):
        classify_readings(readings, {"black": 64, "white": 192})


def test_feed_saved_thresholds_deduplicates_and_clears(tmp_path):
    path = tmp_path / "line_sensor_calibration.json"
    path.write_text(json.dumps({"black": 200, "white": 40, "sensor_radius_mm": 75}))
    feed = LineSensorFeed(path, clock=lambda: 10, use_pcb=True)
    calls = []
    lidar = SimpleNamespace(set_line_readings=lambda *args: calls.append(args),
                            clear_line_readings=lambda: calls.append("clear"))
    snapshot = {"timestamp_s": 9.9, "readings": [0, 100, 255, 40] * 8, "valid": True}
    hardware = SimpleNamespace(get_pcb_snapshot=lambda: snapshot)
    feed.update(lidar, hardware)
    assert calls == [(["white", "green", "black", "white"] * 8, 9.9)]
    feed.update(lidar, hardware)
    assert len(calls) == 1
    for stamp, valid in [(9, True), (10.1, True), (float("nan"), True), (None, True), (9.95, False)]:
        snapshot.update(timestamp_s=stamp, valid=valid)
        feed.update(lidar, hardware)
        assert calls[-1] == "clear"
    snapshot.update(timestamp_s=9.99, valid=True)
    feed.update(lidar, hardware)
    assert calls[-1][1] == 9.99


@pytest.mark.parametrize("content", [None, "broken JSON", '{"black": 20, "white": 20}'])
def test_missing_or_invalid_calibration_never_uses_defaults(tmp_path, content):
    path = tmp_path / "line_sensor_calibration.json"
    if content is not None:
        path.write_text(content)
    feed = LineSensorFeed(path, use_pcb=True)
    cleared = []
    assert feed.error and feed.thresholds is None
    feed.update(SimpleNamespace(clear_line_readings=lambda: cleared.append(True)), object())
    assert cleared == [True]


def test_native_staging_does_not_change_pose(tmp_path):
    from lib import lidar

    lidar.test_mcl_start(2430, 1820, use_pcb=USE_PCB)
    try:
        pose = lidar.get_coordinates_info()
        timestamp = time.monotonic()
        colours = ["black", "white", "green", "green"] * 8
        path = tmp_path / "line_sensor_calibration.json"
        path.write_text('{"black": 64, "white": 192}')
        hardware = SimpleNamespace(get_pcb_snapshot=lambda: {
            "readings": [0, 255, 100, 150] * 8, "timestamp_s": timestamp, "valid": True,
        })
        LineSensorFeed(path, use_pcb=True).update(lidar, hardware)
        stored = lidar.get_line_readings()
        assert stored == {"colours": colours, "timestamp_s": timestamp, "valid": True,
                          "applied_count": 0, "last_applied_timestamp_s": 0.0}
        assert lidar.get_coordinates_info() == pose
        stored["colours"][0] = "green"
        assert lidar.get_line_readings()["colours"][0] == "black"
        with pytest.raises(ValueError):
            lidar.set_line_readings(["red"] * 32, timestamp)
        with pytest.raises(ValueError):
            lidar.set_line_readings(["green"], timestamp)
        lidar.clear_line_readings()
        assert not lidar.get_line_readings()["valid"]
        lidar.set_line_readings(colours, timestamp - 1)
        assert not lidar.get_line_readings()["valid"]
        lidar.set_line_readings(colours, time.monotonic())
        lidar.test_mcl_reset()
        assert not lidar.get_line_readings()["valid"]
    finally:
        lidar.test_mcl_stop()
    assert not lidar.get_line_readings()["valid"]
