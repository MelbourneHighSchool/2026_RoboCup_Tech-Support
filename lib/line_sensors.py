"""Classify PCB readings using the dashboard's saved raw-byte thresholds."""

import json
import math
import time
from pathlib import Path

CALIBRATION_FILE = Path(__file__).resolve().parents[1] / "line_sensor_calibration.json"
MAX_AGE_S = 0.5
USE_PCB = False


def line_thresholds(value):
    if not isinstance(value, dict):
        raise TypeError("Expected black and white thresholds")
    result = {}
    for colour in ("black", "white"):
        raw = value.get(colour)
        if isinstance(raw, bool):
            raise TypeError("Invalid line sensor threshold")
        try:
            level = float(raw)
        except (ValueError, TypeError) as exc:
            raise ValueError("Invalid line sensor threshold") from exc
        if not math.isfinite(level) or not level.is_integer() or not 0 <= level <= 255:
            raise ValueError("Line sensor thresholds must be whole numbers between 0 and 255")
        result[colour] = int(level)
    if result["black"] == result["white"]:
        raise ValueError("Black and white thresholds must differ; green is between them")
    return result


def classify_readings(readings, thresholds):
    """Return 32 colour names, in sensor order; threshold endpoints are inclusive."""
    if len(readings) != 32 or any(type(v) is not int or not 0 <= v <= 255 for v in readings):
        raise ValueError("Expected 32 integer sensor readings between 0 and 255")
    black, white = thresholds["black"], thresholds["white"]
    if black < white:
        return ["black" if v <= black else "white" if v >= white else "green" for v in readings]
    return ["black" if v >= black else "white" if v <= white else "green" for v in readings]


class LineSensorFeed:
    """Load once per session and forward each fresh snapshot once, without I2C I/O."""

    def __init__(self, path=CALIBRATION_FILE, clock=time.monotonic, *, use_pcb=USE_PCB):
        self.enabled = use_pcb
        self.clock = clock
        self.last_timestamp = None
        self.thresholds = None
        self.error = None
        if not use_pcb:
            return
        try:
            self.thresholds = line_thresholds(json.loads(Path(path).read_text()))
        except (OSError, ValueError, TypeError) as exc:
            # Never use provisional dashboard defaults as measured calibration.
            self.error = f"Line sensor classification disabled: {exc}"

    def update(self, lidar, hardware):
        if not self.enabled:
            return
        if self.thresholds is None:
            lidar.clear_line_readings()
            return
        snapshot = hardware.get_pcb_snapshot()
        timestamp = snapshot["timestamp_s"]
        if (not snapshot["valid"] or timestamp is None or not math.isfinite(timestamp)
                or not 0 <= self.clock() - timestamp <= MAX_AGE_S):
            lidar.clear_line_readings()
            return
        if self.last_timestamp is not None and timestamp <= self.last_timestamp:
            return
        colours = classify_readings(snapshot["readings"], self.thresholds)
        lidar.set_line_readings(colours, timestamp)
        self.last_timestamp = timestamp
