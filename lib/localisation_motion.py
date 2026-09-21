"""Timestamped hardware feed and bounded, asynchronous diagnostic recording."""

import atexit
import json
import math
import os
import queue
import threading
import time

_captures = {}


class MotionCapture:
    def __init__(self, path, config):
        self.path = path
        with open(path, "x", encoding="utf-8") as stream:
            stream.write(json.dumps({"type": "header", "version": 1, **config}) + "\n")
        self.queue = queue.Queue(maxsize=256)
        self.dropped = 0
        self.error = None
        self.last_line_time = 0
        self.thread = threading.Thread(target=self._write, daemon=True)
        self.thread.start()

    def _write(self):
        try:
            with open(self.path, "a", encoding="utf-8") as stream:
                while True:
                    item = self.queue.get()
                    if item is None:
                        stream.write(json.dumps({"type": "footer", "dropped_events": self.dropped}) + "\n")
                        break
                    stream.write(json.dumps(item, allow_nan=False) + "\n")
                    stream.flush()
        except (OSError, ValueError) as exc:
            self.error = str(exc)

    def record(self, event):
        if self.error:
            return
        try:
            self.queue.put_nowait({"recorded_s": time.monotonic(), **event})
        except queue.Full:
            self.dropped += 1

    def close(self):
        # Never block on a failed writer with a full queue.
        while self.thread.is_alive():
            try:
                self.queue.put(None, timeout=0.1)
                break
            except queue.Full:
                continue
        self.thread.join()
        if self.error or self.dropped:
            print(f"Localisation capture: dropped={self.dropped}, error={self.error}")


def configure_motion(lidar, *, use_pcb=False, pitch=(2430, 1820), motion_noise=0.30):
    """Configure once after starting localisation. Native rotation gate stays on."""
    if not hasattr(lidar, "configure_deskew"):
        return  # Test doubles and older hardware-free utilities.
    mode = os.environ.get("SOCCER_DESKEW", "full")
    mount = [float(os.environ.get(f"SOCCER_LIDAR_{key}", "0"))
             for key in ("FORWARD_MM", "LEFT_MM", "YAW_DEG")]
    if not all(math.isfinite(value) for value in mount):
        raise ValueError("LIDAR mount offsets must be finite")
    lidar.configure_deskew(mode, *mount)
    close_motion_capture(lidar)
    path = os.environ.get("SOCCER_LOCALISATION_RECORD")
    if path:
        config = {"mode": mode, "mount": mount, "pitch": pitch, "use_pcb": use_pcb,
                  "motion_noise": motion_noise, "seed": 42}
        _captures[id(lidar)] = MotionCapture(path, config)
        lidar.enable_scan_capture(True)


def feed_timed_motion(lidar, hardware, *, sample=None):
    """Read wheel acquisition time and native IMU history, then feed one snapshot."""
    if sample is None:
        sample = hardware.get_localisation_sample()
    lidar.feed_motion(sample)
    capture = _captures.get(id(lidar))
    if capture:
        capture.record({"type": "motion", "sample": sample})
        scans, dropped = lidar.drain_scan_capture()
        for scan in scans:
            capture.record({"type": "scan", **scan})
        capture.record({"type": "health", "dropped_scans": dropped,
                        "dropped_events": capture.dropped, "writer_error": capture.error})
    return sample


def record_floor(lidar):
    capture = _captures.get(id(lidar))
    if capture:
        snapshot = lidar.get_line_readings()
        if snapshot["valid"] and snapshot["timestamp_s"] > capture.last_line_time:
            capture.last_line_time = snapshot["timestamp_s"]
            capture.record({"type": "floor", "sample": snapshot})


def close_motion_capture(lidar):
    capture = _captures.pop(id(lidar), None)
    if capture:
        scans, dropped = lidar.drain_scan_capture(stop=True)
        for scan in scans:
            capture.record({"type": "scan", **scan})
        capture.record({"type": "health", "dropped_events": capture.dropped,
                        "dropped_scans": dropped, "writer_error": capture.error})
        capture.close()


@atexit.register
def _close_remaining():
    for capture in list(_captures.values()):
        capture.close()
    _captures.clear()
