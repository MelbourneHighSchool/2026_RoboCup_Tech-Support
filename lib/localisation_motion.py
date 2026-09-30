"""Timestamped hardware feed and bounded, asynchronous diagnostic recording."""

import atexit
import json
import math
import os
import queue
import threading
import time
from pathlib import Path

_captures = {}


class MotionCapture:
    def __init__(self, path, config):
        self.path = path
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        with open(path, "x", encoding="utf-8") as stream:
            stream.write(json.dumps({"type": "header", "version": 1, **config}) + "\n")
        self.queue = queue.Queue(maxsize=256)
        self.dropped = 0
        self.error = None
        self.last_line_time = 0
        self.thread = threading.Thread(target=self._write, daemon=True)
        self.thread.start()
        self.pump_stop = threading.Event()
        self.pump_thread = None

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
        self.pump_stop.set()
        if self.pump_thread is not None:
            self.pump_thread.join()
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
    """Configure once after starting localisation. Angular-speed gating is disabled."""
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
        capture = _captures[id(lidar)]
        if getattr(lidar, "autonomous_motion_enabled", lambda: False)():
            def pump():
                try:
                    while not capture.pump_stop.wait(0.02):
                        drain_native_capture(lidar, capture)
                except Exception as exc:
                    capture.error = str(exc)
            capture.pump_thread = threading.Thread(target=pump, name="motion-capture", daemon=True)
            capture.pump_thread.start()


def feed_timed_motion(lidar, hardware, *, sample=None):
    """Read wheel acquisition time and native IMU history, then feed one snapshot."""
    if sample is None:
        sample = hardware.get_localisation_sample()
    if not getattr(lidar, "autonomous_motion_enabled", lambda: False)():
        lidar.feed_motion(sample)
    capture = _captures.get(id(lidar))
    if capture and not getattr(lidar, "autonomous_motion_enabled", lambda: False)():
        capture.record({"type": "motion", "sample": sample})
        scans, dropped = lidar.drain_scan_capture()
        for scan in scans:
            capture.record({"type": "scan", **scan})
        capture.record({"type": "health", "dropped_scans": dropped,
                        "dropped_events": capture.dropped, "writer_error": capture.error})
    return sample


def record_floor(lidar):
    capture = _captures.get(id(lidar))
    if capture and not getattr(lidar, "autonomous_motion_enabled", lambda: False)():
        snapshot = lidar.get_line_readings()
        if snapshot["valid"] and snapshot["timestamp_s"] > capture.last_line_time:
            capture.last_line_time = snapshot["timestamp_s"]
            capture.record({"type": "floor", "sample": snapshot})


def record_diagnostic_event(lidar, event_type, **fields):
    """Append command/phase/live diagnostics to an active motion capture."""
    if event_type in {"header", "motion", "scan", "floor", "health", "footer"}:
        raise ValueError(f"{event_type!r} is reserved for the capture format")
    capture = _captures.get(id(lidar))
    if capture:
        capture.record({"type": event_type, **fields})


def drain_native_capture(lidar, capture, *, stop=False):
    events, dropped = lidar.drain_localisation_events(stop=stop)
    for event in events:
        capture.record(event)
    diagnostics = lidar.get_worker_diagnostics()
    capture.record({"type": "health", "dropped_scans": diagnostics["skipped_scans"],
                    "dropped_events": dropped + capture.dropped,
                    "dropped_motion": diagnostics["dropped_motion"],
                    "writer_error": capture.error or diagnostics["error"] or None})


def close_motion_capture(lidar):
    capture = _captures.pop(id(lidar), None)
    if capture:
        capture.pump_stop.set()
        if capture.pump_thread is not None:
            capture.pump_thread.join()
            drain_native_capture(lidar, capture, stop=True)
            capture.close()
            return
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
