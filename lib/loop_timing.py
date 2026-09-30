"""Monotonic loop pacing and reporting independent of game logic."""

import math
import threading
import time
from collections import deque


def percentile(values, fraction):
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, math.ceil(len(ordered) * fraction) - 1)] if ordered else 0.0


class RateLimiter:
    """One wait at the iteration boundary; skip slots missed by slow work."""

    def __init__(self, hz, *, clock=time.monotonic, sleep=time.sleep):
        self.period = 1.0 / hz
        self.clock = clock
        self.sleep = sleep
        self.next_deadline = None
        self.started = None
        self.missed_deadlines = 0
        self._work = deque(maxlen=1500)
        self._ages = deque(maxlen=1500)
        self._lock = threading.Lock()

    def wait(self):
        now = self.clock()
        if self.started is not None:
            with self._lock:
                self._work.append(now - self.started)
        if self.next_deadline is None:
            self.next_deadline = now
        elif now > self.next_deadline:
            missed = math.floor((now - self.next_deadline) / self.period) + 1
            self.missed_deadlines += missed
            self.next_deadline += missed * self.period
        remaining = self.next_deadline - now
        if remaining > 0:
            self.sleep(remaining)
        self.started = self.clock()
        self.next_deadline += self.period

    def observe_pose_age(self, age):
        if age >= 0:
            with self._lock:
                self._ages.append(age)

    def diagnostics(self):
        with self._lock:
            work, ages = list(self._work), list(self._ages)
        return {
            "logic_missed": self.missed_deadlines,
            "logic_work_p95_ms": round(percentile(work, 0.95) * 1000, 2),
            "logic_work_max_ms": round(max(work, default=0) * 1000, 2),
            "pose_age_p95_ms": round(percentile(ages, 0.95) * 1000, 2),
            "pose_age_max_ms": round(max(ages, default=0) * 1000, 2),
        }


class FpsMonitor:
    """Sample counters and print from a dedicated reporting thread."""

    def __init__(self, interval_s=1.0):
        self.interval_s = interval_s
        self.sources = []
        self.diagnostic_sources = []
        self._stop = threading.Event()
        self._thread = None

    def add(self, name, getter):
        self.sources.append((name, getter))

    def add_diagnostics(self, getter):
        self.diagnostic_sources.append(getter)

    def start(self):
        self._thread = threading.Thread(target=self._run, name="fps-report", daemon=True)
        self._thread.start()

    def _run(self):
        last_time = time.monotonic()
        counts = {name: int(getter()) for name, getter in self.sources}
        while not self._stop.wait(self.interval_s):
            now = time.monotonic()
            parts = []
            try:
                for name, getter in self.sources:
                    count = int(getter())
                    parts.append(f"{name}={(count - counts[name]) / (now - last_time):.1f}")
                    counts[name] = count
                for getter in self.diagnostic_sources:
                    parts.extend(f"{key}={value}" for key, value in getter().items())
                print("FPS " + " ".join(parts), flush=True)
            except Exception as exc:
                print(f"FPS reporting failed: {exc}", flush=True)
            last_time = now

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2)
