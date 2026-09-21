"""Session-only dashboard polling settings and measured one-second windows."""

import math
import time

DEFAULT_RATES = {"motor_hz": 50, "odometry_hz": 50, "imu_hz": 100,
                 "imu_poll_hz": 500, "pcb_hz": 50}
RATE_CHOICES = {"motor_hz": (25, 50, 100, 200), "odometry_hz": (25, 50, 100, 200),
                "imu_hz": (50, 100, 200, 400), "imu_poll_hz": (100, 200, 500, 1000),
                "pcb_hz": (10, 25, 50, 100, 200)}


def polling_rates(data):
    rates = {}
    for name, default in DEFAULT_RATES.items():
        value = float(data.get(name, default))
        if not math.isfinite(value) or value not in RATE_CHOICES[name]:
            raise ValueError(f"Unsupported {name}: {value}")
        rates[name] = value
    return rates


class PollingMeasurements:
    def __init__(self, controller, rates, clock=time.monotonic):
        self.rates = rates
        self.clock = clock
        self.previous = controller.timing_diagnostics()
        self.started = clock()
        self.latest = {"requested": rates, "window_s": 0, "measured": {}, "native": {}}

    def update(self, controller):
        now = self.clock()
        elapsed = now - self.started
        if elapsed < 1:
            return self.latest
        current = controller.timing_diagnostics()
        measured = {}
        for name in ("motor", "odometry", "pcb", "imu_poll", "yaw", "gyro"):
            count = current.get(name + "_count", 0) - self.previous.get(name + "_count", 0)
            measured[name + "_hz"] = count / elapsed
            for metric in ("wait", "work"):
                key = name + "_" + metric + "_s"
                measured[name + "_" + metric + "_ms"] = (
                    1000 * (current.get(key, 0) - self.previous.get(key, 0)) / count
                    if count else None
                )
        measured["motor_overruns"] = current.get("motor_overruns", 0) - self.previous.get("motor_overruns", 0)
        self.latest = {"requested": self.rates, "window_s": elapsed,
                       "measured": measured, "native": current}
        self.previous, self.started = current, now
        return self.latest
