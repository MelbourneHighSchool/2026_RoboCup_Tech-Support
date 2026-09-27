"""Convert libcamera CLOCK_BOOTTIME exposure timestamps to CLOCK_MONOTONIC."""

import math
import time


def exposure_monotonic(metadata):
    """Return exposure midpoint seconds, or None when timing is untrustworthy.

    SensorTimestamp is the first-row exposure start, not callback arrival. Read
    both Linux clocks together on every frame, including after system suspend.
    Rolling-shutter readout uncertainty is handled by the fusion timing margin.
    """
    stamp = metadata.get("SensorTimestamp")
    if stamp is None or not hasattr(time, "CLOCK_BOOTTIME"):
        return None
    before = time.monotonic()
    boot = time.clock_gettime(time.CLOCK_BOOTTIME)
    after = time.monotonic()
    try:
        exposure = float(metadata.get("ExposureTime", 0)) * 1e-6
        value = float(stamp) * 1e-9 + (before + after) / 2 - boot + exposure / 2
    except (ValueError, TypeError, OverflowError):
        return None
    if (after - before > 0.001 or not math.isfinite(value)
            or not math.isfinite(exposure) or exposure < 0
            or value <= 0 or not 0 <= after - value <= 1):
        return None
    return value
