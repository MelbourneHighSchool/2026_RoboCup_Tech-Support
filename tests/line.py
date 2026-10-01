"""Drive forward until any PCB line sensor sees white."""

import argparse
import math
import sys
import time

from lib.hardware_test_utils import create_hardware, set_startup_yaw
from lib.line_sensors import MAX_AGE_S, LineSensorFeed, classify_readings

POLL_INTERVAL_S = 0.01
STARTUP_TIMEOUT_S = 5.0


def fresh_colours(hardware, thresholds):
    snapshot = hardware.get_pcb_snapshot()
    timestamp = snapshot["timestamp_s"]
    if (
        not snapshot["valid"]
        or timestamp is None
        or not math.isfinite(timestamp)
        or not 0 <= time.monotonic() - timestamp <= MAX_AGE_S
    ):
        return None
    return classify_readings(snapshot["readings"], thresholds)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("speed", type=float, help="Forward speed in mm/s")
    args = parser.parse_args(argv)
    if not math.isfinite(args.speed) or args.speed <= 0:
        parser.error("speed must be a positive, finite number")

    feed = LineSensorFeed(use_pcb=True)
    if feed.error:
        parser.error(feed.error)

    hardware = None
    try:
        hardware = create_hardware(use_pcb=True)
        set_startup_yaw(hardware)

        deadline = time.monotonic() + STARTUP_TIMEOUT_S
        colours = fresh_colours(hardware, feed.thresholds)
        while colours is None and time.monotonic() < deadline:
            time.sleep(POLL_INTERVAL_S)
            colours = fresh_colours(hardware, feed.thresholds)
        if colours is None:
            raise RuntimeError("No fresh PCB sensor readings received")
        if "white" in colours:
            print("White already detected; staying stopped.")
            return 0

        print(f"Moving forward at {args.speed:g} mm/s; Ctrl+C to stop.")
        hardware.move(0, args.speed, 0, 0)
        while True:
            colours = fresh_colours(hardware, feed.thresholds)
            if colours is None:
                raise RuntimeError("PCB sensor readings became stale; stopping")
            if "white" in colours:
                print("White detected; stopping.")
                return 0
            time.sleep(POLL_INTERVAL_S)
    except KeyboardInterrupt:
        print("\nStopping.")
        return 0
    finally:
        if hardware is not None:
            hardware.stop()


if __name__ == "__main__":
    sys.exit(main())
