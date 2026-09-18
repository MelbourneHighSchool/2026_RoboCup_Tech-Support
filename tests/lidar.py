#!/usr/bin/env python3
"""Print LIDAR range at 0, 90, 180, and 270 degrees once per completed scan."""

import sys
import time

from lib import lidar

LIDAR_BAUDRATE = 460800
CARDINAL_ANGLES = (0.0, 90.0, 180.0, 270.0)


def format_distance(distance_mm):
    if distance_mm is None or distance_mm < 0:
        return "none"
    return f"{distance_mm:.0f} mm"


def main():
    from lib.config import load_config

    lidar_port = load_config().lidar_port
    print("LIDAR range test")
    print("=" * 40)
    print(f"Port: {lidar_port}, baud: {LIDAR_BAUDRATE}")
    print()

    try:
        lidar.init(lidar_port, LIDAR_BAUDRATE)
    except Exception as e:
        print(f"Failed to initialize LIDAR: {e}", file=sys.stderr)
        sys.exit(1)

    print("Waiting for first scan...")
    while not lidar.is_scan_ready():
        time.sleep(0.01)
    print("Scan ready. Printing ranges (Ctrl+C to stop):\n")

    last_generation = 0
    try:
        while True:
            generation = lidar.get_scan_generation()
            if generation == last_generation:
                time.sleep(0.005)
                continue
            last_generation = generation

            ranges = [
                f"{int(angle)}={format_distance(lidar.get_distance_at_angle(angle))}"
                for angle in CARDINAL_ANGLES
            ]
            print(f"scan#{generation} {' '.join(ranges)}")
    except KeyboardInterrupt:
        print("\nInterrupted by user")

    print("Shutting down LIDAR...")
    lidar.shutdown()
    print("Done.")


if __name__ == "__main__":
    main()
