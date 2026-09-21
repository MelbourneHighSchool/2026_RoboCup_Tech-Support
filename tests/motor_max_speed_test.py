"""Ramp native robot speed and report measured wheel-odometry speed."""

from __future__ import annotations

import argparse
import math
import sys
import time

from lib.hardware_test_utils import create_hardware, set_startup_yaw

USE_PCB = False

SETTLE_SECONDS = 0.5


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description="Ramp the complete native drive system.")
    parser.add_argument("--start", type=float, default=100, help="Initial speed in mm/s.")
    parser.add_argument("--step", type=float, default=100, help="Speed increment in mm/s.")
    parser.add_argument("--limit", type=float, default=2000, help="Maximum command in mm/s.")
    args = parser.parse_args(argv)
    if args.start < 0 or args.step <= 0 or args.limit < args.start:
        parser.error("require 0 <= --start <= --limit and --step > 0")

    hardware = create_hardware(use_pcb=USE_PCB)
    maximum = 0.0
    try:
        set_startup_yaw(hardware)
        requested = args.start
        while requested <= args.limit:
            hardware.move(0, requested, 0, 0)
            time.sleep(SETTLE_SECONDS)
            vx, vy = hardware.get_measured_body_velocity_mm_s(0)
            measured = math.hypot(vx, vy)
            maximum = max(maximum, measured)
            print(f"requested={requested:.0f} mm/s measured={measured:.1f} mm/s")
            requested += args.step
    except KeyboardInterrupt:
        print("\nStopping ramp.")
    finally:
        hardware.stop()
        print(f"Maximum measured speed: {maximum:.1f} mm/s")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
