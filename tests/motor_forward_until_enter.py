"""Drive forward at a fixed linear speed until Enter is pressed."""

from __future__ import annotations

import argparse
import sys

from lib.hardware_test_utils import create_hardware, set_startup_yaw

USE_PCB = False


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description="Drive forward until Enter is pressed.")
    parser.add_argument(
        "--speed", type=float, default=100, help="Robot speed in mm/s (default: 100)."
    )
    args = parser.parse_args(argv)
    if args.speed < 0:
        parser.error("--speed must be non-negative")

    hardware = create_hardware(use_pcb=USE_PCB)
    try:
        set_startup_yaw(hardware)
        print(f"Commanding forward speed: {args.speed:g} mm/s")
        print("Press Enter to stop...")
        hardware.move(0, args.speed, 0, 0)
        input()
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received; stopping...")
    finally:
        hardware.stop()
        print("Stopped. Exiting.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
