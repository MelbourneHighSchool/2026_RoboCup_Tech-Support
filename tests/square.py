from __future__ import annotations

import argparse
import sys
import time

from lib.hardware_controller import MotorCommunicationError

from lib.hardware_test_utils import create_hardware, set_startup_yaw

USE_PCB = False

SQUARE_DIRECTIONS = (0, 90, 180, 270)
DEFAULT_SIDE_SECONDS = 1.0
DEFAULT_SPEED = 100  # mm/s


def _run_side(direction, duration, speed, movement_controller):
    print(f"Moving at {direction} degrees for {duration:.2f} seconds")
    end_time = time.monotonic() + duration
    while time.monotonic() < end_time:
        movement_controller.move(direction, speed, 0, 0.0, 0)


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description="Move the robot in a one-second-per-side square.")
    parser.add_argument("--speed", type=float, default=DEFAULT_SPEED, help="Translation speed in mm/s.")
    parser.add_argument(
        "--side-seconds",
        type=float,
        default=DEFAULT_SIDE_SECONDS,
        help="Seconds to drive each side of the square.",
    )
    args = parser.parse_args(argv)

    if args.speed < 0:
        parser.error("--speed must be non-negative")
    if args.side_seconds <= 0:
        parser.error("--side-seconds must be positive")

    hardware = None
    try:
        hardware = create_hardware(max_motor_rpm=400, use_pcb=USE_PCB)
        set_startup_yaw(hardware)
        print("Running square path. Press Ctrl+C to stop.")
        for direction in SQUARE_DIRECTIONS:
            _run_side(direction, args.side_seconds, args.speed, hardware)
    except KeyboardInterrupt:
        print("\nStopping square path.")
    except MotorCommunicationError as exc:
        print(exc)
        raise
    finally:
        if hardware is not None:
            hardware.stop()
            print("Motors stopped.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
