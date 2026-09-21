"""Find the lowest tested drive current that reaches a requested QDR forward speed.

Run on the robot: python -m tests.speed_current --speed 500
QDR measures wheel-derived body velocity; it cannot distinguish wheel slip.
"""

import argparse
import math
import time

USE_PCB = False


def measure_trial(hardware, speed, duration, tolerance, hold):
    """Allow the entire trial to reach speed, then require a short stable interval."""
    started = time.monotonic()
    deadline = started + duration
    reached_at = None
    next_report = started
    peak = 0.0
    hardware.move(0, speed, 0, 1.0)
    while time.monotonic() < deadline:
        vx, vy = hardware.get_measured_body_velocity_mm_s(0)
        now = time.monotonic()
        if not math.isfinite(vx) or not math.isfinite(vy):
            raise RuntimeError("Non-finite QDR velocity")
        peak = max(peak, vx)
        error = abs(vx - speed) / speed
        if error <= tolerance:
            if reached_at is None:
                reached_at = now
            if now - reached_at >= hold:
                print(f"  PASS: forward={vx:.1f} mm/s, error={error:.1%}, "
                      f"reached after {reached_at - started:.2f}s", flush=True)
                return True
        else:
            reached_at = None
        if now >= next_report:
            print(f"  t={now - started:.1f}s forward={vx:.1f} sideways={vy:.1f} mm/s "
                  f"target={speed:g} error={error:.1%}", flush=True)
            next_report = now + 0.5
        time.sleep(min(0.05, max(0, deadline - time.monotonic())))
    print(f"  No pass within {duration:g}s; peak forward={peak:.1f} mm/s", flush=True)
    return False


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--speed", type=float, default=500, help="Target forward speed (mm/s).")
    parser.add_argument("--start-current", type=float, default=0.25, help="Initial current (amps).")
    parser.add_argument("--step", type=float, default=0.25, help="Current increment (amps).")
    parser.add_argument("--max-current", type=float, default=8.0, help="Last current to try (amps).")
    parser.add_argument("--duration", type=float, default=10.0, help="Seconds allowed per current.")
    parser.add_argument("--tolerance", type=float, default=0.05, help="Fractional speed error (0.05 = 5%%).")
    parser.add_argument("--hold", type=float, default=0.25,
                        help="Seconds continuously within tolerance; 0 accepts one sample.")
    args = parser.parse_args(argv)
    if not all(math.isfinite(value) for value in vars(args).values()):
        parser.error("all parameters must be finite")
    if not 0 < args.start_current <= args.max_current <= 8 or args.step < 1 / 65536:
        parser.error("require 0 < start-current <= max-current <= 8 amps and step >= 1/65536 A")
    if args.speed <= 0 or args.duration <= 0 or not 0 <= args.hold < args.duration:
        parser.error("require speed > 0, duration > 0 and 0 <= hold < duration")
    if not 0 <= args.tolerance < 1:
        parser.error("tolerance must be in [0, 1)")

    # Import hardware only after parsing so --help works on a desktop.
    from lib.hardware_controller import HardwareController

    from lib.config import load_config
    from lib.hardware_test_utils import (
        MAX_MOTOR_RPM,
        MAX_YAW_RPM,
        WHEEL_DIAMETER,
        YAW_CORRECT_THRESHOLD,
        set_startup_yaw,
    )

    forward_wheel_rpm = args.speed * 60 / (WHEEL_DIAMETER * math.pi * math.sqrt(2))
    if forward_wheel_rpm + MAX_YAW_RPM > MAX_MOTOR_RPM:
        parser.error("target exceeds the wheel RPM limit with heading-correction headroom")
    config = load_config()
    current = args.start_current
    print(f"Target {args.speed:g} mm/s; tolerance {args.tolerance:.1%}; "
          f"up to {args.duration:g}s per current. Ctrl+C stops the test.", flush=True)
    hardware = None
    try:
        hardware = HardwareController.from_i2c_addresses(
            config.i2c_addresses, WHEEL_DIAMETER, MAX_YAW_RPM,
            MAX_MOTOR_RPM, YAW_CORRECT_THRESHOLD,
            drive_motor_current_limit=current, use_pcb=USE_PCB)
        set_startup_yaw(hardware)
        while True:
            print(f"Testing {current:g} A drive current", flush=True)
            try:
                hardware.set_drive_current_limits(current, current)
                passed = measure_trial(hardware, args.speed, args.duration, args.tolerance, args.hold)
            finally:
                hardware.move(0, 0, 0, 0)
            if passed:
                print(f"Lowest passing tested current: {current:g} A at {args.speed:g} mm/s.")
                return 0
            if current >= args.max_current:
                print("No tested current reached the target within the trial duration.")
                return 1
            current = min(round(current + args.step, 10), args.max_current)
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\nCurrent sweep stopped.")
        return 130
    finally:
        if hardware is not None:
            hardware.stop()


if __name__ == "__main__":
    raise SystemExit(main())
