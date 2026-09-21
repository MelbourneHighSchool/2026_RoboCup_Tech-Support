"""Find the lowest tested current that reaches speed by an acceleration deadline.

Run: python -m tests.acceleration_current --acceleration 1000 --speed 500
Acceleration is in mm/s². QDR velocity is wheel-derived and includes wheel slip.
"""

import argparse
import math
import time

USE_PCB = False


def wait_stationary(hardware):
    hardware.move(0, 0, 0, 0)
    deadline = time.monotonic() + 5
    stationary_since = None
    while time.monotonic() < deadline:
        vx, vy = hardware.get_measured_body_velocity_mm_s(0)
        now = time.monotonic()
        if math.hypot(vx, vy) <= 5:
            if stationary_since is None:
                stationary_since = now
            if now - stationary_since >= 0.25:
                return
        else:
            stationary_since = None
        time.sleep(0.02)
    raise RuntimeError("QDR did not settle below 5 mm/s before the acceleration trial")


def measure_trial(hardware, speed, acceleration, grace, tolerance, hold):
    """Ramp targets by elapsed time; require final speed during the post-ramp window."""
    wait_stationary(hardware)
    ramp_duration = speed / acceleration
    started = time.monotonic()
    deadline = started + ramp_duration + grace
    reached_at = None
    next_report = started
    vx = 0.0
    while True:
        now = time.monotonic()
        if now >= deadline:
            break
        elapsed = now - started
        command = min(speed, acceleration * elapsed)
        hardware.move(0, command, 0, 1.0)
        vx, vy = hardware.get_measured_body_velocity_mm_s(0)
        now = time.monotonic()
        if not math.isfinite(vx) or not math.isfinite(vy):
            raise RuntimeError("Non-finite QDR velocity")
        error = abs(vx - speed) / speed
        # Only count samples after sending the final target, and within the deadline.
        if now > deadline:
            break
        if command == speed and error <= tolerance:
            if reached_at is None:
                reached_at = now
            if now - reached_at >= hold:
                lag = reached_at - started - ramp_duration
                print(f"  PASS: forward={vx:.1f} mm/s, error={error:.1%}, "
                      f"reached {lag:.3f}s after ramp end", flush=True)
                return True
        else:
            reached_at = None
        if now >= next_report:
            print(f"  t={now - started:.2f}s command={command:.1f} "
                  f"forward={vx:.1f} sideways={vy:.1f} mm/s", flush=True)
            next_report = now + 0.2
        time.sleep(min(0.02, max(0, deadline - time.monotonic())))
    print(f"  FAIL: final QDR forward={vx:.1f} mm/s; target={speed:g} mm/s "
          f"not confirmed by {ramp_duration + grace:.3f}s", flush=True)
    return False


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--speed", type=float, default=500, help="Final forward speed (mm/s).")
    parser.add_argument("--acceleration", type=float, default=1000, help="Target acceleration (mm/s²).")
    parser.add_argument("--start-current", type=float, default=0.25, help="Initial current (amps).")
    parser.add_argument("--step", type=float, default=0.25, help="Current increment (amps).")
    parser.add_argument("--max-current", type=float, default=8.0, help="Last current to try (amps).")
    parser.add_argument("--grace", type=float, default=0.25, help="Seconds allowed after ramp end.")
    parser.add_argument("--tolerance", type=float, default=0.05, help="Fractional speed error (0.05 = 5%%).")
    parser.add_argument("--hold", type=float, default=0.1,
                        help="Seconds within tolerance after ramp end; 0 accepts one sample.")
    args = parser.parse_args(argv)
    if not all(math.isfinite(value) for value in vars(args).values()):
        parser.error("all parameters must be finite")
    if not 0 < args.start_current <= args.max_current <= 8 or args.step < 1 / 65536:
        parser.error("require 0 < start-current <= max-current <= 8 amps and step >= 1/65536 A")
    if args.speed <= 0 or not 0 < args.acceleration <= 8000:
        parser.error("require speed > 0 and 0 < acceleration <= native limit of 8000 mm/s²")
    if not 0 <= args.hold < args.grace or not 0 <= args.tolerance < 1:
        parser.error("require 0 <= hold < grace and 0 <= tolerance < 1")

    from lib.hardware_controller import HardwareController

    from lib.config import load_config
    from lib.hardware_test_utils import (
        MAX_MOTOR_RPM,
        MAX_YAW_RPM,
        WHEEL_DIAMETER,
        YAW_CORRECT_THRESHOLD,
        set_startup_yaw,
    )

    rpm = args.speed * 60 / (WHEEL_DIAMETER * math.pi * math.sqrt(2))
    if rpm + MAX_YAW_RPM > MAX_MOTOR_RPM:
        parser.error("target exceeds wheel RPM limit with heading-correction headroom")
    config = load_config()
    current = args.start_current
    print(f"Ramp to {args.speed:g} mm/s at {args.acceleration:g} mm/s² "
          f"in {args.speed / args.acceleration:.3f}s; grace={args.grace:g}s. Ctrl+C stops.")
    hardware = None
    try:
        hardware = HardwareController.from_i2c_addresses(
            config.i2c_addresses, WHEEL_DIAMETER, MAX_YAW_RPM,
            MAX_MOTOR_RPM, YAW_CORRECT_THRESHOLD, drive_motor_current_limit=current, use_pcb=USE_PCB)
        set_startup_yaw(hardware)
        while True:
            print(f"Testing {current:g} A drive current", flush=True)
            try:
                hardware.set_drive_current_limits(current, current)
                passed = measure_trial(hardware, args.speed, args.acceleration,
                                       args.grace, args.tolerance, args.hold)
            finally:
                hardware.move(0, 0, 0, 0)
            if passed:
                print(f"Lowest passing tested current: {current:g} A for "
                      f"{args.acceleration:g} mm/s² to {args.speed:g} mm/s.")
                return 0
            if current >= args.max_current:
                print("No tested current met the acceleration deadline.")
                return 1
            current = min(round(current + args.step, 10), args.max_current)
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\nAcceleration sweep stopped.")
        return 130
    finally:
        if hardware is not None:
            hardware.stop()


if __name__ == "__main__":
    raise SystemExit(main())
