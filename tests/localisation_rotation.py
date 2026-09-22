"""Record motor-driven clockwise/anticlockwise localisation rotation trials."""

import argparse
import math
import os
import time


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--motor-rpm", type=float, default=100,
                        help="Yaw wheel RPM limit, 1–400 (default: 100); not body deg/s")
    parser.add_argument("--duration", type=float, default=5,
                        help="Seconds rotating in each direction, 0–30 (default: 5)")
    parser.add_argument("--direction", choices=("cw", "ccw", "both"), default="both")
    parser.add_argument("--deskew", choices=("off", "rotation", "full"), default="rotation")
    parser.add_argument("--record-motion", required=True, help="New JSONL capture filename")
    args = parser.parse_args()
    for name, limit in (("motor_rpm", 400), ("duration", 30)):
        value = getattr(args, name)
        if not math.isfinite(value) or not 0 < value <= limit:
            parser.error(f"--{name.replace('_', '-')} must be greater than 0 and at most {limit}")
    return args


def main():
    args = parse_args()
    from lib.hardware_controller import HardwareController

    from lib import lidar
    from lib.config import load_config
    from lib.localisation_motion import close_motion_capture, record_diagnostic_event
    from lib.localisation_service import (
        LidarVelocityEstimator,
        LocalisationSession,
        capture_startup_yaw,
        feed_imu_yaw_prior,
    )

    os.environ["SOCCER_DESKEW"] = args.deskew
    os.environ["SOCCER_LOCALISATION_RECORD"] = args.record_motion
    hardware = None
    try:
        config = load_config()
        lidar.init(config.lidar_port, 460800)
        hardware = HardwareController.from_i2c_addresses(
            config.i2c_addresses, 50, args.motor_rpm, args.motor_rpm, 3, use_pcb=False,
        )
        startup = capture_startup_yaw(hardware)
        feed_imu_yaw_prior(lidar, hardware, startup)
        lidar.start_coordinates(2430, 1820, use_pcb=False)
        session = LocalisationSession(lidar, hardware, startup, LidarVelocityEstimator(), use_pcb=False)
        deadline = time.monotonic() + 30
        print("Waiting for initial localisation (30 second timeout)...")
        while True:
            state = session.tick()
            if state["pose"][4] and state["fresh"]:
                break
            if time.monotonic() > deadline:
                raise TimeoutError("Initial localisation unavailable")
            time.sleep(0.02)

        directions = [1, -1] if args.direction == "both" else [1 if args.direction == "cw" else -1]
        phases = [("stationary", 0, 3)]
        for sign in directions:
            phases.extend([("clockwise" if sign > 0 else "anticlockwise", sign, args.duration),
                           ("stationary", 0, 3)])
        for label, sign, duration in phases:
            print(f"{label}: {duration:g}s; yaw wheel limit {args.motor_rpm:g} RPM")
            record_diagnostic_event(
                lidar,
                "phase",
                event="start",
                name=label,
                direction_sign=sign,
                duration_s=duration,
                command_speed_mm_s=0,
                yaw_wheel_limit_rpm=args.motor_rpm,
            )
            end = time.monotonic() + duration
            last_print = 0
            while time.monotonic() < end:
                state = session.tick()
                yaw, rate = hardware.get_yaw(), hardware.get_gyro_z_deg_s()
                if (any(v is None or not math.isfinite(v) for v in (yaw, rate))
                        or state["scan_age_s"] > 0.5 or state["imu_age_s"] > 0.1):
                    raise RuntimeError("Rotation stopped: scan or IMU data unavailable/stale")
                # Keep a moving heading target so rotation does not settle at a fixed angle.
                hardware.move(0, 0, yaw + sign * 90, 1.0 if sign else 0.0, 0)
                now = time.monotonic()
                if now - last_print >= 0.2:
                    deskew = state.get("deskew", {})
                    print(f"gyro={rate:+.1f} deg/s pose_ok={state['pose'][4]} "
                          f"scan={deskew.get('reason')} residual="
                          f"{deskew.get('raw_residual_mm', -1):.1f}->"
                          f"{deskew.get('corrected_residual_mm', -1):.1f} mm")
                    record_diagnostic_event(
                        lidar,
                        "live",
                        phase=label,
                        direction_sign=sign,
                        gyro_deg_s=rate,
                        state=state,
                        timing=hardware.timing_diagnostics(),
                    )
                    last_print = now
                time.sleep(0.02)
            hardware.move(0, 0, yaw, 0.0, 0)
            record_diagnostic_event(
                lidar,
                "phase",
                event="end",
                name=label,
                direction_sign=sign,
            )
    except KeyboardInterrupt:
        print("Stopping rotation test.")
    finally:
        try:
            if hardware is not None:
                try:
                    hardware.move(0, 0, 0, 0, 0)
                    record_diagnostic_event(
                        lidar,
                        "command",
                        phase="cleanup",
                        direction_deg=0,
                        speed_mm_s=0,
                        rotation_deg=0,
                        rotation_strength=0,
                    )
                except Exception as exc:
                    print(f"Warning: zero command before stop failed: {exc}")
                hardware.stop()
        finally:
            try:
                close_motion_capture(lidar)
            finally:
                lidar.shutdown()


if __name__ == "__main__":
    main()
