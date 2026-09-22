"""Run bounded translation, reversal, and combined-motion localisation trials.

Examples:
    python -m tests.localisation_trajectory --trajectory translation --speed 500 \
        --direction-deg 0 --travel-mm 600 --record-motion run/motion.jsonl
    python -m tests.localisation_trajectory --trajectory arc --speed 300 \
        --direction-deg 0 --duration 2 --yaw-rpm 100 --record-motion run/motion.jsonl

Directions are clockwise-positive field headings. With the robot initially facing
field +x, 0 is forward, 90 is right, -90 is left, and 180 is backward. Arc
directions are body-relative offsets while the robot continuously turns.
"""

import argparse
import math
import os
import time

PITCH_X_MM = 2430.0
PITCH_Y_MM = 1820.0
WHEEL_DIAMETER_MM = 50.0
MAX_TRANSLATION_MM_S = 2000.0
MAX_YAW_RPM = 400.0
MAX_CONTROLLER_RPM = 1000.0
NATIVE_ACCELERATION_MM_S2 = 8000.0


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--trajectory",
        choices=("translation", "reversal", "arc"),
        required=True,
    )
    parser.add_argument("--speed", type=float, required=True, help="Translation command in mm/s.")
    parser.add_argument(
        "--direction-deg",
        type=float,
        default=0,
        help="Field direction, or body-relative direction for arcs (default: 0).",
    )
    parser.add_argument(
        "--travel-mm",
        type=float,
        default=600,
        help="Nominal distance per translation leg; used to derive duration (default: 600).",
    )
    parser.add_argument(
        "--duration",
        type=float,
        help="Seconds per moving phase; required for arcs, overrides --travel-mm otherwise.",
    )
    parser.add_argument("--yaw-rpm", type=float, default=100, help="Yaw wheel RPM limit (default: 100).")
    parser.add_argument(
        "--turn-direction",
        choices=("cw", "ccw", "both"),
        default="both",
        help="Arc turn sign(s) (default: both).",
    )
    parser.add_argument("--repetitions", type=int, default=1)
    parser.add_argument("--stationary-s", type=float, default=3)
    parser.add_argument("--pause-s", type=float, default=2)
    parser.add_argument(
        "--wall-margin-mm",
        type=float,
        default=180,
        help="Abort when a valid pose enters this wall margin (default: 180).",
    )
    parser.add_argument("--deskew", choices=("off", "rotation", "full"), default="full")
    parser.add_argument("--motion-noise", type=float, default=0.30)
    parser.add_argument("--record-motion", required=True, help="New JSONL capture filename.")
    args = parser.parse_args(argv)

    finite_values = (
        args.speed,
        args.direction_deg,
        args.travel_mm,
        args.yaw_rpm,
        args.stationary_s,
        args.pause_s,
        args.wall_margin_mm,
        args.motion_noise,
    )
    if args.duration is not None:
        finite_values += (args.duration,)
    if not all(math.isfinite(value) for value in finite_values):
        parser.error("all numeric parameters must be finite")
    if not 0 < args.speed <= MAX_TRANSLATION_MM_S:
        parser.error("--speed must be greater than zero and at most 2000 mm/s")
    if not 0 < args.yaw_rpm <= MAX_YAW_RPM:
        parser.error("--yaw-rpm must be greater than zero and at most 400")
    if args.repetitions not in range(1, 11):
        parser.error("--repetitions must be between 1 and 10")
    if args.travel_mm <= 0 or args.stationary_s < 0 or args.pause_s < 0:
        parser.error("travel must be positive and stationary/pause durations nonnegative")
    if not 110 <= args.wall_margin_mm < min(PITCH_X_MM, PITCH_Y_MM) / 2:
        parser.error("--wall-margin-mm must be at least 110 and less than half the short pitch axis")
    if args.motion_noise < 0:
        parser.error("--motion-noise must be nonnegative")
    if args.trajectory == "arc" and args.duration is None:
        parser.error("--duration is required for arc trials")

    if args.duration is None:
        args.duration = args.travel_mm / args.speed
    if not 0 < args.duration <= 10:
        parser.error("--duration must be greater than zero and at most 10 seconds")
    ramp_s = args.speed / NATIVE_ACCELERATION_MM_S2
    if args.duration < ramp_s:
        parser.error(
            f"moving phase is shorter than the {ramp_s:.3f}s native acceleration ramp; "
            "increase --travel-mm or --duration"
        )
    return args


def _safe_state(state, hardware, wall_margin_mm, *, require_pose):
    yaw = hardware.get_yaw()
    rate = hardware.get_gyro_z_deg_s()
    if any(value is None or not math.isfinite(value) for value in (yaw, rate)):
        raise RuntimeError("Trial stopped: yaw or gyro is unavailable")
    if state["scan_age_s"] > 0.5 or state["imu_age_s"] > 0.1:
        raise RuntimeError("Trial stopped: scan or IMU data is stale")
    x_pos, y_pos, _mcl_yaw, _confidence, pose_ok = state["pose"]
    if require_pose and (not pose_ok or not state["fresh"]):
        raise RuntimeError("Trial stopped: localisation pose was lost or stale during translation")
    if pose_ok and not (
        wall_margin_mm <= x_pos <= PITCH_X_MM - wall_margin_mm
        and wall_margin_mm <= y_pos <= PITCH_Y_MM - wall_margin_mm
    ):
        raise RuntimeError(
            f"Trial stopped: pose ({x_pos:.1f}, {y_pos:.1f}) entered "
            f"the {wall_margin_mm:g} mm wall margin"
        )
    return yaw, rate


def _record_live(lidar, hardware, state, phase, command):
    from lib.localisation_motion import record_diagnostic_event

    record_diagnostic_event(
        lidar,
        "live",
        phase=phase,
        command=command,
        gyro_deg_s=hardware.get_gyro_z_deg_s(),
        state=state,
        timing=hardware.timing_diagnostics(),
    )


def _number(value, precision=1):
    return "n/a" if value is None else f"{value:.{precision}f}"


def _run_phase(
    lidar,
    hardware,
    session,
    *,
    name,
    duration_s,
    speed_mm_s,
    direction_deg,
    wall_margin_mm,
    turn_sign=0,
    body_relative=False,
):
    from lib.localisation_motion import record_diagnostic_event

    command = {
        "direction_deg": direction_deg,
        "speed_mm_s": speed_mm_s,
        "turn_sign": turn_sign,
        "body_relative": body_relative,
        "duration_s": duration_s,
    }
    record_diagnostic_event(lidar, "phase", event="start", name=name, **command)
    print(
        f"{name}: {duration_s:.3f}s speed={speed_mm_s:g} mm/s "
        f"direction={direction_deg:+g} deg turn={turn_sign:+d}",
        flush=True,
    )
    deadline = time.monotonic() + duration_s
    last_print = 0.0
    while time.monotonic() < deadline:
        state = session.tick()
        yaw, rate = _safe_state(
            state,
            hardware,
            wall_margin_mm,
            require_pose=speed_mm_s > 0,
        )
        command_direction = yaw + direction_deg if body_relative else direction_deg
        rotation = yaw + turn_sign * 90 if turn_sign else 0.0
        hardware.move(
            command_direction,
            speed_mm_s,
            rotation,
            1.0 if turn_sign else (1.0 if speed_mm_s else 0.0),
            0,
        )
        now = time.monotonic()
        if now - last_print >= 0.2:
            deskew = state.get("deskew", {})
            x_pos, y_pos, mcl_yaw, confidence, pose_ok = state["pose"]
            print(
                f"  pose=({_number(x_pos)},{_number(y_pos)},{_number(mcl_yaw)}) "
                f"ok={pose_ok} conf={_number(confidence, 2)} gyro={rate:+.1f} "
                f"wheel=({state['odometry']['wheel_vx']:+.0f},"
                f"{state['odometry']['wheel_vy']:+.0f}) "
                f"residual={deskew.get('raw_residual_mm', -1):.1f}->"
                f"{deskew.get('corrected_residual_mm', -1):.1f}",
                flush=True,
            )
            _record_live(lidar, hardware, state, name, command)
            last_print = now
        time.sleep(0.02)
    record_diagnostic_event(lidar, "phase", event="end", name=name, **command)


def _stop_phase(lidar, hardware, session, name, duration_s, wall_margin_mm):
    yaw = hardware.get_yaw()
    hardware.move(0, 0, yaw if yaw is not None else 0, 0, 0)
    _run_phase(
        lidar,
        hardware,
        session,
        name=name,
        duration_s=duration_s,
        speed_mm_s=0,
        direction_deg=0,
        wall_margin_mm=wall_margin_mm,
    )


def main(argv=None):
    args = parse_args(argv)
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
    translation_rpm = args.speed * 60 / (math.pi * WHEEL_DIAMETER_MM)
    controller_rpm = max(400, math.ceil(translation_rpm + args.yaw_rpm))
    if controller_rpm > MAX_CONTROLLER_RPM:
        raise ValueError(
            f"Requested command needs {controller_rpm} RPM including yaw headroom; "
            f"controller test cap is {MAX_CONTROLLER_RPM:g} RPM"
        )

    hardware = None
    exit_code = 0
    try:
        config = load_config()
        print(
            f"trajectory={args.trajectory} speed={args.speed:g} mm/s "
            f"duration={args.duration:g}s deskew={args.deskew} "
            f"controller_limit={controller_rpm} RPM yaw_limit={args.yaw_rpm:g} RPM"
        )
        lidar.init(config.lidar_port, 460800)
        hardware = HardwareController.from_i2c_addresses(
            config.i2c_addresses,
            WHEEL_DIAMETER_MM,
            args.yaw_rpm,
            controller_rpm,
            3,
            use_pcb=False,
        )
        startup = capture_startup_yaw(hardware)
        feed_imu_yaw_prior(lidar, hardware, startup)
        lidar.set_motion_noise(args.motion_noise)
        lidar.start_coordinates(PITCH_X_MM, PITCH_Y_MM, use_pcb=False)
        session = LocalisationSession(
            lidar,
            hardware,
            startup,
            LidarVelocityEstimator(),
            use_pcb=False,
        )
        deadline = time.monotonic() + 30
        print("Waiting for initial localisation (30 second timeout)...")
        while True:
            state = session.tick()
            if state["pose"][4] and state["fresh"]:
                break
            if time.monotonic() > deadline:
                raise TimeoutError("Initial localisation unavailable")
            time.sleep(0.02)

        record_diagnostic_event(
            lidar,
            "trial",
            event="start",
            arguments=vars(args),
            startup_yaw_raw_deg=startup,
            controller_limit_rpm=controller_rpm,
        )
        _stop_phase(
            lidar,
            hardware,
            session,
            "initial_stationary",
            args.stationary_s,
            args.wall_margin_mm,
        )

        for repetition in range(1, args.repetitions + 1):
            if args.trajectory == "translation":
                for suffix, direction in (
                    ("positive", args.direction_deg),
                    ("negative", args.direction_deg + 180),
                ):
                    _run_phase(
                        lidar,
                        hardware,
                        session,
                        name=f"rep{repetition}_{suffix}",
                        duration_s=args.duration,
                        speed_mm_s=args.speed,
                        direction_deg=direction,
                        wall_margin_mm=args.wall_margin_mm,
                    )
                    _stop_phase(
                        lidar,
                        hardware,
                        session,
                        f"rep{repetition}_{suffix}_recovery",
                        args.pause_s,
                        args.wall_margin_mm,
                    )
            elif args.trajectory == "reversal":
                _run_phase(
                    lidar,
                    hardware,
                    session,
                    name=f"rep{repetition}_positive",
                    duration_s=args.duration,
                    speed_mm_s=args.speed,
                    direction_deg=args.direction_deg,
                    wall_margin_mm=args.wall_margin_mm,
                )
                _run_phase(
                    lidar,
                    hardware,
                    session,
                    name=f"rep{repetition}_reverse",
                    duration_s=args.duration,
                    speed_mm_s=args.speed,
                    direction_deg=args.direction_deg + 180,
                    wall_margin_mm=args.wall_margin_mm,
                )
                _stop_phase(
                    lidar,
                    hardware,
                    session,
                    f"rep{repetition}_recovery",
                    args.pause_s,
                    args.wall_margin_mm,
                )
            else:
                signs = (
                    (1, -1)
                    if args.turn_direction == "both"
                    else ((1,) if args.turn_direction == "cw" else (-1,))
                )
                for sign in signs:
                    suffix = "cw" if sign > 0 else "ccw"
                    _run_phase(
                        lidar,
                        hardware,
                        session,
                        name=f"rep{repetition}_{suffix}",
                        duration_s=args.duration,
                        speed_mm_s=args.speed,
                        direction_deg=args.direction_deg,
                        wall_margin_mm=args.wall_margin_mm,
                        turn_sign=sign,
                        body_relative=True,
                    )
                    _stop_phase(
                        lidar,
                        hardware,
                        session,
                        f"rep{repetition}_{suffix}_recovery",
                        args.pause_s,
                        args.wall_margin_mm,
                    )
        record_diagnostic_event(lidar, "trial", event="end", status="complete")
    except KeyboardInterrupt:
        print("\nTrajectory test interrupted.")
        exit_code = 130
    except Exception as exc:
        print(f"Trajectory test failed: {exc}")
        exit_code = 1
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
                try:
                    hardware.stop()
                except Exception as exc:
                    print(f"Warning: hardware stop failed: {exc}")
                    exit_code = 1
        finally:
            try:
                close_motion_capture(lidar)
            finally:
                lidar.shutdown()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
