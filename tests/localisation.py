#!/usr/bin/env python3
"""Drive to user-specified field coordinates using LIDAR MCL pose feedback.

Interactive test: enter a target (x, y) in mm and the robot drives there while
printing its localized position. Requires LIDAR, IMU, and motors.

With ``--no-move``, the native controller keeps the motors at zero while the
script only prints / streams the localized pose.

With ``-s`` / ``--stream``, poses are pushed to the websocket log server for live
viewing with ``python simulate.py --connect 127.0.0.1:8765``.
"""

import argparse
import math
import queue
import threading
import time

from lib.line_sensors import LineSensorFeed
from lib.localisation_service import (
    LidarVelocityEstimator,
    capture_startup_yaw,
    feed_imu_yaw_prior,
    get_position,
    get_yaw,
    predict_odometry,
)

USE_PCB = False
LINE_SENSOR_FEED = LineSensorFeed(use_pcb=USE_PCB)

TARGET_TOLERANCE_MM = 10
MAX_SPEED_MM_S = 500
SLOW_RADIUS_MM = 300
LOOP_DELAY_SECONDS = 0.02
STATUS_PRINT_INTERVAL_S = 0.2

MAX_MOTOR_RPM = 400

LIDAR_BAUDRATE = 460800
PITCH_X = 2430
PITCH_Y = 1820




def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Drive to user-specified field coordinates using LIDAR MCL pose feedback."
        )
    )
    parser.add_argument(
        "-s",
        "--stream",
        action="store_true",
        help="Enable websocket live pose streaming for simulate.py --connect.",
    )
    parser.add_argument(
        "--no-move",
        action="store_true",
        help="Disable motors; only print/stream localized pose.",
    )
    odometry_mode = parser.add_mutually_exclusive_group()
    odometry_mode.add_argument(
        "--raw-odometry", dest="raw_odometry", action="store_true", default=True,
        help="Feed measured wheel velocity without trust scaling (default).",
    )
    odometry_mode.add_argument(
        "--scaled-odometry", dest="raw_odometry", action="store_false",
        help="Restore legacy trust scaling for comparison tests.",
    )
    parser.add_argument(
        "--max-speed", type=float, default=MAX_SPEED_MM_S,
        help="Maximum translation speed in mm/s (default: 500).",
    )
    parser.add_argument(
        "--motion-noise", type=float, default=0.30,
        help="Speed-dependent motion noise coefficient in sqrt(s); 0 isolates the timestamp fix.",
    )
    args = parser.parse_args()
    if not math.isfinite(args.motion_noise) or args.motion_noise < 0:
        parser.error("--motion-noise must be finite and nonnegative")
    if not math.isfinite(args.max_speed) or not 0 < args.max_speed <= MAX_SPEED_MM_S:
        parser.error("--max-speed must be greater than zero and at most 500 mm/s")
    return args


def format_pose_log_line(x_pos, y_pos, yaw):
    """CSV line matching simulate.py parse_log_frame (x, y, yaw, ball_x, ball_y)."""
    values = [x_pos, y_pos, yaw, None, None]
    return ",".join("None" if value is None else str(value) for value in values)


def stream_pose(stream_enabled, send_log_module, x_pos, y_pos, yaw):
    if not stream_enabled or send_log_module is None:
        return
    if x_pos is None or y_pos is None or yaw is None:
        return
    send_log_module.update_latest_log(format_pose_log_line(x_pos, y_pos, yaw))








def print_localisation_status(lidar_module, odometry=None):
    """Print a readable snapshot of pose, odometry, and filter health."""
    x_pos, y_pos, yaw, confidence, ok = lidar_module.get_coordinates_info()
    scans_enabled = lidar_module.scan_updates_enabled()
    scan_generation = lidar_module.get_scan_generation()
    mcl_updates = lidar_module.get_mcl_update_count()
    scan_quality, baseline, bad_scans, global_fraction, recovery_valid = (
        lidar_module.get_recovery_status()
    )
    confidence_s = "n/a" if confidence is None else f"{confidence:.2f}"

    if ok and x_pos is not None and y_pos is not None and yaw is not None:
        print(
            f"[POSE] x={x_pos:7.1f}  y={y_pos:7.1f} mm  yaw={yaw:6.1f} deg  "
            f"confidence={confidence_s}"
        )
    else:
        print(f"[POSE] unavailable  confidence={confidence_s}")

    if odometry is None or not odometry.has_wheel_odometry:
        odometry_text = "wheel=unavailable"
        if odometry is not None:
            odometry_text += (
                f"  dt={odometry.dt_s * 1000.0:5.1f} ms"
                f"  gyro={odometry.omega_deg_s:+6.1f} deg/s"
            )
        print(f"[ODOM] {odometry_text}")
    else:
        wheel_speed = math.hypot(odometry.wheel_vx, odometry.wheel_vy)
        lidar_speed = math.hypot(odometry.lidar_vx, odometry.lidar_vy)
        freshness = "fresh" if odometry.lidar_fresh else "stale"
        flags = []
        if odometry.trust <= 0.01:
            flags.append("TRUST ZERO")
        if odometry.dt_s > 0.1:
            flags.append("LARGE DT")
        flag_text = f"  !! {', '.join(flags)}" if flags else ""
        print(
            f"[ODOM] dt={odometry.dt_s * 1000.0:5.1f} ms  "
            f"gyro={odometry.omega_deg_s:+6.1f} deg/s  "
            f"yaw={odometry.yaw_deg:6.1f} deg  trust={odometry.trust:4.0%}"
            f"{flag_text}"
        )
        print(
            f"       wheel=({odometry.wheel_vx:+7.1f}, {odometry.wheel_vy:+7.1f}) "
            f"|v|={wheel_speed:6.1f}  "
            f"lidar=({odometry.lidar_vx:+7.1f}, {odometry.lidar_vy:+7.1f}) "
            f"|v|={lidar_speed:6.1f} [{freshness}]  "
            f"fed=({odometry.fed_vx:+7.1f}, {odometry.fed_vy:+7.1f}) mm/s"
        )

    scan_mode = "enabled" if scans_enabled else "PAUSED (predict only)"
    if recovery_valid:
        recovery_text = (
            f"quality={scan_quality:6.2f}/{baseline:6.2f}  "
            f"bad={bad_scans}  global={global_fraction:4.0%}"
        )
    else:
        recovery_text = "quality=not ready"
    print(
        f"[MCL ] scans={scan_generation}  corrections={mcl_updates}  "
        f"scan updates={scan_mode}  {recovery_text}"
    )


def print_scan_correction_if_new(lidar_module, last_sequence):
    """If a new LIDAR scan update landed, print odometry vs LIDAR correction error.

    When confidence drops below the tracking threshold, MCL stops recording a
    correction, so also print a one-line collapse notice once per scan.

    Returns the latest correction sequence number.
    """
    (
        sequence,
        pred_x,
        pred_y,
        pred_yaw,
        corr_x,
        corr_y,
        corr_yaw,
        error_mm,
        yaw_error_deg,
        valid,
    ) = lidar_module.get_last_scan_correction()
    _x, _y, _yaw, confidence, ok = lidar_module.get_coordinates_info()
    generation = lidar_module.get_scan_generation()
    last_collapse_generation = getattr(
        print_scan_correction_if_new, "_collapse_generation", 0
    )

    if not ok and generation != last_collapse_generation:
        recovery = lidar_module.get_recovery_status()
        scan_quality, baseline, bad_scans, global_fraction, recovery_valid = recovery
        confidence_s = "n/a" if confidence is None else f"{confidence:.2f}"
        recovery_text = (
            f"quality={scan_quality:.2f}/{baseline:.2f}  "
            f"bad={bad_scans}  global={global_fraction:.0%}"
            if recovery_valid
            else "quality=not ready"
        )
        print(
            f"[SCAN {generation:06d}] NO FIX  confidence={confidence_s}  "
            f"{recovery_text}"
        )
        print_scan_correction_if_new._collapse_generation = generation

    if not valid or sequence == last_sequence:
        return last_sequence

    recovery = lidar_module.get_recovery_status()
    scan_quality, baseline, bad_scans, global_fraction, _valid = recovery
    confidence_s = "n/a" if confidence is None else f"{confidence:.2f}"
    print(
        f"[SCAN {sequence:06d}] correction={error_mm:6.1f} mm  "
        f"yaw correction={yaw_error_deg:+5.1f} deg  confidence={confidence_s}"
    )
    print(
        f"              predicted=({pred_x:7.1f}, {pred_y:7.1f}, {pred_yaw:6.1f})"
        f" -> corrected=({corr_x:7.1f}, {corr_y:7.1f}, {corr_yaw:6.1f})  "
        f"quality={scan_quality:.2f}/{baseline:.2f}  "
        f"bad={bad_scans}  global={global_fraction:.0%}"
    )
    return sequence






def drive_to_target(
    target_x,
    target_y,
    lidar_module,
    movement_controller,
    imu,
    startup_yaw,
    lidar_velocity,
    last_pose_time,
    last_mcl_yaw,
    last_scan_sequence=0,
    stream_enabled=False,
    send_log_module=None,
    apply_trust=False,
    max_speed=MAX_SPEED_MM_S,
):
    """Drive toward (target_x, target_y) using localized pose feedback."""
    last_status_print = 0.0
    while True:
        now = time.monotonic()
        yaw_for_odom = last_mcl_yaw if last_mcl_yaw is not None else 0.0
        last_pose_time, odometry = predict_odometry(
            lidar_module,
            movement_controller,
            imu,
            startup_yaw,
            lidar_velocity,
            yaw_for_odom,
            last_pose_time,
            apply_trust=apply_trust,
            line_feed=LINE_SENSOR_FEED,
        )
        last_scan_sequence = print_scan_correction_if_new(
            lidar_module, last_scan_sequence
        )

        if now - last_status_print >= STATUS_PRINT_INTERVAL_S:
            print_localisation_status(lidar_module, odometry)
            last_status_print = now

        pose = get_position(lidar_module)
        if pose is None:
            # Pause safely without terminating the controller's drive thread.
            movement_controller.move(0.0, 0.0, yaw_for_odom, 0.0)
            time.sleep(LOOP_DELAY_SECONDS)
            continue

        current_x, current_y, mcl_yaw = pose
        yaw = get_yaw(imu, startup_yaw, mcl_yaw)
        if yaw is None:
            # Pause safely without terminating the controller's drive thread.
            movement_controller.move(0.0, 0.0, yaw_for_odom, 0.0)
            time.sleep(LOOP_DELAY_SECONDS)
            continue

        last_mcl_yaw = mcl_yaw
        lidar_velocity.update(current_x, current_y, yaw, now)
        stream_pose(stream_enabled, send_log_module, current_x, current_y, yaw)

        dx = target_x - current_x
        dy = target_y - current_y
        distance = math.hypot(dx, dy)
        if distance <= TARGET_TOLERANCE_MM:
            break

        direction = math.degrees(math.atan2(dy, dx))
        speed = min(max_speed, distance / SLOW_RADIUS_MM * max_speed)
        movement_controller.move(direction, speed, yaw, 1.0)
        time.sleep(LOOP_DELAY_SECONDS)

    movement_controller.move(0.0, 0.0, yaw, 0.0)
    return last_pose_time, last_mcl_yaw, last_scan_sequence


def _read_target_coordinates(result_queue):
    """Blocking stdin prompt; runs on a worker thread so MCL can keep updating."""
    try:
        target_x = float(input("What x position to move to? "))
        target_y = float(input("What y position to move to? "))
        if not (math.isfinite(target_x) and math.isfinite(target_y)):
            raise ValueError("Target coordinates must be finite")
        if not (0 <= target_x <= PITCH_X and 0 <= target_y <= PITCH_Y):
            raise ValueError("Target coordinates must be inside the pitch")
        result_queue.put(("ok", target_x, target_y))
    except ValueError as exc:
        result_queue.put(("error", exc))
    except EOFError as exc:
        result_queue.put(("error", exc))


def wait_for_target_while_localising(
    lidar_module,
    imu,
    startup_yaw,
    lidar_velocity,
    last_pose_time,
    last_mcl_yaw,
    last_scan_sequence=0,
    stream_enabled=False,
    send_log_module=None,
    movement_controller=None,
    apply_trust=False,
):
    """Keep feeding MCL until the user enters a target (x, y).

    ``input()`` blocks the calling thread, which would otherwise pause IMU/odom
    updates and stall particle-filter convergence. Prompt on a side thread and
    keep running the usual localisation loop here until a target arrives.
    """
    result_queue = queue.Queue()
    prompt_thread = threading.Thread(
        target=_read_target_coordinates,
        args=(result_queue,),
        daemon=True,
    )
    prompt_thread.start()

    # last_status_print = 0.0
    while True:
        try:
            status, *payload = result_queue.get_nowait()
        except queue.Empty:
            status = None

        if status == "ok":
            target_x, target_y = payload
            return (
                target_x,
                target_y,
                last_pose_time,
                last_mcl_yaw,
                last_scan_sequence,
            )
        if status == "error":
            raise payload[0]

        now = time.monotonic()
        yaw_for_odom = last_mcl_yaw if last_mcl_yaw is not None else 0.0
        last_pose_time, _odometry = predict_odometry(
            lidar_module,
            movement_controller,
            imu,
            startup_yaw,
            lidar_velocity,
            yaw_for_odom,
            last_pose_time,
            apply_trust=apply_trust,
            line_feed=LINE_SENSOR_FEED,
        )
        last_scan_sequence = print_scan_correction_if_new(
            lidar_module, last_scan_sequence
        )

        # if now - last_status_print >= STATUS_PRINT_INTERVAL_S:
        #     print_localisation_status(lidar_module)
        #     last_status_print = now

        pose = get_position(lidar_module)
        if pose is not None:
            current_x, current_y, mcl_yaw = pose
            yaw = get_yaw(imu, startup_yaw, mcl_yaw)
            if yaw is not None:
                last_mcl_yaw = mcl_yaw
                lidar_velocity.update(current_x, current_y, yaw, now)
                stream_pose(
                    stream_enabled, send_log_module, current_x, current_y, yaw
                )

        time.sleep(LOOP_DELAY_SECONDS)


def monitor_pose(
    lidar_module,
    imu,
    startup_yaw,
    lidar_velocity,
    last_pose_time,
    last_mcl_yaw,
    last_scan_sequence=0,
    stream_enabled=False,
    send_log_module=None,
):
    """Print and optionally stream localized pose without commanding motors."""
    last_status_print = 0.0
    while True:
        now = time.monotonic()
        yaw_for_odom = last_mcl_yaw if last_mcl_yaw is not None else 0.0
        last_pose_time, odometry = predict_odometry(
            lidar_module,
            None,
            imu,
            startup_yaw,
            lidar_velocity,
            yaw_for_odom,
            last_pose_time,
            line_feed=LINE_SENSOR_FEED,
        )
        last_scan_sequence = print_scan_correction_if_new(
            lidar_module, last_scan_sequence
        )

        if now - last_status_print >= STATUS_PRINT_INTERVAL_S:
            print_localisation_status(lidar_module, odometry)
            last_status_print = now

        pose = get_position(lidar_module)
        if pose is None:
            time.sleep(LOOP_DELAY_SECONDS)
            continue

        current_x, current_y, mcl_yaw = pose
        yaw = get_yaw(imu, startup_yaw, mcl_yaw)
        if yaw is None:
            time.sleep(LOOP_DELAY_SECONDS)
            continue

        last_mcl_yaw = mcl_yaw
        lidar_velocity.update(current_x, current_y, yaw, now)
        stream_pose(stream_enabled, send_log_module, current_x, current_y, yaw)
        time.sleep(LOOP_DELAY_SECONDS)


def main():
    from lib.config import load_config

    lidar_port = load_config().lidar_port
    args = parse_args()
    from lib.hardware_controller import MotorCommunicationError

    from lib import lidar
    from lib.hardware_test_utils import create_hardware

    mode = "RAW (trust diagnostic only)" if args.raw_odometry else "TRUST-SCALED"
    print(f"Odometry mode: {mode}; maximum speed: {args.max_speed:g} mm/s")
    send_log_module = None
    if args.stream:
        from lib import send_log

        send_log_module = send_log
        send_log.start_server_background()
        time.sleep(0.05)
        print(f"Websocket log server running on ws://0.0.0.0:{send_log.PORT}")
        print(f"Connect with: python simulate.py --connect 127.0.0.1:{send_log.PORT}")

    imu = None
    movement_controller = None
    try:
        print(f"Initializing LIDAR on {lidar_port} at {LIDAR_BAUDRATE} baud...")
        lidar.init(lidar_port, LIDAR_BAUDRATE)

        print("Waiting for first scan...")
        while not lidar.is_scan_ready():
            time.sleep(0.1)

        print("Initializing native motors and IMU...")
        imu = create_hardware(max_motor_rpm=MAX_MOTOR_RPM, use_pcb=USE_PCB)
        movement_controller = imu
        startup_yaw = capture_startup_yaw(imu)
        print(f"Startup yaw reference set to {startup_yaw:.1f} deg")
        feed_imu_yaw_prior(lidar, imu, startup_yaw)

        lidar.set_motion_noise(args.motion_noise)
        print(f"Motion noise coefficient: {args.motion_noise:g} sqrt(s)")
        lidar.start_coordinates(PITCH_X, PITCH_Y, use_pcb=USE_PCB)

        print("Waiting for first pose estimate...")
        last_pose_time = time.monotonic()
        last_status_print = 0.0
        while not lidar.is_coordinates_ready():
            now = time.monotonic()
            omega = 0.0
            gyro_z = imu.get_gyro_z_deg_s()
            if gyro_z is not None:
                omega = gyro_z
            feed_imu_yaw_prior(lidar, imu, startup_yaw)
            vx, vy = imu.get_measured_body_velocity_mm_s(imu.get_yaw() or 0.0)
            lidar.predict_odometry(vx, vy, omega, now - last_pose_time)
            LINE_SENSOR_FEED.update(lidar, imu)
            last_pose_time = now
            if now - last_status_print >= 0.5:
                print_localisation_status(lidar)
                print(f"[LIDAR] points in latest scan={lidar.get_scan_count()}")
                last_status_print = now
            time.sleep(0.1)

        lidar_velocity = LidarVelocityEstimator()
        last_mcl_yaw = None
        last_scan_sequence = 0

        if args.no_move:
            print("Movement disabled (--no-move). Monitoring pose. Press Ctrl+C to quit.")
            monitor_pose(
                lidar,
                imu,
                startup_yaw,
                lidar_velocity,
                last_pose_time,
                last_mcl_yaw,
                last_scan_sequence,
                stream_enabled=args.stream,
                send_log_module=send_log_module,
            )
        else:
            print(
                "Enter target coordinates in mm. Localisation keeps running "
                "while you type. Press Ctrl+C to quit."
            )
            while True:
                (
                    target_x,
                    target_y,
                    last_pose_time,
                    last_mcl_yaw,
                    last_scan_sequence,
                ) = wait_for_target_while_localising(
                    lidar,
                    imu,
                    startup_yaw,
                    lidar_velocity,
                    last_pose_time,
                    last_mcl_yaw,
                    last_scan_sequence,
                    stream_enabled=args.stream,
                    send_log_module=send_log_module,
                    movement_controller=movement_controller,
                    apply_trust=not args.raw_odometry,
                )

                print_localisation_status(lidar)
                pose = get_position(lidar)
                if pose is not None:
                    stream_pose(args.stream, send_log_module, pose[0], pose[1], pose[2])

                last_pose_time, last_mcl_yaw, last_scan_sequence = drive_to_target(
                    target_x,
                    target_y,
                    lidar,
                    movement_controller,
                    imu,
                    startup_yaw,
                    lidar_velocity,
                    last_pose_time,
                    last_mcl_yaw,
                    last_scan_sequence,
                    stream_enabled=args.stream,
                    send_log_module=send_log_module,
                    apply_trust=not args.raw_odometry,
                    max_speed=args.max_speed,
                )
                print(f"Reached target ({target_x:.1f}, {target_y:.1f})")
    except KeyboardInterrupt:
        print("\nStopping test.")
    except MotorCommunicationError as exc:
        print(exc)
        raise
    finally:
        if imu is not None:
            imu.stop()
        try:
            lidar.shutdown()
        except Exception as exc:
            print(f"Warning: failed to shut down lidar cleanly: {exc}")


if __name__ == "__main__":
    main()
