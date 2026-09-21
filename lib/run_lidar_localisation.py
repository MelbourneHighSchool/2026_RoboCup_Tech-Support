#!/usr/bin/env python3
"""Run MCL localisation using LIDAR only (no real IMU, no motors).

Starts the scan + localisation threads, keeps calling predict_odometry with
zero motion so particles get process noise and can converge, then prints pose
until Ctrl+C.

Feeds a fixed IMU yaw prior of 0° so MCL can break the 180° field symmetry
without a physical IMU (bot assumed facing startup-forward).

Example:
    python lib/run_lidar_localisation.py
    python lib/run_lidar_localisation.py --port /dev/ttyUSB1 -s
"""

from __future__ import annotations

import argparse
import sys
import time

from lib import lidar

USE_PCB = False

PITCH_X = 2430
PITCH_Y = 1820
DEFAULT_PORT = "/dev/ttyUSB1"
DEFAULT_BAUDRATE = 460800
LOOP_DT_S = 0.05
STATUS_PRINT_INTERVAL_S = 0.5
# Soft MCL yaw prior (startup-relative). Fixed at 0 so the filter prefers the
# facing-forward hypothesis over the opposite 180° pose.
ASSUMED_IMU_YAW_DEG = 0.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run LIDAR-only MCL localisation (no IMU or motors)."
    )
    parser.add_argument(
        "--port",
        default=DEFAULT_PORT,
        help=f"Serial port for the LIDAR (default: {DEFAULT_PORT}).",
    )
    parser.add_argument(
        "--baudrate",
        type=int,
        default=DEFAULT_BAUDRATE,
        help=f"LIDAR baud rate (default: {DEFAULT_BAUDRATE}).",
    )
    parser.add_argument(
        "-s",
        "--stream",
        action="store_true",
        help="Stream poses to the websocket log server for simulate.py --connect.",
    )
    return parser.parse_args()


def format_pose_log_line(x_pos: float, y_pos: float, yaw: float) -> str:
    """CSV line matching simulate.py parse_log_frame (x, y, yaw, ball_x, ball_y)."""
    return f"{x_pos},{y_pos},{yaw},None,None"


def print_status(prefix: str = "") -> None:
    x_pos, y_pos, yaw, confidence, ok = lidar.get_coordinates_info()
    scan_count = lidar.get_scan_count()
    scans_on = lidar.scan_updates_enabled()
    source = "LIDAR" if scans_on else "predict-only"
    if ok and x_pos is not None and y_pos is not None and yaw is not None:
        print(
            f"{prefix}pose=({x_pos:.1f}, {y_pos:.1f}) yaw={yaw:.1f}° "
            f"conf={confidence:.2f} scans={scan_count} source={source}"
        )
    else:
        print(
            f"{prefix}pose=unavailable conf={confidence:.2f} "
            f"scans={scan_count} source={source}"
        )


def stream_pose(stream_enabled: bool, send_log_module, x_pos, y_pos, yaw) -> None:
    if not stream_enabled or send_log_module is None:
        return
    if x_pos is None or y_pos is None or yaw is None:
        return
    send_log_module.update_latest_log(format_pose_log_line(x_pos, y_pos, yaw))


def wait_for_pose() -> None:
    """Block until the first confident pose, feeding zero-motion predict each tick."""
    print("Waiting for first pose estimate...")
    last_predict = time.monotonic()
    last_status = 0.0
    while not lidar.is_coordinates_ready():
        now = time.monotonic()
        lidar.set_imu_yaw(ASSUMED_IMU_YAW_DEG)
        # Zero motion still applies process noise so particles can explore after
        # resampling; without this the filter often never reaches confidence.
        lidar.predict_odometry(0.0, 0.0, 0.0, now - last_predict)
        last_predict = now
        if now - last_status >= STATUS_PRINT_INTERVAL_S:
            print_status("  ")
            last_status = now
        time.sleep(LOOP_DT_S)
    print("Pose ready.\n")


def main() -> int:
    args = parse_args()
    send_log_module = None
    if args.stream:
        import send_log

        send_log_module = send_log
        send_log.start_server_background()
        time.sleep(0.05)
        print(f"Websocket log server running on ws://0.0.0.0:{send_log.PORT}")
        print(f"Connect with: python simulate.py --connect 127.0.0.1:{send_log.PORT}")

    print("LIDAR-only localisation")
    print("=" * 40)
    print(f"Port: {args.port}, baud: {args.baudrate}")
    print(f"Pitch: {PITCH_X} x {PITCH_Y} mm")
    print(f"Assumed IMU yaw prior: {ASSUMED_IMU_YAW_DEG:.1f}° (no physical IMU)")
    print()

    try:
        lidar.init(args.port, args.baudrate)
    except Exception as exc:
        print(f"Failed to initialize LIDAR: {exc}", file=sys.stderr)
        return 1

    print("Waiting for first scan...")
    while not lidar.is_scan_ready():
        time.sleep(0.1)
    print("Scan ready.")

    # Seed the yaw prior before MCL starts so init particles sample around 0°.
    lidar.set_imu_yaw(ASSUMED_IMU_YAW_DEG)
    lidar.start_coordinates(PITCH_X, PITCH_Y, use_pcb=USE_PCB)
    wait_for_pose()
    print("Printing pose (Ctrl+C to stop):\n")

    last_predict = time.monotonic()
    try:
        while True:
            now = time.monotonic()
            lidar.set_imu_yaw(ASSUMED_IMU_YAW_DEG)
            lidar.predict_odometry(0.0, 0.0, 0.0, now - last_predict)
            last_predict = now

            x_pos, y_pos, yaw, confidence = lidar.get_pose()
            if x_pos is not None and y_pos is not None and yaw is not None:
                print(
                    f"x={x_pos:.1f} mm  y={y_pos:.1f} mm  "
                    f"yaw={yaw:.1f}°  conf={confidence:.2f}"
                )
                stream_pose(args.stream, send_log_module, x_pos, y_pos, yaw)
            else:
                print_status()
            time.sleep(LOOP_DT_S)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        print("Shutting down LIDAR...")
        lidar.shutdown()
        print("Done.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
