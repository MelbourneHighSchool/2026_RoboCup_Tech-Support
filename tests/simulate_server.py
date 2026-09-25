"""Stream real measured world state for ``simulate.py --connect``."""

import math
import select
import sys
import time

from lib import lidar, send_log
from lib.camera import Camera
from lib.hardware_test_utils import create_hardware, set_startup_yaw
from lib.line_sensors import LineSensorFeed
from main import BALL_TIMEOUT, CAMERA_PORT, LIDAR_BAUDRATE

USE_PCB = False

PORT = send_log.PORT
PITCH_WIDTH = 2430
PITCH_HEIGHT = 1820


def _enter_pressed():
    if not sys.stdin.isatty():
        return False
    ready, _, _ = select.select([sys.stdin], [], [], 0)
    if not ready:
        return False
    sys.stdin.readline()
    return True


def format_log_line(x_pos, y_pos, yaw_relative, ball_x, ball_y, other_bot_positions):
    values = [x_pos, y_pos, yaw_relative, ball_x, ball_y]
    for other_x, other_y in other_bot_positions:
        values.extend((other_x, other_y))
    return ",".join("None" if value is None else str(value) for value in values)


def main():
    from lib.config import load_config

    lidar_port = load_config().lidar_port
    camera = None
    hardware = None

    send_log.start_server_background()
    time.sleep(0.05)
    print(f"Websocket log server running on ws://0.0.0.0:{PORT}")
    print(f"Connect with: python simulate.py --connect 127.0.0.1:{PORT}")

    try:
        print(f"Initializing LIDAR on {lidar_port} at {LIDAR_BAUDRATE} baud...")
        try:
            lidar.init(lidar_port, LIDAR_BAUDRATE)
        except Exception as exc:
            raise RuntimeError(f"Failed to initialize LIDAR: {exc}") from exc

        print("Waiting for first scan data...")
        while not lidar.is_scan_ready():
            if _enter_pressed():
                print("Shutdown requested, exiting.")
                return
            time.sleep(0.1)

        camera = Camera(CAMERA_PORT, resolution=(2000, 2000), frame_rate=90)
        camera.start()
        hardware = create_hardware(use_pcb=USE_PCB)

        startup_yaw = set_startup_yaw(hardware)
        print(f"Startup yaw reference set to {startup_yaw:.6f} deg")
        yaw_relative = hardware.get_yaw()
        if yaw_relative is not None:
            lidar.set_imu_yaw(yaw_relative)

        lidar.start_coordinates(PITCH_WIDTH, PITCH_HEIGHT, use_pcb=USE_PCB)
        line_sensor_feed = LineSensorFeed(use_pcb=USE_PCB)

        print("Waiting for first coordinate estimate...")
        last_pose_time = time.monotonic()
        while not lidar.is_coordinates_ready():
            if _enter_pressed():
                print("Shutdown requested, exiting.")
                return
            yaw_relative = hardware.get_yaw()
            if yaw_relative is not None:
                lidar.set_imu_yaw(yaw_relative)
            now = time.monotonic()
            vx, vy = hardware.get_measured_body_velocity_mm_s(yaw_relative or 0.0)
            lidar.predict_odometry(vx, vy, hardware.get_gyro_z_deg_s() or 0.0, now - last_pose_time)
            last_pose_time = now
            line_sensor_feed.update(lidar, hardware)
            time.sleep(0.1)

        print("Streaming measured positions. Press Enter to shut down.")

        ball_dx = 0.0
        ball_dy = 0.0
        last_ball_update = time.time()
        last_ball_x = None
        last_ball_y = None
        last_camera_frame_id = camera.frame_id

        while True:
            if _enter_pressed():
                print("Shutdown requested, exiting.")
                break

            yaw_relative = hardware.get_yaw()
            if yaw_relative is None:
                time.sleep(0.01)
                continue

            lidar.set_imu_yaw(yaw_relative)
            gyro_z = hardware.get_gyro_z_deg_s()
            now = time.monotonic()
            vx, vy = hardware.get_measured_body_velocity_mm_s(yaw_relative)
            lidar.predict_odometry(vx, vy, gyro_z or 0.0, now - last_pose_time)
            last_pose_time = now
            line_sensor_feed.update(lidar, hardware)

            x_pos, y_pos, mcl_yaw, _confidence = lidar.get_pose()
            if mcl_yaw is not None:
                yaw_relative = mcl_yaw
            elif x_pos is None or y_pos is None:
                time.sleep(0.01)
                continue

            camera_frame_id, ball_direction, ball_distance = camera.get_measurement()
            has_new_camera_frame = camera_frame_id != last_camera_frame_id
            last_camera_frame_id = camera_frame_id

            if has_new_camera_frame:
                if (
                    ball_distance is not None
                    and ball_direction is not None
                    and x_pos is not None
                    and y_pos is not None
                ):
                    ball_global_direction = yaw_relative + ball_direction
                    ball_x = x_pos + ball_distance * math.cos(
                        math.radians(ball_global_direction)
                    )
                    ball_y = y_pos + ball_distance * math.sin(
                        math.radians(ball_global_direction)
                    )
                else:
                    ball_x = None
                    ball_y = None
            else:
                ball_x = None
                ball_y = None

            now = time.time()
            if ball_x is not None and ball_y is not None:
                dt = now - last_ball_update
                if last_ball_x is not None and last_ball_y is not None and dt > 0:
                    ball_dx = (ball_x - last_ball_x) / dt
                    ball_dy = (ball_y - last_ball_y) / dt
                last_ball_x = ball_x
                last_ball_y = ball_y
                last_ball_update = now
            elif (
                last_ball_x is not None
                and last_ball_y is not None
                and now - last_ball_update < BALL_TIMEOUT
            ):
                dt_lost = now - last_ball_update
                ball_x = last_ball_x + ball_dx * dt_lost
                ball_y = last_ball_y + ball_dy * dt_lost
            else:
                ball_x = None
                ball_y = None

            send_log.update_latest_log(
                format_log_line(
                    x_pos,
                    y_pos,
                    yaw_relative,
                    ball_x,
                    ball_y,
                    [],
                )
            )
            time.sleep(0.01)
    finally:
        if camera is not None:
            try:
                camera.stop()
            except Exception as exc:
                print(f"Warning: failed to stop camera cleanly: {exc}")
        if hardware is not None:
            try:
                hardware.stop()
            except Exception as exc:
                print(f"Warning: failed to stop hardware cleanly: {exc}")
        try:
            lidar.shutdown()
        except Exception as exc:
            print(f"Warning: failed to shut down lidar cleanly: {exc}")


if __name__ == "__main__":
    main()
