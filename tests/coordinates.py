"""Print the result of get_pose for testing. Requires LIDAR to be connected."""
import sys
import time

USE_PCB = False

LIDAR_BAUDRATE = 460800


def main():
    from lib.config import load_config

    lidar_port = load_config().lidar_port
    from lib import lidar

    print(f"Initializing LIDAR on {lidar_port} at {LIDAR_BAUDRATE} baud...")
    try:
        lidar.init(lidar_port, LIDAR_BAUDRATE)
    except Exception as e:
        print(f"Failed to initialize LIDAR: {e}", file=sys.stderr)
        sys.exit(1)

    print("Waiting for first scan...")
    while not lidar.is_scan_ready():
        time.sleep(0.1)

    lidar.start_coordinates(2430, 1820, use_pcb=USE_PCB)

    print("Waiting for pose estimate...")
    while not lidar.is_coordinates_ready():
        time.sleep(0.1)

    x_pos, y_pos, yaw, confidence = lidar.get_pose()
    print(f"x_pos={x_pos}")
    print(f"y_pos={y_pos}")
    print(f"yaw={yaw}")
    print(f"confidence={confidence}")

    lidar.shutdown()


if __name__ == "__main__":
    main()
