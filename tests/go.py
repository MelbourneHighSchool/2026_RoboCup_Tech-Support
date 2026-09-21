import math
import time

from lib.hardware_controller import MotorCommunicationError

from lib import lidar
from lib.hardware_test_utils import create_hardware, set_startup_yaw

USE_PCB = False

MAX_MOTOR_RPM = 400

TEST_DIRECTION = 0
TEST_SPEED = 100
TEST_ROTATION = 0
TEST_ROTATION_SPEED = 1.0
COMMAND_INTERVAL = 0.05

MOVE_POINT = (1000, 500)


def main():
    hardware = None
    try:
        hardware = create_hardware(max_motor_rpm=MAX_MOTOR_RPM, use_pcb=USE_PCB)
        startup_yaw = set_startup_yaw(hardware)
        print(f"Startup yaw reference set to {startup_yaw:.6f} deg")
        print(
            "Running move() test with "
            f"direction={TEST_DIRECTION}, speed={TEST_SPEED}, rotation={TEST_ROTATION}, using live IMU yaw"
        )
        print("Press Ctrl+C to stop.")

        while True:
            x_pos, y_pos, _mcl_yaw, _confidence = lidar.get_pose()
            yaw = hardware.get_yaw()
            if yaw is None:
                time.sleep(0.01)
                continue
            vx, vy = hardware.get_measured_body_velocity_mm_s(yaw)
            measured_speed = math.hypot(vx, vy)
            print(
                f"Yaw: {yaw:.6f} deg relative | "
                f"measured {measured_speed:.1f} mm/s "
                f"(vx={vx:.1f} forward, vy={vy:.1f} left)"
            )
            # set direction and point here

            vector = (MOVE_POINT[0] - x_pos), (MOVE_POINT[1] - y_pos)
            direction = math.degrees(math.atan2(vector[1], vector[0]))
            hardware.move(
                direction,
                TEST_SPEED,
                TEST_ROTATION,
                TEST_ROTATION_SPEED,
                1
            )
            time.sleep(COMMAND_INTERVAL)
    except KeyboardInterrupt:
        print("\nStopping test.")
    except MotorCommunicationError as exc:
        print(exc)
        raise
    finally:
        if hardware is not None:
            hardware.stop()


if __name__ == "__main__":
    main()
