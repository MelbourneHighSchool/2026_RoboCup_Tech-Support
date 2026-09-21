import time

from lib.hardware_controller import MotorCommunicationError

from lib.hardware_test_utils import create_hardware, set_startup_yaw

USE_PCB = False

COMMAND_INTERVAL = 0.05
# Keep the target this far clockwise of the current yaw so it never settles on a
# fixed heading.
CLOCKWISE_YAW_ERROR = 90.0


def main():
    hardware = None
    try:
        hardware = create_hardware(max_motor_rpm=400, use_pcb=USE_PCB)
        set_startup_yaw(hardware)
        print("Dribbler on; rotating clockwise. Press Ctrl+C to stop.")
        while True:
            yaw = hardware.get_yaw()
            if yaw is not None:
                hardware.move(
                    0,
                    0,
                    yaw + CLOCKWISE_YAW_ERROR,
                    1.0,
                    1,
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
