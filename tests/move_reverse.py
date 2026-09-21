import time

from lib.hardware_controller import MotorCommunicationError

from lib.hardware_test_utils import create_hardware, set_startup_yaw

USE_PCB = False

COMMAND_INTERVAL = 0.05


def main():
    hardware = None
    try:
        hardware = create_hardware(max_motor_rpm=400, kicker=True, use_pcb=USE_PCB)
        startup = set_startup_yaw(hardware)
        print(f"Startup yaw reference set to {startup:.6f} deg")
        print("Forward dribbler, reverse dribbler, then kick. Press Ctrl+C to stop.")
        iteration = 0
        while True:
            dribbler = 1 if iteration < 60 else -1
            kick = iteration == 120
            hardware.move(0, 100, 0, 1.0, dribbler=dribbler, kick=kick)
            iteration += 1
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
