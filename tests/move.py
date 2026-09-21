import math
import time

from lib.hardware_controller import MotorCommunicationError

from lib.hardware_test_utils import create_hardware, set_startup_yaw

USE_PCB = False

COMMAND_INTERVAL = 0.05


def main():
    hardware = None
    try:
        hardware = create_hardware(max_motor_rpm=400, use_pcb=USE_PCB)
        startup = set_startup_yaw(hardware)
        print(f"Startup yaw reference set to {startup:.6f} deg")
        print("Running native move test. Press Ctrl+C to stop.")
        while True:
            yaw = hardware.get_yaw()
            if yaw is None:
                time.sleep(0.01)
                continue
            vx, vy = hardware.get_measured_body_velocity_mm_s(yaw)
            print(
                f"Yaw: {yaw:.6f} deg relative | measured {math.hypot(vx, vy):.1f} mm/s "
                f"(vx={vx:.1f} forward, vy={vy:.1f} left)"
            )
            hardware.move(0, 100, 0, 1.0, dribbler=1)
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
