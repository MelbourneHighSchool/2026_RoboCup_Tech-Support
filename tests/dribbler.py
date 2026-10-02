"""Spin only the dribbler: python -m tests.dribbler."""

import time

from lib.hardware_controller import HardwareController

from lib.config import load_config
from lib.hardware_test_utils import WHEEL_DIAMETER, YAW_CORRECT_THRESHOLD

USE_PCB = False

DRIBBLER_SPEED_MODE = True
DRIBBLER_TORQUE = 4.0  # Amps
DRIBBLER_SPEED = 1000  # RPM; current is limited by DRIBBLER_TORQUE
RPM_PRINT_INTERVAL = 0.25


def main():
    config = load_config()
    if len(config.i2c_addresses) != 5:
        raise ValueError("Configure four drive motors and a fifth dribbler motor for this test")

    hardware = None
    try:
        hardware = HardwareController.from_i2c_addresses(
            config.i2c_addresses,
            WHEEL_DIAMETER,
            0,
            0,
            YAW_CORRECT_THRESHOLD,
            drive_motor_current_limit=0.0,
            dribbler_motor_current_limit=DRIBBLER_TORQUE,
            dribbler_speed_mode=DRIBBLER_SPEED_MODE,
            dribbler_speed_rpm=DRIBBLER_SPEED,
            use_pcb=USE_PCB)
        setting = (f"{DRIBBLER_SPEED:g} RPM (limit {DRIBBLER_TORQUE:g} A)"
                   if DRIBBLER_SPEED_MODE else f"{DRIBBLER_TORQUE:g} A")
        print(f"Spinning dribbler at {setting}. Press Ctrl+C to stop.")
        next_rpm_print = time.monotonic()
        while True:
            # No translation or yaw correction. Repeated calls also surface native faults.
            hardware.move(0, 0, 0, 0, dribbler=1)
            now = time.monotonic()
            if now >= next_rpm_print:
                rpm, error1, error2 = hardware.get_dribbler_qdr()
                print(f"Dribbler speed: {rpm:.0f} RPM")
                if error1 or error2:
                    print(f"Dribbler QDR errors: ERROR1=0x{error1:02X}, "
                          f"ERROR2=0x{error2:02X}", flush=True)
                next_rpm_print = now + RPM_PRINT_INTERVAL
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\nStopping dribbler.")
    finally:
        if hardware is not None:
            hardware.stop()


if __name__ == "__main__":
    main()
