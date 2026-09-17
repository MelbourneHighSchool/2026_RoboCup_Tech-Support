"""Spin only the dribbler: python -m tests.dribbler."""

import time

from lib.hardware_controller import HardwareController

from lib.config import load_config
from lib.hardware_test_utils import WHEEL_DIAMETER, YAW_CORRECT_THRESHOLD

DRIBBLER_TORQUE = 1.0  # Amps
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
        )
        print(f"Spinning dribbler at {DRIBBLER_TORQUE:g} A. Press Ctrl+C to stop.")
        next_rpm_print = time.monotonic()
        while True:
            # No translation or yaw correction. Repeated calls also surface native faults.
            hardware.move(0, 0, 0, 0, dribbler=1)
            now = time.monotonic()
            if now >= next_rpm_print:
                print(f"Dribbler speed: {hardware.get_dribbler_rpm():.0f} RPM")
                next_rpm_print = now + RPM_PRINT_INTERVAL
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\nStopping dribbler.")
    finally:
        if hardware is not None:
            hardware.stop()


if __name__ == "__main__":
    main()
