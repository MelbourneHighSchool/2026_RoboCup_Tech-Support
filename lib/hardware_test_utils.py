"""Shared setup for interactive native hardware diagnostics."""

import math
import time

from lib.config import load_config
from lib.hardware_controller import HardwareController

WHEEL_DIAMETER = 50
MAX_YAW_RPM = 100
MAX_MOTOR_RPM = 1000
YAW_CORRECT_THRESHOLD = 3
USE_PCB = False


def create_hardware(*, max_motor_rpm=MAX_MOTOR_RPM, kicker=False, use_pcb=USE_PCB):
    config = load_config()
    kwargs = {"kicker_pin": int(config.kicker_pin.id)} if kicker else {}
    return HardwareController.from_i2c_addresses(
        config.i2c_addresses,
        WHEEL_DIAMETER,
        MAX_YAW_RPM,
        max_motor_rpm,
        YAW_CORRECT_THRESHOLD,
        use_pcb=use_pcb,
        **kwargs,
    )


def set_startup_yaw(hardware, sample_count=25, sample_interval=0.02):
    """Average native raw-yaw samples and install the startup reference."""
    print("Stabilizing IMU yaw reference...")
    sin_sum = cos_sum = 0.0
    samples = 0
    deadline = time.monotonic() + 5.0
    while samples < sample_count:
        if time.monotonic() >= deadline:
            raise TimeoutError("No IMU yaw received during startup")
        yaw = hardware.get_raw_imu_yaw()
        if yaw is not None:
            yaw_rad = math.radians(yaw)
            sin_sum += math.sin(yaw_rad)
            cos_sum += math.cos(yaw_rad)
            samples += 1
        time.sleep(sample_interval)
    startup_yaw = math.degrees(math.atan2(sin_sum, cos_sum))
    hardware.set_startup_yaw(startup_yaw)
    return startup_yaw
