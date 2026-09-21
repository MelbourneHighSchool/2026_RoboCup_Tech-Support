import time

from lib.hardware_test_utils import create_hardware, set_startup_yaw

USE_PCB = False

hardware = create_hardware(use_pcb=USE_PCB)

try:
    startup = set_startup_yaw(hardware)
    print(f"Startup yaw reference set to {startup:.6f} deg")
    while True:
        time.sleep(0.5)
        yaw = hardware.get_yaw()
        gyro_z = hardware.get_gyro_z_deg_s()
        print("Yaw: waiting for quaternion..." if yaw is None else f"Yaw: {yaw:.6f} deg")
        print("Gyro Z: waiting for report..." if gyro_z is None else f"Gyro Z: {gyro_z:.3f} deg/s")
except KeyboardInterrupt:
    pass
finally:
    hardware.stop()
