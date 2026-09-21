from lib.break_beam import Breakbeam
from lib.config import load_config
from lib.hardware_test_utils import create_hardware

USE_PCB = False

config = load_config()
break_beam = Breakbeam(config.break_beam_pin)
hardware = create_hardware(kicker=True, use_pcb=USE_PCB)

print("IR Breakbeam Sensor Test Initialized.")
print("Waiting for beam to be broken...")

try:
    while True:
        broken = break_beam.read()
        print("Beam is BROKEN!" if broken else "Beam is solid (unbroken).")
        hardware.move(0, 0, 0, 0, kick=broken)
except KeyboardInterrupt:
    print("Test stopped.")
finally:
    hardware.stop()
