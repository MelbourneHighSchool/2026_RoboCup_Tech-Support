from lib.hardware_test_utils import create_hardware

USE_PCB = False


def main() -> None:
    hardware = create_hardware(use_pcb=USE_PCB)
    hardware.stop()
    print("Motors stopped.")


if __name__ == "__main__":
    main()
