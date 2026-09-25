"""Exercise the PCB LED brightness command over Raspberry Pi I2C."""

import argparse
import time

try:
    from smbus2 import SMBus
except ImportError:
    try:
        from smbus import SMBus
    except ImportError as exc:
        raise SystemExit("Install smbus2 with: pip install smbus2") from exc


PCB_ADDRESS = 0x37
MAX_BRIGHTNESS = 0xFE  # 0xFF is the kicker command.
DEFAULT_LEVELS = (0, 32, 64, 127, 191, MAX_BRIGHTNESS, 191, 127, 64, 32)


def brightness(value: str) -> int:
    level = int(value, 0)
    if not 0 <= level <= MAX_BRIGHTNESS:
        raise argparse.ArgumentTypeError("brightness must be between 0 and 254")
    return level


def set_brightness(bus: SMBus, level: int) -> None:
    # Send exactly one byte. The PCB treats any value except 0xFF as brightness.
    bus.write_byte(PCB_ADDRESS, level)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Set or repeatedly ramp the PCB LED brightness over I2C."
    )
    parser.add_argument("level", nargs="?", type=brightness, help="fixed brightness (0-254)")
    parser.add_argument("--bus", type=int, default=1, help="Linux I2C bus number (default: 1)")
    parser.add_argument(
        "--delay", type=float, default=0.5, help="seconds between ramp levels (default: 0.5)"
    )
    args = parser.parse_args()

    if args.delay < 0:
        parser.error("--delay must not be negative")

    with SMBus(args.bus) as bus:
        try:
            if args.level is not None:
                set_brightness(bus, args.level)
                print(f"PCB LED brightness set to {args.level}/254")
                return

            print("Ramping PCB LEDs. Press Ctrl+C to stop and turn them off.")
            while True:
                for level in DEFAULT_LEVELS:
                    set_brightness(bus, level)
                    print(f"Brightness: {level:3}/254")
                    time.sleep(args.delay)
        except KeyboardInterrupt:
            print("\nStopping test.")
        finally:
            # Do not leave the LEDs powered after a ramp or failed test.
            set_brightness(bus, 0)


if __name__ == "__main__":
    main()
