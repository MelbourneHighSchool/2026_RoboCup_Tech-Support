import math
from dataclasses import dataclass
from pathlib import Path

from state import BotMode

CONFIG_PATH = Path(__file__).resolve().parent.parent / "config.txt"  # see example_config.txt


@dataclass(frozen=True)
class Config:
    i2c_addresses: list[int]
    mode_switch_off: BotMode
    mode_switch_on: BotMode
    mode_switch_pin: object
    pause_switch_pin: object
    kicker_pin: object
    break_beam_pin: object
    lidar_port: str = "/dev/ttyUSB0"
    camera_bearing_offset_deg: float = 270.0


def _read_values(path: Path) -> dict[str, str]:
    # Parse the config file into a dictionary following these rules:
    # - Lines starting with # are ignored as comments
    # - Any key-value pair must be in the format "key=value"
    values: dict[str, str] = {}
    with path.open(encoding="utf-8") as config_file:
        for raw_line in config_file:
            line = raw_line.split("#", 1)[0].strip()
            if not line or "=" not in line:
                continue
            key, value = line.split("=", 1)
            values[key.strip().lower()] = value.strip()

    return values


def _camera_offset(values, path):
    try:
        offset = float(values.get("camera_bearing_offset_deg", "270"))
        if not math.isfinite(offset):
            raise ValueError
        return offset
    except ValueError as exc:
        raise ValueError(f"{path.name}: camera_bearing_offset_deg must be a finite number") from exc


def load_camera_bearing_offset(path: Path = CONFIG_PATH) -> float:
    """Read camera mounting without requiring GPIO settings or Pi hardware.

    Standalone camera/calibration tools retain the old mount if config is absent.
    """
    return _camera_offset(_read_values(path), path) if path.is_file() else 270.0


def load_config(path: Path = CONFIG_PATH) -> Config:
    # Load all settings from the config file

    # If the config file is not found, raise an error
    if not path.is_file():
        raise FileNotFoundError(
            f"Missing {path.name}. Copy example_config.txt to config.txt and edit as needed."
        )

    values = _read_values(path)

    # If included in the config file, parse the motor I2C addresses as a comma-separated list of integers
    # Motor I2C addresses follow the order: back left, back right, front right, front left, dribbler (optional)
    try:
        i2c_addresses = [int(part.strip()) for part in values["i2c_addresses"].split(",")]
    except (KeyError, ValueError) as exc:
        raise ValueError(
            f"{path.name}: i2c_addresses must be a comma-separated list of integers"
        ) from exc

    valid_modes = ", ".join(mode.name for mode in BotMode) # Generate a comma-separated string of the bot modes for error messages``

    def parse_mode(key: str) -> BotMode:
        # Parse the mode from the config file for a given key into a BotMode enum value
        try:
            return BotMode[values[key].upper()]
        except KeyError as exc:
            raise ValueError(
                f"{path.name}: {key} must be one of: {valid_modes}"
            ) from exc

    def parse_pin(key: str):
        import board

        # Parse a BCM GPIO number (e.g. 16) into the matching board.D# pin
        try:
            pin_number = int(values[key])
            return getattr(board, f"D{pin_number}")
        except (KeyError, ValueError) as exc:
            raise ValueError(
                f"{path.name}: {key} must be a BCM GPIO pin number"
            ) from exc
        except AttributeError as exc:
            raise ValueError(
                f"{path.name}: {key}={values.get(key)!r} is not a valid board pin"
            ) from exc

    lidar_port = values.get("lidar_port", "/dev/ttyUSB0")
    if not lidar_port:
        raise ValueError(f"{path.name}: lidar_port must be a non-empty serial device path")

    return Config(
        i2c_addresses,
        parse_mode("mode_switch_off"),
        parse_mode("mode_switch_on"),
        parse_pin("mode_switch_pin"),
        parse_pin("pause_switch_pin"),
        parse_pin("kicker_pin"),
        parse_pin("break_beam_pin"),
        lidar_port,
        _camera_offset(values, path),
    )
