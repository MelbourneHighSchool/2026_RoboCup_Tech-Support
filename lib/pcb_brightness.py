"""Shared persistent PCB LED brightness for the dashboard and game."""

import json
from pathlib import Path

BRIGHTNESS_PATH = Path(__file__).resolve().parents[1] / "pcb_brightness.json"


def validate_brightness(level):
    if type(level) is not int or not 0 <= level <= 254:
        raise ValueError("PCB brightness must be a whole number from 0 to 254")
    return level


def load_brightness(path=BRIGHTNESS_PATH):
    """None means leave firmware brightness unchanged until explicitly saved."""
    try:
        data = json.loads(Path(path).read_text())
    except FileNotFoundError:
        return None
    if not isinstance(data, dict):
        raise TypeError("PCB brightness settings must be an object")
    return validate_brightness(data.get("brightness"))


def apply_brightness(level):
    level = validate_brightness(level)
    from lib.hardware_controller import PcbSensorReader

    # This is a single addressed I2C transaction, with no register-selection
    # state. It can coexist with sensor reads and the native motor worker.
    PcbSensorReader().set_brightness(level)


def restore_brightness(path=BRIGHTNESS_PATH):
    level = load_brightness(path)
    if level is not None:
        apply_brightness(level)
    return level
