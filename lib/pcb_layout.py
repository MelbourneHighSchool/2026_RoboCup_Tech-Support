"""Packed PCB readings and physical bearings; see i2c_protocol.md.

The native/firmware equivalent is STM32/Core/Inc/pcb_sensor_layout.h.
Cross-language tests keep this hardware-free dashboard metadata in sync.
"""

SENSOR_COUNT = 30
POSITION_COUNT = 32
DEAD_POSITIONS = (5, 6)
SENSOR_CHANNELS = (
    *((2, pin) for pin in range(9, 14)),
    (2, 15),
    *((1, pin) for pin in range(1, 16)),
    *((2, pin) for pin in range(9)),
)


def sensor_layout():
    """Return owned metadata for 30 working sensors and two dead positions."""
    return {
        "sensor_count": SENSOR_COUNT,
        "position_count": POSITION_COUNT,
        "sensors": [
            {"index": index, "multiplexer": mux, "pin": pin,
             "bearing_deg": (index if index < 5 else index + 2) * 360 / POSITION_COUNT}
            for index, (mux, pin) in enumerate(SENSOR_CHANNELS)
        ],
        "dead_bearings_deg": [position * 360 / POSITION_COUNT for position in DEAD_POSITIONS],
    }
