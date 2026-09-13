# C++ hardware controller

`main.py` uses `lib.hardware_controller.HardwareController`, a pybind11 extension.
The C++ controller owns the four drive motors and optional fifth dribbler through
`PowerfulBLDCdriver`, plus a BNO08x IMU through the portable SH-2/SHTP core.
It also owns an optional GPIO kicker. The display remains in Python.

Build on the target Pi using its runtime Python environment:

```bash
.venv/bin/python lib/setup.py build_ext --inplace
```

To build only the motor/IMU extension without the LIDAR SDK:

```bash
SOCCER_HARDWARE_ONLY=1 .venv/bin/python lib/setup.py build_ext --inplace
```

Requires Linux I2C and GPIO v2 headers, C11 and C++17 compilers, setuptools, and pybind11. The native
module must be rebuilt for the Pi's architecture and Python version; desktop
binaries cannot be copied to the Pi. Run the existing `calibration/motors.py` to
create `calibration_data.json` before starting the game.

```python
from lib.hardware_controller import HardwareController
import time

with HardwareController.from_i2c_addresses(
    [25, 26, 27, 28, 29],  # Use this robot's actual addresses in wheel order.
    diameter=50,
    max_yaw_rpm=100,
    max_rpm=1000,
    yaw_correct_threshold=3,
    calibration_file="calibration_data.json",
    i2c_device="/dev/i2c-1",
    imu_address=0x4A,
    imu_report_interval_ms=10,
) as hardware:
    raw_yaw = hardware.get_raw_imu_yaw()
    while raw_yaw is None:
        time.sleep(0.01)
        raw_yaw = hardware.get_raw_imu_yaw()
    hardware.set_startup_yaw(raw_yaw)  # main.py averages a burst of raw samples.
    hardware.move(direction=0, speed=0, rotation=0,
                  rotation_speed=0, dribbler=0)
    yaw = hardware.get_yaw()
    gyro_z = hardware.get_gyro_z_deg_s()
    vx, vy = hardware.get_measured_body_velocity_mm_s(yaw_deg=0)
```

Relative calibration paths resolve against the project root. Entries are matched
by I2C address, so their order in the JSON file does not matter. The requested
address order is back-left, back-right, front-right, front-left, optional dribbler.
Missing/duplicate calibration addresses, invalid limits, and unsupported firmware
(expected version 3) fail initialization. The controller loads calibration; it
does not run physical calibration.

The four drive motors use speed command mode with an 8 A FOC current limit. The
optional fifth motor is configured separately in torque command mode with a 1 A
FOC current limit. Its `-1`, `0`, or `1` command maps to `-1 A`, `0 A`, or `1 A`
of requested Q-axis current.

`move()` stores targets. A native thread runs at 50 Hz, limits vector acceleration
to 8000 mm/s² with a 40 ms maximum step, reserves wheel RPM headroom for heading
correction, and writes speed/torque. It continues while Python is busy, without
acquiring the GIL. Headings are clockwise-positive from startup-forward; speed is
mm/s, rotation strength is clamped to 0..1, and dribbler is -1, 0, or 1.
`move(direction, speed, rotation, rotation_speed, dribbler=0, kick=False)` no longer accepts
yaw: the drive thread reads the latest native IMU sample every tick. There is no
command-age watchdog. Measured velocity is forward/left in mm/s; the odometry
getter still accepts `yaw_deg` for API
compatibility, but direct body-frame inversion does not require it.

The IMU enables game rotation vector (no magnetometer) and calibrated gyroscope
at 10 ms intervals by default. Its native worker drains bounded batches of
reports every 2 ms. `get_raw_imu_yaw()` is used for startup sampling;
`set_startup_yaw(raw_yaw)` sets the reference; `get_yaw()` returns
`wrap(startup_yaw - raw_yaw)`. `get_gyro_z_deg_s()` returns negated sensor gyro Z in degrees/second
(clockwise-positive with the existing upside-down mounting), and
`get_latest_quaternion()` returns components in `(i, j, k, real)` order.

Yaw/quaternion and gyro have independent 100 ms freshness deadlines. Getters
return `None` before data arrives or when their stream is stale; relative yaw
also needs a startup reference. While yaw is unavailable, heading correction is
disabled and translation uses the last known relative yaw (zero before the first
reference). Translation and dribbler continue. Correction resumes on a fresh yaw
report. Reset notifications immediately invalidate both streams and re-enable
both reports. The stored startup reference is retained; a sensor reset can change
its raw yaw origin, so re-zero while paused if the physical heading reference shifts.

`loop_count`, `current_speed`, `current_direction`, and `imu_update_count` are
read-only diagnostics. The IMU counter counts decoded quaternion reports, not polls.
`stop()` joins all workers, closes SH-2, and disables motors; it can be repeated.
A stopped or faulted controller cannot restart; create a new instance. Motor I2C read or
write faults stop the loop, latch a `MotorCommunicationError`, and trigger attempts
to disable every motor. Explicit `stop()` reports failed shutdown writes and can
be retried. Destruction also attempts shutdown. Software cannot guarantee a stop
if the physical bus/driver fails or the process is forcibly killed.

`linux_wire.*` provides only the Arduino Wire operations used by the supplied
driver. It uses addressed Linux `I2C_RDWR` messages and checks failed/short
transfers. A native mutex serializes motor operations and complete IMU service
batches, including the repeated SHTP headers in 32-byte I2C reads. The kernel
serializes messages with Python display transfers. The controller must be the
sole owner of its motor and IMU addresses; do not run the legacy Python IMU,
movement controller, calibration, or dashboard hardware sessions alongside it.
The SH-2 core has global session state, so only one native BNO08x session may be
open per process. A second instance is rejected before motor writes.

`imu/linux_bno08x.*` provides Linux HAL callbacks. The unused Arduino Adafruit
wrapper has been removed; its license and source attribution are retained. HAL callbacks contain I/O
exceptions and return explicit write errors to avoid unbounded SHTP retries.
Initialization verifies reset completion, reads product IDs with a one-second
timeout, and configures reports before starting workers. IMU initialization errors
fail construction and disable the motors; runtime read errors age the cached data.

Enable the kicker by passing `kicker_pin=21` (the configured BCM GPIO number) to
`from_i2c_addresses()`. The default `-1` disables it; requesting a kick without a
configured pin raises an error. `main.py` obtains the BCM number from the existing
configured Blinka pin's `id`. Linux GPIO v2 calls run without Python or its GIL;
the GPIO chip is discovered from the Pi's device tree, or can be overridden with
`kicker_gpiochip="/dev/gpiochip0"` using the correct chip for the system.

A separate kicker thread consumes each accepted `move(..., kick=True)` request
once: output high for 20 ms, then low and input with pull-down, matching
`legacy/kicker.py`. Requests during a pulse or the 500 ms start-to-start cooldown are
ignored. An old target does not fire again; another accepted `move()` is required.
A newer move can replace a pending request but does not interrupt an active pulse.
The pulse uses no I2C lock. Shutdown interrupts it and joins the kicker worker
before waiting for motor/IMU shutdown. GPIO faults latch a controller error using
the existing `MotorCommunicationError` exception and stop the workers. Pulse
duration is nominal: Linux scheduling can extend it. The legacy Python kicker
remains available for standalone scripts, which must not share this GPIO.

Offline verification (does not open a hardware device):

```bash
.venv/bin/python -m unittest tests.test_hardware_controller
.venv/bin/ruff check
```

The tests exercise the actual C++ driver against a fake Wire transport, including
packet encoding, signed QDR speeds, frame conventions, RPM saturation, delayed-I2C
acceleration limits, optional dribbler, firmware failure, fault latching, and
shutdown cleanup. Fake SHTP advertisements/reports exercise the actual SH-2 parser,
including chunk framing, yaw/gyro decoding, staleness, report reconfiguration,
shared-bus serialization, native yaw control, and bounded initialization failures.
Fake GPIO tests cover pulses while I2C is blocked, cooldown, consumed requests,
shutdown during a pulse, repeated shutdown, and GPIO failure cleanup.
Physical motor direction, IMU signs, timing, and bus coexistence still
need verification on the Pi with the wheels lifted before a field run.
