# C++ hardware controller

`main.py` uses `lib.hardware_controller.HardwareController`, a pybind11 extension.
The C++ controller owns the four drive motors and optional fifth dribbler through
`PowerfulBLDCdriver`, plus a BNO08x IMU through the portable SH-2/SHTP core.
It also owns an optional GPIO kicker and shares its native I2C transport with
`StatusDisplay`, a landscape SSD1306 status screen. Python does no display I2C.

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
also needs a startup reference. Without fresh yaw, native heading correction is
disabled and translation uses the last known heading. Native health reporting
does not replace movement targets or interrupt a kick.

`main.py` handles IMU loss by setting `run = False` and entering the same paused
branch as the pause switch. That branch sends `move(0, 0, 0, 0, 0)`: translation
ramps down normally, rotation and dribbler targets are zero, and an already active
kick finishes its pulse. Only shutdown or a fatal controller failure interrupts
a pulse. The screen reports `RUN OFF / PAUSED` with the IMU error.

Recovery remains in Python: the operator pauses, main collects 25 distinct fresh
yaw readings at 20 ms intervals with a fresh gyro, and re-zeroes before permitting
a later run transition. Reconnection alone cannot restart the game. A new outage
invalidates the sampling window; `health()["imu_recovery_generation"]` detects
outages even between Python polls. There is no native pause/acknowledgement API.

`health()` returns `imu_healthy`, `imu_recovery_generation`, `fault_source`,
`error`, and `motor_address` (decimal, `-1` when inapplicable). Motor faults use
source `MOTOR`; kicker faults use `OTHER`. IMU constructor failures are reported
as `IMU`, rather than motor disconnections. A sensor that never reports is marked
unavailable after one second. Reset notifications invalidate reports and
re-enable both streams; a reset can shift the raw yaw origin.

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
batches, including the repeated SHTP headers in 32-byte I2C reads. With a display,
both objects share the same `TwoWire` object and its transaction mutex. The controller must be the
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
before waiting for motor/IMU shutdown. GPIO faults latch an `OTHER` controller
error, raise `RuntimeError` to callers, and stop the workers. Pulse
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

## Native OLED status

```python
from lib.hardware_controller import StatusDisplay

display = StatusDisplay(i2c_device="/dev/i2c-1", address=0x3C)
# Pass display=display to HardwareController.from_i2c_addresses(...).
# The supplied display determines the shared I2C transport.
display.update("STRIKER", True, "RUNNING")
display.component("LIDAR", "!", "DISCONNECTED - ODOM")
# Stop hardware, camera, LIDAR, and other resources first.
display.stop()  # Clears RAM, turns the screen/charge pump off, joins its worker.
```

`update(mode, run, state, detail="")` publishes text without I2C. `component(source,
health, error="")` uses `?` for pending initialization, `+` for healthy, and `!` for
unavailable. Passing an empty error clears that source's error. Native motor/IMU
workers publish their own health, and a fatal native controller fault overrides
the run label to `BLOCKED` even if Python is stalled. IMU pauses are handled by main. `display.error` reports OLED transport errors.

The 128×64 screen has a large mode heading, run/state row, and `L`, `C`, `I`, `M`
health indicators. Remaining rows cycle errors every two seconds with a page/count
label. Long details scroll in 21-character steps. With no errors it shows startup
progress, waiting for data, or `ALL SYSTEMS OK`. Fonts and rendering are native;
no Luma/Pillow runtime dependency is added. Do not run `lib/display.py` against the
same OLED while this screen is active.

The display worker runs at at most 5 Hz, sends changed 16-byte framebuffer chunks,
tries the shared bus without blocking, and yields between chunks. OLED failures
are reported on stderr and retried every two seconds without stopping the motors.
Display lifetime spans hardware initialization and fault shutdown. `main.py`
starts it before sensor setup, then clears it after cleanup without an exit delay.
Consequently a fatal error may only appear briefly before exit; full details
remain in the console. A disconnected/unpowered OLED cannot show a final message.

`lib/game_status.py` monitors scans and camera capture/inference independently of
the strategy loop, including while paused. One second without progress reports
an outage. Raw scans, not MCL corrections/confidence, determine LIDAR health, so
rotation gating and dead reckoning do not count as disconnection. Camera outages
remove stale scenes while retaining the existing short ball extrapolation,
teammate input (when communication is enabled), and break-beam possession.
`lidar.clear_imu_yaw()` removes the retained IMU prior without resetting particles;
main clears it while IMU input is unavailable, then restores it after a fresh re-zero. Startup still requires
successful sensor initialization and a first pose. Automatic device reopening
is not added.

Offline checks:

```bash
.venv/bin/python -m unittest tests.test_game_status tests.test_hardware_controller tests.test_localisation_motion
.venv/bin/ruff check
```

Pi acceptance: rebuild both extensions; check orientation/readability for all
three modes; toggle pause; unplug each sensor independently while observing the
screen and console; verify LIDAR prediction and camera fallback; verify IMU loss
enters the normal paused branch and requires a paused re-zero before running again;
verify an active kick finishes normally.
Check simultaneous faults, motor-address reporting, missing OLED operation, and
blanking on normal/error exit. Compare `main.py --fps` drive/IMU rates with display
updates active against the prior baseline. Physical timing and driver response
must be checked on the robot; the offline suites emulate the bus and GPIO.
