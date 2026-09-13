"""Single-owner hardware worker for the LAN calibration dashboard."""

import copy
import json
import math
import queue
import threading
import time
from collections import deque

PITCH = (2430, 1820)


def target_command(pose, target, speed, target_yaw=None):
    """Return direction/speed/yaw and the RPM-limited speed for dashboard display."""
    x, y, yaw = pose[:3]
    dx, dy = target[0] - x, target[1] - y
    distance = math.hypot(dx, dy)
    direction = math.degrees(math.atan2(dy, dx))
    requested = 0 if distance <= 10 else min(speed, distance / 300.0 * speed)
    angle = math.radians(yaw - direction + 45)
    peak = max(abs(math.sin(angle)), abs(math.cos(angle)))
    limit = 400 * 50 * math.pi / 60 / peak
    rotation = yaw if target_yaw is None else target_yaw
    yaw_error = (rotation - yaw + 180) % 360 - 180
    arrived = distance <= 10 and abs(yaw_error) <= 5
    return direction, requested, rotation, min(requested, limit), arrived


class Hardware:
    def __init__(self, root, notify, *, port="/dev/ttyUSB0", baud=460800):
        self.root = root
        self.notify = notify
        self.port = port
        self.baud = baud
        self.lock = threading.RLock()
        self.controller = None
        self.session = None
        self.pause = None
        self.break_beam = None
        self.last_kick = float("-inf")
        self.active_cancel = threading.Event()
        self.localisation_stop_requested = threading.Event()
        self.target = None
        self.target_yaw = None
        self.speed = 500
        self.manual = None
        self.motion_cancel = threading.Event()
        self.motion_cancel.set()
        self.jobs = queue.Queue(maxsize=4)
        self.closing = threading.Event()
        self.status = {"mode": "idle", "localisation": None, "error": None}
        self.trail = deque(maxlen=300)
        self.thread = threading.Thread(target=self._run, name="dashboard-hardware", daemon=True)
        self.thread.start()

    def submit(self, action, data, cancel):
        with self.lock:
            if self.status["mode"] not in ("idle", "stopped", "monitoring"):
                raise ValueError("Hardware operation already running")
            self.jobs.put_nowait((action, data, cancel))
            self.status["mode"] = "queued " + action

    def snapshot(self):
        with self.lock:
            return copy.deepcopy({**self.status, "target": self.target, "trajectory": list(self.trail)})

    def stop_drive(self):
        with self.lock:
            controller, self.controller = self.controller, None
            self.target = None
            self.manual = None
            if controller is None:
                return
            self.status["mode"] = "stopping"
        errors = []
        try:
            if self.session is not None and controller is self.session.imu:
                controller.move(0, 0, 0, 0)
            else:
                controller.stop()
        except Exception as exc:
            errors.append(str(exc))
        finally:
            with self.lock:
                if errors:
                    self.status["error"] = "Motor shutdown failed: " + "; ".join(errors)
                self.status["mode"] = "monitoring" if self.session else "idle"
        if errors:
            self.notify("Motor shutdown failed: " + "; ".join(errors), error=True)

    def _localise(self, cancel):
        if self.session is not None:
            return
        from lib.hardware_controller import HardwareController

        from lib import lidar
        from lib.config import load_config
        from lib.localisation_service import (
            LidarVelocityEstimator,
            LocalisationSession,
            capture_startup_yaw,
            feed_imu_yaw_prior,
        )

        imu = None
        try:
            lidar.init(self.port, self.baud)
            config = load_config()
            imu = HardwareController.from_i2c_addresses(
                config.i2c_addresses,
                50,
                100,
                400,
                3,
                calibration_file=str(self.root / "calibration_data.json"),
            )
            startup = capture_startup_yaw(imu, cancel_event=cancel)
            feed_imu_yaw_prior(lidar, imu, startup)
            lidar.start_coordinates(*PITCH)
            self.session = LocalisationSession(lidar, imu, startup, LidarVelocityEstimator())
        except BaseException:
            if imu is not None:
                imu.stop()
            lidar.shutdown()
            raise

    def request_stop_localisation(self):
        """Ask the hardware thread to stop driving and release LIDAR/IMU."""
        self.active_cancel.set()
        self.motion_cancel.set()
        self.localisation_stop_requested.set()
        with self.lock:
            self.status["mode"] = "stopping localisation"

    def _stop_localisation(self):
        self.stop_drive()
        with self.lock:
            session, self.session = self.session, None
        if session is not None:
            session.close()
        with self.lock:
            self.status.update(mode="idle", localisation=None, error=None)
            self.trail.clear()
        self.localisation_stop_requested.clear()
        self.notify("Localisation stopped")

    def _start_gpio(self):
        from lib.break_beam import Breakbeam
        from lib.config import load_config

        if self.break_beam is None:
            self.break_beam = Breakbeam(load_config().break_beam_pin)

    def _kick(self, cancel):
        import digitalio

        from lib.config import load_config
        from lib.switch import Switch

        config = load_config()
        if self.pause is None:
            self.pause = Switch(config.pause_switch_pin)
        if self.pause.read():
            raise ValueError("Physical pause switch is active")
        if cancel.is_set():
            return
        if time.monotonic() - self.last_kick < 0.5:
            raise ValueError("Kicker is cooling down")
        pin = digitalio.DigitalInOut(config.kicker_pin)
        try:
            pin.switch_to_input(pull=digitalio.Pull.DOWN)
            if cancel.is_set():
                return
            self.last_kick = time.monotonic()
            pin.switch_to_output(value=True)
            cancel.wait(0.02)
        finally:
            try:
                pin.switch_to_input(pull=digitalio.Pull.DOWN)
            finally:
                pin.deinit()
        self.notify("Kicker pulse complete")

    def _drive(self, data, cancel):
        from lib.config import load_config
        from lib.switch import Switch

        if self.session is None or not self.session.state.get("fresh"):
            raise ValueError("Start localisation and wait for a fresh pose before driving")
        if not self.session.state["pose"][4]:
            raise ValueError("No confident pose")
        if self.pause is None:
            self.pause = Switch(load_config().pause_switch_pin)
        if self.pause.read():
            raise ValueError("Physical pause switch is active")
        saved = json.loads((self.root / "calibration_data.json").read_text())
        saved_addresses = [motor["address"] for motor in saved["motors"]]
        if saved_addresses[:len(data["addresses"])] != data["addresses"]:
            raise ValueError("Drive addresses/order must match the saved motor calibration")
        controller = self.session.imu
        with self.lock:
            if cancel.is_set():
                controller.move(0, 0, 0, 0)
                return
            self.controller = controller
            self.target = data["target"]
            self.target_yaw = data.get("target_yaw")
            self.speed = data["speed"]
            self.motion_cancel = cancel
            self.status["mode"] = "driving"

    def _start_manual(self, cancel):
        from lib.hardware_controller import HardwareController

        from lib.config import load_config
        from lib.localisation_service import capture_startup_yaw
        from lib.switch import Switch

        if self.session is not None:
            raise ValueError("Stop localisation before starting manual control")
        config = load_config()
        if self.pause is None:
            self.pause = Switch(config.pause_switch_pin)
        if self.pause.read():
            raise ValueError("Physical pause switch is active")
        controller = HardwareController.from_i2c_addresses(
            config.i2c_addresses, 50, 100, 400, 3,
            calibration_file=str(self.root / "calibration_data.json"),
        )
        try:
            capture_startup_yaw(controller, cancel_event=cancel)
            with self.lock:
                if cancel.is_set():
                    controller.stop()
                    return
                self.controller = controller
                self.motion_cancel = cancel
                self.manual = (0, 0, time.monotonic() + 1.0)  # Allow the browser to observe startup.
                self.status.update(mode="manual", manual_tick=time.monotonic())
        except BaseException:
            controller.stop()
            raise

    def update_manual(self, forward, right):
        with self.lock:
            if self.status["mode"] != "manual" or self.motion_cancel.is_set():
                raise ValueError("Start manual control first")
            self.manual = (forward, right, time.monotonic())

    def _tick_manual(self):
        with self.lock:
            if self.manual is None or self.controller is None:
                return
            forward, right, received = self.manual
            if self.motion_cancel.is_set() or self.pause.read():
                raise InterruptedError("Manual drive stopped: pause or lost control")
            if time.monotonic() - received > 0.35:
                raise InterruptedError("Manual drive stopped: input timed out")
            yaw = self.controller.get_yaw()
            direction = yaw + math.degrees(math.atan2(right, forward))
            self.controller.move(direction, math.hypot(forward, right), yaw, 0)
            self.status["manual_tick"] = time.monotonic()

    def _calibrate(self, data, cancel):
        from legacy.movement import calibrate_motors, get_motors_for_calibration

        self.stop_drive()
        if self.session is not None:
            self.session.close()
            self.session = None
        with self.lock:
            self.status.update(mode="calibrating", localisation=None)
        motors, count, addresses = get_motors_for_calibration(data["addresses"])

        def progress(info):
            with self.lock:
                self.status["calibration"] = info

        result = calibrate_motors(
            motors, count, addresses, str(self.root / "calibration_data.json"),
            cancel_event=cancel, progress=progress, timeout_s=120,
        )
        with self.lock:
            self.status["calibration"] = {"results": result["motors"], "complete": True}
        self.notify("Motor calibration saved")

    def _run(self):
        next_map = 0
        try:
            while not self.closing.is_set():
                started = time.monotonic()
                if self.localisation_stop_requested.is_set():
                    self._stop_localisation()
                    continue
                try:
                    action, data, cancel = self.jobs.get_nowait()
                except queue.Empty:
                    action = None
                try:
                    if action is not None and cancel.is_set():
                        with self.lock:
                            self.status["mode"] = "monitoring" if self.session else "idle"
                    if action is not None and not cancel.is_set():
                        self.active_cancel = cancel
                        with self.lock:
                            self.status.update(mode="starting " + action, error=None)
                        if action == "localise":
                            self._localise(cancel)
                        elif action == "gpio":
                            self._start_gpio()
                        elif action == "kick":
                            self._kick(cancel)
                            cancel.set()
                        elif action == "manual_start":
                            self._start_manual(cancel)
                        elif action == "drive":
                            self._drive(data, cancel)
                        elif action == "calibrate":
                            self._calibrate(data, cancel)
                            cancel.set()
                        if self.controller is None:
                            with self.lock:
                                self.status["mode"] = "monitoring" if self.session else "idle"
                    self._tick_manual()
                    if self.break_beam is not None:
                        try:
                            blocked = self.break_beam.read()
                            with self.lock:
                                self.status["break_beam"] = {"blocked": blocked, "error": None}
                        except Exception as exc:
                            with self.lock:
                                self.status["break_beam"] = {"blocked": None, "error": str(exc)}
                    if self.session is not None:
                        with self.lock:
                            controller = self.controller
                        state = self.session.tick(controller)
                        if time.monotonic() >= next_map:
                            state["scan"] = list(self.session.lidar.get_scan_list())[::2]
                            next_map = time.monotonic() + 0.2
                            if state["pose"][4]:
                                with self.lock:
                                    self.trail.append(state["pose"][:3])
                        else:
                            state["scan"] = (self.status.get("localisation") or {}).get("scan", [])
                        with self.lock:
                            self.status["localisation"] = state
                        if controller is not None:
                            if (self.motion_cancel.is_set() or not state["fresh"]
                                    or not state["pose"][4] or self.pause.read()):
                                raise InterruptedError("Drive stopped: pause, lost control, or stale/invalid pose")
                            with self.lock:
                                target = self.target
                            if target is not None:
                                direction, speed, yaw, limited, arrived = target_command(
                                    # Use the same native heading as the motor yaw loop.
                                    (*state["pose"][:2], controller.get_yaw()),
                                    target, self.speed, self.target_yaw,
                                )
                                with self.lock:
                                    self.status["drive"] = {
                                        "requested_speed": self.speed, "ramped_request": speed,
                                        "rpm_limited_speed": limited, "rpm_limited": limited < speed,
                                    }
                                if arrived:
                                    self.motion_cancel.set()
                                    self.stop_drive()
                                    self.notify("Target reached; disarmed")
                                else:
                                    controller.move(direction, speed, yaw, 1.0)
                except (Exception, SystemExit) as exc:
                    stopping_localisation = (
                        action == "localise"
                        and self.localisation_stop_requested.is_set()
                    )
                    self.motion_cancel.set()
                    if action is not None:
                        cancel.set()
                    self.stop_drive()
                    if stopping_localisation:
                        self._stop_localisation()
                        continue
                    with self.lock:
                        self.status["error"] = str(exc) or "Hardware initialization failed"
                        if self.status["mode"] != "stopping":
                            self.status["mode"] = "stopped"
                    self.notify(str(exc) or "Hardware initialization failed", error=True)
                    if action is None and self.session is not None:
                        with self.lock:
                            if self.status["localisation"] is not None:
                                self.status["localisation"]["fresh"] = False
                        self.session.close()
                        self.session = None
                self.closing.wait(max(0, 0.02 - (time.monotonic() - started)))
        finally:
            self.stop_drive()
            if self.session is not None:
                self.session.close()
            if self.break_beam is not None:
                self.break_beam.switch.deinit()
            if self.pause is not None:
                self.pause.switch.deinit()

    def close(self):
        self.motion_cancel.set()
        self.active_cancel.set()
        self.localisation_stop_requested.set()
        self.closing.set()
        self.stop_drive()
        self.thread.join(timeout=6)
