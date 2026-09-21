"""Dashboard state and image processing; imports Pi drivers only when requested."""

import copy
import json
import math
import os
import queue
import secrets
import shutil
import threading
import time
from collections import OrderedDict, deque
from pathlib import Path

import cv2

from calibration.ball_distance import (
    calculate_ball_bearing_deg,
    fit_distance_calibration,
    load_distance_calibration,
    predict_distance_from_calibration,
    save_distance_calibration,
)
from calibration.dashboard_hardware import PITCH, Hardware
from lib.config import load_camera_bearing_offset
from lib.line_sensors import line_thresholds
from lib.opencv import DEFAULT_THRESHOLDS, OpenCV, load_thresholds, validate_thresholds

USE_PCB = False


def discover_models(root):
    """Return compiled Hailo model variants available in the project root."""
    labels = {"n": "Nano", "s": "Small", "m": "Medium", "l": "Large", "x": "Extra large"}
    models = []
    for path in sorted(Path(root).glob("open-soccer-detect-*_hailo_model")):
        if not (path / "model.hef").is_file():
            continue
        model_id = path.name.removeprefix("open-soccer-detect-").removesuffix("_hailo_model")
        input_size = None
        try:
            import yaml

            metadata = yaml.safe_load((path / "metadata.yaml").read_text(encoding="utf-8")) or {}
            if isinstance(metadata, str):
                metadata = yaml.safe_load(metadata) or {}
            if not isinstance(metadata, dict):
                metadata = {}
            size = metadata.get("imgsz")
            if isinstance(size, int):
                input_size = [size, size]
            elif isinstance(size, list) and len(size) == 2:
                input_size = [int(size[0]), int(size[1])]
        except (OSError, TypeError, ValueError, yaml.YAMLError):
            pass
        models.append({
            "id": model_id,
            "label": labels.get(model_id, model_id.upper()),
            "input_size": input_size,
            "path": str(path),
        })
    return models


def number(value, low, high, label):
    if isinstance(value, bool):
        raise TypeError(f"Invalid {label}")
    try:
        value = float(value)
    except (ValueError, TypeError) as exc:
        raise ValueError(f"Invalid {label}") from exc
    if not math.isfinite(value) or not low <= value <= high:
        raise ValueError(f"{label} must be between {low} and {high}")
    return value


def addresses(value):
    if not isinstance(value, list) or not 1 <= len(value) <= 8:
        raise ValueError("Enter 1–8 ordered I2C addresses")
    if any(type(v) is not int or not 8 <= v <= 119 for v in value) or len(set(value)) != len(value):
        raise ValueError("I2C addresses must be unique integers between 8 and 119")
    return value


def backup(path):
    path = Path(path)
    if path.exists():
        folder = path.parent / "calibration_backups"
        folder.mkdir(exist_ok=True)
        stamp = time.strftime("%Y%m%d-%H%M%S") + f"-{time.time_ns() % 1000000000:09d}"
        shutil.copy2(path, folder / f"{path.stem}-{stamp}.json")


def save_json(path, value):
    backup(path)
    temp = Path(str(path) + ".tmp")
    with temp.open("w") as file:
        json.dump(value, file, indent=2, allow_nan=False)
        file.flush()
        os.fsync(file.fileno())
    os.replace(temp, path)


class Lease:
    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.token = None
        self.deadline = 0
        self.cancel = threading.Event()
        self.cancel.set()
        self.armed = False

    def expire(self):
        if self.token and self.clock() >= self.deadline:
            self.cancel.set()
            self.armed = False
            self.token = None
            return True
        return False

    def claim(self):
        self.expire()
        if self.token:
            raise ValueError("Another browser currently has control")
        self.token = secrets.token_urlsafe(24)
        self.deadline = self.clock() + 2
        return self.token

    def check(self, token):
        self.expire()
        if not token or token != self.token:
            raise PermissionError("Take control first; previous control may have expired")

    def heartbeat(self, token):
        self.check(token)
        self.deadline = self.clock() + 2

    def arm(self, token):
        self.check(token)
        self.cancel = threading.Event()
        self.armed = True

    def stop(self):
        self.cancel.set()
        self.armed = False


def encode(frame, extension=".jpg"):
    ok, encoded = cv2.imencode(extension, frame)
    if not ok:
        raise RuntimeError("Could not encode camera frame")
    return encoded.tobytes()


def pixel_values(frame, x, y):
    height, width = frame.shape[:2]
    if type(x) is not int or type(y) is not int or not (0 <= x < width and 0 <= y < height):
        raise ValueError("Pixel outside image")
    bgr = frame[y, x]
    hsv = cv2.cvtColor(frame[y:y + 1, x:x + 1], cv2.COLOR_BGR2HSV)[0, 0]
    return {"x": x, "y": y, "bgr": bgr.tolist(), "rgb": bgr[::-1].tolist(), "hsv": hsv.tolist()}


def scene(frame, ball, bots, calibration, bot_calibration=None):
    """Render only detections belonging to this exact source frame."""
    overlay = frame.copy()
    height, width = frame.shape[:2]
    bearing_offset = load_camera_bearing_offset()
    results = []
    for label, detections, colour in (("Ball", [ball] if ball else [], (0, 165, 255)),
                                      ("Bot", bots, (255, 180, 30))):
        for detection in detections:
            x, y, w, h = detection["bbox"]
            point = detection.get("point", detection["centre"])
            bearing = calculate_ball_bearing_deg(*point, width, height) + bearing_offset
            distance = predict_distance_from_calibration(
                calibration if label == "Ball" else bot_calibration, detection["radial_pixels"]
            )
            result = {**detection, "label": label, "bearing": bearing, "distance": distance}
            results.append(result)
            cv2.rectangle(overlay, (x, y), (x + w, y + h), colour, 2)
            text = f"{label} {detection['confidence']:.0%} {bearing % 360:.0f}deg"
            if distance is not None:
                text += f" {distance:.0f}mm"
            cv2.putText(overlay, text, (max(0, x), max(15, y - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, colour, 1)
    return overlay, results


class Dashboard:
    def __init__(self, root, *, fps=15, camera_factory=None, hardware_factory=Hardware, start=True,
                 lidar_port="/dev/ttyUSB0", lidar_baud=460800, use_pcb=USE_PCB):
        self.root = Path(root)
        self.fps = fps
        self.lock = threading.RLock()
        self.condition = threading.Condition(self.lock)
        self.lease = Lease()
        self.closing = threading.Event()
        self.camera = None
        self.camera_factory = camera_factory
        self.camera_status = "starting"
        self.analogue_gain = 10.0
        self.analogue_gain_range = None
        self.model_options = discover_models(self.root)
        model_ids = {model["id"] for model in self.model_options}
        self.requested_model = "n" if "n" in model_ids else (self.model_options[0]["id"] if self.model_options else None)
        self.active_model = None
        self.model_change = threading.Event()
        self.latest = None
        self.streams = {}
        self.stream_sequence = 0
        self.frozen = OrderedDict()
        self.events = deque(maxlen=300)
        self.history = deque(maxlen=300)
        self.thresholds = load_thresholds(self.root / "goal_thresholds.json")
        self.line_thresholds = {"black": 64, "white": 192}
        try:
            self.line_thresholds = line_thresholds(json.loads(
                (self.root / "line_sensor_calibration.json").read_text()))
        except FileNotFoundError:
            pass
        except (OSError, ValueError, TypeError) as exc:
            self.notify(f"Line sensor calibration: {exc}; using provisional thresholds", error=True)
        self.samples = []
        self.sample_resolution = None
        self.fit = None
        self.calibration = None
        self.bot_samples = []
        self.bot_fit = None
        self.bot_calibration = None
        self.detections = []
        self.rates = {"capture": 0, "inference": 0, "preview": 0}
        self.jobs = queue.Queue(maxsize=8)
        self.default_addresses = []
        try:
            for line in (self.root / "config.txt").read_text().splitlines():
                key, sep, value = line.partition("=")
                if sep and key.strip() == "i2c_addresses":
                    self.default_addresses = addresses([int(v) for v in value.split("#")[0].split(",")])
        except (OSError, ValueError) as exc:
            self.notify(f"Motor configuration: {exc}", error=True)
        self.hardware = hardware_factory(self.root, self.notify, port=lidar_port, baud=lidar_baud,
                                         use_pcb=use_pcb)
        self.threads = []
        if start:
            for name, target in (("preview", self._camera_loop), ("actions", self._jobs), ("watchdog", self._watchdog)):
                thread = threading.Thread(target=target, name="dashboard-" + name, daemon=True)
                thread.start()
                self.threads.append(thread)

    def notify(self, message, error=False):
        with self.lock:
            self.events.append({"time": time.time(), "message": message, "error": error})

    def _watchdog(self):
        last_history = 0
        while not self.closing.wait(0.05):
            hardware_state = self.hardware.snapshot()
            local = hardware_state.get("localisation") or {}
            loop_stale = (hardware_state["mode"] == "driving"
                          and time.monotonic() - local.get("timestamp", 0) > 0.5)
            loop_stale = loop_stale or (
                hardware_state["mode"] == "manual"
                and time.monotonic() - hardware_state.get("manual_tick", 0) > 0.5
            )
            with self.lock:
                if loop_stale:
                    self.lease.stop()
                    self.notify("Drive watchdog: localisation worker stalled", error=True)
                expired = self.lease.expire()
                stopped = self.lease.cancel.is_set()
                if stopped:
                    self.lease.armed = False
            if stopped:
                self.hardware.stop_drive()
            if expired:
                self.notify("Control lease expired; disarmed")
            if time.monotonic() - last_history > 1:
                with self.lock:
                    self.history.append({"time": time.time(), "hardware": self.hardware.snapshot(),
                                         "fps": dict(self.rates)})
                last_history = time.monotonic()

    def state(self):
        with self.lock:
            age = None if self.latest is None else time.monotonic() - self.latest["timestamp"]
            return {
                "camera": {"status": self.camera_status, "age_s": age, "fps": self.rates,
                           "frame_id": self.latest["frame_id"] if self.latest else None,
                           "resolution": self.sample_resolution, "detections": self.detections,
                           "models": [
                               {key: copy.deepcopy(model[key]) for key in ("id", "label", "input_size")}
                               for model in self.model_options
                           ],
                           "analogue_gain": self.analogue_gain,
                           "analogue_gain_range": self.analogue_gain_range,
                           "active_model": self.active_model,
                           "requested_model": self.requested_model},
                "control": {"occupied": self.lease.token is not None, "armed": self.lease.armed},
                "hardware": self.hardware.snapshot(), "thresholds": copy.deepcopy(self.thresholds),
                "line_thresholds": dict(self.line_thresholds),
                "bot_samples": list(self.bot_samples), "bot_fit": self.bot_fit,
                "samples": list(self.samples), "fit": self.fit, "addresses": self.default_addresses,
                "events": list(self.events)[-20:], "pitch": PITCH,
            }

    def command(self, action, data, token):
        with self.lock:
            if action == "claim":
                return {"token": self.lease.claim()}
            if action == "stop":
                self.lease.stop()
                self.hardware.request_stop_pcb()
                self.notify("Stop requested; disarmed")
                return {}
            self.lease.check(token)
            if action == "heartbeat":
                self.lease.heartbeat(token)
                return {}
            if action == "pcb_stop":
                self.hardware.request_stop_pcb()
                return {}
            if action == "pcb_start":
                # Reserve hardware immediately so a following stop cannot be
                # overtaken by a start still waiting in the dashboard queue.
                self.hardware.submit(action, {}, threading.Event())
                return {"queued": True}
            if action == "save_line_thresholds":
                thresholds = line_thresholds(data)
                save_json(self.root / "line_sensor_calibration.json",
                          {**thresholds, "sensor_radius_mm": 75, "sensor_count": 32})
                self.line_thresholds = thresholds
                self.notify("Line sensor thresholds saved")
                return {}
            if action == "release":
                self.lease.stop()
                self.lease.token = None
                return {}
            if action == "arm":
                mode = self.hardware.snapshot()["mode"]
                if mode not in ("idle", "stopped", "monitoring") or self.lease.armed:
                    raise ValueError("Stop the current operation before arming")
                self.lease.arm(token)
                return {}
            if action == "stop_localise":
                self.hardware.request_stop_localisation()
                return {}
            if action in ("drive", "calibrate", "kick", "manual_start", "manual_input") and not self.lease.armed:
                raise ValueError("Arm motors first")
            if action == "manual_input":
                forward = number(data.get("forward"), -1000, 1000, "forward speed")
                right = number(data.get("right"), -1000, 1000, "right speed")
                if math.hypot(forward, right) > 1000.001:
                    raise ValueError("Manual speed must not exceed 1000 mm/s")
                self.hardware.update_manual(forward, right)
                return {}
            if action == "drive":
                data["speed"] = number(data.get("speed"), 0, 5000, "speed")
                target = data.get("target")
                if not isinstance(target, list) or len(target) != 2:
                    raise ValueError("Expected target X and Y")
                data["target"] = [number(target[i], 0, PITCH[i], "target") for i in range(2)]
                if data.get("target_yaw") is not None:
                    data["target_yaw"] = number(data["target_yaw"], -180, 360, "target yaw")
                data["addresses"] = addresses(data.get("addresses"))
                if len(data["addresses"]) not in (4, 5):
                    raise ValueError("Driving requires four wheels and an optional dribbler")
            if action == "calibrate":
                data["addresses"] = addresses(data.get("addresses"))
                if data.get("wheels_clear") is not True:
                    raise ValueError("Confirm that wheels are clear before calibration")
            allowed = {"manual_start", "gpio", "kick", "drive", "calibrate", "localise", "thresholds", "save_goals", "revert_goals",
                       "default_goals", "sample", "remove_sample", "clear_samples", "fit", "save_ball",
                       "select_model", "analogue_gain"}
            if action not in allowed:
                raise ValueError("Unknown action")
            if action == "thresholds":
                data = {"thresholds": validate_thresholds(data.get("thresholds"))}
            self.jobs.put_nowait((action, copy.deepcopy(data), token, self.lease.cancel))
            return {"queued": True}

    def _jobs(self):
        while not self.closing.is_set():
            try:
                action, data, token, cancel = self.jobs.get(timeout=0.1)
            except queue.Empty:
                continue
            try:
                with self.lock:
                    self.lease.check(token)
                    if action in ("drive", "calibrate", "localise", "gpio", "kick", "manual_start"):
                        if action not in ("localise", "gpio") and cancel.is_set():
                            raise ValueError("Operation cancelled; arm again")
                        if self.hardware.snapshot()["mode"] not in ("idle", "stopped", "monitoring"):
                            raise ValueError("Hardware operation already running")
                        if action == "calibrate":
                            backup(self.root / "calibration_data.json")
                        # Localisation monitoring does not require armed motors.
                        if action in ("localise", "gpio"):
                            cancel = threading.Event()
                        self.hardware.submit(action, data, cancel)
                    else:
                        self._edit(action, data)
            except (Exception, SystemExit) as exc:
                self.notify(f"{action}: {exc}", error=True)

    def _edit(self, action, data):
        goal_path = self.root / "goal_thresholds.json"
        if action == "select_model":
            model_id = data.get("model")
            if model_id not in {model["id"] for model in self.model_options}:
                raise ValueError("Selected model is not available on this Pi")
            if model_id != self.requested_model:
                self.requested_model = model_id
                self.camera_status = "switching model"
                self.model_change.set()
                self.notify(f"Switching detection model to {model_id}")
        elif action == "analogue_gain":
            if self.camera_status != "running" or self.analogue_gain_range is None:
                raise ValueError("Camera analogue gain is unavailable")
            gain = number(data.get("gain"), *self.analogue_gain_range, "analogue gain")
            self.camera.picam2.set_controls({"AnalogueGain": gain})
            self.analogue_gain = gain
            self.notify(f"Analogue gain set to {gain:g}×")
        elif action == "thresholds":
            self.thresholds = data["thresholds"]
        elif action == "save_goals":
            save_json(goal_path, self.thresholds)
            if self.camera is not None:
                self.camera.goal_detector = OpenCV(self.thresholds)
            self.notify("Goal thresholds saved")
        elif action == "revert_goals":
            self.thresholds = load_thresholds(goal_path)
        elif action == "default_goals":
            self.thresholds = copy.deepcopy(DEFAULT_THRESHOLDS)
        elif action in ("sample", "remove_sample", "clear_samples", "fit", "save_ball"):
            self._edit_distance(action, data)

    def _edit_distance(self, action, data):
        target = data.get("target", "ball")
        if target not in ("ball", "bot"):
            raise ValueError("Unknown calibration target")
        prefix = "bot_" if target == "bot" else ""
        samples = getattr(self, prefix + "samples")
        fit = getattr(self, prefix + "fit")
        if action == "sample":
            snap = self.latest
            if snap is None or time.monotonic() - snap["timestamp"] > 0.5:
                raise ValueError("A fresh detection is required")
            if target == "bot":
                bots = snap.get("bots", [])
                if len(bots) != 1:
                    raise ValueError("Keep exactly one detected bot in view when sampling")
                det = bots[0]
            else:
                det = snap["ball"]
                if det is None:
                    raise ValueError("A fresh detected ball is required")
            distance = number(data.get("distance"), 0.001, 50000, "distance (mm)")
            x, y = det["centre"]
            captured = target == "ball" and data.get("captured") is True
            sample = {"distance_mm": distance, "radial_pixels": det["radial_pixels"],
                      "centre_x": x, "centre_y": y, "bounding_box_area": det["bbox"][2] * det["bbox"][3],
                      "captured": captured}
            if captured:
                from calibration.ball_distance import apply_camera_bearing_offset
                sample["bearing_deg"] = apply_camera_bearing_offset(
                    calculate_ball_bearing_deg(x, y, *self.sample_resolution))
            if len(samples) >= 500:
                raise ValueError("Maximum 500 samples; remove samples before adding more")
            samples.append(sample)
            fit = None
        elif action == "remove_sample":
            index = data.get("index")
            if type(index) is not int or not 0 <= index < len(samples):
                raise ValueError("Invalid sample index")
            samples.pop(index)
            fit = None
        elif action == "clear_samples":
            samples = []
            fit = None
        elif action in ("fit", "save_ball"):
            radial = [s["radial_pixels"] for s in samples]
            if len(radial) < 2 or len({round(r, 5) for r in radial}) != len(radial):
                raise ValueError("Use at least two distinct radial positions; remove duplicate positions")
            fit = fit_distance_calibration(samples)
            if not all(math.isfinite(v) for v in fit["coefficients"]):
                raise ValueError("Degenerate fit")
            if action == "save_ball":
                if self.camera is None or self.sample_resolution is None:
                    raise ValueError("Camera unavailable")
                path = self.root / f"{target}_distance_calibration.json"
                backup(path)
                calibration, _ = save_distance_calibration(samples, self.sample_resolution, str(path))
                setattr(self, prefix + "calibration", calibration)
                if target == "bot":
                    self.camera.reload_distance_calibration(str(path), target="bot")
                else:
                    self.camera.reload_distance_calibration(str(path))
                self.notify(f"{target.title()} distance calibration saved and reloaded")

        setattr(self, prefix + "samples", samples)
        setattr(self, prefix + "fit", fit)

    def freeze(self):
        with self.lock:
            if self.latest is None:
                raise ValueError("No camera frame available")
            key = secrets.token_hex(12)
            self.frozen[key] = self.latest["frame"].copy()
            while len(self.frozen) > 8:
                self.frozen.popitem(last=False)
            height, width = self.frozen[key].shape[:2]
            return {"id": key, "width": width, "height": height, "frame_id": self.latest["frame_id"]}

    def frozen_frame(self, key):
        with self.lock:
            if key not in self.frozen:
                raise ValueError("Frozen frame expired; freeze again")
            return self.frozen[key]

    def _camera_loop(self):
        if self.camera_factory is None:
            from lib.camera import Camera
            self.camera_factory = Camera
        while not self.closing.is_set():
            with self.lock:
                model_id = self.requested_model
                model = next((item for item in self.model_options if item["id"] == model_id), None)
                self.model_change.clear()
                self.latest = None
                self.detections = []
                self.streams = {}
            self._camera_session(model_id, model)
            if not self.closing.is_set() and not self.model_change.is_set():
                self.model_change.wait()

    def _camera_session(self, model_id, model):
        try:
            camera_args = {
                "PORT": 0,
                "diagnostics": True,
                "distance_calibration_file": str(self.root / "ball_distance_calibration.json"),
                "bot_distance_calibration_file": str(self.root / "bot_distance_calibration.json"),
            }
            if model is not None:
                camera_args["ball_model_path"] = model["path"]
            camera = self.camera_factory(**camera_args)
            with self.lock:
                self.camera = camera
                picam2 = getattr(camera, "picam2", None)
                limits = picam2.camera_controls.get("AnalogueGain") if picam2 else None
                self.analogue_gain_range = [float(v) for v in limits[:2]] if limits else None
                if self.analogue_gain_range is not None:
                    gain = max(self.analogue_gain_range[0], min(self.analogue_gain_range[1], self.analogue_gain))
                    picam2.set_controls({"AnalogueGain": gain})
                    self.analogue_gain = gain
            camera.start()
            with self.lock:
                self.sample_resolution = list(camera.resolution)
                self.calibration = load_distance_calibration(camera.resolution, str(self.root / "ball_distance_calibration.json"))
                if self.calibration is not None:
                    self.samples = self.calibration.get("samples", [])[:500]
                    self.fit = self.calibration.get("model")
                self.bot_calibration = load_distance_calibration(
                    camera.resolution, str(self.root / "bot_distance_calibration.json")
                )
                if self.bot_calibration is not None:
                    self.bot_samples = self.bot_calibration.get("samples", [])[:500]
                    self.bot_fit = self.bot_calibration.get("model")
                self.active_model = model_id
                self.camera_status = "running"
            counts = (0, 0, 0)
            last_rate = time.monotonic()
            while not self.closing.is_set() and not self.model_change.is_set():
                started = time.monotonic()
                if camera.inference_error:
                    raise RuntimeError(camera.inference_error)
                snap = camera.get_diagnostic_snapshot()
                if snap is not None:
                    with self.lock:
                        bounds = copy.deepcopy(self.thresholds)
                        calibration = self.calibration
                        bot_calibration = self.bot_calibration
                    frame = snap["frame"]
                    overlay, detections = scene(frame, snap["ball"], snap["bots"], calibration, bot_calibration)
                    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                    detector = OpenCV(bounds)
                    blue, yellow = detector.mask(hsv, True), detector.mask(hsv, False)
                    goals = overlay.copy()
                    for is_blue, colour in ((True, (255, 160, 0)), (False, (0, 255, 255))):
                        cv2.drawContours(goals, detector.process_image(hsv, is_blue), -1, colour, 2)
                    streams = {"camera": encode(overlay), "raw": encode(frame),
                               "blue": encode(blue), "yellow": encode(yellow), "goals": encode(goals)}
                    with self.condition:
                        self.latest = snap
                        self.detections = detections
                        self.streams = streams
                        self.stream_sequence += 1
                        self.condition.notify_all()
                now = time.monotonic()
                if now - last_rate >= 1:
                    new_counts = (camera.capture_count, camera.infer_count, self.stream_sequence)
                    with self.lock:
                        self.rates = dict(zip(("capture", "inference", "preview"),
                                              [(a - b) / (now - last_rate) for a, b in zip(new_counts, counts, strict=True)], strict=True))
                    counts, last_rate = new_counts, now
                self.closing.wait(max(0, 1 / self.fps - (time.monotonic() - started)))
        except (Exception, SystemExit) as exc:
            with self.lock:
                self.active_model = None
                self.camera_status = f"unavailable: {exc}"
            self.notify(f"Camera: {exc}", error=True)
        finally:
            with self.lock:
                camera = self.camera
                self.camera = None
                self.analogue_gain_range = None
            if camera is not None:
                camera.stop()

    def close(self):
        with self.condition:
            self.lease.stop()
            self.closing.set()
            self.model_change.set()
            self.condition.notify_all()
        self.hardware.close()
        for thread in self.threads:
            thread.join(timeout=6)
