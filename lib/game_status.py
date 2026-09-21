"""Game health publication and camera freshness; no Python I2C access."""

import threading
import time

USE_PCB = False


class ProgressHealth:
    """Track progress, allowing a one-second startup interval before reporting loss."""

    def __init__(self, now):
        self.count = 0
        self.changed_at = now
        self.seen = False

    def update(self, count, now):
        if count != self.count:
            self.count = count
            self.changed_at = now
            self.seen = True
        if now - self.changed_at >= 1.0:
            return "!"
        return "+" if self.seen else "?"


class ImuPause:
    """Apply the game's normal pause until the operator pauses and re-zeroes."""

    def __init__(self, generation):
        self.generation = generation
        self.pending = False

    def update(self, requested_run, health):
        generation = health["imu_recovery_generation"]
        if not health["imu_healthy"] or generation != self.generation:
            self.pending = True
        self.generation = generation
        return requested_run and not self.pending

    def rezeroed(self, requested_run, health):
        # Re-zeroing during an automatic pause cannot bypass the operator's pause.
        if (not requested_run and health["imu_healthy"]
                and health["imu_recovery_generation"] == self.generation):
            self.pending = False


class GameStatus:
    """One lightweight monitor runs during startup waits and paused operation too."""

    def __init__(self, display, lidar, mode, clock=time.monotonic, *, use_pcb=USE_PCB):
        self.use_pcb = use_pcb
        self.display = display
        self.lidar = lidar
        self.clock = clock
        self.hardware = None
        self.camera = None
        self.recording = None
        self.mode = mode
        self.run = False
        self.stage = "STARTING"
        self.detail = "LIDAR"
        self.camera_ready = False
        self._trackers = {}
        self._errors = {}
        self._lock = threading.RLock()
        self._stop = threading.Event()
        self._thread = None

    def start(self):
        self._thread = threading.Thread(target=self._worker, name="game-status", daemon=True)
        self._thread.start()

    def update(self, mode, run, stage="", detail=""):
        with self._lock:
            self.mode, self.run, self.stage, self.detail = mode, run, stage, detail

    def attach_hardware(self, hardware):
        with self._lock:
            self.hardware = hardware

    def attach_camera(self, camera):
        with self._lock:
            self.camera = camera

    def attach_recording(self, recording):
        with self._lock:
            self.recording = recording

    def report(self, source, error):
        message = str(error)
        with self._lock:
            if self._errors.get(source) != message:
                if message:
                    print(f"{source}: {message}", flush=True)
                elif source in self._errors:
                    print(f"{source}: recovered", flush=True)
            if message:
                self._errors[source] = message
            else:
                self._errors.pop(source, None)
            if self.display is not None:
                self.display.component(source, "!" if message else "+", message)

    def _progress(self, name, count, now):
        tracker = self._trackers.setdefault(name, ProgressHealth(now))
        return tracker.update(count, now)

    def poll(self):
        with self._lock:
            now = self.clock()
            lidar_health = self._progress("lidar", self.lidar.get_scan_generation(), now)
            fallback = "ODOM"
            if lidar_health == "!" and self.use_pcb:
                lines = self.lidar.get_line_readings()
                if (lines["applied_count"] > 0 and
                        0 <= now - lines["last_applied_timestamp_s"] <= 0.5):
                    fallback = "ODOM+PCB"
            self.report("LIDAR", f"DISCONNECTED - {fallback}" if lidar_health == "!" else "")
            if self.display is not None:
                self.display.component("LIDAR", lidar_health, self._errors.get("LIDAR", ""))
            if self.camera is not None:
                capture = self._progress("capture", self.camera.capture_count, now)
                infer = self._progress("infer", self.camera.infer_count, now)
                error = self.camera.inference_error
                if capture == "!":
                    error = "DISCONNECTED - NO FRAMES"
                elif not error and infer == "!":
                    error = "INFERENCE STALLED"
                self.camera_ready = capture == "+" and infer == "+" and not error
                self.report("CAMERA", error or "")
                if self.display is not None:
                    self.display.component("CAMERA", "!" if error else (
                        "+" if self.camera_ready else "?"), error or "")
                recording_error = self.camera.recording_info.get("recording_error")
                if recording_error:
                    self.report("RECORDING", recording_error)
            if self.recording is not None:
                for source, writer in (("GAME LOG", self.recording.game_writer),
                                       ("DETECTION LOG", self.recording.detection_writer)):
                    if writer.error is not None:
                        self.report(source, writer.error)
            blocked = False
            if self.hardware is not None:
                health = self.hardware.health()
                blocked = not health["imu_healthy"]
                if blocked:
                    self.lidar.clear_imu_yaw()
                if health["error"]:
                    self.report(health["fault_source"] or "OTHER", health["error"])
                    blocked = True
            state = self.stage or ("BLOCKED" if self.run and blocked else (
                "RUNNING" if self.run else "PAUSED"))
            if self.display is not None:
                self.display.update(self.mode, self.run, state, self.detail)
            return self.camera_ready

    def _worker(self):
        while not self._stop.is_set():
            try:
                self.poll()
            except Exception as exc:
                self.report("OTHER", f"STATUS: {exc}")
            self._stop.wait(0.1)

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join()
