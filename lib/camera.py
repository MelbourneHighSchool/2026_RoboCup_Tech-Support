import asyncio
import copy
import io
import logging
import math
import socketserver
import threading
import time
from http import server
from pathlib import Path
from threading import Condition

import cv2
import numpy as np
from picamera2 import Picamera2
from picamera2.encoders import H264Encoder, JpegEncoder
from picamera2.outputs import FileOutput, PyavOutput
from picamera2.request import MappedArray

from calibration.ball_distance import (
    DEFAULT_BOT_DISTANCE_CALIBRATION_FILE,
    DEFAULT_DISTANCE_CALIBRATION_FILE,
    calculate_ball_bearing_deg,
    get_distance_calibration_resolution,
    load_distance_calibration,
    predict_distance_from_calibration,
)
from lib.config import load_camera_bearing_offset
from lib.hailo_ball import HailoBallDetector
from lib.opencv import OpenCV

logger = logging.getLogger(__name__)

_PROJECT_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_BALL_MODEL_PATH = _PROJECT_ROOT / "open-soccer-detect-n_hailo_model"
DEFAULT_RESOLUTION = (640, 640)


def _goal_lined_up(contours, frame_width, frame_height, bearing_offset_deg=270.0):
    """Check the filled target goal along the configured forward image ray."""
    mask = np.zeros((frame_height, frame_width), dtype=np.uint8)
    cv2.drawContours(mask, contours, -1, 255, cv2.FILLED)
    ray = np.zeros_like(mask)
    angle = math.radians(90.0 - bearing_offset_deg)
    length = math.hypot(frame_width, frame_height)
    centre = (frame_width // 2, frame_height // 2)
    endpoint = (
        round(centre[0] + length * math.cos(angle)),
        round(centre[1] + length * math.sin(angle)),
    )
    cv2.line(ray, centre, endpoint, 255, 1)
    return bool(np.any(mask & ray))


def _detection_dict_from_xyxy(xyxy, confidence, frame_width, frame_height, *, point="centre"):
    """Build the ball-style detection dict from an xyxy box.

    ``point`` selects which image point feeds radial_pixels / bearing:
    - ``"centre"``: bbox centre (ball and bots)
    - ``"bottom_centre"``: bottom edge midpoint (legacy option)
    """
    x1, y1, x2, y2 = xyxy
    centre_x = (x1 + x2) / 2.0
    centre_y = (y1 + y2) / 2.0
    if point == "bottom_centre":
        point_x = centre_x
        point_y = float(y2)
    else:
        point_x = centre_x
        point_y = centre_y
    return {
        "bbox": (
            round(x1),
            round(y1),
            round(x2 - x1),
            round(y2 - y1),
        ),
        "centre": (centre_x, centre_y),
        "point": (point_x, point_y),
        "radial_pixels": float(
            np.hypot(point_x - frame_width / 2.0, point_y - frame_height / 2.0)
        ),
        "confidence": float(confidence),
        "polygon": None,
    }


def _resolve_model_path(model_path):
    model_path = Path(model_path)
    if model_path.is_absolute():
        return model_path
    return _PROJECT_ROOT / model_path

class Camera:
    def __init__(
        self,
        PORT,
        resolution=None,
        frame_rate=90,
        distance_calibration_file=DEFAULT_DISTANCE_CALIBRATION_FILE,
        ball_model_path=DEFAULT_BALL_MODEL_PATH,
        ball_confidence=0.25,
        recording_path=None,
        session_epoch_monotonic=None,
        detection_callback=None,
        diagnostics=False,
        bot_distance_calibration_file=DEFAULT_BOT_DISTANCE_CALIBRATION_FILE,
        enable_goal_detection=True,
        camera_bearing_offset_deg=None,
    ):
        self.camera_bearing_offset_deg = (
            load_camera_bearing_offset()
            if camera_bearing_offset_deg is None else camera_bearing_offset_deg
        )
        self.ball_model_path = _resolve_model_path(ball_model_path)
        self.diagnostics_enabled = diagnostics
        self._diagnostic_snapshot = None
        self.inference_error = None
        self.enable_goal_detection = enable_goal_detection
        self.goal_detector = OpenCV() if enable_goal_detection else None
        self.ball_confidence = ball_confidence
        self.ball_model = None
        self.picam2 = None
        try:
            self.ball_model = HailoBallDetector(
                self.ball_model_path,
                conf=ball_confidence,
            )
            calib_res = get_distance_calibration_resolution(distance_calibration_file)
            if resolution is None:
                resolution = calib_res or (self.ball_model.imgsz, self.ball_model.imgsz)
            self.PORT = PORT
            self.resolution = resolution
            self.frame_rate = frame_rate
            self.recording_path = Path(recording_path) if recording_path is not None else None
            self.session_epoch_monotonic = session_epoch_monotonic
            self.detection_callback = detection_callback
            self.picam2 = Picamera2()
            config_options = {
                "main": {"size": resolution, "format": "RGB888"},
                "buffer_count": 8 if self.recording_path is not None else 4,
            }
            if self.recording_path is not None:
                # Feed the software H.264 encoder YUV directly so it does not convert
                # the RGB inference stream on the CPU.
                config_options["lores"] = {"size": resolution, "format": "YUV420"}
            video_config = self.picam2.create_video_configuration(**config_options)
            self.picam2.configure(video_config)
            self.picam2.controls.FrameRate = frame_rate
            # self.picam2.controls.ExposureTime = 30000
            self.picam2.controls.AnalogueGain = 10.0
            self.output = self.StreamingOutput()
            self.server = None
            self.server_thread = None
            self.user_callback = None
            self._bearing = None
            self._distance = None
            self._bot_measurements = []
            self._lined_up = not enable_goal_detection
            self._frame_id = 0
            self._measurement_lock = threading.Lock()
            self._capture_started = False
            self._recording = False
            self._video_recording = False
            self._video_encoder = None
            self._video_output = None
            self._recording_error = None
            self._first_sensor_timestamp_ns = None
            self._video_start_elapsed_s = None
            self._server_started = False
            self._is_shutting_down = False
            # Latest-frame slot: callback only publishes; infer thread drops stale frames.
            self._latest_lock = threading.Lock()
            self._latest_buf = None
            self._latest_seq = 0
            self._latest_sensor_timestamp_ns = None
            self._latest_capture_monotonic = None
            self._last_detection = None
            self._infer_stop = threading.Event()
            self._infer_thread = None
            self.calibrated = True
            self.upperbound = 0
            self.lowerbound = 0
            self.colours = []
            self.distance_calibration = load_distance_calibration(
                resolution=resolution,
                calibration_file=distance_calibration_file,
            )
            self._distance_calibration_warning_logged = False
            self.bot_distance_calibration = load_distance_calibration(
                self.resolution, calibration_file=bot_distance_calibration_file,
            )
            if self.bot_distance_calibration is None:
                logger.warning(
                    "Bot distance calibration '%s' unavailable; bot distances will be unavailable.",
                    bot_distance_calibration_file,
                )
            if self.distance_calibration is None:
                logger.warning(
                    "Ball distance calibration file '%s' was not loaded; distance estimates will be unavailable.",
                    distance_calibration_file,
                )

            # Enable detection callback by default
            self.picam2.pre_callback = self._proxy_callback

            # # print camera modes
            # print(self.picam2.sensor_modes)

            # Define async tasks
            self.async_tasks = [self.run_server]
        except BaseException:
            if self.picam2 is not None:
                self.picam2.close()
            if self.ball_model is not None:
                self.ball_model.close()
            raise

    @property
    def stream_enabled(self):
        return self._server_started

    class StreamingOutput(io.BufferedIOBase):
        def __init__(self):
            self.frame = None
            self.condition = Condition()

        def write(self, buf):
            with self.condition:
                self.frame = buf
                self.condition.notify_all()

    class StreamingHandler(server.BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path in ('/', '/index.html'):
                self.serve_index()
            elif self.path.startswith('/stream.mjpg'):
                self.serve_stream()
            else:
                self.send_error(404)

        def serve_index(self):
            content = (
                b"<html><head><title>Camera Preview</title></head>"
                b"<body><img src='/stream.mjpg' /></body></html>"
            )
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(content)))
            self.end_headers()
            self.wfile.write(content)

        def serve_stream(self):
            self.send_response(200)
            self.send_header('Age', 0)
            self.send_header('Cache-Control', 'no-cache, private')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
            self.end_headers()
            try:
                while True:
                    with self.server.camera.output.condition:
                        self.server.camera.output.condition.wait()
                        frame = self.server.camera.output.frame
                    self.wfile.write(b'--FRAME\r\n')
                    self.send_header('Content-Type', 'image/jpeg')
                    self.send_header('Content-Length', len(frame))
                    self.end_headers()
                    self.wfile.write(frame)
                    self.wfile.write(b'\r\n')
            except Exception as e:
                logger.warning(
                    'Removed streaming client %s: %s',
                    self.client_address, str(e))

    class StreamingServer(socketserver.ThreadingMixIn, server.HTTPServer):
        allow_reuse_address = True
        daemon_threads = True

        def __init__(self, camera, *args, **kwargs):
            self.camera = camera
            super().__init__(*args, **kwargs)

    @property
    def bearing(self):
        if self._is_shutting_down:
            return None
        with self._measurement_lock:
            return self._bearing

    @property
    def distance(self):
        if self._is_shutting_down:
            return None
        with self._measurement_lock:
            return self._distance

    @property
    def frame_id(self):
        with self._measurement_lock:
            return self._frame_id

    @property
    def capture_count(self):
        """Number of frames published by the Picamera2 callback."""
        with self._latest_lock:
            return self._latest_seq

    @property
    def infer_count(self):
        """Number of Hailo inference completions (same counter as frame_id)."""
        with self._measurement_lock:
            return self._frame_id

    @property
    def recording_info(self):
        return {
            "video_start_elapsed_s": self._video_start_elapsed_s,
            "captured_frames": self.capture_count,
            "inference_frames": self.infer_count,
            "recording_error": (
                str(self._recording_error) if self._recording_error is not None else None
            ),
        }

    def get_measurement(self):
        if self._is_shutting_down:
            with self._measurement_lock:
                return self._frame_id, None, None
        with self._measurement_lock:
            return self._frame_id, self._bearing, self._distance

    def get_scene_measurement(self):
        """Atomically return frame ID, ball bearing/distance, bots, and goal alignment.

        Each bot entry is ``(bearing_deg, distance_mm)``. Distance may be ``None``
        when calibration is missing; bots without a usable bearing are omitted.
        """
        if self._is_shutting_down:
            with self._measurement_lock:
                return self._frame_id, None, None, [], False
        with self._measurement_lock:
            return (
                self._frame_id,
                self._bearing,
                self._distance,
                list(self._bot_measurements),
                self._lined_up,
            )

    def set_callback(self, callback_function):
        self.user_callback = callback_function
        self.picam2.pre_callback = self._proxy_callback

    def _detect_scene(self, frame):
        """One multi-class inference → best ball dict + list of bot dicts."""
        detections = self.ball_model.predict(frame)
        frame_h, frame_w = frame.shape[:2]
        ball_detection = None
        bot_detections = []
        for det in detections:
            class_id = int(det["class_id"])
            if class_id == self.ball_model.ball_class_id:
                candidate = _detection_dict_from_xyxy(
                    det["xyxy"],
                    det["confidence"],
                    frame_w,
                    frame_h,
                    point="centre",
                )
                if (
                    ball_detection is None
                    or candidate["confidence"] > ball_detection["confidence"]
                ):
                    ball_detection = candidate
            elif class_id == self.ball_model.bot_class_id:
                bot_detections.append(
                    _detection_dict_from_xyxy(
                        det["xyxy"],
                        det["confidence"],
                        frame_w,
                        frame_h,
                        point="centre",
                    )
                )
        return ball_detection, bot_detections

    def _detect_ball(self, frame):
        ball_detection, _bot_detections = self._detect_scene(frame)
        return ball_detection

    def _polar_from_detection(self, detection, frame_width, frame_height, *, target="ball"):
        point_x, point_y = detection.get("point", detection["centre"])
        bearing = calculate_ball_bearing_deg(
            point_x,
            point_y,
            frame_width,
            frame_height,
        )
        bearing += self.camera_bearing_offset_deg
        distance = predict_distance_from_calibration(
            self.bot_distance_calibration if target == "bot" else self.distance_calibration,
            detection["radial_pixels"],
        )
        return bearing, distance

    def _start_infer_thread(self):
        if self._infer_thread is not None:
            return
        self._infer_stop.clear()
        self._infer_thread = threading.Thread(
            target=self._infer_worker,
            name="hailo-latest-infer",
            daemon=True,
        )
        self._infer_thread.start()

    def _stop_infer_thread(self):
        self._infer_stop.set()
        thread = self._infer_thread
        if thread is not None:
            thread.join(timeout=5.0)
            if thread.is_alive():
                logger.error(
                    "Hailo inference thread did not stop; detector will remain open"
                )
                return False
        self._infer_thread = None
        return True

    def _handle_recording_error(self, error):
        self._recording_error = error
        logger.error("Video recording failed: %s", error)

    def get_diagnostic_snapshot(self):
        """Return an owned copy of the exact, unannotated inference frame and results."""
        with self._measurement_lock:
            return copy.deepcopy(self._diagnostic_snapshot)

    def reload_distance_calibration(self, calibration_file=None, *, target="ball"):
        if target not in ("ball", "bot"):
            raise ValueError("Unknown calibration target")
        if calibration_file is None:
            calibration_file = (
                DEFAULT_BOT_DISTANCE_CALIBRATION_FILE if target == "bot"
                else DEFAULT_DISTANCE_CALIBRATION_FILE
            )
        calibration = load_distance_calibration(self.resolution, calibration_file)
        if calibration is None:
            raise ValueError("Calibration is invalid or has a different resolution")
        with self._measurement_lock:
            if target == "bot":
                self.bot_distance_calibration = calibration
            else:
                self.distance_calibration = calibration
                self._distance_calibration_warning_logged = False

    def _infer_worker(self):
        try:
            self._infer_loop()
        except Exception as exc:
            self.inference_error = str(exc)
            logger.exception("Camera inference stopped")

    def _infer_loop(self):
        """Always infer the newest published frame; skip anything older."""
        last_seq = -1
        infer_buf = None
        while not self._infer_stop.is_set() and not self._is_shutting_down:
            with self._latest_lock:
                seq = self._latest_seq
                if seq == last_seq or self._latest_buf is None:
                    frame = None
                else:
                    if infer_buf is None or infer_buf.shape != self._latest_buf.shape:
                        infer_buf = np.empty_like(self._latest_buf)
                    np.copyto(infer_buf, self._latest_buf)
                    frame = infer_buf
                    last_seq = seq
                    sensor_timestamp_ns = self._latest_sensor_timestamp_ns
                    capture_time = getattr(self, "_latest_capture_monotonic", None) or time.monotonic()
            if frame is None:
                time.sleep(0.0005)
                continue

            inference_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            detection, bot_detections = self._detect_scene(inference_frame)
            frame_h, frame_w = frame.shape[:2]
            bot_measurements = []
            if detection is not None:
                new_bearing, new_distance = self._polar_from_detection(
                    detection, frame_w, frame_h
                )
                if new_distance is None and (
                    self.distance_calibration is None
                    and not self._distance_calibration_warning_logged
                ):
                    logger.warning(
                        "Ball detected, but no valid distance calibration is loaded."
                    )
                    self._distance_calibration_warning_logged = True
            else:
                new_bearing = None
                new_distance = None

            for bot_det in bot_detections:
                bot_bearing, bot_distance = self._polar_from_detection(
                    bot_det, frame_w, frame_h, target="bot"
                )
                if bot_bearing is None:
                    continue
                bot_measurements.append((bot_bearing, bot_distance))

            if getattr(self, "enable_goal_detection", True):
                hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                cv = getattr(self, "goal_detector", None) or OpenCV()
                contours = cv.process_image(hsv_frame, True)
                yellow_contours = cv.process_image(hsv_frame, False)
                lined_up = _goal_lined_up(
                    contours, frame_w, frame_h, self.camera_bearing_offset_deg
                )
            else:
                contours, yellow_contours, lined_up = [], [], True

            with self._measurement_lock:
                self._last_detection = detection
                self._bearing = new_bearing
                self._distance = new_distance
                self._goal_contours = contours
                self._yellow_goal_contours = yellow_contours
                self._bot_measurements = bot_measurements
                self._lined_up = lined_up
                self._frame_id += 1
                inference_sequence = self._frame_id
                if getattr(self, "diagnostics_enabled", False):
                    self._diagnostic_snapshot = {
                        "frame": frame.copy(),
                        "frame_id": inference_sequence,
                        "capture_sequence": seq,
                        "timestamp": capture_time,
                        "sensor_timestamp_ns": sensor_timestamp_ns,
                        "ball": copy.deepcopy(detection),
                        "bots": copy.deepcopy(bot_detections),
                    }

            if self.detection_callback is not None:
                if (
                    sensor_timestamp_ns is not None
                    and self._first_sensor_timestamp_ns is not None
                ):
                    video_time_s = (
                        sensor_timestamp_ns - self._first_sensor_timestamp_ns
                    ) / 1_000_000_000
                else:
                    video_time_s = None
                if (
                    video_time_s is not None
                    and self._video_start_elapsed_s is not None
                ):
                    # Use the sensor timeline after anchoring its first frame to
                    # the session. Callback scheduling jitter then cannot drift
                    # detections relative to encoded video.
                    elapsed_s = self._video_start_elapsed_s + video_time_s
                else:
                    elapsed_s = None
                try:
                    self.detection_callback(
                        {
                            "elapsed_s": elapsed_s,
                            "video_time_s": video_time_s,
                            "capture_sequence": seq,
                            "sensor_timestamp_ns": sensor_timestamp_ns,
                            "inference_sequence": inference_sequence,
                            "detection": detection,
                            "bots": bot_detections,
                        }
                    )
                except Exception as exc:
                    logger.warning("Detection metadata callback failed: %s", exc)

    def _start_capture(self, enable_stream=False):
        if self._capture_started:
            return

        if enable_stream and self.recording_path is not None:
            raise RuntimeError(
                "session recording and the MJPEG HTTP preview cannot run together"
            )

        if enable_stream:
            # JPEG encode frames into the MJPEG buffer used by the HTTP preview.
            self.picam2.start_recording(JpegEncoder(), FileOutput(self.output))
            self._recording = True
        elif self.recording_path is not None:
            self.recording_path.parent.mkdir(parents=True, exist_ok=True)
            try:
                encoder = H264Encoder(
                    bitrate=4_000_000,
                    framerate=self.frame_rate,
                    iperiod=max(1, round(self.frame_rate)),
                )
                # This is already the Pi 5 default, but setting it documents and
                # preserves the low-CPU recording choice.
                if hasattr(encoder, "preset"):
                    encoder.preset = "ultrafast"
                # The recording-session path uses .ts (MPEG transport stream),
                # which stays readable after an unclean shutdown because it has
                # no MP4-style end-of-file index to finalize. PyAV selects the
                # muxer from this suffix.
                output = PyavOutput(str(self.recording_path))
                if hasattr(output, "error_callback"):
                    output.error_callback = self._handle_recording_error
                self.picam2.start_recording(encoder, output, name="lores")
                self._video_encoder = encoder
                self._video_output = output
                self._video_recording = True
                self._recording = True
            except Exception as exc:
                self._recording_error = exc
                logger.warning("Video recording unavailable; continuing capture: %s", exc)
                self.picam2.start()
                self._recording = False
        else:
            # Capture-only path: run the detection callback without JPEG encoding.
            self.picam2.start()
            self._recording = False
        self._capture_started = True
        self._start_infer_thread()

    def _start_http_server(self):
        if self._server_started:
            return

        address = ("", self.PORT)
        self.server = self.StreamingServer(self, address, self.StreamingHandler)
        self.server_thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.server_thread.start()
        self._server_started = True
        print(f"Camera preview available at http://localhost:{self.PORT}/")
        print(f"Raw MJPEG stream available at http://localhost:{self.PORT}/stream.mjpg")

    def start(self):
        """Start camera capture so bearing updates in callback (no MJPEG preview)."""
        self._start_capture(enable_stream=False)

    def start_stream(self):
        """Start camera capture and the MJPEG HTTP stream."""
        self._start_capture(enable_stream=True)
        self._start_http_server()

    def _proxy_callback(self, request):
        if self._is_shutting_down:
            return
        try:
            with MappedArray(request, "main") as m:
                if self._is_shutting_down:
                    return

                preview_active = self._server_started
                needs_hsv = (not self.calibrated) or self.user_callback is not None
                hsv = cv2.cvtColor(m.array, cv2.COLOR_BGR2HSV) if needs_hsv else None

                if self.calibrated:
                    metadata = request.get_metadata()
                    sensor_timestamp_ns = metadata.get("SensorTimestamp")
                    capture_monotonic = time.monotonic()
                    if (
                        sensor_timestamp_ns is not None
                        and self._first_sensor_timestamp_ns is None
                    ):
                        self._first_sensor_timestamp_ns = int(sensor_timestamp_ns)
                        if self.session_epoch_monotonic is not None:
                            self._video_start_elapsed_s = (
                                capture_monotonic - self.session_epoch_monotonic
                            )
                    # Publish latest frame only (infer thread drops anything older).
                    with self._latest_lock:
                        if (
                            self._latest_buf is None
                            or self._latest_buf.shape != m.array.shape
                        ):
                            self._latest_buf = np.empty_like(m.array)
                        np.copyto(self._latest_buf, m.array)
                        self._latest_seq += 1
                        self._latest_sensor_timestamp_ns = sensor_timestamp_ns
                        self._latest_capture_monotonic = capture_monotonic

                    with self._measurement_lock:
                        detection = self._last_detection

                    if detection is not None and preview_active:
                        x, y, w, h = detection["bbox"]
                        centre_x, centre_y = detection["centre"]
                        if detection["polygon"] is not None:
                            points = detection["polygon"].astype(np.int32).reshape(-1, 1, 2)
                            cv2.polylines(m.array, [points], True, (0, 165, 255), 2)
                        else:
                            cv2.rectangle(m.array, (x, y), (x + w, y + h), (0, 165, 255), 2)
                        cv2.circle(
                            m.array,
                            (round(centre_x), round(centre_y)),
                            5,
                            (255, 255, 255),
                            -1,
                        )

                    if self.user_callback:
                        self.user_callback(m.array, hsv)
                else:
                    # Only run for calibration, when self.calibrated is false
                    center_y = m.array.shape[0] - 250
                    center_x = m.array.shape[1] - 1200

                    if preview_active:
                        cv2.line(
                            m.array,
                            (center_x, center_y - 10),
                            (center_x, center_y + 10),
                            (0, 0, 255),
                            2,
                        )
                        cv2.line(
                            m.array,
                            (center_x - 10, center_y),
                            (center_x + 10, center_y),
                            (0, 0, 255),
                            2,
                        )

                    # Check for colour values in certain range
                    pixel_colour = hsv[center_y, center_x]
                    if (
                        10 < pixel_colour[0] < 25
                        and 100 < pixel_colour[1] < 255
                        and 100 < pixel_colour[2] < 255
                        and (pixel_colour[0], pixel_colour[1], pixel_colour[2]) not in self.colours
                    ):
                        self.colours.append((pixel_colour[0], pixel_colour[1], pixel_colour[2]))

                    # Add detected colour to text file
                    with open("calibrate_camera.txt", "w") as c:
                        for i in self.colours:
                            c.write(str(i[0]) + " " + str(i[1]) + " " + str(i[2]))
                            c.write("\n")
                    with self._measurement_lock:
                        self._bearing = None
                        self._distance = None
                        self._frame_id += 1

        except Exception as e:
            logger.warning("Camera callback error (may be during shutdown): %s", e)

    async def run_server(self):
        """Async task to run the camera server"""
        self.start_stream()

        try:
            # Keep the task alive
            while True:
                await asyncio.sleep(1)
        except asyncio.CancelledError:
            self.stop()

    def stop(self):
        if self._is_shutting_down:
            return
        self._is_shutting_down = True
        infer_stopped = self._stop_infer_thread()


        try:
            self.picam2.pre_callback = None
        except Exception as e:
            logger.warning("Error clearing camera callback: %s", e)

        if self.server:
            try:
                self.server.shutdown()
                self.server.server_close()
            except Exception as e:
                logger.warning("Error shutting down camera HTTP server: %s", e)
            self.server = None
        self.server_thread = None
        self._server_started = False
        if self._capture_started:
            try:
                if self._recording:
                    self.picam2.stop_recording()
                else:
                    self.picam2.stop()
            except Exception as e:
                logger.warning("Error stopping camera capture: %s", e)
            self._capture_started = False
            self._recording = False
            self._video_recording = False
            self._video_encoder = None
            self._video_output = None
        if infer_stopped:
            try:
                self.ball_model.close()
            except Exception as e:
                logger.warning("Error closing Hailo detector: %s", e)
        try:
            self.picam2.close()
        except Exception as exc:
            logger.warning("Error closing camera: %s", exc)
        print("Camera stopped")

async def main():
    camera = Camera(PORT=8000, frame_rate=90)
    await camera.run_server()

if __name__ == "__main__":
    asyncio.run(main())
