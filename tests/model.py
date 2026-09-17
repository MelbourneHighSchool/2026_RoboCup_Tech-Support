import argparse
import threading
import time
from pathlib import Path

import cv2
import numpy as np

from calibration.ball_distance import (
    DEFAULT_DISTANCE_CALIBRATION_FILE,
    apply_camera_bearing_offset,
    calculate_ball_bearing_deg,
    get_distance_calibration_resolution,
    load_distance_calibration,
    predict_distance_from_calibration,
)
from lib.hailo_ball import HailoBallDetector

MODEL_DIR = Path(__file__).resolve().parents[1] / "open-soccer-detect-n_hailo_model"
BALL_CONFIDENCE = 0.25
PRINT_EVERY = 30
DEFAULT_RESOLUTION = (640, 640)


class LatestFrameCamera:
    """Background capture that always keeps only the newest frame (drops backlog)."""

    def __init__(self, picam2):
        self._picam2 = picam2
        self._lock = threading.Lock()
        self._buf: np.ndarray | None = None
        self._seq = 0
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, name="latest-frame", daemon=True)
        self.captures = 0

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)

    def _loop(self) -> None:
        while not self._stop.is_set():
            frame = self._picam2.capture_array()
            with self._lock:
                if self._buf is None or self._buf.shape != frame.shape:
                    self._buf = np.empty(frame.shape, dtype=frame.dtype)
                np.copyto(self._buf, frame)
                self._seq += 1
                self.captures += 1

    def get_latest(self, out: np.ndarray | None = None) -> tuple[np.ndarray | None, int]:
        """Copy the newest frame into ``out`` (or a new array). Returns (frame, seq)."""
        with self._lock:
            if self._buf is None or self._seq == 0:
                return None, -1
            if out is None or out.shape != self._buf.shape or out.dtype != self._buf.dtype:
                out = np.empty_like(self._buf)
            np.copyto(out, self._buf)
            return out, self._seq


def _det_from_xyxy(det: dict, frame_width: int, frame_height: int) -> dict:
    x1, y1, x2, y2 = det["xyxy"]
    centre_x = (x1 + x2) / 2.0
    centre_y = (y1 + y2) / 2.0
    return {
        "bbox": (
            round(x1),
            round(y1),
            round(x2 - x1),
            round(y2 - y1),
        ),
        "centre": (centre_x, centre_y),
        "radial_pixels": float(
            np.hypot(centre_x - frame_width / 2.0, centre_y - frame_height / 2.0)
        ),
        "confidence": float(det["confidence"]),
        "class_id": int(det["class_id"]),
        "polygon": None,
    }


def _best_named(dets: list[dict], names: dict, label: str) -> dict | None:
    label_l = label.strip().lower()
    class_ids = {
        int(cid) for cid, name in names.items() if str(name).strip().lower() == label_l
    }
    matches = [d for d in dets if int(d["class_id"]) in class_ids]
    if not matches:
        return None
    return max(matches, key=lambda d: d["confidence"])


def _print_detection(detection, frame_width, frame_height, distance_calibration, prefix=""):
    if detection is None:
        print(f"{prefix}ball: not found")
        return

    centre_x, centre_y = detection["centre"]
    angle_deg = apply_camera_bearing_offset(
        calculate_ball_bearing_deg(centre_x, centre_y, frame_width, frame_height)
    )
    distance_mm = predict_distance_from_calibration(
        distance_calibration,
        detection["radial_pixels"],
    )
    dist_txt = "n/a" if distance_mm is None else f"{distance_mm:.0f} mm"
    print(
        f"{prefix}ball: angle={angle_deg:.1f} deg  distance={dist_txt}  "
        f"conf={detection['confidence']:.3f}  bbox={detection['bbox']}  "
        f"centre=({centre_x:.1f}, {centre_y:.1f})"
    )


def _print_bot(detection, prefix=""):
    if detection is None:
        print(f"{prefix}bot: not found")
        return
    centre_x, centre_y = detection["centre"]
    x, y, w, h = detection["bbox"]
    print(
        f"{prefix}bot: centre=({centre_x:.1f}, {centre_y:.1f})  "
        f"bbox=({x}, {y}, {w}, {h})  conf={detection['confidence']:.3f}"
    )


def run_image(image_path: Path, conf: float, debug: bool = False) -> None:
    bgr = cv2.imread(str(image_path))
    if bgr is None:
        raise SystemExit(f"Could not read image: {image_path}")
    frame = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    frame_height, frame_width = frame.shape[:2]

    resolution = (frame_width, frame_height)
    distance_calibration = load_distance_calibration(
        get_distance_calibration_resolution(DEFAULT_DISTANCE_CALIBRATION_FILE) or resolution,
        DEFAULT_DISTANCE_CALIBRATION_FILE,
    )

    with HailoBallDetector(MODEL_DIR, conf=conf) as detector:
        if debug:
            stats = detector.debug_scores(frame)
            print(
                f"debug: dtype={stats['dtype']} shapes={stats['shape']} "
                f"merged={stats.get('merged_shape')} "
                f"outputs={stats.get('output_names')} "
                f"qp={stats['out_qp']} pad={stats['pad']} "
                f"lb_scale={stats['letterbox_scale']:.4f}"
            )
            for class_id, info in stats["classes"].items():
                print(
                    f"  class {class_id} {info['name']}: "
                    f"max={info['max']:.4f} mean={info['mean']:.4f} "
                    f"box@max={info['box']}"
                )

        t0 = time.perf_counter()
        dets = detector.predict(frame)
        elapsed_ms = (time.perf_counter() - t0) * 1000.0
        names = detector.names_map

    ball_xyxy = _best_named(dets, names, "Ball")
    bot_xyxy = _best_named(dets, names, "Bot")
    ball = (
        _det_from_xyxy(ball_xyxy, frame_width, frame_height) if ball_xyxy is not None else None
    )
    bot = _det_from_xyxy(bot_xyxy, frame_width, frame_height) if bot_xyxy is not None else None

    prefix = f"{elapsed_ms:.1f}ms  "
    _print_detection(
        ball,
        frame_width,
        frame_height,
        distance_calibration,
        prefix=prefix,
    )
    _print_bot(bot, prefix=prefix)

    if ball is not None:
        x, y, w, h = ball["bbox"]
        cv2.rectangle(bgr, (x, y), (x + w, y + h), (0, 165, 255), 2)
        cx, cy = ball["centre"]
        cv2.circle(bgr, (int(cx), int(cy)), 4, (0, 255, 0), -1)
    if bot is not None:
        x, y, w, h = bot["bbox"]
        cv2.rectangle(bgr, (x, y), (x + w, y + h), (255, 0, 0), 2)
        cx, cy = bot["centre"]
        cv2.circle(bgr, (int(cx), int(cy)), 4, (255, 0, 0), -1)
        cv2.putText(
            bgr,
            "bot",
            (x, max(0, y - 6)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 0, 0),
            2,
            cv2.LINE_AA,
        )
    out_path = image_path.with_name(f"{image_path.stem}_hailo{image_path.suffix}")
    cv2.imwrite(str(out_path), bgr)
    print(f"wrote {out_path}")


def run_camera(conf: float) -> None:
    from picamera2 import Picamera2

    calib_res = get_distance_calibration_resolution(DEFAULT_DISTANCE_CALIBRATION_FILE)
    resolution = calib_res or DEFAULT_RESOLUTION
    distance_calibration = load_distance_calibration(
        resolution,
        DEFAULT_DISTANCE_CALIBRATION_FILE,
    )

    detector = HailoBallDetector(MODEL_DIR, conf=conf)
    if calib_res is None:
        resolution = (detector.imgsz, detector.imgsz)

    picam2 = Picamera2()
    picam2.preview_configuration.main.size = resolution
    picam2.preview_configuration.main.format = "RGB888"
    picam2.preview_configuration.controls = {"FrameRate": 90}
    picam2.preview_configuration.buffer_count = 4
    picam2.preview_configuration.align()
    picam2.configure("preview")
    picam2.start()

    latest = LatestFrameCamera(picam2)
    latest.start()

    # Wait for first frame.
    infer_frame: np.ndarray | None = None
    for _ in range(200):
        infer_frame, seq = latest.get_latest(infer_frame)
        if seq >= 0:
            break
        time.sleep(0.005)
    if infer_frame is None:
        latest.stop()
        detector.close()
        picam2.stop()
        raise SystemExit("No camera frames received")

    print(
        f"Camera {resolution[0]}x{resolution[1]}, model imgsz={detector.imgsz}, "
        f"latest-only capture (drop backlog)"
    )

    prev_time = time.perf_counter()
    frame_i = 0
    last_seq = -1
    sum_grab_ms = 0.0
    sum_infer_ms = 0.0
    sum_skipped = 0
    processed = 0

    try:
        while True:
            t0 = time.perf_counter()
            # Spin until a newer frame than last processed is available.
            while True:
                infer_frame, seq = latest.get_latest(infer_frame)
                if seq < 0:
                    time.sleep(0.0005)
                    continue
                if seq != last_seq:
                    break
                time.sleep(0.0002)
            # Frames between last_seq and seq were never inferred (dropped as stale).
            skipped = max(0, seq - last_seq - 1) if last_seq >= 0 else 0
            last_seq = seq
            t1 = time.perf_counter()

            frame_height, frame_width = infer_frame.shape[:2]
            rgb_frame = cv2.cvtColor(infer_frame, cv2.COLOR_BGR2RGB)
            detections = detector.predict(rgb_frame)
            ball_xyxy = _best_named(detections, detector.names_map, "Ball")
            detection = (
                _det_from_xyxy(ball_xyxy, frame_width, frame_height)
                if ball_xyxy is not None
                else None
            )
            t2 = time.perf_counter()

            sum_grab_ms += (t1 - t0) * 1000.0
            sum_infer_ms += (t2 - t1) * 1000.0
            sum_skipped += skipped
            frame_i += 1
            processed += 1

            now = time.perf_counter()
            fps = 1.0 / max(now - prev_time, 1e-9)
            prev_time = now

            if frame_i % PRINT_EVERY != 0:
                continue

            avg_grab = sum_grab_ms / PRINT_EVERY
            avg_infer = sum_infer_ms / PRINT_EVERY
            avg_skip = sum_skipped / PRINT_EVERY
            sum_grab_ms = 0.0
            sum_infer_ms = 0.0
            sum_skipped = 0
            prefix = (
                f"{fps:.1f} FPS  grab={avg_grab:.1f}ms  "
                f"infer+post={avg_infer:.1f}ms  "
                f"skip={avg_skip:.1f}/loop  "
                f"cap={latest.captures} proc={processed}  "
            )
            _print_detection(
                detection,
                frame_width,
                frame_height,
                distance_calibration,
                prefix=prefix,
            )
    finally:
        latest.stop()
        detector.close()
        picam2.stop()


def main() -> None:
    parser = argparse.ArgumentParser(description="Run Hailo ball detector on camera or an image.")
    parser.add_argument(
        "--image",
        type=Path,
        default=None,
        help="Run once on this image file (RGB via OpenCV) instead of the camera",
    )
    parser.add_argument("--conf", type=float, default=BALL_CONFIDENCE)
    parser.add_argument(
        "--debug",
        action="store_true",
        help="With --image, print per-class max scores / output shape",
    )
    args = parser.parse_args()

    if args.image is not None:
        run_image(args.image.resolve(), conf=args.conf, debug=args.debug)
    else:
        run_camera(conf=args.conf)


if __name__ == "__main__":
    main()
