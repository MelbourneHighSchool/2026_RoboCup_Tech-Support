"""Hailo-8 (AI HAT+) YOLO26 detect inference for ball bearing/distance."""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import yaml

BALL_CLASS_NAME = "ball"
BOT_CLASS_NAME = "bot"
_PROJECT_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_HAIL_MODEL_DIR = _PROJECT_ROOT / "open-soccer-detect-n_hailo_model"
DEFAULT_CLASS_NAMES = {
    0: "Ball",
    1: "Bot",
}


def letterbox(
    image: np.ndarray,
    imgsz: int = 640,
    pad_value: int = 114,
    canvas: np.ndarray | None = None,
) -> tuple[np.ndarray, float, tuple[int, int]]:
    """Resize with aspect ratio preserved and gray pad to imgsz×imgsz (NHWC RGB)."""
    import cv2

    height, width = image.shape[:2]
    scale = min(imgsz / height, imgsz / width)
    new_w = round(width * scale)
    new_h = round(height * scale)
    resized = cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
    if (
        canvas is None
        or canvas.shape != (imgsz, imgsz, 3)
        or canvas.dtype != np.uint8
    ):
        canvas = np.full((imgsz, imgsz, 3), pad_value, dtype=np.uint8)
    else:
        canvas.fill(pad_value)
    top = (imgsz - new_h) // 2
    left = (imgsz - new_w) // 2
    canvas[top : top + new_h, left : left + new_w] = resized
    return canvas, scale, (left, top)


def _load_metadata(model_dir: Path) -> dict:
    meta_path = model_dir / "metadata.yaml"
    defaults = {"names": DEFAULT_CLASS_NAMES, "imgsz": 640, "task": "detect"}
    if not meta_path.is_file():
        return defaults
    # Some deployment/copy workflows leave the YAML document serialized as a
    # quoted string. Decode that once more, and treat other scalar documents as
    # absent metadata instead of failing during camera startup.
    try:
        with meta_path.open(encoding="utf-8") as handle:
            meta = yaml.safe_load(handle) or {}
        if isinstance(meta, str):
            meta = yaml.safe_load(meta) or {}
    except (OSError, yaml.YAMLError):
        return defaults
    if not isinstance(meta, dict):
        return defaults
    names = meta.get("names") or DEFAULT_CLASS_NAMES
    if isinstance(names, list):
        names = {i: name for i, name in enumerate(names)}
    elif isinstance(names, dict):
        names = {int(k): v for k, v in names.items()}
    meta["names"] = names
    return meta


def _quant_params(vstream_info) -> tuple[float, float]:
    """Return (scale, zero_point) from a Hailo vstream info object."""
    qi = getattr(vstream_info, "quant_info", None)
    if qi is None:
        return 1.0, 0.0
    scale = getattr(qi, "qp_scale", None)
    if scale is None:
        scale = getattr(qi, "scale", 1.0)
    zp = getattr(qi, "qp_zp", None)
    if zp is None:
        zp = getattr(qi, "zero_point", 0.0)
    return float(scale), float(zp)


def _as_channels_first(output: np.ndarray, *, min_channels: int = 4) -> np.ndarray:
    """Normalize Hailo/ONNX YOLO26 tensors to (C, N).

    Observed layouts:
    - (1, 1, C, N) Hailo NHWC-ish H=1
    - (1, C, N) / (C, N)
    - (1, N, C) / (N, C)
    """
    output = np.asarray(output)
    output = np.squeeze(output)
    if output.ndim == 3:
        if output.shape[0] == 1:
            output = output[0]
        elif output.shape[1] == 1:
            output = output[:, 0, :]
        elif output.shape[2] == 1:
            output = output[:, :, 0]
        else:
            raise RuntimeError(f"Unexpected Hailo output shape {output.shape}")

    if output.ndim != 2:
        raise RuntimeError(
            f"Unexpected Hailo output shape {output.shape}; expected (C, N) "
            "or (N, C) YOLO26 detections."
        )

    # Prefer channels-first (C, N) with C small and N anchors large.
    if output.shape[0] > output.shape[1] and output.shape[1] <= 16:
        output = output.T
    if output.shape[0] < min_channels:
        raise RuntimeError(
            f"Unexpected Hailo output shape {output.shape}; "
            f"expected at least {min_channels} channels."
        )
    return output


def _merge_box_score_outputs(outputs: list[np.ndarray]) -> np.ndarray:
    """Merge separate box (4,N) and score (nc,N) HEF outputs into (4+nc, N).

    Prefer channel-count identity (boxes always have 4 channels; scores have
    nc, which may be < 4). Fall back to value-range heuristics when nc == 4.
    """
    if len(outputs) == 1:
        return _as_channels_first(outputs[0], min_channels=6)
    if len(outputs) != 2:
        raise RuntimeError(
            f"Expected 1 or 2 Hailo outputs (boxes+scores), got {len(outputs)}"
        )

    # nc can be 2 (Ball/Bot); do not require 4 channels on both streams.
    a = _as_channels_first(outputs[0], min_channels=1)
    b = _as_channels_first(outputs[1], min_channels=1)
    if a.shape[1] != b.shape[1]:
        raise RuntimeError(
            f"Box/score anchor mismatch: {a.shape} vs {b.shape}"
        )

    a_is_boxes = a.shape[0] == 4
    b_is_boxes = b.shape[0] == 4
    if a_is_boxes and not b_is_boxes:
        boxes, scores = a, b
    elif b_is_boxes and not a_is_boxes:
        boxes, scores = b, a
    else:
        # Ambiguous (e.g. nc == 4): score tensor is mostly in [0, 1].
        def _scoreish(t: np.ndarray) -> float:
            sample = t.reshape(-1)
            if sample.size > 4096:
                sample = sample[:: max(1, sample.size // 4096)]
            return float(np.mean((sample >= -0.05) & (sample <= 1.05)))

        if _scoreish(a) >= _scoreish(b):
            scores, boxes = a, b
        else:
            scores, boxes = b, a
        if boxes.shape[0] != 4:
            raise RuntimeError(f"Expected 4 box channels, got {boxes.shape}")
    return np.concatenate([boxes, scores], axis=0)


def _nms_xyxy(
    boxes: np.ndarray,
    scores: np.ndarray,
    iou_thres: float = 0.45,
    max_det: int = 300,
) -> list[int]:
    """NMS for xyxy boxes; prefers OpenCV C++ impl when available."""
    if boxes.size == 0:
        return []
    try:
        import cv2

        idxs = cv2.dnn.NMSBoxes(
            bboxes=boxes.tolist(),
            scores=scores.tolist(),
            score_threshold=0.0,
            nms_threshold=float(iou_thres),
            top_k=int(max_det),
        )
        if idxs is None or len(idxs) == 0:
            return []
        return [int(i) for i in np.asarray(idxs).reshape(-1)]
    except Exception:
        pass

    x1, y1, x2, y2 = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
    areas = np.maximum(0.0, x2 - x1) * np.maximum(0.0, y2 - y1)
    order = scores.argsort()[::-1]
    keep: list[int] = []
    while order.size > 0 and len(keep) < max_det:
        i = int(order[0])
        keep.append(i)
        if order.size == 1:
            break
        rest = order[1:]
        xx1 = np.maximum(x1[i], x1[rest])
        yy1 = np.maximum(y1[i], y1[rest])
        xx2 = np.minimum(x2[i], x2[rest])
        yy2 = np.minimum(y2[i], y2[rest])
        inter = np.maximum(0.0, xx2 - xx1) * np.maximum(0.0, yy2 - yy1)
        iou = inter / (areas[i] + areas[rest] - inter + 1e-9)
        order = rest[iou <= iou_thres]
    return keep


def _dequant(values: np.ndarray, scale: float, zp: float) -> np.ndarray:
    return (values.astype(np.float32, copy=False) - zp) * scale


def _decode_detections(
    output: np.ndarray,
    conf_thres: float,
    iou_thres: float = 0.45,
    max_det: int = 300,
    class_id_filter: int | None = None,
    scale: float = 1.0,
    zp: float = 0.0,
) -> list[tuple[float, float, float, float, float, int]]:
    """Decode to (x1,y1,x2,y2,conf,cls) in letterbox space."""
    output = _as_channels_first(output)
    _channels, num_anchors = output.shape
    if output.dtype != np.float32:
        # Dequant whole map only for full multi-class predict path.
        output = _dequant(output, scale, zp)
    boxes = output[:4]
    class_scores = output[4:]

    if class_id_filter is not None:
        confidences = class_scores[class_id_filter]
        class_ids = np.full(num_anchors, class_id_filter, dtype=np.int32)
    else:
        class_ids = np.argmax(class_scores, axis=0)
        confidences = class_scores[class_ids, np.arange(num_anchors)]

    mask = confidences >= conf_thres
    if not np.any(mask):
        return []
    boxes_t = boxes[:, mask].T
    confidences = confidences[mask]
    class_ids = class_ids[mask]
    keep = _nms_xyxy(boxes_t, confidences, iou_thres=iou_thres, max_det=max_det)
    return [
        (
            float(boxes_t[i, 0]),
            float(boxes_t[i, 1]),
            float(boxes_t[i, 2]),
            float(boxes_t[i, 3]),
            float(confidences[i]),
            int(class_ids[i]),
        )
        for i in keep
    ]


def _best_class_detection(
    output: np.ndarray,
    class_id: int,
    conf_thres: float,
    scale: float = 1.0,
    zp: float = 0.0,
) -> tuple[float, float, float, float, float, int] | None:
    """Top-1 detection for one class — no NMS.

    For UINT8 outputs, argmax on the quantized ball row (monotonic if scale>0),
    then dequant only the winning box + score.
    """
    output = _as_channels_first(output)
    if output.shape[0] < 4 + class_id + 1:
        return None

    scores_q = output[4 + class_id]
    if scale < 0:
        idx = int(np.argmin(scores_q))
    else:
        idx = int(np.argmax(scores_q))

    if output.dtype == np.float32:
        conf = float(scores_q[idx])
        x1, y1, x2, y2 = (float(v) for v in output[:4, idx])
    else:
        conf = (float(scores_q[idx]) - zp) * scale
        box = _dequant(output[:4, idx], scale, zp)
        x1, y1, x2, y2 = (float(v) for v in box)

    if conf < conf_thres:
        return None
    return x1, y1, x2, y2, conf, class_id


class HailoBallDetector:
    """Run a compiled YOLO26 detect HEF on HailoRT and return ball candidates."""

    def __init__(
        self,
        model_dir: str | Path = DEFAULT_HAIL_MODEL_DIR,
        conf: float = 0.25,
        iou: float = 0.45,
    ):
        self.model_dir = Path(model_dir)
        self.hef_path = self.model_dir / "model.hef"
        if not self.hef_path.is_file():
            raise FileNotFoundError(
                f"Hailo HEF not found: {self.hef_path}\n"
                "Train/export ONNX with training/train.py, then compile with "
                "training/compile_hailo.py on an x86 host with the Hailo DFC, and copy "
                f"{self.model_dir.name}/ to this machine."
            )

        try:
            from hailo_platform import (
                HEF,
                ConfigureParams,
                FormatType,
                HailoStreamInterface,
                InferVStreams,
                InputVStreamParams,
                OutputVStreamParams,
                VDevice,
            )
        except ImportError as exc:
            raise ImportError(
                "hailo_platform is not importable. On the Pi install hailo-all and use a "
                "venv created with --system-site-packages."
            ) from exc

        self.conf = float(conf)
        self.iou = float(iou)
        self.meta = _load_metadata(self.model_dir)
        self.names = self.meta.get("names") or DEFAULT_CLASS_NAMES
        imgsz = self.meta.get("imgsz") or 640
        if isinstance(imgsz, (list, tuple)):
            self.imgsz = int(imgsz[0])
        else:
            self.imgsz = int(imgsz)

        self._ball_class_id = 0
        self._bot_class_id = 1
        for class_id, name in self.names.items():
            name_l = str(name).strip().lower()
            if name_l == BALL_CLASS_NAME:
                self._ball_class_id = int(class_id)
            elif name_l == BOT_CLASS_NAME:
                self._bot_class_id = int(class_id)

        self._letterbox_canvas: np.ndarray | None = None
        self._input_batch: np.ndarray | None = None

        self._hef = HEF(str(self.hef_path))
        self._target = VDevice()
        configure_params = ConfigureParams.create_from_hef(
            hef=self._hef,
            interface=HailoStreamInterface.PCIe,
        )
        self._network_group = self._target.configure(self._hef, configure_params)[0]
        self._network_group_params = self._network_group.create_params()

        self._input_vstream_info = self._hef.get_input_vstream_infos()[0]
        self._output_vstream_infos = list(self._hef.get_output_vstream_infos())
        self._input_name = self._input_vstream_info.name
        self._output_names = [info.name for info in self._output_vstream_infos]
        self._out_qps = [_quant_params(info) for info in self._output_vstream_infos]
        # Legacy single-output HEFs: keep first stream qp for UINT8 fallback.
        self._out_scale, self._out_zp = self._out_qps[0]

        self._input_vstreams_params = InputVStreamParams.make_from_network_group(
            self._network_group,
            quantized=False,
            format_type=FormatType.UINT8,
        )
        # FLOAT32: HailoRT dequants on the host. UINT8 + manual qp drops all
        # detections when quant_info is wrong/missing (scores never pass conf).
        self._output_vstreams_params = OutputVStreamParams.make_from_network_group(
            self._network_group,
            quantized=False,
            format_type=FormatType.FLOAT32,
        )
        if len(self._output_names) == 1:
            import warnings

            warnings.warn(
                "HEF has a single detection output (boxes+scores concatenated). "
                "INT8 quantization usually crushes sigmoid scores to ~0 — recompile "
                "with training/compile_hailo.py (end nodes Mul_2 + Sigmoid).",
                stacklevel=2,
            )

        self._infer = InferVStreams(
            self._network_group,
            self._input_vstreams_params,
            self._output_vstreams_params,
        )
        self._activated = self._network_group.activate(self._network_group_params)
        self._activated.__enter__()
        self._infer.__enter__()
        self._closed = False

    @property
    def names_map(self):
        return self.names

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        try:
            self._infer.__exit__(None, None, None)
        except Exception:
            pass
        try:
            self._activated.__exit__(None, None, None)
        except Exception:
            pass
        target = getattr(self, "_target", None)
        if target is None:
            return
        if hasattr(target, "release"):
            try:
                target.release()
            except Exception:
                pass
        elif hasattr(target, "__exit__"):
            try:
                target.__exit__(None, None, None)
            except Exception:
                pass

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()
        return False

    def _is_ball_class(self, class_id: int) -> bool:
        return int(class_id) == self._ball_class_id

    def _is_bot_class(self, class_id: int) -> bool:
        return int(class_id) == self._bot_class_id

    @property
    def ball_class_id(self) -> int:
        return self._ball_class_id

    @property
    def bot_class_id(self) -> int:
        return self._bot_class_id

    def _preprocess(self, frame_rgb: np.ndarray) -> tuple[np.ndarray, float, tuple[int, int]]:
        height, width = frame_rgb.shape[:2]
        if height == self.imgsz and width == self.imgsz:
            # Already model-sized: skip letterbox resize/pad.
            if (
                self._input_batch is None
                or self._input_batch.shape != (1, height, width, 3)
            ):
                self._input_batch = np.empty((1, height, width, 3), dtype=np.uint8)
            np.copyto(self._input_batch[0], frame_rgb)
            return self._input_batch, 1.0, (0, 0)

        padded, scale, pad = letterbox(
            frame_rgb,
            self.imgsz,
            canvas=self._letterbox_canvas,
        )
        self._letterbox_canvas = padded
        if self._input_batch is None or self._input_batch.shape[1:] != padded.shape:
            self._input_batch = np.empty((1, *padded.shape), dtype=np.uint8)
        np.copyto(self._input_batch[0], padded)
        return self._input_batch, scale, pad

    def _infer_raw(self, frame_rgb: np.ndarray) -> tuple[np.ndarray, float, tuple[int, int]]:
        if self._closed:
            raise RuntimeError("HailoBallDetector is closed")
        input_batch, scale, pad = self._preprocess(frame_rgb)
        raw = self._infer.infer({self._input_name: input_batch})
        outputs = [np.asarray(raw[name]) for name in self._output_names]
        merged = _merge_box_score_outputs(outputs)
        return merged, scale, pad

    def predict(self, frame_rgb: np.ndarray) -> list[dict]:
        """Return detections as dicts with xyxy (original frame), conf, cls."""
        output, scale, (pad_left, pad_top) = self._infer_raw(frame_rgb)
        detections: list[dict] = []
        frame_h, frame_w = frame_rgb.shape[:2]
        for x1, y1, x2, y2, conf, class_id in _decode_detections(
            output,
            conf_thres=self.conf,
            iou_thres=self.iou,
            scale=self._out_scale,
            zp=self._out_zp,
        ):
            x1 = (x1 - pad_left) / scale
            y1 = (y1 - pad_top) / scale
            x2 = (x2 - pad_left) / scale
            y2 = (y2 - pad_top) / scale
            x1 = min(max(x1, 0.0), frame_w - 1.0)
            y1 = min(max(y1, 0.0), frame_h - 1.0)
            x2 = min(max(x2, 0.0), frame_w - 1.0)
            y2 = min(max(y2, 0.0), frame_h - 1.0)
            if x2 <= x1 or y2 <= y1:
                continue
            detections.append(
                {
                    "xyxy": (x1, y1, x2, y2),
                    "confidence": conf,
                    "class_id": class_id,
                }
            )
        return detections

    def debug_scores(self, frame_rgb: np.ndarray) -> dict:
        """Return raw output stats for diagnosing empty detections."""
        if self._closed:
            raise RuntimeError("HailoBallDetector is closed")
        input_batch, scale, pad = self._preprocess(frame_rgb)
        raw = self._infer.infer({self._input_name: input_batch})
        outputs = [np.asarray(raw[name]) for name in self._output_names]
        cf = _merge_box_score_outputs(outputs)
        if cf.dtype != np.float32:
            cf = _dequant(cf, self._out_scale, self._out_zp)
        num_classes = cf.shape[0] - 4
        per_class = {}
        for class_id in range(num_classes):
            scores = cf[4 + class_id]
            idx = int(np.argmax(scores))
            per_class[class_id] = {
                "name": self.names.get(class_id, str(class_id)),
                "max": float(scores[idx]),
                "mean": float(np.mean(scores)),
                "box": [float(v) for v in cf[:4, idx]],
            }
        return {
            "dtype": str(cf.dtype),
            "shape": [list(o.shape) for o in outputs],
            "merged_shape": list(cf.shape),
            "output_names": list(self._output_names),
            "letterbox_scale": scale,
            "pad": pad,
            "out_qp": [
                {"name": name, "scale": qp[0], "zp": qp[1]}
                for name, qp in zip(self._output_names, self._out_qps)
            ],
            "classes": per_class,
        }

    def best_ball(self, frame_rgb: np.ndarray) -> dict | None:
        """Highest-confidence ball detection with centre / radial_pixels."""
        output, scale, (pad_left, pad_top) = self._infer_raw(frame_rgb)
        det = _best_class_detection(
            output,
            self._ball_class_id,
            self.conf,
            scale=self._out_scale,
            zp=self._out_zp,
        )
        if det is None:
            return None

        x1, y1, x2, y2, conf, _class_id = det
        frame_h, frame_w = frame_rgb.shape[:2]
        x1 = (x1 - pad_left) / scale
        y1 = (y1 - pad_top) / scale
        x2 = (x2 - pad_left) / scale
        y2 = (y2 - pad_top) / scale
        x1 = min(max(x1, 0.0), frame_w - 1.0)
        y1 = min(max(y1, 0.0), frame_h - 1.0)
        x2 = min(max(x2, 0.0), frame_w - 1.0)
        y2 = min(max(y2, 0.0), frame_h - 1.0)
        if x2 <= x1 or y2 <= y1:
            return None

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
            "radial_pixels": math.hypot(
                centre_x - (frame_w / 2.0),
                centre_y - (frame_h / 2.0),
            ),
            "confidence": conf,
            "polygon": None,
        }
