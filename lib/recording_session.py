"""Low-overhead, timestamped recording-session metadata writers."""

from __future__ import annotations

import csv
import json
import os
import queue
import threading
import time
from collections.abc import Iterable, Mapping
from pathlib import Path

SESSION_SCHEMA_VERSION = 1
GAME_FIELDS = (
    "elapsed_s",
    "x",
    "y",
    "yaw",
    "ball_x",
    "ball_y",
    "ball_captured",
    "bot_mode",
    "state",
    "direction",
    "speed",
    "rotation",
    "kick",
    "dribbler",
    "other_bots",
)
DETECTION_FIELDS = (
    "elapsed_s",
    "video_time_s",
    "capture_sequence",
    "sensor_timestamp_ns",
    "inference_sequence",
    "detected",
    "bbox_x",
    "bbox_y",
    "bbox_w",
    "bbox_h",
    "centre_x",
    "centre_y",
    "confidence",
    "bots",
)


class AsyncCsvWriter:
    """Write small CSV records without blocking the producer threads."""

    def __init__(
        self,
        path: Path,
        fields: Iterable[str],
        queue_size: int = 4096,
        sync_interval_s: float = 1.0,
    ):
        self.path = Path(path)
        self.fields = tuple(fields)
        self._sync_interval_s = sync_interval_s
        self._queue: queue.Queue[tuple[object, ...]] = queue.Queue(maxsize=queue_size)
        self._stop = threading.Event()
        self._dropped = 0
        self._written = 0
        self._counter_lock = threading.Lock()
        self._error: Exception | None = None
        self._thread = threading.Thread(
            target=self._run,
            name=f"csv-writer-{self.path.stem}",
            daemon=True,
        )
        # Create and durably sync the header before returning. A session is
        # therefore structurally replayable even if power is lost immediately.
        with self.path.open("w", encoding="utf-8", newline="") as handle:
            csv.writer(handle).writerow(self.fields)
            handle.flush()
            os.fsync(handle.fileno())
        directory_fd = os.open(self.path.parent, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)
        self._thread.start()

    @property
    def dropped(self) -> int:
        with self._counter_lock:
            return self._dropped

    @property
    def written(self) -> int:
        with self._counter_lock:
            return self._written

    @property
    def error(self) -> Exception | None:
        return self._error

    def submit(self, values: Iterable[object]) -> bool:
        if self._stop.is_set():
            return False
        row = tuple(values)
        if len(row) != len(self.fields):
            raise ValueError(f"Expected {len(self.fields)} values, got {len(row)}")
        try:
            self._queue.put_nowait(row)
            return True
        except queue.Full:
            with self._counter_lock:
                self._dropped += 1
            return False

    def _run(self) -> None:
        try:
            with self.path.open("a", encoding="utf-8", newline="") as handle:
                writer = csv.writer(handle)
                last_sync = time.monotonic()
                while not self._stop.is_set() or not self._queue.empty():
                    try:
                        row = self._queue.get(timeout=0.1)
                    except queue.Empty:
                        row = None
                    if row is not None:
                        writer.writerow(row)
                        with self._counter_lock:
                            self._written += 1
                    now = time.monotonic()
                    if now - last_sync >= self._sync_interval_s:
                        handle.flush()
                        os.fsync(handle.fileno())
                        last_sync = now
                handle.flush()
                os.fsync(handle.fileno())
        except Exception as exc:
            self._error = exc

    def close(self, timeout: float = 3.0) -> None:
        self._stop.set()
        self._thread.join(timeout=timeout)
        if self._thread.is_alive() and self._error is None:
            self._error = TimeoutError(f"Timed out closing {self.path}")


class RecordingSession:
    """Own a video path and asynchronous game/detection metadata files."""

    def __init__(
        self,
        directory: str | Path,
        *,
        resolution: tuple[int, int],
        requested_fps: float,
    ):
        self.directory = Path(directory)
        if self.directory.exists() and any(self.directory.iterdir()):
            raise FileExistsError(f"Recording session directory is not empty: {self.directory}")
        self.directory.mkdir(parents=True, exist_ok=True)
        # MPEG-TS has no end-of-file index to finalize, unlike MP4.  Each
        # packet is usable as it is written, so a sudden power loss preserves
        # the already-flushed beginning of a session recording.
        self.video_path = self.directory / "video.ts"
        self.game_path = self.directory / "game.csv"
        self.detections_path = self.directory / "detections.csv"
        self.metadata_path = self.directory / "metadata.json"
        self.epoch_monotonic = time.monotonic()
        self._metadata_lock = threading.Lock()
        self._metadata = {
            "schema_version": SESSION_SCHEMA_VERSION,
            "created_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "resolution": [int(resolution[0]), int(resolution[1])],
            "requested_fps": float(requested_fps),
            "video_file": self.video_path.name,
            "game_file": self.game_path.name,
            "detections_file": self.detections_path.name,
            "annotation": "top-confidence Ball and all Bot detections; box centres",
            "finalized": False,
        }
        self._write_metadata_locked()
        self.game_writer = AsyncCsvWriter(self.game_path, GAME_FIELDS)
        self.detection_writer = AsyncCsvWriter(
            self.detections_path,
            DETECTION_FIELDS,
            queue_size=8192,
        )

    def elapsed(self) -> float:
        return time.monotonic() - self.epoch_monotonic

    def record_game(
        self, values: Iterable[object], elapsed_s: float | None = None, *, other_bots=()
    ) -> bool:
        if elapsed_s is None:
            elapsed_s = self.elapsed()
        return self.game_writer.submit((elapsed_s, *values, json.dumps(list(other_bots))))

    def record_detection(self, event: Mapping[str, object]) -> bool:
        detection = event.get("detection")
        if isinstance(detection, Mapping):
            bbox = tuple(detection.get("bbox") or (None, None, None, None))
            centre = tuple(detection.get("centre") or (None, None))
            confidence = detection.get("confidence")
            detected = True
        else:
            bbox = (None, None, None, None)
            centre = (None, None)
            confidence = None
            detected = False
        return self.detection_writer.submit(
            (
                event.get("elapsed_s"),
                event.get("video_time_s"),
                event.get("capture_sequence"),
                event.get("sensor_timestamp_ns"),
                event.get("inference_sequence"),
                detected,
                *bbox,
                *centre,
                confidence,
                json.dumps(event.get("bots") or []),
            )
        )

    def update_metadata(self, values: Mapping[str, object]) -> None:
        with self._metadata_lock:
            self._metadata.update(values)
            self._write_metadata_locked()

    def checkpoint(self, values: Mapping[str, object] | None = None) -> None:
        """Durably save enough live state to replay after an abrupt shutdown."""
        with self._metadata_lock:
            if values is not None:
                self._metadata.update(values)
            self._metadata.update(
                {
                    "duration_s": self.elapsed(),
                    "game_rows": self.game_writer.written,
                    "detection_rows": self.detection_writer.written,
                    "dropped_game_rows": self.game_writer.dropped,
                    "dropped_detection_rows": self.detection_writer.dropped,
                }
            )
            self._write_metadata_locked()

    def _write_metadata_locked(self) -> None:
        temporary_path = self.metadata_path.with_suffix(".json.tmp")
        with temporary_path.open("w", encoding="utf-8") as handle:
            json.dump(self._metadata, handle, indent=2, sort_keys=True)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        temporary_path.replace(self.metadata_path)
        directory_fd = os.open(self.directory, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)

    def close(self) -> None:
        self.game_writer.close()
        self.detection_writer.close()
        with self._metadata_lock:
            self._metadata.update(
                {
                    "duration_s": self.elapsed(),
                    "game_rows": self.game_writer.written,
                    "detection_rows": self.detection_writer.written,
                    "dropped_game_rows": self.game_writer.dropped,
                    "dropped_detection_rows": self.detection_writer.dropped,
                    "game_writer_error": (
                        str(self.game_writer.error) if self.game_writer.error else None
                    ),
                    "detection_writer_error": (
                        str(self.detection_writer.error)
                        if self.detection_writer.error
                        else None
                    ),
                    "finalized": True,
                }
            )
            self._write_metadata_locked()
