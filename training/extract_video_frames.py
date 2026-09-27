#!/usr/bin/env python3
"""Extract evenly spaced frames from videos into Label Studio Local Files Photos/.

Each input video becomes one clip with a new monotonic ``output<N>`` prefix so
``export_label_studio.py`` can split train/val by clip. Source video filenames
are not used in the output names. Pi H.264 recordings in ``.ts`` containers
are supported alongside MP4 and the other common video formats.

Example:
  python training/extract_video_frames.py /path/to/videos
  python training/extract_video_frames.py /path/to/videos --fps 0.5
  python training/extract_video_frames.py /path/to/videos --fps 2 --photos-dir label-studio-data/Photos

After writing frames, sync/refresh Label Studio Local Files storage so the new
images appear as tasks.
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

import cv2

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PHOTOS_DIR = REPO_ROOT / "label-studio-data" / "Photos"
VIDEO_SUFFIXES = {".mp4", ".mov", ".avi", ".mkv", ".webm", ".m4v", ".ts"}
_CLIP_FRAME_NAME = re.compile(r"^output(\d*)_(\d+)\.(?:png|jpe?g)$", re.IGNORECASE)


def next_clip_number(photos_dir: Path) -> int:
    """Return the next unused clip number based on existing ``output<N>_*.png`` files."""
    if not photos_dir.is_dir():
        return 1
    highest = 0
    for path in photos_dir.iterdir():
        if not path.is_file():
            continue
        match = _CLIP_FRAME_NAME.match(path.name)
        if not match:
            continue
        raw = match.group(1)
        if raw == "":
            # Legacy bare ``output_000001.png`` — treat as clip 0 for numbering.
            highest = max(highest, 0)
            continue
        highest = max(highest, int(raw))
    return highest + 1


def frame_filename(clip_number: int, frame_index: int) -> str:
    """``output23``, frame 1 → ``output23_000001.png`` (1-based frame index)."""
    if clip_number < 1:
        raise ValueError(f"clip_number must be >= 1, got {clip_number}")
    if frame_index < 1:
        raise ValueError(f"frame_index must be >= 1, got {frame_index}")
    return f"output{clip_number}_{frame_index:06d}.png"


def list_videos(video_dir: Path) -> list[Path]:
    if not video_dir.is_dir():
        raise FileNotFoundError(f"Video folder not found: {video_dir}")
    videos = sorted(
        path
        for path in video_dir.iterdir()
        if path.is_file() and path.suffix.lower() in VIDEO_SUFFIXES
    )
    if not videos:
        raise RuntimeError(f"No videos found in {video_dir}")
    return videos


def sample_frame_indices(frame_count: int, video_fps: float, sample_fps: float) -> list[int]:
    """Frame indices for samples at 0, 1/sample_fps, 2/sample_fps, ... seconds."""
    if sample_fps <= 0:
        raise ValueError(f"fps must be > 0, got {sample_fps}")
    if video_fps <= 0:
        raise ValueError(f"video_fps must be > 0, got {video_fps}")
    if frame_count <= 0:
        return []

    duration_s = frame_count / video_fps
    step_s = 1.0 / sample_fps
    indices: list[int] = []
    seen: set[int] = set()
    t = 0.0
    while t < duration_s - 1e-9:
        index = min(frame_count - 1, round(t * video_fps))
        if index not in seen:
            seen.add(index)
            indices.append(index)
        t += step_s
    return indices


def extract_frames_from_video(
    video_path: Path,
    dest_dir: Path,
    clip_number: int,
    fps: float,
) -> int:
    """Write sampled frames for one video. Returns the number of frames written."""
    capture = cv2.VideoCapture(str(video_path))
    if not capture.isOpened():
        raise RuntimeError(f"Could not open video: {video_path}")

    try:
        video_fps = float(capture.get(cv2.CAP_PROP_FPS) or 0.0)
        frame_count = int(capture.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
        if video_fps <= 0:
            raise RuntimeError(f"Video has no FPS metadata: {video_path}")
        if frame_count <= 0:
            raise RuntimeError(f"Video has no frames: {video_path}")

        # Sequential decode: seeking by msec is unreliable across codecs.
        target_indices = set(sample_frame_indices(frame_count, video_fps, fps))
        if not target_indices:
            return 0

        written = 0
        frame_i = 0
        last_target = max(target_indices)
        while True:
            ok, frame = capture.read()
            if not ok:
                break
            if frame_i in target_indices:
                written += 1
                dest = dest_dir / frame_filename(clip_number, written)
                if dest.exists():
                    raise FileExistsError(f"Refusing to overwrite existing frame: {dest}")
                if not cv2.imwrite(str(dest), frame):
                    raise RuntimeError(f"Failed to write frame: {dest}")
            frame_i += 1
            if frame_i > last_target:
                break
        return written
    finally:
        capture.release()


def extract_folder(
    video_dir: Path,
    photos_dir: Path,
    fps: float,
) -> list[tuple[Path, int, int]]:
    """Extract every video in ``video_dir`` into ``photos_dir``.

    Returns a list of ``(video_path, clip_number, frames_written)``.
    """
    if fps <= 0:
        raise ValueError(f"fps must be > 0, got {fps}")

    videos = list_videos(video_dir)
    photos_dir.mkdir(parents=True, exist_ok=True)
    next_clip = next_clip_number(photos_dir)

    results: list[tuple[Path, int, int]] = []
    for offset, video_path in enumerate(videos):
        clip_number = next_clip + offset
        print(f"{video_path.name} → clip output{clip_number} @ {fps:g} fps …", flush=True)
        try:
            written = extract_frames_from_video(video_path, photos_dir, clip_number, fps)
        except Exception as exc:
            print(f"  ERROR: {exc}", file=sys.stderr, flush=True)
            results.append((video_path, clip_number, 0))
            continue
        print(f"  wrote {written} frame(s)", flush=True)
        results.append((video_path, clip_number, written))
    return results


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Extract frames from a folder of videos into Label Studio Photos/ "
            "as output<N>_######.png clips."
        )
    )
    parser.add_argument(
        "video_dir",
        type=Path,
        help="Folder containing source videos",
    )
    parser.add_argument(
        "--fps",
        type=float,
        default=1.0,
        help="Frames to extract per second of video (default: 1)",
    )
    parser.add_argument(
        "--photos-dir",
        type=Path,
        default=DEFAULT_PHOTOS_DIR,
        help="Label Studio Local Files Photos directory",
    )
    args = parser.parse_args()

    if args.fps <= 0:
        raise SystemExit("--fps must be > 0")

    results = extract_folder(
        video_dir=args.video_dir.resolve(),
        photos_dir=args.photos_dir.resolve(),
        fps=args.fps,
    )
    total = sum(written for _path, _clip, written in results)
    failed = [path for path, _clip, written in results if written == 0]
    print(
        f"Done: {total} frame(s) from {len(results)} video(s) → {args.photos_dir.resolve()}"
    )
    if failed:
        names = ", ".join(path.name for path in failed)
        raise SystemExit(f"No frames written for: {names}")
    print(
        "Sync/refresh Label Studio Local Files storage so the new images appear as tasks."
    )


if __name__ == "__main__":
    main()
