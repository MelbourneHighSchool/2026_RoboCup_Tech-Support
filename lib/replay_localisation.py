"""Compare deskew modes on one raw capture, without hardware.

Run: python -m lib.replay_localisation capture.jsonl --output comparison
CSV residuals compare rays at the same estimated pose; they are not ground truth.
"""

import argparse
import csv
import json
import math
from collections import Counter, deque
from pathlib import Path


def write_scan_svg(path, points, mode, scan_time):
    """Two labelled panels of hit endpoints in the scan-midpoint robot frame."""
    parts = ['<svg xmlns="http://www.w3.org/2000/svg" width="940" height="530" viewBox="0 0 940 530">',
             '<rect width="940" height="530" fill="white"/>',
             f'<text x="25" y="25" font-family="sans-serif" font-size="16">Scan {scan_time:.6f} s · {mode} compensation · hit endpoints</text>']
    for panel, title in enumerate(("Raw", "Compensated")):
        left = 30 + panel * 465
        parts.append(f'<text x="{left}" y="55" font-family="sans-serif">{title}</text>')
        for value in (-3000, -1500, 0, 1500, 3000):
            pixel = (value + 3000) / 6000 * 380
            parts.extend([
                f'<path d="M {left+pixel} 70 V 450 M {left} {70+pixel} H {left+380}" stroke="#ddd" fill="none"/>',
                f'<text x="{left+pixel}" y="470" text-anchor="middle" font-size="10">{value}</text>',
            ])
        for point in points:
            if not point[4]:
                continue
            x, y = point[panel*2:panel*2+2]
            if abs(x) <= 3000 and abs(y) <= 3000:
                parts.append(f'<circle cx="{left+(x+3000)/6000*380:.2f}" cy="{70+(y+3000)/6000*380:.2f}" r="1.5" fill="#145da0"/>')
        parts.extend([
            f'<text x="{left+150}" y="490" font-family="sans-serif" font-size="12">Forward x (mm)</text>',
            f'<text x="{left}" y="510" font-family="sans-serif" font-size="12">Right y: −3000 mm (top) to +3000 mm (bottom)</text>',
        ])
    parts.append('</svg>')
    path.write_text("\n".join(parts), encoding="utf-8")


def replay(path, output, mode, *, plot_every=50):
    from lib import lidar

    pending = deque()
    counts = Counter()
    residuals = []
    last_pose = None
    processed = 0
    capture_complete = False
    dropped_scans = dropped_events = 0
    writer_error = None
    clock = 0.0
    fields = ["scan_time_s", "x", "y", "yaw", "confidence", "valid", "pose_jump_mm",
              "reason", "accepted", "history_ok", "duration_s", "age_s", "processing_ms",
              "max_translation_mm", "max_rotation_deg", "raw_residual_mm", "corrected_residual_mm"]
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    try:
        with Path(path).open(encoding="utf-8") as stream, (output / f"{mode}.csv").open("x", newline="") as csv_file:
            header = json.loads(next(stream))
            if header.get("type") != "header" or header.get("version") != 1:
                raise ValueError("Unsupported localisation capture")
            lidar.seed(header["seed"])
            lidar.test_mcl_start(*header["pitch"], use_pcb=header["use_pcb"])
            lidar.configure_deskew(mode, *header["mount"])
            lidar.set_motion_noise(header["motion_noise"])
            writer = csv.DictWriter(csv_file, fieldnames=fields)
            writer.writeheader()

            def process_pending():
                nonlocal last_pose, processed
                while pending:
                    scan = pending[0]
                    if scan["time_s"] <= 0:
                        counts["invalid_scan_time"] += 1
                        pending.popleft()
                        continue
                    lidar.test_mcl_update_scan(scan["points"], scan["time_s"])
                    status = lidar.get_deskew_status()
                    if status["reason"] == "waiting_motion":
                        break
                    pending.popleft()
                    processed += 1
                    counts[status["reason"]] += 1
                    x, y, yaw, confidence, valid = lidar.get_coordinates_info()
                    jump = math.hypot(x-last_pose[0], y-last_pose[1]) if valid and last_pose else None
                    if valid:
                        last_pose = (x, y)
                    row = {key: status[key] for key in fields if key in status}
                    row.update(x=x, y=y, yaw=yaw, confidence=confidence, valid=valid, pose_jump_mm=jump)
                    writer.writerow(row)
                    if status["corrected_residual_mm"] >= 0:
                        residuals.append(status["corrected_residual_mm"])
                    if plot_every and processed % plot_every == 0 and status["history_ok"]:
                        points = lidar.preview_scan(scan["points"], scan["time_s"])
                        if points:
                            write_scan_svg(output / f"{mode}-{processed:05}.svg", points, mode, scan["time_s"])

            for line in stream:
                event = json.loads(line)
                clock = max(clock, event.get("recorded_s", clock))
                lidar.set_replay_time(clock)
                kind = event["type"]
                if kind == "motion":
                    lidar.feed_motion(event["sample"])
                elif kind == "scan":
                    pending.append(event)
                elif kind == "floor":
                    sample = event["sample"]
                    lidar.set_line_readings(sample["colours"], sample["timestamp_s"])
                elif kind in ("health", "footer"):
                    dropped_scans = max(dropped_scans, event.get("dropped_scans", 0))
                    dropped_events = max(dropped_events, event.get("dropped_events", 0))
                    writer_error = event.get("writer_error") or writer_error
                    capture_complete |= kind == "footer"
                process_pending()
            lidar.set_replay_time(clock + 0.5)
            process_pending()
    finally:
        lidar.test_mcl_stop()
        lidar.set_replay_time(-1)
        lidar.configure_deskew("off")
    residuals.sort()
    return {"mode": mode, "scans": processed, "reasons": dict(counts),
            "mean_wall_residual_mm": sum(residuals)/len(residuals) if residuals else None,
            "p95_wall_residual_mm": residuals[math.ceil(0.95*len(residuals))-1] if residuals else None,
            "capture_complete": capture_complete, "dropped_scans": dropped_scans,
            "dropped_events": dropped_events, "writer_error": writer_error,
            "note": "Residuals use estimated pose, not independent ground truth. Pose jumps include actual motion."}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--mode", choices=("off", "rotation", "full", "all"), default="all")
    parser.add_argument("--plot-every", type=int, default=50, help="Save scan overlays every N scans; 0 disables")
    args = parser.parse_args()
    if args.plot_every < 0:
        parser.error("--plot-every must be nonnegative")
    modes = ("off", "rotation", "full") if args.mode == "all" else (args.mode,)
    summaries = [replay(args.capture, args.output, mode, plot_every=args.plot_every) for mode in modes]
    with (args.output / "summary.json").open("x", encoding="utf-8") as stream:
        json.dump(summaries, stream, indent=2)
    for result in summaries:
        print(f"{result['mode']}: scans={result['scans']}, reasons={result['reasons']}, "
              f"mean residual={result['mean_wall_residual_mm']} mm")
        if not result["capture_complete"] or result["dropped_scans"] or result["dropped_events"] or result["writer_error"]:
            print("  Capture incomplete or contains drops/errors; inspect summary.json before comparing.")


if __name__ == "__main__":
    main()
