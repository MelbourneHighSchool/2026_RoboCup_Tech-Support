"""Summarise phase-aligned localisation capture and replay diagnostics."""

import argparse
import csv
import json
import math
import statistics
from collections import defaultdict
from pathlib import Path


def _percentile(values, fraction):
    ordered = sorted(values)
    if not ordered:
        return None
    return ordered[max(0, math.ceil(fraction * len(ordered)) - 1)]


def _distribution(values):
    finite = [value for value in values if math.isfinite(value)]
    if not finite:
        return {"count": 0, "mean": None, "median": None, "p95": None}
    return {
        "count": len(finite),
        "mean": statistics.fmean(finite),
        "median": statistics.median(finite),
        "p95": _percentile(finite, 0.95),
    }


def _paired_residuals(rows):
    pairs = [
        (float(row["raw_residual_mm"]), float(row["corrected_residual_mm"]))
        for row in rows
        if float(row["raw_residual_mm"]) >= 0
        and float(row["corrected_residual_mm"]) >= 0
    ]
    raw = [pair[0] for pair in pairs]
    corrected = [pair[1] for pair in pairs]
    return {
        "count": len(pairs),
        "raw_mm": _distribution(raw),
        "corrected_mm": _distribution(corrected),
        "mean_change_mm": (
            statistics.fmean(after - before for before, after in pairs)
            if pairs
            else None
        ),
        "fraction_improved": (
            sum(after < before for before, after in pairs) / len(pairs)
            if pairs
            else None
        ),
    }


def _load_capture(path):
    events = []
    with Path(path).open(encoding="utf-8") as stream:
        for line in stream:
            events.append(json.loads(line))
    if not events or events[0].get("type") != "header":
        raise ValueError("Capture has no supported header")
    return events


def _phase_intervals(events):
    intervals = []
    active = defaultdict(list)
    counts = defaultdict(int)
    for event in events:
        if event.get("type") != "phase":
            continue
        name = event["name"]
        if event.get("event") == "start":
            counts[name] += 1
            key = f"{name}-{counts[name]}"
            interval = {
                "key": key,
                "name": name,
                "start_s": event["recorded_s"],
                "end_s": None,
                "command": {
                    field: value
                    for field, value in event.items()
                    if field not in {"type", "event", "name", "recorded_s"}
                },
            }
            intervals.append(interval)
            active[name].append(interval)
        elif event.get("event") == "end" and active[name]:
            active[name].pop(0)["end_s"] = event["recorded_s"]
    capture_end = max((event.get("recorded_s", 0) for event in events), default=0)
    for interval in intervals:
        if interval["end_s"] is None:
            interval["end_s"] = capture_end
    return intervals


def _phase_at(intervals, timestamp):
    for interval in intervals:
        if interval["start_s"] <= timestamp <= interval["end_s"]:
            return interval["key"]
    return "unphased"


def _motion_by_phase(events, intervals):
    wheel = defaultdict(list)
    gyro = defaultdict(list)
    projected = defaultdict(list)
    seen_gyro_times = set()
    commands = {interval["key"]: interval["command"] for interval in intervals}
    for event in events:
        if event.get("type") != "motion":
            continue
        sample = event["sample"]
        timestamp = float(sample["timestamp_s"])
        phase = _phase_at(intervals, timestamp)
        vx = float(sample["vx"])
        vy = float(sample["vy"])
        wheel[phase].append(math.hypot(vx, vy))
        command = commands.get(phase, {})
        direction = command.get("direction_deg")
        speed = command.get("speed_mm_s", command.get("command_speed_mm_s", 0))
        yaw_history = sample.get("yaw", [])
        if direction is not None and speed and yaw_history:
            yaw = math.radians(float(yaw_history[-1][1]))
            global_x = vx * math.cos(yaw) + vy * math.sin(yaw)
            global_y = vx * math.sin(yaw) - vy * math.cos(yaw)
            command_rad = math.radians(float(direction))
            if command.get("body_relative"):
                command_rad += yaw
            projected[phase].append(
                global_x * math.cos(command_rad) + global_y * math.sin(command_rad)
            )
        for gyro_time, value in sample.get("gyro", []):
            gyro_time = float(gyro_time)
            if gyro_time in seen_gyro_times:
                continue
            seen_gyro_times.add(gyro_time)
            gyro[_phase_at(intervals, gyro_time)].append(float(value))
    phases = set(wheel) | set(gyro) | set(projected)
    return {
        phase: {
            "wheel_speed_mm_s": _distribution(wheel[phase]),
            "command_projected_speed_mm_s": _distribution(projected[phase]),
            "gyro_deg_s": _distribution(gyro[phase]),
            "gyro_signed_mean_deg_s": (
                statistics.fmean(gyro[phase]) if gyro[phase] else None
            ),
        }
        for phase in sorted(phases)
    }


def _live_by_phase(events, intervals):
    poses = defaultdict(list)
    confidences = defaultdict(list)
    invalid = defaultdict(int)
    for event in events:
        if event.get("type") != "live":
            continue
        phase = _phase_at(intervals, float(event["recorded_s"]))
        state = event.get("state", {})
        pose = state.get("pose")
        if not pose:
            continue
        valid = bool(pose[4])
        if not valid:
            invalid[phase] += 1
            continue
        if all(value is not None and math.isfinite(float(value)) for value in pose[:3]):
            poses[phase].append([float(value) for value in pose[:3]])
        if pose[3] is not None:
            confidences[phase].append(float(pose[3]))
    result = {}
    for phase in sorted(set(poses) | set(confidences) | set(invalid)):
        phase_poses = poses[phase]
        drift = None
        yaw_drift = None
        if len(phase_poses) >= 2:
            drift = math.hypot(
                phase_poses[-1][0] - phase_poses[0][0],
                phase_poses[-1][1] - phase_poses[0][1],
            )
            yaw_drift = (
                phase_poses[-1][2] - phase_poses[0][2] + 180
            ) % 360 - 180
        result[phase] = {
            "valid_samples": len(phase_poses),
            "invalid_samples": invalid[phase],
            "confidence": _distribution(confidences[phase]),
            "first_pose": phase_poses[0] if phase_poses else None,
            "last_pose": phase_poses[-1] if phase_poses else None,
            "endpoint_drift_mm": drift,
            "endpoint_yaw_change_deg": yaw_drift,
        }
    return result


def analyse(capture_path, replay_dir):
    events = _load_capture(capture_path)
    intervals = _phase_intervals(events)
    replay = {}
    for mode in ("off", "rotation", "full"):
        path = Path(replay_dir) / f"{mode}.csv"
        if not path.exists():
            continue
        with path.open(newline="", encoding="utf-8") as stream:
            rows = list(csv.DictReader(stream))
        grouped = defaultdict(list)
        for row in rows:
            grouped[_phase_at(intervals, float(row["scan_time_s"]))].append(row)
        replay[mode] = {
            phase: _paired_residuals(phase_rows)
            for phase, phase_rows in sorted(grouped.items())
        }
    footer = next(
        (event for event in reversed(events) if event.get("type") == "footer"),
        None,
    )
    health = [event for event in events if event.get("type") == "health"]
    return {
        "capture": str(capture_path),
        "header": events[0],
        "capture_complete": footer is not None,
        "dropped_events": max(
            [event.get("dropped_events", 0) for event in health]
            + ([footer.get("dropped_events", 0)] if footer else [0])
        ),
        "dropped_scans": max(
            [event.get("dropped_scans", 0) for event in health] or [0]
        ),
        "writer_errors": sorted(
            {
                event["writer_error"]
                for event in health
                if event.get("writer_error")
            }
        ),
        "phases": intervals,
        "measured_motion": _motion_by_phase(events, intervals),
        "live_tracking": _live_by_phase(events, intervals),
        "replay_paired_residuals": replay,
        "limitations": [
            "Wall residuals use estimated poses and are not independent ground truth.",
            "Endpoint drift includes legitimate commanded motion.",
            "Phase boundaries use host monotonic command-event times and scan midpoints.",
        ],
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path)
    parser.add_argument("--replay-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    result = analyse(args.capture, args.replay_dir)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x", encoding="utf-8") as stream:
        json.dump(result, stream, indent=2)
    print(
        f"capture_complete={result['capture_complete']} "
        f"dropped_events={result['dropped_events']} "
        f"dropped_scans={result['dropped_scans']}"
    )
    for mode, phases in result["replay_paired_residuals"].items():
        for phase, metrics in phases.items():
            if metrics["count"]:
                print(
                    f"{mode} {phase}: n={metrics['count']} "
                    f"{metrics['raw_mm']['mean']:.2f}->"
                    f"{metrics['corrected_mm']['mean']:.2f} mm "
                    f"improved={metrics['fraction_improved']:.1%}"
                )


if __name__ == "__main__":
    main()
