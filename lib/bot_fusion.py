"""Conservative camera-confirmed LIDAR bot ranging, independent of hardware.

A return is an interior point of a bounded footprint, not its centre or a known
circular surface. Never add a fixed radius to a surface measurement.
"""

import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path


@dataclass(frozen=True)
class FusionConfig:
    # Provisional diagnostic defaults: measure these before enabling strategy use.
    radius_mm: float = 110.0
    range_error_mm: float = 20.0
    bearing_error_deg: float = 2.0
    camera_error_mm: float = 150.0
    camera_error_fraction: float = 0.25
    max_time_delta_s: float = 0.05
    max_camera_age_s: float = 0.25
    timing_error_s: float = 0.005
    target_speed_mm_s: float = 2000.0
    static_clearance_mm: float = 100.0
    cluster_gap_mm: float = 60.0
    cluster_angle_deg: float = 3.0
    min_points: int = 2
    min_confidence: float = 0.5

    def __post_init__(self):
        for name, value in asdict(self).items():
            if isinstance(value, bool) or not math.isfinite(value) or value < 0:
                raise ValueError(f"{name} must be finite and nonnegative")
        if (self.radius_mm <= 0 or self.cluster_gap_mm <= 0
                or not 0 < self.cluster_angle_deg < 30
                or not 0 <= self.bearing_error_deg < 30
                or not 0 < self.max_time_delta_s <= 0.1
                or not 0 < self.max_camera_age_s <= 0.25
                or self.min_confidence > 1
                or not isinstance(self.min_points, int) or self.min_points < 2):
            raise ValueError("Invalid fusion geometry, timing, confidence or point count")

    @classmethod
    def load(cls, path):
        return cls(**json.loads(Path(path).read_text())) if path else cls()


def _finite(value):
    return isinstance(value, (int, float)) and math.isfinite(value)


def _clusters(points, config):
    """Small spatial/angular runs, including a run crossing the bearing seam."""
    ordered = sorted(points, key=lambda p: math.atan2(p[1], p[0]))
    if not ordered:
        return []

    def adjacent(a, b):
        angle = math.degrees(math.atan2(b[1], b[0]) - math.atan2(a[1], a[0]))
        return (abs(math.remainder(angle, 360)) <= config.cluster_angle_deg
                and math.hypot(a[0] - b[0], a[1] - b[1]) <= config.cluster_gap_mm)

    groups = [[ordered[0]]]
    for point in ordered[1:]:
        if adjacent(groups[-1][-1], point):
            groups[-1].append(point)
        else:
            groups.append([point])
    if len(groups) > 1 and adjacent(groups[-1][-1], groups[0][0]):
        groups[0] = groups.pop() + groups[0]
    return groups


def _interval(point, bearing, reference_time, config):
    x, y, beam_time, _quality, _static_distance = point
    c, s = math.cos(bearing), math.sin(bearing)
    along, across = x * c + y * s, -x * s + y * c
    # Bearing uncertainty grows with range. Other-bot motion is NOT deskewed by
    # our odometry, so explicitly allow its displacement and camera timing error.
    radius = (config.radius_mm + config.range_error_mm
              + math.hypot(x, y) * math.sin(math.radians(config.bearing_error_deg))
              + config.target_speed_mm_s
              * (abs(beam_time - reference_time) + config.timing_error_s))
    if abs(across) > radius:
        return None
    half = math.sqrt(max(0, radius * radius - across * across))
    return max(0, along - half), along + half


def fuse_bots(bots, context, reference_time, config=None):
    """Return one result per camera bot, retaining order and camera bearing.

    Context points are native-transformed (x, y, time_s, quality, static_distance).
    Several support clusters may constrain one bot, but each cluster must have
    a unique camera owner and all must admit the same centre-distance interval.
    """
    config = config or FusionConfig()
    results = [{"bearing_deg": bot["bearing_deg"],
                "distance_mm": bot["distance_mm"],
                "camera_distance_mm": bot["distance_mm"],
                "source": "camera", "bounds_mm": None,
                "reason": context["reason"]} for bot in bots]
    if context["reason"] != "ok":
        return results
    points = [tuple(p) for p in context["points"]
              if len(p) == 5 and all(_finite(v) for v in p) and p[3] >= 5
              and abs(p[2] - reference_time) <= config.max_time_delta_s]
    # Deduplicate malformed/repeated input without turning copies into support.
    groups = _clusters(list(dict.fromkeys(points)), config)
    candidates = [[] for _ in bots]
    owners = {}
    for index, bot in enumerate(bots):
        result = results[index]
        bearing, distance = bot["bearing_deg"], bot["distance_mm"]
        if (not _finite(bearing) or not _finite(distance) or distance <= 0
                or not _finite(bot.get("confidence"))
                or bot["confidence"] < config.min_confidence):
            result["reason"] = "unusable_camera"
            continue
        angle = math.radians(bearing)
        allowance = max(config.camera_error_mm, config.camera_error_fraction * distance)
        near_static = False
        result["reason"] = "no_match"
        for cluster_id, group in enumerate(groups):
            intervals = [_interval(p, angle, reference_time, config) for p in group]
            plausible = [i for i in intervals if i is not None
                         and i[0] <= distance + allowance and i[1] >= distance - allowance]
            if not plausible:
                continue
            # Reject a camera candidate near static geometry rather than silently
            # removing the wall and accidentally selecting another object behind it.
            if any(p[4] <= config.static_clearance_mm for p in group):
                near_static = True
                continue
            if len(plausible) != len(group):
                result["reason"] = "inconsistent_returns"
                continue
            low = max(i[0] for i in plausible)
            high = min(i[1] for i in plausible)
            if low > high or high <= 0:
                result["reason"] = "inconsistent_returns"
                continue
            candidates[index].append((cluster_id, low, high, len(group)))
            owners.setdefault(cluster_id, set()).add(index)
        if near_static:
            # Keep ownership claims to prevent this bot's cluster being handed
            # to a second detection after the first was rejected for geometry.
            result["reason"] = "static_ambiguity"

    for index, choices in enumerate(candidates):
        result = results[index]
        if result["reason"] in {"static_ambiguity", "inconsistent_returns"} or not choices:
            continue
        if any(len(owners[cluster_id]) != 1 for cluster_id, *_ in choices):
            result["reason"] = "ambiguous_match"
            continue
        # Every disconnected support must stand on its own. Do not combine
        # isolated noise points merely to reach the minimum return count.
        if any(count < config.min_points for _, _, _, count in choices):
            result["reason"] = "isolated_return"
            continue
        # Intersect ALL plausible supports: an overlap means one centre on the
        # camera bearing can contain every return within its footprint/error
        # bounds. Never select a convenient subset or chain pairwise overlaps;
        # either can conceal a competing object at a different range.
        low = max(low for _, low, _, _ in choices)
        high = min(high for _, _, high, _ in choices)
        if low > high:
            result["reason"] = "ambiguous_match"
            continue
        distance = result["camera_distance_mm"]
        allowance = max(config.camera_error_mm, config.camera_error_fraction * distance)
        if high - low >= 2 * allowance:
            result["reason"] = "weak_constraint"
            continue
        selected = min(max(distance, low), high)
        if abs(selected - distance) > allowance:
            result["reason"] = "range_disagreement"
            continue
        result.update(distance_mm=selected, source="fused", bounds_mm=[low, high],
                      reason="constrained" if selected != distance else "camera_within_bounds")
    return results


def project_bots(bots, pose):
    """Project observation-time polar centres with a pose in the same time frame."""
    x, y, yaw = pose[:3]
    output = []
    for bot in bots:
        bearing, distance = bot["bearing_deg"], bot["distance_mm"]
        if not _finite(bearing) or not _finite(distance) or distance <= 0:
            continue
        angle = math.radians(yaw + bearing)
        output.append((x + distance * math.cos(angle), y + distance * math.sin(angle)))
    return output


class BotRangeFusion:
    """One evaluation per new camera frame; diagnostic mode preserves strategy."""

    def __init__(self, mode="diagnostic", config=None, record=None):
        if mode not in {"off", "diagnostic", "active"}:
            raise ValueError("Fusion mode must be off, diagnostic or active")
        self.mode = mode
        self.config = config or FusionConfig()
        self.record = record

    def project(self, scene, lidar, current_pose, now):
        bots = scene["bots"]
        original = project_bots(bots, current_pose)
        if self.mode == "off" or not bots:
            return original
        timestamp = scene["timestamp_s"]
        context = {"reason": "missing_camera_time", "points": []}
        scan = None
        if _finite(timestamp):
            if not 0 <= now - timestamp <= self.config.max_camera_age_s:
                context["reason"] = "stale_camera"
            elif not hasattr(lidar, "get_scan_history") or not hasattr(lidar, "fusion_context"):
                context["reason"] = "native_api_unavailable"
            else:
                scans = [s for s in lidar.get_scan_history(timestamp)
                         if _finite(s["time_s"]) and s["time_s"] > 0
                         and 0 <= now - s["received_s"] <= self.config.max_camera_age_s]
                if scans:
                    scan = min(scans, key=lambda s: abs(s["time_s"] - timestamp))
                    context = lidar.fusion_context(scan["points"], timestamp,
                                                   self.config.max_time_delta_s)
                else:
                    context["reason"] = "no_scan"
        results = fuse_bots(bots, context, timestamp, self.config)
        proposed = (project_bots(results, context["pose"])
                    if context["reason"] == "ok" else original)
        if self.record is not None:
            self.record({"scene": scene, "scan": scan, "context": context,
                         "results": results, "camera_positions": original,
                         "proposed_positions": proposed, "mode": self.mode})
        return proposed if self.mode == "active" else original
