"""Hardware-free partial-return association, orchestration and clock tests."""

import math
import subprocess
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from lib.bot_fusion import BotRangeFusion, FusionConfig, fuse_bots
from lib.sensor_timing import exposure_monotonic


def bot(distance=1250, bearing=0):
    return {"bearing_deg": bearing, "distance_mm": distance, "confidence": 0.9}


def context(points):
    return {"reason": "ok", "pose": (500, 500, 0, 0.9), "points": points}


def support(x=1000, y=0, stamp=10, static=500):
    return [(x, y - 3, stamp, 30, static), (x, y + 3, stamp + 0.001, 30, static)]


class FusionTests(unittest.TestCase):
    def setUp(self):
        self.config = FusionConfig(range_error_mm=0, bearing_error_deg=0,
                                   target_speed_mm_s=0)

    def fuse(self, points, bots=None):
        return fuse_bots(bots or [bot()], context(points), 10, self.config)

    def test_front_support_constrains_without_fixed_radius_shift(self):
        result = self.fuse(support())[0]
        self.assertEqual(result["source"], "fused")
        self.assertAlmostEqual(result["distance_mm"], 1000 + math.sqrt(110**2 - 3**2))
        self.assertEqual(result["bearing_deg"], 0)

    def test_off_centre_support_uses_both_axes(self):
        result = self.fuse(support(y=90))[0]
        self.assertEqual(result["source"], "fused")
        self.assertLess(result["distance_mm"], 1060)
        self.assertGreater(result["distance_mm"], 1000)

    def test_camera_within_interval_remains_unchanged(self):
        result = self.fuse(support(), [bot(1030)])[0]
        self.assertEqual(result["distance_mm"], 1030)
        self.assertEqual(result["reason"], "camera_within_bounds")

    def test_full_surface_is_not_required_but_also_works(self):
        points = [(1000 - math.sqrt(110**2 - y*y), y, 10, 30, 500)
                  for y in range(-80, 81, 10)]
        result = self.fuse(points, [bot(1150)])[0]
        self.assertEqual(result["source"], "fused")
        self.assertAlmostEqual(result["distance_mm"], 1000)

    def test_isolated_or_duplicate_return_does_not_confirm(self):
        point = support()[0]
        for points in [[point], [point, point]]:
            self.assertEqual(self.fuse(points)[0]["reason"], "isolated_return")

    def test_sparse_low_quality_and_misses_do_not_create_support(self):
        self.assertEqual(self.fuse([(1000, 0, 10, 1, 500)])[0]["source"], "camera")
        self.assertEqual(self.fuse([])[0]["source"], "camera")

    def test_walls_and_posts_veto_match(self):
        for clearance in [-10, 0, 40, 100]:
            result = self.fuse(support(static=clearance))[0]
            self.assertEqual(result["reason"], "static_ambiguity")
            self.assertEqual(result["distance_mm"], 1250)
        # A wall behind the support must not be interpreted as another bot.
        result = self.fuse(support() + support(1400, static=0))[0]
        self.assertEqual(result["source"], "camera")

    def test_two_camera_detections_cannot_share_cluster(self):
        results = self.fuse(support(), [bot(), bot(bearing=1)])
        self.assertTrue(all(r["reason"] == "ambiguous_match" for r in results))

    def test_multiple_plausible_clusters_fall_back(self):
        result = self.fuse(support() + support(1250))[0]
        self.assertEqual(result["reason"], "ambiguous_match")

    def test_two_side_supports_150mm_apart_share_one_footprint(self):
        points = support(y=-75) + support(y=75)
        for config in [self.config, FusionConfig()]:
            for ordered in [points, list(reversed(points))]:
                result = fuse_bots([bot()], context(ordered), 10, config)[0]
                self.assertEqual(result["source"], "fused")
                self.assertEqual(result["reason"], "constrained")
                self.assertEqual(result["bearing_deg"], 0)
        result = self.fuse(points)[0]
        half = math.sqrt(110**2 - 78**2)
        self.assertAlmostEqual(result["bounds_mm"][0], 1000 - half)
        self.assertAlmostEqual(result["bounds_mm"][1], 1000 + half)

    def test_supports_150mm_apart_in_depth_tighten_interval(self):
        near, far = support(), support(1150)
        combined = self.fuse(near + far)[0]
        a, b = self.fuse(near)[0], self.fuse(far)[0]
        self.assertEqual(combined["source"], "fused")
        self.assertEqual(combined["bounds_mm"],
                         [b["bounds_mm"][0], a["bounds_mm"][1]])
        self.assertLess(combined["bounds_mm"][1] - combined["bounds_mm"][0], 70)
        kept = self.fuse(near + far, [bot(1075)])[0]
        self.assertEqual(kept["distance_mm"], 1075)
        self.assertEqual(kept["reason"], "camera_within_bounds")

    def test_three_supports_need_one_global_overlap_not_a_chain(self):
        compatible = self.fuse(support(y=-75) + support() + support(y=75))[0]
        self.assertEqual(compatible["source"], "fused")
        # Adjacent pairs overlap, but the first and last cannot share a centre.
        conflicting = self.fuse(support(900, y=-70) + support(1050)
                                + support(1200, y=70))[0]
        self.assertEqual(conflicting["reason"], "ambiguous_match")
        self.assertEqual(conflicting["distance_mm"], 1250)

    def test_one_shared_support_prevents_merging_with_an_exclusive_support(self):
        results = self.fuse(support(y=-75) + support(y=75),
                            [bot(), bot(bearing=9)])
        self.assertTrue(all(r["reason"] == "ambiguous_match" for r in results))
        self.assertTrue(all(r["source"] == "camera" for r in results))

    def test_separate_single_hits_cannot_be_promoted_to_a_support(self):
        points = [support(y=-75)[0], support(y=75)[0]]
        self.assertEqual(self.fuse(points)[0]["reason"], "isolated_return")
        self.assertEqual(self.fuse(support(y=-75) + points[1:])[0]["source"], "camera")

    def test_a_static_support_vetoes_an_otherwise_compatible_merge(self):
        for static in [0, 90]:
            result = self.fuse(support(y=-75) + support(y=75, static=static))[0]
            self.assertEqual(result["reason"], "static_ambiguity")
            self.assertEqual(result["source"], "camera")

    def test_disconnected_supports_merge_across_bearing_seam(self):
        points = [(-x, y, t, q, d)
                  for x, y, t, q, d in support(y=-75) + support(y=75)]
        result = self.fuse(points, [bot(bearing=180)])[0]
        self.assertEqual(result["source"], "fused")

    def test_inconsistent_cluster_cannot_be_overridden_by_a_second_match(self):
        # A long connected object crosses the candidate sector but cannot fit
        # inside one robot footprint. A second small cluster must not rescue it.
        long_object = [(x, 0, 10, 30, 500) for x in range(900, 1201, 30)]
        result = self.fuse(long_object + support(1000, y=100))[0]
        self.assertEqual(result["source"], "camera")
        self.assertEqual(result["reason"], "inconsistent_returns")

    def test_disjoint_bots_can_both_match(self):
        points = support() + [(y, x, t, q, d) for x, y, t, q, d in support()]
        results = self.fuse(points, [bot(), bot(bearing=90)])
        self.assertTrue(all(r["source"] == "fused" for r in results))

    def test_bearing_seam_cluster(self):
        points = [(-x, y, t, q, d) for x, y, t, q, d in support()]
        result = self.fuse(points, [bot(bearing=180)])[0]
        self.assertEqual(result["source"], "fused")
        result = self.fuse(support(), [bot(bearing=360)])[0]
        self.assertEqual(result["source"], "fused")

    def test_invalid_camera_and_stale_beams(self):
        for b in [bot(None), bot(float("nan")), bot(bearing=float("inf")),
                  {**bot(), "confidence": 0.2}]:
            self.assertEqual(self.fuse(support(), [b])[0]["reason"], "unusable_camera")
        self.assertEqual(self.fuse(support(stamp=9))[0]["reason"], "no_match")
        self.assertEqual(self.fuse(support(3000))[0]["source"], "camera")

    def test_target_motion_widens_bounds(self):
        a = fuse_bots([bot()], context(support(stamp=9.98)), 10, self.config)[0]
        b = fuse_bots([bot()], context(support(stamp=9.98)), 10,
                      replace(self.config, target_speed_mm_s=1000))[0]
        self.assertLess(b["bounds_mm"][0], a["bounds_mm"][0])
        self.assertGreater(b["bounds_mm"][1], a["bounds_mm"][1])

    def test_missing_context_always_preserves_camera(self):
        for reason in ["missing_motion", "uncertain_pose", "stale_camera"]:
            r = fuse_bots([bot()], {"reason": reason}, None)[0]
            self.assertEqual(r["source"], "camera")
            self.assertEqual(r["distance_mm"], 1250)

    def test_config_rejects_unsafe_inputs(self):
        for kwargs in [{"radius_mm": -1}, {"min_points": 1},
                       {"min_points": 2.5}, {"max_time_delta_s": 1},
                       {"range_error_mm": float("nan")}]:
            with self.assertRaises(ValueError):
                FusionConfig(**kwargs)


class OrchestrationTests(unittest.TestCase):
    def setUp(self):
        self.scene = {"frame_id": 3, "timestamp_s": 10, "bots": [bot()]}
        self.native_context = context(support())
        # Historical robot heading differs from the current heading.
        self.native_context["pose"] = (500, 500, 90, 0.9)
        self.scans = [{"time_s": 10, "received_s": 10.01, "points": []}]
        self.lidar = SimpleNamespace(get_scan_history=lambda _reference: self.scans,
                                     fusion_context=lambda *_: self.native_context)
        self.events = []

    def run_mode(self, mode, scene=None):
        return BotRangeFusion(mode, record=self.events.append).project(
            scene or self.scene, self.lidar, (600, 500, 0), 10.02)

    def test_diagnostic_leaves_strategy_and_records_both(self):
        self.assertEqual(self.run_mode("diagnostic"), [(1850, 500)])
        self.assertEqual(len(self.events), 1)
        self.assertEqual(self.events[0]["results"][0]["source"], "fused")
        self.assertAlmostEqual(self.events[0]["proposed_positions"][0][0], 500)

    def test_active_uses_observation_time_pose(self):
        positions = self.run_mode("active")
        self.assertAlmostEqual(positions[0][0], 500)
        self.assertGreater(positions[0][1], 1500)
        self.assertLess(positions[0][1], 1750)

    def test_off_does_not_read_lidar(self):
        self.lidar = None
        self.assertEqual(self.run_mode("off"), [(1850, 500)])
        self.assertEqual(self.events, [])

    def test_missing_stale_future_timestamps_and_native_api(self):
        for timestamp in [None, 9, 11]:
            self.assertEqual(self.run_mode("active", {**self.scene, "timestamp_s": timestamp}),
                             [(1850, 500)])
        self.lidar = SimpleNamespace()
        self.assertEqual(self.run_mode("active"), [(1850, 500)])
        self.assertEqual(self.events[-1]["context"]["reason"], "native_api_unavailable")

    def test_no_scan_or_motion_falls_back(self):
        self.scans.clear()
        self.assertEqual(self.run_mode("active"), [(1850, 500)])
        self.scans.append({"time_s": 10, "received_s": 10.01, "points": []})
        self.native_context.update(reason="missing_motion")
        self.assertEqual(self.run_mode("active"), [(1850, 500)])


class SensorClockTests(unittest.TestCase):
    def test_boot_clock_offset_and_exposure_midpoint(self):
        with patch("lib.sensor_timing.time.monotonic", side_effect=[10, 10.0002]), \
                patch("lib.sensor_timing.time.clock_gettime", return_value=110.0001):
            value = exposure_monotonic({"SensorTimestamp": 109_950_000_000,
                                        "ExposureTime": 10000})
        self.assertAlmostEqual(value, 9.955)

    def test_missing_future_stale_or_jittery_clock_rejected(self):
        self.assertIsNone(exposure_monotonic({}))
        for stamp, after in [(111_000_000_000, 10.0002),
                             (108_000_000_000, 10.0002),
                             (109_950_000_000, 10.1)]:
            with patch("lib.sensor_timing.time.monotonic", side_effect=[10, after]), \
                    patch("lib.sensor_timing.time.clock_gettime", return_value=110.0001):
                self.assertIsNone(exposure_monotonic({"SensorTimestamp": stamp}))


class NativeFusionTests(unittest.TestCase):
    def test_full_resolution_transform_and_map(self):
        root = Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory() as directory:
            binary = str(Path(directory) / "fusion")
            subprocess.run(["g++", "-std=c++11", "-O2", "-Wall", "-Wextra", "-Werror",
                            "-pthread", str(root / "tests/bot_fusion_native.cpp"),
                            "-o", binary], check=True)
            subprocess.run([binary], check=True, timeout=30)
