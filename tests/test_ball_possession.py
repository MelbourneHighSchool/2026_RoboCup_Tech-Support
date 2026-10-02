import unittest

from lib.ball_possession import (
    BallExtrapolator,
    BallPossessionTracker,
    ball_is_near_bot,
    resolve_ball_position,
    teammate_visible_ball_position,
)


class TeammateBallPriorityTests(unittest.TestCase):
    def resolve(self, *, timed_out=True, self_candidate=False, carrier=None,
                captured=False, local_position=None, visible=True):
        position, captured, _used_teammate = resolve_ball_position(
            local_position,
            captured=captured,
            teammate_position=teammate_visible_ball_position({
                "ball_visible": visible, "observed_ball_x": 800,
                "observed_ball_y": 900,
            }),
            prediction_timed_out=timed_out,
            self_candidate=self_candidate,
            carrier_position=carrier,
            robot_position=(100, 100),
            yaw=0,
        )
        x, y = position if position is not None else (None, None)
        return x, y, captured

    def test_teammate_overrides_self_capture_assumption_after_timeout(self):
        self.assertEqual(self.resolve(self_candidate=True), (800, 900, False))

    def test_teammate_overrides_enemy_carrier_after_timeout(self):
        self.assertEqual(self.resolve(carrier=(300, 400)), (800, 900, False))

    def test_local_prediction_keeps_priority_before_timeout(self):
        self.assertEqual(
            self.resolve(timed_out=False, local_position=(200, 250)),
            (200, 250, False),
        )

    def test_break_beam_capture_keeps_priority(self):
        self.assertEqual(
            self.resolve(captured=True, local_position=(200, 100)),
            (200, 100, True),
        )

    def test_inferred_teammate_report_does_not_override_assumption(self):
        self.assertEqual(
            self.resolve(visible=False, carrier=(300, 400)), (300, 400, False)
        )

    def test_legacy_and_invalid_reports_are_not_direct_sightings(self):
        for message in (None, {"ball_x": 800, "ball_y": 900},
                        {"ball_visible": True, "observed_ball_x": float("nan"),
                         "observed_ball_y": 900}):
            self.assertIsNone(teammate_visible_ball_position(message))


class BallPossessionTrackerTests(unittest.TestCase):
    def setUp(self):
        self.tracker = BallPossessionTracker()

    def arm(self, enemy=(130.0, 100.0)):
        result = self.tracker.update(
            (100.0, 100.0), [enemy], 1.0, new_camera_frame=True
        )
        self.assertIsNone(result)

    def test_hidden_ball_follows_nearby_enemy(self):
        self.arm()
        self.assertEqual(
            self.tracker.update(None, [(180.0, 100.0)], 1.1, new_camera_frame=True),
            (180.0, 100.0),
        )

    def test_proximity_is_measured_from_edge_of_bot(self):
        self.tracker.update(
            (100.0, 100.0), [(260.0, 100.0)], 1.0, new_camera_frame=True
        )
        self.assertEqual(
            self.tracker.update(None, [(270.0, 100.0)], 1.1, new_camera_frame=True),
            (270.0, 100.0),
        )

    def test_ball_more_than_50mm_from_bot_edge_does_not_arm(self):
        self.tracker.update(
            (100.0, 100.0), [(260.1, 100.0)], 1.0, new_camera_frame=True
        )
        self.assertIsNone(
            self.tracker.update(None, [(270.0, 100.0)], 1.1, new_camera_frame=True)
        )

    def test_ball_near_own_bot_can_replace_broken_break_beam(self):
        self.assertTrue(ball_is_near_bot((260.0, 100.0), (100.0, 100.0)))
        self.assertFalse(ball_is_near_bot((260.1, 100.0), (100.0, 100.0)))

    def test_closest_detection_to_old_position_wins(self):
        self.arm()
        self.assertEqual(
            self.tracker.update(
                None,
                [(300.0, 100.0), (140.0, 100.0)],
                1.1,
                new_camera_frame=True,
            ),
            (140.0, 100.0),
        )

    def test_two_metres_per_second_is_not_same_bot(self):
        self.arm(enemy=(100.0, 100.0))
        self.assertIsNone(
            self.tracker.update(
                None, [(300.0, 100.0)], 1.1, new_camera_frame=True
            )
        )

    def test_carrier_disappearing_ends_inference(self):
        self.arm()
        self.assertIsNone(
            self.tracker.update(None, [], 1.1, new_camera_frame=True)
        )
        self.assertIsNone(
            self.tracker.update(
                None, [(140.0, 100.0)], 1.2, new_camera_frame=True
            )
        )

    def test_ball_seen_again_ends_old_carrier_tracking(self):
        self.arm()
        self.tracker.update(
            (500.0, 500.0), [(130.0, 100.0)], 1.1, new_camera_frame=True
        )
        self.assertIsNone(
            self.tracker.update(
                None, [(140.0, 100.0)], 1.2, new_camera_frame=True
            )
        )

    def test_repeated_logic_iteration_keeps_last_carrier_position(self):
        self.arm()
        self.assertEqual(
            self.tracker.update(None, [(999.0, 999.0)], 1.05, new_camera_frame=False),
            (130.0, 100.0),
        )

    def test_unhealthy_camera_drops_carrier(self):
        self.arm()
        self.assertIsNone(
            self.tracker.update(
                None,
                [(130.0, 100.0)],
                1.1,
                new_camera_frame=False,
                camera_healthy=False,
            )
        )

class BallExtrapolatorTests(unittest.TestCase):
    def test_predicts_velocity_until_timeout(self):
        tracker = BallExtrapolator(0.5)
        tracker.update((100.0, 100.0), 1.0)
        tracker.update((110.0, 100.0), 1.1)
        predicted = tracker.update(None, 1.3)
        self.assertIsNotNone(predicted)
        self.assertAlmostEqual(predicted[0], 130.0)
        self.assertAlmostEqual(predicted[1], 100.0)
        self.assertIsNone(tracker.update(None, 1.6))
        self.assertTrue(tracker.timed_out(1.6))

    def test_no_prediction_before_first_observation(self):
        tracker = BallExtrapolator(0.5)
        self.assertIsNone(tracker.update(None, 1.0))
        self.assertFalse(tracker.timed_out(1.0))


if __name__ == "__main__":
    unittest.main()
