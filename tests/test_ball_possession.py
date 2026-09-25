import unittest

from lib.ball_possession import (
    BallExtrapolator,
    BallPossessionTracker,
    ball_is_near_bot,
)


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
