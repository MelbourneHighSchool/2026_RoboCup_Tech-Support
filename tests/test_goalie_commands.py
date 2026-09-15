import math
import unittest

from defence import goalie


class GoalieCommandTests(unittest.TestCase):
    def test_turn_to_ball_behind_without_translation(self):
        direction, speed, rotation, kick, dribbler = goalie(
            550, 910, 90, 450, 912.6
        )
        self.assertTrue(math.isfinite(direction))
        self.assertEqual(speed, 0)
        self.assertAlmostEqual(rotation, 178.510645, places=5)
        self.assertFalse(kick)
        self.assertEqual(dribbler, 1)

    def test_captured_ball_holds_position_while_aiming_or_kicking(self):
        for yaw, expected_kick, expected_dribbler in [(90, False, 1), (0, True, -1)]:
            with self.subTest(yaw=yaw):
                direction, speed, rotation, kick, dribbler = goalie(
                    550, 910, yaw, 560, 910, ball_captured=True
                )
                self.assertTrue(math.isfinite(direction))
                self.assertEqual(speed, 0)
                self.assertEqual(rotation, 0)
                self.assertEqual(kick, expected_kick)
                self.assertEqual(dribbler, expected_dribbler)

    def test_aligned_with_ball_behind_still_approaches(self):
        direction, speed, *_ = goalie(550, 910, 180, 450, 910)
        self.assertEqual(direction, 180)
        self.assertEqual(speed, 200)
