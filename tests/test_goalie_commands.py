import math
import unittest

from defence import GOALIE_BOX_Y_MAX, GOALIE_BOX_Y_MIN, goalie


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

    def test_boundary_correction_does_not_cancel_turn_to_ball_behind(self):
        direction, speed, rotation, *_ = goalie(
            525, 1361, 80, 400, 1500
        )

        self.assertAlmostEqual(direction, -90)
        self.assertEqual(speed, 700)
        self.assertAlmostEqual(
            rotation,
            math.degrees(math.atan2(1500 - 1361, 400 - 525)) % 360,
        )

    def test_chases_outside_ball_only_as_far_as_top_box_line(self):
        direction, speed, rotation, *_ = goalie(
            525, GOALIE_BOX_Y_MAX - 20, 80, 400, 1700
        )

        self.assertAlmostEqual(direction, 90)
        self.assertEqual(speed, 130)
        self.assertAlmostEqual(
            rotation,
            math.degrees(
                math.atan2(1700 - (GOALIE_BOX_Y_MAX - 20), 400 - 525)
            ) % 360,
        )

        _, speed_at_line, rotation_at_line, *_ = goalie(
            525, GOALIE_BOX_Y_MAX, 80, 400, 1700
        )
        self.assertEqual(speed_at_line, 0)
        self.assertNotEqual(rotation_at_line, 0)

    def test_chases_outside_ball_only_as_far_as_bottom_box_line(self):
        direction, speed, rotation, *_ = goalie(
            525, GOALIE_BOX_Y_MIN + 20, 280, 400, 100
        )

        self.assertAlmostEqual(direction, -90)
        self.assertEqual(speed, 130)
        self.assertNotEqual(rotation, 0)

        _, speed_at_line, rotation_at_line, *_ = goalie(
            525, GOALIE_BOX_Y_MIN, 280, 400, 100
        )
        self.assertEqual(speed_at_line, 0)
        self.assertNotEqual(rotation_at_line, 0)
