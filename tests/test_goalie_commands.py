import math
import unittest

from defence import GOALIE_BLOCK_X, goalie


class GoalieCommandTests(unittest.TestCase):
    def test_tracks_up_and_down_without_leaving_line(self):
        for ball_y, sign in [(1200, 1), (600, -1)]:
            direction, speed, *_ = goalie(GOALIE_BLOCK_X, 910, 0, 1000, ball_y)
            self.assertAlmostEqual(speed * math.cos(math.radians(direction)), 0)
            self.assertGreater(sign * speed * math.sin(math.radians(direction)), 0)

    def test_corrects_displacement_from_line(self):
        for x, sign in [(450, 1), (650, -1)]:
            direction, speed, *_ = goalie(x, 910, 0, 1800, 910)
            self.assertGreater(sign * speed * math.cos(math.radians(direction)), 0)
            self.assertAlmostEqual(speed * math.sin(math.radians(direction)), 0)

    def test_stops_at_target_and_slows_when_near(self):
        self.assertEqual(goalie(GOALIE_BLOCK_X, 910, 0, 1800, 910)[1], 0)
        self.assertEqual(goalie(GOALIE_BLOCK_X, 930, 0, 1800, 910)[1], 60)
        self.assertEqual(goalie(GOALIE_BLOCK_X, 1200, 0, 1800, 910)[1], 700)

    def test_tracks_through_centre_without_intermediate_home_target(self):
        for y in (800, 890, 910, 930):
            direction, speed, *_ = goalie(GOALIE_BLOCK_X, y, 0, 1000, 1250)
            self.assertAlmostEqual(direction, 90)
            self.assertGreater(speed, 0)

    def test_missing_ball_uses_centre_on_same_line(self):
        self.assertEqual(goalie(GOALIE_BLOCK_X, 910, 90, None, None), (0, 0, 0, False, 0))
        direction, speed, *_ = goalie(650, 910, 0, None, None)
        self.assertAlmostEqual(direction, 180)
        self.assertGreater(speed, 0)

    def test_limits_sideways_travel(self):
        for y, ball_y in [(1360, 1700), (460, 100)]:
            command = goalie(GOALIE_BLOCK_X, y, 0, 400, ball_y)
            self.assertEqual(command[1], 0)

    def test_nearby_or_captured_ball_never_triggers_collection_or_kick(self):
        for captured in (False, True):
            command = goalie(GOALIE_BLOCK_X, 910, 0, 600, 910, ball_captured=captured)
            self.assertEqual(command[1:], (0, 0, False, 0))
            self.assertTrue(all(math.isfinite(value) for value in command))


if __name__ == "__main__":
    unittest.main()
