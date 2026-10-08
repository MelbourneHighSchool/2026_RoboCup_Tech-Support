"""Offline sparse MCL checks; build tof_native before running this module."""

import math
import unittest

from lib import tof_native
from tests.tof_localisation import target_command


class TargetMovementTests(unittest.TestCase):
    def test_rightward_target_uses_native_heading_frame(self):
        command, arrived = target_command(
            (800, 1000), (800, 400, 90, 0.8, True), fresh=True,
            max_speed=500, heading_offset=90,
        )
        self.assertEqual(command, (0, 500, 0, 1))
        self.assertFalse(arrived)

    def test_slows_near_target_and_stops_on_arrival(self):
        command, arrived = target_command(
            (950, 400), (800, 400, 0, 0.8, True), fresh=True,
            max_speed=500, heading_offset=0,
        )
        self.assertEqual(command[1], 250)
        self.assertFalse(arrived)
        command, arrived = target_command(
            (805, 400), (800, 400, 0, 0.8, True), fresh=True,
            max_speed=500, heading_offset=0,
        )
        self.assertEqual(command[1], 0)
        self.assertTrue(arrived)

    def test_stale_or_invalid_pose_stops_without_claiming_arrival(self):
        for fresh, valid in ((False, True), (True, False)):
            with self.subTest(fresh=fresh, valid=valid):
                command, arrived = target_command(
                    (800, 400), (800, 400, 0, 0.8, valid), fresh=fresh,
                    max_speed=500, heading_offset=0,
                )
                self.assertEqual(command[1], 0)
                self.assertEqual(command[3], 0)
                self.assertFalse(arrived)


class SparseLocalisationTests(unittest.TestCase):
    def setUp(self):
        tof_native.start(2430, 1820)
        tof_native.set_imu_yaw(0)

    def tearDown(self):
        tof_native.stop()

    def test_eight_offset_sensors_acquire_pose(self):
        readings = []
        for bearing in range(0, 360, 45):
            c = math.cos(math.radians(bearing))
            s = math.sin(math.radians(bearing))
            distances = []
            if abs(c) > 1e-8:
                distances.append((1630 if c > 0 else -800) / c)
            if abs(s) > 1e-8:
                distances.append((1420 if s > 0 else -400) / s)
            readings.append((bearing, min(distances) - 75, 75 * c, 75 * s))
        for _ in range(180):
            tof_native.predict_odometry(0, 0, 0, 0.05)
            tof_native.update(readings)
        x, y, yaw, confidence, ok = tof_native.get_coordinates_info()
        self.assertTrue(ok)
        self.assertLess(abs(x - 800), 40)
        self.assertLess(abs(y - 400), 40)
        self.assertLess(abs(yaw), 5)
        self.assertGreater(confidence, 0.5)

    def test_insufficient_sensors_do_not_acquire_pose(self):
        for _ in range(20):
            tof_native.predict_odometry(0, 0, 0, 0.05)
            tof_native.update([(0, 1000, 0, 0)] * 8)
        self.assertFalse(tof_native.get_coordinates_info()[-1])


if __name__ == '__main__':
    unittest.main()
