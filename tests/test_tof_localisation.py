"""Offline sparse MCL checks; build tof_native before running this module."""

import math
import unittest

from lib import tof_native


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
