"""Eight SteelBar ToFs feeding the shared C++ MCL, without the LIDAR SDK.

Addresses are assumed clockwise from forward. Bearings are clockwise-positive;
mounting offsets are forward/right in mm. Invalid/stale readings are omitted,
not interpreted as evidence of a missing wall. One bus owner polls all sensors.
"""

import math
import time

ADDRESSES = (*range(0x50, 0x57), 0x5F)
DEFAULT_BEARINGS = tuple(i * 45.0 for i in range(8))


class ToFLocalisation:
    def __init__(self, pitch_x=2430, pitch_y=1820, *, bus_number=1,
                 bearings=DEFAULT_BEARINGS, radius_mm=0, min_range=40,
                 max_range=4000):
        if len(bearings) != 8 or not all(math.isfinite(b) for b in bearings):
            raise ValueError("Provide eight finite clockwise sensor bearings")
        if not math.isfinite(radius_mm) or radius_mm < 0:
            raise ValueError("radius_mm must be finite and nonnegative")
        if not 0 < min_range < max_range:
            raise ValueError("Require 0 < min_range < max_range")
        from smbus2 import SMBus, i2c_msg

        from lib import tof_native

        self._native = tof_native
        self._msg = i2c_msg
        self._bus = SMBus(bus_number)
        self.bearings = tuple(bearings)
        self.radius_mm = radius_mm
        self.min_range, self.max_range = min_range, max_range
        self._sequences = {}
        self.distances = [None] * 8
        self.errors = {}
        self._last_update = time.monotonic()
        self.last_scan_time = None
        try:
            self._native.start(pitch_x, pitch_y)
        except BaseException:
            self._bus.close()
            raise

    def update(self, *, yaw=None, vx=0, vy=0, omega=0):
        """Poll fresh readings; velocity is forward/left, omega clockwise deg/s.

        With no motion inputs this is a stationary test. Yaw must be in the
        pitch frame, not an unreferenced raw IMU heading. Calls are synchronous.
        """
        now = time.monotonic()
        self._native.predict_odometry(vx, vy, omega, now - self._last_update)
        self._last_update = now
        if yaw is not None:
            self._native.set_imu_yaw(yaw)
        readings = []
        self.distances = [None] * 8
        self.errors = {}
        for index, (address, bearing) in enumerate(zip(ADDRESSES, self.bearings)):
            try:
                write = self._msg.write(address, [0x10])
                read = self._msg.read(address, 5)
                self._bus.i2c_rdwr(write, read)
                data = bytes(read)
                sequence = data[0]
                if self._sequences.get(address) == sequence:
                    continue
                self._sequences[address] = sequence
                distance = int.from_bytes(data[1:5], 'little', signed=True)
                if not self.min_range <= distance <= self.max_range:
                    continue
                self.distances[index] = distance
                angle = math.radians(bearing)
                readings.append((bearing, distance, self.radius_mm * math.cos(angle),
                                 self.radius_mm * math.sin(angle)))
            except OSError as exc:
                self.errors[address] = str(exc)
        if self._native.update(readings, self.min_range, self.max_range):
            self.last_scan_time = time.monotonic()
        return self._native.get_coordinates_info()

    def close(self):
        try:
            self._native.stop()
        finally:
            self._bus.close()
