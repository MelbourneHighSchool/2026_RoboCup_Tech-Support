"""Stationary hardware test: python -m tests.tof_localisation --yaw 0.

Build first: .venv/bin/python lib/setup.py --tof-only build_ext --inplace
No motors are opened. Use the robot's known field heading for --yaw; without a
heading prior the rectangular pitch has a 180-degree ambiguity.
"""

import argparse
import time

from lib.tof_localisation import DEFAULT_BEARINGS, ToFLocalisation


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bus', type=int, default=1)
    parser.add_argument('--yaw', type=float, help='Known pitch heading, clockwise degrees')
    parser.add_argument('--radius', type=float, default=0, help='Sensor radius from robot centre, mm')
    parser.add_argument('--bearings', type=float, nargs=8, default=DEFAULT_BEARINGS)
    parser.add_argument('--max-range', type=float, default=4000)
    parser.add_argument('--pitch', type=float, nargs=2, default=(2430, 1820))
    parser.add_argument('--duration', type=float, default=0, help='Seconds; zero runs until Ctrl-C')
    args = parser.parse_args()
    localisation = ToFLocalisation(*args.pitch, bus_number=args.bus,
                                  bearings=args.bearings, radius_mm=args.radius,
                                  max_range=args.max_range)
    started = time.monotonic()
    next_print = started
    try:
        while not args.duration or time.monotonic() - started < args.duration:
            x, y, yaw, confidence, ok = localisation.update(yaw=args.yaw)
            now = time.monotonic()
            if now >= next_print:
                print(f'x={x:.0f} y={y:.0f} yaw={yaw:.1f} confidence={confidence:.3f} '
                      f'valid={ok} ranges={localisation.distances} errors={localisation.errors}')
                next_print = now + 0.5
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        localisation.close()


if __name__ == '__main__':
    main()
