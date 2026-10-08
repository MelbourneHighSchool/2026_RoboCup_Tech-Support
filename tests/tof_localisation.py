"""Drive to field coordinates using eight ToFs, native motors and IMU.

ToF build: .venv/bin/python lib/setup.py --tof-only build_ext --inplace
Motor/IMU build: SOCCER_HARDWARE_ONLY=1 .venv/bin/python lib/setup.py build_ext --inplace
Run: python -m tests.tof_localisation --radius 75
Use --no-move for a sensor-only stationary test. --yaw is the known initial
pitch heading (default forward), used to resolve the field's 180-degree ambiguity.
"""

import argparse
import math
import queue
import threading
import time

from lib.tof_localisation import DEFAULT_BEARINGS, ToFLocalisation

TARGET_TOLERANCE_MM = 10
SLOW_RADIUS_MM = 300
MAX_POSE_AGE_S = 0.3


def target_command(target, pose, *, fresh, max_speed, heading_offset):
    """Return native-frame direction/speed/heading/strength and arrival status."""
    x, y, yaw, _confidence, ok = pose
    heading = yaw - heading_offset
    if not ok or not fresh or target is None:
        return (0, 0, heading, 0), False
    dx, dy = target[0] - x, target[1] - y
    distance = math.hypot(dx, dy)
    if distance <= TARGET_TOLERANCE_MM:
        return (0, 0, heading, 0), True
    direction = math.degrees(math.atan2(dy, dx)) - heading_offset
    speed = min(max_speed, distance / SLOW_RADIUS_MM * max_speed)
    return (direction, speed, heading, 1), False


def prompt_target(results, pitch):
    """Read stdin on a daemon thread while the main loop keeps updating MCL."""
    try:
        x = float(input('What x position to move to? '))
        y = float(input('What y position to move to? '))
        if not (math.isfinite(x) and math.isfinite(y)):
            raise ValueError('Target coordinates must be finite')
        if not (0 <= x <= pitch[0] and 0 <= y <= pitch[1]):
            raise ValueError('Target coordinates must be inside the pitch')
        results.put(('target', (x, y)))
    except ValueError as exc:
        results.put(('error', str(exc)))
    except EOFError:
        results.put(('quit', None))


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bus', type=int, default=1)
    parser.add_argument('--yaw', type=float, default=0, help='Initial pitch heading, clockwise degrees')
    parser.add_argument('--radius', type=float, default=0, help='Sensor radius from robot centre, mm')
    parser.add_argument('--bearings', type=float, nargs=8, default=DEFAULT_BEARINGS)
    parser.add_argument('--max-range', type=float, default=4000)
    parser.add_argument('--pitch', type=float, nargs=2, default=(2430, 1820))
    parser.add_argument('--duration', type=float, default=0, help='Seconds; zero runs until Ctrl-C')
    parser.add_argument('--no-move', action='store_true', help='Monitor ToFs without opening motors/IMU')
    parser.add_argument('--max-speed', type=float, default=500, help='Maximum speed, mm/s (up to 2000)')
    parser.add_argument('-s', '--stream', action='store_true', help='Stream poses to simulate.py')
    args = parser.parse_args()
    if not math.isfinite(args.max_speed) or not 0 < args.max_speed <= 2000:
        parser.error('--max-speed must be between 0 and 2000 mm/s')
    if not math.isfinite(args.yaw):
        parser.error('--yaw must be finite')
    if not math.isfinite(args.duration) or args.duration < 0:
        parser.error('--duration must be finite and nonnegative')
    if not all(math.isfinite(value) and value > 0 for value in args.pitch):
        parser.error('--pitch dimensions must be finite and positive')
    return args


def main():
    args = parse_args()
    hardware = localisation = None
    try:
        if not args.no_move:
            from lib.hardware_test_utils import (
                MAX_YAW_RPM,
                WHEEL_DIAMETER,
                create_hardware,
                set_startup_yaw,
            )

            motor_rpm = max(400, math.ceil(
                args.max_speed * 60 / (math.pi * WHEEL_DIAMETER) + MAX_YAW_RPM
            ))
            hardware = create_hardware(max_motor_rpm=motor_rpm, use_pcb=False)
            set_startup_yaw(hardware)
        localisation = ToFLocalisation(*args.pitch, bus_number=args.bus,
                                      bearings=args.bearings, radius_mm=args.radius,
                                      max_range=args.max_range)
        send_log = None
        if args.stream:
            from lib import send_log

            send_log.start_server_background()
            print(f'Connect with: python simulate.py --connect 127.0.0.1:{send_log.PORT}')
        started = time.monotonic()
        next_print = started
        results = queue.Queue()
        target = None
        prompting = False
        print('Waiting for first pose estimate...')
        while not args.duration or time.monotonic() - started < args.duration:
            imu_yaw, vx, vy, omega = 0, 0, 0, 0
            imu_ok = True
            if hardware is not None:
                imu_yaw = hardware.get_yaw()
                omega = hardware.get_gyro_z_deg_s()
                imu_ok = imu_yaw is not None and omega is not None
                if imu_ok:
                    vx, vy = hardware.get_measured_body_velocity_mm_s(imu_yaw)
            pose = localisation.update(
                yaw=args.yaw + imu_yaw if imu_ok else None,
                vx=vx, vy=vy, omega=omega if omega is not None else 0,
            )
            now = time.monotonic()
            fresh = (localisation.last_scan_time is not None
                     and now - localisation.last_scan_time <= MAX_POSE_AGE_S and imu_ok)
            if hardware is not None:
                try:
                    status, payload = results.get_nowait()
                except queue.Empty:
                    status = None
                if status is not None:
                    prompting = False
                    if status == 'quit':
                        break
                    if status == 'target':
                        target = payload
                        print(f'Driving to {target}')
                    else:
                        print(payload)
                command, arrived = target_command(
                    target, pose, fresh=fresh, max_speed=args.max_speed,
                    heading_offset=args.yaw,
                )
                hardware.move(*command)
                if arrived:
                    print(f'Reached target ({target[0]:.1f}, {target[1]:.1f})')
                    target = None
                if target is None and not prompting and pose[-1] and fresh:
                    threading.Thread(target=prompt_target, args=(results, args.pitch),
                                     daemon=True).start()
                    prompting = True
            x, y, yaw, confidence, ok = pose
            if send_log is not None and ok and fresh:
                send_log.update_latest_log(f'{x},{y},{yaw},None,None')
            if now >= next_print:
                print(f'x={x:.0f} y={y:.0f} yaw={yaw:.1f} confidence={confidence:.3f} '
                      f'valid={ok} fresh={fresh} ranges={localisation.distances} '
                      f'errors={localisation.errors}')
                next_print = now + 0.5
            time.sleep(0.02)
    except KeyboardInterrupt:
        print('\nStopping test.')
    finally:
        try:
            if hardware is not None:
                hardware.stop()
        finally:
            if localisation is not None:
                localisation.close()


if __name__ == '__main__':
    main()
