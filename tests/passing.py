"""On both bots: python -m tests.passing --session drill1

Both dribblers stay on. No translation. One pass, then hold until Ctrl+C.
Geometry and the handshake live in lib.passing.
This script owns hardware setup and runs one stationary attempt.
"""

import argparse
import math
import time
from dataclasses import asdict

from lib.bot_fusion import FusionConfig, evaluate_bot_scene
from lib.passing import PassConfig, PassControllerConfig, angle_error, step_pass


def run_passing_test(*, session, port=5006, startup_heading=0.0, fusion_config_path=None,
                     pass_config=None, duration=None, camera_stream=False, record_path=None, use_pcb=True):
    """Run the same stationary one-pass exercise on both bots until Ctrl+C.

    Reusable perception checks gate the test-local readiness handshake. Motors
    receive no translation commands; the break-beam alone establishes possession.
    """
    if not isinstance(session, str) or not session or len(session) > 64:
        raise ValueError("Use the same nonempty session name (up to 64 chars) on both bots")
    if not 1 <= port <= 65535 or not math.isfinite(startup_heading):
        raise ValueError("Invalid UDP port or startup heading")
    if duration is not None and (not math.isfinite(duration) or duration <= 0):
        raise ValueError("Duration must be finite and positive")
    config = pass_config or PassControllerConfig()
    fusion_config = FusionConfig.load(fusion_config_path)

    from lib import lidar
    from lib.break_beam import Breakbeam
    from lib.camera import Camera
    from lib.communication import Peer
    from lib.config import load_config
    from lib.hardware_test_utils import create_hardware, set_startup_yaw
    from lib.localisation_motion import (
        MotionCapture,
        close_motion_capture,
        configure_motion,
        feed_timed_motion,
    )

    hardware = camera = peer = beam = recorder = None
    lidar_started = False
    try:
        robot_config = load_config()
        if len(robot_config.i2c_addresses) < 5:
            raise ValueError("This test requires four wheels and a dribbler motor")
        if not hasattr(lidar, "fusion_context"):
            raise RuntimeError("Rebuild lib/setup.py on this Pi: timed LIDAR fusion is required")
        hardware = create_hardware(max_motor_rpm=400, kicker=True, use_pcb=use_pcb)
        raw_reference = set_startup_yaw(hardware)
        # Both robots' MCL priors must use the SAME field frame, even if their
        # physical startup headings differ. Clockwise-positive native yaw is
        # startup_reference - raw_yaw.
        hardware.set_startup_yaw(raw_reference + startup_heading)
        imu_generation = hardware.health()["imu_recovery_generation"]
        beam = Breakbeam(robot_config.break_beam_pin)
        lidar.init(robot_config.lidar_port, 460800)
        lidar_started = True
        lidar.start_coordinates(2430, 1820, use_pcb=False)
        configure_motion(lidar, use_pcb=False)
        camera = Camera(8000, resolution=(640, 640), frame_rate=90)
        if camera_stream:
            camera.start_stream()
        else:
            camera.start()
        peer = Peer(port=port, peer_timeout_s=config.max_peer_age_s)
        peer.start()
        if record_path:
            recorder = MotionCapture(record_path, {"pass_config": asdict(config),
                                     "fusion_config": asdict(fusion_config), "session": session,
                                     "bot_id": peer.bot_id, "startup_heading": startup_heading})
        print(f"Passing test {session!r}, bot {peer.bot_id}, UDP {port}.", flush=True)
        print("Both dribblers on. Waiting for one bot to hold the ball. "
              "One pass only; Ctrl+C stops motors and sensors.", flush=True)
        started = time.monotonic()
        state = evidence = None
        last_frame = None
        last_phase = None
        last_scan_time = 0.0
        while duration is None or time.monotonic() - started < duration:
            tick = time.monotonic()
            health = hardware.health()
            if (not health["imu_healthy"]
                    or health["imu_recovery_generation"] != imu_generation):
                raise RuntimeError("IMU lost/reset; restart the passing test to restore its field heading")
            imu_yaw = hardware.get_yaw()
            gyro = hardware.get_gyro_z_deg_s()
            if imu_yaw is None or gyro is None:
                raise RuntimeError("IMU heading/rate stale")
            lidar.set_imu_yaw(imu_yaw)
            feed_timed_motion(lidar, hardware)
            deskew = lidar.get_deskew_status()
            if deskew["accepted"]:
                last_scan_time = deskew["scan_time_s"]
            x, y, field_yaw, confidence = lidar.get_pose()
            pose_ok = (all(v is not None and math.isfinite(v) for v in (x, y, field_yaw, confidence))
                       and confidence >= 0.7 and 0 <= tick - last_scan_time <= 0.25)
            if not pose_ok:
                x = y = field_yaw = None
            scene = camera.get_scene_snapshot()
            new_frame = scene is not None and scene["frame_id"] != last_frame
            if camera.inference_error is not None or scene is None:
                evidence = None
            elif new_frame:
                evidence = evaluate_bot_scene(scene, lidar, time.monotonic(), fusion_config)
                last_frame = scene["frame_id"]
            peer_snapshot = peer.receive_snapshot()
            captured = beam.read()
            now = time.monotonic()
            direction, speed, rotation, state, kick, dribbler = step_pass(
                (x, y, field_yaw), captured, bot_id=peer.bot_id, session=session,
                now=now, gyro_deg_s=gyro, peer_snapshot=peer_snapshot,
                evidence=evidence, state=state, config=config,
            )
            # The strategy uses MCL field yaw, whereas native heading hold uses
            # its IMU yaw. Translate the error; never send an MCL angle directly.
            native_target = imu_yaw + angle_error(rotation, field_yaw) if pose_ok else imu_yaw
            hardware.move(direction, speed, native_target, 0.5 if pose_ok else 0.0,
                          dribbler, kick=kick)
            peer.send(state.outgoing)
            changed = state.phase != last_phase
            if changed:
                print(f"[{state.role}] {state.phase}", flush=True)
                last_phase = state.phase
            if recorder and (new_frame or changed or kick):
                recorder.record({"type": "passing", "command": [direction, speed, rotation, kick, dribbler],
                                 "packet": state.outgoing, "peer": peer_snapshot, "evidence": evidence})
            time.sleep(max(0, 0.02 - (time.monotonic() - tick)))
    except KeyboardInterrupt:
        print("\nStopping passing test.", flush=True)
    finally:
        # Attempt every cleanup even if one device raises. Stop the motors first.
        closers = [("motors", hardware.stop if hardware else None),
                   ("camera", camera.stop if camera else None),
                   ("peer", peer.stop if peer else None),
                   ("break-beam", beam.switch.deinit if beam else None),
                   ("motion recorder", lambda: close_motion_capture(lidar)),
                   ("lidar", lidar.shutdown if lidar_started else None),
                   ("pass recorder", recorder.close if recorder else None)]
        for name, close in closers:
            if close is not None:
                try:
                    close()
                except Exception as exc:
                    print(f"Failed to close {name}: {exc}", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session", required=True, help="Same exercise name on both bots")
    parser.add_argument("--port", type=int, default=5006, help="Dedicated test UDP port")
    parser.add_argument("--startup-heading", type=float, default=0,
                        help="Initial field heading: 0 toward +X, 90 toward +Y")
    parser.add_argument("--match-mm", type=float, default=100, help="Camera/communicated position tolerance")
    parser.add_argument("--lane-clearance-mm", type=float, default=PassConfig().lane_clearance_mm, help="Other bot CENTRE clearance (default: 500 mm)")
    parser.add_argument("--lane-margin-mm", type=float, default=30, help="Extra uncertainty allowance")
    parser.add_argument("--fusion-config", help="Measured FusionConfig JSON overrides")
    parser.add_argument("--camera-stream", action="store_true")
    parser.add_argument("--use-pcb", action=argparse.BooleanOptionalAction, default=True,
                        help="Kick via the PCB (default); --no-use-pcb uses configured GPIO")
    parser.add_argument("--record", help="New diagnostic JSONL path")
    parser.add_argument("--duration", type=float, help="Optional runtime limit in seconds")
    args = parser.parse_args()
    try:
        config = PassControllerConfig(match_mm=args.match_mm, lane_clearance_mm=args.lane_clearance_mm,
                            lane_margin_mm=args.lane_margin_mm)
        run_passing_test(session=args.session, port=args.port, startup_heading=args.startup_heading,
                         pass_config=config, fusion_config_path=args.fusion_config,
                         duration=args.duration, camera_stream=args.camera_stream,
                         record_path=args.record, use_pcb=args.use_pcb)
    except ValueError as exc:
        parser.error(str(exc))


if __name__ == "__main__":
    main()
