import argparse
import math
import select
import sys
import threading
import time
from collections import deque

from lib.hardware_controller import (
    HardwareController,
    MotorCommunicationError,
    StatusDisplay,
)

import defence
import striker
from lib import lidar, switch
from lib.break_beam import Breakbeam
from lib.camera import Camera
from lib.communication import Peer
from lib.config import BotMode, load_config
from lib.game_status import GameStatus, ImuPause
from lib.recording_session import RecordingSession

LOG_FPS = 30 # How often the bot state is written to the log file
FPS_REPORT_INTERVAL = 1.0 # seconds; how often the FPS is printed to the console when --fps is used
PEER_PORT = 5005 # Port for bot to bot communication.
ENABLE_COMMUNICATION = False # Use lib/communication.py to communicate between bots.
USE_PAUSE = True # Whether to pause the bot when the pause switch is pressed. Set to False for debugging.

WHEEL_DIAMETER = 50 # mm; used to convert between motor RPM and robot mm/s
CONSTANT_SPEED_TORQUE = 8.0  # Amps; The current limit when the bot is at a constant speed.
ACCELERATION_TORQUE = 8.0  # Amps; The current limit when the bot is accelerating.
MAX_YAW_RPM = 100 # Maximum rpm that can be added or subtracted from the wheel speeds to correct yaw

LIDAR_BAUDRATE = 460800

MAX_MOTOR_RPM = 1000  # This converts to a maximum linear translation of ~2618 mm/s with 50 mm wheels; driver hardware max is ~1984 RPM (~5194 mm/s)
YAW_CORRECT_THRESHOLD = 3 # deg; threshold of allowable yaw error.

CAMERA_PORT = 8000 # Port used for streaming the camera feed for debugging.
CAMERA_RESOLUTION = (640, 640)
CAMERA_FPS = 90
ENABLE_GOAL_DETECTION = True  # False skips goal contours and always reports lined_up=True.
# Camera-detected bots within this distance of self or the UDP teammate are
# treated as self/friendly rather than enemies.
CAMERA_BOT_MATCH_MM = 100

BALL_TIMEOUT = 0.5 # seconds; maximum time for which the ball position can be extrapolated from velocity without assuming 'lost' state.
STARTUP_YAW_SAMPLE_COUNT = 25
STARTUP_YAW_SAMPLE_INTERVAL = 0.02

SWITCHM = 2

def _distance_mm(ax, ay, bx, by):
    return math.hypot(ax - bx, ay - by)


def classify_camera_bot_positions(
    camera_bot_positions,
    self_xy,
    peer_xy=None,
    match_mm=CAMERA_BOT_MATCH_MM,
):
    """Split camera-projected bots into friendly matches vs enemies.

    Detections within ``match_mm`` of this robot are discarded (self-view).
    Detections within ``match_mm`` of the UDP teammate are treated as friendly
    matches (peer position remains authoritative). Everything else is enemy.
    """
    friendly_matches = []
    enemy_bot_positions = []
    self_x, self_y = self_xy
    for bot_x, bot_y in camera_bot_positions:
        if _distance_mm(bot_x, bot_y, self_x, self_y) <= match_mm:
            continue
        if peer_xy is not None and _distance_mm(bot_x, bot_y, peer_xy[0], peer_xy[1]) <= match_mm:
            friendly_matches.append((bot_x, bot_y))
            continue
        enemy_bot_positions.append((bot_x, bot_y))
    return friendly_matches, enemy_bot_positions


# Load the config into the global variables
config = load_config()
I2C_ADDRESSES = config.i2c_addresses
LIDAR_PORT = config.lidar_port
MODE_SWITCH_OFF = config.mode_switch_off
MODE_SWITCH_ON = config.mode_switch_on
MODE_SWITCH_PIN = config.mode_switch_pin
PAUSE_SWITCH_PIN = config.pause_switch_pin
KICKER_PIN = config.kicker_pin
BREAK_BEAM_PIN = config.break_beam_pin

# Hardware switches are initialized after the status display is available.
bot_mode = MODE_SWITCH_OFF

run = not USE_PAUSE

parser = argparse.ArgumentParser(
    description="Run defence controller with optional live streaming and log recording."
)
parser.add_argument(
    "-s",
    "--stream",
    action="store_true",
    help="Enable websocket live log streaming for simulate.py --connect.",
)
parser.add_argument(
    "--camera-stream",
    action="store_true",
    help="Enable the camera MJPEG HTTP preview (adds JPEG encode + overlay draw cost).",
)
parser.add_argument(
    "-l",
    "--save-log",
    metavar="PATH",
    help=f"Write game log lines to PATH at {LOG_FPS} FPS for playback with simulate.py.",
)
parser.add_argument(
    "--record-session",
    metavar="DIRECTORY",
    help=(
        "Record synchronized video, controller state, and model detections in DIRECTORY."
    ),
)
parser.add_argument(
    "--fps",
    action="store_true",
    help="Print logic-loop and background-thread rates once per second.",
)
args = parser.parse_args()
if args.record_session is not None and args.camera_stream:
    parser.error("--record-session cannot be combined with --camera-stream")


# Used to debug speeds of various parts of the code.
class FpsMonitor:
    """Sample monotonic counters and print Hz once per reporting interval."""

    def __init__(self, interval_s=FPS_REPORT_INTERVAL):
        self._interval_s = interval_s
        self._sources: list[tuple[str, object]] = []
        self._last_counts: dict[str, int] = {}
        self._last_t = time.monotonic()

    def add(self, name: str, getter) -> None:
        self._sources.append((name, getter))
        self._last_counts[name] = int(getter())

    def maybe_print(self) -> None:
        now = time.monotonic()
        elapsed = now - self._last_t
        if elapsed < self._interval_s:
            return
        parts = []
        for name, getter in self._sources:
            count = int(getter())
            rate = (count - self._last_counts[name]) / elapsed
            self._last_counts[name] = count
            parts.append(f"{name}={rate:.1f}")
        self._last_t = now
        print("FPS " + " ".join(parts), flush=True)


if args.stream:
    from lib import send_log
    # Start websocket log server in the background (it runs its own asyncio loop).
    send_log.start_server_background()
    time.sleep(0.05)

status = None
log_recorder_stop = threading.Event()
log_recorder_thread = None
_latest_log_line = None
_latest_log_lock = threading.Lock()
_log_write_count = 0
_log_error = None
_log_write_count_lock = threading.Lock()
_logic_loop_count = 0


def update_latest_log_snapshot(log_line: str) -> None:
    global _latest_log_line
    with _latest_log_lock:
        _latest_log_line = log_line


def _log_recorder_loop(path: str) -> None:
    global _log_error, _log_write_count
    interval = 1.0 / LOG_FPS
    next_write = time.monotonic()
    try:
        with open(path, "w", encoding="utf-8") as log_file_handle:
            while not log_recorder_stop.is_set():
                now = time.monotonic()
                if now < next_write:
                    time.sleep(min(next_write - now, 0.005))
                    continue
                next_write += interval
                if next_write < now:
                    # Avoid catch-up bursts after a stall.
                    next_write = now + interval
                with _latest_log_lock:
                    line = _latest_log_line
                if line is not None:
                    log_file_handle.write(line + "\n")
                    log_file_handle.flush()
                    with _log_write_count_lock:
                        _log_write_count += 1
    except Exception as exc:
        print(f"Warning: log recorder stopped with error: {exc}")
        _log_error = str(exc)
        if status is not None:
            status.report("LOG", exc)


def get_log_write_count() -> int:
    with _log_write_count_lock:
        return _log_write_count


if args.save_log is not None:
    log_recorder_thread = threading.Thread(
        target=_log_recorder_loop,
        args=(args.save_log,),
        daemon=True,
        name="log-recorder",
    )
    log_recorder_thread.start()
    print(f"Saving game log to {args.save_log} at {LOG_FPS} FPS")

camera = None
hardware_controller = None
peer = None
recording_session = None
last_pose_time = None
display = None
startup_stage = "OTHER"

def enter_pressed():
    if not sys.stdin.isatty():
        return False

    ready, _, _ = select.select([sys.stdin], [], [], 0)
    if not ready:
        return False

    sys.stdin.readline()
    return True


class RollingYawSampler:
    """Circularly average the latest complete window of yaw samples."""

    def __init__(self, sample_count=STARTUP_YAW_SAMPLE_COUNT):
        self._samples = deque(maxlen=sample_count)

    def reset(self):
        self._samples.clear()

    def add(self, yaw):
        if yaw is None:
            self.reset()
            return None
        self._samples.append(float(yaw))
        if len(self._samples) < self._samples.maxlen:
            return None
        sin_sum = sum(math.sin(math.radians(sample)) for sample in self._samples)
        cos_sum = sum(math.cos(math.radians(sample)) for sample in self._samples)
        return math.degrees(math.atan2(sin_sum, cos_sum))


def capture_startup_yaw(
    imu_sensor,
    sample_count=STARTUP_YAW_SAMPLE_COUNT,
    sample_interval=STARTUP_YAW_SAMPLE_INTERVAL,
):
    """Average a short burst of IMU samples so startup yaw is not just the first reading."""
    print("Stabilizing IMU yaw reference...")
    sampler = RollingYawSampler(sample_count)
    last_count = -1
    last_generation = -1
    while True:
        if enter_pressed():
            raise KeyboardInterrupt
        count = imu_sensor.imu_update_count
        health = imu_sensor.health()
        generation = health["imu_recovery_generation"]
        if generation != last_generation or not health["imu_healthy"]:
            sampler.reset()
        last_generation = generation
        if health["imu_healthy"] and count != last_count:
            startup_yaw = sampler.add(imu_sensor.get_raw_imu_yaw())
            if startup_yaw is not None:
                return startup_yaw
        last_count = count
        time.sleep(sample_interval)


def feed_imu_yaw_prior(imu_sensor):
    """Push startup-relative IMU yaw into MCL as a soft heading prior."""
    imu_yaw = imu_sensor.get_yaw()
    health = imu_sensor.health()
    if imu_yaw is None or not health["imu_healthy"]:
        lidar.clear_imu_yaw()
        return
    lidar.set_imu_yaw(imu_yaw)


try:
    try:
        display = StatusDisplay()
    except Exception as exc:
        print(f"Warning: display unavailable: {exc}")
    status = GameStatus(display, lidar, bot_mode.name)
    status.start()
    if _log_error is not None:
        status.report("LOG", _log_error)
    mode_switch = switch.Switch(MODE_SWITCH_PIN)
    pause_switch = switch.Switch(PAUSE_SWITCH_PIN)
    bot_mode = MODE_SWITCH_ON if mode_switch.read() else MODE_SWITCH_OFF
    status.update(bot_mode.name, False, "STARTING", "LIDAR")
    break_beam = Breakbeam(BREAK_BEAM_PIN)
    startup_stage = "LIDAR"
    print(f"Initializing LIDAR on {LIDAR_PORT} at {LIDAR_BAUDRATE} baud...")
    try:
        lidar.init(LIDAR_PORT, LIDAR_BAUDRATE)
    except Exception as e:
        raise RuntimeError(f"Failed to initialize LIDAR: {e}")

    print("LIDAR initialized successfully!")
    print()

    print("Waiting for first scan data...")
    while not lidar.is_scan_ready():
        if enter_pressed():
            print("Shutdown requested, exiting.")
            raise KeyboardInterrupt
        time.sleep(0.1)

    startup_stage = "HARDWARE"
    status.update(bot_mode.name, False, "STARTING", "MOTORS / IMU")
    print(f"Initializing Hardware Controller with motor I2C addresses {I2C_ADDRESSES}...")
    hardware_controller = HardwareController.from_i2c_addresses(
        I2C_ADDRESSES,
        WHEEL_DIAMETER,
        MAX_YAW_RPM,
        MAX_MOTOR_RPM,
        YAW_CORRECT_THRESHOLD,
        kicker_pin=int(KICKER_PIN.id),
        display=display,
    )
    status.attach_hardware(hardware_controller)
    hardware_controller.set_drive_current_limits(CONSTANT_SPEED_TORQUE, ACCELERATION_TORQUE)
    status.update(bot_mode.name, False, "STARTING", "IMU REFERENCE")
    startup_yaw = capture_startup_yaw(hardware_controller)
    hardware_controller.set_startup_yaw(startup_yaw)
    print(f"Startup yaw reference set to {startup_yaw:.6f} deg")
    feed_imu_yaw_prior(hardware_controller)

    lidar.start_coordinates(2430, 1820)

    startup_stage = "LIDAR"
    status.update(bot_mode.name, False, "STARTING", "FIRST POSE")
    print("Waiting for first pose estimate...")
    last_wait_time = time.monotonic()
    while not lidar.is_coordinates_ready():
        if enter_pressed():
            print("Shutdown requested, exiting.")
            raise KeyboardInterrupt
        feed_imu_yaw_prior(hardware_controller)
        now = time.monotonic()
        omega = hardware_controller.get_gyro_z_deg_s()
        lidar.predict_odometry(0, 0, omega if omega is not None else 0.0, now - last_wait_time)
        last_wait_time = now
        time.sleep(0.1)

    if args.record_session is not None:
        recording_session = RecordingSession(
            args.record_session,
            resolution=CAMERA_RESOLUTION,
            requested_fps=CAMERA_FPS,
        )
        status.attach_recording(recording_session)
        recording_session.update_metadata(
            {
                "bot_mode_at_start": bot_mode.name,
                "performance_note": (
                    "Use --fps with and without --record-session to compare camera_cap, "
                    "camera_infer, and logic rates on this robot."
                ),
            }
        )
        print(f"Recording synchronized session to {recording_session.directory}")

    startup_stage = "CAMERA"
    status.update(bot_mode.name, False, "STARTING", "CAMERA")
    camera = Camera(
        CAMERA_PORT,
        resolution=CAMERA_RESOLUTION,
        frame_rate=CAMERA_FPS,
        enable_goal_detection=ENABLE_GOAL_DETECTION,
        recording_path=(
            recording_session.video_path if recording_session is not None else None
        ),
        session_epoch_monotonic=(
            recording_session.epoch_monotonic
            if recording_session is not None
            else None
        ),
        detection_callback=(
            recording_session.record_detection
            if recording_session is not None
            else None
        ),
    )
    if args.camera_stream:
        camera.start_stream()
    else:
        camera.start()
        print("Camera preview disabled (pass --camera-stream to enable MJPEG stream)")

    status.attach_camera(camera)
    startup_stage = "OTHER"
    if ENABLE_COMMUNICATION:
        peer = Peer(port=PEER_PORT)
        peer.start()
        print(f"Peer communication started on UDP port {PEER_PORT} (bot_id={peer.bot_id})")
    print("Press Enter to shut down.")
    steering_state = False

    ball_dx = 0
    ball_dy = 0
    last_ball_update = time.time()
    last_ball_x = None
    last_ball_y = None
    last_camera_frame_id = camera.frame_id
    last_camera_bot_positions = []
    last_pose_time = time.monotonic()

    fps_monitor = None
    if args.fps:
        fps_monitor = FpsMonitor()
        fps_monitor.add("logic", lambda: _logic_loop_count)
        fps_monitor.add("camera_cap", lambda: camera.capture_count)
        fps_monitor.add("camera_infer", lambda: camera.infer_count)
        fps_monitor.add("drive", lambda: hardware_controller.loop_count)
        fps_monitor.add("imu", lambda: hardware_controller.imu_update_count)
        if hasattr(lidar, "get_scan_generation"):
            fps_monitor.add("lidar_scan", lidar.get_scan_generation)
        if hasattr(lidar, "get_mcl_update_count"):
            fps_monitor.add("lidar_mcl", lidar.get_mcl_update_count)
        if peer is not None:
            fps_monitor.add("peer_rx", lambda: peer.receive_count)
        if log_recorder_thread is not None:
            fps_monitor.add("log", get_log_write_count)
        if recording_session is not None:
            fps_monitor.add("session_game", lambda: recording_session.game_writer.written)
            fps_monitor.add(
                "session_detection",
                lambda: recording_session.detection_writer.written,
            )
        print("FPS monitoring enabled")

    paused_yaw_sampler = RollingYawSampler()
    next_paused_yaw_sample_time = time.monotonic()
    last_paused_imu_count = -1
    last_imu_recovery_generation = -1
    next_recording_checkpoint_time = time.monotonic()

    pause_was_pressed = False
    requested_run = run
    imu_pause = ImuPause(hardware_controller.health()["imu_recovery_generation"])
    while True:
        if enter_pressed():
            break
        if recording_session is not None:
            checkpoint_time = time.monotonic()
            if checkpoint_time >= next_recording_checkpoint_time:
                recording_session.checkpoint(camera.recording_info)
                next_recording_checkpoint_time = checkpoint_time + 1.0
        if fps_monitor is not None:
            fps_monitor.maybe_print()
        was_run = run
        was_requested_run = requested_run
        pause_pressed = pause_switch.read() if USE_PAUSE else False
        if USE_PAUSE:
            if SWITCHM == 1 and pause_was_pressed and not pause_pressed:
                requested_run = not requested_run
                last_pose_time = time.monotonic()
            elif SWITCHM == 2:
                requested_run = pause_pressed
        pause_was_pressed = pause_pressed
        health = hardware_controller.health()
        run = imu_pause.update(requested_run, health)
        status.report("IMU RECOVERY", "PAUSE TO RE-ZERO" if (
            imu_pause.pending and health["imu_healthy"]) else "")
        if not run:
            bot_mode = MODE_SWITCH_ON if mode_switch.read() else MODE_SWITCH_OFF
        status.update(bot_mode.name, run)
        if (was_run and not run) or (was_requested_run and not requested_run):
            paused_yaw_sampler.reset()
            next_paused_yaw_sample_time = time.monotonic()
        if not run:
            steering_state = False
            now = time.monotonic()
            health = hardware_controller.health()
            generation = health["imu_recovery_generation"]
            if generation != last_imu_recovery_generation or not health["imu_healthy"]:
                paused_yaw_sampler.reset()
            last_imu_recovery_generation = generation
            if not health["imu_healthy"]:
                lidar.clear_imu_yaw()
            if now >= next_paused_yaw_sample_time:
                count = hardware_controller.imu_update_count
                if health["imu_healthy"] and count != last_paused_imu_count:
                    sampled_yaw = paused_yaw_sampler.add(hardware_controller.get_raw_imu_yaw())
                    if sampled_yaw is not None:
                        startup_yaw = sampled_yaw
                        hardware_controller.set_startup_yaw(startup_yaw)
                        imu_pause.rezeroed(requested_run, hardware_controller.health())
                        feed_imu_yaw_prior(hardware_controller)
                last_paused_imu_count = count
                next_paused_yaw_sample_time = now + STARTUP_YAW_SAMPLE_INTERVAL
        if run:
            _logic_loop_count += 1

            now_pose = time.monotonic()
            dt_pose = now_pose - last_pose_time
            last_pose_time = now_pose
            gyro_z = hardware_controller.get_gyro_z_deg_s()
            omega = gyro_z if gyro_z is not None else 0.0
            yaw = hardware_controller.get_yaw()
            feed_imu_yaw_prior(hardware_controller)
            vx, vy = 0.0, 0.0
            if hardware_controller is not None:
                yaw_for_odom = yaw if yaw is not None else 0.0
                # Fused-pose speed agreement is diagnostic, not a velocity scale.
                vx, vy = hardware_controller.get_measured_body_velocity_mm_s(
                    yaw_for_odom
                )
            lidar.predict_odometry(vx, vy, omega, dt_pose)

            x_pos, y_pos, _mcl_yaw, _confidence = lidar.get_pose()
            if x_pos is None or y_pos is None or yaw is None:
                hardware_controller.move(0, 0, 0, 0, 0)
                status.update(bot_mode.name, run, "BLOCKED", "WAITING FOR POSE")
                time.sleep(0.01)
                continue

            (
                camera_frame_id,
                ball_direction,
                ball_distance,
                bot_measurements,
                lined_up,
            ) = camera.get_scene_measurement()
            camera_healthy = status.camera_ready
            if not camera_healthy:
                ball_direction = ball_distance = None
                bot_measurements = []
                lined_up = False
                last_camera_bot_positions = []
            has_new_camera_frame = camera_healthy and camera_frame_id != last_camera_frame_id
            last_camera_frame_id = camera_frame_id
            if has_new_camera_frame:
                ball_global_direction = (
                    yaw + ball_direction if ball_direction is not None else None
                )
                ball_x = x_pos + ball_distance * math.cos(math.radians(ball_global_direction)) if ball_distance is not None and ball_global_direction is not None else None
                ball_y = y_pos + ball_distance * math.sin(math.radians(ball_global_direction)) if ball_distance is not None and ball_global_direction is not None else None
                camera_bot_positions = []
                for bot_bearing, bot_distance in bot_measurements:
                    if bot_bearing is None or bot_distance is None:
                        continue
                    bot_global_direction = yaw + bot_bearing
                    camera_bot_positions.append(
                        (
                            x_pos
                            + bot_distance * math.cos(math.radians(bot_global_direction)),
                            y_pos
                            + bot_distance * math.sin(math.radians(bot_global_direction)),
                        )
                    )
                last_camera_bot_positions = camera_bot_positions
            else:
                ball_x = None
                ball_y = None
                camera_bot_positions = last_camera_bot_positions
            now = time.time()
            if ball_x is not None and ball_y is not None:
                dt = now - last_ball_update
                if last_ball_x is not None and last_ball_y is not None and dt > 0:
                    ball_dx = (ball_x - last_ball_x) / dt
                    ball_dy = (ball_y - last_ball_y) / dt
                last_ball_x = ball_x
                last_ball_y = ball_y
                last_ball_update = now
            elif (
                last_ball_x is not None
                and last_ball_y is not None
                and now - last_ball_update < BALL_TIMEOUT
            ):
                dt_lost = now - last_ball_update
                ball_x = last_ball_x + ball_dx * dt_lost
                ball_y = last_ball_y + ball_dy * dt_lost
            else:
                ball_x = None
                ball_y = None
            if break_beam.read():
                ball_captured = True
                ball_x = x_pos + 100 * math.cos(math.radians(yaw))
                ball_y = y_pos + 100 * math.sin(math.radians(yaw))
            else:
                ball_captured = False

            peer_msg = None
            if peer is not None:
                peer.send(
                    {
                        "x": x_pos,
                        "y": y_pos,
                        "yaw": yaw,
                        "mode": bot_mode.name,
                        "ball_x": ball_x,
                        "ball_y": ball_y,
                    }
                )
                peer_msg = peer.receive()
            peer_xy = None
            if (
                peer_msg is not None
                and peer_msg.get("x") is not None
                and peer_msg.get("y") is not None
            ):
                peer_xy = (peer_msg["x"], peer_msg["y"])
                friendly_bot_positions = [peer_xy]
            else:
                friendly_bot_positions = []
            _friendly_matches, enemy_bot_positions = classify_camera_bot_positions(
                camera_bot_positions,
                (x_pos, y_pos),
                peer_xy=peer_xy,
            )
            if (
                ball_x is None
                and ball_y is None
                and peer_msg is not None
                and peer_msg.get("ball_x") is not None
                and peer_msg.get("ball_y") is not None
            ):
                ball_x = peer_msg["ball_x"]
                ball_y = peer_msg["ball_y"]

            if bot_mode == BotMode.DEFENCE:
                direction, speed, rotation, steering_state, kick, dribbler = defence.defence(
                    x_pos,
                    y_pos,
                    yaw,
                    ball_x,
                    ball_y,
                    ball_captured,
                    steering_state=steering_state,
                    friendly_bot_positions=friendly_bot_positions,
                    enemy_bot_positions=enemy_bot_positions,
                    lined_up=lined_up,
                )
            elif bot_mode == BotMode.STRIKER:
                direction, speed, rotation, steering_state, kick, dribbler = striker.striker(
                    x_pos,
                    y_pos,
                    yaw,
                    ball_x,
                    ball_y,
                    ball_captured,
                    steering_state=steering_state,
                    friendly_bot_positions=friendly_bot_positions,
                    enemy_bot_positions=enemy_bot_positions,
                    lined_up=lined_up,
                )
            elif bot_mode == BotMode.GOALIE:
                direction, speed, rotation, kick, dribbler = defence.goalie(
                    x_pos,
                    y_pos,
                    yaw,
                    ball_x,
                    ball_y,
                    ball_captured,
                    friendly_bot_positions=friendly_bot_positions,
                    enemy_bot_positions=enemy_bot_positions,
                    lined_up=lined_up,
                )
                steering_state = False
            if (
                args.stream
                or log_recorder_thread is not None
                or recording_session is not None
            ):
                log_values = [
                    x_pos,
                    y_pos,
                    yaw,
                    ball_x,
                    ball_y,
                    ball_captured,
                    bot_mode.name,
                    steering_state,
                    direction,
                    speed,
                    rotation,
                    kick,
                    dribbler,
                ]
                other_bot_positions = friendly_bot_positions + enemy_bot_positions
                log_line = ",".join(
                    "None" if value is None else str(value)
                    for value in (
                        *log_values,
                        *(coordinate for position in other_bot_positions for coordinate in position),
                    )
                )
                if args.stream:
                    send_log.update_latest_log(log_line)
                if log_recorder_thread is not None:
                    update_latest_log_snapshot(log_line)
                if recording_session is not None:
                    recording_session.record_game(
                        log_values, other_bots=other_bot_positions
                    )
            try:
                hardware_controller.move(direction, speed, rotation, 1.0, dribbler, kick=kick)
            except MotorCommunicationError as exc:
                print(exc)
                raise
        else:
            last_pose_time = time.monotonic()
            time.sleep(0.01)
            if hardware_controller is not None:
                hardware_controller.move(0, 0, 0, 0, 0)

except KeyboardInterrupt:
    pass
except Exception as exc:
    if status is not None:
        # Native constructor failures already publish MOTOR/IMU details directly.
        source = startup_stage if startup_stage != "HARDWARE" else "OTHER"
        status.report(source, exc)
        status.update(bot_mode.name, run, "BLOCKED", "SHUTTING DOWN")
        try:
            status.poll()
        except Exception as status_error:
            print(f"Warning: failed to publish final status: {status_error}")
    raise
finally:
    if status is not None:
        status.stop()
    if log_recorder_thread is not None:
        log_recorder_stop.set()
        log_recorder_thread.join(timeout=1.0)
    if peer is not None:
        try:
            peer.stop()
        except Exception as exc:
            print(f"Warning: failed to stop peer communication cleanly: {exc}")
            if status is not None:
                status.report("OTHER", exc)
    if hardware_controller is not None:
        try:
            hardware_controller.stop()
        except Exception as exc:
            print(f"Warning: failed to stop hardware cleanly: {exc}")
            if status is not None:
                status.report("OTHER", exc)
    if camera is not None:
        try:
            camera.stop()
        except Exception as exc:
            print(f"Warning: failed to stop camera cleanly: {exc}")
            if status is not None:
                status.report("CAMERA", exc)
    if recording_session is not None:
        try:
            if camera is not None:
                recording_session.update_metadata(camera.recording_info)
            recording_session.close()
            dropped_game = recording_session.game_writer.dropped
            dropped_detection = recording_session.detection_writer.dropped
            print(
                "Session saved "
                f"(dropped game rows={dropped_game}, "
                f"detection rows={dropped_detection})"
            )
        except Exception as exc:
            print(f"Warning: failed to finalize recording session cleanly: {exc}")
            if status is not None:
                status.report("RECORDING", exc)
    try:
        lidar.shutdown()
    except Exception as exc:
        print(f"Warning: failed to shut down lidar cleanly: {exc}")
        if status is not None:
            status.report("LIDAR", exc)

    if display is not None:
        display.stop()
