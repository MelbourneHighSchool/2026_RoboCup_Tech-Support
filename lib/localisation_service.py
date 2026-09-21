"""Reusable localisation prediction and diagnostics, with lazy hardware imports."""

import math
import time
from dataclasses import asdict, dataclass

from lib.line_sensors import LineSensorFeed
from lib.localisation_motion import (
    close_motion_capture,
    configure_motion,
    feed_timed_motion,
    record_floor,
)

USE_PCB = False


@dataclass(frozen=True)
class OdometryDiagnostics:
    """Values used for the latest MCL odometry prediction."""

    dt_s: float
    omega_deg_s: float
    yaw_deg: float
    wheel_vx: float
    wheel_vy: float
    lidar_vx: float
    lidar_vy: float
    lidar_fresh: bool
    trust: float
    fed_vx: float
    fed_vy: float
    has_wheel_odometry: bool


def global_velocity_to_body_mm_s(global_vx, global_vy, yaw_deg):
    yaw_rad = math.radians(yaw_deg)
    return (
        global_vx * math.cos(yaw_rad) + global_vy * math.sin(yaw_rad),
        global_vx * math.sin(yaw_rad) - global_vy * math.cos(yaw_rad),
    )


class LidarVelocityEstimator:
    """Estimate smoothed body velocity from successive confident LIDAR poses."""

    def __init__(self, smoothing=0.3):
        self.smoothing = smoothing
        self.last_x = self.last_y = self.last_time = None
        self.global_vx = self.global_vy = 0.0
        self.last_update_time = None

    def update(self, x, y, _yaw, now):
        if self.last_x is None:
            self.last_x, self.last_y, self.last_time = x, y, now
            return
        dt = now - self.last_time
        if dt < 0.02 or math.hypot(x - self.last_x, y - self.last_y) < 2.0:
            return
        alpha = self.smoothing
        self.global_vx = alpha * (x - self.last_x) / dt + (1 - alpha) * self.global_vx
        self.global_vy = alpha * (y - self.last_y) / dt + (1 - alpha) * self.global_vy
        self.last_x, self.last_y, self.last_time = x, y, now
        self.last_update_time = now

    def get_body_velocity(self, yaw_deg):
        return global_velocity_to_body_mm_s(self.global_vx, self.global_vy, yaw_deg)

    def is_fresh(self, now):
        return self.last_update_time is not None and now - self.last_update_time <= 0.15


def compute_wheel_odometry_trust(wheel_vx, wheel_vy, lidar_vx, lidar_vy, lidar_fresh):
    if not lidar_fresh:
        return 1.0
    wheel_speed = math.hypot(wheel_vx, wheel_vy)
    if wheel_speed < 50.0:
        return 1.0
    slip_ratio = abs(wheel_speed - math.hypot(lidar_vx, lidar_vy)) / wheel_speed
    return 0.0 if slip_ratio >= 0.5 else 1.0 - slip_ratio / 0.5


def capture_startup_yaw(imu, sample_count=25, sample_interval=0.02, cancel_event=None):
    """Average a short burst of IMU samples so startup yaw is not just the first reading."""
    print("Stabilizing IMU yaw reference...")
    sin_sum = 0.0
    cos_sum = 0.0
    samples = 0
    deadline = time.monotonic() + 5.0
    while samples < sample_count:
        if (cancel_event is not None and cancel_event.is_set()):
            raise InterruptedError("IMU startup cancelled")
        if time.monotonic() > deadline:
            raise TimeoutError("No IMU yaw received during startup")
        yaw = imu.get_raw_imu_yaw()
        if yaw is not None:
            yaw_rad = math.radians(yaw)
            sin_sum += math.sin(yaw_rad)
            cos_sum += math.cos(yaw_rad)
            samples += 1
        time.sleep(sample_interval)
    startup_yaw = math.degrees(math.atan2(sin_sum, cos_sum))
    imu.set_startup_yaw(startup_yaw)
    return startup_yaw


def get_yaw(imu, _startup_yaw, mcl_yaw):
    """Prefer MCL yaw when available, otherwise use startup-relative IMU yaw."""
    if mcl_yaw is not None:
        return mcl_yaw
    imu_yaw = imu.get_yaw()
    return imu_yaw


def get_position(lidar_module):
    """Return the latest confident (x, y, yaw) pose, or None if unavailable."""
    x_pos, y_pos, yaw, _confidence = lidar_module.get_pose()
    if x_pos is None or y_pos is None or yaw is None:
        return None
    return x_pos, y_pos, yaw


def feed_imu_yaw_prior(lidar_module, imu, _startup_yaw):
    """Push startup-relative IMU yaw into MCL as a soft heading prior."""
    imu_yaw = imu.get_yaw()
    if imu_yaw is None:
        return
    lidar_module.set_imu_yaw(imu_yaw)


def predict_odometry(
    lidar_module,
    movement_controller,
    imu,
    startup_yaw,
    lidar_velocity,
    yaw_deg,
    last_pose_time,
    *,
    apply_trust=False,
    line_feed=None,
    clock=time.monotonic,
):
    """Feed raw wheel and gyro measurements into MCL, retaining diagnostic trust.

    ``apply_trust=True`` restores legacy scaling for comparison tests only.
    Fused-pose velocity is not independent evidence of wheel slip.
    """
    now = clock()
    if movement_controller is None and hasattr(imu, "get_measured_body_velocity_mm_s"):
        movement_controller = imu
    dt = now - last_pose_time
    omega = 0.0
    gyro_z = imu.get_gyro_z_deg_s()
    if gyro_z is not None:
        omega = gyro_z
    timed_sample = None
    if hasattr(movement_controller, "get_localisation_sample") and hasattr(lidar_module, "feed_motion"):
        timed_sample = movement_controller.get_localisation_sample()
    else:
        feed_imu_yaw_prior(lidar_module, imu, startup_yaw)

    vx, vy = 0.0, 0.0
    vx_wheel, vy_wheel = 0.0, 0.0
    lidar_vx, lidar_vy = 0.0, 0.0
    lidar_fresh = False
    trust = 1.0
    has_wheel_odometry = movement_controller is not None and (timed_sample is not None or yaw_deg is not None)
    if has_wheel_odometry:
        if timed_sample is not None:
            vx_wheel, vy_wheel = timed_sample["vx"], timed_sample["vy"]
        else:
            vx_wheel, vy_wheel = movement_controller.get_measured_body_velocity_mm_s(yaw_deg)
        lidar_vx, lidar_vy = lidar_velocity.get_body_velocity(yaw_deg if yaw_deg is not None else 0)
        lidar_fresh = lidar_velocity.is_fresh(now)
        trust = compute_wheel_odometry_trust(
            vx_wheel,
            vy_wheel,
            lidar_vx,
            lidar_vy,
            lidar_fresh,
        )
        scale = trust if apply_trust else 1.0
        vx = scale * vx_wheel
        vy = scale * vy_wheel

    if timed_sample is not None:
        timed_sample = {**timed_sample, "vx": vx, "vy": vy}
        feed_timed_motion(lidar_module, movement_controller, sample=timed_sample)
    else:
        lidar_module.predict_odometry(vx, vy, omega, dt)
    if line_feed is not None:
        line_feed.update(lidar_module, imu)
        record_floor(lidar_module)
    diagnostics = OdometryDiagnostics(
        dt_s=dt,
        omega_deg_s=omega,
        yaw_deg=yaw_deg,
        wheel_vx=vx_wheel,
        wheel_vy=vy_wheel,
        lidar_vx=lidar_vx,
        lidar_vy=lidar_vy,
        lidar_fresh=lidar_fresh,
        trust=trust,
        fed_vx=vx,
        fed_vy=vy,
        has_wheel_odometry=has_wheel_odometry,
    )
    return now, diagnostics



class LocalisationSession:
    """One hardware owner calls tick at 50 Hz, even without a confident fix."""

    SENSOR_TIMEOUT_S = 0.5

    def __init__(self, lidar, imu, startup_yaw, velocity, clock=time.monotonic, *, line_feed=None, use_pcb=USE_PCB):
        configure_motion(lidar, use_pcb=use_pcb)
        self.lidar = lidar
        self.imu = imu
        self.startup_yaw = startup_yaw
        self.velocity = velocity
        self.clock = clock
        self.line_feed = (line_feed or LineSensorFeed(use_pcb=use_pcb)) if use_pcb else None
        self.last_time = clock()
        self.last_yaw = 0.0
        self.scan_generation = -1
        self.last_scan_time = self.last_time
        self.mcl_updates = -1
        self.last_mcl_time = self.last_time
        self.imu_count = -1
        self.last_imu_time = self.last_time
        self.state = {}

    def tick(self, controller=None):
        now = self.clock()
        self.last_time, odometry = predict_odometry(
            self.lidar, controller, self.imu, self.startup_yaw,
            self.velocity, self.last_yaw, self.last_time, line_feed=self.line_feed, clock=self.clock,
        )
        pose = self.lidar.get_coordinates_info()
        x, y, yaw, confidence, ok = pose
        generation = self.lidar.get_scan_generation()
        if generation != self.scan_generation:
            self.scan_generation = generation
            self.last_scan_time = now
        count = self.imu.imu_update_count
        if count != self.imu_count:
            self.imu_count = count
            self.last_imu_time = now
        if ok and all(v is not None for v in (x, y, yaw)):
            self.last_yaw = yaw
            self.velocity.update(x, y, yaw, now)
        updates = self.lidar.get_mcl_update_count()
        if updates != self.mcl_updates:
            self.mcl_updates = updates
            self.last_mcl_time = now
        scans_enabled = self.lidar.scan_updates_enabled()
        mcl_age = now - self.last_mcl_time
        fresh = (
            now - self.last_scan_time <= self.SENSOR_TIMEOUT_S
            and now - self.last_imu_time <= self.SENSOR_TIMEOUT_S
            and mcl_age <= self.SENSOR_TIMEOUT_S
        )
        self.state = {
            "pose": [x, y, yaw, confidence, bool(ok)],
            "fresh": fresh, "timestamp": now,
            "scan_age_s": now - self.last_scan_time,
            "imu_age_s": now - self.last_imu_time,
            "scan_generation": generation,
            "scan_count": self.lidar.get_scan_count(),
            "mcl_updates": updates,
            "mcl_age_s": mcl_age,
            "scan_updates_enabled": scans_enabled,
            "rotation_prediction": False,  # Retained for diagnostic consumers.
            "correction": list(self.lidar.get_last_scan_correction()),
            "recovery": list(self.lidar.get_recovery_status()),
            "odometry": asdict(odometry),
        }
        if hasattr(self.lidar, "get_deskew_status"):
            self.state["deskew"] = self.lidar.get_deskew_status()
        return self.state

    def close(self):
        close_motion_capture(self.lidar)
        try:
            self.imu.stop()
        finally:
            self.lidar.shutdown()
