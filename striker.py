import math
import time
from dataclasses import dataclass

WHEEL_DIAMETER = 50 # mm, used to convert mm/s to RPM
MAX_YAW_RPM = 100 # Maximum rpm that can be added or subtracted from the wheel speeds to correct yaw
# Coordinates of the centre of the goal zone, which the goalie uses for blocking.
CYAN_GOAL_CENTRE_X = 400
YELLOW_GOAL_CENTRE_X = 1980
# Y is shared between goals because it is the same
GOAL_CENTRE_Y = 910
# Coordinates of the back of the goal zone, which defence uses for aiming.
YELLOW_GOAL_BACK_X = 226
GOAL_BACK_Y_MIN = 700
GOAL_BACK_Y_MAX = 1125
CYAN_GOAL_BACK_X = 2204
# Enemy (cyan) goal mouth: side walls run from here back to CYAN_GOAL_BACK_X.
CYAN_GOAL_MOUTH_X = 2130
GOAL_SIDE_WALL_Y_MIN = 685
GOAL_SIDE_WALL_Y_MAX = 1135
GOAL_LINE_WIDTH = 10
MAX_MOTOR_RPM = 400 # Maximum rpm that the wheels can spin at
LIDAR_PORT = "/dev/ttyUSB0"
LIDAR_BAUDRATE = 460800

# Pitch boundary coordinates. Used to keep bot within the pitch.
WHITE_MIN_X = 250
WHITE_MAX_X = 2180
WHITE_MIN_Y = 250
WHITE_MAX_Y = 1570

# Short pitch axis (Y): used to pick which sideline to drive toward.
PITCH_WIDTH = 1820
# How close to the white sideline before switching from wall-drive to upfield.
BALL_HIDING_LINE_THRESHOLD = 150
# Set to False to disable the ball-hiding strategy.
BALL_HIDING_ENABLED = True
# Distance to goal (mm) at which ball hiding starts / ends.
BALL_HIDING_START_DIST = 900
BALL_HIDING_END_DIST = 600
# Distance to goal (mm) at which to aim and shoot when ball hiding is disabled.
BALL_HIDING_DISABLED_END_DIST = 1000

CLOSE_SHOOTING_Y_DIST = 100

BALL_RADIUS = 21 # mm, radius of the ball
ROBOT_RADIUS = 110 # mm, radius used for shot clearance around enemy bot centres
# Minimum angular clearance (deg) from the near goal side wall when banking off the far wall.
SIDE_WALL_CLEARANCE_DEG = 5
# When no shot/rebound is possible, pull this far toward own goal while drifting to mid Y.
SHOT_REPOSITION_PULL_X = 400

YAW_CORRECT_THRESHOLD = 3 # deg, threshold of allowable yaw error.
OWN_GOAL_PREVENTION_OFFSET = 10 # deg, how much to offset the direction to the ball when preventing own goals.

CAMERA_PORT = 8000
I2C_ADDRESSES = [29, 27, 26, 25]

BALL_TIMEOUT = 1 # seconds, time to extrapolate the ball position from velocity without assuming 'lost' state.


def wrap_angle_deg(angle):
    return ((angle + 180) % 360) - 180


def _angle_to(x0, y0, x1, y1):
    return math.degrees(math.atan2(y1 - y0, x1 - x0))


def _mouth_sector(ball_x, ball_y, mouth_x):
    """Near/far mouth angles and opposite inside-wall Y for a shot from (ball_x, ball_y)."""
    mouth_y_min = GOAL_SIDE_WALL_Y_MIN + BALL_RADIUS
    mouth_y_max = GOAL_SIDE_WALL_Y_MAX - BALL_RADIUS
    if ball_y >= (GOAL_SIDE_WALL_Y_MIN + GOAL_SIDE_WALL_Y_MAX) / 2:
        near_y, far_y = mouth_y_max, mouth_y_min
        opposite_y = GOAL_SIDE_WALL_Y_MIN + BALL_RADIUS
    else:
        near_y, far_y = mouth_y_min, mouth_y_max
        opposite_y = GOAL_SIDE_WALL_Y_MAX - BALL_RADIUS
    near_angle = _angle_to(ball_x, ball_y, mouth_x, near_y)
    far_angle = _angle_to(ball_x, ball_y, mouth_x, far_y)
    return near_angle, far_angle, opposite_y


def _ray_hit(ball_x, ball_y, angle_deg, *, wall_x=None, wall_y=None):
    """Intersection of a forward ray with x=wall_x or y=wall_y. Returns (x, y) or None."""
    rad = math.radians(angle_deg)
    cos_a, sin_a = math.cos(rad), math.sin(rad)
    if wall_x is not None:
        if abs(cos_a) < 1e-9:
            return None
        t = (wall_x - ball_x) / cos_a
        if t <= 0:
            return None
        return wall_x, ball_y + t * sin_a
    if abs(sin_a) < 1e-9:
        return None
    t = (wall_y - ball_y) / sin_a
    if t <= 0:
        return None
    return ball_x + t * cos_a, wall_y


def _clears_posts(ball_x, ball_y, angle_deg, mouth_x):
    """Ball body must stay clear of both mouth posts along the kick ray."""
    min_dist = BALL_RADIUS + GOAL_LINE_WIDTH / 2.0
    rad = math.radians(angle_deg)
    ux, uy = math.cos(rad), math.sin(rad)
    for post_y in (GOAL_SIDE_WALL_Y_MIN, GOAL_SIDE_WALL_Y_MAX):
        wx, wy = mouth_x - ball_x, post_y - ball_y
        proj = wx * ux + wy * uy
        dist = math.hypot(wx, wy) if proj < 0 else abs(wx * uy - wy * ux)
        if dist + 1e-9 < min_dist:
            return False
    return True


def _point_to_segment_distance(px, py, start, end):
    """Shortest distance from a point to a finite line segment."""
    dx, dy = end[0] - start[0], end[1] - start[1]
    length_sq = dx * dx + dy * dy
    if length_sq <= 1e-9:
        return math.hypot(px - start[0], py - start[1])
    t = ((px - start[0]) * dx + (py - start[1]) * dy) / length_sq
    t = max(0.0, min(1.0, t))
    return math.hypot(px - (start[0] + t * dx), py - (start[1] + t * dy))


def _scoring_path(ball_x, ball_y, angle_deg, target_x, mouth_x, mouth_sector):
    """Return direct/rebound ball-path segments when angle_deg scores, otherwise None."""
    near_angle, far_angle, opposite_y = mouth_sector
    span = wrap_angle_deg(far_angle - near_angle)
    from_near = wrap_angle_deg(angle_deg - near_angle)
    if abs(span) + 1e-9 < SIDE_WALL_CLEARANCE_DEG:
        return None
    if from_near * span <= 0:
        return None
    if abs(from_near) + 1e-9 < SIDE_WALL_CLEARANCE_DEG:
        return None
    if abs(from_near) > abs(span) + 1e-9:
        return None
    if not _clears_posts(ball_x, ball_y, angle_deg, mouth_x):
        return None

    start = (ball_x, ball_y)
    back_hit = _ray_hit(ball_x, ball_y, angle_deg, wall_x=target_x)
    side_hit = _ray_hit(ball_x, ball_y, angle_deg, wall_y=opposite_y)
    hits_side_first = (
        side_hit is not None
        and min(mouth_x, target_x) <= side_hit[0] <= max(mouth_x, target_x)
        and abs(side_hit[0] - ball_x) + 1e-9 < abs(target_x - ball_x)
    )
    if not hits_side_first:
        if back_hit is not None and GOAL_BACK_Y_MIN <= back_hit[1] <= GOAL_BACK_Y_MAX:
            return [(start, back_hit)]
        return None

    # A horizontal side-wall bounce reverses the Y component of the ball's direction.
    rebound_hit = _ray_hit(side_hit[0], side_hit[1], -angle_deg, wall_x=target_x)
    if rebound_hit is None or not GOAL_BACK_Y_MIN <= rebound_hit[1] <= GOAL_BACK_Y_MAX:
        return None
    return [(start, side_hit), (side_hit, rebound_hit)]


def kick_direction_scores(
    ball_x,
    ball_y,
    angle_deg,
    target_x,
    mouth_x,
    enemy_bot_positions=None,
    *,
    mouth_sector=None,
):
    """True when the kick scores and its complete path clears every enemy bot."""
    if mouth_sector is None:
        mouth_sector = _mouth_sector(ball_x, ball_y, mouth_x)
    path = _scoring_path(ball_x, ball_y, angle_deg, target_x, mouth_x, mouth_sector)
    if path is None:
        return False

    clearance = ROBOT_RADIUS + BALL_RADIUS
    for bot_x, bot_y, *_ in enemy_bot_positions or ():
        if any(
            _point_to_segment_distance(bot_x, bot_y, start, end) <= clearance
            for start, end in path
        ):
            return False
    return True


def goal_shot_aim(
    ball_x, ball_y, target_x, mouth_x, enemy_bot_positions=None, *, preferred_angle=None
):
    """Return (aim_angle_deg, True) for a back-wall or rebound shot, or (None, False)."""
    dx_back = target_x - ball_x
    if abs(dx_back) < 1e-6:
        return None, False

    mouth_y_min = GOAL_SIDE_WALL_Y_MIN + BALL_RADIUS
    mouth_y_max = GOAL_SIDE_WALL_Y_MAX - BALL_RADIUS
    aim_y_min, aim_y_max = GOAL_BACK_Y_MIN, GOAL_BACK_Y_MAX
    # Outside the mouth: clip the back-wall window to rays that pass through it.
    before_mouth = (dx_back > 0 and ball_x < mouth_x) or (dx_back < 0 and ball_x > mouth_x)
    if before_mouth and abs(mouth_x - ball_x) > 1e-6:
        scale = dx_back / (mouth_x - ball_x)
        y_lo = ball_y + (mouth_y_min - ball_y) * scale
        y_hi = ball_y + (mouth_y_max - ball_y) * scale
        visible_lo, visible_hi = min(y_lo, y_hi), max(y_lo, y_hi)
        aim_y_min = max(GOAL_BACK_Y_MIN, visible_lo)
        aim_y_max = min(GOAL_BACK_Y_MAX, visible_hi)

    candidates = [] if preferred_angle is None else [preferred_angle]
    if aim_y_min <= aim_y_max:
        # Prefer the centre, but try off-centre direct shots when a bot blocks it.
        for fraction in (0.5, 0.25, 0.75, 0.0, 1.0):
            aim_y = aim_y_min + (aim_y_max - aim_y_min) * fraction
            candidates.append(_angle_to(ball_x, ball_y, target_x, aim_y))

    # Rebound: opposite inside wall, as close to the back as possible, with near-wall clearance.
    mouth_sector = _mouth_sector(ball_x, ball_y, mouth_x)
    near_angle, far_angle, opposite_y = mouth_sector
    span = wrap_angle_deg(far_angle - near_angle)
    if abs(span) + 1e-9 >= SIDE_WALL_CLEARANCE_DEG:
        ideal = _angle_to(ball_x, ball_y, target_x, opposite_y)
        clear = wrap_angle_deg(near_angle + math.copysign(SIDE_WALL_CLEARANCE_DEG, span))
        from_near = wrap_angle_deg(ideal - near_angle)
        if from_near * span > 0 and abs(from_near) + 1e-9 >= SIDE_WALL_CLEARANCE_DEG:
            candidates.append(ideal if abs(from_near) <= abs(span) + 1e-9 else far_angle)
        else:
            candidates.append(clear)

    for aim in candidates:
        if kick_direction_scores(
            ball_x,
            ball_y,
            aim,
            target_x,
            mouth_x,
            enemy_bot_positions,
            mouth_sector=mouth_sector,
        ):
            return aim, True
    return None, False


SHOT_OPEN_HOLD_SECONDS = 0.4
SHOT_BLOCKED_HOLD_SECONDS = 0.3


@dataclass
class ShotState:
    """Per-robot shot commitment, separate from the logged ball-hiding flag."""

    aim: float | None = None
    blocked_since: float | None = None
    open_since: float | None = None
    repositioning: bool = False

    def reset(self):
        self.aim = None
        self.blocked_since = None
        self.open_since = None
        self.repositioning = False

    def select(self, candidate, now):
        # A newly preferred lane must not immediately reverse an existing aim.
        if self.aim is not None and candidate != self.aim:
            candidate = None
        if candidate is None:
            self.open_since = None
            if self.blocked_since is None:
                self.blocked_since = now
            if self.aim is None or now - self.blocked_since >= SHOT_BLOCKED_HOLD_SECONDS:
                self.aim = None
                self.repositioning = True
        else:
            self.blocked_since = None
            if self.repositioning:
                if self.open_since is None:
                    self.open_since = now
                if now - self.open_since < SHOT_OPEN_HOLD_SECONDS:
                    return None
            self.aim = candidate
            self.repositioning = False
            self.open_since = None
        return self.aim


def striker(
    x_pos,
    y_pos,
    yaw,
    ball_x,
    ball_y,
    ball_captured=False,
    steering_state=False,
    friendly_bot_positions=None,
    enemy_bot_positions=None,
    *,
    shot_state=None,
    now=None,
):
    """Striker strategy: approach ball, hide along sideline when far, then aim and kick."""
    if shot_state is None:
        shot_state = ShotState()
    if now is None:
        now = time.monotonic()
    if not ball_captured or ball_x is None or ball_y is None:
        shot_state.reset()
    if friendly_bot_positions is None:
        friendly_bot_positions = []
    if enemy_bot_positions is None:
        enemy_bot_positions = []
    # If the ball is not detected, the bot should move to the centre of the pitch.
    if ball_x is None or ball_y is None:
        if x_pos > 1415:
            target_x = 1315
            
        else:
            target_x = 1550
        target_y = 910
        vector = (target_x - x_pos), (target_y - y_pos)
        direction = math.degrees(math.atan2(vector[1], vector[0]))
        speed = 0
        rotation = 0
        kick = False
        dribbler = 1
        return direction, speed, rotation, False, kick, dribbler
    # Calculate the direction to the ball in vector form. Direction is relative to the bot's ideal heading (the direction towards the goal it should be scoring towards from the goal it is defending)
    vector = (ball_x - x_pos), (ball_y - y_pos)
    direction = math.degrees(math.atan2(vector[1], vector[0])) # Convert the vector to a direction in degrees, relative to the ideal heading.
    dist = math.sqrt(vector[0] ** 2 + vector[1] ** 2) # Calculate the distance to the ball.
    rotation = 0 # Sets the desired rotation. 0 is always the startup/ideal heading in this frame.
    speed = 500 # mm/s, Default speed of the bot.
    offset = 0 # deg, Offset to the direction to the ball. Used to avoid own goals.
    dribbler = 1 # Dribbler should be on by default
    # Skip approach offset while captured: ball is in front, so direction ≈ yaw and
    # ±80 would clear goal rotation and oscillate against facing forward (rotation=0).
    if not ball_captured and dist < 300:
        if -10 < direction < 10:
            speed = 700
        elif 0 < direction < 180:
            offset = 60
        else:
            offset = -60
    elif dist > 500:
        speed = 1200
        dribbler = 0
    elif ball_captured:
        speed = 1000

    # By default, the bot should not kick the ball.
    kick = False

    # steering_state persists ball-hiding across calls (hysteresis between START/END).
    ball_hiding = (
        BALL_HIDING_ENABLED and bool(steering_state) if ball_captured else False
    )

    # Only kick if the ball is captured and lined up with the goal.
    # Kicks ball into goal when it's close to the goal
    if ball_captured:
        target_x = CYAN_GOAL_BACK_X

        dist_to_goal = abs(target_x - ball_x)
        shooting_end_dist = (
            BALL_HIDING_END_DIST
            if BALL_HIDING_ENABLED
            else BALL_HIDING_DISABLED_END_DIST
        )
        # Back-wall shot, or rebound off the opposite side wall; None if neither is possible.
        degrees_to_goal, shot_possible = goal_shot_aim(
            x_pos, y_pos, target_x, CYAN_GOAL_MOUTH_X, enemy_bot_positions,
            preferred_angle=shot_state.aim,
        )

        # Enter hide when far; stay hidden until closer than END (no dead-zone flutter).
        if (
            BALL_HIDING_ENABLED
            and not ball_hiding
            and dist_to_goal >= BALL_HIDING_START_DIST
        ):
            ball_hiding = True
        elif ball_hiding and offset == 0 and dist_to_goal < BALL_HIDING_END_DIST:
            ball_hiding = False

        if ball_hiding:
            shot_state.reset()
            # Drive perpendicular to the goals toward a sideline until close enough to
            # aim. Keep dribbler on so losing the ball does not drop rotation to 0.
            offset = 0
            speed = 400
            if y_pos > PITCH_WIDTH / 2:
                rotation = 90
                near_line = y_pos >= WHITE_MAX_Y - BALL_HIDING_LINE_THRESHOLD
            else:
                rotation = 270
                near_line = y_pos <= WHITE_MIN_Y + BALL_HIDING_LINE_THRESHOLD
            # Once tucked against the sideline, advance upfield while still facing the wall.
            direction = 0 if near_line else rotation
        else:
            # Plan from the robot centre: rotating the captured ball must not
            # alternately open and close the lane. Validate the actual ball below.
            aim = shot_state.select(degrees_to_goal if shot_possible else None, now)
            if aim is None:
                rotation = 0
                direction = _angle_to(
                    x_pos, y_pos, x_pos - SHOT_REPOSITION_PULL_X, GOAL_CENTRE_Y
                )
                speed = 400
            elif offset == 0 and dist_to_goal < shooting_end_dist:
                speed = 0
                rotation = aim
                kick = kick_direction_scores(
                    ball_x, ball_y, yaw, target_x, CYAN_GOAL_MOUTH_X,
                    enemy_bot_positions,
                )
    if kick == True:
        dribbler = -1

    return direction + offset, speed, rotation, ball_hiding, kick, dribbler
