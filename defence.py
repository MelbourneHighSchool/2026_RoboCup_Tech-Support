import math
from dataclasses import dataclass

# Goalie tracks along this fixed X line, with bounded sideways travel.
GOALIE_BLOCK_X = 550
GOALIE_MAX_SPEED = 3000
GOALIE_BRAKING_ACCEL = 6000  # mm/s²; lower values start braking earlier.
GOALIE_STOP_DISTANCE = 10
GOALIE_BOX_Y_MIN = 460
GOALIE_BOX_Y_MAX = 1360
YELLOW_GOAL_CENTRE_X = 1980
# Y is shared between goals because it is the same
GOAL_CENTRE_Y = 910
# Coordinates of the back of the goal zone, which defence uses for aiming.
YELLOW_GOAL_BACK_X = 226
GOAL_BACK_Y_MIN = 700
GOAL_BACK_Y_MAX = 1125
CYAN_GOAL_BACK_X = 2204

# Pitch boundary coordinates. Used to keep bot within the pitch.
WHITE_MIN_X = 250
WHITE_MAX_X = 2180
WHITE_MIN_Y = 250
WHITE_MAX_Y = 1570

BALL_RADIUS = 21  # mm, radius of the ball
ROBOT_RADIUS = 110  # mm
BOUNDARY_STOP_MARGIN = 40  # mm reserved for braking/localisation error

# Wrap to the shortest signed angle in [-180, 180).
def wrap_angle_deg(angle):
    return ((angle + 180) % 360) - 180


def point_to_segment_distance(px, py, x1, y1, x2, y2):
    dx = x2 - x1
    dy = y2 - y1
    line_len_sq = dx * dx + dy * dy
    if line_len_sq <= 1e-9:
        return math.hypot(px - x1, py - y1)

    t = ((px - x1) * dx + (py - y1) * dy) / line_len_sq
    t = max(0.0, min(1.0, t))
    closest_x = x1 + t * dx
    closest_y = y1 + t * dy
    return math.hypot(px - closest_x, py - closest_y)

# Checks if the ball is outside the pitch by using the pitch boundaries, the ball position and the ball radius.
def is_ball_out(ball_x, ball_y):
    closest_x = max(WHITE_MIN_X, min(ball_x, WHITE_MAX_X))
    closest_y = max(WHITE_MIN_Y, min(ball_y, WHITE_MAX_Y))
    dx = ball_x - closest_x
    dy = ball_y - closest_y
    distance = math.hypot(dx, dy)
    return distance > BALL_RADIUS


def keep_motion_inside_white_lines(x_pos, y_pos, direction, speed):
    """Remove command components that would take the robot over a white line."""
    if direction is None or speed <= 0:
        return direction, speed

    min_x = WHITE_MIN_X + ROBOT_RADIUS
    max_x = WHITE_MAX_X - ROBOT_RADIUS
    min_y = WHITE_MIN_Y + ROBOT_RADIUS
    max_y = WHITE_MAX_Y - ROBOT_RADIUS
    stop_min_x = min_x + BOUNDARY_STOP_MARGIN
    stop_max_x = max_x - BOUNDARY_STOP_MARGIN
    stop_min_y = min_y + BOUNDARY_STOP_MARGIN
    stop_max_y = max_y - BOUNDARY_STOP_MARGIN

    direction_rad = math.radians(direction)
    velocity_x = speed * math.cos(direction_rad)
    velocity_y = speed * math.sin(direction_rad)

    if (x_pos <= stop_min_x and velocity_x < 0) or (
        x_pos >= stop_max_x and velocity_x > 0
    ):
        velocity_x = 0
    if (y_pos <= stop_min_y and velocity_y < 0) or (
        y_pos >= stop_max_y and velocity_y > 0
    ):
        velocity_y = 0

    guarded_speed = math.hypot(velocity_x, velocity_y)
    if guarded_speed <= 1e-9:
        return direction, 0
    return math.degrees(math.atan2(velocity_y, velocity_x)), guarded_speed

# Inputs:
# x_pos: x position of the robot
# y_pos: y position of the robot
# yaw: yaw value of the robot
# ball_x: x position of the ball
# ball_y: y position of the ball
# ball_captured: True when the ball is touching the capture zone
# state: controller-owned persistent state, or None on first call
# friendly_bot_positions: optional iterable of (x, y) positions for friendly robots
# enemy_bot_positions: optional iterable of (x, y) positions for enemy robots
# Outputs:
# direction: degrees to move in
# speed: mm/s to move at
# rotation: yaw value to rotate towards
# state: controller-owned state to pass into the next call.
# kick: True if the bot should kick the ball
@dataclass
class DefenceState:
    steering: bool = False


def defence(
    x_pos,
    y_pos,
    yaw,
    ball_x,
    ball_y,
    ball_captured=False,
    last_ball_y=None,
    state=None,
    friendly_bot_positions=None,
    enemy_bot_positions=None,
):
    if not isinstance(state, DefenceState):
        state = DefenceState()
    dribbler = 0 # Whether the dribbler should be on.
    if friendly_bot_positions is None:
        friendly_bot_positions = []
    if enemy_bot_positions is None:
        enemy_bot_positions = []
    # If the ball is not detected, the bot should move to the centre of the pitch.
    if ball_x is None or ball_y is None:
        
        target_x = 815
        if last_ball_y is not None:
            if last_ball_y < 850:
                target_y = 1210
            elif last_ball_y > 950:
                target_y = 610
            else:
                target_y = 910
        else:
            target_y = 910
        vector = (target_x - x_pos), (target_y - y_pos)
        direction = math.degrees(math.atan2(vector[1], vector[0]))
        dx, dy = target_x - x_pos, target_y - y_pos
        distance = math.hypot(dx, dy)
        if distance > 10:
            speed = 500
        else:
            speed = 0
        rotation = 0
        steering = False
        kick = False
        state.steering = steering
        return direction, speed, rotation, state, kick, dribbler
    steering = state.steering
    # Calculate the direction to the ball in vector form. Direction is relative to the bot's ideal heading (the direction towards the goal it should be scoring towards from the goal it is defending)
    vector = (ball_x - x_pos), (ball_y - y_pos)
    direction = math.degrees(math.atan2(vector[1], vector[0])) # Convert the vector to a direction in degrees, relative to the ideal heading.
    dist = math.sqrt(vector[0] ** 2 + vector[1] ** 2) # Calculate the distance to the ball.
    rotation = 0 # Sets the desired rotation. 0 is always the startup/ideal heading in this frame.
    speed = 500 # mm/s, Default speed of the bot.
    offset = 0 # deg, Offset to the direction to the ball. Used to avoid own goals.
    # Only activate own goal prevention if the ball is close to the bot.
    if dist < 400:
        if -10 < direction < 10:
            speed = 1000
            if steering and y_pos < 850 and dist < 200:
                offset = 40
            elif steering and y_pos > 1050 and dist < 200:
                offset = -40
            if y_pos < 800 and ball_captured:
                offset = 40
                steering = True
            elif y_pos > 1000 and ball_captured:
                offset = -40
                steering = True
            else:
                steering = False
        elif 0 < direction < 180:
            offset = 80
        else:
            offset = -80
    elif dist > 500:
        speed = 600

    # By default, the bot should not kick the ball.
    kick = False

    # Only kick if the ball is captured and lined up with the goal.
    if ball_captured:
        dribbler = 1

        target_x = CYAN_GOAL_BACK_X
        target_y_min = GOAL_BACK_Y_MIN
        target_y_max = GOAL_BACK_Y_MAX

        yaw_rad = math.radians(yaw % 360)
        dir_x = math.cos(yaw_rad)
        dir_y = math.sin(yaw_rad)
        epsilon = 1e-6

        speed = 300

        if abs(dir_x) > epsilon:
            t = (target_x - ball_x) / dir_x
            if t >= 0:
                y_hit = ball_y + t * dir_y
                if target_y_min <= y_hit <= target_y_max:
                    dribbler = -1
                    kick = True
    if kick == True:
        dribbler = -1
    direction, speed = keep_motion_inside_white_lines(
        x_pos, y_pos, direction + offset, speed
    )
    state.steering = steering
    return direction, speed, rotation, state, kick, dribbler

# Inputs:
# x_pos: x position of the robot
# y_pos: y position of the robot
# yaw: yaw value of the robot
# ball_x: x position of the ball
# ball_y: y position of the ball
# ball_captured: True when the ball is touching the capture zone
# state: controller-owned persistent state, or None on first call
# friendly_bot_positions: optional iterable of (x, y) positions for friendly robots
# enemy_bot_positions: optional iterable of (x, y) positions for enemy robots
# Outputs:
# direction: degrees to move in
# speed: mm/s to move at
# rotation: yaw value to rotate towards
# state: controller-owned state to pass into the next call.
# kick: True if the bot should kick the ball
def goalie(
    x_pos,
    y_pos,
    yaw,
    ball_x,
    ball_y,
    ball_captured=False,
    state=None,
    friendly_bot_positions=None,
    enemy_bot_positions=None,
):
    """Track the ball-to-goal line at fixed X; correct any displacement from it."""
    target_x = GOALIE_BLOCK_X
    target_y = GOAL_CENTRE_Y
    rotation = 0
    if ball_x is not None and ball_y is not None:
        if ball_x > GOALIE_BLOCK_X:
            fraction = (GOALIE_BLOCK_X - YELLOW_GOAL_BACK_X) / (ball_x - YELLOW_GOAL_BACK_X)
            target_y += fraction * (ball_y - GOAL_CENTRE_Y)
        else:
            if ball_y < GOALIE_BOX_Y_MIN or ball_y > GOALIE_BOX_Y_MAX:
                target_x = ball_x
            target_y = ball_y
        rotation = math.degrees(math.atan2(ball_y - y_pos, ball_x - x_pos)) % 360

    dribbler = 0
    kick = False
    if ball_captured:
        if (yaw > 350 or yaw < 10):
            dribbler = -1
            kick = True
        else:
            dribbler = 1
            rotation = 0

    target_x = max(WHITE_MIN_X+180, min(target_x, WHITE_MAX_X))
    target_y = max(GOALIE_BOX_Y_MIN, min(target_y, GOALIE_BOX_Y_MAX))
    dx, dy = target_x - x_pos, target_y - y_pos
    distance = math.hypot(dx, dy)
    direction = math.degrees(math.atan2(dy, dx))
    remaining = max(0, distance - GOALIE_STOP_DISTANCE)
    speed = min(GOALIE_MAX_SPEED, math.sqrt(2 * GOALIE_BRAKING_ACCEL * remaining))
    direction, speed = keep_motion_inside_white_lines(x_pos, y_pos, direction, speed)
    return direction, speed, rotation, None, kick, dribbler
