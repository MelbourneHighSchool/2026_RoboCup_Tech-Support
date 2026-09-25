import argparse
import asyncio
import bisect
import math
import random
import sys
import threading
import time
import tkinter as tk
from collections.abc import Callable, Sequence
from dataclasses import dataclass, field
from pathlib import Path

import pygame
import websockets

from defence import defence, goalie
from lib.ball_possession import (
    BallExtrapolator,
    BallPossessionTracker,
    ball_is_near_bot,
)
from lib.controller_state import decode_state
from lib.session_replay import (
    EventTimeline,
    VideoReader,
    annotate_video_frame,
    game_event_tokens,
    load_recorded_session,
)
from striker import striker
from tests.bot import bot

parser = argparse.ArgumentParser(
    description="Visualize a simulated match from a log file.",
    epilog=(
        "Game log format: "
        "x_pos,y_pos,yaw,ball_x,ball_y,"
        "ball_captured,bot_mode,state,direction,speed,rotation,kick,dribbler"
        "[,bot1_x,bot1_y,...]"
    ),
)
parser.add_argument(
    "log_file",
    nargs="?",
    help="Path to the game log file (CSV format) containing frame-by-frame positions.",
)
parser.add_argument(
    "-c",
    "--connect",
    metavar="ADDR",
    help="Connect to the log server at ADDR (IP:PORT), e.g. -c 127.0.0.1:8765",
)
parser.add_argument(
    "-d",
    "--defence",
    action="store_true",
    help="Use the defence strategy from defence.py instead of keyboard control.",
)
parser.add_argument(
    "-g",
    "--goalie",
    action="store_true",
    help="Use the goalie strategy from defence.py instead of keyboard control.",
)
parser.add_argument(
    "-s",
    "--striker",
    action="store_true",
    help="Use the striker strategy from striker.py instead of keyboard control.",
)

parser.add_argument(
    "-t",
    "--test_bot",
    action="store_true",
    help="Run a bot that does nothing.",
)

parser.add_argument(
    "--vision",
    action="store_true",
    help="Hide the ball from bots whose centre-to-ball ray hits another bot's black wall.",
)
parser.add_argument(
    "--vision-ball-possession",
    action="store_true",
    help=(
        "With --vision, infer a hidden ball at the position of a nearby tracked "
        "enemy bot."
    ),
)

parser.add_argument(
    "--team1",
    nargs="+",
    metavar="ROLE",
    help="Space-separated roles (e.g., d g) for Team 1 (yellow, yaw 0).",
)
parser.add_argument(
    "--team2",
    nargs="+",
    metavar="ROLE",
    help="Space-separated roles (e.g., d d) for Team 2 (cyan, yaw 180).",
)
args = parser.parse_args()
if args.vision_ball_possession and not args.vision:
    parser.error("--vision-ball-possession requires --vision")

lines = []
session_provided = args.log_file is not None and Path(args.log_file).is_dir()
log_provided = args.log_file is not None and not session_provided

if log_provided:
    try:
        with open(args.log_file, "r") as f:
            lines = f.readlines()
    except OSError as exc:
        print(f"Unable to read log file '{args.log_file}': {exc}")
        sys.exit(1)

pygame.init()

INITIAL_WINDOW_SIZE = (1215, 910)

display = pygame.display.set_mode(INITIAL_WINDOW_SIZE, pygame.RESIZABLE)

pitch = pygame.Surface((2430, 1820))
PITCH_WIDTH = pitch.get_width()
PITCH_HEIGHT = pitch.get_height()

green = (20, 110, 44)
white = (255, 255, 255)
black = (0, 0, 0)
cyan = (0, 255, 255)
yellow = (255, 255, 0)
orange = (255, 165, 0)
red = (255, 0, 0)

GOAL_LEFT_EDGE_X = 0
GOAL_RIGHT_EDGE_X = PITCH_WIDTH
GOAL_LEFT_BACK_X = 226
GOAL_RIGHT_BACK_X = 2204
GOAL_LEFT_FRONT_X = 300
GOAL_RIGHT_FRONT_X = 2130
GOAL_TOP_Y = 685
GOAL_BOTTOM_Y = 1135
GOAL_BACK_BOTTOM_Y = 1140
GOAL_LINE_WIDTH = 10

ControllerFunc = Callable[..., Sequence[float]]

ROLE_CONTROLLER_MAP = {
    "d": ("defence", defence),
    "defence": ("defence", defence),
    "g": ("goalie", goalie),
    "goalie": ("goalie", goalie),
    "s": ("striker", striker),
    "striker": ("striker", striker),
    "t": ("test_bot", bot),
    "test_bot": ("test_bot", bot),
}

TEAM_DEFAULTS = {
    1: {"color": yellow, "yaw": 0.0},
    2: {"color": cyan, "yaw": 180.0},
}

WHITE_MIN_X = 250
WHITE_MAX_X = 2180
WHITE_MIN_Y = 250
WHITE_MAX_Y = 1570

ROBOT_RADIUS = 110
BALL_CAPTURE_ZONE_WIDTH = 50
CAPTURE_OFFSET = ROBOT_RADIUS - 15
CONNECTOR_ANGLE_DEG = 45
CONNECTOR_ANGLE_RAD = math.radians(CONNECTOR_ANGLE_DEG)
CUTOUT_ARC_SEGMENTS = 50
CAPTURE_LINE_WIDTH = 4
CAPTURE_LINE_HALF_WIDTH = CAPTURE_LINE_WIDTH / 2.0
BALL_RADIUS = 21
BALL_DECELERATION_SPEED = 1000  # mm/s^2
YAW_CORRECT_SPEED = 1000  # mm/s spin speed used when aligning yaw
WALL_BOUNCE_ENERGY_LOSS = 0.7
EPSILON = 1e-6
ACCELERATION = 8000  # mm/s^2
KICK_SPEED = 2000

log_line = None

pitch.fill(green)

#Pitch markings

pygame.draw.rect(pitch, white, pygame.Rect(250, 250, 1930, 1320), 50)
pygame.draw.circle(pitch, black, (1215, 910), 10)
pygame.draw.circle(pitch, black, (1215, 610), 10)
pygame.draw.circle(pitch, black, (1215, 1210), 10)

#Goal Boxes

pygame.draw.line(pitch, black, (300, 460), (610, 460), 20)
pygame.draw.line(pitch, black, (600, 460), (600, 1370), 20)
pygame.draw.line(pitch, black, (600, 1360), (300, 1360), 20)
pygame.draw.line(pitch, black, (2130, 460), (1820, 460), 20)
pygame.draw.line(pitch, black, (1830, 460), (1830, 1370), 20)
pygame.draw.line(pitch, black, (1830, 1360), (2130, 1360), 20)

#Goals

pygame.draw.rect(pitch, cyan, pygame.Rect(226, 685, 74, 450))
pygame.draw.rect(pitch, yellow, pygame.Rect(2130, 685, 74, 450))
GOAL_LINES = [
    ((GOAL_LEFT_FRONT_X, GOAL_TOP_Y), (GOAL_LEFT_EDGE_X, GOAL_TOP_Y), GOAL_LINE_WIDTH),
    ((GOAL_LEFT_BACK_X, GOAL_TOP_Y), (GOAL_LEFT_BACK_X, GOAL_BACK_BOTTOM_Y), GOAL_LINE_WIDTH),
    ((GOAL_LEFT_EDGE_X, GOAL_BOTTOM_Y), (GOAL_LEFT_FRONT_X, GOAL_BOTTOM_Y), GOAL_LINE_WIDTH),
    ((GOAL_RIGHT_FRONT_X, GOAL_TOP_Y), (GOAL_RIGHT_EDGE_X, GOAL_TOP_Y), GOAL_LINE_WIDTH),
    ((GOAL_RIGHT_BACK_X, GOAL_TOP_Y), (GOAL_RIGHT_BACK_X, GOAL_BACK_BOTTOM_Y), GOAL_LINE_WIDTH),
    ((GOAL_RIGHT_EDGE_X, GOAL_BOTTOM_Y), (GOAL_RIGHT_FRONT_X, GOAL_BOTTOM_Y), GOAL_LINE_WIDTH),
]

for start, end, line_width in GOAL_LINES:
    pygame.draw.line(pitch, black, start, end, line_width)

BLACK_POINTS = [
    (1215, 610),
    (1215, 1210),
]

CENTRE_POINT = (1215, 910)

base_pitch = pitch.copy()

clock = pygame.time.Clock()

# Scale factor: millimeters per pixel
# Pitch playable area is approximately 1930x1320 pixels
# Adjust this constant based on real-world pitch dimensions
MM_PER_PIXEL = 1.0  # Default: 1mm per pixel (adjust as needed)
FPS = 120
LOG_FPS = 30  # Matches main.py --save-log recording rate
VISION_BALL_TIMEOUT_S = 0.5

BOT_RADIUS = ROBOT_RADIUS
BOT_MIN_X = BOT_RADIUS
BOT_MAX_X = pitch.get_width() - BOT_RADIUS
BOT_MIN_Y = BOT_RADIUS
BOT_MAX_Y = pitch.get_height() - BOT_RADIUS
RED_DURATION_MS = 1000
BOT_DIAMETER = BOT_RADIUS * 2


def keep_bot_inside_white_lines(bot):
    """Clamp the whole robot body to the playable side of the white lines."""
    min_x = WHITE_MIN_X + BOT_RADIUS
    max_x = WHITE_MAX_X - BOT_RADIUS
    min_y = WHITE_MIN_Y + BOT_RADIUS
    max_y = WHITE_MAX_Y - BOT_RADIUS
    bot.x = max(min_x, min(max_x, bot.x))
    bot.y = max(min_y, min(max_y, bot.y))


def parse_optional_float(token: str | None) -> float | None:
    """Return float(token) or None if token is missing/None/empty/'None'."""
    if token is None:
        return None
    token = token.strip()
    if token == "" or token.lower() == "none":
        return None
    try:
        return float(token)
    except (TypeError, ValueError):
        return None


def parse_optional_bool(token: str | None) -> bool | None:
    if token is None:
        return None
    token = token.strip().lower()
    if token == "" or token == "none":
        return None
    if token in {"1", "true", "yes", "on"}:
        return True
    if token in {"0", "false", "no", "off"}:
        return False
    return None


CONTROLLER_LOG_FIELDS = (
    "ball_captured",
    "bot_mode",
    "state",
    "direction",
    "speed",
    "rotation",
    "kick",
    "dribbler",
)
CONTROLLER_LOG_START = 5
CONTROLLER_LOG_END = CONTROLLER_LOG_START + len(CONTROLLER_LOG_FIELDS)


def has_controller_log_fields(tokens: Sequence[str]) -> bool:
    if len(tokens) < CONTROLLER_LOG_END:
        return False
    mode = tokens[CONTROLLER_LOG_START + 1].strip().upper()
    if mode in {"DEFENCE", "GOALIE", "STRIKER"}:
        return True
    return parse_optional_bool(tokens[CONTROLLER_LOG_START]) is not None


def format_log_value(value) -> str:
    if value is None:
        return "—"
    if isinstance(value, bool):
        return "True" if value else "False"
    if isinstance(value, float):
        return f"{value:.2f}"
    return str(value)


def parse_log_frame(tokens: Sequence[str]) -> dict | None:
    """Parse one CSV log line into pose / ball / optional controller fields."""
    if len(tokens) < 3:
        return None

    x_raw = parse_optional_float(tokens[0])
    y_raw = parse_optional_float(tokens[1])
    yaw_raw = parse_optional_float(tokens[2])
    if x_raw is None or y_raw is None or yaw_raw is None:
        return None

    ball_x_raw = parse_optional_float(tokens[3]) if len(tokens) > 3 else None
    ball_y_raw = parse_optional_float(tokens[4]) if len(tokens) > 4 else None

    controller = None
    other_start = 5
    if has_controller_log_fields(tokens):
        dribbler = None
        other_start = CONTROLLER_LOG_END - 1
        if len(tokens) > CONTROLLER_LOG_END - 1:
            dribbler_raw = parse_optional_bool(tokens[CONTROLLER_LOG_END - 1])
            if dribbler_raw is not None:
                dribbler = dribbler_raw
                other_start = CONTROLLER_LOG_END
        controller = {
            "ball_captured": parse_optional_bool(tokens[5]),
            "bot_mode": tokens[6].strip() if tokens[6].strip().lower() != "none" else None,
            "state": decode_state(tokens[7]),
            "direction": parse_optional_float(tokens[8]),
            "speed": parse_optional_float(tokens[9]),
            "rotation": parse_optional_float(tokens[10]),
            "kick": parse_optional_bool(tokens[11]),
            "dribbler": dribbler,
        }

    other_bots = []
    for i in range(other_start, len(tokens), 2):
        bot_x_raw = parse_optional_float(tokens[i])
        bot_y_raw = parse_optional_float(tokens[i + 1]) if i + 1 < len(tokens) else None
        if bot_x_raw is None or bot_y_raw is None:
            continue
        other_bots.append((int(bot_x_raw), int(bot_y_raw)))

    return {
        "x_pos": int(x_raw),
        "y_pos": int(y_raw),
        "yaw": float(yaw_raw),
        "ball_x": int(ball_x_raw) if ball_x_raw is not None else None,
        "ball_y": int(ball_y_raw) if ball_y_raw is not None else None,
        "controller": controller,
        "other_bots": other_bots,
    }


@dataclass
class Bot:
    x: float
    y: float
    yaw: float
    base_color: tuple
    controller: ControllerFunc | None = None
    manual: bool = False
    name: str = "bot"
    red_until_time: int | None = None
    was_out_of_bounds: bool = False
    current_color: tuple = field(init=False)
    push_speed: float = 0.0
    desired_velocity: tuple = field(default_factory=lambda: (0.0, 0.0))
    velocity_x: float = 0.0
    velocity_y: float = 0.0
    state: object = None
    ball_possession_tracker: BallPossessionTracker = field(
        default_factory=BallPossessionTracker
    )
    ball_extrapolator: BallExtrapolator = field(
        default_factory=lambda: BallExtrapolator(VISION_BALL_TIMEOUT_S)
    )
    self_ball_candidate: bool = False

    def __post_init__(self):
        self.current_color = self.base_color

    def as_render_state(self):
        return {
            "x": self.x,
            "y": self.y,
            "yaw": self.yaw,
            "color": self.current_color,
            "draw_geometry": True,
        }


def mmps_to_pixels(value_mm_per_s, dt_seconds):
    """Convert a mm/s value to pixels travelled during this frame."""
    return (value_mm_per_s / MM_PER_PIXEL) * dt_seconds


def parse_role_token(token):
    key = token.lower()
    if key not in ROLE_CONTROLLER_MAP:
        parser.error(
            f"Unknown role '{token}'. Use one of: {', '.join(sorted(set(ROLE_CONTROLLER_MAP)))}"
        )
    return ROLE_CONTROLLER_MAP[key]


def create_team(role_tokens: Sequence[str], team_number: int) -> list[Bot]:
    defaults = TEAM_DEFAULTS.get(team_number, TEAM_DEFAULTS[1])
    bots: list[Bot] = []
    for idx, token in enumerate(role_tokens, start=1):
        if team_number == 1:
            start_x = random.randint(450, 600)
            start_y = random.randint(510, 1410)
        else:
            start_x = random.randint(1830, 1980)
            start_y = random.randint(510, 1410)
        if token == "m":
            bots.append(
                Bot(
                    x=start_x,
                    y=start_y,
                    yaw=defaults["yaw"],
                    base_color=defaults["color"],
                    controller=None,
                    manual=True,
                    name=f"Team {team_number} Manual #{idx}",
                )
            )
        else:
            role_name, controller = parse_role_token(token)
            bots.append(
                Bot(
                    x=start_x,
                    y=start_y,
                    yaw=defaults["yaw"],
                    base_color=defaults["color"],
                    controller=controller,
                    manual=False,
                    name=f"Team {team_number} {role_name.title()} #{idx}",
                )
            )
    return bots


def resolve_bot_collisions(bot_states):
    num_bots = len(bot_states)
    if num_bots < 2:
        return

    for i in range(num_bots):
        for j in range(i + 1, num_bots):
            state_a = bot_states[i]
            state_b = bot_states[j]
            bot_a = state_a["bot"]
            bot_b = state_b["bot"]

            dx = bot_b.x - bot_a.x
            dy = bot_b.y - bot_a.y
            distance = math.hypot(dx, dy)
            if distance >= BOT_DIAMETER or distance < EPSILON:
                continue

            overlap = BOT_DIAMETER - distance if distance > EPSILON else BOT_DIAMETER
            speed_a = bot_a.push_speed
            speed_b = bot_b.push_speed

            pushing_state = None
            pushed_state = None
            dir_x = 0.0
            dir_y = 0.0

            if abs(speed_a - speed_b) <= 1e-3:
                dx_a, dy_a = state_a["delta"]
                dx_b, dy_b = state_b["delta"]
                mag_a = math.hypot(dx_a, dy_a)
                mag_b = math.hypot(dx_b, dy_b)

                if mag_a < EPSILON and mag_b < EPSILON:
                    overlap = BOT_DIAMETER - distance if distance > EPSILON else BOT_DIAMETER
                    if overlap <= 0:
                        continue
                    if distance > EPSILON:
                        dir_x = dx / distance
                        dir_y = dy / distance
                    else:
                        dir_x, dir_y = 1.0, 0.0
                    shift = overlap / 2.0
                    bot_a.x -= dir_x * shift
                    bot_a.y -= dir_y * shift
                    bot_b.x += dir_x * shift
                    bot_b.y += dir_y * shift

                    bot_a.x = max(BOT_MIN_X, min(BOT_MAX_X, bot_a.x))
                    bot_a.y = max(BOT_MIN_Y, min(BOT_MAX_Y, bot_a.y))
                    bot_b.x = max(BOT_MIN_X, min(BOT_MAX_X, bot_b.x))
                    bot_b.y = max(BOT_MIN_Y, min(BOT_MAX_Y, bot_b.y))

                    # If separation pushes into goal hardware, revert to previous positions.
                    if check_collision_with_goal_lines(bot_a.x, bot_a.y, BOT_RADIUS, GOAL_LINES) or check_collision_with_goal_lines(bot_b.x, bot_b.y, BOT_RADIUS, GOAL_LINES):
                        bot_a.x, bot_a.y = state_a["prev"]
                        bot_b.x, bot_b.y = state_b["prev"]
                        state_a["delta"] = (0.0, 0.0)
                        state_b["delta"] = (0.0, 0.0)
                        bot_a.push_speed = 0.0
                        bot_b.push_speed = 0.0
                        continue

                    state_a["delta"] = (bot_a.x - state_a["prev"][0], bot_a.y - state_a["prev"][1])
                    state_b["delta"] = (bot_b.x - state_b["prev"][0], bot_b.y - state_b["prev"][1])
                    bot_a.push_speed = math.hypot(*state_a["delta"])
                    bot_b.push_speed = math.hypot(*state_b["delta"])
                    continue

                dot_product = dx_a * dx_b + dy_a * dy_b
                if dot_product < -EPSILON:
                    bot_a.x, bot_a.y = state_a["prev"]
                    bot_b.x, bot_b.y = state_b["prev"]
                    state_a["delta"] = (0.0, 0.0)
                    state_b["delta"] = (0.0, 0.0)
                    bot_a.push_speed = 0.0
                    bot_b.push_speed = 0.0
                    continue

                if mag_a >= mag_b:
                    pushing_state = state_a
                    pushed_state = state_b
                else:
                    pushing_state = state_b
                    pushed_state = state_a

                rel_x = pushed_state["bot"].x - pushing_state["bot"].x
                rel_y = pushed_state["bot"].y - pushing_state["bot"].y
                rel_dist = math.hypot(rel_x, rel_y)
                if rel_dist > EPSILON:
                    dir_x = rel_x / rel_dist
                    dir_y = rel_y / rel_dist
                else:
                    dir_x, dir_y = 1.0, 0.0
            else:
                if speed_a > speed_b:
                    pushing_state = state_a
                    pushed_state = state_b
                    dir_x = dx / distance if distance > EPSILON else 1.0
                    dir_y = dy / distance if distance > EPSILON else 0.0
                else:
                    pushing_state = state_b
                    pushed_state = state_a
                    dir_x = -dx / distance if distance > EPSILON else -1.0
                    dir_y = -dy / distance if distance > EPSILON else 0.0

            pushed_bot = pushed_state["bot"]
            pushed_bot.x += dir_x * overlap
            pushed_bot.y += dir_y * overlap
            pushed_bot.x = max(BOT_MIN_X, min(BOT_MAX_X, pushed_bot.x))
            pushed_bot.y = max(BOT_MIN_Y, min(BOT_MAX_Y, pushed_bot.y))

            boundary_block = False
            pushed_normals = get_bot_boundary_normals(pushed_bot.x, pushed_bot.y)
            if pushed_normals:
                for normal_x, normal_y in pushed_normals:
                    if dir_x * normal_x + dir_y * normal_y < -EPSILON:
                        boundary_block = True
                        break

            if boundary_block:
                bot_a.x, bot_a.y = state_a["prev"]
                bot_b.x, bot_b.y = state_b["prev"]
                state_a["delta"] = (0.0, 0.0)
                state_b["delta"] = (0.0, 0.0)
                bot_a.push_speed = 0.0
                bot_b.push_speed = 0.0
                continue

            prev_x, prev_y = pushed_state["prev"]
            new_dx = pushed_bot.x - prev_x
            new_dy = pushed_bot.y - prev_y
            pushed_state["delta"] = (new_dx, new_dy)
            pushed_bot.push_speed = math.hypot(new_dx, new_dy)

            pushing_bot = pushing_state["bot"]
            push_dx = pushing_bot.x - pushing_state["prev"][0]
            push_dy = pushing_bot.y - pushing_state["prev"][1]
            pushing_state["delta"] = (push_dx, push_dy)
            pushing_bot.push_speed = math.hypot(push_dx, push_dy)


def manual_control_from_keys(keys, current_yaw):
    up = keys[pygame.K_UP]
    down = keys[pygame.K_DOWN]
    left = keys[pygame.K_LEFT]
    right = keys[pygame.K_RIGHT]
    ctrl = keys[pygame.K_LCTRL]
    alt = keys[pygame.K_LALT]
    space = keys[pygame.K_SPACE]

    direction = None
    speed = 0
    rotation = current_yaw
    kick_state = False
    if space:
        kick_state = True

    if up and not down:
        if left and not right:
            direction = (270 + 180) / 2
        elif right and not left:
            direction = (270 + 360) / 2
        elif not left and not right:
            direction = 270
    elif down and not up:
        if left and not right:
            direction = (90 + 180) / 2
        elif right and not left:
            direction = (90 + 0) / 2
        elif not left and not right:
            direction = 90
    elif left and not right and not up and not down:
        direction = 180
    elif right and not left and not up and not down:
        direction = 0

    if direction is not None:
        speed = 700

    if ctrl:
        rotation = (current_yaw - 10) % 360
    elif alt:
        rotation = (current_yaw + 10) % 360

    return direction, speed, rotation, kick_state


# Precompute per-second values for reuse; per-frame scaling uses actual dt.
YAW_CORRECT_PIXELS_PER_S = YAW_CORRECT_SPEED / MM_PER_PIXEL

# Game log format:
# x_pos,y_pos,yaw,ball_x,ball_y,ball_captured,bot_mode,state,direction,speed,rotation,kick,dribbler[,bot1_x,bot1_y,...]

def normalize_angle_deg(angle):
    return angle % 360


def shortest_angle_delta(current, target):
    return (target - current + 180) % 360 - 180


def invert_pitch_point(x_pos, y_pos):
    return PITCH_WIDTH - x_pos, PITCH_HEIGHT - y_pos


def invert_optional_pitch_point(x_pos, y_pos):
    if x_pos is None or y_pos is None:
        return x_pos, y_pos
    return invert_pitch_point(x_pos, y_pos)


def invert_angle_deg(angle):
    if angle is None:
        return None
    return normalize_angle_deg(angle + 180)

def is_out_of_white_boundary(x_pos, y_pos, bot_radius):
    # Find closest point on rectangle to circle center
    closest_x = max(WHITE_MIN_X, min(x_pos, WHITE_MAX_X))
    closest_y = max(WHITE_MIN_Y, min(y_pos, WHITE_MAX_Y))
    
    # Calculate distance from circle center to closest point on rectangle
    dx = x_pos - closest_x
    dy = y_pos - closest_y
    distance = math.sqrt(dx * dx + dy * dy)
    
    # Circle is completely outside if distance > radius
    return distance > bot_radius


def point_to_line_segment_distance(px, py, x1, y1, x2, y2):
    # Vector from line start to end
    dx = x2 - x1
    dy = y2 - y1

    # If line segment has zero length, return distance to endpoint
    line_length_sq = dx * dx + dy * dy
    if line_length_sq == 0:
        dist = math.sqrt((px - x1) ** 2 + (py - y1) ** 2)
        return dist, (x1, y1)

    # Vector from line start to point
    to_point_x = px - x1
    to_point_y = py - y1

    # Project point onto line (t is parameter along line segment)
    t = max(0, min(1, (to_point_x * dx + to_point_y * dy) / line_length_sq))

    # Closest point on line segment
    closest_x = x1 + t * dx
    closest_y = y1 + t * dy

    # Distance from point to closest point on segment
    dist_x = px - closest_x
    dist_y = py - closest_y
    distance = math.sqrt(dist_x * dist_x + dist_y * dist_y)

    return distance, (closest_x, closest_y)


def segment_segment_distance(a0, a1, b0, b1):
    """Closest distance between two finite line segments."""
    ax0, ay0 = a0
    ax1, ay1 = a1
    bx0, by0 = b0
    bx1, by1 = b1

    abx = ax1 - ax0
    aby = ay1 - ay0
    cdx = bx1 - bx0
    cdy = by1 - by0
    acx = ax0 - bx0
    acy = ay0 - by0

    ab_len_sq = abx * abx + aby * aby
    cd_len_sq = cdx * cdx + cdy * cdy
    ab_dot_cd = abx * cdx + aby * cdy
    ab_dot_ac = abx * acx + aby * acy
    cd_dot_ac = cdx * acx + cdy * acy

    denom = ab_len_sq * cd_len_sq - ab_dot_cd * ab_dot_cd
    if abs(denom) > EPSILON:
        s = (ab_dot_cd * cd_dot_ac - cd_len_sq * ab_dot_ac) / denom
        t = (ab_len_sq * cd_dot_ac - ab_dot_cd * ab_dot_ac) / denom
        if 0.0 <= s <= 1.0 and 0.0 <= t <= 1.0:
            px = ax0 + s * abx
            py = ay0 + s * aby
            qx = bx0 + t * cdx
            qy = by0 + t * cdy
            return math.hypot(px - qx, py - qy)

    dist_a0, _ = point_to_line_segment_distance(ax0, ay0, bx0, by0, bx1, by1)
    dist_a1, _ = point_to_line_segment_distance(ax1, ay1, bx0, by0, bx1, by1)
    dist_b0, _ = point_to_line_segment_distance(bx0, by0, ax0, ay0, ax1, ay1)
    dist_b1, _ = point_to_line_segment_distance(bx1, by1, ax0, ay0, ax1, ay1)
    return min(dist_a0, dist_a1, dist_b0, dist_b1)


def ball_visible_from(observer, ball_x, ball_y, bots):
    """True if the centre-to-ball ray does not hit another bot's black wall."""
    ray_start = (observer.x, observer.y)
    ray_end = (ball_x, ball_y)
    for other in bots:
        if other is observer:
            continue
        geometry = get_capture_geometry(other.x, other.y, other.yaw)
        for start, end in geometry["segments"]:
            if segment_segment_distance(ray_start, ray_end, start, end) <= (
                CAPTURE_LINE_HALF_WIDTH + EPSILON
            ):
                return False
    return True



def check_collision_with_goal_lines(x_pos, y_pos, bot_radius, goal_lines):
    for line in goal_lines:
        (x1, y1), (x2, y2), line_width = line
        distance, _ = point_to_line_segment_distance(x_pos, y_pos, x1, y1, x2, y2)
        
        # Collision if distance < (circle_radius + line_width/2)
        if distance < (bot_radius + line_width / 2):
            return True
    return False


def get_bot_boundary_normals(x_pos, y_pos):
    normals = []
    if x_pos <= BOT_MIN_X + EPSILON:
        normals.append((1.0, 0.0))
    if x_pos >= BOT_MAX_X - EPSILON:
        normals.append((-1.0, 0.0))
    if y_pos <= BOT_MIN_Y + EPSILON:
        normals.append((0.0, 1.0))
    if y_pos >= BOT_MAX_Y - EPSILON:
        normals.append((0.0, -1.0))

    for (x1, y1), (x2, y2), line_width in GOAL_LINES:
        distance, closest_point = point_to_line_segment_distance(x_pos, y_pos, x1, y1, x2, y2)
        allowed_distance = BOT_RADIUS + line_width / 2.0
        if distance <= allowed_distance + EPSILON:
            diff_x = x_pos - closest_point[0]
            diff_y = y_pos - closest_point[1]
            norm = math.hypot(diff_x, diff_y)
            if norm > EPSILON:
                normals.append((diff_x / norm, diff_y / norm))
    return normals


def calculate_capture_zone_back(x_pos, y_pos, angle_rad):
    line_center_x = x_pos + CAPTURE_OFFSET * math.cos(angle_rad)
    line_center_y = y_pos + CAPTURE_OFFSET * math.sin(angle_rad)

    half_width = BALL_CAPTURE_ZONE_WIDTH / 2.0
    perpendicular_angle = angle_rad + math.pi / 2.0
    dx = half_width * math.cos(perpendicular_angle)
    dy = half_width * math.sin(perpendicular_angle)

    line_start = (line_center_x - dx, line_center_y - dy)
    line_end = (line_center_x + dx, line_center_y + dy)
    return line_start, line_end


def ray_circle_intersection(px, py, angle_rad, circle_center, radius):
    cx, cy = circle_center
    dir_x = math.cos(angle_rad)
    dir_y = math.sin(angle_rad)

    fx = px - cx
    fy = py - cy

    b = fx * dir_x + fy * dir_y
    c = fx * fx + fy * fy - radius * radius
    discriminant = b * b - c

    if discriminant < 0:
        return circle_center

    sqrt_disc = math.sqrt(discriminant)
    t1 = -b - sqrt_disc
    t2 = -b + sqrt_disc

    ts = [t for t in (t1, t2) if t > 0]
    if not ts:
        return circle_center

    t = min(ts)
    hit_x = px + t * dir_x
    hit_y = py + t * dir_y
    return hit_x, hit_y


def build_clockwise_arc_points(start_point, end_point, center, radius, segments):
    if segments < 2 or start_point == center or end_point == center or radius <= 0:
        return []

    cx, cy = center
    two_pi = 2 * math.pi

    start_angle = math.atan2(start_point[1] - cy, start_point[0] - cx) % two_pi
    end_angle = math.atan2(end_point[1] - cy, end_point[0] - cx) % two_pi

    cw_span = (start_angle - end_angle) % two_pi
    ccw_span = (end_angle - start_angle) % two_pi
    use_cw = True
    span = cw_span if use_cw else ccw_span

    if math.isclose(span, 0, abs_tol=1e-5):
        return []

    angle_step = span / segments

    points = []
    for i in range(1, segments):
        if use_cw:
            angle = (start_angle - angle_step * i) % two_pi
        else:
            angle = (start_angle + angle_step * i) % two_pi
        px = cx + radius * math.cos(angle)
        py = cy + radius * math.sin(angle)
        points.append((px, py))
    return points


def get_capture_geometry(x_pos, y_pos, yaw):
    angle = math.radians(yaw)
    line_start, line_end = calculate_capture_zone_back(x_pos, y_pos, angle)
    robot_center = (x_pos, y_pos)

    angle_left = angle - CONNECTOR_ANGLE_RAD
    angle_right = angle + CONNECTOR_ANGLE_RAD
    left_circle = ray_circle_intersection(
        line_start[0], line_start[1], angle_left, robot_center, ROBOT_RADIUS
    )
    right_circle = ray_circle_intersection(
        line_end[0], line_end[1], angle_right, robot_center, ROBOT_RADIUS
    )

    connector_lines = [
        (line_start, left_circle),
        (line_end, right_circle),
        (line_start, line_end),
    ]

    arc_segments = []
    if left_circle != robot_center and right_circle != robot_center:
        arc_points = build_clockwise_arc_points(
            left_circle, right_circle, robot_center, ROBOT_RADIUS, CUTOUT_ARC_SEGMENTS
        )
        arc_points_full = [left_circle] + arc_points + [right_circle]
        for i in range(len(arc_points_full) - 1):
            arc_segments.append((arc_points_full[i], arc_points_full[i + 1]))

    segments = connector_lines + arc_segments

    return {
        "angle": angle,
        "line_start": line_start,
        "line_end": line_end,
        "robot_center": robot_center,
        "left_circle": left_circle,
        "right_circle": right_circle,
        "connector_lines": connector_lines,
        "arc_segments": arc_segments,
        "segments": segments,
    }


def is_ball_touching_capture_zone(ball_x, ball_y, geometry):
    threshold = BALL_RADIUS + CAPTURE_LINE_HALF_WIDTH
    for start, end in geometry["connector_lines"]:
        distance, _ = point_to_line_segment_distance(
            ball_x, ball_y, start[0], start[1], end[0], end[1]
        )
        if distance <= threshold + EPSILON:
            return True
    return False


def get_dribbled_ball_position(bot):
    """Ball center when held against the capture-zone back line."""
    angle = math.radians(bot.yaw)
    offset = CAPTURE_OFFSET + BALL_RADIUS + CAPTURE_LINE_HALF_WIDTH
    return (
        bot.x + offset * math.cos(angle),
        bot.y + offset * math.sin(angle),
    )


def is_point_in_polygon(point, polygon):
    x, y = point
    inside = False
    num_points = len(polygon)
    if num_points < 3:
        return False

    j = num_points - 1
    for i in range(num_points):
        xi, yi = polygon[i]
        xj, yj = polygon[j]
        intersects = (yi > y) != (yj > y)
        if intersects:
            x_intersect = (xj - xi) * (y - yi) / ((yj - yi) + EPSILON) + xi
            if x < x_intersect:
                inside = not inside
        j = i
    return inside


def is_point_in_capture_zone(ball_x, ball_y, geometry):
    left_circle = geometry.get("left_circle")
    right_circle = geometry.get("right_circle")
    if left_circle is None or right_circle is None:
        return False

    polygon = [left_circle, geometry["line_start"], geometry["line_end"], right_circle]
    return is_point_in_polygon((ball_x, ball_y), polygon)


def is_ball_inside_bot_body(ball_x, ball_y, bot):
    dist_to_center = math.hypot(ball_x - bot.x, ball_y - bot.y)
    if dist_to_center >= ROBOT_RADIUS - BALL_RADIUS - EPSILON:
        return False

    geometry = get_capture_geometry(bot.x, bot.y, bot.yaw)
    return not is_point_in_capture_zone(ball_x, ball_y, geometry)


def find_nearest_free_black_point(ball_x, ball_y, bots):
    available_points = []
    for point in BLACK_POINTS:
        covered = any(math.hypot(bot.x - point[0], bot.y - point[1]) <= BOT_RADIUS for bot in bots)
        if not covered:
            available_points.append(point)

    if not available_points:
        centre_covered = any(math.hypot(bot.x - CENTRE_POINT[0], bot.y - CENTRE_POINT[1]) <= BOT_RADIUS for bot in bots)
        if not centre_covered:
            return CENTRE_POINT
        return None

    return min(available_points, key=lambda p: math.hypot(p[0] - ball_x, p[1] - ball_y))


def build_frame(ball_x, ball_y, bots):
    frame_pitch = base_pitch.copy()

    def _int_point(point):
        return (int(point[0]), int(point[1]))

    for bot in bots:
        if isinstance(bot, Bot):
            bot_data = bot.as_render_state()
        else:
            bot_data = bot

        bot_x = bot_data.get("x", 0)
        bot_y = bot_data.get("y", 0)
        bot_color = bot_data.get("color", yellow)
        draw_geometry = bot_data.get("draw_geometry", bot_data.get("yaw") is not None)
        yaw = bot_data.get("yaw")

        pygame.draw.circle(frame_pitch, bot_color, (int(bot_x), int(bot_y)), ROBOT_RADIUS)

        if yaw is None or not draw_geometry:
            continue

        geometry = get_capture_geometry(bot_x, bot_y, yaw)
        connector_lines = [
            (_int_point(start), _int_point(end)) for start, end in geometry["connector_lines"]
        ]

        for start, end in geometry["arc_segments"]:
            pygame.draw.line(
                frame_pitch, black, _int_point(start), _int_point(end), CAPTURE_LINE_WIDTH
            )

        for start, end in connector_lines:
            pygame.draw.line(frame_pitch, black, start, end, CAPTURE_LINE_WIDTH)

    if ball_x is not None and ball_y is not None:
        pygame.draw.circle(frame_pitch, orange, (int(ball_x), int(ball_y)), BALL_RADIUS)
    return frame_pitch


def resolve_circle_segment_penetration(
    circle_x,
    circle_y,
    radius,
    segment_start,
    segment_end,
    line_half_width,
    outward_reference=None,
):
    distance, closest_point = point_to_line_segment_distance(
        circle_x, circle_y, segment_start[0], segment_start[1], segment_end[0], segment_end[1]
    )
    allowed_distance = radius + line_half_width
    if distance >= allowed_distance:
        return circle_x, circle_y, None

    diff_x = circle_x - closest_point[0]
    diff_y = circle_y - closest_point[1]
    norm = math.hypot(diff_x, diff_y)
    if norm < EPSILON and outward_reference is not None:
        diff_x = circle_x - outward_reference[0]
        diff_y = circle_y - outward_reference[1]
        norm = math.hypot(diff_x, diff_y)
    if norm < EPSILON:
        # Default normal if we still cannot determine direction
        diff_x, diff_y = 1.0, 0.0
        norm = 1.0

    normal_x = diff_x / norm
    normal_y = diff_y / norm
    penetration = allowed_distance - distance
    circle_x += normal_x * penetration
    circle_y += normal_y * penetration
    return circle_x, circle_y, (normal_x, normal_y)


def resolve_ball_capture_collisions(ball_x, ball_y, ball_vx, ball_vy, geometry, bot_velocity):
    ball_being_pushed = False
    bot_speed = math.hypot(bot_velocity[0], bot_velocity[1])
    for segment_start, segment_end in geometry["segments"]:
        ball_x, ball_y, normal = resolve_circle_segment_penetration(
            ball_x,
            ball_y,
            BALL_RADIUS,
            segment_start,
            segment_end,
            CAPTURE_LINE_HALF_WIDTH,
            geometry["robot_center"],
        )
        if normal is None:
            continue
        impact = 0.0
        if bot_speed > EPSILON:
            impact = bot_velocity[0] * normal[0] + bot_velocity[1] * normal[1]
        if impact > 0:
            alignment = min(1.0, impact / (bot_speed + EPSILON))
            ball_vx = bot_velocity[0] * alignment
            ball_vy = bot_velocity[1] * alignment
            ball_being_pushed = True
        else:
            vel_dot = ball_vx * normal[0] + ball_vy * normal[1]
            if vel_dot < 0:
                ball_vx -= vel_dot * normal[0]
                ball_vy -= vel_dot * normal[1]

    return ball_x, ball_y, ball_vx, ball_vy, ball_being_pushed


def keep_ball_in_pitch_bounds(ball_x, ball_y, ball_vx, ball_vy):
    ball_min_x = BALL_RADIUS
    ball_max_x = PITCH_WIDTH - BALL_RADIUS
    ball_min_y = BALL_RADIUS
    ball_max_y = PITCH_HEIGHT - BALL_RADIUS

    against_boundary = False

    if ball_x < ball_min_x:
        ball_x = ball_min_x
        ball_vx = abs(ball_vx) * WALL_BOUNCE_ENERGY_LOSS
        against_boundary = True
    elif ball_x > ball_max_x:
        ball_x = ball_max_x
        ball_vx = -abs(ball_vx) * WALL_BOUNCE_ENERGY_LOSS
        against_boundary = True

    if ball_y < ball_min_y:
        ball_y = ball_min_y
        ball_vy = abs(ball_vy) * WALL_BOUNCE_ENERGY_LOSS
        against_boundary = True
    elif ball_y > ball_max_y:
        ball_y = ball_max_y
        ball_vy = -abs(ball_vy) * WALL_BOUNCE_ENERGY_LOSS
        against_boundary = True

    return ball_x, ball_y, ball_vx, ball_vy, against_boundary


def is_ball_touching_pitch_bounds(ball_x, ball_y):
    ball_min_x = BALL_RADIUS
    ball_max_x = PITCH_WIDTH - BALL_RADIUS
    ball_min_y = BALL_RADIUS
    ball_max_y = PITCH_HEIGHT - BALL_RADIUS
    return (
        ball_x <= ball_min_x + EPSILON
        or ball_x >= ball_max_x - EPSILON
        or ball_y <= ball_min_y + EPSILON
        or ball_y >= ball_max_y - EPSILON
    )


def is_ball_touching_goal_lines(ball_x, ball_y, goal_lines):
    for (x1, y1), (x2, y2), line_width in goal_lines:
        distance, _ = point_to_line_segment_distance(ball_x, ball_y, x1, y1, x2, y2)
        allowed_distance = BALL_RADIUS + line_width / 2.0
        if distance <= allowed_distance + EPSILON:
            return True
    return False


def resolve_ball_goal_line_collisions(ball_x, ball_y, ball_vx, ball_vy, goal_lines):
    against_goal_line = False
    for (x1, y1), (x2, y2), line_width in goal_lines:
        distance, closest_point = point_to_line_segment_distance(ball_x, ball_y, x1, y1, x2, y2)
        allowed_distance = BALL_RADIUS + line_width / 2.0
        if distance >= allowed_distance:
            continue
        diff_x = ball_x - closest_point[0]
        diff_y = ball_y - closest_point[1]
        norm = math.hypot(diff_x, diff_y)
        if norm < EPSILON:
            continue
        normal_x = diff_x / norm
        normal_y = diff_y / norm
        penetration = allowed_distance - distance
        ball_x += normal_x * penetration
        ball_y += normal_y * penetration
        vel_dot = ball_vx * normal_x + ball_vy * normal_y
        if vel_dot < 0:
            ball_vx -= 2 * vel_dot * normal_x
            ball_vy -= 2 * vel_dot * normal_y
            ball_vx *= WALL_BOUNCE_ENERGY_LOSS
            ball_vy *= WALL_BOUNCE_ENERGY_LOSS
        against_goal_line = True
    return ball_x, ball_y, ball_vx, ball_vy, against_goal_line
def get_frame_render_metrics(frame_surface):
    window_width, window_height = display.get_size()
    frame_width, frame_height = frame_surface.get_size()
    scale = min(window_width / frame_width, window_height / frame_height)
    scaled_width = max(1, int(frame_width * scale))
    scaled_height = max(1, int(frame_height * scale))
    offset_x = (window_width - scaled_width) // 2
    offset_y = (window_height - scaled_height) // 2
    return scaled_width, scaled_height, offset_x, offset_y


def window_to_frame_point(window_x, window_y, frame_surface):
    scaled_width, scaled_height, offset_x, offset_y = get_frame_render_metrics(
        frame_surface
    )
    local_x = window_x - offset_x
    local_y = window_y - offset_y

    if not (0 <= local_x < scaled_width and 0 <= local_y < scaled_height):
        return None

    frame_width, frame_height = frame_surface.get_size()
    frame_x = int(local_x * frame_width / scaled_width)
    frame_y = int(local_y * frame_height / scaled_height)
    frame_x = min(max(frame_x, 0), frame_width - 1)
    frame_y = min(max(frame_y, 0), frame_height - 1)
    return frame_x, frame_y


def blit_frame(frame_surface):
    scaled_width, scaled_height, offset_x, offset_y = get_frame_render_metrics(
        frame_surface
    )
    frame_width, frame_height = frame_surface.get_size()

    if (scaled_width, scaled_height) == (frame_width, frame_height):
        scaled = frame_surface
    else:
        scaled = pygame.transform.smoothscale(
            frame_surface, (scaled_width, scaled_height)
        )

    display.fill(black)
    display.blit(scaled, (offset_x, offset_y))
    pygame.display.flip()


def blit_pitch_and_video(pitch_surface, video_rgb):
    """Show the field and camera frame side by side in one Pygame window."""
    video_surface = pygame.image.frombuffer(
        video_rgb.tobytes(),
        (video_rgb.shape[1], video_rgb.shape[0]),
        "RGB",
    )
    target_height = pitch_surface.get_height()
    target_width = max(
        1,
        int(video_surface.get_width() * target_height / video_surface.get_height()),
    )
    video_surface = pygame.transform.smoothscale(
        video_surface,
        (target_width, target_height),
    )
    combined = pygame.Surface(
        (pitch_surface.get_width() + target_width, target_height)
    )
    combined.blit(pitch_surface, (0, 0))
    combined.blit(video_surface, (pitch_surface.get_width(), 0))
    blit_frame(combined)

log_client_error: Exception | None = None
log_client_stop_event = threading.Event()


async def connect_to_log_server(addr, stop_event):
    global log_line, log_client_error
    retry_delay = 1.0
    while not stop_event.is_set():
        try:
            print(f"[log-client] connecting to ws://{addr} ...")
            async with websockets.connect(f"ws://{addr}") as websocket:
                print("[log-client] connected")
                log_client_error = None
                async for message in websocket:
                    print(f"[log-client] recv: {message}")
                    log_line = message
                    if stop_event.is_set():
                        break
            if stop_event.is_set():
                break
            print("[log-client] connection closed, retrying ...")
        except Exception as exc:
            log_client_error = exc
            if stop_event.is_set():
                break
            print(f"[log-client] error: {exc} ; retrying in {retry_delay:.1f}s")
        await asyncio.sleep(retry_delay)
    print("[log-client] stopped")


def start_log_client(addr, stop_event):
    def _runner():
        asyncio.run(connect_to_log_server(addr, stop_event))

    thread = threading.Thread(target=_runner, daemon=True)
    thread.start()
    return thread

def build_log_pitch(frame):
    ball_x = frame["ball_x"] if frame["ball_x"] is not None else CENTRE_POINT[0]
    ball_y = frame["ball_y"] if frame["ball_y"] is not None else CENTRE_POINT[1]

    render_bots = [
        {
            "x": frame["x_pos"],
            "y": frame["y_pos"],
            "yaw": frame["yaw"],
            "color": yellow,
            "draw_geometry": True,
        }
    ]
    render_bots.extend(
        {"x": bot_x, "y": bot_y, "yaw": None, "color": cyan, "draw_geometry": False}
        for bot_x, bot_y in frame["other_bots"]
    )
    return build_frame(ball_x, ball_y, render_bots)


def render_line(line_data):
    frame = parse_log_frame(line_data)
    if frame is None:
        return None
    blit_frame(build_log_pitch(frame))
    return frame


class LogControllerDebug:
    """Separate window showing controller inputs/outputs for the current log frame."""

    INPUT_ROWS = (
        ("x_pos", "X"),
        ("y_pos", "Y"),
        ("yaw", "Yaw"),
        ("ball_x", "Ball X"),
        ("ball_y", "Ball Y"),
        ("ball_captured", "Ball captured"),
        ("bot_mode", "Bot mode"),
        ("state_in", "State (in)"),
    )
    OUTPUT_ROWS = (
        ("direction", "Direction"),
        ("speed", "Speed"),
        ("rotation", "Rotation"),
        ("state", "State (out)"),
        ("kick", "Kick"),
        ("dribbler", "Dribbler"),
    )

    def __init__(self, master: tk.Misc):
        self.window = tk.Toplevel(master)
        self.window.title("Controller Debug")
        self.window.resizable(True, True)
        self.closed = False
        self.window.protocol("WM_DELETE_WINDOW", self._on_close)

        self.values: dict[str, tk.StringVar] = {}
        self._build_section("Inputs", self.INPUT_ROWS, row=0)
        self._build_section("Outputs", self.OUTPUT_ROWS, row=1)
        self._other_bots_frame = tk.LabelFrame(
            self.window, text="Other bots", padx=10, pady=8
        )
        self._other_bots_frame.grid(row=2, column=0, sticky="nsew", padx=10, pady=8)
        self.window.rowconfigure(2, weight=1)
        self._other_bots_var = tk.StringVar(value="—")
        tk.Label(
            self._other_bots_frame,
            textvariable=self._other_bots_var,
            anchor="nw",
            justify="left",
            width=28,
        ).grid(row=0, column=0, sticky="nsew")
        self._other_bots_frame.columnconfigure(0, weight=1)
        self.clear()

    def _build_section(self, title: str, rows: Sequence[tuple[str, str]], row: int) -> None:
        frame = tk.LabelFrame(self.window, text=title, padx=10, pady=8)
        frame.grid(row=row, column=0, sticky="nsew", padx=10, pady=8)
        self.window.rowconfigure(row, weight=1)
        self.window.columnconfigure(0, weight=1)
        for index, (key, label) in enumerate(rows):
            tk.Label(frame, text=label, anchor="w").grid(
                row=index, column=0, sticky="w", padx=(0, 12), pady=2
            )
            var = tk.StringVar(value="—")
            self.values[key] = var
            tk.Label(frame, textvariable=var, anchor="w", width=18).grid(
                row=index, column=1, sticky="ew", pady=2
            )
        frame.columnconfigure(1, weight=1)

    def _on_close(self) -> None:
        self.closed = True
        try:
            self.window.destroy()
        except tk.TclError:
            pass

    def close(self) -> None:
        if not self.closed:
            self._on_close()

    def clear(self) -> None:
        for var in self.values.values():
            var.set("—")
        self._other_bots_var.set("—")

    def update(self, frame: dict | None, previous_frame: dict | None = None) -> None:
        if self.closed or frame is None:
            return
        controller = frame.get("controller") or {}
        previous_controller = (previous_frame or {}).get("controller") or {}
        self.values["x_pos"].set(format_log_value(frame.get("x_pos")))
        self.values["y_pos"].set(format_log_value(frame.get("y_pos")))
        self.values["yaw"].set(format_log_value(frame.get("yaw")))
        self.values["ball_x"].set(format_log_value(frame.get("ball_x")))
        self.values["ball_y"].set(format_log_value(frame.get("ball_y")))
        self.values["ball_captured"].set(format_log_value(controller.get("ball_captured")))
        self.values["bot_mode"].set(format_log_value(controller.get("bot_mode")))
        # Controller state input is the previous frame's output.
        state_in = previous_controller.get("state")
        self.values["state_in"].set(format_log_value(state_in))
        self.values["direction"].set(format_log_value(controller.get("direction")))
        self.values["speed"].set(format_log_value(controller.get("speed")))
        self.values["rotation"].set(format_log_value(controller.get("rotation")))
        self.values["state"].set(format_log_value(controller.get("state")))
        self.values["kick"].set(format_log_value(controller.get("kick")))
        self.values["dribbler"].set(format_log_value(controller.get("dribbler")))
        other_bots = frame.get("other_bots") or []
        if not other_bots:
            self._other_bots_var.set("—")
        else:
            self._other_bots_var.set(
                "\n".join(
                    f"Bot {i + 1}: ({bot_x}, {bot_y})"
                    for i, (bot_x, bot_y) in enumerate(other_bots)
                )
            )


class LogPlaybackControls:
    """Separate tkinter window for pause / play / scrub during log playback."""

    def __init__(
        self,
        frame_count: int,
        playback_fps: float | None = None,
        playback_times: Sequence[float] | None = None,
    ):
        self.frame_count = max(frame_count, 1)
        self.playback_fps = playback_fps
        self.playback_times = list(playback_times) if playback_times else None
        self.frame_index = 0
        self.playing = True
        self.closed = False
        self._updating_slider = False

        self.root = tk.Tk()
        self.root.title("Log Playback")
        self.root.resizable(True, False)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        self.play_button = tk.Button(
            self.root,
            text="Pause",
            width=8,
            command=self.toggle_play,
        )
        self.play_button.grid(row=0, column=0, padx=(12, 6), pady=12)

        self.step_back_button = tk.Button(
            self.root,
            text="|<",
            width=4,
            command=lambda: self.seek(self.frame_index - 1),
        )
        self.step_back_button.grid(row=0, column=1, padx=3, pady=12)

        self.step_forward_button = tk.Button(
            self.root,
            text=">|",
            width=4,
            command=lambda: self.seek(self.frame_index + 1),
        )
        self.step_forward_button.grid(row=0, column=2, padx=3, pady=12)

        self.slider = tk.Scale(
            self.root,
            from_=0,
            to=self.frame_count - 1,
            orient=tk.HORIZONTAL,
            length=420,
            showvalue=False,
            command=self._on_slider,
        )
        self.slider.grid(row=0, column=3, padx=6, pady=12, sticky="ew")
        self.slider.bind("<ButtonPress-1>", self._on_scrub_start)
        self.slider.bind("<ButtonRelease-1>", self._on_scrub_end)

        self.status_label = tk.Label(self.root, text="", width=16, anchor="e")
        self.status_label.grid(row=0, column=4, padx=(6, 12), pady=12)

        self.root.columnconfigure(3, weight=1)
        self._was_playing_before_scrub = False
        self._scrubbing = False
        self._last_advance_time = time.monotonic()
        self._fractional_frames = 0.0
        self._timeline_target_time = (
            self.playback_times[0] if self.playback_times else 0.0
        )
        self._refresh_status()

    def _on_close(self) -> None:
        self.closed = True
        try:
            self.root.destroy()
        except tk.TclError:
            pass

    def close(self) -> None:
        if not self.closed:
            self._on_close()

    def _on_scrub_start(self, _event=None) -> None:
        self._scrubbing = True
        self._was_playing_before_scrub = self.playing
        self.playing = False
        self._refresh_play_button()

    def _on_scrub_end(self, _event=None) -> None:
        self._scrubbing = False
        self.playing = self._was_playing_before_scrub
        self._refresh_play_button()

    def _on_slider(self, value: str) -> None:
        if self._updating_slider:
            return
        index = int(float(value))
        # Tk can deliver Scale.set() callbacks on the next event pump, after
        # _updating_slider is cleared. Do not reset the playback clock for
        # our own position updates: that discards elapsed time every frame.
        if index != self.frame_index:
            self.seek(index, update_slider=False)

    def _refresh_play_button(self) -> None:
        self.play_button.config(text="Pause" if self.playing else "Play")

    def _refresh_status(self) -> None:
        if self.playback_times:
            elapsed = self.playback_times[self.frame_index]
            duration = self.playback_times[-1]
            text = f"{elapsed:.2f}s / {duration:.2f}s"
        elif self.playback_fps:
            elapsed = self.frame_index / self.playback_fps
            duration = (self.frame_count - 1) / self.playback_fps
            text = f"{elapsed:.2f}s / {duration:.2f}s"
        else:
            text = f"{self.frame_index + 1} / {self.frame_count}"
        self.status_label.config(text=text)

    def toggle_play(self) -> None:
        if self.frame_index >= self.frame_count - 1 and not self.playing:
            self.seek(0)
        self.playing = not self.playing
        self._last_advance_time = time.monotonic()
        self._fractional_frames = 0.0
        if self.playback_times:
            self._timeline_target_time = self.playback_times[self.frame_index]
        self._refresh_play_button()

    def seek(self, index: int, update_slider: bool = True) -> None:
        self.frame_index = max(0, min(int(index), self.frame_count - 1))
        self._last_advance_time = time.monotonic()
        self._fractional_frames = 0.0
        if self.playback_times:
            self._timeline_target_time = self.playback_times[self.frame_index]
        if update_slider:
            self._updating_slider = True
            try:
                self.slider.set(self.frame_index)
            finally:
                self._updating_slider = False
        self._refresh_status()

    def advance_if_playing(self) -> None:
        now = time.monotonic()
        elapsed = now - self._last_advance_time
        self._last_advance_time = now
        if not self.playing or self._scrubbing:
            return
        if self.frame_index >= self.frame_count - 1:
            self.playing = False
            self._refresh_play_button()
            return
        if self.playback_times:
            self._timeline_target_time += elapsed
            new_index = bisect.bisect_right(
                self.playback_times,
                self._timeline_target_time,
            ) - 1
            new_index = max(self.frame_index, min(new_index, self.frame_count - 1))
            if new_index == self.frame_index:
                return
            steps = new_index - self.frame_index
        elif self.playback_fps is None:
            steps = 1
        else:
            self._fractional_frames += elapsed * self.playback_fps
            steps = int(self._fractional_frames)
            self._fractional_frames -= steps
            if steps == 0:
                return
        new_index = min(self.frame_index + steps, self.frame_count - 1)
        self.frame_index = new_index
        self._updating_slider = True
        try:
            self.slider.set(self.frame_index)
        finally:
            self._updating_slider = False
        self._refresh_status()

    def pump(self) -> bool:
        """Process UI events. Returns False if the control window was closed."""
        if self.closed:
            return False
        try:
            self.root.update_idletasks()
            self.root.update()
        except tk.TclError:
            self.closed = True
            return False
        return not self.closed


if args.connect:
    log_client_stop_event.clear()
    log_thread = start_log_client(args.connect, log_client_stop_event)
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        if log_line is not None:
            line_data = log_line.strip().split(",")
            render_line(line_data)
            log_line = None
        clock.tick(FPS)
    log_client_stop_event.set()
    log_thread.join(timeout=2.0)
    pygame.quit()
    sys.exit()
elif session_provided:
    try:
        recorded_session = load_recorded_session(args.log_file)
        video_reader = VideoReader(recorded_session.video_path)
    except (OSError, ValueError, RuntimeError) as exc:
        print(f"Unable to load recording session '{args.log_file}': {exc}")
        pygame.quit()
        sys.exit(1)

    game_timeline = EventTimeline(recorded_session.game_events, "elapsed_s")
    timed_detections = [
        event
        for event in recorded_session.detection_events
        if event.get("video_time_s") is not None
    ]
    detection_timeline = EventTimeline(timed_detections, "video_time_s")
    metadata_fps = float(recorded_session.metadata.get("requested_fps", 30.0))
    video_fps = video_reader.fps
    if not 1.0 <= video_fps <= 240.0:
        video_fps = metadata_fps
    duration_s = video_reader.duration_s
    if duration_s <= 0:
        duration_s = max(
            0.0,
            float(recorded_session.metadata.get("duration_s", 0.0))
            - float(recorded_session.metadata.get("video_start_elapsed_s") or 0.0),
        )
    playback_times = video_reader.frame_times
    if not playback_times:
        frame_count = max(1, round(duration_s * video_fps))
        playback_times = [index / video_fps for index in range(frame_count)]
    frame_count = len(playback_times)
    video_start_elapsed_s = float(
        recorded_session.metadata.get("video_start_elapsed_s") or 0.0
    )

    pygame.display.set_caption("Soccer Session Playback")
    controls = LogPlaybackControls(
        frame_count,
        playback_fps=video_fps,
        playback_times=playback_times,
    )
    debug = LogControllerDebug(controls.root)
    last_rendered_index = None
    running = True
    try:
        while running:
            if not controls.pump():
                break
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_SPACE:
                        controls.toggle_play()
                    elif event.key == pygame.K_LEFT:
                        controls.seek(controls.frame_index - 1)
                    elif event.key == pygame.K_RIGHT:
                        controls.seek(controls.frame_index + 1)

            controls.advance_if_playing()
            frame_index = controls.frame_index
            if frame_index != last_rendered_index:
                video_time_s = playback_times[frame_index]
                session_time_s = video_start_elapsed_s + video_time_s
                game_event = game_timeline.latest(session_time_s)
                parsed_frame = (
                    parse_log_frame(game_event_tokens(game_event))
                    if game_event is not None
                    else None
                )
                previous_event = (
                    game_timeline.latest(float(game_event["elapsed_s"]) - 1e-9)
                    if game_event is not None
                    else None
                )
                previous_frame = (
                    parse_log_frame(game_event_tokens(previous_event))
                    if previous_event is not None
                    else None
                )
                video_rgb = video_reader.frame_at(video_time_s)
                if video_rgb is None:
                    break
                detection = detection_timeline.nearest(
                    video_time_s,
                    tolerance=0.6 / video_fps,
                )
                annotated_video = annotate_video_frame(video_rgb, detection)
                if parsed_frame is None:
                    pitch_frame = build_frame(*CENTRE_POINT, [])
                else:
                    pitch_frame = build_log_pitch(parsed_frame)
                blit_pitch_and_video(pitch_frame, annotated_video)
                debug.update(parsed_frame, previous_frame)
                last_rendered_index = frame_index
            clock.tick(FPS)
    finally:
        video_reader.close()
        debug.close()
        if not controls.closed:
            controls.close()
        pygame.quit()
    sys.exit(0)
elif log_provided:
    frames = [line.strip().split(",") for line in lines if line.strip()]
    if not frames:
        print(f"Log file '{args.log_file}' has no frames to play back.")
        pygame.quit()
        sys.exit(1)

    pygame.display.set_caption("Soccer Log Playback")
    controls = LogPlaybackControls(len(frames))
    debug = LogControllerDebug(controls.root)
    last_debug_index = None
    running = True
    while running:
        if not controls.pump():
            break
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    controls.toggle_play()
                elif event.key == pygame.K_LEFT:
                    controls.seek(controls.frame_index - 1)
                elif event.key == pygame.K_RIGHT:
                    controls.seek(controls.frame_index + 1)
        controls.advance_if_playing()
        frame_index = controls.frame_index
        if frame_index != last_debug_index:
            frame = render_line(frames[frame_index])
            previous = (
                parse_log_frame(frames[frame_index - 1]) if frame_index > 0 else None
            )
            debug.update(frame, previous)
            last_debug_index = frame_index
        clock.tick(LOG_FPS)
    debug.close()
    if not controls.closed:
        controls.close()
    pygame.quit()
    sys.exit(0)
else:
    ball_x = pitch.get_width() // 2
    ball_y = pitch.get_height() // 2
    ball_vx = 0.0
    ball_vy = 0.0

    team_mode = bool(args.team1 or args.team2)
    bots: list[Bot] = []
    if team_mode:
        if args.team1:
            bots.extend(create_team(args.team1, 1))
        if args.team2:
            bots.extend(create_team(args.team2, 2))
        if args.defence or args.goalie or args.striker or args.test_bot:
            print("Team selections override single-bot '-d'/'-g'/'-s' flags.")
    else:
        start_x = 500
        start_y = pitch.get_height() // 2
        controller = None
        if args.defence:
            controller = defence
        elif args.goalie:
            controller = goalie
        elif args.striker:
            controller = striker
        elif args.test_bot:
            controller = bot
        bots.append(
            Bot(
                x=start_x,
                y=start_y,
                yaw=0.0,
                base_color=yellow,
                controller=controller,
                manual=controller is None,
                name="Player",
            )
        )

    waiting = True
    while waiting:
        dt_seconds = clock.tick(FPS) / 1000.0
        dt_safe = max(dt_seconds, EPSILON)
        yaw_correction_deg_per_frame = math.degrees(
            (YAW_CORRECT_PIXELS_PER_S * dt_seconds) / ROBOT_RADIUS
        )
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                waiting = False
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                # Hide the ball the same way a missed camera detection does.
                ball_x = None
                ball_y = None
                ball_vx = 0.0
                ball_vy = 0.0
            elif event.type == pygame.MOUSEBUTTONDOWN:
                ball_position = window_to_frame_point(event.pos[0], event.pos[1], pitch)
                if ball_position is not None:
                    ball_x, ball_y = ball_position
                    ball_vx = 0.0
                    ball_vy = 0.0

        ball_on_field = ball_x is not None and ball_y is not None
        manual_keys = pygame.key.get_pressed() if any(bot.manual for bot in bots) else None

        bot_states = []
        current_time = pygame.time.get_ticks()
        for bot in bots:
            prev_x = bot.x
            prev_y = bot.y
            ball_captured = ball_on_field and is_ball_touching_capture_zone(
                ball_x, ball_y, get_capture_geometry(bot.x, bot.y, bot.yaw)
            )
            controller_ball_captured = ball_captured
            kick_state = False
            dribbler_state = False

            if bot.manual:
                if manual_keys is None:
                    manual_keys = pygame.key.get_pressed()
                direction, speed, rotation, kick_state = manual_control_from_keys(manual_keys, bot.yaw)
            elif bot.controller is not None:
                controller_x = bot.x
                controller_y = bot.y
                controller_yaw = bot.yaw
                controller_ball_x = ball_x
                controller_ball_y = ball_y
                ball_is_visible = not (
                    args.vision
                    and ball_on_field
                    and not ball_visible_from(bot, ball_x, ball_y, bots)
                )
                if not ball_is_visible:
                    controller_ball_x = None
                    controller_ball_y = None
                observed_ball_position = (
                    (controller_ball_x, controller_ball_y)
                    if controller_ball_x is not None and controller_ball_y is not None
                    else None
                )
                controller_friendly_bot_positions = [
                    (other_bot.x, other_bot.y)
                    for other_bot in bots
                    if other_bot is not bot and other_bot.base_color == bot.base_color
                ]
                controller_enemy_bot_positions = [
                    (other_bot.x, other_bot.y)
                    for other_bot in bots
                    if other_bot.base_color != bot.base_color
                ]
                if args.vision_ball_possession:
                    if observed_ball_position is not None:
                        bot.self_ball_candidate = ball_is_near_bot(
                            observed_ball_position, (bot.x, bot.y)
                        )
                    predicted_ball_position = bot.ball_extrapolator.update(
                        observed_ball_position, current_time / 1000.0
                    )
                    if ball_captured:
                        bot.ball_possession_tracker.clear()
                    else:
                        carried_ball_position = bot.ball_possession_tracker.update(
                            observed_ball_position,
                            controller_enemy_bot_positions,
                            current_time / 1000.0,
                            new_camera_frame=True,
                        )
                        if observed_ball_position is None:
                            if predicted_ball_position is not None:
                                controller_ball_x, controller_ball_y = (
                                    predicted_ball_position
                                )
                            elif bot.ball_extrapolator.timed_out(
                                current_time / 1000.0
                            ):
                                if bot.self_ball_candidate:
                                    controller_ball_captured = True
                                    bot.ball_possession_tracker.clear()
                                    controller_ball_x = bot.x + 100 * math.cos(
                                        math.radians(bot.yaw)
                                    )
                                    controller_ball_y = bot.y + 100 * math.sin(
                                        math.radians(bot.yaw)
                                    )
                                elif carried_ball_position is not None:
                                    controller_ball_x, controller_ball_y = (
                                        carried_ball_position
                                    )
                controller_inverted = (
                    bot.controller in (defence, striker, goalie, bot) and bot.base_color != yellow
                )
                if controller_inverted:
                    controller_x, controller_y = invert_pitch_point(bot.x, bot.y)
                    controller_yaw = invert_angle_deg(bot.yaw)
                    controller_ball_x, controller_ball_y = invert_optional_pitch_point(
                        controller_ball_x, controller_ball_y
                    )
                    controller_friendly_bot_positions = [
                        invert_pitch_point(other_x, other_y)
                        for other_x, other_y in controller_friendly_bot_positions
                    ]
                    controller_enemy_bot_positions = [
                        invert_pitch_point(other_x, other_y)
                        for other_x, other_y in controller_enemy_bot_positions
                    ]

                direction, speed, rotation, bot.state, kick_state, dribbler_state = bot.controller(
                    controller_x,
                    controller_y,
                    controller_yaw,
                    controller_ball_x,
                    controller_ball_y,
                    controller_ball_captured,
                    state=bot.state,
                    friendly_bot_positions=controller_friendly_bot_positions,
                    enemy_bot_positions=controller_enemy_bot_positions,
                )
                if controller_inverted:
                    direction = invert_angle_deg(direction)
                    rotation = invert_angle_deg(rotation)
            else:
                direction, speed, rotation = 0, 0, bot.yaw

            # Kick releases the ball; skip dribbler hold this frame so the impulse sticks.
            if kick_state and ball_captured:
                ball_vx += KICK_SPEED * math.cos(math.radians(bot.yaw))
                ball_vy += KICK_SPEED * math.sin(math.radians(bot.yaw))
                dribbler_state = False

            # Determine desired velocity vector in mm/s relative to world axes.
            if direction is not None:
                direction_rad = math.radians(direction)
                target_vx = speed * math.cos(direction_rad)
                target_vy = speed * math.sin(direction_rad)
            else:
                if speed <= EPSILON:
                    target_vx = 0.0
                    target_vy = 0.0
                else:
                    current_velocity_speed = math.hypot(bot.velocity_x, bot.velocity_y)
                    if current_velocity_speed > EPSILON:
                        scale = speed / current_velocity_speed
                        target_vx = bot.velocity_x * scale
                        target_vy = bot.velocity_y * scale
                    else:
                        forward_rad = math.radians(bot.yaw)
                        target_vx = speed * math.cos(forward_rad)
                        target_vy = speed * math.sin(forward_rad)

            # Clamp acceleration magnitude so bots decelerate before reversing.
            acceleration_limit = ACCELERATION * dt_seconds
            delta_vx = target_vx - bot.velocity_x
            delta_vy = target_vy - bot.velocity_y
            delta_magnitude = math.hypot(delta_vx, delta_vy)
            if delta_magnitude > acceleration_limit and delta_magnitude > EPSILON:
                scale = acceleration_limit / delta_magnitude
                delta_vx *= scale
                delta_vy *= scale

            bot.velocity_x += delta_vx
            bot.velocity_y += delta_vy

            rotation_target = rotation if rotation is not None else bot.yaw
            yaw_error = shortest_angle_delta(bot.yaw, rotation_target)
            if abs(yaw_error) > EPSILON:
                yaw_step = math.copysign(
                    min(abs(yaw_error), yaw_correction_deg_per_frame),
                    yaw_error,
                )
                bot.yaw = normalize_angle_deg(bot.yaw + yaw_step)

            bot_step_x = mmps_to_pixels(bot.velocity_x, dt_seconds)
            bot_step_y = mmps_to_pixels(bot.velocity_y, dt_seconds)

            bot.x += bot_step_x
            bot.y += bot_step_y

            if check_collision_with_goal_lines(bot.x, bot.y, BOT_RADIUS, GOAL_LINES):
                bot.x = prev_x
                bot.y = prev_y

            keep_bot_inside_white_lines(bot)
            bot_actual_dx = bot.x - prev_x
            bot_actual_dy = bot.y - prev_y
            bot.velocity_x = (bot_actual_dx * MM_PER_PIXEL) / dt_safe
            bot.velocity_y = (bot_actual_dy * MM_PER_PIXEL) / dt_safe
            bot.push_speed = math.hypot(bot_actual_dx, bot_actual_dy)
            bot.desired_velocity = (bot_step_x, bot_step_y)

            is_out_of_bounds = is_out_of_white_boundary(bot.x, bot.y, BOT_RADIUS)
            if is_out_of_bounds:
                bot.current_color = red
                bot.red_until_time = None
            else:
                if bot.was_out_of_bounds:
                    bot.red_until_time = current_time + RED_DURATION_MS
                bot.current_color = bot.base_color
                if bot.red_until_time is not None and current_time <= bot.red_until_time:
                    bot.current_color = red
                elif bot.red_until_time is not None and current_time > bot.red_until_time:
                    bot.red_until_time = None
            bot.was_out_of_bounds = is_out_of_bounds

            bot_states.append(
                {
                    "bot": bot,
                    "prev": (prev_x, prev_y),
                    "delta": (bot_actual_dx, bot_actual_dy),
                    "dribbler": dribbler_state,
                    "ball_captured": ball_captured,
                }
            )

        resolve_bot_collisions(bot_states)
        for state in bot_states:
            bot_ref = state["bot"]
            keep_bot_inside_white_lines(bot_ref)
            prev_x, prev_y = state["prev"]
            bot_actual_dx = bot_ref.x - prev_x
            bot_actual_dy = bot_ref.y - prev_y
            bot_ref.velocity_x = (bot_actual_dx * MM_PER_PIXEL) / dt_safe
            bot_ref.velocity_y = (bot_actual_dy * MM_PER_PIXEL) / dt_safe
            bot_ref.push_speed = math.hypot(bot_actual_dx, bot_actual_dy)
            state["delta"] = (bot_actual_dx, bot_actual_dy)

        if ball_on_field:
            # Sub-step the ball movement so it cannot tunnel through thin walls/backboards
            ball_speed_pixels = mmps_to_pixels(math.hypot(ball_vx, ball_vy), 1.0)
            max_ball_step = max(BALL_RADIUS, 1.0)
            substeps = max(1, math.ceil((ball_speed_pixels * dt_seconds) / max_ball_step))
            sub_dt = dt_seconds / substeps if substeps > 0 else dt_seconds

            boundary_collision = False
            goal_line_collision = False

            for _ in range(substeps):
                ball_x += mmps_to_pixels(ball_vx, sub_dt)
                ball_y += mmps_to_pixels(ball_vy, sub_dt)

                ball_x, ball_y, ball_vx, ball_vy, boundary_hit = keep_ball_in_pitch_bounds(
                    ball_x, ball_y, ball_vx, ball_vy
                )
                ball_x, ball_y, ball_vx, ball_vy, goal_hit = resolve_ball_goal_line_collisions(
                    ball_x, ball_y, ball_vx, ball_vy, GOAL_LINES
                )

                boundary_collision = boundary_collision or boundary_hit
                goal_line_collision = goal_line_collision or goal_hit

            ball_being_pushed = False
            pushing_states = []
            for idx, state in enumerate(bot_states):
                geometry = get_capture_geometry(state["bot"].x, state["bot"].y, state["bot"].yaw)
                ball_x, ball_y, ball_vx, ball_vy, bot_pushing = resolve_ball_capture_collisions(
                    ball_x,
                    ball_y,
                    ball_vx,
                    ball_vy,
                    geometry,
                    (state["bot"].velocity_x, state["bot"].velocity_y),
                )
                if bot_pushing:
                    pushing_states.append({"state": state, "geometry": geometry})
                    ball_being_pushed = True

            # Determine if ball is pinched between bots (normals roughly opposing)
            ball_pinched_between_bots = False
            if len(pushing_states) >= 2:
                normals = []
                for entry in pushing_states:
                    bot = entry["state"]["bot"]
                    geometry = entry["geometry"]
                    # Approximate outward normal from bot to ball
                    diff_x = ball_x - bot.x
                    diff_y = ball_y - bot.y
                    norm = math.hypot(diff_x, diff_y)
                    if norm > EPSILON:
                        normals.append((diff_x / norm, diff_y / norm))
                for i in range(len(normals)):
                    for j in range(i + 1, len(normals)):
                        dot = normals[i][0] * normals[j][0] + normals[i][1] * normals[j][1]
                        if dot <= -0.3:  # roughly opposing
                            ball_pinched_between_bots = True
                            break
                    if ball_pinched_between_bots:
                        break

            ball_against_surface = (
                boundary_collision
                or goal_line_collision
                or is_ball_touching_pitch_bounds(ball_x, ball_y)
                or is_ball_touching_goal_lines(ball_x, ball_y, GOAL_LINES)
            )

            pushing_with_motion = any(
                abs(entry["state"]["delta"][0]) > EPSILON or abs(entry["state"]["delta"][1]) > EPSILON
                for entry in pushing_states
            )

            if (ball_against_surface or ball_pinched_between_bots) and pushing_states and pushing_with_motion:
                for entry in pushing_states:
                    bot_ref = entry["state"]["bot"]
                    prev_x, prev_y = entry["state"]["prev"]
                    bot_ref.x = prev_x
                    bot_ref.y = prev_y
                    entry["state"]["delta"] = (0.0, 0.0)
                ball_vx = 0.0
                ball_vy = 0.0
                for bot in bots:
                    geometry = get_capture_geometry(bot.x, bot.y, bot.yaw)
                    ball_x, ball_y, ball_vx, ball_vy, _ = resolve_ball_capture_collisions(
                        ball_x, ball_y, ball_vx, ball_vy, geometry, (0.0, 0.0)
                    )
                ball_being_pushed = False

            if not ball_being_pushed:
                ball_speed = math.hypot(ball_vx, ball_vy)
                ball_deceleration = BALL_DECELERATION_SPEED * dt_seconds
                if ball_speed > ball_deceleration:
                    scale = (ball_speed - ball_deceleration) / ball_speed
                    ball_vx *= scale
                    ball_vy *= scale
                else:
                    ball_vx = 0.0
                    ball_vy = 0.0

            for bot in bots:
                if is_ball_inside_bot_body(ball_x, ball_y, bot):
                    diff_x = ball_x - bot.x
                    diff_y = ball_y - bot.y
                    norm = math.hypot(diff_x, diff_y)
                    if norm < EPSILON:
                        diff_x, diff_y, norm = 1.0, 0.0, 1.0
                    normal_x = diff_x / norm
                    normal_y = diff_y / norm
                    push_dist = ROBOT_RADIUS + BALL_RADIUS + CAPTURE_LINE_HALF_WIDTH
                    ball_x = bot.x + normal_x * push_dist
                    ball_y = bot.y + normal_y * push_dist
            for bot in bots:
                if is_ball_inside_bot_body(ball_x, ball_y, bot):
                    reset_point = find_nearest_free_black_point(ball_x, ball_y, bots)
                    if reset_point is not None:
                        ball_x, ball_y = reset_point
                    ball_vx = 0.0
                    ball_vy = 0.0
                    break

            # While the dribbler is on and the ball was in the capture zone, keep it fixed there.
            for state in bot_states:
                if not state["dribbler"] or not state["ball_captured"]:
                    continue
                bot = state["bot"]
                ball_x, ball_y = get_dribbled_ball_position(bot)
                ball_vx = bot.velocity_x
                ball_vy = bot.velocity_y
                break

        frame_pitch = build_frame(ball_x, ball_y, bots)
        blit_frame(frame_pitch)

pygame.quit()
sys.exit()
