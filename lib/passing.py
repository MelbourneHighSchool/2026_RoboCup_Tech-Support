"""Hardware-independent passing geometry and symmetric handshake.

Call step_pass while a role has selected passing/receiving. Retain its PassState
and send state.outgoing after each step. Each attempt permits one kick; call
state.reset_attempt() to rearm while preserving communication sequence numbers.
"""

import math
import secrets
from dataclasses import asdict, dataclass, field
from enum import StrEnum

from lib.bot_fusion import project_bots
from state import FusionSource


@dataclass(frozen=True)
class PassConfig:
    match_mm: float = 100.0
    bearing_match_deg: float = 3.0
    max_bounds_width_mm: float = 400.0
    lane_clearance_mm: float = 500.0  # Distance from other BOT CENTRES.
    lane_margin_mm: float = 30.0
    max_peer_age_s: float = 0.2
    max_observation_age_s: float = 0.15
    min_distance_mm: float = 350.0
    max_distance_mm: float = 2000.0

    def __post_init__(self):
        if any(not finite(v) or v < 0 for v in asdict(self).values()):
            raise ValueError("Pass tolerances must be finite and nonnegative")
        if (self.match_mm <= 0 or self.max_peer_age_s <= 0
                or self.max_observation_age_s <= 0
                or self.min_distance_mm <= 220 or self.max_distance_mm <= self.min_distance_mm
                or not 0 < self.bearing_match_deg <= 10):
            raise ValueError("Invalid pass timing, separation or angle tolerance")


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def valid_pose(pose):
    return isinstance(pose, (tuple, list)) and len(pose) >= 3 and all(finite(v) for v in pose[:3])


def angle_error(a, b):
    return math.remainder(a - b, 360)


def bearing(a, b):
    return math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))


def segment_distance(point, start, end):
    dx, dy = end[0] - start[0], end[1] - start[1]
    length2 = dx * dx + dy * dy
    fraction = (max(0, min(1, ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy)
                           / length2)) if length2 else 0)
    return math.hypot(point[0] - start[0] - fraction * dx,
                      point[1] - start[1] - fraction * dy)


def lane_clear(start, end, obstacles, config):
    return all(segment_distance(p, start, end) > config.lane_clearance_mm + config.lane_margin_mm
               for p in obstacles)


@dataclass(frozen=True)
class PassCandidate:
    """Owned geometry verified for one tick; never cache across control ticks."""

    target_pose: tuple[float, float, float]
    obstacles: tuple[tuple[float, float], ...]

    def lane_clear(self, start, end, config):
        """Check an actual kick segment against the already verified scene."""
        return lane_clear(start, end, self.obstacles, config)


def evaluate_pass(pose, peer_snapshot, evidence, now, config=None):
    """Return a verified candidate for this tick, or None when unavailable.

    peer_snapshot is Peer.receive_snapshot(), with message["pose"] in field
    coordinates. evidence comes from evaluate_bot_scene(); now is monotonic.
    Every other visible bot must clear the finite centre-to-centre segment by
    500 mm plus a 30 mm uncertainty margin by default. Unknown ranges veto.
    This function has no state and does not check possession or execute a pass.
    """
    config = config or PassConfig()
    if not isinstance(peer_snapshot, dict):
        return None
    message, received = peer_snapshot.get("message"), peer_snapshot.get("received_at")
    if (not isinstance(message, dict) or not finite(received)
            or not 0 <= now - received <= config.max_peer_age_s):
        return None
    peer_pose = message.get("pose")
    if not valid_pose(pose) or not valid_pose(peer_pose):
        return None
    distance = math.dist(pose[:2], peer_pose[:2])
    if not config.min_distance_mm <= distance <= config.max_distance_mm:
        return None
    if not evidence:
        return None
    scene, context = evidence["scene"], evidence["context"]
    stamp = scene.get("timestamp_s")
    if not finite(stamp) or not 0 <= now - stamp <= config.max_observation_age_s:
        return None
    if context["reason"] != "ok" or not valid_pose(context.get("pose")):
        return None
    historical = context["pose"]
    # Movement during the frame/packet interval is deliberately not extrapolated
    # here. Large localisation changes invalidate the match.
    if math.dist(historical[:2], pose[:2]) > config.match_mm:
        return None
    bots, results = scene["bots"], evidence["results"]
    if len(bots) != len(results):
        return None
    matches = []
    for index, bot in enumerate(bots):
        positions = project_bots([bot], historical)
        if positions and math.dist(positions[0], peer_pose[:2]) <= config.match_mm:
            matches.append(index)
    if len(matches) != 1:
        return None
    target_index = matches[0]
    target = results[target_index]
    if target["source"] != FusionSource.FUSED or target["bounds_mm"] is None:
        return None
    low, high = target["bounds_mm"]
    peer_range = math.dist(historical[:2], peer_pose[:2])
    peer_bearing = angle_error(bearing(historical, peer_pose), historical[2])
    if (not low <= peer_range <= high or high - low > config.max_bounds_width_mm
            or abs(angle_error(peer_bearing, target["bearing_deg"])) > config.bearing_match_deg):
        return None
    target_position = project_bots([target], historical)
    if not target_position or math.dist(target_position[0], peer_pose[:2]) > config.match_mm:
        return None
    obstacles = []
    for index, result in enumerate(results):
        if index == target_index:
            continue
        positions = project_bots([result], historical)
        if not positions:
            return None
        position = positions[0]
        if math.dist(position, historical[:2]) <= 130:  # Own 110 mm body + small margin.
            continue
        obstacles.append(position)
    if not lane_clear(pose, peer_pose, obstacles, config):
        return None
    return PassCandidate(tuple(peer_pose[:3]), tuple(obstacles))


def pass_available(pose, peer_snapshot, evidence, now, config=None, *, kick_segment=None):
    """Boolean convenience API, optionally checking the actual ball path too."""
    config = config or PassConfig()
    candidate = evaluate_pass(pose, peer_snapshot, evidence, now, config)
    return candidate is not None and (
        kick_segment is None or candidate.lane_clear(*kick_segment, config)
    )


class PassRole(StrEnum):
    UNASSIGNED = "unassigned"
    SENDER = "sender"
    RECEIVER = "receiver"


class PassPhase(StrEnum):
    UNKNOWN = "unknown"
    WAITING_FOR_BALL = "waiting_for_ball"
    WAITING_FOR_POSE = "waiting_for_pose"
    WAITING_FOR_PEER = "waiting_for_peer"
    WAITING_FOR_RECEIVER = "waiting_for_receiver"
    BOTH_HAVE_BALL = "both_have_ball"
    FACING_PEER = "facing_peer"
    SETTLING = "settling"
    READY_TO_RECEIVE = "ready_to_receive"
    ALIGNING_KICK = "aligning_kick"
    PASS_SENT = "pass_sent"
    RECEIVING = "receiving"
    RECEIVED = "received"
    DELIVERED = "delivered"
    PASS_TIMEOUT = "pass_timeout"
    RECEIVE_TIMEOUT = "receive_timeout"
    UNAVAILABLE = "unavailable"


PROTOCOL = "soccer-verified-pass-v1"


@dataclass(frozen=True)
class PassControllerConfig(PassConfig):
    yaw_tolerance_deg: float = 3.0
    max_gyro_deg_s: float = 10.0
    receive_half_width_mm: float = 50.0
    settle_s: float = 0.15
    receive_timeout_s: float = 3.0

    def __post_init__(self):
        super().__post_init__()
        if (self.settle_s <= 0 or self.receive_timeout_s <= 0
                or not 0 < self.yaw_tolerance_deg <= 10):
            raise ValueError("Pass settling and receive timeout must be positive")


@dataclass
class PassState:
    role: PassRole = PassRole.UNASSIGNED
    phase: PassPhase = PassPhase.WAITING_FOR_BALL
    peer_id: str | None = None
    peer_sequence: int = -1
    peer_message: dict | None = None
    peer_received_at: float | None = None
    attempt: str | None = None
    kicked: bool = False
    finished: bool = False
    fired_at: float | None = None
    settled_since: float | None = None
    last_frame: int | None = None
    settled_frames: int = 0
    sequence: int = 0
    sent_at: dict = field(default_factory=dict)
    outgoing: dict = field(default_factory=dict)

    def __post_init__(self):
        # Accept previously saved string values at the construction boundary.
        self.role = PassRole.UNASSIGNED if self.role is None else PassRole(self.role)
        self.phase = PassPhase(self.phase)

    def reset_settling(self):
        self.settled_since = None
        self.last_frame = None
        self.settled_frames = 0

    def reset_attempt(self):
        """Cancel/rearm a pass while preserving peer identity and packet ordering.

        Keep this state across role changes within one communication session.
        A new session/bot identity needs a new PassState instead.
        """
        self.role = PassRole.UNASSIGNED
        self.phase = PassPhase.WAITING_FOR_BALL
        self.attempt = None
        self.kicked = False
        self.finished = False
        self.fired_at = None
        self.reset_settling()
        self.sent_at.clear()
        self.outgoing.clear()


def _peer(state, snapshot, bot_id, session, now, config):
    """Accept monotonic packet sequences; duplicates never refresh receipt age."""
    if snapshot is not None:
        message, received = snapshot.get("message"), snapshot.get("received_at")
        if isinstance(message, dict):
            identity, sequence = message.get("bot_id"), message.get("sequence")
            acceptable = (
                message.get("protocol") == PROTOCOL and message.get("session") == session
                and isinstance(identity, str) and identity and identity != bot_id
                and message.get("to_id") in (None, bot_id)
                and state.peer_id in (None, identity)
                and isinstance(sequence, int) and not isinstance(sequence, bool) and sequence >= 0
                and isinstance(message.get("has_ball"), bool)
                and finite(received) and 0 <= now - received <= config.max_peer_age_s
            )
            if acceptable and sequence > state.peer_sequence:
                state.peer_id, state.peer_sequence = identity, sequence
                state.peer_message, state.peer_received_at = dict(message), received
                try:
                    state.peer_message["phase"] = PassPhase(message.get("phase", PassPhase.UNKNOWN))
                except (ValueError, TypeError):
                    state.peer_message["phase"] = PassPhase.UNKNOWN
    if state.peer_received_at is None or not 0 <= now - state.peer_received_at <= config.max_peer_age_s:
        return None
    return state.peer_message


def _receiver_ready(state, peer, pose, target, now, config):
    """Validate receiver alignment and its acknowledgement on our own clock."""
    peer_age = peer.get("observation_age_s")
    ready_target = peer.get("ready_target")
    acknowledged = peer.get("ready_for_sequence")
    acknowledged_at = (state.sent_at.get(acknowledged)
                       if isinstance(acknowledged, int) and not isinstance(acknowledged, bool) else None)
    return (
        peer.get("ready_for") == state.attempt and not peer["has_ball"]
        and isinstance(ready_target, (list, tuple)) and len(ready_target) == 2
        and all(finite(v) for v in ready_target)
        and math.dist(ready_target, pose[:2]) <= config.match_mm
        and finite(peer_age) and peer_age >= 0 and acknowledged_at is not None
        # Echoing an intent packet bounds the whole round trip on OUR clock;
        # a newly received but delayed ready packet must not appear fresh.
        and 0 <= peer_age + now - acknowledged_at <= config.max_observation_age_s
        and finite(peer.get("gyro_deg_s")) and abs(peer["gyro_deg_s"]) <= config.max_gyro_deg_s
        and abs(angle_error(bearing(target, pose), target[2])) <= config.yaw_tolerance_deg
    )


def _kick_segment(pose, target, config):
    """Return the held-ball ray to the receiver, or None if it misses."""
    x_pos, y_pos, yaw = pose
    c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
    dx, dy = target[0] - x_pos, target[1] - y_pos
    along, across = dx * c + dy * s, -dx * s + dy * c
    if along <= 100 or abs(across) > config.receive_half_width_mm:
        return None
    return ((x_pos + 100 * c, y_pos + 100 * s),
            (x_pos + along * c, y_pos + along * s))


def step_pass(pose, ball_captured=False, *, bot_id, session, now,
            gyro_deg_s, peer_snapshot=None, evidence=None, state=None, config=None):
    """Advance one pass attempt without accessing hardware.

    Send state.outgoing each tick via Peer.send(). Both robots run this function.
    The first holder sends once; the receiver never automatically passes back.
    pose is (x_mm, y_mm, clockwise_yaw_deg) in the evidence/peer field frame.
    ball_captured must be confirmed possession, not a predicted ball position.
    Returns (direction, speed, rotation, pass_state, kick, dribbler). Translation
    is zero, dribbler stays on, and kick is true for just one tick per attempt.
    Finished attempts hold until the caller invokes state.reset_attempt().
    """
    config = config or PassControllerConfig()
    state = state if isinstance(state, PassState) else PassState()
    yaw = pose[2]
    peer = _peer(state, peer_snapshot, bot_id, session, now, config)
    target = peer.get("pose") if peer else None
    pose_ok = valid_pose(pose) and finite(gyro_deg_s)
    scene = evidence.get("scene", {}) if evidence else {}
    stamp = scene.get("timestamp_s")
    age = now - stamp if finite(stamp) and stamp <= now else None

    def output(phase: PassPhase, rotation=None, kick=False, ready=None, intent=None):
        state.phase = PassPhase(phase)
        state.sequence += 1
        state.sent_at = {seq: sent for seq, sent in state.sent_at.items()
                         if 0 <= now - sent <= config.max_peer_age_s}
        state.sent_at[state.sequence] = now
        state.outgoing = {
            "protocol": PROTOCOL, "session": session, "sequence": state.sequence,
            "to_id": state.peer_id, "pose": list(pose) if pose_ok else None,
            "pass_id": state.attempt, "has_ball": bool(ball_captured), "gyro_deg_s": gyro_deg_s if pose_ok else None,
            "intent": intent, "ready_for": ready,
            "ready_for_sequence": peer["sequence"] if ready else None,
            "ready_target": list(target[:2]) if ready else None,
            "observation_age_s": age, "kicked": state.attempt if state.kicked else None,
            "finished": state.finished, "phase": phase,
        }
        # Both dribblers remain ON, including during the single kick command.
        return 0, 0, rotation if rotation is not None else (yaw if finite(yaw) else 0), state, kick, 1

    if state.role == PassRole.RECEIVER and state.attempt and ball_captured:
        state.finished = True
        return output(PassPhase.RECEIVED)
    if state.finished:
        return output(state.phase)
    if state.kicked:
        if (peer and peer.get("finished") and peer.get("phase") == PassPhase.RECEIVED
                and peer.get("pass_id") == state.attempt and peer["has_ball"]):
            state.finished = True
            return output(PassPhase.DELIVERED)
        if now - state.fired_at > config.receive_timeout_s:
            state.finished = True
            return output(PassPhase.PASS_TIMEOUT)
        return output(PassPhase.PASS_SENT)
    if not pose_ok or not peer or not valid_pose(target):
        state.reset_settling()
        return output(PassPhase.WAITING_FOR_POSE if not pose_ok else PassPhase.WAITING_FOR_PEER)
    if ball_captured and peer["has_ball"]:
        state.reset_settling()
        return output(PassPhase.BOTH_HAVE_BALL)
    if state.role is PassRole.UNASSIGNED:
        if ball_captured:
            state.role = PassRole.SENDER
        elif peer["has_ball"] and isinstance(peer.get("intent"), str) and peer["intent"]:
            state.role = PassRole.RECEIVER
    if state.role == PassRole.SENDER:
        if not ball_captured:
            state.attempt = None
            state.reset_settling()
            return output(PassPhase.WAITING_FOR_BALL)
        if state.attempt is None:
            state.attempt = secrets.token_hex(8)
    elif state.role == PassRole.RECEIVER:
        if peer.get("kicked") == state.attempt and state.attempt:
            if state.fired_at is None:
                state.fired_at = now
            if now - state.fired_at > config.receive_timeout_s:
                state.finished = True
                return output(PassPhase.RECEIVE_TIMEOUT)
            return output(PassPhase.RECEIVING)
        intent = peer.get("intent")
        if not peer["has_ball"] or not isinstance(intent, str) or not intent:
            state.reset_settling()
            return output(PassPhase.WAITING_FOR_BALL)
        if intent != state.attempt:
            state.attempt = intent
            state.reset_settling()
    else:
        state.reset_settling()
        return output(PassPhase.WAITING_FOR_BALL)
    intent = state.attempt if state.role == PassRole.SENDER else None
    candidate = evaluate_pass(
        pose, {"message": peer, "received_at": state.peer_received_at}, evidence, now, config)
    if candidate is None:
        state.reset_settling()
        return output(PassPhase.UNAVAILABLE, intent=intent)
    aim = bearing(pose, target)
    aligned = (abs(angle_error(aim, yaw)) <= config.yaw_tolerance_deg
               and abs(gyro_deg_s) <= config.max_gyro_deg_s)
    if not aligned:
        state.reset_settling()
        return output(PassPhase.FACING_PEER, rotation=aim, intent=intent)
    if state.settled_since is None:
        state.settled_since = now
    frame_id = scene["frame_id"]
    if frame_id != state.last_frame:
        state.last_frame = frame_id
        state.settled_frames += 1
    if now - state.settled_since < config.settle_s or state.settled_frames < 2:
        return output(PassPhase.SETTLING, rotation=aim, intent=intent)
    if state.role == PassRole.RECEIVER:
        return output(PassPhase.READY_TO_RECEIVE, rotation=aim, ready=state.attempt)
    if not _receiver_ready(state, peer, pose, target, now, config):
        return output(PassPhase.WAITING_FOR_RECEIVER, rotation=aim, intent=intent)
    segment = _kick_segment(pose, target, config)
    if segment is None:
        return output(PassPhase.ALIGNING_KICK, rotation=aim, intent=intent)
    if not candidate.lane_clear(*segment, config):
        return output(PassPhase.UNAVAILABLE, rotation=aim, intent=intent)
    state.kicked, state.fired_at = True, now
    return output(PassPhase.PASS_SENT, rotation=aim, kick=True)
