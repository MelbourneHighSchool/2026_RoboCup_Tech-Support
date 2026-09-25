"""Track an enemy that may be carrying a temporarily hidden ball."""

from __future__ import annotations

import math
from collections.abc import Sequence

BALL_CARRIER_DISTANCE_MM = 50.0
BOT_RADIUS_MM = 110.0
MAX_BOT_MATCH_SPEED_MM_S = 2000.0


def ball_is_near_bot(
    ball_position: tuple[float, float],
    bot_position: tuple[float, float],
    *,
    distance_mm: float = BALL_CARRIER_DISTANCE_MM,
    bot_radius_mm: float = BOT_RADIUS_MM,
) -> bool:
    """Return whether a ball is within ``distance_mm`` of a bot's body."""
    distance_from_bot = max(
        0.0, math.dist(ball_position, bot_position) - bot_radius_mm
    )
    return distance_from_bot <= distance_mm


class BallExtrapolator:
    """Predict a briefly hidden ball using its last observed velocity."""

    def __init__(self, timeout_s: float) -> None:
        self.timeout_s = timeout_s
        self._last_position: tuple[float, float] | None = None
        self._last_observed_at: float | None = None
        self._velocity = (0.0, 0.0)

    def update(
        self, observed_position: tuple[float, float] | None, now: float
    ) -> tuple[float, float] | None:
        if observed_position is not None:
            if self._last_position is not None and self._last_observed_at is not None:
                elapsed = now - self._last_observed_at
                if elapsed > 0:
                    self._velocity = (
                        (observed_position[0] - self._last_position[0]) / elapsed,
                        (observed_position[1] - self._last_position[1]) / elapsed,
                    )
            self._last_position = observed_position
            self._last_observed_at = now
            return observed_position

        if self._last_position is None or self._last_observed_at is None:
            return None
        elapsed = now - self._last_observed_at
        if elapsed >= self.timeout_s:
            return None
        return (
            self._last_position[0] + self._velocity[0] * elapsed,
            self._last_position[1] + self._velocity[1] * elapsed,
        )

    def timed_out(self, now: float) -> bool:
        return (
            self._last_observed_at is not None
            and now - self._last_observed_at >= self.timeout_s
        )


class BallPossessionTracker:
    """Follow a likely ball carrier between camera detections.

    A visible ball arms the tracker when it is close to an enemy. Once the
    ball disappears, each new camera frame associates that enemy with the
    detection nearest its previous position, subject to a speed limit.
    """

    def __init__(
        self,
        carrier_distance_mm: float = BALL_CARRIER_DISTANCE_MM,
        bot_radius_mm: float = BOT_RADIUS_MM,
        max_match_speed_mm_s: float = MAX_BOT_MATCH_SPEED_MM_S,
    ) -> None:
        self.carrier_distance_mm = carrier_distance_mm
        self.bot_radius_mm = bot_radius_mm
        self.max_match_speed_mm_s = max_match_speed_mm_s
        self._carrier_position: tuple[float, float] | None = None
        self._carrier_update_time: float | None = None

    def clear(self) -> None:
        self._carrier_position = None
        self._carrier_update_time = None

    def update(
        self,
        ball_position: tuple[float, float] | None,
        enemy_positions: Sequence[tuple[float, float]],
        now: float,
        *,
        new_camera_frame: bool,
        camera_healthy: bool = True,
    ) -> tuple[float, float] | None:
        """Return the inferred ball position when a tracked enemy carries it."""
        if not camera_healthy:
            self.clear()
            return None

        # Do not treat a repeated read of one camera result as another
        # observation. Keep returning the last known carrier position meanwhile.
        if not new_camera_frame:
            return self._carrier_position if ball_position is None else None

        if ball_position is not None:
            self.clear()
            if not enemy_positions:
                return None
            nearest = min(
                enemy_positions,
                key=lambda position: math.dist(ball_position, position),
            )
            if ball_is_near_bot(
                ball_position,
                nearest,
                distance_mm=self.carrier_distance_mm,
                bot_radius_mm=self.bot_radius_mm,
            ):
                self._carrier_position = nearest
                self._carrier_update_time = now
            return None

        if self._carrier_position is None or self._carrier_update_time is None:
            return None
        if not enemy_positions:
            self.clear()
            return None

        elapsed = now - self._carrier_update_time
        nearest = min(
            enemy_positions,
            key=lambda position: math.dist(self._carrier_position, position),
        )
        distance = math.dist(self._carrier_position, nearest)
        speed = distance / elapsed if elapsed > 0 else math.inf
        if speed > self.max_match_speed_mm_s or math.isclose(
            speed, self.max_match_speed_mm_s
        ):
            self.clear()
            return None

        self._carrier_position = nearest
        self._carrier_update_time = now
        return nearest
