import math

import pytest

import striker


def test_penalty_box_guard_stops_motion_through_front_line():
    x_pos = (
        striker.OWN_PENALTY_MAX_X
        + striker.ROBOT_RADIUS
        + striker.BOUNDARY_STOP_MARGIN
    )

    _direction, speed = striker.keep_motion_in_legal_area(
        x_pos, striker.GOAL_CENTRE_Y, 180, 600, [(400, striker.GOAL_CENTRE_Y)]
    )

    assert speed == 0


def test_penalty_box_guard_preserves_motion_along_front_line():
    x_pos = (
        striker.OWN_PENALTY_MAX_X
        + striker.ROBOT_RADIUS
        + striker.BOUNDARY_STOP_MARGIN
    )

    direction, speed = striker.keep_motion_in_legal_area(
        x_pos, striker.GOAL_CENTRE_Y, 135, 600, [(400, striker.GOAL_CENTRE_Y)]
    )

    assert math.isclose(direction, 90)
    assert math.isclose(speed, 600 / math.sqrt(2))


def test_penalty_box_guard_allows_motion_away_from_box():
    direction, speed = striker.keep_motion_in_legal_area(
        striker.OWN_PENALTY_MAX_X, striker.GOAL_CENTRE_Y, 0, 600,
        [(400, striker.GOAL_CENTRE_Y)]
    )

    assert math.isclose(direction, 0)
    assert math.isclose(speed, 600)


def test_striker_does_not_chase_ball_into_own_penalty_box():
    x_pos = (
        striker.OWN_PENALTY_MAX_X
        + striker.ROBOT_RADIUS
        + striker.BOUNDARY_STOP_MARGIN
    )

    direction, speed, *_ = striker.striker(
        x_pos,
        striker.GOAL_CENTRE_Y,
        0,
        striker.OWN_PENALTY_MAX_X - 100,
        striker.GOAL_CENTRE_Y,
        friendly_bot_positions=[(400, striker.GOAL_CENTRE_Y)],
    )

    assert abs(speed * math.cos(math.radians(direction))) < 1e-9


def test_striker_white_line_guard_stops_outward_motion():
    x_pos = striker.WHITE_MAX_X - striker.ROBOT_RADIUS - striker.BOUNDARY_STOP_MARGIN

    _direction, speed = striker.keep_motion_in_legal_area(
        x_pos, striker.GOAL_CENTRE_Y, 0, 600
    )

    assert speed == 0


@pytest.mark.parametrize(
    "friendly_positions",
    [None, [], [(800, striker.GOAL_CENTRE_Y)], [(400, 400)], [(400, 1400)]],
)
def test_striker_can_enter_unoccupied_own_penalty_box(friendly_positions):
    direction, speed, *_ = striker.striker(
        striker.OWN_PENALTY_MAX_X + striker.ROBOT_RADIUS + striker.BOUNDARY_STOP_MARGIN,
        striker.GOAL_CENTRE_Y,
        0,
        striker.OWN_PENALTY_MAX_X - 100,
        striker.GOAL_CENTRE_Y,
        friendly_bot_positions=friendly_positions,
        enemy_bot_positions=[(400, striker.GOAL_CENTRE_Y)],
    )

    assert speed > 0
    assert speed * math.cos(math.radians(direction)) < 0


@pytest.mark.parametrize("friendly_positions", [[], [(400, striker.GOAL_CENTRE_Y)]])
def test_white_line_guard_is_independent_of_penalty_occupancy(friendly_positions):
    _direction, speed = striker.keep_motion_in_legal_area(
        striker.WHITE_MIN_X + striker.ROBOT_RADIUS + striker.BOUNDARY_STOP_MARGIN,
        striker.GOAL_CENTRE_Y,
        180,
        600,
        friendly_positions,
    )

    assert speed == 0

@pytest.mark.parametrize(
    ('x_pos', 'y_pos', 'expected_direction'),
    [
        (striker.WHITE_MIN_X - striker.ROBOT_RADIUS - 20, 910, 0),
        (striker.WHITE_MAX_X + striker.ROBOT_RADIUS + 20, 910, 180),
        (1000, striker.WHITE_MIN_Y - striker.ROBOT_RADIUS - 20, 90),
        (1000, striker.WHITE_MAX_Y + striker.ROBOT_RADIUS + 20, -90),
    ],
)
def test_striker_fully_outside_recovers_before_chasing_ball(
    x_pos, y_pos, expected_direction
):
    direction, speed, rotation, _state, kick, dribbler = striker.striker(
        x_pos, y_pos, 0, x_pos + 1000, y_pos, ball_captured=True
    )

    assert math.isclose(direction, expected_direction)
    assert speed > 0
    assert rotation == 0
    assert kick is False
    assert dribbler == 1


def test_striker_fully_outside_corner_takes_diagonal_shortest_path():
    x_pos = striker.WHITE_MIN_X - striker.ROBOT_RADIUS - 20
    y_pos = striker.WHITE_MIN_Y - striker.ROBOT_RADIUS - 30
    direction, speed, *_ = striker.striker(x_pos, y_pos, 0, None, None)
    target_x = striker.WHITE_MIN_X + striker.ROBOT_RADIUS + striker.BOUNDARY_STOP_MARGIN
    target_y = striker.WHITE_MIN_Y + striker.ROBOT_RADIUS + striker.BOUNDARY_STOP_MARGIN

    assert math.isclose(direction, math.degrees(math.atan2(target_y - y_pos, target_x - x_pos)))
    assert speed > 0
