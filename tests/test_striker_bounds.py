import math

import striker


def test_penalty_box_guard_stops_motion_through_front_line():
    x_pos = (
        striker.OWN_PENALTY_MAX_X
        + striker.ROBOT_RADIUS
        + striker.BOUNDARY_STOP_MARGIN
    )

    _direction, speed = striker.keep_motion_in_legal_area(
        x_pos, striker.GOAL_CENTRE_Y, 180, 600
    )

    assert speed == 0


def test_penalty_box_guard_preserves_motion_along_front_line():
    x_pos = (
        striker.OWN_PENALTY_MAX_X
        + striker.ROBOT_RADIUS
        + striker.BOUNDARY_STOP_MARGIN
    )

    direction, speed = striker.keep_motion_in_legal_area(
        x_pos, striker.GOAL_CENTRE_Y, 135, 600
    )

    assert math.isclose(direction, 90)
    assert math.isclose(speed, 600 / math.sqrt(2))


def test_penalty_box_guard_allows_motion_away_from_box():
    direction, speed = striker.keep_motion_in_legal_area(
        striker.OWN_PENALTY_MAX_X, striker.GOAL_CENTRE_Y, 0, 600
    )

    assert math.isclose(direction, 0)
    assert math.isclose(speed, 600)


def test_striker_does_not_chase_ball_into_own_penalty_box():
    x_pos = (
        striker.OWN_PENALTY_MAX_X
        + striker.ROBOT_RADIUS
        + striker.BOUNDARY_STOP_MARGIN
    )

    _direction, speed, *_ = striker.striker(
        x_pos,
        striker.GOAL_CENTRE_Y,
        0,
        striker.OWN_PENALTY_MAX_X - 100,
        striker.GOAL_CENTRE_Y,
    )

    assert speed == 0


def test_striker_white_line_guard_stops_outward_motion():
    x_pos = striker.WHITE_MAX_X - striker.ROBOT_RADIUS - striker.BOUNDARY_STOP_MARGIN

    _direction, speed = striker.keep_motion_in_legal_area(
        x_pos, striker.GOAL_CENTRE_Y, 0, 600
    )

    assert speed == 0
