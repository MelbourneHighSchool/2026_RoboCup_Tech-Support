import math

import striker


def goal_aim_selects_an_open_direct_shot():
    ball_x, ball_y = 1700, 910
    target_x = striker.CYAN_GOAL_BACK_X
    mouth_x = striker.CYAN_GOAL_MOUTH_X

    centre_aim, possible = striker.goal_shot_aim(ball_x, ball_y, target_x, mouth_x)
    assert possible

    enemy_bots = [(2150, 910)]
    assert not striker.kick_direction_scores(
        ball_x, ball_y, centre_aim, target_x, mouth_x, enemy_bots
    )

    open_aim, possible = striker.goal_shot_aim(
        ball_x, ball_y, target_x, mouth_x, enemy_bots
    )
    assert possible
    assert not math.isclose(open_aim, centre_aim)
    assert striker.kick_direction_scores(
        ball_x, ball_y, open_aim, target_x, mouth_x, enemy_bots
    )


def enemy_blocks_the_second_leg_of_a_rebound():
    ball_x, ball_y = 1800, 500
    target_x = striker.CYAN_GOAL_BACK_X
    mouth_x = striker.CYAN_GOAL_MOUTH_X
    angle = 57.278337236630875
    sector = striker._mouth_sector(ball_x, ball_y, mouth_x)
    path = striker._scoring_path(ball_x, ball_y, angle, target_x, mouth_x, sector)

    assert path is not None
    assert len(path) == 2
    second_start, second_end = path[1]
    enemy_bots = [
        (
            (second_start[0] + second_end[0]) / 2,
            (second_start[1] + second_end[1]) / 2,
        )
    ]

    assert not striker.kick_direction_scores(
        ball_x, ball_y, angle, target_x, mouth_x, enemy_bots
    )


def midfield_capture_hides_instead_of_reversing_yaw():
    # Centre-spot is inside the old START/END dead zone; still hide rather than
    # facing 0 one frame and 120/240 the next as shot_possible flickers.
    direction, speed, rotation, hiding, kick, _dribbler = striker.striker(
        1100,
        910,
        0,
        1215,
        910,
        ball_captured=True,
        enemy_bot_positions=[(2030, 910)],
    )
    assert hiding.ball_hiding
    assert not kick
    assert speed > 0
    assert rotation in (120, 240)
    assert direction in (0, 120, 240)


def close_blocked_shot_keeps_facing_enemy_goal():
    direction, _speed, rotation, hiding, kick, _dribbler = striker.striker(
        1550,
        950,
        0,
        1650,
        950,
        ball_captured=True,
        enemy_bot_positions=[(1900, 910)],
    )
    assert not hiding.ball_hiding
    assert not kick
    assert rotation == 0
    assert striker.wrap_angle_deg(direction) < 0  # pull toward own goal / centre Y


def hiding_holds_until_a_shot_is_open():
    _d, _s, rotation, hiding, _k, _dr = striker.striker(
        1100,
        250,
        240,
        1215,
        250,
        ball_captured=True,
        state=striker.HIDE_LOW,
        enemy_bot_positions=[(2100, 910)],
    )
    assert hiding == striker.HIDE_LOW
    assert rotation == 240


def blocked_wall_shot_rotates_180_then_crosses():
    enemy = [(2100, 910)]
    direction, speed, rotation, state, kick, _dribbler = striker.striker(
        1500,
        250,
        240,
        1600,
        250,
        ball_captured=True,
        state=striker.HIDE_LOW,
        enemy_bot_positions=enemy,
    )
    assert not kick
    assert state == striker.CROSS_TO_HIGH
    assert rotation == 180
    assert speed == 0

    direction, speed, rotation, state, kick, _dribbler = striker.striker(
        1500,
        250,
        180,
        1600,
        250,
        ball_captured=True,
        state=state,
        enemy_bot_positions=enemy,
    )
    assert state == striker.CROSS_TO_HIGH
    assert rotation == 180
    assert speed > 0
    assert direction == 90


def after_cross_without_shot_cuts_inward():
    enemy = [(2100, 910)]
    # Still facing 180 from the cross: stop and turn to the enemy goal first.
    direction, speed, rotation, state, kick, _dribbler = striker.striker(
        1500,
        1570,
        180,
        1600,
        1570,
        ball_captured=True,
        state=striker.CUT_IN,
        enemy_bot_positions=enemy,
    )
    assert not kick
    assert state == striker.CUT_IN
    assert rotation == 0
    assert speed == 0

    direction, speed, rotation, state, kick, _dribbler = striker.striker(
        1500,
        1570,
        0,
        1600,
        1570,
        ball_captured=True,
        state=striker.CUT_IN,
        enemy_bot_positions=enemy,
    )
    assert not kick
    assert state == striker.CUT_IN
    assert rotation == 0
    assert speed > 0
    assert abs(striker.wrap_angle_deg(direction)) < 90  # toward enemy goal, not own
    assert striker.wrap_angle_deg(direction) < 0  # inward toward mid Y

    # A flickering shot must not leave CUT_IN and start another top-to-bottom cross.
    _d, _s, rotation, state, _k, _dr = striker.striker(
        1500,
        1570,
        0,
        1600,
        1570,
        ball_captured=True,
        state=state,
        enemy_bot_positions=[],
    )
    assert state == striker.CUT_IN
    _d, _s, _r, state, _k, _dr = striker.striker(
        1500,
        1570,
        0,
        1600,
        1570,
        ball_captured=True,
        state=state,
        enemy_bot_positions=enemy,
    )
    assert state == striker.CUT_IN
    assert state not in (striker.CROSS_TO_HIGH, striker.CROSS_TO_LOW)
