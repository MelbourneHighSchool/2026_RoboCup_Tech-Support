import math

import pytest

import striker


def step(state, yaw=0, enemies=(), now=0, x=1800, y=1040, captured=True):
    return striker.striker(
        x, y, yaw,
        x + 100 * math.cos(math.radians(yaw)),
        y + 100 * math.sin(math.radians(yaw)),
        ball_captured=captured, enemy_bot_positions=enemies,
        state=state, now=now,
    )


def test_stationary_blocker_does_not_flip_target_as_ball_rotates():
    state = striker.StrikerState()
    outputs = [step(state, yaw, [(2050, 1040)], i * 0.1)
               for i, yaw in enumerate([105, 110, 105, 110, 120, 100])]
    assert len({output[2] for output in outputs}) == 1
    assert all(not output[4] for output in outputs)


def test_reposition_requires_continuously_open_lane():
    state = striker.StrikerState()
    step(state, enemies=[(1900, 1040)])
    assert state.repositioning
    step(state, now=1)
    step(state, enemies=[(1900, 1040)], now=1.2)
    step(state, now=1.3)
    assert state.repositioning
    step(state, now=1.6)
    assert state.repositioning
    step(state, now=1.8)
    assert not state.repositioning
    assert state.aim is not None


def test_blocked_aim_is_held_briefly_but_never_kicks_through_blocker():
    state = striker.StrikerState()
    step(state, x=1800, y=910)
    aim = state.aim
    assert aim is not None
    blocked = [(2000, 910)]
    out = step(state, yaw=aim, enemies=blocked, now=0.1, y=910)
    assert out[2] == aim
    assert not out[4]
    out = step(state, yaw=aim, enemies=blocked, now=0.5, y=910)
    assert state.repositioning
    assert out[1] > 0
    assert not out[4]


def test_capture_loss_resets_commitment_and_robots_do_not_share_state():
    first, second = striker.StrikerState(), striker.StrikerState()
    step(first, enemies=[(1900, 1040)])
    step(second)
    assert first.repositioning
    assert second.aim is not None
    step(first, captured=False)
    # Capture loss resets shot commitment; the close-ball approach starts anew.
    assert first == striker.StrikerState(capturing=True)


def test_open_lane_can_still_kick():
    state = striker.StrikerState()
    step(state, y=910)
    out = step(state, yaw=state.aim, y=910, now=0.1)
    assert out[4]


def test_two_enemies_in_goal_box_do_not_block_direct_shot():
    enemies = [(1950, 910), (2050, 910)]
    aim, possible = striker.goal_shot_aim(
        1800, 910, striker.CYAN_GOAL_BACK_X, striker.CYAN_GOAL_MOUTH_X,
        enemies,
    )
    assert possible
    assert abs(aim) < 2
    assert step(striker.StrikerState(), enemies=enemies, y=910)[4]


def test_goal_box_exception_only_ignores_bots_inside_box():
    in_box = (1950, 910)
    outside_box = (1820, 910)
    assert not step(striker.StrikerState(), enemies=[in_box], y=910)[4]
    assert not step(
        striker.StrikerState(), enemies=[in_box, (2050, 910), outside_box], y=910
    )[4]


def test_different_lane_cannot_instantly_reverse_aim():
    state = striker.StrikerState(aim=30)
    assert state.select(-30, 0) == 30
    assert state.select(-30, 0.2) == 30
    assert state.select(-30, 0.4) is None
    assert state.select(-30, 0.5) is None
    assert state.select(-30, 1.0) == -30


@pytest.mark.parametrize("y, early_yaw", [(600, 25), (1220, -25)])
def test_side_capture_waits_for_selected_aim_even_when_rebound_scores(y, early_yaw):
    state = striker.StrikerState()
    out = step(state, yaw=early_yaw, y=y)
    ball_x = 1800 + 100 * math.cos(math.radians(early_yaw))
    ball_y = y + 100 * math.sin(math.radians(early_yaw))
    # This alternate scoring ray used to trigger a kick during the turn.
    assert striker.kick_direction_scores(
        ball_x, ball_y, early_yaw,
        striker.CYAN_GOAL_BACK_X, striker.CYAN_GOAL_MOUTH_X,
    )
    assert not state.ball_hiding
    assert out[1] == 0
    assert abs(striker.wrap_angle_deg(out[2] - early_yaw)) > 10
    assert not out[4]
    assert out[5] == 1

    aligned = step(state, yaw=state.aim, y=y, now=0.1)
    assert aligned[4]
    assert aligned[5] == -1


@pytest.mark.parametrize("y", [400, 500, 600, 1220, 1320, 1420])
def test_side_rotation_only_kicks_near_aim_and_with_clear_ball_path(y):
    state = striker.StrikerState()
    kicks = []
    for yaw in range(-90, 91):
        out = step(state, yaw=yaw, y=y)
        if out[4]:
            kicks.append(yaw)
            assert abs(striker.wrap_angle_deg(out[2] - yaw)) <= 2
            assert striker.kick_direction_scores(
                1800 + 100 * math.cos(math.radians(yaw)),
                y + 100 * math.sin(math.radians(yaw)),
                yaw, striker.CYAN_GOAL_BACK_X, striker.CYAN_GOAL_MOUTH_X,
            )
    assert kicks


def test_shot_alignment_wraps_yaw_at_360_degrees():
    state = striker.StrikerState()
    step(state, y=1220)
    assert state.aim < 0
    out = step(state, yaw=state.aim + 360, y=1220, now=0.1)
    assert out[4]
