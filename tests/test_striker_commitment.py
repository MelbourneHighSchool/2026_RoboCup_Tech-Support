import math

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
    assert first == striker.StrikerState()


def test_open_lane_can_still_kick():
    state = striker.StrikerState()
    step(state, y=910)
    out = step(state, yaw=state.aim, y=910, now=0.1)
    assert out[4]


def test_different_lane_cannot_instantly_reverse_aim():
    state = striker.StrikerState(aim=30)
    assert state.select(-30, 0) == 30
    assert state.select(-30, 0.2) == 30
    assert state.select(-30, 0.4) is None
    assert state.select(-30, 0.5) is None
    assert state.select(-30, 1.0) == -30
