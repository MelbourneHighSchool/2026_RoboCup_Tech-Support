from dataclasses import asdict

from defence import DefenceState, defence, goalie
from lib.controller_state import decode_state, encode_state
from lib.recording_session import GAME_FIELDS
from lib.session_replay import game_event_tokens
from striker import StrikerState, striker
from tests.bot import bot


def test_striker_returns_hiding_and_shot_memory_in_one_state():
    result = striker(1100, 910, 0, 1200, 910, True)
    state = result[3]
    assert isinstance(state, StrikerState)
    assert state.ball_hiding
    result = striker(1200, 910, 0, 1300, 910, True, state=state)
    assert result[3] is state
    assert state.ball_hiding
    result = striker(1800, 910, 0, 1900, 910, True, state=state)
    assert result[3] is state
    assert not state.ball_hiding
    assert state.aim is not None
    result = striker(1800, 910, 0, None, None, state=state)
    assert result[3] == StrikerState()


def test_controllers_replace_foreign_state_and_do_not_share_defaults():
    striker_state = StrikerState(ball_hiding=True, aim=30)
    defence_state = defence(1500, 910, 0, None, None, state=striker_state)[3]
    assert isinstance(defence_state, DefenceState)
    assert not defence_state.steering
    assert striker_state.ball_hiding
    new_striker_state = striker(1500, 910, 0, None, None, state=defence_state)[3]
    assert isinstance(new_striker_state, StrikerState)
    assert new_striker_state is not striker_state
    assert defence(1500, 910, 0, None, None)[3] is not defence_state
    for controller in (goalie, bot):
        result = controller(1500, 910, 0, None, None, state=striker_state)
        assert len(result) == 6
        assert result[3] is None


def test_state_survives_recording_and_plain_csv_tokenization():
    state = StrikerState(ball_hiding=True, aim=-35, blocked_since=12.5)
    token = encode_state(state)
    assert ',' not in token
    assert decode_state(token) == asdict(state)
    assert GAME_FIELDS[8] == 'state'
    tokens = game_event_tokens({'state': token, 'other_bots': '[[10,20]]'})
    assert decode_state(tokens[7]) == asdict(state)
    assert tokens[13:] == ['10', '20']
    assert decode_state(game_event_tokens({'steering_state': 'True'})[7]) is True
    assert decode_state(encode_state(None)) is None
