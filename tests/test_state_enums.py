"""Typed state transitions with stable JSON, peer and replay boundaries."""

import json
import unittest
from dataclasses import asdict
from types import SimpleNamespace

from lib.bot_fusion import BotRangeFusion, evaluate_bot_scene, fuse_bots
from lib.config import BotMode as ConfigBotMode
from lib.controller_state import decode_state, encode_state
from lib.game_status import GameStatus, ProgressHealth
from lib.passing import PassPhase, PassRole, PassState, pass_available, step_pass
from state import (
    BotMode,
    FusionMode,
    FusionSource,
    GameState,
    HealthState,
    StartupStage,
)
from tests.test_passing import A, evidence, packet


class StateEnumTests(unittest.TestCase):
    def test_old_bot_mode_import_is_the_same_enum(self):
        self.assertIs(ConfigBotMode, BotMode)
        self.assertEqual([mode.value for mode in BotMode], [1, 2, 3])

    def test_pass_defaults_and_legacy_construction_are_typed(self):
        state = PassState()
        self.assertIs(state.role, PassRole.UNASSIGNED)
        self.assertIs(state.phase, PassPhase.WAITING_FOR_BALL)
        previous = PassState(role="sender", phase="settling")
        self.assertIs(previous.role, PassRole.SENDER)
        self.assertIs(previous.phase, PassPhase.SETTLING)
        self.assertIs(PassState(role=None).role, PassRole.UNASSIGNED)
        with self.assertRaises(ValueError):
            PassState(phase="typo")

    def test_wire_and_controller_logs_keep_existing_values(self):
        original = PassState(role=PassRole.RECEIVER, phase=PassPhase.READY_TO_RECEIVE)
        decoded = json.loads(json.dumps(asdict(original)))
        self.assertEqual(decoded["role"], "receiver")
        self.assertEqual(decoded["phase"], "ready_to_receive")
        self.assertEqual(decode_state(encode_state(original)), decoded)
        self.assertIs(PassState(**decoded).phase, PassPhase.READY_TO_RECEIVE)
        self.assertEqual(decode_state(encode_state({"ball_hiding": True})), {"ball_hiding": True})

    def test_unavailable_lidar_returns_false_without_controller_state(self):
        for reason in ("missing_motion", "no_scan", "uncertain_pose", "new_native_failure"):
            sample = evidence()
            sample["context"]["reason"] = reason
            self.assertIs(pass_available(A, packet(), sample, 10), False)
        self.assertIs(pass_available(A, packet(), evidence(), 10), True)

    def test_peer_phases_are_decoded_and_unknown_values_fail_closed(self):
        for phase, expected in [("received", PassPhase.RECEIVED),
                                ("future_phase", PassPhase.UNKNOWN), (None, PassPhase.UNKNOWN)]:
            result = step_pass(A, True, bot_id="a", session="test", now=10,
                             gyro_deg_s=0, peer_snapshot=packet(phase=phase), evidence=evidence())
            state = result[3]
            self.assertIs(state.peer_message["phase"], expected)
            self.assertIsInstance(state.phase, PassPhase)
            self.assertIsInstance(state.outgoing["phase"], PassPhase)
            self.assertFalse(result[4])
            self.assertEqual(json.loads(json.dumps(state.outgoing))["phase"], state.phase.value)

    def test_fusion_modes_and_sources_are_enums_but_reasons_are_strings(self):
        fusion = BotRangeFusion("diagnostic")
        self.assertIs(fusion.mode, FusionMode.DIAGNOSTIC)
        with self.assertRaises(ValueError):
            BotRangeFusion("typo")
        sample = evidence()
        result = sample["results"][0]
        self.assertIs(result["source"], FusionSource.FUSED)
        self.assertIs(type(result["reason"]), str)
        fallback = fuse_bots(sample["scene"]["bots"], {"reason": "missing_motion"}, 10)[0]
        self.assertIs(fallback["source"], FusionSource.CAMERA)
        self.assertEqual(fallback["reason"], "missing_motion")
        unknown = fuse_bots(sample["scene"]["bots"], {"reason": "future_native_reason"}, 10)[0]
        self.assertEqual(unknown["reason"], "future_native_reason")
        evaluated = evaluate_bot_scene(sample["scene"], SimpleNamespace(), 10)
        self.assertEqual(evaluated["context"]["reason"], "native_api_unavailable")
        self.assertEqual(json.loads(json.dumps(result))["source"], "fused")

    def test_game_status_converts_legacy_values_at_boundary(self):
        status = GameStatus(None, None, "STRIKER")
        self.assertIs(status.mode, BotMode.STRIKER)
        self.assertIs(status.stage, GameState.STARTING)
        status.update(BotMode.DEFENCE, True, "BLOCKED")
        self.assertIs(status.stage, GameState.BLOCKED)
        self.assertIs(status.mode, BotMode.DEFENCE)
        status.update("GOALIE", False)
        self.assertIs(status.stage, GameState.AUTO)
        with self.assertRaises(ValueError):
            status.update("GOALIE", False, "misspelled_state")

    def test_health_and_startup_states_keep_display_labels(self):
        tracker = ProgressHealth(10)
        self.assertIs(tracker.update(0, 10), HealthState.UNKNOWN)
        self.assertIs(tracker.update(1, 10.1), HealthState.READY)
        self.assertIs(tracker.update(1, 11.2), HealthState.FAILED)
        self.assertEqual(str(StartupStage.HARDWARE), "HARDWARE")
        self.assertEqual(json.dumps(HealthState.FAILED), '\"!\"')
