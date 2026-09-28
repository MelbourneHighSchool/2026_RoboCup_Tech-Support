"""Typed state transitions with stable JSON, peer and replay boundaries."""

import json
import unittest

from lib.bot_fusion import BotRangeFusion, fuse_bots
from lib.config import BotMode as ConfigBotMode
from lib.game_status import GameStatus, ProgressHealth
from state import (
    BotMode,
    FusionMode,
    FusionSource,
    GameState,
    HealthState,
    StartupStage,
)
from tests.test_bot_fusion import bot, context, support


class StateEnumTests(unittest.TestCase):
    def test_old_bot_mode_import_is_the_same_enum(self):
        self.assertIs(ConfigBotMode, BotMode)
        self.assertEqual([mode.value for mode in BotMode], [1, 2, 3])


    def test_fusion_modes_and_sources_are_enums_but_reasons_are_strings(self):
        fusion = BotRangeFusion("diagnostic")
        self.assertIs(fusion.mode, FusionMode.DIAGNOSTIC)
        with self.assertRaises(ValueError):
            BotRangeFusion("typo")
        bots = [bot()]
        result = fuse_bots(bots, context(support()), 10)[0]
        self.assertIs(result["source"], FusionSource.FUSED)
        self.assertIs(type(result["reason"]), str)
        fallback = fuse_bots(bots, {"reason": "missing_motion"}, 10)[0]
        self.assertIs(fallback["source"], FusionSource.CAMERA)
        self.assertEqual(fallback["reason"], "missing_motion")
        unknown = fuse_bots(bots, {"reason": "future_native_reason"}, 10)[0]
        self.assertEqual(unknown["reason"], "future_native_reason")
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
