"""Offline passing handshake and sensor-gate regressions; no motor commands."""

import copy
import math
import threading
import unittest
from dataclasses import replace
from unittest.mock import patch

from lib.bot_fusion import fuse_bots
from lib.communication import Peer
from lib.passing import (
    PROTOCOL,
    PassConfig,
    PassControllerConfig,
    PassPhase,
    PassRole,
    PassState,
    bearing,
    evaluate_pass,
    pass_available,
    step_pass,
)

A = (500, 910, 0)
B = (1500, 910, 180)


def evidence(pose=A, target=B, now=10, frame=1, obstacles=()):
    def detection(position):
        return {"bearing_deg": bearing(pose, position) - pose[2],
                "distance_mm": math.dist(pose[:2], position[:2]), "confidence": 0.9}
    bots = [detection(target)] + [detection(p) for p in obstacles]
    angle = math.radians(bots[0]["bearing_deg"])
    distance = bots[0]["distance_mm"] - 50
    c, s = math.cos(angle), math.sin(angle)
    points = [(distance*c - across*s, distance*s + across*c, now - 0.01, 30, 400)
              for across in (-3, 3)]
    context = {"reason": "ok", "pose": (*pose, 0.9), "points": points}
    scene = {"timestamp_s": now - 0.01, "frame_id": frame, "bots": bots}
    return {"scene": scene, "context": context,
            "results": fuse_bots(bots, context, now - 0.01)}


def packet(bot_id="b", pose=B, captured=False, now=10, sequence=1, **kwargs):
    return {"message": {"bot_id": bot_id, "protocol": PROTOCOL, "session": "test",
                        "sequence": sequence, "pose": pose, "has_ball": captured,
                        "gyro_deg_s": 0, "observation_age_s": 0.01, **kwargs},
            "received_at": now}


class ValidationTests(unittest.TestCase):
    def test_availability_requires_fresh_communication_and_sensor_match(self):
        self.assertTrue(pass_available(A, packet(), evidence(), 10))
        for snapshot in (None, {}, packet(now=9), packet(now=11)):
            available = pass_available(A, snapshot, evidence(), 10)
            self.assertFalse(available)
        self.assertFalse(pass_available(A, packet(pose=(1800, 910, 180)), evidence(), 10))


    def test_valid_camera_and_lidar_peer(self):
        self.assertTrue(pass_available(A, {'message': {'pose': B}, 'received_at': 10}, evidence(), 10))

    def test_lidar_agreement_required_even_if_camera_matches(self):
        for change in ["camera", "bounds", "context"]:
            sample = evidence()
            if change == "camera":
                sample["results"][0]["source"] = "camera"
            elif change == "bounds":
                sample["results"][0]["bounds_mm"] = [800, 990]
            else:
                sample["context"]["reason"] = "missing_motion"
            self.assertFalse(pass_available(A, {'message': {'pose': B}, 'received_at': 10}, sample, 10))

    def test_camera_identity_must_be_unique_and_close(self):
        self.assertFalse(pass_available(A, {'message': {'pose': (1700, 910, 180)}, 'received_at': 10}, evidence(), 10))
        sample = evidence()
        sample["scene"]["bots"] *= 2
        sample["results"] *= 2
        self.assertFalse(pass_available(A, {'message': {'pose': B}, 'received_at': 10}, sample, 10))

    def test_150mm_boundary_and_finite_segment(self):
        config = PassConfig(lane_clearance_mm=150, lane_margin_mm=0)
        for y, allowed in [(1059.9, False), (1060, False), (1060.1, True)]:
            ok = pass_available(A, {'message': {'pose': B}, 'received_at': 10}, evidence(obstacles=[(1000, y)]), 10, config)
            self.assertEqual(ok, allowed)
        # Beyond the endpoint but within 150 mm of it still blocks.
        self.assertFalse(pass_available(A, {'message': {'pose': B}, 'received_at': 10}, evidence(obstacles=[(1630, 910)]), 10, config))
        # A point on the infinite line but far past the receiver does not block.
        self.assertTrue(pass_available(A, {'message': {'pose': B}, 'received_at': 10}, evidence(obstacles=[(2200, 910)]), 10, config))

    def test_default_500mm_clearance_includes_30mm_margin(self):
        self.assertEqual(PassConfig().lane_clearance_mm, 500)
        for y, allowed in [(1440, False), (1440.1, True)]:
            self.assertEqual(pass_available(A, {'message': {'pose': B}, 'received_at': 10}, evidence(obstacles=[(1000, y)]), 10), allowed)

    def test_camera_only_obstacles_still_block_and_unknown_ranges_veto(self):
        sample = evidence(obstacles=[(1000, 910)])
        self.assertEqual(sample["results"][1]["source"], "camera")
        self.assertFalse(pass_available(A, {'message': {'pose': B}, 'received_at': 10}, sample, 10))
        sample["results"][1]["distance_mm"] = None
        self.assertFalse(pass_available(A, {'message': {'pose': B}, 'received_at': 10}, sample, 10))

    def test_stale_future_camera_and_uncertain_bounds(self):
        for now in [9, 10.2]:
            self.assertFalse(pass_available(A, {'message': {'pose': B}, 'received_at': now}, evidence(), now))
        sample = evidence()
        sample["results"][0]["bounds_mm"] = [700, 1200]
        self.assertFalse(pass_available(A, {'message': {'pose': B}, 'received_at': 10}, sample, 10))


    def test_candidate_checks_actual_segment_and_owns_its_geometry(self):
        sample = evidence(obstacles=[(1000, 1510)])
        snapshot = packet(pose=list(B))
        candidate = evaluate_pass(A, snapshot, sample, 10)
        self.assertIsNotNone(candidate)
        self.assertTrue(candidate.lane_clear(A, B, PassConfig()))
        self.assertFalse(candidate.lane_clear((500, 1510), (1500, 1510), PassConfig()))
        snapshot["message"]["pose"][0] = 9999
        self.assertEqual(candidate.target_pose, B)
        self.assertIsNone(evaluate_pass(A, packet(now=10.2), sample, 10.2))



class ControllerTests(unittest.TestCase):
    def step(self, *, now=10, state=None, peer=None, sample=None, captured=True, pose=A, gyro=0,
             bot_id="a", config=None):
        return step_pass(pose, captured, bot_id=bot_id, session="test", now=now,
                       gyro_deg_s=gyro, peer_snapshot=peer,
                       evidence=sample if sample is not None else evidence(pose, B, now),
                       state=state, config=config)

    def test_waits_for_ball_with_dribbler_on_no_translation(self):
        result = self.step(captured=False, peer=packet())
        self.assertEqual(result[:2], (0, 0))
        self.assertFalse(result[4])
        self.assertEqual(result[5], 1)
        self.assertEqual(result[3].phase, PassPhase.WAITING_FOR_BALL)

    def test_both_balls_or_wrong_session_cannot_kick(self):
        for peer in [packet(captured=True), packet(session="different"), packet(pose=None),
                     packet(sequence=-1), packet(protocol="game")]:
            result = self.step(peer=peer)
            self.assertFalse(result[4])
            self.assertIsNone(result[3].attempt)

    def test_faces_peer_only_with_all_sensor_checks(self):
        pose = (500, 910, 90)
        result = self.step(pose=pose, peer=packet(), sample=evidence(pose, B))
        self.assertEqual(result[2], 0)
        self.assertEqual(result[3].phase, PassPhase.FACING_PEER)
        sample = evidence(pose, B)
        sample["results"][0]["source"] = "camera"
        result = self.step(pose=pose, peer=packet(), sample=sample)
        self.assertEqual(result[2], 90)
        self.assertFalse(result[4])

    def exchange(self, obstruct_receiver=False):
        states = [None, None]
        snapshots = [None, None]
        kicks = [0, 0]
        captured = [True, False]
        for tick in range(40):
            now = 10 + tick * 0.02
            outputs = []
            for i, (pose, target) in enumerate([(A, B), (B, A)]):
                sample = evidence(pose, target, now, tick,
                                  obstacles=[(1000, 910)] if i == 1 and obstruct_receiver else [])
                command = self.step(now=now, state=states[i], peer=snapshots[i], sample=sample,
                                    captured=captured[i], pose=pose, bot_id="ab"[i])
                states[i] = command[3]
                self.assertIsInstance(states[i].phase, PassPhase)
                self.assertIsInstance(states[i].role, PassRole)
                self.assertIsInstance(states[i].outgoing["phase"], PassPhase)
                self.assertEqual(command[:2], (0, 0))
                self.assertEqual(command[5], 1)
                if command[4]:
                    kicks[i] += 1
                outputs.append(copy.deepcopy(states[i].outgoing))
            snapshots = [{"message": {**outputs[1-i], "bot_id": "ab"[1-i]}, "received_at": now}
                         for i in range(2)]
            if kicks[0]:
                captured = [False, True]
        return states, kicks

    def test_two_bots_handshake_kick_once_receive_without_passing_back(self):
        states, kicks = self.exchange()
        self.assertEqual(kicks, [1, 0])
        self.assertEqual([s.phase for s in states], [PassPhase.DELIVERED, PassPhase.RECEIVED])
        self.assertTrue(all(s.finished for s in states))

    def test_receiver_must_independently_verify_lane(self):
        states, kicks = self.exchange(obstruct_receiver=True)
        self.assertEqual(kicks, [0, 0])
        self.assertEqual(states[1].phase, PassPhase.UNAVAILABLE)

    def ready_sender(self, *, sample_change=None, packet_change=None, pose=A, gyro=0):
        state = None
        commands = []
        for tick in range(15):
            now = 10 + tick * 0.02
            sample = evidence(pose, B, now, tick)
            if sample_change:
                sample_change(sample, tick)
            message = packet(now=now, sequence=tick, ready_for=state.attempt if state else None,
                             ready_target=A[:2], ready_for_sequence=state.sequence if state else None)
            if packet_change:
                packet_change(message, tick)
            cmd = self.step(now=now, state=state, peer=message, sample=sample, pose=pose, gyro=gyro)
            state = cmd[3]
            commands.append(cmd)
        return commands

    def test_missing_ack_wrong_attempt_stale_evidence_or_peer_yaw_prevent_kick(self):
        changes = [lambda p, _: p["message"].update(ready_for=None),
                   lambda p, _: p["message"].update(ready_for="old-attempt"),
                   lambda p, _: p["message"].update(observation_age_s=1),
                   lambda p, _: p["message"].update(pose=(1500, 910, 90)),
                   lambda p, _: p["message"].update(gyro_deg_s=30),
                   lambda p, _: p["message"].update(ready_for_sequence=1)]
        for change in changes:
            self.assertFalse(any(c[4] for c in self.ready_sender(packet_change=change)))

    def test_loss_of_confirmation_cancels_ready_and_actual_ray_must_hit_receiver(self):
        def lose_camera(sample, tick):
            if tick > 4:
                sample["results"][0]["source"] = "camera"
        self.assertFalse(any(c[4] for c in self.ready_sender(sample_change=lose_camera)))
        # At 1 m, 3 degrees misses the configured 50 mm receiving half-width.
        self.assertFalse(any(c[4] for c in self.ready_sender(pose=(500, 910, 3))))
        self.assertFalse(any(c[4] for c in self.ready_sender(gyro=20)))

    def test_repeated_frame_cannot_establish_settled_state(self):
        commands = self.ready_sender(sample_change=lambda s, _: s["scene"].update(frame_id=0))
        self.assertFalse(any(c[4] for c in commands))

    def test_duplicate_or_out_of_order_packet_does_not_refresh_age(self):
        state = self.step(peer=packet(sequence=5))[3]
        for sequence in [5, 4]:
            result = self.step(now=10.21, state=state, peer=packet(now=10.21, sequence=sequence))
            self.assertEqual(result[3].phase, PassPhase.WAITING_FOR_PEER)
            self.assertFalse(result[4])

    def test_kick_is_not_retried_even_if_breakbeam_stays_blocked(self):
        commands = self.ready_sender()
        self.assertEqual(sum(c[4] for c in commands), 1)
        state = commands[-1][3]
        result = self.step(now=14, state=state, peer=packet(now=14, sequence=99))
        self.assertEqual(result[3].phase, PassPhase.PASS_TIMEOUT)
        self.assertFalse(result[4])

    def test_loss_of_ball_withdraws_intent_and_new_capture_gets_new_attempt(self):
        first = self.step(peer=packet())[3]
        attempt = first.attempt
        second = self.step(state=first, captured=False, peer=packet(sequence=2))[3]
        self.assertIsNone(second.outgoing["intent"])
        third = self.step(state=second, peer=packet(sequence=3))[3]
        self.assertNotEqual(third.attempt, attempt)

    def test_reset_attempt_preserves_ordering_and_rejects_old_readiness(self):
        state = self.ready_sender()[-1][3]
        self.assertTrue(state.kicked)
        previous_attempt = state.attempt
        sequence, peer_sequence = state.sequence, state.peer_sequence
        peer_id = state.peer_id
        state.reset_attempt()
        self.assertFalse(state.kicked)
        self.assertFalse(state.finished)
        self.assertEqual(state.sequence, sequence)
        self.assertEqual(state.peer_sequence, peer_sequence)
        self.assertEqual(state.peer_id, peer_id)
        self.assertEqual(state.sent_at, {})
        self.assertEqual(state.outgoing, {})
        # A duplicate cannot refresh the old packet's local receipt time.
        command = self.step(now=11, state=state, peer=packet(now=11, sequence=peer_sequence))
        self.assertEqual(command[3].phase, PassPhase.WAITING_FOR_PEER)
        self.assertGreater(state.sequence, sequence)
        # Fresh packets still cannot acknowledge the previous attempt.
        kicks = []
        for tick in range(15):
            now = 11.02 + tick * 0.02
            command = self.step(now=now, state=state,
                                sample=evidence(now=now, frame=100 + tick),
                                peer=packet(now=now, sequence=peer_sequence + 1 + tick,
                                            ready_for=previous_attempt,
                                            ready_for_sequence=sequence, ready_target=A[:2]))
            kicks.append(command[4])
        self.assertNotEqual(state.attempt, previous_attempt)
        self.assertEqual(state.phase, PassPhase.WAITING_FOR_RECEIVER)
        self.assertFalse(any(kicks))

    def test_foreign_state_is_replaced_and_configs_validated(self):
        self.assertIsInstance(self.step(state=object())[3], PassState)
        for settings in [{"match_mm": -1}, {"settle_s": 0}, {"lane_margin_mm": float("nan")}]:
            with self.assertRaises(ValueError):
                replace(PassControllerConfig(), **settings)


class PeerSnapshotTests(unittest.TestCase):
    def test_snapshot_is_atomic_owned_and_uses_local_receipt_age(self):
        peer = Peer()
        peer._lock = threading.Lock()
        peer._last_received_at = 10
        peer._last_message = {"pose": [1, 2, 3]}
        with patch("lib.communication.time.monotonic", return_value=10.1):
            snapshot = peer.receive_snapshot()
        snapshot["message"]["pose"][0] = 99
        self.assertEqual(peer._last_message["pose"][0], 1)
        self.assertEqual(snapshot["received_at"], 10)
        with patch("lib.communication.time.monotonic", return_value=11):
            self.assertIsNone(peer.receive_snapshot())
