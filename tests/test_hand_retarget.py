"""Offline MANUS/retarget contract tests. No SdkManager or hardware is used."""
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/output_devices/wujihand_output'))
from wujihand_output.keypoints import (  # noqa: E402
    LatestKeypoints, MEDIAPIPE_TO_MANUS, SourceFrameGate, manus_to_keypoints, validate_keypoints,
)
from wujihand_output.retargeting import Hand2Retargeter, validate_joint_positions  # noqa: E402
from wujihand_output._internal.hand_interface import WujiHand  # noqa: E402


def open_hand():
    points = np.zeros((21, 3), dtype=np.float32)
    for base, x in [(1, -0.04), (5, -0.03), (9, -0.01), (13, 0.01), (17, 0.03)]:
        for k in range(4):
            points[base+k] = [x, .03*(k+1), 0]
    points[1] = [-.03, .02, .01]
    return points


def manus_message():
    nodes = [SimpleNamespace(node_id=node_id, pose=SimpleNamespace(
        position=SimpleNamespace(x=float(p[0]), y=float(p[1]), z=float(p[2]))))
        for node_id, p in zip(MEDIAPIPE_TO_MANUS, open_hand())]
    return SimpleNamespace(raw_nodes=list(reversed(nodes)), side='right')


class ManusValidationTests(unittest.TestCase):
    def test_mapping_uses_ids_and_configurable_reflection(self):
        msg = manus_message()
        np.testing.assert_array_equal(manus_to_keypoints(msg, flip_y=False), open_hand())
        np.testing.assert_array_equal(manus_to_keypoints(msg), open_hand() * [1, -1, 1])

    def test_missing_and_duplicate_ids_are_rejected(self):
        msg = manus_message()
        msg.raw_nodes.pop()
        with self.assertRaisesRegex(ValueError, 'Missing'):
            manus_to_keypoints(msg)
        msg = manus_message()
        msg.raw_nodes.append(msg.raw_nodes[0])
        with self.assertRaisesRegex(ValueError, 'Duplicate'):
            manus_to_keypoints(msg)

    def test_invalid_geometry_and_units(self):
        bad = open_hand()
        bad[4, 0] = np.nan
        for points in (bad, np.zeros((21, 3)), open_hand() * 1000, open_hand()[:20]):
            with self.subTest(shape=points.shape), self.assertRaises(ValueError):
                validate_keypoints(points)
        bad = open_hand()
        bad[9] = bad[5] * 2
        with self.assertRaisesRegex(ValueError, 'degenerate'):
            validate_keypoints(bad)

    def test_rigid_translation_and_valid_rotation(self):
        points = open_hand()
        rotation = np.array([[0., 0., 1.], [1., 0., 0.], [0., 1., 0.]])
        moved = points @ rotation + [1.2, -.3, 2.]
        np.testing.assert_allclose(validate_keypoints(moved), moved, atol=1e-6)

    def test_joint_output_validation(self):
        np.testing.assert_array_equal(validate_joint_positions(np.zeros((5, 4))), np.zeros(20))
        for data in (np.zeros(19), [float('inf')] * 20, [float('nan')] * 20):
            with self.assertRaises(ValueError):
                validate_joint_positions(data)


class FreshnessTests(unittest.TestCase):
    def setUp(self):
        self.now = 0.
        self.stream = LatestKeypoints(.2, clock=lambda: self.now)

    def test_consumes_latest_once_and_resets_once_after_timeout(self):
        self.stream.push(open_hand())
        newer = open_hand() + [.01, 0, 0]
        self.stream.push(newer)
        points, reset = self.stream.poll()
        np.testing.assert_allclose(points, newer, atol=1e-8)
        self.assertFalse(reset)
        self.assertEqual(self.stream.poll(), (None, False))
        self.now = .201
        self.assertEqual(self.stream.poll(), (None, True))
        self.assertEqual(self.stream.poll(), (None, False))
        self.stream.push(open_hand())
        self.assertFalse(self.stream.poll()[1])

    def test_gap_detected_even_when_callback_precedes_timer(self):
        self.stream.push(open_hand())
        self.now = 1.
        self.stream.push(open_hand())
        points, reset = self.stream.poll()
        self.assertIsNotNone(points)
        self.assertTrue(reset)

    def test_invalid_frame_drops_pending_command_and_resets_once(self):
        self.stream.push(open_hand())
        self.stream.invalidate()
        self.assertEqual(self.stream.poll(), (None, True))
        self.stream.invalidate()
        self.assertEqual(self.stream.poll(), (None, False))
        self.now = 1.
        self.stream.push(open_hand())
        self.assertIsNotNone(self.stream.poll()[0])

    def test_rejects_invalid_timeout(self):
        for timeout in (0, -1, float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                LatestKeypoints(timeout)

    def test_source_age_shortens_receipt_deadline(self):
        self.stream.push(open_hand(), source_age=.19)
        self.now = .011
        self.assertEqual(self.stream.poll(), (None, True))


class SourceFrameTests(unittest.TestCase):
    def frame(self, seq, stamp_ns):
        seconds, nanoseconds = divmod(stamp_ns, 1_000_000_000)
        return SimpleNamespace(sequence=seq, header=SimpleNamespace(
            stamp=SimpleNamespace(sec=seconds, nanosec=nanoseconds)))

    def test_frozen_cached_message_never_renews_input(self):
        gate = SourceFrameGate(.2)
        frame = self.frame(1, 1_000_000_000)
        self.assertEqual(gate.accept(frame, 1_000_000_000), 0.)
        for now in (1_010_000_000, 1_100_000_000, 1_500_000_000):
            with self.assertRaisesRegex(ValueError, 'sequence'):
                gate.accept(frame, now)

    def test_rejects_old_future_and_out_of_order_source_data(self):
        gate = SourceFrameGate(.2)
        with self.assertRaisesRegex(ValueError, 'timestamp'):
            gate.accept(self.frame(1, 1_000_000_000), 1_200_000_000)
        with self.assertRaisesRegex(ValueError, 'timestamp'):
            gate.accept(self.frame(1, 2_000_000_000), 1_000_000_000)
        self.assertAlmostEqual(gate.accept(self.frame(2, 1_000_000_000), 1_100_000_000), .1)
        with self.assertRaisesRegex(ValueError, 'sequence'):
            gate.accept(self.frame(1, 1_050_000_000), 1_100_000_000)
        with self.assertRaisesRegex(ValueError, 'timestamp'):
            gate.accept(self.frame(3, 999_000_000), 1_100_000_000)

    def test_rejects_unstamped_messages_and_counter_restart(self):
        gate = SourceFrameGate(.2)
        with self.assertRaises(ValueError):
            gate.accept(self.frame(1, 0), 0)
        gate.accept(self.frame(10, 1_000_000_000), 1_000_000_000)
        with self.assertRaises(ValueError):
            gate.accept(self.frame(1, 2_000_000_000), 2_000_000_000)

    def test_generated_message_stamp_and_sequence_roundtrip(self):
        try:
            from manus_ros2_msgs.msg import ManusGlove
            from rclpy.serialization import deserialize_message, serialize_message
        except ImportError:
            self.skipTest('Build and source manus_ros2_msgs for its serialization check')
        message = ManusGlove()
        message.header.stamp.sec = 1
        message.header.frame_id = 'manus_left'
        message.sequence = 42
        message.side = 'left'
        restored = deserialize_message(serialize_message(message), ManusGlove)
        self.assertEqual(restored.sequence, 42)
        self.assertEqual(restored.header.frame_id, 'manus_left')
        self.assertEqual(SourceFrameGate(.2).accept(restored, 1_000_000_000), 0.)


class RetargetTests(unittest.TestCase):
    def test_session_contract_and_reset(self):
        class Session:
            resets = 0
            def step(self, points):
                self.points = points
                return np.arange(20, dtype=np.float32)
            def reset(self):
                self.resets += 1
        session = Session()
        retargeter = Hand2Retargeter('left', session=session)
        np.testing.assert_array_equal(retargeter.retarget(open_hand()), np.arange(20))
        self.assertEqual(session.points.dtype, np.float32)
        retargeter.reset()
        self.assertEqual(session.resets, 1)

    def test_official_sdk_both_sides_without_hardware(self):
        try:
            import wuji_sdk  # noqa: F401
        except ImportError:
            self.skipTest('Install the pinned wuji-sdk for the official offline integration check')
        root = Path(__file__).resolve().parents[1]
        model = root / 'src/wuji-description/hand2/hand2_2/body/urdf'
        if not (model / 'left-ros.urdf').is_file() or not (model / 'right-ros.urdf').is_file():
            self.skipTest('Fetch the pinned official Hand 2.2 model for the bridge-limit integration check')
        sys.path.insert(0, str(root / 'src/wuji_hand2_driver'))
        from wuji_hand2_driver.joint_mapping import JointLimits
        for side in ('left', 'right'):
            with self.subTest(side=side):
                retargeter = Hand2Retargeter(side)
                limits = JointLimits.from_urdf(model / f'{side}-ros.urdf', side)
                points = open_hand()
                if side == 'left':
                    points[:, 0] *= -1
                for _ in range(3):
                    result = retargeter.retarget(points)
                    self.assertEqual(result.shape, (20,))
                    self.assertTrue(np.isfinite(result).all())
                    limits.validate_command(result)
                retargeter.reset()
                reference = retargeter.retarget(points)
                self.assertTrue(np.isfinite(reference).all())
                limits.validate_command(reference)
                retargeter.reset()
                rotation = np.array([[0., 0., 1.], [1., 0., 0.], [0., 1., 0.]], dtype=np.float32)
                moved = points @ rotation + np.array([.5, .3, -.2], dtype=np.float32)
                # Native optimization may terminate with small numerical differences.
                np.testing.assert_allclose(retargeter.retarget(moved), reference, atol=.02, rtol=0)


class DisableServiceTests(unittest.TestCase):
    def make_transport(self):
        hand = WujiHand.__new__(WujiHand)
        hand.side, hand.hand_name = 'left', 'left_hand'
        hand.node = SimpleNamespace(context=object())
        hand.logger = Mock()
        hand._set_bool_type = SimpleNamespace(Request=lambda: SimpleNamespace(data=True))
        hand._enable_client = Mock()
        hand._enable_client.wait_for_service.return_value = True
        return hand

    def test_disable_calls_real_service_request_with_false(self):
        hand = self.make_transport()
        future = hand._enable_client.call_async.return_value
        future.done.return_value = True
        future.result.return_value = SimpleNamespace(success=True)
        ros = SimpleNamespace(ok=lambda **kwargs: True, spin_until_future_complete=Mock())
        with patch.dict(sys.modules, {'rclpy': ros}):
            self.assertTrue(hand.disable())
        request = hand._enable_client.call_async.call_args.args[0]
        self.assertFalse(request.data)
        ros.spin_until_future_complete.assert_called_once()

    def test_shutdown_does_not_claim_disable_without_response(self):
        hand = self.make_transport()
        future = hand._enable_client.call_async.return_value
        future.done.return_value = False
        ros = SimpleNamespace(ok=lambda **kwargs: True, spin_until_future_complete=Mock())
        with patch.dict(sys.modules, {'rclpy': ros}):
            self.assertFalse(hand.disable(timeout=.01))
        future.cancel.assert_called_once()


if __name__ == '__main__':
    unittest.main()
