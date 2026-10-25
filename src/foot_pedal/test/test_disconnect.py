"""Regression for Linux's synthetic key-up immediately before USB removal."""
import unittest
from types import SimpleNamespace
from unittest.mock import patch
from foot_pedal.gestures import Gestures
from foot_pedal.node import PedalNode


class DisconnectTests(unittest.TestCase):
    def make_node(self):
        n = SimpleNamespace(simulate=False, gestures=Gestures(), pending=[], published=[])
        n.publish_event = n.published.append
        n.status = lambda message: None
        n.emit = lambda *args: PedalNode.emit(n, *args)
        n.disconnect = lambda reason: PedalNode.disconnect(n, reason)
        return n

    def test_synthetic_release_cancelled(self):
        n = self.make_node()
        n.emit(30, 1, 0)
        n.emit(30, 0, 2)
        self.assertEqual([e['event'] for e in n.published], ['press'])
        self.assertEqual(len(n.pending), 2)
        def removed():
            raise OSError(19, 'No such device')
        n.device = SimpleNamespace(read=removed, close=lambda: None)
        PedalNode.poll(n)
        self.assertEqual(n.pending, [])
        self.assertEqual([e['event'] for e in n.published], ['press'])
        self.assertIsNone(n.device)

    def test_real_release_published_after_guard(self):
        n = self.make_node()
        n.device = SimpleNamespace(read=lambda: [])
        with patch('foot_pedal.node.time.monotonic', return_value=10):
            n.emit(46, 1, 0)
            n.emit(46, 0, 2)
        with patch('foot_pedal.node.time.monotonic', return_value=10.2):
            PedalNode.poll(n)
        self.assertEqual([e['event'] for e in n.published], ['press', 'release', 'long'])
