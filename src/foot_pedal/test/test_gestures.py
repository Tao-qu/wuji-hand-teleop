import unittest
from foot_pedal.gestures import Gestures


class GestureTests(unittest.TestCase):
    def test_short_repeat_and_duplicate_press(self):
        g = Gestures()
        self.assertEqual(g.feed(30, 1, 1)[0]['event'], 'press')
        self.assertEqual(g.feed(30, 2, 1.1), [])
        self.assertEqual(g.feed(30, 1, 1.2), [])
        self.assertEqual([e['event'] for e in g.feed(30, 0, 1.3)], ['release', 'short'])
        self.assertEqual(g.feed(30, 0, 1.4), [])

    def test_long_boundary(self):
        g = Gestures()
        g.feed(48, 1, 0)
        events = g.feed(48, 0, 2)
        self.assertEqual([e['event'] for e in events], ['release', 'long'])
        self.assertEqual(events[-1]['pedal'], 'middle')

    def test_independent_pedals(self):
        g = Gestures()
        g.feed(30, 1, 0)
        g.feed(46, 1, 1)
        self.assertEqual(g.feed(46, 0, 1.2)[-1]['event'], 'short')
        self.assertEqual(g.feed(30, 0, 2)[-1]['event'], 'long')

    def test_disconnect_cancels_and_reconnect_held_is_ignored(self):
        g = Gestures()
        g.feed(30, 1, 0)
        g.reset([30])
        self.assertEqual(g.feed(30, 1, 2), [])
        self.assertEqual(g.feed(30, 0, 3), [])
        self.assertEqual(g.feed(30, 1, 4)[0]['event'], 'press')
        g.reset()
        self.assertEqual(g.feed(30, 0, 5), [])

    def test_below_two_seconds_is_short(self):
        for duration in (1.0, 1.999):
            with self.subTest(duration=duration):
                g = Gestures()
                g.feed(48, 1, 0)
                self.assertEqual(g.feed(48, 0, duration)[-1]['event'], 'short')

    def test_unknown_key(self):
        self.assertEqual(Gestures().feed(99, 1, 0), [])


if __name__ == '__main__':
    unittest.main()
