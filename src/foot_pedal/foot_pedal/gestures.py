"""Pure gesture state machine; timestamps are seconds from a monotonic source."""

LONG_PRESS_SEC = 2.0

KEYS = {30: 'left', 48: 'middle', 46: 'right'}  # Linux KEY_A/B/C, not X11 codes


class Gestures:
    def __init__(self):
        self.down = {}
        self.blocked = set()

    def reset(self, held=()):
        self.down.clear()
        self.blocked = set(held)

    def feed(self, code, value, timestamp):
        if code not in KEYS or value not in (0, 1):
            return []  # Ignore auto-repeat and unrelated keys.
        if code in self.blocked:
            if value == 0:
                self.blocked.remove(code)
            return []
        base = {'pedal': KEYS[code], 'key': {30: 'A', 48: 'B', 46: 'C'}[code]}
        if value == 1:
            if code in self.down:
                return []
            self.down[code] = timestamp
            return [dict(base, event='press', duration_sec=0.0)]
        if code not in self.down:
            return []
        duration = max(0.0, timestamp - self.down.pop(code))
        return [dict(base, event='release', duration_sec=duration),
                dict(base, event='long' if duration >= LONG_PRESS_SEC else 'short',
                     duration_sec=duration)]
