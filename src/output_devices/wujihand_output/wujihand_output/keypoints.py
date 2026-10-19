"""Validated MANUS input and latest-frame timing, independent of ROS and hardware."""
from __future__ import annotations

import threading
import time
from typing import Optional

import numpy as np


# MediaPipe order: wrist, thumb, index, middle, ring, pinky.
MEDIAPIPE_TO_MANUS = (
    1, 22, 23, 24, 25, 3, 4, 5, 6, 8, 9, 10, 11,
    13, 14, 15, 16, 18, 19, 20, 21,
)


def validate_keypoints(points) -> np.ndarray:
    """Return a private float32 (21, 3) array in metres, or reject the frame."""
    points = np.asarray(points, dtype=np.float32)
    if points.shape != (21, 3) or not np.isfinite(points).all():
        raise ValueError("Hand keypoints must be finite with shape (21, 3)")
    centered = points - points[0]
    if float(np.max(np.linalg.norm(centered, axis=1))) > 0.5:
        raise ValueError("Hand keypoints exceed 0.5 m; check input units")
    index, middle = centered[5], centered[9]
    scale = float(np.linalg.norm(index) * np.linalg.norm(middle))
    if scale < 1e-8 or float(np.linalg.norm(np.cross(index, middle))) <= 1e-3 * scale:
        raise ValueError("Wrist, index MCP and middle MCP form a degenerate palm")
    return points.copy()


def manus_to_keypoints(message, *, flip_y: bool = False) -> np.ndarray:
    """Convert MANUS IDs without applying the old retargeter's wrist transforms.

    The publisher explicitly requests right-handed MANUS coordinates, so the
    default preserves XYZ and chirality. flip_y is an opt-in reflection for a
    separately validated input convention. The SDK receives MediaPipe-order
    points in metres; validate recorded left/right poses before enabling motors.
    """
    positions = {}
    required = set(MEDIAPIPE_TO_MANUS)
    for node in message.raw_nodes:
        if node.node_id not in required:
            continue
        if node.node_id in positions:
            raise ValueError(f"Duplicate MANUS node ID: {node.node_id}")
        p = node.pose.position
        positions[node.node_id] = (p.x, -p.y if flip_y else p.y, p.z)
    missing = required.difference(positions)
    if missing:
        raise ValueError(f"Missing MANUS node IDs: {sorted(missing)}")
    return validate_keypoints([positions[i] for i in MEDIAPIPE_TO_MANUS])


class LatestKeypoints:
    """Consume each valid frame once and signal one reset per tracking gap.

    Receipt deadlines use a monotonic clock. The source age supplied by the
    caller shortens that deadline so queued ROS messages cannot renew old data.
    """

    def __init__(self, timeout: float, clock=time.monotonic):
        if not np.isfinite(timeout) or timeout <= 0:
            raise ValueError("input_timeout must be finite and positive")
        self.timeout = timeout
        self._clock = clock
        self._lock = threading.Lock()
        self._latest: Optional[np.ndarray] = None
        self._received: Optional[float] = None
        self._expires: Optional[float] = None
        self._reset_pending = False
        self._in_gap = False

    def push(self, points, *, source_age: float = 0.0) -> None:
        points = validate_keypoints(points)
        if not np.isfinite(source_age) or source_age < 0 or source_age >= self.timeout:
            raise ValueError("Source frame is already expired")
        now = self._clock()
        with self._lock:
            # A callback may arrive before the timer noticed the preceding gap.
            if self._expires is not None and now >= self._expires:
                if not self._in_gap:
                    self._reset_pending = True
            self._latest = points
            self._received = now
            self._expires = now + self.timeout - source_age
            self._in_gap = False

    def invalidate(self) -> None:
        """Discard a malformed frame and any pending target immediately."""
        with self._lock:
            if self._received is not None and not self._in_gap:
                self._reset_pending = True
            self._in_gap = True
            self._latest = None

    def poll(self) -> tuple[Optional[np.ndarray], bool]:
        now = self._clock()
        with self._lock:
            if self._expires is not None and now >= self._expires:
                self._latest = None
                if not self._in_gap:
                    self._reset_pending = True
                    self._in_gap = True
            points, reset = self._latest, self._reset_pending
            self._latest = None
            self._reset_pending = False
            return points, reset


class SourceFrameGate:
    """Reject duplicate/reordered SDK frames and stale ROS acquisition stamps.

    The publisher and controller share the ROS clock (system time in the real
    rig). Sequence state is deliberately retained over tracking gaps: replaying
    old frames must not become valid after a timeout. Restart the controller
    together with the publisher if its per-glove sequence counter is reset.
    """

    def __init__(self, timeout: float, future_tolerance: float = 0.05):
        if not np.isfinite(timeout) or timeout <= 0:
            raise ValueError("Source timeout must be finite and positive")
        self._timeout_ns = int(timeout * 1_000_000_000)
        self._future_ns = int(future_tolerance * 1_000_000_000)
        self._last_sequence = 0
        self._last_stamp_ns = 0

    def accept(self, message, now_ns: int) -> float:
        sequence = message.sequence
        stamp = message.header.stamp
        if (not isinstance(sequence, int) or isinstance(sequence, bool)
                or sequence <= self._last_sequence):
            raise ValueError("Duplicate or out-of-order MANUS source sequence")
        if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:
            raise ValueError("Invalid MANUS source timestamp")
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        age_ns = now_ns - stamp_ns
        if stamp_ns <= self._last_stamp_ns:
            raise ValueError("Duplicate or out-of-order MANUS source timestamp")
        if age_ns >= self._timeout_ns or age_ns < -self._future_ns:
            raise ValueError("Stale or future MANUS source timestamp")
        self._last_sequence = sequence
        self._last_stamp_ns = stamp_ns
        return max(age_ns / 1_000_000_000, 0.0)
