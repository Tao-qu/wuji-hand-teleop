"""Official Wuji Hand 2 retargeting; no device discovery or connection."""
from __future__ import annotations

import numpy as np

from .keypoints import validate_keypoints


def validate_joint_positions(positions) -> np.ndarray:
    positions = np.asarray(positions, dtype=np.float32)
    if positions.shape == (5, 4):
        positions = positions.reshape(20)
    if positions.shape != (20,) or not np.isfinite(positions).all():
        raise ValueError("Joint positions must be finite with shape (20,) or (5, 4)")
    return positions.copy()


class Hand2Retargeter:
    """One persistent SDK session per hand, producing firmware-order radians."""

    def __init__(self, side: str, *, session=None):
        if side not in ("left", "right"):
            raise ValueError("side must be left or right")
        self.side = side
        if session is None:
            from wuji_sdk import HandModel, Handedness, RetargetSession

            handedness = Handedness.Left if side == "left" else Handedness.Right
            session = RetargetSession.for_hand(HandModel.WujiHand2, side=handedness)
        self._session = session

    def retarget(self, points) -> np.ndarray:
        return validate_joint_positions(self._session.step(validate_keypoints(points)))

    def reset(self) -> None:
        self._session.reset()
