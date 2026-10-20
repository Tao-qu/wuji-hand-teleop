"""Official Wuji Hand 2 retargeting; no device discovery or connection."""
from __future__ import annotations

import numpy as np


class WujiHand2Retargeter:
    """One persistent SDK session per hand, producing firmware-order radians."""

    def __init__(self, side: str, *, session=None):
        self.side = side
        if session is None:
            from wuji_sdk import HandModel, Handedness, RetargetSession

            handedness = Handedness.Left if side == "left" else Handedness.Right
            session = RetargetSession.for_hand(HandModel.WujiHand2, side=handedness)
        self._session = session

    def retarget(self, points) -> np.ndarray:
        return self._session.step(np.asarray(points, dtype=np.float32))

    def reset(self) -> None:
        self._session.reset()
