"""Official Wuji Hand 2 retargeting and the ROS bridge command interface."""
from __future__ import annotations

import logging
from typing import Optional, Tuple

import numpy as np

from ._internal.hand_interface import WujiHand
from .retargeting import Hand2Retargeter


class WujiHandController:
    """One controller per side; construction never enables the hardware."""

    NUM_JOINTS = 20

    def __init__(self, side: str, hand_name: str, *, logger=None, node=None):
        if side not in ("left", "right"):
            raise ValueError("side must be left or right")
        if node is None:
            raise ValueError("A ROS2 node is required")
        self.side = side
        self.hand_name = hand_name
        self.logger = logger or logging.getLogger(f"WujiHand2Controller_{side}")
        # Fail startup if the official SDK cannot build the correct model.
        self.retargeter = Hand2Retargeter(side)
        self.hand: Optional[WujiHand] = WujiHand(hand_name, side, node, self.logger)
        self.logger.info(f"{side} Wuji Hand 2 ready; official SDK retargeting, explicit motor enable required")

    def set_joint_positions(self, positions: np.ndarray) -> bool:
        return self.hand is not None and self.hand.set_joint_positions(positions)

    def get_joint_positions(self) -> Optional[np.ndarray]:
        return self.hand.get_joint_positions() if self.hand is not None else None

    def retarget(self, keypoints: np.ndarray) -> Optional[np.ndarray]:
        try:
            return self.retargeter.retarget(keypoints)
        except Exception as exc:
            self.logger.error(f"{self.side} Wuji Hand 2 retarget failed: {exc}")
            self.retargeter.reset()
            return None

    def reset_retargeting(self) -> None:
        self.retargeter.reset()

    def set_keypoints(self, keypoints: np.ndarray) -> Tuple[bool, Optional[np.ndarray]]:
        angles = self.retarget(keypoints)
        if angles is None:
            return False, None
        return self.set_joint_positions(angles), angles

    def is_connected(self) -> bool:
        return self.hand is not None and self.hand.is_connected()

    def disable_and_release(self) -> None:
        if self.hand is not None:
            self.hand.release()
            self.hand = None
