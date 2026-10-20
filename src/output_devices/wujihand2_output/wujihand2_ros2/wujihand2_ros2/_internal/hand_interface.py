"""ROS transport for the Wuji Hand 2 bridge; commands are firmware-order radians."""
from __future__ import annotations

import threading
import time
from typing import Optional

import numpy as np


class WujiHand2:
    """Publish targets and read feedback through the Hand 2 ROS driver.

    The bridge publishes states in SDK firmware order (five fingers, four joints
    each). Commands intentionally omit names and use that same documented order.
    Enable/disable belongs to the driver's SetBool service.
    """

    NUM_JOINTS = 20
    CONNECTION_FRESHNESS_SEC = 1.0

    def __init__(self, hand_name: str, side: str, node, logger=None):
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        from sensor_msgs.msg import JointState

        self.hand_name = hand_name
        self.side = side
        self.node = node
        self.logger = logger or node.get_logger()
        self._joint_state_type = JointState
        self._latest_positions: Optional[np.ndarray] = None
        self._last_msg_time: Optional[float] = None
        self._lock = threading.Lock()
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self._cmd_pub = node.create_publisher(JointState, f"/{hand_name}/joint_commands", qos)
        self._state_sub = node.create_subscription(
            JointState, f"/{hand_name}/joint_states", self._state_callback, qos)

    def _state_callback(self, msg) -> None:
        if not msg.position or len(msg.position) != self.NUM_JOINTS:
            return
        with self._lock:
            self._latest_positions = np.array(msg.position, dtype=np.float32)
            self._last_msg_time = time.monotonic()

    def is_connected(self) -> bool:
        with self._lock:
            last = self._last_msg_time
        return last is not None and time.monotonic() - last < self.CONNECTION_FRESHNESS_SEC

    def set_joint_positions(self, positions) -> bool:
        try:
            positions = np.asarray(positions, dtype=np.float32)
            if positions.shape == (5, 4):
                positions = positions.flatten()
            elif positions.shape != (self.NUM_JOINTS,):
                self.logger.error(f"Invalid joint-position shape: {positions.shape}")
                return False
            msg = self._joint_state_type()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.position = positions.tolist()
            self._cmd_pub.publish(msg)
            return True
        except Exception as exc:
            self.logger.error(f"Could not publish {self.side} hand target: {exc}")
            return False

    def get_joint_positions(self) -> Optional[np.ndarray]:
        with self._lock:
            if self._latest_positions is None:
                return None
            return self._latest_positions.copy()

    def release(self) -> None:
        self.node.destroy_publisher(self._cmd_pub)
        self.node.destroy_subscription(self._state_sub)
        with self._lock:
            self._last_msg_time = None
            self._latest_positions = None
