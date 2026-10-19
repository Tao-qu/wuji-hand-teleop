"""ROS transport for the Wuji Hand 2 bridge; commands are firmware-order radians."""
from __future__ import annotations

import threading
import time
from typing import Optional

import numpy as np

from ..retargeting import validate_joint_positions


class WujiHand:
    """Publish targets and read feedback without discovering or enabling devices.

    The bridge publishes states in SDK firmware order (five fingers, four joints
    each). Commands intentionally omit names and use that same documented order.
    Enable/disable belongs to the bridge's explicit SetBool service.
    """

    NUM_JOINTS = 20
    CONNECTION_FRESHNESS_SEC = 0.25

    def __init__(self, hand_name: str, side: str, node, logger=None):
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        from sensor_msgs.msg import JointState
        from std_srvs.srv import SetBool

        self.hand_name = hand_name
        self.side = side
        self.node = node
        self.logger = logger or node.get_logger()
        self._joint_state_type = JointState
        self._set_bool_type = SetBool
        self._latest_positions: Optional[np.ndarray] = None
        self._last_msg_time: Optional[float] = None
        self._lock = threading.Lock()
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self._cmd_pub = node.create_publisher(JointState, f"/{hand_name}/joint_commands", qos)
        self._state_sub = node.create_subscription(
            JointState, f"/{hand_name}/joint_states", self._state_callback, qos)
        self._enable_client = node.create_client(SetBool, f"/{hand_name}/set_enabled")

    def _state_callback(self, msg) -> None:
        try:
            positions = validate_joint_positions(msg.position)
        except (TypeError, ValueError):
            return
        if msg.name and (len(msg.name) != self.NUM_JOINTS or len(set(msg.name)) != self.NUM_JOINTS):
            return
        with self._lock:
            self._latest_positions = positions
            self._last_msg_time = time.monotonic()

    def is_connected(self) -> bool:
        with self._lock:
            last = self._last_msg_time
        return last is not None and time.monotonic() - last < self.CONNECTION_FRESHNESS_SEC

    def set_joint_positions(self, positions) -> bool:
        try:
            positions = validate_joint_positions(positions)
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
            if (self._latest_positions is None or self._last_msg_time is None
                    or time.monotonic() - self._last_msg_time >= self.CONNECTION_FRESHNESS_SEC):
                return None
            return self._latest_positions.copy()

    def disable(self, timeout: float = 0.5) -> bool:
        """Actually request bridge disable; call outside an executor callback.

        This bounded cleanup complements the bridge's own watchdog and shutdown.
        A ROS context already stopped by SIGINT cannot transmit a service call.
        """
        import rclpy

        if not rclpy.ok(context=self.node.context):
            self.logger.warning("ROS context stopped; bridge watchdog/shutdown must disable the hand")
            return False
        try:
            if not self._enable_client.wait_for_service(timeout_sec=timeout):
                self.logger.warning(f"/{self.hand_name}/set_enabled unavailable during shutdown")
                return False
            request = self._set_bool_type.Request()
            request.data = False
            future = self._enable_client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
            if not future.done():
                future.cancel()
                self.logger.warning(f"/{self.hand_name}/set_enabled disable timed out")
                return False
            response = future.result()
            if response is None or not response.success:
                self.logger.warning(f"/{self.hand_name}/set_enabled rejected disable")
                return False
            return True
        except Exception as exc:
            self.logger.warning(f"Could not confirm {self.side} hand disable: {exc}")
            return False

    def release(self) -> None:
        self.disable()
        self.node.destroy_publisher(self._cmd_pub)
        self.node.destroy_subscription(self._state_sub)
        self.node.destroy_client(self._enable_client)
        with self._lock:
            self._last_msg_time = None
            self._latest_positions = None
