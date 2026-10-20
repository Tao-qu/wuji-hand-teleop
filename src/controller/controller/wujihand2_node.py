"""MANUS skeletons to Wuji Hand 2 commands, one official session per side."""
from __future__ import annotations

import argparse
import sys
import threading
from typing import Optional

from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.utilities import remove_ros_args
from std_msgs.msg import Int8, String

from wujihand2_ros2 import WujiHand2Controller
from wujihand2_ros2.keypoints import manus_to_keypoints
from .common import ROS2LoggerAdapter, get_default_qos, run_ros_node


class WujiHand2ControllerNode(Node):
    """Consume the latest MANUS message once per control tick."""

    def __init__(self, side: str, hand_name: str):
        super().__init__(f"wujihand2_controller_{side}")
        self._side = side
        self.declare_parameter("control_rate", 120.0)
        self.declare_parameter("flip_y", False)
        self.declare_parameter("wait_for_arm", False)
        self.declare_parameter("startup_wait_sec", 3.0)
        rate = float(self.get_parameter("control_rate").value)
        if rate <= 0:
            raise ValueError("control_rate must be positive")
        self._flip_y = bool(self.get_parameter("flip_y").value)
        self._following = False
        self._startup_timer = None
        self._startup_wait_sec = float(self.get_parameter("startup_wait_sec").value)
        self._latest_keypoints = None
        self._keypoints_lock = threading.Lock()
        self.controller = WujiHand2Controller(
            side=side, hand_name=hand_name, node=self,
            logger=ROS2LoggerAdapter(self.get_logger()))
        from manus_ros2_msgs.msg import ManusGlove

        for topic in ("/manus_glove_0", "/manus_glove_1"):
            self.create_subscription(ManusGlove, topic, self._manus_callback, get_default_qos())
        self._control_timer = self.create_timer(1.0 / rate, self._teleop_loop)
        startup_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
        if bool(self.get_parameter("wait_for_arm").value):
            self.create_subscription(
                Int8, "/tianji_arm/lifecycle_state", self._arm_ready, startup_qos)
            self.get_logger().info("Waiting for the shared arm Home and preparation sequence")
        else:
            self.create_subscription(
                String, f"/{hand_name}/lifecycle_state", self._hand_ready, startup_qos)
        self.get_logger().info(f"Wuji Hand 2: {side}, {rate:g} Hz, flip_y={self._flip_y}")

    def _arm_ready(self, msg) -> None:
        # READY is published once Home, the shared wait, and mode switch finish.
        if msg.data == 2 and not self._following:
            self._start_following()

    def _hand_ready(self, msg) -> None:
        if msg.data not in ("READY", "ACTIVE") or self._following or self._startup_timer:
            return
        self.get_logger().info(f"Hand ready; following starts in {self._startup_wait_sec:g} seconds")
        if self._startup_wait_sec <= 0:
            self._start_following()
        else:
            self._startup_timer = self.create_timer(self._startup_wait_sec, self._start_following)

    def _start_following(self) -> None:
        if self._startup_timer is not None:
            self._startup_timer.cancel()
            self.destroy_timer(self._startup_timer)
            self._startup_timer = None
        self._following = True
        self.get_logger().info("Hand teleop following started")

    def _manus_callback(self, msg) -> None:
        if msg.side.lower() != self._side:
            return
        keypoints = manus_to_keypoints(msg, flip_y=self._flip_y)
        with self._keypoints_lock:
            self._latest_keypoints = keypoints

    def _teleop_loop(self) -> None:
        if not self._following:
            return
        with self._keypoints_lock:
            keypoints = self._latest_keypoints
            self._latest_keypoints = None
        if keypoints is not None:
            self.controller.set_keypoints(keypoints)

    def shutdown(self) -> None:
        if self._startup_timer is not None:
            self._startup_timer.cancel()
        self._control_timer.cancel()
        self.controller.disable_and_release()


def _parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="MANUS to official Wuji Hand 2 retargeting")
    parser.add_argument("--side", required=True, choices=["left", "right"])
    parser.add_argument("--hand-name", help="Wuji Hand 2 bridge namespace")
    return parser.parse_args(argv)


def main(argv: Optional[list[str]] = None):
    raw_argv = sys.argv if argv is None else ["wujihand2_controller", *argv]
    args = _parse_args(remove_ros_args(raw_argv)[1:])
    run_ros_node(
        lambda: WujiHand2ControllerNode(args.side, args.hand_name or f"{args.side}_hand"),
        args=raw_argv)


if __name__ == "__main__":
    main()
