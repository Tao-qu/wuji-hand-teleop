"""MANUS skeletons to Wuji Hand 2 commands, one official session per side."""
from __future__ import annotations

import argparse
import sys
import threading
from typing import Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

from wujihand_output import WujiHandController
from wujihand_output.keypoints import manus_to_keypoints
from .common import ROS2LoggerAdapter, get_default_qos


class WujiHandControllerNode(Node):
    """Consume the latest MANUS message once per control tick."""

    def __init__(self, side: str, hand_name: str):
        super().__init__(f"wujihand_controller_{side}")
        self._side = side
        self.declare_parameter("control_rate", 120.0)
        self.declare_parameter("flip_y", False)
        rate = float(self.get_parameter("control_rate").value)
        if rate <= 0:
            raise ValueError("control_rate must be positive")
        self._flip_y = bool(self.get_parameter("flip_y").value)
        self._latest_keypoints = None
        self._keypoints_lock = threading.Lock()
        self.controller = WujiHandController(
            side=side, hand_name=hand_name, node=self,
            logger=ROS2LoggerAdapter(self.get_logger()))
        from manus_ros2_msgs.msg import ManusGlove

        for topic in ("/manus_glove_0", "/manus_glove_1"):
            self.create_subscription(ManusGlove, topic, self._manus_callback, get_default_qos())
        self._control_timer = self.create_timer(1.0 / rate, self._teleop_loop)
        self.get_logger().info(f"Wuji Hand 2: {side}, {rate:g} Hz, flip_y={self._flip_y}")

    def _manus_callback(self, msg) -> None:
        if msg.side.lower() != self._side:
            return
        keypoints = manus_to_keypoints(msg, flip_y=self._flip_y)
        with self._keypoints_lock:
            self._latest_keypoints = keypoints

    def _teleop_loop(self) -> None:
        with self._keypoints_lock:
            keypoints = self._latest_keypoints
            self._latest_keypoints = None
        if keypoints is not None:
            self.controller.set_keypoints(keypoints)

    def shutdown(self) -> None:
        self._control_timer.cancel()
        self.controller.disable_and_release()


def _parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="MANUS to official Wuji Hand 2 retargeting")
    parser.add_argument("--side", required=True, choices=["left", "right"])
    parser.add_argument("--hand-name", help="Wuji Hand 2 bridge namespace")
    return parser.parse_args(argv)


def main(argv: Optional[list[str]] = None):
    raw_argv = sys.argv if argv is None else ["wujihand_controller", *argv]
    args = _parse_args(remove_ros_args(raw_argv)[1:])
    rclpy.init(args=raw_argv)
    node = None
    try:
        node = WujiHandControllerNode(args.side, args.hand_name or f"{args.side}_hand")
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            try:
                node.shutdown()
            finally:
                node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
