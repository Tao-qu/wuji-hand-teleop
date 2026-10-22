"""Map calibrated human Tracker TF to the robot arm target interface."""
from __future__ import annotations

import argparse
import sys
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from scipy.spatial.transform import Rotation as R
from tf2_ros import Buffer, TransformListener
import tf2_ros

from tianji_output.configuration import load_arm_config
from .common import get_default_qos, run_ros_node


class TrackerArmRetargetNode(Node):
    """Express calibrated chest-relative palm poses as robot-base TCP targets.

    The existing retargeting convention maps each calibrated human chest frame
    directly onto that arm's base coordinates, without scaling or an additional
    transform. The robot TCP remains the palm, and the upper-arm Y axis remains
    the existing zero-space reference.
    """

    def __init__(self, *, config=None):
        super().__init__('tracker_arm_retarget')
        config = load_arm_config() if config is None else config
        self.declare_parameter('control_rate', float(config['controller']['control_rate']))
        self._control_rate_hz = float(self.get_parameter('control_rate').value)
        if self._control_rate_hz <= 0.0:
            raise ValueError(f"control_rate must be > 0, got {self._control_rate_hz}")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        qos = get_default_qos()
        self._target_publishers = {}
        self._reference_publishers = {}
        for side in ('left', 'right'):
            self._target_publishers[side] = self.create_publisher(
                PoseStamped, f'/{side}_arm/tcp_target', qos)
            self._reference_publishers[side] = self.create_publisher(
                Vector3Stamped, f'/{side}_arm/upper_arm_reference', qos)
        self._timer = self.create_timer(
            1.0 / self._control_rate_hz, self._publish_targets)
        self.get_logger().info(
            f"Tracker arm retargeting ready: control_rate={self._control_rate_hz:.1f}Hz")

    def _publish_targets(self):
        for side in ('left', 'right'):
            palm = self._lookup_transform(f'{side}_chest', f'{side}_human_palm')
            if palm is not None:
                target = PoseStamped()
                target.header.stamp = palm.header.stamp
                target.header.frame_id = f'{side}_base'
                transform = palm.transform
                target.pose.position.x = transform.translation.x
                target.pose.position.y = transform.translation.y
                target.pose.position.z = transform.translation.z
                target.pose.orientation = transform.rotation
                self._target_publishers[side].publish(target)

            upper_arm = self._lookup_transform(f'{side}_chest', f'{side}_arm')
            if upper_arm is not None:
                axis = self._transform_to_matrix(upper_arm)[:3, 1]
                reference = Vector3Stamped()
                reference.header.stamp = upper_arm.header.stamp
                reference.header.frame_id = f'{side}_base'
                reference.vector.x = float(axis[0])
                reference.vector.y = float(axis[1])
                reference.vector.z = float(axis[2])
                self._reference_publishers[side].publish(reference)

    def _lookup_transform(self, from_frame: str, to_frame: str):
        try:
            return self.tf_buffer.lookup_transform(
                from_frame, to_frame, rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None

    @staticmethod
    def _transform_to_matrix(transform) -> np.ndarray:
        t = transform.transform
        quat = [t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w]
        matrix = np.eye(4, dtype=np.float32)
        matrix[:3, :3] = R.from_quat(quat).as_matrix()
        matrix[:3, 3] = [t.translation.x, t.translation.y, t.translation.z]
        return matrix

    def shutdown(self):
        """Stop streaming targets; the driver owns hardware shutdown."""
        self._timer.cancel()


def _parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='Tracker arm retargeting')
    parser.add_argument('-c', '--config', help='Config file path')
    return parser.parse_args(argv)


def main(argv: Optional[list[str]] = None):
    program_name = sys.argv[0] if sys.argv else 'tracker_arm_retarget'
    raw_argv = sys.argv if argv is None else [program_name, *argv]
    cli_argv = remove_ros_args(raw_argv)[1:]
    args = _parse_args(cli_argv)
    config = load_arm_config(args.config)
    run_ros_node(lambda: TrackerArmRetargetNode(config=config), args=raw_argv)


if __name__ == '__main__':
    main()
