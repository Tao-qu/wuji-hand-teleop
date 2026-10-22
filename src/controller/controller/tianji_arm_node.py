"""Robot TCP targets to arm joints; hardware execution belongs to tianji_output."""
from __future__ import annotations

import argparse
import sys
import time
from typing import Optional

import numpy as np
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy,
)
from rclpy.utilities import remove_ros_args
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Int8
from std_srvs.srv import SetBool

from tianji_output.kinematics import TianjiKinematics
from tianji_output.configuration import load_arm_config
from .common import ROS2LoggerAdapter, get_default_qos, run_ros_node


LIFECYCLE_READY = 2


class TianjiArmControllerNode(Node):
    """Convert robot-base TCP poses and upper-arm references to joint commands."""

    def __init__(self, *, config=None):
        super().__init__("tianji_arm_controller")
        config = load_arm_config() if config is None else config
        settings = config['controller']
        self.declare_parameter('control_rate', float(settings['control_rate']))
        self.declare_parameter('handoff_hold_sec', float(settings['handoff_hold_sec']))
        self.declare_parameter('handoff_ramp_sec', float(settings['handoff_ramp_sec']))
        self._control_rate_hz = float(self.get_parameter('control_rate').value)
        self._handoff_hold_sec = max(0.0, float(self.get_parameter('handoff_hold_sec').value))
        self._handoff_ramp_sec = max(0.0, float(self.get_parameter('handoff_ramp_sec').value))
        if self._control_rate_hz <= 0.0:
            raise ValueError(f"control_rate must be > 0, got {self._control_rate_hz}")

        self.kinematics = TianjiKinematics(
            logger=ROS2LoggerAdapter(self.get_logger()), config=config)
        self._arm_enabled = False
        self._measured_joints = {'left': None, 'right': None}
        self._handoff_pending = False
        self._handoff_start_at: Optional[float] = None
        self._handoff_start_left: Optional[list] = None
        self._handoff_start_right: Optional[list] = None
        self.left_pose = None
        self.right_pose = None
        self.left_y_axis = None
        self.right_y_axis = None

        qos = get_default_qos()
        self.command_pub = self.create_publisher(
            JointState, '/tianji_arm/joint_commands', qos)
        for side in ('left', 'right'):
            self.create_subscription(
                PoseStamped, f'/{side}_arm/tcp_target',
                lambda message, s=side: self._on_tcp_target(s, message), qos)
            self.create_subscription(
                Vector3Stamped, f'/{side}_arm/upper_arm_reference',
                lambda message, s=side: self._on_upper_arm_reference(s, message), qos)
            self.create_subscription(
                JointState, f'/{side}_arm/joint_states',
                lambda message, s=side: self._on_joint_state(s, message), qos)
        self.left_zsp_para_pub = self.create_publisher(
            Float64MultiArray, '/left_arm/zsp_para', qos)
        self.right_zsp_para_pub = self.create_publisher(
            Float64MultiArray, '/right_arm/zsp_para', qos)
        self.left_ee_pose_pub = self.create_publisher(
            Float64MultiArray, '/left_arm/ee_pose', qos)
        self.right_ee_pose_pub = self.create_publisher(
            Float64MultiArray, '/right_arm/ee_pose', qos)
        lifecycle_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(
            Int8, '/tianji_arm/lifecycle_state', self._on_lifecycle, lifecycle_qos)
        self._enable_client = self.create_client(SetBool, '/tianji_arm/set_enabled')
        self.create_timer(1.0 / self._control_rate_hz, self._control_loop)
        self.get_logger().info(
            f"Arm IK ready: control_rate={self._control_rate_hz:.1f}Hz")

    def _on_tcp_target(self, side: str, message: PoseStamped):
        """Accept a palm TCP pose in {side}_base (metres, quaternion xyzw)."""
        if not self._arm_enabled:
            return
        pose = message.pose
        matrix = np.eye(4, dtype=np.float32)
        matrix[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
        matrix[:3, :3] = R.from_quat([
            pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w,
        ]).as_matrix()
        setattr(self, f'{side}_pose', self._matrix_to_pose(matrix))

    def _on_upper_arm_reference(self, side: str, message: Vector3Stamped):
        """Retain the existing upper-arm Y-axis zero-space convention."""
        if not self._arm_enabled:
            return
        axis = np.array([message.vector.x, message.vector.y, message.vector.z])
        setattr(self, f'{side}_y_axis', axis)
        setattr(self.kinematics, f'{side}_zsp_para', [*axis, 0, 0, 0])

    def _on_joint_state(self, side: str, message: JointState):
        # The ROS interface is radians; the existing IK model takes degrees.
        values = dict(zip(message.name, message.position))
        self._measured_joints[side] = np.rad2deg([
            values[f'{side}_joint_{index + 1}'] for index in range(7)
        ]).tolist()

    def _on_lifecycle(self, message: Int8):
        enabled = message.data == LIFECYCLE_READY
        if enabled and not self._arm_enabled:
            self._handoff_pending = True
        elif not enabled:
            self.left_pose = None
            self.right_pose = None
            self._handoff_start_at = None
            self._handoff_pending = False
        self._arm_enabled = enabled

    def _arm_handoff_snapshot(self):
        """Use the driver's completed-Home feedback for the existing hold/ramp."""
        self._handoff_pending = False
        if self._handoff_hold_sec <= 0.0 and self._handoff_ramp_sec <= 0.0:
            self._handoff_start_at = None
            return
        self._handoff_start_left = list(self._measured_joints['left'])
        self._handoff_start_right = list(self._measured_joints['right'])
        self._handoff_start_at = time.monotonic()
        self.get_logger().info(
            f"Handoff ramp armed: hold={self._handoff_hold_sec:.2f}s, "
            f"ramp={self._handoff_ramp_sec:.2f}s")

    def _control_loop(self):
        if not self._arm_enabled:
            return
        # IK uses measured joints as its reference, supplied by the driver.
        if any(joints is None for joints in self._measured_joints.values()):
            return
        if self._handoff_pending:
            self._arm_handoff_snapshot()
        self._teleop_control()

    def _teleop_control(self):
        if self.left_pose is not None or self.right_pose is not None:
            blend_s = self._handoff_blend_factor()
            try:
                _, _, left, right = self.kinematics.solve(
                    left_pose=self.left_pose, right_pose=self.right_pose,
                    ref_left=self._measured_joints['left'],
                    ref_right=self._measured_joints['right'], unit='m')
                if blend_s is not None:
                    left = self._blend_joints(self._handoff_start_left, left, blend_s)
                    right = self._blend_joints(self._handoff_start_right, right, blend_s)
                self._publish_command(left, right)
            except Exception as exc:
                # Retain the existing disable-on-control-exception behavior.
                self._arm_enabled = False
                self.left_pose = None
                self.right_pose = None
                self.get_logger().error(f"Arm IK failed: {exc}; requesting standby")
                self._enable_client.call_async(SetBool.Request(data=False))
                return
        self._publish_zsp_para_and_pose()

    def _handoff_blend_factor(self) -> Optional[float]:
        """Return the IK-weight for the handoff ramp, or None if the ramp is done.

        0.0 = full hold at snapshot; 1.0 = full live IK target. During the hold
        phase we return 0.0 (hardware tracks the snapshot); during the ramp we
        smoothstep from 0 to 1; after the ramp we clear state and return None
        so the steady-state IK targets pass through unchanged.
        """
        if self._handoff_start_at is None:
            return None
        elapsed = time.monotonic() - self._handoff_start_at
        ramp_end = self._handoff_hold_sec + self._handoff_ramp_sec
        if elapsed >= ramp_end:
            self._handoff_start_at = None
            self._handoff_start_left = None
            self._handoff_start_right = None
            self.get_logger().info("Handoff ramp complete; full-rate teleop engaged")
            return None
        if elapsed < self._handoff_hold_sec or self._handoff_ramp_sec <= 0.0:
            return 0.0
        t = (elapsed - self._handoff_hold_sec) / self._handoff_ramp_sec
        # Quintic smoothstep (same shape as move_to_joints_smooth in the driver).
        return 10.0 * t**3 - 15.0 * t**4 + 6.0 * t**5

    @staticmethod
    def _blend_joints(start: Optional[list], target: Optional[list],
                      s: float) -> Optional[list]:
        """Return start*(1-s) + target*s; preserve target if start is missing,
        and preserve start if target is missing (lets a per-side target dropout
        still hold its snapshot instead of going None)."""
        if target is None and start is None:
            return None
        if start is None:
            return list(target)
        if target is None:
            return list(start)
        return [start[i] + s * (target[i] - start[i]) for i in range(len(start))]

    def _publish_command(self, left: Optional[list], right: Optional[list]):
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        for side, positions in (('left', left), ('right', right)):
            if positions is not None:
                message.name.extend(f'{side}_joint_{index + 1}' for index in range(7))
                message.position.extend(np.deg2rad(positions).tolist())
        if message.name:
            self.command_pub.publish(message)

    def _publish_zsp_para_and_pose(self):
        """Keep the existing target-pose and upper-arm-reference outputs."""
        for side in ('left', 'right'):
            zsp = getattr(self.kinematics, f'{side}_zsp_para', None)
            if zsp is not None and len(zsp):
                getattr(self, f'{side}_zsp_para_pub').publish(
                    Float64MultiArray(data=[float(value) for value in zsp]))
            pose = getattr(self, f'{side}_pose')
            if pose is not None:
                getattr(self, f'{side}_ee_pose_pub').publish(
                    Float64MultiArray(data=[float(value) for value in pose]))

    @staticmethod
    def _matrix_to_pose(matrix: np.ndarray) -> np.ndarray:
        """4x4 matrix -> [x, y, z, RX, RY, RZ] (degrees)."""
        xyz = matrix[:3, 3]
        rpy = R.from_matrix(matrix[:3, :3]).as_euler('ZYX', degrees=True)[::-1]
        return np.array([xyz[0], xyz[1], xyz[2], rpy[0], rpy[1], rpy[2]])

    def shutdown(self):
        """Stop producing targets; the separately managed driver owns shutdown."""
        self._arm_enabled = False


# -------------------- Entry point --------------------

def _parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Tianji arm controller")
    parser.add_argument("-c", "--config", help="Config file path")
    return parser.parse_args(argv)


def main(argv: Optional[list[str]] = None):
    program_name = sys.argv[0] if sys.argv else "tianji_arm_controller"
    raw_argv = sys.argv if argv is None else [program_name, *argv]
    cli_argv = remove_ros_args(raw_argv)[1:]
    args = _parse_args(cli_argv)

    # Load config.
    config = load_arm_config(args.config)

    run_ros_node(
        lambda: TianjiArmControllerNode(config=config),
        args=raw_argv)


if __name__ == "__main__":
    main()
