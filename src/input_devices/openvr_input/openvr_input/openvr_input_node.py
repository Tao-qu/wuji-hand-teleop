"""ROS2 node that streams OpenVR Tracker data and publishes to ROS topics and TF."""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Optional

import numpy as np

try:
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import TransformBroadcaster
    from rclpy.utilities import remove_ros_args
except ImportError as exc:
    raise ImportError("This module requires ROS 2 Python packages.") from exc

from .openvr_tracker_wrapper import OpenVRTrackerWrapper


@dataclass
class OpenVRInputConfig:
    """Configuration for the OpenVR input streaming node."""

    config_path: Optional[str] = None

    # Tracker serial numbers
    tracker_serials: Optional[Dict[str, str]] = None
    # e.g., {'chest': 'LHR-xxx', 'right_wrist': 'LHR-yyy', 'left_wrist': 'LHR-zzz'}

    # Wrist tracker offset in tracker's local frame [x, y, z] (meters)
    # Applied before coordinate correction to map tracker position to actual wrist
    wrist_offset: Optional[list] = None

    # Tracker-local rotations for the calibrated upper-arm frames.
    upper_arm_rotations: Optional[Dict[str, list]] = None

    # Publishing settings
    publish_rate_hz: float = 120.0

    # TF settings
    publish_tf: bool = True
    parent_frame: str = "world"

    @classmethod
    def from_file(cls, path: str | Path | None = None) -> "OpenVRInputConfig":
        """Read input settings and the selected profile from the arm config."""
        from tianji_output.configuration import arm_config_path, load_arm_config
        from .tracker_calibration_io import load_selected_profile

        cfg_path = arm_config_path(path or None)
        arm_config = load_arm_config(cfg_path)
        profile = load_selected_profile(arm_config)
        settings = arm_config["openvr"]
        data = {
            key: settings[key]
            for key in ("tracker_serials", "publish_rate_hz", "publish_tf", "parent_frame")
            if key in settings
        }
        return cls(
            config_path=str(cfg_path),
            wrist_offset=profile["wrist_offset"],
            upper_arm_rotations=profile["upper_arm_rotations"],
            **data,
        )


class OpenVRInputNode(Node):
    """ROS2 node that reads OpenVR Tracker data and publishes TF."""

    def __init__(self, config: OpenVRInputConfig):
        super().__init__("openvr_input")
        self.config = config

        if not self.config.tracker_serials:
            raise ValueError(
                "openvr.tracker_serials must be specified in the arm configuration."
            )

        self.get_logger().info(
            f"[OpenVR Input] Initializing with trackers: {list(self.config.tracker_serials.keys())}"
        )
        if self.config.wrist_offset:
            self.get_logger().info(
                f"[OpenVR Input] Wrist offset (local frame): {self.config.wrist_offset}"
            )

        # Initialize OpenVR tracker wrapper
        self.tracker_wrapper = OpenVRTrackerWrapper(
            self.config.tracker_serials,
            wrist_offset=self.config.wrist_offset,
            upper_arm_rotations=self.config.upper_arm_rotations,
        )

        # Create TF broadcaster
        if self.config.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)
        else:
            self.tf_broadcaster = None

        # Create timer for periodic publishing
        self.timer = self.create_timer(
            1.0 / max(self.config.publish_rate_hz, 1.0),
            self._publish_callback
        )

        self.get_logger().info(
            f"[OpenVR Input] Publishing TF at {self.config.publish_rate_hz:.1f} Hz"
        )
        self.get_logger().info("[OpenVR Input] Publishing TF: chest, right_wrist, left_wrist, left_arm, right_arm")

    def _publish_callback(self) -> None:
        """Timer callback to fetch and publish tracker data as TF."""
        if self.tf_broadcaster is None:
            return

        try:
            poses = self.tracker_wrapper.get_poses()
        except RuntimeError as e:
            self.get_logger().error(f"Failed to get tracker poses: {e}")
            return

        current_time = self.get_clock().now().to_msg()

        # Publish chest reference pose
        chest_pose = poses.get('chest')
        if chest_pose is not None:
            # Publish chest TF
            self._broadcast_tf(
                chest_pose,
                self.config.parent_frame,
                "chest",
                current_time
            )

        # Process right wrist
        right_wrist_pose = poses.get('right_wrist')
        if right_wrist_pose is not None:
            self._broadcast_tf(
                right_wrist_pose,
                self.config.parent_frame,
                "right_wrist",
                current_time
            )

        # Process left wrist
        left_wrist_pose = poses.get('left_wrist')
        if left_wrist_pose is not None:
            self._broadcast_tf(
                left_wrist_pose,
                self.config.parent_frame,
                "left_wrist",
                current_time
            )

        # Process left arm tracker (upper arm)
        left_arm_pose = poses.get('left_arm')
        if left_arm_pose is not None:
            self._broadcast_tf(
                left_arm_pose,
                self.config.parent_frame,
                "left_arm",
                current_time
            )

        # Process right arm tracker (upper arm)
        right_arm_pose = poses.get('right_arm')
        if right_arm_pose is not None:
            self._broadcast_tf(
                right_arm_pose,
                self.config.parent_frame,
                "right_arm",
                current_time
            )

    def _broadcast_tf(
        self,
        matrix: np.ndarray,
        parent_frame: str,
        child_frame: str,
        stamp
    ) -> None:
        """Broadcast a 4x4 matrix as a TF transform."""
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame

        # Extract translation
        t.transform.translation.x = float(matrix[0, 3])
        t.transform.translation.y = float(matrix[1, 3])
        t.transform.translation.z = float(matrix[2, 3])

        # Extract rotation and convert to quaternion
        rotation_matrix = matrix[:3, :3]
        quat = self._rotation_matrix_to_quaternion(rotation_matrix)
        t.transform.rotation.x = float(quat[0])
        t.transform.rotation.y = float(quat[1])
        t.transform.rotation.z = float(quat[2])
        t.transform.rotation.w = float(quat[3])

        self.tf_broadcaster.sendTransform(t)

    @staticmethod
    def _rotation_matrix_to_quaternion(R: np.ndarray) -> np.ndarray:
        """Convert 3x3 rotation matrix to quaternion [x, y, z, w]."""
        trace = np.trace(R)

        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s

        return np.array([x, y, z, w])

    def shutdown(self) -> None:
        """Shutdown the node and cleanup resources."""
        if hasattr(self, 'tracker_wrapper') and self.tracker_wrapper:
            try:
                self.tracker_wrapper.shutdown()
            except Exception as exc:
                self.get_logger().error(f"Error while closing tracker wrapper: {exc}")


def _parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Publish OpenVR Tracker data to ROS2 topics and TF."
    )
    parser.add_argument(
        "-c",
        "--config",
        default=None,
        help="Path to the arm YAML configuration (default: tianji_chest.yaml).",
    )
    return parser.parse_args(argv)


def main(argv: Optional[list[str]] = None) -> None:
    program_name = sys.argv[0] if sys.argv else "openvr_input"
    raw_argv = sys.argv if argv is None else [program_name, *argv]
    cli_argv = remove_ros_args(raw_argv)[1:]
    args = _parse_args(cli_argv)

    config = OpenVRInputConfig.from_file(args.config)

    rclpy.init(args=raw_argv)
    node = OpenVRInputNode(config)

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main(sys.argv[1:])
