"""Map robot-independent human motion onto a robot's configured Home pose."""
from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation


def load_home_reference(config):
    """Load the robot-specific Home reference only when creating a mapping."""
    from tianji_output.kinematics import load_home_reference as load_robot_home_reference

    return load_robot_home_reference(config)


class ArmRetargetMapping:
    """Anchor chest-relative motion at Home in one robot arm's base frame.

    Human chest and the common robot body both use X forward, Y left, Z up.
    ``base_from_body`` maps vectors in those body axes into this arm's base.
    ``neutral_pose`` contains the human palm pose and raw upper-arm Tracker
    orientation captured in human chest coordinates, independently of the robot.
    All positions and the Home TCP transform use metres.
    """

    def __init__(self, base_from_body, neutral_pose, home_palm,
                 home_upper_arm_direction):
        self.base_from_body = np.asarray(base_from_body, dtype=float)
        self.neutral_position = np.asarray(neutral_pose["position_m"], dtype=float)
        self.neutral_rotation = Rotation.from_quat(
            neutral_pose["quaternion_xyzw"]).as_matrix()
        self.neutral_upper_rotation = Rotation.from_quat(
            neutral_pose["upper_arm_quaternion_xyzw"]).as_matrix()
        self.home_palm = np.asarray(home_palm, dtype=float)
        self.home_upper_arm_direction = np.asarray(home_upper_arm_direction, dtype=float)

    def map_palm(self, palm_pose) -> np.ndarray:
        """Return the robot TCP pose from a chest-relative human palm transform."""
        palm = np.asarray(palm_pose, dtype=float)
        base = self.base_from_body
        target = np.eye(4)
        target[:3, 3] = self.home_palm[:3, 3] + base @ (
            palm[:3, 3] - self.neutral_position)
        target[:3, :3] = (base @ palm[:3, :3] @ self.neutral_rotation.T
                          @ base.T @ self.home_palm[:3, :3])
        return target

    def map_upper_arm(self, upper_arm_rotation) -> np.ndarray:
        """Rotate the Home M6 reference by the raw upper Tracker's motion."""
        base = self.base_from_body
        return (base @ np.asarray(upper_arm_rotation, dtype=float)
                @ self.neutral_upper_rotation.T @ base.T
                @ self.home_upper_arm_direction)
