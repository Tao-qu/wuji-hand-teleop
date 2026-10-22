"""Local Tianji FK/IK with no robot connection.

The native model uses millimetres and degrees. ROS callers convert measured
joint angles at this boundary; tool geometry and solver parameters are shared
with Tracker calibration and the robot configuration.
"""
import logging
from pathlib import Path
import time

from ._internal.fx_kine import Marvin_Kine
from .configuration import kinematics_path, load_arm_config
from .tool_parameters import tool_transform


class TianjiKinematics:
    """Configured dual-arm kinematics, independent of hardware transport."""

    def __init__(self, *, config=None, config_path=None, logger=None):
        self.logger = logger or logging.getLogger(__name__)
        cfg = load_arm_config() if config is None else config
        self._tool_config = cfg['driver']
        model_path = (Path(config_path).expanduser().resolve()
                      if config_path is not None else kinematics_path(cfg))
        config_path = str(model_path)
        if not model_path.exists():
            raise FileNotFoundError(f"Config file not found: {config_path}")

        self.logger.debug(f"Loading config: {config_path}")

        # ---------------------- Kinematics init ----------------------
        self.logger.debug("[arm A] Initializing kinematics SDK...")
        self.kine_left = Marvin_Kine()
        config_result = self.kine_left.load_config(config_path=config_path)
        time.sleep(0.3)
        self.kine_left.initial_kine(
            robot_serial=0,
            robot_type=config_result['TYPE'][0],
            dh=config_result['DH'][0],
            pnva=config_result['PNVA'][0],
            j67=config_result['BD'][0]
        )
        self.kine_left.set_tool_kine(0, tool_transform(self._tool_config, 'left'))

        self.logger.debug("[arm B] Initializing kinematics SDK...")
        self.kine_right = Marvin_Kine()
        config_result = self.kine_right.load_config(config_path=config_path)
        time.sleep(0.3)
        self.kine_right.initial_kine(
            robot_serial=1,
            robot_type=config_result['TYPE'][1],
            dh=config_result['DH'][1],
            pnva=config_result['PNVA'][1],
            j67=config_result['BD'][1]
        )
        self.kine_right.set_tool_kine(1, tool_transform(self._tool_config, 'right'))

        self.zsp_type = 1
        self.left_zsp_para = [0, -1, -0.5, 0, 0, 0]
        self.right_zsp_para = [0, 1, -0.5, 0, 0, 0]
        self.zsp_angle = 0.0
        self.dgr = [5.0, 5.0, 5.0]

    def forward(self, side, joints):
        """Return the configured TCP transform in mm from joint angles in degrees."""
        if side not in ('left', 'right'):
            raise ValueError("side must be left or right")
        solver = self.kine_left if side == 'left' else self.kine_right
        return solver.fk(0 if side == 'left' else 1, list(joints))

    def solve(self, left_pose=None, right_pose=None, *, ref_left=None, ref_right=None, unit='mm'):
        """Solve targets using caller-supplied measured seeds, all angles in degrees.

        Translation is in ``unit`` (mm or m). Failed or absent arms return
        ``None`` for their joints, matching the native solver result.
        """
        if left_pose is not None and ref_left is None:
            raise ValueError("Left target requires measured left joint angles")
        if right_pose is not None and ref_right is None:
            raise ValueError("Right target requires measured right joint angles")
        left_mm = None
        right_mm = None
        if left_pose is not None:
            left_mm = list(left_pose)
            if unit == 'm':
                for i in range(3):
                    left_mm[i] *= 1000
        if right_pose is not None:
            right_mm = list(right_pose)
            if unit == 'm':
                for i in range(3):
                    right_mm[i] *= 1000

        left_success = False
        right_success = False
        left_joints = None
        right_joints = None

        if left_mm is not None:
            try:
                left_mat = self.kine_left.xyzabc_to_mat4x4(left_mm)
                left_ik = self.kine_left.ik(
                    robot_serial=0,
                    pose_mat=left_mat,
                    ref_joints=ref_left,
                    zsp_type=self.zsp_type,
                    zsp_para=self.left_zsp_para,
                    zsp_angle=self.zsp_angle,
                    dgr=self.dgr,
                )
                if left_ik is not False:
                    if not left_ik.m_Output_IsOutRange and not left_ik.m_Output_IsJntExd:
                        left_joints = left_ik.m_Output_RetJoint.to_list()
                        left_success = True
            except Exception as e:
                self.logger.debug(f"left IK exception: {e}")

        if right_mm is not None:
            try:
                right_mat = self.kine_right.xyzabc_to_mat4x4(right_mm)
                right_ik = self.kine_right.ik(
                    robot_serial=1,
                    pose_mat=right_mat,
                    ref_joints=ref_right,
                    zsp_type=self.zsp_type,
                    zsp_para=self.right_zsp_para,
                    zsp_angle=self.zsp_angle,
                    dgr=self.dgr,
                )
                if right_ik is not False:
                    if not right_ik.m_Output_IsOutRange and not right_ik.m_Output_IsJntExd:
                        right_joints = right_ik.m_Output_RetJoint.to_list()
                        right_success = True
            except Exception as e:
                self.logger.debug(f"right IK exception: {e}")

        if left_joints is not None:
            left_joints_str = ', '.join([f'{j:7.2f}' for j in left_joints])
            self.logger.debug(f"[LEFT_IK]  joints: [{left_joints_str}]")
        else:
            self.logger.debug("[LEFT_IK]  FAILED!")

        if right_joints is not None:
            right_joints_str = ', '.join([f'{j:7.2f}' for j in right_joints])
            self.logger.debug(f"[RIGHT_IK] joints: [{right_joints_str}]")
        else:
            self.logger.debug("[RIGHT_IK] FAILED!")

        return left_success, right_success, left_joints, right_joints

