#!/usr/bin/env python3
"""Tianji hardware connection, joint commands, mode switching, and Home."""
try:
    from tianji_output._internal.fx_robot import Marvin_Robot
    from tianji_output._internal.structure_data import DCSS
    from tianji_output.fault_codes import describe_fault_codes, read_servo_fault_codes
except ImportError:
    from ._internal.fx_robot import Marvin_Robot
    from ._internal.structure_data import DCSS
    from .fault_codes import describe_fault_codes, read_servo_fault_codes
import time
import logging
from .configuration import load_arm_config
from .tool_parameters import apply_tool_parameters


class TianjiChestDriver:
    """Single dual-arm hardware connection; all joint angles are in degrees."""

    def __init__(self, robot_ip=None, logger=None, *, config=None, on_command=None, stop_requested=None):
        """
        Initialize both arms.

        Args:
            robot_ip: optional override for driver.robot_ip.
            logger: optional external logger (e.g. ROS2 node logger).
            config: complete arm configuration; otherwise load the default file.
        """
        if logger is not None:
            self.logger = logger
        else:
            self.logger = logging.getLogger('TianjiChestDriver')
            self.logger.setLevel(logging.INFO)
            if not self.logger.handlers:
                handler = logging.StreamHandler()
                handler.setFormatter(logging.Formatter('[%(name)s] %(message)s'))
                self.logger.addHandler(handler)

        self._on_command = on_command
        self._stop_requested = stop_requested
        cfg = load_arm_config() if config is None else config
        self._tool_config = cfg['driver']
        self._init_joints_left = list(cfg['left_arm']['home_joints'])
        self._init_joints_right = list(cfg['right_arm']['home_joints'])
        if robot_ip is None:
            robot_ip = cfg['driver']['robot_ip']
        # ---------------------- Robot connection ----------------------
        self.logger.debug("Initializing robot control...")
        self.robot = Marvin_Robot()

        init = self.robot.connect(robot_ip)
        if init == 0:
            raise ConnectionError("Connection failed: port in use")

        time.sleep(0.5)
        self.robot.clear_set()
        self.robot.clear_error('A')
        self.robot.clear_error('B')
        self.robot.send_cmd()
        time.sleep(0.5)

        if not self._verify_connection():
            raise ConnectionError("Robot connection failed")

        # Tool dynamic parameters (Wuji Hand)
        self._set_tool_params()

        self.logger.info("Dual-arm controller initialized")

    def _set_tool_params(self):
        """Apply the configured cabinet tool-dynamics source."""
        self._active_tool_dynamics = apply_tool_parameters(self.robot, self._tool_config, self.logger)
        time.sleep(0.3)

    def _verify_connection(self):
        """Sanity check: subscribe a few times, look for frame_serial advancing."""
        dcss = DCSS()
        motion_tag = 0
        frame_update = None
        for i in range(5):
            sub_data = self.robot.subscribe(dcss)
            serial = sub_data['outputs'][0]['frame_serial']
            if serial != 0 and frame_update != serial:
                motion_tag += 1
                frame_update = serial
            time.sleep(0.1)
        return motion_tag > 0

    # ==================== State queries ====================

    def get_current_joints(self):
        """Return (left_joints, right_joints), each 7 joint angles in degrees."""
        dcss = DCSS()
        sub_data = self.robot.subscribe(dcss)
        left_joints = sub_data["outputs"][0]["fb_joint_pos"]
        right_joints = sub_data["outputs"][1]["fb_joint_pos"]
        return left_joints, right_joints

    def get_current_joint_velocities(self):
        """Return (left_velocities, right_velocities), each 7 joint velocities in deg/s."""
        dcss = DCSS()
        sub_data = self.robot.subscribe(dcss)
        left_vel = sub_data["outputs"][0]["fb_joint_vel"]
        right_vel = sub_data["outputs"][1]["fb_joint_vel"]
        return left_vel, right_vel

    def get_current_joint_torques(self):
        """Return (left_torques, right_torques), each 7 joint torques in Nm."""
        dcss = DCSS()
        sub_data = self.robot.subscribe(dcss)
        left_torque = sub_data["outputs"][0]["fb_joint_tor"]
        right_torque = sub_data["outputs"][1]["fb_joint_tor"]
        return left_torque, right_torque

    def get_full_state(self):
        """Return dual-arm joint positions, velocities, and torques as a dict."""
        dcss = DCSS()
        sub_data = self.robot.subscribe(dcss)
        return {
            'left': {
                'joints': sub_data["outputs"][0]["fb_joint_pos"],
                'velocities': sub_data["outputs"][0]["fb_joint_vel"],
                'torques': sub_data["outputs"][0]["fb_joint_tor"],
            },
            'right': {
                'joints': sub_data["outputs"][1]["fb_joint_pos"],
                'velocities': sub_data["outputs"][1]["fb_joint_vel"],
                'torques': sub_data["outputs"][1]["fb_joint_tor"],
            }
        }

    def get_arm_states_only(self):
        """Return (left_cur_state, right_cur_state) from cabinet feedback."""
        dcss = DCSS()
        sub_data = self.robot.subscribe(dcss)
        return (sub_data["states"][0]["cur_state"], sub_data["states"][1]["cur_state"])

    def get_arm_status(self):
        """Dual-arm state codes, servo error codes, and trajectory-realtime metrics.

        Returns:
            dict: {'left': {'state': int, 'err_code': int,
                            'servo_errors': list[str],          # 7 hex strings
                            'servo_descriptions': list[str],    # 7 EN strings (empty=no error)
                            'realtime': dict},                  # frame_miss_cnt etc.
                   'right': ...}
        """
        dcss = DCSS()
        sub_data = self.robot.subscribe(dcss)
        result = {}
        for idx, (arm_id, side) in enumerate([('A', 'left'), ('B', 'right')]):
            servo_errors = self._get_servo_errors_raw(arm_id)
            inputs = sub_data.get('inputs') or []
            rt_in = inputs[idx] if idx < len(inputs) else {}
            frame_miss_cnt = int(rt_in.get('frame_miss_cnt', 0))
            result[side] = {
                'state': sub_data["states"][idx]["cur_state"],
                'err_code': sub_data["states"][idx]["err_code"],
                'servo_errors': servo_errors,
                'servo_descriptions': describe_fault_codes(servo_errors, empty=''),
                'realtime': {
                    'frame_miss_cnt': frame_miss_cnt,
                    'max_frame_miss_cnt': int(rt_in.get('max_frame_miss_cnt', 0)),
                    'in_frame_serial': int(rt_in.get('in_frame_serial', 0)),
                    'sys_cyc_miss_cnt': int(rt_in.get('sys_cyc_miss_cnt', 0)),
                    'max_sys_cyc_miss_cnt': int(rt_in.get('max_sys_cyc_miss_cnt', 0)),
                    'quality': 'good' if frame_miss_cnt < 20 else 'poor',
                },
            }
        return result

    def _get_servo_errors_raw(self, arm: str):
        """Return 7 servo fault codes for the given arm (hex string list)."""
        return read_servo_fault_codes(self.robot.robot, arm)

    # ==================== Impedance mode ====================

    def set_impedance_mode(self, mode='joint', K=None, D=None):
        """
        Set dual-arm impedance mode.

        Set impedance parameters and subtype, then request state3.

        Args:
            mode: 'joint' or 'cart'.
            K: stiffness list (7 elements).
            D: damping list (7 elements).
        """
        if mode == 'cart':
            K = K or [8000, 8000, 8000, 100, 100, 100, 20]
            D = D or [0.3, 0.3, 0.3, 0.4, 0.4, 0.4, 0.4]

            self.robot.clear_set()
            self.robot.set_cart_kd_params(arm='A', K=K, D=D, type=2)
            self.robot.set_cart_kd_params(arm='B', K=K, D=D, type=2)
            self.robot.set_impedance_type(arm='A', type=2)
            self.robot.set_impedance_type(arm='B', type=2)
            self.robot.send_cmd()
            time.sleep(0.5)

            self.logger.info(f"Dual-arm Cartesian impedance mode K={K}")

        elif mode == 'joint':
            K = K or [2, 2, 2, 1.6, 1, 1, 1]
            D = D or [0.3, 0.3, 0.3, 0.2, 0.2, 0.2, 0.2]

            self.robot.clear_set()
            self.robot.set_joint_kd_params(arm='A', K=K, D=D)
            self.robot.set_joint_kd_params(arm='B', K=K, D=D)
            self.robot.set_impedance_type(arm='A', type=1)
            self.robot.set_impedance_type(arm='B', type=1)
            self.robot.send_cmd()
            time.sleep(0.5)

            self.logger.info(f"Dual-arm joint impedance mode K={K}")

        # Servo-on after params are loaded.
        self.robot.clear_set()
        self.robot.set_state(arm='A', state=3)
        self.robot.set_state(arm='B', state=3)
        self.robot.set_vel_acc(arm='A', velRatio=60, AccRatio=60)
        self.robot.set_vel_acc(arm='B', velRatio=60, AccRatio=60)
        self.robot.send_cmd()

        # Poll for the requested state and report the result.
        ok, diag = self._poll_arm_state_after_switch(target_state=3, timeout=5.0)
        if not ok:
            self.logger.warning("set_impedance_mode: %s — proceeding (main-parity)", diag)
        else:
            self.logger.info("Dual-arm impedance mode active (cur_state=3)")

    def _poll_arm_state_after_switch(self, target_state, timeout=5.0, poll_interval=0.05):
        """Poll for bilateral target state; return (confirmed, diagnostic)."""
        dcss = DCSS()
        deadline = time.monotonic() + timeout
        a_state, b_state, a_err, b_err = -1, -1, -1, -1
        while time.monotonic() < deadline:
            sub_data = self.robot.subscribe(dcss)
            a_state = sub_data["states"][0]["cur_state"]
            b_state = sub_data["states"][1]["cur_state"]
            a_err = sub_data["states"][0]["err_code"]
            b_err = sub_data["states"][1]["err_code"]
            if a_state == target_state and b_state == target_state:
                return True, ""
            # Continue polling until the target state or deadline.
            time.sleep(poll_interval)
        diag = (
            f"state-switch timed out after {timeout}s target={target_state} "
            f"left(state={a_state}, err_code={a_err}); "
            f"right(state={b_state}, err_code={b_err})"
        )
        return False, diag

    # ==================== Joint-space control ====================

    def move_to_joints_direct(self, left_joints=None, right_joints=None):
        """
        Joint-space control: dispatch joint angle commands directly.
        Non-blocking, suitable for realtime tracking.

        Args:
            left_joints: [j1..j7] left target angles (degrees); None to skip.
            right_joints: [j1..j7] right target angles (degrees); None to skip.

        Returns:
            tuple: (left_success, right_success).
        """
        left_success = left_joints is not None
        right_success = right_joints is not None

        self.robot.clear_set()
        if left_joints is not None:
            self.robot.set_joint_cmd_pose(arm='A', joints=list(left_joints))
            left_joints_str = ', '.join([f'{j:7.2f}' for j in left_joints])
            self.logger.debug(f"[LEFT]  joints: [{left_joints_str}]")
        if right_joints is not None:
            self.robot.set_joint_cmd_pose(arm='B', joints=list(right_joints))
            right_joints_str = ', '.join([f'{j:7.2f}' for j in right_joints])
            self.logger.debug(f"[RIGHT] joints: [{right_joints_str}]")
        submitted = self.robot.send_cmd()
        if submitted and self._on_command is not None:
            self._on_command(left_joints, right_joints)
        return left_success and submitted, right_success and submitted

    def move_to_joints_smooth(self, left_target=None, right_target=None, duration=3.0, dt=0.01):
        """
        Smoothly move both arms to the target joint angles using a quintic blend.

        Args:
            left_target: [j1..j7] left target angles; None keeps current.
            right_target: [j1..j7] right target angles; None keeps current.
            duration: trajectory duration (s); larger = slower / smoother.
            dt: interpolation step (s).

        Returns:
            bool: success.
        """
        left_joints, right_joints = self.get_current_joints()
        start_left = list(left_joints)
        start_right = list(right_joints)

        if left_target is None:
            left_target = start_left
        if right_target is None:
            right_target = start_right

        num_points = int(duration / dt)

        self.logger.debug(f"Smooth move to target ({duration}s quintic blend)...")

        for i in range(num_points + 1):
            if self._stop_requested is not None and self._stop_requested():
                raise InterruptedError("Home interrupted by driver shutdown")
            t = i / num_points
            s = 10 * (t ** 3) - 15 * (t ** 4) + 6 * (t ** 5)

            target_left = [
                start_left[j] + s * (left_target[j] - start_left[j])
                for j in range(7)
            ]
            target_right = [
                start_right[j] + s * (right_target[j] - start_right[j])
                for j in range(7)
            ]

            self.move_to_joints_direct(target_left, target_right)

            time.sleep(dt)

        return True

    # ==================== Home pose / release ====================

    def move_to_init(self, wait=True, timeout=1, duration=3.0, dt=0.01):
        """
        Smoothly move both arms to the configured init pose.

        Args:
            wait: if True, sleep `timeout` seconds after the move.
            timeout: extra settle time (s).
            duration: trajectory duration (s).
            dt: interpolation step (s).

        Returns:
            bool: success.
        """
        INIT_JOINTS_LEFT = list(self._init_joints_left)
        INIT_JOINTS_RIGHT = list(self._init_joints_right)

        self.logger.debug(f"Moving to init pose ({duration}s quintic blend)...")

        self.move_to_joints_smooth(
            left_target=INIT_JOINTS_LEFT,
            right_target=INIT_JOINTS_RIGHT,
            duration=duration,
            dt=dt
        )

        if wait:
            time.sleep(timeout)

        final_left, final_right = self.get_current_joints()
        left_errors = [abs(final_left[i] - INIT_JOINTS_LEFT[i]) for i in range(7)]
        right_errors = [abs(final_right[i] - INIT_JOINTS_RIGHT[i]) for i in range(7)]
        max_left_error = max(left_errors)
        max_right_error = max(right_errors)

        success = True
        if max_left_error < 5.0:
            self.logger.debug("[arm A] reached init pose")
        else:
            self.logger.warning(f"[arm A] init-pose residual {max_left_error:.1f} deg")
            success = False

        if max_right_error < 5.0:
            self.logger.debug("[arm B] reached init pose")
        else:
            self.logger.warning(f"[arm B] init-pose residual {max_right_error:.1f} deg")
            success = False

        return success

    def set_standby(self):
        """Servo-off (state=0), keep connection, ready to resume. State-driven (waits cur_state=0)."""
        self.logger.info("Dual-arm STANDBY (state=0, connection preserved)")
        self.robot.clear_set()
        self.robot.set_state(arm='A', state=0)
        self.robot.set_state(arm='B', state=0)
        self.robot.send_cmd()

        ok, diag = self._poll_arm_state_after_switch(target_state=0, timeout=2.0)
        if not ok:
            self.logger.warning("set_standby: %s — proceeding (main-parity)", diag)
        else:
            self.logger.info("Dual-arm STANDBY reached (cur_state=0)")

    def set_active(self, mode='joint', K=None, D=None):
        """Clear errors and request impedance mode (state3)."""
        self.logger.info("Dual-arm ACTIVE (impedance mode)")
        self.robot.clear_set()
        self.robot.clear_error('A')
        self.robot.clear_error('B')
        self.robot.send_cmd()
        time.sleep(0.5)
        self.set_impedance_mode(mode=mode, K=K, D=D)

    def set_position_mode(self, velRatio=30, AccRatio=30):
        """Switch to position-tracking mode (state=1), with state-machine poll (target=1, timeout=2s)."""
        self.robot.clear_set()
        self.robot.set_state(arm='A', state=1)
        self.robot.set_state(arm='B', state=1)
        self.robot.set_vel_acc(arm='A', velRatio=velRatio, AccRatio=AccRatio)
        self.robot.set_vel_acc(arm='B', velRatio=velRatio, AccRatio=AccRatio)
        self.robot.send_cmd()

        ok, diag = self._poll_arm_state_after_switch(target_state=1, timeout=2.0)
        if not ok:
            self.logger.warning("set_position_mode: %s — proceeding (main-parity)", diag)
        else:
            self.logger.info("Dual-arm position mode active (cur_state=1)")

    def release_brake(self, arm: str = 'A'):
        """Release the brake. Direct SDK call, not via clear_set/send_cmd batching."""
        param = 'BRAK0' if arm == 'A' else 'BRAK1'
        self.robot.set_param('int', param, 2)
        self.logger.info(f"{'left' if arm == 'A' else 'right'} arm brake released")

    def hold_brake(self, arm: str = 'A'):
        """Engage the brake. Direct SDK call."""
        param = 'BRAK0' if arm == 'A' else 'BRAK1'
        self.robot.set_param('int', param, 1)
        self.logger.info(f"{'left' if arm == 'A' else 'right'} arm brake held")

    def clear_arm_error(self, arm: str = 'A'):
        """Clear error state on a single arm."""
        side = 'left' if arm == 'A' else 'right'
        self.robot.clear_set()
        self.robot.clear_error(arm)
        self.robot.send_cmd()
        time.sleep(0.5)
        self.logger.info(f"{side} arm error cleared (clear_error {arm})")

    def disable_and_release(self, poll_timeout=3.0):
        """Request state0, poll for confirmation, log errors, and release."""
        self.logger.info("Disabling arms (set_state=0)...")
        try:
            self.robot.clear_set()
            self.robot.set_state(arm='A', state=0)
            self.robot.set_state(arm='B', state=0)
            self.robot.send_cmd()
        except Exception as exc:
            self.logger.warning("set_state(0) on shutdown failed (ignored): %s", exc)

        a, b = -1, -1
        try:
            dcss = DCSS()
            deadline = time.monotonic() + max(0.5, float(poll_timeout))
            while time.monotonic() < deadline:
                sub = self.robot.subscribe(dcss)
                a = sub["states"][0]["cur_state"]
                b = sub["states"][1]["cur_state"]
                if a == 0 and b == 0:
                    self.logger.info("Both arms at cur_state=0")
                    break
                time.sleep(0.05)
            else:
                self.logger.warning(
                    "Servos may still be on (cur_state left=%d, right=%d); "
                    "releasing TCP anyway", a, b)
        except Exception as exc:
            self.logger.warning("cur_state poll failed (ignored): %s", exc)

        self.logger.debug("Releasing connection...")
        try:
            self.robot.release_robot()
        except Exception as exc:
            self.logger.warning("release_robot failed (ignored): %s", exc)
        self.logger.info("Safely exited")
