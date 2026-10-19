"""Tianji arm controller node: TF -> IK -> hardware."""
from __future__ import annotations

import argparse
from functools import wraps
import sys
import threading
import time
from typing import Optional

import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy,
)
from rclpy.utilities import remove_ros_args
from rclpy._rclpy_pybind11 import RCLError
from tf2_ros import Buffer, TransformListener
import tf2_ros
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Int8
from std_srvs.srv import SetBool, Trigger

from tianji_output import TianjiChestDriver
from tianji_output.feedback_guard import joint_vector, limit_joint_step
from .common import (
    ArmState,
    ROS2LoggerAdapter,
    get_default_qos,
    load_yaml_config,
    get_package_config_path,
)


# Topic names
LEFT_ARM_CMD_TOPIC = "/left_arm/joint_commands"
RIGHT_ARM_CMD_TOPIC = "/right_arm/joint_commands"
LEFT_ARM_STATE_TOPIC = "/left_arm/joint_states"
RIGHT_ARM_STATE_TOPIC = "/right_arm/joint_states"

# Latched lifecycle state on /tianji_arm/lifecycle_state (Int8). Replaces the
# old Bool /tianji_arm/teleop_ready — that signal was edge-only ("ready or
# not"), so a Monitor waiting on it couldn't tell the difference between
# "still initializing" and "_do_enable failed with move_to_init residual".
# The Monitor's Stop button got stuck disabled in the second case. Lifecycle
# is the state machine the operator actually needs to see:
#
#   INITIALIZING  - node __init__ running; controller not yet armed
#   ENABLING      - _do_enable in progress (set_position_mode + move_to_init
#                   + set_impedance_mode); Marvin holds servos
#   READY         - _do_enable completed; teleop loop running
#   DISABLED      - _do_disable called (set_standby); waiting for re-enable
#   ENABLE_FAILED - _do_enable raised; operator should Stop or call retry
#   SDK_ERROR     - _on_sdk_error called; controller wedged
LIFECYCLE_INITIALIZING = 0
LIFECYCLE_ENABLING = 1
LIFECYCLE_READY = 2
LIFECYCLE_DISABLED = 3
LIFECYCLE_ENABLE_FAILED = 4
LIFECYCLE_SDK_ERROR = 5


class TianjiArmControllerNode(Node):
    """Tianji arm controller node."""

    def __init__(self, robot_ip: str = '192.168.1.190'):
        super().__init__("tianji_arm_controller")

        # auto_enable=True: node powers up the arm asynchronously
        # via a one-shot timer after __init__ returns. Setting it False keeps
        # the arm in standby on startup; operators then call ~/set_enabled to
        # bring it up. Decoupling enable from __init__ keeps SIGINT during the
        # ~10s move_to_init from leaving the node half-constructed (so
        # shutdown() never ran and the arm stayed live).
        self.declare_parameter('auto_enable', False)
        self.declare_parameter('enable_timeout', 25.0)
        self._enable_timeout = float(self.get_parameter('enable_timeout').value)
        if not np.isfinite(self._enable_timeout) or self._enable_timeout <= 0:
            raise ValueError('enable_timeout must be finite and positive')
        self.declare_parameter('tracker_timeout', 0.25)
        self._tracker_timeout = float(self.get_parameter('tracker_timeout').value)
        if not np.isfinite(self._tracker_timeout) or self._tracker_timeout <= 0:
            raise ValueError('tracker_timeout must be finite and positive')
        # Control loop / state-publish rates (Hz, two timers decoupled):
        # - control_rate: TF -> IK control path. 120Hz matches the upper bound
        #   of HTC tracker / MANUS input streams; higher adds no
        #   new command information.
        # - state_publish_rate: /{side}_arm/joint_states publish rate.
        #   Downstream inference + data capture want finer state feedback;
        #   500Hz is stable under Python + DDS.
        # No rebuild required to change rates — launch / param yaml overrides work.
        self.declare_parameter('control_rate', 120.0)
        self.declare_parameter('state_publish_rate', 500.0)
        self.declare_parameter('max_joint_velocity_deg_s', 30.0)
        self._max_joint_velocity = float(self.get_parameter('max_joint_velocity_deg_s').value)
        if not np.isfinite(self._max_joint_velocity) or not 0 < self._max_joint_velocity <= 30.0:
            raise ValueError('max_joint_velocity_deg_s must be finite and within (0,30]')
        # Handoff after move_to_init: hold the init pose for `handoff_hold_sec`,
        # then run a `handoff_ramp_sec` smoothstep ramp from the snapshot of
        # current hardware joints to the live IK target. Avoids the snap from
        # init pose to the operator's tracker pose on the first IK frame.
        # Set either to 0.0 to disable that phase. Default 0.5s hold + 1.0s
        # ramp is the value that field-tested cleanly on Wuji Hand + Tianji.
        self.declare_parameter('handoff_hold_sec', 0.5)
        self.declare_parameter('handoff_ramp_sec', 1.0)
        auto_enable = bool(self.get_parameter('auto_enable').value)
        self._control_rate_hz = float(self.get_parameter('control_rate').value)
        self._state_publish_rate_hz = float(self.get_parameter('state_publish_rate').value)
        self._handoff_hold_sec = float(self.get_parameter('handoff_hold_sec').value)
        self._handoff_ramp_sec = float(self.get_parameter('handoff_ramp_sec').value)
        if not np.isfinite(self._control_rate_hz) or self._control_rate_hz <= 0.0:
            raise ValueError(
                f"control_rate must be > 0, got {self._control_rate_hz}")
        if not np.isfinite(self._state_publish_rate_hz) or self._state_publish_rate_hz <= 0.0:
            raise ValueError(
                f"state_publish_rate must be > 0, got {self._state_publish_rate_hz}")
        if (not np.isfinite(self._handoff_hold_sec) or not np.isfinite(self._handoff_ramp_sec)
                or self._handoff_hold_sec < 0.0 or self._handoff_ramp_sec < 0.0):
            raise ValueError('handoff times must be finite and nonnegative')

        self._arm_state = ArmState.IMPEDANCE
        self._logger_adapter = ROS2LoggerAdapter(self.get_logger())
        self._log_counter = 0
        self._state_error_count = 0  # consecutive get_current_joints failures

        # Initialize the controller.
        self.get_logger().info(f"Connecting to robot {robot_ip}...")
        self.controller = TianjiChestDriver(robot_ip=robot_ip, logger=self._logger_adapter)

        # STANDBY / ACTIVE state management.
        self._arm_enabled = False
        # Every SDK access uses this lock. Only the request flag may be set
        # concurrently, so a Stop can interrupt initialization without a
        # second thread writing a competing cabinet command.
        self._enable_lock = threading.RLock()
        self._request_lock = threading.Lock()
        self._disable_requests = 0
        self._stop_requested = threading.Event()
        self._enable_deadline = None
        self._shutdown_requested = threading.Event()

        # Pose and joint caches.
        self.left_pose = None
        self.right_pose = None
        self.left_y_axis = None
        self.right_y_axis = None
        self._last_left_state_joints = None
        self._last_right_state_joints = None

        # Handoff ramp state — populated at the end of _do_enable, consumed by
        # _teleop_control. `_handoff_start_at is None` means no ramp pending.
        self._handoff_start_at: Optional[float] = None
        self._handoff_start_left: Optional[list] = None
        self._handoff_start_right: Optional[list] = None
        self._last_command_left = None
        self._last_command_right = None
        self._last_command_at = None

        # Always enter standby first. set_standby failure (e.g. state=100
        # under e-stop) must not block node startup, otherwise launch
        # respawn=True crash-loops until the fault is cleared externally.
        standby_ready = False
        try:
            self.controller.set_standby()
            standby_ready = True
            self.get_logger().info("Arm connected; servos in standby")
        except Exception as exc:
            self.get_logger().error(
                f"set_standby failed at startup (node will continue, "
                f"awaiting operator intervention): {exc}"
            )

        # If auto_enable, schedule async enable via a one-shot timer. Keeping
        # the ~10s move_to_init out of __init__ means SIGINT during startup
        # arrives at a fully-constructed node, so shutdown() runs and the arm
        # is brought back to standby.
        self._auto_enable_timer = None
        if auto_enable:
            self._auto_enable_timer = self.create_timer(0.1, self._auto_enable_once)

        # TF listener.
        self.tf_buffer = Buffer()
        # Long initialization motions run synchronously on the control node.
        # Keep TF reception alive on its own node/executor; this thread never
        # accesses the robot SDK or sends a motion command.
        self._tf_node = Node('tianji_tracker_listener', context=self.context)
        self.tf_listener = TransformListener(self.tf_buffer, self._tf_node)
        self._tf_executor = SingleThreadedExecutor(context=self.context)
        self._tf_executor.add_node(self._tf_node)
        self._tf_thread = threading.Thread(target=self._spin_tf, name='tianji-tf', daemon=True)
        self._tf_thread.start()

        qos = get_default_qos()

        # Publishers.
        self.left_cmd_pub = self.create_publisher(JointState, LEFT_ARM_CMD_TOPIC, qos)
        self.right_cmd_pub = self.create_publisher(JointState, RIGHT_ARM_CMD_TOPIC, qos)
        self.left_state_pub = self.create_publisher(JointState, LEFT_ARM_STATE_TOPIC, qos)
        self.right_state_pub = self.create_publisher(JointState, RIGHT_ARM_STATE_TOPIC, qos)

        # zsp_para and pose publishers.
        self.left_zsp_para_pub = self.create_publisher(Float64MultiArray, '/left_arm/zsp_para', qos)
        self.right_zsp_para_pub = self.create_publisher(Float64MultiArray, '/right_arm/zsp_para', qos)
        self.left_ee_pose_pub = self.create_publisher(Float64MultiArray, '/left_arm/ee_pose', qos)
        self.right_ee_pose_pub = self.create_publisher(Float64MultiArray, '/right_arm/ee_pose', qos)

        # Latched lifecycle state. TRANSIENT_LOCAL so late-joining
        # subscribers (e.g. the Monitor UI opened after the controller is up)
        # immediately receive the last value, rather than racing the next
        # state transition. Monitor uses this to gate the Stop Teleop button
        # on real controller state (see the LIFECYCLE_* constants at top).
        lifecycle_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._lifecycle_pub = self.create_publisher(
            Int8, '/tianji_arm/lifecycle_state', lifecycle_qos)
        self._publish_lifecycle(LIFECYCLE_INITIALIZING)

        # Services.
        self._enable_callback_group = ReentrantCallbackGroup()
        self.create_service(SetBool, '~/set_enabled', self._set_enabled_callback,
                            callback_group=self._enable_callback_group)
        self.create_service(SetBool, '/tianji_arm/set_arm_state',
                            self._serialized_service(self._set_arm_state_callback))
        self.create_service(SetBool, '~/left/brake', self._serialized_service(self._left_brake_cb))
        self.create_service(SetBool, '~/right/brake', self._serialized_service(self._right_brake_cb))
        self.create_service(Trigger, '~/arm_status', self._serialized_service(self._arm_status_cb))
        self.create_service(SetBool, '~/left/clear_error',
                            self._serialized_service(self._left_clear_error_cb))
        self.create_service(SetBool, '~/right/clear_error',
                            self._serialized_service(self._right_clear_error_cb))

        # Control loop + state-publish: two timers (rates read from ROS2 params; see top of __init__).
        self.create_timer(1.0 / self._control_rate_hz, self._control_loop)
        self.create_timer(1.0 / self._state_publish_rate_hz, self._state_publish_loop)

        self.get_logger().info(
            f"Init complete: control_rate={self._control_rate_hz:.1f}Hz, "
            f"state_publish_rate={self._state_publish_rate_hz:.1f}Hz"
        )
        if not auto_enable:
            self._publish_lifecycle(LIFECYCLE_DISABLED if standby_ready else LIFECYCLE_SDK_ERROR)

    # -------------------- Latched lifecycle state --------------------

    def _publish_lifecycle(self, state: int):
        """Push the latched lifecycle state. Safe to call before the
        publisher exists; silently no-ops in that case."""
        pub = getattr(self, '_lifecycle_pub', None)
        if pub is None:
            return
        msg = Int8()
        msg.data = int(state)
        try:
            pub.publish(msg)
        except Exception as exc:
            self.get_logger().debug(f"lifecycle_state publish skipped: {exc}")

    # -------------------- SDK exception handling --------------------

    def _on_sdk_error(self, context: str, error: Exception):
        """On SDK exception, auto-disable the arm to avoid a deadlocked state.
        Respawn recovers automatically."""
        self.get_logger().error(f"[{context}] SDK exception: {error}; auto-disabling arm")
        self._arm_enabled = False
        self._arm_state = ArmState.IMPEDANCE
        self.left_pose = None
        self.right_pose = None
        self._publish_lifecycle(LIFECYCLE_SDK_ERROR)
        try:
            self.controller.set_standby()
        except Exception as e2:
            # Error-recovery path; do not re-raise (would clobber the outer
            # timer callback). Must still log: SDK is already faulted and
            # set_standby also failing means a real hardware fault.
            self.get_logger().error(
                f"[{context}] set_standby also failed during recovery: {e2} — "
                f"SDK may be wedged, awaiting respawn"
            )

    # -------------------- Enable / disable --------------------

    def _serialized_service(self, callback):
        """Other services never queue an SDK operation behind initialization."""
        @wraps(callback)
        def serialized(request, response):
            if not self._enable_lock.acquire(blocking=False):
                response.success = False
                response.message = 'Arm SDK busy; initialization or stopping is in progress'
                return response
            try:
                with self._request_lock:
                    stopping = self._disable_requests > 0 or self._shutdown_requested.is_set()
                if stopping:
                    response.success = False
                    response.message = 'Arm stop is pending; request rejected'
                    return response
                return callback(request, response)
            finally:
                self._enable_lock.release()
        return serialized

    def _do_enable(self):
        """Bring the arms from STANDBY to teleop-ready.

        Sequence:
            1. set_position_mode (rigid joint control) — required so
               move_to_init can actually drive the joints to the target;
               impedance K=2 is too soft for a 100°+ traverse in 3 s and
               leaves residuals of 120° (real failure observed 2026-06-11
               on tianji_arm_controller @ 192.168.1.190; Marvin replied
               cur_state=3 but the joints sat where they were).
            2. move_to_init quintic blend — accurate now that the joints
               are rigid.
            3. set_impedance_mode (the joint K/D from the README) — this
               is the mode the teleop loop runs in.

        Idempotent under _enable_lock: if already enabled, returns
        immediately without re-publishing lifecycle.
        """
        with self._enable_lock:
            if self._arm_enabled:
                return
            if not self._trackers_ready():
                raise RuntimeError('All five Trackers must have fresh valid poses before enabling arms')
            # Position mode with FULL velocity/acceleration ratio so the joints
            # actually reach init pose in the allotted window. Earlier defaults
            # (velRatio=30 / duration=3 / timeout=10) only moved ~40° per joint;
            # init pose differs by 120° on J1 / J4 — far past 40° — so the
            # residual check failed and the operator's Stop got gated forever.
            # quintic-blend duration 8s + 15s settle covers the worst case
            # (~170° single-joint traverse at this rate).
            self._enable_deadline = time.monotonic() + self._enable_timeout
            try:
                if self._initialization_should_abort():
                    raise RuntimeError('Arm initialization cancelled before enabling')
                self._publish_lifecycle(LIFECYCLE_ENABLING)
                self.controller.set_position_mode(
                    velRatio=100, AccRatio=100, should_abort=self._initialization_should_abort)
                result = self.controller.move_to_init(
                    wait=True, timeout=15, duration=8.0,
                    should_abort=self._initialization_should_abort)
                if result is False:
                    raise RuntimeError("move_to_init failed")
                if self._initialization_should_abort():
                    raise RuntimeError('Arm initialization stopped, timed out, or lost Tracker input')
                self.controller.set_impedance_mode(
                    mode='joint', should_abort=self._initialization_should_abort)
                if self._initialization_should_abort():
                    raise RuntimeError('Arm initialization stopped, timed out, or lost Tracker input')
                self._arm_handoff_snapshot()
                with self._request_lock:
                    if self._initialization_should_abort():
                        raise RuntimeError('Arm initialization cancelled before READY')
                    self._arm_state = ArmState.IMPEDANCE
                    self._arm_enabled = True
                    self._publish_lifecycle(LIFECYCLE_READY)
            except BaseException:
                self._arm_enabled = False
                self._handoff_start_at = None
                try:
                    self.controller.set_standby()
                except Exception as stop_error:
                    self.get_logger().error(f"Standby after initialization failure failed: {stop_error}")
                raise
            finally:
                self._enable_deadline = None

    def _do_disable(self):
        """Standby (servo-off, keep connection). Idempotent under _enable_lock."""
        self._stop_requested.set()
        with self._enable_lock:
            self._arm_enabled = False
            self._arm_state = ArmState.IMPEDANCE
            self.left_pose = None
            self.right_pose = None
            self.left_y_axis = None
            self.right_y_axis = None
            self._handoff_start_at = None
            self._handoff_start_left = None
            self._handoff_start_right = None
            self._last_command_left = None
            self._last_command_right = None
            self._last_command_at = None
            try:
                self.controller.set_standby()
            except Exception:
                self._publish_lifecycle(LIFECYCLE_SDK_ERROR)
                raise
            self._publish_lifecycle(LIFECYCLE_DISABLED)

    def _auto_enable_once(self):
        """One-shot timer callback: run _do_enable asynchronously. A failure
        here leaves the node up (operator can retry via ~/set_enabled) but
        publishes LIFECYCLE_ENABLE_FAILED so the Monitor can re-enable Stop
        Teleop instead of waiting forever for a READY signal that never
        comes."""
        # Cancel + destroy the timer first so a slow _do_enable doesn't get
        # re-entered by the next tick.
        if self._auto_enable_timer is not None:
            try:
                self._auto_enable_timer.cancel()
                self.destroy_timer(self._auto_enable_timer)
            except Exception:
                pass
            self._auto_enable_timer = None
        response = self._set_enabled_callback(
            SetBool.Request(data=True), SetBool.Response(), _automatic=True)
        if not response.success:
            self.get_logger().error(f'Arm auto-enable failed: {response.message}')

    def _arm_handoff_snapshot(self):
        """Snapshot current hardware joints to seed the handoff ramp.

        After move_to_init the arm is at init pose; the operator's tracker is
        in some arbitrary position. Without a ramp the first IK frame would
        snap the arm directly to the operator's pose. Snapshot here, then
        _teleop_control blends from the snapshot to live IK over
        handoff_hold_sec + handoff_ramp_sec.
        """
        try:
            left, right = self.controller.get_current_joints()
            # Driver validates advancing cabinet frames, so a legitimate zero
            # pose is distinguishable from a default/uninitialized zero cache.
            self._handoff_start_left = joint_vector(left, 'left handoff snapshot')
            self._handoff_start_right = joint_vector(right, 'right handoff snapshot')
            self._last_command_left = self._handoff_start_left.copy()
            self._last_command_right = self._handoff_start_right.copy()
            self._last_command_at = time.monotonic()
            if self._handoff_hold_sec <= 0.0 and self._handoff_ramp_sec <= 0.0:
                self._handoff_start_at = None
                return
            self._handoff_start_at = time.monotonic()
            self.get_logger().info(
                f"Handoff ramp armed: hold={self._handoff_hold_sec:.2f}s, "
                f"ramp={self._handoff_ramp_sec:.2f}s"
            )
        except Exception as exc:
            self._handoff_start_at = None
            self._handoff_start_left = None
            self._handoff_start_right = None
            raise RuntimeError(f"Cannot start teleoperation without measured handoff pose: {exc}") from exc

    def _set_enabled_callback(self, request: SetBool.Request, response: SetBool.Response,
                              _automatic=False):
        """Stop is visible immediately; the SDK owner performs ordered rollback.

        This service alone is reentrant. Repeated enables fail immediately;
        disables set the event before waiting for the SDK lock. Pending stops
        prevent a later enable from clearing that event ahead of its waiter.
        Success still means the requested state completed, not just acceptance.
        """
        if not request.data:
            with self._request_lock:
                self._disable_requests += 1
                self._stop_requested.set()
            try:
                try:
                    self._do_disable()
                except Exception as error:
                    response.success = False
                    response.message = f'Arm stop unconfirmed: {error}'
                    self.get_logger().error(response.message)
                else:
                    response.success = True
                    response.message = 'Arms stopped (both standby states confirmed)'
                return response
            finally:
                with self._request_lock:
                    self._disable_requests -= 1

        if not self._enable_lock.acquire(blocking=False):
            response.success = False
            response.message = 'Arm SDK busy; enable already running or stopping'
            return response
        try:
            with self._request_lock:
                if (self._disable_requests or self._shutdown_requested.is_set()
                        or (_automatic and self._stop_requested.is_set())):
                    response.success = False
                    response.message = 'Arm stop is pending; enable rejected'
                    return response
                self._stop_requested.clear()
            if self._arm_enabled:
                try:
                    left_s, right_s = self.controller.get_arm_states_only()
                    ready = left_s == 3 and right_s == 3 and self._trackers_ready()
                except Exception:
                    ready = False
                if ready:
                    response.success = True
                    response.message = 'Already enabled (hardware in state=3)'
                    return response
                try:
                    self._do_disable()
                    response.message = 'Existing arm session lost readiness; stopped both arms'
                except Exception as error:
                    response.message = f'Existing arm session lost readiness; stop unconfirmed: {error}'
                response.success = False
                return response
            try:
                self.get_logger().info('Enabling arms with cancellable initialization')
                self._do_enable()
                response.success = True
                response.message = 'Arms enabled, move_to_init complete'
            except Exception as e:
                self.get_logger().error(f"Enable failed: {e}")
                self._arm_enabled = False
                self._publish_lifecycle(LIFECYCLE_ENABLE_FAILED)
                response.success = False
                response.message = f"Arm enable failed: {e}"
            return response
        finally:
            self._enable_lock.release()

    # -------------------- Brake (release / hold) --------------------

    def _left_brake_cb(self, request, response):
        return self._brake_cb('A', request, response)

    def _right_brake_cb(self, request, response):
        return self._brake_cb('B', request, response)

    def _brake_cb(self, arm, request, response):
        side = 'left' if arm == 'A' else 'right'
        try:
            if request.data:
                self.controller.release_brake(arm)
                response.message = f"{side} arm: brake released"
            else:
                self.controller.hold_brake(arm)
                response.message = f"{side} arm: brake held"
            response.success = True
        except Exception as e:
            response.success = False
            response.message = f"{side} arm brake op failed: {e}"
        return response

    # -------------------- Clear error --------------------

    def _left_clear_error_cb(self, request, response):
        return self._clear_error_cb('A', request, response)

    def _right_clear_error_cb(self, request, response):
        return self._clear_error_cb('B', request, response)

    def _clear_error_cb(self, arm, request, response):
        side = 'left' if arm == 'A' else 'right'
        if self._arm_enabled:
            response.success = False
            response.message = 'Stop both arms before requesting an explicit fault clear'
            return response
        if not request.data:
            response.success = False
            response.message = 'Fault clear requires an explicit true request'
            return response
        try:
            self.controller.clear_arm_error(arm)
            response.success = True
            response.message = f'{side} arm error cleared; fresh standby confirmed'
        except Exception as e:
            response.success = False
            response.message = f"{side} arm clear-error failed: {e}"
        return response

    def _arm_status_cb(self, request, response):
        """ROS2 service: read both arms' state code / error code / servo error code."""
        try:
            import json
            status = self.controller.get_arm_status()
            response.success = True
            response.message = json.dumps(status)
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    # -------------------- Service callbacks --------------------

    def _set_arm_state_callback(self, request: SetBool.Request, response: SetBool.Response):
        """Switch arm control state:
            data=true  -> POSITION   (state=1)
            data=false -> IMPEDANCE  (state=3)

        Fast-path based on SDK cur_state; do NOT trust _arm_state / _arm_enabled
        flags (they can drift from hardware).
        """
        target_state = 1 if request.data else 3
        target_enum = ArmState.POSITION if request.data else ArmState.IMPEDANCE
        _NAMES = {1: "rigid (position)", 3: "compliant (impedance)"}

        if not self._arm_enabled:
            response.success = False
            response.message = 'Arms not enabled; enable explicitly before changing mode'
            return response
        try:
            states = self.controller.get_arm_controller_states()
            if any(state['err_code'] != 0 or state['cur_state'] not in (1, 3) for state in states):
                response.success = False
                response.message = f'Cannot switch mode with a controller fault or transition: {states}'
                return response
            left_s, right_s = (state['cur_state'] for state in states)
        except Exception as e:
            response.success = False
            response.message = f"Failed to read hardware state, cannot determine mode: {e}"
            return response

        if left_s == target_state and right_s == target_state:
            self._arm_state = target_enum
            response.success = True
            response.message = f"Already in {_NAMES[target_state]} mode (cur_state={target_state})"
            return response

        if left_s == 0 or right_s == 0:
            response.success = False
            response.message = (
                f"Arms not enabled (left state={left_s}, right state={right_s}); "
                f"call enable_all first"
            )
            return response

        try:
            if target_state == 1:
                self.controller.set_position_mode(
                    velRatio=30, AccRatio=30, should_abort=self._initialization_should_abort)
            else:
                self.controller.set_impedance_mode(
                    mode='joint', should_abort=self._initialization_should_abort)
        except Exception as e:
            self._on_sdk_error("set_arm_state", e)
            response.success = False
            response.message = f"Arm communication error: {e}"
            return response

        self._arm_state = target_enum
        self.get_logger().info(f"Arms switched to {_NAMES[target_state]} mode")
        response.success = True
        response.message = _NAMES[target_state]
        return response

    # -------------------- Control loop --------------------

    def _state_publish_loop(self):
        """500Hz: publish joint_states only."""
        if self._shutdown_requested.is_set() or not self._enable_lock.acquire(blocking=False):
            return
        try:
            self._publish_state()
        finally:
            self._enable_lock.release()

    def _control_loop(self):
        """120Hz: TF -> IK -> hardware. State publish is handled by a separate timer."""
        if (self._stop_requested.is_set() or self._shutdown_requested.is_set()
                or not self._enable_lock.acquire(blocking=False)):
            return
        try:
            if self._arm_enabled and not self._stop_requested.is_set():
                self._teleop_control()
        finally:
            self._enable_lock.release()

    def _teleop_control(self):
        """Look up TF -> local IK -> hardware."""
        # Query TF
        left_tf = self._lookup_transform("left_chest", "tianji_left")
        if left_tf is not None:
            self.left_pose = self._matrix_to_pose(left_tf)

        right_tf = self._lookup_transform("right_chest", "tianji_right")
        if right_tf is not None:
            self.right_pose = self._matrix_to_pose(right_tf)

        # Upper-arm direction
        left_arm_tf = self._lookup_transform("left_chest", "left_arm")
        if left_arm_tf is not None:
            self.left_y_axis = left_arm_tf[:3, 1]

        right_arm_tf = self._lookup_transform("right_chest", "right_arm")
        if right_arm_tf is not None:
            self.right_y_axis = right_arm_tf[:3, 1]

        if any(value is None for value in (left_tf, right_tf, left_arm_tf, right_arm_tf)):
            self.get_logger().error("Tracker pose stale or unavailable; stopping both arms")
            try:
                self._do_disable()
            except Exception as exc:
                self._on_sdk_error("tracker_timeout", exc)
            return

        # Log every 3 seconds.
        self._log_counter += 1
        if self._log_counter >= 300:
            self._log_counter = 0
            self.get_logger().info(
                f"TF: left={'OK' if self.left_pose is not None else 'None'}, "
                f"right={'OK' if self.right_pose is not None else 'None'}")

        # Run IK.
        if self.left_pose is not None or self.right_pose is not None:
            # Update null-space parameters.
            if self.left_y_axis is not None:
                self.controller.left_zsp_para = [*self.left_y_axis, 0, 0, 0]
            if self.right_y_axis is not None:
                self.controller.right_zsp_para = [*self.right_y_axis, 0, 0, 0]

            blend_s = self._handoff_blend_factor()  # None = ramp finished, otherwise 0..1
            try:
                # Always solve both sides before any write. One failed IK
                # must not leave the other arm following the operator alone.
                left_ok, right_ok, l_target, r_target = self.controller.move_to_pose_direct(
                    left_pose=self.left_pose, right_pose=self.right_pose, unit='m', send=False)
                if not (left_ok and right_ok):
                    raise RuntimeError('dual-arm IK incomplete; stopping both arms')
                l_joints = joint_vector(l_target, 'left IK target')
                r_joints = joint_vector(r_target, 'right IK target')
                if blend_s is not None:
                    l_joints = self._blend_joints(self._handoff_start_left, l_joints, blend_s)
                    r_joints = self._blend_joints(self._handoff_start_right, r_joints, blend_s)
                now = time.monotonic()
                dt = min(1.0 / self._control_rate_hz, max(0.0, now - self._last_command_at))
                max_step = self._max_joint_velocity * dt
                l_joints = limit_joint_step(self._last_command_left, l_joints, max_step)
                r_joints = limit_joint_step(self._last_command_right, r_joints, max_step)
                self.controller.move_to_joints_direct(left_joints=l_joints, right_joints=r_joints)
                self._last_command_left, self._last_command_right = l_joints, r_joints
                self._last_command_at = now
                self._publish_command(l_joints, r_joints)
            except Exception as e:
                self._on_sdk_error("teleop_control", e)
                return

        # Publish null-space params and end-effector poses (per-arm; no need for both online).
        self._publish_zsp_para_and_pose()

    def _handoff_blend_factor(self) -> Optional[float]:
        """Return the IK-weight for the handoff ramp, or None if the ramp is done.

        0.0 = full hold at snapshot; 1.0 = full live IK target. During the hold
        phase we return 0.0 (hardware tracks the snapshot); during the ramp we
        smoothstep from 0 to 1; after the ramp we clear state and return None
        so the steady-state fused path takes over.
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
        """Blend only complete measured snapshots and finite IK targets."""
        start = joint_vector(start, 'handoff start')
        target = joint_vector(target, 'handoff target')
        if not np.isfinite(s) or not 0.0 <= s <= 1.0:
            raise ValueError('handoff blend factor must be finite and within [0,1]')
        return [start[i] + s * (target[i] - start[i]) for i in range(7)]

    # -------------------- Publish --------------------

    def _publish_command(self, left: Optional[list], right: Optional[list]):
        stamp = self.get_clock().now().to_msg()
        if left is not None:
            msg = JointState()
            msg.header.stamp = stamp
            msg.header.frame_id = "left_base_cmd"
            msg.name = [f'left_joint_{i+1}' for i in range(7)]
            msg.position = list(left)
            self.left_cmd_pub.publish(msg)
        if right is not None:
            msg = JointState()
            msg.header.stamp = stamp
            msg.header.frame_id = "right_base_cmd"
            msg.name = [f'right_joint_{i+1}' for i in range(7)]
            msg.position = list(right)
            self.right_cmd_pub.publish(msg)

    def _publish_state(self):
        try:
            left_joints, right_joints = self.controller.get_current_joints()
            left_joints = joint_vector(left_joints, 'left published feedback')
            right_joints = joint_vector(right_joints, 'right published feedback')
            self._state_error_count = 0  # reset on success
            self._last_left_state_joints = list(left_joints) if left_joints is not None else None
            self._last_right_state_joints = list(right_joints) if right_joints is not None else None
            stamp = self.get_clock().now().to_msg()

            if left_joints is not None:
                msg = JointState()
                msg.header.stamp = stamp
                msg.header.frame_id = "left_base_state"
                msg.name = [f'left_joint_{i+1}' for i in range(7)]
                msg.position = list(left_joints)
                self.left_state_pub.publish(msg)

            if right_joints is not None:
                msg = JointState()
                msg.header.stamp = stamp
                msg.header.frame_id = "right_base_state"
                msg.name = [f'right_joint_{i+1}' for i in range(7)]
                msg.position = list(right_joints)
                self.right_state_pub.publish(msg)
        except Exception as e:
            self._state_error_count += 1
            # Only escalate to SDK fault while the arm is still enabled (avoid infinite loop after disable).
            if self._arm_enabled and self._state_error_count >= 50:
                self._on_sdk_error("publish_state", e)
                self._state_error_count = 0

    def _publish_zsp_para_and_pose(self):
        """Publish null-space parameters and end-effector poses."""
        try:
            # --- left_zsp_para ---
            # try/except guards against the controller being torn down mid-read.
            if hasattr(self.controller, 'left_zsp_para') and self.controller.left_zsp_para is not None:
                raw_data = self.controller.left_zsp_para
                if len(raw_data) > 0:
                    msg = Float64MultiArray()
                    # Coerce to list[float] for DDS compatibility.
                    msg.data = [float(x) for x in raw_data]
                    self.left_zsp_para_pub.publish(msg)

            # --- right_zsp_para ---
            if hasattr(self.controller, 'right_zsp_para') and self.controller.right_zsp_para is not None:
                raw_data = self.controller.right_zsp_para
                if len(raw_data) > 0:
                    msg = Float64MultiArray()
                    msg.data = [float(x) for x in raw_data]
                    self.right_zsp_para_pub.publish(msg)

            # --- left_pose ---
            if self.left_pose is not None:
                msg = Float64MultiArray()
                msg.data = [float(x) for x in self.left_pose]
                self.left_ee_pose_pub.publish(msg)

            # --- right_pose ---
            if self.right_pose is not None:
                msg = Float64MultiArray()
                msg.data = [float(x) for x in self.right_pose]
                self.right_ee_pose_pub.publish(msg)

        except Exception as e:
            # During shutdown, a freed C++ object can still appear briefly.
            # Log only; do not crash the node.
            self.get_logger().warn(f"Failed to publish debug info during shutdown: {e}")

    # -------------------- TF utilities --------------------

    def _spin_tf(self):
        try:
            self._tf_executor.spin()
        except (ExternalShutdownException, RCLError):
            pass

    def _initialization_should_abort(self):
        return (self._shutdown_requested.is_set()
                or self._stop_requested.is_set()
                or (self._enable_deadline is not None
                    and time.monotonic() >= self._enable_deadline)
                or not rclpy.ok(context=self.context) or not self._trackers_ready())

    def _trackers_ready(self) -> bool:
        return all(self._lookup_transform(parent, child) is not None for parent, child in (
            ("left_chest", "tianji_left"), ("right_chest", "tianji_right"),
            ("left_chest", "left_arm"), ("right_chest", "right_arm")))

    def _lookup_transform(self, from_frame: str, to_frame: str) -> Optional[np.ndarray]:
        try:
            tf = self.tf_buffer.lookup_transform(from_frame, to_frame, rclpy.time.Time())
            stamp = rclpy.time.Time.from_msg(tf.header.stamp)
            age = (self.get_clock().now() - stamp).nanoseconds / 1e9
            if not 0.0 <= age <= self._tracker_timeout:
                return None
            matrix = self._transform_to_matrix(tf)
            return matrix if np.isfinite(matrix).all() else None
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException, ValueError, TypeError):
            return None

    @staticmethod
    def _transform_to_matrix(transform) -> np.ndarray:
        t = transform.transform
        quat = [t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w]
        T = np.eye(4, dtype=np.float32)
        T[:3, :3] = R.from_quat(quat).as_matrix()
        T[:3, 3] = [t.translation.x, t.translation.y, t.translation.z]
        return T

    @staticmethod
    def _matrix_to_pose(matrix: np.ndarray) -> np.ndarray:
        """4x4 matrix -> [x, y, z, RX, RY, RZ] (degrees)."""
        xyz = matrix[:3, 3]
        rpy = R.from_matrix(matrix[:3, :3]).as_euler('ZYX', degrees=True)[::-1]
        return np.array([xyz[0], xyz[1], xyz[2], rpy[0], rpy[1], rpy[2]])

    def shutdown(self):
        self.get_logger().info("Shutting down...")
        self._shutdown_requested.set()
        self._stop_requested.set()
        try:
            if hasattr(self, 'controller'):
                with self._enable_lock:
                    self._arm_enabled = False
                    self.controller.disable_and_release()
        finally:
            if hasattr(self, '_tf_executor'):
                self._tf_executor.shutdown(timeout_sec=2.0)
                self._tf_thread.join(timeout=2.0)
                self._tf_executor.remove_node(self._tf_node)
                self.tf_listener.unregister()
                self._tf_node.destroy_node()
        self.get_logger().info("Exited cleanly")


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
    config_path = args.config or get_package_config_path("tianji_output", "tianji_chest.yaml")
    config = load_yaml_config(config_path)

    rclpy.init(args=raw_argv)
    node = None
    executor = None
    try:
        node = TianjiArmControllerNode(robot_ip=config.get("robot_ip", "192.168.1.190"))
        executor = MultiThreadedExecutor(num_threads=3, context=node.context)
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException, RCLError):
        # KeyboardInterrupt: direct SIGINT to a non-rclpy code path.
        # ExternalShutdownException: rclpy's documented signal-shutdown signal.
        # RCLError: same signal, race window — rclpy's SIGINT handler
        # invalidated the context mid-iteration and spin's next WaitSet
        # tripped on "context is not valid". All three mean "exit cleanly".
        pass
    finally:
        if node is not None:
            node._shutdown_requested.set()
            node._stop_requested.set()
        if executor is not None:
            # Let an active initialization observe cancellation and finish its
            # ordered rollback before releasing or destroying SDK/ROS objects.
            executor.shutdown()
        if node is not None:
            try:
                node.shutdown()
            except Exception as e:
                print(f"shutdown() raised: {e}", file=sys.stderr)
            try:
                node.destroy_node()
            except Exception:
                pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
