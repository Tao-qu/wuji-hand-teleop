"""ROS transport for one Tianji dual-arm hardware connection.

Commands and feedback use radians. The hardware library and Home configuration
use degrees. A worker serializes SDK mutations while ROS subscriptions, state
publication, and service responses remain active during Home trajectories.
"""
from collections import deque
import argparse
import json
import math
import signal
import sys
import threading

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rclpy.task import Future
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import JointState
from std_msgs.msg import Int8, String
from std_srvs.srv import SetBool, Trigger

from .configuration import load_arm_config
from .tianji_chest_driver import TianjiChestDriver


INITIALIZING, ENABLING, READY, DISABLED, ENABLE_FAILED, SDK_ERROR = range(6)
JOINT_NAMES = {
    side: [f'{side}_joint_{i + 1}' for i in range(7)]
    for side in ('left', 'right')
}


class _LoggerAdapter:
    """Accept the hardware library's standard-logging format arguments."""

    def __init__(self, logger):
        self.logger = logger

    def __getattr__(self, name):
        method = getattr(self.logger, name)
        return lambda message, *args: method(message % args if args else message)


class TianjiDriverNode(Node):
    """Own the cabinet connection and accept named bilateral joint targets."""

    def __init__(self, *, config=None):
        super().__init__('driver', namespace='tianji_arm')
        config = load_arm_config() if config is None else config
        settings = config['controller']
        self.declare_parameter('auto_enable', True)
        self._home_duration = float(settings['home_duration_sec'])
        self._startup_wait = 3.0
        self._enabled = False
        self._stop = threading.Event()
        self._wake = threading.Event()
        self._work_lock = threading.Lock()
        self._jobs = deque()
        self._latest_command = None
        self._state_errors = 0
        self._pending_service_count = 0
        self._services_group = ReentrantCallbackGroup()
        self._worker = None
        self.driver = None

        state_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        latched_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._state_pubs = {
            side: self.create_publisher(JointState, f'/{side}_arm/joint_states', state_qos)
            for side in ('left', 'right')
        }
        self._command_pub = self.create_publisher(
            JointState, '/tianji_arm/commanded_joint_states', state_qos)
        self._lifecycle_pub = self.create_publisher(
            Int8, '/tianji_arm/lifecycle_state', latched_qos)
        self._home_pub = self.create_publisher(String, '/tianji_arm/home_state', latched_qos)
        self._publish_lifecycle(INITIALIZING)
        self.create_subscription(
            JointState, '/tianji_arm/joint_commands', self._on_command, state_qos)
        self.create_service(
            SetBool, '/tianji_arm/set_enabled', self._set_enabled,
            callback_group=self._services_group)
        self.create_service(
            SetBool, '/tianji_arm/set_arm_state', self._set_arm_state,
            callback_group=self._services_group)
        self.create_service(
            Trigger, '/tianji_arm/home', self._home,
            callback_group=self._services_group)
        self.create_service(
            Trigger, '/tianji_arm/arm_status', self._arm_status,
            callback_group=self._services_group)
        for side, arm in (('left', 'A'), ('right', 'B')):
            self.create_service(
                SetBool, f'/tianji_arm/{side}/brake', self._brake_callback(arm),
                callback_group=self._services_group)
            self.create_service(
                SetBool, f'/tianji_arm/{side}/clear_error', self._clear_callback(arm),
                callback_group=self._services_group)

        try:
            self.driver = TianjiChestDriver(
                config=config, logger=_LoggerAdapter(self.get_logger()),
                on_command=self._publish_submitted, stop_requested=self._stop.is_set)
        except Exception:
            if self.driver is not None:
                self.driver.disable_and_release()
            raise
        try:
            self.driver.set_standby()
        except Exception as exc:
            self.get_logger().error(f'Standby failed at startup: {exc}')
        self._publish_lifecycle(DISABLED)
        self.create_timer(1.0 / float(settings['state_publish_rate']), self._publish_state)
        self._worker = threading.Thread(target=self._work, name='tianji-sdk', daemon=False)
        self._worker.start()
        self._auto_enable_timer = None
        if self.get_parameter('auto_enable').value:
            self._auto_enable_timer = self.create_timer(0.1, self._auto_enable_once)

    def _auto_enable_once(self):
        self._auto_enable_timer.cancel()
        self.destroy_timer(self._auto_enable_timer)
        self._auto_enable_timer = None
        self._submit(self._enable, failure_state=ENABLE_FAILED)

    def _publish_lifecycle(self, state):
        self._lifecycle_pub.publish(Int8(data=state))

    def _submit(self, operation, *, failure_state=None):
        future = Future()
        with self._work_lock:
            if self._stop.is_set():
                future.set_result((False, 'Driver shutting down'))
                return future
            self._jobs.append((operation, future, failure_state))
        self._wake.set()
        return future

    async def _service_result(self, operation, *, failure_state=None):
        self._pending_service_count += 1
        try:
            return await self._submit(operation, failure_state=failure_state)
        finally:
            self._pending_service_count -= 1

    def _work(self):
        while not self._stop.is_set():
            self._wake.wait()
            self._wake.clear()
            while not self._stop.is_set():
                with self._work_lock:
                    job = self._jobs.popleft() if self._jobs else None
                    command = None
                    if job is None:
                        command, self._latest_command = self._latest_command, None
                if job is None and command is None:
                    break
                if job is not None:
                    operation, future, failure_state = job
                    if self._stop.is_set():
                        future.set_result((False, 'Driver shutting down'))
                        continue
                    try:
                        result = operation()
                        future.set_result((True, result or 'Complete'))
                    except Exception as exc:
                        self.get_logger().error(str(exc))
                        if failure_state is not None:
                            self._enabled = False
                            self._publish_lifecycle(failure_state)
                        future.set_result((False, str(exc)))
                elif self._enabled:
                    try:
                        self.driver.move_to_joints_direct(*command)
                    except Exception as exc:
                        self._sdk_error(exc)

    def _on_command(self, message):
        if not self._enabled:
            return
        try:
            if len(message.name) != len(message.position) or len(set(message.name)) != len(message.name):
                raise ValueError('Joint command names must match positions and be unique')
            values = dict(zip(message.name, message.position))
            if not values or set(values) - set(JOINT_NAMES['left'] + JOINT_NAMES['right']):
                raise ValueError('Joint command must contain named Tianji joints')
            command = []
            for side in ('left', 'right'):
                names = JOINT_NAMES[side]
                present = [name in values for name in names]
                if any(present) and not all(present):
                    raise ValueError(f'{side} command requires all seven joints')
                command.append([math.degrees(values[name]) for name in names] if all(present) else None)
        except (TypeError, ValueError) as exc:
            self.get_logger().error(f'Invalid joint command: {exc}')
            return
        with self._work_lock:
            self._latest_command = tuple(command)
        self._wake.set()

    def _publish_submitted(self, left, right):
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        for side, positions in (('left', left), ('right', right)):
            if positions is not None:
                message.name.extend(JOINT_NAMES[side])
                message.position.extend(math.radians(float(value)) for value in positions)
        self._command_pub.publish(message)

    def _publish_state(self):
        if self.driver is None or self._stop.is_set():
            return
        try:
            state = self.driver.get_full_state()
            stamp = self.get_clock().now().to_msg()
            for side in ('left', 'right'):
                sample = state[side]
                message = JointState()
                message.header.stamp = stamp
                message.header.frame_id = f'{side}_base'
                message.name = JOINT_NAMES[side]
                message.position = [math.radians(float(value)) for value in sample['joints']]
                message.velocity = [math.radians(float(value)) for value in sample['velocities']]
                message.effort = [float(value) for value in sample['torques']]
                self._state_pubs[side].publish(message)
            self._state_errors = 0
        except Exception as exc:
            self._state_errors += 1
            if self._enabled and self._state_errors >= 50:
                self._enabled = False
                self._state_errors = 0
                self._submit(lambda error=exc: self._sdk_error(error))

    def _discard_command(self):
        with self._work_lock:
            self._latest_command = None

    def _enable(self):
        if self._enabled:
            return 'Already enabled'
        self._publish_lifecycle(ENABLING)
        self.driver.set_position_mode(velRatio=100, AccRatio=100)
        self.get_logger().info(
            f'Moving arms Home over {self._home_duration:g} seconds; '
            f'following starts after {self._startup_wait:g} seconds of preparation')
        result = self.driver.move_to_init(
            wait=True, timeout=self._startup_wait, duration=self._home_duration)
        if result is False:
            raise RuntimeError('Arm Home operation reported failure')
        if self._stop.is_set():
            raise InterruptedError('Driver shutting down')
        self.driver.set_impedance_mode(mode='joint')
        self._discard_command()
        self._publish_state()
        self._enabled = True
        self._publish_lifecycle(READY)
        return 'Arms enabled, Home complete'

    def _disable(self):
        self._enabled = False
        self._discard_command()
        self._publish_lifecycle(DISABLED)
        self.driver.set_standby()
        return 'Arms stopped'

    def _sdk_error(self, error):
        self._enabled = False
        self._discard_command()
        self._publish_lifecycle(SDK_ERROR)
        self.get_logger().error(f'SDK exception: {error}')
        try:
            self.driver.set_standby()
        except Exception as exc:
            self.get_logger().error(f'Standby failed: {exc}')

    async def _set_enabled(self, request, response):
        def operation():
            if not request.data:
                return self._disable()
            try:
                left, right = self.driver.get_arm_states_only()
            except Exception:
                left, right = -1, -1
            if left == 3 and right == 3:
                self._publish_state()
                self._enabled = True
                self._publish_lifecycle(READY)
                return 'Already enabled (hardware in state=3)'
            return self._enable()
        response.success, response.message = await self._service_result(
            operation, failure_state=ENABLE_FAILED if request.data else SDK_ERROR)
        return response

    async def _home(self, _request, response):
        self._enabled = False
        self._discard_command()
        self._publish_lifecycle(ENABLING)
        self._home_pub.publish(String(data='MOVING'))

        def operation():
            self._enabled = False
            self._discard_command()
            self._publish_lifecycle(ENABLING)
            try:
                self.driver.set_position_mode(velRatio=100, AccRatio=100)
                if self.driver.move_to_init(wait=False, duration=self._home_duration) is False:
                    raise RuntimeError('Arm Home operation reported failure')
                self._publish_state()
                self._home_pub.publish(String(data='ACTIVE'))
                return 'Arm Home complete'
            except Exception:
                self._home_pub.publish(String(data='ERROR'))
                raise
        response.success, response.message = await self._service_result(
            operation, failure_state=ENABLE_FAILED)
        return response

    async def _set_arm_state(self, request, response):
        def operation():
            target = 1 if request.data else 3
            left, right = self.driver.get_arm_states_only()
            if left == target and right == target:
                return f'Already in state={target}'
            if left == 0 or right == 0:
                raise RuntimeError(f'Arms not enabled (left={left}, right={right})')
            try:
                self.driver.robot.clear_set()
                self.driver.robot.clear_error('A')
                self.driver.robot.clear_error('B')
                self.driver.robot.send_cmd()
                if self._stop.wait(0.5):
                    raise InterruptedError('Driver shutting down')
                if target == 1:
                    self.driver.set_position_mode(velRatio=30, AccRatio=30)
                else:
                    self.driver.set_impedance_mode(mode='joint')
            except Exception as exc:
                self._sdk_error(exc)
                raise
            return f'Arms switched to state={target}'
        response.success, response.message = await self._service_result(operation)
        return response

    def _brake_callback(self, arm):
        async def callback(request, response):
            operation = self.driver.release_brake if request.data else self.driver.hold_brake
            response.success, response.message = await self._service_result(lambda: operation(arm))
            return response
        return callback

    def _clear_callback(self, arm):
        async def callback(_request, response):
            response.success, response.message = await self._service_result(lambda: self.driver.clear_arm_error(arm))
            return response
        return callback

    def _arm_status(self, _request, response):
        if self._stop.is_set():
            response.success = False
            response.message = 'Driver shutting down'
            return response
        try:
            response.message = json.dumps(self.driver.get_arm_status())
            response.success = True
        except Exception as exc:
            response.message = str(exc)
            response.success = False
        return response

    def close(self):
        self._enabled = False
        self._stop.set()
        for timer in tuple(self.timers):
            timer.cancel()
        self._wake.set()
        if self._worker is not None:
            self._worker.join()
        with self._work_lock:
            queued, self._jobs = self._jobs, deque()
            self._latest_command = None
        for _operation, future, _failure_state in queued:
            future.set_result((False, 'Driver shutting down'))
        if self.driver is not None:
            self.driver.disable_and_release()


def main(args=None):
    raw_args = sys.argv if args is None else [sys.argv[0], *args]
    parser = argparse.ArgumentParser(description='Tianji dual-arm hardware driver')
    parser.add_argument('-c', '--config', help='Arm configuration path')
    parsed = parser.parse_args(remove_ros_args(raw_args)[1:])
    config = load_arm_config(parsed.config)
    rclpy.init(args=raw_args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    executor = SingleThreadedExecutor()
    stopping = False
    initializing = True

    def interrupt(_signum, _frame):
        nonlocal stopping
        if not stopping:
            stopping = True
            if not initializing:
                raise KeyboardInterrupt

    previous = {sig: signal.signal(sig, interrupt) for sig in (signal.SIGINT, signal.SIGTERM)}
    try:
        node = TianjiDriverNode(config=config)
        initializing = False
        if not stopping:
            executor.add_node(node)
            executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        stopping = True
        try:
            if node is not None:
                node.close()
                # Worker completions resume suspended service callbacks on this
                # executor. Finish their responses before destroying ROS handles.
                while node._pending_service_count and rclpy.ok():
                    executor.spin_once(timeout_sec=0.1)
                executor.remove_node(node)
                node.destroy_node()
            executor.shutdown(timeout_sec=0.0)
            if rclpy.ok():
                rclpy.shutdown()
        finally:
            for sig, handler in previous.items():
                signal.signal(sig, handler)
