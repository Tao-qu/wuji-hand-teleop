"""Offline cabinet/arm checks: mocks only; never construct a robot or ROS node."""
from pathlib import Path
import sys
import threading
import time
from types import MethodType, SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/output_devices/tianji_output'))
sys.path.insert(0, str(ROOT / 'src/controller'))
pytest.importorskip('ament_index_python')
pytest.importorskip('rclpy')

from tianji_output.feedback_guard import (ArmJointLimits, DualArmFeedbackGuard, joint_vector,
                                         guarded_wait, limit_joint_step, pose_mm, disable_and_release)
from tianji_output.tool_parameters import resolve_dynamics, apply_tool_parameters
from tianji_output.tianji_chest_driver import TianjiChestDriver
from tianji_output.tianji_arm_controller import TianjiArmController
from controller.tianji_arm_node import TianjiArmControllerNode, LIFECYCLE_READY


def model_limits():
    return ArmJointLimits([[[170, -170, 180, 450]] * 7] * 2)


def snapshot(left_serial, right_serial, joints=None):
    return {'outputs': [{'frame_serial': serial, 'fb_joint_pos': [0.0] * 7 if joints is None else joints}
                        for serial in (left_serial, right_serial)],
            'inputs': [{'tool_dyn': [0.0] * 10}, {'tool_dyn': [0.0] * 10}]}


def test_default_zero_cache_and_one_live_arm_cannot_pass_feedback_guard():
    clock = [0.0]
    guard = DualArmFeedbackGuard(clock=lambda: clock[0])
    for frame in (snapshot(0, 0), snapshot(0, 0), snapshot(1, 0), snapshot(2, 0)):
        with pytest.raises(ValueError, match='advancing'):
            guard.observe(frame)
    # Once both frames advance, a legitimate measured zero pose is allowed.
    assert guard.observe(snapshot(3, 1)) == ([0.0] * 7, [0.0] * 7)
    clock[0] = 0.251
    with pytest.raises(ValueError, match='stale'):
        guard.observe(snapshot(4, 1))
    assert guard.observe(snapshot(5, 2)) == ([0.0] * 7, [0.0] * 7)
    with pytest.raises(ValueError, match='backwards'):
        guard.observe(snapshot(4, 3))


@pytest.mark.parametrize('bad', [None, [0] * 6, [[0] * 7], [float('nan')] * 7, [float('inf')] * 7])
def test_joint_feedback_requires_shape_and_finite_values(bad):
    with pytest.raises(ValueError):
        joint_vector(bad)
    guard = DualArmFeedbackGuard()
    with pytest.raises(ValueError):
        guard.observe(snapshot(1, 1, joints=bad) if bad is not None else {'outputs': [None, None]})


@pytest.mark.parametrize('inputs', [None, [], [{}], [{}, {}], [None, {}], [{'tool_dyn': [0] * 9}, {}]])
def test_missing_payload_is_value_error(inputs):
    with pytest.raises(ValueError):
        resolve_dynamics({}, inputs)


def test_payload_is_not_written_from_an_uninitialized_snapshot():
    robot = Mock()
    robot.subscribe.return_value = snapshot(0, 0)
    with patch('tianji_output.tool_parameters.time.monotonic', side_effect=[0.0, 2.1]):
        with pytest.raises(ValueError, match='advancing'):
            apply_tool_parameters(robot, {}, Mock())
    robot.clear_set.assert_not_called()
    robot.set_tool.assert_not_called()
    robot.send_cmd.assert_not_called()


def test_valid_zero_payload_is_preserved_only_after_both_frames_advance():
    robot = Mock()
    robot.subscribe.side_effect = [snapshot(10, 20), snapshot(11, 21)]
    with patch('tianji_output.tool_parameters.time.sleep'):
        values = apply_tool_parameters(robot, {}, Mock())
    assert values == {'left': [0.0] * 10, 'right': [0.0] * 10}
    assert robot.set_tool.call_count == 2


@pytest.mark.parametrize('bad', [None, [0] * 6, [float('nan')] * 7, [float('inf')] * 7])
def test_handoff_rejects_either_incomplete_or_nonfinite_arm(bad):
    for pair in ((bad, [0] * 7), ([0] * 7, bad)):
        node = SimpleNamespace(
            controller=Mock(), _handoff_hold_sec=0.5, _handoff_ramp_sec=1.0,
            get_logger=lambda: Mock())
        node.controller.get_current_joints.return_value = pair
        with pytest.raises(RuntimeError, match='measured handoff pose'):
            TianjiArmControllerNode._arm_handoff_snapshot(node)
        assert node._handoff_start_at is None
        assert node._handoff_start_left is None
        assert node._handoff_start_right is None


def test_disabling_ramp_does_not_bypass_snapshot_validation():
    node = SimpleNamespace(controller=Mock(), _handoff_hold_sec=0.0, _handoff_ramp_sec=0.0,
                           get_logger=lambda: Mock())
    node.controller.get_current_joints.return_value = (None, [0] * 7)
    with pytest.raises(RuntimeError):
        TianjiArmControllerNode._arm_handoff_snapshot(node)


def test_missing_handoff_start_never_jumps_directly_to_target():
    with pytest.raises(ValueError):
        TianjiArmControllerNode._blend_joints(None, [10] * 7, 0.0)
    assert TianjiArmControllerNode._blend_joints([2] * 7, [4] * 7, 0.25) == [2.5] * 7


def test_loss_of_any_tracker_stops_both_arms_before_ik():
    for missing in range(4):
        transforms = [np.eye(4) for _ in range(4)]
        transforms[missing] = None
        node = SimpleNamespace(
            _lookup_transform=Mock(side_effect=transforms),
            _matrix_to_pose=lambda _: np.zeros(6), get_logger=lambda: Mock(),
            _do_disable=Mock(), _on_sdk_error=Mock(), controller=Mock())
        TianjiArmControllerNode._teleop_control(node)
        node._do_disable.assert_called_once()
        node.controller.move_to_pose_direct.assert_not_called()


def test_initialization_abort_stops_without_marking_ready():
    controller = Mock()
    controller.move_to_init.side_effect = RuntimeError('tracker lost mid-motion')
    node = SimpleNamespace(
        _enable_lock=threading.Lock(), _arm_enabled=False, _trackers_ready=lambda: True,
        _enable_timeout=25.0,
        _publish_lifecycle=Mock(), controller=controller,
        _initialization_should_abort=lambda: False, get_logger=lambda: Mock(),
        _arm_handoff_snapshot=Mock())
    with pytest.raises(RuntimeError, match='tracker lost'):
        TianjiArmControllerNode._do_enable(node)
    assert not node._arm_enabled
    controller.set_standby.assert_called_once()
    node._arm_handoff_snapshot.assert_not_called()
    assert all(call.args != (LIFECYCLE_READY,) for call in node._publish_lifecycle.call_args_list)
    assert controller.move_to_init.call_args.kwargs['should_abort'] is node._initialization_should_abort


@pytest.mark.parametrize('driver_class', [TianjiChestDriver, TianjiArmController])
def test_smooth_motion_abort_prevents_further_send(driver_class):
    driver = SimpleNamespace(get_current_joints=Mock(return_value=([0] * 7, [0] * 7)),
                             robot=Mock(), logger=Mock(), _joint_limits=model_limits())
    checks = [False, False, True]
    with patch('time.sleep'):
        with pytest.raises(RuntimeError, match='cancelled'):
            driver_class.move_to_joints_smooth(
                driver, [1] * 7, [2] * 7, duration=1, dt=0.1,
                should_abort=lambda: checks.pop(0))
    assert driver.robot.send_cmd.call_count == 1


def test_settle_wait_checks_abort_instead_of_sleeping_full_timeout():
    now = [0.0]
    def sleep(amount): now[0] += amount
    with pytest.raises(RuntimeError, match='cancelled'):
        guarded_wait(15, should_abort=lambda: now[0] >= 0.03,
                     sleep=sleep, clock=lambda: now[0])
    assert now[0] <= 0.04


@pytest.mark.parametrize('driver_class', [TianjiChestDriver, TianjiArmController])
def test_direct_bad_target_is_rejected_before_any_robot_write(driver_class):
    driver = SimpleNamespace(robot=Mock(), logger=Mock(), _joint_limits=model_limits())
    with pytest.raises(ValueError):
        driver_class.move_to_joints_direct(driver, [0] * 7, [float('nan')] * 7)
    driver.robot.clear_set.assert_not_called()
    driver.robot.send_cmd.assert_not_called()


def test_peak_velocity_and_acceleration_limits_guard_initial_trajectory():
    limits = model_limits()
    start, target = [90, -90, -90, -90, 0, 0, 0], [55, -65, -70, -60, 60, 0, 0]
    assert limits.validate_quintic('left', start, target, 8.0) == (start, target)
    with pytest.raises(ValueError, match='velocity/acceleration'):
        limits.validate_quintic('left', start, target, 1.0)
    with pytest.raises(ValueError, match='model joint limits'):
        limits.validate('right', [180] * 7)


def test_tracker_or_nullspace_jump_is_slew_limited():
    # The observed fixed-TCP nullspace change was ~24.67 degrees; neither it
    # nor a crossing 170-degree target may become one full-size command.
    for target in ([24.67] * 7, [170] * 7, [-170] * 7):
        bounded = limit_joint_step([0] * 7, target, 30 / 120)
        assert max(abs(q) for q in bounded) == 0.25


def test_pose_metres_and_angle_degrees_are_kept_distinct():
    assert pose_mm([.5, -.2, .1, 90, 0, -45], 'm') == [500, -200, 100, 90, 0, -45]
    assert pose_mm([500, -200, 100, 90, 0, -45], 'mm') == [500, -200, 100, 90, 0, -45]
    with pytest.raises(ValueError):
        pose_mm([0] * 6, 'rad')


@pytest.mark.parametrize('driver_class', [TianjiChestDriver, TianjiArmController])
def test_one_side_ik_failure_never_sends_the_other_side(driver_class):
    kine = Mock()
    kine.ik.return_value = False
    driver = SimpleNamespace(
        get_current_joints=Mock(return_value=([0] * 7, [0] * 7)), kine_left=kine,
        kine_right=kine, zsp_type=0, left_zsp_para=[0] * 6, right_zsp_para=[0] * 6,
        zsp_angle=0, dgr=[5] * 3, _joint_limits=model_limits(), logger=Mock(),
        move_to_joints_direct=Mock(), robot=Mock())
    with pytest.raises(RuntimeError, match='IK failed'):
        driver_class.move_to_pose_direct(driver, [0] * 6, [0] * 6)
    driver.move_to_joints_direct.assert_not_called()
    driver.robot.send_cmd.assert_not_called()


def test_shutdown_still_releases_and_reports_disable_and_release_failures():
    robot = Mock()
    robot.send_cmd.side_effect = RuntimeError('link lost')
    robot.release_robot.side_effect = RuntimeError('release failed')
    with pytest.raises(RuntimeError, match='disable:.*release:'):
        disable_and_release(robot, DualArmFeedbackGuard())
    robot.release_robot.assert_called_once()


@pytest.mark.parametrize('driver_class', [TianjiChestDriver, TianjiArmController])
def test_driver_shutdown_propagates_failure(driver_class):
    driver = SimpleNamespace(robot=Mock(), logger=Mock(), _feedback_guard=DualArmFeedbackGuard())
    driver.robot.send_cmd.side_effect = RuntimeError('cannot disable')
    with pytest.raises(RuntimeError, match='disable:'):
        driver_class.disable_and_release(driver)
    driver.robot.release_robot.assert_called_once()


def cancellable_node(controller):
    """Bind production coordination methods without constructing a ROS node."""
    node = SimpleNamespace(
        controller=controller, _enable_lock=threading.RLock(),
        _request_lock=threading.Lock(), _disable_requests=0,
        _stop_requested=threading.Event(), _shutdown_requested=threading.Event(),
        _enable_timeout=25.0, _enable_deadline=None, _arm_enabled=False,
        context=object(), _trackers_ready=lambda: True,
        _publish_lifecycle=Mock(), get_logger=lambda: Mock(),
        _arm_handoff_snapshot=Mock(), _publish_state=Mock(), _teleop_control=Mock())
    for name in ('_do_enable', '_do_disable', '_initialization_should_abort',
                 '_set_enabled_callback', '_serialized_service',
                 '_state_publish_loop', '_control_loop'):
        setattr(node, name, MethodType(getattr(TianjiArmControllerNode, name), node))
    return node


@pytest.mark.parametrize('blocked_phase', ['position', 'motion', 'impedance'])
def test_disable_interrupts_initialization_without_concurrent_sdk_writes(blocked_phase):
    entered, abort_seen, release_owner = (threading.Event() for _ in range(3))
    sdk_calls = []
    controller = Mock()

    def phase(name):
        def execute(*args, should_abort, **kwargs):
            sdk_calls.append((name, threading.current_thread().name))
            if name == blocked_phase:
                entered.set()
                deadline = time.monotonic() + 2
                while not should_abort():
                    assert time.monotonic() < deadline, 'disable never reached the SDK owner'
                    time.sleep(.001)
                abort_seen.set()
                assert release_owner.wait(2), 'test did not release the SDK owner'
                raise RuntimeError('motion cancelled by Stop')
            return True
        return execute

    controller.set_position_mode.side_effect = phase('position')
    controller.move_to_init.side_effect = phase('motion')
    controller.set_impedance_mode.side_effect = phase('impedance')
    controller.set_standby.side_effect = lambda: sdk_calls.append(
        ('standby', threading.current_thread().name))
    node = cancellable_node(controller)
    results = {}
    def call(key, enabled):
        results[key] = node._set_enabled_callback(
            SimpleNamespace(data=enabled), SimpleNamespace(success=None, message=''))

    enable = threading.Thread(target=call, args=('enable', True), name='sdk-initializer')
    stop = threading.Thread(target=call, args=('stop', False), name='stop-request')
    with patch('controller.tianji_arm_node.rclpy.ok', return_value=True):
        enable.start()
        try:
            assert entered.wait(2)
            # A duplicate enable and unrelated SDK callbacks must not wait,
            # touch the hardware, or exhaust all executor threads.
            call('duplicate', True)
            assert not results['duplicate'].success
            other_operation = Mock()
            response = node._serialized_service(other_operation)(None, SimpleNamespace())
            assert not response.success
            other_operation.assert_not_called()
            node._state_publish_loop()
            node._control_loop()
            node._publish_state.assert_not_called()
            node._teleop_control.assert_not_called()
            stop.start()
            assert abort_seen.wait(2)
            assert node._disable_requests == 1
            assert 'stop' not in results  # waits for the original SDK owner
            assert not any(name == 'standby' for name, _ in sdk_calls)
            call('racing_enable', True)
            assert not results['racing_enable'].success
        finally:
            release_owner.set()
            enable.join(3)
            if stop.ident is not None:
                stop.join(3)
    assert not enable.is_alive() and not stop.is_alive()
    assert not results['enable'].success
    assert results['stop'].success
    assert sdk_calls[-2:] == [('standby', 'sdk-initializer'), ('standby', 'stop-request')]
    assert not node._arm_enabled and node._stop_requested.is_set()
    assert node._disable_requests == 0 and node._enable_deadline is None
    assert all(call.args != (LIFECYCLE_READY,) for call in node._publish_lifecycle.call_args_list)


def test_pending_stop_cannot_be_erased_by_a_later_enable():
    node = cancellable_node(Mock())
    node._disable_requests = 1
    node._stop_requested.set()
    response = node._set_enabled_callback(SimpleNamespace(data=True), SimpleNamespace())
    assert not response.success
    assert node._stop_requested.is_set()
    node.controller.set_position_mode.assert_not_called()


def test_failed_disable_reports_unconfirmed_and_keeps_stop_latched():
    node = cancellable_node(Mock())
    node.controller.set_standby.side_effect = RuntimeError('right state100 error6')
    response = node._set_enabled_callback(SimpleNamespace(data=False), SimpleNamespace())
    assert not response.success and 'unconfirmed' in response.message
    assert node._disable_requests == 0 and node._stop_requested.is_set()


def test_enable_deadline_cancels_even_with_fresh_trackers():
    node = cancellable_node(Mock())
    node._enable_deadline = time.monotonic() - .01
    with patch('controller.tianji_arm_node.rclpy.ok', return_value=True):
        assert node._initialization_should_abort()
    assert not node._stop_requested.is_set()


def test_stop_at_handoff_prevents_ready_and_startup_timer_cannot_undo_stop():
    node = cancellable_node(Mock())
    node._arm_handoff_snapshot.side_effect = node._stop_requested.set
    with patch('controller.tianji_arm_node.rclpy.ok', return_value=True):
        response = node._set_enabled_callback(SimpleNamespace(data=True), SimpleNamespace())
    assert not response.success and not node._arm_enabled
    node.controller.set_standby.assert_called_once()
    assert all(call.args != (LIFECYCLE_READY,) for call in node._publish_lifecycle.call_args_list)
    response = node._set_enabled_callback(SimpleNamespace(data=False), SimpleNamespace())
    assert response.success
    response = node._set_enabled_callback(
        SimpleNamespace(data=True), SimpleNamespace(), _automatic=True)
    assert not response.success and node._stop_requested.is_set()


def test_shutdown_waits_for_initialization_rollback_before_releasing_sdk():
    entered, abort_seen, release_owner = (threading.Event() for _ in range(3))
    calls = []
    controller = Mock()
    node = cancellable_node(controller)
    def initialize(*args, should_abort, **kwargs):
        entered.set()
        deadline = time.monotonic() + 2
        while not should_abort():
            assert time.monotonic() < deadline
            time.sleep(.001)
        abort_seen.set()
        assert release_owner.wait(2)
        raise RuntimeError('shutdown cancellation')
    controller.move_to_init.side_effect = initialize
    controller.set_standby.side_effect = lambda: calls.append('standby')
    controller.disable_and_release.side_effect = lambda: calls.append('release')
    enable = threading.Thread(target=node._set_enabled_callback,
        args=(SimpleNamespace(data=True), SimpleNamespace()))
    stop = threading.Thread(target=TianjiArmControllerNode.shutdown, args=(node,))
    with patch('controller.tianji_arm_node.rclpy.ok', return_value=True):
        enable.start()
        try:
            assert entered.wait(2)
            stop.start()
            assert abort_seen.wait(2)
            assert calls == []  # release cannot race the initializing SDK owner
        finally:
            release_owner.set()
            enable.join(3)
            if stop.ident is not None:
                stop.join(3)
    assert not enable.is_alive() and not stop.is_alive()
    assert calls == ['standby', 'release']


@pytest.mark.parametrize('reported_state', [100, 3])
def test_controller_fault_rejects_mode_even_when_servo_codes_are_zero(reported_state):
    driver = SimpleNamespace(robot=Mock(), _feedback_guard=Mock())
    driver.robot.subscribe.return_value = {'states': [
        {'cur_state': 3, 'err_code': 0}, {'cur_state': reported_state, 'err_code': 6}],
        'servo_errs': [[0] * 7, [0] * 7]}
    ok, reason = TianjiChestDriver._poll_arm_state_after_switch(driver, target_state=3)
    assert not ok and 'err_code=6' in reason
    assert driver.robot.subscribe.call_count == 1
    driver.robot.clear_error.assert_not_called()


def test_mode_poll_observes_stop_before_another_sdk_read():
    driver = SimpleNamespace(robot=Mock(), _feedback_guard=Mock())
    with pytest.raises(RuntimeError, match='cancelled'):
        TianjiChestDriver._poll_arm_state_after_switch(
            driver, target_state=3, should_abort=lambda: True)
    driver.robot.subscribe.assert_not_called()


@pytest.mark.parametrize('method', ['set_position_mode', 'set_impedance_mode'])
def test_mode_timeout_cannot_report_success(method):
    robot = Mock()
    for name in ('clear_set', 'set_joint_cmd_pose', 'set_state', 'set_vel_acc',
                 'set_joint_kd_params', 'set_impedance_type', 'send_cmd'):
        getattr(robot, name).return_value = True
    robot.subscribe.return_value = mode_snapshot(1)
    driver = mode_test_driver(robot, poll_result=(False, 'timeout'))
    with patch('tianji_output.tianji_chest_driver.guarded_wait'):
        with pytest.raises(RuntimeError, match='did not confirm'):
            getattr(TianjiChestDriver, method)(driver)


class ModeCommandSdk:
    """One serialized SDK stream, with an injected failed staged operation."""
    def __init__(self, fail_at=None, result=0):
        self.fail_at, self.result, self.calls = fail_at, result, []
        self.frame = 0
        self.snapshots = None

    def subscribe(self, _):
        self.frame += 1
        return self.snapshots.pop(0) if self.snapshots is not None else mode_snapshot(self.frame)

    def __getattr__(self, method):
        def call(**arguments):
            self.calls.append((method, arguments))
            return self.result if len(self.calls) == self.fail_at else True
        return call


def mode_snapshot(serial, right_state=1, right_error=0, positions=None):
    data = snapshot(serial, serial)
    data['states'] = [{'cur_state': 1, 'err_code': 0},
                      {'cur_state': right_state, 'err_code': right_error}]
    for output, joints in zip(data['outputs'], positions or ([10] * 7, [-10] * 7)):
        output['fb_joint_pos'] = joints
    return data


def mode_test_driver(robot, poll_result=(True, ''), clock=None):
    guard = DualArmFeedbackGuard(**({'clock': clock} if clock else {}))
    with pytest.raises(ValueError, match='advancing'):
        guard.observe(mode_snapshot(0))
    return SimpleNamespace(robot=robot, logger=Mock(), _feedback_guard=guard,
        get_current_joints=lambda: ([10] * 7, [-10] * 7),
        _poll_arm_state_after_switch=Mock(return_value=poll_result))


@pytest.mark.parametrize('method,mode,fail_at', [
    (method, mode, step)
    for method, mode, count in [('set_position_mode', None, 8),
                                ('set_impedance_mode', 'joint', 14),
                                ('set_impedance_mode', 'cart', 14)]
    for step in range(1, count + 1)
])
def test_each_rejected_mode_stage_stops_without_sending_later_steps(method, mode, fail_at):
    robot = ModeCommandSdk(fail_at)
    driver = mode_test_driver(robot)
    with patch('tianji_output.tianji_chest_driver.guarded_wait'):
        with pytest.raises(RuntimeError, match='was not accepted'):
            getattr(TianjiChestDriver, method)(driver, **({'mode': mode} if mode else {}))
    assert len(robot.calls) == fail_at
    driver._poll_arm_state_after_switch.assert_not_called()


@pytest.mark.parametrize('returned', [False, 0, None, -1, 2])
@pytest.mark.parametrize('phase,fail_at', [('position', 5), ('impedance', 5), ('impedance', 14)])
def test_unaccepted_mode_command_uses_existing_enable_rollback(returned, phase, fail_at):
    robot = ModeCommandSdk(fail_at=fail_at, result=returned)
    driver = mode_test_driver(robot)
    driver.set_position_mode = (MethodType(TianjiChestDriver.set_position_mode, driver)
                               if phase == 'position' else Mock())
    driver.set_impedance_mode = MethodType(TianjiChestDriver.set_impedance_mode, driver)
    driver.set_standby = Mock()
    driver.move_to_init = Mock(return_value=True)
    node = cancellable_node(driver)
    with patch('controller.tianji_arm_node.rclpy.ok', return_value=True), \
         patch('tianji_output.tianji_chest_driver.guarded_wait'):
        response = node._set_enabled_callback(SimpleNamespace(data=True), SimpleNamespace())
    assert not response.success and 'was not accepted' in response.message
    assert not node._arm_enabled
    driver.set_standby.assert_called_once()
    assert driver.move_to_init.call_count == (1 if phase == 'impedance' else 0)
    node._arm_handoff_snapshot.assert_not_called()
    assert all(call.args != (LIFECYCLE_READY,) for call in node._publish_lifecycle.call_args_list)


@pytest.mark.parametrize('method,mode,count,target', [
    ('set_position_mode', None, 8, 1),
    ('set_impedance_mode', 'joint', 14, 3),
    ('set_impedance_mode', 'cart', 14, 3),
])
def test_accepted_mode_batches_still_require_cabinet_confirmation(method, mode, count, target):
    robot = ModeCommandSdk()
    driver = mode_test_driver(robot)
    with patch('tianji_output.tianji_chest_driver.guarded_wait'):
        getattr(TianjiChestDriver, method)(driver, **({'mode': mode} if mode else {}))
    assert len(robot.calls) == count
    assert driver._poll_arm_state_after_switch.call_args.kwargs['target_state'] == target
    holds = [args['joints'] for name, args in robot.calls if name == 'set_joint_cmd_pose']
    assert holds == [[10] * 7, [-10] * 7]


@pytest.mark.parametrize('mode,kd_name,subtype', [('joint', 'set_joint_kd_params', 1),
                                               ('cart', 'set_cart_kd_params', 2)])
def test_impedance_stages_parameters_before_state_and_seeds_latest_measured_pose(mode, kd_name, subtype):
    robot = ModeCommandSdk()
    latest = ([11] * 7, [-12] * 7)
    robot.snapshots = [mode_snapshot(1), mode_snapshot(2, positions=latest)]
    driver = mode_test_driver(robot)
    with patch('tianji_output.tianji_chest_driver.guarded_wait') as wait:
        TianjiChestDriver.set_impedance_mode(driver, mode=mode)
    wait.assert_called_once_with(.5, None)
    first, second = robot.calls[:6], robot.calls[6:]
    assert [(name, args.get('arm')) for name, args in first] == [
        ('clear_set', None), (kd_name, 'A'), ('set_vel_acc', 'A'),
        (kd_name, 'B'), ('set_vel_acc', 'B'), ('send_cmd', None)]
    assert [(name, args.get('arm')) for name, args in second] == [
        ('clear_set', None), ('set_joint_cmd_pose', 'A'), ('set_joint_cmd_pose', 'B'),
        ('set_state', 'A'), ('set_impedance_type', 'A'),
        ('set_state', 'B'), ('set_impedance_type', 'B'), ('send_cmd', None)]
    assert [args['joints'] for name, args in second if name == 'set_joint_cmd_pose'] == list(latest)
    assert all(args['type'] == subtype for name, args in second if name == 'set_impedance_type')
    assert all(args['velRatio'] == args['AccRatio'] == 60 for name, args in first if name == 'set_vel_acc')


@pytest.mark.parametrize('argument', ['velRatio', 'AccRatio'])
@pytest.mark.parametrize('invalid', [0, -1, 101, True, 10.0, float('nan'), float('inf'), '10', None])
def test_impedance_rejects_invalid_ratios_before_any_device_access(argument, invalid):
    driver = SimpleNamespace(robot=Mock(), _feedback_guard=Mock(), logger=Mock())
    with pytest.raises(ValueError, match='integer percentage'):
        TianjiChestDriver.set_impedance_mode(driver, **{argument: invalid})
    assert driver.robot.mock_calls == []
    driver._feedback_guard.observe.assert_not_called()


@pytest.mark.parametrize('mode', ['joint', 'cart'])
def test_impedance_explicit_ratios_reach_both_arms_without_changing_mode_sequence(mode):
    robot = ModeCommandSdk()
    driver = mode_test_driver(robot)
    with patch('tianji_output.tianji_chest_driver.guarded_wait'):
        TianjiChestDriver.set_impedance_mode(driver, mode=mode, velRatio=10, AccRatio=8)
    ratios = [arguments for method, arguments in robot.calls if method == 'set_vel_acc']
    assert ratios == [{'arm': 'A', 'velRatio': 10, 'AccRatio': 8},
                      {'arm': 'B', 'velRatio': 10, 'AccRatio': 8}]
    assert len(robot.calls) == 14
    driver._poll_arm_state_after_switch.assert_called_once()


@pytest.mark.parametrize('when', ['before_parameters', 'before_enable'])
@pytest.mark.parametrize('state,error', [(100, 0), (100, 6), (1, 6), (101, 0)])
def test_impedance_snapshot_fault_or_transition_cannot_reach_enable(when, state, error):
    robot = ModeCommandSdk()
    bad = mode_snapshot(2, right_state=state, right_error=error)
    robot.snapshots = [bad] if when == 'before_parameters' else [mode_snapshot(1), bad]
    driver = mode_test_driver(robot)
    with patch('tianji_output.tianji_chest_driver.guarded_wait'):
        with pytest.raises(RuntimeError, match='fault-free non-transition'):
            TianjiChestDriver.set_impedance_mode(driver)
    assert len(robot.calls) == (0 if when == 'before_parameters' else 6)
    assert not any(name in ('set_state', 'set_joint_cmd_pose') for name, _ in robot.calls)
    driver._poll_arm_state_after_switch.assert_not_called()


def test_impedance_stale_feedback_after_parameter_wait_prevents_enable():
    now = [0.]
    robot = ModeCommandSdk()
    robot.snapshots = [mode_snapshot(1), mode_snapshot(1)]
    driver = mode_test_driver(robot, clock=lambda: now[0])
    def wait(duration, should_abort):
        now[0] += duration
    with patch('tianji_output.tianji_chest_driver.guarded_wait', side_effect=wait):
        with pytest.raises(ValueError, match='stale'):
            TianjiChestDriver.set_impedance_mode(driver)
    assert len(robot.calls) == 6
    driver._poll_arm_state_after_switch.assert_not_called()


def test_impedance_cancellation_between_batches_prevents_enable():
    stopped = [False]
    robot = ModeCommandSdk()
    driver = mode_test_driver(robot)
    with patch('tianji_output.tianji_chest_driver.guarded_wait',
               side_effect=lambda *_: stopped.__setitem__(0, True)):
        with pytest.raises(RuntimeError, match='cancelled'):
            TianjiChestDriver.set_impedance_mode(driver, should_abort=lambda: stopped[0])
    assert len(robot.calls) == 6
    driver._poll_arm_state_after_switch.assert_not_called()


def run_settle_feedback(feedback, timeout=15.0):
    """Drive the real settling logic with an offline clock and frame source."""
    now = [0.0]
    driver = SimpleNamespace(_feedback_guard=SimpleNamespace(serials=[0, 0], timeout=.25))
    def read():
        left, right, serials = feedback(now[0])
        driver._feedback_guard.serials = serials
        return left, right
    def wait(duration, should_abort):
        now[0] += duration
    driver.get_current_joints = read
    with patch('tianji_output.tianji_chest_driver.time.monotonic', side_effect=lambda: now[0]), \
         patch('tianji_output.tianji_chest_driver.guarded_wait', side_effect=wait):
        result = TianjiChestDriver._wait_for_init_settle(driver, [0] * 7, [0] * 7, timeout)
    return result, now[0]


def test_settle_returns_early_only_after_both_arms_stay_stable():
    def feedback(now):
        return [0] * 7, [0] * 7, [round(now * 100) + 1] * 2
    (settled, left, right), elapsed = run_settle_feedback(feedback)
    assert settled and left == right == [0] * 7
    assert .25 <= elapsed < .28  # does not sleep the entire 15-second budget


def test_settle_does_not_accept_outside_original_five_degree_tolerance():
    for error in (5.0, 6.0):
        result, elapsed = run_settle_feedback(
            lambda now: ([0] * 7, [error] * 7, [round(now * 100) + 1] * 2), timeout=.5)
        assert not result[0]
        assert elapsed == pytest.approx(.5)


def test_one_close_frame_or_moving_joints_cannot_finish_settle():
    def feedback(now):
        # Both sides stay within 5 degrees but are still moving at 4 deg/s.
        return [4 * now] * 7, [0] * 7, [round(now * 100) + 1] * 2
    result, elapsed = run_settle_feedback(feedback, timeout=.5)
    assert not result[0] and elapsed == pytest.approx(.5)


def test_repeated_cached_feedback_never_counts_as_settled():
    for frozen_side in (0, 1):
        def feedback(now):
            serials = [round(now * 100) + 1] * 2
            serials[frozen_side] = 1
            return [0] * 7, [0] * 7, serials
        result, elapsed = run_settle_feedback(feedback, timeout=.5)
        assert not result[0] and elapsed == pytest.approx(.5)


def test_position_drift_resets_settle_window_even_at_low_velocity():
    # 1.5 deg/s is below the velocity ceiling but moves >.25 degree per dwell.
    result, elapsed = run_settle_feedback(
        lambda now: ([1.5 * now] * 7, [0] * 7, [round(now * 100) + 1] * 2), timeout=.5)
    assert not result[0] and elapsed == pytest.approx(.5)


def test_unsettled_feedback_cannot_be_reported_as_success_by_move_to_init():
    driver = SimpleNamespace(_init_joints_left=[0] * 7, _init_joints_right=[0] * 7,
        logger=Mock(), move_to_joints_smooth=Mock(),
        _wait_for_init_settle=Mock(return_value=(False, [0] * 7, [0] * 7)))
    assert not TianjiChestDriver.move_to_init(driver, wait=True, timeout=15)


def run_clear_feedback(state, error, advancing=True, freeze_left=False):
    now = [0.0]
    counter = [0]
    robot = Mock()
    def read(_):
        counter[0] += 1
        serial = counter[0] if advancing else 984732
        result = snapshot(984732 if freeze_left else serial, serial)
        result['states'] = [{'cur_state': 0, 'err_code': 0},
                            {'cur_state': state, 'err_code': error}]
        return result
    robot.subscribe.side_effect = read
    driver = SimpleNamespace(robot=robot, logger=Mock())
    def sleep(duration):
        now[0] += duration
    with patch('tianji_output.tianji_chest_driver.time.monotonic', side_effect=lambda: now[0]), \
         patch('tianji_output.tianji_chest_driver.time.sleep', side_effect=sleep), \
         patch('tianji_output.tianji_chest_driver.DualArmFeedbackGuard',
               side_effect=lambda: DualArmFeedbackGuard(clock=lambda: now[0])):
        try:
            TianjiChestDriver.clear_arm_error(driver, 'B', timeout=.3)
            failure = None
        except RuntimeError as error:
            failure = error
    robot.clear_error.assert_called_once_with('B')
    robot.clear_set.assert_not_called()
    robot.send_cmd.assert_not_called()
    robot.set_state.assert_not_called()
    return driver, failure, now[0]


def test_clear_fault_requires_fresh_disabled_zero_error_feedback():
    driver, failure, elapsed = run_clear_feedback(0, 0)
    assert failure is None and .1 <= elapsed < .2
    driver.logger.info.assert_called_once()


@pytest.mark.parametrize('state,error', [(100, 0), (100, 6), (0, 6), (3, 0)])
def test_clear_fault_never_treats_error_state_or_enabled_state_as_success(state, error):
    driver, failure, elapsed = run_clear_feedback(state, error)
    assert failure is not None and 'unconfirmed' in str(failure)
    assert elapsed == pytest.approx(.3)
    driver.logger.info.assert_not_called()


@pytest.mark.parametrize('advancing,freeze_left', [(False, False), (True, True)])
def test_clear_fault_rejects_cached_or_one_sided_feedback(advancing, freeze_left):
    driver, failure, elapsed = run_clear_feedback(0, 0, advancing, freeze_left)
    assert failure is not None and 'advancing' in str(failure)
    driver.logger.info.assert_not_called()


@pytest.mark.parametrize('enabled,request_data', [(True, True), (False, False)])
def test_node_rejects_clear_while_enabled_or_without_explicit_request(enabled, request_data):
    node = SimpleNamespace(_arm_enabled=enabled, controller=Mock())
    response = TianjiArmControllerNode._clear_error_cb(
        node, 'B', SimpleNamespace(data=request_data), SimpleNamespace())
    assert not response.success
    node.controller.clear_arm_error.assert_not_called()


def test_node_clear_does_not_hide_unconfirmed_result():
    node = SimpleNamespace(_arm_enabled=False, controller=Mock())
    node.controller.clear_arm_error.side_effect = RuntimeError('right clear-error unconfirmed')
    response = TianjiArmControllerNode._clear_error_cb(
        node, 'B', SimpleNamespace(data=True), SimpleNamespace())
    assert not response.success and 'unconfirmed' in response.message


@pytest.mark.parametrize('state,error', [(100, 0), (100, 6), (3, 6)])
def test_manual_mode_switch_never_clears_or_bypasses_a_controller_fault(state, error):
    controller = Mock()
    controller.get_arm_controller_states.return_value = [
        {'cur_state': 3, 'err_code': 0}, {'cur_state': state, 'err_code': error}]
    node = SimpleNamespace(_arm_enabled=True, controller=controller)
    response = TianjiArmControllerNode._set_arm_state_callback(
        node, SimpleNamespace(data=False), SimpleNamespace())
    assert not response.success
    controller.set_impedance_mode.assert_not_called()
    controller.set_position_mode.assert_not_called()
    controller.robot.clear_error.assert_not_called()
    controller.robot.send_cmd.assert_not_called()


@pytest.mark.parametrize('state,error', [(100, 0), (100, 6), (0, 6)])
def test_legacy_set_active_rejects_faults_without_clearing_them(state, error):
    driver = SimpleNamespace(robot=Mock(), logger=Mock(), set_impedance_mode=Mock(),
        get_arm_controller_states=lambda: [
            {'cur_state': 0, 'err_code': 0}, {'cur_state': state, 'err_code': error}])
    with pytest.raises(RuntimeError, match='fault or transition'):
        TianjiChestDriver.set_active(driver)
    driver.robot.clear_error.assert_not_called()
    driver.robot.send_cmd.assert_not_called()
    driver.set_impedance_mode.assert_not_called()


def test_legacy_set_active_uses_faultfree_mode_switch_without_reset_batch():
    driver = SimpleNamespace(robot=Mock(), logger=Mock(), set_impedance_mode=Mock(),
        get_arm_controller_states=lambda: [{'cur_state': 0, 'err_code': 0}] * 2)
    TianjiChestDriver.set_active(driver)
    driver.set_impedance_mode.assert_called_once()
    driver.robot.clear_error.assert_not_called()
    driver.robot.clear_set.assert_not_called()
    driver.robot.send_cmd.assert_not_called()
