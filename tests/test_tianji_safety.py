"""Offline cabinet/arm checks: mocks only; never construct a robot or ROS node."""
from pathlib import Path
import sys
import threading
from types import SimpleNamespace
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
        _publish_lifecycle=Mock(), controller=controller,
        _initialization_should_abort=lambda: True, get_logger=lambda: Mock(),
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
