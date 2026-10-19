"""Actual production mode methods with a fake cabinet and clock; no native load."""
import ast
import copy
import ctypes
import json
from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
sys.path.insert(0, str(ROOT / 'src/output_devices/tianji_output'))

from verify_arm_mode import MODE_COMMANDS, JogJointLimits, ModeProbe, json_safe


class Clock:
    value = 0.0

    def monotonic(self):
        return self.value

    def time(self):
        return 1_000_000 + self.value

    def sleep(self, duration):
        self.value += duration


class FakeCabinet:
    """Only explicit staged fake writes update a simulated cabinet snapshot."""
    controller_version = 100343009

    def __init__(self, clock):
        self.clock = clock
        self.calls, self.batch = [], []
        self.serial = 50
        self.connected = self.released = False
        self.position_at = self.torque_at = None
        self.position_spike = self.torque_spike = self.torque_drift = False
        self.fault_on_torque = self.freeze = self.freeze_after_disable = False
        self.position_stuck = self.torque_stuck = self.disable_rejected = False
        self.disable_clear_rejected = False
        self.wrong_impedance_echo = False
        self.ratio_echo_override = None
        self.torque_profile = None
        self.kd_wait_spike = False
        self.kd_sent_at = None
        self.follow_jog = False
        self.jog_problem = None
        self.jog_has_moved = False
        self.fail_method = None
        self.on_read = lambda: None
        self.states = [{'cur_state': 0, 'err_code': 0} for _ in range(2)]
        self.inputs = [{'imp_type': 0, 'joint_cmd_pos': [sign * 55.] * 7,
                        'joint_vel_ratio': 0, 'joint_acc_ratio': 0,
                        'joint_k': [0.] * 7, 'joint_d': [0.] * 7,
                        'tool_dyn': [0.] * 10, 'tool_kine': [0.] * 6}
                       for sign in (1, -1)]

    def connect(self, ip):
        self.connected = True
        return True

    def SDK_version(self):
        return 100343014

    def release_robot(self):
        self.released = True
        return True

    def subscribe(self, unused):
        self.on_read()
        if not self.freeze:
            self.serial += 1
        outputs = [{'frame_serial': self.serial, 'fb_joint_pos': [sign * 55.] * 7,
                    'fb_joint_vel': [0.] * 7, 'tip_di': b'\x00'} for sign in (1, -1)]
        if self.follow_jog:
            for index, sign in enumerate((1, -1)):
                target = self.inputs[index]['joint_cmd_pos']
                delta = target[6] - sign * 55.
                if abs(delta) > .1:
                    self.jog_has_moved = True
                outputs[index]['fb_joint_pos'] = target[:]
                if self.jog_problem == 'stuck' and index == 1:
                    outputs[index]['fb_joint_pos'][6] = sign * 55.
                if self.jog_problem == 'wrong-direction' and index == 1:
                    outputs[index]['fb_joint_pos'][6] = sign * 55. - delta
                if self.jog_problem == 'other-drift' and abs(delta) > .05:
                    outputs[index]['fb_joint_pos'][0] += .11
                if self.jog_problem == 'return-error' and self.jog_has_moved and abs(delta) < .15:
                    outputs[index]['fb_joint_pos'][6] = sign * 55. + sign * .15
        if self.position_spike and self.position_at is not None and self.torque_at is None:
            if self.clock.monotonic() - self.position_at < .2:
                outputs[1]['fb_joint_vel'][0] = 2.08
        if (self.kd_wait_spike and self.kd_sent_at is not None and self.torque_at is None
                and any(state['cur_state'] == 1 for state in self.states)):
            outputs[0]['fb_joint_vel'][0] = 2.5
        if self.torque_at is not None and any(state['cur_state'] == 3 for state in self.states):
            if self.torque_spike:
                outputs[1]['fb_joint_vel'][0] = 2.08
            if self.torque_drift:
                outputs[1]['fb_joint_pos'][0] += 2.01
            if self.torque_profile is not None:
                speed, drift = self.torque_profile(self.clock.monotonic() - self.torque_at)
                outputs[0]['fb_joint_vel'][0] = speed
                outputs[0]['fb_joint_pos'][0] += drift
        return copy.deepcopy({'states': self.states, 'inputs': self.inputs, 'outputs': outputs})

    def __getattr__(self, method):
        if method not in MODE_COMMANDS:
            raise AssertionError(f'Unexpected SDK method: {method}')

        def command(**arguments):
            self.calls.append((method, copy.deepcopy(arguments)))
            if (self.jog_problem == 'partial-sdk-failure' and method == 'set_joint_cmd_pose'
                    and arguments['arm'] == 'B' and arguments['joints'][6] != -55.):
                return False
            if method == self.fail_method:
                return False
            if (self.disable_rejected and method == 'set_state' and arguments.get('state') == 0
                    and (self.disable_rejected is True or arguments.get('arm') == self.disable_rejected)):
                return False
            if self.disable_clear_rejected and method == 'clear_set' and self.torque_at is not None:
                return False
            if method == 'clear_set':
                self.batch = []
            elif method == 'send_cmd':
                for name, args in self.batch:
                    index = 0 if args['arm'] == 'A' else 1
                    if name == 'set_state':
                        state = args['state']
                        if state == 1:
                            self.position_at = self.clock.monotonic()
                            state = 101 if self.position_stuck else 1
                        if state == 3:
                            self.torque_at = self.clock.monotonic()
                            state = 103 if self.torque_stuck else 3
                        self.states[index]['cur_state'] = state
                        if state == 0 and self.freeze_after_disable:
                            self.freeze = True
                    elif name == 'set_joint_cmd_pose':
                        self.inputs[index]['joint_cmd_pos'] = args['joints'][:]
                    elif name == 'set_vel_acc':
                        self.inputs[index].update(
                            joint_vel_ratio=args['velRatio'] if self.ratio_echo_override is None else self.ratio_echo_override,
                            joint_acc_ratio=args['AccRatio'] if self.ratio_echo_override is None else self.ratio_echo_override)
                    elif name == 'set_joint_kd_params':
                        self.kd_sent_at = self.clock.monotonic()
                        self.inputs[index].update(joint_k=args['K'][:], joint_d=args['D'][:])
                    elif name == 'set_impedance_type':
                        self.inputs[index]['imp_type'] = 2 if self.wrong_impedance_echo else args['type']
                if self.fault_on_torque and self.torque_at is not None:
                    self.states[1] = {'cur_state': 100, 'err_code': 6}
                self.batch = []
            else:
                self.batch.append((method, copy.deepcopy(arguments)))
            return True
        return command


@pytest.fixture
def rig(monkeypatch):
    # Fail closed if any import or constructor accidentally loads native code.
    def no_native(*args, **kwargs):
        raise AssertionError('Offline mode-probe tests must never load a native SDK')
    monkeypatch.setattr(ctypes, 'CDLL', no_native)
    pytest.importorskip('ament_index_python')
    from tianji_output import tianji_chest_driver as chest
    from tianji_output.feedback_guard import DualArmFeedbackGuard, guarded_wait
    clock = Clock()
    monkeypatch.setattr(chest, 'time', clock)
    monkeypatch.setattr(chest, 'guarded_wait', lambda duration, should_abort=None:
                        guarded_wait(duration, should_abort, sleep=clock.sleep, clock=clock.monotonic))
    robot = FakeCabinet(clock)
    probe = ModeProbe(robot, DualArmFeedbackGuard(clock=clock.monotonic), object,
                      chest.TianjiChestDriver, clock)
    return probe, robot, clock


def idle_calls(robot):
    return [arguments for method, arguments in robot.calls
            if method == 'set_state' and arguments['state'] == 0]


def test_import_and_constructor_have_no_actuator_or_connection_calls(rig):
    probe, robot, _ = rig
    assert not robot.calls and not robot.connected and not robot.released
    tree = ast.parse((ROOT / 'tools/verify_arm_mode.py').read_text())
    # Hardware imports stay inside main, which is guarded by __name__.
    for node in tree.body:
        if isinstance(node, (ast.Import, ast.ImportFrom)):
            assert not isinstance(node, ast.ImportFrom) or not (node.module or '').startswith('tianji')


def test_default_readonly_never_stages_commands_or_disables(rig):
    probe, robot, clock = rig
    report = probe.run('fake-cabinet')
    assert report['success'] and robot.released
    assert robot.calls == [] and 3 <= clock.value < 3.1
    assert not report['standby_state_confirmed']
    assert report['sdk_version'] == 100343014
    assert report['controller_version'] == 100343009


def test_real_production_methods_hold_measured_pose_and_disable_once(rig):
    probe, robot, clock = rig
    report = probe.run('fake-cabinet', mode_hold=True)
    assert report['success'], report['errors']
    assert report['standby_state_confirmed'] and report['final_controller_errors'] == [0, 0]
    assert report['hold_observation_seconds'] >= 3 and clock.value < 15
    assert robot.released and idle_calls(robot) == [{'arm': 'A', 'state': 0}, {'arm': 'B', 'state': 0}]
    poses = [args for method, args in robot.calls if method == 'set_joint_cmd_pose']
    assert len(poses) == 4
    assert all(args['joints'] == [55. if args['arm'] == 'A' else -55.] * 7 for args in poses)
    for item in report['impedance_inputs_echo']:
        assert item['imp_type'] == 1
        assert item['joint_vel_ratio'] == item['joint_acc_ratio'] == pytest.approx(10, abs=1)
        assert item['joint_k'] == [2, 2, 2, 1.6, 1, 1, 1]
        assert item['joint_d'] == [.3, .3, .3, .2, .2, .2, .2]
        assert item['tool_dyn'] == [0.] * 10
    assert all(entry['method'] in MODE_COMMANDS for entry in report['commands'])
    ratios = [args for method, args in robot.calls if method == 'set_vel_acc']
    assert len(ratios) == 4 and all(args['velRatio'] == args['AccRatio'] == 10 for args in ratios)
    assert report['limits']['impedance_velocity_acceleration_ratio'] == 10
    assert 'bytes_hex' in json.dumps(json_safe(report), allow_nan=False)


def test_position_startup_speed_grace_requires_stable_window_before_torque(rig):
    probe, robot, _ = rig
    robot.position_spike = True
    report = probe.run('fake-cabinet', mode_hold=True)
    assert report['success'], report['errors']
    assert robot.torque_at - robot.position_at >= .2 + .25 + .5
    assert report['max_speed_deg_s'] == 2.08


@pytest.mark.parametrize('problem', ['torque_spike', 'torque_drift', 'fault_on_torque'])
def test_torque_has_no_grace_and_any_failure_disables_without_clear_or_retry(rig, problem):
    probe, robot, _ = rig
    setattr(robot, problem, True)
    report = probe.run('fake-cabinet', mode_hold=True)
    assert not report['success'] and report['failed_stage'] == 'impedance-entry'
    assert idle_calls(robot) == [{'arm': 'A', 'state': 0}, {'arm': 'B', 'state': 0}]
    assert len([1 for method, args in robot.calls if method == 'set_state' and args['state'] == 3]) == 2
    assert not any(method in ('clear_error', 'set_tool') for method, _ in robot.calls)
    assert robot.released
    if problem == 'fault_on_torque':
        assert not report['standby_state_confirmed']
        assert report['last_states'][1] == {'cur_state': 100, 'err_code': 6}


@pytest.mark.parametrize('state,error', [(100, 0), (100, 6), (1, 0)])
def test_nonzero_initial_state_or_fault_cannot_enable(rig, state, error):
    probe, robot, _ = rig
    robot.states[1] = {'cur_state': state, 'err_code': error}
    report = probe.run('fake-cabinet', mode_hold=True)
    assert not report['success'] and report['failed_stage'] == 'baseline'
    assert all(args['state'] == 0 for method, args in robot.calls if method == 'set_state')
    assert not any(method == 'set_joint_cmd_pose' for method, _ in robot.calls)


def test_never_advancing_feedback_cannot_enable_or_confirm_disable(rig):
    probe, robot, clock = rig
    robot.freeze = True
    report = probe.run('fake-cabinet', mode_hold=True)
    assert not report['success'] and not report['standby_state_confirmed']
    assert all(args['state'] == 0 for method, args in robot.calls if method == 'set_state')
    assert robot.released and clock.value < 15


def test_cached_zero_state_after_disable_is_not_confirmation(rig):
    probe, robot, _ = rig
    robot.freeze_after_disable = True
    report = probe.run('fake-cabinet', mode_hold=True)
    assert not report['success'] and not report['standby_state_confirmed']
    assert any('DISABLE UNCONFIRMED' in error for error in report['errors'])


@pytest.mark.parametrize('problem,stage', [('position_stuck', 'position-entry'),
                                         ('torque_stuck', 'impedance-entry')])
def test_actual_production_mode_timeouts_fail_and_disable(rig, problem, stage):
    probe, robot, clock = rig
    setattr(robot, problem, True)
    report = probe.run('fake-cabinet', mode_hold=True)
    assert not report['success'] and report['failed_stage'] == stage
    assert report['standby_state_confirmed'] and clock.value < 15
    assert len(idle_calls(robot)) == 2


def test_unaccepted_parameter_write_stops_before_torque_enable(rig):
    probe, robot, _ = rig
    robot.fail_method = 'set_joint_kd_params'
    report = probe.run('fake-cabinet', mode_hold=True)
    assert not report['success'] and report['failed_stage'] == 'impedance-entry'
    assert not any(method == 'set_state' and args['state'] == 3 for method, args in robot.calls)
    assert report['standby_state_confirmed'] and robot.released


def test_stop_request_during_position_prevents_torque_enable(rig):
    probe, robot, _ = rig
    robot.on_read = lambda: probe.request_stop() if robot.position_at is not None else None
    report = probe.run('fake-cabinet', mode_hold=True)
    assert not report['success']
    assert not any(method == 'set_state' and args['state'] == 3 for method, args in robot.calls)
    assert report['standby_state_confirmed'] and len(idle_calls(robot)) == 2


def test_rejected_disable_is_not_retried_or_reported_as_success(rig):
    probe, robot, _ = rig
    robot.disable_rejected = 'A'
    report = probe.run('fake-cabinet', mode_hold=True)
    assert not report['success'] and not report['standby_state_confirmed']
    assert len(idle_calls(robot)) == 2
    assert robot.states[0]['cur_state'] == 3 and robot.states[1]['cur_state'] == 0
    assert robot.calls[-1] == ('send_cmd', {})
    assert robot.released


def test_rejected_shutdown_clear_never_sends_unknown_buffer(rig):
    probe, robot, _ = rig
    robot.disable_clear_rejected = True
    report = probe.run('fake-cabinet', mode_hold=True)
    assert not report['success'] and not report['standby_state_confirmed']
    assert robot.calls[-1] == ('clear_set', {})
    assert idle_calls(robot) == []
    assert robot.released


def test_wrong_subtype_echo_stops_before_three_second_hold(rig):
    probe, robot, clock = rig
    robot.wrong_impedance_echo = True
    report = probe.run('fake-cabinet', mode_hold=True)
    assert not report['success'] and report['failed_stage'] == 'impedance-hold'
    assert 'hold_observation_seconds' not in report
    assert report['standby_state_confirmed'] and clock.value < 2


@pytest.mark.parametrize('echo,accepted', [(9, True), (10, True), (8, False), (11, False), (59, False)])
def test_ratio_echo_tolerates_only_known_downward_integer_quantization(rig, echo, accepted):
    probe, robot, _ = rig
    robot.ratio_echo_override = echo
    report = probe.run('fake-cabinet', mode_hold=True)
    assert report['success'] is accepted, report['errors']
    assert report['standby_state_confirmed']
    if not accepted:
        assert report['failed_stage'] == 'impedance-hold'
        assert 'hold_observation_seconds' not in report


def test_bounded_transition_is_opt_in_and_default_speed_limit_is_unchanged(rig):
    probe, robot, _ = rig
    robot.torque_profile = lambda age: (2.5, .1) if age < .1 else (0., .1)
    report = probe.run('fake-cabinet', mode_hold=True)
    assert not report['success'] and not report['bounded_transition']
    assert report['failed_stage'] == 'impedance-entry'
    rejected = next(sample for sample in report['samples'] if sample.get('rejection', '').startswith('Measured speed'))
    assert rejected['speed_limit_deg_s'] == 2 and not rejected['bounded_transition_active']
    assert report['max_drift_deg'] == pytest.approx(.1)
    assert report['standby_state_confirmed']


def test_bounded_transition_starts_at_state3_send_not_earlier_kd_batch(rig):
    probe, robot, _ = rig
    robot.torque_profile = lambda age: (2.5, .1) if age < .1 else (0., .1)
    report = probe.run('fake-cabinet', mode_hold=True, bounded_transition=True)
    assert report['success'], report['errors']
    sends = [entry for entry in report['commands']
             if entry['stage'] == 'impedance-entry' and entry['method'] == 'send_cmd']
    assert len(sends) == 2 and report['torque_send_at'] == sends[1]['at']
    assert report['torque_send_at'] - sends[0]['at'] >= .5
    assert report['torque_hold_joints_deg'] == [[55.] * 7, [-55.] * 7]
    transition = [sample for sample in report['samples'] if sample.get('bounded_transition_active')]
    assert transition and all(0 <= sample['seconds_since_torque_send'] < .15 for sample in transition)
    assert all(sample['speed_limit_deg_s'] == 3 for sample in transition)
    assert report['hold_observation_seconds'] >= 3 and report['standby_state_confirmed']


@pytest.mark.parametrize('speed,drift,message', [(3.1, .1, 'Measured speed'),
                                               (2.5, .16, 'Transition hold drift')])
def test_bounded_transition_keeps_both_extra_limits_and_records_rejected_drift(rig, speed, drift, message):
    probe, robot, _ = rig
    robot.torque_profile = lambda age: (speed, drift)
    report = probe.run('fake-cabinet', mode_hold=True, bounded_transition=True)
    assert not report['success'] and report['failed_stage'] == 'impedance-entry'
    rejected = next(sample for sample in report['samples'] if sample.get('rejection', '').startswith(message))
    assert rejected['bounded_transition_active']
    assert rejected['torque_hold_drift_deg'] == pytest.approx(drift)
    assert report['max_drift_deg'] == pytest.approx(drift)
    assert report['max_torque_hold_drift_deg'] == pytest.approx(drift)
    assert report['standby_state_confirmed'] and len(idle_calls(robot)) == 2


def test_bounded_transition_returns_to_strict_speed_after_window(rig):
    probe, robot, _ = rig
    robot.torque_profile = lambda age: (2.5, .1) if age >= .16 else (0., .1)
    report = probe.run('fake-cabinet', mode_hold=True, bounded_transition=True)
    assert not report['success'] and report['failed_stage'] == 'impedance-hold'
    rejected = next(sample for sample in report['samples'] if sample.get('rejection', '').startswith('Measured speed'))
    assert rejected['seconds_since_torque_send'] >= .15
    assert not rejected['bounded_transition_active'] and rejected['speed_limit_deg_s'] == 2
    assert report['standby_state_confirmed']


def test_bounded_transition_does_not_allow_excess_speed_during_kd_wait(rig):
    probe, robot, _ = rig
    robot.kd_wait_spike = True
    report = probe.run('fake-cabinet', mode_hold=True, bounded_transition=True)
    assert not report['success'] and report['failed_stage'] == 'impedance-entry'
    assert 'torque_send_at' not in report
    assert not any(method == 'set_state' and args['state'] == 3 for method, args in robot.calls)
    assert report['standby_state_confirmed']


def test_bounded_transition_cannot_implicitly_enable_a_readonly_probe(rig):
    probe, robot, _ = rig
    with pytest.raises(ValueError, match='explicit mode_hold'):
        probe.run('fake-cabinet', bounded_transition=True)
    assert not robot.connected and robot.calls == []


def test_position_jog_uses_actual_methods_fixed_signed_targets_and_bounded_rate(rig, monkeypatch):
    probe, robot, clock = rig
    robot.follow_jog = True
    actual_position = probe.driver_class.set_position_mode
    actual_direct = probe.driver_class.move_to_joints_direct
    invoked = []

    def position(owner, **kwargs):
        invoked.append('position')
        return actual_position(owner, **kwargs)

    def direct(owner, *targets):
        invoked.append('direct')
        return actual_direct(owner, *targets)

    monkeypatch.setattr(probe.driver_class, 'set_position_mode', position)
    monkeypatch.setattr(probe.driver_class, 'move_to_joints_direct', direct)
    report = probe.run('fake-cabinet', position_jog=True)
    assert report['success'], report['errors']
    assert report['mode'] == 'dual-arm-position-jog' and report['jog_completed']
    assert invoked[0] == 'position' and invoked.count('position') == 1
    assert invoked.count('direct') >= 200
    assert report['jog_outbound_signed_peak_deg'] == pytest.approx([.25, .25])
    assert report['jog_max_return_error_deg'] == 0
    assert report['jog_max_other_joint_drift_deg'] == 0
    assert report['standby_state_confirmed'] and clock.value < 15 and robot.released
    assert not any(method in ('set_joint_kd_params', 'set_impedance_type', 'set_tool', 'clear_error')
                   for method, _ in robot.calls)
    assert all(args['state'] in (0, 1) for method, args in robot.calls if method == 'set_state')
    sends = [entry for entry in report['commands']
             if entry['method'] == 'send_cmd' and entry['stage'].startswith('position-jog-')]
    assert all(b['at'] - a['at'] >= .02 - 1e-12 for a, b in zip(sends, sends[1:]))
    for entry in report['commands']:
        if entry['method'] != 'set_joint_cmd_pose' or not entry['stage'].startswith('position-jog-'):
            continue
        args = entry['arguments']
        sign = 1 if args['arm'] == 'A' else -1
        assert args['joints'][:6] == [sign * 55.] * 6
        assert -1e-9 <= (args['joints'][6] - sign * 55.) * sign <= .25 + 1e-9
    assert idle_calls(robot) == [{'arm': 'A', 'state': 0}, {'arm': 'B', 'state': 0}]


@pytest.mark.parametrize('problem,message', [
    ('stuck', 'must move at least 0.1'),
    ('wrong-direction', 'opposite the commanded jog direction'),
    ('other-drift', 'Non-jog joint drift'),
    ('return-error', 'return every joint within 0.1'),
    ('partial-sdk-failure', 'set_joint_cmd_pose was not accepted'),
])
def test_position_jog_requires_feedback_direction_return_and_native_acceptance(rig, problem, message):
    probe, robot, _ = rig
    robot.follow_jog, robot.jog_problem = True, problem
    report = probe.run('fake-cabinet', position_jog=True)
    assert not report['success']
    assert any(message in error for error in report['errors']), report['errors']
    assert report['standby_state_confirmed'] and robot.released
    assert len(idle_calls(robot)) == 2
    assert not report.get('jog_completed')
    assert not any(args['state'] == 3 for method, args in robot.calls if method == 'set_state')
    if problem == 'partial-sdk-failure':
        rejected = next(index for index, entry in enumerate(report['commands']) if entry.get('error'))
        # A partially staged jog is never sent. Shutdown first clears it and
        # then makes exactly one bilateral state0 request.
        after = report['commands'][rejected + 1:]
        assert after[0]['method'] == 'clear_set' and after[0]['stage'] == 'disable'
        assert [entry['method'] for entry in after] == ['clear_set', 'set_state', 'set_state', 'send_cmd']


def test_position_jog_cannot_be_combined_with_torque_or_its_transition_allowance(rig):
    probe, robot, _ = rig
    with pytest.raises(ValueError, match='mutually exclusive'):
        probe.run('fake-cabinet', position_jog=True, mode_hold=True)
    with pytest.raises(ValueError, match='explicit mode_hold'):
        probe.run('fake-cabinet', position_jog=True, bounded_transition=True)
    assert not robot.connected and robot.calls == []


@pytest.mark.parametrize('side,target', [
    ('left', [0.] * 6 + [-.01]),
    ('right', [0.] * 6 + [.01]),
    ('left', [0.] * 6 + [.251]),
    ('right', [0.] * 6 + [-.251]),
    ('left', [.001] + [0.] * 6),
    ('left', [0.] * 6 + [float('nan')]),
    ('left', [0.] * 6),
])
def test_jog_command_validator_rejects_any_unapproved_target(side, target):
    with pytest.raises(ValueError):
        JogJointLimits([[0.] * 7, [0.] * 7]).validate(side, target)


def test_jog_command_validator_rejects_j7_model_boundary():
    bounds = JogJointLimits([[0.] * 6 + [89.9], [0.] * 7])
    with pytest.raises(ValueError, match='signed/model bounds'):
        bounds.validate('left', [0.] * 6 + [90.15])
