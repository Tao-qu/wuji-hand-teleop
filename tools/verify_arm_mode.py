#!/usr/bin/env python3
"""Attended dual-arm probe; read-only unless an explicit motion option is used.

--mode-hold invokes the REAL production position and joint-impedance methods
without constructing their driver, loading IK, moving Home, using Trackers, or
writing tool parameters. Both modes request the vendor's commissioning ratios
10/10; joint K/D stay at production defaults. Both arms can enable.
Keep the operator at the E-stop.
--bounded-transition explicitly permits <=3 deg/s for only 0.15 s after the
torque-state batch, with an additional <=0.15 deg hold-relative drift bound.
The default torque speed limit remains 2 deg/s with no transition allowance.
--position-jog instead uses position mode only: left J7 +0.25 deg, right J7
-0.25 deg, with two-second quintic ramps out and back at no more than 50 Hz.
All other targets retain the settled measured pose. It does not test teleop.
Python deadlines cannot interrupt a native SDK call that does not return.
Importing this module and constructing ModeProbe never access a device.
"""
import argparse
import json
import logging
import math
import os
from pathlib import Path
import signal
import time
from types import MethodType, SimpleNamespace


ROOT = Path(__file__).resolve().parents[1]
MODE_COMMANDS = frozenset(('clear_set', 'set_joint_cmd_pose', 'set_state',
                           'set_vel_acc', 'set_joint_kd_params',
                           'set_impedance_type', 'send_cmd'))


def json_safe(value):
    if isinstance(value, bytes):
        return {'bytes_hex': value.hex()}
    if isinstance(value, dict):
        return {str(key): json_safe(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [json_safe(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return {'nonfinite': repr(value)}
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    return repr(value)


def seven(values, label):
    if (not isinstance(values, (list, tuple)) or len(values) != 7 or any(
            isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value) for value in values)):
        raise ValueError(f'{label} must contain seven finite numbers')
    return list(values)


class JogJointLimits:
    """Only the fixed probe trajectory; not a general arm/IK model validator.

    The checked-in ccs_m6.MvKDCfg gives J7 bounds of -90..90 degrees for
    both arms. All other targets must equal the captured measured hold.
    No native kinematics loader or driver constructor is needed.
    """
    def __init__(self, baseline):
        self.baseline = [seven(pose, 'jog hold') for pose in baseline]

    def validate(self, side, values):
        if side not in ('left', 'right'):
            raise ValueError('Unknown jog arm')
        index = 0 if side == 'left' else 1
        target = seven(values, f'{side} jog target')
        start = self.baseline[index]
        if any(value != hold for value, hold in zip(target[:6], start[:6])):
            raise ValueError('Position jog must hold J1-J6 exactly')
        signed_delta = (target[6] - start[6]) * (1 if index == 0 else -1)
        if not -1e-9 <= signed_delta <= .25 + 1e-9 or not -90 <= target[6] <= 90:
            raise ValueError('Position jog J7 target exceeds its signed/model bounds')
        return target


class ModeProbe:
    """One SDK owner; production calls and all monitoring run on one thread."""
    def __init__(self, robot, guard, snapshot_factory, driver_class, clock=time):
        self.robot, self.guard, self.snapshot_factory = robot, guard, snapshot_factory
        self.driver_class, self.clock = driver_class, clock
        self.stage = 'created'
        self.mode_hold = self.stopping = self.stop_requested = False
        self.position_jog = self.actuating = False
        self.jog_baseline = self.jog_last_sent_at = None
        self.baseline = self.deadline = self.action_deadline = None
        self.position_sent_at = None
        self.bounded_transition = False
        self.torque_sent_at = self.torque_hold = None
        self._staged_torque_arms = set()
        self._staged_torque_hold = [None, None]
        self.data = {
            'started_unix': clock.time(), 'samples': [], 'commands': [], 'errors': [],
            'standby_state_confirmed': False, 'physical_disable_claimed': False,
            'sdk_library_override': os.environ.get('TIANJI_SDK_LIBRARY'),
            'limits': {'drift_deg': 2.0, 'speed_deg_s': 2.0, 'stable_seconds': .25,
                       'stable_span_deg': .25, 'hold_seconds': 3.0,
                       'position_startup_speed_grace_seconds': .5,
                       'bounded_transition_seconds': .15,
                       'bounded_transition_speed_deg_s': 3.0,
                       'bounded_transition_hold_drift_deg': .15,
                       'total_python_seconds_after_connect': 15.0,
                       'position_velocity_acceleration_ratio': 10,
                       'impedance_velocity_acceleration_ratio': 10},
        }

    def __getattr__(self, method):
        if method not in MODE_COMMANDS:
            raise AttributeError(method)
        return lambda **arguments: self.command(method, **arguments)

    def request_stop(self, signum=None, frame=None):
        # Signal handlers only set a flag; never call the SDK concurrently.
        self.stop_requested = True
        self.data['stop_signal'] = signum

    def subscribe(self, dcss):
        snapshot = self.robot.subscribe(dcss)
        entry = {'at': self.clock.monotonic(), 'stage': self.stage,
                 'snapshot': json_safe(snapshot)}
        self.data['samples'].append(entry)
        try:
            self.guard.observe(snapshot)
            states = snapshot.get('states')
            if (not isinstance(states, (tuple, list)) or len(states) != 2 or any(
                    not isinstance(state, dict) or type(state.get('cur_state')) is not int
                    or type(state.get('err_code')) is not int for state in states)):
                raise ValueError('Invalid bilateral controller states')
            velocities = [seven(output.get('fb_joint_vel'), 'joint velocity')
                          for output in snapshot['outputs'][:2]]
            self.data['last_states'] = json_safe(states)
            entry['fresh'] = True
            speed = max(abs(value) for arm in velocities for value in arm)
            entry['measured_max_speed_deg_s'] = speed
            drift = hold_drift = None
            transition_elapsed = (None if self.torque_sent_at is None
                                  else entry['at'] - self.torque_sent_at)
            transition_active = (self.bounded_transition and not self.stopping
                                 and self.stage in ('impedance-entry', 'impedance-hold')
                                 and transition_elapsed is not None
                                 and 0 <= transition_elapsed < .15)
            entry['bounded_transition_active'] = transition_active
            entry['seconds_since_torque_send'] = transition_elapsed
            entry['speed_limit_deg_s'] = 3.0 if transition_active else 2.0
            if self.actuating:
                # Account for the rejecting sample and shutdown observations
                # before any speed/fault/drift check can raise.
                self.data['max_speed_deg_s'] = max(self.data.get('max_speed_deg_s', 0.), speed)
                if self.baseline is not None:
                    drift = max(abs(value - start) for output, starts in
                                zip(snapshot['outputs'], self.baseline)
                                for value, start in zip(output['fb_joint_pos'], starts))
                    entry['baseline_drift_deg'] = drift
                    self.data['max_drift_deg'] = max(self.data.get('max_drift_deg', 0.), drift)
                if self.torque_hold is not None:
                    hold_drift = max(abs(value - start) for output, starts in
                                     zip(snapshot['outputs'], self.torque_hold)
                                     for value, start in zip(output['fb_joint_pos'], starts))
                    entry['torque_hold_drift_deg'] = hold_drift
                    self.data['max_torque_hold_drift_deg'] = max(
                        self.data.get('max_torque_hold_drift_deg', 0.), hold_drift)
            jog_deltas = held_drift = None
            if self.jog_baseline is not None:
                deltas = [[value - start for value, start in zip(output['fb_joint_pos'], starts)]
                          for output, starts in zip(snapshot['outputs'][:2], self.jog_baseline)]
                jog_deltas = [deltas[0][6], -deltas[1][6]]
                held_drift = max(abs(value) for arm in deltas for value in arm[:6])
                entry['jog_signed_j7_delta_deg'] = jog_deltas
                entry['jog_other_joint_drift_deg'] = held_drift
                self.data['jog_max_other_joint_drift_deg'] = max(
                    self.data.get('jog_max_other_joint_drift_deg', 0.), held_drift)
                if self.stage == 'position-jog-out':
                    old = self.data.get('jog_outbound_signed_peak_deg', [0., 0.])
                    self.data['jog_outbound_signed_peak_deg'] = [max(a, b) for a, b in zip(old, jog_deltas)]
            if self.actuating and not self.stopping:
                if any(state['err_code'] != 0 for state in states):
                    raise RuntimeError(f'Controller fault; never clear automatically: {states}')
                permitted = {
                    'baseline': (0,), 'position-entry': (0, 101, 1),
                    'position-settle': (1,), 'impedance-entry': (1, 103, 3),
                    'impedance-hold': (3,), 'position-jog-out': (1,),
                    'position-jog-back': (1,), 'position-jog-settle': (1,),
                }[self.stage]
                if any(state['cur_state'] not in permitted for state in states):
                    raise RuntimeError(f'Unexpected controller state in {self.stage}: {states}')
                position_grace = (self.stage in ('position-entry', 'position-settle')
                                  and self.position_sent_at is not None
                                  and self.clock.monotonic() - self.position_sent_at < .5)
                entry['position_speed_grace'] = position_grace
                if drift is not None and drift > 2.0:
                    raise RuntimeError(f'Measured drift {drift:.6f} exceeds 2 degrees')
                if transition_active and (hold_drift is None or hold_drift > .15):
                    raise RuntimeError(f'Transition hold drift {hold_drift!r} exceeds 0.15 degrees')
                limit = entry['speed_limit_deg_s']
                if speed > limit and not position_grace:
                    raise RuntimeError(f'Measured speed {speed:.6f} exceeds {limit:g} degrees/s')
                if jog_deltas is not None:
                    if held_drift > .1:
                        raise RuntimeError(f'Non-jog joint drift {held_drift:.6f} exceeds 0.1 degrees')
                    # Ignore tiny encoder/holding variation, but reject a
                    # measurable response opposite to the commanded direction.
                    if min(jog_deltas) < -.025:
                        raise RuntimeError(f'J7 moved opposite the commanded jog direction: {jog_deltas}')
            return snapshot
        except (ValueError, RuntimeError) as error:
            entry['rejection'] = str(error)
            entry.setdefault('fresh', False)
            raise

    def read(self):
        return self.subscribe(self.snapshot_factory())

    def current_joints(self):
        snapshot = self.read()
        return tuple(seven(output['fb_joint_pos'], 'joint position')
                     for output in snapshot['outputs'][:2])

    def should_abort(self):
        if self.stop_requested:
            raise RuntimeError('Operator requested stop')
        if self.clock.monotonic() >= self.action_deadline:
            raise RuntimeError('Probe action deadline reached')
        self.read()
        if self.stop_requested or self.clock.monotonic() >= self.action_deadline:
            raise RuntimeError('Stop or action deadline reached during feedback read')
        # A synchronous monitor exception uses the production rollback path.
        return False

    def command(self, method, **arguments):
        if not self.actuating:
            raise RuntimeError('Read-only probe cannot stage actuator commands')
        if not self.stopping:
            self.should_abort()
        elif method not in ('clear_set', 'set_state', 'send_cmd') or (
                method == 'set_state' and arguments.get('state') != 0):
            raise RuntimeError('Shutdown permits only one idle batch')
        entry = {'at': self.clock.monotonic(), 'stage': self.stage,
                 'method': method, 'arguments': json_safe(arguments)}
        self.data['commands'].append(entry)
        try:
            torque_send = (self.stage == 'impedance-entry' and method == 'send_cmd'
                           and bool(self._staged_torque_arms))
            if torque_send and self.torque_sent_at is not None:
                raise RuntimeError('Torque entry is once only; never restart its transition window')
            if torque_send and (self._staged_torque_arms != {'A', 'B'}
                                or any(pose is None for pose in self._staged_torque_hold)):
                raise RuntimeError('Torque batch requires both measured holds and both state3 commands')
            result = getattr(self.robot, method)(**arguments)
            entry['result'] = json_safe(result)
            entry['returned_at'] = self.clock.monotonic()
            if result is not True and not (type(result) is int and result == 1):
                raise RuntimeError(f'{method} was not accepted: {result!r}')
            if method == 'send_cmd' and self.stage == 'position-entry':
                self.position_sent_at = self.clock.monotonic()
            if method == 'send_cmd' and self.stage.startswith('position-jog-'):
                self.jog_last_sent_at = entry['at']
            if self.stage == 'impedance-entry':
                if method == 'clear_set':
                    self._staged_torque_arms = set()
                    self._staged_torque_hold = [None, None]
                elif method == 'set_joint_cmd_pose':
                    index = 0 if arguments['arm'] == 'A' else 1
                    self._staged_torque_hold[index] = seven(arguments['joints'], 'torque hold')
                elif method == 'set_state' and arguments.get('state') == 3:
                    self._staged_torque_arms.add(arguments['arm'])
                elif torque_send:
                    # Start from send invocation, not its return: a slow SDK
                    # call must not extend the window after physical activation.
                    self.torque_sent_at = entry['at']
                    self.torque_hold = [pose[:] for pose in self._staged_torque_hold]
                    self.data.update(torque_send_at=self.torque_sent_at,
                                     torque_send_returned_at=entry['returned_at'],
                                     torque_hold_joints_deg=self.torque_hold)
            return result
        except BaseException as error:
            entry['error'] = repr(error)
            raise

    def wait_stable(self, state, until, *, prime=False, duration=.25):
        stable_at = minima = maxima = previous = None
        while self.clock.monotonic() < until:
            if not self.stopping and self.stop_requested:
                raise RuntimeError('Operator requested stop')
            try:
                snapshot = self.read()
            except ValueError as error:
                if not self.stopping and (not prime or str(error) !=
                        'waiting for advancing feedback frames from both arms'):
                    raise
                stable_at = minima = maxima = previous = None
                self.clock.sleep(.01)
                continue
            states = snapshot['states']
            if self.clock.monotonic() >= until:
                break
            speed = max(abs(value) for output in snapshot['outputs'][:2]
                        for value in output['fb_joint_vel'])
            if any(item['cur_state'] != state for item in states) or speed > 2.0:
                stable_at = minima = maxima = previous = None
            else:
                serials = tuple(self.guard.serials)
                if previous is None or all(a != b for a, b in zip(previous, serials)):
                    previous = serials
                    positions = [value for output in snapshot['outputs'][:2]
                                 for value in output['fb_joint_pos']]
                    now = self.clock.monotonic()
                    if stable_at is None:
                        stable_at, minima, maxima = now, positions[:], positions[:]
                    else:
                        minima = [min(a, b) for a, b in zip(minima, positions)]
                        maxima = [max(a, b) for a, b in zip(maxima, positions)]
                        if max(b - a for a, b in zip(minima, maxima)) > .25:
                            stable_at, minima, maxima = now, positions[:], positions[:]
                        elif now - stable_at >= duration:
                            return snapshot
            self.clock.sleep(.01)
        raise RuntimeError(f'Fresh stable bilateral state={state} unconfirmed before deadline')

    def mode_sequence(self):
        self.stage = 'baseline'
        snapshot = self.wait_stable(0, min(self.action_deadline, self.clock.monotonic() + 2.), prime=True)
        self.baseline = [list(output['fb_joint_pos']) for output in snapshot['outputs'][:2]]
        self.data['baseline_joints_deg'] = self.baseline
        self.data['baseline_inputs'] = json_safe(snapshot.get('inputs'))
        owner = SimpleNamespace(robot=self, _feedback_guard=self.guard,
                                logger=logging.getLogger('verify-arm-mode'),
                                get_current_joints=self.current_joints)
        owner._poll_arm_state_after_switch = MethodType(
            self.driver_class._poll_arm_state_after_switch, owner)
        self.stage = 'position-entry'
        self.driver_class.set_position_mode(owner, velRatio=10, AccRatio=10,
                                            should_abort=self.should_abort)
        self.stage = 'position-settle'
        snapshot = self.wait_stable(1, min(self.action_deadline, self.clock.monotonic() + 2.))
        self.data['position_settled_inputs'] = json_safe(snapshot.get('inputs'))
        if self.position_jog:
            self.jog_sequence(owner, snapshot)
            return
        self.stage = 'impedance-entry'
        self.driver_class.set_impedance_mode(owner, mode='joint', should_abort=self.should_abort,
                                             velRatio=10, AccRatio=10)
        self.stage = 'impedance-hold'
        self.check_impedance_echo(self.read())
        active_at = self.clock.monotonic()
        if active_at + 3.0 > self.action_deadline:
            raise RuntimeError('Insufficient remaining deadline for full 3-second hold')
        while self.clock.monotonic() < active_at + 3.0:
            self.should_abort()
            self.clock.sleep(.01)
        snapshot = self.read()
        if self.stop_requested or self.clock.monotonic() >= self.action_deadline:
            raise RuntimeError('Stop or action deadline reached at hold completion')
        self.check_impedance_echo(snapshot)
        self.data['hold_observation_seconds'] = self.clock.monotonic() - active_at

    def jog_sequence(self, owner, snapshot):
        """Actual production direct commands with a fixed measured-pose envelope."""
        self.jog_baseline = [seven(output['fb_joint_pos'], 'settled jog hold')
                             for output in snapshot['outputs'][:2]]
        self.data['jog_baseline_joints_deg'] = self.jog_baseline
        owner._joint_limits = JogJointLimits(self.jog_baseline)
        ends = [pose[:] for pose in self.jog_baseline]
        ends[0][6] += .25
        ends[1][6] -= .25
        for side, start, end in zip(('left', 'right'), self.jog_baseline, ends):
            owner._joint_limits.validate(side, start)
            owner._joint_limits.validate(side, end)
        self.check_ratio_echo(snapshot, expected=10)
        if self.clock.monotonic() + 5.0 > self.action_deadline:
            raise RuntimeError('Insufficient deadline for full position jog and return verification')
        for outward in (True, False):
            self.stage = 'position-jog-out' if outward else 'position-jog-back'
            phase_started = self.clock.monotonic()
            while True:
                if self.jog_last_sent_at is not None:
                    delay = self.jog_last_sent_at + .02 - self.clock.monotonic()
                    if delay > 0:
                        self.clock.sleep(delay)
                self.should_abort()
                fraction = min(1., (self.clock.monotonic() - phase_started) / 2.)
                blend = fraction**3 * (10. - 15. * fraction + 6. * fraction**2)
                amplitude = blend if outward else 1. - blend
                targets = [pose[:] for pose in self.jog_baseline]
                targets[0][6] += .25 * amplitude
                targets[1][6] -= .25 * amplitude
                self.driver_class.move_to_joints_direct(owner, *targets)
                self.read()
                if fraction >= 1.:
                    break
            if outward and min(self.data.get('jog_outbound_signed_peak_deg', [0., 0.])) < .1:
                raise RuntimeError('Both J7 joints must move at least 0.1 degrees in the commanded directions')
        self.stage = 'position-jog-settle'
        snapshot = self.wait_stable(1, min(self.action_deadline, self.clock.monotonic() + 1.))
        errors = [[abs(value - start) for value, start in zip(output['fb_joint_pos'], starts)]
                  for output, starts in zip(snapshot['outputs'][:2], self.jog_baseline)]
        self.data['jog_return_errors_deg'] = errors
        self.data['jog_max_return_error_deg'] = max(value for arm in errors for value in arm)
        if self.data['jog_max_return_error_deg'] > .1:
            raise RuntimeError('Position jog did not return every joint within 0.1 degrees')
        self.data['jog_completed'] = True

    def check_ratio_echo(self, snapshot, expected):
        inputs = snapshot.get('inputs')
        if not isinstance(inputs, (list, tuple)) or len(inputs) != 2:
            raise RuntimeError('Missing bilateral velocity/acceleration echoes')
        for item in inputs:
            for key in ('joint_vel_ratio', 'joint_acc_ratio'):
                value = item.get(key) if isinstance(item, dict) else None
                if type(value) is not int or not expected - 1 <= value <= expected:
                    raise RuntimeError(f'Unexpected {key} echo {value!r}; requested {expected}')

    def check_impedance_echo(self, snapshot):
        inputs = snapshot.get('inputs')
        self.data['impedance_inputs_echo'] = json_safe(inputs)
        if (not isinstance(inputs, (list, tuple)) or len(inputs) != 2 or any(
                not isinstance(item, dict) or item.get('imp_type') != 1 for item in inputs)):
            raise RuntimeError('Both joint-impedance subtype echoes must equal 1')
        # This cabinet has echoed integer 59 for requested 60. Accept at most
        # one percentage point of downward quantization, never an increased
        # ratio or a materially different retained setting. Record raw values.
        expected = self.data['limits']['impedance_velocity_acceleration_ratio']
        self.check_ratio_echo(snapshot, expected)

    def disable(self):
        self.stopping, self.stage = True, 'disable'
        # A failed clear could leave staged enable commands: never send that
        # unknown buffer. Once cleared, only state0 can be queued, so try both
        # arms and send once even if one idle setter rejects. Never retry.
        try:
            self.command('clear_set')
        except BaseException as error:
            self.data['errors'].append(f'Disable request failed: {error!r}')
            self.data.setdefault('failed_stage', 'disable-request')
        else:
            for method, arguments in (('set_state', {'arm': 'A', 'state': 0}),
                                      ('set_state', {'arm': 'B', 'state': 0}),
                                      ('send_cmd', {})):
                try:
                    self.command(method, **arguments)
                except BaseException as error:
                    self.data['errors'].append(f'Disable {method} failed: {error!r}')
                    self.data.setdefault('failed_stage', 'disable-request')
        try:
            snapshot = self.wait_stable(0, min(self.deadline, self.clock.monotonic() + 2.),
                                        prime=True, duration=.1)
            self.data['standby_state_confirmed'] = True
            self.data['final_controller_errors'] = [state['err_code'] for state in snapshot['states']]
            if any(self.data['final_controller_errors']):
                self.data['errors'].append('State0 confirmed but controller errors remain')
                self.data.setdefault('failed_stage', 'disable-confirm')
        except BaseException as error:
            self.data['errors'].append(f'DISABLE UNCONFIRMED: {error!r}; operator must verify physical stop')
            self.data.setdefault('failed_stage', 'disable-confirm')

    def run(self, ip, *, mode_hold=False, bounded_transition=False, position_jog=False):
        if (type(mode_hold) is not bool or type(position_jog) is not bool
                or (mode_hold and position_jog)):
            raise ValueError('mode_hold and position_jog must be mutually exclusive booleans')
        if type(bounded_transition) is not bool or (bounded_transition and not mode_hold):
            raise ValueError('bounded_transition requires a boolean and explicit mode_hold')
        self.bounded_transition = bounded_transition
        self.mode_hold = mode_hold
        self.position_jog, self.actuating = position_jog, mode_hold or position_jog
        self.data.update(ip=ip, mode=('dual-arm-position-jog' if position_jog else
                                    'dual-arm-mode-hold' if mode_hold else 'read-only'),
                         bounded_transition=bounded_transition)
        if position_jog:
            self.data['limits'].update(jog_joint_1based=7, jog_signed_amplitude_deg=[.25, -.25],
                                       jog_ramp_seconds=2., jog_max_command_hz=50.,
                                       jog_min_signed_response_deg=.1, jog_other_joint_drift_deg=.1,
                                       jog_return_error_deg=.1, jog_opposite_direction_tolerance_deg=.025)
        connected = False
        try:
            self.stage = 'connect'
            connected = bool(self.robot.connect(ip))
            if not connected:
                raise RuntimeError('SDK connection was not accepted')
            self.deadline = self.clock.monotonic() + 15.0
            self.action_deadline = self.deadline - 3.0
            self.data['controller_version'] = getattr(self.robot, 'controller_version', None)
            self.data['sdk_version'] = self.robot.SDK_version()
            if self.clock.monotonic() >= self.action_deadline:
                raise RuntimeError('Version query exhausted the action deadline')
            if self.actuating:
                self.mode_sequence()
            else:
                self.stage = 'read-only'
                until, valid = self.clock.monotonic() + 3., False
                while self.clock.monotonic() < until:
                    if self.stop_requested:
                        raise RuntimeError('Operator requested stop')
                    try:
                        self.read()
                        valid = True
                    except ValueError as error:
                        if str(error) != 'waiting for advancing feedback frames from both arms':
                            raise
                    self.clock.sleep(.02)
                if not valid:
                    raise RuntimeError('No advancing bilateral feedback')
        except BaseException as error:
            self.data['failed_stage'] = self.stage
            self.data['errors'].append(repr(error))
        finally:
            if connected and self.actuating:
                self.disable()
            self.stage = 'release'
            try:
                result = self.robot.release_robot()
                self.data['release_result'] = json_safe(result)
                if result is not True and not (type(result) is int and result == 1):
                    self.data['errors'].append(f'SDK release was not accepted: {result!r}')
                    self.data.setdefault('failed_stage', 'release')
            except BaseException as error:
                self.data['errors'].append(f'SDK release failed: {error!r}')
                self.data.setdefault('failed_stage', 'release')
            self.data['finished_unix'] = self.clock.time()
            self.data['success'] = not self.data['errors']
        return self.data


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    actions = parser.add_mutually_exclusive_group()
    actions.add_argument('--mode-hold', action='store_true',
                         help='Enable BOTH arms at measured pose: position then joint impedance')
    actions.add_argument('--position-jog', action='store_true',
                         help='Position only: BOTH J7 joints +0.25/-0.25 degrees and back, no Home')
    parser.add_argument('--bounded-transition', action='store_true',
                        help='With --mode-hold only: 0.15 s torque transition, <=3 deg/s and <=0.15 deg drift')
    parser.add_argument('--ip', default='6.6.7.190')
    parser.add_argument('--output', type=Path,
                        default=ROOT / 'work' / f'arm-mode-probe-{time.time_ns()}.json')
    args = parser.parse_args(argv)
    if args.bounded_transition and not args.mode_hold:
        parser.error('--bounded-transition requires explicit --mode-hold')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    # Reserve evidence before constructing/loading anything; never overwrite a prior run.
    with args.output.open('x', encoding='utf-8') as stream:
        json.dump({'success': False, 'status': 'preparing', 'mode_hold': args.mode_hold,
                   'position_jog': args.position_jog,
                   'bounded_transition': args.bounded_transition}, stream)
    report = {'success': False, 'standby_state_confirmed': False, 'errors': []}
    handlers = {}
    try:
        from tianji_output._internal.fx_robot import Marvin_Robot
        from tianji_output._internal.structure_data import DCSS
        from tianji_output.feedback_guard import DualArmFeedbackGuard
        from tianji_output.tianji_chest_driver import TianjiChestDriver
        probe = ModeProbe(Marvin_Robot(), DualArmFeedbackGuard(), DCSS, TianjiChestDriver)
        for signum in (signal.SIGINT, signal.SIGTERM):
            handlers[signum] = signal.signal(signum, probe.request_stop)
        report = probe.run(args.ip, mode_hold=args.mode_hold, bounded_transition=args.bounded_transition,
                           position_jog=args.position_jog)
    except BaseException as error:
        report['errors'].append(f'Probe preparation failed: {error!r}')
    finally:
        for signum, handler in handlers.items():
            signal.signal(signum, handler)
        args.output.write_text(json.dumps(json_safe(report), indent=2, allow_nan=False) + '\n',
                               encoding='utf-8')
    print(json.dumps({'success': report['success'], 'output': str(args.output),
                      'standby_state_confirmed': report['standby_state_confirmed'],
                      'failed_stage': report.get('failed_stage'), 'errors': report['errors']}), flush=True)
    return 0 if report['success'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
