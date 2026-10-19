"""Pure validation of cabinet feedback and bounded motion cancellation."""
import math
import time
import numpy as np


def joint_vector(values, label="joint positions"):
    try:
        result = np.asarray(values, dtype=float)
    except (ValueError, TypeError) as exc:
        raise ValueError(f"{label} must be seven finite numbers") from exc
    if result.shape != (7,) or not np.isfinite(result).all():
        raise ValueError(f"{label} must be seven finite numbers")
    return result.tolist()


class ArmJointLimits:
    """Cabinet model PNVA: positive limit, negative limit, speed, acceleration (degrees)."""
    def __init__(self, pnva):
        data = np.asarray(pnva, dtype=float)
        if (data.shape != (2, 7, 4) or not np.isfinite(data).all()
                or np.any(data[:, :, 0] <= data[:, :, 1]) or np.any(data[:, :, 2:] <= 0)):
            raise ValueError('invalid dual-arm PNVA model limits')
        self.data = data.copy()

    def validate(self, side, values):
        q = np.asarray(joint_vector(values, f'{side} target'), dtype=float)
        bounds = self.data[0 if side == 'left' else 1]
        if np.any(q < bounds[:, 1] - 1e-5) or np.any(q > bounds[:, 0] + 1e-5):
            raise ValueError(f'{side} target exceeds model joint limits (degrees)')
        return np.clip(q, bounds[:, 1], bounds[:, 0]).tolist()

    def validate_quintic(self, side, start, target, duration, max_velocity=30.0):
        start = self.validate(side, start)
        target = self.validate(side, target)
        delta = np.abs(np.asarray(target) - start)
        bounds = self.data[0 if side == 'left' else 1]
        if (np.any(1.875 * delta / duration > np.minimum(bounds[:, 2], max_velocity))
                or np.any((10 / math.sqrt(3)) * delta / duration**2 > bounds[:, 3])):
            raise ValueError('initial trajectory exceeds joint velocity/acceleration limits; increase duration')
        return start, target


def limit_joint_step(previous, target, max_step):
    previous = np.asarray(joint_vector(previous, 'previous command'))
    target = np.asarray(joint_vector(target, 'IK target'))
    if not math.isfinite(max_step) or max_step < 0:
        raise ValueError('max joint step must be finite and nonnegative')
    return (previous + np.clip(target - previous, -max_step, max_step)).tolist()


def pose_mm(pose, unit):
    if unit not in ('m', 'mm'):
        raise ValueError('pose unit must be m or mm; rotation is always degrees')
    values = np.asarray(pose, dtype=float)
    if values.shape != (6,) or not np.isfinite(values).all():
        raise ValueError('pose must contain six finite values')
    values = values.copy()
    if unit == 'm':
        values[:3] *= 1000
    return values.tolist()


class DualArmFeedbackGuard:
    """Require observed advancement on both arms; reject stale zero-init caches.

    A measured all-zero pose is valid once advancing frames establish that it
    came from the cabinet. Rejecting zeros by angle alone would reject a real
    pose while doing nothing to detect a stale nonzero pose.
    """
    def __init__(self, timeout=0.25, clock=time.monotonic):
        self.timeout = timeout
        self.clock = clock
        self.serials = [None, None]
        self.changed_at = [None, None]
        self.advanced = [False, False]

    def observe(self, snapshot):
        outputs = snapshot.get("outputs") if isinstance(snapshot, dict) else None
        if not isinstance(outputs, (list, tuple)) or len(outputs) < 2:
            raise ValueError("cabinet feedback must contain both arm outputs")
        now = self.clock()
        positions = []
        for index, side in enumerate(("left", "right")):
            output = outputs[index]
            if not isinstance(output, dict):
                raise ValueError(f"missing {side} cabinet output")
            positions.append(joint_vector(output.get("fb_joint_pos"), f"{side} feedback"))
            serial = output.get("frame_serial")
            if isinstance(serial, bool) or not isinstance(serial, int) or not 0 <= serial <= 1000000:
                raise ValueError(f"invalid {side} feedback frame_serial")
            previous = self.serials[index]
            if previous is None:
                self.serials[index] = serial
                self.changed_at[index] = now
            else:
                delta = (serial - previous) % 1000001
                if delta > 500000:
                    raise ValueError(f"{side} feedback frame_serial moved backwards")
                if delta > 0:
                    self.serials[index] = serial
                    self.changed_at[index] = now
                    self.advanced[index] = True
        if not all(self.advanced):
            raise ValueError("waiting for advancing feedback frames from both arms")
        if any(now - changed > self.timeout for changed in self.changed_at):
            raise ValueError("cabinet joint feedback is stale")
        return tuple(positions)


def check_motion_allowed(should_abort):
    if should_abort is not None and should_abort():
        raise RuntimeError("initial motion cancelled: tracker data unavailable or shutdown requested")


def guarded_wait(duration, should_abort=None, sleep=time.sleep, clock=time.monotonic):
    if not math.isfinite(duration) or duration < 0:
        raise ValueError("settle timeout must be finite and nonnegative")
    deadline = clock() + duration
    while True:
        check_motion_allowed(should_abort)
        remaining = deadline - clock()
        if remaining <= 0:
            return
        sleep(min(0.01, remaining))


def confirm_standby(robot, feedback_guard, timeout=3.0):
    """Request both arms off and require live feedback confirming state zero."""
    from ._internal.structure_data import DCSS
    if not math.isfinite(timeout) or timeout <= 0:
        raise ValueError('standby timeout must be finite and positive')
    robot.clear_set()
    robot.set_state(arm='A', state=0)
    robot.set_state(arm='B', state=0)
    robot.send_cmd()
    deadline = time.monotonic() + timeout
    states = None
    while time.monotonic() < deadline:
        snapshot = robot.subscribe(DCSS())
        try:
            feedback_guard.observe(snapshot)
            states = snapshot.get('states')
            if (isinstance(states, (list, tuple)) and len(states) >= 2
                    and all(isinstance(state, dict) and state.get('cur_state') == 0
                            for state in states[:2])):
                return
        except ValueError:
            pass
        time.sleep(0.01)
    raise RuntimeError(f'disable unconfirmed after {timeout}s; arm states={states}')


def disable_and_release(robot, feedback_guard, timeout=3.0):
    """Release even after a failed disable, but propagate all unconfirmed errors."""
    errors = []
    try:
        confirm_standby(robot, feedback_guard, timeout)
    except Exception as exc:
        errors.append(f'disable: {exc}')
    finally:
        try:
            robot.release_robot()
        except Exception as exc:
            errors.append(f'release: {exc}')
    if errors:
        raise RuntimeError('; '.join(errors))
