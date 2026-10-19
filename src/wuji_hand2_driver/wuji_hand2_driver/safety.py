"""Deterministic, ROS-independent safety state machine for a single hand."""
from dataclasses import dataclass
import math
import time


@dataclass(frozen=True)
class DriverConfig:
    command_rate: float = 200.0
    publish_rate: float = 200.0
    diagnostics_rate: float = 10.0
    command_timeout: float = 0.25
    feedback_timeout: float = 0.25
    diagnostics_timeout: float = 0.5
    enable_timeout: float = 5.0
    startup_timeout: float = 5.0
    first_command_timeout: float = 5.0
    ramp_duration: float = 1.5
    max_velocity: float = 0.5
    kp: float = 3.0
    kd: float = 0.05
    effort_limit: float = 1.5

    def __post_init__(self):
        for name, value in vars(self).items():
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
        if self.command_rate > 1000 or self.publish_rate > 1000:
            raise ValueError("command/publish rates must not exceed 1000 Hz")


class HandSafety:
    """All methods run on the hardware worker; snapshots come from SDK caches.

    Faults latch. Neither a new command nor restored Ethernet/feedback re-enables a
    hand. clear_fault is explicit and requires a fresh disabled, fault-free
    diagnostic frame before a subsequent explicit enable can succeed.
    """
    ARMED_STATES = ("ENABLING", "READY", "ACTIVE")

    def __init__(self, backend, limits, config, read_only=False, monotonic=time.monotonic):
        self.backend = backend
        self.limits = limits
        self.config = config
        self.read_only = read_only
        self._clock = monotonic
        self.lifecycle = "DISCONNECTED"
        self.fault_reason = ""
        self.command_at = None
        self._target = None
        self._last_sent = None
        self._start_pose = None
        self._ramp_started = None
        self._connected_at = None
        self._enable_at = None
        self._ready_at = None
        self._clear_requested_at = None
        self._last_tick = None

    def connected(self, now):
        self._connected_at = now
        self.lifecycle = "READ_ONLY" if self.read_only else "STOPPED"

    def _fresh(self, sample, timeout, now):
        return sample is not None and 0 <= now - sample.received_at <= timeout

    def feedback_problem(self, snapshot, now):
        if snapshot.error:
            return snapshot.error
        if not self._fresh(snapshot.state, self.config.feedback_timeout, now):
            return "joint feedback missing or stale"
        if not self._fresh(snapshot.diagnostics, self.config.diagnostics_timeout, now):
            return "joint diagnostics missing or stale"
        if len(snapshot.diagnostics.joints) != 20:
            return "diagnostics must include all 20 joints"
        if any(j.error_code != 0 for j in snapshot.diagnostics.joints):
            return "joint diagnostics report an active error"
        try:
            self.limits.validate_command(snapshot.state.positions)
        except ValueError:
            return "actual joint positions are outside model limits or invalid"
        return ""

    def _forget_command(self):
        self._target = None
        self.command_at = None
        self._last_sent = None
        self._start_pose = None
        self._ramp_started = None

    def fail(self, reason, emergency=False):
        self.lifecycle = "FAULT"
        self.fault_reason = str(reason)
        self._clear_requested_at = None
        self._forget_command()
        if not self.read_only:
            errors = []
            if emergency:
                try:
                    self.backend.emergency_stop()
                except Exception as exc:
                    errors.append(f"emergency_stop failed: {exc}")
            try:
                self.backend.disable()
            except Exception as exc:
                errors.append(f"disable failed: {exc}")
            if errors:
                self.fault_reason += "; " + "; ".join(errors)

    def set_enabled(self, enabled, now=None):
        if self.read_only:
            return False, "read_only forbids enabling or disabling hardware"
        if not enabled:
            was_fault = self.lifecycle == "FAULT"
            try:
                self.backend.disable()
            except Exception as exc:
                self.fail(f"disable failed: {exc}")
                return False, self.fault_reason
            self._forget_command()
            if not was_fault:
                self.lifecycle = "STOPPED"
            return True, "disable requested; verify disabled diagnostics"
        if self.lifecycle != "STOPPED":
            return False, f"enable requires STOPPED, current state is {self.lifecycle}"
        snapshot = self.backend.snapshot()
        # Read the clock after the atomic cache snapshot. A callback can arrive
        # during any RPC; a timestamp captured at the start of the worker loop
        # would incorrectly classify that new frame as being in the future.
        observed_at = self._clock() if now is None else now
        problem = self.feedback_problem(snapshot, observed_at)
        if problem:
            return False, problem
        if any(j.ext_state == 2 for j in snapshot.diagnostics.joints):
            return False, "awaiting confirmation that all joints are disabled"
        try:
            self.backend.configure(self.config.kp, self.config.kd, self.config.effort_limit)
            self.backend.enable()
        except Exception as exc:
            self.fail(f"enable failed: {exc}")
            return False, self.fault_reason
        self._forget_command()
        # Require acknowledgement frames acquired after the enable RPC finishes.
        self._enable_at = self._clock() if now is None else now
        self.lifecycle = "ENABLING"
        return True, "enable requested; waiting for all 20 Enabled diagnostics"

    def clear_fault(self, now=None):
        if self.read_only:
            return False, "read_only forbids clearing hardware faults"
        if self.lifecycle != "FAULT":
            return False, "clear_fault requires a latched FAULT"
        try:
            self.backend.clear_fault()
            self.backend.disable()
        except Exception as exc:
            self.fail(f"clear_fault failed: {exc}")
            return False, self.fault_reason
        self._clear_requested_at = self._clock() if now is None else now
        return True, "clear requested; awaiting fresh fault-free disabled diagnostics; enable remains explicit"

    def try_startup_enable(self):
        """Return whether a requested one-shot startup enable is still pending.

        Connection disable is asynchronous. Wait through missing telemetry or
        still-enabled diagnostics, but consume the request once accepted or if
        any fault/state transition prevents startup. Manual actions cancel the
        pending request in the worker before this method is called.
        """
        if self.lifecycle != "STOPPED":
            return False
        snapshot = self.backend.snapshot()
        if self.feedback_problem(snapshot, self._clock()):
            return True
        accepted, _message = self.set_enabled(True)
        return not accepted and self.lifecycle == "STOPPED"

    def command(self, positions, names, received_at, now=None):
        if self.read_only:
            return False, "read_only ignores joint commands"
        if self.lifecycle not in ("READY", "ACTIVE"):
            return False, f"joint command ignored while {self.lifecycle}"
        now = self._clock() if now is None else now
        try:
            values = self.limits.validate_command(positions, names)
            if not 0 <= now - received_at <= self.config.command_timeout:
                raise ValueError("command arrival is stale")
            if received_at < self._ready_at:
                return False, "command predates current enable session"
        except ValueError as exc:
            self.fail(f"invalid command: {exc}")
            return False, self.fault_reason
        self._target = values
        self.command_at = received_at
        if self.lifecycle == "READY":
            self._ramp_started = now
            self.lifecycle = "ACTIVE"
        return True, "command accepted"

    def tick(self, now=None):
        snapshot = self.backend.snapshot()
        now = self._clock() if now is None else now
        dt = 1.0 / self.config.command_rate
        if self._last_tick is not None:
            # A delayed worker must not catch up by issuing a large target jump.
            dt = max(0.0, min(dt, now - self._last_tick))
        self._last_tick = now
        if self.lifecycle == "DISCONNECTED":
            return
        if self.lifecycle == "FAULT":
            if (self._clear_requested_at is not None
                    and not self.feedback_problem(snapshot, now)
                    and snapshot.diagnostics.received_at > self._clear_requested_at
                    and all(j.ext_state != 2 for j in snapshot.diagnostics.joints)):
                self.lifecycle = "STOPPED"
                self.fault_reason = ""
                self._clear_requested_at = None
            return
        problem = self.feedback_problem(snapshot, now)
        if problem:
            if (self.lifecycle in self.ARMED_STATES
                    or now - self._connected_at > self.config.startup_timeout):
                self.fail(problem)
            return
        if self.lifecycle == "READ_ONLY":
            return
        if self.lifecycle == "STOPPED":
            return
        if self.lifecycle == "ENABLING":
            if now - self._enable_at > self.config.enable_timeout:
                self.fail("timed out waiting for all 20 joints Enabled")
                return
            if (snapshot.diagnostics.received_at <= self._enable_at
                    or snapshot.state.received_at <= self._enable_at
                    or not all(j.ext_state == 2 for j in snapshot.diagnostics.joints)):
                return
            self.lifecycle = "READY"
            self._ready_at = now
            # Complete, finite, model-valid actual feedback is mandatory. No zero fallback.
            self._start_pose = tuple(snapshot.state.positions)
            self._last_sent = self._start_pose
        elif not all(j.ext_state == 2 for j in snapshot.diagnostics.joints):
            self.fail("one or more joints left Enabled state")
            return
        if self.lifecycle == "READY":
            if now - self._ready_at > self.config.first_command_timeout:
                self.fail("no fresh command after enable")
                return
            target = self._start_pose  # Hold current pose while waiting; never home to zero.
        else:
            if self.command_at is None or now - self.command_at > self.config.command_timeout:
                self.fail("joint command watchdog timeout")
                return
            alpha = min(1.0, max(0.0, (now - self._ramp_started) / self.config.ramp_duration))
            target = tuple(start + alpha * (goal - start)
                           for start, goal in zip(self._start_pose, self._target))
        outgoing = tuple(
            last + max(-min(v, self.config.max_velocity) * dt,
                       min(min(v, self.config.max_velocity) * dt, goal - last))
            for last, goal, v in zip(self._last_sent, target, self.limits.velocity))
        # Defense in depth: trajectory output has the same finite/limit contract.
        try:
            outgoing = self.limits.validate_command(outgoing)
            self.backend.send(outgoing)
        except Exception as exc:
            self.fail(f"command send failed: {exc}")
            return
        self._last_sent = outgoing

    def close(self):
        try:
            if not self.read_only and self.lifecycle != "DISCONNECTED":
                self.backend.disable()
        finally:
            self.lifecycle = "DISCONNECTED"
            self._forget_command()
            self.backend.close()
