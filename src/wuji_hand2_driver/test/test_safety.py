import pytest

from wuji_hand2_driver.joint_mapping import JointLimits, joint_names
from wuji_hand2_driver.safety import DriverConfig, HandSafety
from wuji_hand2_driver.sdk_backend import DiagnosticSample, JointDiagnostic, Snapshot, StateSample


class FakeBackend:
    def __init__(self):
        self.calls = []
        self.data = Snapshot(None, None)
        self.send_failure = False

    def update(self, now, enabled=False, count=20, error=0, position=0.4):
        self.data = Snapshot(StateSample(now, (position,) * 20, (0.0,) * 20),
                             DiagnosticSample(now, tuple(
                                 JointDiagnostic((i // 4) * 5 + i % 4 + 1,
                                                 2 if enabled else 1, 0.0, 25.0, 24.0, error)
                                 for i in range(count))))

    def snapshot(self):
        return self.data

    def configure(self, *args): self.calls.append(("configure", args))
    def enable(self): self.calls.append(("enable",))
    def disable(self): self.calls.append(("disable",))
    def clear_fault(self): self.calls.append(("clear_fault",))
    def emergency_stop(self): self.calls.append(("emergency_stop",))
    def close(self): self.calls.append(("close",))

    def send(self, positions):
        if self.send_failure:
            raise RuntimeError("transport lost")
        self.calls.append(("send", tuple(positions)))


def make_core(read_only=False, monotonic=None, **kwargs):
    backend = FakeBackend()
    limits = JointLimits(joint_names("left"), (-1.0,) * 20, (2.0,) * 20, (2.0,) * 20)
    clock_args = {} if monotonic is None else {"monotonic": monotonic}
    core = HandSafety(backend, limits, DriverConfig(**kwargs), read_only, **clock_args)
    core.connected(0.0)
    backend.update(0.0)
    return core, backend


def arm(core, backend):
    assert core.set_enabled(True, 0.0)[0]
    assert core.lifecycle == "ENABLING"
    backend.update(0.01, enabled=True)
    core.tick(0.01)
    assert core.lifecycle == "READY"


def test_default_no_command_does_not_enable_or_send_zero():
    core, backend = make_core()
    core.tick(0.0)
    assert core.lifecycle == "STOPPED"
    assert backend.calls == []
    assert not core.command([0.0] * 20, (), 0.0, 0.0)[0]
    assert backend.calls == []
    arm(core, backend)
    assert backend.calls[-1] == ("send", (0.4,) * 20)
    # Before the first command it holds a measured pose, never a zero fallback.
    backend.update(0.02, enabled=True)
    core.tick(0.02)
    assert backend.calls[-1] == ("send", (0.4,) * 20)


def test_diagnostic_fault_identifies_joint_and_hexadecimal_code():
    from dataclasses import replace
    core, backend = make_core()
    arm(core, backend)
    backend.update(0.02, enabled=True)
    joints = list(backend.data.diagnostics.joints)
    joints[17] = replace(joints[17], error_code=7)
    backend.data = replace(backend.data, diagnostics=DiagnosticSample(0.02, tuple(joints)))
    core.tick(0.02)
    assert core.lifecycle == "FAULT"
    assert "l_pinky_mcp_abd (nid=22, code=0x0007)" in core.fault_reason
    assert backend.calls[-1] == ("disable",)


def test_enable_requires_complete_fresh_feedback_and_20_enabled():
    core, backend = make_core()
    backend.update(0.0, count=19)
    assert not core.set_enabled(True, 0.0)[0]
    backend.update(0.0)
    assert core.set_enabled(True, 0.0)[0]
    backend.update(0.01, enabled=False)
    core.tick(0.01)
    assert core.lifecycle == "ENABLING"
    assert not any(c[0] == "send" for c in backend.calls)
    backend.update(0.02, enabled=True, count=19)
    core.tick(0.02)
    assert core.lifecycle == "FAULT"
    assert backend.calls[-1] == ("disable",)


def test_ramp_velocity_bound_and_no_catchup_jump():
    core, backend = make_core(command_timeout=2.0)
    arm(core, backend)
    assert core.command([1.4] * 20, (), 0.02, 0.02)[0]
    previous = backend.calls[-1][1]
    for now in (0.02, 0.025, 0.03, 0.20, 0.205):
        backend.update(now, enabled=True)
        core.tick(now)
        outgoing = backend.calls[-1][1]
        assert max(abs(a - b) for a, b in zip(outgoing, previous)) <= 0.5 / 200 + 1e-12
        assert all(0.4 <= q <= 1.4 for q in outgoing)
        previous = outgoing


@pytest.mark.parametrize("fault_kind", ["command", "feedback", "diagnostics", "joint_error", "disabled", "send"])
def test_faults_disable_and_never_auto_reenable(fault_kind):
    core, backend = make_core()
    arm(core, backend)
    core.command([0.8] * 20, (), 0.02, 0.02)
    now = 0.30
    backend.update(now, enabled=True)
    if fault_kind != "command":
        core.command([0.8] * 20, (), now, now)
    if fault_kind == "feedback":
        backend.data = Snapshot(StateSample(0.0, (0.4,) * 20, (0.0,) * 20), backend.data.diagnostics)
    elif fault_kind == "diagnostics":
        backend.data = Snapshot(backend.data.state, None)
    elif fault_kind == "joint_error":
        backend.update(now, enabled=True, error=1)
    elif fault_kind == "disabled":
        backend.update(now, enabled=False)
    elif fault_kind == "send":
        backend.send_failure = True
    core.tick(now)
    assert core.lifecycle == "FAULT"
    assert backend.calls[-1] == ("disable",)
    count = len(backend.calls)
    backend.update(0.31, enabled=False)
    core.command([0.8] * 20, (), 0.31, 0.31)
    core.tick(0.31)
    assert core.lifecycle == "FAULT"
    assert len(backend.calls) == count
    assert not core.set_enabled(True, 0.31)[0]


def test_explicit_clear_waits_for_new_disabled_diagnostics_then_manual_enable():
    core, backend = make_core()
    arm(core, backend)
    core.fail("operator stop", emergency=True)
    assert backend.calls[-2:] == [("emergency_stop",), ("disable",)]
    backend.update(0.02, enabled=False)
    assert core.clear_fault(0.02)[0]
    core.tick(0.02)
    assert core.lifecycle == "FAULT"  # Old frame must not acknowledge a new clear.
    backend.update(0.03, enabled=True)
    core.tick(0.03)
    assert core.lifecycle == "FAULT"
    backend.update(0.04, enabled=False)
    core.tick(0.04)
    assert core.lifecycle == "STOPPED"
    assert [c[0] for c in backend.calls].count("enable") == 1
    assert core.set_enabled(True, 0.04)[0]
    assert core.lifecycle == "ENABLING"
    assert not core.command([0.8] * 20, (), 0.01, 0.04)[0]


def test_read_only_never_writes_even_on_fault_or_shutdown():
    core, backend = make_core(read_only=True)
    core.tick(0.0)
    assert core.lifecycle == "READ_ONLY"
    assert not core.set_enabled(True, 0.0)[0]
    assert not core.set_enabled(False, 0.0)[0]
    assert not core.clear_fault(0.0)[0]
    assert not core.command([0.8] * 20, (), 0.0, 0.0)[0]
    core.fail("stale feedback", emergency=True)
    core.close()
    assert backend.calls == [("close",)]


@pytest.mark.parametrize("bad", [[0.0] * 19, [float("nan")] * 20, [3.0] * 20])
def test_invalid_active_command_faults_without_sending(bad):
    core, backend = make_core()
    arm(core, backend)
    sent = [c for c in backend.calls if c[0] == "send"]
    assert not core.command(bad, (), 0.02, 0.02)[0]
    assert core.lifecycle == "FAULT"
    assert [c for c in backend.calls if c[0] == "send"] == sent


def test_wait_for_first_command_is_bounded_and_holds_measured_pose():
    core, backend = make_core(first_command_timeout=0.1)
    arm(core, backend)
    backend.update(0.12, enabled=True)
    core.tick(0.12)
    assert core.lifecycle == "FAULT"
    assert backend.calls[-1] == ("disable",)


def test_shutdown_disables_then_releases_device():
    core, backend = make_core()
    arm(core, backend)
    core.close()
    assert backend.calls[-2:] == [("disable",), ("close",)]


def test_enable_timeout_cannot_be_satisfied_by_pre_enable_frames():
    core, backend = make_core(enable_timeout=0.1)
    assert core.set_enabled(True, 0.0)[0]
    backend.update(0.0, enabled=True)
    core.tick(0.01)
    assert core.lifecycle == "ENABLING"
    backend.update(0.12, enabled=False)
    core.tick(0.12)
    assert core.lifecycle == "FAULT"
    assert not any(c[0] == "send" for c in backend.calls)


def test_pre_enable_commands_cannot_be_replayed_into_new_session():
    core, backend = make_core()
    arm(core, backend)
    assert not core.command([0.8] * 20, (), 0.0, 0.02)[0]
    assert core.lifecycle == "READY"
    assert core.command([0.8] * 20, (), 0.02, 0.02)[0]
    core.set_enabled(False, 0.03)
    assert core.lifecycle == "STOPPED"
    assert core.command_at is None
    assert core._target is None


def test_invalid_config_rejected_before_connect():
    for kwargs in ({"command_rate": 0}, {"command_rate": 1001},
                   {"kp": float("nan")}, {"max_velocity": -1}):
        with pytest.raises(ValueError):
            DriverConfig(**kwargs)


def test_enable_rpc_and_snapshot_callbacks_use_fresh_evaluation_time():
    clock = [0.0]
    core, backend = make_core(monotonic=lambda: clock[0])

    def configure(*_args):
        clock[0] = 0.01
        backend.update(clock[0])  # A callback arrives during the blocking RPC.

    def enable():
        clock[0] = 0.02
        backend.update(clock[0], enabled=True)

    backend.configure = configure
    backend.enable = enable
    assert core.set_enabled(True)[0]
    assert core._enable_at == 0.02
    core.tick()
    assert core.lifecycle == "ENABLING"  # RPC-time feedback cannot acknowledge it.

    def snapshot():
        clock[0] = 0.03
        backend.update(clock[0], enabled=True)
        return backend.data

    backend.snapshot = snapshot
    core.tick()  # The evaluation clock must be read after this cache snapshot.
    assert core.lifecycle == "READY"
    assert backend.calls[-1] == ("send", (0.4,) * 20)
    # A ROS callback after the prior tick must not be judged against its old time.
    clock[0] = 0.031
    assert core.command([0.8] * 20, (), clock[0])[0]
    assert core.lifecycle == "ACTIVE"


def test_clear_acknowledgement_must_follow_rpc_completion():
    clock = [0.0]
    core, backend = make_core(monotonic=lambda: clock[0])
    core.fail("test fault")

    def clear_fault():
        clock[0] = 0.01
        backend.update(clock[0])

    def disable():
        clock[0] = 0.02
        backend.update(clock[0])

    backend.clear_fault = clear_fault
    backend.disable = disable
    assert core.clear_fault()[0]
    assert core._clear_requested_at == 0.02
    core.tick()
    assert core.lifecycle == "FAULT"
    clock[0] = 0.03
    backend.update(clock[0])
    core.tick()
    assert core.lifecycle == "STOPPED"


def test_startup_auto_enable_waits_for_async_disable_then_is_consumed():
    clock = [0.0]
    core, backend = make_core(monotonic=lambda: clock[0])
    backend.data = Snapshot(None, None)
    assert core.try_startup_enable()
    backend.update(clock[0], enabled=True)
    assert core.try_startup_enable()  # Connection disable has not finished.
    assert not any(call[0] == "enable" for call in backend.calls)
    clock[0] = 0.01
    backend.update(clock[0], enabled=False)
    assert not core.try_startup_enable()
    assert core.lifecycle == "ENABLING"
    assert [call[0] for call in backend.calls].count("enable") == 1
    assert not core.try_startup_enable()
    core.fail("transport lost")
    backend.update(clock[0], enabled=False)
    assert not core.try_startup_enable()
    assert [call[0] for call in backend.calls].count("enable") == 1
