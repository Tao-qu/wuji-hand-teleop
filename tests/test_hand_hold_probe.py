"""Probe regressions use production safety logic with fake clocks/backends only."""
from pathlib import Path
import sys
from types import SimpleNamespace as NS

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "src/wuji_hand2_driver"))

from tools.verify_hand_hold import COMM_FIELDS, CONFIG, fixed_extension, raw_diagnostic_frame, run_probe
from tools import verify_hand_hold as probe
from wuji_hand2_driver.joint_mapping import JointLimits, joint_names
from wuji_hand2_driver.sdk_backend import DiagnosticSample, JointDiagnostic, Snapshot, StateSample


class Clock:
    def __init__(self):
        self.value = 0.0

    def __call__(self):
        return self.value


class FakeBackend:
    """No SDK, ROS, sockets, locks, or real sleeps. Feedback advances on sleep."""
    def __init__(self, clock):
        self.clock = clock
        self.calls = []
        self.enabled = False
        self.freeze_after_disable = False
        self.freeze_while_enabled = False
        self.disable_seen = False
        self.enable_jump = 0.0
        self.drift_after_ready = 0.02
        self.warning_at = None
        self.copy_error_count = 0
        self.events = []
        self.data = Snapshot(None, None)
        self.raw = None
        self.first_enabled_feedback_at = None
        self.close_at = None

    def update(self):
        now = self.clock()
        first = self.enabled and self.first_enabled_feedback_at is None
        if first:
            self.first_enabled_feedback_at = now
        position = 0.4 + (self.enable_jump if self.enabled else 0.0)
        if self.enabled and not first:
            position += self.drift_after_ready
        warning = self.enabled and self.warning_at is not None and now >= self.warning_at
        if warning and not self.events:
            # Simulate a transient warning that cleared in the latest cache.
            self.events.append({"error_joints": [{"nid": 13, "error_code_current": 3}]})
        joints = tuple(JointDiagnostic((i // 4) * 5 + i % 4 + 1,
                                       2 if self.enabled else 1, 0.0, 30.0, 12.4, 0)
                       for i in range(20))
        self.data = Snapshot(StateSample(now, (position,) * 20, (0.0,) * 20), DiagnosticSample(now, joints))
        self.raw = {"received_at": now, "num_joints": 20,
                    "joints": [{} for _ in range(20)], "error_joints": []}

    def sleep(self, seconds):
        self.clock.value += seconds
        if self.freeze_after_disable and self.disable_seen:
            return
        if self.freeze_while_enabled and self.enabled:
            return
        self.update()

    def connect(self):
        self.calls.append(("connect", self.clock()))
        self.update()

    def snapshot(self):
        return self.data

    def evidence(self):
        return {"latest_raw_diagnostics": self.raw, "raw_copy_error_count": self.copy_error_count,
                "raw_copy_error": "missing SDK field" if self.copy_error_count else "",
                "raw_error_frame_count": len(self.events), "raw_error_frames": list(self.events)}

    def configure(self, *args):
        self.calls.append(("configure", self.clock(), args))

    def enable(self):
        self.calls.append(("enable", self.clock()))
        self.enabled = True

    def disable(self):
        self.calls.append(("disable", self.clock()))
        self.enabled = False
        self.disable_seen = True
        if self.freeze_after_disable:
            # Even a cached Ready status cannot substitute for a new frame.
            old = self.data.diagnostics
            joints = tuple(JointDiagnostic(j.nid, 1, j.current_a, j.temperature_c,
                                           j.voltage_v, j.error_code) for j in old.joints)
            self.data = Snapshot(self.data.state, DiagnosticSample(old.received_at, joints))

    def send(self, positions):
        self.calls.append(("send", self.clock(), tuple(positions)))

    def clear_fault(self):
        raise AssertionError("probe must never clear faults")

    def emergency_stop(self):
        raise AssertionError("probe must not add an emergency-stop RPC")

    def close(self):
        self.calls.append(("close", self.clock()))
        self.close_at = self.clock()


@pytest.fixture
def setup():
    clock = Clock()
    backend = FakeBackend(clock)
    limits = JointLimits(joint_names("right"), (-1.0,) * 20, (2.0,) * 20, (2.0,) * 20)
    return backend, limits, clock


def run(setup, **kwargs):
    backend, limits, clock = setup
    return run_probe(backend, limits, clock=clock, sleep=backend.sleep, **kwargs)


def test_default_is_readonly_without_actuator_calls(setup):
    report = run(setup)
    backend, _, _ = setup
    assert [call[0] for call in backend.calls] == ["connect", "close"]
    assert report["completed"] and report["disabled_confirmed"] and report["closed"]
    assert report["enable_attempts"] == 0 and not report["errors"]
    assert report["disabled_confirmed_at"] - report["disable_confirmation_boundary"] >= .1


def test_readonly_cannot_accidentally_connect_a_writable_backend(setup):
    backend, _, _ = setup
    backend.read_only = False
    with pytest.raises(ValueError, match="read_only mode"):
        run(setup)
    assert not backend.calls


def test_fixed_ready_capture_is_not_following_later_measured_drift(setup):
    report = run(setup, hold=True)
    backend, _, _ = setup
    sends = [call for call in backend.calls if call[0] == "send"]
    assert report["q0"] == [0.4] * 20
    assert all(call[2] == (0.4,) * 20 for call in sends)
    assert any(row["snapshot"]["state"]["positions"][0] > .4
               for row in report["samples"] if row["stage"] == "static_hold")
    assert sends[0][1] == backend.first_enabled_feedback_at
    assert [call[2] for call in backend.calls if call[0] == "configure"] == [(5.0, 0.1, 0.8)]
    assert [call[0] for call in backend.calls].count("enable") == 1
    assert report["static_completed"] and report["disabled_confirmed"] and not report["errors"]
    disable = next(call[1] for call in backend.calls if call[0] == "disable")
    assert disable - backend.first_enabled_feedback_at >= 3.0
    assert backend.close_at - disable >= .1


def test_extension_after_three_seconds_changes_only_index_ten_with_rate_cap(setup):
    report = run(setup, hold=True, extension_rad=.05)
    backend, _, _ = setup
    sends = [call for call in backend.calls if call[0] == "send"]
    moving = [call for call in sends if call[2][10] < .4]
    assert moving[0][1] - backend.first_enabled_feedback_at >= 3.0
    assert report["extension_target"][10] == pytest.approx(.35)
    for _, _, values in sends:
        assert all(q == .4 for i, q in enumerate(values) if i != 10)
        assert .35 - 1e-12 <= values[10] <= .4
    for before, after in zip(sends, sends[1:]):
        assert abs(after[2][10] - before[2][10]) <= CONFIG.max_velocity / CONFIG.command_rate + 1e-12
    assert sends[-1][2][10] == pytest.approx(.35)
    assert report["static_completed"] and report["extension_completed"] and not report["errors"]


@pytest.mark.parametrize("velocity", [0.0, -.1, 1.0001, float("nan"), float("inf"), -float("inf")])
def test_velocity_bounds_reject_before_connection(setup, velocity):
    with pytest.raises(ValueError, match="max-velocity"):
        run(setup, hold=True, extension_rad=.05, max_velocity=velocity)
    assert not setup[0].calls


def test_tiny_positive_velocity_cannot_make_an_unbounded_extension(setup):
    with pytest.raises(ValueError, match="5 s extension-stage limit"):
        run(setup, hold=True, extension_rad=.05, max_velocity=1e-320)
    assert not setup[0].calls


def test_selected_one_radian_slew_is_local_and_preserves_extent_and_other_joints(setup):
    report = run(setup, hold=True, extension_rad=.05, max_velocity=1.0)
    backend, _, _ = setup
    sends = [call for call in backend.calls if call[0] == "send"]
    steps = [abs(after[2][10] - before[2][10]) for before, after in zip(sends, sends[1:])]
    assert max(steps) == pytest.approx(1.0 / CONFIG.command_rate)
    assert max(steps) > CONFIG.max_velocity / CONFIG.command_rate
    assert report["config"]["max_velocity"] == 1.0
    assert CONFIG.max_velocity == .1  # The immutable default is never mutated.
    assert report["config"]["kp"] == 5.0 and report["config"]["kd"] == .1
    assert report["config"]["effort_limit"] == .8
    assert all(all(q == .4 for i, q in enumerate(call[2]) if i != 10) for call in sends)
    assert all(.35 - 1e-12 <= call[2][10] <= .4 for call in sends)
    assert sends[-1][2][10] == pytest.approx(.35)
    moving = [call for call in sends if call[2][10] < .4]
    assert moving[0][1] - backend.first_enabled_feedback_at >= 3.0
    assert report["observed_drift_limit_rad"] == .1
    assert report["completed"] and report["disabled_confirmed"] and not report["errors"]


def test_cli_invalid_velocity_fails_before_model_read_or_backend_creation(monkeypatch):
    def forbidden(*_args, **_kwargs):
        raise AssertionError("invalid velocity must fail before model read or backend creation")

    monkeypatch.setattr(probe.JointLimits, "from_urdf", forbidden)
    monkeypatch.setattr(probe, "RecordingBackend", forbidden)
    with pytest.raises(SystemExit) as failure:
        probe.main(["--hold", "--max-velocity", "nan"])
    assert failure.value.code == 2


@pytest.mark.parametrize("amount", [-.001, .0501, float("nan"), float("inf")])
def test_extension_bounds_reject_before_connection(setup, amount):
    with pytest.raises(ValueError):
        run(setup, hold=True, extension_rad=amount)
    assert not setup[0].calls


def test_extension_requires_explicit_hold_and_model_bounds(setup):
    backend, limits, _ = setup
    with pytest.raises(ValueError, match="explicit --hold"):
        run(setup, extension_rad=.02)
    q0 = [.4] * 20
    q0[10] = -1.0
    with pytest.raises(ValueError, match="URDF joint limits"):
        fixed_extension(q0, .02, limits)
    assert not backend.calls


def test_missing_raw_field_prevents_enable(setup):
    backend, _, _ = setup
    backend.copy_error_count = 1
    report = run(setup, hold=True)
    assert not any(call[0] in ("enable", "configure", "send") for call in backend.calls)
    assert not report["completed"]
    assert any("Raw diagnostics copy failed" in error for error in report["errors"])
    assert report["disabled_confirmed"] and report["closed"]


def test_transient_warning_cleared_in_latest_cache_still_latches_and_disables(setup):
    backend, _, _ = setup
    backend.warning_at = .1
    report = run(setup, hold=True)
    assert all(j.error_code == 0 for j in backend.snapshot().diagnostics.joints)
    assert not report["completed"] and not report["static_completed"]
    assert any("nid=13, code=0x0003" in error for error in report["errors"])
    assert report["raw_error_frame_count"] == 1
    assert report["final_snapshot"]["lifecycle"] == "FAULT"
    assert report["disabled_confirmed"] and report["closed"]
    assert [call[0] for call in backend.calls].count("enable") == 1


def test_enable_jump_cannot_hide_in_ready_baseline_or_send_pose(setup):
    backend, _, _ = setup
    backend.enable_jump = .11
    report = run(setup, hold=True)
    assert report["pre_enable_pose"] == [.4] * 20
    assert report["max_observed_drift_rad"] == pytest.approx(.11)
    assert "q0" not in report
    assert not any(call[0] == "send" for call in backend.calls)
    assert any("Measured motion exceeded" in error for error in report["errors"])
    assert report["disabled_confirmed"] and report["closed"]


def test_frozen_feedback_after_enable_uses_production_watchdog(setup):
    backend, _, _ = setup
    backend.freeze_while_enabled = True
    report = run(setup, hold=True)
    assert not report["completed"]
    assert any("feedback missing or stale" in error for error in report["errors"])
    assert report["final_snapshot"]["lifecycle"] == "FAULT"
    assert report["disabled_confirmed"] and report["closed"]


def test_cached_disabled_feedback_cannot_confirm_post_disable(setup):
    backend, _, _ = setup
    backend.freeze_after_disable = True
    report = run(setup, hold=True)
    assert report["completed"] and not report["disabled_confirmed"]
    assert report["closed"]
    assert all(j.ext_state == 1 for j in backend.snapshot().diagnostics.joints)
    assert any("Fresh advancing" in error for error in report["errors"])
    assert backend.close_at - report["disable_confirmation_boundary"] >= 3.0


def test_stop_flag_uses_owner_cleanup_and_never_retries(setup):
    backend, _, clock = setup
    report = run(setup, hold=True, stop_requested=lambda: clock() >= .1)
    assert not report["completed"] and report["disabled_confirmed"] and report["closed"]
    assert any("Stop signal received" in error for error in report["errors"])
    assert [call[0] for call in backend.calls].count("enable") == 1
    assert backend.calls[-1][0] == "close"


def test_warning_on_final_hold_sleep_cannot_pass(setup):
    backend, _, _ = setup
    backend.warning_at = .005 + 3.0
    report = run(setup, hold=True)
    assert not report["completed"] and not report["static_completed"]
    assert report["raw_error_frame_count"] == 1
    matching = [error for error in report["errors"] if "Raw nonzero joint" in error]
    assert len(matching) == 1 and "nid=13, code=0x0003" in matching[0]
    assert report["disabled_confirmed"] and report["closed"]


@pytest.mark.parametrize("failure", ["warning", "copy"])
def test_raw_failure_during_disable_cannot_turn_into_success(setup, failure):
    backend, _, _ = setup
    disable = backend.disable

    def disable_with_event():
        disable()
        if failure == "warning":
            backend.events.append({"error_joints": [{"nid": 13, "error_code_current": 3}]})
        else:
            backend.copy_error_count = 1

    backend.disable = disable_with_event
    report = run(setup, hold=True)
    assert report["static_completed"] and not report["completed"]
    assert report["disabled_confirmed"] and report["closed"]
    expected = "Raw nonzero joint" if failure == "warning" else "Raw diagnostics copy failed"
    assert sum(expected in error for error in report["errors"]) == 1


def test_measured_drift_on_final_sleep_cannot_pass(setup):
    backend, _, clock = setup
    sleep = backend.sleep

    def sleep_with_final_drift(seconds):
        sleep(seconds)
        if (backend.enabled and backend.first_enabled_feedback_at is not None
                and clock() >= backend.first_enabled_feedback_at + 3.0):
            backend.data = Snapshot(StateSample(clock(), (.51,) * 20, (0.0,) * 20),
                                    backend.data.diagnostics)

    backend.sleep = sleep_with_final_drift
    report = run(setup, hold=True)
    assert not report["completed"] and not report["static_completed"]
    assert report["max_observed_drift_rad"] == pytest.approx(.11)
    assert any("Measured motion exceeded" in error for error in report["errors"])
    assert report["disabled_confirmed"] and report["closed"]


def test_stop_flag_on_final_sleep_cannot_pass(setup):
    backend, _, clock = setup
    report = run(setup, hold=True, stop_requested=lambda: clock() >= .005 + 3.0)
    assert not report["completed"] and not report["static_completed"]
    assert any("Stop signal received" in error for error in report["errors"])
    assert report["disabled_confirmed"] and report["closed"]


def test_raw_evidence_keeps_all_fault_nids_codes_flags_and_comm_fields():
    def joint(nid, error):
        return NS(nid=nid, error_code_current=error, current=.1, vbus_v_fb=12.4,
                  mcu_temp_c_fb=54., comm_response_rate_pct=100, comm_timeout_total=0,
                  status_word=NS(ext_state=2, ext_state_name="Enabled", position_limit_active=False,
                                 velocity_limit_active=True, current_limit_active=False))
    frame = NS(header=NS(seq=123, timestamp_us=456, frame_id=b"r_wrist"), num_joints=3,
               joints=[joint(13, 3), joint(1, 0), joint(23, 7)],
               comm=NS(**{name: i for i, name in enumerate(COMM_FIELDS)}))
    raw = raw_diagnostic_frame(frame, 1.0)
    assert [(j["nid"], j["error_code_current"]) for j in raw["error_joints"]] == [(13, 3), (23, 7)]
    assert raw["header"] == {"seq": 123, "timestamp_us": 456, "frame_id": "b'r_wrist'"}
    assert raw["joints"][0]["status_word"]["velocity_limit_active"] is True
    assert raw["comm"] == {name: i for i, name in enumerate(COMM_FIELDS)}


def test_existing_output_is_never_overwritten_or_followed_by_connection(tmp_path, monkeypatch, setup):
    output = tmp_path / "existing.json"
    output.write_text("sentinel")
    monkeypatch.setattr(probe.JointLimits, "from_urdf", lambda *_args: setup[1])
    monkeypatch.setattr(probe.signal, "signal", lambda *_args: None)

    def forbidden_backend(**_kwargs):
        raise AssertionError("output collision must precede backend construction")

    monkeypatch.setattr(probe, "RecordingBackend", forbidden_backend)
    with pytest.raises(FileExistsError):
        probe.main(["--hold", "--output", str(output)])
    assert output.read_text() == "sentinel"
