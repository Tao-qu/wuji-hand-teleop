#!/usr/bin/env python3
"""Right-hand probe using the production backend/safety core. Default: read only.

Source tools/env.sh first. --hold explicitly enables the fixed right serial;
it holds the first READY-measured pose for 3 s. Optional --extension-rad changes
only joint index 10 toward extension after that hold. A 0.10 rad observed-motion
cap relative to pre-enable feedback aborts the probe. --max-velocity selects a
probe-only command slew cap (default 0.1 rad/s; maximum 1.0). No reset, calibration,
homing, automatic retry, ROS transport, or other device is used.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict, replace
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import signal
import threading
import time
from uuid import uuid4

from wuji_hand2_driver.joint_mapping import JointLimits
from wuji_hand2_driver.safety import DriverConfig, HandSafety
from wuji_hand2_driver.sdk_backend import SdkBackend


ROOT = Path(__file__).resolve().parents[1]
RIGHT_SERIAL = "WH2KA01260823036"
CONFIG = DriverConfig(kp=5.0, kd=0.1, effort_limit=0.8, max_velocity=0.1)
MAX_SAMPLES = 4096
MAX_OBSERVED_DRIFT_RAD = 0.10
MAX_EXTENSION_SECONDS = 5.0
COMM_FIELDS = ("age_ms", "tactile_online_mask", "e2e_received", "e2e_lost",
               "e2e_reordered", "e2e_duplicates", "e2e_window_loss_x100", "rpc_total",
               "rpc_retries", "rpc_timeouts", "comm_get_failures", "sdk_dropped")


def json_safe(value):
    if isinstance(value, float) and not math.isfinite(value):
        return str(value)
    if isinstance(value, dict):
        return {str(k): json_safe(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [json_safe(v) for v in value]
    return value


def raw_diagnostic_frame(frame, received_at):
    """Copy typed SDK values only. This function performs no SDK RPC."""
    joints = []
    for joint in frame.joints:
        joints.append({
            "nid": int(joint.nid), "error_code_current": int(joint.error_code_current),
            "current": float(joint.current), "vbus_v_fb": float(joint.vbus_v_fb),
            "mcu_temp_c_fb": float(joint.mcu_temp_c_fb),
            "comm_response_rate_pct": int(joint.comm_response_rate_pct),
            "comm_timeout_total": int(joint.comm_timeout_total),
            "status_word": {name: getattr(joint.status_word, name) for name in
                            ("ext_state", "ext_state_name", "position_limit_active",
                             "velocity_limit_active", "current_limit_active")},
        })
    return json_safe({
        "received_at": received_at,
        "header": {"seq": int(frame.header.seq), "timestamp_us": int(frame.header.timestamp_us),
                   "frame_id": str(frame.header.frame_id)},
        "num_joints": int(frame.num_joints), "joints": joints,
        "error_joints": [j for j in joints if j["error_code_current"]],
        "comm": {name: getattr(frame.comm, name) for name in COMM_FIELDS},
    })


class RecordingBackend(SdkBackend):
    def __init__(self, **kwargs):
        super().__init__(RIGHT_SERIAL, "right", **kwargs)
        self._raw_lock = threading.Lock()
        self.raw_latest = None
        self.raw_events = []
        self.raw_event_count = self.raw_copy_error_count = 0
        self.raw_copy_error = ""

    def _on_diagnostics(self, frame):
        try:
            raw = raw_diagnostic_frame(frame, self._clock())
            with self._raw_lock:
                self.raw_latest = raw
                if raw["error_joints"]:
                    self.raw_event_count += 1
                    # Preserve first occurrence, rather than overwriting it.
                    if len(self.raw_events) < 128:
                        self.raw_events.append(raw)
        except Exception as exc:
            with self._raw_lock:
                self.raw_copy_error_count += 1
                self.raw_copy_error = str(exc)
        # Validation, cache ordering, and fault handling remain production code.
        super()._on_diagnostics(frame)

    def evidence(self):
        with self._raw_lock:
            return {"latest_raw_diagnostics": self.raw_latest,
                    "raw_error_frames": list(self.raw_events),
                    "raw_error_frame_count": self.raw_event_count,
                    "raw_error_frames_truncated": max(0, self.raw_event_count - len(self.raw_events)),
                    "raw_copy_error_count": self.raw_copy_error_count,
                    "raw_copy_error": self.raw_copy_error}


def fixed_extension(q0, amount, limits):
    if not math.isfinite(amount) or not 0 <= amount <= 0.05:
        raise ValueError("extension-rad must be finite and within [0, 0.05]")
    target = list(q0)
    target[10] -= amount
    return limits.validate_command(target)


def probe_config(max_velocity, extension_rad):
    if not math.isfinite(max_velocity) or not 0 < max_velocity <= 1.0:
        raise ValueError("max-velocity must be finite and within (0, 1.0] rad/s")
    if extension_rad > max_velocity * (MAX_EXTENSION_SECONDS - 0.5):
        raise ValueError("extension and max-velocity would exceed the 5 s extension-stage limit")
    return replace(CONFIG, max_velocity=max_velocity)


def run_probe(backend, limits, *, hold=False, extension_rad=0.0, max_velocity=CONFIG.max_velocity,
              clock=time.monotonic, sleep=time.sleep, stop_requested=lambda: False):
    if not math.isfinite(extension_rad) or not 0 <= extension_rad <= 0.05:
        raise ValueError("extension-rad must be finite and within [0, 0.05]")
    if extension_rad and not hold:
        raise ValueError("extension-rad requires explicit --hold")
    config = probe_config(max_velocity, extension_rad)
    if getattr(backend, "read_only", not hold) != (not hold):
        raise ValueError("backend read_only mode must match the explicit hold request")
    core = HandSafety(backend, limits, config, read_only=not hold, monotonic=clock)
    report = {"serial_number": RIGHT_SERIAL, "side": "right", "hold_requested": hold,
              "extension_rad": extension_rad, "config": asdict(config),
              "enable_attempts": 0, "static_completed": False, "extension_completed": False,
              "completed": False, "disabled_confirmed": False, "closed": False,
              "max_observed_drift_rad": 0.0,
              "observed_drift_limit_rad": MAX_OBSERVED_DRIFT_RAD,
              "samples": [], "samples_truncated": 0, "errors": []}
    connected = False
    armed = False
    stage = "connect"
    period = 1.0 / config.command_rate

    def record():
        snapshot = backend.snapshot()
        now = clock()
        row = {"at": now, "stage": stage, "lifecycle": core.lifecycle,
               "fault_reason": core.fault_reason, "snapshot": asdict(snapshot),
               "command_target": core._target, "last_sent": core._last_sent,
               "state_fresh": snapshot.state is not None and 0 <= now - snapshot.state.received_at <= config.feedback_timeout,
               "diagnostics_fresh": snapshot.diagnostics is not None and 0 <= now - snapshot.diagnostics.received_at <= config.diagnostics_timeout}
        report["final_snapshot"] = row
        if len(report["samples"]) < MAX_SAMPLES:
            report["samples"].append(row)
        else:
            report["samples_truncated"] += 1
        return snapshot, now

    def checked_tick():
        check_abort()
        core.tick()
        snapshot, now = record()
        if core.lifecycle == "FAULT":
            raise RuntimeError(core.fault_reason)
        check_abort()
        return snapshot, now

    def require_raw_evidence():
        evidence = backend.evidence()
        raw = evidence.get("latest_raw_diagnostics")
        if evidence.get("raw_copy_error_count", 0):
            raise RuntimeError("Raw diagnostics copy failed: " + evidence.get("raw_copy_error", "unknown error"))
        if (raw is None or raw.get("num_joints") != 20 or len(raw.get("joints", ())) != 20
                or not 0 <= clock() - raw["received_at"] <= config.diagnostics_timeout):
            raise RuntimeError("Fresh complete raw diagnostic evidence is required before enabling")
        if evidence.get("raw_error_frame_count", 0):
            events = evidence.get("raw_error_frames", ())
            joints = events[0].get("error_joints", ()) if events else ()
            detail = "; ".join(f"nid={j['nid']}, code=0x{j['error_code_current']:04X}" for j in joints)
            raise RuntimeError("Raw nonzero joint warning/error event latched: " + detail)

    def check_abort():
        if stop_requested():
            raise RuntimeError("Stop signal received")
        if armed:
            # Copy errors and brief warnings remain latched even if a newer SDK
            # cache frame has already cleared them between owner-loop ticks.
            require_raw_evidence()
            snapshot = backend.snapshot()
            if snapshot.state is not None:
                measured = limits.validate_command(snapshot.state.positions)
                drift = max(abs(q - start) for q, start in zip(measured, report["pre_enable_pose"]))
                report["max_observed_drift_rad"] = max(report["max_observed_drift_rad"], drift)
                if drift > MAX_OBSERVED_DRIFT_RAD:
                    raise RuntimeError(f"Measured motion exceeded diagnostic cap: {drift:.6f} rad > {MAX_OBSERVED_DRIFT_RAD:.2f} rad")

    def observe_for(seconds, target=None):
        deadline = clock() + seconds
        while clock() < deadline:
            check_abort()
            if target is not None:
                accepted, reason = core.command(target, (), clock())
                if not accepted:
                    raise RuntimeError(reason)
            snapshot, now = checked_tick()
            if not hold and snapshot.diagnostics and any(j.ext_state == 2 for j in snapshot.diagnostics.joints):
                raise RuntimeError("Read-only probe observed enabled joints; no hardware action taken")
            sleep(period)

    try:
        check_abort()
        backend.connect()
        connected = True
        core.connected(clock())
        stage = "await_disabled_feedback"
        deadline = clock() + config.startup_timeout
        while True:
            snapshot, now = checked_tick()
            problem = core.feedback_problem(snapshot, now)
            if not problem and all(j.ext_state == 1 for j in snapshot.diagnostics.joints):
                break
            if snapshot.diagnostics and any(j.ext_state == 2 for j in snapshot.diagnostics.joints) and not hold:
                raise RuntimeError("Read-only probe found enabled joints; no hardware action taken")
            if now >= deadline:
                raise RuntimeError(problem or "20 disabled Ready joints were not confirmed")
            sleep(period)
        if not hold:
            stage = "read_only"
            observe_for(3.0)
        else:
            stage = "enable_wait"
            require_raw_evidence()
            baseline = backend.snapshot()
            problem = core.feedback_problem(baseline, clock())
            if problem:
                raise RuntimeError(problem)
            report["pre_enable_pose"] = limits.validate_command(baseline.state.positions)
            report["pre_enable_state_received_at"] = baseline.state.received_at
            report["enable_requested_at"] = clock()
            report["enable_attempts"] += 1
            armed = True
            accepted, reason = core.set_enabled(True)
            if not accepted:
                raise RuntimeError(reason)
            while core.lifecycle != "READY":
                checked_tick()
                if core.lifecycle != "READY":
                    sleep(period)
            # Use precisely the production core's post-enable READY capture.
            # Do not recapture q0 from subsequent measured feedback.
            q0 = limits.validate_command(tuple(core._start_pose))
            report["q0"] = q0
            stage = "static_hold"
            observe_for(3.0, q0)
            report["static_completed"] = True
            if extension_rad:
                target = fixed_extension(q0, extension_rad, limits)
                report["extension_target"] = target
                stage = "extension"
                observe_for(extension_rad / config.max_velocity + 0.5, target)
                report["extension_completed"] = True
        report["completed"] = True
    except (Exception, KeyboardInterrupt) as exc:
        if armed and core.lifecycle != "FAULT":
            # The extra diagnostic guards latch just like production failures;
            # cleanup may confirm disabled feedback but never clears this fault.
            core.fail(str(exc))
        report["errors"].append(f"{type(exc).__name__}: {exc}")
    finally:
        if connected:
            stage = "disable_confirmation" if hold else "read_only_final"
            try:
                if hold:
                    accepted, reason = core.set_enabled(False)
                    report["disable_request_accepted"] = accepted
                    report["disable_request_message"] = reason
                    if not accepted:
                        report["errors"].append(reason)
                # Require new frames after the disable RPC (or observation
                # boundary for read-only), then 0.1 s of advancing Ready data.
                boundary = clock()
                report["disable_confirmation_boundary"] = boundary
                deadline, stable_from = boundary + 3.0, None
                first_state = first_diag = None
                while clock() < deadline:
                    snapshot, now = record()
                    valid = (snapshot.state is not None and snapshot.diagnostics is not None
                             and not snapshot.error
                             and boundary < snapshot.state.received_at <= now
                             and boundary < snapshot.diagnostics.received_at <= now
                             and now - snapshot.state.received_at <= config.feedback_timeout
                             and now - snapshot.diagnostics.received_at <= config.diagnostics_timeout
                             and len(snapshot.diagnostics.joints) == 20
                             and all(j.ext_state == 1 for j in snapshot.diagnostics.joints))
                    if not valid:
                        stable_from = first_state = first_diag = None
                    elif stable_from is None:
                        stable_from = now
                        first_state, first_diag = snapshot.state.received_at, snapshot.diagnostics.received_at
                    elif (now - stable_from >= 0.1 and snapshot.state.received_at - first_state >= 0.1
                          and snapshot.diagnostics.received_at - first_diag >= 0.1):
                        report["disabled_confirmed"] = True
                        report["disabled_confirmed_at"] = now
                        break
                    sleep(period)
                if not report["disabled_confirmed"]:
                    report["errors"].append("Fresh advancing 20-joint disabled Ready confirmation not obtained")
            except (Exception, KeyboardInterrupt) as exc:
                report["errors"].append(f"Cleanup: {type(exc).__name__}: {exc}")
            finally:
                try:
                    backend.close()
                    report["closed"] = True
                except Exception as exc:
                    report["errors"].append(f"Close: {exc}")
        if hasattr(backend, "evidence"):
            report.update(backend.evidence())
    return json_safe(report)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--hold", action="store_true", help="Explicitly enable only the fixed right hand for a measured-pose hold")
    parser.add_argument("--extension-rad", type=float, default=0.0, help="Optional right middle PIP extension, 0..0.05 rad, after the 3 s hold")
    parser.add_argument("--max-velocity", type=float, default=CONFIG.max_velocity,
                        help="Probe-only slew cap in rad/s, finite (0, 1.0], default 0.1; extension stage must fit 5 s")
    parser.add_argument("--output", type=Path, help="New JSON path; an existing file is never overwritten")
    args = parser.parse_args(argv)
    if not math.isfinite(args.extension_rad) or not 0 <= args.extension_rad <= .05:
        parser.error("extension-rad must be finite and within [0, 0.05]")
    if args.extension_rad and not args.hold:
        parser.error("extension-rad requires --hold")
    try:
        probe_config(args.max_velocity, args.extension_rad)
    except ValueError as exc:
        parser.error(str(exc))
    model = ROOT / "src/wuji-description/hand2/hand2_2/body/urdf/right-ros.urdf"
    limits = JointLimits.from_urdf(model, "right")
    output = args.output or ROOT / "work" / f"hand2-hold-{datetime.now(timezone.utc):%Y%m%dT%H%M%S}-{uuid4().hex[:8]}.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    # Claim the artifact before connecting, so a collision cannot occur after motion.
    stop = {"requested": False}

    def request_stop(_signum, _frame):
        # Device operations belong exclusively to the normal owner loop.
        stop["requested"] = True

    previous = {sig: signal.signal(sig, request_stop) for sig in (signal.SIGINT, signal.SIGTERM)}
    try:
        with output.open("x", encoding="utf-8") as stream:
            report = run_probe(RecordingBackend(read_only=not args.hold), limits,
                               hold=args.hold, extension_rad=args.extension_rad,
                               max_velocity=args.max_velocity,
                               stop_requested=lambda: stop["requested"])
            json.dump(report, stream, indent=2, allow_nan=False)
            stream.write("\n")
    finally:
        for sig, handler in previous.items():
            signal.signal(sig, handler)
    print(json.dumps({"output": str(output), "completed": report["completed"],
                      "disabled_confirmed": report["disabled_confirmed"], "errors": report["errors"]}))
    return 0 if report["completed"] and report["disabled_confirmed"] and report["closed"] and not report["errors"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
