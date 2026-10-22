"""Official Wuji Hand 2 SDK calls and callback caches for the ROS driver."""
from dataclasses import dataclass
import threading
import time

from .joint_mapping import joint_vector, ordered_entries


@dataclass(frozen=True)
class StateSample:
    received_at: float
    received_stamp_ns: int
    positions: tuple
    velocities: tuple


@dataclass(frozen=True)
class JointDiagnostic:
    nid: int
    ext_state: int
    current_a: float
    temperature_c: float
    voltage_v: float
    error_code: int


@dataclass(frozen=True)
class DiagnosticSample:
    received_at: float
    joints: tuple


@dataclass(frozen=True)
class Snapshot:
    state: StateSample | None
    diagnostics: DiagnosticSample | None
    error: str = ""


class SdkBackend:
    def __init__(self, serial_number, expected_side, sdk=None, monotonic=time.monotonic,
                 timestamp_ns=time.time_ns):
        self.serial_number = serial_number
        self.expected_side = expected_side
        self.firmware_version = ""
        self._sdk = sdk
        self._clock = monotonic
        self._timestamp_ns = timestamp_ns
        self._hand = None
        self._publisher = None
        self._subscriptions = []
        self._cache_lock = threading.Lock()
        self._state = None
        self._diagnostics = None
        self._stream_errors = {}
        self._closed = False

    def connect(self):
        if self._sdk is None:
            import wuji_sdk
            self._sdk = wuji_sdk
        try:
            self._hand = self._sdk.SdkManager.instance().connect(
                sn=self.serial_number, device_name=f"teleop_{self.expected_side}")
            info = self._hand.info
            self.firmware_version = str(info.firmware_version) if info is not None else ""
            self._subscriptions.append(self._hand.joint_states().subscribe_with_callback(
                self._on_states, lambda error: self._on_error("states", error)))
            self._subscriptions.append(self._hand.joint_diagnostics().subscribe_with_callback(
                self._on_diagnostics, lambda error: self._on_error("diagnostics", error)))
            self._publisher = self._hand.joint_command().publish()
        except Exception:
            self.close()
            raise

    def _on_error(self, stream, error):
        with self._cache_lock:
            self._stream_errors[stream] = str(error)

    def _on_states(self, frame):
        received_at, received_stamp_ns = self._clock(), self._timestamp_ns()
        try:
            entries = ordered_entries(frame.joints)
            sample = StateSample(received_at, received_stamp_ns,
                                 joint_vector([j.position for j in entries]),
                                 joint_vector([j.velocity for j in entries]))
            with self._cache_lock:
                if not self._closed:
                    self._state = sample
                    self._stream_errors.pop("states", None)
        except Exception as exc:
            self._on_error("states", exc)

    def _on_diagnostics(self, frame):
        try:
            entries = ordered_entries(frame.joints)
            joints = tuple(JointDiagnostic(
                int(j.nid), int(j.status_word.ext_state), float(j.current),
                float(j.mcu_temp_c_fb), float(j.vbus_v_fb), int(j.error_code_current),
            ) for j in entries)
            with self._cache_lock:
                if not self._closed:
                    self._diagnostics = DiagnosticSample(self._clock(), joints)
                    self._stream_errors.pop("diagnostics", None)
        except Exception as exc:
            self._on_error("diagnostics", exc)

    def snapshot(self):
        with self._cache_lock:
            return Snapshot(self._state, self._diagnostics,
                            "; ".join(f"{k}: {v}" for k, v in self._stream_errors.items()))

    def configure(self, kp, kd):
        self._hand.mit_params().set((kp, kd))

    def enable(self):
        self._hand.enable()

    def disable(self):
        self._hand.disable()

    def emergency_stop(self):
        self._hand.emergency_stop()

    def clear_fault(self):
        self._hand.clear_fault()

    def send(self, positions):
        self._publisher.send([self._sdk.JointCommand(p, 0.0, 0.0) for p in positions])

    def close(self):
        with self._cache_lock:
            self._closed = True
        errors = []
        for resource in [self._publisher, *self._subscriptions]:
            if resource is not None:
                try:
                    resource.close()
                except Exception as exc:
                    errors.append(str(exc))
        self._subscriptions = []
        self._publisher = None
        if self._hand is not None:
            try:
                self._hand.disconnect()
            except Exception as exc:
                errors.append(str(exc))
        self._hand = None
        if errors:
            raise RuntimeError("SDK cleanup: " + "; ".join(errors))
