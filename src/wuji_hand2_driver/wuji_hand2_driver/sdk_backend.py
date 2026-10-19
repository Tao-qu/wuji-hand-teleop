"""Single-owner wrapper around official wuji-sdk 2026.10.12.

Based on wuji-technology/wuji-sdk examples/python/wuji_hand_2/{0,2}.*.
SDK callbacks only validate/copy stream data into caches. All device RPCs and
command sends belong to the driver's one worker thread, never to a callback.
"""
from dataclasses import dataclass
import fcntl
import hashlib
from importlib.metadata import version
import math
import os
import re
import tempfile
import threading
import time

from .joint_mapping import finite_vector, ordered_entries


@dataclass(frozen=True)
class StateSample:
    received_at: float
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


class DeviceOwner:
    """Prevent two copies of this adapter claiming one SN on this host."""

    def __init__(self, serial_number):
        digest = hashlib.sha256(serial_number.encode()).hexdigest()[:24]
        self.path = os.path.join(tempfile.gettempdir(), f"wuji-hand2-{os.getuid()}-{digest}.lock")
        self.fd = None

    def acquire(self):
        fd = os.open(self.path, os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
        try:
            if os.fstat(fd).st_uid != os.getuid():
                raise RuntimeError("device lock has a different owner")
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except Exception:
            os.close(fd)
            raise RuntimeError("another Hand 2 adapter already owns this serial number")
        self.fd = fd

    def release(self):
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None
        # Do not unlink: another waiter may already have opened this inode.


class SdkBackend:
    def __init__(self, serial_number, expected_side, expected_firmware="2.5.1",
                 read_only=False, sdk=None, monotonic=time.monotonic):
        if not serial_number or serial_number.startswith("YOUR_"):
            raise ValueError("an explicit serial_number is required")
        if expected_side not in ("left", "right"):
            raise ValueError("expected_side must be left or right")
        self.serial_number = serial_number
        self.expected_side = expected_side
        self.expected_firmware = expected_firmware
        self.read_only = read_only
        self.firmware_version = ""
        self._sdk = sdk
        self._clock = monotonic
        self._owner = DeviceOwner(serial_number)
        self._thread_id = None
        self._hand = None
        self._publisher = None
        self._subscriptions = []
        self._cache_lock = threading.Lock()
        self._state = None
        self._diagnostics = None
        self._stream_errors = {}
        self._last_source_stamp = {}
        self._closed = False

    def _check_thread(self):
        if threading.get_ident() != self._thread_id:
            raise RuntimeError("SDK operation attempted outside the device owner thread")

    def _writable(self):
        self._check_thread()
        if self.read_only:
            raise RuntimeError("read_only forbids device writes")
        if self._hand is None:
            raise RuntimeError("device is disconnected")

    def connect(self):
        self._thread_id = threading.get_ident()
        if self._sdk is None:
            if version("wuji-sdk") != "2026.10.12":
                raise RuntimeError("this adapter requires wuji-sdk==2026.10.12")
            import wuji_sdk
            self._sdk = wuji_sdk
        self._owner.acquire()
        try:
            manager = self._sdk.SdkManager.instance()
            # No first-device or handedness-only fallback: the configured SN is mandatory.
            name = f"teleop_{self.expected_side}_{hashlib.sha256(self.serial_number.encode()).hexdigest()[:8]}"
            self._hand = manager.connect(sn=self.serial_number, device_name=name)
            if not isinstance(self._hand, self._sdk.WujiHand2):
                raise RuntimeError("configured serial number is not a Wuji Hand 2")
            if self._hand.serial_number != self.serial_number:
                raise RuntimeError("connected device serial number differs from configuration")
            # The SDK's final HandednessResource override returns a string,
            # unlike RetargetSession's enum parameter. Enum __str__ is also
            # left/right, but compare normalized text, never enum identity.
            physical_side = str(self._hand.handedness().get()).strip().lower()
            if physical_side not in ("left", "right") or physical_side != self.expected_side:
                raise RuntimeError("physical handedness differs from expected_side")
            info = self._hand.info
            self.firmware_version = str(info.firmware_version) if info is not None else ""
            match = re.search(r"(?<!\d)(\d+\.\d+\.\d+)(?!\d)", self.firmware_version)
            if not match or match.group(1) != self.expected_firmware:
                raise RuntimeError(
                    f"firmware mismatch: expected {self.expected_firmware}, got {self.firmware_version!r}")
            self._subscriptions.append(self._hand.joint_states().subscribe_with_callback(
                self._on_states, lambda error: self._on_error("states", error)))
            self._subscriptions.append(self._hand.joint_diagnostics().subscribe_with_callback(
                self._on_diagnostics, lambda error: self._on_error("diagnostics", error)))
            if not self.read_only:
                # Connecting must never inherit an armed state from an earlier session.
                self._hand.disable()
                self._publisher = self._hand.joint_command().publish()
        except Exception:
            self.close()
            raise

    def _accept_stamp(self, stream, frame):
        stamp = (int(frame.header.timestamp_us), int(frame.header.seq))
        previous = self._last_source_stamp.get(stream)
        if previous is not None and stamp <= previous:
            return False
        self._last_source_stamp[stream] = stamp
        return True

    def _on_error(self, stream, error):
        with self._cache_lock:
            self._stream_errors[stream] = str(error)

    def _on_states(self, frame):
        try:
            entries = ordered_entries(frame.joints)
            positions = finite_vector([j.position for j in entries], "feedback positions")
            velocities = finite_vector([j.velocity for j in entries], "feedback velocities")
            with self._cache_lock:
                if self._closed or not self._accept_stamp("states", frame):
                    return
                self._state = StateSample(self._clock(), positions, velocities)
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
            if not all(math.isfinite(v) for j in joints
                       for v in (j.current_a, j.temperature_c, j.voltage_v)):
                raise ValueError("non-finite diagnostic values")
            with self._cache_lock:
                if self._closed or not self._accept_stamp("diagnostics", frame):
                    return
                self._diagnostics = DiagnosticSample(self._clock(), joints)
                self._stream_errors.pop("diagnostics", None)
        except Exception as exc:
            self._on_error("diagnostics", exc)

    def snapshot(self):
        with self._cache_lock:
            return Snapshot(self._state, self._diagnostics,
                            "; ".join(f"{k}: {v}" for k, v in self._stream_errors.items()))

    def configure(self, kp, kd, effort_limit):
        self._writable()
        self._hand.effort_limit().set(effort_limit)
        self._hand.mit_params().set((kp, kd))

    def enable(self):
        self._writable()
        self._hand.enable()

    def disable(self):
        self._writable()
        self._hand.disable()

    def emergency_stop(self):
        self._writable()
        self._hand.emergency_stop()

    def clear_fault(self):
        self._writable()
        self._hand.clear_fault()

    def send(self, positions):
        self._writable()
        self._publisher.send([self._sdk.JointCommand(p, 0.0, 0.0) for p in positions])

    def close(self):
        self._check_thread()
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
        self._owner.release()
        if errors:
            raise RuntimeError("SDK cleanup: " + "; ".join(errors))
