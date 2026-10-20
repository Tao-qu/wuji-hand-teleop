"""Synchronized raw Tracker capture in a dedicated OpenVR session."""

from __future__ import annotations

import time
from typing import Any, Mapping

import numpy as np

try:
    import openvr
except ImportError:
    openvr = None

from .openvr_tracker_wrapper import pose_matrix_to_numpy


TRACKER_ROLES = ("chest", "left_wrist", "right_wrist", "left_arm", "right_arm")


class TrackerCapture:
    """Read raw standing-world poses without applying mounting corrections.

    Construction does not initialize OpenVR. Use this class as a context manager
    in its own process, since OpenVR initialization and shutdown are process-wide.
    """

    def __init__(self, tracker_serials: Mapping[str, str]):
        missing = [role for role in TRACKER_ROLES if not tracker_serials.get(role)]
        if missing:
            raise ValueError(f"Missing Tracker serials for: {', '.join(missing)}")
        self.tracker_serials = {role: tracker_serials[role] for role in TRACKER_ROLES}
        if any(not isinstance(serial, str) for serial in self.tracker_serials.values()):
            raise ValueError("Tracker serial numbers must be strings")
        if len(set(self.tracker_serials.values())) != len(TRACKER_ROLES):
            raise ValueError("Each Tracker role must use a different serial number")
        self._vr_system = None
        self._initialized = False
        self._device_indices: dict[str, int | None] = {}

    def __enter__(self) -> TrackerCapture:
        if self._initialized:
            raise RuntimeError("Tracker capture session is already open")
        if openvr is None:
            raise ImportError("The openvr Python package is required for Tracker capture")
        try:
            openvr.init(openvr.VRApplication_Other)
            self._initialized = True
            self._vr_system = openvr.VRSystem()
            self._enumerate_roles()
        except BaseException:
            self.close()
            raise
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> bool:
        self.close()
        return False

    def close(self) -> None:
        """Release this session, including when capture is interrupted."""
        initialized = self._initialized
        self._initialized = False
        self._vr_system = None
        self._device_indices = {}
        if initialized:
            openvr.shutdown()

    def _require_session(self) -> None:
        if self._vr_system is None:
            raise RuntimeError("Open TrackerCapture with a 'with' statement first")

    def _enumerate_roles(self) -> None:
        self._require_session()
        serial_to_index = {}
        for index in range(openvr.k_unMaxTrackedDeviceCount):
            if self._vr_system.getTrackedDeviceClass(index) != openvr.TrackedDeviceClass_GenericTracker:
                continue
            try:
                serial = self._vr_system.getStringTrackedDeviceProperty(
                    index, openvr.Prop_SerialNumber_String
                )
            except Exception:
                continue
            serial_to_index[serial] = index
        self._device_indices = {
            role: serial_to_index.get(serial)
            for role, serial in self.tracker_serials.items()
        }

    def _read_frame(self) -> tuple[dict[str, dict[str, Any]], dict[str, np.ndarray]]:
        # All roles in this frame come from the same OpenVR pose query.
        poses = self._vr_system.getDeviceToAbsoluteTrackingPose(
            openvr.TrackingUniverseStanding, 0, openvr.k_unMaxTrackedDeviceCount
        )
        statuses = {}
        matrices = {}
        for role, serial in self.tracker_serials.items():
            index = self._device_indices.get(role)
            status = {
                "serial": serial,
                "device_index": index,
                "connected": False,
                "pose_valid": False,
                "matrix_valid": False,
                "tracking_result": None,
                "status": "not_found",
            }
            statuses[role] = status
            if index is None or index >= len(poses):
                continue
            pose = poses[index]
            status["connected"] = bool(pose.bDeviceIsConnected)
            status["pose_valid"] = bool(pose.bPoseIsValid)
            status["tracking_result"] = int(pose.eTrackingResult)
            if not status["connected"]:
                status["status"] = "disconnected"
                continue
            if not status["pose_valid"]:
                status["status"] = "invalid_pose"
                continue
            matrix = pose_matrix_to_numpy(pose.mDeviceToAbsoluteTracking).astype(np.float64)
            if not np.isfinite(matrix).all():
                status["status"] = "nonfinite_matrix"
                continue
            rotation = matrix[:3, :3]
            if not (
                np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-3, rtol=0)
                and np.isclose(np.linalg.det(rotation), 1.0, atol=1e-3, rtol=0)
            ):
                status["status"] = "invalid_matrix"
                continue
            status["matrix_valid"] = True
            status["status"] = "valid"
            matrices[role] = matrix
        return statuses, matrices

    def inspect(self) -> dict[str, dict[str, Any]]:
        """Return serial, device index, connection and pose status for each role."""
        self._enumerate_roles()
        statuses, _ = self._read_frame()
        return statuses

    def capture(
        self, duration: float = 5.0, rate_hz: float = 60.0
    ) -> tuple[dict[str, np.ndarray], dict[str, Any]]:
        """Return complete five-Tracker frames and capture statistics.

        Each sample array has shape ``(N, 4, 4)`` in standing-world coordinates.
        Invalid or absent roles discard the entire frame. ``invalid_counts``
        counts each role's invalid frames; ``duration`` is elapsed capture time.
        """
        duration = float(duration)
        rate_hz = float(rate_hz)
        if not np.isfinite(duration) or duration <= 0:
            raise ValueError("Capture duration must be finite and positive")
        if not np.isfinite(rate_hz) or rate_hz <= 0:
            raise ValueError("Capture rate must be finite and positive")
        self._enumerate_roles()
        collected = {role: [] for role in TRACKER_ROLES}
        invalid_counts = {role: 0 for role in TRACKER_ROLES}
        iterations = 0
        valid_frames = 0
        period = 1.0 / rate_hz
        started = time.monotonic()
        deadline = started + duration
        next_sample = started
        while time.monotonic() < deadline:
            statuses, matrices = self._read_frame()
            iterations += 1
            for role, status in statuses.items():
                if status["status"] != "valid":
                    invalid_counts[role] += 1
            if len(matrices) == len(TRACKER_ROLES):
                for role in TRACKER_ROLES:
                    collected[role].append(matrices[role])
                valid_frames += 1
            now = time.monotonic()
            next_sample = max(next_sample + period, now)
            delay = min(next_sample, deadline) - now
            if delay > 0:
                time.sleep(delay)
        elapsed = time.monotonic() - started
        samples = {
            role: np.stack(frames) if frames else np.empty((0, 4, 4), dtype=np.float64)
            for role, frames in collected.items()
        }
        stats = {
            "iterations": iterations,
            "valid_frames": valid_frames,
            "invalid_counts": invalid_counts,
            "duration": elapsed,
            "requested_duration": duration,
            "rate_hz": rate_hz,
        }
        return samples, stats
