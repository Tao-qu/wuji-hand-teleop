"""Read existing robot topics and assemble timestamped recording samples.

This node publishes only recording frames. It has no hardware connection,
control publisher, or robot service client.
"""
from bisect import bisect_right
from collections import deque
import math
import threading
import time

import numpy as np
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int8, String

from data_collection_interfaces.msg import RecordingFrame
from data_collection_interfaces.srv import SessionCommand
from .schema import FPS, JOINT_NAMES, ROBOT_SOURCE_STREAMS


SOURCE_STREAMS = ROBOT_SOURCE_STREAMS
JOINT_GROUPS = {
    "left_arm": tuple(JOINT_NAMES[:7]), "right_arm": tuple(JOINT_NAMES[7:14]),
    "left_hand": tuple(JOINT_NAMES[14:34]), "right_hand": tuple(JOINT_NAMES[34:54]),
}
REQUIRED_NODES = {
    "/tracker_arm_retarget", "/tianji_arm_controller",
    "/left_hand/controller", "/right_hand/controller",
}


def stamp_ns(header):
    return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec


class SampleBuffer:
    """Small source-time history, shared by multiple 30 Hz sampling ticks."""

    def __init__(self):
        self.samples = deque(maxlen=1200)

    def add(self, timestamp, value):
        if timestamp <= 0:
            return
        if self.samples and timestamp <= self.samples[-1][0]:
            # Repeated cached SDK feedback is not a new source sample.
            return
        self.samples.append((timestamp, value))

    def at(self, timestamp):
        samples = list(self.samples)
        index = bisect_right([item[0] for item in samples], timestamp) - 1
        if index < 0 or timestamp - samples[index][0] > 250_000_000:
            raise ValueError("no source sample within 250 ms before the recording tick")
        return samples[index]


class RobotRecorder(Node):
    def __init__(self):
        super().__init__("session", namespace="data_collection")
        # Imported only on the desktop: this native model does not connect to a robot.
        from tianji_output.kinematics import TianjiKinematics
        from tianji_output._internal.native_ik_output import call_with_filtered_stdout
        from scipy.spatial.transform import Rotation
        self.kinematics = TianjiKinematics()
        self.filtered_call = call_with_filtered_stdout
        self.rotation = Rotation
        self.lock = threading.RLock()
        self.buffers = {name: SampleBuffer() for name in SOURCE_STREAMS}
        self.lifecycle = {}
        self.input_error = ""
        self.recording_error = ""
        self.active = False
        self.stop_at_ns = None
        self.frame_count = 0
        self.origin_ns = 0
        self.publisher = self.create_publisher(RecordingFrame, "/data_collection/frame", 60)
        self.client = self.create_client(SessionCommand, "/data_collection/command")
        qos = qos_profile_sensor_data
        for side in ("left", "right"):
            for part in ("arm", "hand"):
                group = f"{side}_{part}"
                self.create_subscription(
                    JointState, f"/{group}/joint_states",
                    lambda msg, g=group: self._joints(msg, g, measured=True), qos)
            self.create_subscription(
                JointState, f"/{side}_hand/commanded_joint_states",
                lambda msg, s=side: self._joints(msg, f"{s}_hand", measured=False), qos)
            self.create_subscription(
                PoseStamped, f"/{side}_arm/tcp_target",
                lambda msg, s=side: self._pose(msg, s), qos)
        self.create_subscription(JointState, "/tianji_arm/commanded_joint_states", self._arm_commands, qos)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Int8, "/tianji_arm/lifecycle_state",
                                 lambda msg: self._lifecycle("arms", msg.data), latched)
        for side in ("left", "right"):
            self.create_subscription(String, f"/{side}_hand/lifecycle_state",
                                     lambda msg, s=side: self._lifecycle(s, msg.data), latched)
        self.create_timer(0.01, self._tick)

    def _lifecycle(self, name, value):
        with self.lock:
            self.lifecycle[name] = value

    def _arm_commands(self, message):
        for side in ("left", "right"):
            self._joints(message, f"{side}_arm", measured=False, partial=True)

    def _joints(self, message, group, *, measured, partial=False):
        names = JOINT_GROUPS[group]
        lookup = {name: i for i, name in enumerate(message.name)}
        if not all(name in lookup for name in names):
            if not partial:
                self.input_error = f"{group}: missing named joints"
            return
        try:
            positions = [float(message.position[lookup[name]]) for name in names]
            velocities = [float(message.velocity[lookup[name]]) for name in names] if measured else []
            if not all(math.isfinite(value) for value in positions + velocities):
                raise ValueError("non-finite joint data")
            key = f"{'state' if measured else 'action'}.{group}"
            with self.lock:
                self.buffers[key].add(stamp_ns(message.header), (positions, velocities))
        except (IndexError, ValueError) as exc:
            self.input_error = f"{group}: {exc}"

    def _pose(self, message, side):
        try:
            if message.header.frame_id != f"{side}_base":
                raise ValueError(f"expected {side}_base, got {message.header.frame_id}")
            p, q = message.pose.position, message.pose.orientation
            values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
            if not all(math.isfinite(v) for v in values) or abs(np.linalg.norm(values[3:]) - 1) > 0.001:
                raise ValueError("invalid TCP pose")
            with self.lock:
                self.buffers[f"action.{side}_ee"].add(stamp_ns(message.header), values)
        except ValueError as exc:
            self.input_error = f"{side} TCP target: {exc}"

    def readiness(self):
        with self.lock:
            if self.lifecycle.get("arms") != 2:
                raise ValueError("both arms must be READY in the running teleoperation session")
            for side in ("left", "right"):
                if self.lifecycle.get(side) not in ("READY", "ACTIVE"):
                    raise ValueError(f"{side} hand must be enabled in the running teleoperation session")
            now = self.get_clock().now().nanoseconds - 100_000_000
            for name, buffer in self.buffers.items():
                try:
                    buffer.at(now)
                except ValueError as exc:
                    raise ValueError(f"{name}: {exc}; {self.input_error}") from exc
        names = {f"{namespace.rstrip('/')}/{name}" for name, namespace in self.get_node_names_and_namespaces()}
        missing = REQUIRED_NODES - names
        if missing:
            raise ValueError("start teleoperation first; missing nodes: " + ", ".join(sorted(missing)))

    def start_recording(self):
        with self.lock:
            self.origin_ns = self.get_clock().now().nanoseconds
            self.frame_count = 0
            self.recording_error = ""
            self.stop_at_ns = None
            self.active = True

    def stop_recording(self):
        with self.lock:
            if not self.active:
                return self.frame_count
            self.stop_at_ns = self.get_clock().now().nanoseconds
        # Flush ticks up to the keypress, including the 100 ms sampling delay.
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            with self.lock:
                if not self.active:
                    return self.frame_count
            time.sleep(0.005)
        with self.lock:
            self.active = False
            self.recording_error = "Recording sampling did not drain before stop"
            return self.frame_count

    def make_frame(self, timestamp, index):
        values = {}
        stamps = []
        for stream in SOURCE_STREAMS:
            try:
                stamp, value = self.buffers[stream].at(timestamp)
            except ValueError as exc:
                raise ValueError(f"{stream}: {exc}") from exc
            stamps.append(stamp)
            values[stream] = value
        msg = RecordingFrame()
        msg.frame_index = index
        msg.timestamp_ns = timestamp
        msg.source_timestamps_ns = stamps
        msg.action_joint_position = sum((values[f"action.{group}"][0] for group in JOINT_GROUPS), [])
        msg.state_joint_position = sum((values[f"state.{group}"][0] for group in JOINT_GROUPS), [])
        msg.state_joint_velocity = sum((values[f"state.{group}"][1] for group in JOINT_GROUPS), [])
        msg.action_ee_pose = values["action.left_ee"] + values["action.right_ee"]
        measured_poses = []
        for side in ("left", "right"):
            q = values[f"state.{side}_arm"][0]
            transform = np.asarray(
                self.filtered_call(self.kinematics.forward, side, np.rad2deg(q)), dtype=np.float64)
            if transform.shape != (4, 4):
                raise ValueError(f"{side}: measured TCP forward kinematics failed")
            pose = list(transform[:3, 3] / 1000.0) + list(self.rotation.from_matrix(transform[:3, :3]).as_quat())
            if not np.all(np.isfinite(pose)):
                raise ValueError(f"{side}: invalid measured TCP forward kinematics")
            measured_poses.extend(pose)
        msg.state_ee_pose = measured_poses
        return msg

    def _tick(self):
        with self.lock:
            if not self.active:
                return
            timestamp = self.origin_ns + round(self.frame_count * 1_000_000_000 / FPS)
            if self.stop_at_ns is not None and timestamp >= self.stop_at_ns:
                self.active = False
                return
            now = self.get_clock().now().nanoseconds
            # Read robot data after this source-time tick has reached the desktop.
            if now < timestamp + 100_000_000:
                return
            try:
                frame = self.make_frame(timestamp, self.frame_count)
                self.publisher.publish(frame)
                self.frame_count += 1
            except (ValueError, RuntimeError) as exc:
                self.recording_error = str(exc)
                self.active = False
