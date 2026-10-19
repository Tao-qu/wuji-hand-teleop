"""Actuator-free MANUS targets and Tracker diagnostics; no device transport.

Targets use preview-only topics and original acquisition stamps. Model poses
are desired geometry, not measured hardware or a collision-safety assessment.
"""
from __future__ import annotations

import math
from pathlib import Path
import time

from wujihand_output.keypoints import LatestKeypoints, SourceFrameGate, manus_to_keypoints
from wujihand_output.retargeting import Hand2Retargeter
from wuji_hand2_driver.joint_mapping import JointLimits


TRACKERS = ("chest", "left_wrist", "right_wrist", "left_arm", "right_arm")
STATUS_COLORS = {
    "FRESH": (0.2, 1.0, 0.35, 1.0),
    "NO_INPUT": (1.0, 0.75, 0.1, 1.0),
    "STALE": (1.0, 0.2, 0.15, 1.0),
    "INVALID": (1.0, 0.25, 0.8, 1.0),
}


def status_marker_specs(snapshots):
    """Visible text/color/layout from the same snapshot used by diagnostics."""
    by_name = {label: (state_reason[0], values) for label, state_reason, values in snapshots}
    result = []
    for marker_id, (side, x) in enumerate((("left", -0.22), ("right", 0.22))):
        state, values = by_name[f"hand/{side}"]
        if not values["preview_targets"]:
            meaning = "NO TARGET RECEIVED"
        elif state == "FRESH":
            meaning = "CURRENT TARGET PREVIEW"
        else:
            meaning = "LAST TARGET - NOT LIVE"
        result.append({"id": marker_id, "position": (x, 0.0, 1.38),
                       "text": f"{side.upper()} HAND: {state}\n{meaning}\nNOT MEASURED HARDWARE",
                       "color": STATUS_COLORS[state], "height": 0.022})
    states = {name: by_name[f"tracker/{name}"][0] for name in TRACKERS}
    worst = next(state for state in ("INVALID", "STALE", "NO_INPUT", "FRESH")
                 if state in states.values())
    result.append({"id": 2, "position": (0.0, 0.0, 0.80), "height": 0.020,
                   "text": (f"TRACKER INPUTS - AXES ONLY\nChest: {states['chest']}\n"
                            f"L wrist: {states['left_wrist']} | R wrist: {states['right_wrist']}\n"
                            f"L arm: {states['left_arm']} | R arm: {states['right_arm']}\n"
                            "No arm model or actuator feedback"),
                   "color": STATUS_COLORS[worst]})
    return result


class HandPreview:
    """Pure latest-frame conversion, with injectable session/clock for tests."""

    def __init__(self, side, limits, timeout=0.2, *, retargeter=None, clock=time.monotonic):
        self.side, self.limits, self.timeout, self.clock = side, limits, timeout, clock
        self.retargeter = retargeter if retargeter is not None else Hand2Retargeter(side)
        self.latest = LatestKeypoints(timeout, clock=clock)
        self.gate = SourceFrameGate(timeout)
        self.state, self.reason = "NO_INPUT", "Waiting for MANUS source frames"
        self.expires = None
        self.sequence = self.stamp_ns = self.accepted = self.published = self.rejected = 0

    def accept(self, message, now_ns):
        try:
            age = self.gate.accept(message, now_ns)
            # Production input is explicitly right-handed; preserve chirality.
            points = manus_to_keypoints(message, flip_y=False)
            self.latest.push(points, source_age=age)
        except (TypeError, ValueError, AttributeError) as exc:
            self.latest.invalidate()
            self.state, self.reason = "INVALID", str(exc)
            self.rejected += 1
            return False
        self.expires = self.clock() + self.timeout - age
        self.sequence = message.sequence
        self.stamp_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        self.accepted += 1
        self.state, self.reason = "FRESH", "New source frame; preview targets only"
        return True

    def poll(self):
        points, reset = self.latest.poll()
        if reset:
            self.retargeter.reset()
        if points is None:
            self.status()
            return None
        try:
            positions = self.limits.validate_command(self.retargeter.retarget(points))
            if self.clock() >= self.expires:
                self.latest.invalidate()
                self.state, self.reason = "STALE", "Source expired during retargeting"
                return None
        except Exception as exc:
            self.latest.invalidate()
            self.retargeter.reset()
            self.rejected += 1
            self.state, self.reason = "INVALID", f"Target rejected: {exc}"
            return None
        self.published += 1
        return positions, self.stamp_ns

    def status(self):
        if self.state == "FRESH" and self.expires is not None and self.clock() >= self.expires:
            self.state, self.reason = "STALE", "MANUS source deadline expired; last display is historical"
        return self.state, self.reason


class TrackerPreview:
    """Track source/receipt freshness without a TF lookup or any arm SDK."""

    def __init__(self, timeout=0.2, *, clock=time.monotonic):
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("Tracker timeout must be finite and positive")
        self.timeout, self.clock = timeout, clock
        self.frames = {name: {"stamp": 0, "expires": None, "count": 0,
                              "state": "NO_INPUT", "reason": "Waiting for Tracker TF"}
                       for name in TRACKERS}

    def accept(self, transform, now_ns):
        name = transform.child_frame_id
        if name not in self.frames:
            return
        entry = self.frames[name]
        stamp = transform.header.stamp
        ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        age = (now_ns - ns) / 1e9
        p, q = transform.transform.translation, transform.transform.rotation
        values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        norm = sum(v * v for v in values[3:])
        if (transform.header.frame_id != "world" or stamp.sec < 0
                or not 0 <= stamp.nanosec < 1_000_000_000 or ns <= entry["stamp"]
                or age >= self.timeout or age < -0.05
                or not all(math.isfinite(v) for v in values) or abs(norm - 1.0) > 0.05):
            entry.update(state="INVALID", reason="Rejected frame, stamp, age or pose")
            return
        entry.update(stamp=ns, expires=self.clock() + self.timeout - max(age, 0.0),
                     count=entry["count"] + 1, state="FRESH", reason="Tracker axes only; no arm IK")

    def status(self, name):
        entry = self.frames[name]
        if entry["state"] == "FRESH" and self.clock() >= entry["expires"]:
            entry.update(state="STALE", reason="Tracker source deadline expired")
        return entry["state"], entry["reason"]


def create_preview_node():
    # ROS is imported only by this explicit entrypoint; pure tests use the
    # classes above and never construct a ROS node or a physical hand.
    from ament_index_python.packages import get_package_share_directory
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from manus_ros2_msgs.msg import ManusGlove
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState
    from tf2_msgs.msg import TFMessage
    from visualization_msgs.msg import Marker, MarkerArray

    class PreviewNode(Node):
        def __init__(self):
            super().__init__("hand2_preview", namespace="preview", enable_rosout=False)
            self.declare_parameter("input_timeout", 0.2)
            self.declare_parameter("control_rate", 120.0)
            timeout = float(self.get_parameter("input_timeout").value)
            rate = float(self.get_parameter("control_rate").value)
            if not math.isfinite(rate) or rate <= 0:
                raise ValueError("control_rate must be finite and positive")
            model = Path(get_package_share_directory("wuji_hand2_2_description")) / "urdf"
            self.hands = {side: HandPreview(side, JointLimits.from_urdf(model / f"{side}-ros.urdf", side), timeout)
                          for side in ("left", "right")}
            self.outputs = {side: self.create_publisher(JointState, f"/preview/{side}/joint_states", 1)
                            for side in self.hands}
            self.trackers = TrackerPreview(timeout)
            self.diagnostics = self.create_publisher(DiagnosticArray, "/preview/diagnostics", 5)
            self.status_markers = self.create_publisher(MarkerArray, "/preview/status_markers", 5)
            for topic in ("/preview/manus_glove_0", "/preview/manus_glove_1"):
                self.create_subscription(ManusGlove, topic, self.on_manus, qos_profile_sensor_data)
            self.create_subscription(TFMessage, "/preview/tf", self.on_tf, qos_profile_sensor_data)
            self.create_timer(1.0 / rate, self.update)
            self.create_timer(0.2, self.publish_diagnostics)

        def on_manus(self, message):
            side = message.side.lower()
            if side in self.hands:
                self.hands[side].accept(message, self.get_clock().now().nanoseconds)

        def on_tf(self, message):
            now = self.get_clock().now().nanoseconds
            for transform in message.transforms:
                self.trackers.accept(transform, now)

        def update(self):
            for side, hand in self.hands.items():
                result = hand.poll()
                if result is None:
                    continue
                positions, stamp_ns = result
                message = JointState()
                message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
                message.name, message.position = list(hand.limits.names), list(positions)
                self.outputs[side].publish(message)

        def publish_diagnostics(self):
            message = DiagnosticArray()
            message.header.stamp = self.get_clock().now().to_msg()
            snapshots = [
                *[(f"hand/{side}", hand.status(), {"source_sequence": hand.sequence,
                    "accepted_frames": hand.accepted, "preview_targets": hand.published,
                    "rejected_frames": hand.rejected}) for side, hand in self.hands.items()],
                *[(f"tracker/{name}", self.trackers.status(name),
                   {"accepted_frames": self.trackers.frames[name]["count"]}) for name in TRACKERS],
            ]
            for label, (state, reason), values in snapshots:
                status = DiagnosticStatus()
                status.name, status.hardware_id = f"preview/{label}", "INPUT_AND_TARGET_PREVIEW_ONLY"
                status.level = (DiagnosticStatus.OK if state == "FRESH" else
                                DiagnosticStatus.STALE if state in ("NO_INPUT", "STALE") else DiagnosticStatus.WARN)
                status.message = f"{state}: {reason}"
                status.values = [KeyValue(key=k, value=str(v)) for k, v in values.items()]
                message.status.append(status)
            self.diagnostics.publish(message)
            markers = MarkerArray()
            for spec in status_marker_specs(snapshots):
                marker = Marker()
                marker.header.frame_id, marker.header.stamp = "world", message.header.stamp
                marker.ns, marker.id = "preview_status", spec["id"]
                marker.type, marker.action = Marker.TEXT_VIEW_FACING, Marker.ADD
                marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = spec["position"]
                marker.pose.orientation.w = 1.0
                marker.scale.z, marker.text = spec["height"], spec["text"]
                marker.color.r, marker.color.g, marker.color.b, marker.color.a = spec["color"]
                # A stopped processor must not leave a persistent green label.
                marker.lifetime.sec, marker.lifetime.nanosec = 0, 600_000_000
                markers.markers.append(marker)
            self.status_markers.publish(markers)

    return PreviewNode()


def main(args=None):
    import rclpy
    from rclpy.executors import ExternalShutdownException

    rclpy.init(args=args)
    node = None
    try:
        node = create_preview_node()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
