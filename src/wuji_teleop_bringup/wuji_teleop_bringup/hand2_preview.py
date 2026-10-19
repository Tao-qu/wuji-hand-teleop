"""Visualize MANUS hand targets and Tracker poses on preview topics."""
from __future__ import annotations

from wujihand_output.keypoints import manus_to_keypoints
from wujihand_output.retargeting import Hand2Retargeter
from wuji_hand2_driver.joint_mapping import joint_names


TRACKERS = ("chest", "left_wrist", "right_wrist", "left_arm", "right_arm")
STATUS_COLORS = {
    "RECEIVED": (0.2, 1.0, 0.35, 1.0),
    "NO_INPUT": (1.0, 0.75, 0.1, 1.0),
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
        elif state == "RECEIVED":
            meaning = "LAST TARGET PREVIEW"
        else:
            meaning = "LAST TARGET - NOT LIVE"
        result.append({"id": marker_id, "position": (x, 0.0, 1.38),
                       "text": f"{side.upper()} HAND: {state}\n{meaning}\nNOT MEASURED HARDWARE",
                       "color": STATUS_COLORS[state], "height": 0.022})
    states = {name: by_name[f"tracker/{name}"][0] for name in TRACKERS}
    worst = next(state for state in ("INVALID", "NO_INPUT", "RECEIVED")
                 if state in states.values())
    result.append({"id": 2, "position": (0.0, 0.0, 0.80), "height": 0.020,
                   "text": (f"TRACKER INPUTS - AXES ONLY\nChest: {states['chest']}\n"
                            f"L wrist: {states['left_wrist']} | R wrist: {states['right_wrist']}\n"
                            f"L arm: {states['left_arm']} | R arm: {states['right_arm']}\n"
                            "No arm model or actuator feedback"),
                   "color": STATUS_COLORS[worst]})
    return result


class HandPreview:
    """Convert the latest MANUS frame into hand preview targets."""

    def __init__(self, side):
        self.names = joint_names(side)
        self.retargeter = Hand2Retargeter(side)
        self.latest = None
        self.state, self.reason = "NO_INPUT", "Waiting for MANUS frames"
        self.sequence = self.stamp_ns = self.accepted = self.published = self.rejected = 0

    def accept(self, message):
        self.latest = manus_to_keypoints(message, flip_y=False)
        self.sequence = message.sequence
        self.stamp_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        self.accepted += 1
        self.state, self.reason = "RECEIVED", "Last MANUS frame"

    def poll(self):
        points, self.latest = self.latest, None
        if points is None:
            return None
        try:
            positions = self.retargeter.retarget(points)
        except Exception as exc:
            self.rejected += 1
            self.state, self.reason = "INVALID", str(exc)
            return None
        self.published += 1
        return positions, self.stamp_ns

    def status(self):
        return self.state, self.reason


class TrackerPreview:
    """Record Tracker inputs for the preview display."""

    def __init__(self):
        self.frames = {name: {"count": 0, "state": "NO_INPUT", "reason": "Waiting for Tracker TF"}
                       for name in TRACKERS}

    def accept(self, transform):
        entry = self.frames.get(transform.child_frame_id)
        if entry is not None:
            entry.update(count=entry["count"] + 1, state="RECEIVED", reason="Last Tracker TF")

    def status(self, name):
        entry = self.frames[name]
        return entry["state"], entry["reason"]


def create_preview_node():
    # Load ROS dependencies when creating the preview node.
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
            self.declare_parameter("control_rate", 120.0)
            rate = float(self.get_parameter("control_rate").value)
            self.hands = {side: HandPreview(side) for side in ("left", "right")}
            self.outputs = {side: self.create_publisher(JointState, f"/preview/{side}/joint_states", 1)
                            for side in self.hands}
            self.trackers = TrackerPreview()
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
                self.hands[side].accept(message)

        def on_tf(self, message):
            for transform in message.transforms:
                self.trackers.accept(transform)

        def update(self):
            for side, hand in self.hands.items():
                result = hand.poll()
                if result is None:
                    continue
                positions, stamp_ns = result
                message = JointState()
                message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
                message.name, message.position = list(hand.names), list(positions)
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
                status.level = (DiagnosticStatus.OK if state == "RECEIVED" else
                                DiagnosticStatus.STALE if state == "NO_INPUT" else DiagnosticStatus.WARN)
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
