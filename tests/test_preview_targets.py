"""Pure preview-state regressions: fake sessions/messages, no ROS nodes/devices."""
import ast
from pathlib import Path
import sys
from types import SimpleNamespace as NS

import pytest


ROOT = Path(__file__).resolve().parents[1]
for relative in ("src/wuji_teleop_bringup", "src/output_devices/wujihand_output", "src/wuji_hand2_driver"):
    sys.path.insert(0, str(ROOT / relative))

from wuji_teleop_bringup.hand2_preview import HandPreview, TrackerPreview, status_marker_specs
from wujihand_output.keypoints import MEDIAPIPE_TO_MANUS
from wuji_hand2_driver.joint_mapping import JointLimits


class Clock:
    value = 0.0

    def __call__(self):
        return self.value


class FakeRetargeter:
    def __init__(self):
        self.calls = self.resets = 0
        self.positions = [0.3] * 20
        self.on_retarget = lambda: None

    def retarget(self, points):
        self.calls += 1
        self.points = points.copy()
        self.on_retarget()
        return self.positions

    def reset(self):
        self.resets += 1


def glove(sequence=1, stamp_ns=1_000_000_000):
    points = [(0., 0., 0.)]
    for x, y in ((.02, -.03), (.04, -.015), (.045, 0.), (.04, .015), (.035, .03)):
        points.extend((x + .015 * joint, y, .002 * joint) for joint in range(4))
    nodes = [NS(node_id=node_id, pose=NS(position=NS(x=p[0], y=p[1], z=p[2])))
             for node_id, p in zip(MEDIAPIPE_TO_MANUS, points)]
    sec, ns = divmod(stamp_ns, 1_000_000_000)
    return NS(sequence=sequence, side="Left", header=NS(stamp=NS(sec=sec, nanosec=ns)), raw_nodes=nodes)


@pytest.fixture
def preview():
    clock, session = Clock(), FakeRetargeter()
    limits = JointLimits(tuple(f"l_joint_{i}" for i in range(20)), (-1.,) * 20, (2.,) * 20, (1.,) * 20)
    return HandPreview("left", limits, retargeter=session, clock=clock), clock, session


def test_no_input_and_each_source_frame_publish_at_most_once(preview):
    hand, _, session = preview
    assert hand.status()[0] == "NO_INPUT"
    assert hand.poll() is None
    assert hand.accept(glove(), 1_000_000_000)
    positions, stamp_ns = hand.poll()
    assert positions == (0.3,) * 20
    assert stamp_ns == 1_000_000_000
    assert hand.poll() is None
    assert session.calls == 1
    # The production convention preserves negative Y; no reflection is applied.
    assert session.points[1, 1] < 0


def test_stale_input_stops_updates_resets_once_and_recovers(preview):
    hand, clock, session = preview
    assert hand.accept(glove(), 1_000_000_000)
    assert hand.poll() is not None
    clock.value = .21
    assert hand.poll() is None
    assert hand.status()[0] == "STALE"
    assert session.resets == 1
    assert hand.poll() is None
    assert session.resets == 1
    assert hand.accept(glove(2, 1_210_000_000), 1_210_000_000)
    assert hand.poll() is not None


def test_duplicate_source_is_rejected_without_new_target(preview):
    hand, _, session = preview
    assert hand.accept(glove(), 1_000_000_000)
    assert hand.poll() is not None
    assert not hand.accept(glove(), 1_010_000_000)
    assert hand.poll() is None
    assert session.resets == 1
    assert session.calls == 1
    assert hand.status()[0] == "INVALID"


@pytest.mark.parametrize("bad", [[3.] * 20, [float("nan")] * 20, [0.] * 19])
def test_invalid_official_targets_are_not_published(preview, bad):
    hand, _, session = preview
    session.positions = bad
    assert hand.accept(glove(), 1_000_000_000)
    assert hand.poll() is None
    assert hand.published == 0
    assert hand.status()[0] == "INVALID"


def test_source_expiring_during_retarget_is_not_retimestamped(preview):
    hand, clock, session = preview
    session.on_retarget = lambda: setattr(clock, "value", .21)
    assert hand.accept(glove(), 1_000_000_000)
    assert hand.poll() is None
    assert hand.published == 0
    assert hand.status()[0] == "STALE"


def test_queued_source_age_shortens_monotonic_deadline(preview):
    hand, clock, _ = preview
    assert hand.accept(glove(), 1_190_000_000)
    clock.value = .011
    assert hand.poll() is None
    assert hand.status()[0] == "STALE"


def tracker(stamp_ns=1_000_000_000):
    sec, ns = divmod(stamp_ns, 1_000_000_000)
    return NS(child_frame_id="chest", header=NS(frame_id="world", stamp=NS(sec=sec, nanosec=ns)),
              transform=NS(translation=NS(x=0., y=0., z=1.), rotation=NS(x=0., y=0., z=0., w=1.)))


def test_trackers_require_advancing_finite_source_and_expire():
    clock = Clock()
    inputs = TrackerPreview(clock=clock)
    assert all(inputs.status(name)[0] == "NO_INPUT" for name in inputs.frames)
    inputs.accept(tracker(), 1_000_000_000)
    assert inputs.status("chest")[0] == "FRESH"
    clock.value = .21
    assert inputs.status("chest")[0] == "STALE"
    inputs.accept(tracker(), 1_210_000_000)
    assert inputs.status("chest")[0] == "INVALID"
    fresh = tracker(1_210_000_000)
    fresh.transform.rotation.w = float("nan")
    inputs.accept(fresh, 1_210_000_000)
    assert inputs.status("chest")[0] == "INVALID"
    inputs.accept(tracker(1_210_000_000), 1_210_000_000)
    assert inputs.status("chest")[0] == "FRESH"


def test_preview_launch_has_only_input_visualization_and_preview_nodes():
    source = ROOT / "src/wuji_teleop_bringup/launch/wuji_preview.launch.py"
    tree = ast.parse(source.read_text())
    packages = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and node.func.id == "Node":
            package = next(keyword.value for keyword in node.keywords if keyword.arg == "package")
            assert isinstance(package, ast.Constant)
            packages.add(package.value)
    assert packages == {"manus_ros2", "openvr_input", "wuji_teleop_bringup",
                        "robot_state_publisher", "tf2_ros", "rviz2"}


def test_preview_node_has_no_service_clients_or_device_connection_calls():
    source = ROOT / "src/wuji_teleop_bringup/wuji_teleop_bringup/hand2_preview.py"
    tree = ast.parse(source.read_text())
    forbidden = {"create_client", "create_service", "connect", "enable", "set_enabled", "discover"}
    calls = {node.func.attr for node in ast.walk(tree)
             if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)}
    assert not forbidden.intersection(calls)
    assert "WujiHandController(" not in source.read_text()


def marker_snapshots(hand_state="FRESH", targets=1, tracker_state="FRESH"):
    return [
        (f"hand/{side}", (hand_state, "diagnostic reason"), {"preview_targets": targets})
        for side in ("left", "right")
    ] + [
        (f"tracker/{name}", (tracker_state, "diagnostic reason"), {})
        for name in ("chest", "left_wrist", "right_wrist", "left_arm", "right_arm")
    ]


@pytest.mark.parametrize("state", ["STALE", "INVALID"])
def test_visible_nonlive_hand_status_cannot_label_retained_pose_as_current(state):
    fresh = status_marker_specs(marker_snapshots())
    expired = status_marker_specs(marker_snapshots(hand_state=state))
    for current, stale in zip(fresh[:2], expired[:2]):
        assert "CURRENT TARGET PREVIEW" in current["text"]
        assert f": {state}" in stale["text"]
        assert "LAST TARGET - NOT LIVE" in stale["text"]
        assert "CURRENT" not in stale["text"]
        assert "NOT MEASURED HARDWARE" in stale["text"]
        assert stale["color"] != current["color"]
        assert stale["color"][0] == 1.0


def test_visible_no_input_status_and_tracker_summary_match_snapshot():
    snapshots = marker_snapshots(hand_state="NO_INPUT", targets=0)
    snapshots[-1] = ("tracker/right_arm", ("STALE", "deadline expired"), {})
    markers = status_marker_specs(snapshots)
    assert len(markers) == 3
    assert len({marker["id"] for marker in markers}) == 3
    assert all("NO_INPUT" in marker["text"] and "NO TARGET RECEIVED" in marker["text"]
               for marker in markers[:2])
    assert "R arm: STALE" in markers[2]["text"]
    assert "L wrist: FRESH" in markers[2]["text"]
    assert "No arm model or actuator feedback" in markers[2]["text"]
    assert markers[2]["color"][0] == 1.0
