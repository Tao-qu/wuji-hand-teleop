#!/usr/bin/env python3
"""Subscribe to existing MANUS/Tracker ROS streams and emit metadata-only JSON.

No device SDK is imported, no process is launched, and no robot command or
service is sent. Poses, calibration data, and SDK log text are never recorded.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys
import time


TRACKER_FRAMES = ("chest", "left_wrist", "right_wrist", "left_arm", "right_arm")


class StreamStats:
    """Bounded-memory statistics, with separate source and receipt clocks."""

    def __init__(self, max_age=.20, future_tolerance=.05, *, static=False):
        self.max_age = max_age
        self.future_tolerance = future_tolerance
        self.static = static
        self.frames = 0
        self.first_receipt = self.last_receipt = None
        self.last_stamp_ns = None
        self.last_sequence = None
        self.last_progress_receipt = self.last_progress_stamp_ns = None
        self._highest_stamp_ns = self._highest_sequence = 0
        self.source_age_sum = 0.
        self.source_age_min = self.source_age_max = None
        self.source_age_count = 0
        self.max_interval = 0.
        self.source_stale_frames = self.source_future_frames = 0
        self.invalid_stamps = self.repeated_stamps = self.reordered_stamps = 0
        self.invalid_sequences = self.repeated_sequences = self.reordered_sequences = 0
        self.sequence_gaps = 0

    def observe(self, *, receipt, ros_now_ns, stamp_ns, sequence=None):
        stamp_progress = sequence_progress = False
        self.frames += 1
        if self.first_receipt is None:
            self.first_receipt = receipt
        if self.last_receipt is not None:
            self.max_interval = max(self.max_interval, receipt - self.last_receipt)
        self.last_receipt = receipt
        if not isinstance(stamp_ns, int) or stamp_ns <= 0:
            self.invalid_stamps += 1
            self.last_stamp_ns = None
        else:
            age = (ros_now_ns - stamp_ns) / 1e9
            stamp_progress = (stamp_ns > self._highest_stamp_ns
                              and -self.future_tolerance <= age < self.max_age)
            self.source_age_sum += age
            self.source_age_count += 1
            self.source_age_min = age if self.source_age_min is None else min(age, self.source_age_min)
            self.source_age_max = age if self.source_age_max is None else max(age, self.source_age_max)
            if not self.static:
                self.source_stale_frames += age >= self.max_age
                self.source_future_frames += age < -self.future_tolerance
            if self.last_stamp_ns is not None:
                self.repeated_stamps += stamp_ns == self.last_stamp_ns
                self.reordered_stamps += stamp_ns < self.last_stamp_ns
            self.last_stamp_ns = stamp_ns
        if sequence is not None:
            if not isinstance(sequence, int) or isinstance(sequence, bool) or sequence <= 0:
                self.invalid_sequences += 1
            else:
                sequence_progress = sequence > self._highest_sequence
                if self.last_sequence is not None:
                    self.repeated_sequences += sequence == self.last_sequence
                    self.reordered_sequences += sequence < self.last_sequence
                    self.sequence_gaps += max(sequence - self.last_sequence - 1, 0)
                self.last_sequence = sequence
        else:
            sequence_progress = True
        if stamp_progress and sequence_progress:
            self.last_progress_receipt = receipt
            self.last_progress_stamp_ns = stamp_ns
            self._highest_stamp_ns = stamp_ns
            if sequence is not None:
                self._highest_sequence = sequence

    def report(self, *, receipt_now, ros_now_ns, duration):
        receipt_age = None if self.last_receipt is None else max(receipt_now - self.last_receipt, 0.)
        source_age = None if self.last_stamp_ns is None else (ros_now_ns - self.last_stamp_ns) / 1e9
        progress_age = None if self.last_progress_receipt is None else max(receipt_now - self.last_progress_receipt, 0.)
        progress_source_age = (None if self.last_progress_stamp_ns is None
                               else (ros_now_ns - self.last_progress_stamp_ns) / 1e9)
        fresh = self.frames > 0 and (self.static or (
            progress_age is not None and progress_age < self.max_age
            and progress_source_age is not None
            and -self.future_tolerance <= progress_source_age < self.max_age))
        span = 0. if self.first_receipt is None else self.last_receipt - self.first_receipt
        return {
            "frames": self.frames,
            "sample_hz": self.frames / duration if duration > 0 else 0.,
            "stream_hz": (self.frames - 1) / span if span > 0 else None,
            "max_receipt_interval_ms": self.max_interval * 1000 if self.frames > 1 else None,
            "last_receipt_age_ms": None if receipt_age is None else receipt_age * 1000,
            "last_source_age_ms": None if source_age is None else source_age * 1000,
            "last_progress_age_ms": None if progress_age is None else progress_age * 1000,
            "source_age_ms": {
                "min": None if self.source_age_min is None else self.source_age_min * 1000,
                "max": None if self.source_age_max is None else self.source_age_max * 1000,
                "mean": self.source_age_sum * 1000 / self.source_age_count if self.source_age_count else None,
            },
            "fresh_at_end": bool(fresh),
            "invalid_stamps": self.invalid_stamps,
            "repeated_stamps": self.repeated_stamps,
            "reordered_stamps": self.reordered_stamps,
            "source_stale_frames": self.source_stale_frames,
            "source_future_frames": self.source_future_frames,
            "last_sequence": self.last_sequence,
            "invalid_sequences": self.invalid_sequences,
            "repeated_sequences": self.repeated_sequences,
            "reordered_sequences": self.reordered_sequences,
            "sequence_gaps": self.sequence_gaps,
        }


def stamp_nanoseconds(stamp):
    if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:
        return None
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def parser():
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("--duration", type=float, default=10., help="Sampling seconds (default: 10)")
    result.add_argument("--mode", choices=("all", "hands", "trackers"), default="all")
    result.add_argument("--sides", nargs="+", choices=("left", "right"), default=["left", "right"])
    result.add_argument("--frames", nargs="+", default=list(TRACKER_FRAMES), help="Expected dynamic TF child frames")
    result.add_argument("--max-age", type=float, default=.20, help="Source/receipt freshness limit in seconds")
    result.add_argument("--output", type=Path, help="Write JSON here; otherwise print only JSON to stdout")
    result.add_argument("--require-fresh", action="store_true", help="Exit 1 unless every expected stream is fresh at the end")
    return result


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    if "--help" in argv or "-h" in argv:
        parser().parse_args(argv)
        return 0
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
    from rclpy.utilities import remove_ros_args

    args = parser().parse_args(remove_ros_args(["sample_ros_inputs", *argv])[1:])
    if not math.isfinite(args.duration) or args.duration <= 0:
        parser().error("--duration must be finite and positive")
    if not math.isfinite(args.max_age) or args.max_age <= 0:
        parser().error("--max-age must be finite and positive")
    sides = tuple(dict.fromkeys(args.sides))
    frames = tuple(dict.fromkeys(name.lstrip("/") for name in args.frames))
    rclpy.init(args=["sample_ros_inputs", *argv])
    node = Node("teleop_input_sampler", enable_rosout=False, start_parameter_services=False)
    qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT,
                     history=HistoryPolicy.KEEP_LAST)
    hand_stats = {side: StreamStats(args.max_age) for side in sides} if args.mode != "trackers" else {}
    hand_topics = {side: set() for side in hand_stats}
    tf_stats = {frame: StreamStats(args.max_age) for frame in frames} if args.mode != "hands" else {}
    tf_parents = {frame: set() for frame in tf_stats}
    static_stats = {}
    unknown_side_frames = 0

    def observe_hand(message, topic):
        nonlocal unknown_side_frames
        side = message.side.lower()
        if side not in hand_stats:
            if side not in ("left", "right"):
                unknown_side_frames += 1
            return
        hand_topics[side].add(topic)
        hand_stats[side].observe(receipt=time.monotonic(), ros_now_ns=node.get_clock().now().nanoseconds,
                                 stamp_ns=stamp_nanoseconds(message.header.stamp), sequence=message.sequence)

    def observe_tf(message, *, static=False):
        receipt, ros_ns = time.monotonic(), node.get_clock().now().nanoseconds
        for transform in message.transforms:
            child = transform.child_frame_id.lstrip("/")
            parent = transform.header.frame_id.lstrip("/")
            if static:
                key = f"{parent} -> {child}"
                stats = static_stats.setdefault(key, StreamStats(args.max_age, static=True))
            else:
                if child not in tf_stats:
                    continue
                tf_parents[child].add(parent)
                stats = tf_stats[child]
            stats.observe(receipt=receipt, ros_now_ns=ros_ns,
                          stamp_ns=stamp_nanoseconds(transform.header.stamp))

    started = time.monotonic()
    started_utc = datetime.now(timezone.utc).isoformat()
    interrupted = False
    try:
        if hand_stats:
            from manus_ros2_msgs.msg import ManusGlove
            for topic in ("/manus_glove_0", "/manus_glove_1"):
                node.create_subscription(ManusGlove, topic, lambda msg, t=topic: observe_hand(msg, t), qos)
        if tf_stats:
            from tf2_msgs.msg import TFMessage
            node.create_subscription(TFMessage, "/tf", observe_tf, qos)
            static_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                                    durability=DurabilityPolicy.TRANSIENT_LOCAL)
            node.create_subscription(TFMessage, "/tf_static", lambda msg: observe_tf(msg, static=True), static_qos)
        deadline = started + args.duration
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=min(.2, max(deadline - time.monotonic(), 0.)))
        interrupted = not rclpy.ok()
    except (KeyboardInterrupt, ExternalShutdownException):
        interrupted = True
    finally:
        ended, ros_ns = time.monotonic(), node.get_clock().now().nanoseconds
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    duration = ended - started
    report_args = dict(receipt_now=ended, ros_now_ns=ros_ns, duration=duration)
    hands = {side: dict(stats.report(**report_args), topics=sorted(hand_topics[side]))
             for side, stats in hand_stats.items()}
    trackers = {child: dict(stats.report(**report_args), parents=sorted(tf_parents[child]))
                for child, stats in tf_stats.items()}
    fresh = all(item["fresh_at_end"] for item in [*hands.values(), *trackers.values()])
    report = {
        "started_at_utc": started_utc, "duration_sec": duration, "interrupted": interrupted,
        "mode": args.mode, "max_age_sec": args.max_age, "freshness_ok": fresh,
        "manus": hands, "unknown_manus_side_frames": unknown_side_frames,
        "tracker_tf": trackers,
        "static_tf": {key: stats.report(**report_args) for key, stats in sorted(static_stats.items())},
        "notes": ["Metadata only; no device SDK calls, pose recording, commands, or services.",
                  "Source freshness assumes synchronized ROS clocks.",
                  "Static TF is latched; its acquisition age is not a freshness failure.",
                  "Sequence gaps can reflect dropped frames or latest-frame sampling, not only transport loss."],
    }
    output = json.dumps(report, indent=2, allow_nan=False) + "\n"
    if args.output:
        destination = args.output.expanduser()
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_text(output)
    else:
        print(output, end="")
    return 1 if args.require_fresh and (not fresh or interrupted) else 0


if __name__ == "__main__":
    raise SystemExit(main())
