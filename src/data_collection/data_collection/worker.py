"""Jetson-local image capture and LeRobot episode storage."""

import argparse
from collections import deque
from dataclasses import dataclass, field
import json
from pathlib import Path
import queue
import threading
import time

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.task import Future
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage

from data_collection_interfaces.msg import RecordingFrame
from data_collection_interfaces.srv import SessionCommand

from .imaging import image_from_cdr, image_to_numpy, nearest_image
from .configuration import DEFAULT_OUTPUT, validate_output
from .schema import DEPTH_KEY, FPS, ROBOT_SOURCE_STREAMS
from .storage import DatasetStore


CAMERA_TOPICS = {
    "observation.rgb.head_left": "/cameras/head_left/image_raw",
    "observation.rgb.head_right": "/cameras/head_right/image_raw",
    "observation.rgb.left_wrist": "/cameras/left_wrist/image_raw",
    "observation.rgb.right_wrist": "/cameras/right_wrist/image_raw",
    "observation.rgb.d435": "/cameras/d435/color/image_raw",
    "observation.depth.d435": "/cameras/d435/depth/image_rect_raw",
}
NUMERIC_FIELDS = {
    "action.joint_position": "action_joint_position",
    "action.ee_pose": "action_ee_pose",
    "state.joint_position": "state_joint_position",
    "state.ee_pose": "state_ee_pose",
    "state.joint_velocity": "state_joint_velocity",
}


@dataclass
class Request:
    command: str
    payload: dict
    completion: Future
    success: bool = False
    result: dict = field(default_factory=dict)


class RecordingWorker(Node):
    """Own dataset files while keeping image and ROS callbacks off the disk thread."""

    def __init__(self, output=None):
        super().__init__("writer", namespace="data_collection")
        self._output_override = (
            Path(validate_output(output)).expanduser().resolve() if output is not None else None
        )
        self._output = self._output_override or Path(DEFAULT_OUTPUT).expanduser().resolve()
        self._store = DatasetStore(self._output, fps=FPS)
        self._lock = threading.RLock()
        self._state = "idle"
        self._error = ""
        self._next_index = self._store.next_episode_index
        self._written_count = 0
        self._received_count = 0
        self._frames = queue.Queue(maxsize=128)
        self._matched_frames = queue.Queue(maxsize=16)
        self._requests = queue.Queue(maxsize=16)
        self._pending = deque()
        # Retain one second of source history plus two frames while delayed
        # robot samples and the other camera streams reach the matcher.
        self._images = {key: deque(maxlen=FPS + 2) for key in CAMERA_TOPICS}
        self._camera_info = {}
        self._transforms = {}
        self._shutdown = threading.Event()
        self._group = ReentrantCallbackGroup()
        self._frame_group = MutuallyExclusiveCallbackGroup()
        self._input_subscriptions = []
        for key, topic in CAMERA_TOPICS.items():
            self._input_subscriptions.append(self.create_subscription(
                Image, topic, lambda message, key=key: self._image(key, message),
                qos_profile_sensor_data, callback_group=self._group, raw=True,
            ))
            info_topic = topic.rsplit("/", 1)[0] + "/camera_info"
            self._input_subscriptions.append(self.create_subscription(
                CameraInfo, info_topic,
                lambda message, key=key: self._info(key, message),
                qos_profile_sensor_data, callback_group=self._group,
            ))
        self._input_subscriptions.append(self.create_subscription(
            TFMessage, "/tf_static", self._static_tf,
            QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE),
            callback_group=self._group,
        ))
        self._input_subscriptions.append(self.create_subscription(
            RecordingFrame, "/data_collection/frame", self._frame,
            QoSProfile(depth=128, reliability=ReliabilityPolicy.RELIABLE),
            callback_group=self._frame_group,
        ))
        self._service = self.create_service(
            SessionCommand, "/data_collection/command", self._command,
            callback_group=self._group,
        )
        self._thread = threading.Thread(target=self._run, name="episode-storage", daemon=True)
        self._match_thread = threading.Thread(
            target=self._match_run, name="camera-frame-matcher", daemon=True
        )
        self._match_thread.start()
        self._thread.start()
        self.get_logger().info(f"Dataset writer ready: {self._output}")

    def _image(self, key, serialized):
        try:
            message = image_from_cdr(serialized)
        except (TypeError, ValueError) as exc:
            self.get_logger().error(
                f"Invalid serialized image on {key}: {exc}", throttle_duration_sec=5.0
            )
            return
        stamp = message.stamp_ns
        if stamp > 0:
            with self._lock:
                self._images[key].append((stamp, message))

    def _info(self, key, message):
        with self._lock:
            self._camera_info[key] = {
                "frame_id": message.header.frame_id,
                "width": int(message.width), "height": int(message.height),
                "K": list(message.k), "D": list(message.d),
                "R": list(message.r), "P": list(message.p),
                "distortion_model": message.distortion_model,
            }

    def _static_tf(self, message):
        optical_frames = {
            f"{name}_optical_frame"
            for name in ("head_left", "head_right", "left_wrist", "right_wrist")
        }
        with self._lock:
            for transform in message.transforms:
                parent = transform.header.frame_id
                child = transform.child_frame_id
                if not (child.startswith("d435") or child in optical_frames):
                    continue
                value = transform.transform
                self._transforms[child] = {
                    "parent": parent, "child": child,
                    "translation": [value.translation.x, value.translation.y, value.translation.z],
                    "rotation": [value.rotation.x, value.rotation.y, value.rotation.z, value.rotation.w],
                }

    def _frame(self, message):
        with self._lock:
            if self._state not in ("recording", "stopping"):
                return
            try:
                self._frames.put_nowait((message, time.monotonic()))
            except queue.Full:
                self._fail("Recording frame queue filled before frames could be written")

    def _status(self):
        now_ns = self.get_clock().now().nanoseconds
        with self._lock:
            camera_status = {}
            for key, buffer in self._images.items():
                latest = max((item[0] for item in buffer), default=0)
                age = (now_ns - latest) / 1e9 if latest else None
                camera_status[key] = {
                    "source_timestamp_ns": latest, "age_sec": age,
                    "camera_info_received": key in self._camera_info,
                }
            return {
                "state": self._state, "next_episode_index": self._next_index,
                "frame_count": self._written_count,
                "received_frame_count": self._received_count,
                "error": self._error, "output": str(self._output),
                "cameras": camera_status,
                "cameras_ready": all(
                    value["age_sec"] is not None and -0.1 <= value["age_sec"] < 1.0
                    and value["camera_info_received"]
                    for value in camera_status.values()
                ),
            }

    async def _command(self, request, response):
        try:
            payload = json.loads(request.payload_json or "{}")
            if not isinstance(payload, dict):
                raise ValueError("Command payload must be a JSON object")
            if request.command == "status":
                response.success = True
                response.message_json = json.dumps(self._status())
                return response
            if request.command not in ("configure", "start", "stop", "save", "discard"):
                raise ValueError(f"Unknown recording command: {request.command}")
            operation = Request(request.command, payload, Future(executor=self.executor))
            self._requests.put_nowait(operation)
            await operation.completion
            response.success = operation.success
            response.message_json = json.dumps(operation.result)
        except Exception as exc:
            response.success = False
            response.message_json = json.dumps({**self._status(), "message": str(exc)})
        return response

    def _fail(self, message):
        with self._lock:
            if self._state != "error":
                self.get_logger().error(message)
                self._state = "error"
                self._error = message

    def _clear_frames(self):
        with self._lock:
            self._pending.clear()
            for frames in (self._frames, self._matched_frames):
                while True:
                    try:
                        frames.get_nowait()
                    except queue.Empty:
                        break

    def _accept_frames(self):
        # Limit each pass so commands still run when frames are continuously arriving.
        for _ in range(128):
            try:
                message, arrived = self._frames.get_nowait()
            except queue.Empty:
                break
            if int(message.frame_index) != self._received_count:
                raise ValueError(
                    f"Expected recording frame {self._received_count}, received {message.frame_index}"
                )
            self._received_count += 1
            self._pending.append((message, arrived))
        if len(self._pending) > 128:
            raise RuntimeError("Pending recording frames exceeded the storage queue capacity")

    def _match_next(self):
        if not self._pending:
            return False
        message, arrived = self._pending[0]
        target_ns = int(message.timestamp_ns)
        tolerance_ns = (1_000_000_000 + FPS - 1) // FPS
        have_future = all(
            buffer and max(item[0] for item in buffer) >= target_ns
            for buffer in self._images.values()
        )
        waited = time.monotonic() - arrived
        waiting = waited < 0.5
        if not have_future and waiting:
            return False
        matched = {
            key: nearest_image(buffer, target_ns, tolerance_ns)
            for key, buffer in self._images.items()
        }
        missing = [key for key, value in matched.items() if value is None]
        if missing:
            # A newer image can arrive before a matching older image. Allow
            # transport/callback latency without widening source-time alignment.
            if waiting:
                return False
            details = []
            for key in missing:
                stamps = [item[0] for item in self._images[key]]
                if stamps:
                    nearest = min(stamps, key=lambda stamp: (abs(stamp - target_ns), stamp))
                    details.append(
                        f"{key} (nearest={(nearest - target_ns) / 1e6:+.1f} ms, "
                        f"range=[{(min(stamps) - target_ns) / 1e6:+.1f}, "
                        f"{(max(stamps) - target_ns) / 1e6:+.1f}] ms)"
                    )
                else:
                    details.append(f"{key} (no images received)")
            age_ms = (self.get_clock().now().nanoseconds - target_ns) / 1e6
            raise RuntimeError(
                f"Frame {message.frame_index} has no image within one frame period: "
                + ", ".join(details)
                + f"; target age={age_ms:.1f} ms, waited={waited * 1000:.1f} ms"
            )
        try:
            # Keep references now, before encoder or disk work can age the camera
            # history out. ROS messages own their buffers until append finishes.
            self._matched_frames.put_nowait((message, matched))
        except queue.Full as exc:
            raise RuntimeError("Matched camera queue filled before encoding could keep up") from exc
        self._pending.popleft()
        return True

    def _write_next(self):
        try:
            message, matched = self._matched_frames.get_nowait()
        except queue.Empty:
            return False
        timestamps = {
            f"source_timestamp.{key}": int(stamp)
            for key, stamp in zip(ROBOT_SOURCE_STREAMS, message.source_timestamps_ns)
        }
        images = {}
        for key, (stamp, image) in matched.items():
            images[key] = image_to_numpy(image, depth=(key == DEPTH_KEY), copy=False)
            timestamps[f"source_timestamp.{key}"] = stamp
        numeric_row = {
            key: getattr(message, field) for key, field in NUMERIC_FIELDS.items()
        }
        numeric_row["frame_index"] = int(message.frame_index)
        self._store.append(numeric_row, images, timestamps)
        with self._lock:
            self._written_count += 1
        return True

    def _resolve_output(self, value):
        output = validate_output(value)
        return self._output_override or Path(output).expanduser().resolve()

    def _configure(self, payload):
        with self._lock:
            if self._state != "idle":
                raise ValueError("Select a dataset before starting an episode")
        output = self._resolve_output(payload["output"])
        if output == self._output:
            return
        # Open the selected dataset before releasing the previous idle store.
        store = DatasetStore(output, fps=FPS)
        try:
            self._store.close()
        except Exception:
            store.close()
            raise
        self._clear_frames()
        with self._lock:
            self._store = store
            self._output = output
            self._next_index = store.next_episode_index
            self._written_count = self._received_count = 0
            self._error = ""

    def _start(self, payload):
        with self._lock:
            if self._state != "idle":
                raise ValueError(f"Cannot start an episode while writer is {self._state}")
        if self._resolve_output(payload.get("output", str(self._output))) != self._output:
            raise ValueError("The writer dataset changed; restart the recording client")
        if not self._status()["cameras_ready"]:
            raise RuntimeError("All six image and CameraInfo streams are required before recording starts")
        index = int(payload["episode_index"])
        task = str(payload["task"]).strip()
        if not task:
            raise ValueError("A task name is required")
        self._clear_frames()
        self._store.begin(index, task)
        with self._lock:
            shapes = {
                key: (int(buffer[-1][1].height), int(buffer[-1][1].width))
                + (() if key == DEPTH_KEY else (3,))
                for key, buffer in self._images.items()
            }
        try:
            self._store.prepare(shapes)
        except Exception:
            self._store.discard()
            raise
        with self._lock:
            self._written_count = self._received_count = 0
            self._error = ""
            self._state = "recording"

    def _stop(self, payload):
        with self._lock:
            if self._state == "error":
                raise RuntimeError(self._error)
            if self._state != "recording":
                raise ValueError(f"Cannot stop an episode while writer is {self._state}")
            self._state = "stopping"
        expected = int(payload["frame_count"])
        if expected <= 0:
            raise ValueError("An episode must contain at least one frame")
        deadline = time.monotonic() + 5.0
        while not self._shutdown.is_set():
            if self._state == "error":
                raise RuntimeError(self._error)
            if self._received_count > expected:
                raise ValueError(f"Received more recording frames than stop count {expected}")
            self._write_next()
            if self._written_count == expected:
                break
            if time.monotonic() >= deadline:
                raise RuntimeError(
                    f"Stop expected {expected} frames; received {self._received_count}, "
                    f"wrote {self._written_count}"
                )
            time.sleep(0.002)
        if self._shutdown.is_set():
            raise RuntimeError("Recording interrupted while finishing the episode")
        self._store.finish()
        with self._lock:
            self._state = "pending"

    def _save(self):
        with self._lock:
            if self._state != "pending":
                raise ValueError(f"Cannot save an episode while writer is {self._state}")
            self._state = "saving"
            calibration = {
                "intrinsics": dict(self._camera_info),
                "extrinsics": sorted(
                    self._transforms.values(), key=lambda value: (value["parent"], value["child"])
                ),
            }
        self._store.set_camera_calibration(calibration)
        result = self._store.save()
        with self._lock:
            self._next_index = self._store.next_episode_index
            self._state = "idle"
        return result

    def _discard(self):
        with self._lock:
            if self._state not in ("pending", "error"):
                raise ValueError(f"Cannot discard an episode while writer is {self._state}")
        self._store.discard()
        self._clear_frames()
        with self._lock:
            self._written_count = self._received_count = 0
            self._error = ""
            self._state = "idle"

    def _execute(self, operation):
        try:
            extra = None
            if operation.command == "configure":
                self._configure(operation.payload)
            elif operation.command == "start":
                self._start(operation.payload)
            elif operation.command == "stop":
                self._stop(operation.payload)
            elif operation.command == "save":
                extra = self._save()
            elif operation.command == "discard":
                self._discard()
            operation.success = True
            operation.result = self._status()
            if extra is not None:
                operation.result["saved_episode"] = extra
        except Exception as exc:
            if operation.command != "configure" and self._state in ("recording", "stopping", "saving"):
                self._fail(str(exc))
            operation.result = {**self._status(), "message": str(exc)}
        finally:
            operation.completion.set_result(None)

    def _run(self):
        try:
            while not self._shutdown.is_set():
                try:
                    operation = self._requests.get_nowait()
                except queue.Empty:
                    operation = None
                if operation is not None:
                    self._execute(operation)
                if self._state == "recording":
                    try:
                        self._write_next()
                    except Exception as exc:
                        self._fail(str(exc))
                self._shutdown.wait(0.002)
        finally:
            try:
                self._store.close()
            except Exception as exc:
                self.get_logger().error(f"Could not close recording files: {exc}")
            while True:
                try:
                    operation = self._requests.get_nowait()
                except queue.Empty:
                    break
                operation.result = {"message": "Dataset writer stopped"}
                operation.completion.set_result(None)

    def _match_run(self):
        while not self._shutdown.is_set():
            try:
                with self._lock:
                    if self._state in ("recording", "stopping"):
                        self._accept_frames()
                        while self._match_next():
                            pass
            except Exception as exc:
                self._fail(str(exc))
            self._shutdown.wait(0.002)

    def close(self):
        self._shutdown.set()
        self._match_thread.join()
        self._thread.join()


def main(args=None):
    parser = argparse.ArgumentParser(description="Store LeRobot episodes beside the Jetson cameras")
    parser.add_argument("--output", default=None,
                        help="Override the desktop recording.yaml output; paths resolve on this machine")
    options = parser.parse_args(remove_ros_args(args=args)[1:])
    rclpy.init(args=args)
    node = None
    executor = None
    try:
        node = RecordingWorker(options.output)
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if node is not None:
                node.close()
        finally:
            if executor is not None:
                executor.shutdown()
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
