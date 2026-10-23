"""Streaming LeRobot v2.1 episodes with H.264 RGB and lossless uint16 depth."""

import copy
from concurrent.futures import ProcessPoolExecutor, ThreadPoolExecutor, wait
import fcntl
from functools import lru_cache
import json
import math
import multiprocessing
import os
from pathlib import Path
import select
import shutil
import signal
import subprocess
import time
import uuid

import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq

from .schema import (
    DEPTH_KEY, FPS, IMAGE_KEYS, NUMERIC_SHAPES, RGB_KEYS, ROBOT_TYPE,
    SOURCE_TIMESTAMP_KEYS, make_features,
)

CHUNK_SIZE = 1000
DATA_PATH = "data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet"
VIDEO_PATH = "videos/chunk-{episode_chunk:03d}/{video_key}/episode_{episode_index:06d}.mp4"


def _json(path, default=None):
    return json.loads(path.read_text()) if path.exists() else default


def _jsonlines(path):
    return [json.loads(line) for line in path.read_text().splitlines() if line] if path.exists() else []


def _atomic_text(path, content):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{uuid.uuid4().hex}.tmp")
    with temporary.open("w") as stream:
        stream.write(content)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)


def _dump(value):
    return json.dumps(value, ensure_ascii=False, allow_nan=False, separators=(",", ":"))


class _Array2DType(pa.ExtensionType):
    """Wire-compatible Hugging Face Array2D without its optional ML dependencies."""

    def __init__(self, shape, dtype):
        self.shape, self.dtype = tuple(shape), dtype
        storage_type = pa.list_(pa.list_(pa.from_numpy_dtype(np.dtype(dtype))))
        super().__init__(storage_type, "datasets.features.features.Array2DExtensionType")

    def __arrow_ext_serialize__(self):
        return json.dumps((self.shape, self.dtype)).encode()

    @classmethod
    def __arrow_ext_deserialize__(cls, storage_type, serialized):
        shape, dtype = json.loads(serialized)
        return cls(shape, dtype)


def _arrow_schema(features):
    """Use the same HF features as LeRobot without importing its training stack."""
    fields, hf_features = [], {}
    for key, feature in features.items():
        dtype, shape = feature["dtype"], feature["shape"]
        if dtype == "video":
            continue
        arrow_type = pa.from_numpy_dtype(np.dtype(dtype))
        value = {"dtype": dtype, "_type": "Value"}
        if shape == [1]:
            hf_features[key] = value
        elif len(shape) == 1:
            arrow_type = pa.list_(arrow_type, shape[0])
            hf_features[key] = {"feature": value, "length": shape[0], "_type": "Sequence"}
        elif len(shape) == 2:
            # Array2D's underlying Arrow storage is list<list<uint16>>.
            arrow_type = _Array2DType(shape, dtype)
            hf_features[key] = {"shape": shape, "dtype": dtype, "_type": "Array2D"}
        else:
            raise ValueError(f"Unsupported recorded feature shape: {key} {shape}")
        fields.append(pa.field(key, arrow_type))
    metadata = {b"huggingface": _dump({"info": {"features": hf_features}}).encode()}
    return pa.schema(fields, metadata=metadata)


class _Moments:
    """Online moments; image/depth summaries reduce pixels, not disk precision."""

    def __init__(self):
        self.count = 0

    def add(self, values, axes=None, count=1):
        values = np.asarray(values)
        if values.dtype != np.uint16:
            values = values.astype(np.float64, copy=False)
        low = np.asarray(values.min(axis=axes), dtype=np.float64)
        high = np.asarray(values.max(axis=axes), dtype=np.float64)
        mean = values.mean(axis=axes, dtype=np.float64)
        variance = values.var(axis=axes, dtype=np.float64)
        if not self.count:
            self.low, self.high = low, high
            self.mean, self.variance = mean, variance
        else:
            self.low, self.high = np.minimum(self.low, low), np.maximum(self.high, high)
            delta = mean - self.mean
            total = self.count + count
            self.mean = self.mean + delta * count / total
            self.variance = (
                self.count * self.variance + count * variance
                + delta * delta * self.count * count / total
            ) / total
        self.count += count

    def result(self, video=False):
        def convert(value):
            array = np.atleast_1d(value)
            if video:
                array = array.reshape(3, 1, 1)
            return array.tolist()
        return {
            "min": convert(self.low), "max": convert(self.high),
            "mean": convert(self.mean), "std": convert(np.sqrt(self.variance)),
            "count": [self.count],
        }


class _Video:
    @staticmethod
    @lru_cache(maxsize=1)
    def _gstreamer():
        try:
            import gi
            gi.require_version("Gst", "1.0")
            from gi.repository import Gst
        except (ImportError, ValueError):
            return None
        Gst.init(None)
        plugins = ("appsrc", "videoconvert", "nvvidconv", "nvv4l2h264enc", "h264parse", "mp4mux", "filesink")
        return Gst if all(Gst.ElementFactory.find(plugin) for plugin in plugins) else None

    @staticmethod
    def hardware_encoding_available():
        return _Video._gstreamer() is not None

    def __init__(self, path, shape, fps):
        height, width, _ = shape
        self.path = path
        self.log_path = path.with_suffix(".log")
        self.log = None
        self.process = None
        self.pipeline = None
        self.source = None
        self.bus = None
        self._closed = False
        self._gst = self._gstreamer()
        self._fps = fps
        self._frame_index = 0
        if self._gst is not None:
            # Feed frames directly to GStreamer, avoiding five large raw-video pipes.
            bitrate = max(1_000_000, round(width * height * fps * 0.2))
            self._queue_bytes = height * width * 3 * 2
            Gst = self._gst
            try:
                self.pipeline = Gst.parse_launch(
                    "appsrc name=source format=time is-live=false block=false "
                    f"max-bytes={self._queue_bytes} ! videoconvert ! "
                    "video/x-raw,format=RGBA ! nvvidconv ! "
                    "video/x-raw(memory:NVMM),format=NV12 ! nvv4l2h264enc "
                    f"control-rate=0 bitrate={bitrate} iframeinterval={fps} "
                    "maxperf-enable=true insert-sps-pps=true ! h264parse ! mp4mux ! filesink name=sink"
                )
                self.source = self.pipeline.get_by_name("source")
                self.source.set_property("caps", Gst.Caps.from_string(
                    f"video/x-raw,format=RGB,width={width},height={height},framerate={fps}/1"
                ))
                self.pipeline.get_by_name("sink").set_property("location", str(path))
                self.bus = self.pipeline.get_bus()
                if self.pipeline.set_state(Gst.State.PLAYING) == Gst.StateChangeReturn.FAILURE:
                    self._check_gstreamer_error()
                    raise RuntimeError("RGB encoder could not start")
                self._check_gstreamer_error()
            except Exception:
                self.abort()
                raise
        else:
            command = [
                "ffmpeg", "-hide_banner", "-loglevel", "error", "-nostdin", "-n",
                "-f", "rawvideo", "-pix_fmt", "rgb24", "-video_size", f"{width}x{height}",
                "-framerate", str(fps), "-i", "pipe:0", "-an", "-c:v", "libx264",
                "-preset", "ultrafast", "-crf", "18", "-pix_fmt", "yuv420p",
                "-threads", "2", "-movflags", "+faststart", str(path),
            ]
            self.log = self.log_path.open("wb")
            try:
                self.process = subprocess.Popen(
                    command, stdin=subprocess.PIPE, stdout=subprocess.DEVNULL,
                    stderr=self.log, bufsize=0,
                )
                os.set_blocking(self.process.stdin.fileno(), False)
            except Exception:
                self.abort()
                raise

    def _error(self):
        return self.log_path.read_text(errors="replace")[-2000:]

    @staticmethod
    def _gstreamer_error(message):
        error, debug = message.parse_error()
        return RuntimeError(f"RGB encoder failed: {error.message}; {debug or ''}")

    def _check_gstreamer_error(self):
        message = self.bus.pop_filtered(self._gst.MessageType.ERROR)
        if message is not None:
            raise self._gstreamer_error(message)

    def write(self, array):
        if self._closed:
            raise RuntimeError("RGB encoder is already closed")
        deadline = time.monotonic() + 10.0
        if self._gst is not None:
            self._check_gstreamer_error()
            # Bound queued raw frames without blocking inside push-buffer on encoder failure.
            while self.source.get_property("current-level-bytes") >= self._queue_bytes:
                self._check_gstreamer_error()
                if time.monotonic() >= deadline:
                    raise RuntimeError("RGB encoder cannot keep up with recorded frames")
                time.sleep(0.001)
            Gst = self._gst
            pixels = np.ascontiguousarray(array)
            owner = pixels
            while isinstance(owner, (np.ndarray, memoryview)):
                parent = owner.base if isinstance(owner, np.ndarray) else owner.obj
                if parent is None:
                    break
                owner = parent
            if isinstance(owner, bytes):
                # Raw ROS images view their serialized bytes after a small header.
                # Wrap that owner and trim the span instead of copying pixels twice.
                offset = (pixels.__array_interface__["data"][0]
                          - np.frombuffer(owner, dtype=np.uint8).__array_interface__["data"][0])
                if not 0 <= offset <= len(owner) - pixels.nbytes:
                    owner, offset = pixels.tobytes(), 0
            else:
                owner, offset = pixels.tobytes(), 0
            # The notify closure keeps the immutable bytes alive until GStreamer
            # releases this read-only memory, without copying a native image.
            buffer = Gst.Buffer.new_wrapped_full(
                Gst.MemoryFlags.READONLY, owner, len(owner), 0, None,
                lambda *args, owner=owner: None,
            )
            buffer.resize(offset, pixels.nbytes)
            buffer.pts = self._frame_index * Gst.SECOND // self._fps
            buffer.duration = (self._frame_index + 1) * Gst.SECOND // self._fps - buffer.pts
            buffer.offset = self._frame_index
            buffer.offset_end = self._frame_index + 1
            result = self.source.emit("push-buffer", buffer)
            if result != Gst.FlowReturn.OK:
                self._check_gstreamer_error()
                raise RuntimeError(f"RGB encoder rejected a frame: {result.value_nick}")
            self._frame_index += 1
            return
        remaining = memoryview(np.ascontiguousarray(array)).cast("B")
        descriptor = self.process.stdin.fileno()
        while remaining:
            if self.process.poll() is not None:
                raise RuntimeError(f"RGB encoder exited: {self._error()}")
            timeout = deadline - time.monotonic()
            if timeout <= 0:
                raise RuntimeError("RGB encoder cannot keep up with recorded frames")
            if not select.select([], [descriptor], [], min(timeout, 0.1))[1]:
                continue
            try:
                written = os.write(descriptor, remaining)
            except BlockingIOError:
                continue
            except BrokenPipeError as error:
                raise RuntimeError(f"RGB encoder failed: {self._error()}") from error
            remaining = remaining[written:]

    def finish(self):
        if self._closed:
            return
        if self._gst is not None:
            Gst = self._gst
            try:
                self._check_gstreamer_error()
                result = self.source.emit("end-of-stream")
                if result != Gst.FlowReturn.OK:
                    self._check_gstreamer_error()
                    raise RuntimeError(f"RGB encoder rejected end of stream: {result.value_nick}")
                message = self.bus.timed_pop_filtered(
                    60 * Gst.SECOND, Gst.MessageType.EOS | Gst.MessageType.ERROR,
                )
                if message is None:
                    raise RuntimeError("RGB encoder did not finish")
                if message.type == Gst.MessageType.ERROR:
                    raise self._gstreamer_error(message)
            finally:
                self.abort()
            return
        if self.process.stdin and not self.process.stdin.closed:
            self.process.stdin.close()
        try:
            result = self.process.wait(timeout=60)
        except subprocess.TimeoutExpired:
            self.abort()
            raise RuntimeError("RGB encoder did not finish") from None
        self.log.close()
        self._closed = True
        if result:
            raise RuntimeError(f"RGB encoder failed: {self._error()}")

    def abort(self):
        if self.pipeline is not None:
            self.pipeline.set_state(self._gst.State.NULL)
            self.pipeline = None
            self.source = None
            self.bus = None
        if self.process is not None:
            if self.process.poll() is None:
                self.process.kill()
            self.process.wait()
            if self.process.stdin and not self.process.stdin.closed:
                self.process.stdin.close()
        if self.log is not None:
            self.log.close()
        self._closed = True


class _BatchWriter:
    """Parquet and statistics owner, created only in the spawned worker."""

    _current = None

    def __init__(self, path, features):
        # Trigger Arrow's optional imports before recording starts.
        pa.array(np.empty(0, dtype=np.uint16), type=pa.uint16())
        self.schema = _arrow_schema(features)
        self.stats = {}
        self.writer = pq.ParquetWriter(
            path, self.schema, compression="zstd", compression_level=1,
            use_dictionary=False, use_byte_stream_split=True,
        )
        self.stats_pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix="statistics")

    @classmethod
    def initialize(cls, path, features):
        # The parent handles Ctrl-C and joins this writer before releasing its lock.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        cls._current = cls(path, features)

    @classmethod
    def ready(cls):
        return cls._current is not None

    @classmethod
    def write(cls, batch):
        cls._current._write(batch)

    @classmethod
    def finish(cls):
        instance = cls._current
        cls.close()
        return {
            key: value.result(video=key in RGB_KEYS)
            for key, value in instance.stats.items()
        }

    @classmethod
    def close(cls):
        instance = cls._current
        if instance is None:
            return
        if instance.stats_pool is not None:
            instance.stats_pool.shutdown(wait=True)
            instance.stats_pool = None
        if instance.writer is not None:
            try:
                instance.writer.close()
            finally:
                instance.writer = None

    def _write(self, batch):
        count = len(batch)
        rows = [entry[0] for entry in batch]
        depth = np.stack([entry[1] for entry in batch])
        stats_job = self.stats_pool.submit(self._update_stats, rows, depth, batch)
        try:
            _, height, width = depth.shape
            arrays = []
            for field in self.schema:
                if field.name == DEPTH_KEY:
                    pixels = pa.array(depth.reshape(-1), type=pa.uint16())
                    image_rows = pa.ListArray.from_arrays(
                        np.arange(count * height + 1, dtype=np.int32) * width, pixels,
                    )
                    array = pa.ListArray.from_arrays(
                        np.arange(count + 1, dtype=np.int32) * height, image_rows,
                    )
                    array = pa.ExtensionArray.from_storage(field.type, array)
                else:
                    array = pa.array([row[field.name] for row in rows], type=field.type)
                arrays.append(array)
            self.writer.write_table(
                pa.Table.from_arrays(arrays, schema=self.schema), row_group_size=10,
            )
        finally:
            # A failed Parquet write must not leave statistics touching this batch.
            wait([stats_job])
        stats_job.result()

    def _update_stats(self, rows, depth, batch):
        count = len(batch)
        for key in rows[0]:
            values = np.asarray([row[key] for row in rows], dtype=np.float64)
            self.stats.setdefault(key, _Moments()).add(values, axes=0, count=count)
        for key in RGB_KEYS:
            samples = np.asarray([entry[2][key] for entry in batch], dtype=np.float64) / 255.0
            self.stats.setdefault(key, _Moments()).add(
                samples, axes=(0, 1, 2), count=count,
            )
        self.stats.setdefault(DEPTH_KEY, _Moments()).add(depth, count=count)


class DatasetStore:
    """One writer and at most one unsaved candidate episode per dataset."""

    def __init__(self, root, fps=FPS):
        if fps != FPS:
            raise ValueError(f"Recording uses {FPS} Hz")
        self.root = Path(root).expanduser().resolve()
        self.root.mkdir(parents=True, exist_ok=True)
        self.fps = fps
        self._lock = (self.root / ".recording.lock").open("a")
        try:
            fcntl.flock(self._lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            self._lock.close()
            raise RuntimeError(f"Another recorder is using {self.root}") from None
        self._candidate = None
        self._calibration = None
        try:
            # Complete cold imports before the writer subscribes to native images.
            pa.array(np.empty(0, dtype=np.uint16), type=pa.uint16())
            _Video._gstreamer()
            self._recover_commit()
            self._load()
        except Exception:
            self.close()
            raise

    def _load(self):
        self.info = _json(self.root / "meta/info.json")
        self.episodes = _jsonlines(self.root / "meta/episodes.jsonl")
        self.tasks = _jsonlines(self.root / "meta/tasks.jsonl")
        self.episode_stats = _jsonlines(self.root / "meta/episodes_stats.jsonl")
        if self.info is None:
            if self.episodes or self.tasks or self.episode_stats or any((self.root / "data").glob("**/*.parquet")):
                raise ValueError("Dataset metadata is incomplete; refusing to replace existing episodes")
            return
        if (self.info["codebase_version"] != "v2.1"
                or self.info["robot_type"] != ROBOT_TYPE or self.info["fps"] != self.fps):
            raise ValueError("Existing dataset must use LeRobot v2.1, tianji-wuji2 and 30 Hz")
        count = self.info["total_episodes"]
        if (len(self.episodes) != count or len(self.episode_stats) != count
                or [e["episode_index"] for e in self.episodes] != list(range(count))
                or [e["episode_index"] for e in self.episode_stats] != list(range(count))
                or sum(e["length"] for e in self.episodes) != self.info["total_frames"]):
            raise ValueError("Existing dataset episode indices or metadata are inconsistent")
        if ([item["task_index"] for item in self.tasks] != list(range(len(self.tasks)))
                or len(self.tasks) != self.info["total_tasks"]):
            raise ValueError("Existing dataset task indices are inconsistent")
        for episode in self.episodes:
            index = episode["episode_index"]
            paths = [self._data_path(index), *(self._video_path(index, key) for key in RGB_KEYS)]
            if any(not (self.root / path).is_file() for path in paths):
                raise ValueError(f"Episode {index} is missing its Parquet or video file")

    @property
    def next_episode_index(self):
        return len(self.episodes)

    @property
    def frame_count(self):
        return self._candidate["frames"] if self._candidate else 0

    @staticmethod
    def _data_path(index):
        return DATA_PATH.format(episode_chunk=index // CHUNK_SIZE, episode_index=index)

    @staticmethod
    def _video_path(index, key):
        return VIDEO_PATH.format(episode_chunk=index // CHUNK_SIZE, episode_index=index, video_key=key)

    def set_camera_calibration(self, data):
        """Keep camera intrinsics/extrinsics once for the dataset."""
        data = json.loads(_dump(data))
        existing = _json(self.root / "meta/camera_calibration.json")
        if existing is not None and existing != data:
            raise ValueError("Camera calibration differs from this dataset; use another dataset directory")
        self._calibration = data

    def begin(self, index, task):
        if self._candidate is not None:
            raise RuntimeError("Save or discard the current episode first")
        if index != self.next_episode_index:
            raise ValueError(f"Next episode must be {self.next_episode_index}; received {index}")
        if not isinstance(task, str) or not task.strip():
            raise ValueError("Set a non-empty task in recording.yaml")
        directory = self.root / ".recording" / f"episode_{index:06d}_{uuid.uuid4().hex}"
        directory.mkdir(parents=True)
        task_index = next((item["task_index"] for item in self.tasks if item["task"] == task), len(self.tasks))
        self._candidate = {
            "index": index, "task": task, "task_index": task_index, "directory": directory,
            "frames": 0, "videos": {}, "rows": [], "stats": {},
            "features": None, "finished": False, "error": None,
            "video_pool": None, "parquet_pool": None, "writes": [],
        }

    def prepare(self, image_shapes):
        """Start encoders before the session begins sending its first frame."""
        if self._candidate is None or self._candidate["frames"]:
            raise RuntimeError("Prepare image dimensions after begin and before the first frame")
        if self._candidate["features"] is not None:
            return
        candidate = self._candidate
        features = make_features({key: list(image_shapes[key]) for key in IMAGE_KEYS})
        if self.info is not None and features != self.info["features"]:
            raise ValueError("Recorded feature names or camera dimensions differ from this dataset")
        candidate["features"] = features
        try:
            candidate["parquet_pool"] = ProcessPoolExecutor(
                max_workers=1, mp_context=multiprocessing.get_context("spawn"),
                initializer=_BatchWriter.initialize,
                initargs=(candidate["directory"] / "episode.parquet", features),
            )
            # Spawn, import and open the child writer before timed frames arrive.
            candidate["parquet_pool"].submit(_BatchWriter.ready).result()
            for key in RGB_KEYS:
                candidate["videos"][key] = _Video(
                    candidate["directory"] / f"{key}.mp4", image_shapes[key], self.fps,
                )
            candidate["video_pool"] = ThreadPoolExecutor(
                max_workers=len(RGB_KEYS), thread_name_prefix="rgb",
            )
        except Exception as error:
            candidate["error"] = error
            self._close_workers(candidate)
            for encoder in candidate["videos"].values():
                encoder.abort()
            raise

    def append(self, numeric_row, images, timestamps):
        candidate = self._candidate
        if candidate is None or candidate["finished"]:
            raise RuntimeError("Begin an episode before appending frames")
        if candidate["error"] is not None:
            raise candidate["error"]
        self._check_writes()
        index = candidate["frames"]
        if "frame_index" in numeric_row and numeric_row["frame_index"] != index:
            raise ValueError("Episode frame indices must be contiguous")
        if "timestamp" in numeric_row and abs(numeric_row["timestamp"] - index / self.fps) > 1e-6:
            raise ValueError("Episode timestamps must follow the 30 Hz frame grid")
        row = {}
        for key, size in NUMERIC_SHAPES.items():
            value = np.asarray(numeric_row[key], dtype=np.float64)
            if value.shape != (size,):
                raise ValueError(f"{key} must contain {size} finite values")
            values = value.tolist()
            if not all(math.isfinite(item) for item in values):
                raise ValueError(f"{key} must contain {size} finite values")
            row[key] = values
        for key in IMAGE_KEYS:
            image = images[key]
            if key == DEPTH_KEY:
                if image.dtype != np.uint16 or image.ndim != 2:
                    raise ValueError("Depth must be a uint16 two-dimensional array")
            elif (image.dtype != np.uint8 or image.ndim != 3 or image.shape[2] != 3
                    or image.shape[0] % 2 or image.shape[1] % 2):
                raise ValueError(f"{key} must be RGB8 with even native width and height for yuv420p")
        for key in SOURCE_TIMESTAMP_KEYS:
            value = timestamps[key] if key in timestamps else timestamps[key.removeprefix("source_timestamp.")]
            if not isinstance(value, (int, np.integer)) or not 0 < int(value) < (1 << 63):
                raise ValueError(f"{key} must be positive int64 nanoseconds")
            row[key] = int(value)
        if candidate["features"] is None:
            self.prepare({key: images[key].shape for key in IMAGE_KEYS})
        elif any(list(images[key].shape) != candidate["features"][key]["shape"] for key in IMAGE_KEYS):
            raise ValueError("Image dimensions changed during the episode")
        row.update({
            "timestamp": index / self.fps, "frame_index": index,
            "episode_index": candidate["index"], "task_index": candidate["task_index"],
            "index": (self.info["total_frames"] if self.info else 0) + index,
        })
        video_jobs = []
        try:
            for key, encoder in candidate["videos"].items():
                video_jobs.append(candidate["video_pool"].submit(encoder.write, images[key]))
            samples = {}
            for key in RGB_KEYS:
                image = images[key]
                step = max(1, max(image.shape[:2]) // 150)
                # Samples own their small buffers; never pin a native RGB image
                # in the Parquet queue through a strided NumPy view.
                samples[key] = image[::step, ::step].copy()
            candidate["rows"].append((row, images[DEPTH_KEY], samples))
            if len(candidate["rows"]) >= 10:
                self._flush()
            # Complete this frame before dispatching another write to any encoder.
            wait(video_jobs)
            for job in video_jobs:
                job.result()
            self._check_writes()
            candidate["frames"] += 1
        except Exception as error:
            candidate["error"] = error
            raise
        finally:
            # Cleanup must never race an encoder still using this frame's pixels.
            wait(video_jobs)

    def _check_writes(self, drain=False):
        candidate = self._candidate
        while candidate["writes"] and (drain or candidate["writes"][0].done()):
            job = candidate["writes"][0]
            try:
                job.result()
            except Exception as error:
                candidate["error"] = error
                raise
            candidate["writes"].pop(0)

    def _flush(self):
        candidate = self._candidate
        self._check_writes()
        if candidate["rows"]:
            # At most two ten-frame groups are in flight, including the active write.
            if len(candidate["writes"]) >= 2:
                candidate["writes"][0].result()
                self._check_writes()
            batch = candidate["rows"]
            candidate["writes"].append(candidate["parquet_pool"].submit(
                _BatchWriter.write, batch,
            ))
            candidate["rows"] = []

    @staticmethod
    def _close_workers(candidate):
        video_pool = candidate["video_pool"]
        if video_pool is not None:
            video_pool.shutdown(wait=True, cancel_futures=True)
            candidate["video_pool"] = None
        parquet_pool = candidate["parquet_pool"]
        if parquet_pool is not None:
            for job in candidate["writes"]:
                job.cancel()
            try:
                # This runs after any uncancellable batch in the ordered worker.
                parquet_pool.submit(_BatchWriter.close).result()
            except Exception:
                # A failed initializer or dead child cannot close its writer;
                # shutdown still joins it before staging files can be removed.
                pass
            finally:
                parquet_pool.shutdown(wait=True, cancel_futures=True)
                candidate["parquet_pool"] = None
        candidate["writes"].clear()

    def finish(self):
        if self._candidate is None:
            raise RuntimeError("No episode is being recorded")
        candidate = self._candidate
        if candidate["finished"]:
            return
        try:
            if candidate["error"] is not None:
                raise candidate["error"]
            self._flush()
            self._check_writes(drain=True)
            if candidate["parquet_pool"] is not None:
                candidate["stats"] = candidate["parquet_pool"].submit(_BatchWriter.finish).result()
            for encoder in candidate["videos"].values():
                if candidate["frames"]:
                    encoder.finish()
                else:
                    encoder.abort()
            candidate["finished"] = True
        except Exception as error:
            candidate["error"] = error
            raise
        finally:
            self._close_workers(candidate)

    def save(self):
        candidate = self._candidate
        if candidate is None or not candidate["finished"]:
            raise RuntimeError("Finish the episode before saving it")
        if not candidate["frames"]:
            raise ValueError("An empty episode cannot be saved; press r to retry")
        index, count = candidate["index"], candidate["frames"]
        if self.info is None:
            info = {
                "codebase_version": "v2.1", "robot_type": ROBOT_TYPE,
                "total_episodes": 0, "total_frames": 0, "total_tasks": 0,
                "total_videos": 0, "total_chunks": 0, "chunks_size": CHUNK_SIZE,
                "fps": self.fps, "splits": {}, "data_path": DATA_PATH,
                "video_path": VIDEO_PATH, "features": candidate["features"],
            }
        else:
            info = copy.deepcopy(self.info)
        tasks = list(self.tasks)
        if candidate["task_index"] == len(tasks):
            tasks.append({"task_index": len(tasks), "task": candidate["task"]})
        episodes = [*self.episodes, {"episode_index": index, "tasks": [candidate["task"]], "length": count}]
        stats = [*self.episode_stats, {
            "episode_index": index,
            "stats": candidate["stats"],
        }]
        info.update({
            "total_episodes": index + 1, "total_frames": info["total_frames"] + count,
            "total_tasks": len(tasks), "total_videos": (index + 1) * len(RGB_KEYS),
            "total_chunks": index // CHUNK_SIZE + 1, "splits": {"train": f"0:{index + 1}"},
        })
        media = [[str(candidate["directory"] / "episode.parquet"), self._data_path(index)]]
        media.extend([str(candidate["directory"] / f"{key}.mp4"), self._video_path(index, key)] for key in RGB_KEYS)
        for _, destination in media:
            if (self.root / destination).exists():
                raise FileExistsError(f"Refusing to overwrite {destination}")
        metadata = {
            "meta/tasks.jsonl": "".join(_dump(item) + "\n" for item in tasks),
            "meta/episodes.jsonl": "".join(_dump(item) + "\n" for item in episodes),
            "meta/episodes_stats.jsonl": "".join(_dump(item) + "\n" for item in stats),
        }
        if self._calibration is not None:
            metadata["meta/camera_calibration.json"] = _dump(self._calibration) + "\n"
        # info.json is the final commit marker; a journal completes interrupted saves.
        metadata["meta/info.json"] = _dump(info) + "\n"
        transaction = {"media": media, "metadata": metadata, "staging": str(candidate["directory"])}
        _atomic_text(self.root / ".recording/commit.json", _dump(transaction))
        self._recover_commit()
        self._candidate = None
        self._load()
        return {"episode_index": index, "length": count, "path": str(self.root / self._data_path(index))}

    def _recover_commit(self):
        journal = self.root / ".recording/commit.json"
        transaction = _json(journal)
        if transaction is None:
            return
        for source, destination in transaction["media"]:
            source, destination = Path(source), self.root / destination
            destination.parent.mkdir(parents=True, exist_ok=True)
            if source.exists():
                if destination.exists():
                    raise FileExistsError(f"Cannot recover save over existing file {destination}")
                os.rename(source, destination)
            elif not destination.is_file():
                raise FileNotFoundError(f"Interrupted save is missing {source}")
        for path, content in transaction["metadata"].items():
            _atomic_text(self.root / path, content)
        shutil.rmtree(transaction["staging"], ignore_errors=True)
        journal.unlink()

    def discard(self):
        candidate = self._candidate
        if candidate is None:
            return
        if (self.root / ".recording/commit.json").exists():
            raise RuntimeError("A save is in progress; reopen the dataset to complete it")
        self._close_workers(candidate)
        for encoder in candidate["videos"].values():
            encoder.abort()
        shutil.rmtree(candidate["directory"])
        self._candidate = None

    def close(self):
        if self._candidate is not None:
            candidate = self._candidate
            self._close_workers(candidate)
            for encoder in candidate["videos"].values():
                encoder.abort()
            # Only an explicit discard removes an unfinished candidate.
            self._candidate = None
        if not self._lock.closed:
            fcntl.flock(self._lock, fcntl.LOCK_UN)
            self._lock.close()
