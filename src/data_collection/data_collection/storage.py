"""Streaming LeRobot v2.1 episodes with H.264 RGB and lossless uint16 depth."""

import copy
import fcntl
from functools import lru_cache
import json
import os
from pathlib import Path
import select
import shutil
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

    def add(self, values, axes=None):
        values = np.asarray(values, dtype=np.float64)
        low, high = values.min(axis=axes), values.max(axis=axes)
        mean, variance = values.mean(axis=axes), values.var(axis=axes)
        if not self.count:
            self.low, self.high = low, high
            self.mean, self.variance = mean, variance
        else:
            self.low, self.high = np.minimum(self.low, low), np.maximum(self.high, high)
            delta = mean - self.mean
            self.mean = self.mean + delta / (self.count + 1)
            self.variance = (
                self.count * self.variance + variance
                + delta * delta * self.count / (self.count + 1)
            ) / (self.count + 1)
        self.count += 1

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
    def hardware_encoding_available():
        if not shutil.which("gst-launch-1.0") or not shutil.which("gst-inspect-1.0"):
            return False
        for plugin in ("nvv4l2h264enc", "nvvidconv", "rawvideoparse", "h264parse", "mp4mux"):
            result = subprocess.run(
                ["gst-inspect-1.0", plugin], stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL, timeout=5, check=False,
            )
            if result.returncode:
                return False
        return True

    def __init__(self, path, shape, fps):
        height, width, _ = shape
        self.path = path
        self.log_path = path.with_suffix(".log")
        self.log = self.log_path.open("wb")
        if self.hardware_encoding_available():
            # Jetson hardware encoding keeps the five native streams off the CPU.
            bitrate = max(1_000_000, round(width * height * fps * 0.2))
            command = [
                "gst-launch-1.0", "-q", "fdsrc", "fd=0", "!", "rawvideoparse",
                "format=rgb", f"width={width}", f"height={height}", f"framerate={fps}/1",
                "!", "videoconvert", "!", "video/x-raw,format=RGBA", "!", "nvvidconv",
                "!", "video/x-raw(memory:NVMM),format=NV12", "!", "nvv4l2h264enc",
                "control-rate=0", f"bitrate={bitrate}", f"iframeinterval={fps}",
                "insert-sps-pps=true", "!", "h264parse", "!", "mp4mux", "!", "filesink",
                f"location={path}",
            ]
        else:
            command = [
                "ffmpeg", "-hide_banner", "-loglevel", "error", "-nostdin", "-n",
                "-f", "rawvideo", "-pix_fmt", "rgb24", "-video_size", f"{width}x{height}",
                "-framerate", str(fps), "-i", "pipe:0", "-an", "-c:v", "libx264",
                "-preset", "ultrafast", "-crf", "18", "-pix_fmt", "yuv420p",
                "-threads", "2", "-movflags", "+faststart", str(path),
            ]
        self.process = subprocess.Popen(
            command, stdin=subprocess.PIPE, stdout=subprocess.DEVNULL,
            stderr=self.log, bufsize=0,
        )
        os.set_blocking(self.process.stdin.fileno(), False)

    def _error(self):
        return self.log_path.read_text(errors="replace")[-2000:]

    def write(self, array):
        remaining = memoryview(np.ascontiguousarray(array)).cast("B")
        deadline = time.monotonic() + 10.0
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
        if self.process.stdin and not self.process.stdin.closed:
            self.process.stdin.close()
        try:
            result = self.process.wait(timeout=60)
        except subprocess.TimeoutExpired:
            self.abort()
            raise RuntimeError("RGB encoder did not finish") from None
        self.log.close()
        if result:
            raise RuntimeError(f"RGB encoder failed: {self._error()}")

    def abort(self):
        if self.process.poll() is None:
            self.process.kill()
        self.process.wait()
        if self.process.stdin and not self.process.stdin.closed:
            self.process.stdin.close()
        self.log.close()


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
            "frames": 0, "videos": {}, "writer": None, "tables": [], "stats": {},
            "features": None, "finished": False,
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
        candidate["schema"] = _arrow_schema(features)
        candidate["writer"] = pq.ParquetWriter(
            candidate["directory"] / "episode.parquet", candidate["schema"],
            compression="zstd", compression_level=3, use_dictionary=False,
        )
        for key in RGB_KEYS:
            candidate["videos"][key] = _Video(candidate["directory"] / f"{key}.mp4", image_shapes[key], self.fps)

    def append(self, numeric_row, images, timestamps):
        candidate = self._candidate
        if candidate is None or candidate["finished"]:
            raise RuntimeError("Begin an episode before appending frames")
        index = candidate["frames"]
        if "frame_index" in numeric_row and numeric_row["frame_index"] != index:
            raise ValueError("Episode frame indices must be contiguous")
        if "timestamp" in numeric_row and abs(numeric_row["timestamp"] - index / self.fps) > 1e-6:
            raise ValueError("Episode timestamps must follow the 30 Hz frame grid")
        row = {}
        for key, size in NUMERIC_SHAPES.items():
            value = np.asarray(numeric_row[key], dtype=np.float64)
            if value.shape != (size,) or not np.isfinite(value).all():
                raise ValueError(f"{key} must contain {size} finite values")
            row[key] = value
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
            if not isinstance(value, (int, np.integer)) or not 0 < int(value) <= np.iinfo(np.int64).max:
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
        for key, encoder in candidate["videos"].items():
            encoder.write(images[key])
        arrays = []
        for field in candidate["schema"]:
            if field.name == DEPTH_KEY:
                depth = np.ascontiguousarray(images[DEPTH_KEY])
                height, width = depth.shape
                pixels = pa.array(depth.reshape(-1), type=pa.uint16())
                rows = pa.ListArray.from_arrays(np.arange(height + 1, dtype=np.int32) * width, pixels)
                array = pa.ListArray.from_arrays(np.array([0, height], dtype=np.int32), rows)
                array = pa.ExtensionArray.from_storage(field.type, array)
            else:
                value = row[field.name]
                array = pa.array([value.tolist() if isinstance(value, np.ndarray) else value], type=field.type)
            arrays.append(array)
        candidate["tables"].append(pa.Table.from_arrays(arrays, schema=candidate["schema"]))
        if len(candidate["tables"]) >= 10:
            self._flush()
        for key, value in row.items():
            moments = candidate["stats"].setdefault(key, _Moments())
            # Keep every joint independently; scalar statistics retain one dimension.
            moments.add(np.asarray(value)[None, ...], axes=0)
        for key in RGB_KEYS:
            image = images[key]
            step = max(1, max(image.shape[:2]) // 150)
            sample = image[::step, ::step].astype(np.float64) / 255.0
            candidate["stats"].setdefault(key, _Moments()).add(sample, axes=(0, 1))
        candidate["stats"].setdefault(DEPTH_KEY, _Moments()).add(images[DEPTH_KEY])
        candidate["frames"] += 1

    def _flush(self):
        candidate = self._candidate
        if candidate["tables"]:
            candidate["writer"].write_table(pa.concat_tables(candidate["tables"]), row_group_size=10)
            candidate["tables"].clear()

    def finish(self):
        if self._candidate is None:
            raise RuntimeError("No episode is being recorded")
        candidate = self._candidate
        if candidate["finished"]:
            return
        self._flush()
        if candidate["writer"] is not None:
            candidate["writer"].close()
            candidate["writer"] = None
        for encoder in candidate["videos"].values():
            if candidate["frames"]:
                encoder.finish()
            else:
                encoder.abort()
        candidate["finished"] = True

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
            "stats": {key: value.result(video=key in RGB_KEYS) for key, value in candidate["stats"].items()},
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
        for encoder in candidate["videos"].values():
            encoder.abort()
        if candidate["writer"] is not None:
            candidate["writer"].close()
        shutil.rmtree(candidate["directory"])
        self._candidate = None

    def close(self):
        if self._candidate is not None:
            candidate = self._candidate
            for encoder in candidate["videos"].values():
                encoder.abort()
            if candidate["writer"] is not None:
                candidate["writer"].close()
            # Only an explicit discard removes an unfinished candidate.
            self._candidate = None
        if not self._lock.closed:
            fcntl.flock(self._lock, fcntl.LOCK_UN)
            self._lock.close()
