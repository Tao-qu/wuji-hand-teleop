"""View ROS image storage without changing its resolution or depth values."""

from dataclasses import dataclass
import struct

import numpy as np


@dataclass(frozen=True, slots=True)
class ImageView:
    stamp_ns: int
    frame_id: str
    height: int
    width: int
    encoding: str
    is_bigendian: int
    step: int
    data: memoryview


def image_from_cdr(serialized):
    """Read a CDR1 sensor_msgs/Image while retaining its serialized pixel buffer.

    CDR alignment starts after the four-byte encapsulation header. Only scalar
    fields and strings are decoded; data keeps the original bytes alive.
    """
    raw = memoryview(serialized).cast("B")
    if len(raw) < 4 or bytes(raw[:2]) not in (b"\x00\x00", b"\x00\x01"):
        raise ValueError("Expected a CDR1 serialized sensor_msgs/Image")
    endian = "<" if raw[1] == 1 else ">"
    payload = raw[4:]
    offset = 0

    def scalar(code):
        nonlocal offset
        offset = (offset + 3) & ~3
        if offset + 4 > len(payload):
            raise ValueError("Truncated scalar in serialized Image")
        value = struct.unpack_from(endian + code, payload, offset)[0]
        offset += 4
        return value

    def string():
        nonlocal offset
        length = scalar("I")
        end = offset + length
        if length < 1 or end > len(payload) or payload[end - 1] != 0:
            raise ValueError("Invalid CDR string in serialized Image")
        try:
            value = bytes(payload[offset:end - 1]).decode("utf-8")
        except UnicodeDecodeError as exc:
            raise ValueError("Invalid UTF-8 string in serialized Image") from exc
        offset = end
        return value

    seconds, nanoseconds = scalar("i"), scalar("I")
    frame_id = string()
    height, width = scalar("I"), scalar("I")
    encoding = string()
    if offset >= len(payload):
        raise ValueError("Missing endianness in serialized Image")
    is_bigendian = int(payload[offset])
    offset += 1
    step, length = scalar("I"), scalar("I")
    if offset + length > len(payload):
        raise ValueError("Truncated pixel buffer in serialized Image")
    return ImageView(
        seconds * 1_000_000_000 + nanoseconds, frame_id, height, width,
        encoding, is_bigendian, step, payload[offset:offset + length],
    )


def image_to_numpy(message, *, depth=False, copy=True):
    """Return RGB8/native uint16; copy=False shares compatible pixel storage.

    Padded rows and BGR channel reversal remain strided views. Non-native uint16
    depth still needs a byte-order conversion to preserve its numeric values.
    """
    height, width, step = int(message.height), int(message.width), int(message.step)
    if height <= 0 or width <= 0:
        raise ValueError("Image dimensions must be positive")
    encoding = message.encoding.lower()
    if depth:
        if encoding != "16uc1":
            raise ValueError(f"Depth requires 16UC1, received {message.encoding}")
        if step < width * 2 or len(message.data) < height * step:
            raise ValueError("Depth buffer is shorter than its declared shape/stride")
        dtype = np.dtype(">u2" if message.is_bigendian else "<u2")
        values = np.ndarray(
            (height, width), dtype=dtype, buffer=message.data, strides=(step, 2)
        )
        return values.astype(np.uint16, copy=copy)
    if encoding not in ("rgb8", "bgr8"):
        raise ValueError(f"RGB requires rgb8/bgr8, received {message.encoding}")
    if step < width * 3 or len(message.data) < height * step:
        raise ValueError("RGB buffer is shorter than its declared shape/stride")
    values = np.ndarray(
        (height, width, 3), dtype=np.uint8, buffer=message.data,
        strides=(step, 3, 1),
    )
    if encoding == "bgr8":
        values = values[:, :, ::-1]
    return values.copy() if copy else values


def stamp_nanoseconds(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def latest_image_at_or_before(buffer, timestamp_ns):
    """Select the latest cached source stamp that does not exceed the tick."""
    return max(
        (item for item in buffer if item[0] <= timestamp_ns),
        key=lambda item: item[0],
        default=None,
    )
