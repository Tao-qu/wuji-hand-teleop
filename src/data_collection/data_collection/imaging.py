"""Convert ROS image storage without changing its resolution or depth values."""

import numpy as np


def image_to_numpy(message, *, depth=False):
    """Return an owned RGB8 or native-endian uint16 array, respecting row padding."""
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
        return values.astype(np.uint16, copy=True)
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
    return values.copy()


def stamp_nanoseconds(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def nearest_image(buffer, timestamp_ns, tolerance_ns):
    """Select the closest source stamp; the earlier image wins an exact tie."""
    if not buffer:
        return None
    selected = min(buffer, key=lambda item: (abs(item[0] - timestamp_ns), item[0]))
    return selected if abs(selected[0] - timestamp_ns) <= tolerance_ns else None
