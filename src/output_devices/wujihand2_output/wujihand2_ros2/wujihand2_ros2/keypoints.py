"""MANUS skeleton conversion, independent of ROS and hardware."""
from __future__ import annotations

import numpy as np


# MediaPipe order: wrist, thumb, index, middle, ring, pinky.
MEDIAPIPE_TO_MANUS = (
    1, 22, 23, 24, 25, 3, 4, 5, 6, 8, 9, 10, 11,
    13, 14, 15, 16, 18, 19, 20, 21,
)


def manus_to_keypoints(message, *, flip_y: bool = False) -> np.ndarray:
    """Map MANUS IDs to MediaPipe order, retaining zeros for missing nodes.

    The publisher requests right-handed coordinates, so XYZ is preserved by
    default. An explicitly selected flip_y reflects the Y coordinate.
    """
    positions = {}
    for node in message.raw_nodes:
        p = node.pose.position
        positions[node.node_id] = (p.x, -p.y if flip_y else p.y, p.z)
    points = np.zeros((21, 3), dtype=np.float32)
    for index, node_id in enumerate(MEDIAPIPE_TO_MANUS):
        if node_id in positions:
            points[index] = positions[node_id]
    return points
