"""Recorded feature names and ordering, shared by the session and writer."""

FPS = 30
ROBOT_TYPE = "tianji-wuji2"
HAND_SUFFIXES = (
    "thumb_cmc_flex", "thumb_cmc_abd", "thumb_mcp", "thumb_ip",
    "index_finger_mcp_flex", "index_finger_mcp_abd", "index_finger_pip", "index_finger_dip",
    "middle_finger_mcp_flex", "middle_finger_mcp_abd", "middle_finger_pip", "middle_finger_dip",
    "ring_finger_mcp_flex", "ring_finger_mcp_abd", "ring_finger_pip", "ring_finger_dip",
    "pinky_mcp_flex", "pinky_mcp_abd", "pinky_pip", "pinky_dip",
)
ARM_JOINT_NAMES = {
    side: tuple(f"{side}_joint_{i}" for i in range(1, 8))
    for side in ("left", "right")
}
HAND_JOINT_NAMES = {
    side: tuple(f"{side[0]}_{suffix}" for suffix in HAND_SUFFIXES)
    for side in ("left", "right")
}
JOINT_NAMES = (
    *ARM_JOINT_NAMES["left"], *ARM_JOINT_NAMES["right"],
    *HAND_JOINT_NAMES["left"], *HAND_JOINT_NAMES["right"],
)
EE_NAMES = tuple(
    f"{side}.{component}"
    for side in ("left", "right")
    for component in ("x", "y", "z", "qx", "qy", "qz", "qw")
)
NUMERIC_SHAPES = {
    "action.joint_position": 54,
    "action.ee_pose": 14,
    "state.joint_position": 54,
    "state.ee_pose": 14,
    "state.joint_velocity": 54,
}
RGB_KEYS = tuple(
    f"observation.rgb.{camera}"
    for camera in ("head_left", "head_right", "left_wrist", "right_wrist", "d435")
)
DEPTH_KEY = "observation.depth.d435"
IMAGE_KEYS = (*RGB_KEYS, DEPTH_KEY)
ROBOT_SOURCE_STREAMS = (
    "action.left_arm", "action.right_arm", "action.left_hand", "action.right_hand",
    "action.left_ee", "action.right_ee",
    "state.left_arm", "state.right_arm", "state.left_hand", "state.right_hand",
)
SOURCE_STREAMS = (*ROBOT_SOURCE_STREAMS, *IMAGE_KEYS)
SOURCE_TIMESTAMP_KEYS = tuple(f"source_timestamp.{stream}" for stream in SOURCE_STREAMS)
DEFAULT_FEATURES = {
    "timestamp": {"dtype": "float32", "shape": [1], "names": None},
    **{
        key: {"dtype": "int64", "shape": [1], "names": None}
        for key in ("frame_index", "episode_index", "index", "task_index")
    },
}


def make_features(image_shapes):
    """Build the v2.1 feature dictionary without changing camera dimensions."""
    features = {
        key: {
            "dtype": "float64", "shape": [size],
            "names": list(EE_NAMES if key.endswith("ee_pose") else JOINT_NAMES),
        }
        for key, size in NUMERIC_SHAPES.items()
    }
    for key in RGB_KEYS:
        height, width, channels = image_shapes[key]
        features[key] = {
            "dtype": "video", "shape": [height, width, channels],
            "names": ["height", "width", "channels"],
            "info": {
                "video.height": height, "video.width": width,
                "video.codec": "h264", "video.pix_fmt": "yuv420p",
                "video.is_depth_map": False, "video.fps": FPS,
                "video.channels": 3, "has_audio": False,
            },
        }
    features[DEPTH_KEY] = {
        "dtype": "uint16", "shape": list(image_shapes[DEPTH_KEY]),
        "names": ["height", "width"],
    }
    features.update({
        key: {"dtype": "int64", "shape": [1], "names": None}
        for key in SOURCE_TIMESTAMP_KEYS
    })
    features.update(DEFAULT_FEATURES)
    return features
