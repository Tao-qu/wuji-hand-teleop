"""Shared TF utilities for launch files."""

from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def load_tf_config() -> dict:
    """Load static transforms config."""
    share_dir = Path(get_package_share_directory("wuji_teleop_bringup"))
    with open(share_dir / "config" / "static_transforms.yaml") as f:
        return yaml.safe_load(f)


def create_chest_tf_nodes(config: dict = None, sides: list = None) -> list:
    """
    Create chest mount TF nodes.

    Args:
        config: Config dict, loads from yaml if None
        sides: ["left", "right"] or subset, defaults to both
    """
    if config is None:
        config = load_tf_config()
    if sides is None:
        sides = ["left", "right"]

    chest = config["chest_mount"]
    shared = chest["shared"]
    nodes = []

    for side in sides:
        s = chest[side]
        # Layer 1: chest -> {side}_chest_base
        # The legacy CLI read the YAML roll/pitch/yaw values as yaw/pitch/roll.
        # Preserve that effective rotation and the existing Home calibration.
        nodes.append(Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name=f"{side}_chest_base_tf",
            arguments=[
                "--x", str(shared["x"]),
                "--y", str(shared["y"]),
                "--z", str(shared["z"]),
                "--yaw", str(s["roll"]),
                "--pitch", str(s["pitch"]),
                "--roll", str(s["yaw"]),
                "--frame-id", "chest",
                "--child-frame-id", f"{side}_chest_base",
            ],
        ))
        # Layer 2: {side}_chest_base -> {side}_chest
        nodes.append(Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name=f"{side}_chest_tf",
            arguments=[
                "--x", str(s["local_x"]),
                "--y", str(s["local_y"]),
                "--z", str(s["local_z"]),
                "--yaw", "0", "--pitch", "0", "--roll", "0",
                "--frame-id", f"{side}_chest_base",
                "--child-frame-id", f"{side}_chest",
            ],
        ))

    return nodes


def create_tianji_tf_nodes(config: dict = None, sides: list = None) -> list:
    """Create wrist to tianji TF nodes."""
    if config is None:
        config = load_tf_config()
    if sides is None:
        sides = ["left", "right"]

    tianji = config["wrist_to_tianji"]
    nodes = []

    for side in sides:
        t = tianji[side]
        nodes.append(Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name=f"tianji_{side}_tf",
            arguments=[
                "--x", str(t["x"]),
                "--y", str(t["y"]),
                "--z", str(t["z"]),
                "--qx", str(t["qx"]),
                "--qy", str(t["qy"]),
                "--qz", str(t["qz"]),
                "--qw", str(t["qw"]),
                "--frame-id", f"{side}_wrist",
                "--child-frame-id", f"tianji_{side}",
            ],
        ))

    return nodes
