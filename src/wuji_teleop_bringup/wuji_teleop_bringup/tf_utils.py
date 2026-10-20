"""Publish the fixed transforms from the selected Tracker calibration."""

from launch_ros.actions import Node


def create_chest_tf_nodes(config: dict, sides: list = None) -> list:
    """
    Create chest mount TF nodes.

    Args:
        config: Selected Tracker calibration
        sides: ["left", "right"] or subset, defaults to both
    """
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


def create_tianji_tf_nodes(config: dict, sides: list = None) -> list:
    """Create wrist to tianji TF nodes."""
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
