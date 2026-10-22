"""Visualize MANUS targets and Tracker poses."""
from pathlib import Path
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from openvr_input.configuration import input_config_path
from wuji_teleop_bringup.launch_utils import resolve_config_path


TF_REMAPS = [("/tf", "/preview/tf"), ("/tf_static", "/preview/tf_static")]


def _nodes(context):
    enabled = lambda name: LaunchConfiguration(name).perform(context).lower() == "true"
    nodes = []
    with open(resolve_config_path("wujihand2_ros2", "wujihand2.yaml")) as stream:
        hand_config = yaml.safe_load(stream)
    if enabled("start_inputs"):
        profiles = hand_config.get("manus", {})
        config_path = input_config_path(LaunchConfiguration("input_config").perform(context))
        nodes += [
            Node(package="manus_ros2", executable="manus_data_publisher", name="preview_manus",
                 output="log", parameters=[profiles],
                 remappings=[("/manus_glove_0", "/preview/manus_glove_0"),
                             ("/manus_glove_1", "/preview/manus_glove_1")]),
            Node(package="openvr_input", executable="openvr_input", name="preview_openvr",
                 output="screen", arguments=["-c", str(config_path)], remappings=TF_REMAPS),
        ]
    preview = Node(package="wuji_teleop_bringup", executable="wujihand2_preview", output="screen",
                   parameters=[{"flip_y": bool(hand_config.get("controller", {}).get("flip_y", False))}])
    nodes.append(preview)
    model = Path(get_package_share_directory("wuji_hand2_2_description")) / "urdf"
    for side, x in (("left", -0.18), ("right", 0.18)):
        prefix = f"preview/{side}/"
        nodes.append(Node(
            package="robot_state_publisher", executable="robot_state_publisher",
            namespace=f"preview/{side}", name="model", output="screen",
            parameters=[{"robot_description": ParameterValue(
                             (model / f"{side}-ros.urdf").read_text(), value_type=str),
                         "frame_prefix": prefix}],
            remappings=[("joint_states", f"/preview/{side}/joint_states"), *TF_REMAPS]))
        nodes.append(Node(
            package="tf2_ros", executable="static_transform_publisher", name=f"preview_{side}_mount",
            arguments=["--x", str(x), "--y", "0", "--z", "1",
                       "--frame-id", "world", "--child-frame-id", f"{prefix}{side[0]}_mount"],
            remappings=TF_REMAPS))
    if enabled("enable_rviz"):
        rviz = Path(get_package_share_directory("wuji_teleop_bringup")) / "rviz/preview.rviz"
        viewer = Node(package="rviz2", executable="rviz2", name="preview_rviz",
                      arguments=["-d", str(rviz)], remappings=TF_REMAPS, output="screen")
        nodes += [viewer, RegisterEventHandler(OnProcessExit(
            target_action=viewer, on_exit=[EmitEvent(event=Shutdown(reason="Preview viewer closed"))]))]
    # A failed/stopped target processor must not leave input subprocesses behind.
    nodes.append(RegisterEventHandler(OnProcessExit(
        target_action=preview, on_exit=[EmitEvent(event=Shutdown(reason="Preview processor exited"))])))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("start_inputs", default_value="true", choices=["true", "false"],
                              description="Start input readers; false permits a no-hardware graph check"),
        DeclareLaunchArgument("enable_rviz", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("input_config", default_value="", description="OpenVR input configuration YAML"),
        OpaqueFunction(function=_nodes),
    ])
