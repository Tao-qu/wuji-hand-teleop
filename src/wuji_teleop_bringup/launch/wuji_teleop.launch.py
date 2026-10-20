"""MANUS + five HTC trackers: hands, arms, or both."""
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from tianji_output.configuration import arm_config_path, load_arm_config
from openvr_input.tracker_calibration_io import load_selected_profile
from wuji_teleop_bringup.launch_utils import resolve_config_path
from wuji_teleop_bringup.tf_utils import create_chest_tf_nodes, create_tianji_tf_nodes


def _enabled(context, name):
    return LaunchConfiguration(name).perform(context).lower() == "true"


def _nodes(context):
    hand, arm = _enabled(context, "enable_hand"), _enabled(context, "enable_arm")
    startup_wait = float(LaunchConfiguration("startup_wait_sec").perform(context))
    nodes = []
    if hand:
        path = LaunchConfiguration("hand_config").perform(context)
        path = path or resolve_config_path("wujihand_output", "hand2.yaml")
        with open(path) as stream:
            config = yaml.safe_load(stream)
        selection = LaunchConfiguration("hand_side").perform(context)
        sides = ("left", "right") if selection == "both" else (selection,)
        nodes.append(Node(package="manus_ros2", executable="manus_data_publisher", output="screen",
                          parameters=[config.get("manus", {})]))
        for side in sides:
            cfg = config[f"{side}_hand"]
            serial = str(cfg["serial_number"])
            name = cfg.get("name", f"{side}_hand")
            nodes.append(Node(
                package="wuji_hand2_driver", executable="wuji_hand2_driver",
                name="wuji_hand2_driver", namespace=name, output="screen",
                parameters=[dict(config.get("driver", {}), serial_number=serial,
                                 expected_side=side,
                                 auto_enable=_enabled(context, "hand_auto_enable"))]))
            nodes.append(Node(
                package="controller", executable="wujihand_controller",
                name=f"wujihand_controller_{side}", output="screen",
                parameters=[dict(config.get("controller", {}),
                                 wait_for_arm=arm, startup_wait_sec=startup_wait)],
                arguments=["--side", side, "--hand-name", name]))
    if arm:
        arm_path = arm_config_path(LaunchConfiguration("arm_config").perform(context))
        arm_config = load_arm_config(arm_path)
        calibration = load_selected_profile(arm_config)
        nodes.append(Node(package="openvr_input", executable="openvr_input", output="screen",
                          arguments=["-c", str(arm_path)]))
        nodes.extend(create_chest_tf_nodes(calibration))
        nodes.extend(create_tianji_tf_nodes(calibration))
        nodes.append(Node(package="controller", executable="tianji_arm_controller", output="screen",
                          arguments=["-c", str(arm_path)],
                          parameters=[{"auto_enable": _enabled(context, "arm_auto_enable"),
                                       "startup_wait_sec": startup_wait}]))
    if _enabled(context, "enable_rviz"):
        rviz = Path(get_package_share_directory("openvr_input")) / "rviz" / "openvr_visualization.rviz"
        nodes.append(Node(package="rviz2", executable="rviz2", arguments=["-d", str(rviz)], output="screen"))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("enable_hand", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("enable_arm", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("enable_rviz", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("hand_config", default_value="", description="Optional hand hardware YAML"),
        DeclareLaunchArgument("arm_config", default_value="", description="Optional arm and Tracker YAML"),
        DeclareLaunchArgument("hand_side", default_value="both", choices=["both", "left", "right"]),
        DeclareLaunchArgument("hand_auto_enable", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("arm_auto_enable", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("startup_wait_sec", default_value="3.0",
                              description="Preparation wait shared by hand and arm following"),
        OpaqueFunction(function=_nodes),
    ])
