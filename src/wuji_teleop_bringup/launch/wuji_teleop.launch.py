"""MANUS + five HTC trackers: hands, arms, or both."""
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from wuji_teleop_bringup.launch_utils import resolve_config_path
from wuji_teleop_bringup.tf_utils import create_chest_tf_nodes, create_tianji_tf_nodes


def _enabled(context, name):
    return LaunchConfiguration(name).perform(context).lower() == "true"


def _nodes(context):
    hand, arm = _enabled(context, "enable_hand"), _enabled(context, "enable_arm")
    if not (hand or arm):
        raise ValueError("Enable at least one of enable_hand or enable_arm")
    nodes = []
    if hand:
        path = LaunchConfiguration("hand_config").perform(context)
        path = path or resolve_config_path("wujihand_output", "hand2.yaml")
        with open(path) as stream:
            config = yaml.safe_load(stream)
        read_only = _enabled(context, "hand_read_only")
        selection = LaunchConfiguration("hand_side").perform(context)
        sides = ("left", "right") if selection == "both" else (selection,)
        serials = [str(config[f"{side}_hand"]["serial_number"]) for side in sides]
        names = [config[f"{side}_hand"].get("name", f"{side}_hand") for side in sides]
        if len(set(serials)) != len(serials) or len(set(names)) != len(names):
            raise ValueError("Each hand must have a distinct serial number and namespace")
        if not read_only:
            nodes.append(Node(package="manus_ros2", executable="manus_data_publisher", output="screen",
                              parameters=[config.get("manus", {})]))
        for side in sides:
            cfg = config[f"{side}_hand"]
            serial = str(cfg["serial_number"])
            if not serial or serial.startswith("YOUR_"):
                raise ValueError(f"Configure {side}_hand.serial_number in {path}")
            name = cfg.get("name", f"{side}_hand")
            nodes.append(Node(
                package="wuji_hand2_driver", executable="wuji_hand2_driver",
                name="wuji_hand2_driver", namespace=name, output="screen",
                parameters=[dict(config.get("driver", {}), serial_number=serial,
                                 expected_side=side, read_only=read_only,
                                 auto_enable=_enabled(context, "hand_auto_enable") and not read_only)]))
            if not read_only:
                nodes.append(Node(
                    package="controller", executable="wujihand_controller",
                    name=f"wujihand_controller_{side}", output="screen",
                    parameters=[config.get("controller", {})],
                    arguments=["--side", side, "--hand-name", name]))
    if arm:
        nodes.append(Node(package="openvr_input", executable="openvr_input", output="screen",
                          arguments=["-c", resolve_config_path("openvr_input", "openvr_input.yaml")]))
        nodes.extend(create_chest_tf_nodes())
        nodes.extend(create_tianji_tf_nodes())
        nodes.append(Node(package="controller", executable="tianji_arm_controller", output="screen",
                          parameters=[{"auto_enable": _enabled(context, "arm_auto_enable")}]))
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
        DeclareLaunchArgument("hand_side", default_value="both", choices=["both", "left", "right"]),
        DeclareLaunchArgument("hand_read_only", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("hand_auto_enable", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("arm_auto_enable", default_value="false", choices=["true", "false"]),
        OpaqueFunction(function=_nodes),
    ])
