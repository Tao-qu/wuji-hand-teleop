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
        path = path or resolve_config_path("wujihand_output", "wujihand_ik.yaml")
        with open(path) as stream:
            config = yaml.safe_load(stream)
        nodes.append(Node(package="manus_ros2", executable="manus_data_publisher", output="screen"))
        for side in ("left", "right"):
            cfg = config[f"{side}_hand"]
            serial = str(cfg["serial_number"])
            if not serial or serial.startswith("YOUR_"):
                raise ValueError(f"Configure {side}_hand.serial_number in {path}")
            name = cfg.get("name", f"{side}_hand")
            nodes.append(Node(
                package="wujihand_driver", executable="wujihand_driver_node",
                name="wujihand_driver", namespace=name, output="screen",
                parameters=[dict(config.get("driver", {}), serial_number=serial)]))
            nodes.append(Node(
                package="controller", executable="wujihand_controller",
                name=f"wujihand_controller_{side}", output="screen",
                arguments=["--side", side, "--hand-name", name]))
    if arm:
        nodes.append(Node(package="openvr_input", executable="openvr_input", output="screen",
                          arguments=["-c", resolve_config_path("openvr_input", "openvr_input.yaml")]))
        nodes.extend(create_chest_tf_nodes())
        nodes.extend(create_tianji_tf_nodes())
        nodes.append(Node(package="controller", executable="tianji_arm_controller", output="screen"))
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
        OpaqueFunction(function=_nodes),
    ])
