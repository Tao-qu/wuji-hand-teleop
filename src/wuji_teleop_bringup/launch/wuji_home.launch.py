"""Hold configured Home targets without MANUS or Tracker input readers."""
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from wuji_teleop_bringup.launch_utils import resolve_config_path


def _nodes(context):
    mode = LaunchConfiguration("mode").perform(context)
    hand_path = LaunchConfiguration("hand_config").perform(context)
    arm_path = LaunchConfiguration("arm_config").perform(context)
    nodes = []
    if mode in ("both", "hands"):
        hand_path = hand_path or resolve_config_path("wujihand_output", "hand2.yaml")
        with open(hand_path) as stream:
            config = yaml.safe_load(stream)
        for side in ("left", "right"):
            hand = config[f"{side}_hand"]
            nodes.append(Node(
                package="wuji_hand2_driver", executable="wuji_hand2_driver",
                name="wuji_hand2_driver", namespace=hand.get("name", f"{side}_hand"),
                output="screen",
                parameters=[dict(config.get("driver", {}),
                                 serial_number=str(hand["serial_number"]),
                                 expected_side=side, auto_enable=True)]))
    home = Node(
        package="wuji_teleop_bringup", executable="home_position", output="screen",
        parameters=[{"mode": mode, "hand_config": hand_path, "arm_config": arm_path}])
    nodes.append(home)
    nodes.append(RegisterEventHandler(OnProcessExit(
        target_action=home, on_exit=[EmitEvent(event=Shutdown(reason="Home session exited"))])))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="both", choices=["both", "arms", "hands"]),
        DeclareLaunchArgument("hand_config", default_value="", description="Optional hand hardware YAML"),
        DeclareLaunchArgument("arm_config", default_value="", description="Optional arm and Tracker YAML"),
        OpaqueFunction(function=_nodes),
    ])
