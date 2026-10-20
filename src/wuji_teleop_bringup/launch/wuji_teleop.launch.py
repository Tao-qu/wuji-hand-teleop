#!/usr/bin/env python3
"""Run hand/arm teleoperation or Home with the fixed project configuration."""
import argparse
import sys

import yaml
from launch import LaunchDescription, LaunchService
from launch.actions import EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node
from tianji_output.configuration import arm_config_path, load_arm_config
from openvr_input.tracker_calibration_io import load_selected_profile
from wuji_teleop_bringup.launch_utils import resolve_config_path
from wuji_teleop_bringup.tf_utils import create_chest_tf_nodes, create_tcp_target_tf_nodes


def _options(argv):
    parser = argparse.ArgumentParser(
        description="Start both hands and both arms, or return them Home.",
        allow_abbrev=False,
    )
    parser.add_argument("--disable-hand", action="store_true", help="omit both hands and MANUS")
    parser.add_argument("--disable-arm", action="store_true", help="omit both arms and Trackers")
    parser.add_argument("--home", action="store_true",
                        help="return the selected devices Home, then exit without input readers")
    options = parser.parse_args(argv)
    if options.disable_hand and options.disable_arm:
        parser.error("--disable-hand and --disable-arm cannot be used together")
    return options


def _nodes(context):
    options = _options(context.argv)
    hand, arm = not options.disable_hand, not options.disable_arm
    nodes = []
    if hand:
        with open(resolve_config_path("wujihand_output", "hand2.yaml")) as stream:
            config = yaml.safe_load(stream)
        if not options.home:
            nodes.append(Node(
                package="manus_ros2", executable="manus_data_publisher", output="screen",
                parameters=[config.get("manus", {})]))
        for side in ("left", "right"):
            cfg = config[f"{side}_hand"]
            name = cfg.get("name", f"{side}_hand")
            nodes.append(Node(
                package="wuji_hand2_driver", executable="wuji_hand2_driver",
                name="wuji_hand2_driver", namespace=name, output="screen",
                parameters=[dict(config.get("driver", {}), serial_number=str(cfg["serial_number"]),
                                 expected_side=side, auto_enable=True)]))
            if not options.home:
                nodes.append(Node(
                    package="controller", executable="wujihand_controller",
                    name=f"wujihand_controller_{side}", output="screen",
                    parameters=[dict(config.get("controller", {}),
                                     wait_for_arm=arm, startup_wait_sec=3.0)],
                    arguments=["--side", side, "--hand-name", name]))
    if options.home:
        mode = "both" if hand and arm else "hands" if hand else "arms"
        home = Node(
            package="wuji_teleop_bringup", executable="home_position", output="screen",
            parameters=[{"mode": mode}])
        nodes.append(RegisterEventHandler(OnProcessExit(
            target_action=home, on_exit=[EmitEvent(event=Shutdown(reason="Home session exited"))])))
        nodes.append(home)
    elif arm:
        arm_path = arm_config_path()
        calibration = load_selected_profile(load_arm_config(arm_path))
        nodes.append(Node(package="openvr_input", executable="openvr_input", output="screen",
                          arguments=["-c", str(arm_path)]))
        nodes.extend(create_chest_tf_nodes(calibration))
        nodes.extend(create_tcp_target_tf_nodes(calibration))
        nodes.append(Node(
            package="controller", executable="tianji_arm_controller", output="screen",
            arguments=["-c", str(arm_path)],
            parameters=[{"auto_enable": True, "startup_wait_sec": 3.0}]))
    return nodes


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=_nodes)])


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    _options(argv)
    service = LaunchService(argv=argv)
    service.include_launch_description(generate_launch_description())
    return service.run()


if __name__ == "__main__":
    raise SystemExit(main())
