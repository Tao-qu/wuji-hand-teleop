"""Launch independent GMSL and official RealSense processes on the Jetson."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml


def _enabled(context, name):
    return LaunchConfiguration(name).perform(context).lower() == "true"


def _nodes(context):
    share = Path(get_package_share_directory("camera_bringup"))
    source = Path((share / "source_directory").read_text().strip())
    config_file = source / "config" / "cameras.yaml"
    with config_file.open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    nodes = []
    if _enabled(context, "gmsl"):
        settings = config["gmsl"]
        parameters = {key: settings[key] for key in ("width", "height", "fps")}
        parameters["cameras"] = list(settings["cameras"])
        parameters.update({f"preview.{key}": value for key, value in settings["preview"].items()})
        for role, camera in settings["cameras"].items():
            for key, value in camera.items():
                if key == "calibration_file":
                    value = str(source / "calibration" / value) if value else ""
                parameters[f"{role}.{key}"] = value
        nodes.append(Node(
            package="gmsl_camera", executable="gmsl_camera_node", name="gmsl",
            namespace="/cameras", parameters=[parameters], output="screen"))
    if _enabled(context, "d435"):
        parameters = config["d435"].copy()
        parameters["serial_no"] = ParameterValue(str(parameters["serial_no"]), value_type=str)
        nodes.append(Node(
            package="realsense2_camera", executable="realsense2_camera_node", name="d435",
            namespace="/cameras", parameters=[parameters], output="screen"))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("gmsl", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("d435", default_value="true", choices=["true", "false"]),
        OpaqueFunction(function=_nodes),
    ])
