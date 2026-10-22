"""Launch only the d435 camera process."""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from pathlib import Path


def generate_launch_description():
    path = Path(get_package_share_directory("camera_bringup")) / "launch" / "cameras.launch.py"
    return LaunchDescription([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(path)), launch_arguments={"gmsl": "false"}.items())])
