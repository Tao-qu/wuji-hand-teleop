"""Launch file for OpenVR Tracker input node."""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package share directory
    pkg_dir = get_package_share_directory('openvr_input')

    # Declare launch arguments
    input_config_arg = DeclareLaunchArgument(
        'input_config',
        default_value='',
        description='Optional OpenVR input configuration YAML (default: openvr.yaml)'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch RViz for visualization'
    )

    # OpenVR input node
    openvr_input_node = Node(
        package='openvr_input',
        executable='openvr_input',
        name='openvr_input',
        output='screen',
        arguments=['-c', LaunchConfiguration('input_config')],
    )

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_dir, 'rviz', 'openvr_visualization.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        input_config_arg,
        rviz_arg,
        openvr_input_node,
        rviz_node,
    ])
