#!/usr/bin/env bash
# Source this file in every terminal used for building or running the system.
_TELEOP_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# Use only the ROS underlay and this checkout's Python packages.
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
export PYTHONNOUSERSITE=1
source /opt/ros/humble/setup.bash
if [[ -f "$_TELEOP_ROOT/.venv/bin/activate" ]]; then
    source "$_TELEOP_ROOT/.venv/bin/activate"
fi
if [[ -f "$_TELEOP_ROOT/install/local_setup.bash" ]]; then
    source "$_TELEOP_ROOT/install/local_setup.bash"
fi
export LD_LIBRARY_PATH="$_TELEOP_ROOT/src/output_devices/tianji_output/tianji_output/_internal/lib:$_TELEOP_ROOT/src/input_devices/manus_input/manus_ros2/ManusSDK/lib:${LD_LIBRARY_PATH:-}"
# Match colcon-generated Python nodes to the project venv even when ros2's
# system-Python entry point is used by the shell.
if [[ -d "$_TELEOP_ROOT/.venv/lib/python3.10/site-packages" ]]; then
    export PYTHONPATH="$_TELEOP_ROOT/.venv/lib/python3.10/site-packages:${PYTHONPATH:-}"
fi
unset _TELEOP_ROOT
