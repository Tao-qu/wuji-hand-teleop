#!/usr/bin/env bash
# Source on the Jetson after building the camera workspace.
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
    printf 'Use: source tools/camera_env.sh\n' >&2
    exit 2
fi
_CAMERA_ENV_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ ! -f /opt/ros/humble/setup.bash || ! -f "$_CAMERA_ENV_ROOT/install_camera/local_setup.bash" ]]; then
    printf 'Camera workspace is not built. On the Jetson, run ./tools/setup_camera.sh first. On the desktop, use source tools/env.sh.\n' >&2
    unset _CAMERA_ENV_ROOT
    return 1
fi
# Camera nodes use system Python and their own ROS overlay.
if declare -F deactivate >/dev/null; then
    deactivate
fi
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
export PYTHONNOUSERSITE=1
source /opt/ros/humble/setup.bash
_CAMERA_DEP_PREFIX="$_CAMERA_ENV_ROOT/camera_dependencies/root/opt/ros/humble"
if [[ -f "$_CAMERA_DEP_PREFIX/local_setup.bash" ]]; then
    source "$_CAMERA_DEP_PREFIX/local_setup.bash"
fi
source "$_CAMERA_ENV_ROOT/install_camera/local_setup.bash"
export PYTHONPATH="$_CAMERA_ENV_ROOT/camera_dependencies/python${PYTHONPATH:+:$PYTHONPATH}"
# Native GMSL RGB frames exceed the default Fast DDS shared-memory segment.
# Use the same transport for the Jetson camera publishers and dataset writer.
export FASTRTPS_DEFAULT_PROFILES_FILE="$_CAMERA_ENV_ROOT/src/camera/camera_bringup/config/fastdds.xml"
unset _CAMERA_DEP_PREFIX _CAMERA_ENV_ROOT
