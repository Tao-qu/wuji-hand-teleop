#!/usr/bin/env bash
# Usage: tools/home.sh [both|arms|hands] [additional ROS launch arguments]
set -e

_TELEOP_HOME_MODE="${1:-both}"
if [[ $# -gt 0 ]]; then
    shift
fi
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
exec ros2 launch wuji_teleop_bringup wuji_home.launch.py "mode:=$_TELEOP_HOME_MODE" "$@"
