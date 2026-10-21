#!/usr/bin/env bash
# Prepare this checkout after installing Ubuntu 22.04 and ROS 2 Humble.
set -eo pipefail

TELEOP_SETUP_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
skip_system=false
skip_build=false
for option in "$@"; do
    case "$option" in
        --skip-system) skip_system=true ;;
        --skip-build) skip_build=true ;;
        --help|-h)
            cat <<'USAGE'
Usage: ./tools/setup.sh [--skip-system] [--skip-build]

Install project dependencies, fetch SDK/model assets, create missing local
configuration, build the ROS workspace, and check the installed environment.
Existing configuration and calibration files are preserved.

  --skip-system  Skip apt, rosdep, USB rules, and group changes.
  --skip-build   Prepare dependencies and configuration without building.
USAGE
            exit 0 ;;
        *) printf 'Unknown option: %s\n' "$option" >&2; exit 2 ;;
    esac
done

if [[ "$(uname -m)" != x86_64 || ! -f /opt/ros/humble/setup.bash ]]; then
    printf 'Install ROS 2 Humble on Ubuntu 22.04 x86_64 first. See docs/LOCAL_SETUP.md.\n' >&2
    exit 1
fi
/usr/bin/python3 -c 'import sys; assert sys.version_info[:2] == (3, 10), "Python 3.10 is required"'
cd "$TELEOP_SETUP_ROOT"
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
export PYTHONNOUSERSITE=1
source /opt/ros/humble/setup.bash

if ! "$skip_system"; then
    sudo apt-get update
    sudo apt-get install -y build-essential cmake git git-lfs curl usbutils \
        python3-dev python3-pip python3-venv python3-colcon-common-extensions \
        python3-rosdep python3-pyqt5 python3-pyqt5.qtsvg \
        libusb-1.0-0-dev libncurses-dev libspdlog-dev libgl1-mesa-glx
    if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
        sudo rosdep init
    fi
    rosdep update --rosdistro humble
    sudo install -m 644 src/input_devices/manus_input/config/udev/99-manus-libusb.rules \
        /etc/udev/rules.d/99-manus-libusb.rules
    sudo usermod -aG plugdev,dialout "$(id -un)"
    sudo udevadm control --reload-rules
fi

git lfs install --local
git lfs pull
/usr/bin/python3 tools/fetch_dependencies.py
/usr/bin/python3 tools/configure.py
/usr/bin/python3 -m venv --system-site-packages .venv
.venv/bin/python -m pip install 'pip>=23,<27' 'setuptools==68.2.2' 'wheel==0.45.1'
.venv/bin/python -m pip install -r requirements.txt

if ! "$skip_system"; then
    rosdep install --from-paths src --ignore-src --rosdistro humble -y
fi

if "$skip_build"; then
    printf '\nDependencies and configuration are ready. Run ./tools/setup.sh to build.\n'
    exit 0
fi

source tools/env.sh
python3 -m colcon --log-base "$TELEOP_SETUP_ROOT/log" build \
    --build-base "$TELEOP_SETUP_ROOT/build" \
    --install-base "$TELEOP_SETUP_ROOT/install" \
    --symlink-install --parallel-workers 2 --event-handlers console_cohesion+ \
    --cmake-args -DBUILD_TESTING=OFF \
        -DPython3_EXECUTABLE="$TELEOP_SETUP_ROOT/.venv/bin/python3"
source tools/env.sh
python3 tools/check_environment.py --mode all
printf '\nReady. Review the two device YAMLs, then source tools/env.sh in each terminal.\n'
if ! "$skip_system"; then
    printf 'Reconnect the MANUS receivers. Log in again if device group membership changed.\n'
fi
