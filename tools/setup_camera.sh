#!/usr/bin/env bash
# Build cameras and the dataset writer independently of the robot SDKs.
set -eo pipefail
CAMERA_SETUP_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
skip_system=false
skip_build=false
user_deps=false
for option in "$@"; do
    case "$option" in
        --skip-system) skip_system=true ;;
        --skip-build) skip_build=true ;;
        --user-deps) user_deps=true ;;
        --help|-h)
            cat <<'USAGE'
Usage: ./tools/setup_camera.sh [--skip-system] [--skip-build] [--user-deps]

Requires Ubuntu 22.04 / ROS 2 Humble. Capture requires the Tianji Jetson BSP.
Builds src/camera, data_collection, and data_collection_interfaces into install_camera.
  --skip-system  Use existing native/ROS dependencies; do not run sudo or apt install.
  --skip-build   Initialize configuration and dependencies without compiling.
  --user-deps    Download missing official RealSense ROS packages into a private
                 prefix without sudo. Native build dependencies must be installed.
No camera stream or system service is started, stopped, or enabled.
USAGE
            exit 0 ;;
        *) printf 'Unknown option: %s\n' "$option" >&2; exit 2 ;;
    esac
done
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
    printf 'Install ROS 2 Humble first. See docs/CAMERAS.md.\n' >&2
    exit 1
fi
cd "$CAMERA_SETUP_ROOT"
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
export PYTHONNOUSERSITE=1
source /opt/ros/humble/setup.bash
if ! "$skip_system" && ! "$user_deps"; then
    sudo apt-get update
    sudo apt-get install -y build-essential cmake pkg-config python3-yaml python3-numpy \
        python3-pip ffmpeg \
        python3-colcon-common-extensions libgstreamer1.0-dev \
        libgstreamer-plugins-base1.0-dev libopencv-dev libyaml-cpp-dev \
        gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
        gstreamer1.0-plugins-bad ros-humble-realsense2-camera ros-humble-compressed-image-transport \
        ros-humble-rosidl-default-generators ros-humble-tf2-ros-py
fi
if "$user_deps"; then
    /usr/bin/python3 - "$CAMERA_SETUP_ROOT/camera_dependencies" <<'PY'
from pathlib import Path
import os
import re
import subprocess
import sys

root = Path(sys.argv[1])
cache = root / 'archives'
prefix = root / 'root'
cache.mkdir(parents=True, exist_ok=True)
prefix.mkdir(parents=True, exist_ok=True)
packages = ['ros-humble-realsense2-camera', 'ros-humble-compressed-image-transport',
            'ros-humble-rosidl-default-generators', 'ros-humble-tf2-ros-py']
plan = subprocess.check_output(['apt-get', '-s', 'install', *packages], text=True,
                               env={**os.environ, 'LC_ALL': 'C'})
resolved = []
missing_native = []
for line in plan.splitlines():
    match = re.match(r'Inst (\S+)(?: \[[^]]+\])? \((\S+)', line)
    if not match:
        continue
    package, version = match.groups()
    if package.startswith('ros-humble-'):
        resolved.append((package, version))
    elif package.split(':')[0].endswith('-dev'):
        print(f'Skipping development-only dependency of the prebuilt ROS package: {package}')
    else:
        missing_native.append(package)
if missing_native:
    raise SystemExit('Install missing native runtime dependencies first: sudo apt-get install ' +
                     ' '.join(missing_native))
for package, version in resolved:
    # Extract only this resolved version, never arbitrary old archives in the cache.
    target = cache / f'{package}_{version.replace(":", "_")}'
    target.mkdir(parents=True, exist_ok=True)
    subprocess.run(['apt-get', 'download', f'{package}={version}'], cwd=target, check=True)
    for archive in target.glob('*.deb'):
        subprocess.run(['dpkg-deb', '--extract', str(archive), str(prefix)], check=True)
overlay = prefix / 'opt/ros/humble'
overlay.mkdir(parents=True, exist_ok=True)
(overlay / 'local_setup.bash').write_text('''
CAMERA_PRIVATE_PREFIX="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CAMERA_MULTIARCH="$(dpkg-architecture -qDEB_HOST_MULTIARCH)"
export AMENT_PREFIX_PATH="$CAMERA_PRIVATE_PREFIX${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
export CMAKE_PREFIX_PATH="$CAMERA_PRIVATE_PREFIX${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
export LD_LIBRARY_PATH="$CAMERA_PRIVATE_PREFIX/lib:$CAMERA_PRIVATE_PREFIX/lib/$CAMERA_MULTIARCH${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export PYTHONPATH="$CAMERA_PRIVATE_PREFIX/local/lib/python3.10/dist-packages:$CAMERA_PRIVATE_PREFIX/lib/python3.10/site-packages${PYTHONPATH:+:$PYTHONPATH}"
unset CAMERA_PRIVATE_PREFIX CAMERA_MULTIARCH
''')
print(f'Official ROS dependencies extracted to {prefix}')
PY
fi

/usr/bin/python3 tools/configure.py --profile camera
# Keep the Jetson's NumPy/OpenCV pair supplied by Ubuntu. Only the Arrow wheel is
# installed privately; this does not alter system Python or user site-packages.
/usr/bin/python3 -m pip install --only-binary=:all: --no-deps --upgrade \
    --target "$CAMERA_SETUP_ROOT/camera_dependencies/python" \
    -r "$CAMERA_SETUP_ROOT/requirements-recording.txt"
export PYTHONPATH="$CAMERA_SETUP_ROOT/camera_dependencies/python${PYTHONPATH:+:$PYTHONPATH}"
/usr/bin/python3 -c 'import numpy, pyarrow'
if ! command -v ffmpeg >/dev/null || ! ffmpeg -hide_banner -encoders 2>/dev/null | grep ' libx264 ' >/dev/null; then
    printf 'Install FFmpeg with the libx264 encoder: sudo apt-get install ffmpeg\n' >&2
    exit 1
fi
if [[ -f "$CAMERA_SETUP_ROOT/camera_dependencies/root/opt/ros/humble/local_setup.bash" ]]; then
    source "$CAMERA_SETUP_ROOT/camera_dependencies/root/opt/ros/humble/local_setup.bash"
fi
pkg-config --exists gstreamer-app-1.0 gstreamer-video-1.0 opencv4 yaml-cpp || {
    printf 'Install native build dependencies listed in docs/CAMERAS.md.\n' >&2; exit 1;
}
realsense_prefix="$(ros2 pkg prefix realsense2_camera)" || {
    printf 'Install ros-humble-realsense2-camera, or rerun with --user-deps.\n' >&2; exit 1;
}
missing_libraries="$(ldd "$realsense_prefix/lib/librealsense2_camera.so" | awk '/not found/{print $1}')"
if [[ -n "$missing_libraries" ]]; then
    printf 'Missing RealSense runtime libraries:\n%s\n' "$missing_libraries" >&2
    exit 1
fi
if "$skip_build"; then
    printf 'Camera configuration and dependencies are ready.\n'
    exit 0
fi
/usr/bin/python3 -m colcon --log-base "$CAMERA_SETUP_ROOT/log_camera" build \
    --base-paths "$CAMERA_SETUP_ROOT/src/camera" \
        "$CAMERA_SETUP_ROOT/src/data_collection" "$CAMERA_SETUP_ROOT/src/data_collection_interfaces" \
    --build-base "$CAMERA_SETUP_ROOT/build_camera" \
    --install-base "$CAMERA_SETUP_ROOT/install_camera" \
    --symlink-install --parallel-workers 2 --event-handlers console_cohesion+ \
    --cmake-args -DBUILD_TESTING=OFF -DPython3_EXECUTABLE=/usr/bin/python3
printf '\nCamera and dataset writer build complete. On the Jetson: source tools/camera_env.sh. See docs/CAMERAS.md and docs/RECORDING.md.\n'
