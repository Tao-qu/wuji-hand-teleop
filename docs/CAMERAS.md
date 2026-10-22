# Jetson cameras

The repository contains all custom code deployed to the Jetson. Camera capture runs
on the Jetson; the desktop can subscribe to previews and robot topics. The camera
workspace does not load the MANUS, SteamVR, Wuji, or Tianji control libraries.

## Nodes and streams

Two processes run under `/cameras`:

- `/cameras/gmsl`: four independent Argus/GStreamer pipelines and capture threads.
- `/cameras/d435`: the official `realsense2_camera` executable.

| Camera | Device | Image topic | Default |
| --- | --- | --- | --- |
| Head left | Argus sensor 0 | `/cameras/head_left/image_raw` | RGB8, 2560 x 1984, 30 fps requested |
| Head right | Argus sensor 1 | `/cameras/head_right/image_raw` | RGB8, 2560 x 1984, 30 fps requested |
| Left wrist | Argus sensor 4 | `/cameras/left_wrist/image_raw` | RGB8, 2560 x 1984, 30 fps requested |
| Right wrist | Argus sensor 5 | `/cameras/right_wrist/image_raw` | RGB8, 2560 x 1984, 30 fps requested |
| D435 color | Serial `348122072721` | `/cameras/d435/color/image_raw` | RGB8, 1280 x 720, 30 fps |
| D435 depth | Same D435 | `/cameras/d435/depth/image_rect_raw` | 16UC1, 1280 x 720, 30 fps |

Each stream has a corresponding `camera_info` topic. RealSense depth values are
millimeters in the published ROS image. The GMSL images are full sensor-output
resolution, with color conversion only: no rectification, resize, rotation, overlay,
or stitching. These are RGB images from the Argus NV12 stream, not Bayer RAW.

For each GMSL role, `/cameras/<role>/preview/compressed` publishes JPEG previews
only while subscribed (default width 640, maximum 10 fps, quality 80). Preview
resizing does not change the raw image. `/cameras/diagnostics` reports each GMSL
channel's observed rate, frame count, timestamp source, and calibration status.

The D435's optional aligned stream is
`/cameras/d435/aligned_depth_to_color/image_raw`; enable `align_depth.enable` in the
configuration to publish it alongside the original depth stream. D435 image
transport plugins expose compressed image topics when installed. The internal
RealSense TF tree is provided by its official driver. No robot-to-camera mounting
transform is invented for either camera family.

## Jetson prerequisites

- Ubuntu 22.04 and ROS 2 Humble on the Jetson.
- NVIDIA JetPack 6 / L4T 36.4.x, including `nvarguscamerasrc` and `nvvidconv`.
- Tianji's GEAC board support package, device tree, and sensor drivers. The supplied
  initializer is expected at `/home/nvidia/cam_geac/rb_camera.sh`; override its
  directory with `GMSL_VENDOR_DIR` when needed. These board-specific vendor files
  are prerequisites, not copies of the previous streaming application.
- D435 connected over USB 3, with RealSense udev rules installed.
- Desktop and Jetson on a common network and synchronized system clocks. Use the
  same `ROS_DOMAIN_ID` on both machines and `ROS_LOCALHOST_ONLY=0` for desktop
  subscriptions. The camera environment preserves these shell settings.
- SSH access and rsync on both machines for the remote deployment script.

The build uses Ubuntu GStreamer/OpenCV/yaml-cpp development packages and the
[official RealSense ROS wrapper](https://github.com/realsenseai/realsense-ros).
It does not require `jetson-utils` or the previous application's shared library.
The Argus pipeline follows NVIDIA's
[JetPack 6 GStreamer interface](https://docs.nvidia.com/jetson/archives/r36.4.4/DeveloperGuide/SD/Multimedia/AcceleratedGstreamer.html).

## Install directly on the Jetson

Clone this repository on the Jetson, then:

```bash
cd ~/Workspace/wuji-hand-teleop
./tools/setup.sh --profile camera
source install_camera/camera_env.sh
```

`./tools/setup_camera.sh` is the equivalent camera-only entry point. The script
installs camera dependencies, creates missing camera configuration from templates,
and builds only `src/camera` into `build_camera/`, `install_camera/`, and
`log_camera/`. It never starts capture or changes the running camera service.
Existing local configuration and calibration are preserved.

Options:

| Option | Effect |
| --- | --- |
| `--skip-system` | Use dependencies already installed; skip apt installation. |
| `--skip-build` | Prepare dependencies and configuration only. |
| `--user-deps` | Download missing official RealSense ROS packages and dependencies into `camera_dependencies/`, without sudo. |

For `--user-deps`, ROS Humble and native build dependencies must already exist:

```bash
sudo apt-get install build-essential cmake pkg-config python3-yaml \
  python3-colcon-common-extensions libgstreamer1.0-dev \
  libgstreamer-plugins-base1.0-dev libopencv-dev libyaml-cpp-dev
./tools/setup_camera.sh --user-deps
```

The private dependency prefix is included by `install_camera/camera_env.sh`.
This mode downloads only missing ROS packages. Development-only dependencies of
prebuilt ROS packages are reported and skipped; missing native runtime dependencies
are reported with an apt installation command. The RealSense component's dynamic
library dependencies are checked before setup completes. Package
extraction does not install udev rules or run system package maintainer scripts;
reuse the Jetson's existing RealSense rules, or install the official rules as an
administrator on a new machine. Firmware is not updated by these scripts.

## Deploy from the desktop

```bash
cd ~/Workspace/wuji-hand-teleop
./tools/deploy_camera.sh --user-deps
```

The defaults are `nvidia@6.6.7.100` and
`/home/nvidia/Workspace/wuji-hand-teleop`. Override them with:

```bash
./tools/deploy_camera.sh --host nvidia@JETSON_IP \
  --path /home/nvidia/Workspace/wuji-hand-teleop --user-deps
```

The script uses SSH and rsync to copy `src/camera`, the camera setup and configuration
tools, and this document. It then performs the camera-only build remotely. It does
not transfer desktop build products, SDKs, credentials, or local camera YAML files.
Existing destination camera YAML files are preserved; missing files are initialized
from the transferred templates. System installation through the deployment script
requires an interactive terminal for sudo; use `--user-deps` for a noninteractive
SSH deployment with existing native prerequisites. Review device mapping and serial numbers on each
Jetson after deployment. Use `--skip-system` when all dependencies are already
installed, or `--skip-build` to prepare without compiling.

## Configuration and intrinsics

The only editable camera configuration is:

```text
src/camera/camera_bringup/config/cameras.yaml
```

Four camera-specific calibration files are in:

```text
src/camera/camera_bringup/calibration/head_left.yaml
src/camera/camera_bringup/calibration/head_right.yaml
src/camera/camera_bringup/calibration/left_wrist.yaml
src/camera/camera_bringup/calibration/right_wrist.yaml
```

Templates contain zero camera matrices: ROS `CameraInfo.k[0] == 0` means
**uncalibrated**. Replace each local file with that camera's measured intrinsics at
the configured resolution. Do not copy one camera's intrinsics to the other three.
Raw images remain available without intrinsics. RealSense reads its own factory
calibration. Local YAML files are ignored by Git; templates are committed.

Launch loads the source configuration directory recorded during the build, so there
is one active copy, not separate source/install configuration sets. Rebuild if the
checkout is moved. Restart the camera launch after changing configuration.

## Start and stop

On the supplied Jetson, the existing `tj-camera-stream.service` owns the same Argus
cameras. Stop it before starting the new GMSL node:

```bash
sudo systemctl stop tj-camera-stream.service
cd ~/Workspace/wuji-hand-teleop
source install_camera/camera_env.sh
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
ros2 run camera_bringup init_gmsl.sh
ros2 launch camera_bringup cameras.launch.py
```

The explicit initializer uses the vendor `init 15` operation once per boot and
shares the existing service's initialization marker. It does not run capture.
Once initialization has completed, subsequent starts in the same boot reuse it.
Stop the launch with Ctrl+C. To return to the existing camera application:

```bash
sudo systemctl start tj-camera-stream.service
```

In the desktop viewer terminal, source `tools/env.sh` and set the same
`export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0`. These exports apply to that terminal
and its child processes; they do not change system settings.

On a new Jetson without that service, omit the two `systemctl` commands. To start
only one camera family:

```bash
ros2 launch camera_bringup gmsl.launch.py
ros2 launch camera_bringup d435.launch.py
```

The combined launch also accepts `gmsl:=false` or `d435:=false`. Launching D435 alone
does not require stopping the GMSL-only service, provided no other application
holds the D435. No camera command enables or moves the robot.

## Timing, bandwidth, and data collection

GMSL `Image` and `CameraInfo` share the same timestamp and optical frame ID. The
backend converts each buffer's GStreamer PTS and pipeline base time to Jetson
system time. If PTS is unavailable, it uses receive time; diagnostics identifies
which is in use. PTS has not been established as the exact exposure time. There
is no claim of hardware synchronization between four Argus sources or the D435.
Independent workers publish frames as they arrive; future recording/observation
assembly must match timestamps explicitly. RealSense timestamp conversion remains
owned by the official driver.

The default GMSL raw payload alone is approximately 1.83 GB/s (14.6 Gbit/s), before
D435 and DDS overhead. The current 1 Gbit/s Jetson link cannot carry it all. Keep
full-resolution recording/processing on the Jetson, subscribe to JPEG previews
from the desktop, and send only policy-sized images for inference. Actual capture
and storage throughput must be selected for the intended recording mode. System
clock synchronization does not remove transport delay or align exposures.

For a local software/device inspection after starting capture:

```bash
ros2 node list
ros2 topic list
ros2 topic echo /cameras/diagnostics --once
ros2 topic echo /cameras/head_left/camera_info --once --qos-reliability best_effort
```

Image streams use sensor-data QoS. Subscribe with best-effort reliability and a
small queue. Avoid measuring all raw stream rates across the desktop link: that
measurement itself transfers the full image payload.
