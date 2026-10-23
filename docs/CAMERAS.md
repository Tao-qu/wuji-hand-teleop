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
| Head left | Argus sensor 0 | `/cameras/head_left/image_raw` | RGB8, 640 x 480, 30 fps requested |
| Head right | Argus sensor 1 | `/cameras/head_right/image_raw` | RGB8, 640 x 480, 30 fps requested |
| Left wrist | Argus sensor 4 | `/cameras/left_wrist/image_raw` | RGB8, 640 x 480, 30 fps requested |
| Right wrist | Argus sensor 5 | `/cameras/right_wrist/image_raw` | RGB8, 640 x 480, 30 fps requested |
| D435 color | Serial `348122072721` | `/cameras/d435/color/image_raw` | RGB8, 640 x 480, 30 fps requested |
| D435 depth | Same D435 | `/cameras/d435/depth/image_rect_raw` | 16UC1, 640 x 480, 30 fps requested |

Each stream has a corresponding `camera_info` topic. The GMSL backend captures
Argus sensor mode 0 at 2560 x 1984 internally, removes 32 rows from both the top
and bottom, and scales the resulting 2560 x 1920 image to 640 x 480 with Jetson
hardware before converting to RGB and publishing to ROS. This preserves the
4:3 aspect ratio. The four views remain separate, without stitching or overlays.
The D435 supplies native 640 x 480 color and depth profiles. Its published depth
is `uint16` in millimeters. Only the 640 x 480 images are published and recorded;
no original-resolution image stream or file is produced.

For each GMSL role, `/cameras/<role>/preview/compressed` publishes JPEG previews
only while subscribed (default width 640, maximum 10 fps, quality 80). Previews
use the published 640 x 480 image. `/cameras/diagnostics` reports each GMSL
channel's observed rate, frame count, timestamp source, and calibration status.

The D435's optional aligned stream is
`/cameras/d435/aligned_depth_to_color/image_raw`; enable `align_depth.enable` in the
configuration to publish it alongside the unaligned 640 x 480 depth stream. D435 image
transport plugins expose compressed image topics when installed. The internal
RealSense TF tree is provided by its official driver. No robot-to-camera mounting
transform is invented for either camera family.

## Jetson prerequisites

- Ubuntu 22.04 and ROS 2 Humble on the Jetson.
- NVIDIA JetPack 6 / L4T 36.4.x, including `nvarguscamerasrc` and `nvvidconv`.
  The dataset writer uses the BSP's `nvv4l2h264enc` hardware encoder when available.
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

## Environment entry points

| Machine | Build | Environment |
| --- | --- | --- |
| Desktop | `./tools/setup.sh` | `source tools/env.sh` |
| Jetson | `./tools/setup_camera.sh` | `source tools/camera_env.sh` |

Run each environment command from that machine's checkout. Deployment builds
`install_camera/` on the Jetson only. The camera environment helper is tracked in
`tools/`; it loads system Python, ROS Humble, the optional private dependencies,
and the camera overlay, clearing inherited workspace paths.

## Install directly on the Jetson

Clone this repository on the Jetson, then:

```bash
cd ~/Workspace/wuji-hand-teleop
./tools/setup.sh --profile camera
source tools/camera_env.sh
```

`./tools/setup_camera.sh` is the equivalent camera-only entry point. The script
installs camera dependencies, creates missing camera configuration from templates,
and builds `src/camera`, `src/data_collection`, and `src/data_collection_interfaces` into `build_camera/`, `install_camera/`, and
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
sudo apt-get install build-essential cmake pkg-config python3-yaml python3-numpy python3-pip \
  python3-gi gir1.2-gstreamer-1.0 ffmpeg \
  python3-colcon-common-extensions libgstreamer1.0-dev \
  libgstreamer-plugins-base1.0-dev libopencv-dev libyaml-cpp-dev \
  gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad
./tools/setup_camera.sh --user-deps
```

The private dependency prefix is included by `tools/camera_env.sh`. The dataset
writer's pinned Arrow wheel is installed under `camera_dependencies/python`; the
system NumPy/OpenCV installation is preserved. The writer automatically uses
GStreamer's NVIDIA H.264 encoder through Python GObject Introspection and an
in-process `appsrc` pipeline when its plugins are available, with FFmpeg
`libx264` as the software alternative. Standard GStreamer base, good, and bad
plugins provide raw video parsing, color conversion, H.264 parsing, and MP4
muxing; NVIDIA plugins come from the Jetson BSP. These dependencies also apply
to `--skip-system` setups.
The ROS download step retrieves only missing ROS packages. Development-only dependencies of
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

The script uses SSH and rsync to copy `src/camera`, `src/data_collection`,
`src/data_collection_interfaces`, the shared recording requirements, setup,
environment, and configuration tools, and the camera/recording guides. It then
builds the camera and dataset writer workspace remotely. It does
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
**uncalibrated**. Replace each local file with that camera's measured intrinsics for
the published 640 x 480 view, including its crop and scale. Do not copy one camera's
intrinsics to the other three.
Images can be published without intrinsics. RealSense reads its own factory
calibration. Local YAML files are ignored by Git; templates are committed.

Launch loads the source configuration directory recorded during the build, so there
is one active copy, not separate source/install configuration sets. Rebuild if the
checkout is moved. Restart the camera launch after changing configuration.

## Start and stop

Open a Jetson terminal from the **desktop**:

```bash
ssh nvidia@6.6.7.100
```

Run these commands **on the Jetson**:

```bash
cd ~/Workspace/wuji-hand-teleop
source tools/camera_env.sh
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
ros2 run camera_bringup init_gmsl.sh
ros2 launch camera_bringup cameras.launch.py
```

The initializer requests sudo and performs the vendor `init 15` operation once per
boot. It does not run capture. Subsequent starts in the same boot reuse completed
initialization. Ctrl+C stops both camera processes. Only one application may own
the Argus cameras or D435 at a time; stop any other capture application first.

In a separate **desktop** terminal:

```bash
cd ~/Workspace/wuji-hand-teleop
source tools/env.sh
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
ros2 topic echo /cameras/head_left/camera_info --once --qos-reliability best_effort
```

The ROS exports apply to each terminal and its child processes; they do not change
system settings. Use the same domain on both hosts. A ROS CLI daemon started with
different settings can cache a different graph; run `ros2 daemon stop` in that
terminal before inspecting the graph again.

To start only one camera family **on the Jetson**:

```bash
ros2 launch camera_bringup gmsl.launch.py
ros2 launch camera_bringup d435.launch.py
```

The combined launch also accepts `gmsl:=false` or `d435:=false`. D435 alone does not
need the GEAC initializer. No camera command enables or moves the robot.

## Timing, bandwidth, and data collection

GMSL `Image` and `CameraInfo` share the same timestamp and optical frame ID. The
backend converts each buffer's GStreamer PTS and pipeline base time to Jetson
system time. If PTS is unavailable, it uses receive time; diagnostics identifies
which is in use. PTS has not been established as the exact exposure time. There
is no claim of hardware synchronization between four Argus sources or the D435.
Independent workers publish frames as they arrive. For each 30 Hz output tick,
the dataset writer selects the nearest cached source image without a timestamp
offset cutoff and preserves its original source timestamp. Gaps may reuse an
image and its timestamp; use those timestamps for postprocessing. See
[recording alignment](RECORDING.md#recorded-fields) for the arrival grace period.
RealSense timestamp conversion remains owned by the official driver.

At 30 fps, the five 640 x 480 RGB8 streams and one 640 x 480 16UC1 depth stream
carry approximately 156.7 MB/s (1.25 Gbit/s) before DDS overhead. The current
1 Gbit/s Jetson link cannot carry all six raw streams. Keep recording on the
Jetson and subscribe to JPEG previews from the desktop. The configured rate is
30 fps; diagnostics reports the observed capture rate. System clock synchronization
does not remove transport delay or align exposures.

`source tools/camera_env.sh` loads the project's Fast DDS transport profile for
both camera publishers and the dataset writer. It uses a 256 MiB shared-memory
segment per participant, a 16 MiB maximum message size, and an 8192-entry receive
queue for image messages. UDP remains available for discovery and communication
with the desktop. Start both processes from terminals that have sourced this
environment; the default 512 KiB shared-memory segment is too small for a single
640 x 480 RGB8 frame. See the [Fast DDS shared-memory documentation](https://fast-dds.docs.eprosima.com/en/2.6.x/fastdds/transport/shared_memory/shared_memory.html).

Start `ros2 run data_collection writer` in another Jetson terminal after capture
is running. The desktop's independent `ros2 run data_collection record` controls
episode start, stop, retry, and save. RGB encoding and lossless depth storage stay
on the Jetson; only robot numeric samples cross the network. See
[continuous recording](RECORDING.md) for the complete workflow.

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
