# MANUS–Tianji–Wuji Hand 2.2 Teleoperation

A ROS 2 Humble system for two **Wuji Hand 2.2** hands and two Tianji arms. Two MANUS gloves supply finger motion; five HTC Vive Trackers supply chest, hand-back, and upper-arm poses. Finger retargeting and hand communication use the official `wuji-sdk==2026.10.12`. RViz uses the official Hand 2.2 model.

Jetson camera capture provides four separate GMSL images and D435 color/depth streams. Camera acquisition can run independently of teleoperation. Dataset recording and policy inference are not implemented yet.

## Hardware and modes

| Mode | Input | Output |
| --- | --- | --- |
| Hands + arms | Two MANUS gloves and five Trackers | Two hands and two arms |
| Hands only | Two MANUS gloves | Two hands |
| Arms only | Five Trackers | Two Tianji arms |
| Home | No input devices | Configured hand and/or arm Home poses |
| Preview | MANUS gloves and five Trackers | RViz hand targets and Tracker axes |

Mount Trackers on the chest, the back of each hand, and the outside of each upper arm. Hands-only operation does not require SteamVR. Hand connections use SDK discovery and configured serial numbers; the arms use the configured cabinet address.

## Setup

Prepare Ubuntu 22.04 x86_64, system Python 3.10, Git, and ROS 2 Humble Desktop. For arm input, install SteamVR and pair all five Trackers in the same graphical session. MANUS requires vendor SDK licensing and receiver access. Configure the host network for the hands and Tianji cabinet. See [Local setup](docs/LOCAL_SETUP.md) for these prerequisites and native libraries.

Clone the repository and run setup:

```bash
git clone https://github.com/Tao-qu/wuji-hand-teleop.git
cd wuji-hand-teleop
./tools/setup.sh
```

Setup fetches Git LFS assets and the pinned official model, creates `.venv`, installs Python and ROS dependencies, prepares local configuration, builds the desktop robot packages with `--symlink-install`, and checks the environment. It installs required system packages and MANUS USB access rules through `sudo`. Running it again preserves existing local configuration. Use `--skip-system` when system dependencies and USB permissions are already managed, or `--skip-build` to prepare dependencies without building. For the separate Jetson camera workspace, use `./tools/setup.sh --profile camera`; see [camera deployment](docs/CAMERAS.md).

Load the environment in every terminal used for this project:

```bash
source tools/env.sh
```

### Configuration

Teleoperation uses one hand configuration, one arm configuration, and three calibration files:

| File | Purpose |
| --- | --- |
| `src/output_devices/wujihand2_output/wujihand2_ros2/config/wujihand2.yaml` | Hand serials and namespaces, Home positions, SDK rates and gains, MANUS parameters |
| `src/output_devices/tianji_output/tianji_output/config/tianji_chest.yaml` | Cabinet address, arm Home and control settings, tool geometry and dynamics, five Tracker serials |
| `src/input_devices/manus_input/manus_ros2/calibration/LeftMetaglovePro.mcal` | Left MANUS calibration |
| `src/input_devices/manus_input/manus_ros2/calibration/RightMetaglovePro.mcal` | Right MANUS calibration |
| `src/input_devices/openvr_input/calibration/tracker.yaml` | Chest transforms, hand-back-to-palm alignment and measured offsets, and upper-arm mounting rotations |

All five runtime files are local and ignored by Git. Each has an adjacent tracked `.template` file, including `LeftMetaglovePro.mcal.template`, `RightMetaglovePro.mcal.template`, and `tracker.yaml.template`. Setup or `python3 tools/configure.py --profile teleop` copies only missing files and preserves existing configuration and calibration. Add `--check` to check that these five files exist without writing. The camera profile initializes its own configuration and four intrinsic-calibration files; `configure.py` without a profile processes both sets.

Templates describe the supplied installation; the `.mcal.template` files are complete binary profiles, not editable text. Check serial numbers, cabinet IP, mounting dimensions, and operator calibration for your hardware before operation. Edit the source configuration files and use the calibration tools to update profiles; the symlink build makes them available to ROS.

The arm TCP is the robot palm, with identity flange-to-palm rotation and `[0, 0, 70]` mm translation on both sides. It is not a fingertip frame. The glove-mounted hand-back Trackers have separate offsets from the human palms: `hand_back_to_palm.left/right.tracker_offset_m` records palm-to-Tracker displacement in aligned human palm axes, supplied as `[0, -0.05, 0]` m on the left and `[0, 0.05, 0]` m on the right. Measure these offsets for the wearer and mounting, then [calibrate against arm Home](docs/STEAMVR.md#neutral-pose-calibration).

`manus.left_calibration_file` and `manus.right_calibration_file` select the MANUS profiles by filename. `openvr.calibration_file: tracker.yaml` selects the Tracker profile. Calibration filenames resolve in their respective package calibration directories; absolute paths are also supported. Teleoperation and Preview read the same MANUS settings. See [configuration fields and units](docs/LOCAL_SETUP.md#configuration-reference), [MANUS calibration](src/input_devices/manus_input/manus_ros2/CALIBRATION_GUIDE.md), and [Tracker calibration](docs/STEAMVR.md#neutral-pose-calibration).

## Start teleoperation

```bash
source tools/env.sh
ros2 run wuji_teleop_monitor monitor
```

Select hands + arms, hands only, or arms only, then click **Start Teleop**. **Selected hands and arms auto-enable.** The arms move to Home, prepare for three seconds, and begin Tracker following. In combined mode the hands wait for arm readiness; hands-only mode has its own three-second preparation interval.

The monitor provides joint feedback, currents, command rates, individual enable/disable buttons, and hand driver status. **Scan SNs** discovers hands and saves their serial numbers to the local hand configuration. **Stop Teleop** or terminal Ctrl-C stops the launch process group and closes its drivers.

The same modes are available from the command line:

```bash
ros2 run wuji_teleop_bringup wuji_teleop.launch.py                # Hands + arms
ros2 run wuji_teleop_bringup wuji_teleop.launch.py --disable-arm  # Hands only
ros2 run wuji_teleop_bringup wuji_teleop.launch.py --disable-hand # Arms only
```

| Option | Meaning |
| --- | --- |
| `--disable-hand` | Omit both hands and MANUS |
| `--disable-arm` | Omit both arms and Trackers |
| `--home` | Return selected devices Home, then exit |
| `--help` | Show command usage |

The two disable switches cannot be combined. Teleoperation and Home always use the local `wujihand2.yaml` and `tianji_chest.yaml`. Use `ros2 run` for these switches; native `ros2 launch` expects `name:=value` arguments. A bare `ros2 launch wuji_teleop_bringup wuji_teleop.launch.py` starts the default hands + arms mode.

Individual enable/disable services are also available:

```bash
ros2 service call /left_hand/set_enabled std_srvs/srv/SetBool '{data: true}'
ros2 service call /right_hand/set_enabled std_srvs/srv/SetBool '{data: true}'
ros2 service call /tianji_arm/set_enabled std_srvs/srv/SetBool '{data: false}'
```

Hand service responses acknowledge queued SDK operations. Check lifecycle and diagnostics for their results. The monitor expects the `/left_hand` and `/right_hand` namespaces.

### Return to Home

Stop the current session, select the desired mode, and click **Return Home**. No gloves or Trackers are required. **Stop Home** interrupts the operation; **Start Teleop** starts input following again.

```bash
ros2 run wuji_teleop_bringup wuji_teleop.launch.py --home
ros2 run wuji_teleop_bringup wuji_teleop.launch.py --home --disable-hand
ros2 run wuji_teleop_bringup wuji_teleop.launch.py --home --disable-arm
```

Hand Home uses 20 joint positions in radians from each hand's `home_joints`, in thumb, index, middle, ring, pinky order, four joints each. Arm Home uses seven joint positions per arm in degrees and `controller.home_duration_sec`. The supplied duration is 8 seconds; hands-only Home publishes targets for 8 seconds. Initialization and shutdown add time outside the motion interval. The session exits and closes the selected devices when the operation ends.

The monitor shows feedback and `/wuji_home/state`. `ACTIVE` means hand targets have been published and the selected arm Home call has completed; it does not measure hand arrival. Home calls the shared arm driver's `/tianji_arm/home` service, whose response reports the completed operation. `/tianji_arm/home_state` provides its display status. Submitted arm commands, including Home interpolation, appear on `/tianji_arm/commanded_joint_states`.

### Arm brake tool

Stop the arm session before opening the separate brake tool, which connects directly to the cabinet:

```bash
ros2 run wuji_teleop_monitor brake
```

Optional desktop shortcuts can be installed with `bash src/wuji_teleop_monitor/install_desktop.sh`.

## Preview inputs and targets

```bash
source tools/env.sh
ros2 launch wuji_teleop_bringup wuji_preview.launch.py
```

Preview starts the configured MANUS profiles, five Trackers, Hand 2.2 target models, and RViz without actuator drivers. It uses `/preview/*` topics. The models show requested finger geometry, and Tracker axes show input poses. Labels describe the last received inputs and targets; they do not expire when input stops. Close RViz or press Ctrl-C to stop.

Use `start_inputs:=false` to omit input readers or `enable_rviz:=false` to omit RViz. Preview and standalone OpenVR also accept `arm_config:=/absolute/path/to/tianji_chest.yaml`; the default is the project's local arm configuration.

## Data flow and code layout

```text
MANUS → manus_ros2 → 21 keypoints per hand
      → official RetargetSession(WujiHand2)
      → /{side}_hand/joint_commands → wujihand2_driver → SDK → hand

5 Trackers → OpenVR → TF → tracker_arm_retarget
      → /{side}_arm/tcp_target + /{side}_arm/upper_arm_reference
      → tianji_arm_controller (local IK)
      → /tianji_arm/joint_commands → /tianji_arm/driver → SDK → arms
      ← /{side}_arm/joint_states ← /tianji_arm/driver
```

Finger retargeting receives a `(21, 3)` array in metres in MediaPipe landmark order and outputs 20 positions in radians in firmware order. `controller.flip_y: false` preserves the publisher's right-handed coordinates. Each hand has one persistent SDK retargeting session. MANUS and hand controllers run at 120 Hz by default; the enabled hand bridge sends its latest target at 200 Hz. See the [MANUS input contract](src/input_devices/manus_input/README.md#input-contract) and [hand ROS interfaces](src/output_devices/wujihand2_output/README.md).

| Directory | Implementation |
| --- | --- |
| `src/input_devices/manus_input/` | MANUS publisher, messages, and calibration |
| `src/input_devices/openvr_input/` | Five-Tracker acquisition, calibration, and TF |
| `src/controller/` | Hand retargeting, Tracker-to-palm arm targets, and arm IK |
| `src/output_devices/wujihand2_output/wujihand2_ros2/` | Hand SDK adapter, retargeting, and configuration |
| `src/output_devices/wujihand2_output/wuji-description/hand2/hand2_2/body/` | Pinned official Hand 2.2 model |
| `src/output_devices/tianji_output/` | Shared Tianji ROS driver, pure kinematics, and tool parameters |
| `src/wuji_teleop_bringup/` | Teleoperation, Home, Preview, and static transforms |
| `src/wuji_teleop_monitor/` | Qt monitor and brake tool |
| `src/camera/gmsl_camera/` | Four full-resolution GMSL streams and optional JPEG previews |
| `src/camera/camera_bringup/` | Jetson camera configuration, initialization, and GMSL/D435 launch |
| `tools/` | Setup, environment, configuration, and dependency tools |

### Shared robot interfaces

Teleoperation and Home use the same hardware drivers. `/tracker_arm_retarget` reads calibrated human-palm and upper-arm TF and publishes targets in each robot arm's base frame. `/tianji_arm_controller` consumes these typed targets, measured joints, and driver lifecycle state to run local IK. It loads the robot model and palm TCP configuration without owning a cabinet connection.

| Interface | Meaning |
| --- | --- |
| `/{side}_hand/joint_commands` | Twenty named hand target positions, radians |
| `/{side}_arm/tcp_target` | `geometry_msgs/msg/PoseStamped`: robot palm target in `{side}_base`, position in metres, quaternion in `x, y, z, w` order |
| `/{side}_arm/upper_arm_reference` | `geometry_msgs/msg/Vector3Stamped`: unit upper-arm reference direction in the same `{side}_base` frame |
| `/tianji_arm/joint_commands` | One named joint command for both arms (normally 14 positions), radians |
| `/{side}_hand/joint_states` | Measured hand position and velocity, radians and rad/s |
| `/{side}_arm/joint_states` | Measured arm position, velocity, and torque, radians, rad/s, and Nm |
| `/{side}_hand/commanded_joint_states` | Hand target submitted successfully to the SDK |
| `/tianji_arm/commanded_joint_states` | Arm targets submitted together to the SDK, including Home |
| `/tianji_arm/lifecycle_state` | Shared arm readiness and operation state |
| `/tianji_arm/home_state` | Driver Home request status: MOVING, ACTIVE, or ERROR |

The joint topics use `sensor_msgs/msg/JointState`. Arm command names are `left_joint_1` through `left_joint_7`, followed by `right_joint_1` through `right_joint_7`. One complete seven-joint side is also accepted when only that side has an IK result; the driver submits the included sides in one SDK call. Hand names follow the official model's `l_`/`r_` names in [firmware order](src/output_devices/wujihand2_output/README.md#topics-and-services). The upper-arm reference uses Tracker orientation only; its measured position is unused.

All joint ROS interfaces use radians; arm Home configuration and the native Tianji SDK use degrees. The monitor converts arm feedback to degrees for display. Submission timestamps identify SDK submission, not physical arrival. Hand feedback is stamped in the SDK receive callback; arm feedback is stamped when reading the SDK cache, which may repeat a hardware sample. `/left_arm/ee_pose` and `/right_arm/ee_pose` remain compatibility arrays for target palm TCP poses in metres/degrees, not measured end-effector poses.

The arm driver provides `/tianji_arm/set_enabled`, `/tianji_arm/set_arm_state`, and `/tianji_arm/{left,right}/{brake,clear_error}` (`SetBool`), plus `/tianji_arm/home` and `/tianji_arm/arm_status` (`Trigger`). A policy or another target source can publish the same `tcp_target` and `upper_arm_reference` messages to the generic IK controller without OpenVR or Tracker retargeting. A joint-level command source can instead reuse the hardware command interface. Use one command source at a time; the driver connection, Home, enable/disable, and feedback interfaces are shared across sources.

## Jetson cameras

On the **desktop**, deploy the camera source and build it on the Jetson:

```bash
./tools/deploy_camera.sh --user-deps
```

This uses the existing Jetson native dependencies and downloads missing ROS camera packages into a local prefix. The default destination is `nvidia@6.6.7.100:/home/nvidia/Workspace/wuji-hand-teleop`. For a new Jetson, install the prerequisites described in [Cameras](docs/CAMERAS.md) first.

Connect to the **Jetson**, then initialize and start capture there:

```bash
ssh nvidia@6.6.7.100
cd ~/Workspace/wuji-hand-teleop
source tools/camera_env.sh
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
ros2 run camera_bringup init_gmsl.sh
ros2 launch camera_bringup cameras.launch.py
```

`/cameras/gmsl` publishes independent `head_left`, `head_right`, `left_wrist`, and `right_wrist` images, each at 2560 × 1984 with 30 fps requested. `/cameras/d435` uses the official RealSense driver for 1280 × 720 color and depth at 30 fps. Ctrl-C stops both processes. `gmsl:=false` or `d435:=false` selects one camera family.

On the **desktop**, load the teleoperation environment for ROS subscriptions:

```bash
cd ~/Workspace/wuji-hand-teleop
source tools/env.sh
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
```

`tools/camera_env.sh` loads the Jetson camera build; `tools/env.sh` loads the desktop teleoperation build. Deploying cameras does not create `install_camera/` on the desktop.

Keep full-resolution capture and recording on the Jetson: the four GMSL RGB streams alone would require about 14.6 Gbit/s at 30 fps. Desktop viewers can use `/cameras/<role>/preview/compressed`. See [camera topics, configuration, timestamps, and deployment](docs/CAMERAS.md) for details.

## Environment checks and documentation

```bash
source tools/env.sh
python3 tools/check_environment.py --mode all
ros2 topic hz /left_hand/joint_states
ros2 topic echo /left_hand/lifecycle_state
ros2 topic echo /left_hand/diagnostics
```

The environment check verifies dependencies, configuration files, calibration files, and retargeting sessions without connecting to hardware. Device connectivity and physical calibration must be checked during setup for the actual hardware. The [documentation index](docs/README.md) covers setup, operation, calibration, and device interfaces.

## License

See [LICENSE](LICENSE). MANUS, Tianji, Wuji SDK, and model resources retain their respective licenses and copyright notices.
