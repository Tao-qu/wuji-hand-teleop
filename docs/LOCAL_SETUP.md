# Local setup and configuration

## Prerequisites

Use Ubuntu 22.04 x86_64 with system Python 3.10, Git, and ROS 2 Humble Desktop installed under `/opt/ros/humble`. The [ROS 2 Humble installation guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) describes the ROS package sources and installation.

This page covers the desktop robot workspace (`./tools/setup.sh --profile teleop`, the default). The Jetson camera workspace has separate dependencies and build outputs; see [Cameras](CAMERAS.md) for `--profile camera` and remote deployment.

Prepare device requirements separately:

- MANUS gloves and receivers with vendor SDK licensing. Setup installs the repository's USB rules and adds the current user to `plugdev` and `dialout`.
- Steam and SteamVR in the same user's graphical session, with five paired Trackers and Lighthouse base stations. See [SteamVR and Tracker setup](STEAMVR.md). Hands-only operation does not use SteamVR.
- A PCsensor USB foot pedal (`3553:b001`) for standalone pedal events. Setup installs its access rule for the active local desktop user; the pedal need not be connected during setup.
- Network access to both Wuji hands for SDK discovery and to the configured Tianji cabinet endpoint.
- Internet access for Git LFS, the pinned model, Python packages, and ROS dependencies. System installation requires `sudo`.

The repository includes the MANUS Integrated SDK and Tianji native libraries as Git LFS assets. Vendor licensing remains a separate prerequisite. Setup retrieves the libraries from Git LFS for the project runtime.

## Install and build

Run this command from the repository root:

```bash
./tools/setup.sh
```

Setup performs the following steps:

1. Installs required system packages, MANUS and PCsensor foot pedal USB rules, and device-access groups.
2. Fetches Git LFS assets and the official model revision in `dependencies.lock.json`.
3. Creates missing hand, arm, OpenVR, and recording configurations and three calibration files from their templates.
4. Creates a Python 3.10 `.venv` with system packages available and installs `requirements.txt`, including `evdev==1.9.3` for the pedal and the shared recording requirements.
5. Resolves ROS dependencies, builds the workspace including `foot_pedal` with `--symlink-install`, and checks the environment.

Existing local configuration and calibration are preserved. `./tools/setup.sh --skip-system` skips system package, rosdep installation, and USB permission changes; use it when these are already configured. `--skip-build` skips the build and checks that require an installed workspace. `--help` lists the options.

After a device-group change, log out and back in. Reconnect the MANUS receivers and foot pedal after USB rules are installed. The pedal rule creates `/dev/input/pcsensor-foot-pedal` and grants access through the active local desktop session; it does not grant general keyboard access. Load the project environment in each terminal:

```bash
source tools/env.sh
```

`requirements.txt` pins `wuji-sdk==2026.10.12`, which supplies the Hand 2 retargeting runtime and network interface. The official model checkout at `src/output_devices/wujihand2_output/wuji-description` provides `wuji_hand2_2_description` from `hand2/hand2_2/body`. `tools/fetch_dependencies.py` enforces the locked revision and preserves existing checkout changes by rejecting a conflicting checkout.

For a source-only rebuild after setup:

```bash
source tools/env.sh
python3 -m colcon build --symlink-install \
  --base-paths src/input_devices src/output_devices src/controller \
    src/wuji_teleop_bringup src/wuji_teleop_monitor src/foot_pedal \
    src/data_collection src/data_collection_interfaces \
  --cmake-args -DPython3_EXECUTABLE="$VIRTUAL_ENV/bin/python3"
source tools/env.sh
python3 tools/check_environment.py --mode all
```

Build outputs contain workspace and interpreter paths. Rebuild with fresh `build/`, `install/`, and `log/` directories if those paths change. `tools/env.sh` loads ROS Humble and this checkout, clears inherited workspace paths, and excludes user-installed Python packages from `~/.local`.

## Native libraries

`tools/env.sh` selects the workspace Python environment and adds the bundled MANUS and Tianji library directories to `LD_LIBRARY_PATH`. The MANUS publisher uses `libManusSDK_Integrated.so`; Tianji uses `libMarvinSDK.so` for cabinet control and `libKine.so` for kinematics. All three libraries are retrieved through Git LFS during setup.

The cabinet address is configured in `tianji_chest.yaml`. The control wrapper sends a VERSION request after connecting to register its feedback destination. Keep the Python kinematics binding paired with the bundled `libKine.so`: its IK parameter structure is 472 bytes and includes `m_Output_JntExdABS`. Restart arm processes after changing native libraries or bindings.

## Configuration reference

The six teleoperation configuration and calibration files are local and ignored by Git:

- `src/output_devices/wujihand2_output/wujihand2_ros2/config/wujihand2.yaml`
- `src/output_devices/tianji_output/tianji_output/config/tianji_chest.yaml`
- `src/input_devices/openvr_input/config/openvr.yaml`
- `src/input_devices/manus_input/manus_ros2/calibration/LeftMetaglovePro.mcal`
- `src/input_devices/manus_input/manus_ros2/calibration/RightMetaglovePro.mcal`
- `src/input_devices/openvr_input/calibration/tracker.yaml`

Each has an adjacent tracked file with `.template` appended to its name. Setup or `python3 tools/configure.py --profile teleop` copies only missing files, preserving existing configuration and calibration. This also creates `src/data_collection/config/recording.yaml` from its template; see [recording configuration](RECORDING.md#configuration-and-arguments). The templates hold the supplied installation's settings and profiles. The `.mcal.template` files are complete binary MANUS profiles, not editable text; use `manus_calibrate` to produce a new runtime profile. Check device serials, network addresses, tool geometry, and calibration for the actual installation. `python3 tools/configure.py --profile teleop --check` reports missing runtime files without writing. Use `--profile camera` for camera settings or omit `--profile` to include both sets.

The source files are the editable configuration for a symlink build. Teleoperation and Home use the default hand and arm configurations; arm teleoperation also uses the OpenVR input configuration. Preview and standalone OpenVR support an explicit `input_config` launch argument. A regular install reads its package-share configuration copies.

### Hands and MANUS

| Field | Meaning |
| --- | --- |
| `left_hand.serial_number`, `right_hand.serial_number` | Hand serials used for SDK discovery |
| `left_hand.name`, `right_hand.name` | ROS namespaces; the monitor expects `left_hand` and `right_hand` |
| `left_hand.home_joints`, `right_hand.home_joints` | 20 positions in radians: thumb, index, middle, ring, pinky; four joints each |
| `driver.command_rate` | SDK command rate in Hz; supplied value 200 |
| `driver.publish_rate` | ROS feedback publication timer in Hz; supplied value 200 |
| `driver.diagnostics_rate` | Diagnostics and lifecycle rate in Hz; supplied value 10 |
| `driver.kp`, `driver.kd` | SDK MIT gains; supplied configuration uses 5.0 and 0.1 |
| `controller.control_rate` | Retargeting loop rate in Hz; supplied value 120 |
| `controller.flip_y` | Optional Y reflection; supplied value `false` preserves right-handed MANUS XYZ |
| `manus.left_calibration_file`, `manus.right_calibration_file` | Calibration filenames or absolute paths |

Bringup sets `auto_enable: true` for selected hand drivers. The canonical profiles are `LeftMetaglovePro.mcal` and `RightMetaglovePro.mcal` in `src/input_devices/manus_input/manus_ros2/calibration`. An empty parameter selects the corresponding canonical file. Preview uses the same configured MANUS profiles and `controller.flip_y` setting as live retargeting. See [MANUS calibration](../src/input_devices/manus_input/manus_ros2/CALIBRATION_GUIDE.md).

### Arms

| Field | Meaning |
| --- | --- |
| `left_arm.home_joints`, `right_arm.home_joints` | Seven joint positions per arm in degrees |
| `left_arm.base_from_body`, `right_arm.base_from_body` | 3×3 rotations mapping common robot body axes (X forward, Y left, Z up) into each arm base |
| `driver.robot_ip` | Cabinet SDK endpoint |
| `driver.kinematics_model` | Filename in the Tianji package `config` directory, or an absolute path |
| `driver.tool_transforms.left`, `.right` | Flange-to-robot-palm TCP 4×4 transforms, translation in millimetres |
| `driver.tool_dynamics` | Payload source and per-arm mass, COM, and inertia |
| `controller.control_rate` | Arm command rate in Hz; supplied value 120 |
| `controller.state_publish_rate` | Feedback publication rate in Hz; supplied value 500 |
| `controller.home_duration_sec` | Home trajectory duration in seconds; supplied value 8 |
| `controller.handoff_hold_sec`, `handoff_ramp_sec` | Measured-pose hold and blend into Tracker following, in seconds |

The supplied arm Home is left `[55, -65, -70, -60, 60, 0, 0]` and right `[-55, -65, 70, -60, -60, 0, 0]` degrees. Normal startup enters position mode, moves Home, then enters joint-impedance mode. The launch preparation interval is fixed at three seconds. Disable requests standby while keeping the connection. The shared `/tianji_arm/driver` owns the connection and serializes SDK operations; an enable or disable request waits for an operation already running. `/tracker_arm_retarget` reads TF and publishes palm targets and upper-arm directions. `/tianji_arm_controller` runs pure local IK from those typed messages, measured joint feedback, and driver lifecycle state.

Arm ROS joint commands and feedback use radians, velocities use rad/s, and measured torques use Nm. Degree conversion happens at the hardware and local-IK boundaries; the Home YAML remains in degrees. The monitor displays arm values in degrees. `/{side}_arm/tcp_target` is a `PoseStamped` in `{side}_base` with metres and an `x, y, z, w` quaternion. `/{side}_arm/upper_arm_reference` is a `Vector3Stamped` unit direction in that same base frame. A policy can publish this pair without OpenVR; use one command source at a time. See [shared robot interfaces](../README.md#shared-robot-interfaces) for topics and services.

### Tool geometry and payload

The robot TCP is the palm, with axes parallel to the flange axes. Both supplied flange-to-palm transforms have identity rotation and translation `[0, 0, 70]` mm. This mounting places flange +Z toward the fingers, with the hand back facing flange −Y on the left and +Y on the right. The TCP is not a fingertip. Measure the robot installation, including its adapter, and update these values when needed. Local Tianji IK applies this transform once.

The left/right 50 mm displacement describes the human glove-mounted hand-back Tracker relative to the human palm. It belongs in the Tracker profile below, separately from robot tool geometry.

`driver.tool_dynamics.source: configured` sends the separate `left` and `right` ten-element arrays to the SDK. `source: controller` uses the cabinet's existing payload parameters. Array order and units are:

```text
[mass, com_x, com_y, com_z, Ixx, Ixy, Ixz, Iyy, Iyz, Izz]
 mass: kg; COM: mm; inertia about the flange origin: kg·m²
```

The supplied values model each Hand 2.2 in its zero pose with the supplied mount, approximately 0.688 kg per side. Additional cabling and mounting parts are excluded. The conversion assumes coincident mount and flange origins and a mount-to-flange rotation of `diag(1, -1, -1)`. For a different mounting, transform the COM and inertia into the flange frame, including a parallel-axis shift if the origins differ. The driver passes the configured arrays to the SDK without unit conversion.

### OpenVR input

`src/input_devices/openvr_input/config/openvr.yaml` contains these top-level fields:

| Field | Meaning |
| --- | --- |
| `tracker_serials` | Distinct serials for `chest`, `left_hand_back`, `right_hand_back`, `left_arm`, `right_arm` |
| `publish_rate_hz` | Tracker publication rate in Hz; supplied value 120 |
| `publish_tf` | Publish Tracker and anatomical palm TF; supplied value `true` |
| `parent_frame` | World frame; supplied value `world` |
| `calibration_file` | Runtime Tracker profile filename or absolute path; supplied value `tracker.yaml` |

These input settings are independent of the robot arm configuration. The runtime OpenVR node loads this file and its selected human calibration profile without loading robot Home, tool geometry, or kinematics.

### Tracker calibration

`src/input_devices/openvr_input/calibration/tracker.yaml` is the canonical human input profile. Its fields are:

| Field | Meaning |
| --- | --- |
| `chest_rotation` | 3×3 mounting rotation whose columns are human body axes in raw chest Tracker coordinates: X forward, Y left, Z up |
| `hand_back_to_palm.left`, `.right` | `qx`, `qy`, `qz`, `qw` orientation of the anatomical palm in raw hand-back Tracker coordinates, plus `tracker_offset_m`: palm-to-Tracker displacement in palm axes, metres |
| `neutral_pose.left.position_m`, `.right.position_m` | Captured neutral human palm positions in the human chest frame, metres |
| `neutral_pose.left.quaternion_xyzw`, `.right.quaternion_xyzw` | Neutral anatomical palm orientations in the human chest frame |
| `neutral_pose.left.upper_arm_quaternion_xyzw`, `.right.upper_arm_quaternion_xyzw` | Per-side mean raw upper-arm Tracker orientations at neutral, in the human chest frame |

Both anatomical palms use X toward the fingers, Z toward the hand back, and Y = Z × X. Both supplied `tracker_offset_m` vectors are `[0, 0, 0.05]`. The static TF from `{side}_hand_back` to `{side}_human_palm` uses the quaternion rotation `R` and translation `-R @ tracker_offset_m`. The human chest origin remains at the raw chest Tracker, rather than an estimated shoulder position. Upper-arm Tracker positions are unused for IK; the saved raw orientation does not identify anatomical upper-arm twist.

Measure and edit the offsets in the calibration output profile before running calibration. `tracker_calibrate --calibrate` reads and saves `tracker.yaml`, or the filename supplied with `--output`; a missing output file starts from the bundled `tracker.yaml.template`. This input choice is independent of `calibration_file` in `openvr.yaml`. Inspection always reads `tracker.yaml` with the same fallback.

The CLI reads Tracker settings from `openvr.yaml` and robot Home and tool geometry from `tianji_chest.yaml`. It displays the locally computed Home reference without connecting to the cabinet. The input fit uses a symmetric, palms-down, fingers-forward human neutral pose to fit hand mounting rotations and record neutral palm poses and raw upper-arm orientations. It preserves the chest mounting rotation and measured offsets; a static capture cannot measure the mounting distance. Robot geometry does not enter this fit.

At runtime, the retargeting controller uses `base_from_body` to map motion from the saved neutral pose onto robot Home, including the Home upper-arm reference direction. Restart teleoperation after changing robot Home or tool geometry to recompute this mapping and reload robot geometry. Recalibrate after changing Tracker mounting, updating measured offsets when needed. The [Tracker calibration tool](STEAMVR.md#neutral-pose-calibration) saves the complete input profile directly; a symlink build requires only a process restart after saving. MANUS profiles likewise take effect when their publisher restarts.

## Check and run

```bash
source tools/env.sh
python3 tools/check_environment.py --mode all
ros2 run wuji_teleop_monitor monitor
```

The checker verifies configuration and calibration files, imports dependencies, checks pinned versions and native libraries, and constructs retargeting sessions without connecting to devices. The default `--mode all` also imports the foot pedal node and `evdev`; it does not open or capture input devices. `--mode hands` or `--mode arms` narrows device-specific checks. See the [project README](../README.md#start-teleoperation) for teleoperation, Home, Preview, and services, and the [foot pedal guide](../src/foot_pedal/README.md) for standalone input events.

## Troubleshooting

| Symptom | Check |
| --- | --- |
| SDK file is text or a Git LFS pointer | Run `git lfs pull` |
| Shared library fails to load | Source `tools/env.sh`; inspect `ldd path/to/library.so` |
| Package version mismatch | Load `.venv` through `tools/env.sh` and rerun setup |
| ROS package or configuration not found | Run setup, then source the environment again |
| Hand not discovered | Check serial, network links/routes, and discovery visibility |
| Hand stays `STOPPED` or an SDK operation fails | Inspect driver logs and diagnostics; use Enable after resolving the reported issue |
| No MANUS skeleton or failed calibration load | Check receiver permissions, licensing, selected profile, and publisher logs |
| SteamVR cannot see Trackers | Use the same graphical session and confirm all five paired devices |
| Cabinet connection fails | Check the IP, network route, emergency stop, and competing SDK processes |

Stop arm teleoperation before opening the separate brake tool, which uses its own cabinet connection.
