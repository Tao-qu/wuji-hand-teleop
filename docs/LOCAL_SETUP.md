# Local setup and configuration

## Prerequisites

Use Ubuntu 22.04 x86_64 with system Python 3.10, Git, and ROS 2 Humble Desktop installed under `/opt/ros/humble`. The [ROS 2 Humble installation guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) describes the ROS package sources and installation.

Prepare device requirements separately:

- MANUS gloves and receivers with vendor SDK licensing. Setup installs the repository's USB rules and adds the current user to `plugdev` and `dialout`.
- Steam and SteamVR in the same user's graphical session, with five paired Trackers and Lighthouse base stations. See [SteamVR and Tracker setup](STEAMVR.md). Hands-only operation does not use SteamVR.
- Network access to both Wuji hands for SDK discovery and to the configured Tianji cabinet endpoint.
- Internet access for Git LFS, the pinned model, Python packages, and ROS dependencies. System installation requires `sudo`.

The repository includes the MANUS Integrated SDK and Tianji native libraries as Git LFS assets. Vendor licensing remains a separate prerequisite. Setup retrieves the libraries from Git LFS for the project runtime.

## Install and build

Run this command from the repository root:

```bash
./tools/setup.sh
```

Setup performs the following steps:

1. Installs required system packages, MANUS USB rules, and device-access groups.
2. Fetches Git LFS assets and the official model revision in `dependencies.lock.json`.
3. Creates missing local hand and arm configuration files from their templates.
4. Creates a Python 3.10 `.venv` with system packages available and installs `requirements.txt`.
5. Resolves ROS dependencies, builds the workspace with `--symlink-install`, and checks the environment.

Existing local configuration is preserved. `./tools/setup.sh --skip-system` skips system package, rosdep installation, and USB permission changes; use it when these are already configured. `--skip-build` skips the build and checks that require an installed workspace. `--help` lists the options.

After a device-group change, log out and back in, then reconnect the receivers. Load the project environment in each terminal:

```bash
source tools/env.sh
```

`requirements.txt` pins `wuji-sdk==2026.10.12`, which supplies the Hand 2 retargeting runtime and network interface. The official model checkout at `src/output_devices/wujihand2_output/wuji-description` provides `wuji_hand2_2_description` from `hand2/hand2_2/body`. `tools/fetch_dependencies.py` enforces the locked revision and preserves existing checkout changes by rejecting a conflicting checkout.

For a source-only rebuild after setup:

```bash
source tools/env.sh
python3 -m colcon build --symlink-install \
  --cmake-args -DPython3_EXECUTABLE="$VIRTUAL_ENV/bin/python3" \
               -DPYTHON_EXECUTABLE="$VIRTUAL_ENV/bin/python3"
source tools/env.sh
python3 tools/check_environment.py --mode all
```

Build outputs contain workspace and interpreter paths. Rebuild with fresh `build/`, `install/`, and `log/` directories if those paths change. `tools/env.sh` loads ROS Humble and this checkout, clears inherited workspace paths, and excludes user-installed Python packages from `~/.local`.

## Native libraries

`tools/env.sh` selects the workspace Python environment and adds the bundled MANUS and Tianji library directories to `LD_LIBRARY_PATH`. The MANUS publisher uses `libManusSDK_Integrated.so`; Tianji uses `libMarvinSDK.so` for cabinet control and `libKine.so` for kinematics. All three libraries are retrieved through Git LFS during setup.

The cabinet address is configured in `tianji_chest.yaml`. The control wrapper sends a VERSION request after connecting to register its feedback destination. Keep the Python kinematics binding paired with the bundled `libKine.so`: its IK parameter structure is 472 bytes and includes `m_Output_JntExdABS`. Restart arm processes after changing native libraries or bindings.

## Configuration reference

The two runtime YAML files are local and ignored by Git:

- `src/output_devices/wujihand2_output/wujihand2_ros2/config/wujihand2.yaml`
- `src/output_devices/tianji_output/tianji_output/config/tianji_chest.yaml`

Setup creates them only when missing, from the adjacent `.yaml.template` files. The templates hold the supplied installation's settings. Check device serials, network addresses, tool geometry, and calibration for the actual installation. `python3 tools/configure.py --check` reports missing local files without writing.

The source files are the editable configuration for a symlink build. Teleoperation and Home use these default hand and arm configurations. Preview and standalone OpenVR also support an explicit `arm_config` launch argument. A regular install reads its package-share configuration copies.

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

Bringup sets `auto_enable: true` for selected hand drivers. The canonical profiles are `LeftMetaglovePro.mcal` and `RightMetaglovePro.mcal` in `src/input_devices/manus_input/manus_ros2/calibration`. An empty parameter selects the corresponding canonical file. Preview uses the same configured MANUS profiles. See [MANUS calibration](../src/input_devices/manus_input/manus_ros2/CALIBRATION_GUIDE.md).

### Arms and Tracker input

| Field | Meaning |
| --- | --- |
| `left_arm.home_joints`, `right_arm.home_joints` | Seven joint positions per arm in degrees |
| `driver.robot_ip` | Cabinet SDK endpoint |
| `driver.kinematics_model` | Filename in the Tianji package `config` directory, or an absolute path |
| `driver.tool_transforms.left`, `.right` | Flange-to-TCP 4×4 transforms, translation in millimetres |
| `driver.tool_dynamics` | Payload source and per-arm mass, COM, and inertia |
| `controller.control_rate` | Arm command rate in Hz; supplied value 120 |
| `controller.state_publish_rate` | Feedback publication rate in Hz; supplied value 500 |
| `controller.home_duration_sec` | Home trajectory duration in seconds; supplied value 8 |
| `controller.handoff_hold_sec`, `handoff_ramp_sec` | Measured-pose hold and blend into Tracker following, in seconds |
| `openvr.tracker_serials` | Distinct serials for `chest`, `left_hand_back`, `right_hand_back`, `left_arm`, `right_arm` |
| `openvr.publish_rate_hz`, `publish_tf`, `parent_frame` | Tracker publication rate, TF switch, and world frame |
| `openvr.calibration_file` | Tracker calibration filename or absolute path; supplied value `tracker.yaml` |

The supplied arm Home is left `[55, -65, -70, -60, 60, 0, 0]` and right `[-55, -65, 70, -60, -60, 0, 0]` degrees. Normal startup enters position mode, moves Home, then enters joint-impedance mode. The launch preparation interval is fixed at three seconds. Disable requests standby while keeping the connection. Enable and disable use the controller's enable lock; a disable request waits for an initialization already holding that lock.

### Tool geometry and payload

The virtual hand-back TCP axes are parallel to the flange axes. The supplied transforms have identity rotation and translations `[0, -50, 70]` mm on the left and `[0, 50, 70]` mm on the right. This mounting places flange +Z toward the fingers, with the hand back facing flange −Y on the left and +Y on the right. Measure the installation, including its adapter, and update these values when needed. Local Tianji IK applies this transform once.

`driver.tool_dynamics.source: configured` sends the separate `left` and `right` ten-element arrays to the SDK. `source: controller` uses the cabinet's existing payload parameters. Array order and units are:

```text
[mass, com_x, com_y, com_z, Ixx, Ixy, Ixz, Iyy, Iyz, Izz]
 mass: kg; COM: mm; inertia about the flange origin: kg·m²
```

The supplied values model each Hand 2.2 in its zero pose with the supplied mount, approximately 0.688 kg per side. Additional cabling and mounting parts are excluded. The conversion assumes coincident mount and flange origins and a mount-to-flange rotation of `diag(1, -1, -1)`. For a different mounting, transform the COM and inertia into the flange frame, including a parallel-axis shift if the origins differ. The driver passes the configured arrays to the SDK without unit conversion.

### Tracker calibration

`src/input_devices/openvr_input/calibration/tracker.yaml` is the canonical profile. Its fields are:

| Field | Meaning |
| --- | --- |
| `chest_mount.shared` | Shared chest translation in metres |
| `chest_mount.left`, `.right` | Per-side roll, pitch, yaw in radians and local translation in metres |
| `hand_back_to_tcp.left`, `.right` | `qx`, `qy`, `qz`, `qw` orientation alignment; zero translation |
| `upper_arm_rotations.left_arm`, `.right_arm` | 3×3 rotations whose columns are calibrated axes in the raw Tracker frame |

The raw hand-back Tracker origins supply target position directly. The orientation alignment defines `left_tcp_target` and `right_tcp_target`; the calibrated upper-arm Y axes provide the arm reference directions for IK. Tool translation is supplied only by the robot's flange-to-TCP transforms.

Recalibrate after changing tool geometry, Tracker mounting, or arm Home. The [Tracker calibration tool](STEAMVR.md#neutral-pose-calibration) saves the complete profile directly; a symlink build requires only a process restart after saving. MANUS profiles likewise take effect when their publisher restarts.

## Check and run

```bash
source tools/env.sh
python3 tools/check_environment.py --mode all
ros2 run wuji_teleop_monitor monitor
```

The checker verifies configuration and calibration files, imports dependencies, checks pinned versions and native libraries, and constructs retargeting sessions without connecting to devices. `--mode hands` or `--mode arms` narrows device-specific checks. See the [project README](../README.md#start-teleoperation) for teleoperation, Home, Preview, and services.

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
