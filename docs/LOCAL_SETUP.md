# Local environment setup

## 1. System and ROS 2

Use Ubuntu 22.04 x86_64 and the system Python 3.10 interpreter. Follow the [official ROS 2 Humble Ubuntu installation guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) to configure package sources and install `ros-humble-desktop`. Use a system-Python virtual environment for this workspace.

```bash
sudo apt update
sudo apt install -y build-essential cmake git git-lfs curl usbutils \
  python3-dev python3-pip python3-venv python3-colcon-common-extensions \
  python3-rosdep python3-pyqt5 python3-pyqt5.qtsvg \
  libusb-1.0-0-dev libncursesw5-dev libspdlog-dev libgl1-mesa-glx
source /opt/ros/humble/setup.bash
# Run once if rosdep has not been initialized on this machine
sudo rosdep init
rosdep update
```

Run the remaining commands from the repository root. Project installation and builds do not require sudo.

## 2. Official model and Python environment

```bash
git lfs install
git lfs pull
python3 tools/fetch_dependencies.py
python3 tools/configure.py
/usr/bin/python3 -m venv --system-site-packages .venv
source tools/env.sh
python3 -m pip install "pip>=23,<27" "setuptools==68.2.2" "wheel==0.45.1"
python3 -m pip install -r requirements.txt
```

The environment inherits ROS and Qt packages. `requirements.txt` pins `wuji-sdk==2026.10.12`; the SDK contains the Hand 2 retargeting runtime, and its NumPy interface is used directly. No separate retargeting project is installed.

`dependencies.lock.json` pins the official `wuji-description` revision. The fetch tool checks out only `hand2/hand2_2/body` under `src/wuji-description`, providing the `wuji_hand2_2_description` ROS package. It rejects changed revisions or dirty existing checkouts rather than resetting them. The rest of the driver is maintained in this repository under `src/wuji_hand2_driver`.

## 3. SDK libraries and device access

MANUS uses the bundled `libManusSDK_Integrated.so`; Tianji uses `libKine.so` and `libMarvinSDK.so`. Fetch these through Git LFS, and check that they are ELF libraries rather than text pointers:

```bash
file src/input_devices/manus_input/manus_ros2/ManusSDK/lib/libManusSDK_Integrated.so
file src/output_devices/tianji_output/tianji_output/_internal/lib/*.so
sudo install -m 644 src/input_devices/manus_input/config/udev/99-manus-libusb.rules /etc/udev/rules.d/
sudo usermod -aG plugdev,dialout "$USER"
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Log out and back in after changing groups, then reconnect the MANUS receivers. SDK licensing and calibration are separate from USB permissions. SteamVR handles Tracker pairing and access; see [SteamVR setup](STEAMVR.md).

Wuji Hand 2 uses the official network SDK. Configure each hand's serial number in `hand2.yaml`; the official SDK discovers its network address. Place the host and hands on a network where SDK discovery works. The Tianji cabinet IP is set separately in `tianji_chest.yaml`; configure it for the actual cabinet instead of assuming the template address applies.

`tools/env.sh` adds the MANUS and Tianji library directories to the process environment and selects the project Python packages. It does not modify global SDK libraries.

To select a vendor-supplied Tianji control SDK, create the ignored local file `tools/env.local.sh` and set its absolute library path:

```bash
# tools/env.local.sh — replace this example path with the vendor-supplied library
export TIANJI_SDK_LIBRARY="/absolute/path/to/libMarvinSDK.so"
```

`tools/env.sh` sources this optional file before loading ROS and the workspace. The control wrapper uses `TIANJI_SDK_LIBRARY` when set, otherwise the bundled library. The kinematics library remains the bundled `libKine.so`. `tools/check_environment.py` checks the selected control library. Configure the cabinet's SDK endpoint in `tianji_chest.yaml`. After connecting, the wrapper sends a VERSION request to register the feedback destination. Keep local library paths in `tools/env.local.sh`.

The bundled Linux kinematics library uses a 472-byte IK parameter structure, including `m_Output_JntExdABS`. Keep its Python binding paired with the bundled library; another `libKine.so` may use a different structure. Restart running arm nodes after changing native bindings.

## 4. Configuration and build

Fill in the two local files generated from templates: `src/output_devices/wujihand_output/config/hand2.yaml` and `src/output_devices/tianji_output/tianji_output/config/tianji_chest.yaml`. Prepare MANUS calibration files for the current operator before running input. The hand configuration contains:

- `left_hand` and `right_hand`: unique `serial_number` and ROS `name`.
- `driver`: command, feedback publication, and diagnostic rates; MIT gains `kp`/`kd`; and `auto_enable` (default `true`).
- `controller`: MANUS `control_rate` and `flip_y: false`. The publisher explicitly requests right-handed coordinates, so the controller preserves XYZ. Set `flip_y: true` only for a separately validated input convention; it reverses chirality and is not a wrist rotation.
- `manus`: optional absolute `left_calibration_file` and `right_calibration_file` paths. Empty paths select the bundled TPX profiles.

The default arm configuration and relative kinematics filenames are read from the package configuration directory: `src/output_devices/tianji_output/tianji_output/config` with `--symlink-install`, or the package-share `config` directory in a regular install. Directory lookup does not depend on individual configuration files. An explicit `arm_config` path overrides the default YAML.

The arm configuration contains:

- `left_arm.home_joints` and `right_arm.home_joints`: seven joint positions per arm, in degrees.
- `driver.robot_ip`: cabinet SDK endpoint. `driver.kinematics_model`: the Tianji kinematics configuration filename in the `tianji_output` package's `config` directory, or an absolute path.
- `driver.tool_dynamics`: payload source and optional per-arm parameters, described below.
- `controller.control_rate` and `controller.state_publish_rate`: command and feedback publication rates in Hz. `home_duration_sec` controls the Home trajectory duration; `handoff_hold_sec` and `handoff_ramp_sec` control the measured-pose hold and blend into Tracker following, all in seconds.
- `openvr.tracker_serials`: `chest`, `left_hand_back`, `right_hand_back`, `left_arm`, and `right_arm` serial numbers. `publish_rate_hz`, `publish_tf`, and `parent_frame` control Tracker publication.
- `openvr.calibration_file`: the selected Tracker profile. A filename resolves in the package calibration directory (`src/input_devices/openvr_input/calibration` with `--symlink-install`); an absolute path is read directly. Directory lookup does not depend on a particular profile existing.

Keep `driver.tool_dynamics.source: controller` to reuse the cabinet's payload values. To supply measured payloads, set `source: configured` and provide separate `left` and `right` arrays of ten values:

```text
[mass, com_x, com_y, com_z, Ixx, Ixy, Ixz, Iyy, Iyz, Izz]
```

Confirm payload units and inertia conventions with the Tianji vendor before supplying configured values. Include the complete hand, adapter, and cable assembly. Payload units are separate from the kinematic translation units below.

`driver.tool_transforms.left` and `.right` are rigid 4×4 transforms from each robot flange to its virtual hand-back TCP. Their translation columns use millimetres; the TCP axes are chosen parallel to the corresponding flange axes, so the 3×3 rotation blocks are identity. Express the measured TCP position in the flange frame, including the installed adapter. The local Tianji IK uses these transforms once to solve for the flange pose. The Tracker target TF chain contains no additional copy of this tool translation.

The current approximate mounting uses flange +Z toward the fingers, with the back of the hand facing flange −Y on the left and +Y on the right. A TCP 70 mm toward the fingers, 0 mm across the palm, and 50 mm toward the back of the hand therefore gives translation columns `[0, -50, 70]` and `[0, 50, 70]` mm respectively. These directions describe this installation; update the matrices when the measured mounting differs.

The selected Tracker profile has three top-level fields:

- `chest_mount`: shared chest-frame translation and per-side `roll`, `pitch`, `yaw`, `local_x`, `local_y`, `local_z`. Translations are in metres and Euler angles in radians.
- `hand_back_to_tcp`: per-side quaternion `qx`, `qy`, `qz`, `qw` aligning the hand-back Tracker orientation with the target TCP orientation. This transform has zero translation.
- `upper_arm_rotations`: `left_arm` and `right_arm` 3×3 rotations. Columns are calibrated axes in the raw Tracker frame; the Y column supplies the arm reference direction to IK.

The raw `left_hand_back` and `right_hand_back` Tracker origins provide position directly, without a wrist offset. Their orientation-corrected target frames are `left_tcp_target` and `right_tcp_target`. The bundled profile is `src/input_devices/openvr_input/calibration/tracker-neutral.yaml`; runtime loads the selected profile directly. After changing tool transforms, Tracker mounting, or Home joint targets, [export a new neutral-pose profile](STEAMVR.md#neutral-pose-calibration), select its filename with `openvr.calibration_file`, and restart teleoperation. A newly exported profile needs no rebuild. Home joint targets and controller settings remain independent of the tool geometry.

The configuration tool manages only the hand and arm templates and never replaces an existing local file. `python3 tools/configure.py --check` reports missing local files without writing. Generate local configuration before building, so the files are installed into ROS share directories:

```bash
source tools/env.sh
rosdep install --from-paths src --ignore-src -r -y
python3 -m colcon build --symlink-install \
  --cmake-args -DPython3_EXECUTABLE="$VIRTUAL_ENV/bin/python3" \
               -DPYTHON_EXECUTABLE="$VIRTUAL_ENV/bin/python3"
source tools/env.sh
python3 tools/check_environment.py --mode all
```

Load `tools/env.sh` in every new terminal. If the workspace path or interpreter changes, regenerate this project's `build/`, `install/`, and `log/` outputs and rebuild. Do not source an unrelated workspace over this one.

## 5. Launch the system

Normal launch starts both hand and arm pipelines and automatically enables their motors. Arm startup moves to the configured initial joint pose before accepting Tracker targets:

```bash
source tools/env.sh
ros2 launch wuji_teleop_bringup wuji_teleop.launch.py
```

Use `enable_arm:=false` for hands only, or `enable_hand:=false` for arms only. `hand_side:=left` or `hand_side:=right` selects one hand. To control enabling through the GUI or ROS services instead of startup, pass `hand_auto_enable:=false` and/or `arm_auto_enable:=false`. The `hand_auto_enable` launch argument overrides the hand YAML's `auto_enable` value.

Teleoperation, Home, Preview, and the standalone OpenVR launch accept `arm_config:=/absolute/path/to/tianji_chest.yaml`. Leaving it empty selects the local arm configuration installed in the `tianji_output` package.

The hand bridge sets MIT gains and sends the latest 20-joint target through the official SDK. The SDK and hand firmware manage motor control; the project does not configure a current limit or add command timeout, ramp, or joint-limit interlocks. The GUI displays feedback and diagnostics and provides Enable/Disable buttons. The hand driver also exposes SDK emergency-stop and clear-fault operations as ROS services. See the [README](../README.md) for launch options and services.

## 6. Troubleshooting

| Symptom | Check |
| --- | --- |
| SDK file is ASCII or a Git LFS pointer | Run `git lfs pull` |
| Shared library fails to load | Source the environment; use `ldd path/to/library.so` |
| Environment checker reports a package version mismatch | Select `.venv` and reinstall `requirements.txt` |
| Model package or configuration not found | Fetch dependencies, generate configuration, build, and source again |
| Hand not discovered | Check the configured SN, network links/routes, and discovery visibility |
| Hand SDK operation fails | Inspect the driver log and per-joint diagnostics for the SDK result |
| Hand remains STOPPED | Check `hand_auto_enable`; use the GUI or `set_enabled` service if automatic enabling was disabled |
| No MANUS skeleton data | Check receiver permissions, SDK licensing, calibration file paths, and publisher logs |
| SteamVR fails | Run it in the same graphical session and confirm all five Trackers |
| Cabinet connection fails | Check the configured IP, routes, emergency stop, and competing SDK processes |

The GUI's separate brake tool opens a direct cabinet connection. Stop arm teleoperation before using it.

## 7. Arm initialization and Stop

The default Home targets are left `[55, -65, -70, -60, 60, 0, 0]` and right `[-55, -65, 70, -60, -60, 0, 0]` degrees. Set `left_arm.home_joints` and `right_arm.home_joints` in `tianji_chest.yaml` to change them.

Normal initialization enters position mode, executes the Home trajectory using `controller.home_duration_sec` (8 seconds by default), then requests joint-impedance mode for Tracker control. The measured-pose hold and target blend use `controller.handoff_hold_sec` and `controller.handoff_ramp_sec`. The GUI's Disable operation or `/tianji_arm_controller/set_enabled` with `data: false` requests standby while keeping the connection. Initialization and enable/disable services run under the controller's enable lock; a stop request does not interrupt an initialization already holding that lock.
