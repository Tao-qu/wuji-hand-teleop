# MANUS–Tianji–Wuji Teleoperation

A local ROS 2 Humble teleoperation system for two dexterous hands and two robot arms. A pair of MANUS gloves controls the Wuji hands, while five HTC Vive Trackers control the Tianji arms.

## Hardware and operating modes

| Mode | Input | Output |
| --- | --- | --- |
| Hands + arms | Left and right MANUS gloves, five Trackers | Two Wuji hands, two Tianji arms |
| Hands only | Left and right MANUS gloves | Two Wuji hands |
| Arms only | Five Trackers | Two Tianji arms |

Mount the Trackers on the chest, both wrists, and both upper arms. Each Tracker uses a paired receiver, with Lighthouse base stations providing tracking coverage. Hands-only operation does not require SteamVR; arms-only operation does not start glove or dexterous-hand nodes.

## Installation

The development environment is Ubuntu 22.04 x86_64, Python 3.10, and ROS 2 Humble. Follow [Local environment setup](docs/LOCAL_SETUP.md) to install system dependencies, fetch pinned drivers, configure USB permissions, and build the workspace.

```bash
git clone https://github.com/Tao-qu/wuji-hand-teleop.git
cd wuji-hand-teleop
git lfs install
git lfs pull
python3 tools/fetch_dependencies.py
python3 tools/configure.py
```

`dependencies.lock.json` pins the external hand driver and retargeting algorithm to specific commits. Downloaded dependency directories are excluded from this project's version control. Changes to the lock file require revalidation of driver interfaces, retargeting configuration, and hardware behavior.

## Hardware configuration

The configuration tool copies missing templates without overwriting existing settings. Build with `--symlink-install` so installed configuration files remain linked to their sources.

| Configuration | Contents |
| --- | --- |
| `src/input_devices/openvr_input/config/openvr_input.yaml` | Five Tracker serial numbers and wrist mounting offset |
| `src/output_devices/wujihand_output/config/wujihand_ik.yaml` | Left/right hand serial numbers, namespaces, and driver rates |
| `src/output_devices/wujihand_output/config/retarget_manus_{left,right}.yaml` | MANUS skeleton retargeting and filtering parameters |
| `src/output_devices/tianji_output/tianji_output/config/tianji_chest.yaml` | Controller cabinet IP address and initial joint poses |
| `src/wuji_teleop_bringup/config/static_transforms.yaml` | Chest and wrist mounting transforms |

Each MANUS glove requires calibration for the current operator; see [MANUS setup](src/input_devices/manus_input/README.md). For Tracker pairing and role assignment, see [SteamVR setup](docs/STEAMVR.md) and the [wearing guide](docs/tracker-wearing-guide.md). The host network interface must be able to reach the robot controller cabinet.

## Running

Load the environment from the repository root in each terminal:

```bash
source tools/env.sh
# GUI: select hands + arms, hands only, or arms only
ros2 run wuji_teleop_monitor monitor
```

Alternatively, use the unified launch entry point:

```bash
# Hands + arms
ros2 launch wuji_teleop_bringup wuji_teleop.launch.py
# Hands only
ros2 launch wuji_teleop_bringup wuji_teleop.launch.py enable_arm:=false
# Arms only
ros2 launch wuji_teleop_bringup wuji_teleop.launch.py enable_hand:=false
# Visualize Tracker coordinate frames
ros2 launch wuji_teleop_bringup wuji_teleop.launch.py enable_hand:=false enable_rviz:=true
```

Enabling the arms moves them to their initial pose and enters impedance mode. Check the workspace and emergency stop before starting, and calibrate mounting positions and coordinate offsets for the actual hardware. Validate the hand and arm pipelines separately before running the full system.

The GUI displays joint feedback, command rates, and logs. Readiness requires data from the selected pipelines: feedback and commands from both hands, and the READY lifecycle state from the arms. `Scan SNs` scans and configures Wuji Hands only; MANUS messages are routed by their left/right side field. The GUI monitors `/left_hand` and `/right_hand` by default. Update the monitor subscriptions if the driver namespaces change.

Use Stop or Ctrl-C in the terminal to stop teleoperation. The separate brake tool connects directly to the controller cabinet through the SDK; stop teleoperation first to release the connection:

```bash
ros2 run wuji_teleop_monitor brake
# Optional: install local desktop shortcuts
bash src/wuji_teleop_monitor/install_desktop.sh
```

## Data flow and repository layout

```text
MANUS → manus_ros2 → left/right hand controllers → wuji_retargeting
      → /{side}_hand/joint_commands → wujihand_driver → Wuji Hand

5 Trackers → OpenVR → TF → tianji_arm_controller
           → chest-frame IK → Tianji SDK → both arms
```

- `src/input_devices/manus_input/`: MANUS C++ driver, messages, calibration, and USB permission rules.
- `src/input_devices/openvr_input/`: Five-Tracker pose acquisition, coordinate corrections, and TF publishing.
- `src/controller/`: Per-hand control loops, arm lifecycle management, and IK scheduling.
- `src/output_devices/`: Hand retargeting interfaces and Tianji hardware wrappers.
- `src/wuji_teleop_bringup/`: Mode-specific node creation and static TF setup.
- `src/wuji_teleop_monitor/`: Local Qt monitor and brake tool.
- `src/wujihand_urdf/`: Dexterous-hand model descriptions.
- `tools/`: Dependency fetching, configuration generation, environment loading, and read-only checks.

Each hand has an independent control process, running at 120 Hz by default. Arm control defaults to 120 Hz, with state publishing at 500 Hz. Actual rates depend on input, solver performance, and communication.

## Checks and development

```bash
source tools/env.sh
python3 tools/check_environment.py --mode all
ros2 topic hz /manus_glove_0
ros2 topic hz /left_hand/joint_commands
ros2 topic hz /left_arm/joint_states
ros2 topic echo /tianji_arm/lifecycle_state
```

Environment checks only import modules and load SDK libraries; they do not connect to hardware. Successful builds and environment checks do not replace hardware validation. After changing control or kinematic parameters, validate both hands, both arms, startup, shutdown, and emergency-stop recovery.

## License

See [LICENSE](LICENSE) for the project code. MANUS and Tianji SDKs and external dependencies retain their respective licenses. Preserve their copyright and license notices.
