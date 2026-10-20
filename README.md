# MANUS–Tianji–Wuji Hand 2 Teleoperation

A ROS 2 Humble system for two **Wuji Hand 2.2** hands and two Tianji arms. A pair of MANUS gloves supplies finger motion; five HTC Vive Trackers supply chest, wrist, and upper-arm poses. Finger retargeting and hand communication use the official `wuji-sdk==2026.10.12`; visualization uses the official Hand 2.2 model.

## Hardware and modes

| Mode | Input | Output |
| --- | --- | --- |
| Hands + arms | Two MANUS gloves and five Trackers | Two hands and two arms |
| Hands only | MANUS gloves | Both hands or one selected hand |
| Arms only | Five Trackers | Two Tianji arms |
| Preview | MANUS gloves and five Trackers | RViz hand targets and Tracker poses |

Mount the Trackers on the chest, both wrists, and both upper arms. Hands-only operation does not require SteamVR. Hand connections use SDK discovery and the configured serial numbers.

## Installation and configuration

Use Ubuntu 22.04 x86_64, Python 3.10, and ROS 2 Humble. Follow [Local environment setup](docs/LOCAL_SETUP.md) for dependency installation, USB rules, and build commands.

```bash
git clone https://github.com/Tao-qu/wuji-hand-teleop.git
cd wuji-hand-teleop
git lfs install
git lfs pull
python3 tools/fetch_dependencies.py
python3 tools/configure.py
```

The fetch tool installs the pinned official model under `src/wuji-description`, using `hand2/hand2_2/body`. Python versions are pinned in [requirements.txt](requirements.txt); the model revision is recorded in [dependencies.lock.json](dependencies.lock.json). The configuration tool copies missing templates and preserves existing local settings. Configure devices before building with `--symlink-install`.

| Configuration | Purpose |
| --- | --- |
| `src/output_devices/wujihand_output/config/hand2.yaml` | Hand serials and namespaces, Home joint positions, SDK/ROS rates, MIT gains, and MANUS parameters |
| `src/input_devices/openvr_input/config/openvr_input.yaml` | Five Tracker serials and wrist mounting offset |
| `src/output_devices/tianji_output/tianji_output/config/tianji_chest.yaml` | Cabinet IP, initial joint angles, tool dynamics source, and optional tool transforms |
| `src/wuji_teleop_bringup/config/static_transforms.yaml` | Chest and wrist coordinate transforms |

MANUS publishes right-handed coordinates. `controller.flip_y: false` preserves those XYZ positions for the SDK's built-in retargeting model. The bundled profiles retain the filenames `tpxLeftMetaglovePro.mcal` and `tpxRightMetaglovePro.mcal`; select other profiles through `manus.left_calibration_file` and `manus.right_calibration_file`. See the [MANUS guide](src/input_devices/manus_input/README.md).

`tool_dynamics.source: controller` reuses the cabinet's payload values. Select `configured` to supply independent left/right mass, centre-of-mass, and inertia arrays as described in the template. Tool-transform translations use millimetres; Tracker offsets use metres. Apply a geometric tool offset once, in either the IK configuration or target TF chain.

## Start teleoperation

Load the environment in every terminal:

```bash
source tools/env.sh
ros2 run wuji_teleop_monitor monitor
```

Select a mode and click **Start Teleop**. **Hands and arms auto-enable by default.** The arms move to the configured home position, then enter impedance mode and follow Tracker targets. The hands follow MANUS targets. The GUI also provides individual enable/disable buttons, joint feedback, current readings, and command rates. **Scan SNs** discovers Hand 2 devices and saves serial numbers to the local configuration.

The same launch file supports terminal operation:

```bash
# Hands + arms
ros2 launch wuji_teleop_bringup wuji_teleop.launch.py
# Both hands only
ros2 launch wuji_teleop_bringup wuji_teleop.launch.py enable_arm:=false
# Left hand only
ros2 launch wuji_teleop_bringup wuji_teleop.launch.py enable_arm:=false hand_side:=left
# Arms only
ros2 launch wuji_teleop_bringup wuji_teleop.launch.py enable_hand:=false
```

| Argument | Default | Meaning |
| --- | --- | --- |
| `enable_hand` | `true` | Start MANUS, hand controllers, and hand drivers |
| `enable_arm` | `true` | Start OpenVR, static transforms, and arm controller |
| `hand_side` | `both` | Select `both`, `left`, or `right` |
| `hand_auto_enable` | `true` | Enable each hand after connecting and configuring gains |
| `arm_auto_enable` | `true` | Run arm initialization and start following |
| `hand_config` | empty | Override the default local Hand 2 YAML path |
| `enable_rviz` | `false` | Show the Tracker visualization |

To start processes with automatic enable turned off:

```bash
ros2 launch wuji_teleop_bringup wuji_teleop.launch.py hand_auto_enable:=false arm_auto_enable:=false
```

Enable or disable individual chains through the GUI or ROS services:

```bash
ros2 service call /left_hand/set_enabled std_srvs/srv/SetBool '{data: true}'
ros2 service call /right_hand/set_enabled std_srvs/srv/SetBool '{data: true}'
ros2 service call /tianji_arm_controller/set_enabled std_srvs/srv/SetBool '{data: true}'
ros2 service call /left_hand/set_enabled std_srvs/srv/SetBool '{data: false}'
```

The monitor uses `/left_hand` and `/right_hand` namespaces. Hand service responses acknowledge queued SDK operations; lifecycle and diagnostic topics report their results. **Stop Teleop** or terminal Ctrl-C stops the launch process group and closes the drivers.

The separate brake tool connects directly to the cabinet. Stop the arm controller before opening it:

```bash
ros2 run wuji_teleop_monitor brake
# Optional desktop shortcuts
bash src/wuji_teleop_monitor/install_desktop.sh
```

## Return to Home

In the monitor, stop the current session, select the hands + arms, hands-only, or arms-only preset, and click **Return Home**. This operation needs no MANUS gloves or Trackers. **Stop Home** ends the session; select **Start Teleop** to resume input following.

The command-line script provides the same three modes and loads the project environment:

```bash
./tools/home.sh both       # Both hands and both arms (default)
./tools/home.sh arms       # Both arms
./tools/home.sh hands      # Both hands
```

Home reads `left_hand.home_joints` and `right_hand.home_joints` from `hand2.yaml`: 20 positions in radians, in firmware order (thumb, index, middle, ring, pinky; four joints each). Both hands default to all zeros, the open-hand reset target used by the Wuji Hand 2 implementation in `tianji_tele`. Arm Home reads `init_joints.left` and `init_joints.right` from `tianji_chest.yaml`, in degrees.

The operation publishes hand targets through the existing hand transport and driver. Arms use the existing position-mode initialization and `move_to_init` trajectory, then remain in position mode. The session keeps the configured targets active until **Stop Home** or Ctrl-C, which closes the devices through their existing shutdown methods. Normal teleoperation startup is unchanged.

For direct ROS launch or a separate hand configuration:

```bash
source tools/env.sh
ros2 launch wuji_teleop_bringup wuji_home.launch.py mode:=both
./tools/home.sh hands hand_config:=/absolute/path/to/hand2.yaml
```

The monitor shows joint feedback, hand driver states, and the Home operation state. `ACTIVE` on `/wuji_home/state` means the hand targets are being published and the selected arm Home call has completed; it is not a hand-arrival measurement. Arm Home calls the SDK directly, so the monitor's ROS **Cmd Hz** column does not measure that trajectory's command rate.

## Preview inputs and targets

```bash
source tools/env.sh
ros2 launch wuji_teleop_bringup wuji_preview.launch.py
```

This entrypoint starts MANUS with the bundled TPX profiles, the configured five Trackers, Hand 2 target models, and RViz. It uses `/preview/*` topics and does not start actuator drivers. The models show requested finger geometry; Tracker axes show the incoming poses. Labels show the last received input and targets. Close RViz or press Ctrl-C to stop. `start_inputs:=false` omits input readers, and `enable_rviz:=false` omits RViz.

## Data contracts and code layout

```text
MANUS → manus_ros2 → 21 keypoints per side
      → official RetargetSession(WujiHand2)
      → /{side}_hand/joint_commands → wuji_hand2_driver → SDK → hand

5 Trackers → OpenVR → TF → tianji_arm_controller → Tianji IK/SDK → arms
```

The retargeting input is a `(21, 3)` array in metres, in MediaPipe landmark order. MANUS node IDs determine the mapping; missing points remain zero and repeated IDs use the last value. `controller.flip_y` controls the optional Y reflection. The output contains 20 joint positions in radians, in firmware order: thumb, index, middle, ring, pinky, four joints each.

Each hand uses one persistent SDK retargeting session in its own controller process. The MANUS publisher runs at 120 Hz and can republish its cached skeleton. Controllers consume the latest received message once per control tick. The Hand 2 bridge sends the latest target at 200 Hz while enabled. SDK feedback is sorted by joint NID and published with the official joint names; motor current is reported separately in amperes.

| Directory | Implementation |
| --- | --- |
| `src/input_devices/manus_input/` | MANUS C++ publisher, messages, and calibration |
| `src/input_devices/openvr_input/` | Five-Tracker acquisition and TF |
| `src/controller/` | Per-hand input loops and arm lifecycle |
| `src/output_devices/wujihand_output/` | Landmark mapping, official retargeting, and ROS transport |
| `src/wuji_hand2_driver/` | Hand 2 SDK commands, feedback, and diagnostics |
| `src/wuji-description/hand2/hand2_2/body/` | Pinned official model package |
| `src/output_devices/tianji_output/` | Tianji SDK/IK and tool parameters |
| `src/wuji_teleop_bringup/` | Launch configuration, preview, and static transforms |
| `src/wuji_teleop_monitor/` | Qt monitor and brake tool |
| `tools/` | Dependency, environment, and configuration tools |

## Environment checks and references

```bash
source tools/env.sh
python3 tools/check_environment.py --mode all
ros2 topic hz /left_hand/joint_states
ros2 topic echo /left_hand/lifecycle_state
ros2 topic echo /left_hand/diagnostics
```

The environment check loads dependencies and constructs retargeting sessions without connecting to hardware.

Implementation references: official [SDK retargeting examples](https://github.com/wuji-technology/wuji-sdk/tree/v2026.10.12/examples/python/retargeting), [Hand 2 examples](https://github.com/wuji-technology/wuji-sdk/tree/v2026.10.12/examples/python/wuji_hand_2), and [Hand 2.2 model](https://github.com/wuji-technology/wuji-description/tree/829c9aa27cb12c3a49d5b8d63c9869dd8f656702/hand2/hand2_2/body). See the [documentation index](docs/README.md) for installation, calibration, Tracker wearing, and SDK interfaces.

## License

See [LICENSE](LICENSE). MANUS, Tianji, Wuji SDK, and model resources retain their respective licenses and copyright notices.
