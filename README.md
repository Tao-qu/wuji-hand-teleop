# MANUS–Tianji–Wuji Hand 2 Teleoperation

A ROS 2 Humble system for two **Wuji Hand 2.2** hands and two Tianji arms. A pair of MANUS gloves supplies finger motion; five HTC Vive Trackers supply chest, hand-back, and upper-arm poses. Finger retargeting and hand communication use the official `wuji-sdk==2026.10.12`; visualization uses the official Hand 2.2 model.

## Hardware and modes

| Mode | Input | Output |
| --- | --- | --- |
| Hands + arms | Two MANUS gloves and five Trackers | Two hands and two arms |
| Hands only | MANUS gloves | Both hands |
| Arms only | Five Trackers | Two Tianji arms |
| Preview | MANUS gloves and five Trackers | RViz hand targets and Tracker poses |

Mount the Trackers on the chest, the back of each hand, and both upper arms. Hands-only operation does not require SteamVR. Hand connections use SDK discovery and the configured serial numbers.

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
| `src/output_devices/wujihand2_output/wujihand2_ros2/config/wujihand2.yaml` | Hand serials and namespaces, Home joint positions, SDK/ROS rates, MIT gains, and MANUS parameters |
| `src/output_devices/tianji_output/tianji_output/config/tianji_chest.yaml` | Arm Home, driver/controller settings, five Tracker serials, and calibration profile selection |
| `src/input_devices/openvr_input/calibration/tracker-neutral.yaml` | Chest transforms, hand-back-to-TCP orientations, and upper-arm mounting rotations |

MANUS publishes right-handed coordinates. `controller.flip_y: false` preserves those XYZ positions for the SDK's built-in retargeting model. The bundled profiles retain the filenames `tpxLeftMetaglovePro.mcal` and `tpxRightMetaglovePro.mcal`; select other profiles through `manus.left_calibration_file` and `manus.right_calibration_file`. See the [MANUS guide](src/input_devices/manus_input/README.md).

The arm configuration groups settings under `left_arm`, `right_arm`, `driver`, `controller`, and `openvr`. `driver.tool_dynamics.source: controller` reuses the cabinet's payload values; `configured` uses independent left/right mass, centre-of-mass, and inertia arrays. See [Local environment setup](docs/LOCAL_SETUP.md#4-configuration-and-build) for field units and structure.

`openvr.calibration_file` selects one Tracker profile, loaded directly at startup. A filename such as `tracker-neutral.yaml` resolves inside the `openvr_input` package's `calibration` directory; an absolute path is read directly. The profile contains `chest_mount`, `hand_back_to_tcp`, and `upper_arm_rotations`. The raw `left_hand_back` and `right_hand_back` Tracker origins supply position directly; the per-side `hand_back_to_tcp` quaternions align their orientations with `left_tcp_target` and `right_tcp_target`.

`driver.tool_transforms.left/right` define the robot flange-to-virtual-hand-back-TCP geometry, with translations in millimetres and TCP axes parallel to the flange axes. The local Tianji IK applies this geometry once. Neutral-pose calibration aligns the human Tracker targets with the robot TCPs at Home. After changing tool geometry, Tracker mounting, or Home, [capture a new profile](docs/STEAMVR.md#neutral-pose-calibration), select it in the arm configuration, and restart teleoperation. Home joint targets and the arm control sequence remain configured as before.

## Start teleoperation

Load the environment in every terminal:

```bash
source tools/env.sh
ros2 run wuji_teleop_monitor monitor
```

Select a mode and click **Start Teleop**. **Hands and arms auto-enable by default.** The arms move to the configured home position, then enter impedance mode and follow Tracker targets. The hands follow MANUS targets. The GUI also provides individual enable/disable buttons, joint feedback, current readings, and command rates. **Scan SNs** discovers Hand 2 devices and saves serial numbers to the local configuration.

The launch file is also installed as an executable so all operation switches use `--` syntax:

```bash
# Hands + arms
ros2 run wuji_teleop_bringup wuji_teleop.launch.py
# Both hands only
ros2 run wuji_teleop_bringup wuji_teleop.launch.py --disable-arm
# Both arms only
ros2 run wuji_teleop_bringup wuji_teleop.launch.py --disable-hand
```

| Option | Default | Meaning |
| --- | --- | --- |
| `--disable-hand` | Off | Omit both hands and MANUS |
| `--disable-arm` | Off | Omit both arms and Trackers |
| `--home` | Off | Return the selected devices Home, then exit |

Use `--help` for usage. The two disable switches cannot be combined. Teleoperation and Home load the fixed local `wujihand2.yaml` and `tianji_chest.yaml`; automatic enable is always on. Teleoperation uses a fixed three-second preparation interval after arm Home, or after hand initialization in hands-only mode. In combined mode, the hands wait for the arms to become ready. RViz is available through the separate Preview entrypoint.

Use `ros2 run` for these switches: native `ros2 launch` expects `name:=value` arguments and cannot pass custom `--` options. The bare `ros2 launch wuji_teleop_bringup wuji_teleop.launch.py` command still starts the default hands + arms mode.

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

In the monitor, stop the current session, select the hands + arms, hands-only, or arms-only preset, and click **Return Home**. This operation needs no MANUS gloves or Trackers. The session exits automatically after the Home motion; **Stop Home** interrupts it. Select **Start Teleop** to resume input following.

Use the same executable with `--home`:

```bash
source tools/env.sh
ros2 run wuji_teleop_bringup wuji_teleop.launch.py --home
ros2 run wuji_teleop_bringup wuji_teleop.launch.py --home --disable-hand  # Arms only
ros2 run wuji_teleop_bringup wuji_teleop.launch.py --home --disable-arm   # Hands only
```

Home reads `left_hand.home_joints` and `right_hand.home_joints` from `wujihand2.yaml`: 20 positions in radians, in firmware order (thumb, index, middle, ring, pinky; four joints each). Both hands default to all zeros. Arm Home reads `left_arm.home_joints` and `right_arm.home_joints` from `tianji_chest.yaml`, in degrees, with the trajectory duration set by `controller.home_duration_sec`.

The operation publishes hand targets through the existing hand transport and driver. Arms use the existing position-mode initialization and `move_to_init` trajectory with no additional wait after the motion. Arms-only and combined sessions exit when the configured trajectory finishes (8 seconds by default); hands-only sessions publish Home targets for 8 seconds and then exit. Exit disables and closes the selected devices through their existing shutdown methods. Device initialization and shutdown add time outside the motion interval. **Stop Home** or Ctrl-C interrupts the session. Normal teleoperation startup is unchanged.

The monitor shows joint feedback, hand driver states, and the Home operation state. `ACTIVE` on `/wuji_home/state` means the hand targets are being published and the selected arm Home call has completed; it is not a hand-arrival measurement. Arm Home calls the SDK directly, so the monitor's ROS **Cmd Hz** column does not measure that trajectory's command rate.

## Preview inputs and targets

```bash
source tools/env.sh
ros2 launch wuji_teleop_bringup wuji_preview.launch.py
```

This entrypoint starts MANUS with the bundled TPX profiles, the configured five Trackers, Hand 2 target models, and RViz. It uses `/preview/*` topics and does not start actuator drivers. The models show requested finger geometry; Tracker axes show the incoming poses. Labels show the last received input and targets. Close RViz or press Ctrl-C to stop. `start_inputs:=false` omits input readers, and `enable_rviz:=false` omits RViz.

Preview and the standalone OpenVR launch accept `arm_config:=/absolute/path/to/tianji_chest.yaml`. Teleoperation and Home always use the default local arm configuration. Input launches resolve the Tracker settings and selected calibration profile from the arm configuration.

## Data contracts and code layout

```text
MANUS → manus_ros2 → 21 keypoints per side
      → official RetargetSession(WujiHand2)
      → /{side}_hand/joint_commands → wujihand2_driver → SDK → hand

5 Trackers → OpenVR → TF → tianji_arm_controller → Tianji IK/SDK → arms
```

The retargeting input is a `(21, 3)` array in metres, in MediaPipe landmark order. MANUS node IDs determine the mapping; missing points remain zero and repeated IDs use the last value. `controller.flip_y` controls the optional Y reflection. The output contains 20 joint positions in radians, in firmware order: thumb, index, middle, ring, pinky, four joints each.

Each hand uses one persistent SDK retargeting session in its own controller process. The MANUS publisher runs at 120 Hz and can republish its cached skeleton. Controllers consume the latest received message once per control tick. The Hand 2 bridge sends the latest target at 200 Hz while enabled. SDK feedback is sorted by joint NID and published with the official joint names; motor current is reported separately in amperes.

| Directory | Implementation |
| --- | --- |
| `src/input_devices/manus_input/` | MANUS C++ publisher, messages, and calibration |
| `src/input_devices/openvr_input/` | Five-Tracker acquisition and TF |
| `src/controller/` | Per-hand input loops and arm lifecycle |
| `src/output_devices/wujihand2_output/` | Hand 2 SDK/ROS adapter, official retargeting, and configuration |
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
