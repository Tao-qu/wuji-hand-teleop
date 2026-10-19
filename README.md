# MANUS–Tianji–Wuji Hand 2 Teleoperation

A ROS 2 Humble system for two **Wuji Hand 2.2** hands and two Tianji arms. A pair of MANUS gloves supplies finger motion; five HTC Vive Trackers supply chest, wrist, and upper-arm poses. The hand pipeline uses the official `wuji-sdk==2026.10.12`, including its `RetargetSession`, and the official Hand 2.2 model. The configured hand firmware is `2.5.1`.

## Hardware and modes

| Mode | Input | Output |
| --- | --- | --- |
| Hands + arms | Two MANUS gloves and five Trackers | Two hands and two arms |
| Hands only | MANUS gloves | Both hands or one selected hand |
| Arms only | Five Trackers | Two Tianji arms |
| Hand read-only | Hand network feedback | Joint state and diagnostics; no hand writes |

Mount the Trackers on the chest, both wrists, and both upper arms. Hands-only operation does not require SteamVR. The hand network uses SDK discovery and explicit serial-number selection; network addresses are not hard-coded.

## Installation and configuration

Use Ubuntu 22.04 x86_64, Python 3.10, and ROS 2 Humble. Follow [Local environment setup](docs/LOCAL_SETUP.md) for the full procedure.

```bash
git clone https://github.com/Tao-qu/wuji-hand-teleop.git
cd wuji-hand-teleop
git lfs install
git lfs pull
python3 tools/fetch_dependencies.py
python3 tools/configure.py
```

The fetch tool installs the pinned official model under `src/wuji-description`, with a sparse checkout of `hand2/hand2_2/body`. It refuses to reset an existing checkout with a different revision or local changes. Python runtime versions are pinned in `requirements.txt`.

The configuration tool copies missing templates and preserves existing local settings. Configure before building with `--symlink-install`:

| Configuration | Purpose |
| --- | --- |
| `src/output_devices/wujihand_output/config/hand2.yaml` | Left/right serials and namespaces; bridge rates, firmware, gains, limits, and MANUS input parameters |
| `src/input_devices/openvr_input/config/openvr_input.yaml` | Five Tracker serials and the human wrist mounting offset |
| `src/output_devices/tianji_output/tianji_output/config/tianji_chest.yaml` | Cabinet IP, arm initial poses, tool dynamics source, optional tool transforms |
| `src/wuji_teleop_bringup/config/static_transforms.yaml` | Chest and wrist coordinate transforms |

`hand2.yaml` is the active hand configuration; the controller uses the SDK's built-in Hand 2 retargeting model. `controller.flip_y` defaults to `false` because MANUS explicitly publishes right-handed coordinates; preserve XYZ and verify left/right flexion, abduction, and thumb opposition with motors disabled before live following. A reflection is an opt-in change for a separately validated source convention. Calibrate MANUS for the current operator using the [MANUS guide](src/input_devices/manus_input/README.md).

The arm configuration defaults to `tool_dynamics.source: controller`: it reads and reuses the cabinet's existing payload values. This does **not** mean that the new hand assembly has been calibrated. After measuring each complete hand, adapter, and cable assembly, select `configured` and supply the independent left/right values described in the template. Tool transforms use millimetres; Tracker offsets use metres. Apply a geometric tool offset in the IK configuration or target TF chain once.

## Start processes, then enable motors

Load the environment in every terminal:

```bash
source tools/env.sh
ros2 run wuji_teleop_monitor monitor
```

In the GUI, **Start Teleop** starts the selected processes. It does not enable hand or arm motors. Use **Enable Left Hand 2**, **Enable Right Hand 2**, or **Enable Arms** explicitly. Enabling the arms moves them to their initial pose and then enters impedance mode. Hand enable waits for complete feedback and diagnostics, then ramps from the measured pose toward incoming commands.

The same launch file supports terminal operation:

```bash
# Hands + arms; motors remain disabled until explicitly enabled
ros2 launch wuji_teleop_bringup wuji_teleop.launch.py
# Both hands only
ros2 launch wuji_teleop_bringup wuji_teleop.launch.py enable_arm:=false
# Left hand only
ros2 launch wuji_teleop_bringup wuji_teleop.launch.py enable_arm:=false hand_side:=left
# Right hand feedback only; skips MANUS and the retarget controller
ros2 launch wuji_teleop_bringup wuji_teleop.launch.py enable_arm:=false hand_side:=right hand_read_only:=true
# Arms only
ros2 launch wuji_teleop_bringup wuji_teleop.launch.py enable_hand:=false
```

`hand_side` accepts `both`, `left`, or `right`. `hand_auto_enable` and `arm_auto_enable` default to `false`. `hand_config` can select another complete hand YAML. Read-only hand mode forbids enable and device writes. For normal operation, enable individual chains with the GUI or these services after checking the work area:

```bash
ros2 service call /left_hand/set_enabled std_srvs/srv/SetBool '{data: true}'
ros2 service call /right_hand/set_enabled std_srvs/srv/SetBool '{data: true}'
ros2 service call /tianji_arm_controller/set_enabled std_srvs/srv/SetBool '{data: true}'
# Disable a hand without stopping its feedback process
ros2 service call /left_hand/set_enabled std_srvs/srv/SetBool '{data: false}'
```

The monitor uses the default `/left_hand` and `/right_hand` namespaces. It shows each chain's lifecycle and feedback, and allows the two hands to be enabled independently. `Scan SNs` discovers Wuji Hand 2 devices and saves selected serials; MANUS input is routed by its left/right `side` field.

A successful hand service response means the request was queued. The monitor confirms `STOPPED (disabled)` and allows another Enable only after fresh diagnostics report all 20 joints online and zero joints enabled.

**Stop Teleop** or terminal Ctrl-C stops the launch process group. Each hardware process performs its own cleanup; the hand bridge also has input, feedback, and diagnostic watchdogs. A hand fault is latched and does not automatically resume motion. Resolve its cause, clear it, then explicitly re-enable:

```bash
ros2 service call /left_hand/clear_fault std_srvs/srv/Trigger '{}'
ros2 service call /left_hand/set_enabled std_srvs/srv/SetBool '{data: true}'
```

The separate brake tool connects directly to the cabinet; stop the arm controller before opening it:

```bash
ros2 run wuji_teleop_monitor brake
# Optional desktop shortcuts
bash src/wuji_teleop_monitor/install_desktop.sh
```

## Data contracts and code layout

```text
MANUS → manus_ros2 → validate 21 keypoints per side
      → official RetargetSession(WujiHand2)
      → /{side}_hand/joint_commands → wuji_hand2_driver → SDK → hand

5 Trackers → OpenVR → TF → tianji_arm_controller → Tianji IK/SDK → arms
```

The hand retargeting input is a finite `(21, 3)` array in metres, in MediaPipe landmark order. Missing nodes, duplicate IDs, implausible scale, and degenerate palm frames are rejected. The output is 20 joint positions in radians, in SDK firmware order: thumb, index, middle, ring, pinky, four joints each. The bridge maps feedback NIDs explicitly; it does not infer motor order from URDF traversal. Named ROS feedback uses the official Hand 2.2 joint names.

Each hand has a persistent retargeting session in a separate process. MANUS control defaults to 120 Hz; the bridge command/state rates default to 200 Hz. The publisher timestamps each new SDK skeleton callback, advances its per-glove sequence, and publishes each sequence once. Its steady-clock source timeout is 0.20 s. The controller rejects repeated or reordered sequences and expired source stamps, and applies a monotonic receipt deadline shortened by the source age. Invalid input or a gap resets the retargeting session; the bridge command watchdog defaults to 0.25 s. Restart publisher and controller together if the publisher restarts its sequence counter. Actual rates depend on input and runtime conditions.

| Directory | Implementation |
| --- | --- |
| `src/input_devices/manus_input/` | MANUS C++ input, messages, calibration |
| `src/input_devices/openvr_input/` | Five-Tracker acquisition and TF |
| `src/controller/` | Per-hand input loops and arm lifecycle |
| `src/output_devices/wujihand_output/` | Input validation, official retargeting, ROS transport |
| `src/wuji_hand2_driver/` | SDK ownership, Hand 2 commands, diagnostics, limits and watchdogs |
| `src/wuji-description/hand2/hand2_2/body/` | Pinned official model package |
| `src/output_devices/tianji_output/` | Tianji SDK/IK and tool parameters |
| `src/wuji_teleop_bringup/` | Launch configuration and static transforms |
| `src/wuji_teleop_monitor/` | Qt monitor, explicit enable/disable, brake tool |
| `tests/`, `tools/` | Offline checks and environment/configuration tools |

## Checks and references

```bash
source tools/env.sh
python3 tools/check_environment.py --mode all
python3 -m pip install pytest
python3 -m pytest -q tests
ros2 topic hz /left_hand/joint_states
ros2 topic echo /left_hand/lifecycle_state
ros2 topic echo /left_hand/diagnostics
```

Environment checks and unit tests do not command hardware. Tests that use `RetargetSession` run the pure retargeting API without creating an SDK device manager. Successful software checks do not establish motor direction, real-world tracking quality, payload calibration, or full-system hardware acceptance.

Implementation follows the official [SDK Python retargeting examples](https://github.com/wuji-technology/wuji-sdk/tree/v2026.10.12/examples/python/retargeting), [Hand 2 examples](https://github.com/wuji-technology/wuji-sdk/tree/v2026.10.12/examples/python/wuji_hand_2), and [Hand 2.2 model](https://github.com/wuji-technology/wuji-description/tree/829c9aa27cb12c3a49d5b8d63c9869dd8f656702/hand2/hand2_2/body). See [documentation](docs/README.md) for the setup and wearing guides.

## License

See [LICENSE](LICENSE). MANUS, Tianji, Wuji SDK, and model resources retain their respective licenses and copyright notices.
