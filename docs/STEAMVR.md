# SteamVR and five-Tracker arm input

## Installation and pairing

Install Steam and SteamVR in the local graphical session using the same operating-system account as teleoperation. Prepare five compatible Vive Trackers, their USB receivers, and Lighthouse base stations covering the workspace. Pair each Tracker in SteamVR and confirm valid tracking before starting the ROS input node.

Mount one Tracker facing forward on the chest, one on the back of each MANUS glove, and one facing outward on each upper arm. Both upper-arm Trackers are required for the arm redundancy calculation, which uses their orientation rather than position. See the [wearing guide](tracker-wearing-guide.md).

## Headset-free configuration

After SteamVR creates its user settings, close it and back up the settings before editing. Paths depend on the Steam library location; Steam's **Browse local files** command locates the installation.

Set `driver_null.enable` to `true` in `drivers/null/resources/settings/default.vrsettings` inside SteamVR. In the user's `steamvr.vrsettings`, merge these keys into the existing `steamvr` object:

```json
{
  "requireHmd": false,
  "forcedDriver": "null",
  "activateMultipleDrivers": true
}
```

Restart SteamVR and verify that the Lighthouse driver and all five Trackers are active. SteamVR updates can replace installation settings. Verify headset-free operation with the installed version; an Ubuntu Xorg session can help diagnose graphical-session issues.

## Assign serial numbers

From the repository root, enumerate the connected Trackers:

```bash
source tools/env.sh
python3 - <<'PYCODE'
import openvr
openvr.init(openvr.VRApplication_Other)
try:
    vr = openvr.VRSystem()
    for index in range(openvr.k_unMaxTrackedDeviceCount):
        if vr.getTrackedDeviceClass(index) == openvr.TrackedDeviceClass_GenericTracker:
            print(vr.getStringTrackedDeviceProperty(index, openvr.Prop_SerialNumber_String))
finally:
    openvr.shutdown()
PYCODE
```

Match the five serials to the top-level `tracker_serials` mapping in `src/input_devices/openvr_input/config/openvr.yaml`:

| Key | Body segment |
| --- | --- |
| `chest` | Chest |
| `left_hand_back` | Back of left hand |
| `right_hand_back` | Back of right hand |
| `left_arm` | Left upper arm |
| `right_arm` | Right upper arm |

The supplied configuration contains the supplied installation's serials. Assign your actual hardware and keep each serial unique. Leave `calibration_file: tracker.yaml` in `openvr.yaml` to use the canonical runtime profile in `src/input_devices/openvr_input/calibration`.

`openvr.yaml` and `tracker.yaml` are local and ignored by Git. Their adjacent `.template` files are tracked. Setup or `python3 tools/configure.py --profile teleop` copies each template only when its runtime file is missing, preserving existing settings and calibration.

## Neutral-pose calibration

Configure Tracker serials in `openvr.yaml`, and arm Home, `base_from_body`, and `driver.tool_transforms` in `tianji_chest.yaml` before calibration. The robot TCP is the palm: both supplied flange-to-palm transforms have identity rotation and `[0, 0, 70]` mm translation. In the output Tracker profile, set `hand_back_to_palm.left/right.tracker_offset_m` to the measured palm-to-Tracker displacement. Both anatomical palms use X toward the fingers, Z toward the hand back, and Y = Z × X; both supplied offsets are `[0, 0, 0.05]` m. A static capture does not measure those mounting distances.

The output profile is `tracker.yaml` unless `--output FILENAME` selects another filename. Calibration reads that file when it exists; otherwise it starts from the bundled `tracker.yaml.template`, including its 50 mm offsets. The `calibration_file` setting in `openvr.yaml` selects the runtime profile and does not select calibration input.

Keep SteamVR running and stop other OpenVR input processes. The tool reads the default local OpenVR settings and robot arm configuration, plus the input profile and Tracker poses. It explicitly computes robot palm Home with local forward kinematics and displays that reference without connecting to the cabinet. The human mounting fit uses anatomical neutral-pose constraints independently of robot geometry.

```bash
source tools/env.sh
ros2 run openvr_input tracker_calibrate --inspect
ros2 run openvr_input tracker_calibrate --calibrate
```

Inspection is the default mode. It displays configured Home joints and palm positions, measured offsets, the initial human-neutral-to-Home position mapping, and each Tracker's status. It reads `tracker.yaml` with the same template fallback. `--output` is accepted only with `--calibrate`. Calibration follows this sequence:

1. Stand upright with upper arms naturally spread, elbows bent, hands in front, palms down, and fingers forward. Keep the pose symmetric.
2. Press Enter when ready. After the three-second countdown, hold still with all five Trackers visible for the five-second capture.
3. Review the fit errors, valid frame count, and initial mapping from human neutral positions to robot palm Home, then enter `save`, `retry`, or `cancel`.

The fit aligns the raw hand-back Trackers to anatomical palms under the neutral-pose and left/right mirror constraints, then subtracts the measured offsets to obtain palm positions. It saves neutral palm positions and orientations, plus each upper-arm Tracker's independent mean raw orientation, in the human chest frame. The saved profile preserves the chest mounting rotation and measured palm-to-Tracker offsets. Upper-arm orientation is a neutral reference for subsequent motion; this capture does not identify anatomical upper-arm twist. Robot Home remains in robot configuration and is used by retargeting to build the mapping.

By default, saving writes `src/input_devices/openvr_input/calibration/tracker.yaml` in a symlink build. The directory is fixed by the package; a regular install uses its package-share calibration directory. A successful save replaces the existing profile after writing it completely. Cancellation leaves the existing file unchanged.

`--output FILENAME` selects both the input and saved profile in the same directory; paths are not accepted. To use that profile at runtime, set `calibration_file` in `openvr.yaml` to its filename. That runtime setting may be a package calibration filename or an absolute path. For the normal canonical workflow, no selection change is needed after saving `tracker.yaml`.

Restart input and teleoperation processes after saving. No rebuild is required with the symlink setup. Measure and update the profile offsets when the glove mounting changes, then recalibrate; other Tracker mounting changes also require recalibration. Changing robot Home or flange-to-palm geometry requires restarting teleoperation to recompute retargeting and reload robot geometry; the human mounting calibration remains valid.

## Frames and verification

The OpenVR node owns the input TF tree. With the supplied `parent_frame: world`, it publishes:

| Transform | Meaning |
| --- | --- |
| `world` → `chest` | Calibrated human body axes: X forward, Y left, Z up; origin at the raw chest Tracker |
| `world` → `{side}_hand_back` | Raw hand-back Tracker pose |
| `world` → `{side}_arm` | Raw upper-arm Tracker pose |
| `{side}_hand_back` → `{side}_human_palm` | Static anatomical palm mounting transform |

The profile's `hand_back_to_palm` quaternions define the rotation `R` of each `{side}_human_palm` child in its raw `{side}_hand_back` frame. The child translation is `-R @ tracker_offset_m`; the offset is expressed in anatomical palm axes. The chest frame is a torso reference at the Tracker, not a shoulder estimate. The runtime input node and mounting-fit math do not load robot geometry. See the [profile fields](LOCAL_SETUP.md#tracker-calibration).

The retargeting controller reads palm poses and raw upper-arm orientations relative to `chest`. It maps displacement and rotation from the saved human neutral pose onto the current robot Home, using each arm's `base_from_body` rotation. Relative upper-arm Tracker rotation rotates the robot's Home redundancy direction; upper-arm Tracker positions are unused for IK. Robot flange-to-palm geometry belongs in `driver.tool_transforms.left/right`, in millimetres, and local Tianji IK applies it once.

Start input alone, then inspect TF from another terminal with the same environment:

```bash
source tools/env.sh
ros2 launch openvr_input openvr_input.launch.py
```

```bash
source tools/env.sh
ros2 run tf2_ros tf2_echo chest left_hand_back
```

The standalone launch reads the same OpenVR input configuration and selected Tracker profile as teleoperation. `rviz:=true` opens its Tracker viewer. An optional `input_config:=/absolute/path/to/openvr.yaml` selects another input configuration. Confirm that all five frames follow their assigned body segments and stop the standalone reader before launching another OpenVR reader.

For combined hand-target and Tracker visualization without actuator drivers:

```bash
ros2 launch wuji_teleop_bringup wuji_preview.launch.py
```

For arm teleoperation:

```bash
ros2 run wuji_teleop_bringup wuji_teleop.launch.py --disable-hand
```

This command automatically enables both arms, moves Home, and prepares for three seconds before following Tracker targets. `/tracker_arm_retarget` maps chest-relative human motion from the saved neutral pose onto configured robot Home and publishes `/{side}_arm/tcp_target` (`PoseStamped`, metres and an `x, y, z, w` quaternion) and `/{side}_arm/upper_arm_reference` (`Vector3Stamped`, unit direction), both in `{side}_base`. `/tianji_arm_controller` uses those targets and measured joint feedback to solve IK. A policy can supply the same messages without OpenVR; use one command source at a time. The existing `/left_arm/ee_pose` and `/right_arm/ee_pose` arrays remain target TCP values in metres/degrees for compatibility, not measured state. Check mounting, coordinate axes, Home, and emergency-stop access before starting motion.

While arm teleoperation is running, use another terminal to read the target messages or inspect the palm transform:

```bash
source tools/env.sh
ros2 topic echo /left_arm/tcp_target --once
ros2 topic echo /left_arm/upper_arm_reference --once
ros2 run tf2_ros tf2_echo left_hand_back left_human_palm
```

The OpenVR input node provides the palm transforms in standalone input, Preview, and teleoperation.
