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

Match the five serials to `openvr.tracker_serials` in `src/output_devices/tianji_output/tianji_output/config/tianji_chest.yaml`:

| Key | Body segment |
| --- | --- |
| `chest` | Chest |
| `left_hand_back` | Back of left hand |
| `right_hand_back` | Back of right hand |
| `left_arm` | Left upper arm |
| `right_arm` | Right upper arm |

The supplied configuration contains the supplied installation's serials. Assign your actual hardware and keep each serial unique. Leave `openvr.calibration_file: tracker.yaml` to use the canonical profile in `src/input_devices/openvr_input/calibration`.

`tracker.yaml` is local and ignored by Git. Its adjacent `tracker.yaml.template` is tracked. Setup or `python3 tools/configure.py --profile teleop` copies the template only when the runtime file is missing, preserving an existing calibration.

## Neutral-pose calibration

Configure arm Home, Tracker serials, and `driver.tool_transforms` before calibration. The robot TCP is the palm: both supplied flange-to-palm transforms have identity rotation and `[0, 0, 70]` mm translation. In the selected Tracker profile, set `hand_back_to_palm.left/right.tracker_offset_m` to the measured palm-to-Tracker displacement in aligned human palm axes. The supplied values are `[0, -0.05, 0]` m on the left and `[0, 0.05, 0]` m on the right. A static Home capture does not measure those mounting distances.

Keep SteamVR running and stop other OpenVR input processes. The tool reads the default local arm configuration, selected profile, Tracker poses, and local kinematics; it does not connect to the cabinet.

```bash
source tools/env.sh
ros2 run openvr_input tracker_calibrate --inspect
ros2 run openvr_input tracker_calibrate --calibrate
```

Inspection is the default mode. It displays the configured Home and each Tracker's status. Calibration follows this sequence:

1. Stand upright with upper arms naturally spread, elbows bent, hands in front, and palms down. Keep the pose symmetric.
2. Press Enter when ready. After the three-second countdown, hold still with all five Trackers visible for the five-second capture.
3. Review the displayed fit errors and valid frame count, then enter `save`, `retry`, or `cancel`.

The fit first aligns hand orientations, then subtracts the configured Tracker offsets to obtain human palm positions. It fits the chest/Home reference against the robot palm poses computed by forward kinematics at Home and calibrates upper-arm orientation.

Saving writes `src/input_devices/openvr_input/calibration/tracker.yaml` in a symlink build. The directory is fixed by the package; a regular install uses its package-share calibration directory. A successful save replaces the existing profile after writing it completely. Cancellation leaves the existing file unchanged.

`--output FILENAME` optionally selects another filename in the same directory; paths are not accepted. If used, set `openvr.calibration_file` to that filename. The configured value may be a package calibration filename or an absolute path. For the normal canonical workflow, no selection change is needed after saving `tracker.yaml`.

Restart input and teleoperation processes after saving. No rebuild is required with the symlink setup. Measure and update the profile offsets when the glove mounting changes, then recalibrate. Recalibrate after other Tracker mounting changes, flange-to-palm geometry changes, or arm Home changes.

## Frames and verification

The profile's `hand_back_to_palm` quaternions define the rotation `R` from each `{side}_hand_back` frame to its `{side}_human_palm` child. The child translation is `-R @ tracker_offset_m`, so the target follows the estimated human palm rather than the raw Tracker origin. The offset is expressed in the aligned human palm axes. The separate robot flange-to-palm geometry belongs in `driver.tool_transforms.left/right`, in millimetres, and local Tianji IK applies it once. The robot TCP is not a fingertip frame. The profile also holds chest transforms and upper-arm orientation corrections. See the [configuration reference](LOCAL_SETUP.md#tracker-calibration).

Start input alone, then inspect TF from another terminal with the same environment:

```bash
source tools/env.sh
ros2 launch openvr_input openvr_input.launch.py
```

```bash
source tools/env.sh
ros2 run tf2_ros tf2_echo chest left_hand_back
```

The standalone launch reads the same arm configuration and selected Tracker profile as teleoperation. `rviz:=true` opens its Tracker viewer. An optional `arm_config:=/absolute/path/to/tianji_chest.yaml` selects another arm configuration. Confirm that all five frames follow their assigned body segments and stop the standalone reader before launching another OpenVR reader.

For combined hand-target and Tracker visualization without actuator drivers:

```bash
ros2 launch wuji_teleop_bringup wuji_preview.launch.py
```

For arm teleoperation:

```bash
ros2 run wuji_teleop_bringup wuji_teleop.launch.py --disable-hand
```

This command automatically enables both arms, moves Home, and prepares for three seconds before following Tracker targets. `/tracker_arm_retarget` reads the calibrated palm poses relative to each chest reference and publishes `/{side}_arm/tcp_target` (`PoseStamped`, metres and an `x, y, z, w` quaternion) and `/{side}_arm/upper_arm_reference` (`Vector3Stamped`, unit direction), both in `{side}_base`. `/tianji_arm_controller` uses those targets and measured joint feedback to solve IK. A policy can supply the same messages without OpenVR; use one command source at a time. The existing `/left_arm/ee_pose` and `/right_arm/ee_pose` arrays remain target TCP values in metres/degrees for compatibility, not measured state. Check mounting, coordinate axes, Home, and emergency-stop access before starting motion.

While arm teleoperation is running, use another terminal to read the target messages or inspect the palm transform:

```bash
source tools/env.sh
ros2 topic echo /left_arm/tcp_target --once
ros2 topic echo /left_arm/upper_arm_reference --once
ros2 run tf2_ros tf2_echo left_hand_back left_human_palm
```

The palm transform is provided by teleoperation bringup; the standalone OpenVR reader publishes the five body-mounted Tracker frames.
