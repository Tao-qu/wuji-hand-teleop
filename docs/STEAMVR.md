# SteamVR and five-Tracker arm input

## Installation and pairing

Install Steam and SteamVR in the local graphical session using the same operating-system account as teleoperation. Prepare five compatible Vive Trackers, their USB receivers, and Lighthouse base stations covering the workspace. Pair each Tracker in SteamVR and confirm valid tracking before starting the ROS input node.

Mount one Tracker on the chest, one on the back of each hand, and one on the outside of each upper arm. Both upper-arm Trackers are required for the arm redundancy calculation. See the [wearing guide](tracker-wearing-guide.md).

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

Configure arm Home, Tracker serials, and `driver.tool_transforms` before calibration. Keep SteamVR running and stop other OpenVR input processes. The tool reads the default local arm configuration, Tracker poses, and local kinematics; it does not connect to the cabinet.

```bash
source tools/env.sh
ros2 run openvr_input tracker_calibrate --inspect
ros2 run openvr_input tracker_calibrate --calibrate
```

Inspection is the default mode. It displays the configured Home and each Tracker's status. Calibration follows this sequence:

1. Stand upright with upper arms naturally spread, elbows bent, hands in front, and palms down. Keep the pose symmetric.
2. Press Enter when ready. After the three-second countdown, hold still with all five Trackers visible for the five-second capture.
3. Review the displayed fit errors and valid frame count, then enter `save`, `retry`, or `cancel`.

Saving writes `src/input_devices/openvr_input/calibration/tracker.yaml` in a symlink build. The directory is fixed by the package; a regular install uses its package-share calibration directory. A successful save replaces the existing profile after writing it completely. Cancellation leaves the existing file unchanged.

`--output FILENAME` optionally selects another filename in the same directory; paths are not accepted. If used, set `openvr.calibration_file` to that filename. The configured value may be a package calibration filename or an absolute path. For the normal canonical workflow, no selection change is needed after saving `tracker.yaml`.

Restart input and teleoperation processes after saving. No rebuild is required with the symlink setup. Recalibrate after changing Tracker mounting, flange-to-TCP geometry, or arm Home.

## Frames and verification

The raw `left_hand_back` and `right_hand_back` origins supply target position directly. The profile's `hand_back_to_tcp` quaternions align orientation with `left_tcp_target` and `right_tcp_target`, using zero translation. The robot's flange-to-virtual-hand-back-TCP geometry belongs in `driver.tool_transforms.left/right`, in millimetres. Its axes are parallel to the flange axes, and local Tianji IK applies it once. The profile also holds chest transforms and upper-arm orientation corrections. See the [configuration reference](LOCAL_SETUP.md#tracker-calibration).

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

This command automatically enables both arms, moves Home, and prepares for three seconds before following Tracker targets. The controller uses the TCP targets relative to each chest frame and the upper-arm reference directions to solve IK. Check mounting, coordinate axes, Home, and emergency-stop access before starting motion.
