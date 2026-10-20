# Arm input with five Trackers

## Installation and pairing

Install Steam and SteamVR in the local graphical session, using the same operating-system account that runs teleoperation. Prepare five compatible Vive Trackers, their USB receivers, and Lighthouse base stations covering the workspace.

Pair each device through SteamVR and confirm valid tracking for every Tracker. Mount them on the chest, both wrists, and both upper arms. Upper-arm directions are used to resolve arm redundancy and must not be omitted. See the [wearing guide](tracker-wearing-guide.md).

## Headset-free configuration

After SteamVR has generated its user settings, close it and back up the configuration before editing. Installation paths depend on the Steam library location; use Steam's “Browse local files” option to find them.

Set `driver_null.enable` to `true` in SteamVR's `drivers/null/resources/settings/default.vrsettings`. In `steamvr.vrsettings` under the user's Steam configuration directory, merge the following keys into the existing `steamvr` object without replacing other settings:

```json
{
  "requireHmd": false,
  "forcedDriver": "null",
  "activateMultipleDrivers": true
}
```

Restart SteamVR and confirm that the Lighthouse driver and all Trackers work correctly. SteamVR updates may overwrite settings in its installation directory; verify headset-free operation on the installed version. If the graphical session is incompatible, try an Ubuntu Xorg session for diagnosis.

## Serial numbers and coordinate frames

Load the environment from the repository root, then enumerate devices:

```bash
source tools/env.sh
python3 - <<'PYCODE'
import openvr
openvr.init(openvr.VRApplication_Other)
try:
    vr = openvr.VRSystem()
    for i in range(openvr.k_unMaxTrackedDeviceCount):
        if vr.getTrackedDeviceClass(i) == openvr.TrackedDeviceClass_GenericTracker:
            print(vr.getStringTrackedDeviceProperty(i, openvr.Prop_SerialNumber_String))
finally:
    openvr.shutdown()
PYCODE
```

Enter five distinct hardware serial numbers under `openvr` in `src/output_devices/tianji_output/tianji_output/config/tianji_chest.yaml`:

```yaml
openvr:
  calibration_file: tracker-neutral.yaml
  tracker_serials:
    chest: LHR-CHEST
    left_wrist: LHR-LEFT-WRIST
    right_wrist: LHR-RIGHT-WRIST
    left_arm: LHR-LEFT-ARM
    right_arm: LHR-RIGHT-ARM
  publish_rate_hz: 120.0
  publish_tf: true
  parent_frame: world
```

The serial numbers above are placeholders. `openvr.calibration_file` selects a profile containing `wrist_offset`, `chest_mount`, `wrist_to_tianji`, and `upper_arm_rotations`. A filename resolves inside the `openvr_input` package's `calibration` directory; an absolute path is read directly. `wrist_offset` is the Tracker-to-wrist offset in the raw Tracker frame, in metres, and must match the mounting.

## Neutral-pose calibration

Set the arm Home joint positions, Tracker serials, and wrist offset before calibration. Keep SteamVR running and stop other OpenVR input processes. The calibration CLI uses the default project `tianji_chest.yaml`, reads Tracker poses and local kinematics, and does not connect to the cabinet.

```bash
source tools/env.sh
ros2 run openvr_input tracker_calibrate --inspect
ros2 run openvr_input tracker_calibrate --calibrate \
  --output "$PWD/src/input_devices/openvr_input/calibration/tracker-my-neutral.yaml"
```

Choose a new output filename in an existing directory. Follow the prompt to stand upright with arms held symmetrically, elbows bent, hands in front, and palms down. Keep all five Trackers visible and hold still for the five-second capture. Review the displayed fit, then choose `save`, `retry`, or `cancel`.

After saving, set `openvr.calibration_file` in `tianji_chest.yaml` to the exported file's absolute path and restart teleoperation. Profiles installed in the `openvr_input` package's `calibration` directory can be selected by filename. The exported profile is loaded directly; no fields need to be copied into other files. Recalibrate after changing mounting or the reference Home pose.

## Verification and operation

Start only the input node first, without connecting to the arms. Run the TF inspection command in another terminal with the environment loaded:

```bash
ros2 launch openvr_input openvr_input.launch.py
ros2 run tf2_ros tf2_echo chest left_wrist
```

The standalone launch reads the same arm configuration and calibration profile as teleoperation. To select another arm configuration, add `arm_config:=/absolute/path/to/tianji_chest.yaml`. Confirm that all five TF frames follow the corresponding body segments correctly. Stop the standalone input process before starting the arm pipeline:

```bash
ros2 launch wuji_teleop_bringup wuji_teleop.launch.py enable_hand:=false enable_rviz:=true
```

This launch automatically enables the arms and moves them to the configured initial pose before Tracker control. Pass `arm_auto_enable:=false` to enable them later through the GUI or ROS service.

The arms use chest-frame target poses and upper-arm directions to compute IK. Incorrect input configuration or mounting orientation can produce incorrect targets. Check RViz coordinate axes, the initial pose, and the emergency stop before the first run.
