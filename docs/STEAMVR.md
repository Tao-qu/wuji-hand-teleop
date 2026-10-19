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

Enter five distinct hardware serial numbers in `src/input_devices/openvr_input/config/openvr_input.yaml`:

```yaml
tracker_serials:
  chest: LHR-CHEST
  left_wrist: LHR-LEFT-WRIST
  right_wrist: LHR-RIGHT-WRIST
  left_arm: LHR-LEFT-ARM
  right_arm: LHR-RIGHT-ARM
```

These values are placeholders. `wrist_offset` specifies the offset from the Tracker mounting point to the wrist and must match the actual installation. Chest and end-effector transforms are defined in `static_transforms.yaml`.

## Verification and operation

Start only the input node first, without connecting to the arms. Run the TF inspection command in another terminal with the environment loaded:

```bash
ros2 run openvr_input openvr_input
ros2 run tf2_ros tf2_echo chest left_wrist
```

Confirm that all five TF frames follow the corresponding body segments correctly. Stop the standalone input process before starting the arm pipeline:

```bash
ros2 launch wuji_teleop_bringup wuji_teleop.launch.py enable_hand:=false enable_rviz:=true
```

This launch automatically enables the arms and moves them to the configured initial pose before Tracker control. Pass `arm_auto_enable:=false` to enable them later through the GUI or ROS service.

The arms use chest-frame target poses and upper-arm directions to compute IK. Incorrect input configuration or mounting orientation can produce incorrect targets. Check RViz coordinate axes, the initial pose, and the emergency stop before the first run.
