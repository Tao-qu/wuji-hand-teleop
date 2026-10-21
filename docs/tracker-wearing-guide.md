# HTC Vive Tracker wearing guide

Pair all five Trackers with SteamVR and assign their actual serial numbers under `openvr.tracker_serials` in the local `tianji_chest.yaml`. See [SteamVR setup and serial discovery](STEAMVR.md) and the [arm configuration template](../src/output_devices/tianji_output/tianji_output/config/tianji_chest.yaml.template).

| Tracker role | Mounting |
| --- | --- |
| `chest` | Secure against the chest, parallel to the torso, with the USB-C port facing upward |
| `left_hand_back` | Secure on the back of the left hand so it follows the hand without shifting |
| `right_hand_back` | Secure on the back of the right hand so it follows the hand without shifting |
| `left_arm` | Outside of the left upper arm, with the USB-C port along the arm bone toward the torso |
| `right_arm` | Outside of the right upper arm, with the USB-C port along the arm bone toward the torso |

![Tracker wearing setup](images/tracker-wearing-combined.jpg)

Keep all mounts stable after calibration and keep each Tracker visible to at least one base station. The system uses each raw hand-back Tracker origin directly as the target position, so a shifted mount changes the commanded position.

Configure the robot's flange-to-virtual-hand-back-TCP transforms in `driver.tool_transforms.left/right`, then [capture the neutral-pose calibration](STEAMVR.md#neutral-pose-calibration) against the configured arm Home. The canonical result is `src/input_devices/openvr_input/calibration/tracker.yaml`, selected by `openvr.calibration_file`. Restart teleoperation after calibration. Repeat calibration when mounting, tool geometry, or Home changes.

During capture, stand upright, hold the arms symmetrically with elbows bent and palms down, and follow the countdown. During operation, maintain a stable torso and move slowly when adjusting posture.
