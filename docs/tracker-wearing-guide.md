# HTC Vive Tracker wearing guide

Pair all five Trackers with SteamVR and assign their actual serial numbers under `tracker_serials` in the local `src/input_devices/openvr_input/config/openvr.yaml`. See [SteamVR setup and serial discovery](STEAMVR.md) and the [input configuration template](../src/input_devices/openvr_input/config/openvr.yaml.template).

| Tracker role | Mounting |
| --- | --- |
| `chest` | Secure against the chest, facing forward and parallel to the torso, with the USB-C port facing upward |
| `left_hand_back` | Secure on the back of the left MANUS glove so it follows the hand without shifting |
| `right_hand_back` | Secure on the back of the right MANUS glove so it follows the hand without shifting |
| `left_arm` | Outside of the left upper arm, facing outward, with the USB-C port along the arm bone toward the torso |
| `right_arm` | Outside of the right upper arm, facing outward, with the USB-C port along the arm bone toward the torso |

![Tracker wearing setup](images/tracker-wearing-combined.jpg)

Keep all mounts stable after calibration and keep each Tracker visible to at least one base station. The system estimates each human palm from the glove-mounted hand-back Tracker using a measured mounting offset. A shifted mount changes that estimate. Upper-arm Trackers supply orientation for the IK reference direction; their positions are unused.

Configure the robot's flange-to-palm TCP transforms in `driver.tool_transforms.left/right`; both supplied transforms use identity rotation and `[0, 0, 70]` mm translation. The TCP is the robot palm, not a fingertip. Separately, measure the displacement from each human palm to its Tracker origin and enter it in `hand_back_to_palm.left/right.tracker_offset_m` in the calibration output profile: `tracker.yaml` by default, or the filename passed to `--output`. Use anatomical palm axes and metres: X points toward the fingers, Z toward the hand back, and Y = Z × X on both sides. Both supplied offsets are `[0, 0, 0.05]` m. These 50 mm offsets describe the glove mounts; robot flange transforms remain separate.

Then [capture the human neutral pose](STEAMVR.md#neutral-pose-calibration). The calibration CLI reads and displays the configured robot Home reference for the initial mapping. It reads the existing output profile or, if missing, the bundled `tracker.yaml.template`. The anatomical fit aligns hand mounting rotations, applies the measured offsets, and saves neutral palm poses and raw upper-arm Tracker orientations in the human chest frame. A static capture cannot determine the mounting distance; saving preserves the input offsets. The canonical result is `src/input_devices/openvr_input/calibration/tracker.yaml`; `calibration_file` in `openvr.yaml` selects the runtime profile. Restart teleoperation after calibration. Repeat input calibration when Tracker mounting changes, updating measured offsets when needed. Robot Home or tool changes require a teleoperation restart to recompute retargeting and reload robot geometry.

During capture, stand upright, hold the arms symmetrically with elbows bent, palms down, and fingers forward, and follow the countdown. During operation, maintain a stable torso and move slowly when adjusting posture.
