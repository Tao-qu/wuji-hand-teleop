# MANUS glove input for Wuji Hand 2

`manus_ros2` uses the Integrated SDK to publish `/manus_glove_0` and `/manus_glove_1`. Each selected hand controller subscribes to both topics, routes messages by `side`, and supplies MediaPipe-order keypoints to the official Wuji Hand 2 `RetargetSession`. Tracker data is used by the arm pipeline, not by this finger retargeting input.

## Preparation and calibration

Follow [Local environment setup](../../../docs/LOCAL_SETUP.md) to fetch SDK libraries through Git LFS, install MANUS USB rules, and build. The publisher links against `ManusSDK/lib/libManusSDK_Integrated.so`. Resolve SDK licensing through the vendor's licensing process.

Use a calibration profile for the current operator and the selected glove side. The publisher accepts ROS parameters `left_calibration_file` and `right_calibration_file`. An empty value uses the corresponding package-share `calibration/tpxLeftMetaglovePro.mcal` or `tpxRightMetaglovePro.mcal`. These profiles are installed with the package under their original filenames. To use another profile, set its absolute path in the local Hand 2 YAML `manus` block; bringup passes that block to the publisher. See the [calibration guide](manus_ros2/CALIBRATION_GUIDE.md) for file placement and parameter examples.

The publisher attempts calibration loading until `SetGloveCalibrationReturnCode_Success` is returned, then records that side as loaded for the process lifetime. Failed loads are retried on subsequent publication ticks and do not suppress skeleton publication. Restart the publisher to load a changed profile.

### Single-glove calibration tool

The `manus_calibrate` executable uses the bundled Integrated SDK and the [official MANUS calibration API sequence](https://docs.manus-meta.com/2.4.0/Plugins/SDK/SDK%20Client/). Stop the teleoperation stack and other MANUS clients, confirm all robot motors are disabled, and preserve the existing profile before calibration. The tool has no robot-control calls. Integrated Core may enumerate other paired gloves, but all calibration operations target the explicit glove ID after checking its actual side and paired state; it never chooses the first available glove.

Build and inspect first, replacing the example ID with the intended glove's verified ID:

```bash
source tools/env.sh
colcon build --packages-select manus_ros2_msgs manus_ros2 --symlink-install
source install/setup.bash
ros2 run manus_ros2 manus_calibrate --glove-id 0x0DB2AEBE --side right --inspect
```

Inspection is also the default when no mode is given. It prints the official step indices, titles, descriptions and durations, then the selected glove's raw node IDs, parent IDs, chain and joint types. It waits at most ten seconds for the raw hierarchy after discovery. Negative step durations mean continuous steps with an estimated duration, as defined by the SDK. Inspection makes no calibration Start, StartStep, Finish or SetGloveCalibration call. Raw SDK logs are suppressed to avoid exposing licensing data.

To calibrate only the verified right glove and export a new file:

```bash
ros2 run manus_ros2 manus_calibrate --glove-id 0x0DB2AEBE --side right \
  --calibrate --robots-disabled
```

`--output` is optional: `--side left` defaults to `LeftMetaglovePro.mcal`, and `--side right` defaults to `RightMetaglovePro.mcal`. Add `--output right-new.mcal` to choose another filename; directory and absolute paths are not accepted. With `--symlink-install`, the output directory is fixed at build time to `src/input_devices/manus_input/manus_ros2/calibration`; a regular install uses the package-share calibration directory. A successful export replaces an existing file of the same name. Cancellation or an incomplete export leaves the existing file unchanged. The tool never imports a previous profile.

The tool first requires `start <decimal glove ID>`, then `step <index>` after the operator is ready for each SDK-provided pose, and finally `finish <decimal glove ID>`. Any other input or EOF cancels; after Start, cancellation requests the official Stop API and does not call Finish. Prompt input is interruptible: Ctrl-C or SIGTERM cancels while waiting for a confirmation, including an incomplete input line. StartStep is a blocking SDK call: an in-flight step must return before Stop is called. Keep following the displayed vendor instructions during each step.

Finish applies and saves calibration in Core, then the tool exports its bytes to the selected file with owner-only permissions. If export fails after Finish, the tool reports that Core already applied the result; it cannot undo that operation. Set `manus.right_calibration_file` (or `left_calibration_file`) to the completed export's absolute path printed by the tool, then restart the publisher. The publisher's path selection is unchanged; a filename alone does not select this export. The `--robots-disabled` option is an operator declaration; the calibration tool does not query or control robot motors.

## Input contract

`wujihand_output/keypoints.py` maps MANUS IDs to these MediaPipe landmarks:

| Landmarks | MANUS node IDs |
| --- | --- |
| Wrist | `1` |
| Thumb | `22, 23, 24, 25` |
| Index | `3, 4, 5, 6` |
| Middle | `8, 9, 10, 11` |
| Ring | `13, 14, 15, 16` |
| Pinky | `18, 19, 20, 21` |

The publisher queries metadata separately for each glove and maps each `NodeInfo.nodeId` to its metadata; it never indexes the SDK array with a node ID. Repeated metadata IDs overwrite their earlier entry. Missing metadata uses a default entry rather than rejecting the raw frame.

The conversion returns a `(21, 3)` float32 array in metres. Missing mapped node IDs remain `(0, 0, 0)`; repeated IDs use their last supplied position. Unmapped node IDs do not contribute to the result. The conversion performs no finite-value, palm-geometry or hand-size rejection.

The publisher first calls `CoordinateSystemVUH_Init`, then explicitly assigns `handedness = Side_Right`, `up = AxisPolarity_PositiveZ`, `view = AxisView_XFromViewer`, and `unitScale = 1.0f` before calling `CoreSdk_InitializeCoordinateSystemWithVUH(..., true)`. This follows the [official MANUS client example](https://docs.manus-meta.com/2.4.0/Plugins/SDK/SDK%20Client/) and requests right-handed, Z-up, X-away-from-viewer world coordinates in metres. Assigning these fields only in the C++ member initializer is insufficient: the SDK struct initializer resets the axis and handedness fields to `Invalid`.

In `src/output_devices/wujihand_output/config/hand2.yaml`, `controller.flip_y: false` preserves the explicit right-handed MANUS XYZ positions. The official Wuji [live retargeting example](https://github.com/wuji-technology/wuji-sdk/blob/v2026.10.12/examples/python/retargeting/1.teleop_real.py) likewise passes glove skeleton positions directly to the session. `flip_y: true` reflects Y and reverses chirality rather than rotating the wrist. The SDK receives the mapped points directly and chooses its built-in Wuji Hand 2 retargeting configuration; there is no extra wrist/MANO transform or per-side rotation YAML in this pipeline.

`ManusGlove.header.stamp` records ROS time at entry to the SDK raw-skeleton callback, before mutex waits or SDK data retrieval. All skeletons copied from that callback share its timestamp. It is not regenerated by the publication timer and is not a glove hardware-clock timestamp. `sequence` increases per glove when a callback updates its cached skeleton. The publisher does not compare SDK timestamps or discard repeated callbacks.

The publisher runs at approximately 120 Hz and repeats the latest cached skeleton, including its original stamp and sequence, until the SDK supplies another one. It has no source-age timeout or publish-once sequence gate. Topic frequency therefore measures ROS publications, including repeats.

The controller consumes the latest received ROS message once per control tick, at 120 Hz by default. It does not reject repeated sequences or old timestamps, and it does not reset its persistent SDK session on a tracking gap. Official session output is sent as all 20 joint positions in firmware order.

## Start and inspect

Run commands from the repository root:

```bash
source tools/env.sh
ros2 run wuji_teleop_bringup wuji_teleop.launch.py --disable-arm
# In another terminal after loading the environment:
ros2 topic hz /manus_glove_0
ros2 topic hz /manus_glove_1
ros2 topic hz /left_hand/joint_commands
ros2 topic hz /right_hand/joint_commands
```

This hands-only command automatically enables both hands and starts following after the fixed three-second preparation interval. The hand `set_enabled` services remain available. Omitting `--disable-arm` also starts and automatically enables both arms.

For MANUS input alone, without starting robot drivers:

```bash
source tools/env.sh
ros2 run manus_ros2 manus_data_publisher
```

For hand-model and Tracker visualization without actuator drivers:

```bash
source tools/env.sh
ROS_DOMAIN_ID=71 ros2 launch wuji_teleop_bringup wuji_preview.launch.py
```

Preview publishes hand targets under `/preview/left/joint_states` and `/preview/right/joint_states`. Its labels describe the last received input or target; there is no stale-input timeout. Stop other MANUS clients before starting another publisher or the calibration tool.
