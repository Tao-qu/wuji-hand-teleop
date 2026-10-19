# MANUS glove input for Wuji Hand 2

`manus_ros2` uses the Integrated SDK to publish `/manus_glove_0` and `/manus_glove_1`. Each selected hand controller subscribes to both topics, routes messages by `side`, and supplies validated MediaPipe-order keypoints to the official Wuji Hand 2 `RetargetSession`. Tracker data is used by the arm pipeline, not by this finger retargeting input.

## Preparation and calibration

Follow [Local environment setup](../../../docs/LOCAL_SETUP.md) to fetch SDK libraries through Git LFS, install MANUS USB rules, and build. The publisher links against `ManusSDK/lib/libManusSDK_Integrated.so`. Resolve SDK licensing through the vendor's licensing process.

Use a calibration profile for the current operator and the selected glove side. The publisher accepts ROS parameters `left_calibration_file` and `right_calibration_file`. An empty value uses the corresponding package-share `calibration/tpxLeftMetaglovePro.mcal` or `tpxRightMetaglovePro.mcal`. The TPX profiles were supplied by the operator and are installed with the package under their original filenames. Those bundled profiles do not replace calibration for the current operator. Keep new local profiles under the ignored `config/local_calibration/` directory and set an absolute path in the local Hand 2 YAML `manus` block. The bringup passes that block to the publisher.

Loading must succeed at both levels: the SDK return code and `SetGloveCalibrationReturnCode`. The publisher suppresses that glove's messages until loading succeeds, retries failures at most once every two seconds, and requires a valid raw frame from a callback that began after the successful load. Callback-entry time is captured before mutex waits and SDK data retrieval, so a callback already in flight across loading remains ineligible even if it completes afterwards. It does not label an old raw snapshot as fresh after loading. Restart the publisher to load a changed profile.

### Single-glove calibration tool

The `manus_calibrate` executable uses the bundled Integrated SDK and the [official MANUS calibration API sequence](https://docs.manus-meta.com/2.4.0/Plugins/SDK/SDK%20Client/). Stop the teleoperation stack and other MANUS clients, confirm all robot motors are disabled, and preserve the existing profile before calibration. The tool has no robot-control calls. Integrated Core may enumerate other paired gloves, but all calibration operations target the explicit glove ID after checking its actual side and paired state; it never chooses the first available glove.

Build and inspect first, replacing the example ID with the intended glove's verified ID:

```bash
source tools/env.sh
colcon build --packages-select manus_ros2 --symlink-install
source install/setup.bash
ros2 run manus_ros2 manus_calibrate --glove-id 0x0DB2AEBE --side right --inspect
```

Inspection is also the default when no mode is given. It prints the official step indices, titles, descriptions and durations, then the selected glove's raw node IDs, parent IDs, chain and joint types. It waits at most ten seconds for the raw hierarchy after discovery. Negative step durations mean continuous steps with an estimated duration, as defined by the SDK. Inspection makes no calibration Start, StartStep, Finish or SetGloveCalibration call. Raw SDK logs are suppressed to avoid exposing licensing data.

To calibrate only the verified right glove and export a new file:

```bash
mkdir -p config/local_calibration
ros2 run manus_ros2 manus_calibrate --glove-id 0x0DB2AEBE --side right \
  --calibrate --robots-disabled \
  --output "$PWD/config/local_calibration/right-20261019.mcal"
```

The output must be a new absolute path in an existing directory. The tool refuses to overwrite a file or symlink and never imports a previous profile. It first requires `start <decimal glove ID>`, then `step <index>` after the operator is ready for each SDK-provided pose, and finally `finish <decimal glove ID>`. Any other input or EOF cancels; after Start, cancellation requests the official Stop API and does not call Finish. Prompt input is interruptible: Ctrl-C or SIGTERM cancels while waiting for a confirmation, including an incomplete input line. StartStep is a blocking SDK call: an in-flight step must return before Stop is called. Keep following the displayed vendor instructions during each step.

Finish applies and saves calibration in Core, then the tool exports its bytes to the new file with owner-only permissions. If export fails after Finish, the tool reports that Core already applied the result; it cannot undo that operation. Set `manus.right_calibration_file` (or `left_calibration_file`) to the successful export before restarting the publisher, otherwise the configured previous profile will be loaded again. With robot motors still disabled, recheck a straight open hand, individual finger flexion, spread and thumb opposition before enabling following. Do not compensate for a bad glove calibration using robot joint offsets or relaxed limits.

Offline regression checks compile the tool against a fake SDK; they do not load the MANUS shared library or connect to devices:

```bash
python3 -m unittest discover -s src/input_devices/manus_input/manus_ros2/tools/tests -v
python3 -m unittest discover -s tests -p test_manus_metadata.py -v
```

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

The publisher queries metadata separately for each glove and maps each `NodeInfo.nodeId` to its metadata; it never indexes the SDK array with a node ID. Duplicate IDs or a mismatch between metadata and raw node IDs suppress publication while the hierarchy is retried. This avoids the previous out-of-bounds lookup for node 25 and incorrect parent/joint labels.

All 21 points must exist once and contain finite XYZ positions in metres. Missing or duplicate required nodes, a degenerate wrist/index/middle palm frame, and a hand radius exceeding 0.5 m are rejected. Other node IDs are ignored. Required points are never filled with zeros to hide missing data.

The publisher first calls `CoordinateSystemVUH_Init`, then explicitly assigns `handedness = Side_Right`, `up = AxisPolarity_PositiveZ`, `view = AxisView_XFromViewer`, and `unitScale = 1.0f` before calling `CoreSdk_InitializeCoordinateSystemWithVUH(..., true)`. This follows the [official MANUS client example](https://docs.manus-meta.com/2.4.0/Plugins/SDK/SDK%20Client/) and requests right-handed, Z-up, X-away-from-viewer world coordinates in metres. Assigning these fields only in the C++ member initializer is insufficient: the SDK struct initializer resets the axis and handedness fields to `Invalid`.

This explicit initialization corrects the previous reliance on an unspecified SDK coordinate fallback. After rebuilding, validate recorded poses with robot motors confirmed disabled before resuming live following. Results obtained with the previous binary do not validate the corrected coordinate contract. The matching controller default is now `flip_y: false`; an old local YAML with `flip_y: true` must be reviewed when migrating.

In `src/output_devices/wujihand_output/config/hand2.yaml`, `controller.flip_y: false` preserves the explicit right-handed MANUS XYZ positions. The official Wuji [live retargeting example](https://github.com/wuji-technology/wuji-sdk/blob/v2026.10.12/examples/python/retargeting/1.teleop_real.py) likewise passes right-handed glove skeleton positions directly to the session. `flip_y: true` remains available only for a separately validated source convention: it is a reflection with determinant -1, which reverses chirality rather than rotating the wrist. Verify both hands with open palm, individual palmar finger flexion, finger spread, and thumb opposition recordings while robot motors remain disabled. Compare both variants offline on identical frames with independent sessions; a flat hand or valid joint bounds alone cannot establish the correct chirality. The SDK receives the mapped points directly and chooses its built-in Wuji Hand 2 retargeting configuration; there is no extra wrist/MANO transform or per-side rotation YAML in this pipeline.

`ManusGlove.header.stamp` records ROS time at entry to the SDK raw-skeleton callback, before mutex waits or SDK data retrieval. All skeletons accepted from that callback share its timestamp. It is not regenerated by the publication timer and is not a glove hardware-clock timestamp. `sequence` increases per glove only when a new SDK frame is accepted. Repeated nonzero SDK publish timestamps are ignored. The ROS publisher emits each sequence once and drops frames older than its monotonic `source_timeout` (default 0.20 s).

The controller consumes the latest valid frame once at up to 120 Hz. It rejects duplicate/reordered sequences, non-increasing acquisition stamps, source age at or beyond `controller.input_timeout` (default 0.20 s), and stamps more than 0.05 s in the future. Its monotonic receipt deadline is shortened by the frame's existing source age. Publisher and controller must use the same ROS clock; for separate hosts, synchronize their system clocks. Restart both together if the publisher restarts its sequence counter. Regenerate `manus_ros2_msgs` and rebuild consumers after changing this message interface.

Invalid input or a tracking gap resets the persistent SDK session. The hand bridge independently watches command freshness and latches a fault on timeout; receiving input again does not automatically re-enable the hand.

## Start and inspect

Run commands from the repository root:

```bash
source tools/env.sh
ros2 launch wuji_teleop_bringup wuji_teleop.launch.py enable_arm:=false
# In another terminal after loading the environment:
ros2 topic hz /manus_glove_0
ros2 topic hz /manus_glove_1
ros2 topic hz /left_hand/joint_commands
ros2 topic hz /right_hand/joint_commands
```

Launch starts the input and retargeting processes with hand motors disabled by default. Confirm side routing, point units, and commands before explicitly enabling one hand through the GUI or `/left_hand/set_enabled` / `/right_hand/set_enabled` services. `hand_side:=left` or `right` restricts the hand pipeline. `hand_read_only:=true` skips MANUS and retargeting and subscribes only to the selected robot hand's feedback.

Offline contract checks, including official SDK retargeting without device discovery:

```bash
source tools/env.sh
python3 -m unittest discover -s tests -p test_hand_retarget.py -v
```
