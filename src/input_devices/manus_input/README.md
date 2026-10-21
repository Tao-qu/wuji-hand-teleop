# MANUS glove input

`manus_ros2` uses the MANUS Integrated SDK to publish `/manus_glove_0` and `/manus_glove_1`. Each hand controller subscribes to both topics, routes messages by `side`, and passes 21 MediaPipe-order keypoints to the official Wuji Hand 2 retargeting session. Tracker input supplies the separate arm pipeline.

## Setup and calibration

Run the project's [one-command setup](../../../docs/LOCAL_SETUP.md) to fetch native libraries, install USB rules, and build the ROS packages. Both MANUS executables use the bundled `ManusSDK/lib/libManusSDK_Integrated.so`. Complete MANUS SDK licensing through the vendor's process.

The current profiles are `manus_ros2/calibration/LeftMetaglovePro.mcal` and `manus_ros2/calibration/RightMetaglovePro.mcal`. Select them in the local hand configuration:

```yaml
manus:
  left_calibration_file: LeftMetaglovePro.mcal
  right_calibration_file: RightMetaglovePro.mcal
```

This block lives in `src/output_devices/wujihand2_output/wujihand2_ros2/config/wujihand2.yaml` and is shared by teleoperation and Preview. Filenames resolve in the MANUS package calibration directory; absolute paths also work. Empty values select the corresponding canonical profiles. A symlink build reads the source calibration directory.

Use profiles for the current operator and glove sides. The publisher retries a failed calibration load on publication ticks and marks the side loaded after SDK success. Skeleton publication continues independently. Restart the publisher after replacing a profile. See the [calibration guide](manus_ros2/CALIBRATION_GUIDE.md) for profile loading and the interactive `manus_calibrate` commands.

## Input contract

The conversion in `wujihand2_ros2/keypoints.py` produces a `(21, 3)` float32 array in metres:

| MediaPipe landmarks | MANUS node IDs |
| --- | --- |
| Wrist | `1` |
| Thumb | `22, 23, 24, 25` |
| Index | `3, 4, 5, 6` |
| Middle | `8, 9, 10, 11` |
| Ring | `13, 14, 15, 16` |
| Pinky | `18, 19, 20, 21` |

Metadata is mapped by `NodeInfo.nodeId` separately for each glove. Missing metadata uses a default entry. Missing mapped positions remain zero; repeated IDs use their last supplied position. Unmapped IDs are ignored. The conversion does not filter finite values, palm geometry, or hand size.

The publisher requests right-handed coordinates with Z up, X away from the viewer, and metre units through `CoreSdk_InitializeCoordinateSystemWithVUH`. The supplied `controller.flip_y: false` preserves these XYZ positions. Setting it to `true` reflects Y and reverses chirality. The SDK receives the mapped points directly through its built-in Wuji Hand 2 retargeting model.

`ManusGlove.header.stamp` records ROS time at entry to the SDK raw-skeleton callback. Skeletons copied in that callback share the stamp. `sequence` increases per glove when a callback updates its cache. The publisher runs at approximately 120 Hz and republishes the latest cached skeleton with its original stamp and sequence until another callback arrives. Topic frequency therefore includes repeated samples.

Each hand controller consumes the latest received ROS message once per control tick, at 120 Hz in the supplied configuration, and keeps one persistent retargeting session. It does not reject repeated sequences or old timestamps. Output contains all 20 joint positions in radians in firmware order: thumb, index, middle, ring, pinky, four joints each.

## Run and inspect

Run from the repository root:

```bash
source tools/env.sh
ros2 run wuji_teleop_bringup wuji_teleop.launch.py --disable-arm
```

This command automatically enables both hands and begins input following after the three-second preparation interval. In another terminal:

```bash
source tools/env.sh
ros2 topic hz /manus_glove_0
ros2 topic hz /manus_glove_1
ros2 topic hz /left_hand/joint_commands
ros2 topic hz /right_hand/joint_commands
```

To start only the publisher, using the canonical profiles and without robot drivers:

```bash
ros2 run manus_ros2 manus_data_publisher
```

To display finger targets and Tracker axes without actuator drivers:

```bash
ros2 launch wuji_teleop_bringup wuji_preview.launch.py
```

Preview uses `/preview/manus_glove_0`, `/preview/manus_glove_1`, `/preview/left/joint_states`, and `/preview/right/joint_states`. Its labels describe the last received inputs and targets. Stop other MANUS clients before starting another publisher or the calibration tool.
