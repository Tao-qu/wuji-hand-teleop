# MANUS glove input

`manus_ros2` uses the Integrated SDK to publish `/manus_glove_0` and `/manus_glove_1`. Both hand controllers subscribe to both topics, route messages by their `side` field, and convert the skeleton into 21 three-dimensional keypoints.

## Preparation

Follow [Local environment setup](../../../docs/LOCAL_SETUP.md) to fetch the SDK through Git LFS, install USB permission rules, and build. The driver links against `ManusSDK/lib/libManusSDK_Integrated.so`. Resolve SDK licensing issues through the vendor's licensing process.

## Calibration

Use MANUS Core to export `.mcal` files for the current operator's left and right hands, then place them at:

- `manus_ros2/calibration/LeftMetaglovePro.mcal`
- `manus_ros2/calibration/RightMetaglovePro.mcal`

The existing files do not replace calibration for the current operator. After replacing them, rebuild `manus_ros2` and check the driver logs for successful calibration loading. See the [calibration guide](manus_ros2/CALIBRATION_GUIDE.md) for details. Connect only the intended glove pair and confirm that the two message sources carry the appropriate left/right side values.

## Running and verification

```bash
source tools/env.sh
ros2 launch wuji_teleop_bringup wuji_teleop.launch.py enable_arm:=false
# Check from another terminal after loading the environment
ros2 topic hz /manus_glove_0
ros2 topic hz /manus_glove_1
ros2 topic hz /left_hand/joint_commands
ros2 topic hz /right_hand/joint_commands
```

Run commands from the repository root. MANUS is the fixed hand input; no input-source selection is required. Retargeting parameters are stored in `src/output_devices/wujihand_output/config/retarget_manus_left.yaml` and `retarget_manus_right.yaml`. Rotation corrections differ between hands. Validate each hand separately after changing filtering, scaling, or pinch parameters.
