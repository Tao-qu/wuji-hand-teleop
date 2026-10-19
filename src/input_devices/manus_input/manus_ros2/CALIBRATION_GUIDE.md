# MANUS calibration profiles and calibration tool

The publisher loads a separate `.mcal` profile for each glove side through the bundled MANUS Integrated SDK. File loading and the interactive `manus_calibrate` executable are already implemented; no C++ or CMake changes are needed to select another profile.

## Installed profiles

The package contains these default files:

```text
manus_ros2/
├── calibration/
│   ├── tpxLeftMetaglovePro.mcal
│   └── tpxRightMetaglovePro.mcal
├── ManusSDK/
├── src/
│   ├── ManusDataPublisher.cpp
│   └── ManusDataPublisher.hpp
└── tools/
    └── manus_calibrate.cpp
```

`CMakeLists.txt` installs `calibration/` into the package share directory. For a normal isolated colcon install, the files are located at:

```text
install/manus_ros2/share/manus_ros2/calibration/tpxLeftMetaglovePro.mcal
install/manus_ros2/share/manus_ros2/calibration/tpxRightMetaglovePro.mcal
```

The publisher resolves that directory through `ament_index_cpp`. Keep the original TPX filenames when using the packaged defaults. A profile supplied through a parameter can have any filename; select the profile for the intended operator and glove side.

## Select profiles

The publisher declares two string parameters:

| Parameter | Default when empty |
| --- | --- |
| `left_calibration_file` | Package-share `calibration/tpxLeftMetaglovePro.mcal` |
| `right_calibration_file` | Package-share `calibration/tpxRightMetaglovePro.mcal` |

For normal bringup, edit the `manus` block in the local `src/output_devices/wujihand_output/config/hand2.yaml`:

```yaml
manus:
  left_calibration_file: /absolute/path/to/left-profile.mcal
  right_calibration_file: /absolute/path/to/right-profile.mcal
```

Use `""` for either value to select that side's packaged TPX profile. The launch file passes this block to `manus_data_publisher`. `hand_side` selects robot hand controllers; it does not stop the MANUS publisher from handling both connected gloves.

To run only the glove publisher with profile overrides, from the repository root:

```bash
source tools/env.sh
ros2 run manus_ros2 manus_data_publisher --ros-args \
  -p left_calibration_file:=/absolute/path/to/left-profile.mcal \
  -p right_calibration_file:=/absolute/path/to/right-profile.mcal
```

This standalone command starts no robot drivers. The normal `wuji_teleop.launch.py` launch automatically enables hands and arms by default; it is not needed to load or create a MANUS calibration profile.

## Build and load behavior

From the repository root:

```bash
source tools/env.sh
colcon build --packages-select manus_ros2_msgs manus_ros2 --symlink-install
source install/setup.bash
ros2 run manus_ros2 manus_data_publisher
```

See [Local environment setup](../../../../docs/LOCAL_SETUP.md) for SDK libraries, USB rules and the workspace environment. Both MANUS executables use `ManusSDK/lib/libManusSDK_Integrated.so`.

For each detected side, the publisher opens the selected file, checks that it is nonempty and reads it in full, then calls `CoreSdk_SetGloveCalibration`. It marks that side as loaded when the calibration result is `SetGloveCalibrationReturnCode_Success`. There is no 16 MiB file-size cap in the publisher.

A failed load is logged and retried on subsequent publication ticks until it succeeds. Skeleton publication continues independently of calibration success. Once successful, loading is skipped for that side for the rest of the process lifetime. Restart the publisher after changing a file or its configured path. Rebuild when changing packaged calibration files so the installed copy is updated.

The publication timer repeats the latest cached skeleton, even when no new SDK callback arrives. The message stamp and sequence describe the callback that supplied the cache; they are not publication freshness checks. A running topic alone does not establish that a profile loaded successfully or that a new glove sample arrived. Check the publisher's calibration result message for the intended side.

## Inspect or create a profile

`manus_calibrate` operates on one explicitly selected glove ID. Stop other MANUS clients before running it. Stop teleoperation and confirm robot motors are disabled before calibration; the tool has no robot-control calls and does not verify motor state itself.

Inspection is the default mode. Replace the example ID with the intended glove's ID, expressed as decimal or `0x` hexadecimal:

```bash
source tools/env.sh
ros2 run manus_ros2 manus_calibrate --glove-id 0x0DB2AEBE --side right --inspect
```

The tool verifies the selected glove's side and paired state, prints the SDK calibration step titles, instructions and durations, and lists the raw node hierarchy by node ID. Discovery waits up to 20 seconds; hierarchy retrieval waits up to 10 seconds. Inspection makes no calibration Start, StartStep, Finish or SetGloveCalibration call. The tool suppresses raw SDK output and prints its own selected fields.

To create a new profile:

```bash
mkdir -p config/local_calibration
ros2 run manus_ros2 manus_calibrate --glove-id 0x0DB2AEBE --side right \
  --calibrate --robots-disabled \
  --output "$PWD/config/local_calibration/right-new.mcal"
```

The output must be a new absolute path whose parent directory exists. Existing files and symlinks are not overwritten. The tool does not import the publisher's configured profile before calibration.

Follow the prompts in order:

1. Enter `start <decimal glove ID>` to begin.
2. Prepare the pose described by the SDK, then enter `step <index>` for each step.
3. Enter `finish <decimal glove ID>` to apply the result in Core and export the profile.

Use the decimal ID printed by the tool in interactive confirmations, even if the command-line ID was hexadecimal. Any other prompt input or EOF cancels. Ctrl-C and SIGTERM also request cancellation, including while waiting for an incomplete input line. A blocking SDK step must return before cancellation can call Stop. A negative SDK step duration denotes a continuous step with an estimated duration.

After Start and before Finish, cancellation requests the SDK Stop operation and does not apply the result through Finish. Cancelling before Start performs no Stop call. Successful Finish applies and saves calibration in Core, then the tool exports the bytes to the new file with owner-only permissions. If export fails after Finish, Core has already applied the result; the tool reports that distinction.

Set the matching `manus.left_calibration_file` or `manus.right_calibration_file` to the completed export before restarting the publisher. Otherwise startup reloads the previously configured profile. The calibration tool retains its own export-size limit of 16 MiB; this is separate from the publisher's file-loading behavior.

## Calibration load results

The SDK declares these `SetGloveCalibrationReturnCode` values:

| Result | Meaning |
| --- | --- |
| `Success` | Calibration loaded |
| `WrongSideError` | Profile side does not match the glove |
| `VersionError` | Calibration version mismatch |
| `GloveNotFoundError` | Requested glove was not found |
| `UserServiceError` | SDK user-service error |
| `DeserializationError` | Calibration data could not be deserialized |
| `Error` | Generic calibration error |

The publisher logs the numeric calibration result on failure. File-not-found, empty-file and incomplete-read messages occur before calibration is sent to the SDK. Keep left and right paths distinct and restart the publisher after correcting a path or replacing a profile.
