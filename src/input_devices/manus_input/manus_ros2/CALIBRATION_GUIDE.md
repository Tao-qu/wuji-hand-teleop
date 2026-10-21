# MANUS calibration

The MANUS publisher loads one `.mcal` profile for each glove side. The `manus_calibrate` executable inspects or calibrates one explicitly selected glove and exports a profile through the bundled Integrated SDK.

## Profiles and selection

The canonical profiles are:

```text
src/input_devices/manus_input/manus_ros2/calibration/LeftMetaglovePro.mcal
src/input_devices/manus_input/manus_ros2/calibration/RightMetaglovePro.mcal
```

These runtime profiles are local and ignored by Git. The adjacent `LeftMetaglovePro.mcal.template` and `RightMetaglovePro.mcal.template` files are tracked, complete binary profiles, not editable text. Setup or `python3 tools/configure.py` copies each template only when the runtime profile is missing. Existing profiles and new calibration results are preserved. `python3 tools/configure.py --check` checks both profiles, the Tracker profile, and the two device configurations without writing.

The package installs `calibration/` into its share directory. The project symlink build reads the source calibration directory. A regular install reads `share/manus_ros2/calibration` under the package install prefix.

Teleoperation and Preview pass the `manus` block from the local `src/output_devices/wujihand2_output/wujihand2_ros2/config/wujihand2.yaml` to the publisher:

```yaml
manus:
  left_calibration_file: LeftMetaglovePro.mcal
  right_calibration_file: RightMetaglovePro.mcal
```

Each parameter accepts a filename in the package calibration directory or an absolute file path. An empty string selects that side's canonical filename. Use the correct profile for the operator and glove side.

For the publisher alone, without robot drivers:

```bash
source tools/env.sh
ros2 run manus_ros2 manus_data_publisher
```

Standalone parameter overrides use standard ROS syntax:

```bash
ros2 run manus_ros2 manus_data_publisher --ros-args \
  -p left_calibration_file:=LeftMetaglovePro.mcal \
  -p right_calibration_file:=RightMetaglovePro.mcal
```

The project's [setup command](../../../../docs/LOCAL_SETUP.md) builds both MANUS executables and prepares their native library and USB access.

## Inspect a glove

Stop other MANUS clients before running the tool. Before calibration, stop teleoperation and confirm that robot motors are disabled. The tool targets the explicit glove ID, checks its side and paired state, and makes no robot-control calls.

Replace the example with the intended glove's ID in decimal or `0x` hexadecimal form:

```bash
source tools/env.sh
ros2 run manus_ros2 manus_calibrate --glove-id 0x0DB2AEBE --side right --inspect
```

Inspection is also the default when no mode is supplied. It prints the SDK step indices, titles, pose instructions, durations, and raw node hierarchy. Discovery waits up to 20 seconds and hierarchy retrieval up to 10 seconds. Inspection does not start or apply calibration. The tool suppresses raw SDK output and displays its own selected fields.

## Calibrate and save

```bash
ros2 run manus_ros2 manus_calibrate --glove-id 0x0DB2AEBE --side right \
  --calibrate --robots-disabled
```

`--robots-disabled` is the operator's declaration; the tool does not query motor state. Follow the displayed SDK instructions:

1. Enter `start <decimal glove ID>` when ready to begin.
2. Prepare each requested pose, then enter `step <index>`.
3. Enter `finish <decimal glove ID>` to apply the result in Core and export the profile.

Use the decimal ID printed by the tool in interactive confirmations even if the command-line ID was hexadecimal. Negative step durations denote continuous SDK steps with an estimated duration. Keep following the displayed pose instructions until each SDK step returns.

The default output is `LeftMetaglovePro.mcal` for `--side left` and `RightMetaglovePro.mcal` for `--side right`. In the project symlink build it is written into the source `calibration` directory above. A regular install uses the package-share directory. `--output FILENAME` optionally chooses another filename in that directory; directory and absolute paths are rejected.

A successful export replaces an existing file of the same name with owner-only permissions. Cancellation or an incomplete export leaves the existing file unchanged. The tool does not import an existing profile before calibration. After saving the canonical filename, restart the publisher to use it. If another filename was supplied, select it in the matching `manus.left_calibration_file` or `manus.right_calibration_file` parameter and restart. A symlink build needs no rebuild after replacing a profile.

Any other confirmation input or EOF cancels. Ctrl-C and SIGTERM also request cancellation while waiting for input, including an incomplete line. After Start, cancellation calls the SDK Stop operation; an in-flight blocking SDK step must return first. Cancellation before Start performs no Stop call.

Finish applies and saves calibration in Core before export. If export subsequently fails, Core has already applied the result and the tool reports that distinction. Export is limited to 16 MiB.

## Verify loading

For each detected side, the publisher reads the selected nonempty file in full and calls `CoreSdk_SetGloveCalibration`. It records the side as loaded only after `SetGloveCalibrationReturnCode_Success`. Failed loads are logged and retried on publication ticks. Skeleton publication continues independently of loading.

| SDK result | Meaning |
| --- | --- |
| `Success` | Calibration loaded |
| `WrongSideError` | Profile side does not match the glove |
| `VersionError` | Calibration version mismatch |
| `GloveNotFoundError` | Requested glove was not found |
| `UserServiceError` | SDK user-service error |
| `DeserializationError` | Calibration data could not be deserialized |
| `Error` | Generic calibration error |

The publisher logs numeric SDK results on failure. File-not-found, empty-file, and incomplete-read errors occur before the SDK call. Check the success log for the intended side, then validate raw poses with robot motors disabled. A running topic alone does not establish that calibration loaded or that a new sample arrived: the publisher repeats its latest cached skeleton.

A successful profile loads once per side for the process lifetime. Restart after changing its path or contents. With a regular install, rebuild to update installed profile copies; an explicitly configured absolute path is read directly.
