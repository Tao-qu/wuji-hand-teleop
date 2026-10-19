# Local environment setup

## 1. System and ROS 2

Use Ubuntu 22.04 x86_64 and the system Python 3.10 interpreter. Follow the [official ROS 2 Humble Ubuntu installation guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) to configure package sources and install `ros-humble-desktop`. Use a system-Python virtual environment for this workspace.

```bash
sudo apt update
sudo apt install -y build-essential cmake git git-lfs curl usbutils \
  python3-dev python3-pip python3-venv python3-colcon-common-extensions \
  python3-rosdep python3-pyqt5 python3-pyqt5.qtsvg \
  libusb-1.0-0-dev libncursesw5-dev libspdlog-dev libgl1-mesa-glx
source /opt/ros/humble/setup.bash
# Run once if rosdep has not been initialized on this machine
sudo rosdep init
rosdep update
```

Run the remaining commands from the repository root. Project installation and builds do not require sudo.

## 2. Official model and Python environment

```bash
git lfs install
git lfs pull
python3 tools/fetch_dependencies.py
python3 tools/configure.py
/usr/bin/python3 -m venv --system-site-packages .venv
source tools/env.sh
python3 -m pip install "pip>=23,<27" "setuptools==68.2.2" "wheel==0.45.1"
python3 -m pip install -r requirements.txt
```

The environment inherits ROS and Qt packages. `requirements.txt` pins `wuji-sdk==2026.10.12`; the SDK contains the Hand 2 retargeting runtime, and its NumPy interface is used directly. No separate retargeting project is installed.

`dependencies.lock.json` pins the official `wuji-description` revision. The fetch tool checks out only `hand2/hand2_2/body` under `src/wuji-description`, providing the `wuji_hand2_2_description` ROS package. It rejects changed revisions or dirty existing checkouts rather than resetting them. The rest of the driver is maintained in this repository under `src/wuji_hand2_driver`.

## 3. SDK libraries and device access

MANUS uses the bundled `libManusSDK_Integrated.so`; Tianji uses `libKine.so` and `libMarvinSDK.so`. Fetch these through Git LFS, and check that they are ELF libraries rather than text pointers:

```bash
file src/input_devices/manus_input/manus_ros2/ManusSDK/lib/libManusSDK_Integrated.so
file src/output_devices/tianji_output/tianji_output/_internal/lib/*.so
sudo install -m 644 src/input_devices/manus_input/config/udev/99-manus-libusb.rules /etc/udev/rules.d/
sudo usermod -aG plugdev,dialout "$USER"
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Log out and back in after changing groups, then reconnect the MANUS receivers. SDK licensing and calibration are separate from USB permissions. SteamVR handles Tracker pairing and access; see [SteamVR setup](STEAMVR.md).

Wuji Hand 2 uses the official network SDK. Configure each hand's exact serial number in `hand2.yaml`; the bridge discovers its address and checks its device type, handedness, and configured firmware version. Place the host and hands on a network where SDK discovery works. The Tianji cabinet IP is set separately in `tianji_chest.yaml`; configure it for the actual cabinet instead of assuming the template address applies.

`tools/env.sh` adds the MANUS and Tianji library directories to the process environment and selects the project Python packages. It does not modify global SDK libraries.

To select a vendor-supplied Tianji control SDK, create the ignored local file `tools/env.local.sh` and set its absolute library path:

```bash
# tools/env.local.sh — replace this example path with the vendor-supplied library
export TIANJI_SDK_LIBRARY="/absolute/path/to/libMarvinSDK.so"
```

`tools/env.sh` sources this optional file before loading ROS and the workspace. The control wrapper uses `TIANJI_SDK_LIBRARY` when set, otherwise the bundled library. The kinematics library remains the bundled `libKine.so`. `tools/check_environment.py` checks the selected control library; loading successfully is not proof that its protocol matches the cabinet. Use the controller's verified SDK endpoint, not an adjacent onboard computer's address. The arm connection issues a read-only VERSION request and checks actual controller feedback before proceeding; a successful socket bind alone is not connection readiness. Keep local library paths out of committed configuration.

The bundled Linux kinematics library uses a 472-byte IK parameter structure, including `m_Output_JntExdABS`. The offline native test checks this layout and verifies that FK/IK calls do not overwrite adjacent memory. Keep its Python binding paired with the bundled library; another `libKine.so` may use a different structure. Restart running arm nodes after changing native bindings.

## 4. Configuration and build

Fill in local `hand2.yaml`, `openvr_input.yaml`, and `tianji_chest.yaml` files generated from their templates. Prepare MANUS calibration files for the current operator before running input. The hand configuration has three sections:

- `left_hand` and `right_hand`: unique `serial_number` and ROS `name`.
- `driver`: model/firmware, update rates, watchdog deadlines, ramp, velocity, MIT gains, current limit, and enable policy.
- `controller`: MANUS `control_rate`, `input_timeout`, and configurable `flip_y` reflection.

The configuration tool manages only the three project templates and never replaces an existing local file. `python3 tools/configure.py --check` reports missing local files without writing. Generate local configuration before building, so the files are installed into ROS share directories:

```bash
source tools/env.sh
rosdep install --from-paths src --ignore-src -r -y
python3 -m colcon build --symlink-install \
  --cmake-args -DPython3_EXECUTABLE="$VIRTUAL_ENV/bin/python3" \
               -DPYTHON_EXECUTABLE="$VIRTUAL_ENV/bin/python3"
source tools/env.sh
python3 tools/check_environment.py --mode all
python3 -m pip install pytest
python3 -m pytest -q tests
```

Load `tools/env.sh` in every new terminal. If the workspace path or interpreter changes, regenerate this project's `build/`, `install/`, and `log/` outputs and rebuild. Do not source an unrelated workspace over this one.

## 5. Separate feedback inspection from motion

First inspect one hand without sending device writes:

```bash
source tools/env.sh
ros2 launch wuji_teleop_bringup wuji_teleop.launch.py \
  enable_arm:=false hand_side:=left hand_read_only:=true
```

Read-only mode connects to the selected hand and subscribes to feedback. It skips MANUS and retargeting, and rejects enable or other device writes. It is not a motor-disable operation: inspect the hand's actual state if another controller previously enabled it. Stop that process before starting a writable controller for the same serial.

Normal launch starts the selected pipelines with hand and arm auto-enable disabled. Use the GUI's individual Enable buttons or the services documented in the [README](../README.md). The hand bridge configures current/gains only through its writable enable path; it checks complete state/diagnostics, ramps from measured posture, and enforces command limits. Check both hands' motion mapping separately before enabling arm motion.

Tianji tool dynamics default to the values already stored in the cabinet. They have not been recalibrated by this migration. Measure the complete assemblies before selecting `tool_dynamics.source: configured`. Optional `tool_transforms.left/right` are rigid flange-to-target 4×4 matrices with translation in millimetres; do not duplicate the same offset in TF.

## 6. Troubleshooting

| Symptom | Check |
| --- | --- |
| SDK file is ASCII or a Git LFS pointer | Run `git lfs pull` |
| Shared library fails to load | Source the environment; use `ldd path/to/library.so` |
| Official SDK version mismatch | Select `.venv` and reinstall `requirements.txt` |
| Model package or configuration not found | Fetch dependencies, generate configuration, build, and source again |
| Hand not discovered | Check the exact SN, network links/routes, and discovery visibility |
| Hand refuses connection or enable | Inspect type, handedness, firmware, full feedback, and diagnostics; stop any other owner of that SN |
| Hand remains STOPPED | Explicitly enable it after checking the input and workspace |
| Hand enters FAULT after lost input | Restore fresh MANUS/feedback, resolve the cause, call `clear_fault`, then explicitly enable |
| MANUS frame rejected | Inspect node IDs, metre units, palm geometry, source sequence/stamp, and matching ROS clocks; restart publisher/controller together after a publisher restart |
| No MANUS skeleton data | Check receiver permissions, SDK licensing, calibration, and publisher logs |
| SteamVR fails | Run it in the same graphical session and confirm all five Trackers |
| Cabinet connection fails | Check the configured IP, routes, emergency stop, and competing SDK processes |

The GUI's separate brake tool owns a direct cabinet connection. Stop arm teleoperation before using it. Software startup checks and offline tests are not evidence of hardware acceptance.

## 7. Read-only ROS input sampling

To inspect inputs that are already being published, use the metadata sampler. It does not launch input processes, import a device SDK, enable motors, or send ROS commands/services:

```bash
source tools/env.sh
python3 tools/sample_ros_inputs.py --duration 10 --mode all \
  --output work/input-summary.json --require-fresh
# Separate pipelines, or one selected glove:
python3 tools/sample_ros_inputs.py --duration 10 --mode trackers --output work/tracker-summary.json
python3 tools/sample_ros_inputs.py --duration 10 --mode hands --sides left --output work/left-input-summary.json
```

The JSON contains MANUS frame counts, observed rates, sequence gaps/repeats/reordering, source and receipt ages, and the five expected dynamic Tracker TF frames. Static TF is reported separately without treating its latched timestamp as stale. `--max-age` defaults to 0.20 s; `--require-fresh` returns a nonzero exit code if a required stream is missing, frozen, or stale at the end. Sequence gaps may also result from latest-frame sampling, so they are not a direct transport-loss measurement. Output contains no poses, SDK log text, license fields, or calibration data. Save this JSON summary for acceptance records instead of copying raw SDK logs.
