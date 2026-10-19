# Local environment setup

## 1. System and ROS 2

Use Ubuntu 22.04 x86_64 and the system Python 3.10 interpreter. Follow the [official ROS 2 Humble Ubuntu installation guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) to configure package sources and install `ros-humble-desktop`. Do not build ROS nodes with a Conda Python interpreter.

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

Run the following commands from the repository root. Do not use sudo for project installation or builds; it is only needed for system packages and udev rules.

## 2. Source dependencies and Python environment

```bash
git lfs install
git lfs pull
python3 tools/fetch_dependencies.py
python3 tools/configure.py
/usr/bin/python3 -m venv --system-site-packages .venv
source tools/env.sh
python3 -m pip install "pip>=23,<27" "setuptools==68.2.2" "wheel==0.45.1"
python3 -m pip install -r requirements.txt
python3 -m pip install --no-deps -e src/wuji-retargeting
```

The virtual environment inherits system ROS and Qt packages. Numerical dependencies are pinned in `requirements.txt`. Only the core retargeting algorithm is used, so it is installed with `--no-deps`; this project manages its runtime dependencies. Do not install all example dependencies from the external project.

The dependency tool fetches pinned commits and initializes only the required hand-model submodules. If an existing checkout differs from the lock file, the tool reports an error instead of resetting the code. Resolve local changes in that directory before updating its revision. `src/wuji-retargeting/COLCON_IGNORE` prevents colcon from building the algorithm library and duplicate model packages as ROS packages.

## 3. SDKs and USB permissions

MANUS uses the bundled `libManusSDK_Integrated.so`; Tianji uses `libKine.so` and `libMarvinSDK.so`. Fetch these libraries through Git LFS. Verify that they are binary libraries rather than text pointer files:

```bash
file src/input_devices/manus_input/manus_ros2/ManusSDK/lib/libManusSDK_Integrated.so
file src/output_devices/tianji_output/tianji_output/_internal/lib/*.so
```

The Wuji ROS driver also requires a system C++ SDK. This project uses version 1.5.1:

```bash
mkdir -p work/downloads
curl -fL https://github.com/wuji-technology/wujihandpy/releases/download/v1.5.1/wujihandcpp-1.5.1-amd64.deb \
  -o work/downloads/wujihandcpp-1.5.1-amd64.deb
sudo apt install ./work/downloads/wujihandcpp-1.5.1-amd64.deb
sudo ldconfig
```

Install device permission rules, then unplug and reconnect the receivers and dexterous hands:

```bash
sudo install -m 644 src/input_devices/manus_input/config/udev/99-manus-libusb.rules /etc/udev/rules.d/
sudo install -m 644 config/udev/99-wuji-hand.rules /etc/udev/rules.d/
sudo usermod -aG plugdev,dialout "$USER"
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Log out and back in after changing group membership. SteamVR handles Tracker USB permissions and pairing; see [SteamVR setup](STEAMVR.md). `tools/env.sh` adds the MANUS and Tianji shared-library directories to the local process environment without modifying system libraries or creating global SDK symlinks.

## 4. Configuration and build

Fill in the serial numbers and controller cabinet IP address listed in the README, and prepare calibration files for both MANUS gloves. Generate local configuration files before building so they are installed into the ROS packages' share directories.

```bash
source tools/env.sh
rosdep install --from-paths src --ignore-src -r -y --skip-keys wujihandcpp
python3 -m colcon build --symlink-install \
  --cmake-args -DPython3_EXECUTABLE="$VIRTUAL_ENV/bin/python3" \
               -DPYTHON_EXECUTABLE="$VIRTUAL_ENV/bin/python3"
source tools/env.sh
python3 tools/check_environment.py
```

`wujihandcpp` is skipped by rosdep because it was installed from the deb package above. Follow the complete installation procedure for this repository. At runtime, `enable_hand:=false` prevents glove and dexterous-hand processes from starting.

Load `tools/env.sh` in every new terminal. Do not overwrite the current environment by sourcing another workspace's `install/setup.bash`. If the virtual environment or workspace path changes, remove this project's generated `build/`, `install/`, and `log/` directories and rebuild.

## 5. Troubleshooting

| Symptom | Check |
| --- | --- |
| SDK file is ASCII or a Git LFS pointer | Run `git lfs pull` from the repository root |
| Shared library fails to load | Run `source tools/env.sh`; use `ldd path/to/library.so` to identify missing dependencies |
| MANUS USB permission error | Install the udev rules, log in again, and reconnect the receiver |
| No MANUS skeleton data | Check SDK licensing, glove connections, and calibration logs; permissions and licensing are separate issues |
| Package or configuration not found | Generate configuration files, build, and source the environment again |
| `pinocchio` or NumPy import fails | Confirm that Python comes from `.venv`, then reinstall the requirements |
| SteamVR initialization fails | Start SteamVR in the same user's graphical session and check all five Trackers |
| GUI remains in starting state | Inspect MANUS commands, hand feedback, and arm lifecycle logs; readiness requires data from the selected pipelines |
| Cannot connect to the controller cabinet | Check its IP address, network routes, emergency stop, and whether another process owns the SDK connection |

The GUI brake tool connects directly to the controller cabinet. Do not connect it while the arm controller node is running. It does not replace the hardware emergency stop.
