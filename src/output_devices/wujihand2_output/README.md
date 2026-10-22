# Wuji Hand 2.2 ROS adapter

`wujihand2_ros2` connects each hand through the official `wuji-sdk==2026.10.12` typed Hand 2 API. Each hand runs in its own driver process with its configured serial number and namespace. The package also provides the official retargeting wrapper, MANUS keypoint conversion, and ROS command interface.

The ROS package is in `wujihand2_ros2/`. Its sibling `wuji-description/` is the pinned official model checkout; `hand2/hand2_2/body` provides `wuji_hand2_2_description`. The [project setup command](../../../docs/LOCAL_SETUP.md) installs dependencies, fetches that model, and builds both packages.

## Configuration and launch

The local configuration is `wujihand2_ros2/config/wujihand2.yaml`, created from its adjacent template when missing. Teleoperation, Home, and the monitor use this file. It supplies each hand's serial number, ROS namespace, 20 Home positions, driver parameters, controller parameters, and MANUS profile selection.

```bash
source tools/env.sh
ros2 run wuji_teleop_bringup wuji_teleop.launch.py --disable-arm
```

The command starts both hands and MANUS, enables the hand drivers, and begins following after the three-second preparation interval. Add `--home` to send the configured Home positions and exit without MANUS. See the [operating commands](../../../README.md#start-teleoperation) for combined mode, monitor, and services.

Both hands use `WujiHand2ControllerNode` with the node name `controller`. Their namespaces come from `left_hand.name` and `right_hand.name` in the hand configuration:

| Hand | Controller node | Driver node |
| --- | --- | --- |
| Left | `/left_hand/controller` | `/left_hand/wujihand2_driver` |
| Right | `/right_hand/controller` | `/right_hand/wujihand2_driver` |

The controller executable is `wujihand2_controller`. Its `--side` argument selects the hand model and MANUS input side; `--hand-name` sets its default namespace and hand topic prefix. Commands, feedback, and lifecycle topics use the hand namespace, such as `/left_hand/joint_commands`. MANUS inputs and the arm readiness topic remain global.

Query each controller's parameters while teleoperation is running:

```bash
ros2 param list /left_hand/controller
ros2 param list /right_hand/controller
```

The driver executable is `wujihand2_driver`. Its parameters are:

| Parameter | Node default | Meaning |
| --- | --- | --- |
| `serial_number` | Empty | SDK device serial number |
| `expected_side` | Empty | `left` or `right`, used for joint names and device label |
| `auto_enable` | `true` | Enable after connection and gain configuration |
| `command_rate` | `200.0` | SDK command loop rate in Hz |
| `publish_rate` | `200.0` | ROS feedback publication timer rate in Hz |
| `diagnostics_rate` | `10.0` | Diagnostics and lifecycle publication rate in Hz |
| `kp` | `3.0` | SDK MIT proportional gain |
| `kd` | `0.05` | SDK MIT derivative gain |

Bringup loads the YAML `driver` block and supplies each hand's serial, side, and `auto_enable: true`. The supplied runtime configuration uses `kp: 5.0` and `kd: 0.1`; these override the node defaults above.

## Topics and services

Interfaces are relative to the driver namespace, normally `/left_hand` or `/right_hand`:

| Interface | ROS type | Content |
| --- | --- | --- |
| `joint_commands` | `sensor_msgs/JointState` | Twenty named target positions in radians |
| `commanded_joint_states` | `sensor_msgs/JointState` | Named targets submitted to the SDK, stamped after each successful send |
| `joint_states` | `sensor_msgs/JointState` | SDK positions and velocities with joint names, stamped when the callback receives the sample |
| `diagnostics` | `diagnostic_msgs/DiagnosticArray` | Device status, motor current, temperature, voltage, and joint error codes |
| `lifecycle_state` | `std_msgs/String` | Driver lifecycle state |
| `set_enabled` | `std_srvs/srv/SetBool` | Queue enable or disable |
| `emergency_stop` | `std_srvs/srv/Trigger` | Queue SDK emergency stop |
| `clear_fault` | `std_srvs/srv/Trigger` | Queue SDK fault clearing |

Project controllers and Home publish names in firmware order. A named command is reordered by joint name; an unnamed command uses firmware order. Velocity, effort, and ROS timestamps do not change target interpretation. The worker sends SDK `JointCommand(position, 0.0, 0.0)` values while enabled and retains the latest target between messages.

`commanded_joint_states` reports each successful SDK submission, including repeated submissions of the current target at the configured command rate. Its timestamp is the submission time, not a measured arrival time. Record this stream for submitted actions and `joint_states` for observations. Feedback timestamps identify host receipt in the SDK callback, not a device hardware clock or the later ROS publication timer.

| Finger | Command indices | Feedback joint NIDs |
| --- | --- | --- |
| Thumb | 0–3 | 1–4 |
| Index | 4–7 | 6–9 |
| Middle | 8–11 | 11–14 |
| Ring | 12–15 | 16–19 |
| Pinky | 16–19 | 21–24 |

Feedback is sorted by joint NID and published once per newly received state sample. Joint names match the official Hand 2.2 model with `l_` or `r_` prefixes. `JointState.effort` is empty; diagnostics report motor current as `current_a` in amperes.

```bash
ros2 topic hz /left_hand/joint_states
ros2 topic echo /left_hand/lifecycle_state
ros2 topic echo /left_hand/diagnostics
ros2 service call /left_hand/set_enabled std_srvs/srv/SetBool '{data: false}'
```

## Lifecycle and shutdown

A driver begins `DISCONNECTED`. After connection and gain configuration it enters `STOPPED`; enabling enters `READY`, and a successful target send enters `ACTIVE`. Disable and emergency stop enter `STOPPED`. Closing the connection returns to `DISCONNECTED`.

Service responses acknowledge queue admission. The SDK worker performs the requested operation and reports the result through lifecycle and diagnostics. Fault clearing calls the SDK without changing the driver's enabled flag.

SDK callbacks cache feedback. One worker owns RPCs and command writes while ROS timers publish cached state and diagnostics. On shutdown, the worker requests disable, closes its streams, and disconnects. The controller releases its ROS publishers and subscriptions; driver shutdown releases the hardware connection.
