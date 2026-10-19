# Wuji Hand 2 ROS adapter

This package connects each configured hand through the official `wuji-sdk==2026.10.12` typed Hand 2 API. Run one driver process per hand with its serial number and left/right namespace. The main [project README](../../README.md) covers installation and launch commands.

## Parameters

| Parameter | Default | Purpose |
| --- | --- | --- |
| `serial_number` | empty | SDK device serial number |
| `expected_side` | empty | `left` or `right`, used for joint names and device label |
| `auto_enable` | `true` | Enable after connection and gain configuration |
| `command_rate` | `200.0` | SDK command loop, Hz |
| `publish_rate` | `200.0` | ROS feedback publication timer, Hz |
| `diagnostics_rate` | `10.0` | ROS diagnostics and lifecycle publication, Hz |
| `kp` | `3.0` | SDK MIT proportional gain |
| `kd` | `0.05` | SDK MIT derivative gain |

The launch file loads the local `hand2.yaml` driver block, then supplies the selected serial, side, and `hand_auto_enable` argument. Local gains may differ from the template defaults.

## Topics and services

All interfaces are relative to the driver namespace (`left_hand` or `right_hand`).

| Interface | ROS type | Content |
| --- | --- | --- |
| `joint_commands` | `sensor_msgs/JointState` | Twenty target positions in radians, firmware order |
| `joint_states` | `sensor_msgs/JointState` | SDK positions and velocities with joint names |
| `diagnostics` | `diagnostic_msgs/DiagnosticArray` | Device status and per-joint current, temperature, voltage, and error code |
| `lifecycle_state` | `std_msgs/String` | Driver lifecycle |
| `set_enabled` | `std_srvs/SetBool` | Queue enable or disable |
| `emergency_stop` | `std_srvs/Trigger` | Queue the SDK emergency-stop operation |
| `clear_fault` | `std_srvs/Trigger` | Queue the SDK clear-fault operation |

Commands use the `position` array in thumb, index, middle, ring, pinky order, four joints per finger. The driver uses that order directly; `name`, `velocity`, `effort`, and the ROS timestamp do not change target interpretation. The worker publishes `JointCommand(position, 0.0, 0.0)` values while enabled, retaining the latest target between messages.

Feedback is ordered by SDK NID:

| Finger | Command indices | Joint NIDs |
| --- | --- | --- |
| Thumb | 0–3 | 1–4 |
| Index | 4–7 | 6–9 |
| Middle | 8–11 | 11–14 |
| Ring | 12–15 | 16–19 |
| Pinky | 16–19 | 21–24 |

The driver publishes each newly received state sample once. `JointState.effort` is empty; `diagnostics` exposes motor current as `current_a`. Joint names use the official Hand 2.2 names with `l_` or `r_` prefixes.

## Lifecycle

Connection starts in `DISCONNECTED`. After connecting, the worker sets MIT gains and enters `STOPPED`; automatic enable then enters `READY`. A successful target send enters `ACTIVE`. Disable and emergency-stop operations enter `STOPPED`; closing the connection returns to `DISCONNECTED`.

Service responses acknowledge queue admission. The SDK worker performs the operation and reports its result through lifecycle and diagnostics. Clear-fault invokes the SDK directly without changing the driver's enabled flag. Device error codes are displayed in diagnostics.

SDK callbacks cache feedback. One worker thread owns RPCs and command writes, while ROS timers publish cached state and diagnostics. On driver shutdown, the worker requests disable, closes its streams, and disconnects. Releasing a hand controller closes its ROS publishers and subscriptions; driver shutdown owns hardware cleanup.
