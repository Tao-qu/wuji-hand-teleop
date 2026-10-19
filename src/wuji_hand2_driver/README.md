# Wuji Hand 2 ROS adapter

This package uses the official `wuji-sdk==2026.10.12` typed Hand 2 API and
checks firmware `2.5.1` by default. It loads joint names and limits from the
official `wuji_hand2_2_description` package. Importing modules or running the
pure/mock tests does not connect to hardware.

Use one process and one configured serial number per hand. `serial_number`
and `expected_side` are mandatory. The backend verifies model, SN, handedness,
and firmware before any write. A host process lock prevents another instance
of this adapter claiming the same SN; external SDK clients must still be stopped.

`read_only=true` never creates a command publisher or calls enable, disable,
clear-fault, emergency-stop, or parameter setters. In control mode a successful
connection first disables the hand. `auto_enable=false` is the default.

All topics/services are relative to the launch namespace (`left_hand` or
`right_hand`). `joint_commands` accepts `sensor_msgs/JointState` with 20 finite
radian positions in firmware order and a fresh nonzero ROS header timestamp.
Names may be empty or the complete official URDF joint names in any order.
Velocity/effort command arrays must be empty. Bad commands while armed latch a
fault. Targets outside official URDF limits are rejected. Outgoing targets
obey both the URDF velocity limit and `max_velocity` (default 0.5 rad/s).

`joint_states` reports actual SDK positions/velocities in explicit firmware
order. It does not republish an unchanged SDK frame with a new ROS timestamp.
`JointState.effort` stays empty because SDK effort currently uses an ampere
placeholder; `diagnostics` exposes `current_a` separately.

Services enqueue work on the single device owner thread. A successful service
response acknowledges the request, not completed physical enable/disable;
confirm the `lifecycle_state` and fresh diagnostics. `set_enabled` is
`std_srvs/SetBool`; `emergency_stop` and `clear_fault` are `std_srvs/Trigger`.
Clear-fault is never automatic and never enables the hand.

Lifecycle: `STOPPED` → `ENABLING` → `READY` → `ACTIVE`. Enable waits for a new,
complete frame with all 20 joints Enabled and fresh measured positions, then
holds that measured pose while awaiting the first new command (up to 5 s).
The first command ramps from that pose over 1.5 s, with rate limits at every
tick. There is no return-to-zero behavior. Active command timeout (0.25 s),
feedback timeout (0.25 s), diagnostic timeout (0.5 s), incomplete frames,
device faults, and send errors latch `FAULT` and request disable. Recovery
requires explicit clear-fault, fresh fault-free disabled diagnostics, and an
explicit enable. Reconnected data or commands cannot re-arm the device.

`READ_ONLY` identifies passive monitoring and `DISCONNECTED` identifies no
connection. `STOPPED` by itself is not physical-disable confirmation: check
all 20 diagnostic states, since device actions are asynchronous.

The official SDK callback threads only cache samples. A dedicated fixed-rate
worker owns all SDK writes/RPCs, while ROS timers publish cached data. On
shutdown it requests disable before closing streams and disconnecting only
its own device. Software disable cannot be confirmed after a lost transport
or forced process termination; use the hardware emergency stop for those cases.

Run tests without ROS or hardware:

```bash
python3 -m pytest -q src/wuji_hand2_driver/test
```
