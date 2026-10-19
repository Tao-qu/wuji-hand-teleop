# Wuji Hand 2 migration

This migration replaces the first-generation Wuji hand path in the MANUS + five-Tracker + Tianji system with two Wuji Hand 2.2 hands. Finger retargeting and device communication use the official Wuji SDK. The ROS bridge, input validation, lifecycle, and GUI integration are maintained in this repository.

This document describes the implementation and acceptance criteria. Record dated hardware results separately, with the exact configuration used. Passing a bounded joint-motion test does not establish MANUS following, arm payload calibration, or complete dual-arm teleoperation acceptance.

## Official dependencies and reference implementations

| Component | Pinned source | Use in this project |
| --- | --- | --- |
| Python runtime | `wuji-sdk==2026.10.12` in [requirements.txt](../requirements.txt) | Device discovery, identity reads, typed command/feedback APIs, and `RetargetSession` |
| SDK examples | [`wuji-sdk` v2026.10.12, commit `b71a55a55b002e4dc4178f38156f65686fdb3156`](https://github.com/wuji-technology/wuji-sdk/tree/b71a55a55b002e4dc4178f38156f65686fdb3156) | Reference API usage; the examples are not launched as the teleoperation driver |
| Hand 2.2 description | [`wuji-description` v2026.10.12, commit `829c9aa27cb12c3a49d5b8d63c9869dd8f656702`](https://github.com/wuji-technology/wuji-description/tree/829c9aa27cb12c3a49d5b8d63c9869dd8f656702/hand2/hand2_2/body) | Official `wuji_hand2_2_description` ROS package, joint names, limits, and geometry |
| Firmware policy | `driver.expected_firmware: "2.5.1"` | Exact configured-version check before writable initialization; this migration does not upgrade hand firmware |

[dependencies.lock.json](../dependencies.lock.json) records the model revision. [tools/fetch_dependencies.py](../tools/fetch_dependencies.py) fetches `hand2/hand2_2/body` into `src/wuji-description` and refuses to reset an existing checkout with a different revision or local changes. The relevant model files are `urdf/left-ros.urdf` and `urdf/right-ros.urdf`; mount frames are `l_mount` and `r_mount`. The old `wujihand_urdf` package is removed from the active source tree.

The implementation follows these official examples:

- [`retargeting/0.retarget_session.py`](https://github.com/wuji-technology/wuji-sdk/blob/b71a55a55b002e4dc4178f38156f65686fdb3156/examples/python/retargeting/0.retarget_session.py): persistent, hardware-independent `RetargetSession.for_hand(HandModel.WujiHand2, side=...)`, `step()`, and `reset()`.
- [`wuji_hand_2/0.subscribe_callback.py`](https://github.com/wuji-technology/wuji-sdk/blob/b71a55a55b002e4dc4178f38156f65686fdb3156/examples/python/wuji_hand_2/0.subscribe_callback.py): typed state and diagnostic callbacks.
- [`wuji_hand_2/2.publish.py`](https://github.com/wuji-technology/wuji-sdk/blob/b71a55a55b002e4dc4178f38156f65686fdb3156/examples/python/wuji_hand_2/2.publish.py): MIT parameter/current configuration, complete Enabled confirmation, and `JointCommand(position, velocity, effort)` publishing.
- [`retargeting/1.teleop_real.py`](https://github.com/wuji-technology/wuji-sdk/blob/b71a55a55b002e4dc4178f38156f65686fdb3156/examples/python/retargeting/1.teleop_real.py): integrating a persistent session with the real hand and entering control from measured posture.

The official publish example returns the hand to zero. This project does **not** copy that homing behavior: after explicit enable, it captures fresh measured positions, initially holds those positions, and ramps toward newly received teleoperation targets. It also rejects incomplete feedback rather than substituting zero for offline joints.

The SDK's built-in retargeting model comes with the pinned wheel; the separately pinned URDF supplies the ROS bridge's names and limits. Both need to be revalidated when either dependency changes. The [official SDK changelog](https://github.com/wuji-technology/wuji-sdk/blob/b71a55a55b002e4dc4178f38156f65686fdb3156/CHANGELOG.md) documents that the current Hand 2 API uses MIT control by default and no longer exposes the old Python `control_mode` setter. Discovery chooses the device data port: this setup uses firmware 2.5.1 on port 7447; firmware 2.10.0 and later uses a different default. Do not hard-code that port or infer a firmware-upgrade requirement from the latest examples.

## Input and joint mapping

```text
MANUS SDK raw skeleton callback
  → per-glove acquisition stamp + sequence
  → validate and map 21 landmarks for the reported side
  → persistent official RetargetSession
  → /{side}_hand/joint_commands
  → Hand 2 bridge limits, ramp, and watchdogs
  → official SDK JointCommand publisher
```

`manus_to_keypoints()` maps node IDs, not their order in a message. Its output is finite `float32` data with shape `(21, 3)`, in metres and MediaPipe landmark order:

| Landmark group | MediaPipe indices | MANUS node IDs |
| --- | --- | --- |
| Wrist | 0 | 1 |
| Thumb | 1–4 | 22, 23, 24, 25 |
| Index | 5–8 | 3, 4, 5, 6 |
| Middle | 9–12 | 8, 9, 10, 11 |
| Ring | 13–16 | 13, 14, 15, 16 |
| Pinky | 17–20 | 18, 19, 20, 21 |

Missing or duplicate required IDs, non-finite coordinates, excessive scale, and a degenerate palm are rejected. `controller.flip_y` defaults to `true` to preserve the current MANUS coordinate convention. Validate open-hand, flexion, abduction, and thumb-opposition poses on both sides before enabling live following. The old retargeter's wrist transforms and custom first-generation retargeting YAML are not applied to the official session.

The publisher advances the per-glove sequence only for a new SDK skeleton and publishes each sequence once. The controller routes by the message's `side`, rejects repeated/reordered sequences and stale acquisition stamps, and shortens its monotonic input deadline by the source age. A tracking gap or invalid frame resets the session and stops new commands. Restart publisher and controllers together if the publisher's sequence restarts.

Session output and unnamed ROS commands contain 20 radian positions in firmware order. Feedback NIDs include a tactile slot between fingers and therefore cannot be converted using `nid - 1`:

| Finger | Command indices | Joint NIDs | Official URDF suffixes in S1–S4 order |
| --- | --- | --- | --- |
| Thumb | 0–3 | 1, 2, 3, 4 | `thumb_cmc_flex`, `thumb_cmc_abd`, `thumb_mcp`, `thumb_ip` |
| Index | 4–7 | 6, 7, 8, 9 | `index_finger_mcp_flex`, `index_finger_mcp_abd`, `index_finger_pip`, `index_finger_dip` |
| Middle | 8–11 | 11, 12, 13, 14 | `middle_finger_mcp_flex`, `middle_finger_mcp_abd`, `middle_finger_pip`, `middle_finger_dip` |
| Ring | 12–15 | 16, 17, 18, 19 | `ring_finger_mcp_flex`, `ring_finger_mcp_abd`, `ring_finger_pip`, `ring_finger_dip` |
| Pinky | 16–19 | 21, 22, 23, 24 | `pinky_mcp_flex`, `pinky_mcp_abd`, `pinky_pip`, `pinky_dip` |

Prefix these suffixes with `l_` or `r_`. `joint_mapping.py` validates exactly 20 unique expected joints and loads bounds/velocity limits by name. Named commands are reordered explicitly. There is no inference from XML traversal order. SDK `effort` currently uses an ampere placeholder, so ROS `JointState.effort` remains empty and diagnostics expose `current_a`; the GUI labels current as A.

## Device ownership and control lifecycle

Each hand runs in a separate bridge process with an exact configured serial number. The backend checks device type, returned serial, handedness, and firmware before writable initialization. A per-serial host lock excludes a second instance of this adapter; it cannot exclude unrelated SDK clients on this or another host.

SDK callback threads only validate and cache samples. One worker owns device RPCs and command sends. Freshness evaluation reads the clock after taking a cache snapshot, and enable/clear acknowledgement must arrive after the associated RPCs complete. This avoids treating callbacks received during an RPC as future data.

| Interface/state | Contract |
| --- | --- |
| `read_only=true` | Subscribe and publish feedback only; no command publisher, parameter setters, enable, disable, clear-fault, or emergency-stop writes. It does not disable a previously enabled device. |
| Writable startup / `STOPPED` | Request disable on connection. Confirm physical disable from fresh complete diagnostics: 20 online, zero Enabled. |
| `set_enabled(true)` / `ENABLING` | Explicitly configure current/MIT gains and request enable. Require fresh complete state and all 20 diagnostics in Enabled before accepting motion. |
| `READY` | Hold the newly measured posture while awaiting the first command from the current enable session; the wait is bounded. |
| `ACTIVE` | Ramp from that posture and enforce official position bounds plus per-tick velocity limits. Send 20 `JointCommand(q, 0, 0)` values. |
| `FAULT` | Latch errors and request disable. Restored feedback or commands cannot re-enable the hand. |
| `clear_fault` | Explicit action; require a new fault-free disabled diagnostic frame before returning to `STOPPED`. A subsequent enable is separate. |
| Service response | Success acknowledges queue admission, not physical completion. Confirm lifecycle and fresh diagnostics. |
| Shutdown | Request disable, close owned streams, and disconnect the owned device. A lost transport cannot provide physical-disable confirmation. |

The default source timeout is 0.20 s, command/feedback watchdogs are 0.25 s, diagnostic timeout is 0.5 s, and startup ramp is 1.5 s. These are software thresholds, not guaranteed physical stop times. Measure detection, disable request, and disabled feedback separately. `hand_auto_enable` and `arm_auto_enable` default to `false`; a requested one-shot hand auto-enable waits for asynchronous startup disable and never resumes after a fault or manual stop.

## Changed implementation files

Paths below are relative to the repository root. See [local setup](LOCAL_SETUP.md) for build/configuration commands and the [bridge README](../src/wuji_hand2_driver/README.md) for its ROS contract.

| Area | Files | Result |
| --- | --- | --- |
| SDK bridge | `src/wuji_hand2_driver/wuji_hand2_driver/{sdk_backend,safety,joint_mapping,driver_node}.py`; package metadata and `test/` | Official SDK ownership, explicit lifecycle, complete feedback/diagnostics, limits, ramp, and fault handling |
| Retargeting and ROS hand transport | `src/output_devices/wujihand_output/wujihand_output/{keypoints,retargeting,wujihand_controller}.py`, `_internal/hand_interface.py`, `src/controller/controller/wujihand_node.py` | Replace the old hand/retargeting implementation with official sessions and bridge topics |
| MANUS freshness | `src/input_devices/manus_input/manus_ros2/src/ManusDataPublisher.{cpp,hpp}`, `manus_ros2_msgs/msg/ManusGlove.msg`, message build metadata | Acquisition timestamp and per-glove sequence; prevent frozen skeletons from appearing fresh |
| Configuration and launch | `src/output_devices/wujihand_output/config/hand2.yaml.template`, `src/wuji_teleop_bringup/launch/wuji_teleop.launch.py`, package/setup metadata | Exact left/right identities, optional single-hand/read-only launch, explicit enable defaults |
| Monitor | `src/wuji_teleop_monitor/wuji_teleop_monitor/ui/{hand2_support,sn_dialog,run_teleop,joint_panel}.py`, package metadata and `tests/` | Network discovery with read-only identity checks, independent enable/disable, live fault detection, named joints, correct current units |
| Models and environment | `dependencies.lock.json`, `requirements.txt`, `tools/{fetch_dependencies,configure,check_environment,sample_ros_inputs}.py`, `tools/env.sh` | Pinned official assets, local configuration preservation, environment checks and input evidence collection |
| Tianji boundary | `src/controller/controller/tianji_arm_node.py`, `src/output_devices/tianji_output/tianji_output/{tianji_arm_controller,tianji_chest_driver,tool_parameters,feedback_guard}.py`, `_internal/fx_robot.py`, `config/tianji_chest.yaml.template` | Explicit arm enable; verified cabinet feedback; configurable tool geometry/dynamics and local SDK selection |
| Retired first-generation assets | `src/wujihand_urdf/`, old `retarget_manus_*.yaml`, `wujihand_ik.yaml.template`, `config/udev/99-wuji-hand.rules` | Remove the first-generation model/configuration/USB path from this migration |
| Regression checks | `tests/test_hand_retarget.py`, `tests/test_input_tools.py`, `tests/test_tianji_safety.py`, `tests/test_tianji_sdk_binding.py`, bridge and monitor tests | Offline contracts, freshness, mapping, limit checks, lifecycle and SDK binding behavior |

## Tianji integration boundary

Changing the mounted hands also changes tool geometry, payload, centre of mass, and potentially cable loads. The default `tool_dynamics.source: controller` preserves cabinet values; it does not certify them for the Hand 2 assemblies. Measure the complete left and right assemblies independently before supplying `configured` values. Tool-transform translation is in millimetres; Tracker offsets are in metres. Apply each geometric offset once in IK or the target TF chain.

Cabinet SDK readiness requires advancing valid feedback for both arms. Socket-connect success alone is insufficient. The local control-library override is configured outside committed files, as described in [local setup](LOCAL_SETUP.md#3-sdk-libraries-and-device-access). Enabling the arms enters the configured initial-pose sequence, so hand-only acceptance and disabled arm feedback checks precede integrated arm motion.

Tactile metadata or an online indication does not establish usable tactile streams. Finger position teleoperation has no tactile dependency; tactile control or recording requires its own stream, unit, calibration, and dropout validation.

## Acceptance gates

These are completion criteria, not a statement that every stage has passed. Keep the configuration, software/model versions, serials, firmware, duration, frame counts, maximum ages, errors, commands, measured motion, and stop confirmation with each dated result.

| Gate | Required evidence |
| --- | --- |
| Offline build and contracts | Build the selected packages; pass input/retarget, mapping, lifecycle, mock SDK, monitor, and Tianji validation tests. Official sessions for both sides produce finite 20-value outputs within the pinned URDF limits. No device connection is needed for these checks. |
| Read-only identity and feedback | Verify each configured serial/side/firmware; receive distinct advancing state and diagnostic frames with all 20 expected NIDs, finite values, and understood error/status fields. No actuator or configuration writes occur in read-only mode. |
| Disabled MANUS pipeline | With physical disable confirmed, collect open, flexed, abducted, and opposition poses on both gloves. Verify freshness/side routing, direction, reasonable retarget output, and URDF bounds. Frozen/disconnected input must stop producing fresh commands. |
| Bounded per-hand motion | Test one hand at a time, from measured posture, with documented current/gain/speed/displacement limits. Confirm measured movement of the selected joint in the intended direction, evaluate unintended motion and tracking residual, and inspect current/error diagnostics. Merely reaching `ACTIVE` or observing encoder noise is insufficient. |
| Stop and recovery | Stop commands and measure watchdog-to-fault and confirmed-disable latency. Check normal disable and Ctrl-C cleanup. After a fault, restored input must not resume motion; explicit clear and a separate enable are required. Use a controlled procedure for any feedback-loss or network-loss test. |
| MANUS following | After bounded motion passes, validate all five fingers and thumb opposition separately for each hand, then both hands together. Assess startup transitions, tracking quality, comfortable range, and dropouts using recorded commands and actual positions. |
| Arms with new assemblies | Verify all five Tracker frames, cabinet endpoint/feedback, initial poses, tool transforms, and measured left/right payload data. Confirm explicit-enable, stop, and tracking-loss behavior with the mounted assemblies. |
| Integrated system | Complete a representative two-hand/two-arm task and sustained run without unexpected cross-side motion, stale-input continuation, or unexplained faults; record rates, latency, residuals, and operator observations. |

Do not extrapolate a small joint test to every finger or the complete MANUS mapping. MIT control is compliant: small commands may produce no measurable movement under static friction, and an observed residual is not automatically a transport failure. Report commanded displacement, measured displacement, gains, current limit, and current feedback together; investigate discrepancies before changing the acceptance claim.
