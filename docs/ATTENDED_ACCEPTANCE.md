# Attended two-hand / two-arm acceptance

This runbook resumes the Hand 2.2 migration on the installed hardware. It is not a passing acceptance record. The right hand previously reported Encoder1 quality warning `3`, and the right arm previously rejected torque entry with error `6`. Position holding, fresh disabled feedback, and passing unit tests do not resolve those failures.

Keep the retained Home, existing payload readback, TPX calibration profiles, and configured Tracker assignments. The operator has deferred mass/centre-of-mass and mounting-transform identification. Do not substitute the older project's Home or guessed tool dynamics during this test.

## 1. Software and actuator-free preview

From the repository root:

```bash
source tools/env.sh
python tools/check_environment.py --mode all
python tools/test_offline.py -q
ROS_DOMAIN_ID=71 ros2 launch wuji_teleop_bringup wuji_preview.launch.py
```

The preview uses a separate ROS domain and publishes preview topics only. It does not start hand or arm drivers or provide motor-enable services. It shows official Hand 2 target geometry and Tracker frames; it is not a simulation of arm mechanics or actual motor feedback. Visible text and `/preview/diagnostics` report input freshness; when stale, the retained model pose is historical. Stop it with Ctrl-C before starting the normal system. All terminals inspecting the preview must use the same `ROS_DOMAIN_ID=71`.

With the operator wearing both gloves and all five Trackers, check:

- Left/right separation, four-finger extension and shallow flexion, individual finger motion, thumb opposition, and abduction direction.
- Chest, both wrists, and both upper-arm frames remain available and fresh. Confirm that left/right arm motion changes the corresponding frames.
- Preview diagnostics distinguish current targets from stale or missing input. A frozen model after input loss is the last target, not continuing valid input.

Do not use a tight fist as the first target. The recorded deep-curl targets contain inter-finger contact in the approximate official model. Earlier operator confirmation of direction predates the MANUS coordinate correction, so repeat it with the current binary.

| Placement | Serial |
| --- | --- |
| Chest | `LHR-8304D4CF` |
| Left wrist | `LHR-9BFC64E3` |
| Right wrist | `LHR-BC4F0E1C` |
| Left upper arm | `LHR-1ACBC7E2` |
| Right upper arm | `LHR-55DCF12E` |

## 2. Fresh disabled hardware baseline

The operator must be present at the emergency stop for the following stages, with the workspace empty and both hand power leads secured. Inspect the right middle finger and its cabling before another bounded movement. Follow the manufacturer's power-off procedure for any physical connector or mechanism inspection.

Use the normal ROS domain in each terminal and start the monitor:

```bash
source tools/env.sh
unset ROS_DOMAIN_ID
ros2 run wuji_teleop_monitor monitor
```

Start Teleop with automatic enable disabled. This normal launch connects to actuators and may request standby during startup; it is different from the actuator-free preview. Before any Enable, require fresh feedback for both arms and all 20 joints of each hand, zero enabled hand joints, no active hand errors, and stable arm state/error `0/0`. A queued service response, cached zero, or cleared error number alone is insufficient. If emergency stop release leaves `100/13`, use the existing explicit one-shot clear path and confirm fresh stable `0/0`; do not run a repeated clear/re-enable loop.

In another normal-domain terminal, capture input metadata without commanding motion:

```bash
python tools/sample_ros_inputs.py --duration 10 --mode all --require-fresh \
  --output work/morning-input-baseline.json
```

Review every expected stream, including stale/reordered source samples and timing margin, rather than relying only on the final fresh flag. The configured source-age limit is 0.20 seconds.

## 3. Separate attended motion stages

Enable and evaluate one chain at a time using the monitor. Keep the other chains disabled.

1. **Hands:** start with an open target, then shallow individual finger movements. Observe current mapping, tracking speed, unintended adjacent-finger contact, and live diagnostics. Stop if warning `3` recurs; retain its exact joint, timestamp, and fresh diagnostics. Do not convert a firmware warning into a pass simply because it auto-clears. The current local limits are 1.0 rad/s and 0.8 A with `kp=5.0`, `kd=0.1`; sustained following at these settings has not passed.
2. **Arms:** the current Enable Arms path moves to the retained Home and then requests joint impedance. The new two-batch mode sequence needs its first attended hardware check. Retain snapshots before parameters, after the parameter wait, after state/subtype staging, and after stop/fault. Check both state `3` and the intended impedance subtype, rather than state alone. If right error `6` returns, stop and retain cabinet logs; do not increase force thresholds, copy old tool dynamics, or repeatedly clear and retry.
3. **Tracker following:** after successful arm initialization, move one wrist slightly in each direction, then rotate it, with the other side stationary. Check corresponding-arm direction and continuity, elbow behavior, reachable targets, and that disabling returns both arms to fresh confirmed standby. The initial Home movement is not a Tracker-following result.

Record successful command acceptance separately from resulting measured movement. On a stop, retain advancing feedback until the actual terminal state is confirmed. If communication prevents confirmation, use the physical stop and record the state as unconfirmed rather than treating process exit as proof.

## 4. Integrated acceptance record

Only after the separate stages pass, enable both hands and both arms for an attended small-motion window. Record the window duration, software commit, configuration hashes, source freshness, target and measured movement, warnings/errors, the operator's direction assessment, and the final fresh disabled/standby states. Explicitly check simultaneous movement and left/right independence. Label all failures and interrupted windows as such.

Do not reuse historical JSON filenames. Raw MANUS SDK logs can contain licensing information; keep them private and archive only filtered status summaries. The [acceptance record](HARDWARE_ACCEPTANCE_2026-10-19.md) contains previous failures and the pytest collection incident. The [encoder diagnostic](HAND2_ENCODER_DIAGNOSTIC.md) describes the right-hand evidence for follow-up. No unattended motor test is authorized by this runbook.
