# Right Hand 2 Encoder 1 warning: support evidence

**Local support draft, 2026-10-19. Not sent to the vendor. Sustained motor following remains unaccepted.** The latest intended MANUS-following test stopped on right middle-finger PIP error `0x0003`. Later disabled observations found no active error, but did not explain the warning or establish reliable encoder behavior during motion. This report preserves that distinction and the existing fault guard.

The complete operational chronology, including an intervening unintended legacy-test execution and its cleanup, is in [the hardware acceptance record](HARDWARE_ACCEPTANCE_2026-10-19.md). The later read-only records below are separate observations, not an uninterrupted recording from the original warning. Times are Asia/Shanghai (UTC+08:00) unless marked UTC.

## Device and software identity

| Item | Recorded value |
| --- | --- |
| Target | Right Wuji Hand 2, serial `WH2KA01260823036` |
| Firmware | `2.5.1` |
| Hardware version reported by SDK | `0.2.0`; no marketing-generation mapping is inferred from this value |
| Python SDK | `wuji-sdk==2026.10.12` |
| Official SDK reference | [Tag v2026.10.12, commit b71a55a55b002e4dc4178f38156f65686fdb3156](https://github.com/wuji-technology/wuji-sdk/tree/b71a55a55b002e4dc4178f38156f65686fdb3156) |
| Host OS | Ubuntu 22.04.5 LTS |
| Affected joint | Diagnostic `nid=13`; SDK zero-based joint index `10`; `middle/S3`; ROS/model `r_middle_finger_pip` |
| Other hand in the failed test | Left serial `WH2JA01260820046`, firmware `2.5.1` |
| Input configuration for the latest failure | Exact operator-supplied TPX calibration profiles; explicit MANUS right-handed coordinates; `controller.flip_y=false` |

The affected-joint mapping uses `WujiHand2.nid_to_joint_index(13)` and the explicit `nid` field in joint diagnostics. NIDs contain gaps between fingers; `nid - 1` is not the command index. The [migration document](HAND2_MIGRATION.md#input-and-joint-mapping) records the full mapping and pinned model.

## Observed failures and stop behavior

These are distinct events; a shared cause has not been established.

| Event | Recorded result | Interpretation boundary |
| --- | --- | --- |
| Earlier MANUS following | Right code `7` (`Stall`); peak right current approximately `1.029 A`. The operator subsequently reported right ring/little-finger contact. | Contact is an operator observation relevant to that run. It does not explain the later middle-finger encoder warning. Evidence: `work/hand2-live-retarget-summary.json`. |
| Latest intended TPX following | Right `nid=13`, code `3` (`Enc1BitRate`); requested 40-second shallow-gesture window aborted early. Both hands reached 20 enabled joints. | No completed following window or all-finger tracking acceptance. Evidence: `work/hand2-tpx-retarget-summary.json`. |

The latest attempt used `max_velocity=1.0 rad/s`, `effort_limit=0.8 A`, `kp=5.0`, and `kd=0.1`. Peak measured currents were `0.2875 A` left and `0.3373 A` right. The operator reported no visible finger contact, cable pulling, or obstruction during this attempt. That observation does not identify or exclude an internal encoder problem. No arm commands were issued; the physical arm emergency stop remained engaged.

The bridge deliberately treats every nonzero joint diagnostic code as a latched fault and requests disable. Its terminal snapshots showed 20 online and zero enabled on each hand, with feedback/diagnostics approximately 2.4–2.5 ms old. Left lifecycle was `STOPPED`; right remained `FAULT`. The collector reports `completed_window=false` and `disabled_confirmed=false`: its latter criterion additionally requires both lifecycles to be `STOPPED`. Preserve those values. Fresh zero-enabled feedback is a scoped observation, not grounds to relabel the failed run as a pass. Motion ranges and counters include the stop-observation interval and do not measure successful gesture tracking.

## Official error semantics and their limits

The pinned SDK's static `WujiHand2.describe_error(3)` returns `Enc1BitRate`, severity `Warning`, clear policy `AutoClear`, and `is_test=false`. Its description identifies an elevated proportion of Encoder 1 frames carrying the encoder chip's data-quality flag. Its suggested investigation concerns the encoder and magnet placement/alignment. The pinned SDK identifies Encoder 1 as the motor-side encoder. This is an encoder data-quality indication, not an Ethernet bitrate measurement.

Code `7` decodes separately as `Stall`, also `Warning` / `AutoClear`. Always decode the device fault code through the SDK; calibration-result error enums are a different namespace. See the official [diagnostics reference](https://docs.wuji.tech/docs/zh/wuji-hand/latest/sdk-reference/diagnostics) and [data reference](https://docs.wuji.tech/docs/en/wuji-sdk/latest/data-reference).

Official [maintenance documentation](https://docs.wuji.tech/docs/zh/wuji-hand/latest/maintenance) distinguishes firmware warning behavior from stop-class faults: the firmware does not itself stop for a warning, and an automatically cleared warning can disappear when its triggering condition subsides. This describes firmware behavior. It does **not** certify encoder data quality for this unit's closed-loop motion or require this integration to continue through an encoder warning. The official [troubleshooting guidance](https://docs.wuji.tech/docs/zh/wuji-hand/latest/troubleshooting) calls for investigating the indicated cause and escalating recurring problems. No public criterion inspected establishes that this specific `Enc1BitRate` occurrence can safely be ignored.

The pinned [real-hand retargeting example](https://github.com/wuji-technology/wuji-sdk/blob/b71a55a55b002e4dc4178f38156f65686fdb3156/examples/python/retargeting/1.teleop_real.py) and [publishing example](https://github.com/wuji-technology/wuji-sdk/blob/b71a55a55b002e4dc4178f38156f65686fdb3156/examples/python/wuji_hand_2/2.publish.py) demonstrate device operation; their absence of this application's warning-latching policy is not evidence that continuing through an encoder-quality warning is safe. The integration's guard has not been weakened. No joint offsets, limit changes, encoder recalibration, or firmware update were introduced to pass this test.

Error names and decoded meanings above are tied to the installed SDK version. The documentation URLs use the vendor's moving `latest` pages and are not a firmware `2.5.1` compatibility guarantee.

## Subsequent read-only evidence

A separate immediate post-stop five-second observation recorded 467 left / 466 right diagnostic frames, zero enabled joints, no active errors, and all joint extended states `1` (`Ready`). Final state and diagnostic ages were below 1 ms. This shows the warning was absent during that disabled sample, not that it would remain absent during motion. Evidence: `work/hand2-post-tpx-fault-readonly.json`.

The later right-only collection began at **22:52:29**, UTC `2026-10-19T14:52:29.017630+00:00`, using exact serial-number connection. Its elapsed session was approximately 7.75 seconds including setup, with a five-second observation after setup. Before resource reads, the collector required at least two distinct state and diagnostic sequences, both streams younger than 0.25 seconds, exactly the 20 expected joint NIDs, and every joint in a disabled state (`Ready` or `Stopped`). Callbacks continued checking for any enabled joint throughout the session. No enabled joint was observed.

The collector used public SDK connection, subscriptions, resource enumeration, and GET interfaces only. It issued no enable, disable, error clear, command publication, parameter setter, calibration, or firmware action. Normal SDK connection housekeeping remained in place, including the initial SDK time synchronization; setting `auto_time_sync_interval_ms=None` suppresses subsequent periodic synchronization, not the first synchronization during connection. All owned subscriptions and the device connection were closed.

| Observation | Result |
| --- | --- |
| Received frames | 5,017 joint diagnostic frames; 5,016 joint state frames; sequence values distinct within each stream |
| Diagnostic completeness/state | All frames contained all 20 expected joints; all were `Ready`; maximum enabled count `0` |
| Active joint errors | None in the observed diagnostic frames |
| Direct SDK joint index 10 reads | Initial/final error code `0`; status `Ready`; position/velocity/current limit flags false |
| Exact `nid=13` current | `-0.03357` to `+0.02589 A` |
| Exact `nid=13` supply voltage | `12.4054` to `12.4516 V` |
| Exact `nid=13` MCU temperature | `54.8668` to `55.7027 °C`; no claim of an encoder temperature or a vendor pass threshold |
| Exact `nid=13` diagnostic bus summary | `comm_response_rate_pct=100`, `comm_timeout_total=0` throughout; 5,017 snapshots with nonsentinel communication-summary age |
| Middle-finger aggregate bus counters | Initial-to-final increments: CRC `0`, frame-format `0`, UART hardware errors `0`; TX frames `18,579`, RX frames `18,538`; initial/final `error_per_sec=0` |
| Enumerated resources | 346 total; 60 error-related paths across the 20 joints |
| Direct encoder-quality resource | None found in the advertised resource metadata inspected on this firmware |

**Identification boundary:** the `nid=13` figures above come from `JointDiagnosticsEntry.nid`. In contrast, `hand.comm_diag().get().fingers[2].nodes[2]` is only a generic array slot. Its `NodeDiagnostics` object has no `nid` field. That slot returned 100% responses and zero timeouts in the initial/final snapshots, but its mapping to `nid=13` has not been established and is not asserted here. The saved summary deliberately calls it `middle_finger_nodes_array_slot2_initial_final`.

These communication figures describe host/device or controller/joint-bus transport. They do not measure the internal encoder chip's data-quality flags and cannot refute `Enc1BitRate`. Aggregate TX/RX differences are not independently interpreted as packet loss. See the official [communication diagnostics documentation](https://docs.wuji.tech/docs/zh/wuji-hand/latest/sdk-reference/diagnostics).

## Advertised error resources and actual GET results

At **22:54:18**, a separate exact-serial, disabled-only session re-enumerated the actual device resources before reading the following paths. Each advertised `can_get=true` and `can_set=can_sub=can_pub=can_exec=false`.

| Actual resource path | Returned value |
| --- | --- |
| `middle/S3/error_status` | `warning=0`, `deferred_stop=0`, `immediate_stop=0`, `fatal=0` |
| `middle/S3/error_log` | `entries=[]` |
| `middle/S3/error_code` | `0` |

This session recorded 19 diagnostic / 13 state frames, maximum enabled count zero, and a final complete set of 20 `Ready` joints with error zero. Both sessions closed successfully. The error-status fields contain active device fault codes by severity, not error counts. Generic `get(path)` exposes these status/history resources even though a dedicated typed joint getter is not provided for each of them.

An empty history response does **not** disprove the previously captured code `3`. Warning inclusion, retention, reset behavior, and firmware support for this history need vendor confirmation. These later sessions do not establish continuous uptime or uninterrupted error-history retention since the failure. No raw Encoder 1 quality rate, chip status, or signal-strength getter was found among the advertised resources; no undocumented paths were probed.

The earlier read-only flash-log export saved 1,208 records. The filtered history contained bus parser retry messages, but no identified explanation of the Encoder 1 warning. No causal link between those messages and code `3` has been established. The export is separate from the per-joint `error_log` response.

## Reproducing the read operations

The static catalog can be decoded without creating a manager or connecting a device:

```python
import wuji_sdk as sdk

for code in (3, 7):
    print(sdk.WujiHand2.describe_error(code))
assert sdk.WujiHand2.nid_to_joint_index(13) == 10
```

The following is the generic GET portion of the collection. It accepts an already connected handle and a **required live guard**; it is not a standalone unattended probe. The caller must verify the exact serial/type, maintain the fresh complete disabled-state checks described above, and stop the read session if any joint becomes enabled. `require_fresh_disabled()` must raise on missing/stale/incomplete feedback or any state outside `Ready`/`Stopped`, before and after each RPC. Its implementation must read the monotonic clock after taking a coherent callback snapshot.

```python
def read_target_error_resources(hand, require_fresh_disabled):
    import wuji_sdk as sdk

    if not isinstance(hand, sdk.WujiHand2):
        raise RuntimeError("Unexpected device type")
    if hand.serial_number != "WH2KA01260823036":
        raise RuntimeError("Unexpected serial number")

    require_fresh_disabled()
    resources = {resource.path: resource for resource in hand.resources()}
    require_fresh_disabled()
    results = {}
    # These paths were observed on this device; do not guess alternatives.
    for path in (
        "middle/S3/error_status",
        "middle/S3/error_log",
        "middle/S3/error_code",
    ):
        resource = resources.get(path)
        if resource is None or not resource.can_get:
            raise RuntimeError(f"GET resource unavailable: {path}")
        if any((resource.can_set, resource.can_sub,
                resource.can_pub, resource.can_exec)):
            raise RuntimeError(f"Resource metadata changed: {path}")
        require_fresh_disabled()
        results[path] = hand.get(path)
        require_fresh_disabled()
    return results  # ErrorStatus, ErrorLog, and integer respectively.
```

The collection scripts retained locally are `work/collect_hand2_right_encoder_readonly.py` and `work/collect_hand2_right_error_history_readonly.py`. They have main guards, exact-serial/version checks, subscription guards, and finally-block subscription closure/disconnection. They are evidence tooling outside the installed application, not pytest inputs or an acceptance runner. Do not run them concurrently with a controller or automatically rerun them while preparing a report. Preserve existing evidence filenames before any separately authorized repeat. Cleanup closes the read session; it must not add a disable/clear operation to this read-only procedure.

## Evidence package retained locally

| File relative to repository root | Contents |
| --- | --- |
| `work/hand2-tpx-retarget-summary.json` | Failed latest intended following attempt, fault reason, terminal states and counters |
| `work/hand2-post-tpx-fault-readonly.json` | Immediate disabled post-stop observation of both hands |
| `work/hand2-right-encoder-readonly.summary.json` | Compact later right-hand results and interpretation boundary |
| `work/hand2-right-encoder-readonly.json` | Full serialized diagnostic frames, sampled state frames, resource metadata, direct GET/communication results, and follow-up error-resource evidence |
| `work/hand2-right-encoder-readonly.json.gz` | Compressed copy of the preceding full record |
| `work/hand2-right-encoder-error-history-readonly.json` | Follow-up resource capabilities, returned error values, and disabled-state evidence |
| `work/right-hand-flash-export.json` | Metadata for the separate 1,208-record flash-log export |
| `work/hand2-flash-logs/flash_WH2KA01260823036_2026-10-19_223023.jsonl` | Exported device history; inspect locally before including any excerpt in a support package |

The full uncompressed right-hand record is **56,832,183 bytes**, SHA-256 `afabf7ede79f44af5a07a1092db35c90278af9db558e3101c763e209755be406`. Raw SDK console logs are kept separately and are not reproduced here. Files under `work/` are local, ignored evidence; they are not installed runtime dependencies or automatically included with this tracked report. No evidence has been sent externally as part of preparing this document.

## Questions for Wuji support

1. For serial `WH2KA01260823036`, hardware `0.2.0`, firmware `2.5.1`, what precise chip flags, sample window, and thresholds trigger `Enc1BitRate`? Does enabled/disabled state change the sampling or detection behavior?
2. Is there a supported **read-only** API or diagnostic tool for joint `nid=13` that exposes Encoder 1 raw quality flags, flag rate, signal/magnet-quality values, or a warning transition trace? None was found in the advertised public resources. Please specify the exact supported interface and firmware requirement.
3. Does firmware `2.5.1` retain `Warning` / `AutoClear` transitions in `middle/S3/error_log`? What resets or evicts that history, and is an empty response after a captured code `3` expected? Are error status/history resources fully supported in this firmware/SDK combination?
4. Is `wuji-sdk==2026.10.12` supported with firmware `2.5.1` for joint diagnostics and error decoding? Are there relevant encoder-quality detection errata or firmware fixes? Please identify the applicable release and reason before proposing any firmware change.
5. What evidence distinguishes a transient acceptable flag from an encoder condition requiring service before closed-loop motion? Firmware `Warning` semantics alone do not supply an acceptance criterion for this unit. Please provide the recommended investigation and motion-resumption criteria.
6. Which inspections are appropriate with the device disabled and powered off, and which require factory/service work? The catalog mentions encoder/magnet positioning; no internal adjustment, recalibration, or disassembly has been attempted for this investigation.
7. What is the documented ordering of `FingerCommunicationDiagnostics.nodes` on this firmware, including any tactile node? The array lacks explicit NIDs, so its slot statistics have intentionally not been mapped to `nid=13`.

Pending those answers, the evidence supports fresh, disabled communication and no active warning during the sampled intervals. It does not establish the warning's cause, safe motion through the warning, sustained all-finger following, or full-system acceptance. Keep the nonzero-error latch and explicit recovery requirement; do not change protection, calibration, current, or joint limits merely to obtain a passing run.
