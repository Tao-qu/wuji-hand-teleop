# Hardware acceptance record — 2026-10-19

**Status: partial validation; integrated two-hand/two-arm teleoperation has not passed acceptance.** This is an interim record for the MANUS + five Trackers + Tianji + Wuji Hand 2 installation. Successful input reception, a bounded finger test, and an initial arm position move are separate results; none establishes complete system acceptance.

At the arm stop point, all software control processes had exited. The operator confirmed that the physical emergency stop was pressed and that both arms had remained stationary, then restarted the cabinet while retaining the emergency stop. The latest read-only arm probe confirmed advancing feedback from both arms, each in state `100`, error `13` (emergency stop). A subsequent hand-only check also kept both hands disabled. Standby after releasing the emergency stop, error clearing, and further arm motion have not been tested; the earlier right-arm mode-transition and recovery failures remain unresolved.

Times below use Asia/Shanghai (UTC+08:00) unless explicitly marked UTC or controller-local. Results are taken from filtered JSON summaries, the operator's session observations, and the limited controller-log excerpt identified below. The controller clock differs from the host clock; its timestamps are not treated as synchronized absolute times. Raw MANUS SDK logs are excluded because they may contain licensing information. Files under `work/` are local evidence, not installed runtime dependencies; retain the filtered summaries with any archived test package.

## Configuration and evidence boundary

| Item | Recorded configuration |
| --- | --- |
| Left hand | `WH2JA01260820046`, firmware `2.5.1` |
| Right hand | `WH2KA01260823036`, firmware `2.5.1` |
| Hand SDK | `wuji-sdk==2026.10.12` |
| Hand model | Official `wuji_hand2_2_description`, pinned revision `829c9aa27cb12c3a49d5b8d63c9869dd8f656702` |
| Tianji endpoint | `6.6.7.190`; `6.6.7.100` is the adjacent computer, not the verified cabinet endpoint |
| Tianji controller version | `100343009`, read with the SDK VERSION request |
| Selected local control SDK | Vendor SDK `100343014`, selected through ignored `tools/env.local.sh` / `TIANJI_SDK_LIBRARY` |
| Kinematics | Bundled `libKine.so` and `ccs_m6.MvKDCfg`, robot type `1017` |
| Payload | `tool_dynamics.source: controller`; existing cabinet values preserved, new hand assemblies **not calibrated** |
| IK tool geometry | No local `tool_transforms` configured; identity means that IK targets the arm flange |

The retained home joint targets are, in degrees and SDK order:

```text
left:  [ 55, -65, -70, -60,  60, 0, 0]
right: [-55, -65,  70, -60, -60, 0, 0]
```

The operator withdrew the proposed home change. These targets remain in the project configuration. This decision does not certify hand mounting orientation, clearance, payload, or every path into and out of home.

At the operator's request, mass/centre-of-mass identification and flange-to-hand mounting calibration are deferred and excluded from the current functional teleoperation acceptance scope. This record does not introduce those deferred activities as a prerequisite for that scope. Existing cabinet payload values are preserved. The present arm blocker is the observed torque-mode error `6` and subsequent IDLE/ERROR loop; absence of calibration alone has not been established as their cause.

## Stage results

| Stage | Result within the observed scope |
| --- | --- |
| Offline build and tests | Earlier, all 10 packages built successfully in 24.2 seconds; subsequent runs passed 118 full-suite tests, 54 Tianji safety tests, and later 144 full-suite tests. A TPX selected build completed 2 packages in 18.6 seconds. After the final CLI cancellation and publisher entry-timestamp fixes, the final 2-package build passed in 19.0 seconds and all four test directories passed **154 tests in 7.62 seconds**. These are separate runs, not additive counts, and do not establish hardware behavior. |
| MANUS acquisition | Both sides produced advancing source sequences, 25 raw nodes, and low source age in the initial sample. |
| Five-Tracker input | Earlier samples failed because the chest frame was missing and some input ages were excessive. A later five-second sample contained all five fresh dynamic frames. |
| Disabled hand pipeline | Both hands reported all 20 joints online and zero enabled while finite, correctly sized hand commands were generated. |
| Corrected MANUS coordinate convention | A later 25-second disabled sample and operator-labelled open/half-closed comparison support `flip_y=false` with explicit right-handed SDK coordinates. Hardware following after this correction remains untested. |
| MANUS metadata and calibration tooling | Node metadata now resolves by node ID with malformed hierarchy/raw-node rejection. The metadata repair, calibration CLI, Ctrl-C cancellation fix, and publisher entry-timestamp race fix are complete and covered by the final software verification. No new calibration was performed because the CLI session was cancelled before Start. |
| TPX-profile disabled pipeline | The latest hand-only check recorded 3,845 left / 3,830 right state frames and 1,409 / 1,598 commands, with zero invalid commands, zero enabled joints, all 20 joints online per hand, and final `STOPPED` feedback below 4 ms. Both earlier TPX loads and the rebuilt standalone publisher's default loads succeeded; installed profile hashes match the source files. |
| TPX pose geometry | A new naturally straight open sample substantially reduced the observed right little-finger bend. Offline median-target inspection found zero contacts in both open targets, but 14 left / 20 right contacts in the half-labelled targets. This is a geometry diagnostic, not an enabled-following or collision-free-motion pass. |
| Small hand commands | First 0.02-rad commands did not produce meaningful motion. Separate 0.05-rad tests produced measurable index MCP flexion on each hand and confirmed the command watchdog's stop path. |
| MANUS-driven hand motion | An earlier run aborted on right-hand code `7` (stall). A later TPX run with the corrected mapping also aborted, on right middle-finger PIP code `3` (encoder data-quality warning). Post-stop read-only feedback confirmed both hands disabled and no active warnings; sustained following remains unaccepted. |
| Arm native FK/IK | A native structure-overrun defect was reproduced and repaired. Offline canary tests and a later fresh TF-to-IK sample passed within their stated scope. |
| Arm initial position move | Motion from the earlier `tianji_tele` Home posture toward the retained project home occurred. This was an initialization move, not Tracker teleoperation. |
| Integrated acceptance window | **Did not complete.** The arm enable service timed out; the right arm ended in state `100`, error `6`; the right hand also lost communication. |
| Subsequent right-arm recovery | Error clear and standby were requested without another enable request. The cabinet log repeatedly returned from IDLE to ERROR because some servos remained enabled. Software standby was **not confirmed**. All software control processes exited; the operator subsequently confirmed physical emergency-stop actuation and stationary arms. |
| Cabinet restart with emergency stop retained | A 12-sample read-only probe recovered advancing feedback on both arms; all samples reported state `100`, error `13`. The initial baseline was followed by 11 fresh samples. No emergency-stop release, error clear, or new motion was performed in this probe. |

Commit `3071577` (`feat(manus): load TPX profiles and validate glove calibration`) installs the operator-supplied TPX files, validates calibration and metadata, and adds the single-glove calibration tool. The coordinate/handedness correction and `flip_y=false` default are recorded in commit `1db611d` (`fix(manus): preserve handedness through Hand 2 retargeting`). Commit `8ed9d45` (`fix(tianji): make arm initialization cancellable and verify settling`) adds cancellable initialization and checks that the home move settles. The offline build and test results above validate the software within their test coverage; they do not convert the earlier failed hardware attempt into a pass. The earlier focused hand/input/native-kinematics run passed 27 tests. Seven independent metadata-helper regression tests subsequently passed using a temporary C++ executable and SDK type headers, without linking or connecting to the SDK. They cover sparse, one-based and reordered IDs plus malformed hierarchy/raw-node rejection. The final software run after the metadata, CLI cancellation, and publisher entry-timestamp fixes passed 154 tests across all four test directories in 7.62 seconds; evidence: `work/pytest-manus-final.log`. Corrected enabled following remains outside the completed verification at this report update.

The subsequent staged arm-command checks are committed in `61cf8cc` (`fix(tianji): reject unaccepted arm mode commands`). The complete four-directory suite then passed **212 tests in 6.17 seconds**, and `tianji_output` built in 3.89 seconds. Evidence: `work/pytest-post-mode-check.log`, `work/build-tianji-return-check.log`. This later run supersedes the earlier test count as the current offline result. No arm commands were sent to validate that change at this point.

## Input acquisition

The initial MANUS sample recorded 1,617 right-hand frames and 1,307 left-hand frames, each containing 25 raw nodes. Maximum measured source ages were 18.65 ms and 16.22 ms respectively; neither side had a non-increasing sequence. Source sequence gaps are possible because the publisher samples the latest SDK frame; they do not alone prove transport loss. Evidence: `work/manus-live-summary.json`.

The first five-second worn-Tracker summary had 588 frames for each wrist/upper-arm Tracker and **zero chest frames**. The later full-input sample also recorded zero chest frames, stale MANUS frames, and Tracker source ages beyond the 200 ms criterion. Those samples failed freshness and are retained as failed observations, not overwritten by a later success. Evidence: `work/worn-tracker-summary.json`, `work/full-input-summary.json`.

The later sample beginning at 21:44:24 lasted 5.000 seconds and passed its 200 ms freshness criterion:

| Stream | Received frames | Maximum source age |
| --- | ---: | ---: |
| MANUS left | 445 | 76.92 ms |
| MANUS right | 481 | 70.15 ms |
| Chest Tracker | 522 | 193.24 ms |
| Left wrist Tracker | 460 | 190.64 ms |
| Right wrist Tracker | 479 | 193.20 ms |
| Left upper-arm Tracker | 435 | 190.63 ms |
| Right upper-arm Tracker | 428 | 184.57 ms |

No repeated/reordered stamps or stale-source frames were recorded in that later window. The Tracker latency was close to the threshold; a five-second pass is not evidence of sustained timing margin. Evidence: `work/full-input-tuned-summary.json`.

## Hand identity, disable state, and bounded motion

The disabled-pipeline sample collected 3,924 left and 3,901 right state messages, plus 207 diagnostic messages per side. It observed 1,621 left and 1,767 right commands with no invalid commands. Both final diagnostic snapshots had all 20 joints online, zero enabled, and state/diagnostic ages below 5 ms. Evidence: `work/hand2-disabled-summary.json`. These are confirmations for that sample, not a continuing guarantee after later disconnects.

Separate bounded tests commanded SDK joint index 4, index-finger MCP flexion. All tests used 0.1 rad/s maximum command velocity:

| Hand / trial | Requested displacement | Current limit | Largest selected-joint displacement | Largest other-joint displacement | Watchdog-to-confirmed-disable |
| --- | ---: | ---: | ---: | ---: | ---: |
| Left, first | 0.02 rad | 0.4 A | 0.000016 rad | 0.000023 rad | 294.5 ms |
| Right, first | 0.02 rad | 0.4 A | 0.000016 rad | 0.000026 rad | 281.4 ms |
| Left, second | 0.05 rad | 0.6 A | 0.03490 rad | 0.01023 rad | 286.4 ms |
| Right, second | 0.05 rad | 0.6 A | 0.03630 rad | 0.00629 rad | 302.2 ms |

The first trials demonstrate command/enable/watchdog operation but **not meaningful motion**. The second trials demonstrate bounded movement of the selected joint, with residual and some coupled motion. Peak recorded currents in the second trials were 0.293 A left and 0.213 A right. All four final snapshots had zero enabled joints and all 20 online with fresh feedback. No nonzero error code was recorded in these bounded trials. One left-trial lifecycle history also included `DISCONNECTED`; this should not be described as an uninterrupted run.

Evidence: `work/hand2-motion-summary.json`, `work/hand2-motion-second-summary.json`, `work/hand2-motion-right-second-summary.json`. This is not an all-joint, grasp-force, collision, or complete retargeting acceptance test.

The subsequent MANUS-following attempt collected 1,664 left / 1,651 right state frames and 832 left / 1,016 right commands. Substantial movement occurred on both hands. Peak recorded currents were 0.435 A left and 1.029 A right. The right hand reported joint error `7`, entered `FAULT`, and the requested window ended early. Final snapshots in this particular attempt were fresh, all 20 joints were online, and enabled counts were zero. Error `7` is recorded as observed; its cause is not inferred from these summaries. Evidence: `work/hand2-live-retarget-summary.json`.

The local configuration was subsequently changed to 1.0 rad/s, 0.8 A, `kp=5.0`, `kd=0.1`. These values appear as intended settings in the integrated attempt's metadata. **High-speed hand following with these settings has not been validated on hardware.** Metadata alone does not prove execution or tracking performance.

## Subsequent MANUS coordinate correction — disabled validation

The MANUS publisher was subsequently corrected to request an explicit right-handed SDK coordinate convention. Earlier operator confirmation of hand direction used the previous MANUS binary and does not establish direction correctness after this change. The original motion observations above remain historical observations, not acceptance of the corrected mapping.

A later 25-second disabled sample recorded 3,744 left / 3,634 right state frames and 1,470 left / 1,624 right commands, with zero invalid or out-of-bounds commands. Both hands stayed at zero enabled joints; final diagnostics reported `STOPPED`, all 20 joints online, and state/diagnostic ages below 3 ms. Its original summary path was `work/hand2-coordinates-disabled-summary.json`; a subsequent TPX run reused that path, so the current file must not be cited as the earlier sample's contents. The earlier counts are retained here as recorded observations. That fresh snapshot independently confirmed disable at that time, but does not make the earlier stale integrated-run confirmation valid.

The operator then labelled open and half-closed glove poses for a same-input comparison of the two reflection settings. With `flip_y=false`, index/middle/ring/pinky MCP flexion increased by `[1.0448, 1.3184, 1.1880, 1.1362]` rad on the left and `[1.1226, 1.2502, 1.1431, 1.2942]` rad on the right. With `flip_y=true`, these MCP changes were negative, while many PIP/DIP outputs reached their upper limits. This supports removing the extra reflection for the corrected right-handed input; the default has been updated to `flip_y=false`. Evidence: `work/coordinate-ab-open.json`, `work/coordinate-ab-half.json`.

The comparison establishes the observed direction of four-finger flexion in those labelled poses. It does not establish full thumb opposition, abduction, comfortable range, or actuator tracking. Some unreflected outputs also approached joint limits, so range calibration still requires attention. No enabled hand-following pass is claimed for the corrected mapping.

The operator explicitly confirmed that the right little finger was physically straight in the earlier labelled open pose. The recorded MANUS geometry nevertheless contained approximately 66.92 degrees of right little-finger PIP bending. The subsequent TPX open-pose result below records the later improvement without rewriting that earlier observation. Separately, code inspection found a MANUS metadata path using a node ID as an array index, which could access outside the array bounds. The repair now uses a per-glove node-ID map and rejects duplicate or missing metadata/raw IDs. Live hierarchy inspection found 25 nodes, including node ID `25` for the thumb tip with parent ID `24`; using ID `25` as an offset into a 25-element array was invalid. Little-finger IDs were `18` proximal, `19` intermediate, `20` distal, and `21` tip. The metadata defect is confirmed, but has not been established as the cause of the apparent little-finger bend. The earlier pose samples are not a post-repair validation.

### TPX profile provenance and latest disabled check

At the operator's request, `/home/taoqu/Downloads/tpxLeftMetaglovePro.mcal` and `/home/taoqu/Downloads/tpxRightMetaglovePro.mcal` were copied into `src/input_devices/manus_input/manus_ros2/calibration/` with their filenames unchanged and hashes preserved. The publisher defaults now use those exact names, and the local YAML explicitly selects those source-package paths. The previous tracked Left/Right profiles were removed, with backups retained locally outside version control. The TPX files' recorded sizes and SHA-256 values are:

| Profile | Size (bytes) | SHA-256 |
| --- | ---: | --- |
| `tpxLeftMetaglovePro.mcal` | 817,801 | `bdc67e3ed44507a76b9327a5b7deb959566d64739f229741282c2d12370c0980` |
| `tpxRightMetaglovePro.mcal` | 750,285 | `bf2c2eb9027a1fb8cf84a0b2168eb1178a3103b68beb1056b20729ac2c95af7b` |

The Downloads files match the historical TPX profiles at `dex` revision `e991c8f` and the earlier ignored local TPX copies. The previously bundled profiles were LCJ profiles with different bytes. This establishes the profile difference and provenance; it does not establish that the LCJ profiles caused the right little-finger discrepancy. The calibration CLI session was cancelled **before Start**. No new calibration steps or Finish operation were performed.

The latest TPX disabled check recorded the following filtered results:

| Side | State messages | Commands | Diagnostic messages | Invalid commands | Maximum enabled joints | Final online joints / lifecycle |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| Left | 3,845 | 1,409 | 208 | 0 | 0 | 20 / `STOPPED` |
| Right | 3,830 | 1,598 | 208 | 0 | 0 | 20 / `STOPPED` |

Final feedback ages were 3.55 ms left and 2.35 ms right; diagnostic ages were also below 4 ms. These observations confirm a disabled command pipeline during that check. The summary is preserved separately as `work/hand2-tpx-disabled-summary.json`; the same data also occupied `work/hand2-coordinates-disabled-summary.json` after that path was reused.

Filtered SDK status lines explicitly confirm successful calibration loading for both the left and right gloves in the earlier TPX run. The subsequent 2-package selected build completed in 18.6 seconds, both installed profiles matched the recorded source hashes, and the standalone publisher successfully loaded both profiles using its new defaults. The final cancellation/timestamp fixes were then rebuilt successfully in 19.0 seconds and included in the 154-test pass. Thus, successful loading is directly observed and does not rely only on command flow. Raw SDK logs are not reproduced here. No new calibration steps, corrected enabled-following pass, or completed system acceptance is claimed.

### TPX open/half-labelled poses and offline target contact audit

After the operator confirmed readiness with naturally straight, open hands, the new open sample recorded 774 left / 753 right frames with maximum source ages 61.19 / 40.82 ms. The subsequent half-labelled sample recorded 744 left / 800 right frames with maximum ages 56.21 / 40.63 ms. Evidence: `work/coordinate-ab-tpx-open.json` and `work/coordinate-ab-tpx-half.json`.

For `flip_y=false`, the new open sample's right little-finger raw PIP segment angle was 8.795847 degrees and its target PIP was 9.780390 degrees, compared with approximately 66.918 / 64.56 degrees in the earlier LCJ-profile observation. The new left values were 9.707849 / 3.678389 degrees. This is a substantial observed improvement after the profile/metadata changes; the two sessions are not an isolated experiment proving which individual change caused it.

All four fingers' target flexion increased between the open and half-labelled medians. Values below are half minus open, in radians, for the active `flip_y=false` setting:

| Side / finger | MCP flexion change | PIP change | DIP change |
| --- | ---: | ---: | ---: |
| Left index | 0.7516 | 2.0628 | 0.6280 |
| Left middle | 1.0757 | 1.8427 | 0.7147 |
| Left ring | 1.1296 | 1.9438 | 0.6057 |
| Left little | 1.1368 | 2.0298 | 0.2926 |
| Right index | 0.7471 | 2.0355 | 0.4882 |
| Right middle | 1.0136 | 1.9092 | 0.7599 |
| Right ring | 1.0619 | 1.8467 | 0.5942 |
| Right little | 0.9860 | 1.9233 | 0.2954 |

The operator subsequently clarified that the pose saved under the label “half” was **deeply curled, with fingertips near the palm**, not a slight bend. Its raw little-finger PIP angles were 125.185100 degrees left and 120.041293 degrees right, while both target PIP angles were 119.977362 degrees, close to the 120-degree limit. Those large angles are consistent with the clarified observation; the filename must not be interpreted as evidence that slight bending produced near-limit flexion.

The offline audit assigned these median targets by joint name to the official Hand 2.2 MJCF at revision `829c9aa27cb12c3a49d5b8d63c9869dd8f656702` and ran `mj_forward` only. It sent no device commands and did not run dynamics. Results for `flip_y=false` were:

| Target | All detected contacts | Ring/little contacts | Maximum penetration, all contacts | Maximum ring/little penetration |
| --- | ---: | ---: | ---: | ---: |
| Left open | 0 | 0 | 0 mm | 0 mm |
| Right open | 0 | 0 | 0 mm | 0 mm |
| Left half | 14 | 3 | 5.168981 mm | 1.312043 mm |
| Right half | 20 | 7 | 5.576695 mm | 2.977171 mm |

All 34 half-target contact pairs are listed below. Values are positive penetration depths; the raw signed distances and original precision are preserved in `work/hand2_tpx_target_contact_audit.json` under `all_contacts` and `ring_pinky_contacts`.

| Target | First model body | Second model body | Penetration (mm) |
| --- | --- | --- | ---: |
| Left half | `l_thumb_proximal_abd` | `l_index_finger_distal` | 0.420015 |
| Left half | `l_thumb_proximal_abd` | `l_index_finger_tip_sensor_frame` | 1.551689 |
| Left half | `l_thumb_middle` | `l_index_finger_distal` | 2.144145 |
| Left half | `l_thumb_middle` | `l_index_finger_tip_sensor_frame` | 4.916922 |
| Left half | `l_thumb_distal` | `l_index_finger_distal` | 0.774749 |
| Left half | `l_thumb_tip_sensor_frame` | `l_index_finger_middle` | 0.026748 |
| Left half | `l_index_finger_middle` | `l_middle_finger_proximal_abd` | 2.895462 |
| Left half | `l_index_finger_middle` | `l_middle_finger_middle` | 4.210398 |
| Left half | `l_index_finger_distal` | `l_middle_finger_proximal_abd` | 0.052772 |
| Left half | `l_index_finger_distal` | `l_middle_finger_middle` | 5.168981 |
| Left half | `l_index_finger_distal` | `l_middle_finger_distal` | 2.446047 |
| Left half | `l_ring_finger_distal` | `l_pinky_distal` | 1.312043 |
| Left half | `l_ring_finger_tip_sensor_frame` | `l_pinky_distal` | 0.627483 |
| Left half | `l_ring_finger_tip_sensor_frame` | `l_pinky_tip_sensor_frame` | 0.206449 |
| Right half | `r_index_finger_middle` | `r_middle_finger_middle` | 0.633988 |
| Right half | `r_index_finger_middle` | `r_middle_finger_distal` | 0.094868 |
| Right half | `r_index_finger_distal` | `r_middle_finger_middle` | 1.649096 |
| Right half | `r_index_finger_distal` | `r_middle_finger_distal` | 2.099827 |
| Right half | `r_middle_finger_proximal_abd` | `r_ring_finger_middle` | 1.588451 |
| Right half | `r_middle_finger_middle` | `r_ring_finger_proximal_abd` | 1.688002 |
| Right half | `r_middle_finger_middle` | `r_ring_finger_middle` | 3.421390 |
| Right half | `r_middle_finger_middle` | `r_ring_finger_distal` | 3.583976 |
| Right half | `r_middle_finger_distal` | `r_ring_finger_middle` | 4.410761 |
| Right half | `r_middle_finger_distal` | `r_ring_finger_distal` | 5.576695 |
| Right half | `r_middle_finger_distal` | `r_ring_finger_tip_sensor_frame` | 2.328000 |
| Right half | `r_middle_finger_tip_sensor_frame` | `r_ring_finger_distal` | 0.581388 |
| Right half | `r_middle_finger_tip_sensor_frame` | `r_ring_finger_tip_sensor_frame` | 1.106635 |
| Right half | `r_ring_finger_middle` | `r_pinky_middle` | 1.426995 |
| Right half | `r_ring_finger_middle` | `r_pinky_distal` | 0.963914 |
| Right half | `r_ring_finger_distal` | `r_pinky_middle` | 1.470431 |
| Right half | `r_ring_finger_distal` | `r_pinky_distal` | 2.977171 |
| Right half | `r_ring_finger_tip_sensor_frame` | `r_pinky_middle` | 1.171751 |
| Right half | `r_ring_finger_tip_sensor_frame` | `r_pinky_distal` | 2.709571 |
| Right half | `r_ring_finger_tip_sensor_frame` | `r_pinky_tip_sensor_frame` | 0.590379 |

These are contacts between the official model's approximate convex meshes at coordinate-wise median targets, not measured hardware positions, contact forces, or synchronized single-frame targets. Zero contacts in the open targets does not prove a collision-free transition. Conversely, the half-target penetrations identify a target-geometry concern but do not prove identical physical penetration. No runtime collision guard or enabled-following validation is established by this audit. The scratch audit now accepts a separate pose prefix and refuses to overwrite existing output, so the earlier profile evidence remains intact.

## TPX enabled-following attempt and encoder warning

After installing the exact TPX profiles and rebuilding the final publisher, a hands-only test requested a 40-second shallow-gesture window at `max_velocity=1.0 rad/s`, `effort_limit=0.8 A`, `kp=5.0`, and `kd=0.1`. Both hands reached an enabled state, but the window aborted early when the right middle-finger PIP (`nid=13`) reported code `0x0003`. The operator reported no visible finger contact, cable pulling, or obstruction. This observation does not establish the warning's internal cause. No arm commands were issued; the physical arm emergency stop remained engaged.

The pinned official SDK's `WujiHand2.describe_error(3)` identifies `Enc1BitRate`, severity `Warning`, with `AutoClear`: Encoder 1 frames have an elevated rate of the chip's data-quality warning flag. The vendor recommendation is to inspect the encoder and its magnet positioning, seating, and alignment. This is distinct from the earlier code `7` stall warning. The bridge deliberately latches any nonzero joint diagnostic code and requested disable; no warning bypass or automatic re-enable was introduced.

Evidence: `work/hand2-tpx-retarget-summary.json`. The attempt observed peak enabled counts of 20 per hand and peak currents of 0.2875 A left / 0.3373 A right. Its terminal snapshots were fresh, with all 20 joints online and zero enabled on each hand. Left lifecycle was `STOPPED`; right lifecycle remained `FAULT`. The collector's stricter `disabled_confirmed` field is **false** because it requires both lifecycles to be `STOPPED`; preserve that result rather than relabelling this failed test as a pass. Its state/command counters and motion ranges include the stop-observation interval and do not establish successful human-gesture tracking.

A separate five-second, read-only post-stop probe collected 467 left / 466 right diagnostic frames, with zero enabled joints and no nonzero joint errors throughout. Both hands' final joint extended states were all `1`; final feedback/diagnostic ages were below 1 ms. The warning had cleared in this disabled observation. This does not establish that it will remain absent under motion. Evidence: `work/hand2-post-tpx-fault-readonly.json`. Further enabled-following acceptance remains pending investigation of this encoder-quality warning. A read-only `export_flash_logs` call saved 1,208 device log records under `work/hand2-flash-logs/`, with export metadata in `work/right-hand-flash-export.json`. Filtering this history found bus parser retry messages but no record identifying the Encoder 1 warning's cause; these retries have not been causally linked to the warning.

## Tianji SDK and native kinematics corrections

Read-only readiness investigation found that `OnLinkTo` establishes the local connection but does not by itself establish advancing controller feedback. A read-only VERSION request is required. The Python binding also needed a 30-byte native parameter-name buffer and an LP64 `c_long` result buffer; a four-byte `c_int` was too small. After correction, both the bundled and the locally selected control libraries returned controller version `100343009` and advancing feedback. This is **not** evidence that the older library was incompatible with the controller protocol.

A separate read-only TF-to-IK probe crashed after its first native IK call. A guarded local FK-to-IK reproduction showed that `libKine.so` wrote bytes 456–471 outside the declared 456-byte `FX_InvKineSolvePara`. The binding omitted `m_Output_JntExdABS`. The corrected structure is 472 bytes, with this double at offset 344, the limit flag at 352, and the two limit vectors at 360 and 416. Native canary tests passed for both arms after correction, including FK-to-IK-to-FK consistency. This identified heap corruption in the IK binding; the observed TF crash was not proof of a SciPy or ROS message ABI defect. Running nodes must be restarted after changing these bindings.

The later dry IK summary contained fresh targets at 18.61 ms left and 22.37 ms right. Both native IK calls succeeded without workspace or joint-limit flags. The probe used existing ROS input and local kinematics; it sent no hardware motion commands. Evidence: `work/arm-dry-ik-summary.json`, `tests/test_tianji_kine_abi.py`. An isolated feasible target does not establish safe transitions or a collision-free path.

## Integrated attempt and unresolved failures

The first integrated attempt started around 21:45:59. Before initialization, measured joints were near the earlier `tianji_tele` Home posture: left `[90.16, -89.96, -90.14, -89.73, -0.08, 0.11, -0.13]` and right `[-90.08, -89.87, 89.85, -89.61, -0.13, 0.33, 0.02]` degrees.

The initial position movement toward the retained project home occurred. Last recorded right joints matched the configured home; the left's largest recorded residual was approximately 3.13 degrees. The collector saw only eight arm state messages per side and zero arm teleoperation command messages. Its enable request timed out, so neither initialization completion nor subsequent Tracker-following performance can be inferred from the position movement alone.

The final arm status reported left state `0`, error `0`; right state `100`, error `6`. The reported right-arm error means **the request to enter torque mode failed**. All seven reported servo error words were zero. This arm mode-transition failure remains a separate issue; it must not be attributed to the right hand's power interruption without evidence.

The right hand lost communication during the same attempt. The operator reported and reorganized its power cable afterward. That physical intervention is recorded as an observation; it does not retroactively prove stop completion or explain the arm mode-transition failure. The integrated run has not yet been repeated successfully after the cable adjustment.

The summary originally reported `hands_disabled_confirmed: true`, which was **invalid evidence for this attempt**: the collector accepted cached enabled counts without requiring fresh feedback. The record has been corrected to `hands_disabled_confirmed: false`, retaining the old result as `hands_disabled_confirmed_original_collector: true` and explaining the correction in `evidence_correction`. The final right-hand diagnostic snapshot was about 28.56 seconds old, in `FAULT`, with `disable failed: Device disconnected`. Cached `enabled_joint_count=0` and `online_joint_count=20` cannot confirm current physical disable. Left-hand feedback was fresh and reported zero enabled, but right-hand disable remained unconfirmed by that record. No hand enable stage was reached in this integrated attempt. The collector now requires fresh state/diagnostic data and the expected terminal state before reporting successful disable; a queued service response is insufficient.

Evidence: `work/full-system-acceptance-summary.json`. This attempt has `completed_window: false` and does not pass integrated acceptance. Any later confirmed device status must be recorded separately, with its own timestamp and freshness evidence.

### Downloaded controller-log evidence

The existing SDK `download_sdk_log()` interface subsequently retrieved the controller's `/home/FUSION/log/LOG.txt` into `work/tianji-cabinet-logs/LOG.txt`. The reviewed excerpt is lines 50090–50140. Its controller-local time is around 05:48:16–05:48:49, which does not agree with the host clock. The position-to-torque transition sequence is consistent with the recorded home initialization, but the timestamps are not directly equated to host wall time.

The excerpt records both arms entering position mode, followed by requests to enter torque mode. The right-arm command initially encounters an `acb` write retry, then logs success on the next cycle and is received by the SI control loop. Thus, this excerpt does not support a conclusion that the torque request never reached the controller. At controller-local 05:48:43, the right arm (`arm1`) enters `___FX_SyncFFDTorque`, reports `difabs fail`, then reports `Servo initial ctrl state is Abnormal Torque2` and transitions to `ARM_STATE_ERROR` (lines 50119–50122). The left arm (`arm0`) enters torque mode and later logs disable completion and return to IDLE (lines 50123–50136). The final host status remains left `0` and right error state `100`; the right error state is not a confirmed normal IDLE state.

These messages localize the right-arm rejection to a controller-internal check during the position-to-torque transition. The excerpt does not name the failing joint, compared quantities, units, or threshold. In particular, the identifier `difabs` alone does not prove an encoder mismatch or an external-force fault. The [vendor error documentation](https://github.com/cynthia-you/TJ_FX_ROBOT_CONTRL_SDK/blob/53ebaa809c2ee5d90b9bed1cafd22c970bd24228/README.md#七主要问题和解决) lists several possible causes of error `6`; it does not map this specific message to one of them. A read-only search of 210 source/documentation files at that official revision found the error-code definition and general troubleshooting guidance, but no implementation or occurrence of `SyncFFDTorque`, `difabs`, `Abnormal Torque2`, or `RequestTransToSensorMode`. The numeric threshold and operands therefore remain undetermined from the inspected public SDK sources. Obtain the definition applicable to controller `100343009`, including the failing axis and compared values, before assigning a physical cause or changing protection settings. Tool dynamics remain uncalibrated; the current all-zero cabinet values are not evidence of a valid loaded-hand model. External-force protection has not been disabled as a workaround.

### Error-clear attempt and final stop observations

The subsequent recovery operation only cleared the right arm's error `6` and requested standby; it did not enable either arm again. The end of `work/tianji-cabinet-logs/LOG-after-clear.txt`, at controller-local 06:05:58, repeatedly records the right arm resetting its error and entering IDLE, followed on the next control cycle by `some servos are still enabled` and a return to `ARM_STATE_ERROR`. This is evidence of an unsuccessful stable return to standby, not a successful recovery. The cabinet clock remains unsynchronized with the host clock.

A later feedback inspection reported left/right states `0` / `100` and zero error codes on both sides, while both feedback serial values remained fixed at `984732`. A cleared error code alone is insufficient: the right state was still ERROR, and unchanged serials did not establish advancing feedback. No successful standby confirmation is claimed from this inspection.

The recovery script also failed to serialize its evidence because a native snapshot contained `bytes` values that JSON could not encode. Inspection of `work/tianji_recover_standby.py` confirms that this exception occurred in the `finally` block **after `release_robot()` returned**. This was an evidence-saving failure, not proof that SDK release was skipped; conversely, a returned release call does not prove physical servo disable. There is no successfully saved recovery JSON to treat as a passing result.

All software control processes were subsequently stopped. The operator then confirmed that the physical emergency stop was pressed and that both arms had remained stationary. This is recorded as an operator observation, distinct from controller feedback. Software standby was not confirmed by that recovery attempt.

### Cabinet restart with the emergency stop retained

The operator subsequently restarted the control cabinet while keeping the physical emergency stop engaged. The read-only probe in `work/tianji-after-reboot.json` recorded 12 samples. Both arms' feedback counters advanced from `33160` to `35932`; the last pair advanced from `35680` to `35932`. The first sample established the baseline and was correctly not marked fresh; the remaining 11 samples were marked fresh. Every sample reported state `100`, error `13` for both arms, consistent with the retained emergency stop.

This later result supersedes the frozen `984732` counters as the latest feedback observation; it does not erase that earlier failure. The probe confirms recovered feedback while the cabinet is in emergency stop. It did not release the emergency stop, clear errors, enable an arm, or send motion. It therefore does not establish successful standby after release or resolution of the original error `6` / IDLE–ERROR loop. Motion remains disabled.

## Home orientation and mounting calibration

Offline FK of the retained home, using the bundled M6 model and identity IK tool transform, gives the following unit axes in each arm's own base frame:

| Side | Flange +Y | Flange +Z |
| --- | --- | --- |
| Left | `[-0.005525, 0.999663, 0.025362]` | `[0.998342, 0.006967, -0.057138]` |
| Right | `[0.005525, 0.999663, -0.025362]` | `[0.998342, -0.006967, -0.057138]` |

The model gravity directions are left `[0, +9.81, 0]` and right `[0, -9.81, 0]`. Under those model conventions, flange +Y is nearly downward on the left and nearly upward on the right, while flange +Z is nearly horizontal. The two rows use different arm base frames and must not be directly compared as world-frame vectors.

The official hand models start at `l_mount` / `r_mount` and define the internal mount-to-wrist geometry. They do **not** establish the actual Tianji flange-to-hand-mount rotation for this installation. No measured transformation for that connection is present in the local configuration; the identity IK tool transform only selects flange control. The wrist-to-Tianji target transforms likewise are teleoperation frame mappings, not an installation calibration record. Consequently, the current evidence cannot certify whether the physical palms face downward or face one another at home. Record the installed adapter orientation and measured left/right flange-to-mount transforms before making that claim.

## Remaining acceptance work

1. Keep motion disabled while diagnosing the prior right-arm IDLE/ERROR loop. Preserve the new advancing emergency-stop feedback separately from the earlier frozen sample and from any later recovery observation. A later authorized recovery must verify fresh controller states rather than assuming that a restart resolved the torque-mode problem.
2. Resolve the right-arm torque-mode transition failure using the controller's failing check, axis, quantities, and threshold. Verify the updated cancellable initialization, settling checks, and stop behavior separately from hand connectivity before repeating integrated acceptance. Preserve the fresh disabled-hand sample as a separate observation and use the corrected collector for all subsequent tests.
3. Extend TPX input/target validation from the observed open and deeply curled poses to controlled shallow bends, asymmetric finger poses, thumb opposition, abduction, comfortable range, and observed anatomical direction. Investigate the deeply curled target contacts as a distinct geometry concern. Preserve the successful exact-profile loading and improved open-pose samples as scoped results; neither numerical bounds nor the open-pose geometry establishes complete mapping or following.
4. Investigate the right middle-finger PIP encoder-quality warning before further enabled following. Then validate all fingers on each hand, first individually and then together, including startup, tracking residual, dropouts, fault latching, explicit recovery, and fresh disable confirmation. The attempted 1.0 rad/s / 0.8 A window aborted and is not accepted.
5. Keep mass/centre-of-mass identification and flange-to-mount calibration explicitly deferred outside the current functional teleoperation acceptance scope. Do not claim calibrated dynamics or a proven physical palm orientation from the current record, and do not change the retained home merely to match an unverified assumption.
6. Complete a controlled two-arm Tracker-following test and then a representative combined task with both hands. Record sustained input timing, commanded versus measured motion, faults, stop latency, final fresh device states, and operator observations.

The supported conclusion is that substantial parts of the migrated pipeline work and several defects have been corrected, but the current functional teleoperation acceptance remains incomplete. The explicitly deferred calibration work is outside that acceptance scope.
