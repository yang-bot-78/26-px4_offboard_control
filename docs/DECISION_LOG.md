# Decision Log

Status date: 2026-08-03

## Recorded decisions

| ID | Decision | State |
| --- | --- | --- |
| D-001 | Phase 0 report SHA is the Phase 1A baseline | accepted; SHA verify PASS |
| D-002 | Do not infer recency or quality from timestamps, names, or "latest" wording | accepted |
| D-003 | C1/C4 decisions may be recorded but are not closed until implementation converges | accepted |
| D-004 | No runtime, gate, deployment, active-link, runner, or production-source change in Phase 1B-D | accepted |
| D-005 | Stable EV adapter selection | `VISION_POSE`, exact input `/Odometry/healthy` |
| D-006 | High-rate shadow selection | `204248/618c2b`, shadow only; active/runner remain B1 |
| D-007 | High-rate remains shadow and flight remains NO regardless of Decision B | accepted |
| D-008 | GNOME autostart disable | authorized rename executed and verified; rollback not executed |
| D-009 | C10 flight profile | `UNRESOLVED`; do not infer current parameters as intent |
| D-010 | Phase 1B-I-S exact input decision | `/Odometry/healthy`, supported by static source and retained flight/prop-off evidence |
| D-011 | Phase 1B-I-M implementation | seven authorized files only; `SOURCE_CONVERGED_STATICALLY`; no build or runtime |
| D-012 | Independent source review | `PASS_SOURCE_STATIC_CONVERGENCE_WITH_DOCUMENT_GAP`; source rollback not required |
| D-013 | Phase 1B-I-P document convergence | nine authority documents only; no source, test, build, runtime, gate, live, or flight authorization |

## Decision A: stable EV entry

Selected: A1 MAVROS `vision_pose`, with exact input `/Odometry/healthy` and
source timestamp preservation. Phase 1B-I-M converges the authorized source
entry statically; it does not establish runtime convergence.

| Criterion | A1: MAVROS `vision_pose` | A2: MAVROS ODOMETRY |
| --- | --- | --- |
| Data path | `/Odometry -> /Odometry/healthy -> /mavros/vision_pose/pose_cov -> PX4` | `/Odometry/healthy -> /mavros/odometry/out -> PX4` |
| Static/historical evidence | reorganization brief calls it flight-validated baseline; older flight and prop-off vision evidence exist | current launch default and later prop-off data-correctness work use it |
| Current source launch state | bridge defaults on | bridge defaults off |
| Current takeoff readiness compatibility | matches `require_vision_pose=true` in concept | conflicts with vision freshness requirement |
| Velocity/covariance path | pose input; optional vision speed exists but `EKF2_EV_CTRL=11` does not fuse EV velocity | REP-147 pose/twist/covariance path; current bridge preserves source stamp |
| Required Phase 1B code alignment | make vision bridge the sole default EV writer; explicitly disable ODOMETRY/direct writers | keep ODOMETRY sole writer; replace vision-only readiness with a health/odometry contract |
| Primary risk | accidentally leaving ODOMETRY enabled would create dual PX4 EV inputs | changing readiness semantics may alter pre-arm safety behavior and requires focused review |
| Evidence gap | runtime/global writer uniqueness is not verified | velocity fusion readiness remains NO |
| Selection effect | does not authorize flight | does not authorize flight |

Recorded response:

```text
STABLE_EV_SELECTION=VISION_POSE
```

```text
STABLE_EV_DECISION_REQUIRED=NO
STABLE_EV_IMPLEMENTATION_CONVERGED=SOURCE_CONVERGED_STATICALLY
EXACT_VISION_BRIDGE_INPUT=/Odometry/healthy
RUNTIME_GLOBAL_EV_WRITER_UNIQUENESS=NOT_VERIFIED
```

## Decision B: high-rate shadow candidate

Selected: B2 `204248/618c2b` for future shadow work only. Selection is not
promotion and does not authorize changing `active`, the runner, a gate, or
flight.

| Criterion | B1: `191236/b5a32c` | B2: `204248/618c2b` |
| --- | --- | --- |
| Root | `...20260801_191236_event_sample_limited` | `...20260801_204248_event_export_outproc` |
| FAST-LIO identity | `b5a32c...`, build-id `3d4c...` | `618c2b...`, build-id `4f2c...` |
| Selector/runner compatibility | current `active` pointer and root runner already bind it | current pointer/root runner do not bind it |
| Event architecture | sampled in-process JSON event path | fixed raw binary batches plus exporter `158cc0...` |
| Frozen evidence | runtime manifest and active-generation evidence | explicit frozen candidate manifest, 112-item identity scope documented |
| No-MAVROS 90 s | static inventory did not establish a bound B1 PASS | bound run documented PASS |
| MAVROS/PX4 90 s | prior failures documented | bound run documented FAIL (`IMU_TIMEOUT`/EV interruption) |
| Recovery evidence | older generation behavior includes recovery defect history | controlled IMU_GAP recovery documented PASS |
| Integration work | less pointer/runner change | needs explicit exporter lifecycle and runner/pointer reconciliation |
| Statistical consistency | UNPROVEN | UNPROVEN |
| Ready to replace stable | NO | NO |

Recorded response:

```text
HIGH_RATE_SHADOW_SELECTION=204248_618C2B
HIGH_RATE_CANDIDATE_DECISION_REQUIRED=NO
ACTIVE_SYMLINK_TARGET=191236_B5A32C
HIGH_RATE_PROMOTION_AUTHORIZED=NO
```

The B2 MAVROS/PX4 90-second FAIL remains binding and unchanged.

## C10 flight-profile intent

Decision A does not answer C10. The user explicitly left flight-profile intent
unresolved. These are observed current defaults only, not accepted intent:

```text
TARGET_Z_M=-0.50
HOVER_SECONDS=5.0
ENABLE_SPIN_TEST=true
POST_SPIN_HOVER_SECONDS=10.0
```

Phase 1B-D labels the guide content historical/inconsistent and does not edit
flight parameters.

## Phase 1B-D changed files and external action

| Path | Change | Rollback |
| --- | --- | --- |
| five `docs/*.md` authority documents | decisions/status/autostart result recorded | restore Phase 1A document bytes |
| `README.md` | authority banner and historical-generation label only | remove inserted banner/label |
| `CODEX_CHECKPOINT.md` | authority banner and historical-generation label only | remove inserted banner/label |
| `ROOT_SCRIPTS_QUICK_REF.md` | safety classification and C10 unresolved notice | revert guide-only patch |
| `SCRIPTS_GUIDE.md` | safety classification and C10 unresolved notice | revert guide-only patch |
| GNOME desktop file | renamed to `.desktop.phase1b-disabled` | separately authorize reverse rename, then verify SHA/metadata |

No production source, launch, takeoff script, test, active symlink, runner,
candidate, manifest, evidence, archive, install, or privileged file changed.

## Phase 1B-I-M source convergence

Phase 1B-I-M authorized exactly the seven files listed below and the specified
syntax/acceptance checks. It did not authorize build, install commands, full
readiness, runtime, hardware, active-link, runner, candidate, helper, gate, or
flight work. The workspace install launch entry is a symlink to the authorized
source launch; no package-install operation was performed.

```text
PHASE1B_I_M_RESULT=SOURCE_CONVERGED_STATICALLY
PHASE1B_I_REVIEW_RESULT=PASS_SOURCE_STATIC_CONVERGENCE_WITH_DOCUMENT_GAP
EXACT_VISION_BRIDGE_INPUT=/Odometry/healthy
START_MAVROS_VISION_BRIDGE=true
START_BRIDGE=false
START_PX4_EV_BRIDGE=false
RUNTIME_GLOBAL_EV_WRITER_UNIQUENESS=NOT_VERIFIED
STATIC_ASSERTION_SCOPE=TEXTUAL_NOT_STRUCTURAL
NEXT_GATE_AUTHORIZED=NO
READY_FOR_DIAGNOSTIC_LIVE=NO
FLIGHT_GO_NO_GO=NO
```

### Phase 1B-I-M seven-file postchange SHA-256

| File | SHA-256 |
| --- | --- |
| `src/px4_ros_com/launch/fastlio_mavros_autofix.launch.py` | `f326608f4b4fb6367732279254f3b8d133e07f160dde2bcb8c151ea829344ade` |
| `一键启动起飞栈.sh` | `2ac41297a1a87e37a749d4c15683bea19f8e3f296fbf52403853fbb2e5fe17c8` |
| `起飞前自检.sh` | `221255fb63ffd107c169b1eedd084fb66f395f965cb1885f0366f4b9a4def97d` |
| `src/px4_ros_com/test/test_ev_odometry_acceptance.py` | `e005e13bbc9ac5c1b93ae716e1e46072b155e36c3e8595bb7f89750bb0eb2cb6` |
| `docs/CURRENT_STATE.md` | `d9a9b50bc86a17122cd96ba71e0b60713fb473b4707ae63622dfe4a27ff3421e` |
| `docs/VALIDATION_MATRIX.md` | `c7e0510013718dc6646d37f21e8a3fd828358d573afd537766bbba6ce2f05d35` |
| `docs/DECISION_LOG.md` | `7e23d3aa4d32b74c19788847a11c4f7601ba689a30744413f9a040456c624fd3` |

### Phase 1B-I-M seven reverse-patch SHA-256

The recorded path `/tmp/phase1b_i_m_r3_final_20260803_ltkLRh/rollback` is a
temporary artifact location. The independent review did not inspect patch
contents and this record does not convert them into durable archive evidence.

| Reverse patch | SHA-256 |
| --- | --- |
| `launch.reverse.patch` | `b1ca7530ef955608da3848c0bf675eaf24a23ea04b3bd2e513f049a8177833c9` |
| `stack.reverse.patch` | `27d383ddcfe759364bb2633b6b5c344620aba81228e664b5acfd04a974444dc8` |
| `check.reverse.patch` | `81811c8b8ddb709581f05211e341397948394ede90b841860cd07509bbcb9184` |
| `test.reverse.patch` | `08eec227cb64bccd6870b433e4d361db01663e2129ea59080e0039812f1046e2` |
| `current_state.reverse.patch` | `daa26122e7b9597690efe3bcd4139c9ebd597db06c7ff9c20767a193ac4d2ae3` |
| `validation.reverse.patch` | `a30ff36d8d6cca76ae0c2282c1e2f473f9009f2f8b9fd4966a0e18c3c8992134` |
| `decision.reverse.patch` | `1e81781516f813731194e6e7e26e9c2fbc59c2e8b47d06d6b698c9836a1643ff` |

### Authority follow-up

| File | Proposed change | Rollback |
| --- | --- | --- |
| `docs/CURRENT_STATE.md` | after authorized implementation, record exact source hashes and convergence state; leave gates/flight NO | revert only the new implementation record |
| `docs/STATE_CONFLICTS.md` | close only conflicts proven by the authorized implementation; retain C6/C10/C11 as applicable | restore Phase 1B-D status table |
| `docs/SAFETY_BOUNDARY.md` | add any newly authorized entry boundary without altering the completed autostart record | revert only the new boundary entry |
| `docs/VALIDATION_MATRIX.md` | bind any static result to exact source identity; do not add runtime PASS | remove the new static row |
| `docs/DECISION_LOG.md` | append Phase 1B-I authorization, changed-file hashes, and rollback | remove the Phase 1B-I entry |
| `README.md` | update the authority banner only after implementation state actually changes | restore Phase 1B-D banner |
| `CODEX_CHECKPOINT.md` | remain historical; only its authority banner may be updated | restore Phase 1B-D banner |
| `ROOT_SCRIPTS_QUICK_REF.md` | update selected-entry status only; keep C10 unresolved | restore Phase 1B-D guide banner |
| `SCRIPTS_GUIDE.md` | update selected-entry status only; keep C10 unresolved | restore Phase 1B-D guide banner |

`README_CODEX_REORGANIZATION.md`, frozen manifests, raw evidence, archives, and
runtime records should remain byte-for-byte unchanged.

### GNOME safety action completed

| Existing file | Proposed change | Rollback |
| --- | --- | --- |
| `/home/robot/.config/autostart/ws_offboard_takeoff_stack.desktop` | renamed to `.desktop.phase1b-disabled`; PASS | separately authorize rename back and verify SHA/metadata |

Do not run `禁用开机自启.sh`; it has broader effects.

### Selected A1 branch: `vision_pose`, implemented in authorized source scope

| File | Proposed minimal change | Rollback |
| --- | --- | --- |
| Phase 1B-I-S static review | selected `/Odometry/healthy` from source and retained validation evidence | documentation-only rollback |
| `src/px4_ros_com/launch/fastlio_mavros_autofix.launch.py` | default vision bridge on; ODOMETRY and direct PX4 bridges off; keep single-output validation | apply its independent reverse patch |
| `一键启动起飞栈.sh` | pass explicit vision-on/other-EV-off arguments so defaults cannot drift silently | revert this file's patch |
| `起飞前自检.sh` | fail closed on source/install defaults, healthy binding, timestamp preservation, single-adapter validation, and explicit one-click arguments | revert this file's patch |
| `src/px4_ros_com/test/test_ev_odometry_acceptance.py` | assert vision-only defaults, healthy input, timestamp preservation, and explicit stack arguments while retaining applicable safety assertions | revert test patch |

No package-install command was executed. Only the explicitly authorized static
checks were run; any build, full readiness check, or runtime test requires a
later authorization. C1/C4/C5 are not runtime-closed. C10 and C11 remain OPEN.

### Non-selected A2 branch: MAVROS ODOMETRY

| File | Proposed minimal change | Rollback |
| --- | --- | --- |
| `src/px4_ros_com/launch/fastlio_mavros_autofix.launch.py` | retain ODOMETRY as sole writer and make all alternatives explicitly off | restore captured file hash/content |
| `src/px4_ros_com/scripts/minipc_mavros_offboard.py` | replace vision-topic-only readiness with a named health/healthy-odometry readiness contract; keep fail-closed freshness | revert control-node patch as one unit |
| `run_takeoff_1m_hold.sh` | pass the selected readiness parameters explicitly; do not change flight profile under C10 | revert root-script patch |
| `一键启动起飞栈.sh` | pass explicit ODOMETRY-on/other-EV-off arguments | revert root-stack patch |
| `起飞前自检.sh` | assert selected launch and readiness contract | revert check patch |
| `src/px4_ros_com/test/test_minipc_mavros_offboard_ev_safety.py` | add fail-closed freshness/startup coverage for the selected readiness contract | revert test patch |
| `src/px4_ros_com/test/test_ev_odometry_acceptance.py` | preserve unique ODOMETRY input, timestamp, covariance, and alternative-writer rejection coverage | revert test patch |

This branch is not selected. Retain it only as decision history; do not
implement it under a VISION_POSE authorization.

### Operational B1 fact

| File/object | Proposed minimal change | Rollback |
| --- | --- | --- |
| Phase 1B-D docs and README banner | retain B1 only as the observed current active/runner identity | revert only a future implementation-status edit |
| `run_high_rate_px4_ev_bench.sh` | no identity change; document that its no-argument hardware mode remains prohibited | revert documentation/comment-only patch if made |
| `active` symlink | no change required by identity, but do not treat existing pointer as authorization | not applicable |

### Selected B2 shadow, not yet bound to runtime

Phase 1B-D records the selection in documents only. Repointing
`active` would immediately affect a hardware-capable no-argument runner and is
therefore a separate atomic implementation task.

| File/object | Proposed later atomic change | Rollback |
| --- | --- | --- |
| `/home/robot/ws_high_rate_odom_project/active` | after separate authorization, point to exact B2 root | atomically restore captured B1 target |
| `run_high_rate_px4_ev_bench.sh` | bind B2 root/SHA/build-id/install and explicit exporter lifecycle; remove ambiguity in no-argument behavior | restore captured runner bytes |
| candidate `run_event_export_live.sh` | either designate one reviewed runner or mark archive-only; do not keep two production-like entries | revert designation/docs; do not delete |

No active-link or runner change is part of Phase 1B-D.

## Phase 1B-I authorization boundary

A valid Phase 1B-I authorization must state:

1. The exact source files approved for static convergence.
2. The exact `/Odometry[/healthy]` conclusion and its evidence.
3. Whether changes are documentation-only or production-source edits.
4. Required rollback hashes for every approved file.
5. Confirmation that active, runner, candidate, helper, gates, live, and flight
   remain out of scope unless separately named.

Absent those values, stop after documentation review.
