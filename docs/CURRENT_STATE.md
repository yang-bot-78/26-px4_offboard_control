# Current State

Status date: 2026-08-03 (Asia/Shanghai)

## Authority

This is the Phase 1B-I-P current-state document. It is derived from:

```text
PHASE0_REPORT=/home/robot/ws_offboard_control/PHASE0_READONLY_INVENTORY_20260803.md
PHASE0_EXPECTED_SHA256=8ac745e5e0b58e595ab5d9f4b0525e57dfcaa09202cb70c8105a57f7c7765fe0
PHASE0_OBSERVED_SHA256=8ac745e5e0b58e595ab5d9f4b0525e57dfcaa09202cb70c8105a57f7c7765fe0
PHASE0_SHA_VERIFY=PASS
```

Actual installed files and frozen manifests remain higher authority than this
summary. Historical README/checkpoint statements do not override an observed
identity. See `STATE_CONFLICTS.md` before using any path named here.

## Fixed state

```text
PHASE1B_D_RESULT=COMPLETE_DECISIONS_RECORDED_AUTOSTART_DISABLED
PHASE1B_I_S_RESULT=STATIC_DECISION_COMPLETE
PHASE1B_I_M_RESULT=SOURCE_CONVERGED_STATICALLY
PHASE1B_I_REVIEW_RESULT=PASS_SOURCE_STATIC_CONVERGENCE_WITH_DOCUMENT_GAP
PHASE1B_I_P_RESULT=DOCUMENT_AUTHORITY_CONVERGED
RESULT=IDENTITY_CONFLICT
C1_C11_RESOLUTION=MIXED_SEE_STATE_CONFLICTS
SAFETY_BLOCKERS=9_TRACKED_8_OPEN_1_MITIGATED
C1_C11_RESOLUTION_TABLE=docs/STATE_CONFLICTS.md_SUMMARY
DECISIONS_RECORDED=VISION_POSE_AND_204248_618C2B_SHADOW
STABLE_EV_SELECTION=VISION_POSE
EXACT_VISION_BRIDGE_INPUT=/Odometry/healthy
START_MAVROS_VISION_BRIDGE=true
START_BRIDGE=false
START_PX4_EV_BRIDGE=false
RUNTIME_GLOBAL_EV_WRITER_UNIQUENESS=NOT_VERIFIED
STATIC_ASSERTION_SCOPE=TEXTUAL_NOT_STRUCTURAL
HIGH_RATE_SHADOW_SELECTION=204248_618C2B
C10_FLIGHT_PROFILE_INTENT=UNRESOLVED
AUTOSTART_RENAME_RESULT=PASS
AUTOSTART_POST_SHA=bc8302d6617baa31e375d46c1a0b06a9e4341d3a129d04ae70886f09acedf1b8
PHASE1B_I_M_CHANGED_FILES=7_AUTHORIZED_FILES
EXTERNAL_PATH_CHANGE=GNOME_AUTOSTART_RENAME_ONLY
ACTIVE_SYMLINK_CHANGED=NO
PRODUCTION_SOURCE_CHANGED=YES_AUTHORIZED_PHASE1B_I_M_ONLY
BUILD_EXECUTED=NONE
STATIC_VALIDATION=PY_COMPILE_BASH_SYNTAX_ACCEPTANCE_ONLY
NEXT_NO_HARDWARE_GATE_AUTHORIZED=NO
READY_FOR_DIAGNOSTIC_LIVE=NO
NEXT_GATE_AUTHORIZED=NO
HIGH_RATE_STATISTICAL_CONSISTENCY=UNPROVEN
READY_TO_REPLACE_DEFAULT_EV_PATH=NO
FLIGHT_GO_NO_GO=NO
RUNTIME_ACTIONS_EXECUTED=NONE
AUTHORIZATION_CONSUMED=PHASE1B_I_P_NINE_DOCUMENTS_ONLY
```

Phase 1B-I-M changed only the seven authorized source/script/test/document
files and ran only the authorized syntax and acceptance checks. No build,
setup overlay, ROS process, hardware process, gate, deployment, active-link
change, runtime readiness chain, or flight action was performed. The earlier
GNOME autostart rename remains the only engineering-external mutation.

## Phase 1B-I-M reviewed identities

The independent review accepted source static convergence and identified a
document-authority gap. It did not inspect the contents of the temporary
rollback patches and did not establish runtime-global EV writer uniqueness.

### Seven-file postchange SHA-256

| Phase 1B-I-M file | SHA-256 |
| --- | --- |
| `src/px4_ros_com/launch/fastlio_mavros_autofix.launch.py` | `f326608f4b4fb6367732279254f3b8d133e07f160dde2bcb8c151ea829344ade` |
| `一键启动起飞栈.sh` | `2ac41297a1a87e37a749d4c15683bea19f8e3f296fbf52403853fbb2e5fe17c8` |
| `起飞前自检.sh` | `221255fb63ffd107c169b1eedd084fb66f395f965cb1885f0366f4b9a4def97d` |
| `src/px4_ros_com/test/test_ev_odometry_acceptance.py` | `e005e13bbc9ac5c1b93ae716e1e46072b155e36c3e8595bb7f89750bb0eb2cb6` |
| `docs/CURRENT_STATE.md` | `d9a9b50bc86a17122cd96ba71e0b60713fb473b4707ae63622dfe4a27ff3421e` |
| `docs/VALIDATION_MATRIX.md` | `c7e0510013718dc6646d37f21e8a3fd828358d573afd537766bbba6ce2f05d35` |
| `docs/DECISION_LOG.md` | `7e23d3aa4d32b74c19788847a11c4f7601ba689a30744413f9a040456c624fd3` |

### Seven independent reverse-patch SHA-256

The recorded location `/tmp/phase1b_i_m_r3_final_20260803_ltkLRh/rollback`
is temporary. Phase 1B-I review did not inspect these patch contents.

| Reverse patch | SHA-256 |
| --- | --- |
| `launch.reverse.patch` | `b1ca7530ef955608da3848c0bf675eaf24a23ea04b3bd2e513f049a8177833c9` |
| `stack.reverse.patch` | `27d383ddcfe759364bb2633b6b5c344620aba81228e664b5acfd04a974444dc8` |
| `check.reverse.patch` | `81811c8b8ddb709581f05211e341397948394ede90b841860cd07509bbcb9184` |
| `test.reverse.patch` | `08eec227cb64bccd6870b433e4d361db01663e2129ea59080e0039812f1046e2` |
| `current_state.reverse.patch` | `daa26122e7b9597690efe3bcd4139c9ebd597db06c7ff9c20767a193ac4d2ae3` |
| `validation.reverse.patch` | `a30ff36d8d6cca76ae0c2282c1e2f473f9009f2f8b9fd4966a0e18c3c8992134` |
| `decision.reverse.patch` | `1e81781516f813731194e6e7e26e9c2fbc59c2e8b47d06d6b698c9836a1643ff` |

## Observed identities

### Workspace

```text
WORKSPACE=/home/robot/ws_offboard_control
GIT_BRANCH=main
GIT_HEAD=f6d27706e09c767c4c2169bac60df57a9ea31ace
WORKTREE_STATE=DIRTY_USER_CHANGES_PRESERVED
```

The dirty worktree includes modified, deleted, and untracked user content.
Untracked or deleted does not mean disposable. No cleanup is authorized.

### Stable sensor and EV chain

| Item | Observed value | State |
| --- | --- | --- |
| Stable FAST-LIO install | `/home/robot/livox_mid360_env/ws_fastlio/install/fast_lio/lib/fast_lio/fastlio_mapping` | SHA `179576271ebca0436e3d853bc3b805304fd9cbae7b45010a6e022167da639460`, build-id `3fda9b51712df1f93d89c6299c4120a350933a4b` |
| Livox driver install | `/home/robot/livox_mid360_env/ws_livox/install/livox_ros_driver2/lib/livox_ros_driver2/livox_ros_driver2_node` | SHA `eade3c846a8f91caa821c19c36a4afbbae60464532bc479e5621e3a38b6271bb` |
| Main EV launch source | `fastlio_mavros_autofix.launch.py` | `SOURCE_CONVERGED_STATICALLY`: vision ON, MAVROS ODOMETRY OFF, direct PX4 EV OFF |
| Main EV launch installed entry | workspace `install/` symlink to the source launch | effective bytes follow the source; no build or package-install command was executed |
| Selected stable flight baseline | MAVROS `vision_pose` | source decision and entry defaults converged statically; runtime not verified |
| Exact selected vision bridge input | `/Odometry/healthy` | statically determined and bound through `healthy_odom_topic`; source stamp preserved |
| Takeoff readiness | fresh `vision_pose` plus continuous fresh EV health | source contract aligned; full readiness chain not executed |

The stable EV source entry is statically converged but not built or runtime-verified:

```text
STABLE_EV_DECISION_REQUIRED=NO
STABLE_EV_SELECTED=VISION_POSE
STABLE_EV_IMPLEMENTATION_CONVERGED=SOURCE_CONVERGED_STATICALLY
EXACT_VISION_BRIDGE_INPUT=/Odometry/healthy
RUNTIME_GLOBAL_EV_WRITER_UNIQUENESS=NOT_VERIFIED
```

### High-rate shadow candidates

| Identity | Root/runtime | Static identity state |
| --- | --- | --- |
| `191236/b5a32c` | `/home/robot/ws_high_rate_odom_20260801_191236_event_sample_limited/runtime/v5/formal_install` | selected by current `active` symlink and root runner; `fastlio_mapping` SHA `b5a32c066507bd633c693721909e07f35169fa8c95955042f38ea9bec337cafa`, build-id `3d4c035bd598a40fbbfc45240379c77ec7d59d2b` |
| `204248/618c2b` | `/home/robot/ws_high_rate_odom_20260801_204248_event_export_outproc/install_restart` | frozen candidate; `fastlio_mapping` SHA `618c2b4db8b1f9df648f31cd51af3bc25511f3d94c2b8f707ec33fc3befdafd1`, build-id `4f2caf70e476e70868067e0e42f4797bc0665892` |
| out-of-process exporter | under the `204248` install | SHA `158cc01e8205d12fe3a6528eb41a06fb52eefc835b8b99f3262f6d9f72a3ef3d` |
| shared installed fresh relay | workspace main install | SHA `bf5b145dd1158e44c90de0c8b451cac4e1a4223756a907aeb87dbbf32991e92d` |

Both identities exist. B2 is selected only as the future high-rate shadow
candidate. The operational `active` pointer and current root runner still bind
B1; neither was changed.

```text
HIGH_RATE_CANDIDATE_DECISION_REQUIRED=NO
HIGH_RATE_SHADOW_SELECTED=204248_618C2B
ACTIVE_SYMLINK_TARGET=191236_B5A32C
SHADOW_SELECTION_IMPLEMENTED_IN_RUNNER=NO
HIGH_RATE_PROMOTION_AUTHORIZED=NO
```

The B2 MAVROS/PX4 nonflight 90-second result remains FAIL. Selection does not
overwrite, weaken, or relabel that result.

### Diagnostic helper

| Item | Observed identity |
| --- | --- |
| installed root helper | `d647a8081e4f1143dd446d83a837a0857b54d5d5dfc5b0e8b33bb7d53c904a50`, root:root 0755 |
| reviewed workspace helper | `ba78b283349c235c1619be769e45e8573cfdd64a95aec0d539bf4d4838c979ff`, robot-owned, not deployed |
| installed/workspace wrapper | `c862b9a1a8dc64da7aaddb8f1ece1c087377692f22846d372df1707ca3450ce2` |
| perf sysctl file | `6b796efad5bc985c5c7077fcd10519272d818762049dbd3e046acdbe31b15547` |

The installed helper is not the reviewed per-thread candidate. Deployment and
the next no-hardware check remain unauthorized.

### GNOME autostart disposition

The authorized same-directory rename completed:

```text
SOURCE=/home/robot/.config/autostart/ws_offboard_takeoff_stack.desktop
SOURCE_ABSENT=YES
DISABLED=/home/robot/.config/autostart/ws_offboard_takeoff_stack.desktop.phase1b-disabled
DISABLED_TYPE=REGULAR_NON_SYMLINK
OWNER_MODE=robot:robot_0644
SHA256=bc8302d6617baa31e375d46c1a0b06a9e4341d3a129d04ae70886f09acedf1b8
METADATA_UNCHANGED=YES
OTHER_AUTOSTART_SYSTEMD_FILES_UNCHANGED=YES
PROCESS_ACTIONS_EXECUTED=NONE
```

The disabled file retains the original contents for rollback but no longer has
a `.desktop` filename. This prevents that specific future GNOME autostart file
from being discovered by filename. It does not prove or change the state of any
process that may already have been running.

## Current safety blockers

Nine blockers are tracked: eight remain open and the GNOME full-stack autostart
is mitigated by its verified reversible rename.

1. Stable EV source is converged, but the runtime chain is not verified.
2. B2 is selected shadow, while `active` and the runner still bind B1.
3. One-click/readiness source contracts align, but the full readiness chain was not run.
4. Cross-launch exclusivity of all PX4 EV writers is not established.
5. The no-argument high-rate runner remains hardware-capable and binds B1.
6. The installed root helper still differs from reviewed source.
7. High-rate statistical consistency remains unproven.
8. The worktree has unreconciled user changes and unknown recovery boundaries.

C10 remains OPEN outside this blocker count: current flight parameters are not
accepted user intent. C11 remains OPEN and is represented by blocker 7.

## Document roles

| Document | Role |
| --- | --- |
| `CURRENT_STATE.md` | current observed facts and fixed authorization state |
| `STATE_CONFLICTS.md` | C1-C11 evidence, decision ownership, treatment, rollback |
| `SAFETY_BOUNDARY.md` | forbidden actions, completed autostart disposition, and rollback design |
| `VALIDATION_MATRIX.md` | evidence-scoped candidate/gate status |
| `DECISION_LOG.md` | recorded choices, source convergence, review, and authorization boundaries |

This document records Phase 1B-I-P document convergence. It does not authorize
a build, full readiness check, gate, live activity, or flight.
