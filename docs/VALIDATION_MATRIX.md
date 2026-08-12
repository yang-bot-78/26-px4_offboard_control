# Validation Matrix

Status date: 2026-08-03

## Interpretation

All rows are evidence-scoped. `DOCUMENTED_PASS` means a retained report states
PASS; neither Phase 1A nor Phase 1B-D reran or independently reproduced it. A
result for one SHA does not transfer to another SHA. A later offline assessor
does not rewrite an original formal FAIL.

```text
PHASE1B_I_M_RESULT=SOURCE_CONVERGED_STATICALLY
PHASE1B_I_REVIEW_RESULT=PASS_SOURCE_STATIC_CONVERGENCE_WITH_DOCUMENT_GAP
PHASE1B_I_M_BUILD=NOT_RUN
PHASE1B_I_M_STATIC_TESTS=AUTHORIZED_SET_ONLY
PHASE1B_I_M_RUNTIME=NOT_RUN
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

## Phase 1B-I-M reviewed SHA scope

### Seven-file postchange SHA-256

| File | SHA-256 |
| --- | --- |
| `src/px4_ros_com/launch/fastlio_mavros_autofix.launch.py` | `f326608f4b4fb6367732279254f3b8d133e07f160dde2bcb8c151ea829344ade` |
| `一键启动起飞栈.sh` | `2ac41297a1a87e37a749d4c15683bea19f8e3f296fbf52403853fbb2e5fe17c8` |
| `起飞前自检.sh` | `221255fb63ffd107c169b1eedd084fb66f395f965cb1885f0366f4b9a4def97d` |
| `src/px4_ros_com/test/test_ev_odometry_acceptance.py` | `e005e13bbc9ac5c1b93ae716e1e46072b155e36c3e8595bb7f89750bb0eb2cb6` |
| `docs/CURRENT_STATE.md` | `d9a9b50bc86a17122cd96ba71e0b60713fb473b4707ae63622dfe4a27ff3421e` |
| `docs/VALIDATION_MATRIX.md` | `c7e0510013718dc6646d37f21e8a3fd828358d573afd537766bbba6ce2f05d35` |
| `docs/DECISION_LOG.md` | `7e23d3aa4d32b74c19788847a11c4f7601ba689a30744413f9a040456c624fd3` |

### Seven reverse-patch SHA-256

The recorded location `/tmp/phase1b_i_m_r3_final_20260803_ltkLRh/rollback`
is temporary. The independent review did not inspect patch contents.

| Reverse patch | SHA-256 |
| --- | --- |
| `launch.reverse.patch` | `b1ca7530ef955608da3848c0bf675eaf24a23ea04b3bd2e513f049a8177833c9` |
| `stack.reverse.patch` | `27d383ddcfe759364bb2633b6b5c344620aba81228e664b5acfd04a974444dc8` |
| `check.reverse.patch` | `81811c8b8ddb709581f05211e341397948394ede90b841860cd07509bbcb9184` |
| `test.reverse.patch` | `08eec227cb64bccd6870b433e4d361db01663e2129ea59080e0039812f1046e2` |
| `current_state.reverse.patch` | `daa26122e7b9597690efe3bcd4139c9ebd597db06c7ff9c20767a193ac4d2ae3` |
| `validation.reverse.patch` | `a30ff36d8d6cca76ae0c2282c1e2f473f9009f2f8b9fd4966a0e18c3c8992134` |
| `decision.reverse.patch` | `1e81781516f813731194e6e7e26e9c2fbc59c2e8b47d06d6b698c9836a1643ff` |

## Stable EV and flight evidence

| Subject | Identity/evidence scope | Result | Current applicability |
| --- | --- | --- | --- |
| selected stable flight baseline | MAVROS `vision_pose`, input `/Odometry/healthy`, source stamp preserved | `SOURCE_CONVERGED_STATICALLY`; historical flight use documented | source entry converged; runtime/global writer uniqueness not verified |
| 2026-07-21 prop-off vision validation | `prop_off_ev_20260721_161611`, input `/Odometry/healthy` | `DOCUMENTED_PASS` for safety, timing, health recovery, axes, stop-to-zero | workspace raw path is currently tracked-deleted; evidence does not establish current runtime state |
| 2026-07-22 prop-off MAVROS ODOMETRY validation | `prop_off_ev_20260722_152742` | data correctness documented; `READY_FOR_EV_VEL_FUSION=NO` | rate 9.45 Hz and missing PX4 evidence remain blockers |
| current one-click source chain | vision ON, MAVROS ODOMETRY/direct PX4 EV OFF, explicit stack args | `SOURCE_CONVERGED_STATICALLY`; assertions are textual, not structural | runtime readiness not checked; C4/C5 not runtime-closed; flight prohibited |
| current flight profile | root script defaults -0.50 m, 5 s, spin enabled | static observation only | intended profile unresolved under C10 |

## High-rate candidates

| Gate/evidence | `191236/b5a32c` | `204248/618c2b` | Binding conclusion |
| --- | --- | --- | --- |
| identity/manifest presence | active pointer and runtime manifest present | frozen manifest and install present | B2 selected shadow only; active/runner remain B1 |
| release/offline tests | README documents Release + 10/10 for active generation | README/frozen record documents Release + 11/11 and exporter tests | `DOCUMENTED_PASS`, not rerun |
| event architecture | sampled in-process JSON event path | raw binary batch + out-of-process exporter | materially different candidates |
| controlled IMU_GAP recovery | older active documented as lacking automatic recovery in one generation | deterministic depth-8 recovery documented PASS | result does not transfer between candidates |
| strict formal 30 Hz | historical formal evidence contains immutable FAIL | no evidence permits relabeling old formal run | historical FAIL remains FAIL |
| no-MAVROS 90 s | no bound PASS established for B1 by the static inventory | `stability_no_mavros_90s_20260802_135631` documented PASS | applies only to frozen identity/manifest scope |
| MAVROS/PX4 nonflight 90 s | README documents prior failures/gaps | `mavros_px4_nonflight_90s_20260802_142709` documented FAIL due `IMU_TIMEOUT`/EV interruption | no end-to-end PASS |
| 600 s soak | no accepted current PASS | no accepted current PASS | NOT PASSED / no gate authorized |
| fault injection/recovery | generation-dependent evidence | one deterministic recovery PASS | not long-duration stability |
| covariance/statistical consistency | unproven | unproven | blocks promotion for both |
| selected shadow | NO | `SELECTED_SHADOW_ONLY` | active/runner remain B1; no promotion |
| ready to replace stable | NO | NO | fixed |

The B2 `mavros_px4_nonflight_90s_20260802_142709` result is an immutable FAIL
for its evidence scope. Selecting B2 as shadow does not overwrite, downgrade,
or convert that result to PASS, partial pass, or superseded.

## Diagnostic control/perf gates

| Gate | Bound identity/evidence | State |
| --- | --- | --- |
| three-stream durable ACK | isolated no-hardware driver/relay/probe fixtures | `DOCUMENTED_PASS` |
| production tail closure | deterministic fixture plus incomplete production coverage | `PARTIAL_PASS` |
| perf no-lost | installed helper `d647a8...` without reviewed per-thread fix | `FAIL/BLOCKED` |
| latest no-hardware check | authorized attempt already consumed and failed closed | `FAIL/PENDING_NEW_IDENTITY_AND_AUTHORIZATION` |
| reviewed helper source | `ba78b2...`, not deployed | offline tests documented PASS; not installed identity |
| next no-hardware gate | no authorization | `NO` |
| diagnostic live | prerequisites incomplete and no authorization | `NO` |
| flight | no authorization | `NO` |

## Required future ordering

This is a dependency matrix, not authorization:

```text
user decisions A and B recorded
-> Phase 1B-I-M source/static convergence
-> independent review
-> explicit identity freeze and static review
-> separately authorized build/tests
-> separately authorized no-hardware rehearsal
-> separately authorized formal no-hardware gate
-> separately authorized nonflight hardware stages
-> statistical/PX4 fusion review
-> promotion review
```

No step may inherit authorization from the previous step.
