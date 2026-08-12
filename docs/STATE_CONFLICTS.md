# State Conflicts

Status date: 2026-08-03

This table records Phase 1B-I-P document convergence without claiming runtime
closure. Source/static convergence is not runtime convergence.

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

## Summary

| ID | Static disposition | User decision required | Blocking scope |
| --- | --- | --- | --- |
| C1 | `DECISION_RECORDED`: B2 selected shadow; active/runner still B1 | implementation pending | high-rate entry remains unresolved |
| C2 | `DOCUMENTATION_CONVERGED` | no | historical content retained |
| C3 | `STATICALLY_RESOLVED_FOR_CURRENT_AUTHORITY` | no | sibling files unchanged |
| C4 | `SOURCE_CONVERGED_STATICALLY_NOT_RUNTIME_CLOSED` | runtime evidence/authorization absent | stable EV entry |
| C5 | `SOURCE_CONTRACT_CONVERGED_STATICALLY_NOT_RUNTIME_CLOSED` | runtime evidence/authorization absent | pre-arm and one-click chain |
| C6 | `OPEN_IDENTITY_CONFLICT` | future admin authorization | perf/no-hardware gate |
| C7 | `STATIC_MATCH_SELECTED_SHADOW_SCOPE` | no runtime authorization | B2 composition only |
| C8 | `STATIC_MATCH_SELECTED_SHADOW_SCOPE` | no runtime authorization | B2 composition only |
| C9 | `STATIC_MATCH_WITH_PROVENANCE_GAP` | future freeze/change control | reproducibility |
| C10 | `OPEN_UNRESOLVED` | no flight-profile inference | guides remain historical |
| C11 | `OPEN_INSUFFICIENT_EVIDENCE` | future estimator review | blocks promotion |

## C1: high-rate selected identity

- Actual observed identity: `active` resolves to
  `/home/robot/ws_high_rate_odom_20260801_191236_event_sample_limited`; the root
  runner expects `b5a32c...` and build-id `3d4c...`.
- Candidate identity: the later frozen record describes
  `/home/robot/ws_high_rate_odom_20260801_204248_event_export_outproc/install_restart`,
  `618c2b...`, build-id `4f2c...`, plus exporter `158cc0...`.
- Impact: runner resolution, event architecture, evidence applicability,
  candidate manifests, and every later gate.
- Decision: B2 `204248/618c2b` is the selected future shadow candidate.
- Implementation state: `active` and the root runner still bind B1. Neither was
  changed or authorized for change.
- Minimum next treatment: keep B2 shadow-only. Any active/runner binding remains
  a separate explicit authorization and must not be inferred from C4/C5 work.
- Rollback: restore the pre-change documents; if a later pointer change is
  authorized, restore the captured original symlink target atomically.

Status: `DECISION_RECORDED`; B2 is selected shadow, while active/runner remain B1.

## C2: README current-active generations

- Actual observed identity: README calls `191236/b5a32c` current active near its
  beginning and later describes/freeze-validates `204248/618c2b`.
- Candidate identity: either Decision B choice, without using document order or
  timestamps as authority.
- Impact: operators can bind the wrong runtime, SHA, runner, or evidence.
- Resolution: B2 is recorded in the authority banner/current docs. Existing
  README generations are retained and labeled historical.
- Rollback: revert only the Phase 1B README labeling patch.

Status: `DOCUMENTATION_CONVERGED`; C1 runtime implementation remains open.

## C3: historical documentation root

- Actual observed identity: current selector points to `191236`, while
  `/home/robot/ws_offboard_control_md/CURRENT_STATE.md` calls the older
  `20260731...062932/f8e7b8...` root active.
- Candidate identity: none. The sibling state is historical evidence, not a
  current candidate selection.
- Impact: broad searches can return a stale `ACTIVE_*` declaration.
- Resolution owner: current authority can be resolved statically; changing the
  sibling archive requires separate scope.
- Minimum treatment: current docs label the sibling root historical and never
  import its `ACTIVE_*` values.
- Rollback: remove the current-doc label. Do not edit or move the sibling root.

Status: `STATICALLY_RESOLVED_FOR_CURRENT_AUTHORITY`; historical artifact remains
unchanged.

## C4: stable PX4 EV entry

- Actual observed identity: source launch SHA `f32660...` defaults
  `start_mavros_vision_bridge=true`, `start_bridge=false`, and
  `start_px4_ev_bridge=false`. The workspace install launch entry is a symlink
  to that authorized source.
- Decision identity: stable output is MAVROS `vision_pose`; the exact bridge
  input is `/Odometry/healthy` and `restamp_message=false`.
- Impact: PX4 input plugin, frame/covariance semantics, health path, flight
  evidence applicability, and mutual exclusion.
- Decision: `VISION_POSE` source entry is statically converged. The acceptance
  assertions are textual, not structural, and no runtime chain was executed.
- Minimum next treatment: none is authorized. Runtime-global writer uniqueness
  and runtime readiness require a separate future decision and authorization.
- Rollback: restore captured hashes of launch/control files and rebuild only
  under separate authorization.

Status: `SOURCE_CONVERGED_STATICALLY_NOT_RUNTIME_CLOSED`.

## C5: one-click readiness contradiction

- Actual observed identity: `run_takeoff_1m_hold.sh` requires fresh vision pose
  and EV health. `一键启动起飞栈.sh` now passes vision ON, MAVROS
  ODOMETRY OFF, and direct PX4 EV OFF explicitly.
- Selected direction: `/Odometry/healthy` feeds the MAVROS vision bridge with
  source timestamps preserved.
- Impact: the source-level one-click and readiness contracts now agree.
  Runtime-global writer uniqueness and actual message delivery remain unverified.
- Resolution owner: any runtime check requires separate authorization.
- Minimum treatment: retain the static contract and do not claim it is runnable
  or runtime-closed from textual assertions alone.
- Rollback: restore launch/root-script/control-node changes as one unit.

Status: `SOURCE_CONTRACT_CONVERGED_STATICALLY_NOT_RUNTIME_CLOSED`.

## C6: privileged helper identity

- Actual observed identity: installed root helper `d647a8...`; installed wrapper
  `c862b9...`.
- Candidate identity: reviewed source helper `ba78b2...` with per-thread mmap;
  wrapper remains `c862b9...`.
- Impact: `PERF_NO_LOST`, no-hardware gate admission, durable trace readiness.
- Resolution owner: static evidence identifies the mismatch, but only the user
  and administrator may authorize a later deployment and gate.
- Minimum treatment: keep the installed identity documented as blocked. Do not
  deploy in Phase 1B unless separately and explicitly authorized.
- Rollback: future deployment must retain the old helper bytes/metadata in an
  approved root-owned rollback location and restore them through the same admin
  procedure.

Status: `OPEN_IDENTITY_CONFLICT`; `NEXT_NO_HARDWARE_GATE_AUTHORIZED=NO`.

## C7: fresh relay identity

- Actual observed identity: workspace installed relay SHA `bf5b14...`, build-id
  `0bbfb0...`.
- Candidate identity: `204248` frozen runtime manifest expects the same
  `bf5b14...` SHA.
- Impact: establishes only relay-file correspondence for the selected B2 frozen
  manifest.
- Resolution owner: static evidence resolves the byte comparison; it grants no
  runtime authorization.
- Minimum treatment: record `STATIC_MATCH`; do not rebuild or relabel the relay.
- Rollback: documentation-only rollback.

Status: `STATIC_MATCH_SELECTED_SHADOW_SCOPE`, not runtime approval.

## C8: out-of-process exporter identity

- Actual observed identity: `204248/install_restart` exporter SHA `158cc0...`,
  build-id `9885ba...`.
- Candidate identity: frozen manifest expects `158cc0...`.
- Impact: establishes exporter correspondence for selected B2 only; the current
  root runner still does not select that candidate/exporter.
- Resolution owner: static byte comparison is clear; operational selection
  remains pending future explicitly authorized runner work.
- Minimum treatment: record `STATIC_MATCH`; do not start or promote it.
- Rollback: documentation-only rollback.

Status: `STATIC_MATCH_SELECTED_SHADOW_SCOPE`, operational role unresolved.

## C9: source/install correspondence versus Git provenance

- Actual observed identity: source/install launch both `38bc83...` and
  source/install Offboard script both `31e717...`.
- Candidate identity: Git `HEAD` is not the complete current worktree identity;
  the Offboard script and related files have user changes.
- Impact: installed runtime corresponds to current source bytes for these two
  files, but the complete build provenance and recoverability of untracked
  dependencies are not proven.
- Resolution owner: static evidence resolves the pairwise matches. The user must
  authorize how current dirty changes are frozen/committed before a release.
- Minimum treatment: record exact hashes and avoid claims such as "clean build
  from HEAD".
- Rollback: docs only now; later source changes require file-specific patches,
  never a worktree reset.

Status: `STATIC_MATCH_WITH_PROVENANCE_GAP`.

## C10: flight-profile documentation

- Actual observed identity: current root script defaults to target Z `-0.50 m`,
  initial hover `5 s`, spin enabled, and post-spin hover `10 s`.
- Candidate identities: existing guides variously state 0.5 m or 1.0 m and do
  not consistently describe the spin behavior.
- Impact: operator expectation, test envelope, safety review, and record labels.
- Decision: `C10_FLIGHT_PROFILE_INTENT=UNRESOLVED`. Current values are observed
  implementation, not accepted intent.
- Minimum treatment: label guide values historical/inconsistent. Do not change
  or normalize flight parameters without a separate user decision.
- Rollback: revert guide-only edits. Any later flight-profile code change needs
  separate flight-control authorization.

Status: `OPEN_UNRESOLVED`; no flight-profile intent is inferred.

## C11: covariance and statistical consistency

- Actual observed identity: older checkpoint documents a velocity covariance
  floor fix and individual validation results.
- Candidate identity: the 30 Hz propagated stream has no closed audit proving
  posterior correlation, covariance propagation, correction age, and PX4
  statistical interpretation.
- Impact: high-rate candidate cannot replace the stable PX4 EV path.
- Resolution owner: not a user preference. It requires the future estimator
  semantics review and separately authorized offline validation.
- Minimum treatment: retain `HIGH_RATE_STATISTICAL_CONSISTENCY=UNPROVEN` and
  keep both candidates shadow-only.
- Rollback: not applicable; evidence, not wording, must change this state.

Status: `OPEN_INSUFFICIENT_EVIDENCE`.
