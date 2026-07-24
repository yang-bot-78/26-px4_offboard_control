# FAST-LIO high-rate propagated EV odometry design

Status: design only; not implemented or flight-authorized  
Safety boundary: keep `EKF2_EV_CTRL=11`; do not replace the current healthy PX4
EV path during development or validation.

Design review status after the final timing/FAULT closure: remaining known
Critical/High/Medium issues in this design scope are `0/0/0`.
`DESIGN_APPROVED_CANDIDATE=YES` means ready for an implementation-readiness review,
not authorization to implement, connect to PX4, arm or fly.

## 1. Objective and invariants

Add a read-only FAST-LIO output, `/Odometry/propagated`, containing a genuinely
IMU-propagated state at 30-50 Hz. The existing `/Odometry` publisher, LiDAR update,
main ESKF object, `/Odometry/healthy`, and `/mavros/odometry/out` path remain
behaviorally and structurally unchanged.

The new publisher must never duplicate or restamp a state. Every message must
integrate at least one previously unused IMU sample, and its `header.stamp` must
equal the timestamp of the last IMU sample integrated into that state. A timer may
rate-limit eligible states, but may neither create a state nor change its timestamp.

## 2. Architecture

```text
main FAST-LIO path (unchanged)
LiDAR + IMU -> main ESKF predict/update -> /Odometry (LiDAR end time, about 10 Hz)
                         |
                         | atomic posterior snapshot after LiDAR correction
                         v
              HighRateEvPropagator (independent ESKF copy)
IMU callback --copy--> bounded ordered IMU replay buffer
                         |
                         +-> replay after each posterior replacement
                         +-> propagate each new IMU sample
                         +-> atomic publish-state exchange
                                      |
                        30-50 Hz rate gate, new-IMU-required
                                      v
                             /Odometry/propagated
                                      |
                    isolated recording/validation only
```

`HighRateEvPropagator` owns its predictor, buffers, status and publisher. It may
read a short-lived copy of the main posterior but must never call `predict()` on,
publish from, or retain a reference to the main ESKF.

## 3. Snapshot and state definitions

The design must use the exact current IKFoM types, not a flattened substitute:

- `state_ikfom::DIM = 24` is the nominal/manifold storage dimension used by
  `get_f()`;
- `state_ikfom::DOF = 23` is the error-state dimension, so `kf.get_P()` is exactly
  `23 x 23`;
- state/error ordering is `pos[0:2]`, `rot[3:5]`,
  `offset_R_L_I[6:8]`, `offset_T_L_I[9:11]`, `vel[12:14]`,
  `bg[15:17]`, `ba[18:20]`, and `grav[21:22]` in the error covariance;
- gravity is physically a 3-vector stored on `S2`, but has only two error-state
  degrees of freedom. No `P(23,23)` gravity element exists;
- `input_ikfom` is `[acc(3), gyro(3)]`; `process_noise_ikfom` and the `12 x 12`
  `Q` order are `ng[0:2], na[3:5], nbg[6:8], nba[9:11]`.

The immutable LiDAR posterior snapshot contains:

- correction timestamp `t_c = lidar_end_time` and a monotonic generation;
- a by-value `state_ikfom x_c` containing position, rotation, LiDAR/IMU
  extrinsics, velocity, both biases and S2 gravity;
- the complete by-value IKFoM covariance `P_c` (`23 x 23`, all cross terms);
- the active propagation context: `mean_acc.norm()`, the four 3-axis covariance
  vectors `cov_gyr`, `cov_acc`, `cov_bias_gyr`, `cov_bias_acc`, and therefore the
  exact populated `12 x 12 Q`;
- the predecessor/raw IMU sample needed to form the first midpoint interval after
  `t_c`, plus the last normalized `input_ikfom` used to reach `t_c` for audit.

The propagation context is mandatory because current `UndistortPcl()` rescales
each midpoint acceleration by `G_m_s2 / mean_acc.norm()` and overwrites the four
diagonal `Q` blocks before every `predict()`. Copying only `(x_c, P_c)` would not
reproduce the current process model.

Copy `kf.get_x()` and `kf.get_P()` immediately after
`update_iterated_dyn_share_modified()` finishes. At that point `state_point` is
also refreshed from `kf.get_x()`, and the logical posterior time remains
`lidar_end_time`. Snapshot copying must finish before unrelated map work. The lock
covers only fixed-size value copies, propagation context, predecessor metadata and
generation assignment; it never covers replay, prediction, eigenvalue checks,
message conversion or publication.

The independent state is `(generation, t_p, state_ikfom x_p, 23x23 P_p,
last_imu_sequence, last_raw_imu)`. Publish it through an atomic immutable shared
state so readers cannot observe a mismatched state/covariance/timestamp.

## 4. IMU buffer

Use a separate bounded, timestamp-ordered deque of immutable raw samples:

```text
ImuSample { sequence, timestamp, angular_velocity, linear_acceleration }
```

The copied sample must be taken after the current `imu_cbk()` timestamp correction
(`time_diff_lidar_to_imu`) so it is in the same time domain as `lidar_end_time`.
Store the raw accelerometer and gyro values; apply midpoint averaging and
acceleration normalization only in the predictor.

The callback validates finite payload and a strictly increasing corrected
timestamp, assigns `sequence`, and only appends under the buffer mutex. Retain the
last sample at or before each correction because current `UndistortPcl()` prepends
`last_imu_` and forms `(head, tail)` midpoint inputs. Retain every later sample
until all possible correction generations have passed it. Configure a time span
and hard count; never silently evict a predecessor or replay-required sample.
Overflow, duplicate/backward corrected time, a missing midpoint predecessor or an
unbridgeable IMU gap enters `FAULT`.

Non-finite payload/stamp, duplicate stamp, backward stamp, wrong time-domain stamp,
or a stamp farther in the future than `future_tolerance_s` is rejected before
buffer insertion and before assignment of an accepted integration sequence. It
cannot become predecessor, head, tail or replay input. Maintain a separate ingress
counter for diagnostics; only valid inserted samples increment `accepted_sequence`.
The actual main `imu_cbk()` clears its own buffer only for strictly backward time
and still pushes that sample, but this isolated safety output deliberately fails
closed for duplicate/backward/non-finite data because it must guarantee unique
output stamps. It does not alter the main callback behavior.

All comparisons use corrected sensor timestamps in the LiDAR/IMU sensor epoch.
ROS time is used only to compute source age. Record `(ros_now, steady_now)` pairs:
a ROS-clock discontinuity is detected when the change in ROS time differs from the
steady-clock change beyond `ros_clock_jump_tolerance_s`. Negative source age beyond
`future_tolerance_s`, a ROS jump, or inconsistent LiDAR/IMU epochs enters time-epoch
`FAULT`; no sample is restamped.

The main FAST-LIO `imu_buffer` remains unchanged. The propagator receives a copy in
the existing IMU callback; it does not consume, reorder, or erase the main buffer.

## 5. Correction replacement and replay sequence

1. Main ESKF completes `update_iterated_dyn_share_modified()` at logical time
   `t_c = lidar_end_time` and atomically exports the full snapshot.
2. The worker captures a replay watermark and copies the predecessor sample at or
   before `t_c` plus all buffered tails strictly newer than `t_c` through that
   watermark. No sample is removed yet.
3. Off-lock, initialize a private
   `esekfom::esekf<state_ikfom, 12, input_ikfom>` with `change_x(x_c)` and
   `change_P(P_c)`. Initialize its process callbacks with
   `init_dyn_runtime_share(get_f, df_dx, df_dw, ...)`, which exists specifically
   without a measurement callback; do not bind the main `h_share_model`. Limits
   are initialized for IKFoM bookkeeping but no measurement update is invoked.
4. For each `(head, tail)`, compute raw midpoint gyro/acceleration exactly as
   `UndistortPcl()`, normalize midpoint acceleration by
   `G_m_s2 / mean_acc.norm()`, populate `input_ikfom`, rebuild the four diagonal
   `Q` blocks, and call `predict(dt, Q, in)` with `dt = tail_time - current_time`.
   The first `current_time` is exactly `t_c`; therefore the first publishable tail
   must be strictly newer than `t_c`. Samples at or before `t_c` are context only,
   never a new output state.
5. Current `UndistortPcl()` also predicts from its last consumed IMU time to the
   scan end using the last `in`. That scan-end prediction is already contained in
   `(x_c, P_c)` after LiDAR correction and must not be repeated by the independent
   predictor.
6. Validate the fully replayed result; intermediate replay states are never
   exposed.
7. Once an active generation exists, keep propagating it with fresh IMU while a
   newer generation is replayed as a separate pending predictor. A correction must
   not freeze or partially reset the active predictor.
8. After pending replay reaches the same IMU watermark as active, evaluate
   continuity, covariance and generation predicates. Only then atomically switch
   active to pending. Switching alone never authorizes a message: the next output
   must still advance beyond the global last-published IMU sequence and timestamp.
9. A still newer posterior cancels and replaces pending replay but not the valid
   active generation. Compare-and-swap requires both
   `pending.parent_generation == active_generation` and
   `pending.generation == latest_snapshot_generation`.
10. Retire samples only when older than the predecessor required by both active
    and newest pending generations.

The first output after a correction must have `t_p > t_c` and include at least one
new IMU sample. A correction itself does not cause a propagated message. This avoids
duplicating `/Odometry` at the LiDAR timestamp.

Correction continuity is evaluated at a common IMU tail timestamp: separately
propagate the old generation and the new posterior generation to the captured
watermark, then compare `pos`, manifold rotation difference and world velocity.
Covariances need not be continuous because LiDAR legitimately reduces uncertainty,
but both must be valid. A state discontinuity beyond explicit limits enters
`FAULT`; it is never hidden by blending, interpolation or restamping.

Generation bookkeeping is explicit and monotonic:

```text
latest_snapshot_generation  highest accepted LiDAR posterior generation
active_generation           generation currently eligible to publish
pending_generation          newer generation being replayed, or none
candidate_generation        generation attached to each immutable candidate
published_generation        generation of the last actual message
published_imu_sequence      global last-published sequence across generations
```

Generation numbers never reset during recovery. A candidate is publishable only
if it belongs to `active_generation`, its lineage is valid, and its IMU sequence
and timestamp both exceed the global published pair.

### Frozen first partial-IMU-interval algorithm

The posterior at `t_c` already contains every main-filter prediction through
`lidar_end_time`, including the final `predict(abs(lidar_end_time-imu_end_time),
Q,last_normalized_input)` performed by `UndistortPcl()`. Therefore
`last_normalized_input` is snapshot audit/parity evidence only; it is never applied
again after `t_c`.

For the independent predictor, choose `head` as the newest accepted corrected-time
IMU sample with `head_time <= t_c`, and `tail` as the oldest accepted sample with
`tail_time > t_c`. Construct midpoint gyro and acceleration from that complete raw
pair exactly as the source does, normalize midpoint acceleration by
`G_m_s2/mean_acc.norm()`, and integrate only `dt = tail_time - t_c`. This exactly
matches the source boundary branch `head_stamp < last_lidar_end_time_` while
avoiding re-integration of the already corrected interval. After the first tail,
normal intervals use the previous integrated tail as head and
`dt = tail_time - head_time`.

| Case | Head/tail or last input | Input and `dt` | Context-only samples | Newly integrated sequence | No omission/duplication rule |
|---|---|---|---|---|---|
| `t_c` equals IMU stamp `S_k` | `head=S_k`, `tail=first S_j>t_c`; never reuse last normalized input | midpoint `(S_k,S_j)`; normalize acceleration; `dt=t_j-t_c` | `S_k` and all earlier samples | `S_j` only | `S_k` anchors the midpoint but is not reintegrated; next pair starts at `S_j` |
| `head_time < t_c < tail_time` | newest `head<=t_c`, oldest `tail>t_c` | full-pair midpoint exactly like `UndistortPcl()`; `dt=tail_time-t_c` | head and every sample `<=t_c` | tail only | truncated `dt` matches `head_stamp < last_lidar_end_time_`; posterior already owns the head-to-`t_c` interval |
| `t_c` later than current latest IMU | retain newest sample `head<t_c`; no tail yet; last normalized input is audit-only | no new input and no `predict()` until a `tail>t_c` arrives; then midpoint `(head,tail)`, `dt=tail_time-t_c` | head and scan-end last input | none until tail arrives | posterior already contains extrapolation to `t_c`; waiting cannot omit or duplicate integration |
| New `tail>t_c` but no legal predecessor | no valid pair | construct nothing; `dt` undefined; enter `FAULT` before prediction | tail is rejected from replay use | none | never approximate with tail alone, zero-order hold, wall time, or last normalized input |
| New IMU arrives during LiDAR correction/snapshot | insertion and snapshot are independently ordered; worker captures its watermark only after snapshot publication | if inserted by the watermark: stamp `<=t_c` may update predecessor context, stamp `>t_c` is replayed; if inserted later it is handled by the next worker pass | every accepted sample `<=t_c` | each accepted tail `>t_c`, exactly once | accepted sequence plus watermark partitions samples; no sample can belong to both replay passes or neither pass |

If a sample exists exactly at `t_c` after a predecessor, the source can form a
zero-duration boundary pair. The independent design does not call `predict(0)` and
does not count that sample as newly integrated; it becomes the canonical head for
the next strictly newer tail. This is state/covariance equivalent and preserves the
source's next midpoint pair without producing a duplicate timestamp.

## 6. Propagation model

The normative implementation is the current IKFoM `predict(dt,Q,in)`, not the
following explanatory Euclidean approximation. For consecutive corrected-time IMU
samples, current `UndistortPcl()` computes:

```text
gyro_mid = 0.5 (gyro_head + gyro_tail)
acc_mid  = 0.5 (acc_head  + acc_tail)
acc_in   = acc_mid * G_m_s2 / mean_acc.norm()
in.gyro  = gyro_mid
in.acc   = acc_in
```

`get_f()` then applies bias and gravity on the manifold:

```text
p_dot     = v
theta_dot = gyro_mid - b_g
v_dot     = R_wb (acc_in - b_a) + g_w
```

Extrinsics and biases have zero nominal derivative; gravity remains on S2. The
actual state step is `x.oplus(f, dt)`, so no hand-written quaternion/Euler update is
acceptable as the implementation of record.

The exact current error covariance step is:

```text
A = F_x1_manifold + f_x_final * dt
B = dt * f_w_final
P_next = A * P * A^T + B * Q * B^T
```

Here `F_x1_manifold` includes IKFoM SO3 and S2 retractions. `df_dx()` couples
position error to velocity (`0:2 <- 12:14`), velocity to attitude, accel bias and
the two-dimensional gravity error, and attitude to gyro bias. `df_dw()` maps gyro,
acceleration, gyro-bias and accel-bias noise in the exact 12-dimensional ordering.
The current implementation therefore applies `Q` through `dt*f_w_final` on both
sides; replacing it with a generic continuous-discrete `GQG^T*dt` changes the
filter and is forbidden in the parity phase.

Use a private IKFoM object with the copied full `23 x 23 P`; copying only selected
blocks loses the cross terms required by propagation. Unit tests must establish
step-for-step parity with `UndistortPcl()` for state and every covariance element.

Reject non-finite input, `dt <= 0`, `dt` beyond the configured IMU-gap limit,
non-finite state/covariance, asymmetric covariance beyond tolerance, negative
eigenvalues beyond numerical tolerance, or a failed generation transition.
Symmetrization may remove floating-point skew via `(P + P^T)/2`, but must not mask
a materially indefinite covariance.

## 7. Message mapping and covariance

Publish `nav_msgs/msg/Odometry` from the predicted IMU/body state as follows:

| Field | Mapping |
|---|---|
| `header.stamp` | exact timestamp of the last integrated IMU sample |
| `header.frame_id` | `camera_init` |
| `child_frame_id` | `body` |
| pose position | `p_w`, in `camera_init` world coordinates |
| pose orientation | quaternion from `R_wb` |
| twist linear | `v_b = R_wb^T v_w`, body FLU |
| twist angular | NaN until a synchronized, explicitly accepted estimate is designed |
| pose covariance | ROS `[position, rotation]` 6x6 from error-state indices `pos=0:2`, `rot=3:5`, preserving cross terms |
| twist linear covariance | `P_velocity_body = J_v P J_v^T` in body FLU |
| angular covariance | existing conservative finite unknown-angular-rate values |

### Covariance Jacobian 1: ESKF error state to ROS pose

IKFoM uses right perturbations: `SO3::boxplus()` applies
`R_true = R_nominal Exp(delta_theta_body)`. ROS pose rotational covariance is in
fixed `camera_init` axes, so for small errors
`delta_theta_world = R_wb delta_theta_body`. Define:

```text
J_pose[:, :]       = 0                  # 6 x 23
J_pose[0:3, 0:3]   = I3
J_pose[3:6, 3:6]   = R_wb
C_pose             = J_pose P J_pose^T
```

This transforms the attitude block and both position/attitude cross blocks. If a
downstream convention test instead proves that local/right rotational covariance
is required, that is a separately gated convention decision, never an implicit
copy of `P(3:6,3:6)`.

Audit finding: current `publish_odometry()` uses `k = i < 3 ? i + 3 : i - 3`
and swaps position/rotation row and column blocks while filling ROS pose
covariance. That mapping must not be copied into the propagated publisher without
a separate convention proof. The propagated design uses the standard ROS
`[position, rotation]` ordering and explicit `J_pose` above. A later implementation
must add a test that exposes any disagreement with the unchanged legacy
`/Odometry` mapping. This document does not authorize changing the legacy path.

### Covariance Jacobian 2: ESKF error state to body-FLU linear velocity

For `v_b = R_wb^T v_w` and the same right attitude perturbation:

```text
delta_v_b = skew(v_b) delta_theta_body + R_wb^T delta_v_world

J_v[:, :]       = 0                     # 3 x 23
J_v[:, 3:6]     = skew(v_b)
J_v[:, 12:15]     = R_wb^T
P_velocity_body   = J_v P J_v^T            # formal published mapping
```

The velocity-only contribution is exactly
`R_wb^T P(12:15,12:15) R_wb`. The full Jacobian additionally retains attitude
uncertainty and attitude/velocity cross-covariance. A finite-difference test using
IKFoM right `boxplus()` must lock the `skew(v_b)` sign.

Angular velocity remains unknown/NaN, so only the linear 3x3 principal twist block
uses `P_velocity_body`; linear/angular cross blocks stay zero and angular diagonals
retain the existing finite conservative unknown value. Do not derive angular-rate
covariance from attitude covariance.

The complete twist covariance transform must also preserve valid linear/angular
cross terms if angular velocity later becomes available. It is forbidden to rotate
only velocity while leaving `Pvv` in world coordinates. Before publishing, require
all published covariance entries to be finite and symmetric within tolerance. Test
PSD on the complete `23 x 23 P` and separately on each published principal block;
require positive diagonal for measured components. Symmetrization may remove only
floating-point skew. Because rotations, S2 retraction and cross coupling can make
an individual diagonal non-monotonic, require overall process-noise-driven
uncertainty growth statistically across correction intervals, not strict
sample-by-sample growth of every diagonal.

## 8. Thread model and lock ranges

- **Main mapping callback:** after LiDAR correction, by-value copy `state_ikfom`,
  `23 x 23 P`, propagation context, predecessor metadata and generation. The main
  ESKF remains executor-owned and is never shared with the worker.
- **IMU callback:** after applying the existing timestamp correction, preserve the
  unchanged main buffer operation and append a raw sample copy to the independent
  deque. No normalization, prediction or publication occurs in the callback.
- **Propagation worker:** owns two private predictor slots, `active` and optional
  `pending`. It advances active for every new IMU, replays pending from its
  posterior, then catches pending up to the same watermark. All prediction is
  off-lock. A new snapshot replaces only pending, never active. One worker event
  loop owns both IKFoM objects, so no IKFoM object is accessed concurrently.
- **Rate-limit timer (20-33 ms):** reads the atomic candidate. It publishes only
  when candidate generation is current, IMU sequence and corrected timestamp are
  newer than the last output, at least one `predict()` step occurred after the
  correction/switch, and the 30-50 Hz interval elapsed. Otherwise it does nothing.
- **Watchdog:** evaluates IMU age, correction age, replay progress, buffer bounds
  and state validity. It changes health state but never fabricates a sample.

The atomic candidate is one immutable object containing `{generation,
parent_generation, imu_sequence, timestamp, x, P, validation_epoch}`; state,
covariance and metadata never use separate atomics. Immediately before publication,
the timer rechecks active generation and the global published pair. A generation
switch racing with a timer read makes that read ineligible rather than publishable.

FAST-LIO currently builds as C++14. Store the immutable candidate in
`std::shared_ptr<const Candidate>` and use the C++11/C++14 free functions
`std::atomic_load_explicit(&candidate_ptr, std::memory_order_acquire)` and
`std::atomic_store_explicit(&candidate_ptr, ..., std::memory_order_release)`.
Do not use C++20 `std::atomic<std::shared_ptr<T>>`. If the actual toolchain does not
provide the required free-function specialization, use one dedicated bounded
candidate mutex whose critical section is only shared-pointer copy/swap and final
eligibility metadata; never hold it during prediction, validation or DDS publish.

No lock is held across main ESKF iteration, private `predict()`, replay, covariance
decomposition, message publication, logging or DDS calls. The worker never invokes
`change_x`, `change_P`, `predict` or `get_P` on the main `kf`. Snapshot and IMU
buffer mutexes are never nested; generation/watermark data are copied in separate
short critical sections and reconciled off-lock.

## 9. State machine and fail-closed behavior

Separate service health from per-generation phase:

```text
service_health: INITIALIZING | HEALTHY | FAULT       # FAULT is latched
generation_phase: EMPTY | REPLAYING | READY | ACTIVE | CANCELLED | REJECTED
```

- No output while service health is `INITIALIZING` or `FAULT`.
- Startup enters `HEALTHY` only after one generation reaches `ACTIVE` and
  integrates an IMU newer than its correction.
- In normal `HEALTHY`, a new generation may be `REPLAYING` while the old one
  remains `ACTIVE` and continues producing candidates. Pending replay is not a
  global no-output condition.
- Pending becomes `READY` only at the active watermark and after all validation.
  One committed transition changes old `ACTIVE -> CANCELLED`, pending
  `READY -> ACTIVE`, and updates `active_generation`.
- A newer snapshot changes an older pending `REPLAYING/READY -> CANCELLED` and
  creates a new `REPLAYING` generation without demoting active service health.
- Failure isolated to an already stale/cancelled pending generation records
  `REJECTED` but does not fault a valid active path. Shared-input corruption,
  active prediction failure, correction-freshness failure, or a lineage invariant
  violation enters service `FAULT`.
- Enter `FAULT` on IMU timeout, LiDAR correction timeout, duplicate/backward
  corrected IMU timestamp, missing midpoint predecessor, `mean_acc.norm()` invalid
  or too small, non-positive/excessive `dt`, output timestamp regression,
  required-buffer overflow, replay gap/failure, a stale generation actually
  committing or any active-lineage invariant violation, non-finite state or `Q`,
  invalid `23 x 23 P`, invalid rotated published covariance, excessive correction
  discontinuity, or worker overrun. An expected compare-and-swap rejection of a
  cancelled pending generation is not itself a service fault.
- Recovery requires a new valid LiDAR posterior, a clean replay from its timestamp,
  fresh IMU samples, covariance validation, and a configurable consecutive-healthy
  interval. Recovery creates a new generation lineage and never resumes the faulted
  predictor. Its first output must exceed the pre-fault global timestamp and
  sequence.

### Frozen FAULT invalidation and recovery policy

On every service `FAULT`, atomically set health first, atomically store a null
candidate, mark active and pending generations `CANCELLED`, clear their publish
eligibility and retention claims, and stop timer output. Preserve only the
node-lifetime monotonic guard `{last_published_timestamp,
last_published_accepted_sequence, highest_generation}` and fault diagnostics.
Neither active state, pending state nor a candidate may be reused for recovery.

For recoverable data/timeout faults, clear the independent IMU buffer and collect
new valid samples into a non-publishable recovery buffer. A new valid posterior
must supply its canonical finite predecessor with `predecessor_time <= t_c`; seed
that predecessor as context, then replay only accepted tails `>t_c`. If no legal
predecessor is available, remain `FAULT`. For hard time-epoch/integrity faults,
clear the buffer and do not accept a recovery posterior until the propagator is
explicitly restarted; ingress may be observed only for diagnostics.

| Fault class | Examples | Buffer action | Recovery authority |
|---|---|---|---|
| Automatic with new posterior | IMU timeout without clock jump, LiDAR timeout, pending replay numerical failure, invalid posterior/Q/`mean_acc.norm()`, continuity rejection, worker-budget overrun | clear; rebuild from posterior predecessor plus later clean samples | new valid posterior, clean replay and full `recovery_healthy_s` |
| Propagator restart required | duplicate/backward corrected stamp, ROS clock jump, LiDAR/IMU epoch mismatch, future-stamp violation indicating epoch error, required predecessor/replay loss, buffer overflow, committed stale generation or lineage invariant failure | clear and quarantine; no automatic replay | explicit propagator restart after time source/integrity cause is corrected |
| Non-recoverable within current publication epoch | every new valid IMU stamp is `<= last_published_timestamp`, including a restarted/lower sensor epoch | clear; never rewrite stamps or reset global guard | remain `FAULT`; only an explicitly new external session with no continuity claim may reset the node-lifetime publication guard |

A single non-finite sample is never buffered. It triggers `FAULT`, but may use the
automatic-new-posterior path if subsequent timestamps prove the same monotonic
epoch and all recovery inputs are finite. Duplicate/backward stamps always require
restart because the corrected-time ordering is ambiguous.

Recovery increments `highest_generation`; generations are never reused. Accepted
sequence numbers remain monotonic for the node lifetime and are not reset when the
buffer is cleared. Before `HEALTHY`, require the new generation, every replay tail,
candidate timestamp and candidate accepted sequence to exceed their relevant
predecessors; before the first recovered publish, require timestamp and accepted
sequence both strictly exceed the preserved global published pair. Otherwise stay
`FAULT` without restamping, synthetic sequence advancement or epoch substitution.

### Proposed configurable defaults

These are future propagator defaults, not current ROS/PX4 parameter changes:

| Parameter | Default | Basis |
|---|---:|---|
| `imu_timeout_s` | `0.05 s` | equals the acceptance maximum source age; missing IMU cannot be hidden beyond the `<50 ms` gate |
| `lidar_correction_timeout_s` | `0.30 s` | about three 10 Hz scans and no looser than the existing `0.3 s` anomaly-to-fault window |
| `max_imu_dt_s` | `0.020 s` | requires at least 50 Hz fresh IMU support and bounds one-step extrapolation below the `<40 ms` lag target |
| `future_tolerance_s` | `0.050 s` | matches the existing EV health future-stamp allowance; not relaxed |
| `ros_clock_jump_tolerance_s` | `0.050 s` | same time-error allowance; compares ROS delta against steady-clock delta |
| `imu_buffer_span_s` | `1.0 s` | exceeds LiDAR timeout plus replay/recovery margin while remaining bounded |
| `imu_buffer_max_count` | `512` | bounds memory and covers 1 s at up to 500 Hz; count overflow fails closed |
| `continuity_position_m` | `0.05 m` | one-third of existing `0.15 m` EV position-jump protection |
| `continuity_attitude_deg` | `5.0 deg` | conservative propeller-off discontinuity gate; must be tightened from evidence, never loosened silently |
| `continuity_velocity_mps` | `0.20 m/s` | stricter than existing `0.45/0.50 m/s` velocity consistency limits |
| `worker_budget_s` | `0.010 s` | fits below a 20 ms high-rate interval and leaves margin for the `<25 ms` p95 age target |
| `recovery_healthy_s` | `2.0 s` | matches the current launch health-recovery default; does not shorten it |

Startup must reject non-positive limits, a publish interval outside 20-33 ms,
buffer count/span inconsistent with timeouts, or continuity thresholds looser than
the table unless a separately reviewed validation profile explicitly tightens or
justifies them. PX4 parameters remain untouched.

Diagnostics must expose generation, correction timestamp/age, corrected predecessor
and last-integrated IMU timestamps/sequences, `mean_acc.norm()`, four `Q` diagonal
vectors, each propagation `dt`, replay sample count/duration, buffer occupancy/span,
publish rate, skipped-by-rate-limit count, full-`P` symmetry error/eigenvalue bounds,
published-block bounds, correction continuity deltas, fault reason and recovery.
Also expose active/pending/last-published generations, parent lineage, generation
phase, cancellation/rejection counts, and the exact predicate that allowed or
denied each switch.

## 10. Isolation from the flight-control chain

The initial launch configuration may start the publisher only behind an explicit
development flag defaulting to false. `/Odometry/propagated` is recorded and
analyzed directly. It must not feed `/Odometry/healthy`, the MAVROS odometry bridge,
`/mavros/odometry/out`, or `/fmu/in/vehicle_visual_odometry` during development.
`EKF2_EV_CTRL` remains `11` throughout all software and propeller-off acceptance.
Changing the source of the current healthy chain is a separate, later decision
requiring explicit acceptance evidence and rollback controls.

## 11. Expected files for a later implementation

FAST-LIO workspace:

- `src/fast_lio/src/laserMapping.cpp`: minimal snapshot hook, copied IMU feed,
  publisher/worker ownership and opt-in parameters; existing `/Odometry` path stays
  untouched.
- `src/fast_lio/include/high_rate_ev_propagator.hpp`: immutable snapshots, buffer,
  state machine, worker API and message mapping declarations.
- `src/fast_lio/src/high_rate_ev_propagator.cpp`: independent predictor, replay,
  validation, atomic switching and diagnostics.
- `src/fast_lio/CMakeLists.txt` and package metadata: build/test registration only.
- `src/fast_lio/test/test_high_rate_ev_propagator.cpp`: deterministic unit tests.
- Relevant FAST-LIO launch/config file: opt-in enable flag and safety timeouts;
  default disabled.

Offboard-control workspace, only after the raw publisher passes unit testing:

- a new isolated validation launch under `src/px4_ros_com/launch/` that records
  `/Odometry`, `/Odometry/propagated`, raw IMU and diagnostics without connecting
  propagated odometry to PX4;
- a new analyzer and acceptance test under `src/px4_ros_com/test/` or validation
  tooling; current health monitor and bridge remain unchanged in the first phase.

## 12. Unit and integration tests

Unit tests use synthetic timestamps and IMU sequences without ROS graph startup:

1. **Snapshot parity:** copy all `state_ikfom` members, every element of the
   `23 x 23 P`, `mean_acc.norm()`, four `Q` vectors, predecessor, normalized last
   input, timestamps and generation; mutate the source afterward and prove the
   snapshot is immutable.
2. **Predict parity:** constant zero motion, acceleration and yaw-rate cases match
   current `UndistortPcl()`/IKFoM state and every `P` element step-for-step,
   including first partial interval from `t_c`, acceleration normalization and the
   actual `(dt*f_w) Q (dt*f_w)^T` convention.
3. **State ordering:** inject one nonzero component at each of the 23 error indices
   and verify `pos`, `rot`, extrinsics, `vel`, `bg`, `ba` and two-DOF S2 gravity
   mappings; explicitly prove there is no gravity covariance index 23.
4. **Propagation covariance:** non-diagonal PSD `P` with cross terms remains finite,
   symmetric and PSD after prediction; injected invalid `Q/P` fails closed.
5. **Pose Jacobian analytic test:** construct non-identity 3D `R`, nonzero
   position/attitude cross covariance and verify
   `C_pose = J_pose P J_pose^T`, including both cross blocks and ROS row-major
   placement.
6. **Pose Jacobian finite difference:** perturb each of 23 IKFoM error axes using
   `boxplus(epsilon e_i)`, compute the numerical derivative of world position and
   fixed-axis orientation residual
   `Log(R_perturbed R_nominal^T)`, and compare every `J_pose` column to the analytic
   Jacobian for positive and negative epsilon.
7. **Velocity Jacobian analytic test:** with nonzero velocity, non-identity 3D
   rotation and correlated attitude/velocity covariance, verify
   `J_v[theta]=skew(v_b)`, `J_v[vel]=R^T`, the formal published
   `P_velocity_body=J_v P J_v^T`, and separately
   the contained `R^T Pvv_world R` contribution.
8. **Velocity Jacobian finite difference:** apply IKFoM right attitude and additive
   world-velocity perturbations, numerically differentiate `R^T v_w`, and verify
   every `J_v` column and the skew sign. A test with zero `Pvv` but nonzero
   attitude covariance must still produce the predicted velocity covariance when
   `v_b != 0`.
9. **Covariance Monte Carlo check:** draw small 23-dimensional errors from a known
   PSD `P`, map through pose and body velocity, and require empirical covariance to
   agree with both Jacobian predictions within deterministic tolerance.
10. **Initial generation:** `EMPTY -> REPLAYING -> READY -> ACTIVE` requires a
    complete replay and at least one IMU newer than correction; correction time
    alone is never publishable.
11. **Seamless pending replay:** while generation N remains active and advances,
    N+1 replays independently; before READY all candidates are from N, then exactly
    one atomic switch occurs at a common watermark.
12. **Global monotonicity across switch:** N+1 at the same or older sequence/stamp
    than the last N publication is suppressed; first N+1 publication advances both
    values and no timer tick can publish twice.
13. **Superseding correction:** N+2 arriving while N+1 is REPLAYING or READY marks
    N+1 CANCELLED. Completion of N+1 cannot switch active; only N+2 with parent N
    and matching latest-snapshot generation can switch.
14. **Stale completion race:** pause N+1 immediately before compare-and-swap,
    activate/cancel another generation, resume N+1, and prove it is discarded
    without changing candidate, active generation or service health.
15. **Pending-local rejection:** corrupt an already superseded pending replay and
    verify REJECTED/counting without faulting valid active N. Corrupt active input,
    shared buffer ordering or latest required replay and verify latched `FAULT`.
16. **FAULT recovery lineage:** after fault, old active and candidates are never
    reused. A new posterior/replay/healthy interval creates a higher generation;
    its first publication exceeds the pre-fault global sequence and timestamp.
17. **Buffer retention by generation:** predecessor/replay samples remain while
    claimed by active or pending and are released after cancellation/switch; forced
    required-sample eviction faults before prediction.
18. **Timer and immutable candidate race:** switch generation between timer load
    and final eligibility check; assert skip, never mixed state/covariance metadata
    and never publication from a non-active generation.
19. **Fault matrix:** duplicate/backward corrected timestamp, IMU/LiDAR timeout,
    invalid `mean_acc.norm()`, excessive `dt`, overflow, replay gap, NaN/Inf,
    indefinite covariance, continuity violation and worker overrun each produce the
    exact expected state/phase/reason and suppress output.
20. **Message contract:** verify frames, exact final IMU stamp, body-FLU velocity,
    NaN angular rate, conservative angular covariance, zero linear/angular cross
    blocks, and finite symmetric PSD pose/linear-velocity blocks.
21. **Legacy non-regression:** existing `/Odometry` fields, rate and timestamps are
    byte/semantic golden-tested with the feature disabled and behavior-tested with
    it enabled; the documented legacy pose-covariance permutation is observed but
    not silently changed by this work.
22. **First-partial truth table:** execute all five frozen cases, including exact
    `t_c`, straddling pair, `t_c` newer than latest IMU, missing predecessor and IMU
    arrival on both sides of the replay watermark. Assert exact head/tail, normalized
    input, `dt`, context/new sequence classification and one-and-only-one integration.
23. **Zero-duration source boundary:** predecessor plus tail exactly at `t_c`
    performs no independent `predict(0)` and emits nothing, but that tail becomes
    the next canonical head; result after the following tail matches the source
    boundary behavior.
24. **Ingress rejection:** NaN/Inf payload or stamp, duplicate/backward corrected
    timestamp, wrong epoch and excessive future stamp never enter either buffer,
    never increment accepted sequence and never become midpoint context.
25. **Clock/epoch matrix:** normal ROS/steady progression passes; positive and
    negative ROS jumps, future age beyond tolerance, LiDAR/IMU epoch mismatch and
    a restarted lower timestamp epoch produce the specified hard `FAULT` without
    restamp. IMU timeout with stable clocks follows automatic-posterior recovery.
26. **FAULT invalidation/recovery:** at each fault barrier prove candidate becomes
    null before any later timer publish, active/pending lose eligibility, recovery
    buffer is rebuilt only from a new posterior predecessor, generation and accepted
    sequence increase, and the global published pair is never reset.
27. **C++14 candidate exchange contract:** compile-time/API test selects free-function
    `atomic_load/atomic_store(shared_ptr)` or the bounded mutex fallback, rejects
    dependence on C++20 `atomic<shared_ptr>`, and race tests verify acquire/release
    readers observe one complete immutable candidate.

Deterministic concurrency tests use barriers at snapshot load, replay watermark,
READY commit, candidate atomic exchange and timer eligibility recheck. Randomized
stress tests then interleave corrections, IMU arrival, cancellations, timer reads
and watchdog faults under ThreadSanitizer/AddressSanitizer. Assert no deadlock,
data race, torn snapshot, reordered sequence, duplicate publication, stale switch,
generation rollback or post-FAULT output.

## 13. Offline replay plan

Use a newly captured, explicitly scoped raw LiDAR+IMU dataset; do not reuse old
flight logs for acceptance. Run the unchanged FAST-LIO path and independent
propagator from identical inputs, recording `/Odometry`, `/Odometry/propagated`,
IMU and diagnostics.

Evaluate:

- output rate and inter-message interval distribution;
- exact membership of each output stamp in the input IMU stamps;
- zero duplicate/backward stamps and at least one new IMU sequence per output;
- source age p50/p95/max using the IMU sample stamp, without restamping;
- correction continuity by propagating pre/post-correction generations to a common
  IMU timestamp;
- static velocity distribution and six-direction/body-frame signs;
- per-axis correlation and best lag against time-aligned position difference;
- covariance finiteness, symmetry error, minimum eigenvalue, dynamic range and
  growth between corrections;
- CPU, replay time, lock time, buffer high-water mark and unchanged `/Odometry`
  contents/rate/timestamps.

Offline pass gates: 30-50 Hz, no duplicate/backward timestamp, p95 source age below
25 ms and maximum below 50 ms, no visible correction jump, static speed p95 below
0.03 m/s, per-axis velocity/position-difference correlation above 0.9, best apparent
lag below 40 ms, valid dynamic covariance, and no `/Odometry` regression.

## 14. Propeller-off acceptance

With propellers removed, vehicle disarmed, no OFFBOARD/control publisher, and
`EKF2_EV_CTRL=11`, run only the sensor/FAST-LIO stack plus the isolated recorder.
Do not connect `/Odometry/propagated` to MAVROS or PX4.

Acceptance requires:

- sustained real output rate 30-50 Hz with no repeated state or stamp;
- source-age p95 `<25 ms`, maximum `<50 ms`;
- zero duplicate/backward stamps and every output tied to a newly integrated IMU;
- no observable position, attitude or velocity discontinuity at LiDAR corrections;
- static speed norm p95 `<0.03 m/s`;
- expected signs for six translations and yaw-90 body-forward;
- each velocity axis correlation with properly time-aligned position difference
  `>0.9`, and best apparent lag `<40 ms`;
- covariance finite, symmetric, PSD, dynamic and reasonably growing between LiDAR
  corrections, including correct rotated non-diagonal `Pvv`;
- injected IMU/LiDAR timeout, backward timestamp and buffer/replay faults suppress
  output and produce the exact `FAULT` reason;
- current `/Odometry` message fields, stamps, covariance, rate and behavior show no
  regression relative to the same run.

Only after these gates pass may a separate design consider feeding the propagated
topic into a duplicated health-validation path. PX4 EV velocity fusion remains out
of scope and `EKF2_EV_CTRL` remains `11`.

## 15. Principal risks

- **Estimator divergence:** a predictor that differs subtly in IMU scaling,
  midpoint timing, bias, gravity, noise or manifold math will drift from FAST-LIO.
- **Correction race:** replaying against a stale generation can create backward
  jumps or overwrite a newer posterior unless generation switching is atomic.
- **False low latency:** using wall time or reusing the last IMU state would meet a
  nominal topic rate while publishing stale information; exact IMU stamps and
  new-sequence enforcement are mandatory.
- **Covariance inconsistency:** anything other than the formal
  `P_velocity_body = J_v P J_v^T` mapping omits attitude/velocity cross terms;
  `R^T Pvv R` alone is only one contribution.
- **CPU/locking regression:** matrix copies and replay can delay the 10 Hz mapping
  path; all expensive work must be off the main ESKF and off locks.
- **Buffer loss:** an undersized buffer or slow replay can make correction recovery
  impossible; fail closed rather than extrapolate across missing IMU.
- **Physical reference point:** the state is the FAST-LIO IMU/body state, not yet a
  measured vehicle control-center reference; no unmeasured lever arm may be added.

## 16. Phased implementation plan

1. **Model parity:** isolate and test the independent predictor against FAST-LIO's
   existing `predict()` over deterministic IMU sequences; no publisher.
2. **Snapshot and replay:** add immutable full-posterior snapshots, copied IMU
   buffer, generation switching, FAULT state and diagnostics; default disabled.
3. **Raw publisher:** add `/Odometry/propagated`, exact IMU stamps, body-FLU twist
   and full covariance mapping; keep it disconnected from all PX4 paths.
4. **Automated verification:** unit, race/sanitizer and current `/Odometry`
   non-regression tests.
5. **New offline replay:** collect a fresh scoped dataset and pass all timing,
   continuity, velocity and covariance gates.
6. **Propeller-off isolation test:** pass the stated acceptance criteria while
   disarmed and still disconnected from PX4.
7. **Later integration review:** only with explicit authorization, design a parallel
   propagated health path and rollback; do not alter `EKF2_EV_CTRL` in this plan.

## Unique next step

Perform one design-only implementation-readiness sign-off against the frozen
partial-interval truth table, FAULT/epoch recovery matrix, C++14 exchange contract,
parameter defaults, two covariance Jacobians and tests 1-27. Only a later explicit
authorization may begin code; until then no thread, publisher or `laserMapping.cpp`
integration is permitted.
