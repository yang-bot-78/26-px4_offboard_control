# Phase 4 high-rate EV ROS runtime validation report

Date: 2026-07-24

Collection: `/home/robot/ws_offboard_control/phase4_operator_records/phase4_20260724_162737`

Rosbag root: `/home/robot/ws_offboard_control/phase4_operator_records/phase4_20260724_162737/bags`

## Scope and evidence integrity

- The collection directory was analyzed read-only and was not modified.
- Every entry in the collection's `EVIDENCE.sha256` verified successfully.
- The Phase 3.5 checkpoint verified before and after collection.
- Preflight found no MAVROS, PX4, OFFBOARD, old Livox/FAST-LIO, or rosbag process.
- The bags contain no `/Odometry/healthy`, `/mavros/odometry/out`, MAVROS, PX4, or control topic.
- The collector did not change `EKF2_EV_CTRL` and recorded `MAVROS_PX4_ALLOWED=NO`.
- The session stopped with `exit_code=1` during the IMU fault test, as required by
  the fail-and-stop policy. LiDAR recovery and three-cycle restart testing were
  consequently not executed.

Independent analysis used `rosbag2_py` deserialization, direct 6x6 NumPy
eigendecomposition, exact integer timestamp set membership, and an independent
position-difference/body-rotation velocity oracle. It did not call FAST-LIO
conversion or covariance functions.

## Result summary

| Area | Result | Evidence |
|---|---|---|
| Default-off runtime | PASS | 67.18 s bag contains 672 `/Odometry`, no propagated or diagnostics topic |
| Legacy `/Odometry` rate | PASS | 10.0000 Hz in comparable A/B 60 s windows |
| Propagated rate | **FAIL** | 20.362 Hz; required 30–50 Hz |
| Source age | PASS | p95 14.352 ms, max 49.058 ms; required p95 <25 ms, max <50 ms |
| Timestamp ordering | PASS | 5,453 messages, zero duplicate/backward stamps |
| Exact IMU integer timestamp | **FAIL** | only 22/5,453 propagated stamps exactly match recorded IMU stamps |
| Diagnostic sequence ordering | PASS | zero non-strict candidate or published updates |
| Frames | PASS | only `camera_init` / `body` |
| Static speed | PASS | 60 s p95 0.02143 m/s; final-static p95 0.02966 m/s; required <0.03 m/s |
| Six-direction/body-FLU signs | PASS | all six translations and post-Yaw body-forward signs are correct |
| Velocity correlation/lag | **FAIL** | best correlation 0.80–0.82 with 85–95 ms lag; required >0.9 and <40 ms |
| Covariance and quaternion | PASS | finite, symmetric, PSD, dynamic; angular NaN and variance 1e6 correct |
| FAULT output gating | PASS for observed fault | zero propagated messages after first FAULT diagnostic |
| Fault/recovery matrix | **INCOMPLETE/FAIL** | C aborted; no recovery, LiDAR timeout, or three-cycle restart evidence |
| Resource/legacy impact | CONDITIONAL | +1 thread, +3.08 CPU percentage points; legacy timing stable but source-age p95 increased |

## Default-off baseline

The A bag contains:

- `/Odometry`: 672 messages at 10.00008 Hz receive rate;
- `/livox/imu`: 13,438 messages;
- `/livox/lidar`: 672 messages;
- `/Odometry/propagated`: absent;
- `/high_rate_ev/diagnostics`: absent.

In the marker-bounded 60 s interval, legacy `/Odometry` was 10.00002 Hz,
with receive-interval p50/p95/max of 99.995/100.942/110.411 ms. No duplicate or
backward source timestamp was found.

The parameter evidence files say `Node not found` because the collection script
queried `/laserMapping` while the actual node was `/laser_mapping`. This prevents
direct parameter-dump confirmation, but the required runtime outcome—both opt-in
topics absent for the entire baseline—is directly established by the bag and
topic snapshot.

## Rate, latency, timestamp, and sequence

The enabled B bag contains 5,453 propagated messages over 267.78 s:

- receive rate: 20.36228 Hz;
- source-stamp rate: 20.36204 Hz;
- receive interval p50/p95/max: 49.996/75.024/2030.978 ms;
- source age p50/p95/max: 8.189/14.352/49.058 ms;
- duplicate/backward timestamps: 0.

The 2.031 s maximum interval coincides with an uncommanded `IMU_TIMEOUT` during
the B hand-test run. Diagnostics transitioned:

1. `HEALTHY/NONE`;
2. `FAULT/IMU_TIMEOUT`;
3. `RECOVERING/IMU_TIMEOUT`;
4. `HEALTHY/NONE` after approximately 2.0 s.

All 10,629 observed candidate updates and all 5,453 published updates advanced
both sequence and timestamp strictly. Every observed published diagnostic
timestamp corresponds to a propagated message.

Exact source timestamp membership fails:

- propagated: 22/5,453 exact IMU timestamp matches;
- diagnostic candidates: 43/10,629 exact matches;
- propagated nearest-IMU difference: p50 74 ns, p95 193 ns, max 245 ns;
- all propagated stamps are within 1 microsecond, but the acceptance rule requires
  exact original integer nanoseconds and permits no reconstruction/rounding.

This is consistent with sub-microsecond timestamp reconstruction or precision
loss and is not an acceptable substitute for exact IMU integer timestamps.

## Frame and velocity

Every propagated sample has:

- `frame_id=camera_init`;
- `child_frame_id=body`.

Body-frame displacement and mean twist signs agree with the operator motions:

| Motion | Principal start-body displacement | Mean body twist sign |
|---|---:|---:|
| Forward | x = +0.420 m | +x |
| Backward | x = -0.398 m | -x |
| Left | y = +0.377 m | +y |
| Right | y = -0.430 m | -y |
| Up | z = +0.339 m | +z |
| Down | z = -0.316 m | -z |
| Yaw | 87.63 degrees | rotation observed |
| Forward after Yaw | x = +0.323 m | +x body-FLU |

The 60 s static speed-norm p95 is 0.02143 m/s. After all movements, the final
static p95 is 0.02966 m/s, narrowly below the 0.03 m/s acceptance limit.

The independent position-difference oracle was evaluated using centered
100/200/300 ms windows and body rotation from the recorded quaternion. The best
300 ms results were correlations x/y/z = 0.803/0.815/0.818 with best lags
85/95/90 ms. These fail the required correlation >0.9 and lag <40 ms. The result
also fails with shorter windows, so no threshold was relaxed.

## Quaternion and covariance

Across all 5,453 B propagated samples:

- quaternion maximum norm error: `2.22e-16`;
- non-finite pose covariance samples: 0;
- non-finite twist covariance samples: 0;
- pose maximum symmetry error: `1.27e-21`;
- twist maximum symmetry error: `1.69e-20`;
- minimum pose eigenvalue: `6.34e-6`;
- minimum twist eigenvalue: `1.52e-4`;
- angular velocity not all NaN: 0;
- angular diagonal variance not 1e6: 0;
- nonzero linear/angular covariance cross blocks: 0;
- distinct pose covariance matrices: 5,453;
- distinct linear covariance matrices: 5,453.

The recorded covariance gate passes for finiteness, symmetry, PSD, angular
unknown encoding, cross-block isolation, and dynamic behavior.

## FAULT stop and recovery

After the C IMU gate was stopped, the bag shows:

- first FAULT: `IMU_TIMEOUT`;
- 24 ms later, while still FAULT, `fault_reason` changed to
  `MISSING_PREDECESSOR`;
- propagated messages after first FAULT: 0.

Thus observed FAULT output gating passed. However, the diagnostic snapshot loop
sampled the later `MISSING_PREDECESSOR` value and the collector stopped because it
could not observe the requested exact `IMU_TIMEOUT`.

The fault reason changing after entry into a latched FAULT weakens diagnostic
traceability and is inconsistent with retaining the initiating fault evidence.
More importantly, because collection stopped:

- IMU recovery and its first new candidate/published pair were not recorded;
- LiDAR timeout and recovery were not executed;
- recovery waiting and stale-candidate non-leakage were not established;
- three consecutive start/stop cycles were not executed.

The complete FAULT/recovery acceptance gate therefore fails.

## Resource and legacy impact

Comparable first-60-second measurements:

| Metric | A default off | B enabled |
|---|---:|---:|
| FAST-LIO threads | 15 | 16 |
| mean CPU | 33.50% | 36.58% |
| CPU p95 | 35.05% | 38.05% |
| mean RSS | 171,940 KiB | 171,056 KiB |
| max RSS | 185,868 KiB | 185,512 KiB |
| `/Odometry` rate | 10.00002 Hz | 9.99988 Hz |
| `/Odometry` receive jitter std | 0.935 ms | 0.600 ms |
| `/Odometry` source-age p95 | 10.330 ms | 15.004 ms |

Enabled diagnostics reported:

- worker queue maximum depth: 1;
- worker maximum processing time: 0.440 ms, below the 10 ms worker budget;
- publish/reject/drop/supersede/timeout maxima:
  5,500 / 25,202 / 753 / 0 / 21.

CPU increased by 3.08 percentage points and one worker thread appeared, while
aligned RSS and legacy receive jitter did not regress. Legacy source-age p95
increased by 4.67 ms. Because B ran four times longer, its eventual RSS maximum
of 252,696 KiB cannot be attributed solely to the worker. Resource impact is not
the decisive failure, but the legacy latency change should be retained as a
Medium observation.

## Findings

### Critical

None. No flight-control connection, arming, motor start, MAVROS, PX4, OFFBOARD,
or control publication occurred.

### High

1. Sustained propagated rate is 20.36 Hz, below the formal 30–50 Hz gate.
2. 99.6% of propagated stamps do not exactly equal a recorded IMU integer
   timestamp, despite being within 245 ns of one.
3. An uncommanded `IMU_TIMEOUT` caused a 2.03 s publication gap during the B run.
4. Independent velocity/position-difference correlation and lag fail the
   >0.9/<40 ms gates.
5. IMU recovery, LiDAR fault/recovery, stale-candidate recovery checks, and all
   three restart cycles are missing because the session correctly stopped on
   failure.

### Medium

1. The latched FAULT reason changed from `IMU_TIMEOUT` to
   `MISSING_PREDECESSOR`, obscuring the initiating cause for snapshot consumers.
2. Comparable legacy `/Odometry` source-age p95 increased from 10.33 ms to
   15.00 ms, although rate and receive jitter remained stable.
3. The default-off parameter query used the wrong node spelling, so direct
   parameter evidence is missing even though topic-level default-off behavior
   passed.

## Required final fields

`PHASE4_RUNTIME=FAIL`

`DEFAULT_OFF_RUNTIME=PASS`

`RATE_LATENCY=FAIL`

`TIMESTAMP_SEQUENCE=FAIL`

`FRAME_VELOCITY=FAIL`

`COVARIANCE=PASS`

`FAULT_RECOVERY=FAIL_INCOMPLETE`

`RESOURCE_IMPACT=CONDITIONAL`

`Critical=0`

`High=5`

`Medium=3`

`MAVROS_PX4_ALLOWED=NO`

`唯一下一步=停止Phase4并返回独立Phase3修正：精确保留IMU整数纳秒、达到30–50Hz、消除非注入FAULT与fault_reason覆盖、满足速度相关性/滞后；重新冻结和审签后方可再次明确授权Phase4。`
