# Propeller-off EV validation report

Bag: `/home/robot/rong_ws/ws_offboard_control/flight_records/20260812/flight_20260812_171323/rosbag`

## Safety evidence

- MAVROS armed=true samples: **0**
- MAVROS OFFBOARD samples: **0**
- Recorded control-topic message counts: `/mavros/setpoint_raw/local`=0, `/fmu/in/trajectory_setpoint`=0, `/fmu/in/vehicle_command`=0
MAVROS state transitions:

- `2026-08-12T17:14:46.268+08:00`: armed=false, mode=STABILIZED

## Timing and frequency

| Topic | Samples | Receive rate | Source rate | Non-monotonic stamps |
|---|---:|---:|---:|---:|
| `/livox/lidar` | 0 | nan Hz | nan Hz | 0 |
| `/livox/imu` | 0 | nan Hz | nan Hz | 0 |
| `/Odometry` | 359 | 10.00 Hz | 10.00 Hz | 0 |
| `/Odometry/healthy` | 186 | 5.14 Hz | 5.14 Hz | 0 |
| `/mavros/odometry/out` | 0 | nan Hz | nan Hz | 0 |
| `/mavros/local_position/odom` | 1048 | 30.06 Hz | 30.06 Hz | 0 |
| `/mavros/local_position/velocity_local` | 1049 | 30.09 Hz | 30.09 Hz | 0 |

### Recorded coverage

| Topic | First sample | Last sample | Span | Silence before bag end |
|---|---|---|---:|---:|
| `/Odometry` | 2026-08-12T17:14:45.226+08:00 | 2026-08-12T17:15:21.026+08:00 | 35.800s | 0.058s |
| `/Odometry/healthy` | 2026-08-12T17:14:45.058+08:00 | 2026-08-12T17:15:21.029+08:00 | 35.971s | 0.055s |
| `/mavros/vision_pose/pose_cov` | 2026-08-12T17:14:48.331+08:00 | 2026-08-12T17:15:21.029+08:00 | 32.699s | 0.055s |
| `/mavros/vision_speed/speed_twist_cov` | 2026-08-12T17:14:48.331+08:00 | 2026-08-12T17:15:21.029+08:00 | 32.699s | 0.055s |
| `/mavros/local_position/odom` | 2026-08-12T17:14:46.257+08:00 | 2026-08-12T17:15:21.084+08:00 | 34.827s | 0.000s |
| `/mavros/local_position/velocity_local` | 2026-08-12T17:14:46.257+08:00 | 2026-08-12T17:15:21.084+08:00 | 34.827s | 0.000s |
| `/race/odom` | 2026-08-12T17:14:45.052+08:00 | 2026-08-12T17:15:21.027+08:00 | 35.975s | 0.057s |
| `/race/control/status` | 2026-08-12T17:14:46.039+08:00 | 2026-08-12T17:15:21.074+08:00 | 35.035s | 0.010s |
| `/ev_health/status` | 2026-08-12T17:14:45.043+08:00 | 2026-08-12T17:15:21.041+08:00 | 35.998s | 0.043s |

## Strict coordinate/TF evidence

- Required topic counts: `/livox/lidar`=0, `/livox/imu`=0, `/Odometry`=359, `/race/odom`=361, `/tf`=0, `/tf_static`=0, `/mavros/vision_pose/pose_cov`=180, `/mavros/local_position/odom`=1048, `/mavros/local_position/velocity_local`=1049
- Sensor source stamp regressions: `/livox/lidar`=0, `/livox/imu`=0
- Dynamic `map -> base_link` found: **False**
- Static `base_link -> body`: **missing**

Source age from `/ev_health/diagnostics`: min=0.0087s, p50=0.0118s, p95=0.0248s, p99=0.0317s, max=0.0421s

## 60-second static velocity acceptance

- **INCOMPLETE:** STATIC_60S markers are missing.

## Health transitions

- Startup hysteresis: **not observable** (recording began after HEALTHY).
- Final recorded state: **HEALTHY** for at least **1.209s**; no later recovery is present in the bag.

Observed health state edges and reasons:

- `2026-08-12T17:14:45.043+08:00`: HEALTHY, reason=`ok`
- `2026-08-12T17:14:45.230+08:00`: SUSPECT, reason=`px4_velocity_unaligned` (px4_alignment_miss_s=0.167101)
- `2026-08-12T17:14:45.540+08:00`: FAULT, reason=`px4_velocity_unaligned` (px4_alignment_miss_s=0.467473)
- `2026-08-12T17:14:48.331+08:00`: HEALTHY, reason=`ok`
- `2026-08-12T17:14:49.230+08:00`: SUSPECT, reason=`px4_velocity_unaligned` (px4_alignment_miss_s=0.164881)
- `2026-08-12T17:14:49.534+08:00`: FAULT, reason=`px4_velocity_unaligned` (px4_alignment_miss_s=0.464881)
- `2026-08-12T17:14:54.430+08:00`: HEALTHY, reason=`ok`
- `2026-08-12T17:14:56.534+08:00`: SUSPECT, reason=`px4_velocity_unaligned` (px4_alignment_miss_s=0.165920)
- `2026-08-12T17:14:58.829+08:00`: FAULT, reason=`px4_velocity_unaligned` (px4_alignment_miss_s=0.400005)
- `2026-08-12T17:15:02.148+08:00`: HEALTHY, reason=`ok`
- `2026-08-12T17:15:07.329+08:00`: SUSPECT, reason=`px4_velocity_unaligned` (px4_alignment_miss_s=0.149244)
- `2026-08-12T17:15:08.628+08:00`: FAULT, reason=`px4_velocity_unaligned` (px4_alignment_miss_s=nan)
- `2026-08-12T17:15:12.841+08:00`: HEALTHY, reason=`ok`
- `2026-08-12T17:15:17.045+08:00`: SUSPECT, reason=`px4_velocity_unaligned` (px4_alignment_miss_s=0.185707)
- `2026-08-12T17:15:17.429+08:00`: FAULT, reason=`recovering (0.00/2.00s)`
- `2026-08-12T17:15:19.832+08:00`: HEALTHY, reason=`ok`

## Axis/sign checks

Raw/healthy positions use ENU-like world axes; ODOMETRY pose keeps that world convention while its twist is expressed in child/body FLU. PX4 NED is `(N,E,D)=(ENU y, ENU x, -ENU z)`.

| Motion | Raw ENU delta | Healthy ENU delta | race/odom ENU delta | PX4 NED delta | Raw->NED error |
|---|---|---|---|---|---:|

### Body-FLU velocity sign checks

| Motion | Expected component | Peak signed component | Result |
|---|---|---:|---|
| FORWARD | axis 0, sign +1 | n/a | INCOMPLETE |
| BACKWARD | axis 0, sign -1 | n/a | INCOMPLETE |
| LEFT | axis 1, sign +1 | n/a | INCOMPLETE |
| RIGHT | axis 1, sign -1 | n/a | INCOMPLETE |
| UP | axis 2, sign +1 | n/a | INCOMPLETE |
| DOWN | axis 2, sign -1 | n/a | INCOMPLETE |
| YAW90_FORWARD | axis 0, sign +1 | n/a | INCOMPLETE |

### Installation-yaw decision

- Forward/left signs do not form a pure 0/180-deg mapping; do not modify TF from this run. Repeat the marked motions.

### Vehicle Odom heading-to-motion checks

The RViz `Vehicle Odom` arrow is `/race/odom` body `+X`. During a marked forward movement, its yaw must agree with the measured XY displacement.

| Motion | XY displacement | Motion heading | Mean arrow yaw | Error | Result |
|---|---:|---:|---:|---:|---|
| FORWARD | nan m | +nan deg | +nan deg | +nan deg | INCOMPLETE |
| YAW90_FORWARD | nan m | +nan deg | +nan deg | +nan deg | INCOMPLETE |

## Stop-to-zero checks


## Interpretation thresholds

- Startup/recovery HEALTHY should not occur before about 7.5 s in this validation launch.
- Raw source stamps must be strictly increasing; normal source age should stay below 0.25 s.
- FAST-LIO timeout should produce SUSPECT after about 0.5 s and FAULT about 0.3 s later.
- No healthy position sample may be emitted during FAULT.
- A stopped motion should fall below 0.05 m/s within 1.0 s.
- Axis deltas must have the expected sign and ENU/NED mapping error should remain small.

## EV velocity fusion readiness

- `FUSION_RATE_READINESS_FAIL`: real FAST-LIO source rate is 10.00 Hz; no sample repetition is permitted.
- PX4 console evidence (`vehicle_visual_odometry`, aid-source flags, and `cs_ev_vel=false`) must be attached separately; a ROS bag cannot prove all uORB states.
- **READY_FOR_EV_VEL_FUSION = NO** until every static, hand-held, PX4-receive, and real-rate criterion passes.

## Run-level verdict

| Criterion | Result |
|---|---|
| Bag readable; clean-save status passes when present | PASS |
| Required EV/MAVROS chain topics recorded | PASS |
| Unarmed and never OFFBOARD | PASS |
| No control/setpoint messages | PASS |
| EV health remained acceptable through recording end | PASS |
| Marked 60-second static test | INCOMPLETE |
| Marked physical axis/sign tests | INCOMPLETE |
| Strict coordinate topics and TF captured | FAIL |
| Livox LiDAR/IMU source stamps strictly increase | FAIL |
| Strict marked motion sequence completed | INCOMPLETE |
| `/race/odom` axis/sign mapping | FAIL |
| `Vehicle Odom` arrow agrees with forward displacement | FAIL |

**OVERALL_PROP_OFF_VALIDATION = NOT PASSED**
