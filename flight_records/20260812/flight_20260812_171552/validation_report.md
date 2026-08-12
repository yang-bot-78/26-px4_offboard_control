# Propeller-off EV validation report

Bag: `/home/robot/rong_ws/ws_offboard_control/flight_records/20260812/flight_20260812_171552/rosbag`

## Safety evidence

- MAVROS armed=true samples: **0**
- MAVROS OFFBOARD samples: **0**
- Recorded control-topic message counts: `/mavros/setpoint_raw/local`=550, `/fmu/in/trajectory_setpoint`=0, `/fmu/in/vehicle_command`=0
MAVROS state transitions:

- `2026-08-12T17:17:01.078+08:00`: armed=false, mode=STABILIZED

## Timing and frequency

| Topic | Samples | Receive rate | Source rate | Non-monotonic stamps |
|---|---:|---:|---:|---:|
| `/livox/lidar` | 0 | nan Hz | nan Hz | 0 |
| `/livox/imu` | 0 | nan Hz | nan Hz | 0 |
| `/Odometry` | 797 | 7.35 Hz | 7.36 Hz | 3 |
| `/Odometry/healthy` | 373 | 9.95 Hz | 9.95 Hz | 0 |
| `/mavros/odometry/out` | 0 | nan Hz | nan Hz | 0 |
| `/mavros/local_position/odom` | 3160 | 29.42 Hz | 29.42 Hz | 0 |
| `/mavros/local_position/velocity_local` | 3160 | 29.42 Hz | 29.42 Hz | 0 |

### Recorded coverage

| Topic | First sample | Last sample | Span | Silence before bag end |
|---|---|---|---:|---:|
| `/Odometry` | 2026-08-12T17:17:00.137+08:00 | 2026-08-12T17:18:48.484+08:00 | 108.347s | 0.031s |
| `/Odometry/healthy` | 2026-08-12T17:17:00.327+08:00 | 2026-08-12T17:17:37.720+08:00 | 37.393s | 70.796s |
| `/mavros/vision_pose/pose_cov` | 2026-08-12T17:17:00.328+08:00 | 2026-08-12T17:17:37.720+08:00 | 37.392s | 70.795s |
| `/mavros/vision_speed/speed_twist_cov` | 2026-08-12T17:17:00.328+08:00 | 2026-08-12T17:17:37.720+08:00 | 37.392s | 70.795s |
| `/mavros/local_position/odom` | 2026-08-12T17:17:01.080+08:00 | 2026-08-12T17:18:48.447+08:00 | 107.367s | 0.069s |
| `/mavros/local_position/velocity_local` | 2026-08-12T17:17:01.080+08:00 | 2026-08-12T17:18:48.447+08:00 | 107.367s | 0.069s |
| `/race/odom` | 2026-08-12T17:17:00.320+08:00 | 2026-08-12T17:18:48.486+08:00 | 108.166s | 0.030s |
| `/race/control/status` | 2026-08-12T17:17:00.444+08:00 | 2026-08-12T17:18:48.516+08:00 | 108.071s | 0.000s |
| `/ev_health/status` | 2026-08-12T17:17:00.324+08:00 | 2026-08-12T17:18:48.422+08:00 | 108.097s | 0.094s |

## Strict coordinate/TF evidence

- Required topic counts: `/livox/lidar`=0, `/livox/imu`=0, `/Odometry`=797, `/race/odom`=795, `/tf`=0, `/tf_static`=0, `/mavros/vision_pose/pose_cov`=373, `/mavros/local_position/odom`=3160, `/mavros/local_position/velocity_local`=3160
- Sensor source stamp regressions: `/livox/lidar`=0, `/livox/imu`=0
- Dynamic `map -> base_link` found: **False**
- Static `base_link -> body`: **missing**

Source age from `/ev_health/diagnostics`: min=0.0105s, p50=0.0155s, p95=9.7987s, p99=15.6271s, max=15.6564s

## 60-second static velocity acceptance

- **INCOMPLETE:** STATIC_60S markers are missing.

## Health transitions

- Startup hysteresis: **not observable** (recording began after HEALTHY).
- Largest raw Odometry gap: **6.833s**
- Last raw sample to SUSPECT: **n/a**
- Last raw sample to FAULT: **n/a**
- `/Odometry/healthy` samples during FAULT before raw recovery: **0**
- First raw sample after gap to HEALTHY: **n/a**
- Final recorded state: **FAULT** for at least **70.601s**; no later recovery is present in the bag.

Observed health state edges and reasons:

- `2026-08-12T17:17:00.324+08:00`: HEALTHY, reason=`ok`
- `2026-08-12T17:17:27.350+08:00`: SUSPECT, reason=`stale_input age=0.342s` (input_age_s=0.341743)
- `2026-08-12T17:17:29.422+08:00`: HEALTHY, reason=`ok`
- `2026-08-12T17:17:37.521+08:00`: SUSPECT, reason=`px4_velocity_unaligned` (px4_alignment_miss_s=0.138737)
- `2026-08-12T17:17:37.821+08:00`: FAULT, reason=`px4_velocity_unaligned` (px4_alignment_miss_s=0.438718)

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

- `FUSION_RATE_READINESS_FAIL`: real FAST-LIO source rate is 7.36 Hz; no sample repetition is permitted.
- PX4 console evidence (`vehicle_visual_odometry`, aid-source flags, and `cs_ev_vel=false`) must be attached separately; a ROS bag cannot prove all uORB states.
- **READY_FOR_EV_VEL_FUSION = NO** until every static, hand-held, PX4-receive, and real-rate criterion passes.

## Run-level verdict

| Criterion | Result |
|---|---|
| Bag readable; clean-save status passes when present | PASS |
| Required EV/MAVROS chain topics recorded | PASS |
| Unarmed and never OFFBOARD | PASS |
| No control/setpoint messages | FAIL |
| EV health remained acceptable through recording end | FAIL |
| Marked 60-second static test | INCOMPLETE |
| Marked physical axis/sign tests | INCOMPLETE |
| Strict coordinate topics and TF captured | FAIL |
| Livox LiDAR/IMU source stamps strictly increase | FAIL |
| Strict marked motion sequence completed | INCOMPLETE |
| `/race/odom` axis/sign mapping | FAIL |
| `Vehicle Odom` arrow agrees with forward displacement | FAIL |

**OVERALL_PROP_OFF_VALIDATION = NOT PASSED**
