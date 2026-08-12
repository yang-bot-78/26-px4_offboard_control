# Propeller-off EV validation report

Bag: `/home/robot/rong_ws/ws_offboard_control/flight_records/20260809/flight_20260809_160702/rosbag`

## Safety evidence

- MAVROS armed=true samples: **0**
- MAVROS OFFBOARD samples: **0**
- Recorded control-topic message counts: `/mavros/setpoint_raw/local`=4578, `/fmu/in/trajectory_setpoint`=0, `/fmu/in/vehicle_command`=0
MAVROS state transitions:

- `2026-08-09T16:07:39.099+08:00`: armed=false, mode=STABILIZED
- `2026-08-09T16:09:15.883+08:00`: armed=false, mode=AUTO.LOITER

## Timing and frequency

| Topic | Samples | Receive rate | Source rate | Non-monotonic stamps |
|---|---:|---:|---:|---:|
| `/Odometry` | 923 | 10.00 Hz | 10.00 Hz | 0 |
| `/Odometry/healthy` | 906 | 9.98 Hz | 9.98 Hz | 0 |
| `/mavros/odometry/out` | 0 | nan Hz | nan Hz | 0 |
| `/mavros/local_position/odom` | 6639 | 29.10 Hz | 29.10 Hz | 0 |
| `/mavros/local_position/velocity_local` | 6639 | 29.10 Hz | 29.10 Hz | 0 |

### Recorded coverage

| Topic | First sample | Last sample | Span | Silence before bag end |
|---|---|---|---:|---:|
| `/Odometry` | 2026-08-09T16:07:38.283+08:00 | 2026-08-09T16:09:10.479+08:00 | 92.196s | 136.758s |
| `/Odometry/healthy` | 2026-08-09T16:07:38.284+08:00 | 2026-08-09T16:09:08.980+08:00 | 90.696s | 138.257s |
| `/mavros/vision_pose/pose_cov` | 2026-08-09T16:07:38.284+08:00 | 2026-08-09T16:09:08.980+08:00 | 90.696s | 138.257s |
| `/mavros/vision_speed/speed_twist_cov` | 2026-08-09T16:07:38.284+08:00 | 2026-08-09T16:09:08.980+08:00 | 90.696s | 138.257s |
| `/mavros/local_position/odom` | 2026-08-09T16:07:39.096+08:00 | 2026-08-09T16:11:27.237+08:00 | 228.141s | 0.000s |
| `/mavros/local_position/velocity_local` | 2026-08-09T16:07:39.096+08:00 | 2026-08-09T16:11:27.237+08:00 | 228.141s | 0.000s |
| `/race/odom` | 2026-08-09T16:07:38.181+08:00 | 2026-08-09T16:09:10.479+08:00 | 92.298s | 136.758s |
| `/race/control/status` | 2026-08-09T16:07:38.109+08:00 | 2026-08-09T16:11:27.223+08:00 | 229.114s | 0.014s |
| `/ev_health/status` | 2026-08-09T16:07:38.284+08:00 | 2026-08-09T16:11:27.197+08:00 | 228.913s | 0.039s |

Source age from `/ev_health/diagnostics`: min=0.0061s, p50=0.0075s, p95=0.0091s, p99=0.0169s, max=0.3284s

## 60-second static velocity acceptance

- **INCOMPLETE:** STATIC_60S markers are missing.

## Health transitions

- Startup hysteresis: **not observable** (recording began after HEALTHY).
- Terminal raw Odometry silence before bag end: **136.758s**
- Final recorded state: **FAULT** for at least **138.200s**; no later recovery is present in the bag.

Observed health state edges and reasons:

- `2026-08-09T16:07:38.284+08:00`: HEALTHY, reason=`ok`
- `2026-08-09T16:07:45.901+08:00`: SUSPECT, reason=`stale_input age=0.328s` (input_age_s=0.328432)
- `2026-08-09T16:07:51.987+08:00`: HEALTHY, reason=`ok`
- `2026-08-09T16:08:30.081+08:00`: SUSPECT, reason=`px4_velocity_unaligned` (px4_alignment_miss_s=0.178038)
- `2026-08-09T16:08:32.381+08:00`: HEALTHY, reason=`ok`
- `2026-08-09T16:09:08.681+08:00`: SUSPECT, reason=`px4_velocity_unaligned` (px4_alignment_miss_s=0.111020)
- `2026-08-09T16:09:08.998+08:00`: FAULT, reason=`px4_velocity_unaligned` (px4_alignment_miss_s=0.410902)

## Axis/sign checks

Raw/healthy positions use ENU-like world axes; ODOMETRY pose keeps that world convention while its twist is expressed in child/body FLU. PX4 NED is `(N,E,D)=(ENU y, ENU x, -ENU z)`.

| Motion | Raw ENU delta | Healthy ENU delta | ODOMETRY ENU delta | PX4 NED delta | Raw->NED error |
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
| No control/setpoint messages | FAIL |
| EV health remained acceptable through recording end | FAIL |
| Marked 60-second static test | INCOMPLETE |
| Marked physical axis/sign tests | INCOMPLETE |

**OVERALL_PROP_OFF_VALIDATION = NOT PASSED**
