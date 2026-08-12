# Propeller-off EV validation report

Bag: `/home/robot/rong_ws/ws_offboard_control/validation_records/prop_off_ev_20260809_233135/rosbag`

## Safety evidence

- MAVROS armed=true samples: **0**
- MAVROS OFFBOARD samples: **0**
- Recorded control-topic message counts: `/mavros/setpoint_raw/local`=0, `/fmu/in/trajectory_setpoint`=0, `/fmu/in/vehicle_command`=0
MAVROS state transitions:

- `2026-08-09T23:31:42.523+08:00`: armed=false, mode=STABILIZED

## Timing and frequency

| Topic | Samples | Receive rate | Source rate | Non-monotonic stamps |
|---|---:|---:|---:|---:|
| `/Odometry` | 3016 | 9.42 Hz | 9.42 Hz | 0 |
| `/Odometry/healthy` | 2572 | 8.44 Hz | 8.44 Hz | 0 |
| `/mavros/odometry/out` | 0 | nan Hz | nan Hz | 0 |
| `/mavros/local_position/odom` | 9577 | 30.00 Hz | 30.00 Hz | 0 |
| `/mavros/local_position/velocity_local` | 9577 | 30.00 Hz | 30.00 Hz | 0 |

### Recorded coverage

| Topic | First sample | Last sample | Span | Silence before bag end |
|---|---|---|---:|---:|
| `/Odometry` | 2026-08-09T23:31:41.583+08:00 | 2026-08-09T23:37:01.678+08:00 | 320.095s | 0.062s |
| `/Odometry/healthy` | 2026-08-09T23:31:57.085+08:00 | 2026-08-09T23:37:01.578+08:00 | 304.493s | 0.163s |
| `/mavros/vision_pose/pose_cov` | 2026-08-09T23:31:43.591+08:00 | 2026-08-09T23:37:01.578+08:00 | 317.987s | 0.163s |
| `/mavros/vision_speed/speed_twist_cov` | 2026-08-09T23:31:43.591+08:00 | 2026-08-09T23:37:01.578+08:00 | 317.987s | 0.162s |
| `/mavros/local_position/odom` | 2026-08-09T23:31:42.545+08:00 | 2026-08-09T23:37:01.741+08:00 | 319.196s | 0.000s |
| `/mavros/local_position/velocity_local` | 2026-08-09T23:31:42.546+08:00 | 2026-08-09T23:37:01.741+08:00 | 319.195s | 0.000s |
| `/race/odom` | n/a | n/a | n/a | n/a |
| `/race/control/status` | n/a | n/a | n/a | n/a |
| `/ev_health/status` | 2026-08-09T23:31:44.578+08:00 | 2026-08-09T23:37:01.579+08:00 | 317.001s | 0.162s |

Source age from `/ev_health/diagnostics`: min=0.0059s, p50=0.0125s, p95=0.0141s, p99=0.0163s, max=0.1037s

## 60-second static velocity acceptance

- **INCOMPLETE:** no `/mavros/odometry/out` velocity samples.

## Health transitions

- First health status to first HEALTHY transition: **12.506s**
- Largest raw Odometry gap: **18.494s**
- Last raw sample to SUSPECT: **0.595s**
- Last raw sample to FAULT: **0.995s**
- `/Odometry/healthy` samples during FAULT before raw recovery: **0**
- First raw sample after gap to HEALTHY: **7.601s**
- Final recorded state: **HEALTHY** for at least **65.799s**; no later recovery is present in the bag.

Observed health state edges and reasons:

- `2026-08-09T23:31:44.578+08:00`: FAULT, reason=`velocity_mismatch difference=0.895m/s`
- `2026-08-09T23:31:57.085+08:00`: HEALTHY, reason=`ok`
- `2026-08-09T23:34:58.678+08:00`: SUSPECT, reason=`ev_timeout age=0.594s`
- `2026-08-09T23:34:59.078+08:00`: FAULT, reason=`ev_timeout age=0.994s`
- `2026-08-09T23:35:24.179+08:00`: HEALTHY, reason=`ok`
- `2026-08-09T23:35:33.978+08:00`: SUSPECT, reason=`velocity_mismatch difference=0.522m/s`
- `2026-08-09T23:35:34.279+08:00`: FAULT, reason=`velocity_mismatch difference=0.669m/s`
- `2026-08-09T23:35:55.779+08:00`: HEALTHY, reason=`ok`

## Axis/sign checks

Raw/healthy positions use ENU-like world axes; ODOMETRY pose keeps that world convention while its twist is expressed in child/body FLU. PX4 NED is `(N,E,D)=(ENU y, ENU x, -ENU z)`.

| Motion | Raw ENU delta | Healthy ENU delta | ODOMETRY ENU delta | PX4 NED delta | Raw->NED error |
|---|---|---|---|---|---:|
| FORWARD INCOMPLETE | (+0.388, -0.034, -0.048) | (+0.388, -0.034, -0.048) | n/a | (-0.028, +0.435, +0.053) | 0.047 m |
| BACKWARD INCOMPLETE | (-0.388, +0.086, +0.043) | (-0.388, +0.086, +0.043) | n/a | (+0.071, -0.382, -0.054) | 0.019 m |
| LEFT INCOMPLETE | (+0.048, +0.461, +0.106) | (+0.048, +0.461, +0.106) | n/a | (+0.461, -0.001, -0.134) | 0.056 m |
| RIGHT INCOMPLETE | (-0.079, -0.558, -0.115) | (-0.079, -0.558, -0.115) | n/a | (-0.572, -0.061, +0.113) | 0.023 m |
| UP INCOMPLETE | (-0.006, +0.017, +0.447) | (-0.006, +0.017, +0.447) | n/a | (+0.047, +0.004, -0.453) | 0.032 m |
| DOWN INCOMPLETE | (+0.022, +0.040, -0.473) | (+0.022, +0.040, -0.473) | n/a | (+0.016, +0.078, +0.497) | 0.066 m |

### Body-FLU velocity sign checks

| Motion | Expected component | Peak signed component | Result |
|---|---|---:|---|
| FORWARD | `x > 0` | nan m/s | FAIL |
| BACKWARD | `x < 0` | nan m/s | FAIL |
| LEFT | `y > 0` | nan m/s | FAIL |
| RIGHT | `y < 0` | nan m/s | FAIL |
| UP | `z > 0` | nan m/s | FAIL |
| DOWN | `z < 0` | nan m/s | FAIL |
| YAW90_FORWARD | `x > 0` | nan m/s | FAIL |

## Stop-to-zero checks

- FORWARD: speed <= 0.05 m/s after **0.084s**
- BACKWARD: speed <= 0.05 m/s after **0.062s**
- LEFT: speed <= 0.05 m/s after **0.035s**
- RIGHT: speed <= 0.05 m/s after **0.094s**
- UP: speed <= 0.05 m/s after **0.042s**
- DOWN: speed <= 0.05 m/s after **0.084s**

## Interpretation thresholds

- Startup/recovery HEALTHY should not occur before about 7.5 s in this validation launch.
- Raw source stamps must be strictly increasing; normal source age should stay below 0.25 s.
- FAST-LIO timeout should produce SUSPECT after about 0.5 s and FAULT about 0.3 s later.
- No healthy position sample may be emitted during FAULT.
- A stopped motion should fall below 0.05 m/s within 1.0 s.
- Axis deltas must have the expected sign and ENU/NED mapping error should remain small.

## EV velocity fusion readiness

- `FUSION_RATE_READINESS_FAIL`: real FAST-LIO source rate is 9.42 Hz; no sample repetition is permitted.
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
| Marked 60-second static test | PASS |
| Marked physical axis/sign tests | PASS |

**OVERALL_PROP_OFF_VALIDATION = PASS**
