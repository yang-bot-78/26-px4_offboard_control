# Propeller-off EV validation report

Bag: `/home/robot/rong_ws/ws_offboard_control/validation_records/prop_off_ev_20260809_224415/rosbag`

## Safety evidence

- MAVROS armed=true samples: **0**
- MAVROS OFFBOARD samples: **0**
- Recorded control-topic message counts: `/mavros/setpoint_raw/local`=0, `/fmu/in/trajectory_setpoint`=0, `/fmu/in/vehicle_command`=0
MAVROS state transitions:

- `2026-08-09T22:44:21.960+08:00`: armed=false, mode=STABILIZED

## Timing and frequency

| Topic | Samples | Receive rate | Source rate | Non-monotonic stamps |
|---|---:|---:|---:|---:|
| `/Odometry` | 11218 | 23.70 Hz | 6.69 Hz | 0 |
| `/Odometry/healthy` | 0 | nan Hz | nan Hz | 0 |
| `/mavros/odometry/out` | 0 | nan Hz | nan Hz | 0 |
| `/mavros/local_position/odom` | 17353 | 30.00 Hz | 30.00 Hz | 0 |
| `/mavros/local_position/velocity_local` | 17352 | 30.00 Hz | 30.00 Hz | 0 |

### Recorded coverage

| Topic | First sample | Last sample | Span | Silence before bag end |
|---|---|---|---:|---:|
| `/Odometry` | 2026-08-09T22:44:21.089+08:00 | 2026-08-09T22:52:14.475+08:00 | 473.385s | 105.901s |
| `/Odometry/healthy` | n/a | n/a | n/a | n/a |
| `/mavros/vision_pose/pose_cov` | n/a | n/a | n/a | n/a |
| `/mavros/vision_speed/speed_twist_cov` | n/a | n/a | n/a | n/a |
| `/mavros/local_position/odom` | 2026-08-09T22:44:21.969+08:00 | 2026-08-09T22:54:00.376+08:00 | 578.407s | 0.000s |
| `/mavros/local_position/velocity_local` | 2026-08-09T22:44:21.969+08:00 | 2026-08-09T22:54:00.338+08:00 | 578.369s | 0.037s |
| `/race/odom` | n/a | n/a | n/a | n/a |
| `/race/control/status` | n/a | n/a | n/a | n/a |
| `/ev_health/status` | 2026-08-09T22:44:22.340+08:00 | 2026-08-09T22:54:00.321+08:00 | 577.981s | 0.055s |

Source age from `/ev_health/diagnostics`: min=1.1211s, p50=148.8814s, p95=411.3537s, p99=411.3537s, max=411.3537s

## 60-second static velocity acceptance

- **INCOMPLETE:** no `/mavros/odometry/out` velocity samples.

## Health transitions

- First health status to first HEALTHY transition: **n/a**
- Largest raw Odometry gap: **9.066s**
- Last raw sample to SUSPECT: **n/a**
- Last raw sample to FAULT: **n/a**
- `/Odometry/healthy` samples during FAULT before raw recovery: **0**
- First raw sample after gap to HEALTHY: **n/a**
- Terminal raw Odometry silence before bag end: **105.901s**
- Final recorded state: **FAULT** for at least **577.782s**; no later recovery is present in the bag.

Observed health state edges and reasons:

- `2026-08-09T22:44:22.340+08:00`: SUSPECT, reason=`non_monotonic_stamp dt=0.000366s`
- `2026-08-09T22:44:22.538+08:00`: FAULT, reason=`non_monotonic_stamp dt=0.000242s`

## Axis/sign checks

Raw/healthy positions use ENU-like world axes; ODOMETRY pose keeps that world convention while its twist is expressed in child/body FLU. PX4 NED is `(N,E,D)=(ENU y, ENU x, -ENU z)`.

| Motion | Raw ENU delta | Healthy ENU delta | ODOMETRY ENU delta | PX4 NED delta | Raw->NED error |
|---|---|---|---|---|---:|
| FORWARD INCOMPLETE | (+584.273, +232.468, -326.747) | n/a | n/a | (-0.592, -0.201, -0.036) | 709.023 m |
| BACKWARD INCOMPLETE | (+419.960, +168.110, -246.847) | n/a | n/a | (-0.918, +0.807, +0.118) | 514.912 m |
| LEFT INCOMPLETE | (+253.132, +108.871, -153.010) | n/a | n/a | (-0.583, -0.342, -0.089) | 315.703 m |
| RIGHT INCOMPLETE | (+169.615, +73.583, -104.120) | n/a | n/a | (-0.122, +0.393, +0.053) | 211.892 m |
| UP INCOMPLETE | (+168.288, +78.000, -107.721) | n/a | n/a | (-0.236, +0.033, -0.499) | 214.807 m |
| DOWN INCOMPLETE | (+85.395, +38.707, -53.966) | n/a | n/a | (-0.215, +0.016, +0.454) | 108.018 m |

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

- FORWARD: speed <= 0.05 m/s after **n/a**
- BACKWARD: speed <= 0.05 m/s after **n/a**
- LEFT: speed <= 0.05 m/s after **n/a**
- RIGHT: speed <= 0.05 m/s after **n/a**
- UP: speed <= 0.05 m/s after **n/a**
- DOWN: speed <= 0.05 m/s after **n/a**

## Interpretation thresholds

- Startup/recovery HEALTHY should not occur before about 7.5 s in this validation launch.
- Raw source stamps must be strictly increasing; normal source age should stay below 0.25 s.
- FAST-LIO timeout should produce SUSPECT after about 0.5 s and FAULT about 0.3 s later.
- No healthy position sample may be emitted during FAULT.
- A stopped motion should fall below 0.05 m/s within 1.0 s.
- Axis deltas must have the expected sign and ENU/NED mapping error should remain small.

## EV velocity fusion readiness

- `FUSION_RATE_READINESS_FAIL`: real FAST-LIO source rate is 6.69 Hz; no sample repetition is permitted.
- PX4 console evidence (`vehicle_visual_odometry`, aid-source flags, and `cs_ev_vel=false`) must be attached separately; a ROS bag cannot prove all uORB states.
- **READY_FOR_EV_VEL_FUSION = NO** until every static, hand-held, PX4-receive, and real-rate criterion passes.

## Run-level verdict

| Criterion | Result |
|---|---|
| Bag readable; clean-save status passes when present | PASS |
| Required EV/MAVROS chain topics recorded | FAIL |
| Unarmed and never OFFBOARD | PASS |
| No control/setpoint messages | PASS |
| EV health remained acceptable through recording end | FAIL |
| Marked 60-second static test | PASS |
| Marked physical axis/sign tests | PASS |

**OVERALL_PROP_OFF_VALIDATION = NOT PASSED**
