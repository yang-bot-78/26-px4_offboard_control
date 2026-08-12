# Propeller-off EV validation report

Bag: `/home/robot/rong_ws/ws_offboard_control/validation_records/prop_off_ev_20260812_075818/rosbag`

## Safety evidence

- MAVROS armed=true samples: **0**
- MAVROS OFFBOARD samples: **0**
- Recorded control-topic message counts: `/mavros/setpoint_raw/local`=0, `/fmu/in/trajectory_setpoint`=0, `/fmu/in/vehicle_command`=0
- Markers recovered from the session wall-time snapshot: `LEFT_SETTLED`
MAVROS state transitions:

- `2026-08-12T07:58:27.729+08:00`: armed=false, mode=STABILIZED

## Timing and frequency

| Topic | Samples | Receive rate | Source rate | Non-monotonic stamps |
|---|---:|---:|---:|---:|
| `/Odometry` | 2325 | 10.00 Hz | 10.00 Hz | 0 |
| `/Odometry/healthy` | 2225 | 9.65 Hz | 9.65 Hz | 0 |
| `/mavros/odometry/out` | 0 | nan Hz | nan Hz | 0 |
| `/mavros/local_position/odom` | 6967 | 30.00 Hz | 30.00 Hz | 0 |
| `/mavros/local_position/velocity_local` | 6967 | 30.00 Hz | 30.00 Hz | 0 |

### Recorded coverage

| Topic | First sample | Last sample | Span | Silence before bag end |
|---|---|---|---:|---:|
| `/Odometry` | 2026-08-12T07:58:27.432+08:00 | 2026-08-12T08:02:19.831+08:00 | 232.398s | 0.094s |
| `/Odometry/healthy` | 2026-08-12T07:58:29.336+08:00 | 2026-08-12T08:02:19.832+08:00 | 230.497s | 0.092s |
| `/mavros/vision_pose/pose_cov` | 2026-08-12T07:58:29.336+08:00 | 2026-08-12T08:02:19.832+08:00 | 230.496s | 0.092s |
| `/mavros/vision_speed/speed_twist_cov` | 2026-08-12T07:58:29.336+08:00 | 2026-08-12T08:02:19.832+08:00 | 230.496s | 0.092s |
| `/mavros/local_position/odom` | 2026-08-12T07:58:27.722+08:00 | 2026-08-12T08:02:19.925+08:00 | 232.202s | 0.000s |
| `/mavros/local_position/velocity_local` | 2026-08-12T07:58:27.722+08:00 | 2026-08-12T08:02:19.925+08:00 | 232.202s | 0.000s |
| `/race/odom` | 2026-08-12T07:58:27.433+08:00 | 2026-08-12T08:02:19.831+08:00 | 232.398s | 0.094s |
| `/race/control/status` | n/a | n/a | n/a | n/a |
| `/ev_health/status` | 2026-08-12T07:58:29.336+08:00 | 2026-08-12T08:02:19.832+08:00 | 230.497s | 0.092s |

## Strict coordinate/TF evidence

- Required topic counts: `/Odometry`=2325, `/race/odom`=2325, `/tf`=4650, `/tf_static`=2, `/mavros/vision_pose/pose_cov`=2225, `/mavros/local_position/odom`=6967, `/mavros/local_position/velocity_local`=6967
- Dynamic `map -> base_link` found: **True**
- Static `base_link -> body` translation: **(+0.000, +0.000, +0.080) m**
- Static `base_link -> body` yaw: **+0.000 deg**
- This is the candidate TF recorded during the run; the marked body-velocity signs below decide whether its yaw is correct.

Source age from `/ev_health/diagnostics`: min=0.0096s, p50=0.0118s, p95=0.0140s, p99=0.0172s, max=0.3258s

## 60-second static velocity acceptance

- Samples: **616**
- Axis mean FLU m/s: **(+0.001, +0.001, -0.000)**
- Axis standard deviation m/s: **(+0.006, +0.006, +0.004)**
- Axis RMS m/s: **(+0.006, +0.007, +0.004)**
- Speed norm p95/p99/max: **0.016 / 0.021 / 0.030 m/s**
- Samples above 0.25 m/s: **0**
- Velocity variance x: mean=0.000135932, range=[0.000131986, 0.000139794] m2/s2
- Velocity variance y: mean=0.000143709, range=[0.000138107, 0.000149265] m2/s2
- Velocity variance z: mean=0.000128305, range=[0.000124404, 0.000132752] m2/s2
- Distinct 3x3 covariance matrices: **616**

## Health transitions

- First health status to first HEALTHY transition: **8.413s**
- Final recorded state: **HEALTHY** for at least **54.401s**; no later recovery is present in the bag.

Observed health state edges and reasons:

- `2026-08-12T07:58:29.336+08:00`: SUSPECT, reason=`px4_velocity_unaligned` (px4_alignment_miss_s=nan)
- `2026-08-12T07:58:29.656+08:00`: FAULT, reason=`px4_velocity_timeout`
- `2026-08-12T07:58:37.749+08:00`: HEALTHY, reason=`ok`
- `2026-08-12T08:01:17.846+08:00`: SUSPECT, reason=`stale_input age=0.326s` (input_age_s=0.325809)
- `2026-08-12T08:01:25.432+08:00`: HEALTHY, reason=`ok`

## Axis/sign checks

Raw/healthy positions use ENU-like world axes; ODOMETRY pose keeps that world convention while its twist is expressed in child/body FLU. PX4 NED is `(N,E,D)=(ENU y, ENU x, -ENU z)`.

| Motion | Raw ENU delta | Healthy ENU delta | race/odom ENU delta | PX4 NED delta | Raw->NED error |
|---|---|---|---|---|---:|
| FORWARD PASS | (+0.688, -0.068, -0.026) | (+0.688, -0.068, -0.026) | (+0.687, -0.068, -0.026) | (-0.065, +0.690, +0.029) | 0.005 m |
| LEFT PASS | (+0.038, +0.794, -0.003) | (+0.038, +0.794, -0.003) | (+0.042, +0.791, -0.003) | (+0.779, +0.052, -0.005) | 0.022 m |
| UP PASS | (-0.064, -0.019, +0.501) | (-0.064, -0.019, +0.501) | (-0.065, -0.019, +0.501) | (-0.013, -0.074, -0.505) | 0.012 m |
| DOWN PASS | (+0.046, -0.007, -0.475) | (+0.046, -0.007, -0.475) | (+0.052, -0.008, -0.475) | (-0.001, +0.054, +0.462) | 0.016 m |

### Body-FLU velocity sign checks

| Motion | Expected component | Peak signed component | Result |
|---|---|---:|---|
| FORWARD | `x > 0` | 0.321 m/s | PASS |
| BACKWARD | axis 0, sign -1 | n/a | INCOMPLETE |
| LEFT | `y > 0` | 0.335 m/s | PASS |
| RIGHT | axis 1, sign -1 | n/a | INCOMPLETE |
| UP | `z > 0` | 0.332 m/s | PASS |
| DOWN | `z < 0` | 0.236 m/s | PASS |
| YAW90_FORWARD | `x > 0` | 0.364 m/s | PASS |

### Installation-yaw decision

- `/race/odom` reports forward `x>0` and left `y>0`: **retain the recorded installation yaw**.

## Stop-to-zero checks

- FORWARD: speed <= 0.05 m/s after **0.051s**
- LEFT: speed <= 0.05 m/s after **0.043s**
- UP: speed <= 0.05 m/s after **0.072s**
- DOWN: speed <= 0.05 m/s after **0.093s**

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
| Marked 60-second static test | PASS |
| Marked physical axis/sign tests | PASS |
| Strict coordinate topics and TF captured | PASS |
| Strict marked motion sequence completed | PASS |
| `/race/odom` axis/sign mapping | PASS |

**OVERALL_PROP_OFF_VALIDATION = PASS**
