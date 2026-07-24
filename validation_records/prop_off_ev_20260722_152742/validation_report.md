# Propeller-off EV validation report

Bag: `/home/robot/ws_offboard_control/validation_records/prop_off_ev_20260722_152742/rosbag`

## Safety evidence

- MAVROS armed=true samples: **0**
- MAVROS OFFBOARD samples: **0**
- Recorded control-topic message counts: `/mavros/setpoint_raw/local`=0, `/fmu/in/trajectory_setpoint`=0, `/fmu/in/vehicle_command`=0
MAVROS state transitions:

- `2026-07-22T15:27:48.058+08:00`: armed=false, mode=STABILIZED

## Timing and frequency

| Topic | Samples | Receive rate | Source rate | Non-monotonic stamps |
|---|---:|---:|---:|---:|
| `/Odometry` | 2565 | 9.45 Hz | 9.45 Hz | 0 |
| `/Odometry/healthy` | 2390 | 8.86 Hz | 8.86 Hz | 0 |
| `/mavros/odometry/out` | 2391 | 8.86 Hz | 8.86 Hz | 0 |
| `/mavros/local_position/odom` | 8111 | 30.00 Hz | 30.00 Hz | 0 |
| `/mavros/local_position/velocity_local` | 8111 | 30.00 Hz | 30.00 Hz | 0 |

Source age from `/ev_health/diagnostics`: min=0.0109s, p50=0.0125s, p95=0.0157s, p99=0.0175s, max=0.0844s

## 60-second static velocity acceptance

- Samples: **594**
- Axis mean FLU m/s: **(-0.000, -0.001, -0.000)**
- Axis standard deviation m/s: **(+0.009, +0.008, +0.007)**
- Axis RMS m/s: **(+0.009, +0.008, +0.007)**
- Speed norm p95/p99/max: **0.024 / 0.028 / 0.034 m/s**
- Samples above 0.25 m/s: **0**
- Velocity variance x: mean=0.01, range=[0.01, 0.01] m2/s2
- Velocity variance y: mean=0.01, range=[0.01, 0.01] m2/s2
- Velocity variance z: mean=0.01, range=[0.01, 0.01] m2/s2
- Distinct 3x3 covariance matrices: **594**

## Health transitions

- First health status to first HEALTHY transition: **8.436s**
- Largest raw Odometry gap: **14.903s**
- Last raw sample to SUSPECT: **0.565s**
- Last raw sample to FAULT: **0.965s**
- `/Odometry/healthy` samples during FAULT before raw recovery: **0**
- First raw sample after gap to HEALTHY: **7.801s**

Observed health state edges:

- `2026-07-22T15:27:48.562+08:00`: SUSPECT
- `2026-07-22T15:27:48.799+08:00`: FAULT
- `2026-07-22T15:27:56.999+08:00`: HEALTHY
- `2026-07-22T15:31:29.662+08:00`: SUSPECT
- `2026-07-22T15:31:30.062+08:00`: FAULT
- `2026-07-22T15:31:51.801+08:00`: HEALTHY

## Axis/sign checks

Raw/healthy positions use ENU-like world axes; ODOMETRY pose keeps that world convention while its twist is expressed in child/body FLU. PX4 NED is `(N,E,D)=(ENU y, ENU x, -ENU z)`.

| Motion | Raw ENU delta | Healthy ENU delta | ODOMETRY ENU delta | PX4 NED delta | Raw->NED error |
|---|---|---|---|---|---:|
| FORWARD PASS | (+0.345, +0.029, -0.004) | (+0.345, +0.029, -0.004) | (+0.345, +0.029, -0.004) | (+0.022, +0.354, -0.006) | 0.015 m |
| BACKWARD PASS | (-0.209, +0.005, -0.014) | (-0.209, +0.005, -0.014) | (-0.209, +0.005, -0.014) | (-0.018, -0.199, +0.007) | 0.026 m |
| LEFT PASS | (+0.024, +0.226, +0.000) | (+0.024, +0.226, +0.000) | (+0.024, +0.226, +0.000) | (+0.237, +0.009, -0.025) | 0.031 m |
| RIGHT PASS | (+0.003, -0.277, -0.014) | (+0.003, -0.277, -0.014) | (+0.003, -0.277, -0.014) | (-0.278, +0.022, +0.020) | 0.020 m |
| UP PASS | (+0.038, -0.007, +0.237) | (+0.038, -0.007, +0.237) | (+0.038, -0.007, +0.237) | (-0.024, +0.048, -0.217) | 0.028 m |
| DOWN PASS | (-0.022, +0.048, -0.226) | (-0.022, +0.048, -0.226) | (-0.022, +0.048, -0.226) | (+0.080, -0.004, +0.218) | 0.037 m |

### Body-FLU velocity sign checks

| Motion | Expected component | Peak signed component | Result |
|---|---|---:|---|
| FORWARD | `x > 0` | 0.168 m/s | PASS |
| BACKWARD | `x < 0` | 0.144 m/s | PASS |
| LEFT | `y > 0` | 0.121 m/s | PASS |
| RIGHT | `y < 0` | 0.166 m/s | PASS |
| UP | `z > 0` | 0.141 m/s | PASS |
| DOWN | `z < 0` | 0.199 m/s | PASS |
| YAW90_FORWARD | `x > 0` | 0.152 m/s | PASS |

## Stop-to-zero checks

- FORWARD: speed <= 0.05 m/s after **0.046s**
- BACKWARD: speed <= 0.05 m/s after **0.085s**
- LEFT: speed <= 0.05 m/s after **0.021s**
- RIGHT: speed <= 0.05 m/s after **0.028s**
- UP: speed <= 0.05 m/s after **0.092s**
- DOWN: speed <= 0.05 m/s after **0.009s**

## Interpretation thresholds

- Startup/recovery HEALTHY should not occur before about 7.5 s in this validation launch.
- Raw source stamps must be strictly increasing; normal source age should stay below 0.25 s.
- FAST-LIO timeout should produce SUSPECT after about 0.5 s and FAULT about 0.3 s later.
- No healthy position sample may be emitted during FAULT.
- A stopped motion should fall below 0.05 m/s within 1.0 s.
- Axis deltas must have the expected sign and ENU/NED mapping error should remain small.

## EV velocity fusion readiness

- Offline body-to-world alignment against timestamped position differencing found a
  best velocity offset of **+0.050 s**, combined correlation **0.752**, and RMSE
  **0.0247 m/s** (`x/y/z` correlations `0.743/0.794/0.711`). Direction is correct,
  but this is moderate rather than conclusively high correlation.
- All 2391 outgoing angular-velocity samples were NaN, correctly representing an
  unknown measurement. Outgoing source stamps were strictly monotonic and no
  consecutive velocity sample was repeated.
- The outgoing linear covariance was symmetric and positive semidefinite (minimum
  eigenvalue `0.00989`, maximum asymmetry `5.4e-20`). However, the bridge's
  `min_linear_velocity_variance=0.01` floor clamped almost every diagonal variance
  to `0.01 m2/s2`, masking FAST-LIO's measured covariance (typically around
  `0.00014-0.0003 m2/s2` while healthy/static). This is conservative, but it fails
  the requirement to carry the real varying velocity covariance.
- `FUSION_RATE_READINESS_FAIL`: real FAST-LIO source rate is 9.45 Hz; no sample repetition is permitted.
- PX4 console evidence (`vehicle_visual_odometry`, aid-source flags, and `cs_ev_vel=false`) must be attached separately; a ROS bag cannot prove all uORB states.
- **READY_FOR_EV_VEL_FUSION = NO** until every static, hand-held, PX4-receive, and real-rate criterion passes.
