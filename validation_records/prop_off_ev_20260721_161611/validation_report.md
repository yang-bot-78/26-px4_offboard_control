# Propeller-off EV validation report

Bag: `/home/robot/ws_offboard_control/validation_records/prop_off_ev_20260721_161611/rosbag`

## Safety evidence

- MAVROS armed=true samples: **0**
- MAVROS OFFBOARD samples: **0**
- Recorded control-topic message counts: `/mavros/setpoint_raw/local`=0, `/fmu/in/trajectory_setpoint`=0, `/fmu/in/vehicle_command`=0
- Markers recovered from the session wall-time snapshot: `FORWARD_START`
MAVROS state transitions:

- `2026-07-21T16:16:16.474+08:00`: armed=false, mode=STABILIZED

## Timing and frequency

| Topic | Samples | Receive rate | Source rate | Non-monotonic stamps |
|---|---:|---:|---:|---:|
| `/Odometry` | 1468 | 9.17 Hz | 9.17 Hz | 0 |
| `/Odometry/healthy` | 1296 | 8.17 Hz | 8.17 Hz | 0 |
| `/mavros/vision_pose/pose_cov` | 1296 | 8.17 Hz | 8.17 Hz | 0 |
| `/mavros/local_position/odom` | 4772 | 30.00 Hz | 30.00 Hz | 0 |
| `/mavros/local_position/velocity_local` | 4772 | 30.00 Hz | 30.00 Hz | 0 |

Source age from `/ev_health/diagnostics`: min=0.0072s, median=0.0091s, p95=0.0152s, max=0.0688s

## Health transitions

- First health status to first HEALTHY transition: **8.499s**
- Largest raw Odometry gap: **13.305s**
- Last raw sample to SUSPECT: **0.502s**
- Last raw sample to FAULT: **0.802s**
- `/Odometry/healthy` samples during FAULT before raw recovery: **0**
- First raw sample after gap to HEALTHY: **7.603s**

Observed health state edges:

- `2026-07-21T16:16:16.928+08:00`: SUSPECT
- `2026-07-21T16:16:17.229+08:00`: FAULT
- `2026-07-21T16:16:25.427+08:00`: HEALTHY
- `2026-07-21T16:18:10.429+08:00`: SUSPECT
- `2026-07-21T16:18:10.729+08:00`: FAULT
- `2026-07-21T16:18:30.835+08:00`: HEALTHY

## Axis/sign checks

Expected current mapping: raw/healthy/MAVROS vision use ENU-like axes; PX4 NED is `(N,E,D)=(ENU y, ENU x, -ENU z)`.

| Motion | Raw ENU delta | Healthy ENU delta | Vision ENU delta | PX4 NED delta | Raw->NED error |
|---|---|---|---|---|---:|
| FORWARD PASS | (+0.540, +0.011, +0.001) | (+0.540, +0.011, +0.001) | (+0.540, +0.011, +0.001) | (+0.020, +0.536, +0.007) | 0.012 m |
| BACKWARD PASS | (-0.227, +0.030, -0.016) | (-0.227, +0.030, -0.016) | (-0.227, +0.030, -0.016) | (-0.007, -0.208, -0.009) | 0.049 m |
| LEFT PASS | (+0.016, +0.407, +0.012) | (+0.016, +0.407, +0.012) | (+0.016, +0.407, +0.012) | (+0.408, +0.061, -0.010) | 0.046 m |
| RIGHT PASS | (-0.104, -0.257, -0.024) | (-0.104, -0.257, -0.024) | (-0.104, -0.257, -0.024) | (-0.284, -0.034, +0.006) | 0.078 m |
| UP PASS | (+0.024, -0.014, +0.345) | (+0.024, -0.014, +0.345) | (+0.024, -0.014, +0.345) | (-0.024, +0.030, -0.316) | 0.031 m |
| DOWN PASS | (-0.008, +0.067, -0.322) | (-0.008, +0.067, -0.322) | (-0.008, +0.067, -0.322) | (+0.051, -0.069, +0.322) | 0.063 m |

## Stop-to-zero checks

- FORWARD: speed <= 0.05 m/s after **0.077s**
- BACKWARD: speed <= 0.05 m/s after **0.068s**
- LEFT: speed <= 0.05 m/s after **0.114s**
- RIGHT: speed <= 0.05 m/s after **0.054s**
- UP: speed <= 0.05 m/s after **0.124s**
- DOWN: speed <= 0.05 m/s after **0.033s**

## Interpretation thresholds

- Startup/recovery HEALTHY should not occur before about 7.5 s in this validation launch.
- Raw source stamps must be strictly increasing; normal source age should stay below 0.25 s.
- FAST-LIO timeout should produce SUSPECT after about 0.5 s and FAULT about 0.3 s later.
- No healthy position sample may be emitted during FAULT.
- A stopped motion should fall below 0.05 m/s within 1.0 s.
- Axis deltas must have the expected sign and ENU/NED mapping error should remain small.
