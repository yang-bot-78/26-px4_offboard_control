# Propeller-off EV validation report

Bag: `/home/robot/ws_offboard_control/validation_records/prop_off_ev_20260721_160508/rosbag`

## Safety evidence

- MAVROS armed=true samples: **3**
- MAVROS OFFBOARD samples: **0**
- Recorded control-topic message counts: `/mavros/setpoint_raw/local`=0, `/fmu/in/trajectory_setpoint`=0, `/fmu/in/vehicle_command`=0
MAVROS state transitions:

- `2026-07-21T16:05:13.358+08:00`: armed=false, mode=STABILIZED
- `2026-07-21T16:08:15.553+08:00`: armed=true, mode=STABILIZED
- `2026-07-21T16:08:18.552+08:00`: armed=false, mode=STABILIZED

## Timing and frequency

| Topic | Samples | Receive rate | Source rate | Non-monotonic stamps |
|---|---:|---:|---:|---:|
| `/Odometry` | 2336 | 8.93 Hz | 8.93 Hz | 0 |
| `/Odometry/healthy` | 2 | 9.46 Hz | 10.02 Hz | 0 |
| `/mavros/vision_pose/pose_cov` | 3 | 9.76 Hz | 10.01 Hz | 0 |
| `/mavros/local_position/odom` | 7816 | 30.00 Hz | 30.00 Hz | 0 |
| `/mavros/local_position/velocity_local` | 7816 | 30.00 Hz | 30.00 Hz | 0 |

Source age from `/ev_health/diagnostics`: min=0.0055s, median=0.0138s, p95=0.0427s, max=0.1042s

## Health transitions

- Startup marker to first HEALTHY: **n/a**
- FAST-LIO stop request to SUSPECT: **n/a**
- FAST-LIO stop request to FAULT: **0.029s**
- `/Odometry/healthy` samples from FAULT until restart: **0**
- Restart confirmation to HEALTHY: **n/a**

## Axis/sign checks

Expected current mapping: raw/healthy/MAVROS vision use ENU-like axes; PX4 NED is `(N,E,D)=(ENU y, ENU x, -ENU z)`.

| Motion | Raw ENU delta | Healthy ENU delta | Vision ENU delta | PX4 NED delta | Raw->NED error |
|---|---|---|---|---|---:|
| FORWARD INCOMPLETE | (+0.084, +0.006, +0.020) | n/a | n/a | (+0.013, +0.073, -0.013) | 0.015 m |
| BACKWARD INCOMPLETE | (-0.377, -0.016, +0.000) | n/a | n/a | (+0.124, -0.419, -0.032) | 0.149 m |
| LEFT INCOMPLETE | (+0.021, +0.444, +0.015) | n/a | n/a | (+0.606, -0.303, -0.026) | 0.363 m |
| RIGHT INCOMPLETE | (+0.004, -0.504, -0.020) | n/a | n/a | (-0.050, -0.033, -0.006) | 0.456 m |
| UP INCOMPLETE | (+0.054, +0.008, +0.433) | n/a | n/a | (+0.177, +0.359, -0.399) | 0.350 m |
| DOWN INCOMPLETE | (-0.016, -0.016, +0.024) | n/a | n/a | (-0.024, -0.044, +0.009) | 0.043 m |

## Stop-to-zero checks

- FORWARD: speed <= 0.05 m/s after **n/a**
- BACKWARD: speed <= 0.05 m/s after **0.043s**
- LEFT: speed <= 0.05 m/s after **0.098s**
- RIGHT: speed <= 0.05 m/s after **0.020s**
- UP: speed <= 0.05 m/s after **0.087s**
- DOWN: speed <= 0.05 m/s after **0.027s**

## Interpretation thresholds

- Startup/recovery HEALTHY should not occur before about 7.5 s in this validation launch.
- Raw source stamps must be strictly increasing; normal source age should stay below 0.25 s.
- FAST-LIO timeout should produce SUSPECT after about 0.5 s and FAULT about 0.3 s later.
- No healthy position sample may be emitted during FAULT.
- A stopped motion should fall below 0.05 m/s within 1.0 s.
- Axis deltas must have the expected sign and ENU/NED mapping error should remain small.
