# Propeller-off EV validation report

Bag: `/home/robot/ws_offboard_control/validation_records/prop_off_ev_20260722_151746/rosbag`

## Safety evidence

- MAVROS armed=true samples: **0**
- MAVROS OFFBOARD samples: **0**
- Recorded control-topic message counts: `/mavros/setpoint_raw/local`=0, `/fmu/in/trajectory_setpoint`=0, `/fmu/in/vehicle_command`=0
MAVROS state transitions:

- `2026-07-22T15:17:52.532+08:00`: armed=false, mode=STABILIZED

## Timing and frequency

| Topic | Samples | Receive rate | Source rate | Non-monotonic stamps |
|---|---:|---:|---:|---:|
| `/Odometry` | 1541 | 9.92 Hz | 10.00 Hz | 0 |
| `/Odometry/healthy` | 0 | nan Hz | nan Hz | 0 |
| `/mavros/odometry/out` | 0 | nan Hz | nan Hz | 0 |
| `/mavros/local_position/odom` | 4632 | 29.99 Hz | 29.99 Hz | 0 |
| `/mavros/local_position/velocity_local` | 4632 | 29.99 Hz | 29.99 Hz | 0 |

Source age from `/ev_health/diagnostics`: min=0.7792s, p50=1.3380s, p95=1.5277s, p99=1.6580s, max=1.8181s

## 60-second static velocity acceptance

- **INCOMPLETE:** no `/mavros/odometry/out` velocity samples.

## Health transitions

- First health status to first HEALTHY transition: **n/a**
- Largest raw Odometry gap: **1.580s**
- Last raw sample to SUSPECT: **n/a**
- Last raw sample to FAULT: **0.848s**
- `/Odometry/healthy` samples during FAULT before raw recovery: **0**
- First raw sample after gap to HEALTHY: **n/a**

Observed health state edges:

- `2026-07-22T15:17:55.105+08:00`: FAULT

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
