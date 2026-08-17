# Flight-ready Drift-Aware LiDAR-Intertial Odometry and Mapping with Self-correcting Maps

<p align="center">
  <img src="assets/img/IMG_2033.jpg" alt="Flight platform with labelled hardware components" />
</p>

[![ci][ci-badge]][ci-link]
[![Pixi][pixi-img]][pixi-link]
[![Watch the demo][youtube-badge]][youtube-video]

> [!NOTE]
> Built on [FAST-LIO2](https://github.com/hku-mars/FAST_LIO) by HKU MARS Lab — please cite their work if you use this.

## Quick start

Prerequisites: [pixi](https://pixi.sh) installed.

```bash
git clone --recursive https://github.com/alvgaona/fr-lio
cd fr-lio
pixi run vcs-import   # fetch Livox-SDK2 + livox_ros_driver2 into ./deps
pixi run build        # colcon build (humble env by default)
```

Run with the default indoor config and RViz:

```bash
pixi run ros2 launch fr_lio lio.launch.py
```

Replay a rosbag with mocap ground truth:

```bash
pixi run ros2 launch fr_lio lio.launch.py \
  config_file:=config/indoors.yaml use_sim_time:=true mocap:=true
# in a second terminal:
pixi run ros2 bag play <bag-dir> --clock
```

### Common launch overrides

```bash
# Switch to the outdoor profile
pixi run ros2 launch fr_lio lio.launch.py config_file:=config/outdoors.yaml

# Compare against mocap ground truth (publishes /ground_truth/odom + /path)
pixi run ros2 launch fr_lio lio.launch.py mocap:=true rigid_body_name:=91

# Headless run inside a namespace
pixi run ros2 launch fr_lio lio.launch.py namespace:=drone1 rviz:=false
```

Use the `jazzy` pixi environment instead of the default humble:

```bash
pixi run -e jazzy build
```

## High-rate Odometry

The flight-oriented build publishes one IMU-rate `nav_msgs/msg/Odometry`
stream on `/Odometry`. Each successful LiDAR/IESKF update resets an 18-state
propagation anchor, then buffered IMU samples are replayed and subsequent IMU
samples are integrated sequentially. Pose and twist covariance are propagated
with the state rather than synthesized at publication time.

The output contract is:

```text
header.frame_id: odom (world ENU)
child_frame_id:  body (body FLU, LIO/IMU origin)
pose:             body pose expressed in odom
twist.linear:     body/FLU linear velocity
twist.angular:    bias-corrected body/FLU angular velocity
QoS:              SensorDataQoS, keep_last(5)
timestamp:        source IMU measurement time; never restamped
```

FR-LIO does not apply the flight-controller lever arm or ENU/NED and FLU/FRD
conversions. Those operations belong at the MAVROS/PX4 bridge boundary and
must be performed exactly once.

### Confirm the IMU unit before enabling output

The supplied configurations intentionally use:

```yaml
imu:
  acceleration_unit: unconfirmed
```

With this setting the main LIO can initialize, but high-rate `/Odometry` is
inhibited. Keep the platform stationary for 10 seconds and read the
`IMU_UNIT_REPORT` log. Then set the parameter explicitly to `mps2` when the
raw norm is near 9.81, or `g` when it is near 1.0, and restart. Runtime unit
guessing is deliberately disabled.

High-rate health is exposed on:

```text
/frlio/high_rate_odom/status
/frlio/high_rate_odom/anchor_age
/frlio/high_rate_odom/predictor_age
/frlio/high_rate_odom/ev_usable
/frlio/high_rate_odom/planner_usable
/frlio/high_rate_odom/localization_health   (diagnostic_msgs/DiagnosticArray)
```

Health is graded on two independent axes, because "the LiDAR posterior is late"
and "the propagated state is unusable" are different failures with different
correct responses:

| Level | Status string | `ev_usable` | `planner_usable` | Trigger |
| --- | --- | --- | --- | --- |
| L0 GOOD | `HEALTHY` | yes | yes | `anchor_age < 0.18 s` |
| L1 DEGRADED | `SUSPECT_STALE_LIDAR` | yes | yes | `anchor_age > 0.18 s`, exits below 0.12 s |
| L2 PLANNER_UNUSABLE | `FAULT_STALE_LIDAR` | **yes** | no | `anchor_age > 0.40 s` |
| L3 STATE_UNUSABLE | `FAULT_STATE_UNUSABLE` | no | no | `predictor_age > 0.15 s`, timestamp rollback, NaN/Inf, uninitialised |

`/Odometry` publication does **not** stop at L2. A stale LiDAR anchor means the
pose is propagating on IMU alone: the covariance is inflated (up to
`stale_covariance_max_multiplier`) and the planner stops switching trajectories,
but the stream stays continuous with its real timestamps so PX4 EKF2 keeps a
position estimate and decides for itself whether to fuse. Publication stops only
when the state itself is not finite, which is L3.

Recovery from L2 requires a newer committed LiDAR posterior plus
`recovery_healthy_samples` consecutive low-age samples — age alone is not enough,
because the age resets when a replay starts, before the posterior is committed.
Recovery from L3 requires the predictor age to fall below
`predictor_recover_age_s` with an advancing predictor generation for
`predictor_recovery_healthy_samples` samples; a frozen predictor reporting a
small age therefore cannot recover on its own.

Every field in `localization_health` comes from one snapshot taken at a single
instant, so a consumer can never observe a contradiction such as `HEALTHY`
together with `anchor_age = 1.1 s`.

### Build boundary and LiDAR input

GTSAM loop closure is excluded from the default flight build. To build the
optional offline loop-closure path, install GTSAM and configure with:

```bash
colcon build --cmake-args -DFR_LIO_ENABLE_LOOP_CLOSURE=ON
```

The default launch reads `/livox/lidar` directly. Extra packet accumulation is
disabled; it can be explicitly enabled with `lidar_accumulator:=true` for
non-flight experiments. The default configs also disable loop closure, shadow
map correction, and live ikd-tree rebuilds so `/Odometry` remains a continuous
local estimate.

## Citation

```bibtex
@software{gaona2026frlio,
  author  = {Gaona, Alvaro J. and Perez-Saura, David and Campoy, Pascual},
  title   = {Flight-ready Drift-Aware LiDAR-Inertial Odometry and Mapping with Self-correcting Maps},
  year    = {2026},
  url     = {https://github.com/alvgaona/fr-lio},
  version = {0.1.0}
}
```

[ci-badge]: https://github.com/alvgaona/fr-lio/actions/workflows/ci.yml/badge.svg
[ci-link]: https://github.com/alvgaona/fr-lio/actions/workflows/ci.yml
[pixi-img]: https://img.shields.io/endpoint?url=https://raw.githubusercontent.com/prefix-dev/pixi/main/assets/badge/v0.json
[pixi-link]: https://pixi.sh
[youtube-badge]: https://img.shields.io/badge/YouTube-Watch%20demo-red?logo=youtube&logoColor=white
[youtube-video]: https://www.youtube.com/watch?v=mVYm7tcp8Lg
