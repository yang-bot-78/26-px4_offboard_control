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
```

The state becomes `SUSPECT_STALE_LIDAR` after 0.15 s without a LiDAR anchor.
At 0.40 s it becomes `FAULT_STALE_LIDAR` and `/Odometry` publication stops.
An IMU timestamp rollback also invalidates propagation until a new LiDAR
anchor arrives.

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
