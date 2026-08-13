# FR-LIO provenance

This directory vendors the ROS 2 package `fr_lio` so the high-rate LIO
implementation is versioned with this offboard-control workspace.

- Upstream: https://github.com/alvgaona/fr-lio
- Imported upstream revision: `3800c3b6ac90889b8d3f5e2b888f9bbe36c2aebd`
- License: GPL-2.0-only. The complete license text is in `LICENSE`.

The imported working tree includes the local high-rate odometry implementation
and its tests. It deliberately excludes upstream Git metadata, build/install
trees, Pixi environments, experiment outputs, simulation scripts, and media
assets. The active PX4 external-vision path remains the guarded
`/Odometry -> /Odometry/healthy -> fastlio_mavros_vision_bridge` chain. FR-LIO
is available through `LIO_BACKEND=fr_lio`, but high-rate promotion is not a
flight authorization.

## Build prerequisites

Build this workspace after sourcing ROS 2 Humble and a built `livox_ros_driver2`
overlay. FR-LIO also requires the CMake package `ikd_tree`; install it through
the upstream Pixi environment or provide it through `CMAKE_PREFIX_PATH`.

```bash
source /opt/ros/humble/setup.bash
source /path/to/livox_ws/install/setup.bash
colcon build --packages-up-to fr_lio
```

For a non-flight integration check, select the vendored package explicitly:

```bash
source install/setup.bash
LIO_BACKEND=fr_lio NO_MAP_VALIDATION=true ENABLE_OUTPUT=false \
  MISSION_ENABLED=false RVIZ=false ./tools/flight/一键启动导航栈.sh
```

This command still starts real hardware dependencies. Do not use it as a
substitute for the propeller-off validation procedure.
