# FR-LIO provenance

This directory vendors the ROS 2 package `fr_lio` so the high-rate LIO
implementation is versioned with this offboard-control workspace.

- Upstream: https://github.com/alvgaona/fr-lio
- Imported upstream revision: `3800c3b6ac90889b8d3f5e2b888f9bbe36c2aebd`
- License: GPL-2.0-only. The complete license text is in `LICENSE`.

The imported working tree includes the local high-rate odometry implementation
and its tests. It deliberately excludes upstream Git metadata, build/install
trees, Pixi environments, experiment outputs, simulation scripts, and media
assets. The required `ikd_tree` library is vendored separately at the fixed
upstream tag `v0.1.0`; see `third_party/ikd_tree/UPSTREAM.md`. The active PX4
external-vision path remains the guarded
`/Odometry -> /Odometry/healthy -> fastlio_mavros_vision_bridge` chain. FR-LIO
is available through `LIO_BACKEND=fr_lio`, but high-rate promotion is not a
flight authorization.

## Build prerequisites

Build this workspace after sourcing ROS 2 Humble and the existing
`livox_ros_driver2` overlay. The fixed `ikd_tree` source is built with FR-LIO by
default, so no external `ikd_tree` installation or `CMAKE_PREFIX_PATH` entry is
required. An exact system package can be selected explicitly with
`-DFR_LIO_USE_SYSTEM_IKD_TREE=ON`.

```bash
source /opt/ros/humble/setup.bash
source ~/livox_mid360_env/ws_livox/install/setup.bash
PYTHONNOUSERSITE=1 colcon build --base-paths src --packages-select fr_lio
```

For a non-flight integration check, select the vendored package explicitly:

```bash
source install/setup.bash
LIO_BACKEND=fr_lio NO_MAP_VALIDATION=true ENABLE_OUTPUT=false \
  MISSION_ENABLED=false RVIZ=false ./tools/flight/一键启动导航栈.sh
```

This command still starts real hardware dependencies. Do not use it as a
substitute for the propeller-off validation procedure.
