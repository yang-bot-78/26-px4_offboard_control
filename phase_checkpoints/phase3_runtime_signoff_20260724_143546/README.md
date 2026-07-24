# Phase 3 runtime-acceptance checkpoint

Captured before Phase 4 ROS runtime acceptance on
2026-07-24 14:35:46 +08:00.

Source root:
`/home/robot/livox_mid360_env/ws_fastlio/src/fast_lio`

Build root:
`/home/robot/livox_mid360_env/ws_fastlio/build/fast_lio`

The checkpoint freezes:

- the seven Phase 3 files;
- the eight unchanged Phase 2 dependencies;
- the frozen design;
- `fastlio_mapping` and the three signed test executables;
- the last 3/3 CTest result log.

Verify from this directory with:

```bash
sha256sum -c MANIFEST.sha256
```

This checkpoint authorizes only isolated `/Odometry/propagated` runtime
acceptance. It does not authorize MAVROS, PX4, OFFBOARD, arming, motor operation,
flight, parameter changes, or connection to `/Odometry/healthy` or
`/mavros/odometry/out`.
