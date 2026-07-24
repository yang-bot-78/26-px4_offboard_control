# Codex checkpoint: FAST-LIO EV drift / Offboard takeoff safety

Updated: 2026-07-22 (Asia/Shanghai)

## 2026-07-22 MAVROS odometry velocity covariance floor fix

- Kept `EKF2_EV_CTRL=11`; no PX4 parameter was changed and no ROS node, arm,
  OFFBOARD, motor, or flight command was started.
- Changed the configurable `fastlio_mavros_odometry_bridge`
  `min_linear_velocity_variance` default from `0.01` to `0.0001 m2/s2`.
- Valid finite positive linear-velocity covariance above the floor remains
  unchanged; only smaller valid diagonal variances are raised to `0.0001`.
- Existing rejection of NaN, Inf, zero, negative, asymmetric, and non-PSD linear
  covariance remains in place. Unknown angular velocity remains NaN and its
  conservative covariance behavior is unchanged.
- Added focused acceptance coverage for `0.0002 -> 0.0002`,
  `0.00005 -> 0.0001`, `0.02 -> 0.02`, invalid variance rejection, and the
  launch-exposed configurable default.
- `PYTHONNOUSERSITE=1 colcon build --symlink-install --packages-select
  px4_ros_com` passed. Direct focused pytest passed 9/9; targeted CTest
  `test_ev_odometry_acceptance` passed 1/1; `git diff --check` passed.

## Safety boundary

- No PX4 parameter was changed.
- `EKF2_EV_CTRL` remains operator-controlled and must remain `11` for this work.
- No arm, takeoff, motor, or propeller-on command was executed.
- This checkpoint does not authorize a flight. Finish build/tests and a propeller-off hand-held test first.

## 2026-07-22 EV velocity fusion pre-acceptance

### Current decision

`READY_FOR_EV_VEL_FUSION = NO`

No PX4 parameter was changed. `EKF2_EV_CTRL` must remain `11`. No ROS node was
started, no motor/arm/mode/setpoint command was sent, and no physical test was run.
The static and hand-held capture has now run. Remaining blockers are real covariance
preservation, stronger velocity/position consistency evidence, PX4
`vehicle_visual_odometry`/aid-source evidence, and the real-rate gate.

### 2026-07-22 actual propeller-off result (`prop_off_ev_20260722_152742`)

- Safety passed: zero armed samples, zero OFFBOARD samples, zero recorded control
  messages; MAVROS remained `armed=false`, `STABILIZED`.
- Unique path passed: exactly one `/mavros/odometry/out` publisher; zero legacy
  `vision_pose` publishers; no direct PX4 ROS EV publisher.
- Timing passed for correctness: raw rate `9.45 Hz`, no non-monotonic stamps;
  source-age p50/p95/p99/max `12.5/15.7/17.5/84.4 ms`.
- Static speed passed: means near zero, axis standard deviations `0.009/0.008/0.007
  m/s`, norm p95/p99/max `0.024/0.028/0.034 m/s`, no sample above `0.25 m/s`.
- All six translations and yaw-90 body-forward had the expected body-FLU signs.
  Stop-to-zero times were `0.009-0.092 s` for the six reported translations.
- Timeout/recovery passed: SUSPECT `0.565 s`, FAULT `0.965 s`, no healthy output
  during FAULT, recovery to HEALTHY `7.801 s` after raw data resumed.
- Internal speed versus position-difference speed had best offset `+0.050 s`,
  combined correlation `0.752`, RMSE `0.0247 m/s`. Signs are correct, but the
  correlation is moderate rather than conclusively high.
- All 2391 outgoing angular velocities were NaN (unknown), stamps were monotonic,
  and no consecutive velocity frame was duplicated.
- Covariance was finite, symmetric, and PSD, but the ODOMETRY bridge's current
  `min_linear_velocity_variance=0.01` floor clamps nearly all output diagonal
  variances to `0.01 m2/s2`, masking FAST-LIO's varying covariance. This blocks
  acceptance even though it is conservative rather than overconfident.
- Real source rate `9.45 Hz` gives `FUSION_RATE_READINESS_FAIL`. PX4 uORB evidence
  proving receipt of finite speed while `cs_ev_vel=false` is still missing.
- Final result remains `READY_FOR_EV_VEL_FUSION = NO`; keep `EKF2_EV_CTRL=11`.

### Verified implementation facts

- The actual flight snapshot used FAST-LIO from
  `/home/robot/livox_mid360_env/ws_fastlio/src/fast_lio`.
- Its state order is position `0..2`, rotation `3..5`, LiDAR extrinsic rotation
  `6..8`, extrinsic translation `9..11`, velocity `12..14`, gyro bias `15..17`,
  accelerometer bias `18..20`, and gravity `21..22`.
- `state_point.vel` is the IMU-state velocity in the `camera_init` world frame.
  The published pose is also the IMU state, not a proven vehicle control-center
  reference point. FAST-LIO's LiDAR-to-IMU extrinsic must not be applied again.
- `/Odometry` uses the LiDAR end time (`lidar_end_time`) and is produced at the
  LiDAR update rate, historically about 10 Hz. It is not an IMU-rate propagated topic.
- Before this continuation, FAST-LIO published before filling pose covariance,
  omitted internal velocity/covariance, and left twist at default zeros.
- Installed MAVROS is `2.14.0`. Its odometry input is `nav_msgs/msg/Odometry` on
  `/mavros/odometry/out`; REP-147 pose is in `header.frame_id`, twist is in
  `child_frame_id`. MAVROS transforms pose to `LOCAL_FRD`, twist to `BODY_FRD`,
  and transforms both 6x6 covariances before emitting MAVLink `ODOMETRY`.

### Implemented behavior

- Actual FAST-LIO `publish_odometry()` now completes pose covariance before
  publication, rotates world velocity to child/body FLU using `R_world_body^T`,
  rotates the full velocity covariance `P(12:14,12:14)`, preserves cross terms,
  symmetrizes numerical error, and publishes the original LiDAR-end timestamp.
- Unknown angular velocity is explicitly NaN rather than a fake zero. Angular
  covariance remains finite and high so it cannot contaminate the valid linear block.
- The health gate continues to derive velocity from position for consistency checks,
  but no longer replaces the internal FAST-LIO velocity. It validates finite,
  symmetric, positive-semidefinite 3x3 covariance and compares internal velocity,
  position-difference velocity, and time-aligned PX4 velocity in common PX4 NED.
- SUSPECT multiplies the complete linear 3x3 covariance; FAULT publishes no healthy
  odometry. Duplicate/backward timestamps remain rejected and are never restamped.
- The MAVROS odometry bridge defaults to `restamp_message=false` and
  `derive_missing_twist=false`, rejects invalid covariance and non-monotonic samples,
  converts sensor-child FLU to vehicle-child FLU, and supports optional
  `sensor_to_body_{x,y,z}_m` lever-arm compensation. Nonzero lever arm is rejected
  when angular velocity is unknown; all lever defaults are zero because dimensions
  have not been measured.
- Default `fastlio_mavros_autofix.launch.py` now enables only the MAVROS ODOMETRY
  path. Legacy `vision_pose` and direct PX4 bridges default off. Launch-time option
  validation and runtime graph checks reject duplicate EV inputs.

### Modified files for this continuation

- External actual source:
  `/home/robot/livox_mid360_env/ws_fastlio/src/fast_lio/src/laserMapping.cpp`
- Recovery record: `patches/fast_lio_internal_velocity.patch`
- `src/px4_ros_com/px4_ros_com/ev_health.py`
- `src/px4_ros_com/scripts/fastlio_ev_health_monitor.py`
- `src/px4_ros_com/src/bridges/fastlio_mavros_odometry_bridge.cpp`
- `src/px4_ros_com/launch/fastlio_mavros_autofix.launch.py`
- `src/px4_ros_com/launch/fastlio_mavros_odometry_bridge.launch.py`
- `src/px4_ros_com/launch/prop_off_ev_validation.launch.py`
- `src/px4_ros_com/test/test_ev_health.py`
- `src/px4_ros_com/test/test_ev_odometry_acceptance.py`
- `src/px4_ros_com/CMakeLists.txt`
- `run_prop_off_ev_validation.sh`
- `analyze_prop_off_ev_validation.py`
- `EV_VEL_FUSION_ACCEPTANCE.md`

### Build and automated verification

- `PYTHONNOUSERSITE=1 colcon build --symlink-install --packages-select fast_lio`:
  passed in the actual FAST-LIO workspace. Only an existing Boost bind deprecation
  message appeared.
- `PYTHONNOUSERSITE=1 colcon build --symlink-install --packages-select px4_ros_com`:
  passed.
- Direct targeted Python tests: `57 passed`.
- CTest targeted suites: 3/3 passed (`test_ev_health`,
  `test_ev_odometry_acceptance`, `test_minipc_mavros_offboard_ev_safety`).
- Shell syntax, Python compilation, both launch `--show-args`, and `git diff --check`:
  passed. The production launch shows ODOMETRY=true, vision=false, direct-PX4=false.
- `colcon test-result --verbose` still reports old package-wide lint result files
  (`1666 tests, 1626 failures, 11 skipped`). These are the explicitly preserved
  historical copyright/style/schema findings; targeted behavior tests pass.

### Required next action (propellers removed, disarmed)

Follow `EV_VEL_FUSION_ACCEPTANCE.md`. Start only the LiDAR driver, rebuilt FAST-LIO,
MAVROS, health monitor, and the unique ODOMETRY bridge, then run:

```bash
cd ~/ws_offboard_control
source /opt/ros/humble/setup.bash
source ~/livox_mid360_env/install/setup.bash
source install/setup.bash
bash ./run_prop_off_ev_validation.sh
```

The script refuses armed/OFFBOARD/control/duplicate-EV states and records 60 seconds
static, six directions, yaw-90 forward, FAST-LIO interruption, and 7.5-second recovery.
Attach read-only QGC MAVLink Console output for `vehicle_visual_odometry`,
`estimator_status_flags`, `estimator_aid_src_ev_pos`, and
`estimator_aid_src_ev_vel`. Do not change `EKF2_EV_CTRL=11`.

### Current horizontal drift thresholds

- Horizontal error at `0.20 m` only emits a warning.
- Sustained horizontal error at `0.50 m` triggers the latched AUTO.LAND safety path.
- Sustained horizontal error at `0.60 m` is the shorter-debounce emergency escalation of
  the same landing path; it never performs airborne Kill or forced disarm.
- The first-3-second transient displacement limit is also `0.50 m`, so the former
  `0.30 m` displacement threshold cannot independently request landing. Transient speed,
  attitude, and EV-health protections remain active.

## Confirmed root cause

Data flow in the affected flight was:

```text
FAST-LIO /Odometry
  -> /Odometry/guarded
  -> /mavros/vision_pose/pose_cov
  -> MAVROS ENU-to-NED conversion
  -> PX4 EKF2
```

## 2026-07-21 post-flight drift/takeoff safety continuation

Safety boundary for this continuation:

- No PX4 parameter was changed; `EKF2_EV_CTRL` remains `11`.
- No ROS node was started, and no mode, arm, motor, or flight command was sent.
- Work was limited to source edits, offline ULog/rosbag analysis, build, and tests.

### Offline evidence from `log_6_UnknownDate.ulg`, `log_7_UnknownDate.ulg`, and flight bag

- `log_6_UnknownDate.ulg` is a 0.94 s STABILIZED arm/disarm fragment. It did not
  take off and was disarmed by the RC switch.
- `log_7_UnknownDate.ulg` is the complete 34.85 s flight. Takeoff was detected
  1.20 s after arming, AUTO.LAND became active at 29.84 s, landed was reported
  at 31.84 s, and PX4 disarmed at 33.86 s. The later kill event was after disarm.
- EV remained HEALTHY throughout the armed flight. During arm-to-land, source
  age was at most 86.8 ms, EV/PX4 horizontal velocity difference at most
  0.165 m/s, and single-frame displacement at most 0.051 m. All logged EV
  position/height/yaw aid samples were fused without rejection or dead reckoning.
- The takeoff transient was real. Actual pitch reached -7.84 deg at 1.705 s,
  producing about 0.37 m forward travel. At 1.5 s actual pitch rate was about
  -36.3 deg/s while the PX4 pitch-rate target had already reversed to +4.9 deg/s;
  at 1.6 s actual remained -36.5 deg/s while the target was +31.0 deg/s. The
  vehicle therefore continued pitching forward after PX4 had commanded braking.
- PX4 did briefly command a small initial nose-down response, but its magnitude
  was much smaller than the measured transient: takeoff-window pitch-rate target
  minimum was about -1.77 deg/s versus -44.1 deg/s actual minimum.
- The control-allocation geometry records motors 0/2 at body +X (front) and 1/3
  at body -X (rear). During the 10-25 s hover window, mean normalized outputs
  were 0.524 for front motors versus 0.453 for rear motors, a persistent 0.071
  difference. No motor output saturated (maximum 0.559; no sample above 0.95).
  The hover pitch-rate integrator averaged about +0.050. This establishes a
  persistent pitch-axis compensation demand, but does not identify CG offset,
  motor/prop thrust mismatch, mounting, or another mechanical cause.
- In the actual flight snapshot and the pre-change current source,
  `drift_land_m=0.30` only logged and returned to HOLD. Only the 0.50 m emergency
  branch requested AUTO.LAND. The recorded 0.31 m trigger therefore did not
  cause landing; the flight continued to its planned hover timeout.

### Files modified in this continuation

- `src/px4_ros_com/scripts/minipc_mavros_offboard.py`
  - Makes `drift_land_m` a sustained, one-shot, latched safety-landing trigger.
  - Defines `drift_emergency_m` as a shorter-debounce escalation of the same
    AUTO.LAND path. It never kills or disarms an airborne vehicle.
  - Keeps fixed XY and a bounded 0.07 m/s default descent setpoint while an
    AUTO.LAND request is rejected, times out, or awaits MAVROS confirmation.
  - Prevents a latched safety landing from returning to HOLD, takeoff, route
    tracking, or normal task flow.
  - Adds a parameterized first-3-second transient guard for horizontal
    displacement, horizontal speed, absolute roll, absolute pitch, and EV health.
- `src/px4_ros_com/test/test_minipc_mavros_offboard_ev_safety.py`
  - Expands Offboard safety coverage from 12 to 21 tests, including drift debounce,
    latch persistence, retry heartbeat/descent, no airborne disarm at emergency,
    takeoff transient debounce, and unchanged planned landing.
- `run_takeoff_1m_hold.sh`
  - Exposes all new debounce/window/limit settings as environment overrides and
    passes them to the Offboard node. It was syntax checked but not executed.
- `CODEX_CHECKPOINT.md`
  - Adds this continuation record.

### New parameters and provisional defaults

These are ROS parameters, not PX4 parameters. Defaults are deliberately
configurable and remain provisional until propeller-off/SITL replay evidence is
reviewed; they must not be treated as final airframe tuning.

- `drift_land_trigger_duration_s=0.30`
- `drift_emergency_trigger_duration_s=0.10`
- `takeoff_transient_guard_enabled=true`
- `takeoff_transient_window_s=3.0`
- `takeoff_transient_trigger_duration_s=0.20`
- `takeoff_transient_max_displacement_m=0.30`
- `takeoff_transient_max_speed_mps=0.50`
- `takeoff_transient_max_roll_deg=10.0`
- `takeoff_transient_max_pitch_deg=10.0`

Offline replay against `log_7` indicates that the default transient displacement
condition would start at about 2.50 s and become sustained at about 2.70 s. The
speed and attitude defaults would not trigger on that log. This is an offline
check only, not authorization to fly.

### Verification results

- `bash -n run_takeoff_1m_hold.sh`: passed.
- Python syntax compilation for the modified node and test: passed.
- Direct behavioral tests: `42 passed in 0.35s` (EV health 21, Offboard safety 21).
- Focused `ament_flake8` on the expanded test file: passed.
- `git diff --check`: passed for the complete worktree.
- `PYTHONNOUSERSITE=1 colcon build --symlink-install --packages-select px4_ros_com`:
  passed.
- Package tests: both behavioral suites passed (21/21 and 21/21), and cppcheck
  passed. The same seven intentionally untouched package-wide lint gates failed:
  copyright, cpplint, flake8, lint_cmake, pep257, uncrustify, and xmllint.
- Latest aggregate test result: `1673 tests, 0 errors, 1633 failures, 11 skipped`.
  The large failure count expands historical lint findings; it is not behavioral
  test failure. Historical lint was not cleaned.

### Remaining risks

- The new latch/retry/descent behavior has unit-test and build coverage but has
  not been exercised against PX4 SITL or real MAVROS services.
- A propeller-off hand test cannot reproduce a real takeoff, attitude transient,
  or AUTO.LAND descent. Do not use `start_takeoff_1m_stack.sh` as a propeller-off
  takeoff/landing test: it can arm real motors and cannot reach its altitude target.
- The persistent front/rear actuator difference needs a separate mechanical and
  actuator investigation. The logs do not prove CG offset or motor thrust mismatch.
- The default transient limits are provisional. Do not make them more aggressive
  without replay/SITL evidence and do not resume propeller-on autonomous flight yet.

### Next safe propeller-off validation commands

The only current hardware validation that remains disarmed and sends no Offboard,
arm, or mode command is the existing EV hand test:

```bash
cd ~/ws_offboard_control
source /opt/ros/humble/setup.bash
source install/setup.bash
bash ./run_prop_off_ev_validation.sh
```

This validates EV timing/axes/FAULT behavior only. Validate the new autonomous
landing state machine in PX4 SITL before any propeller-on test; do not substitute
`./start_takeoff_1m_stack.sh` for SITL or a disarmed hand test.

### Exact software recheck commands

```bash
cd ~/ws_offboard_control
source /opt/ros/humble/setup.bash
source install/setup.bash
bash -n run_takeoff_1m_hold.sh
PYTHONNOUSERSITE=1 /usr/bin/python3 -m py_compile \
  src/px4_ros_com/scripts/minipc_mavros_offboard.py \
  src/px4_ros_com/test/test_minipc_mavros_offboard_ev_safety.py
PYTHONNOUSERSITE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
PYTHONPATH="src/px4_ros_com:${PYTHONPATH}" /usr/bin/python3 -m pytest -q \
  src/px4_ros_com/test/test_ev_health.py \
  src/px4_ros_com/test/test_minipc_mavros_offboard_ev_safety.py
PYTHONNOUSERSITE=1 colcon build --symlink-install --packages-select px4_ros_com
git diff --check
```

Analysis of `log_7_2026-7-19-16-01-06.ulg` and
`flight_records/20260719/flight_20260719_155940/rosbag/rosbag_0.db3` confirmed:

1. The raw, guarded, and MAVROS vision positions are spatially the same. The
   guard/vision bridge did not introduce a forward/back sign inversion.
2. Current mapping is `(PX4 N, E, D) = (ROS y, ROS x, -ROS z)`. In this flight,
   PX4 local `+Y` was approximately aircraft-forward.
3. FAST-LIO stopped producing `/Odometry` for about `1.48 s` at approximately
   `16:00:39.4`, then continued delivering ordered but stale samples with about
   `1.3-1.6 s` source age.
4. The old vision bridge used `restamp_message=true`, which replaced the original
   LiDAR measurement time and made old positions appear current to PX4.
5. In arrival-time windows the EV appeared to continue forward while PX4 velocity
   was backward. When aligned by the original FAST-LIO stamp, the later two windows
   were actually backward in both sources. The apparent sustained forward EV drift
   was mainly delayed backlog being fused as current data.
6. The first short forward motion still contains a real takeoff attitude transient;
   a stale provisional prestream XY/Z target could also create a mode-switch impulse.
7. Offline replay of the new source-age/timeout logic detects the FAST-LIO pause and
   reaches `FAULT` roughly 15 seconds before the recorded takeoff.

## Intended corrected data flow

```text
FAST-LIO /Odometry
  + MAVROS /mavros/local_position/velocity_local
  -> fastlio_ev_health_monitor
       -> /Odometry/healthy
       -> /ev_health/status
       -> /ev_health/fault
       -> /ev_health/diagnostics
       -> /ev_health/velocity_ned
  -> exactly one PX4 EV output bridge
       default: /mavros/vision_pose/pose_cov
       validation: /mavros/vision_speed/speed_twist_cov
  -> PX4 (EKF2 settings are not modified by ROS code)

/Odometry/guarded remains a parallel diagnostic/compatibility stream and is not the
default flight-controller input.
```

## Task-specific modified files and purpose

- `src/px4_ros_com/px4_ros_com/ev_health.py`
  - Pure timestamp/frame/velocity health core and `HEALTHY/SUSPECT/FAULT` state machine.
  - Original-stamp age, monotonicity, dt, jump, NaN/Inf, timeout, PX4 velocity,
    optional effective-points, hysteresis, covariance, ENU/NED checks.
  - EV interval velocity is compared with PX4 velocity over the same source-time interval.
- `src/px4_ros_com/scripts/fastlio_ev_health_monitor.py`
  - ROS wrapper, healthy odometry gate, diagnostics, status/fault topics, filtered speed,
    covariance floor/inflation, and no frozen-last-position publication in `FAULT`.
  - MAVROS velocity/odometry subscriptions use sensor-data Best Effort QoS, matching MAVROS
    Humble publishers; state transition logging uses fixed-severity call sites compatible with
    Humble `rclpy`.
- `src/px4_ros_com/launch/fastlio_mavros_autofix.launch.py`
  - Inserts the health gate before every PX4 EV output.
  - Sets vision `restamp_message=false`, publishes validation speed, exposes health thresholds,
    and rejects simultaneous duplicate EV output bridges.
- `src/px4_ros_com/src/bridges/fastlio_mavros_vision_bridge.cpp`
  - Preserves original stamps and correctly rotates position/speed covariance for yaw offsets.
- `src/px4_ros_com/src/bridges/fastlio_vehicle_visual_odometry.cpp`
  - Makes the optional direct-PX4 path use world ENU velocity converted to NED and marks
    `VELOCITY_FRAME_NED`; applies configured position/yaw alignment.
- `src/px4_ros_com/src/bridges/fastlio_mavros_odometry_bridge.cpp`
  - Makes the optional MAVROS odometry path preserve timestamps and explicitly convert
    validated world velocity to the child/body frame; applies position alignment.
- `src/px4_ros_com/scripts/minipc_mavros_offboard.py`
  - Requires continuous fresh `HEALTHY` EV before arm (default 7.5 s, constrained to 5-10 s).
  - Locks fresh XYZ at confirmed OFFBOARD entry; before that, full prestream XYZ tracks fresh
    local odometry so the mode-switch frame does not receive an old XY/Z target.
  - Uses a configurable 2.5 s default Z ramp (restored after evaluating a slower 4.0 s ramp).
  - Uses a configurable 0.07 m/s default Offboard landing speed to reduce vertical transients;
    slower rates mitigate but do not correct the confirmed forward center-of-gravity offset.
  - `SUSPECT`, `FAULT`, or health timeout stops climb and requests AUTO.LAND when armed.
  - Pending-arm EV failure latches a no-climb hold and waits for actual armed state, avoiding
    an arm/AUTO.LAND race.
  - AUTO.LAND handoff keeps a fixed XYZ heartbeat, retries rejected/timed-out service calls,
    and waits for fresh MAVROS state to confirm `AUTO.LAND`.
  - Retains the existing 0.5 m horizontal emergency protection and makes it request land.
- `run_takeoff_1m_hold.sh`
  - Defaults horizontal relative target to zero, Z ramp to 2.5 s, Offboard landing speed to
    0.07 m/s, and passes EV health gate params. `Z_RAMP_SECONDS` and
    `OFFBOARD_LAND_SPEED_MPS` provide explicit per-run overrides.
- `record_takeoff_debug_bag.sh`
  - Records raw/healthy/diagnostic/PX4-velocity/setpoint topics and snapshots health code/params.
  - Full `/cloud_registered` recording is opt-in (`RECORD_POINTCLOUD=true`) to reduce flight-PC load.
- `check_takeoff_autostart_ready.sh`
  - Verifies the installed EV health executable and launch health arguments.
- `src/px4_ros_com/CMakeLists.txt`, `src/px4_ros_com/package.xml`
  - Install new node/module, add ROS dependencies, and register tests.
- `src/px4_ros_com/test/test_ev_health.py`
  - Covers forward/back signs, false EV forward drift, jump, invalid time/NaN, timeout,
    stationary decay, source age, time alignment, acceleration/reversal, effective-points,
    hysteresis, and coordinate/mount mapping.
- `src/px4_ros_com/test/test_minipc_mavros_offboard_ev_safety.py`
  - Covers pre-arm health continuity, OFFBOARD target lock/prestream refresh, pending-arm race,
    armed safety landing, 0.5 m protection, and AUTO.LAND confirmation/heartbeat behavior.
- `src/px4_ros_com/launch/prop_off_ev_validation.launch.py`
  - Propeller-off validation-only launch containing only the EV health monitor and MAVROS vision
    bridge. It contains no MAVROS launcher, Offboard controller, arming/mode service, or setpoint
    publisher. It overrides health recovery to 7.5 s only for this validation.
- `src/px4_ros_com/launch/prop_off_mavros_only.launch.py`
  - Starts MAVROS alone with the existing identity override and an internally supplied empty GCS
    URL. It works around ROS 2 CLI rejecting `gcs_url:=` and contains no EV/Offboard node.
- `run_prop_off_ev_validation.sh`
  - Interactive safety checks, phase markers and rosbag recording for the six-axis, stop-to-zero,
    FAST-LIO timeout/fault, and hysteresis-recovery hand test.
- `analyze_prop_off_ev_validation.py`
  - Offline rosbag analyzer for safety state, timestamp/rate/source-age, axis mapping, velocity
    settling, health transitions, output suppression, and recovery timing.
- `prop_off_safety_watchdog.py`
  - Read-only MAVROS-state watchdog. If `armed=true` or mode `OFFBOARD` appears at any time, it
    signals the parent validation script to stop recording/nodes; it sends no disarm/mode command.
- `PROP_OFF_EV_VALIDATION.md`
  - Exact no-prop prerequisites, execution steps, axis expectations, topics and acceptance limits.

`src/px4_ros_com/src/bridges/fastlio_odometry_guard.cpp` and several other project files
already contained earlier user/Codex changes. Preserve them; do not reset the worktree.

## Completed

- Located the actual bridge/guard/offboard implementations with `rg`.
- Correlated ULog and rosbag clocks (sub-millisecond alignment) and assigned the fault to
  FAST-LIO backlog plus bridge restamping, not an ENU/NED sign error.
- Implemented EV state machine, diagnostics, original-stamp velocity calculation, source-age
  rejection, common PX4-NED comparisons, covariance inflation, and fault output suppression.
- Integrated pre-arm gate and in-flight safety handling without changing PX4 parameters.
- Added repeatable Python tests for the requested cases and extra timing/safety regressions.
- Ran `git status --short` and `git diff --stat` at this checkpoint.
  - Output was about 2650 lines.
  - Aggregate stat: `1319 files changed, 2126 insertions(+), 198219 deletions(-)`.
  - Most deletions are pre-existing flight-record/history workspace changes unrelated to this task.
  - Do not stage, restore, or commit those unrelated paths without the user's explicit direction.
- Final verification completed on 2026-07-19:
  - `git diff --check`: passed.
  - Shell syntax checks for the four root launch/record/readiness scripts: passed.
  - Python compilation checks for the health core, health node, Offboard node, and launch file:
    passed.
  - Direct targeted tests: `33 passed in 0.50s` (EV health 21, Offboard safety 12).
  - Focused `ament_flake8` on the newly added Offboard safety test: passed after correcting
    one missing blank line.
  - `colcon build --packages-select px4_ros_com`: passed (`Finished <<< px4_ros_com`). This
    compiled the modified C++ bridges and installed the new Python node/module.
  - `colcon test --packages-select px4_ros_com --event-handlers console_direct+` executed.
    Both functional test suites passed (21/21 and 12/12), and `cppcheck` passed. The aggregate
    test result remains failing because seven package-wide lint gates fail on broad historical
    copyright/style/schema debt; see **Not completed yet**.
  - `colcon test-result --verbose` was run and recorded the same lint-only failures.
  - With `install/setup.bash` sourced, launch argument parsing via
    `ros2 launch px4_ros_com fastlio_mavros_autofix.launch.py --show-args`: passed. No nodes
    were launched.
- Continuation verification completed on 2026-07-21:
  - No task source file had changed since the 2026-07-19 checkpoint before verification began.
  - The full-workspace Python build failure was traced to user-site `setuptools 82.0.1`, which
    is incompatible with ROS Humble `colcon-python-setup-py 0.2.9` legacy `develop` options.
    System `setuptools 59.6.0` is compatible; no `offboard_nav2_planning` source edit was needed.
  - An old regular directory at
    `build/px4_msgs/ament_cmake_python/px4_msgs/px4_msgs` conflicted with symlink installation.
    It was preserved as `px4_msgs.pre_symlink_codex_20260721`; the expected generated symlink now
    occupies the original path. Nothing was deleted.
  - `PYTHONNOUSERSITE=1 colcon build --symlink-install`: passed, all four workspace packages
    finished (`px4_msgs`, `fastlio_global_slam`, `offboard_nav2_planning`, `px4_ros_com`).
  - Syntax checks and `git diff --check`: passed again.
  - Direct targeted tests: `33 passed in 0.50s` again.
  - Package test: EV health 21/21 and Offboard safety 12/12 passed; `cppcheck` passed; the same
    seven package-wide lint gates failed.
  - Latest `colcon test-result --verbose`: `1669 tests, 0 errors, 1631 failures, 11 skipped`.
    The large failure count expands individual historical lint findings; it is not 1631 behavior
    test failures.
  - Launch `--show-args`: passed again. No ROS nodes were launched.
- Propeller-off validation tooling generated and verified on 2026-07-21. It has not been connected
  to the aircraft or executed; actual measurements remain pending operator hand movement.
  - `bash -n run_prop_off_ev_validation.sh`: passed.
  - Python compilation and analyzer `--help`: passed.
  - Selected-package build: passed.
  - Dedicated launch `--show-args`: passed and shows validation recovery default `7.5` s.
  - Static audit found no arming/mode/setpoint implementation in the dedicated launch. Control
    topic names in the shell script are refusal checks and evidence-recording topics only.
  - Direct regression tests: `33 passed in 0.45s`.
  - Package test still has only the intentionally untouched seven lint gates; latest summary is
    `1671 tests, 0 errors, 1632 failures, 11 skipped` (individual lint findings, not behavior
    failures).

## Not completed yet

- The package-wide lint baseline is not clean. `copyright`, `cpplint`, `flake8`, `lint_cmake`,
  `pep257`, `uncrustify`, and `xmllint` fail across existing files (and some newly modified files
  also lack the repository's expected copyright/format normalization). The two behavioral suites
  and compilation pass; broad lint cleanup was intentionally not started.
- The EV propeller-off hand test passed at 16:16, but the newly added drift/takeoff safety latch
  has not been exercised in PX4 SITL or against real MAVROS mode-service responses. This does not
  authorize Offboard, arming, motor operation, parameter changes, propeller installation, or flight.

## Known risks at checkpoint time

- Full-workspace builds must currently use `PYTHONNOUSERSITE=1` so ROS Humble uses system
  `setuptools 59.6.0`; without it, user-site `setuptools 82.0.1` breaks ament-python symlink and
  ordinary installation. Do not downgrade or uninstall the user's Python package automatically.
- The recoverable old `px4_msgs` generated directory remains under `build/` as
  `px4_msgs.pre_symlink_codex_20260721`. It is not source and is not used by the successful build.
- Package-level `colcon test` stays red because of the broad lint baseline even though all 33
  behavior tests pass. Do not describe the entire package test suite as green.
- The repository is extremely dirty with unrelated deleted flight records and other user changes;
  broad Git cleanup/reset/commit operations are unsafe.
- The physical MID360 housing front reportedly points aircraft-right, while the currently
  validated interface offset is zero (`raw FAST-LIO +X -> PX4 NED +Y`). Do not change the offset
  automatically. Confirm with the propeller-off hand-held axis test.
- `/mavros/vision_speed/speed_twist_cov` is published for validation, but with
  `EKF2_EV_CTRL=11` PX4 does not fuse EV velocity. Do not recommend `15` until hand tests show
  forward positive, backward negative, quick zero at rest, agreement with PX4, and no spikes/delay.
- An optional effective-points Bool topic is supported but empty by default because FAST-LIO does
  not currently expose a confirmed status topic in this project.

## Build and test commands

```bash
cd ~/ws_offboard_control
source /opt/ros/humble/setup.bash

git diff --check
bash -n run_takeoff_1m_hold.sh record_takeoff_debug_bag.sh \
  check_takeoff_autostart_ready.sh start_takeoff_1m_stack.sh
python3 -m py_compile \
  src/px4_ros_com/px4_ros_com/ev_health.py \
  src/px4_ros_com/scripts/fastlio_ev_health_monitor.py \
  src/px4_ros_com/scripts/minipc_mavros_offboard.py \
  src/px4_ros_com/launch/fastlio_mavros_autofix.launch.py

ROS_PYTHONPATH="$PYTHONPATH"
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
PYTHONPATH="src/px4_ros_com:$ROS_PYTHONPATH" \
/usr/bin/python3 -m pytest -q \
  src/px4_ros_com/test/test_ev_health.py \
  src/px4_ros_com/test/test_minipc_mavros_offboard_ev_safety.py

PYTHONNOUSERSITE=1 colcon build --symlink-install
# A faster task-package-only rebuild is:
PYTHONNOUSERSITE=1 colcon build --packages-select px4_ros_com
source install/setup.bash
PYTHONNOUSERSITE=1 colcon test --packages-select px4_ros_com --event-handlers console_direct+
colcon test-result --verbose
ros2 launch px4_ros_com fastlio_mavros_autofix.launch.py --show-args
```

## Exact continuation order

1. Read this entire checkpoint.
2. Do not alter/reset unrelated dirty-worktree files.
3. Do not rerun or redesign the completed EV-health/Offboard functionality unless code changes.
4. Use `PYTHONNOUSERSITE=1` for every subsequent `colcon build/test` in this Humble workspace.
5. If package lint cleanliness is required, obtain explicit scope first, limit cleanup initially
   to task-specific files, then rerun `colcon test`; expect historical files to keep the aggregate
   result red until separately authorized for cleanup.
6. After any code edit, rerun the syntax checks, 42 targeted tests, selected-package build, package
   tests, test-result summary, and launch `--show-args` in that order.
7. The documented propeller-off EV hand test has passed. The next control-state validation must be
   PX4 SITL; do not use the real-aircraft one-click takeoff stack as a propeller-off substitute.
   Do not arm, fly, start real-aircraft Offboard, clean historical lint, or change PX4 parameters.

## Propeller-off hand validation status

- Protocol/tooling: generated; syntax/build/launch-argument verification passed.
- First operator attempts: safely refused before recording. The first attempt lacked
  `/mavros/state`; the next used the full `fastlio_mavros_autofix.launch.py` and was refused because
  `/fastlio_ev_health_monitor` and `/fastlio_mavros_vision_bridge` were already running.
- Direct `mavros node.launch gcs_url:=''` attempt failed before startup because ROS 2 launch rejects
  an empty CLI value. A dedicated `prop_off_mavros_only.launch.py` now supplies it internally;
  Python syntax, selected-package build and launch argument parsing passed, followed by
  `33 passed in 0.41s` regression tests. The MAVROS-only launch has not been executed by Codex.
- Read-only precheck after MAVROS connection: `connected=true`, `armed=false`, mode `STABILIZED`,
  no ROS publishers on MAVROS local-position setpoint topics, and MAVROS local velocity/odometry
  both approximately 30 Hz. This is precheck evidence only, not the hand-test result.
- Physical execution 2026-07-21 16:05: **COMPLETED BUT FAILED/INCONCLUSIVE; REPEAT REQUIRED**.
- Rosbag path:
  `validation_records/prop_off_ev_20260721_160508/rosbag` (261.60 s, 43,245 messages,
  complete metadata/database).
- Offline report:
  `validation_records/prop_off_ev_20260721_160508/validation_report.md`.
- Actual timestamp/frequency/source-age result: **PARTIAL PASS**. Raw `/Odometry` had 2,336
  samples, 0 non-monotonic source stamps and 8.93 Hz full-session average including the deliberate
  FAST-LIO stop. MAVROS odometry and local velocity were both 30.00 Hz. EV input age was min
  5.5 ms, median 13.8 ms, P95 42.7 ms, max 104.2 ms, below the 250 ms normal limit.
- Actual 7.5 s startup/recovery result: **FAIL/NOT TESTED**. No HEALTHY transition occurred because
  the health node's Reliable subscription was incompatible with MAVROS Best Effort local velocity.
- Actual six-axis ENU/NED sign result: **INCOMPLETE**. Raw FAST-LIO signs supported forward +X,
  backward -X, left +Y, right -Y and up +Z; the down displacement was too small to validate.
  Healthy and MAVROS vision chains were absent due to the QoS fault, so no end-to-end pass exists.
- Actual stop-to-zero result: **INCOMPLETE**. Diagnostic velocity reached <=0.05 m/s in 0.020-
  0.098 s for backward/left/right/up/down, but forward had no qualifying sample and the healthy
  output chain was unavailable.
- Actual FAST-LIO timeout/FAULT/output-suppression result: **INCOMPLETE**. The node was already
  FAULT (`px4_velocity_timeout`) before the deliberate stop; healthy output count during the marked
  fault interval was zero, but SUSPECT->FAULT and recovery hysteresis could not be measured.
- Safety result: **FAIL**. The bag contains three `armed=true`, mode `STABILIZED` samples from
  16:08:15.553 to 16:08:18.552. It contains zero OFFBOARD samples and zero recorded messages on
  `/mavros/setpoint_raw/local`, `/fmu/in/trajectory_setpoint`, and `/fmu/in/vehicle_command`.
  End state was confirmed `connected=true`, `armed=false`, `STABILIZED`.
- Root cause fixed after this bag: MAVROS velocity/odom subscriptions now use Best Effort sensor
  QoS. A safe monitor-only runtime check showed a matching subscriber, `HEALTHY`, PX4 velocity
  present, horizontal velocity difference 0.0058 m/s, input age 7.4 ms, and clean exit. The logging
  severity crash exposed during this check was also fixed. Syntax, build and 33 regression tests
  pass. That failed session was therefore rerun with the continuous safety watchdog; see below.

### Successful repeat: 2026-07-21 16:16

- Overall propeller-off validation: **PASS**.
- Rosbag path: `validation_records/prop_off_ev_20260721_161611/rosbag` (160.03 s,
  30,523 messages, complete metadata/database).
- Offline report: `validation_records/prop_off_ev_20260721_161611/validation_report.md`.
- Safety: **PASS**. All 169 MAVROS state samples were `armed=false`, mode `STABILIZED`; zero
  OFFBOARD samples and zero messages on the recorded local setpoint/trajectory/vehicle-command
  inputs. End state independently reconfirmed `connected=true`, `armed=false`, `STABILIZED`.
- Timestamp/frequency/source age: **PASS**. Raw `/Odometry` had 1,468 samples and zero
  non-monotonic source stamps; its 9.17 Hz full-session average includes the deliberate 13.305 s
  outage. MAVROS local odometry and velocity were both 30.00 Hz. EV source age was min 7.2 ms,
  median 9.1 ms, P95 15.2 ms, max 68.8 ms, below the 250 ms threshold.
- Health gate and recovery: **PASS**. First recorded health status to first HEALTHY transition was
  8.499 s (not earlier than 7.5 s). After the largest raw gap, SUSPECT occurred 0.502 s after the
  last raw sample and FAULT at 0.802 s. No `/Odometry/healthy` samples were emitted during FAULT
  before raw recovery. HEALTHY returned 7.603 s after the first recovered raw sample.
- Six-axis frame/sign chain: **PASS**. Raw, healthy and MAVROS vision deltas matched for all six
  motions. Observed physical signs were forward +ENU X, backward -ENU X, left +ENU Y,
  right -ENU Y, up +ENU Z and down -ENU Z. PX4 NED followed `(N,E,D)=(ENU y,ENU x,-ENU z)`;
  per-motion mapping errors were 0.012-0.078 m.
- Stop-to-zero: **PASS**. Filtered EV speed reached <=0.05 m/s in 0.033-0.124 s for all six
  directions, below the 1.0 s criterion.
- The bag recorder and EV validation nodes stopped cleanly. The safety watchdog emitted an
  `ExternalShutdownException` traceback only during normal parent cleanup; its handler was updated
  afterward to treat that expected shutdown as clean. This did not affect recorded data or safety.

Run only the procedure in `PROP_OFF_EV_VALIDATION.md`. The 16:05 bag is retained as failure
evidence and must not be reported as a pass. Repeat the complete test after the QoS/watchdog fix,
execute `analyze_prop_off_ev_validation.py`, and add a new result block with measured PASS/FAIL.

## Copy/paste continuation prompt

```text
Continue from ~/ws_offboard_control/CODEX_CHECKPOINT.md. Read it fully first. Preserve all
unrelated dirty-worktree changes. Do not modify PX4 parameters, arm, or fly. The selected package
builds with `PYTHONNOUSERSITE=1`; all 42 behavior tests pass. Historical package lint remains red
and must not be cleaned without explicit scope. The 2026-07-21 16:16 disarmed EV hand validation
passed. The drift landing trigger is now debounced, one-shot and latched; emergency drift uses the
same AUTO.LAND path without airborne kill/disarm; the first 3 seconds evaluate displacement,
speed, roll, pitch and EV health. Next validate these state transitions in PX4 SITL. Do not run
the real-aircraft one-click stack as a propeller-off test and do not resume propeller-on flight.
```
