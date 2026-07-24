# EV velocity fusion pre-acceptance (propellers removed)

## Safety boundary

- Keep `EKF2_EV_CTRL=11`. Do not change any PX4 parameter.
- Remove all propellers. Keep the vehicle disarmed and motors unpowered.
- Start only the LiDAR driver, FAST-LIO, MAVROS, EV health monitor, and the
  single MAVROS ODOMETRY bridge.
- Never start `minipc_mavros_offboard.py`, `start_takeoff_1m_stack.sh`, any
  takeoff script, setpoint publisher, arming client, or mode-switch client.

The validation script refuses to run if it sees a known Offboard node, a control
topic publisher, an existing EV publisher, `armed=true`, or `mode=OFFBOARD`.

## Build before validation

```bash
cd ~/livox_mid360_env/ws_fastlio
source /opt/ros/humble/setup.bash
source ~/livox_mid360_env/install/setup.bash
PYTHONNOUSERSITE=1 colcon build --symlink-install --packages-select fast_lio

cd ~/ws_offboard_control
source /opt/ros/humble/setup.bash
source ~/livox_mid360_env/install/setup.bash
source install/setup.bash
PYTHONNOUSERSITE=1 colcon build --symlink-install --packages-select px4_ros_com
```

## Start permitted base processes

In separate terminals start only the existing LiDAR driver, the rebuilt FAST-LIO,
and MAVROS. Do not start `fastlio_mavros_autofix.launch.py` separately because the
validation script starts the health monitor and unique ODOMETRY bridge itself.

Confirm disarmed state:

```bash
ros2 topic echo --once /mavros/state
ros2 node list | rg 'offboard|takeoff|arm'
```

The state must contain `armed: false` and the node search must not show a control node.

## Run the recorded validation

```bash
cd ~/ws_offboard_control
source /opt/ros/humble/setup.bash
source ~/livox_mid360_env/install/setup.bash
source install/setup.bash
bash ./run_prop_off_ev_validation.sh
```

The script records a 60-second stationary interval, six body-direction translations,
a roughly 90-degree yaw followed by body-forward translation, FAST-LIO interruption,
and 7.5-second hysteretic recovery. It automatically writes
`validation_records/prop_off_ev_<time>/validation_report.md`.

## Live read-only checks

```bash
ros2 topic info -v /mavros/odometry/out
ros2 topic info -v /mavros/vision_pose/pose_cov
ros2 topic info -v /fmu/in/vehicle_visual_odometry
ros2 topic hz /Odometry
ros2 topic hz /Odometry/healthy
ros2 topic hz /mavros/odometry/out
ros2 topic echo --once /mavros/odometry/out
ros2 topic echo /mavros/odometry/out --field twist.twist.linear
ros2 topic echo /mavros/odometry/out --field twist.covariance
ros2 topic echo /ev_health/diagnostics
```

Acceptance requires exactly one publisher on `/mavros/odometry/out` and zero
publishers on both legacy/direct EV input topics. `header.stamp` must match the
source measurement; it must never be replaced with arrival time.

For body-FLU twist, expected signs are forward `x>0`, backward `x<0`, left `y>0`,
right `y<0`, up `z>0`, and down `z<0`. After the yaw rotation, forward must still
be `x>0` because twist is expressed in the child/body frame.

## PX4 receives velocity but does not fuse it

With `EKF2_EV_CTRL=11` unchanged, run these read-only commands in the QGC MAVLink
Console and save the complete output in the validation session `snapshot/` folder:

```text
listener vehicle_visual_odometry 20
listener estimator_status_flags 10
listener estimator_aid_src_ev_pos 10
listener estimator_aid_src_ev_vel 10
```

If a topic name is absent, run `uorb top` and `listener <candidate>` to identify the
equivalent topic for the installed PX4 build. Required evidence is finite, varying
three-axis velocity and variance in `vehicle_visual_odometry`; continuing position,
height, and yaw fusion; and `cs_ev_vel=false`. A velocity innovation/rejection flag
is not proof of fusion.

## Rate decision

The current LiDAR-rate FAST-LIO output is expected near 10 Hz. Do not duplicate
samples to claim a higher rate. At 10 Hz the result can be
`DATA_CORRECTNESS_PASS`, but remains `FUSION_RATE_READINESS_FAIL` unless the exact
installed PX4 behavior and measured timing provide separate safety evidence.

Until the generated report and PX4 console evidence pass every criterion:

`READY_FOR_EV_VEL_FUSION = NO`
