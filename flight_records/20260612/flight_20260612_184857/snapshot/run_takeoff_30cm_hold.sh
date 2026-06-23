#!/usr/bin/env bash
set -eo pipefail

cd /home/robot/ws_offboard_control
source /opt/ros/humble/setup.bash
source /home/robot/ws_offboard_control/install/setup.bash
set -u

# Relative target in NED: -0.50 m means climb 50 cm.
exec ros2 run px4_ros_com minipc_mavros_offboard.py --ros-args \
  -p arm_only:=false \
  -p use_rc_offboard:=true \
  -p prestream_count:=100 \
  -p recent_pose_samples:=30 \
  -p require_vision_pose:=true \
  -p vision_freshness_s:=1.0 \
  -p lift_only_seconds:=2.5 \
  -p z_ramp_seconds:=2.0 \
  -p target_x_m:=0.0 \
  -p target_y_m:=0.0 \
  -p target_z_m:=-0.50 \
  -p drift_guard_enabled:=true \
  -p max_horizontal_drift_m:=0.50 \
  -p drift_guard_action:=land \
  -p hover_seconds:=10.0 \
  -p auto_land:=false \
  -p offboard_stabilize_seconds:=20.0
