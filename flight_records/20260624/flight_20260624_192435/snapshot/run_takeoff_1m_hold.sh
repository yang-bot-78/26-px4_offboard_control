#!/usr/bin/env bash
set -eo pipefail

cd /home/robot/ws_offboard_control
source /opt/ros/humble/setup.bash
source /home/robot/ws_offboard_control/install/setup.bash
set -u

control_mode="${CONTROL_MODE:-ramp}"
target_x_m="${TARGET_X_M:-1.0}"
target_y_m="${TARGET_Y_M:-0.0}"
target_z_m="${TARGET_Z_M:--0.50}"
xy_move_unlock_height_m="${XY_MOVE_UNLOCK_HEIGHT_M:-0.50}"
hover_seconds="${HOVER_SECONDS:-10.0}"

# Relative target in NED: climb 0.5 m first, then allow XY motion; y=0.5 m maps to local +X.
exec ros2 run px4_ros_com minipc_mavros_offboard.py --ros-args \
  -p control_mode:="${control_mode}" \
  -p arm_only:=false \
  -p use_rc_offboard:=true \
  -p prestream_count:=100 \
  -p recent_pose_samples:=30 \
  -p require_vision_pose:=true \
  -p vision_freshness_s:=1.0 \
  -p lift_only_seconds:=2.5 \
  -p z_ramp_seconds:=2.0 \
  -p target_x_m:="${target_x_m}" \
  -p target_y_m:="${target_y_m}" \
  -p target_z_m:="${target_z_m}" \
  -p xy_move_unlock_height_m:="${xy_move_unlock_height_m}" \
  -p drift_guard_enabled:=true \
  -p drift_warning_m:=0.20 \
  -p drift_land_m:=0.30 \
  -p drift_emergency_m:=0.50 \
  -p hover_seconds:="${hover_seconds}" \
  -p auto_land:=false \
  -p offboard_land:=true \
  -p offboard_land_speed_mps:=0.10 \
  -p offboard_land_auto_handoff_height_m:=0.10 \
  -p target_reached_tolerance_m:=0.15 \
  -p land_on_offboard_loss:=true \
  -p offboard_stabilize_seconds:=20.0
