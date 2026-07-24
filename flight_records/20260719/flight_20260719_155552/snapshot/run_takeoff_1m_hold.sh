#!/usr/bin/env bash
set -eo pipefail

cd /home/robot/ws_offboard_control
source /opt/ros/humble/setup.bash
source /home/robot/ws_offboard_control/install/setup.bash
set -u

control_mode="${CONTROL_MODE:-ramp}"
target_x_m="${TARGET_X_M:-0.0}"
target_y_m="${TARGET_Y_M:-0.0}"
target_z_m="${TARGET_Z_M:--0.50}"
z_ramp_seconds="${Z_RAMP_SECONDS:-1.0}"
xy_move_unlock_height_m="${XY_MOVE_UNLOCK_HEIGHT_M:-0.50}"
hover_seconds="${HOVER_SECONDS:-10.0}"
auto_disarm_after_land="${AUTO_DISARM_AFTER_LAND:-true}"
disarm_after_land_delay_s="${DISARM_AFTER_LAND_DELAY_S:-2.0}"
land_wait_timeout_s="${LAND_WAIT_TIMEOUT_S:-45.0}"
landed_ground_tolerance_m="${LANDED_GROUND_TOLERANCE_M:-0.06}"
enable_route_tracking="${ENABLE_ROUTE_TRACKING:-false}"
route_path_topic="${ROUTE_PATH_TOPIC:-/nav2_stage1/path}"
route_fixed_flight_height_m="${ROUTE_FIXED_FLIGHT_HEIGHT_M:-1.0}"
route_start_tolerance_m="${ROUTE_START_TOLERANCE_M:-0.40}"
route_goal_tolerance_m="${ROUTE_GOAL_TOLERANCE_M:-0.20}"
route_lookahead_m="${ROUTE_LOOKAHEAD_M:-0.40}"
route_max_vxy_mps="${ROUTE_MAX_VXY_MPS:-0.25}"
route_max_vz_mps="${ROUTE_MAX_VZ_MPS:-0.15}"
route_setpoint_rate_hz="${ROUTE_SETPOINT_RATE_HZ:-30.0}"
route_path_timeout_s="${ROUTE_PATH_TIMEOUT_S:-2.0}"
route_fail_action="${ROUTE_FAIL_ACTION:-hold}"
route_final_hold_seconds="${ROUTE_FINAL_HOLD_SECONDS:-3.0}"

positive_number_re='^[0-9]+([.][0-9]+)?$'
if [[ ! "${z_ramp_seconds}" =~ ${positive_number_re} ]]; then
  echo "Z_RAMP_SECONDS must be a non-negative decimal number" >&2
  exit 1
fi

# Relative target in NED: climb 0.5 m in place, hold, then use the existing landing path.
exec ros2 run px4_ros_com minipc_mavros_offboard.py --ros-args \
  -p control_mode:="${control_mode}" \
  -p arm_only:=false \
  -p use_rc_offboard:=true \
  -p prestream_count:=100 \
  -p recent_pose_samples:=30 \
  -p require_vision_pose:=true \
  -p vision_freshness_s:=1.0 \
  -p lift_only_seconds:=0.0 \
  -p z_ramp_seconds:="${z_ramp_seconds}" \
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
  -p auto_disarm_after_land:="${auto_disarm_after_land}" \
  -p disarm_after_land_delay_s:="${disarm_after_land_delay_s}" \
  -p land_wait_timeout_s:="${land_wait_timeout_s}" \
  -p landed_ground_tolerance_m:="${landed_ground_tolerance_m}" \
  -p target_reached_tolerance_m:=0.15 \
  -p land_on_offboard_loss:=true \
  -p offboard_stabilize_seconds:=20.0 \
  -p enable_route_tracking:="${enable_route_tracking}" \
  -p route_path_topic:="${route_path_topic}" \
  -p route_fixed_flight_height_m:="${route_fixed_flight_height_m}" \
  -p route_start_tolerance_m:="${route_start_tolerance_m}" \
  -p route_goal_tolerance_m:="${route_goal_tolerance_m}" \
  -p route_lookahead_m:="${route_lookahead_m}" \
  -p route_max_vxy_mps:="${route_max_vxy_mps}" \
  -p route_max_vz_mps:="${route_max_vz_mps}" \
  -p route_setpoint_rate_hz:="${route_setpoint_rate_hz}" \
  -p route_path_timeout_s:="${route_path_timeout_s}" \
  -p route_fail_action:="${route_fail_action}" \
  -p route_final_hold_seconds:="${route_final_hold_seconds}"
