#!/usr/bin/env bash
set -eo pipefail

cd /home/robot/ws_offboard_control
source /opt/ros/humble/setup.bash
source /home/robot/ws_offboard_control/install/setup.bash
set -u

control_mode="${CONTROL_MODE:-ramp}"
target_x_m="${TARGET_X_M:-0.0}"
target_y_m="${TARGET_Y_M:-0.0}"
target_z_m="${TARGET_Z_M:--0.80}"
z_ramp_seconds="${Z_RAMP_SECONDS:-2.5}"
offboard_land_speed_mps="${OFFBOARD_LAND_SPEED_MPS:-0.07}"
xy_move_unlock_height_m="${XY_MOVE_UNLOCK_HEIGHT_M:-0.50}"
hover_seconds="${HOVER_SECONDS:-5.0}"
wait_for_land_command="${WAIT_FOR_LAND_COMMAND:-true}"
ev_health_topic="${EV_HEALTH_TOPIC:-/ev_health/status}"
ev_health_required_s="${EV_HEALTH_REQUIRED_S:-7.5}"
ev_health_freshness_s="${EV_HEALTH_FRESHNESS_S:-1.0}"
drift_land_trigger_duration_s="${DRIFT_LAND_TRIGGER_DURATION_S:-0.30}"
drift_emergency_trigger_duration_s="${DRIFT_EMERGENCY_TRIGGER_DURATION_S:-0.10}"
takeoff_transient_guard_enabled="${TAKEOFF_TRANSIENT_GUARD_ENABLED:-true}"
takeoff_transient_window_s="${TAKEOFF_TRANSIENT_WINDOW_S:-3.0}"
takeoff_transient_trigger_duration_s="${TAKEOFF_TRANSIENT_TRIGGER_DURATION_S:-0.20}"
takeoff_transient_max_displacement_m="${TAKEOFF_TRANSIENT_MAX_DISPLACEMENT_M:-0.50}"
takeoff_transient_max_speed_mps="${TAKEOFF_TRANSIENT_MAX_SPEED_MPS:-0.50}"
takeoff_transient_max_roll_deg="${TAKEOFF_TRANSIENT_MAX_ROLL_DEG:-10.0}"
takeoff_transient_max_pitch_deg="${TAKEOFF_TRANSIENT_MAX_PITCH_DEG:-10.0}"
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
if [[ ! "${offboard_land_speed_mps}" =~ ${positive_number_re} ]]; then
  echo "OFFBOARD_LAND_SPEED_MPS must be a non-negative decimal number" >&2
  exit 1
fi
if [[ ! "${hover_seconds}" =~ ${positive_number_re} ]]; then
  echo "HOVER_SECONDS must be a non-negative decimal number" >&2
  exit 1
fi
if [[ ! "${ev_health_required_s}" =~ ${positive_number_re} ]]; then
  echo "EV_HEALTH_REQUIRED_S must be a non-negative decimal number" >&2
  exit 1
fi
if [[ ! "${ev_health_freshness_s}" =~ ${positive_number_re} ]]; then
  echo "EV_HEALTH_FRESHNESS_S must be a non-negative decimal number" >&2
  exit 1
fi
for value_name in \
  drift_land_trigger_duration_s \
  drift_emergency_trigger_duration_s \
  takeoff_transient_window_s \
  takeoff_transient_trigger_duration_s \
  takeoff_transient_max_displacement_m \
  takeoff_transient_max_speed_mps \
  takeoff_transient_max_roll_deg \
  takeoff_transient_max_pitch_deg; do
  if [[ ! "${!value_name}" =~ ${positive_number_re} ]]; then
    echo "${value_name} must be a non-negative decimal number" >&2
    exit 1
  fi
done

# Relative target in NED: climb 0.8 m, hold position, and wait for terminal command "land".
exec ros2 run px4_ros_com minipc_mavros_offboard.py --ros-args \
  -p control_mode:="${control_mode}" \
  -p arm_only:=false \
  -p use_rc_offboard:=true \
  -p prestream_count:=100 \
  -p recent_pose_samples:=30 \
  -p require_vision_pose:=true \
  -p vision_freshness_s:=1.0 \
  -p require_ev_health:=true \
  -p ev_health_topic:="${ev_health_topic}" \
  -p ev_health_required_s:="${ev_health_required_s}" \
  -p ev_health_freshness_s:="${ev_health_freshness_s}" \
  -p lift_only_seconds:=0.0 \
  -p z_ramp_seconds:="${z_ramp_seconds}" \
  -p target_x_m:="${target_x_m}" \
  -p target_y_m:="${target_y_m}" \
  -p target_z_m:="${target_z_m}" \
  -p xy_move_unlock_height_m:="${xy_move_unlock_height_m}" \
  -p drift_guard_enabled:=true \
  -p drift_warning_m:=0.20 \
  -p drift_land_m:=0.50 \
  -p drift_emergency_m:=0.60 \
  -p drift_land_trigger_duration_s:="${drift_land_trigger_duration_s}" \
  -p drift_emergency_trigger_duration_s:="${drift_emergency_trigger_duration_s}" \
  -p takeoff_transient_guard_enabled:="${takeoff_transient_guard_enabled}" \
  -p takeoff_transient_window_s:="${takeoff_transient_window_s}" \
  -p takeoff_transient_trigger_duration_s:="${takeoff_transient_trigger_duration_s}" \
  -p takeoff_transient_max_displacement_m:="${takeoff_transient_max_displacement_m}" \
  -p takeoff_transient_max_speed_mps:="${takeoff_transient_max_speed_mps}" \
  -p takeoff_transient_max_roll_deg:="${takeoff_transient_max_roll_deg}" \
  -p takeoff_transient_max_pitch_deg:="${takeoff_transient_max_pitch_deg}" \
  -p hover_seconds:="${hover_seconds}" \
  -p auto_land:=false \
  -p offboard_land:=true \
  -p offboard_land_speed_mps:="${offboard_land_speed_mps}" \
  -p offboard_land_auto_handoff_height_m:=0.30 \
  -p auto_disarm_after_land:="${auto_disarm_after_land}" \
  -p disarm_after_land_delay_s:="${disarm_after_land_delay_s}" \
  -p land_wait_timeout_s:="${land_wait_timeout_s}" \
  -p landed_ground_tolerance_m:="${landed_ground_tolerance_m}" \
  -p target_reached_tolerance_m:=0.15 \
  -p enable_spin_test:=false \
  -p wait_for_land_command:="${wait_for_land_command}" \
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
