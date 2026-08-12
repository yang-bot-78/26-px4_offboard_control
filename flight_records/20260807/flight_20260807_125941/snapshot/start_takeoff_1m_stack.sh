#!/usr/bin/env bash
set -euo pipefail

driver_delay="${DRIVER_DELAY_SEC:-1}"
fastlio_delay="${FASTLIO_DELAY_SEC:-2}"
mavros_delay="${MAVROS_DELAY_SEC:-3}"
bag_delay="${BAG_DELAY_SEC:-2}"
readiness_timeout_sec="${READINESS_TIMEOUT_SEC:-90}"
flight_timestamp="${FLIGHT_TIMESTAMP:-$(date +%Y%m%d_%H%M%S)}"
flight_date="${flight_timestamp%%_*}"
flight_run_dir="${FLIGHT_RUN_DIR:-${HOME}/ws_offboard_control/flight_records/${flight_date}/flight_${flight_timestamp}}"
flight_log_dir="${flight_run_dir}/logs"
user_systemd_dir="${HOME}/.config/systemd/user"
service_name="ws_offboard_rosbag_shutdown.service"
service_src="${HOME}/ws_offboard_control/${service_name}"
service_dst="${user_systemd_dir}/${service_name}"
lever_arm_config="${MID360_LEVER_ARM_CONFIG:-${HOME}/ws_offboard_control/src/px4_ros_com/config/mid360_lever_arm.conf}"
lever_arm_validator="${HOME}/ws_offboard_control/validate_mid360_lever_arm_config.sh"
readiness_helper="${HOME}/ws_offboard_control/wait_for_ros_readiness.sh"

require_file() {
  local path="$1"
  if [[ ! -e "$path" ]]; then
    echo "missing required file: $path" >&2
    exit 1
  fi
}

open_window() {
  local title="$1"
  local log_name="$2"
  local cmd="$3"
  gnome-terminal --title="$title" -- bash -lc "export FLIGHT_TIMESTAMP='${flight_timestamp}'; export FLIGHT_RUN_DIR='${flight_run_dir}'; mkdir -p '${flight_log_dir}'; { $cmd; } 2>&1 | tee -a '${flight_log_dir}/${log_name}.log'; status=\${PIPESTATUS[0]}; echo; echo '[${title}] exited with status' \${status}; exec bash" &
}

require_file "${HOME}/livox_mid360_env/run_mid360_driver.sh"
require_file "${HOME}/livox_mid360_env/run_fastlio_mid360.sh"
require_file "${HOME}/ws_offboard_control/record_takeoff_debug_bag.sh"
require_file "${HOME}/ws_offboard_control/refresh_last_rosbag_status.sh"
require_file "${HOME}/ws_offboard_control/show_last_rosbag_status.sh"
require_file "${HOME}/ws_offboard_control/stop_takeoff_debug_bag.sh"
require_file "${HOME}/ws_offboard_control/run_takeoff_1m_hold.sh"
require_file "${HOME}/ws_offboard_control/check_takeoff_autostart_ready.sh"
require_file "${lever_arm_validator}"
require_file "${readiness_helper}"
require_file "${lever_arm_config}"
require_file "${HOME}/ws_offboard_control/install/setup.bash"
require_file "${service_src}"

"${lever_arm_validator}" "${lever_arm_config}"
source "${lever_arm_config}"

if [[ ! "${readiness_timeout_sec}" =~ ^[0-9]+$ ]] || (( readiness_timeout_sec < 1 )); then
  echo "READINESS_TIMEOUT_SEC must be a positive integer" >&2
  exit 1
fi

if ! command -v gnome-terminal >/dev/null 2>&1; then
  echo "gnome-terminal not found" >&2
  exit 1
fi

mkdir -p "${flight_run_dir}"
mkdir -p "${flight_log_dir}"
mkdir -p "${user_systemd_dir}"
cp "${lever_arm_config}" "${flight_run_dir}/mid360_lever_arm.conf"
cd "${HOME}/ws_offboard_control"

if [[ "${SKIP_PREFLIGHT_CHECK:-0}" != "1" ]]; then
  ./check_takeoff_autostart_ready.sh
fi

./refresh_last_rosbag_status.sh
cp "${service_src}" "${service_dst}"
chmod 644 "${service_dst}"
systemctl --user daemon-reload || true
systemctl --user enable --now "${service_name}" >/dev/null 2>&1 || true
echo "flight run dir: ${flight_run_dir}"
echo "flight log dir: ${flight_log_dir}"
echo "MID360 body-to-sensor FLU: x=${MID360_BODY_TO_SENSOR_X_M} m y=${MID360_BODY_TO_SENSOR_Y_M} m z=${MID360_BODY_TO_SENSOR_Z_M} m yaw=${MID360_BODY_TO_FASTLIO_YAW_RAD} rad"

open_window "MID360 Driver" "mid360_driver" \
  "cd '${HOME}/livox_mid360_env' && ./run_mid360_driver.sh"

open_window "FAST-LIO" "fastlio" \
  "sleep ${driver_delay}; source /opt/ros/humble/setup.bash && '${readiness_helper}' topic '${readiness_timeout_sec}' /livox/imu && '${readiness_helper}' topic '${readiness_timeout_sec}' /livox/lidar && cd '${HOME}/livox_mid360_env' && ./run_fastlio_mid360.sh"

open_window "PX4 MAVROS" "px4_mavros" \
  "sleep $((driver_delay + fastlio_delay)); source /opt/ros/humble/setup.bash && source '${HOME}/ws_offboard_control/install/setup.bash' && '${readiness_helper}' topic '${readiness_timeout_sec}' /Odometry && cd '${HOME}/ws_offboard_control' && ros2 launch px4_ros_com fastlio_mavros_autofix.launch.py fcu_url:=serial:///dev/ttyUSB0:921600?ids=255,190 start_mavros_vision_bridge:=true start_bridge:=false start_px4_ev_bridge:=false vision_world_yaw_alignment_rad:=0.0 body_to_sensor_x_m:=${MID360_BODY_TO_SENSOR_X_M} body_to_sensor_y_m:=${MID360_BODY_TO_SENSOR_Y_M} body_to_sensor_z_m:=${MID360_BODY_TO_SENSOR_Z_M} body_to_fastlio_yaw_rad:=${MID360_BODY_TO_FASTLIO_YAW_RAD}"

open_window "ROS Bag Debug" "rosbag_debug" \
  "sleep $((driver_delay + fastlio_delay + mavros_delay)); source /opt/ros/humble/setup.bash && '${readiness_helper}' mavros_connected '${readiness_timeout_sec}' && '${readiness_helper}' topic '${readiness_timeout_sec}' /ev_health/status && cd '${HOME}/ws_offboard_control' && ./record_takeoff_debug_bag.sh"

open_window "Takeoff 0.8m Hold" "takeoff_0p8m_hold" \
  "sleep $((driver_delay + fastlio_delay + mavros_delay + bag_delay)); source /opt/ros/humble/setup.bash && '${readiness_helper}' mavros_connected '${readiness_timeout_sec}' && '${readiness_helper}' topic '${readiness_timeout_sec}' /mavros/local_position/odom && '${readiness_helper}' healthy '${readiness_timeout_sec}' /ev_health/status && cd '${HOME}/ws_offboard_control' && ./run_takeoff_1m_hold.sh"

wait
