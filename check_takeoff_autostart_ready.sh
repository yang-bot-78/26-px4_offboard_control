#!/usr/bin/env bash
set -euo pipefail

project_root="${HOME}/ws_offboard_control"
quiet=false

if [[ "${1:-}" == "--quiet" ]]; then
  quiet=true
fi

say() {
  if [[ "${quiet}" != true ]]; then
    echo "$@"
  fi
}

require_file() {
  local path="$1"
  if [[ ! -f "${path}" ]]; then
    echo "missing required file: ${path}" >&2
    exit 1
  fi
}

require_executable() {
  local path="$1"
  require_file "${path}"
  if [[ ! -x "${path}" ]]; then
    echo "required file is not executable: ${path}" >&2
    exit 1
  fi
}

require_command() {
  local name="$1"
  if ! command -v "${name}" >/dev/null 2>&1; then
    echo "missing command: ${name}" >&2
    exit 1
  fi
}

require_ros_executable() {
  local executable="$1"
  if ! ros2 pkg executables px4_ros_com | awk '{print $2}' | grep -Fxq "${executable}"; then
    echo "px4_ros_com executable not installed: ${executable}" >&2
    exit 1
  fi
}

say "Checking offboard autostart prerequisites..."

require_command gnome-terminal
require_command systemctl

require_file "/opt/ros/humble/setup.bash"
require_file "${project_root}/install/setup.bash"
require_executable "${project_root}/start_takeoff_1m_stack.sh"
require_executable "${project_root}/start_takeoff_1m_stack_login.sh"
require_executable "${project_root}/run_takeoff_1m_hold.sh"
require_executable "${project_root}/run_hover_1m_offboard.sh"
require_executable "${project_root}/record_takeoff_debug_bag.sh"
require_executable "${project_root}/stop_takeoff_debug_bag.sh"
require_executable "${project_root}/refresh_last_rosbag_status.sh"
require_file "${project_root}/ws_offboard_rosbag_shutdown.service"
require_executable "${HOME}/livox_mid360_env/run_mid360_driver.sh"
require_executable "${HOME}/livox_mid360_env/run_fastlio_mid360.sh"

set +u
source /opt/ros/humble/setup.bash
source "${project_root}/install/setup.bash"
set -u

require_command ros2
require_ros_executable minipc_mavros_offboard.py
require_ros_executable fastlio_odometry_guard
require_ros_executable fastlio_mavros_vision_bridge
require_ros_executable check_fastlio_vision_yaw.py

if ! ros2 launch px4_ros_com fastlio_mavros_autofix.launch.py --show-args |
  grep -q "vision_position_yaw_offset_rad"; then
  echo "fastlio_mavros_autofix.launch.py does not expose vision_position_yaw_offset_rad" >&2
  exit 1
fi

if [[ ! -e /dev/ttyUSB0 ]]; then
  echo "warning: /dev/ttyUSB0 is not present yet; MAVROS may wait/fail until FCU serial appears" >&2
fi

"${project_root}/refresh_last_rosbag_status.sh" || true

say "Autostart prerequisites look ready."
