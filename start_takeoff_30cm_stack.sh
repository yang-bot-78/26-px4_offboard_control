#!/usr/bin/env bash
set -euo pipefail

driver_delay="${DRIVER_DELAY_SEC:-3}"
fastlio_delay="${FASTLIO_DELAY_SEC:-5}"
mavros_delay="${MAVROS_DELAY_SEC:-8}"

require_file() {
  local path="$1"
  if [[ ! -e "$path" ]]; then
    echo "missing required file: $path" >&2
    exit 1
  fi
}

open_window() {
  local title="$1"
  local cmd="$2"
  gnome-terminal --title="$title" -- bash -lc "$cmd; exec bash" &
}

require_file "${HOME}/livox_mid360_env/run_mid360_driver.sh"
require_file "${HOME}/livox_mid360_env/run_fastlio_mid360.sh"
require_file "${HOME}/ws_offboard_control/run_takeoff_30cm_hold.sh"
require_file "${HOME}/ws_offboard_control/install/setup.bash"

if ! command -v gnome-terminal >/dev/null 2>&1; then
  echo "gnome-terminal not found" >&2
  exit 1
fi

open_window "MID360 Driver" \
  "cd '${HOME}/livox_mid360_env' && ./run_mid360_driver.sh"

open_window "FAST-LIO" \
  "sleep ${driver_delay}; cd '${HOME}/livox_mid360_env' && ./run_fastlio_mid360.sh"

open_window "PX4 MAVROS" \
  "sleep $((driver_delay + fastlio_delay)); source /opt/ros/humble/setup.bash && source '${HOME}/ws_offboard_control/install/setup.bash' && cd '${HOME}/ws_offboard_control' && ros2 launch px4_ros_com fastlio_mavros_autofix.launch.py fcu_url:=serial:///dev/ttyUSB0:921600?ids=255,190"

open_window "Takeoff 30cm" \
  "sleep $((driver_delay + fastlio_delay + mavros_delay)); cd '${HOME}/ws_offboard_control' && ./run_takeoff_30cm_hold.sh"

wait
