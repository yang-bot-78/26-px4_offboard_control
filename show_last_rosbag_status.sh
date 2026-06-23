#!/usr/bin/env bash
set -euo pipefail

status_file="${HOME}/ws_offboard_control/runtime/last_rosbag_status.txt"

if [[ ! -f "${status_file}" ]]; then
  echo "No rosbag status file found: ${status_file}"
  exit 0
fi

cat "${status_file}"
