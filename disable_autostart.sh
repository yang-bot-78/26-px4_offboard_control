#!/usr/bin/env bash
set -euo pipefail

desktop_file="${HOME}/.config/autostart/ws_offboard_takeoff_stack.desktop"
service_file="${HOME}/.config/systemd/user/ws_offboard_rosbag_shutdown.service"

if [[ -f "${desktop_file}" ]]; then
  rm -f "${desktop_file}"
  echo "Autostart removed: ${desktop_file}"
else
  echo "Autostart file not present: ${desktop_file}"
fi

systemctl --user disable --now ws_offboard_rosbag_shutdown.service >/dev/null 2>&1 || true

if [[ -f "${service_file}" ]]; then
  rm -f "${service_file}"
  systemctl --user daemon-reload
  echo "Rosbag shutdown guard removed: ${service_file}"
else
  echo "Rosbag shutdown guard not present: ${service_file}"
fi
