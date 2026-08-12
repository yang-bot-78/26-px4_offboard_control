#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
project_root="$(cd -- "${script_dir}/../.." && pwd -P)"

desktop_file="${HOME}/.config/autostart/ws_offboard_takeoff_stack.desktop"
service_file="${HOME}/.config/systemd/user/ws_offboard_rosbag_shutdown.service"

if [[ -f "${desktop_file}" ]]; then
  rm -f "${desktop_file}"
  echo "开机自启已移除：${desktop_file}"
else
  echo "开机自启文件不存在：${desktop_file}"
fi

systemctl --user disable --now ws_offboard_rosbag_shutdown.service >/dev/null 2>&1 || true

if [[ -f "${service_file}" ]]; then
  rm -f "${service_file}"
  systemctl --user daemon-reload
  echo "rosbag 关机保护已移除：${service_file}"
else
  echo "rosbag 关机保护不存在：${service_file}"
fi
