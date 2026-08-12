#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
project_root="$(cd -- "${script_dir}/../.." && pwd -P)"

status_file="${project_root}/runtime/last_rosbag_status.txt"

if [[ ! -f "${status_file}" ]]; then
  echo "找不到 rosbag 状态文件：${status_file}"
  exit 0
fi

cat "${status_file}"
