#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
project_root="$(cd -- "${script_dir}/../.." && pwd -P)"
set +u
source /opt/ros/humble/setup.bash
source "${project_root}/install/setup.bash"
set -u

# Usage: 生成预设航点任务.sh /absolute/path/course.yaml [B1 B2 C D]
mission_file="${1:?usage: $0 /absolute/path/course.yaml [B1 B2 C D]}"
shift
if [[ "$#" -eq 0 ]]; then
  names=(B1 B2 C D)
else
  names=("$@")
fi
if [[ "${#names[@]}" -ne 4 ]]; then
  echo "exactly four names are required: B1 B2 C D" >&2
  exit 2
fi
exec ros2 run race_offboard export_mission_waypoints.py \
  --mission-file "${mission_file}" --names "${names[@]}"
