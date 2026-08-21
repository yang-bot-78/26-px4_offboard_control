#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
project_root="$(cd -- "${script_dir}/../.." && pwd -P)"
set +u
source /opt/ros/humble/setup.bash
source "${project_root}/install/setup.bash"
set -u

# Usage: 生成预设航点任务.sh /absolute/path/course.yaml [NAME ...]
mission_file="${1:?usage: $0 /absolute/path/course.yaml [NAME ...]}"
shift
if [[ "$#" -gt 0 && "$#" -lt 2 ]]; then
  echo "at least two waypoint names are required" >&2
  exit 2
fi
command=(ros2 run race_offboard export_mission_waypoints.py --mission-file "${mission_file}")
[[ "$#" -eq 0 ]] || command+=(--names "$@")
exec "${command[@]}"
