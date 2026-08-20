#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
project_root="$(cd -- "${script_dir}/../.." && pwd -P)"
set +u
source /opt/ros/humble/setup.bash
source "${project_root}/install/setup.bash"
set -u

# Usage: 保存航点.sh B1 [xy|xyz] [height] [map.pcd]
name="${1:?usage: $0 B1|B2|C|D [xy|xyz] [height] [map.pcd]}"
mode="${2:-xy}"
height="${3:-0.78}"
map_file="${4:-${MAP_FILE:-}}"
command=(ros2 run race_offboard save_waypoint.py --name "${name}" --mode "${mode}" --flight-z "${height}")
if [[ -n "${map_file}" ]]; then
  command+=(--map-file "${map_file}")
fi
exec "${command[@]}"
