#!/usr/bin/env bash
set -eo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
project_root="$(cd -- "${script_dir}/../.." && pwd -P)"

cd ${project_root}
source /opt/ros/humble/setup.bash
source ${project_root}/install/setup.bash
set -u

map_dir="${FASTLIO_GLOBAL_MAP_DIR:-${project_root}/maps/fastlio_global_3d}"
resolution="${FASTLIO_GLOBAL_MAP_RESOLUTION:-0.15}"

exec ros2 service call /fastlio_global_backend/save_map fastlio_global_slam/srv/SaveMap \
  "{directory: '${map_dir}', resolution: ${resolution}}"
