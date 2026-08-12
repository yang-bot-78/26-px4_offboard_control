#!/usr/bin/env bash
set -eo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
project_root="$(cd -- "${script_dir}/../.." && pwd -P)"

cd ${project_root}
source /opt/ros/humble/setup.bash
source ${project_root}/install/setup.bash
set -u

start_fastlio="${START_FASTLIO:-false}"
rviz="${RVIZ:-true}"
backend_config="${BACKEND_CONFIG:-}"

launch_args=(
  "start_fastlio:=${start_fastlio}"
  "rviz:=${rviz}"
)

if [[ -n "${backend_config}" ]]; then
  launch_args+=("backend_config:=${backend_config}")
fi

exec ros2 launch fastlio_global_slam fastlio_global_slam.launch.py "${launch_args[@]}"
