#!/usr/bin/env bash
set -eo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
project_root="$(cd -- "${script_dir}/../.." && pwd -P)"

cd ${project_root}
source /opt/ros/humble/setup.bash
source ${project_root}/install/setup.bash
set -u

export BACKEND_CONFIG="${BACKEND_CONFIG:-${project_root}/install/fastlio_global_slam/share/fastlio_global_slam/config/relocalization.yaml}"
# 重定位后端默认不应打开 FAST-LIO 的 RViz。
# Nav2 路线验证用的是 nav2_stage1_planning.rviz，由后面的 run_nav2_relocalized_map.sh 启动。
export RVIZ="${RVIZ:-false}"
export START_FASTLIO="${START_FASTLIO:-false}"

exec ${project_root}/tools/fastlio/运行全局建图.sh
