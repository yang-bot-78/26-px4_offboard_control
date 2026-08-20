#!/usr/bin/env bash
set -euo pipefail

# Map-localized global planning task.  Default is deliberately propeller-off:
# it starts neither MAVROS/EV nor the Offboard control node.

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
project_root="$(cd -- "${script_dir}/../.." && pwd -P)"
stack_script="${script_dir}/一键启动导航栈.sh"
mission_file="${MISSION_FILE:-${project_root}/src/race_offboard/config/waypoints/main/mission.yaml}"
waypoints_file="${WAYPOINTS_FILE:-${project_root}/src/race_offboard/config/waypoints/main/waypoints.yaml}"
map_file="${MAP_FILE:-${project_root}/maps/main/GlobalMap_去顶端_z2.5m.pcd}"
px4_components_enabled="${PX4_COMPONENTS_ENABLED:-false}"
relocalization_enabled="${RELOCALIZATION_ENABLED:-true}"
rviz="${RVIZ:-true}"

[[ -x "${stack_script}" ]] || { echo "错误：找不到导航启动脚本：${stack_script}" >&2; exit 2; }
[[ -f "${mission_file}" ]] || { echo "错误：任务航点文件不存在：${mission_file}" >&2; exit 2; }
[[ -f "${waypoints_file}" ]] || { echo "错误：保存航点文件不存在：${waypoints_file}" >&2; exit 2; }
[[ -s "${map_file}" ]] || { echo "错误：地图文件不存在或为空：${map_file}" >&2; exit 2; }
[[ "${px4_components_enabled}" == true || "${px4_components_enabled}" == false ]] || {
  echo "错误：PX4_COMPONENTS_ENABLED 必须是 true 或 false" >&2; exit 2;
}
[[ "${relocalization_enabled}" == true || "${relocalization_enabled}" == false ]] || {
  echo "错误：RELOCALIZATION_ENABLED 必须是 true 或 false" >&2; exit 2;
}
[[ "${rviz}" == true || "${rviz}" == false ]] || {
  echo "错误：RVIZ 必须是 true 或 false" >&2; exit 2;
}

if ! python3 - "${mission_file}" <<'PY'
import sys
import yaml

with open(sys.argv[1], encoding='utf-8') as stream:
    document = yaml.safe_load(stream) or {}
points = document.get('preset_points')
if not isinstance(points, list) or len(points) != 4:
    raise SystemExit('任务文件必须包含四个 preset_points：B1、B2、C、D')
PY
then
  exit 2
fi

echo "全局航点规划：${mission_file}"
echo "PX4/MAVROS/控制节点：${px4_components_enabled}"
echo "全局重定位：${relocalization_enabled}"
echo "按 Ctrl-C 停止本次导航栈。"

exec env \
  MAP_FILE="${map_file}" MISSION_FILE="${mission_file}" WAYPOINTS_FILE="${waypoints_file}" \
  PLANNER_BACKEND=super MISSION_ENABLED=false \
  GLOBAL_WAYPOINT_TASK_ENABLED=true PUBLISH_GLOBAL_PATH=true WAYPOINT_VISUALIZER_ENABLED=true \
  ENABLE_OUTPUT=false PX4_COMPONENTS_ENABLED="${px4_components_enabled}" \
  RELOCALIZATION_ENABLED="${relocalization_enabled}" \
  RECORD_BAG=false RVIZ="${rviz}" COMPONENT_WINDOWS=false \
  "${stack_script}"
