#!/usr/bin/env bash
set -eo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
project_root="$(cd -- "${script_dir}/../.." && pwd -P)"

cd ${project_root}
source /opt/ros/humble/setup.bash
source ${project_root}/install/setup.bash
set -u

service_name="/fastlio_global_backend/relocalize"
service_type="fastlio_global_slam/srv/Relocalize"
wait_timeout_s="${RELOCALIZE_SERVICE_WAIT_TIMEOUT_S:-5}"
call_timeout_s="${RELOCALIZE_CALL_TIMEOUT_S:-60}"
map_dir="${FASTLIO_GLOBAL_MAP_DIR:-${project_root}/maps/fastlio_global_3d}"
auto_load_map="${AUTO_LOAD_GLOBAL_MAP:-true}"

if ! timeout "${wait_timeout_s}s" ros2 service type "${service_name}" >/tmp/fastlio_relocalize_service_type.txt 2>/dev/null; then
  echo "重定位服务不可用：${service_name}" >&2
  echo "请先在另一个终端启动后端：" >&2
  echo "  cd ${project_root}" >&2
  echo "  "${project_root}/tools/fastlio/运行全局重定位.sh"" >&2
  echo "" >&2
  echo "然后检查：" >&2
  echo "  ros2 service list | grep fastlio_global_backend" >&2
  echo "  ros2 topic hz /fastlio_global/relocalized_pose" >&2
  exit 2
fi

if [[ "${auto_load_map}" == "true" ]]; then
  if [[ ! -f "${map_dir}/metadata.csv" ]]; then
    echo "全局三维地图元数据缺失：${map_dir}/metadata.csv" >&2
    echo "请设置 FASTLIO_GLOBAL_MAP_DIR，或先保存/加载一份有效的 FAST-LIO 全局地图。" >&2
    exit 3
  fi

  echo "重定位前先加载 FAST-LIO 全局三维地图：${map_dir}"
  timeout "${wait_timeout_s}s" ros2 service call /fastlio_global_backend/load_map fastlio_global_slam/srv/LoadMap \
    "{directory: '${map_dir}'}"
fi

echo "正在调用重定位服务，超时 ${call_timeout_s}s..."
exec timeout "${call_timeout_s}s" ros2 service call "${service_name}" "${service_type}" \
  "{use_latest_scan: true}"
