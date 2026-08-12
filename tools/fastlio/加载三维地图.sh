#!/usr/bin/env bash
set -eo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
project_root="$(cd -- "${script_dir}/../.." && pwd -P)"

cd ${project_root}
source /opt/ros/humble/setup.bash
source ${project_root}/install/setup.bash
set -u

map_dir="${FASTLIO_GLOBAL_MAP_DIR:-${project_root}/maps/fastlio_global_3d}"
service_name="/fastlio_global_backend/load_map"
wait_timeout_s="${LOAD_MAP_SERVICE_WAIT_TIMEOUT_S:-5}"

if [[ ! -f "${map_dir}/metadata.csv" ]]; then
  echo "全局三维地图元数据缺失：${map_dir}/metadata.csv" >&2
  echo "请设置 FASTLIO_GLOBAL_MAP_DIR，或先保存一份有效的 FAST-LIO 全局地图。" >&2
  exit 3
fi

if ! timeout "${wait_timeout_s}s" ros2 service type "${service_name}" >/tmp/fastlio_load_map_service_type.txt 2>/dev/null; then
  echo "加载地图服务不可用：${service_name}" >&2
  echo "请先启动后端：" >&2
  echo "  cd ${project_root}" >&2
  echo "  "${project_root}/tools/fastlio/运行全局重定位.sh"" >&2
  exit 2
fi

exec ros2 service call "${service_name}" fastlio_global_slam/srv/LoadMap \
  "{directory: '${map_dir}'}"
