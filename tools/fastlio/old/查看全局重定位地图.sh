#!/usr/bin/env bash
set -euo pipefail

# 只查看已保存的 GlobalMap.pcd，不启动传感器、重定位或飞行控制。
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
project_root="$(cd -- "${script_dir}/../.." && pwd -P)"
if [[ -n "${GLOBAL_MAP_PCD:-}" ]]; then
  map_file="${GLOBAL_MAP_PCD}"
else
  map_file="$(find "${project_root}/maps" -mindepth 2 -maxdepth 2 -type f \
    -name 'GlobalMap_去顶端_z*.pcd' -printf '%T@ %p\n' 2>/dev/null |
    sort -nr | sed -n '1s/^[^ ]* //p')"
  if [[ -z "${map_file}" ]]; then
    map_file="${project_root}/maps/fastlio_global_3d_20260810/GlobalMap_去顶端_z3m.pcd"
  fi
fi
rviz_config="${project_root}/Lin_shi/fastlio_global_slam/config/fastlio_global_slam.rviz"
log_dir="${project_root}/runtime/查看全局地图_$(date +%Y%m%d_%H%M%S)"

[[ -s "${map_file}" ]] || { echo "[错误] 地图不存在或为空：${map_file}" >&2; exit 2; }
[[ -f "${rviz_config}" ]] || { echo "[错误] RViz 配置不存在：${rviz_config}" >&2; exit 2; }
if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
  echo "[错误] 当前没有图形桌面环境。" >&2
  exit 2
fi

cd "${project_root}"
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"
export COLCON_TRACE="${COLCON_TRACE:-}"
# shellcheck disable=SC1091
source "${project_root}/install/setup.bash"
set -u

mkdir -p "${log_dir}"
map_pid=""
cleanup() {
  trap - EXIT INT TERM HUP
  if [[ -n "${map_pid}" ]]; then
    kill -INT "${map_pid}" 2>/dev/null || true
    wait "${map_pid}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM HUP

echo "[加载] ${map_file}"
ros2 run race_mapping map_io_node --ros-args \
  -p use_sim_time:=false \
  -p map_file:="${map_file}" \
  -p auto_load:=true \
  -p publish_topic:=/fastlio_global/map \
  -p frame_id:=camera_init \
  -p display_leaf_size:=0.05 \
  >"${log_dir}/地图发布.log" 2>&1 &
map_pid=$!

for _ in 1 2 3 4 5 6 7 8 9 10; do
  timeout 2 ros2 topic echo --once /fastlio_global/map >/dev/null 2>&1 && break
  kill -0 "${map_pid}" 2>/dev/null || { echo "[错误] 地图发布节点已退出。" >&2; exit 3; }
  sleep 1
done
if ! timeout 3 ros2 topic echo --once /fastlio_global/map >/dev/null 2>&1; then
  echo "[错误] /fastlio_global/map 没有收到地图，请查看 ${log_dir}/地图发布.log。" >&2
  exit 3
fi

echo "[就绪] 地图已发布到 /fastlio_global/map，Fixed Frame=camera_init。"
echo "[提示] 关闭 RViz 窗口即可结束查看。"
rviz2 -d "${rviz_config}" >"${log_dir}/RViz.log" 2>&1
