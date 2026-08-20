#!/usr/bin/env bash
set -euo pipefail

# Load an existing relocalization map, perform a confirmed relocalization, and
# save the original map plus live scans captured around the successful point.
# This script never starts MAVROS or publishes flight-control setpoints.

show_help() {
  cat <<'EOF'
用法：
  ./tools/fastlio/重定位成功后增补关键帧.sh

作用：
  启动 MID-360、FR-LIO 和全局后端；加载已有关键帧地图；
  成功重定位后，按操作员移动/转动采集新增关键帧，并保存到新目录。

可选环境变量：
  FASTLIO_GLOBAL_MAP_DIR=/绝对路径/原重定位库
  FASTLIO_AUGMENTED_MAP_DIR=/绝对路径/增补后重定位库
  FRLIO_CONFIG=/绝对路径/indoors.yaml
  FRLIO_GLOBAL_MAP_MIN_NEW_KEYFRAMES=3
  FRLIO_GLOBAL_KEYFRAME_WAIT_TIMEOUT=180
  RELOCALIZE_CALL_TIMEOUT_S=120
  MID360_FASTLIO_DELAY_SEC=4
  GLOBAL_MAP_WAIT_TIMEOUT=90
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" || "${1:-}" == "--帮助" ]]; then
  show_help
  exit 0
fi
if (($# > 0)); then
  echo "[错误] 不支持的位置参数：$*" >&2
  show_help >&2
  exit 2
fi
if [[ ! -t 0 ]]; then
  echo "[错误] 必须在交互终端运行，以便确认安全状态和采集操作。" >&2
  exit 2
fi

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
project_root="$(cd -- "${script_dir}/../.." && pwd -P)"
livox_env="${LIVOX_MID360_ENV:-${HOME}/livox_mid360_env}"
frlio_config="${FRLIO_CONFIG:-${project_root}/src/fr_lio/config/indoors.yaml}"
source_map_dir="${FASTLIO_GLOBAL_MAP_DIR:-${project_root}/maps/fastlio_global_3d}"
timestamp="$(date +%Y%m%d_%H%M%S)"
output_map_dir="${FASTLIO_AUGMENTED_MAP_DIR:-${source_map_dir%/}_augmented_${timestamp}}"
backend_config="${script_dir}/重定位成功后增补关键帧参数.yaml"
wait_timeout="${GLOBAL_MAP_WAIT_TIMEOUT:-90}"
relocalize_timeout="${RELOCALIZE_CALL_TIMEOUT_S:-120}"
keyframe_timeout="${FRLIO_GLOBAL_KEYFRAME_WAIT_TIMEOUT:-180}"
driver_delay="${MID360_FASTLIO_DELAY_SEC:-4}"
min_new_keyframes="${FRLIO_GLOBAL_MAP_MIN_NEW_KEYFRAMES:-3}"
log_dir="${project_root}/runtime/重定位增补关键帧_${timestamp}"

is_nonnegative_number() { [[ "$1" =~ ^([0-9]+([.][0-9]*)?|[.][0-9]+)$ ]]; }
for value in "${wait_timeout}" "${relocalize_timeout}" "${keyframe_timeout}" "${min_new_keyframes}"; do
  [[ "${value}" =~ ^[0-9]+$ ]] || { echo "[错误] 超时和关键帧数量必须是整数：${value}" >&2; exit 2; }
done
((wait_timeout >= 10 && relocalize_timeout >= 10 && keyframe_timeout >= 10)) || {
  echo "[错误] 各超时必须不小于 10 秒。" >&2; exit 2;
}
((min_new_keyframes >= 1)) || { echo "[错误] FRLIO_GLOBAL_MAP_MIN_NEW_KEYFRAMES 至少为 1。" >&2; exit 2; }
is_nonnegative_number "${driver_delay}" || { echo "[错误] MID360_FASTLIO_DELAY_SEC 必须是非负数。" >&2; exit 2; }

for required in \
  "${livox_env}/run_mid360_driver.sh" \
  "${livox_env}/setup_mid360.bash" \
  "${frlio_config}" \
  "${backend_config}" \
  "${project_root}/install/setup.bash" \
  "${source_map_dir}/metadata.csv" \
  "${source_map_dir}/GlobalMap.pcd"; do
  [[ -f "${required}" ]] || { echo "[错误] 缺少文件：${required}" >&2; exit 2; }
done
[[ "${source_map_dir}" == /* && "${output_map_dir}" == /* ]] || {
  echo "[错误] 原地图和输出地图路径都必须是绝对路径。" >&2; exit 2;
}
[[ "${source_map_dir}" != "${output_map_dir}" ]] || {
  echo "[错误] 输出目录不能与原地图目录相同，拒绝覆盖原地图。" >&2; exit 2;
}
[[ ! -e "${output_map_dir}" ]] || {
  echo "[错误] 输出目录已存在，拒绝覆盖：${output_map_dir}" >&2; exit 2;
}

source_keyframes="$(find "${source_map_dir}/keyframes" -maxdepth 1 -type f -name 'keyframe_*.pcd' 2>/dev/null | wc -l)"
((source_keyframes > 0)) || { echo "[错误] 原地图没有 keyframes/*.pcd。" >&2; exit 2; }

cat <<EOF
============================================================
           成功重定位后增补 FAST-LIO 关键帧
============================================================
原重定位库：${source_map_dir}（${source_keyframes} 个关键帧）
输出重定位库：${output_map_dir}
目标新增关键帧：${min_new_keyframes}
日志目录：${log_dir}

本流程只启动 MID-360、FR-LIO 和全局后端，不启动 MAVROS、Offboard、解锁、
起飞或任何控制输出。请在无桨、上锁或手持设备状态下使用；成功重定位后，
按提示缓慢移动/转动设备，让雷达从该点的不同距离和角度采集关键帧。
============================================================
EOF
read -r -p "确认安全条件后按回车继续（Ctrl+C 取消）：" _confirmation

cd "${project_root}"
set +u
source /opt/ros/humble/setup.bash
source "${livox_env}/setup_mid360.bash"
export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"
export COLCON_TRACE="${COLCON_TRACE:-}"
source "${project_root}/install/setup.bash"
set -u
export PYTHONNOUSERSITE=1

mkdir -p "${log_dir}"
declare -a component_names=() component_pids=() component_pgids=()
cleanup_started=false

start_component() {
  local name="$1" logfile="$2"; shift 2
  echo "[启动] ${name}"
  setsid "$@" >"${logfile}" 2>&1 &
  component_names+=("${name}"); component_pids+=("$!"); component_pgids+=("$!")
  echo "[启动] ${name} PID=$!，日志=${logfile}"
}
component_is_running() {
  local i name="$1"
  for ((i=0; i<${#component_names[@]}; i++)); do
    [[ "${component_names[i]}" == "${name}" ]] || continue
    kill -0 "${component_pids[i]}" 2>/dev/null || kill -0 -- "-${component_pgids[i]}" 2>/dev/null
    return
  done
  return 1
}
stop_all() {
  [[ "${cleanup_started}" == false ]] || return
  cleanup_started=true
  trap - EXIT INT TERM HUP
  set +e
  echo "[停止] 正在停止本次采集组件。"
  local i pgid
  for ((i=${#component_pgids[@]} - 1; i>=0; i--)); do
    pgid="${component_pgids[i]}"
    kill -INT -- "-${pgid}" 2>/dev/null || true
  done
  sleep 3
  for pgid in "${component_pgids[@]}"; do
    kill -TERM -- "-${pgid}" 2>/dev/null || true
  done
  sleep 2
  for pgid in "${component_pgids[@]}"; do
    kill -KILL -- "-${pgid}" 2>/dev/null || true
  done
  for i in "${component_pids[@]}"; do wait "${i}" 2>/dev/null || true; done
}
trap stop_all EXIT
trap 'exit 130' INT TERM HUP

wait_topic() {
  local topic="$1" name="$2" start="${SECONDS}"
  while ((SECONDS - start < wait_timeout)); do
    component_is_running "${name}" || { echo "[错误] ${name} 已退出，请查看日志。" >&2; return 1; }
    if timeout 3 ros2 topic echo --once "${topic}" >/dev/null 2>&1; then
      echo "[就绪] ${topic}"; return 0
    fi
    sleep 1
  done
  echo "[错误] 等待 ${topic} 超时。" >&2
  return 1
}
wait_service() {
  local service="$1" name="$2" start="${SECONDS}"
  while ((SECONDS - start < wait_timeout)); do
    component_is_running "${name}" || { echo "[错误] ${name} 已退出，请查看日志。" >&2; return 1; }
    if timeout 3 ros2 service type "${service}" >/dev/null 2>&1; then
      echo "[就绪] ${service}"; return 0
    fi
    sleep 1
  done
  echo "[错误] 等待 ${service} 超时。" >&2
  return 1
}
backend_keyframes() {
  timeout 5 ros2 topic echo --once /fastlio_global/backend_status --field data 2>/dev/null \
    | sed -nE 's/.*keyframes=([0-9]+).*/\1/p' | tail -1
}

start_component "MID-360 雷达驱动" "${log_dir}/雷达驱动.log" "${livox_env}/run_mid360_driver.sh"
wait_topic /livox/imu "MID-360 雷达驱动"
wait_topic /livox/lidar "MID-360 雷达驱动"
sleep "${driver_delay}"
start_component "FR-LIO" "${log_dir}/FR-LIO.log" \
  ros2 launch fr_lio lio.launch.py "config_file:=${frlio_config}" rviz:=false lidar_accumulator:=false
wait_topic /Odometry "FR-LIO"
wait_topic /cloud_registered "FR-LIO"
start_component "全局关键帧后端" "${log_dir}/全局后端.log" \
  ros2 launch fastlio_global_slam fastlio_global_slam.launch.py \
  start_fastlio:=false rviz:=false backend_config:="${backend_config}"
wait_service /fastlio_global_backend/load_map "全局关键帧后端"
wait_service /fastlio_global_backend/relocalize "全局关键帧后端"

echo "[加载] 原关键帧地图。"
load_output="$(timeout "${wait_timeout}s" ros2 service call /fastlio_global_backend/load_map \
  fastlio_global_slam/srv/LoadMap "{directory: '${source_map_dir}'}" 2>&1)" || { echo "${load_output}" >&2; exit 4; }
printf '%s\n' "${load_output}"
grep -Eq 'success[=:][[:space:]]*(true|True)' <<<"${load_output}" || { echo "[错误] 地图加载失败。" >&2; exit 4; }
sleep 3

echo "[重定位] 等待严格确认（最长 ${relocalize_timeout} 秒）。"
deadline=$((SECONDS + relocalize_timeout))
relocalized=false
while ((SECONDS < deadline)); do
  output="$(timeout 20 ros2 service call /fastlio_global_backend/relocalize \
    fastlio_global_slam/srv/Relocalize '{use_latest_scan: true}' 2>&1 || true)"
  printf '%s\n' "${output}"
  if grep -Eq 'success[=:][[:space:]]*(true|True)' <<<"${output}"; then relocalized=true; break; fi
  if ! grep -q 'awaiting confirmation' <<<"${output}"; then
    echo "[错误] 当前扫描未通过重定位门限。" >&2; exit 5
  fi
  sleep 1
done
[[ "${relocalized}" == true ]] || { echo "[错误] 重定位确认超时。" >&2; exit 5; }

echo "[成功] 已重定位。请现在缓慢移动/转动设备，至少经过 ${min_new_keyframes} 个关键帧间隔。"
echo "[提示] 关键帧阈值：平移 0.15 m 或旋转 5 度；采集完成后按回车保存。"
start_count=""
count_deadline=$((SECONDS + keyframe_timeout))
while ((SECONDS < count_deadline)); do
  current_count="$(backend_keyframes || true)"
  if [[ "${current_count}" =~ ^[0-9]+$ ]]; then
    if [[ -z "${start_count}" ]]; then
      start_count="${current_count}"
      echo "[采集] 重定位后的后端关键帧基线：${start_count}"
    fi
    new_count=$((current_count - start_count))
    echo "[采集] 当前总关键帧 ${current_count}，新增 ${new_count}/${min_new_keyframes}"
    if ((new_count >= min_new_keyframes)); then break; fi
  fi
  sleep 2
done
[[ -n "${start_count}" ]] || { echo "[错误] 未读到后端关键帧状态。" >&2; exit 6; }
current_count="$(backend_keyframes || true)"
new_count=$((current_count - start_count))
if ((new_count < min_new_keyframes)); then
  echo "[错误] 在 ${keyframe_timeout} 秒内只新增 ${new_count} 个关键帧，拒绝保存不完整增补库。" >&2
  exit 6
fi
read -r -p "确认采集完成并保存到新目录，按回车继续（Ctrl+C 取消）：" _save_confirmation

echo "[保存] 写入增补重定位库：${output_map_dir}"
save_output="$(timeout 120 ros2 service call /fastlio_global_backend/save_map \
  fastlio_global_slam/srv/SaveMap "{directory: '${output_map_dir}', resolution: 0.15}" 2>&1)" || {
  echo "${save_output}" >&2; echo "[错误] 保存失败。" >&2; exit 7;
}
printf '%s\n' "${save_output}"
grep -Eq 'success[=:][[:space:]]*(true|True)' <<<"${save_output}" || { echo "[错误] 后端未确认保存成功。" >&2; exit 7; }
saved_count="$(find "${output_map_dir}/keyframes" -maxdepth 1 -type f -name 'keyframe_*.pcd' 2>/dev/null | wc -l)"
[[ "${saved_count}" =~ ^[0-9]+$ && "${saved_count}" -ge $((source_keyframes + min_new_keyframes)) ]] || {
  echo "[错误] 输出关键帧数量异常：${saved_count}。" >&2; exit 7;
}
echo "[成功] 增补地图已保存：${output_map_dir}（${saved_count} 个关键帧，新增至少 ${min_new_keyframes} 个）"
echo "[说明] 原地图未修改；可将 FASTLIO_GLOBAL_MAP_DIR 指向该新目录进行验证。"
