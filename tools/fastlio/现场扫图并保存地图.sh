#!/usr/bin/env bash
set -euo pipefail

# 现场扫图专用入口：
#   MID-360 -> FAST-LIO（临时开启 /Laser_map）-> map_io_node -> PCD
#
# 本脚本不会启动 MAVROS、EV、Offboard、导航或解锁服务。

show_help() {
  cat <<'EOF'
用法：
  ./tools/fastlio/现场扫图并保存地图.sh

可选环境变量：
  SCAN_MAP_ROOT=/绝对路径  修改地图保存根目录
  SCAN_WAIT_TIMEOUT=90     修改各数据链路的最长等待时间
  MID360_FASTLIO_DELAY_SEC=4
                           雷达真实数据就绪后，再等几秒启动 FAST-LIO

默认输出：
  maps/现场扫图/YYYYMMDD/扫图_YYYYMMDD_HHMMSS/
    现场地图.pcd
    运行信息.txt
    日志/雷达驱动.log
    日志/FAST-LIO.log
    日志/地图保存节点.log
    日志/RViz.log

完成走场后，在本终端输入「保存地图」。不要用 Ctrl+C 代替保存。
EOF
}

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" || "${1:-}" == "--帮助" ]]; then
  show_help
  exit 0
fi
if (($# > 0)); then
  echo "[错误] 不支持的位置参数：$*" >&2
  show_help >&2
  exit 2
fi
if [[ ! -t 0 ]]; then
  echo "[错误] 本脚本需要在交互终端运行，以便确认安全状态并输入保存指令。" >&2
  exit 2
fi

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
project_root="$(cd -- "${script_dir}/../.." && pwd -P)"
livox_env="${LIVOX_MID360_ENV:-${HOME}/livox_mid360_env}"
fastlio_config="${livox_env}/ws_fastlio/src/fast_lio/config/mid360.yaml"
fastlio_rviz_config="${livox_env}/ws_fastlio/src/fast_lio/rviz/fastlio.rviz"
cleanup_script="${project_root}/tools/flight/清理运行环境.sh"

timestamp="$(date +%Y%m%d_%H%M%S)"
date_part="${timestamp%%_*}"
default_scan_root="${project_root}/maps/现场扫图"
map_root="${SCAN_MAP_ROOT:-${default_scan_root}}"
run_dir="${map_root}/${date_part}/扫图_${timestamp}"
log_dir="${run_dir}/日志"
map_file="${run_dir}/现场地图.pcd"
latest_link="${map_root}/最近一次"
wait_timeout="${SCAN_WAIT_TIMEOUT:-90}"
mid360_fastlio_delay_sec="${MID360_FASTLIO_DELAY_SEC:-4}"

if [[ "${map_root}" != /* ]]; then
  echo "[错误] 扫图根目录必须是绝对路径：${map_root}" >&2
  exit 2
fi
if [[ ! "${wait_timeout}" =~ ^[0-9]+$ ]] || ((wait_timeout < 10)); then
  echo "[错误] 扫图等待秒数必须是大于或等于 10 的整数。" >&2
  exit 2
fi
if [[ ! "${mid360_fastlio_delay_sec}" =~ ^([0-9]+([.][0-9]*)?|[.][0-9]+)$ ]]; then
  echo "[错误] MID360_FASTLIO_DELAY_SEC 必须是非负数。" >&2
  exit 2
fi

required_files=(
  "${livox_env}/run_mid360_driver.sh"
  "${livox_env}/setup_fastlio.bash"
  "${fastlio_config}"
  "${fastlio_rviz_config}"
  "${cleanup_script}"
  "${project_root}/install/setup.bash"
)
for file in "${required_files[@]}"; do
  if [[ ! -f "${file}" ]]; then
    echo "[错误] 缺少必需文件：${file}" >&2
    exit 2
  fi
done
if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
  echo "[错误] 当前没有图形桌面环境，无法启动 RViz。" >&2
  echo "[错误] 请在机器人桌面终端运行本脚本。" >&2
  exit 2
fi

cat <<EOF
============================================================
                 MID-360 现场扫图专用流程
============================================================
本脚本只启动：雷达、FAST-LIO、地图保存节点、RViz。
本脚本不会启动：MAVROS、EV、Offboard、导航、解锁或起飞。

本次地图目录：${run_dir}
本次地图文件：${map_file}

开始前必须满足：
  1. 已拆下全部螺旋桨；
  2. 飞机已上锁；
  3. 不运行其他雷达、FAST-LIO、MAVROS 或导航节点；
  4. 从计划的起飞位置和机头方向开始扫图。
============================================================
EOF

read -r -p "确认以上安全条件已满足后按回车继续（Ctrl+C 取消）：" _safety_confirmation

cd "${project_root}"
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "${livox_env}/setup_fastlio.bash"
# setup_fastlio.bash 会重新打开 nounset；再次关闭，并显式初始化
# colcon 生成的 setup.bash 会直接读取的 COLCON_TRACE。
set +u
export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"
export COLCON_TRACE="${COLCON_TRACE:-}"
# shellcheck disable=SC1091
source "${project_root}/install/setup.bash"
set -u
export PYTHONNOUSERSITE=1

echo "[准备] 清理上一次遗留的飞行、传感器、录包和 RViz 进程。"
"${cleanup_script}" offboard rosbag ev rviz sensor mavros

mkdir -p "${log_dir}"
cat >"${run_dir}/运行信息.txt" <<EOF
state=starting
timestamp=${timestamp}
map_file=${map_file}
fastlio_config=${fastlio_config}
rviz_config=${fastlio_rviz_config}
mid360_fastlio_delay_sec=${mid360_fastlio_delay_sec}
EOF

declare -a component_names=()
declare -a component_pgids=()
declare -a component_pids=()
cleanup_started=false
save_succeeded=false

start_component() {
  local name="$1"
  local log_file="$2"
  shift 2
  echo "[启动] ${name}"
  setsid "$@" >"${log_file}" 2>&1 &
  local pid=$!
  component_names+=("${name}")
  component_pids+=("${pid}")
  component_pgids+=("${pid}")
  echo "[启动] ${name} PID=${pid}，日志=${log_file}"
}

stop_all_components() {
  local index pgid pid
  [[ "${cleanup_started}" == false ]] || return
  cleanup_started=true
  trap - EXIT INT TERM HUP
  set +e

  echo "[停止] 正在按逆序停止扫图组件。"
  for ((index=${#component_pgids[@]} - 1; index >= 0; index--)); do
    pgid="${component_pgids[index]}"
    kill -INT -- "-${pgid}" 2>/dev/null || true
  done
  for _ in 1 2 3 4 5 6 7 8; do
    local any_running=false
    for pgid in "${component_pgids[@]}"; do
      kill -0 -- "-${pgid}" 2>/dev/null && any_running=true
    done
    [[ "${any_running}" == false ]] && break
    sleep 1
  done
  for pgid in "${component_pgids[@]}"; do
    if kill -0 -- "-${pgid}" 2>/dev/null; then
      kill -TERM -- "-${pgid}" 2>/dev/null || true
    fi
  done
  sleep 2
  for pgid in "${component_pgids[@]}"; do
    if kill -0 -- "-${pgid}" 2>/dev/null; then
      kill -KILL -- "-${pgid}" 2>/dev/null || true
    fi
  done
  for pid in "${component_pids[@]}"; do
    wait "${pid}" 2>/dev/null || true
  done

  if [[ "${save_succeeded}" == true ]]; then
    sed -i 's/^state=.*/state=saved_cleanly/' "${run_dir}/运行信息.txt"
  else
    sed -i 's/^state=.*/state=stopped_without_save/' "${run_dir}/运行信息.txt"
  fi
}

handle_signal() {
  echo
  echo "[中断] 收到停止信号。本次不会自动声称地图已经保存。" >&2
  exit 130
}

trap stop_all_components EXIT
trap handle_signal INT TERM HUP

component_is_running() {
  local index name="$1"
  for ((index=0; index<${#component_names[@]}; index++)); do
    if [[ "${component_names[index]}" == "${name}" ]]; then
      # setsid 建立新会话/进程组存在极短竞态。先检查启动器 PID，避免
      # 在进程组尚未建立时把仍在启动的组件误判为已退出。
      if kill -0 -- "${component_pids[index]}" 2>/dev/null || \
         kill -0 -- "-${component_pgids[index]}" 2>/dev/null; then
        return 0
      fi
      return 1
    fi
  done
  return 1
}

wait_for_topic_message() {
  local topic="$1"
  local component="$2"
  local start_time="${SECONDS}"
  echo "[等待] ${topic} 出现真实消息。"
  while ((SECONDS - start_time < wait_timeout)); do
    if ! component_is_running "${component}"; then
      echo "[错误] ${component} 已提前退出。" >&2
      return 1
    fi
    if timeout 3 ros2 topic echo --once "${topic}" >/dev/null 2>&1; then
      echo "[就绪] ${topic}"
      return 0
    fi
    sleep 1
  done
  echo "[错误] 等待 ${topic} 超过 ${wait_timeout} 秒。" >&2
  return 1
}

wait_for_service() {
  local service="$1"
  local component="$2"
  local start_time="${SECONDS}"
  echo "[等待] 服务 ${service}。"
  while ((SECONDS - start_time < wait_timeout)); do
    if ! component_is_running "${component}"; then
      echo "[错误] ${component} 已提前退出。" >&2
      return 1
    fi
    if timeout 3 ros2 service type "${service}" >/dev/null 2>&1; then
      echo "[就绪] ${service}"
      return 0
    fi
    sleep 1
  done
  echo "[错误] 等待 ${service} 超过 ${wait_timeout} 秒。" >&2
  return 1
}

start_component \
  "MID-360 雷达驱动" \
  "${log_dir}/雷达驱动.log" \
  "${livox_env}/run_mid360_driver.sh"

wait_for_topic_message /livox/imu "MID-360 雷达驱动"
wait_for_topic_message /livox/lidar "MID-360 雷达驱动"

echo "[等待] MID-360 数据已就绪，再等待 ${mid360_fastlio_delay_sec} 秒后启动 FAST-LIO。"
sleep "${mid360_fastlio_delay_sec}"

start_component \
  "扫图专用 FAST-LIO" \
  "${log_dir}/FAST-LIO.log" \
  ros2 run fast_lio fastlio_mapping --ros-args \
    --params-file "${fastlio_config}" \
    -p use_sim_time:=false \
    -p publish.map_en:=true \
    -p pcd_save.pcd_save_en:=false

wait_for_topic_message /Odometry "扫图专用 FAST-LIO"
wait_for_topic_message /Laser_map "扫图专用 FAST-LIO"

start_component \
  "地图保存节点" \
  "${log_dir}/地图保存节点.log" \
  ros2 launch race_mapping race_mapping.launch.py \
    use_sim_time:=false \
    start_fast_lio:=false \
    rviz:=false \
    rviz_config_file:="${fastlio_rviz_config}" \
    map_file:="${map_file}" \
    map_auto_load:=false

wait_for_service /save_map_pcd "地图保存节点"

start_component \
  "扫图专用 RViz" \
  "${log_dir}/RViz.log" \
  rviz2 -d "${fastlio_rviz_config}"
sleep 3
if ! component_is_running "扫图专用 RViz"; then
  echo "[错误] RViz 未成功启动，请查看：${log_dir}/RViz.log" >&2
  exit 3
fi

cat <<EOF

============================================================
所有组件已经就绪，RViz 正在实时显示 /Laser_map。

扫图建议：
  1. 原地静止 20-30 秒，等待 FAST-LIO 收敛；
  2. 缓慢走完场地边界；
  3. 柱子、门框、狭窄通道从两侧都扫到；
  4. 雷达尽量保持实际飞行高度，避免快速摆动；
  5. 最后回到起点和原朝向，再静止约 20 秒；
  6. 在 RViz 中确认没有明显双墙、倾斜或断裂。

走场完成后，在本终端输入：保存地图
============================================================
EOF

while true; do
  read -r -p "请输入操作（保存地图 / 放弃）：" operation
  case "${operation}" in
    保存地图)
      break
      ;;
    放弃)
      echo "[放弃] 不保存地图，正在停止组件。"
      exit 1
      ;;
    *)
      echo "[提示] 请输入完整的 [保存地图] 或 [放弃]。"
      ;;
  esac
done

echo "[保存] 正在把最新完整 /Laser_map 写入：${map_file}"
save_output="$(timeout 60 ros2 service call /save_map_pcd std_srvs/srv/Trigger '{}' 2>&1)" || {
  echo "${save_output}" >&2
  echo "[错误] 地图保存服务调用失败。组件暂未停止，可查看日志后重试脚本。" >&2
  exit 4
}
printf '%s\n' "${save_output}"
if ! grep -Eq 'success:[[:space:]]+true' <<<"${save_output}"; then
  echo "[错误] 地图保存服务没有返回 success: true。" >&2
  exit 4
fi
if [[ ! -s "${map_file}" ]]; then
  echo "[错误] 服务报告成功，但地图文件不存在或为空：${map_file}" >&2
  exit 4
fi

save_succeeded=true
ln -sfn "${run_dir}" "${latest_link}"
map_size="$(du -h "${map_file}" | awk '{print $1}')"
point_count="$(head -n 20 "${map_file}" | awk '$1 == "POINTS" {print $2; exit}' || true)"
{
  printf 'saved_at=%s\n' "$(date --iso-8601=seconds)"
  printf 'map_size=%s\n' "${map_size}"
  printf 'map_points=%s\n' "${point_count:-unknown}"
} >>"${run_dir}/运行信息.txt"

cat <<EOF

============================================================
地图已经干净保存。

地图文件：${map_file}
文件大小：${map_size}
点数：    ${point_count:-未从 PCD 头读取到}
最近一次：${latest_link}

脚本退出后可检查：
  pcl_viewer '${map_file}'

后续导航必须显式使用：
  MAP_FILE='${map_file}'

注意：地图保存成功不等于跨 FAST-LIO 重启对齐已经通过。
进入 ENABLE_OUTPUT=true 前仍需完成地图坐标对齐和 Shadow 验证。
============================================================
EOF

exit 0
