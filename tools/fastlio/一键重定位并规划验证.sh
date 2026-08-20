#!/usr/bin/env bash
set -euo pipefail

# 无桨、上锁状态下的一键 Shadow 验证：
# MID-360 -> FR-LIO -> 全局重定位 -> map 对齐里程计 -> 静态地图 -> 规划器 -> RViz
# 本脚本绝不启动 MAVROS、EV、Offboard、任务执行器或飞行控制输出。

show_help() {
  cat <<'EOF'
用法：
  ./tools/fastlio/一键重定位并规划验证.sh

默认地图：
  规划点云：maps/fastlio_global_3d_20260810/GlobalMap_去顶端_z3m.pcd
  重定位库：maps/fastlio_global_3d_20260810

可选环境变量：
  PLANNING_MAP_FILE=/绝对路径/地图.pcd
  FASTLIO_GLOBAL_MAP_DIR=/绝对路径/关键帧目录
  MID360_FASTLIO_DELAY_SEC=4
  MAP_LOAD_FRESH_SCAN_DELAY_SEC=3
  CHAIN_WAIT_TIMEOUT=90
  RELOCALIZE_CALL_TIMEOUT_S=120
  RELOCALIZATION_MIN_KEYFRAMES=30

注意：本次测试要求重定位库包含 metadata.csv、GlobalMap.pcd 和至少 3 个 keyframes/*.pcd，
并且必须与规划点云来自同一场地和同一坐标系。旧的 fastlio_global_3d 已知与
2026-08-10 的规划点云不匹配，因此不会作为默认值。
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
  echo "[错误] 必须在交互终端运行本脚本。" >&2
  exit 2
fi

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
project_root="$(cd -- "${script_dir}/../.." && pwd -P)"
livox_env="${LIVOX_MID360_ENV:-${HOME}/livox_mid360_env}"
frlio_config="${FRLIO_CONFIG:-${project_root}/src/fr_lio/config/indoors.yaml}"
planning_map_file="${PLANNING_MAP_FILE:-${project_root}/maps/fastlio_global_3d_20260810/GlobalMap_去顶端_z3m.pcd}"
global_map_dir="${FASTLIO_GLOBAL_MAP_DIR:-${project_root}/maps/fastlio_global_3d_20260810}"
rviz_config="${project_root}/src/race_bringup/rviz/race_click_planner.rviz"
planner_config="${project_root}/src/race_super_planner_ros2/config/navigation.yaml"
frame_bridge="${script_dir}/重定位坐标桥.py"
backend_config="${script_dir}/只重定位一次后端参数.yaml"
cleanup_script="${project_root}/tools/flight/清理运行环境.sh"
wait_timeout="${CHAIN_WAIT_TIMEOUT:-90}"
driver_delay="${MID360_FASTLIO_DELAY_SEC:-4}"
fresh_scan_delay="${MAP_LOAD_FRESH_SCAN_DELAY_SEC:-3}"
relocalize_timeout="${RELOCALIZE_CALL_TIMEOUT_S:-120}"
min_keyframes="${RELOCALIZATION_MIN_KEYFRAMES:-30}"
timestamp="$(date +%Y%m%d_%H%M%S)"
log_dir="${project_root}/runtime/重定位规划验证_${timestamp}"

is_nonnegative_number() {
  [[ "$1" =~ ^([0-9]+([.][0-9]*)?|[.][0-9]+)$ ]]
}
if [[ ! "${wait_timeout}" =~ ^[0-9]+$ ]] || ((wait_timeout < 10)); then
  echo "[错误] CHAIN_WAIT_TIMEOUT 必须是大于或等于 10 的整数。" >&2
  exit 2
fi
if [[ ! "${relocalize_timeout}" =~ ^[0-9]+$ ]] || ((relocalize_timeout < 10)); then
  echo "[错误] RELOCALIZE_CALL_TIMEOUT_S 必须是大于或等于 10 的整数。" >&2
  exit 2
fi
if [[ ! "${min_keyframes}" =~ ^[0-9]+$ ]] || ((min_keyframes < 30)); then
  echo "[错误] RELOCALIZATION_MIN_KEYFRAMES 必须是不小于 30 的整数。" >&2
  exit 2
fi
for value in "${driver_delay}" "${fresh_scan_delay}"; do
  if ! is_nonnegative_number "${value}"; then
    echo "[错误] 启动间隔必须是非负数：${value}" >&2
    exit 2
  fi
done

required_files=(
  "${livox_env}/run_mid360_driver.sh"
  "${livox_env}/setup_mid360.bash"
  "${frlio_config}"
  "${project_root}/install/setup.bash"
  "${planning_map_file}"
  "${global_map_dir}/metadata.csv"
  "${global_map_dir}/GlobalMap.pcd"
  "${rviz_config}"
  "${planner_config}"
  "${frame_bridge}"
  "${backend_config}"
  "${cleanup_script}"
)
for file in "${required_files[@]}"; do
  if [[ ! -f "${file}" ]]; then
    echo "[错误] 缺少必需文件：${file}" >&2
    if [[ "${file}" == "${global_map_dir}/metadata.csv" ]]; then
      echo "[错误] 请先生成与本次规划点云匹配的 2026-08-10 全局关键帧地图。" >&2
      echo "[错误] 不会自动退回已知不匹配的 maps/fastlio_global_3d。" >&2
    fi
    exit 2
  fi
done
if [[ "${planning_map_file}" != /* || "${global_map_dir}" != /* ]]; then
  echo "[错误] 两个地图路径都必须是绝对路径。" >&2
  exit 2
fi
if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
  echo "[错误] 当前没有图形桌面，无法启动 RViz。" >&2
  exit 2
fi
keyframe_count="$(find "${global_map_dir}/keyframes" -maxdepth 1 -type f -name 'keyframe_*.pcd' 2>/dev/null | wc -l)"
if ((keyframe_count < min_keyframes)); then
  echo "[错误] 重定位库只有 ${keyframe_count} 个关键帧，本次测试要求至少 ${min_keyframes} 个。" >&2
  exit 2
fi

cat <<EOF
============================================================
             一键重定位 + 只规划 Shadow 验证
============================================================
将启动：MID-360、FR-LIO、重定位后端、坐标桥、静态地图、规划器、RViz。
不会启动：MAVROS、EV、Offboard、任务执行器、解锁、起飞或控制输出。

规划点云：${planning_map_file}
重定位库：${global_map_dir}（${keyframe_count} 个关键帧，最低 ${min_keyframes}）
启动间隔：雷达就绪后 ${driver_delay}s 启 FAST-LIO；地图加载后 ${fresh_scan_delay}s 重定位。
日志目录：${log_dir}

开始前必须满足：已拆下全部螺旋桨、飞机已上锁、没有运行其他飞行栈。
============================================================
EOF
read -r -p "确认以上条件后按回车继续（Ctrl+C 取消）：" _confirmation

cd "${project_root}"
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "${livox_env}/setup_mid360.bash"
set +u
export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"
export COLCON_TRACE="${COLCON_TRACE:-}"
# shellcheck disable=SC1091
source "${project_root}/install/setup.bash"
set -u
export PYTHONNOUSERSITE=1

mkdir -p "${log_dir}"
echo "[准备] 清理遗留的飞行、传感器、MAVROS 和 RViz 进程。"
"${cleanup_script}" offboard ev rviz sensor mavros

echo "[准备] 重启 ROS 2 discovery daemon，清除陈旧节点缓存。"
ros2 daemon stop >/dev/null 2>&1 || true
ros2 daemon start >/dev/null

# 清理脚本不管理重定位后端和规划器；这里只清理本流程的同名残留节点。
mapfile -t stale_pids < <(pgrep -f 'fastlio_global_backend|planning_relocalization_frame_bridge|super_planner_ros2_node|map_io_node' || true)
if ((${#stale_pids[@]} > 0)); then
  echo "[准备] 停止 ${#stale_pids[@]} 个本流程遗留进程。"
  kill -INT "${stale_pids[@]}" 2>/dev/null || true
  sleep 3
fi

declare -a component_names=()
declare -a component_pids=()
declare -a component_pgids=()
cleanup_started=false

start_component() {
  local name="$1" log_file="$2"
  shift 2
  echo "[启动] ${name}"
  setsid "$@" >"${log_file}" 2>&1 &
  local pid=$!
  component_names+=("${name}")
  component_pids+=("${pid}")
  component_pgids+=("${pid}")
  echo "[启动] ${name} PID=${pid}，日志=${log_file}"
}

component_is_running() {
  local index name="$1"
  for ((index=0; index<${#component_names[@]}; index++)); do
    if [[ "${component_names[index]}" == "${name}" ]]; then
      kill -0 "${component_pids[index]}" 2>/dev/null || \
        kill -0 -- "-${component_pgids[index]}" 2>/dev/null
      return
    fi
  done
  return 1
}

stop_all_components() {
  local index pgid pid any_running
  [[ "${cleanup_started}" == false ]] || return
  cleanup_started=true
  trap - EXIT INT TERM HUP
  set +e
  echo "[停止] 正在按逆序停止全部验证组件。"
  for ((index=${#component_pgids[@]} - 1; index >= 0; index--)); do
    kill -INT -- "-${component_pgids[index]}" 2>/dev/null || true
  done
  for _ in 1 2 3 4 5 6 7 8; do
    any_running=false
    for pgid in "${component_pgids[@]}"; do
      kill -0 -- "-${pgid}" 2>/dev/null && any_running=true
    done
    [[ "${any_running}" == false ]] && break
    sleep 1
  done
  for pgid in "${component_pgids[@]}"; do
    kill -0 -- "-${pgid}" 2>/dev/null && kill -TERM -- "-${pgid}" 2>/dev/null
  done
  sleep 2
  for pgid in "${component_pgids[@]}"; do
    kill -0 -- "-${pgid}" 2>/dev/null && kill -KILL -- "-${pgid}" 2>/dev/null
  done
  for pid in "${component_pids[@]}"; do wait "${pid}" 2>/dev/null || true; done
}

handle_signal() {
  echo
  echo "[中断] 用户停止测试。"
  exit 130
}
trap stop_all_components EXIT
trap handle_signal INT TERM HUP

wait_for_topic() {
  local topic="$1" component="$2" start_time="${SECONDS}"
  echo "[等待] ${topic} 出现真实消息。"
  while ((SECONDS - start_time < wait_timeout)); do
    if ! component_is_running "${component}"; then
      echo "[错误] ${component} 已提前退出，请查看日志。" >&2
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
  local service="$1" component="$2" start_time="${SECONDS}"
  echo "[等待] 服务 ${service}。"
  while ((SECONDS - start_time < wait_timeout)); do
    if ! component_is_running "${component}"; then
      echo "[错误] ${component} 已提前退出，请查看日志。" >&2
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

start_component "MID-360 雷达驱动" "${log_dir}/雷达驱动.log" \
  "${livox_env}/run_mid360_driver.sh"
wait_for_topic /livox/imu "MID-360 雷达驱动"
wait_for_topic /livox/lidar "MID-360 雷达驱动"
echo "[等待] 雷达已就绪，${driver_delay} 秒后启动 FAST-LIO。"
sleep "${driver_delay}"

start_component "重定位专用 FR-LIO" "${log_dir}/FR-LIO.log" \
  ros2 launch fr_lio lio.launch.py "config_file:=${frlio_config}" \
  rviz:=false lidar_accumulator:=false
wait_for_topic /Odometry "重定位专用 FR-LIO"
wait_for_topic /cloud_registered "重定位专用 FR-LIO"

start_component "重定位坐标桥" "${log_dir}/坐标桥.log" \
  python3 "${frame_bridge}"
sleep 1
component_is_running "重定位坐标桥" || { echo "[错误] 坐标桥启动失败。" >&2; exit 3; }

start_component "FAST-LIO 全局重定位后端" "${log_dir}/重定位后端.log" \
  ros2 launch fastlio_global_slam fastlio_global_slam.launch.py \
  start_fastlio:=false rviz:=false \
  backend_config:="${backend_config}"
wait_for_service /fastlio_global_backend/load_map "FAST-LIO 全局重定位后端"
wait_for_service /fastlio_global_backend/relocalize "FAST-LIO 全局重定位后端"

backend_count="$(ros2 node list 2>/dev/null | awk '$0 == "/fastlio_global_backend" {count++} END {print count+0}')"
if [[ "${backend_count}" != 1 ]]; then
  echo "[错误] 检测到 ${backend_count} 个 /fastlio_global_backend，要求恰好 1 个。" >&2
  exit 3
fi

echo "[加载] 全局关键帧地图，只加载这一次。"
load_output="$(timeout "${wait_timeout}s" ros2 service call \
  /fastlio_global_backend/load_map fastlio_global_slam/srv/LoadMap \
  "{directory: '${global_map_dir}'}" 2>&1)" || {
  echo "${load_output}" >&2
  echo "[错误] 加载重定位地图失败。" >&2
  exit 4
}
printf '%s\n' "${load_output}"
if ! grep -Eq 'success[=:][[:space:]]*(true|True)' <<<"${load_output}"; then
  echo "[错误] 地图服务没有返回成功。" >&2
  exit 4
fi

echo "[等待] 地图加载会清空旧扫描；等待 ${fresh_scan_delay} 秒取得新的同步扫描。"
sleep "${fresh_scan_delay}"
wait_for_topic /cloud_registered "重定位专用 FR-LIO"

echo "[重定位] 每次使用新扫描，连续确认 3 次；最长等待 ${relocalize_timeout} 秒。"
relocalize_deadline=$((SECONDS + relocalize_timeout))
relocalized=false
while ((SECONDS < relocalize_deadline)); do
  relocalize_output="$(timeout 20 ros2 service call \
    /fastlio_global_backend/relocalize fastlio_global_slam/srv/Relocalize \
    '{use_latest_scan: true}' 2>&1 || true)"
  printf '%s\n' "${relocalize_output}"
  if grep -Eq 'success[=:][[:space:]]*(true|True)' <<<"${relocalize_output}"; then
    relocalized=true
    break
  fi
  if ! grep -q 'awaiting confirmation' <<<"${relocalize_output}"; then
    echo "[错误] 重定位候选未通过严格门限，规划器不会启动。" >&2
    exit 5
  fi
  sleep 1
done
if [[ "${relocalized}" != true ]]; then
  echo "[错误] 重定位连续确认超时，规划器不会启动。" >&2
  exit 5
fi

wait_for_topic /planning/odom "重定位坐标桥"
for transform in 'map odom' 'odom body' 'map body'; do
  read -r parent child <<<"${transform}"
  echo "[检查] TF ${parent} -> ${child}"
  tf_output="$(timeout 5 ros2 run tf2_ros tf2_echo "${parent}" "${child}" 2>&1 || true)"
  if ! grep -q 'Translation:' <<<"${tf_output}"; then
    echo "[错误] TF ${parent} -> ${child} 不存在，规划器不会启动。" >&2
    exit 6
  fi
done

start_component "静态规划地图" "${log_dir}/静态地图.log" \
  ros2 run race_mapping map_io_node --ros-args \
  -p use_sim_time:=false \
  -p map_file:="${planning_map_file}" \
  -p auto_load:=true \
  -p publish_topic:=/saved_map \
  -p frame_id:=map \
  -p display_leaf_size:=0.10
wait_for_topic /saved_map "静态规划地图"

start_component "只规划 Super Planner" "${log_dir}/规划器.log" \
  ros2 launch race_super_planner_ros2 super_planner_ros2.launch.py \
  use_sim_time:=false \
  enable_output:=false \
  global_only_mode:=true \
  publish_global_path:=true \
  publish_ego_local_goal:=false \
  odom_topic:=/planning/odom \
  fallback_odom_topic:=/planning/odom \
  cloud_topic:=/saved_map \
  fallback_cloud_topic:=/saved_map \
  require_mavros_connected:=false
sleep 3
component_is_running "只规划 Super Planner" || { echo "[错误] 规划器启动失败。" >&2; exit 7; }

start_component "规划验证 RViz" "${log_dir}/RViz.log" \
  rviz2 -d "${rviz_config}" --ros-args -r /race/odom:=/planning/odom
sleep 3
component_is_running "规划验证 RViz" || { echo "[错误] RViz 启动失败。" >&2; exit 7; }

cat <<EOF

============================================================
全部组件已就绪，当前是只规划 Shadow 模式。

  控制输出：关闭（enable_output=false）
  规划起点：/planning/odom
  规划地图：/saved_map
  RViz 目标：工具栏「2D Goal Pose」，只点击一个目标，发布到 /goal_pose
  查看路径：/race/global_path 和 /race/super_planner/raw_path

在 RViz 中点击一个空旷目标点，等待一条路径生成，确认路径起点贴近机体、路径不穿障碍。
测试期间不要启动 MAVROS，不要解锁，不要切换 Offboard。
收到一个目标和一条路径后，在本终端输入：结束测试
============================================================
EOF
wait_for_topic /goal_pose "规划验证 RViz"
wait_for_topic /race/global_path "只规划 Super Planner"
echo "[通过] 已收到一个目标，并生成 /race/global_path。"
read -r -p "确认只规划结果后输入 [结束测试]：" operation
if [[ "${operation}" != "结束测试" ]]; then
  echo "[提示] 未输入结束测试，仍将按 Ctrl+C 方式停止。"
fi
echo "[完成] 正在关闭全部组件。"
