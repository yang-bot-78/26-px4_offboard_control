#!/usr/bin/env bash
set -euo pipefail

# 生成 FAST-LIO 全局重定位数据库，不是现场 PCD 扫图脚本。
# 只启动雷达、FAST-LIO 和全局后端；不启动 MAVROS、EV、Offboard 或飞行控制。

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" || "${1:-}" == "--帮助" ]]; then
  cat <<'EOF'
用法：
  ./tools/fastlio/现场全局建图并保存重定位库.sh

默认保存目录：
  maps/fastlio_global_3d_20260810

可选环境变量：
  FASTLIO_GLOBAL_MAP_DIR=/绝对路径/目录
  MID360_FASTLIO_DELAY_SEC=4
  GLOBAL_MAP_WAIT_TIMEOUT=90
  FASTLIO_GLOBAL_MAP_RESOLUTION=0.15
EOF
  exit 0
fi
if (($# > 0)); then
  echo "[错误] 不支持的位置参数：$*" >&2
  exit 2
fi
if [[ ! -t 0 ]]; then
  echo "[错误] 必须在交互终端运行，以便确认安全状态和保存操作。" >&2
  exit 2
fi

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
project_root="$(cd -- "${script_dir}/../.." && pwd -P)"
livox_env="${LIVOX_MID360_ENV:-${HOME}/livox_mid360_env}"
fastlio_config="${livox_env}/ws_fastlio/src/fast_lio/config/mid360.yaml"
backend_config="${script_dir}/三关键帧测试后端参数.yaml"
rviz_config="${project_root}/Lin_shi/fastlio_global_slam/config/fastlio_global_slam.rviz"
map_dir="${FASTLIO_GLOBAL_MAP_DIR:-${project_root}/maps/fastlio_global_3d_20260810}"
wait_timeout="${GLOBAL_MAP_WAIT_TIMEOUT:-90}"
driver_delay="${MID360_FASTLIO_DELAY_SEC:-4}"
resolution="${FASTLIO_GLOBAL_MAP_RESOLUTION:-0.15}"
timestamp="$(date +%Y%m%d_%H%M%S)"
log_dir="${project_root}/runtime/全局建图_${timestamp}"

[[ "${map_dir}" == /* ]] || { echo "[错误] FASTLIO_GLOBAL_MAP_DIR 必须是绝对路径。" >&2; exit 2; }
if [[ ! "${wait_timeout}" =~ ^[0-9]+$ ]] || ((wait_timeout < 10)); then
  echo "[错误] GLOBAL_MAP_WAIT_TIMEOUT 必须是不小于 10 的整数。" >&2
  exit 2
fi
[[ "${resolution}" =~ ^([0-9]+([.][0-9]*)?|[.][0-9]+)$ ]] || {
  echo "[错误] FASTLIO_GLOBAL_MAP_RESOLUTION 必须是正数。" >&2; exit 2;
}
for required in \
  "${livox_env}/run_mid360_driver.sh" \
  "${livox_env}/setup_fastlio.bash" \
  "${fastlio_config}" \
  "${backend_config}" \
  "${rviz_config}" \
  "${project_root}/install/setup.bash" \
  "${project_root}/tools/flight/清理运行环境.sh"; do
  [[ -f "${required}" ]] || { echo "[错误] 缺少文件：${required}" >&2; exit 2; }
done
if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
  echo "[错误] 当前没有图形桌面环境，无法启动 RViz。" >&2
  exit 2
fi

mkdir -p "${log_dir}"
cat <<EOF
============================================================
          MID-360 全局建图并保存重定位数据库
============================================================
保存目录：${map_dir}
日志目录：${log_dir}

本流程只启动：雷达、FAST-LIO、全局关键帧后端。
本流程不会启动：MAVROS、EV、Offboard、导航、解锁或起飞。

开始前确认：
  1. 已拆下全部螺旋桨；
  2. 飞机已上锁；
  3. 当前没有其他雷达或 FAST-LIO 实例；
  4. 将从规划地图的起点和机头方向开始走场。

若保存目录已存在，保存时会覆盖其中同名 metadata.csv、GlobalMap.pcd 和关键帧。
============================================================
EOF
read -r -p "确认安全条件后按回车继续（Ctrl+C 取消）：" _confirm

cd "${project_root}"
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "${livox_env}/setup_fastlio.bash"
set +u
export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"
export COLCON_TRACE="${COLCON_TRACE:-}"
# shellcheck disable=SC1091
source "${project_root}/install/setup.bash"
set -u
export PYTHONNOUSERSITE=1

"${project_root}/tools/flight/清理运行环境.sh" offboard ev rviz sensor mavros

declare -a names=() pids=() pgids=()
stopping=false
start_component() {
  local name="$1" logfile="$2"; shift 2
  echo "[启动] ${name}"
  setsid "$@" >"${logfile}" 2>&1 &
  names+=("${name}"); pids+=("$!"); pgids+=("$!")
  echo "[启动] ${name} PID=$!，日志=${logfile}"
}
running() {
  local i name="$1"
  for ((i=0; i<${#names[@]}; i++)); do
    [[ "${names[i]}" == "${name}" ]] || continue
    kill -0 "${pids[i]}" 2>/dev/null || kill -0 -- "-${pgids[i]}" 2>/dev/null
    return
  done
  return 1
}
stop_all() {
  [[ "${stopping}" == false ]] || return
  stopping=true; trap - EXIT INT TERM HUP; set +e
  echo "[停止] 按逆序停止全局建图组件。"
  local i pgid
  for ((i=${#pgids[@]}-1; i>=0; i--)); do kill -INT -- "-${pgids[i]}" 2>/dev/null || true; done
  sleep 5
  for pgid in "${pgids[@]}"; do kill -TERM -- "-${pgid}" 2>/dev/null || true; done
  sleep 2
  for pgid in "${pgids[@]}"; do kill -KILL -- "-${pgid}" 2>/dev/null || true; done
  for i in "${pids[@]}"; do wait "${i}" 2>/dev/null || true; done
}
trap stop_all EXIT
trap 'exit 130' INT TERM HUP

wait_topic() {
  local topic="$1" name="$2" start="${SECONDS}"
  echo "[等待] ${topic} 真实消息。"
  while ((SECONDS-start < wait_timeout)); do
    running "${name}" || { echo "[错误] ${name} 已退出，请查看日志。" >&2; return 1; }
    timeout 3 ros2 topic echo --once "${topic}" >/dev/null 2>&1 && {
      echo "[就绪] ${topic}"; return 0;
    }
    sleep 1
  done
  echo "[错误] 等待 ${topic} 超时。" >&2
  return 1
}
wait_service() {
  local service="$1" name="$2" start="${SECONDS}"
  echo "[等待] ${service} 服务。"
  while ((SECONDS-start < wait_timeout)); do
    running "${name}" || { echo "[错误] ${name} 已退出，请查看日志。" >&2; return 1; }
    timeout 3 ros2 service type "${service}" >/dev/null 2>&1 && {
      echo "[就绪] ${service}"; return 0;
    }
    sleep 1
  done
  echo "[错误] 等待 ${service} 超时。" >&2
  return 1
}

start_component "MID-360 雷达驱动" "${log_dir}/雷达驱动.log" "${livox_env}/run_mid360_driver.sh"
wait_topic /livox/imu "MID-360 雷达驱动"
wait_topic /livox/lidar "MID-360 雷达驱动"
echo "[等待] 雷达数据就绪，${driver_delay} 秒后启动 FAST-LIO。"
sleep "${driver_delay}"

start_component "全局建图 FAST-LIO" "${log_dir}/FAST-LIO.log" \
  ros2 run fast_lio fastlio_mapping --ros-args \
  --params-file "${fastlio_config}" -p use_sim_time:=false \
  -p publish.scan_publish_en:=true -p publish.dense_publish_en:=false \
  -p publish.scan_bodyframe_pub_en:=false
wait_topic /Odometry "全局建图 FAST-LIO"
wait_topic /cloud_registered "全局建图 FAST-LIO"

start_component "全局关键帧后端" "${log_dir}/全局后端.log" \
  ros2 launch fastlio_global_slam fastlio_global_slam.launch.py \
  start_fastlio:=false rviz:=false backend_config:="${backend_config}"
wait_service /fastlio_global_backend/save_map "全局关键帧后端"

start_component "全局建图 RViz" "${log_dir}/RViz.log" rviz2 -d "${rviz_config}"
sleep 3
running "全局建图 RViz" || { echo "[错误] RViz 启动失败，请查看 ${log_dir}/RViz.log。" >&2; exit 3; }

cat <<EOF

============================================================
全局建图已经开始，后端正在接收 /Odometry + /cloud_registered，RViz 正在实时显示。

请从规划地图起点开始，按与现场地图相同的路线缓慢走一遍，最后回到起点静止。
本次测试最低只要求 3 个关键帧，生成阈值为平移 0.15m 或旋转 5 度。
建议从起点缓慢移动 0.3-0.5m并改变朝向，然后回到起点静止。
走场完成后输入 1 尝试保存；输入 0 放弃。
若不足 3 个，脚本不会退出，可继续移动后再次输入 1。
============================================================
EOF
while true; do
  read -r -p "请输入操作（1=保存，0=放弃）：" action
  case "${action}" in
    1) ;;
    0) echo "[放弃] 不保存全局重定位库。"; exit 1 ;;
    *) echo "[提示] 只能输入 1 或 0。"; continue ;;
  esac

  echo "[保存] 正在保存到：${map_dir}"
  save_output="$(timeout 120 ros2 service call /fastlio_global_backend/save_map \
    fastlio_global_slam/srv/SaveMap \
    "{directory: '${map_dir}', resolution: ${resolution}}" 2>&1)" || {
    echo "${save_output}" >&2; echo "[错误] 保存服务调用失败。" >&2; exit 4;
  }
  printf '%s\n' "${save_output}"
  grep -Eq 'success[=:][[:space:]]*(true|True)' <<<"${save_output}" || {
    echo "[错误] 保存服务未返回 success=true。" >&2; exit 4;
  }

  metadata="${map_dir}/metadata.csv"
  global_map="${map_dir}/GlobalMap.pcd"
  keyframe_count="$(find "${map_dir}/keyframes" -maxdepth 1 -type f -name 'keyframe_*.pcd' 2>/dev/null | wc -l)"
  if [[ ! -s "${metadata}" || ! -s "${global_map}" ]]; then
    echo "[错误] metadata.csv 或 GlobalMap.pcd 不存在或为空。" >&2
    exit 5
  fi
  if ((keyframe_count < 3)); then
    echo "[不足] 当前只有 ${keyframe_count} 个关键帧，至少需要 3 个。"
    echo "[继续] 请再移动至少 0.15m 或转动至少 5 度，然后再次输入 1。"
    continue
  fi
  break
done

echo "[成功] 重定位库已保存：${map_dir}（${keyframe_count} 个关键帧）"
if ((keyframe_count < 10)); then
  echo "[警告] 当前只有 ${keyframe_count} 个关键帧，仅用于开局单次重定位和规划线测试，不能作为实飞重定位库。"
fi
echo "[提示] 现在可以停止本脚本，再运行 一键重定位并规划验证.sh。"
