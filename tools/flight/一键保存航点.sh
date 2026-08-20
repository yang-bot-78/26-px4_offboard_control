#!/usr/bin/env bash
set -euo pipefail

# 启动地图定位和输出关闭的导航栈；按回车执行一次重定位或保存一个航点。
# Ctrl-C 会停止本次启动的全部组件。

show_help() {
  cat <<'EOF'
用法：./tools/flight/一键保存航点.sh [--strosbag]

  --strosbag 录制重定位专项 rosbag，含已配准点云、后端地图与路径。
EOF
}

record_strosbag=false
while (($# > 0)); do
  case "$1" in
    --strosbag) record_strosbag=true ;;
    -h|--help|--帮助) show_help; exit 0 ;;
    *) echo "错误：不支持的参数：$1" >&2; show_help >&2; exit 2 ;;
  esac
  shift
done

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
project_root="$(cd -- "${script_dir}/../.." && pwd -P)"
stack_script="${script_dir}/一键启动导航栈.sh"
default_map_dir="${project_root}/maps/main"
map_dir="${FRLIO_GLOBAL_MAP_DIR:-${project_root}/maps/main}"
map_file="${MAP_FILE:-}"
waypoint_root="${WAYPOINTS_ROOT:-${project_root}/src/race_offboard/config/waypoints}"
active_waypoint_dir="${waypoint_root}/main"
flight_z="${FLIGHT_Z:-0.78}"
# The first four names (B1/B2/C/D) are reused on every collection run.  Allow
# replacing an existing point by default so a fresh handheld or in-flight
# survey is not blocked by the previous run's YAML.  Set FORCE_WAYPOINTS=false
# to retain the strict "new names only" behavior.
force="${FORCE_WAYPOINTS:-true}"
readiness_timeout="${READINESS_TIMEOUT_SEC:-180}"
mid360_frlio_delay="${MID360_FRLIO_DELAY_SEC:-4}"
relocalization_scan_delay="${RELOCALIZATION_FRESH_SCAN_DELAY_SEC:-3}"
waypoint_max_stddev="${WAYPOINT_MAX_STDDEV_M:-999.0}"
session_id="$(date +%Y%m%d_%H%M%S)"
profile_dir="${waypoint_root}/${session_id}"
profile_mode=false
if [[ -n "${WAYPOINTS_FILE:-}" || -n "${MISSION_FILE:-}" ]]; then
  waypoints_file="${WAYPOINTS_FILE:-${active_waypoint_dir}/waypoints.yaml}"
  mission_file="${MISSION_FILE:-${active_waypoint_dir}/mission.yaml}"
else
  waypoints_file="${profile_dir}/waypoints.yaml"
  mission_file="${profile_dir}/mission.yaml"
  profile_mode=true
fi
session_dir="${project_root}/runtime/waypoint_save_${session_id}"
ready_file="${session_dir}/snapshot/stack_ready.state"
stack_log="${session_dir}/stack.log"
stack_pid=""
visualizer_pid=""
cleanup_started=false
profile_activated=false

[[ -x "${stack_script}" ]] || { echo "错误：找不到启动脚本：${stack_script}" >&2; exit 2; }
find_unique_pcd() {
  local directory="$1"
  local -a matches=()
  [[ -d "${directory}" ]] || {
    echo "错误：地图目录不存在：${directory}" >&2
    return 2
  }
  mapfile -d '' -t matches < <(
    find "${directory}" -maxdepth 1 -type f -iname '*.pcd' -print0 | sort -z
  )
  if ((${#matches[@]} != 1)); then
    echo "错误：地图目录中必须有且只有一个 .pcd 文件，当前找到 ${#matches[@]} 个。" >&2
    ((${#matches[@]} == 0)) || printf '  %s\n' "${matches[@]}" >&2
    return 2
  fi
  printf '%s\n' "${matches[0]}"
}

if [[ -z "${map_file}" ]]; then
  map_file="$(find_unique_pcd "${default_map_dir}")" || exit 2
fi
[[ -s "${map_file}" ]] || { echo "错误：地图文件不存在或为空：${map_file}" >&2; exit 2; }
[[ -d "${map_dir}/keyframes" && -s "${map_dir}/metadata.csv" ]] || {
  echo "错误：重定位库不完整：${map_dir}" >&2; exit 2;
}
[[ "${force}" == true || "${force}" == false ]] || {
  echo "错误：FORCE_WAYPOINTS 必须是 true 或 false" >&2; exit 2;
}
[[ "${readiness_timeout}" =~ ^[0-9]+$ ]] && ((readiness_timeout > 0)) || {
  echo "错误：READINESS_TIMEOUT_SEC 必须是正整数" >&2; exit 2;
}
[[ "${mid360_frlio_delay}" =~ ^[0-9]+([.][0-9]+)?$ ]] || {
  echo "错误：MID360_FRLIO_DELAY_SEC 必须是非负数字" >&2; exit 2;
}
[[ "${relocalization_scan_delay}" =~ ^[0-9]+([.][0-9]+)?$ ]] || {
  echo "错误：RELOCALIZATION_FRESH_SCAN_DELAY_SEC 必须是非负数字" >&2; exit 2;
}
[[ "${waypoint_max_stddev}" =~ ^[0-9]+([.][0-9]+)?$ ]] || {
  echo "错误：WAYPOINT_MAX_STDDEV_M 必须是非负数字" >&2; exit 2;
}

set +u
source /opt/ros/humble/setup.bash
source "${project_root}/install/setup.bash"
set -u

stop_stack() {
  local attempt
  [[ "${cleanup_started}" == false ]] || return
  cleanup_started=true
  # The terminal can deliver repeated Ctrl-C events while child processes are
  # being stopped.  Do not re-enter on_interrupt or interrupt this cleanup.
  trap '' INT TERM HUP
  if [[ -n "${visualizer_pid}" ]] && kill -0 "${visualizer_pid}" 2>/dev/null; then
    kill -INT "${visualizer_pid}" 2>/dev/null || true
    wait "${visualizer_pid}" 2>/dev/null || true
  fi
  if [[ -n "${stack_pid}" ]] && kill -0 "${stack_pid}" 2>/dev/null; then
    echo "正在逆序停止本次保存航点启动的全部程序。"
    kill -INT "${stack_pid}" 2>/dev/null || true
    for attempt in {1..30}; do
      kill -0 "${stack_pid}" 2>/dev/null || break
      sleep 1
    done
    kill -0 "${stack_pid}" 2>/dev/null && kill -TERM "${stack_pid}" 2>/dev/null || true
    wait "${stack_pid}" 2>/dev/null || true
  fi
}

on_interrupt() {
  echo
  echo "已收到退出指令。"
  exit 130
}
trap stop_stack EXIT
trap on_interrupt INT TERM HUP

discard_pending_input() {
  local ignored
  # A return key pressed while the stack is starting must not be reused as a
  # waypoint confirmation after readiness.  Require a fresh key press below.
  while IFS= read -r -t 0 ignored; do :; done
}

activate_profile() {
  [[ "${profile_mode}" == true ]] || return 0
  [[ -s "${waypoints_file}" && -s "${mission_file}" ]] || return 1
  mkdir -p "${waypoint_root}" "${active_waypoint_dir}"
  if find "${active_waypoint_dir}" -mindepth 1 -maxdepth 1 -print -quit 2>/dev/null | grep -q .; then
    local backup_dir="${waypoint_root}/$(date +%Y%m%d_%H%M%S)_previous_main"
    local suffix=0
    while [[ -e "${backup_dir}" ]]; do
      suffix=$((suffix + 1))
      backup_dir="${waypoint_root}/$(date +%Y%m%d_%H%M%S)_previous_main_${suffix}"
    done
    mkdir -p "${backup_dir}"
    cp -a "${active_waypoint_dir}/." "${backup_dir}/"
    echo "历史航点已保留：${backup_dir}"
  fi
  cp -f "${waypoints_file}" "${active_waypoint_dir}/waypoints.yaml"
  cp -f "${mission_file}" "${active_waypoint_dir}/mission.yaml"
  profile_activated=true
  echo "当前应用航点已更新：${active_waypoint_dir}"
}

sync_active_profile() {
  [[ "${profile_mode}" == true && "${profile_activated}" == true ]] || return 0
  cp -f "${waypoints_file}" "${active_waypoint_dir}/waypoints.yaml"
  cp -f "${mission_file}" "${active_waypoint_dir}/mission.yaml"
}

mkdir -p "${session_dir}"
if [[ "${profile_mode}" == true ]]; then
  mkdir -p "${profile_dir}"
fi
echo "============================================================"
echo "                 一键保存航点"
echo "============================================================"
echo "正在启动导航预热、MID-360 雷达、FR-LIO、MAVROS/EV、主地图和 RViz。"
echo "雷达：启动 MID-360 后等待 ${mid360_frlio_delay} 秒，再启动 FR-LIO。"
echo "扫图：已开启全局重定位扫描；加载地图后等待 ${relocalization_scan_delay} 秒获取新扫描。"
echo "重定位：按回车执行一次；失败后可继续按回车重试，不设次数上限。"
echo "地图文件：${map_file}"
echo "重定位库：${map_dir}"
if [[ "${profile_mode}" == true ]]; then
  echo "本次采集目录：${profile_dir}"
  echo "完成 B1/B2/C/D 后将更新当前应用：${active_waypoint_dir}"
else
  echo "自定义航点文件：${waypoints_file}"
fi
echo "航点采样：飞行手动控制或地面手持均可，只要求地图坐标有效，不要求飞机悬停稳定。"
if [[ "${record_strosbag}" == true ]]; then
  echo "重定位专项 rosbag：已启用（高频点云录制，请确认磁盘空间）。"
fi
if [[ "${force}" == true ]]; then
  echo "本次采集内同名航点：允许更新；历史日期目录不会被修改。"
else
  echo "同名航点：禁止覆盖（设置 FORCE_WAYPOINTS=true 可更新已有点）。"
fi
env \
  MAP_FILE="${map_file}" FRLIO_GLOBAL_MAP_DIR="${map_dir}" \
  FLIGHT_RUN_DIR="${session_dir}" FLIGHT_TIMESTAMP="${session_id}" \
  LIO_BACKEND=fr_lio RELOCALIZATION_ENABLED=true NAVIGATION_ENABLED=true \
  MISSION_ENABLED=false ENABLE_OUTPUT=false MANUAL_HANDOVER=true \
  REQUIRE_FLIGHT_READY_FOR_STARTUP=false RECORD_BAG="${record_strosbag}" \
  ROSBAG_PROFILE="$( [[ "${record_strosbag}" == true ]] && printf strosbag || printf full )" RVIZ=true \
  COMPONENT_WINDOWS=false PLANNER_BACKEND=super SKIP_PREFLIGHT_CHECK=1 \
  READINESS_TIMEOUT_SEC="${readiness_timeout}" \
  MID360_FRLIO_DELAY_SEC="${mid360_frlio_delay}" \
  RELOCALIZATION_FRESH_SCAN_DELAY_SEC="${relocalization_scan_delay}" \
  RELOCALIZATION_INTERACTIVE=true RELOCALIZATION_TRIGGER_TTY=/dev/tty \
  WAYPOINTS_FILE="${waypoints_file}" MISSION_FILE="${mission_file}" \
  "${stack_script}" > >(tee -a "${stack_log}") 2>&1 &
stack_pid=$!

deadline=$((SECONDS + readiness_timeout + 180))
while [[ ! -f "${ready_file}" ]]; do
  if ! kill -0 "${stack_pid}" 2>/dev/null; then
    echo "错误：定位启动失败，请查看日志：${stack_log}" >&2
    tail -n 60 "${stack_log}" >&2 || true
    exit 1
  fi
  if ((SECONDS >= deadline)); then
    echo "错误：等待重定位准备完成超时，请查看日志：${stack_log}" >&2
    exit 1
  fi
  sleep 1
done

ros2 run race_offboard waypoint_visualizer.py \
  --waypoints-file "${waypoints_file}" --topic /race/saved_waypoints \
  >"${session_dir}/waypoint_visualizer.log" 2>&1 &
visualizer_pid=$!

echo "定位和地图已准备完成。现在可连续保存航点，按 Ctrl-C 结束并清理全部组件。"
echo "已保存的航点会自动显示在 RViz 中。"
echo "前四个点为 B1、B2、C、D；之后为 P005、P006……，全部点都会写入状态机路线。"
echo "保存时不要求飞机稳定；请在需要记录的位置按回车即可。"
index=1
while true; do
  case "${index}" in
    1) name=B1 ;; 2) name=B2 ;; 3) name=C ;; 4) name=D ;;
    *) name="P$(printf '%03d' "${index}")" ;;
  esac
  discard_pending_input
  read -r -p "[${name}] 确认当前位置后按回车保存（Ctrl-C 结束）: " _
  command=(ros2 run race_offboard save_waypoint.py
    --name "${name}" --mode xy --flight-z "${flight_z}"
    --max-stddev "${waypoint_max_stddev}"
    --waypoints-file "${waypoints_file}" --map-file "${map_file}"
    --odom-topic /race/odom)
  [[ "${force}" == true ]] && command+=(--force)
  if output=$("${command[@]}" 2>&1); then
    echo "航点保存成功：${name}"
    echo "${output}" | sed -n 's/.*x=\([-+0-9.eE]*\) y=\([-+0-9.eE]*\).*/坐标：x=\1，y=\2/p'
    index=$((index + 1))
    if [[ "${index}" -ge 5 ]]; then
      if ros2 run race_offboard export_mission_waypoints.py \
        --waypoints-file "${waypoints_file}" --mission-file "${mission_file}" \
        --names B1 B2 C D --all-waypoints >/dev/null 2>&1; then
        echo "状态机航点文件已更新（全部已保存航点）：${mission_file}"
        if [[ "${profile_mode}" == true ]]; then
          if [[ "${profile_activated}" == false ]]; then
            activate_profile || echo "警告：当前应用航点更新失败，日期采集文件仍已保存：${profile_dir}" >&2
          else
            sync_active_profile
          fi
        fi
      else
        echo "状态机航点文件生成失败，请检查 B1、B2、C、D。" >&2
      fi
    fi
  else
    echo "航点保存失败：${name}" >&2
    echo "${output}" | sed 's/^/原因：/' >&2
    echo "请确认定位有效、坐标在地图坐标系下，再重新按回车。" >&2
  fi
done
