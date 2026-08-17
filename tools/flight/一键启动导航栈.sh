#!/usr/bin/env bash
set -euo pipefail

# 统一的硬件启动顺序：
#   导航预热 -> MID-360 -> LIO -> 可选全局重定位 -> MAVROS/EV -> rosbag。
# 导航先于实时定位链路起来，这样节点发现和规划器初始化就不会让 FAST-LIO 去处理
# 一堆积压的旧雷达数据。数据就绪检查仍然按依赖顺序进行。
#
# Usage:
#   ./tools/flight/一键启动导航栈.sh
# 可选环境变量：MAP_FILE、NO_MAP_VALIDATION、MAP_AUTO_LOAD、
# FCU_URL, WORLD_YAW_ALIGNMENT_RAD, ALLOW_UNVALIDATED_WORLD_YAW,
# PLANNER_BACKEND, NAVIGATION_ENABLED, MISSION_ENABLED, RECOGNITION_ENABLED, RVIZ, ENABLE_OUTPUT,
# MANUAL_HANDOVER, EV_FAULT_AUTO_LAND, RELOCALIZATION_ENABLED, FRLIO_GLOBAL_MAP_DIR,
# RECORD_BAG, READINESS_TIMEOUT_SEC, AUTO_STOP_AFTER_READY_SEC,
# MID360_FASTLIO_DELAY_SEC, LIO_BACKEND, FRLIO_CONFIG,
# COMPONENT_WINDOWS, COMPONENT_WINDOW_GEOMETRY,
# RELOCALIZATION_RETRY_COUNT, RELOCALIZATION_RETRY_DELAY_SEC,
# FLIGHT_CPU_PERFORMANCE, FLIGHT_CPU_AFFINITY, LIVOX_CPU_AFFINITY,
# FRLIO_CPU_AFFINITY, PLANNER_CPU_AFFINITY, ROSBAG_CPU_AFFINITY,
# ROSBAG_NICE_LEVEL,
# SKIP_PREFLIGHT_CHECK.

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
project_root="$(cd -- "${script_dir}/../.." && pwd -P)"
autofix_launch="${project_root}/src/px4_ros_com/launch/fastlio_mavros_autofix.launch.py"
livox_env="${LIVOX_MID360_ENV:-${HOME}/livox_mid360_env}"
livox_setup="${livox_env}/setup_mid360.bash"
lio_backend="${LIO_BACKEND:-fr_lio}"
frlio_config="${FRLIO_CONFIG:-${project_root}/src/fr_lio/config/indoors.yaml}"

default_map_file="${project_root}/maps/main/GlobalMap.pcd"
no_map_validation="${NO_MAP_VALIDATION:-false}"
if [[ "${no_map_validation}" == true ]]; then
  if [[ -n "${MAP_FILE:-}" ]]; then
    printf '[警告] NO_MAP_VALIDATION=true：已忽略 MAP_FILE=%s\n' "${MAP_FILE}" >&2
  fi
  map_file=""
  map_auto_load="${MAP_AUTO_LOAD:-false}"
else
  map_file="${MAP_FILE-${default_map_file}}"
  map_auto_load="${MAP_AUTO_LOAD:-true}"
fi
planner_backend="${PLANNER_BACKEND:-astar_ego}"
effective_planner_backend="${planner_backend}"
tuning_file="${TUNING_FILE:-${project_root}/src/race_bringup/config/astar_ego_tuning.yaml}"
ego_enabled=""
navigation_enabled="${NAVIGATION_ENABLED:-true}"
mission_enabled="${MISSION_ENABLED:-true}"
recognition_enabled="${RECOGNITION_ENABLED:-false}"
rviz="${RVIZ:-true}"
enable_output="${ENABLE_OUTPUT:-true}"
manual_handover="${MANUAL_HANDOVER:-false}"
ev_fault_auto_land="${EV_FAULT_AUTO_LAND:-true}"
record_bag="${RECORD_BAG:-true}"
readiness_timeout_sec="${READINESS_TIMEOUT_SEC:-90}"
auto_stop_after_ready_sec="${AUTO_STOP_AFTER_READY_SEC:-0}"
mid360_fastlio_delay_sec="${MID360_FRLIO_DELAY_SEC:-${MID360_FASTLIO_DELAY_SEC:-4}}"
component_windows="${COMPONENT_WINDOWS:-true}"
component_window_geometry="${COMPONENT_WINDOW_GEOMETRY:-110x30}"
fcu_url="${FCU_URL:-serial:///dev/ttyUSB0:921600?ids=255,190}"
# uXRCE-DDS must use an independent UDP/ethernet endpoint in production. If a
# deployment explicitly selects serial transport, fail before MAVROS starts
# whenever it names the same device as FCU_URL.
uxrce_dds_transport="${UXRCE_DDS_TRANSPORT:-udp}"
uxrce_dds_serial_device="${UXRCE_DDS_SERIAL_DEVICE:-}"
if [[ "${uxrce_dds_transport}" == serial ]]; then
  if [[ -z "${uxrce_dds_serial_device}" ]]; then
    echo "拒绝启动：UXRCE_DDS_TRANSPORT=serial 时必须显式设置 UXRCE_DDS_SERIAL_DEVICE，且不得与 MAVROS 共用。" >&2
    exit 2
  fi
  fcu_serial_device="${fcu_url#serial://}"
  fcu_serial_device="${fcu_serial_device%%:*}"
  if [[ -n "${fcu_serial_device}" && -e "${fcu_serial_device}" && -e "${uxrce_dds_serial_device}" ]] &&
     [[ "$(readlink -f "${fcu_serial_device}")" == "$(readlink -f "${uxrce_dds_serial_device}")" ]]; then
    echo "拒绝启动：MAVROS 与 uXRCE-DDS 配置为共用串口 ${fcu_serial_device}。请改用独立串口或 UDP。" >&2
    exit 2
  fi
fi
world_yaw_alignment_rad="${WORLD_YAW_ALIGNMENT_RAD:-0.0}"
allow_unvalidated_world_yaw="${ALLOW_UNVALIDATED_WORLD_YAW:-false}"
skip_preflight_check="${SKIP_PREFLIGHT_CHECK:-0}"
relocalization_enabled="${RELOCALIZATION_ENABLED:-false}"
global_map_dir="${FRLIO_GLOBAL_MAP_DIR:-${FASTLIO_GLOBAL_MAP_DIR:-${project_root}/maps/main}}"
relocalization_backend_config="${RELOCALIZATION_BACKEND_CONFIG:-${project_root}/tools/fastlio/只重定位一次后端参数.yaml}"
relocalization_bridge="${project_root}/tools/fastlio/重定位坐标桥.py"
relocalization_fresh_scan_delay_sec="${RELOCALIZATION_FRESH_SCAN_DELAY_SEC:-3}"
relocalization_call_timeout_sec="${RELOCALIZATION_CALL_TIMEOUT_SEC:-120}"
relocalization_retry_count="${RELOCALIZATION_RETRY_COUNT:-8}"
relocalization_retry_delay_sec="${RELOCALIZATION_RETRY_DELAY_SEC:-2}"
flight_cpu_performance="${FLIGHT_CPU_PERFORMANCE:-true}"
flight_cpu_affinity="${FLIGHT_CPU_AFFINITY:-true}"
livox_cpu_affinity="${LIVOX_CPU_AFFINITY:-}"
frlio_cpu_affinity="${FRLIO_CPU_AFFINITY:-}"
planner_cpu_affinity="${PLANNER_CPU_AFFINITY:-}"
rosbag_cpu_affinity="${ROSBAG_CPU_AFFINITY:-}"
rosbag_nice_level="${ROSBAG_NICE_LEVEL:-10}"

# Keep LiDAR processing, planning, and recording apart. On this 16-logical-CPU
# host, 8-15 are separate physical cores; 0-7 remain available for the driver,
# MAVROS, DDS, and desktop. Deployments can override any set explicitly.
if [[ "${flight_cpu_affinity}" == true ]]; then
  online_cpu_count="$(nproc --all 2>/dev/null || echo 1)"
  if [[ "${online_cpu_count}" =~ ^[0-9]+$ ]] && ((online_cpu_count >= 16)); then
    livox_cpu_affinity="${livox_cpu_affinity:-0-1}"
    frlio_cpu_affinity="${frlio_cpu_affinity:-8-11}"
    planner_cpu_affinity="${planner_cpu_affinity:-12-13}"
    rosbag_cpu_affinity="${rosbag_cpu_affinity:-14-15}"
  elif [[ "${online_cpu_count}" =~ ^[0-9]+$ ]] && ((online_cpu_count >= 12)); then
    livox_cpu_affinity="${livox_cpu_affinity:-0-1}"
    frlio_cpu_affinity="${frlio_cpu_affinity:-2-6}"
    planner_cpu_affinity="${planner_cpu_affinity:-7-8}"
    rosbag_cpu_affinity="${rosbag_cpu_affinity:-9-11}"
  elif [[ "${online_cpu_count}" =~ ^[0-9]+$ ]] && ((online_cpu_count >= 8)); then
    livox_cpu_affinity="${livox_cpu_affinity:-0-1}"
    frlio_cpu_affinity="${frlio_cpu_affinity:-2-4}"
    planner_cpu_affinity="${planner_cpu_affinity:-5-6}"
    rosbag_cpu_affinity="${rosbag_cpu_affinity:-7}"
  fi
fi

flight_timestamp="${FLIGHT_TIMESTAMP:-$(date +%Y%m%d_%H%M%S)}"
flight_date="${flight_timestamp%%_*}"
flight_run_dir="${FLIGHT_RUN_DIR:-${project_root}/flight_records/${flight_date}/flight_${flight_timestamp}}"
flight_log_dir="${flight_run_dir}/logs"
stack_ready_file="${flight_run_dir}/snapshot/stack_ready.state"

lever_arm_config="${MID360_LEVER_ARM_CONFIG:-${project_root}/src/px4_ros_com/config/mid360_lever_arm.conf}"
lever_arm_validator="${project_root}/tools/fastlio/校验杆臂配置.sh"
readiness_helper="${project_root}/tools/flight/等待ROS就绪.sh"
env_cleanup="${project_root}/tools/flight/清理运行环境.sh"
component_runner="${project_root}/tools/flight/受管组件窗口.sh"
preflight_check="${project_root}/tools/checks/起飞前自检.sh"
bag_script="${project_root}/tools/rosbag/开始录包.sh"
navigation_script="${project_root}/tools/flight/运行导航控制.sh"
service_name="ws_offboard_rosbag_shutdown.service"
service_src="${project_root}/tools/rosbag/${service_name}"
service_dst="${HOME}/.config/systemd/user/${service_name}"

declare -A component_pids=()
declare -A component_pgids=()
declare -A component_launcher_pids=()
declare -A component_logs=()
declare -a started_components=()
cleanup_required=true
cleanup_started=false
cpu_profile_before=""
cpu_profile_changed=false
owner_start_ticks="$(awk '{print $22}' "/proc/$$/stat")"

log_info() {
  printf '[INFO] %s\n' "$*"
}

log_warn() {
  printf '[WARN] %s\n' "$*" >&2
}

log_error() {
  printf '[ERROR] %s\n' "$*" >&2
}

require_file() {
  local path="$1"
  if [[ ! -f "${path}" ]]; then
    log_error "Missing required file: ${path}"
    exit 1
  fi
}

require_executable() {
  local path="$1"
  require_file "${path}"
  if [[ ! -x "${path}" ]]; then
    log_error "Required file is not executable: ${path}"
    exit 1
  fi
}

prefer_workspace_px4_ros_com() {
  # A terminal can retain a previously sourced workspace which also exports
  # px4_ros_com.  Keep all of its other packages, but make this flight
  # workspace's EV implementation and Python module win package resolution.
  local px4_prefix="${project_root}/install/px4_ros_com"
  local px4_python_site
  export AMENT_PREFIX_PATH="${px4_prefix}${AMENT_PREFIX_PATH:+:${AMENT_PREFIX_PATH}}"
  for px4_python_site in \
    "${px4_prefix}"/lib/python*/site-packages \
    "${px4_prefix}"/local/lib/python*/dist-packages \
    "${px4_prefix}"/local/lib/python*/site-packages; do
    [[ -d "${px4_python_site}" ]] || continue
    export PYTHONPATH="${px4_python_site}${PYTHONPATH:+:${PYTHONPATH}}"
  done
}

require_boolean() {
  local name="$1"
  local value="$2"
  case "${value}" in
    true|false) ;;
    *)
      log_error "${name} must be true or false; got: ${value}"
      exit 1
      ;;
  esac
}

resolve_effective_planner_backend() {
  if ! ego_enabled="$(python3 - "${project_root}" "${tuning_file}" <<'PY'
import sys

project_root, tuning_file = sys.argv[1:]
sys.path.insert(0, f'{project_root}/src/race_bringup/launch')
from astar_ego_tuning import load_tuning

print(str(load_tuning(tuning_file)['ego_planner']['enable']).lower())
PY
)"; then
    log_error "Failed to read the EGO enable switch from TUNING_FILE: ${tuning_file}"
    return 1
  fi

  effective_planner_backend="${planner_backend}"
  if [[ "${ego_enabled}" == false ]]; then
    case "${planner_backend}" in
      astar_ego|ego|ego-shadow) effective_planner_backend=super ;;
    esac
  fi
}

component_running() {
  local key="$1"
  local pgid="${component_pgids[${key}]}"
  kill -0 -- "-${pgid}" 2>/dev/null
}

valid_component_group() {
  local pgid="$1"
  local main_pgid
  [[ "${pgid}" =~ ^[0-9]+$ ]] || return 1
  ((pgid > 1)) || return 1
  main_pgid="$(ps -o pgid= -p $$ | tr -d '[:space:]')"
  [[ "${pgid}" != "${main_pgid}" ]]
}

show_component_failure() {
  local key="$1"
  local log_file="${component_logs[${key}]}"
  log_error "Component '${key}' is not ready. Log: ${log_file}"
  if [[ -f "${log_file}" ]]; then
    log_error "Last 30 log lines from '${key}':"
    tail -n 30 "${log_file}" >&2 || true
  fi
}

terminate_components() {
  local index key pgid launcher_pid
  for ((index=${#started_components[@]} - 1; index >= 0; index--)); do
    key="${started_components[index]}"
    pgid="${component_pgids[${key}]}"
    if valid_component_group "${pgid}"; then
      # 即使进程组的原组长已经退出，也要向整个进程组发信号。
      kill -TERM -- "-${pgid}" 2>/dev/null || true
    fi
  done

  for _ in 1 2 3 4 5; do
    local any_running=false
    for key in "${started_components[@]}"; do
      if component_running "${key}"; then
        any_running=true
        break
      fi
    done
    [[ "${any_running}" == false ]] && break
    sleep 1
  done

  for key in "${started_components[@]}"; do
    pgid="${component_pgids[${key}]}"
    if component_running "${key}" && valid_component_group "${pgid}"; then
      kill -KILL -- "-${pgid}" 2>/dev/null || true
    fi
  done

  # 受管终端里的命令会在其组件进程组消失后退出。先给 GNOME Terminal 客户端一点
  # 时间关闭，然后回收所有启动器进程。
  sleep 1
  for key in "${started_components[@]}"; do
    launcher_pid="${component_launcher_pids[${key}]}"
    if kill -0 "${launcher_pid}" 2>/dev/null; then
      kill -TERM "${launcher_pid}" 2>/dev/null || true
    fi
    wait "${launcher_pid}" 2>/dev/null || true
  done
}

restore_cpu_profile() {
  [[ "${cpu_profile_changed}" == true ]] || return 0
  if powerprofilesctl set "${cpu_profile_before}"; then
    log_info "Restored CPU power profile: ${cpu_profile_before}"
  else
    log_warn "Failed to restore CPU power profile: ${cpu_profile_before}"
  fi
  cpu_profile_changed=false
}

on_exit() {
  local status=$?
  [[ "${cleanup_started}" == false ]] || exit "${status}"
  cleanup_started=true
  trap - EXIT INT TERM HUP
  set +e
  if [[ "${cleanup_required}" == true && ${#started_components[@]} -gt 0 ]]; then
    log_warn "Stopping components started by this run..." || true
    terminate_components
  fi
  restore_cpu_profile
  exit "${status}"
}

on_signal() {
  local signal="$1"
  set +e
  log_warn "Received ${signal}; shutting down the complete stack." || true
  case "${signal}" in
    INT) exit 130 ;;
    *) exit 143 ;;
  esac
}

trap on_exit EXIT
trap 'on_signal INT' INT
trap 'on_signal TERM' TERM
trap 'on_signal HUP' HUP

start_component() {
  local title="$1"
  local key="$2"
  shift 2
  local log_file="${flight_log_dir}/${key}.log"
  local runtime_file="${flight_run_dir}/snapshot/component_runtime/${key}.state"
  local launcher_pid pid pgid cpu_affinity
  local -a component_env=()

  case "${key}" in
    mid360_driver) cpu_affinity="${livox_cpu_affinity}" ;;
    fastlio) cpu_affinity="${frlio_cpu_affinity}" ;;
    navigation) cpu_affinity="${planner_cpu_affinity}" ;;
    rosbag_debug) cpu_affinity="${rosbag_cpu_affinity}" ;;
    *) cpu_affinity="" ;;
  esac
  if [[ -n "${cpu_affinity}" ]]; then
    component_env=(env "COMPONENT_CPU_AFFINITY=${cpu_affinity}")
  fi

  log_info "Starting ${title} in its own terminal window; log=${log_file}"
  mkdir -p "$(dirname -- "${runtime_file}")"
  : >"${runtime_file}"
  if [[ "${component_windows}" == true ]]; then
    "${component_env[@]}" gnome-terminal --window --wait \
      --title="${title} | ${flight_timestamp}" \
      --geometry="${component_window_geometry}" \
      --working-directory="${project_root}" \
      -- "${component_runner}" "$$" "${owner_start_ticks}" \
      "${runtime_file}" "${log_file}" "${title}" -- "$@" &
  else
    "${component_env[@]}" "${component_runner}" "$$" "${owner_start_ticks}" \
      "${runtime_file}" "${log_file}" "${title}" -- "$@" >/dev/null 2>&1 &
  fi
  launcher_pid=$!

  for _ in $(seq 1 100); do
    [[ -s "${runtime_file}" ]] && break
    if ! kill -0 "${launcher_pid}" 2>/dev/null; then
      log_error "Component window '${key}' exited before publishing its runtime state"
      wait "${launcher_pid}" 2>/dev/null || true
      return 1
    fi
    sleep 0.1
  done
  if [[ ! -s "${runtime_file}" ]]; then
    log_error "Timed out waiting for component runtime state: ${runtime_file}"
    kill -TERM "${launcher_pid}" 2>/dev/null || true
    wait "${launcher_pid}" 2>/dev/null || true
    return 1
  fi

  pid="$(awk -F= '$1 == "pid" {print $2}' "${runtime_file}")"
  pgid="$(awk -F= '$1 == "pgid" {print $2}' "${runtime_file}")"
  if [[ ! "${pid}" =~ ^[0-9]+$ ]] || ! valid_component_group "${pgid}"; then
    log_error "Invalid component runtime state for ${key}: pid=${pid:-missing}, pgid=${pgid:-missing}"
    kill -TERM "${launcher_pid}" 2>/dev/null || true
    wait "${launcher_pid}" 2>/dev/null || true
    return 1
  fi
  component_pids["${key}"]="${pid}"
  component_pgids["${key}"]="${pgid}"
  component_launcher_pids["${key}"]="${launcher_pid}"
  component_logs["${key}"]="${log_file}"
  if command -v taskset >/dev/null 2>&1; then
    log_info "${title} scheduler affinity: $(taskset --pid "${pid}" 2>/dev/null || true)"
  fi
  started_components+=("${key}")
}

wait_component_ready() {
  local key="$1"
  local mode="$2"
  local topic="$3"
  local pid="${component_pids[${key}]}"
  local readiness_setup_bash=""

  if [[ "${key}" == mid360_driver ]]; then
    readiness_setup_bash="${livox_setup}"
  fi

  if ! component_running "${key}"; then
    show_component_failure "${key}"
    return 1
  fi
  if ! READINESS_PROCESS_PID="${pid}" READINESS_SETUP_BASH="${readiness_setup_bash}" \
    "${readiness_helper}" "${mode}" "${readiness_timeout_sec}" "${topic}"; then
    show_component_failure "${key}"
    return 1
  fi
}

wait_service_ready() {
  local key="$1"
  local service="$2"
  local deadline=$((SECONDS + readiness_timeout_sec))
  while ((SECONDS < deadline)); do
    component_running "${key}" || { show_component_failure "${key}"; return 1; }
    if timeout 5 ros2 service list 2>/dev/null | grep -Fxq "${service}"; then
      log_info "Service ready: ${service}"
      return 0
    fi
    sleep 1
  done
  log_error "Timed out waiting for service ${service}"
  show_component_failure "${key}"
  return 1
}

run_relocalization() {
  local output
  start_component "relocalization frame bridge" "relocalization_bridge" \
    python3 "${relocalization_bridge}" --ros-args \
    -p odom_topic:=/Odometry \
    -p output_odom_topic:=/planning/odom \
    -p map_frame:=map \
    -p odom_frame:=camera_init \
    -p body_frame:=body \
    -p publish_odom_body_tf:=false

  start_component "FAST-LIO global relocalization" "relocalization_backend" \
    ros2 launch fastlio_global_slam fastlio_global_slam.launch.py \
    start_fastlio:=false rviz:=false \
    "backend_config:=${relocalization_backend_config}"
  wait_service_ready "relocalization_backend" /fastlio_global_backend/load_map
  wait_service_ready "relocalization_backend" /fastlio_global_backend/relocalize

  log_info "Loading relocalization keyframes from ${global_map_dir}"
  output="$(timeout "${readiness_timeout_sec}s" ros2 service call \
    /fastlio_global_backend/load_map fastlio_global_slam/srv/LoadMap \
    "{directory: '${global_map_dir}'}" 2>&1)" || {
      printf '%s\n' "${output}" >&2
      log_error "Relocalization map service failed"
      return 1
    }
  printf '%s\n' "${output}"
  grep -Eq 'success[=:][[:space:]]*(true|True)' <<<"${output}" || {
    log_error "Relocalization map was not accepted"
    return 1
  }

  log_info "Waiting ${relocalization_fresh_scan_delay_sec}s for a fresh synchronized scan"
  sleep "${relocalization_fresh_scan_delay_sec}"
  wait_component_ready "fastlio" message /cloud_registered
  local attempt relocalization_ok=false
  for ((attempt=1; attempt<=relocalization_retry_count; attempt++)); do
    log_info "Relocalization attempt ${attempt}/${relocalization_retry_count}; keep the aircraft/vehicle still and in view of the mapped area"
    output="$(timeout "${relocalization_call_timeout_sec}s" ros2 service call \
      /fastlio_global_backend/relocalize fastlio_global_slam/srv/Relocalize \
      '{use_latest_scan: true}' 2>&1)" || true
    printf '%s\n' "${output}"
    if grep -Eq 'success[=:][[:space:]]*(true|True)' <<<"${output}"; then
      relocalization_ok=true
      break
    fi
    if ((attempt < relocalization_retry_count)); then
      log_warn "Relocalization did not find a confident candidate; waiting ${relocalization_retry_delay_sec}s for another synchronized scan"
      sleep "${relocalization_retry_delay_sec}"
    fi
  done
  if [[ "${relocalization_ok}" != true ]]; then
    log_error "Relocalization was rejected after ${relocalization_retry_count} attempts; MAVROS and control will not start"
    return 1
  fi
  wait_component_ready "relocalization_bridge" message /planning/odom
}

validate_configuration() {
  require_file /opt/ros/humble/setup.bash
  require_file "${project_root}/install/setup.bash"
  require_file "${autofix_launch}"
  require_executable "${livox_env}/run_mid360_driver.sh"
  case "${lio_backend}" in
    fast_lio)
      require_executable "${livox_env}/run_fastlio_mid360.sh"
      ;;
    fr_lio)
      require_file "${frlio_config}"
      ;;
    *)
      log_error "LIO_BACKEND must be fast_lio or fr_lio; got: ${lio_backend}"
      exit 1
      ;;
  esac
  require_file "${livox_setup}"
  require_executable "${lever_arm_validator}"
  require_executable "${readiness_helper}"
  require_executable "${env_cleanup}"
  require_executable "${component_runner}"
  require_executable "${preflight_check}"
  require_executable "${bag_script}"
  require_executable "${navigation_script}"
  require_file "${lever_arm_config}"
  require_file "${service_src}"
  require_file "${tuning_file}"

  require_boolean MISSION_ENABLED "${mission_enabled}"
  require_boolean NAVIGATION_ENABLED "${navigation_enabled}"
  require_boolean RECOGNITION_ENABLED "${recognition_enabled}"
  require_boolean RVIZ "${rviz}"
  require_boolean ENABLE_OUTPUT "${enable_output}"
  require_boolean MANUAL_HANDOVER "${manual_handover}"
  require_boolean EV_FAULT_AUTO_LAND "${ev_fault_auto_land}"
  require_boolean RECORD_BAG "${record_bag}"
  require_boolean NO_MAP_VALIDATION "${no_map_validation}"
  require_boolean MAP_AUTO_LOAD "${map_auto_load}"
  require_boolean COMPONENT_WINDOWS "${component_windows}"
  require_boolean RELOCALIZATION_ENABLED "${relocalization_enabled}"
  require_boolean FLIGHT_CPU_PERFORMANCE "${flight_cpu_performance}"
  require_boolean FLIGHT_CPU_AFFINITY "${flight_cpu_affinity}"
  if [[ ! "${rosbag_nice_level}" =~ ^([0-9]|1[0-9])$ ]]; then
    log_error "ROSBAG_NICE_LEVEL must be an integer in [0, 19]; got: ${rosbag_nice_level}"
    exit 1
  fi

  if [[ "${flight_cpu_performance}" == true ]]; then
    if ! command -v powerprofilesctl >/dev/null 2>&1; then
      log_error "FLIGHT_CPU_PERFORMANCE=true requires powerprofilesctl"
      exit 1
    fi
    if ! powerprofilesctl list 2>/dev/null | grep -Eq '^[*[:space:]]+performance:'; then
      log_error "This host does not expose a non-degraded performance CPU profile"
      exit 1
    fi
  fi

  if [[ "${relocalization_enabled}" == true ]]; then
    require_file "${relocalization_backend_config}"
    require_file "${relocalization_bridge}"
    [[ -d "${global_map_dir}/keyframes" ]] || {
      log_error "Relocalization keyframe directory is missing: ${global_map_dir}/keyframes"
      exit 1
    }
    [[ -s "${global_map_dir}/metadata.csv" ]] || {
      log_error "Relocalization metadata is missing: ${global_map_dir}/metadata.csv"
      exit 1
    }
    if awk -v value="${world_yaw_alignment_rad}" 'BEGIN { exit !(value != 0.0) }'; then
      log_error "Relocalization already defines map alignment; WORLD_YAW_ALIGNMENT_RAD must be 0.0"
      exit 1
    fi
    if [[ ! "${relocalization_retry_count}" =~ ^[1-9][0-9]*$ ]]; then
      log_error "RELOCALIZATION_RETRY_COUNT must be a positive integer"
      exit 1
    fi
    if ! awk -v value="${relocalization_retry_delay_sec}" 'BEGIN {exit !(value ~ /^[0-9]+([.][0-9]*)?$/)}'; then
      log_error "RELOCALIZATION_RETRY_DELAY_SEC must be a non-negative number"
      exit 1
    fi
  fi

  if [[ "${component_windows}" == true ]]; then
    if ! command -v gnome-terminal >/dev/null 2>&1; then
      log_error "COMPONENT_WINDOWS=true requires gnome-terminal"
      exit 1
    fi
    if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
      log_error "COMPONENT_WINDOWS=true requires a graphical desktop session"
      exit 1
    fi
  fi

  if [[ "${no_map_validation}" == true ]]; then
    planner_backend="${PLANNER_BACKEND:-ego-shadow}"
    mission_enabled="${MISSION_ENABLED:-false}"
    enable_output="${ENABLE_OUTPUT:-false}"
    rviz="${RVIZ:-false}"
  fi

  case "${planner_backend}" in
    astar_ego|super|ego|ego-shadow) ;;
    *)
      log_error "PLANNER_BACKEND must be astar_ego, super, ego or ego-shadow; got: ${planner_backend}"
      exit 1
      ;;
  esac
  resolve_effective_planner_backend
  if [[ ! "${readiness_timeout_sec}" =~ ^[0-9]+$ ]] || ((readiness_timeout_sec < 1)); then
    log_error "READINESS_TIMEOUT_SEC must be a positive integer"
    exit 1
  fi
  if [[ ! "${auto_stop_after_ready_sec}" =~ ^[0-9]+$ ]]; then
    log_error "AUTO_STOP_AFTER_READY_SEC must be a non-negative integer"
    exit 1
  fi
  if [[ ! "${mid360_fastlio_delay_sec}" =~ ^([0-9]+([.][0-9]*)?|[.][0-9]+)$ ]]; then
    log_error "MID360_FASTLIO_DELAY_SEC must be a non-negative number"
    exit 1
  fi
  if [[ "${skip_preflight_check}" != 0 && "${skip_preflight_check}" != 1 ]]; then
    log_error "SKIP_PREFLIGHT_CHECK must be 0 or 1"
    exit 1
  fi
  if [[ -z "${fcu_url}" ]]; then
    log_error "FCU_URL must not be empty"
    exit 1
  fi
  if [[ ! "${world_yaw_alignment_rad}" =~ ^[-+]?([0-9]+([.][0-9]*)?|[.][0-9]+)([eE][-+]?[0-9]+)?$ ]]; then
    log_error "WORLD_YAW_ALIGNMENT_RAD must be numeric"
    exit 1
  fi
  case "${allow_unvalidated_world_yaw}" in
    true|false) ;;
    *)
      log_error "ALLOW_UNVALIDATED_WORLD_YAW must be true or false"
      exit 1
      ;;
  esac
  if awk -v value="${world_yaw_alignment_rad}" 'BEGIN { exit !(value != 0.0) }'; then
    if [[ "${allow_unvalidated_world_yaw}" != true ]]; then
      log_error "WORLD_YAW_ALIGNMENT_RAD=${world_yaw_alignment_rad} is not validated; normal flight requires 0.0. Unset the stale environment variable."
      exit 1
    fi
    if [[ "${mission_enabled}" != false || "${enable_output}" != false ]]; then
      log_error "Unvalidated world yaw is allowed only with MISSION_ENABLED=false and ENABLE_OUTPUT=false."
      exit 1
    fi
    log_warn "Using unvalidated WORLD_YAW_ALIGNMENT_RAD=${world_yaw_alignment_rad} under explicit experimental override. Propeller-off operation only."
  fi
  if [[ -z "${map_file}" ]]; then
    if [[ "${map_auto_load}" == true ]]; then
      log_error "MAP_FILE is empty but MAP_AUTO_LOAD=true; use a PCD or set MAP_AUTO_LOAD=false for validation"
      exit 1
    fi
    if [[ "${mission_enabled}" == true || "${enable_output}" == true ]]; then
      log_error "No-map validation requires MISSION_ENABLED=false and ENABLE_OUTPUT=false"
      exit 1
    fi
  elif [[ ! -f "${map_file}" ]]; then
    log_error "MAP_FILE does not exist: ${map_file}"
    exit 1
  fi
}

activate_cpu_performance() {
  [[ "${flight_cpu_performance}" == true ]] || {
    log_warn "FLIGHT_CPU_PERFORMANCE=false: CPU performance profile is not enforced."
    return 0
  }

  cpu_profile_before="$(powerprofilesctl get)"
  if [[ "${cpu_profile_before}" != performance ]]; then
    powerprofilesctl set performance
    cpu_profile_changed=true
  fi
  if [[ "$(powerprofilesctl get)" != performance ]]; then
    log_error "Failed to activate the CPU performance profile"
    return 1
  fi

  local policy epp
  for policy in /sys/devices/system/cpu/cpufreq/policy*; do
    [[ -r "${policy}/energy_performance_preference" ]] || continue
    epp="$(<"${policy}/energy_performance_preference")"
    if [[ "${epp}" != performance ]]; then
      log_error "CPU policy ${policy##*/} EPP is ${epp}, expected performance"
      return 1
    fi
  done
  log_info "CPU performance profile locked for this flight (previous=${cpu_profile_before})."
}

prepare_run() {
  mkdir -p "${flight_log_dir}"
  mkdir -p "$(dirname -- "${stack_ready_file}")"
  rm -f "${stack_ready_file}"
  # 把日志转发进程放到本终端前台进程组之外。否则 Ctrl+C 会先杀掉 tee，而信号
  # 处理函数往由此产生的断开管道里写数据，会在 Bash 内部递归触发 SIGPIPE。
  exec > >(setsid --wait tee -a "${flight_log_dir}/startup.log") 2>&1

  # 启动前确保环境干净。上一次运行残留的节点会造成两类真实故障：EV 桥重复启动时
  # PX4 会同时收到两路外部视觉；FAST-LIO 长时间挂在旧驱动上会积压追赶、发散到几十
  # 公里外，而这些症状要到验证跑完才看得出来。本脚本自己会拉起全部组件，所以按
  # all 清理。
  log_info "启动前环境清理..."
  if ! "${env_cleanup}" all; then
    log_error "启动前环境清理失败；请处理上面列出的进程后重试。"
    exit 1
  fi

  "${lever_arm_validator}" "${lever_arm_config}"
  # shellcheck disable=SC1090
  source "${lever_arm_config}"
  # Do not allow the FAST-LIO environment or an old workspace overlay to select
  # a legacy launch with a second PX4 external-vision writer.
  prefer_workspace_px4_ros_com
  local resolved_px4_share expected_px4_share
  resolved_px4_share="$(python3 - <<'PY'
from ament_index_python.packages import get_package_share_directory
print(get_package_share_directory("px4_ros_com"))
PY
)"
  expected_px4_share="${project_root}/install/px4_ros_com/share/px4_ros_com"
  if [[ "${resolved_px4_share}" != "${expected_px4_share}" ]]; then
    log_error "px4_ros_com is shadowed by an old overlay: ${resolved_px4_share}"
    log_error "Expected current workspace: ${expected_px4_share}"
    exit 1
  fi
  if grep -Eq 'DeclareLaunchArgument\("start_(bridge|px4_ev_bridge)' "${autofix_launch}" ||
     grep -Eq 'Only one external-vision output may feed PX4' "${autofix_launch}"; then
    log_error "fastlio_mavros_autofix.launch.py contains the legacy multi-EV path; refusing to start."
    exit 1
  fi
  cp "${lever_arm_config}" "${flight_run_dir}/mid360_lever_arm.conf"

  if [[ "${skip_preflight_check}" == 0 ]]; then
    log_info "Running canonical preflight check..."
    "${preflight_check}"
  else
    log_warn "SKIP_PREFLIGHT_CHECK=1: canonical preflight was explicitly skipped."
  fi

  if [[ "${record_bag}" == true ]]; then
    "${project_root}/tools/rosbag/刷新录包状态.sh" || true
    mkdir -p "$(dirname -- "${service_dst}")"
    cp "${service_src}" "${service_dst}"
    chmod 644 "${service_dst}"
    systemctl --user daemon-reload || true
    systemctl --user enable --now "${service_name}" >/dev/null 2>&1 || true
  fi
}

print_configuration() {
  log_info "Flight run directory: ${flight_run_dir}"
  log_info "Navigation=${navigation_enabled}; planner requested=${planner_backend}, effective=${effective_planner_backend}, ego_enabled=${ego_enabled}, mission=${mission_enabled}, output=${enable_output}, manual_handover=${manual_handover}, rviz=${rviz}"
  log_info "Recognition=${recognition_enabled}, rosbag=${record_bag}, readiness_timeout=${readiness_timeout_sec}s"
  log_info "LIO backend=${lio_backend}; MID-360 to LIO minimum startup delay=${mid360_fastlio_delay_sec}s"
  log_info "CPU performance profile enforcement=${flight_cpu_performance}"
  log_info "CPU affinity: MID-360=${livox_cpu_affinity:-auto} FR-LIO=${frlio_cpu_affinity:-auto} planner=${planner_cpu_affinity:-auto} rosbag=${rosbag_cpu_affinity:-auto}"
  log_info "rosbag recorder nice level: +${rosbag_nice_level}"
  if [[ "${lio_backend}" == fr_lio ]]; then
    log_warn "FR-LIO high-rate output remains diagnostic only and is not authorized to replace the PX4 EV path."
  fi
  log_info "Component windows=${component_windows} (one GNOME Terminal window per component)"
  log_info "Map: ${map_file:-<disabled for validation>} (auto_load=${map_auto_load})"
  log_info "Global relocalization=${relocalization_enabled} keyframes=${global_map_dir}"
  if [[ "${map_file}" == "${default_map_file}" ]]; then
    log_warn "The bundled default map is not confirmed as the arena scan. Set MAP_FILE to the measured arena PCD before field operation."
  fi
  if [[ "${no_map_validation}" == true ]]; then
    log_warn "NO_MAP_VALIDATION=true: planner output and mission control are disabled; this run is for sensor/EV/link validation only."
  fi
  log_info "MID360 body-to-sensor FLU: x=${MID360_BODY_TO_SENSOR_X_M} m, y=${MID360_BODY_TO_SENSOR_Y_M} m, z=${MID360_BODY_TO_SENSOR_Z_M} m, yaw=${MID360_BODY_TO_FASTLIO_YAW_RAD} rad"
  log_warn "Horizontal-drift and takeoff-transient guards are absent. Keep manual RC takeover available."
}

start_stack() {
  if [[ "${navigation_enabled}" == true ]]; then
    start_component "navigation stack (prewarm)" "navigation" \
      env MAP_FILE="${map_file}" \
      PLANNER_BACKEND="${effective_planner_backend}" \
      TUNING_FILE="${tuning_file}" \
      MISSION_ENABLED="${mission_enabled}" \
      RECOGNITION_ENABLED="${recognition_enabled}" \
      RVIZ="${rviz}" \
      ENABLE_OUTPUT="${enable_output}" \
      MANUAL_HANDOVER="${manual_handover}" \
      EV_FAULT_AUTO_LAND="${ev_fault_auto_land}" \
      BODY_TO_SENSOR_X_M="${MID360_BODY_TO_SENSOR_X_M}" \
      BODY_TO_SENSOR_Y_M="${MID360_BODY_TO_SENSOR_Y_M}" \
      BODY_TO_SENSOR_Z_M="${MID360_BODY_TO_SENSOR_Z_M}" \
      BODY_TO_FASTLIO_YAW_RAD="${MID360_BODY_TO_FASTLIO_YAW_RAD}" \
      WORLD_YAW_ALIGNMENT_RAD="${world_yaw_alignment_rad}" \
      PUBLISH_FASTLIO_BRIDGE=true \
      PUBLISH_CAMERA_INIT_TF="$([[ "${relocalization_enabled}" == true ]] && echo false || echo true)" \
      MAP_FRAME_ID="map" \
      FASTLIO_ODOM_TOPIC="$([[ "${relocalization_enabled}" == true ]] && echo /planning/odom || echo /Odometry)" \
      REQUIRE_MAP_LOCAL_ALIGNMENT="${relocalization_enabled}" \
      MAP_AUTO_LOAD="${map_auto_load}" \
      "${navigation_script}"
    wait_component_ready "navigation" message /race/control/status
    if [[ -n "${map_file}" && "${map_auto_load}" == true ]]; then
      wait_component_ready "navigation" topic /saved_map
    else
      log_warn "No map is loaded; skipping /saved_map readiness (validation mode only)."
    fi
    if [[ "${rviz}" == true ]]; then
      wait_component_ready "navigation" node /rviz2
    fi
  else
    log_info "NAVIGATION_ENABLED=false: skipping navigation and all Offboard-capable components."
  fi

  start_component "MID-360 driver" "mid360_driver" \
    "${livox_env}/run_mid360_driver.sh"
  log_info "Waiting ${mid360_fastlio_delay_sec}s after MID-360 startup before ${lio_backend}..."
  sleep "${mid360_fastlio_delay_sec}"
  wait_component_ready "mid360_driver" message /livox/imu
  wait_component_ready "mid360_driver" message /livox/lidar

  if [[ "${lio_backend}" == fr_lio ]]; then
    start_component "FR-LIO" "fastlio" \
      ros2 launch fr_lio lio.launch.py "config_file:=${frlio_config}" \
      rviz:=false lidar_accumulator:=false
  else
    start_component "FAST-LIO" "fastlio" \
      "${livox_env}/run_fastlio_mid360.sh"
  fi
  wait_component_ready "fastlio" message /Odometry
  wait_component_ready "fastlio" message /cloud_registered_body

  if [[ "${relocalization_enabled}" == true ]]; then
    run_relocalization
  fi

  local ev_odom_topic=/Odometry
  if [[ "${relocalization_enabled}" == true ]]; then
    ev_odom_topic=/planning/odom
  fi

  start_component "MAVROS 与 EV 定位链" "px4_mavros" \
    ros2 launch px4_ros_com fastlio_mavros_autofix.launch.py \
    "fcu_url:=${fcu_url}" \
    start_mavros_odometry_bridge:=true \
    start_px4_vehicle_odometry:=false \
    start_odom_guard:=true \
    start_ev_health_monitor:=true \
    start_tf:=true \
    "world_yaw_alignment_rad:=${world_yaw_alignment_rad}" \
    "fastlio_odom_topic:=${ev_odom_topic}" \
    "require_frlio_anchor_status:=true" \
    "body_to_sensor_x_m:=${MID360_BODY_TO_SENSOR_X_M}" \
    "body_to_sensor_y_m:=${MID360_BODY_TO_SENSOR_Y_M}" \
    "body_to_sensor_z_m:=${MID360_BODY_TO_SENSOR_Z_M}" \
    "body_to_fastlio_yaw_rad:=${MID360_BODY_TO_FASTLIO_YAW_RAD}"
  wait_component_ready "px4_mavros" mavros_connected /mavros/state
  wait_component_ready "px4_mavros" message /mavros/local_position/odom
  wait_component_ready "px4_mavros" message /mavros/local_position/velocity_local
  wait_component_ready "px4_mavros" flight_ready /ev_health/flight_ready
  wait_component_ready "px4_mavros" message /Odometry/healthy
  wait_component_ready "px4_mavros" message /mavros/odometry/out

  # MAVROS is the sole EV ingress for this MAVLink-only topology. No uXRCE-DDS
  # or /fmu topic is required or read.
  local odometry_out_info retired_info
  odometry_out_info="$(ros2 topic info -v /mavros/odometry/out 2>/dev/null || true)"
  if ! grep -Eq 'Publisher count:[[:space:]]+1$' <<<"${odometry_out_info}"; then
    log_error "EV 桥接约束失败：/mavros/odometry/out 必须恰有一个发布者。"
    printf '%s\n' "${odometry_out_info}" >&2
    return 1
  fi
  # /mavros/odometry/out is the selected MAVROS EV ingress and was validated
  # above. The retired vision and native-DDS inputs must remain publisher-free.
  for retired_topic in /mavros/vision_pose/pose_cov /mavros/vision_speed/speed_twist_cov /fmu/in/vehicle_visual_odometry; do
    retired_info="$(ros2 topic info -v "${retired_topic}" 2>/dev/null || true)"
    if grep -Eq 'Publisher count:[[:space:]]+[1-9]' <<<"${retired_info}"; then
      log_error "EV 桥接约束失败：已停用输入 ${retired_topic} 仍有发布者。"
      printf '%s\n' "${retired_info}" >&2
      return 1
    fi
  done

  if [[ "${relocalization_enabled}" == true ]]; then
    log_info "等待定位控制链锁定 map 到 PX4 local 的坐标变换。"
    local alignment_deadline=$((SECONDS + 30))
    local alignment_status=""
    while ((SECONDS < alignment_deadline)); do
      alignment_status="$(timeout 5 ros2 topic echo --once /race/control/status --field data 2>/dev/null || true)"
      grep -q 'map_local_alignment=1' <<<"${alignment_status}" && break
      sleep 1
    done
    grep -q 'map_local_alignment=1' <<<"${alignment_status}" || {
      log_error "定位控制链未锁定 map 到 PX4 local 的对齐关系，继续禁止飞行。"
      return 1
    }
  fi

  if [[ "${record_bag}" == true ]]; then
    start_component "debug rosbag" "rosbag_debug" \
      env FLIGHT_TIMESTAMP="${flight_timestamp}" FLIGHT_RUN_DIR="${flight_run_dir}" \
      ROSBAG_CPU_AFFINITY="${rosbag_cpu_affinity}" ROSBAG_NICE_LEVEL="${rosbag_nice_level}" \
      "${bag_script}"
    wait_component_ready "rosbag_debug" node /rosbag2_recorder
  else
    log_warn "RECORD_BAG=false: rosbag recording is disabled for this run."
  fi

  if [[ "${navigation_enabled}" == true ]]; then
    wait_component_ready "navigation" message /race/odom
  fi
  if [[ "${navigation_enabled}" == true ]]; then
    if [[ "${effective_planner_backend}" != super && "${map_auto_load}" == true ]]; then
      # static_live_px4 only publishes after both the static map and the real
      # body cloud have been fused with a synchronized PX4 pose. This gate catches
      # topic/remap mistakes before the pilot is ever invited to arm.
      wait_component_ready "navigation" message /race/ego/cloud
    fi
  fi
}

monitor_stack() {
  local key
  while true; do
    for key in "${started_components[@]}"; do
      if ! component_running "${key}"; then
        show_component_failure "${key}"
        return 1
      fi
    done
    sleep 2
  done
}

main() {
  validate_configuration

  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  # FR-LIO directly subscribes to livox_ros_driver2/CustomMsg.  The MID-360
  # driver sources this overlay for itself, but FR-LIO is launched by this
  # parent process and therefore also needs its typesupport libraries here.
  # shellcheck disable=SC1091
  source "${livox_setup}"
  # shellcheck disable=SC1091
  source "${project_root}/install/setup.bash"
  set -u

  prepare_run
  print_configuration
  activate_cpu_performance
  start_stack

  log_info "PASS: complete chain is ready: MID-360 -> ${lio_backend} -> relocalization(${relocalization_enabled}) -> MAVROS/EV -> rosbag(${record_bag}) -> navigation."
  printf 'state=ready\nflight_run_dir=%s\n' "${flight_run_dir}" >"${stack_ready_file}.tmp.$$"
  mv -f "${stack_ready_file}.tmp.$$" "${stack_ready_file}"
  log_info "Component logs: ${flight_log_dir}"
  log_info "This terminal now monitors the stack. Press Ctrl+C to stop all components from this run."
  if ((auto_stop_after_ready_sec > 0)); then
    log_info "Auto-stop validation: shutting down in ${auto_stop_after_ready_sec}s."
    sleep "${auto_stop_after_ready_sec}"
    return 0
  fi
  monitor_stack
}

main "$@"
