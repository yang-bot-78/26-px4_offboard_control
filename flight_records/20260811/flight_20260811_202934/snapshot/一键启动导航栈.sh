#!/usr/bin/env bash
set -euo pipefail

# 统一的硬件启动顺序：
#   导航预热 -> MID-360 -> FAST-LIO -> MAVROS/EV -> rosbag。
# 导航先于实时定位链路起来，这样节点发现和规划器初始化就不会让 FAST-LIO 去处理
# 一堆积压的旧雷达数据。数据就绪检查仍然按依赖顺序进行。
#
# Usage:
#   ./tools/flight/一键启动导航栈.sh
# 可选环境变量：MAP_FILE、NO_MAP_VALIDATION、MAP_AUTO_LOAD、
# FCU_URL, VISION_WORLD_YAW_ALIGNMENT_RAD,
# PLANNER_BACKEND, MISSION_ENABLED, RECOGNITION_ENABLED, RVIZ, ENABLE_OUTPUT,
# MANUAL_HANDOVER,
# RECORD_BAG, READINESS_TIMEOUT_SEC, AUTO_STOP_AFTER_READY_SEC,
# MID360_FASTLIO_DELAY_SEC,
# COMPONENT_WINDOWS, COMPONENT_WINDOW_GEOMETRY,
# SKIP_PREFLIGHT_CHECK.

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
project_root="$(cd -- "${script_dir}/../.." && pwd -P)"
livox_env="${LIVOX_MID360_ENV:-${HOME}/livox_mid360_env}"
livox_setup="${livox_env}/setup_mid360.bash"

default_map_file="${project_root}/maps/fastlio_global_3d/GlobalMap.pcd"
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
mission_enabled="${MISSION_ENABLED:-true}"
recognition_enabled="${RECOGNITION_ENABLED:-false}"
rviz="${RVIZ:-true}"
enable_output="${ENABLE_OUTPUT:-true}"
manual_handover="${MANUAL_HANDOVER:-false}"
record_bag="${RECORD_BAG:-true}"
readiness_timeout_sec="${READINESS_TIMEOUT_SEC:-90}"
auto_stop_after_ready_sec="${AUTO_STOP_AFTER_READY_SEC:-0}"
mid360_fastlio_delay_sec="${MID360_FASTLIO_DELAY_SEC:-4}"
component_windows="${COMPONENT_WINDOWS:-true}"
component_window_geometry="${COMPONENT_WINDOW_GEOMETRY:-110x30}"
fcu_url="${FCU_URL:-serial:///dev/ttyUSB0:921600?ids=255,190}"
vision_world_yaw_alignment_rad="${VISION_WORLD_YAW_ALIGNMENT_RAD:-0.0}"
skip_preflight_check="${SKIP_PREFLIGHT_CHECK:-0}"

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
  local launcher_pid pid pgid

  log_info "Starting ${title} in its own terminal window; log=${log_file}"
  mkdir -p "$(dirname -- "${runtime_file}")"
  : >"${runtime_file}"
  if [[ "${component_windows}" == true ]]; then
    gnome-terminal --window --wait \
      --title="${title} | ${flight_timestamp}" \
      --geometry="${component_window_geometry}" \
      --working-directory="${project_root}" \
      -- "${component_runner}" "$$" "${owner_start_ticks}" \
      "${runtime_file}" "${log_file}" "${title}" -- "$@" &
  else
    "${component_runner}" "$$" "${owner_start_ticks}" \
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

validate_configuration() {
  require_file /opt/ros/humble/setup.bash
  require_file "${project_root}/install/setup.bash"
  require_executable "${livox_env}/run_mid360_driver.sh"
  require_executable "${livox_env}/run_fastlio_mid360.sh"
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

  require_boolean MISSION_ENABLED "${mission_enabled}"
  require_boolean RECOGNITION_ENABLED "${recognition_enabled}"
  require_boolean RVIZ "${rviz}"
  require_boolean ENABLE_OUTPUT "${enable_output}"
  require_boolean MANUAL_HANDOVER "${manual_handover}"
  require_boolean RECORD_BAG "${record_bag}"
  require_boolean NO_MAP_VALIDATION "${no_map_validation}"
  require_boolean MAP_AUTO_LOAD "${map_auto_load}"
  require_boolean COMPONENT_WINDOWS "${component_windows}"

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
  if [[ ! "${vision_world_yaw_alignment_rad}" =~ ^[-+]?([0-9]+([.][0-9]*)?|[.][0-9]+)([eE][-+]?[0-9]+)?$ ]]; then
    log_error "VISION_WORLD_YAW_ALIGNMENT_RAD must be numeric"
    exit 1
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
  log_info "Planner=${planner_backend}, mission=${mission_enabled}, output=${enable_output}, manual_handover=${manual_handover}, rviz=${rviz}"
  log_info "Recognition=${recognition_enabled}, rosbag=${record_bag}, readiness_timeout=${readiness_timeout_sec}s"
  log_info "MID-360 to FAST-LIO minimum startup delay=${mid360_fastlio_delay_sec}s"
  log_info "Component windows=${component_windows} (one GNOME Terminal window per component)"
  log_info "Map: ${map_file:-<disabled for validation>} (auto_load=${map_auto_load})"
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
  start_component "navigation stack (prewarm)" "navigation" \
    env MAP_FILE="${map_file}" \
    PLANNER_BACKEND="${planner_backend}" \
    MISSION_ENABLED="${mission_enabled}" \
    RECOGNITION_ENABLED="${recognition_enabled}" \
    RVIZ="${rviz}" \
    ENABLE_OUTPUT="${enable_output}" \
    MANUAL_HANDOVER="${manual_handover}" \
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

  start_component "MID-360 driver" "mid360_driver" \
    "${livox_env}/run_mid360_driver.sh"
  log_info "Waiting ${mid360_fastlio_delay_sec}s after MID-360 startup before FAST-LIO..."
  sleep "${mid360_fastlio_delay_sec}"
  wait_component_ready "mid360_driver" message /livox/imu
  wait_component_ready "mid360_driver" message /livox/lidar

  start_component "FAST-LIO" "fastlio" \
    "${livox_env}/run_fastlio_mid360.sh"
  wait_component_ready "fastlio" message /Odometry

  start_component "MAVROS and EV chain" "px4_mavros" \
    ros2 launch px4_ros_com fastlio_mavros_autofix.launch.py \
    "fcu_url:=${fcu_url}" \
    start_mavros_vision_bridge:=true \
    start_odom_guard:=true \
    start_ev_health_monitor:=true \
    start_tf:=true \
    "vision_world_yaw_alignment_rad:=${vision_world_yaw_alignment_rad}" \
    "body_to_sensor_x_m:=${MID360_BODY_TO_SENSOR_X_M}" \
    "body_to_sensor_y_m:=${MID360_BODY_TO_SENSOR_Y_M}" \
    "body_to_sensor_z_m:=${MID360_BODY_TO_SENSOR_Z_M}" \
    "body_to_fastlio_yaw_rad:=${MID360_BODY_TO_FASTLIO_YAW_RAD}"
  wait_component_ready "px4_mavros" mavros_connected /mavros/state
  wait_component_ready "px4_mavros" message /mavros/local_position/odom
  wait_component_ready "px4_mavros" message /mavros/local_position/velocity_local
  wait_component_ready "px4_mavros" healthy /ev_health/status
  wait_component_ready "px4_mavros" message /Odometry/healthy
  wait_component_ready "px4_mavros" message /mavros/vision_pose/pose_cov
  wait_component_ready "px4_mavros" message /mavros/vision_speed/speed_twist_cov

  if [[ "${record_bag}" == true ]]; then
    start_component "debug rosbag" "rosbag_debug" \
      env FLIGHT_TIMESTAMP="${flight_timestamp}" FLIGHT_RUN_DIR="${flight_run_dir}" \
      "${bag_script}"
    wait_component_ready "rosbag_debug" node /rosbag2_recorder
    wait_component_ready "px4_mavros" healthy /ev_health/status
    wait_component_ready "px4_mavros" message /Odometry/healthy
    wait_component_ready "px4_mavros" message /mavros/vision_pose/pose_cov
  else
    log_warn "RECORD_BAG=false: rosbag recording is disabled for this run."
  fi

  wait_component_ready "navigation" message /race/odom
  wait_component_ready "px4_mavros" healthy /ev_health/status
  wait_component_ready "px4_mavros" message /mavros/vision_pose/pose_cov
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
  # shellcheck disable=SC1091
  source "${project_root}/install/setup.bash"
  set -u

  prepare_run
  print_configuration
  start_stack

  log_info "PASS: complete chain is ready: MID-360 -> FAST-LIO -> MAVROS/EV -> rosbag(${record_bag}) -> navigation."
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
