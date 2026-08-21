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
# MANUAL_HANDOVER, EV_FAULT_AUTO_LAND, PX4_COMPONENTS_ENABLED,
# GLOBAL_WAYPOINT_TASK_ENABLED, PUBLISH_GLOBAL_PATH, WAYPOINT_VISUALIZER_ENABLED,
# WAYPOINTS_FILE, RELOCALIZATION_ENABLED, FRLIO_GLOBAL_MAP_DIR,
# RECORD_BAG, READINESS_TIMEOUT_SEC, AUTO_STOP_AFTER_READY_SEC,
# MID360_FASTLIO_DELAY_SEC, LIO_BACKEND, FRLIO_CONFIG,
# COMPONENT_WINDOWS, COMPONENT_WINDOW_GEOMETRY,
# RELOCALIZATION_RETRY_COUNT, RELOCALIZATION_RETRY_DELAY_SEC,
# REQUIRE_MAP_LOCAL_ALIGNMENT,
# RELOCALIZATION_INTERACTIVE, RELOCALIZATION_TRIGGER_TTY,
# SKIP_PREFLIGHT_CHECK, CPU_AFFINITY_ENABLED, CPUSET_MID360,
# CPUSET_FRLIO, CPUSET_MAVROS_EV, CPUSET_NAVIGATION,
# CPUSET_RELOCALIZATION_BRIDGE, CPUSET_RELOCALIZATION_BACKEND, CPUSET_ROSBAG.

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
project_root="$(cd -- "${script_dir}/../.." && pwd -P)"
autofix_launch="${project_root}/src/px4_ros_com/launch/fastlio_mavros_autofix.launch.py"
livox_env="${LIVOX_MID360_ENV:-${HOME}/livox_mid360_env}"
livox_setup="${livox_env}/setup_mid360.bash"
lio_backend="${LIO_BACKEND:-fr_lio}"
frlio_config="${FRLIO_CONFIG:-${project_root}/src/fr_lio/config/indoors.yaml}"
frlio_source="${project_root}/src/fr_lio/src/laser_mapping.cpp"
frlio_executable="${project_root}/install/fr_lio/lib/fr_lio/frlio"

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
global_waypoint_task_enabled="${GLOBAL_WAYPOINT_TASK_ENABLED:-false}"
publish_global_path="${PUBLISH_GLOBAL_PATH:-false}"
px4_components_enabled="${PX4_COMPONENTS_ENABLED:-true}"
waypoint_visualizer_enabled="${WAYPOINT_VISUALIZER_ENABLED:-false}"
waypoint_fsm_enabled="${WAYPOINT_FSM_ENABLED:-false}"
waypoints_file="${WAYPOINTS_FILE:-${project_root}/src/race_offboard/config/waypoints/main/waypoints.yaml}"
recognition_enabled="${RECOGNITION_ENABLED:-false}"
rviz="${RVIZ:-true}"
enable_output="${ENABLE_OUTPUT:-true}"
manual_handover="${MANUAL_HANDOVER:-false}"
ev_fault_auto_land="${EV_FAULT_AUTO_LAND:-true}"
record_bag="${RECORD_BAG:-true}"
require_flight_ready_for_startup="${REQUIRE_FLIGHT_READY_FOR_STARTUP:-true}"
rosbag_profile="${ROSBAG_PROFILE:-full}"
readiness_timeout_sec="${READINESS_TIMEOUT_SEC:-90}"
auto_stop_after_ready_sec="${AUTO_STOP_AFTER_READY_SEC:-0}"
mid360_fastlio_delay_sec="${MID360_FRLIO_DELAY_SEC:-${MID360_FASTLIO_DELAY_SEC:-4}}"
component_windows="${COMPONENT_WINDOWS:-true}"
component_window_geometry="${COMPONENT_WINDOW_GEOMETRY:-110x30}"
fcu_url="${FCU_URL:-serial:///dev/ttyUSB0:921600?ids=255,190}"
world_yaw_alignment_rad="${WORLD_YAW_ALIGNMENT_RAD:-0.0}"
allow_unvalidated_world_yaw="${ALLOW_UNVALIDATED_WORLD_YAW:-false}"
skip_preflight_check="${SKIP_PREFLIGHT_CHECK:-0}"
relocalization_enabled="${RELOCALIZATION_ENABLED:-false}"
require_map_local_alignment="${REQUIRE_MAP_LOCAL_ALIGNMENT:-false}"
global_map_dir="${FRLIO_GLOBAL_MAP_DIR:-${FASTLIO_GLOBAL_MAP_DIR:-${project_root}/maps/main}}"
relocalization_backend_config="${RELOCALIZATION_BACKEND_CONFIG:-${project_root}/tools/fastlio/只重定位一次后端参数.yaml}"
relocalization_bridge="${project_root}/tools/fastlio/重定位坐标桥.py"
relocalization_fresh_scan_delay_sec="${RELOCALIZATION_FRESH_SCAN_DELAY_SEC:-3}"
relocalization_call_timeout_sec="${RELOCALIZATION_CALL_TIMEOUT_SEC:-120}"
relocalization_retry_count="${RELOCALIZATION_RETRY_COUNT:-8}"
relocalization_retry_delay_sec="${RELOCALIZATION_RETRY_DELAY_SEC:-2}"
relocalization_interactive="${RELOCALIZATION_INTERACTIVE:-false}"
relocalization_trigger_tty="${RELOCALIZATION_TRIGGER_TTY:-/dev/tty}"
# The field computer is an i5-1340P: 0-1, 2-3, 4-5, and 6-7 are its four
# P-cores with SMT; 8-15 are E-cores.  Give each critical component whole
# P-cores so separate components never share an SMT pair, and move
# display/recording work to E-cores.  "auto" only enables this map when the
# same 16-CPU asymmetric topology is detected.
cpu_affinity_enabled="${CPU_AFFINITY_ENABLED:-auto}"
cpu_set_mid360="${CPUSET_MID360:-0-1}"
cpu_set_frlio="${CPUSET_FRLIO:-2-5}"
cpu_set_mavros_ev="${CPUSET_MAVROS_EV:-6-7}"
cpu_set_navigation="${CPUSET_NAVIGATION:-8-11}"
cpu_set_relocalization_bridge="${CPUSET_RELOCALIZATION_BRIDGE:-12}"
cpu_set_relocalization_backend="${CPUSET_RELOCALIZATION_BACKEND:-13-15}"
cpu_set_rosbag="${CPUSET_ROSBAG:-12-15}"
cpu_affinity_active=false

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
  printf '[信息] %s\n' "$*"
}

log_warn() {
  printf '[警告] %s\n' "$*" >&2
}

log_error() {
  printf '[错误] %s\n' "$*" >&2
}

require_file() {
  local path="$1"
  if [[ ! -f "${path}" ]]; then
    log_error "缺少必需文件：${path}"
    exit 1
  fi
}

require_executable() {
  local path="$1"
  require_file "${path}"
  if [[ ! -x "${path}" ]]; then
    log_error "必需文件不可执行：${path}"
    exit 1
  fi
}

validate_frlio_livox_qos_build() {
  require_file "${frlio_config}"
  require_file "${frlio_source}"
  require_executable "${frlio_executable}"

  if ! grep -Fq 'const auto livox_qos = rclcpp::SensorDataQoS().keep_last(5);' "${frlio_source}"; then
    log_error "FR-LIO 未包含必需的 Livox Best Effort QoS 订阅修复：${frlio_source}"
    log_error "飞行前请从当前项目源码重新编译。"
    exit 1
  fi
  if [[ "${frlio_source}" -nt "${frlio_executable}" ]]; then
    log_error "FR-LIO 可执行文件早于其 Livox QoS 修复源码：${frlio_executable}"
    log_error "请执行：cd ${project_root} && colcon build --packages-select fr_lio --symlink-install"
    exit 1
  fi

  validate_frlio_flight_config
}

validate_frlio_flight_config() {
  # Loop-closure map correction is a mapping/research path, not a flight path.
  # In particular, correct_working_tree rebuilds the live ikd-tree while the
  # IESKF hot path searches it; FR-LIO documents this as divergence-prone.
  if ! python3 - "${frlio_config}" <<'PY'
import sys
from pathlib import Path

try:
    import yaml
except ImportError as exc:
    raise SystemExit(f"无法解析 FR-LIO YAML（缺少 PyYAML）：{exc}")

path = Path(sys.argv[1])
with path.open(encoding="utf-8") as stream:
    document = yaml.safe_load(stream) or {}

parameters = document.get("/**", {}).get("ros__parameters", {})
checks = {
    "lc.enable": parameters.get("lc", {}).get("enable", False),
    "mapping.enable_map_correction": parameters.get("mapping", {}).get(
        "enable_map_correction", False
    ),
    "mapping.correct_working_tree": parameters.get("mapping", {}).get(
        "correct_working_tree", False
    ),
}
unsafe = [name for name, value in checks.items() if value is True]
invalid = [name for name, value in checks.items() if not isinstance(value, bool)]
if invalid:
    raise SystemExit("FR-LIO 飞行配置必须使用布尔值：" + ", ".join(invalid))
if unsafe:
    raise SystemExit(
        "FR-LIO 飞行配置禁止开启回环/地图修正：" + ", ".join(unsafe)
    )
print("FR-LIO flight config: lc.enable=false, "
      "mapping.enable_map_correction=false, "
      "mapping.correct_working_tree=false")
PY
  then
    log_error "FR-LIO 配置不满足飞行安全门禁：${frlio_config}"
    exit 1
  fi
}

validate_frlio_runtime_overlay() {
  local resolved_prefix expected_prefix
  expected_prefix="$(cd -- "${project_root}/install/fr_lio" && pwd -P)"
  resolved_prefix="$(ros2 pkg prefix fr_lio 2>/dev/null || true)"
  if [[ "${resolved_prefix}" != "${expected_prefix}" ]]; then
    log_error "FR-LIO 解析到了意外的工作区覆盖：${resolved_prefix:-<未找到>}"
    log_error "应使用当前已编译工作区的软件包：${expected_prefix}"
    exit 1
  fi
  log_info "FR-LIO Livox CustomMsg 输入 QoS：SensorDataQoS（尽力而为）；软件包=${resolved_prefix}"
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
      log_error "${name} 必须为 true 或 false；当前值：${value}"
      exit 1
      ;;
  esac
}

require_rosbag_profile() {
  case "$1" in
    full|low|trace|strosbag) ;;
    *)
      log_error "ROSBAG_PROFILE 必须为 full、low、trace 或 strosbag；当前值：$1"
      exit 1
      ;;
  esac
}

component_cpu_set() {
  case "$1" in
    mid360_driver) printf '%s\n' "${cpu_set_mid360}" ;;
    fastlio) printf '%s\n' "${cpu_set_frlio}" ;;
    px4_mavros) printf '%s\n' "${cpu_set_mavros_ev}" ;;
    navigation) printf '%s\n' "${cpu_set_navigation}" ;;
    relocalization_bridge) printf '%s\n' "${cpu_set_relocalization_bridge}" ;;
    relocalization_backend) printf '%s\n' "${cpu_set_relocalization_backend}" ;;
    rosbag_debug) printf '%s\n' "${cpu_set_rosbag}" ;;
    *) return 1 ;;
  esac
}

configure_cpu_affinity() {
  local requested="${cpu_affinity_enabled}"
  local cpu_count performance_max_mhz efficiency_max_mhz cpu_set

  case "${requested}" in
    true|false|auto) ;;
    *)
      log_error "CPU_AFFINITY_ENABLED 必须为 true、false 或 auto；当前值：${requested}"
      exit 1
      ;;
  esac
  [[ "${requested}" != false ]] || return
  if ! command -v taskset >/dev/null 2>&1; then
    [[ "${requested}" == auto ]] && {
      log_warn "CPU 亲和性已禁用：taskset 不可用。"
      return
    }
    log_error "CPU_AFFINITY_ENABLED=true 需要 taskset。"
    exit 1
  fi

  cpu_count="$(nproc)"
  performance_max_mhz="$(cat /sys/devices/system/cpu/cpu0/cpufreq/cpuinfo_max_freq 2>/dev/null || true)"
  efficiency_max_mhz="$(cat /sys/devices/system/cpu/cpu8/cpufreq/cpuinfo_max_freq 2>/dev/null || true)"
  if [[ "${requested}" == auto ]] && {
    [[ "${cpu_count}" != 16 ]] ||
    [[ ! "${performance_max_mhz}" =~ ^[0-9]+$ ]] ||
    [[ ! "${efficiency_max_mhz}" =~ ^[0-9]+$ ]] ||
    ((performance_max_mhz <= efficiency_max_mhz))
  }; then
    log_info "CPU 亲和性自动检测未发现 i5-1340P P/E 核拓扑，不调整调度。"
    return
  fi

  for cpu_set in \
    "${cpu_set_mid360}" "${cpu_set_frlio}" "${cpu_set_mavros_ev}" \
    "${cpu_set_navigation}" "${cpu_set_relocalization_bridge}" \
    "${cpu_set_relocalization_backend}" "${cpu_set_rosbag}"; do
    if ! taskset -c "${cpu_set}" true >/dev/null 2>&1; then
      log_error "CPU 集合无效或不可用：${cpu_set}"
      exit 1
    fi
  done
  cpu_affinity_active=true
  log_info "已启用 CPU 亲和性：MID-360=${cpu_set_mid360}，FR-LIO=${cpu_set_frlio}，MAVROS/EV=${cpu_set_mavros_ev}，导航=${cpu_set_navigation}，后台任务=${cpu_set_rosbag}。"
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
    log_error "无法从 TUNING_FILE 读取 EGO 启用开关：${tuning_file}"
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
  log_error "组件“${key}”未就绪。日志：${log_file}"
  if [[ -f "${log_file}" ]]; then
    log_error "组件“${key}”最后 30 行日志："
    tail -n 30 "${log_file}" >&2 || true
  fi
}

terminate_components() {
  local index key pgid launcher_pid
  for ((index=${#started_components[@]} - 1; index >= 0; index--)); do
    key="${started_components[index]}"
    pgid="${component_pgids[${key}]}"
    if valid_component_group "${pgid}"; then
      # 先给整个会话 SIGINT：rosbag2 需要它正常写 metadata.yaml，ROS 节点也
      # 能在自己的退出路径中释放订阅/串口。进程组原组长已经退出时仍照常发送。
      kill -INT -- "-${pgid}" 2>/dev/null || true
    fi
  done

  for _ in 1 2 3 4 5 6 7 8 9 10; do
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

  # 个别 launch/驱动节点不处理 SIGINT，或其包装器拦截了信号；此时再升级到
  # SIGTERM，最后才使用 SIGKILL，避免正常录包被硬杀。
  for ((index=${#started_components[@]} - 1; index >= 0; index--)); do
    key="${started_components[index]}"
    pgid="${component_pgids[${key}]}"
    if component_running "${key}" && valid_component_group "${pgid}"; then
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
    log_warn "正在停止本次运行启动的组件..." || true
    terminate_components
  fi
  exit "${status}"
}

on_signal() {
  local signal="$1"
  set +e
  log_warn "收到 ${signal} 信号；正在关闭完整运行栈。" || true
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
  local launcher_pid pid pgid cpu_set=""

  if [[ "${cpu_affinity_active}" == true ]] && cpu_set="$(component_cpu_set "${key}" 2>/dev/null || true)" && [[ -n "${cpu_set}" ]]; then
    log_info "${title} 使用的 CPU 集合：${cpu_set}"
    set -- taskset -c "${cpu_set}" "$@"
  fi

  log_info "正在独立终端窗口启动 ${title}；日志=${log_file}"
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
      log_error "组件窗口“${key}”在发布运行状态前退出"
      wait "${launcher_pid}" 2>/dev/null || true
      return 1
    fi
    sleep 0.1
  done
  if [[ ! -s "${runtime_file}" ]]; then
    log_error "等待组件运行状态超时：${runtime_file}"
    kill -TERM "${launcher_pid}" 2>/dev/null || true
    wait "${launcher_pid}" 2>/dev/null || true
    return 1
  fi

  pid="$(awk -F= '$1 == "pid" {print $2}' "${runtime_file}")"
  pgid="$(awk -F= '$1 == "pgid" {print $2}' "${runtime_file}")"
  if [[ ! "${pid}" =~ ^[0-9]+$ ]] || ! valid_component_group "${pgid}"; then
    log_error "组件 ${key} 的运行状态无效：进程号=${pid:-缺失}，进程组号=${pgid:-缺失}"
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

wait_service_ready() {
  local key="$1"
  local service="$2"
  local deadline=$((SECONDS + readiness_timeout_sec))
  while ((SECONDS < deadline)); do
    component_running "${key}" || { show_component_failure "${key}"; return 1; }
    if timeout 5 ros2 service list 2>/dev/null | grep -Fxq "${service}"; then
      log_info "服务已就绪：${service}"
      return 0
    fi
    sleep 1
  done
  log_error "等待服务超时：${service}"
  show_component_failure "${key}"
  return 1
}

run_relocalization() {
  local output
  start_component "重定位坐标桥" "relocalization_bridge" \
    python3 "${relocalization_bridge}" --ros-args \
    -p odom_topic:=/Odometry \
    -p output_odom_topic:=/planning/odom \
    -p map_frame:=map \
    -p odom_frame:=odom \
    -p body_frame:=body \
    -p publish_odom_body_tf:=false

  start_component "FAST-LIO 全局重定位" "relocalization_backend" \
    ros2 launch fastlio_global_slam fastlio_global_slam.launch.py \
    start_fastlio:=false rviz:=false \
    "backend_config:=${relocalization_backend_config}"
  wait_service_ready "relocalization_backend" /fastlio_global_backend/load_map
  wait_service_ready "relocalization_backend" /fastlio_global_backend/relocalize

  log_info "正在从 ${global_map_dir} 加载重定位关键帧"
  output="$(timeout "${readiness_timeout_sec}s" ros2 service call \
    /fastlio_global_backend/load_map fastlio_global_slam/srv/LoadMap \
    "{directory: '${global_map_dir}'}" 2>&1)" || {
      log_error "调用重定位地图服务失败。"
      return 1
    }
  grep -Eq 'success[=:][[:space:]]*(true|True)' <<<"${output}" || {
    log_error "重定位地图未被服务接受。"
    return 1
  }

  log_info "等待 ${relocalization_fresh_scan_delay_sec} 秒，以获取新的同步扫描数据"
  sleep "${relocalization_fresh_scan_delay_sec}"
  wait_component_ready "fastlio" message /cloud_registered
  local attempt relocalization_ok=false
  if [[ "${relocalization_interactive}" == true ]]; then
    [[ -r "${relocalization_trigger_tty}" ]] || {
      log_error "交互重定位需要可读终端：${relocalization_trigger_tty}"
      return 1
    }
    log_info "交互重定位已启用；每次按回车只尝试一次，成功前不设次数上限。"
    while true; do
      if ! IFS= read -r -p "请保持设备在已建图区域，按回车进行一次重定位（Ctrl-C 退出）: " _ <"${relocalization_trigger_tty}"; then
        log_error "无法从终端读取重定位触发按键。"
        return 1
      fi
      log_info "正在尝试重定位（本次按键）；请保持飞行器静止并处于已建图区域内"
      output="$(timeout "${relocalization_call_timeout_sec}s" ros2 service call \
        /fastlio_global_backend/relocalize fastlio_global_slam/srv/Relocalize \
        '{use_latest_scan: true}' 2>&1)" || true
      if grep -Eq 'success[=:][[:space:]]*(true|True)' <<<"${output}"; then
        log_info "重定位成功。"
        relocalization_ok=true
        break
      fi
      log_warn "本次重定位未找到可信候选位姿；准备新的扫描后再次按回车重试"
    done
  else
    for ((attempt=1; attempt<=relocalization_retry_count; attempt++)); do
      log_info "正在尝试重定位（${attempt}/${relocalization_retry_count}）；请保持飞行器静止并处于已建图区域内"
      output="$(timeout "${relocalization_call_timeout_sec}s" ros2 service call \
        /fastlio_global_backend/relocalize fastlio_global_slam/srv/Relocalize \
        '{use_latest_scan: true}' 2>&1)" || true
      if grep -Eq 'success[=:][[:space:]]*(true|True)' <<<"${output}"; then
        log_info "重定位成功。"
        relocalization_ok=true
        break
      fi
      if ((attempt < relocalization_retry_count)); then
        log_warn "重定位未找到可信候选位姿；等待 ${relocalization_retry_delay_sec} 秒后使用新的同步扫描重试"
        sleep "${relocalization_retry_delay_sec}"
      fi
    done
  fi
  if [[ "${relocalization_ok}" != true ]]; then
    log_error "重定位未成功；不会启动 MAVROS 和控制节点。"
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
      validate_frlio_livox_qos_build
      ;;
    *)
      log_error "LIO_BACKEND 必须为 fast_lio 或 fr_lio；当前值：${lio_backend}"
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
  require_boolean GLOBAL_WAYPOINT_TASK_ENABLED "${global_waypoint_task_enabled}"
  require_boolean PUBLISH_GLOBAL_PATH "${publish_global_path}"
  require_boolean PX4_COMPONENTS_ENABLED "${px4_components_enabled}"
  require_boolean WAYPOINT_VISUALIZER_ENABLED "${waypoint_visualizer_enabled}"
  require_boolean WAYPOINT_FSM_ENABLED "${waypoint_fsm_enabled}"
  require_boolean NAVIGATION_ENABLED "${navigation_enabled}"
  require_boolean RECOGNITION_ENABLED "${recognition_enabled}"
  require_boolean RVIZ "${rviz}"
  require_boolean ENABLE_OUTPUT "${enable_output}"
  require_boolean MANUAL_HANDOVER "${manual_handover}"
  require_boolean EV_FAULT_AUTO_LAND "${ev_fault_auto_land}"
  require_boolean RECORD_BAG "${record_bag}"
  require_boolean REQUIRE_FLIGHT_READY_FOR_STARTUP "${require_flight_ready_for_startup}"
  require_rosbag_profile "${rosbag_profile}"
  require_boolean NO_MAP_VALIDATION "${no_map_validation}"
  require_boolean MAP_AUTO_LOAD "${map_auto_load}"
  require_boolean COMPONENT_WINDOWS "${component_windows}"
  require_boolean RELOCALIZATION_ENABLED "${relocalization_enabled}"
  require_boolean REQUIRE_MAP_LOCAL_ALIGNMENT "${require_map_local_alignment}"
  require_boolean RELOCALIZATION_INTERACTIVE "${relocalization_interactive}"
  if [[ "${require_flight_ready_for_startup}" == false && "${enable_output}" != false ]]; then
    log_error "REQUIRE_FLIGHT_READY_FOR_STARTUP=false 仅支持 ENABLE_OUTPUT=false"
    exit 1
  fi
  if [[ "${px4_components_enabled}" == false ]]; then
    if [[ "${enable_output}" != false || "${mission_enabled}" != false ]]; then
      log_error "PX4_COMPONENTS_ENABLED=false 要求 ENABLE_OUTPUT=false 且 MISSION_ENABLED=false"
      exit 1
    fi
  fi
  if [[ "${global_waypoint_task_enabled}" == true && "${mission_enabled}" == true ]]; then
    log_error "GLOBAL_WAYPOINT_TASK_ENABLED=true 不能与 MISSION_ENABLED=true 同时使用"
    exit 1
  fi
  if [[ "${waypoint_visualizer_enabled}" == true ]]; then
    require_file "${waypoints_file}"
  fi
  configure_cpu_affinity

  if [[ "${relocalization_enabled}" == true ]]; then
    require_file "${relocalization_backend_config}"
    require_file "${relocalization_bridge}"
    [[ -d "${global_map_dir}/keyframes" ]] || {
      log_error "缺少重定位关键帧目录：${global_map_dir}/keyframes"
      exit 1
    }
    [[ -s "${global_map_dir}/metadata.csv" ]] || {
      log_error "缺少重定位元数据：${global_map_dir}/metadata.csv"
      exit 1
    }
    if awk -v value="${world_yaw_alignment_rad}" 'BEGIN { exit !(value != 0.0) }'; then
      log_error "重定位已定义地图对齐关系；WORLD_YAW_ALIGNMENT_RAD 必须为 0.0"
      exit 1
    fi
    if [[ "${relocalization_interactive}" != true && ! "${relocalization_retry_count}" =~ ^[1-9][0-9]*$ ]]; then
      log_error "RELOCALIZATION_RETRY_COUNT 必须为正整数（非交互模式）"
      exit 1
    fi
    if ! awk -v value="${relocalization_retry_delay_sec}" 'BEGIN {exit !(value ~ /^[0-9]+([.][0-9]*)?$/)}'; then
      log_error "RELOCALIZATION_RETRY_DELAY_SEC 必须为非负数"
      exit 1
    fi
  fi

  if [[ "${component_windows}" == true ]]; then
    if ! command -v gnome-terminal >/dev/null 2>&1; then
      log_error "COMPONENT_WINDOWS=true 需要 gnome-terminal"
      exit 1
    fi
    if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
      log_error "COMPONENT_WINDOWS=true 需要图形桌面会话"
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
      log_error "PLANNER_BACKEND 必须为 astar_ego、super、ego 或 ego-shadow；当前值：${planner_backend}"
      exit 1
      ;;
  esac
  resolve_effective_planner_backend
  if [[ ! "${readiness_timeout_sec}" =~ ^[0-9]+$ ]] || ((readiness_timeout_sec < 1)); then
    log_error "READINESS_TIMEOUT_SEC 必须为正整数"
    exit 1
  fi
  if [[ ! "${auto_stop_after_ready_sec}" =~ ^[0-9]+$ ]]; then
    log_error "AUTO_STOP_AFTER_READY_SEC 必须为非负整数"
    exit 1
  fi
  if [[ ! "${mid360_fastlio_delay_sec}" =~ ^([0-9]+([.][0-9]*)?|[.][0-9]+)$ ]]; then
    log_error "MID360_FASTLIO_DELAY_SEC 必须为非负数"
    exit 1
  fi
  if [[ "${skip_preflight_check}" != 0 && "${skip_preflight_check}" != 1 ]]; then
    log_error "SKIP_PREFLIGHT_CHECK 必须为 0 或 1"
    exit 1
  fi
  if [[ -z "${fcu_url}" ]]; then
    log_error "FCU_URL 不能为空"
    exit 1
  fi
  if [[ ! "${world_yaw_alignment_rad}" =~ ^[-+]?([0-9]+([.][0-9]*)?|[.][0-9]+)([eE][-+]?[0-9]+)?$ ]]; then
    log_error "WORLD_YAW_ALIGNMENT_RAD 必须为数字"
    exit 1
  fi
  case "${allow_unvalidated_world_yaw}" in
    true|false) ;;
    *)
      log_error "ALLOW_UNVALIDATED_WORLD_YAW 必须为 true 或 false"
      exit 1
      ;;
  esac
  if awk -v value="${world_yaw_alignment_rad}" 'BEGIN { exit !(value != 0.0) }'; then
    if [[ "${allow_unvalidated_world_yaw}" != true ]]; then
      log_error "WORLD_YAW_ALIGNMENT_RAD=${world_yaw_alignment_rad} 未经验证；正常飞行必须为 0.0。请取消旧环境变量。"
      exit 1
    fi
    if [[ "${mission_enabled}" != false || "${enable_output}" != false ]]; then
      log_error "未经验证的世界 yaw 仅允许在 MISSION_ENABLED=false 且 ENABLE_OUTPUT=false 时使用。"
      exit 1
    fi
    log_warn "正在通过显式实验覆盖使用未经验证的 WORLD_YAW_ALIGNMENT_RAD=${world_yaw_alignment_rad}。仅允许无桨运行。"
  fi
  if [[ -z "${map_file}" ]]; then
    if [[ "${map_auto_load}" == true ]]; then
      log_error "MAP_FILE 为空但 MAP_AUTO_LOAD=true；请提供 PCD，或将 MAP_AUTO_LOAD=false 用于验证"
      exit 1
    fi
    if [[ "${mission_enabled}" == true || "${enable_output}" == true ]]; then
      log_error "无地图验证要求 MISSION_ENABLED=false 且 ENABLE_OUTPUT=false"
      exit 1
    fi
  elif [[ ! -f "${map_file}" ]]; then
    log_error "MAP_FILE 不存在：${map_file}"
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
    log_error "px4_ros_com 被旧工作区覆盖：${resolved_px4_share}"
    log_error "应使用当前工作区：${expected_px4_share}"
    exit 1
  fi
  if grep -Eq 'DeclareLaunchArgument\("start_(bridge|px4_ev_bridge)' "${autofix_launch}" ||
     grep -Eq 'Only one external-vision output may feed PX4' "${autofix_launch}"; then
    log_error "fastlio_mavros_autofix.launch.py 包含旧的多路 EV 路径，拒绝启动。"
    exit 1
  fi
  cp "${lever_arm_config}" "${flight_run_dir}/mid360_lever_arm.conf"

  if [[ "${skip_preflight_check}" == 0 ]]; then
    log_info "正在执行标准起飞前检查..."
    "${preflight_check}"
  else
    log_warn "SKIP_PREFLIGHT_CHECK=1：已明确跳过标准起飞前检查。"
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
  log_info "本次飞行记录目录：${flight_run_dir}"
  log_info "导航=${navigation_enabled}；请求规划器=${planner_backend}，实际规划器=${effective_planner_backend}，EGO启用=${ego_enabled}，任务=${mission_enabled}，控制输出=${enable_output}，人工交接=${manual_handover}，RViz=${rviz}"
  log_info "全局航点任务=${global_waypoint_task_enabled}，全局路径话题=${publish_global_path}，PX4/MAVROS/控制=${px4_components_enabled}"
  log_info "识别=${recognition_enabled}，rosbag=${record_bag}，录制模式=${rosbag_profile}，就绪超时=${readiness_timeout_sec} 秒"
  log_info "LIO 后端=${lio_backend}；MID-360 到 LIO 的最小启动延迟=${mid360_fastlio_delay_sec} 秒"
  if [[ "${lio_backend}" == fr_lio ]]; then
    log_warn "FR-LIO 高速输出仅用于诊断，禁止替代 PX4 外部视觉链路。"
  fi
  log_info "组件独立窗口=${component_windows}（每个组件一个 GNOME Terminal 窗口）"
  log_info "地图：${map_file:-<验证模式下禁用>}（自动加载=${map_auto_load}）"
  log_info "全局重定位=${relocalization_enabled}，地图本地对齐门禁=${require_map_local_alignment}，关键帧目录=${global_map_dir}"
  if [[ "${map_file}" == "${default_map_file}" ]]; then
    log_warn "随附默认地图尚未确认是场地扫描结果。现场运行前请将 MAP_FILE 设为实测场地 PCD。"
  fi
  if [[ "${no_map_validation}" == true ]]; then
    log_warn "NO_MAP_VALIDATION=true：已禁用规划输出和任务控制；本次仅验证传感器、EV 和通信链路。"
  fi
  log_info "MID360 机体到传感器 FLU：x=${MID360_BODY_TO_SENSOR_X_M} 米，y=${MID360_BODY_TO_SENSOR_Y_M} 米，z=${MID360_BODY_TO_SENSOR_Z_M} 米，yaw=${MID360_BODY_TO_FASTLIO_YAW_RAD} 弧度"
  log_warn "当前未提供水平漂移与起飞瞬态保护，请始终保持遥控器可人工接管。"
}

start_stack() {
  if [[ "${navigation_enabled}" == true ]]; then
    # Navigation must consume the same fail-closed stream that feeds PX4.
    # In relocalization mode the health monitor still reads /planning/odom,
    # but the planner must never follow that unguarded intermediate topic.
    start_component "导航栈（预热）" "navigation" \
      env MAP_FILE="${map_file}" \
      PLANNER_BACKEND="${effective_planner_backend}" \
      TUNING_FILE="${tuning_file}" \
      MISSION_ENABLED="${mission_enabled}" \
      GLOBAL_WAYPOINT_TASK_ENABLED="${global_waypoint_task_enabled}" \
      PUBLISH_GLOBAL_PATH="${publish_global_path}" \
      CONTROL_NODES_ENABLED="${px4_components_enabled}" \
      WAYPOINT_VISUALIZER_ENABLED="${waypoint_visualizer_enabled}" \
      WAYPOINT_FSM_ENABLED="${waypoint_fsm_enabled}" \
      WAYPOINTS_FILE="${waypoints_file}" \
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
      FASTLIO_ODOM_TOPIC=/Odometry/healthy \
      REQUIRE_MAP_LOCAL_ALIGNMENT="${require_map_local_alignment}" \
      MAP_AUTO_LOAD="${map_auto_load}" \
      "${navigation_script}"
    if [[ "${px4_components_enabled}" == true ]]; then
      wait_component_ready "navigation" message /race/control/status
    else
      wait_component_ready "navigation" node /super_planner_ros2_node
    fi
    if [[ -n "${map_file}" && "${map_auto_load}" == true ]]; then
      wait_component_ready "navigation" topic /saved_map
    else
      log_warn "未加载地图；跳过 /saved_map 就绪检查（仅限验证模式）。"
    fi
    if [[ "${rviz}" == true ]]; then
      wait_component_ready "navigation" node /rviz2
    fi
  else
    log_info "NAVIGATION_ENABLED=false：跳过导航及全部具备 Offboard 控制能力的组件。"
  fi

  start_component "MID-360 驱动" "mid360_driver" \
    "${livox_env}/run_mid360_driver.sh"
  log_info "MID-360 启动后等待 ${mid360_fastlio_delay_sec} 秒，再启动 ${lio_backend}..."
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
    log_info "重定位导航链：/planning/odom -> EV 健康门控 -> /Odometry/healthy -> /race/odom"
  fi

  if [[ "${px4_components_enabled}" == true ]]; then
    start_component "MAVROS 与 EV 链路" "px4_mavros" \
    ros2 launch px4_ros_com fastlio_mavros_autofix.launch.py \
    "fcu_url:=${fcu_url}" \
    start_mavros_vision_bridge:=true \
    start_odom_guard:=true \
    start_ev_health_monitor:=true \
    start_tf:=true \
    "world_yaw_alignment_rad:=${world_yaw_alignment_rad}" \
    "fastlio_odom_topic:=${ev_odom_topic}" \
    "require_frlio_anchor_status:=$([[ "${lio_backend}" == fr_lio ]] && echo true || echo false)" \
    "body_to_sensor_x_m:=${MID360_BODY_TO_SENSOR_X_M}" \
    "body_to_sensor_y_m:=${MID360_BODY_TO_SENSOR_Y_M}" \
    "body_to_sensor_z_m:=${MID360_BODY_TO_SENSOR_Z_M}" \
    "body_to_fastlio_yaw_rad:=${MID360_BODY_TO_FASTLIO_YAW_RAD}"
  wait_component_ready "px4_mavros" mavros_connected /mavros/state
  wait_component_ready "px4_mavros" message /mavros/local_position/odom
  # Some PX4/MAVROS firmware combinations publish local pose/odom but do not
  # emit LOCAL_POSITION_NED_COV, and therefore never produce velocity_local.
  # The EV monitor falls back to the twist in local_position/odom in that case;
  # do not block the entire flight stack on this optional MAVLink stream.
  if ! timeout 5 ros2 topic echo --once --qos-reliability best_effort \
    /mavros/local_position/velocity_local >/dev/null 2>&1; then
    log_warn "未收到 /mavros/local_position/velocity_local 消息；改用 /mavros/local_position/odom 的速度数据。"
  fi
  if [[ "${require_flight_ready_for_startup}" == true ]]; then
    wait_component_ready "px4_mavros" flight_ready /ev_health/flight_ready
  else
    log_warn "控制输出已禁用，跳过 /ev_health/flight_ready 就绪门禁。"
  fi
  wait_component_ready "px4_mavros" message /Odometry/healthy
  wait_component_ready "px4_mavros" message /mavros/vision_pose/pose_cov
  wait_component_ready "px4_mavros" message /mavros/vision_speed/speed_twist_cov

  # The bridge contract is exactly one publisher into MAVROS vision_pose and
  # no publisher on the retired direct-PX4 EV topics.
  local pose_info direct_info
  pose_info="$(ros2 topic info -v /mavros/vision_pose/pose_cov 2>/dev/null || true)"
  if ! grep -Eq 'Publisher count:[[:space:]]+1$' <<<"${pose_info}"; then
    log_error "EV 桥接契约失败：/mavros/vision_pose/pose_cov 必须且只能有一个发布者。"
    printf '%s\n' "${pose_info}" >&2
    return 1
  fi
  for direct_topic in /mavros/odometry/out /fmu/in/vehicle_visual_odometry; do
    direct_info="$(ros2 topic info -v "${direct_topic}" 2>/dev/null || true)"
    if grep -Eq 'Publisher count:[[:space:]]+[1-9]' <<<"${direct_info}"; then
      log_error "EV 桥接契约失败：已停用的输入 ${direct_topic} 仍有发布者。"
      printf '%s\n' "${direct_info}" >&2
      return 1
    fi
  done

  if [[ "${relocalization_enabled}" == true && "${require_map_local_alignment}" == true && "${enable_output}" == true ]]; then
    log_info "等待 Offboard 锁定地图到 PX4 本地坐标系的变换"
    local alignment_deadline=$((SECONDS + 30))
    local alignment_status=""
    while ((SECONDS < alignment_deadline)); do
      alignment_status="$(timeout 5 ros2 topic echo --once /race/control/status --field data 2>/dev/null || true)"
      grep -q 'map_local_alignment=1' <<<"${alignment_status}" && break
      sleep 1
    done
    grep -q 'map_local_alignment=1' <<<"${alignment_status}" || {
    log_error "Offboard 未锁定地图到 PX4 本地坐标系的对齐；飞行继续被阻止。"
      return 1
    }
  fi
  else
    log_info "PX4_COMPONENTS_ENABLED=false：跳过 MAVROS、EV 链路和全部 PX4 就绪检查。"
  fi

  if [[ "${record_bag}" == true ]]; then
    start_component "调试 rosbag" "rosbag_debug" \
      env FLIGHT_TIMESTAMP="${flight_timestamp}" FLIGHT_RUN_DIR="${flight_run_dir}" ROSBAG_PROFILE="${rosbag_profile}" \
      "${bag_script}"
    wait_component_ready "rosbag_debug" node /rosbag2_recorder
    if [[ "${px4_components_enabled}" == true ]]; then
      wait_component_ready "px4_mavros" flight_ready /ev_health/flight_ready
      wait_component_ready "px4_mavros" message /Odometry/healthy
      wait_component_ready "px4_mavros" message /mavros/vision_pose/pose_cov
    fi
  else
    log_warn "RECORD_BAG=false：本次运行未启用 rosbag 录制。"
  fi

  if [[ "${navigation_enabled}" == true ]]; then
    wait_component_ready "navigation" message /race/odom
  fi
  if [[ "${px4_components_enabled}" == true ]]; then
    if [[ "${require_flight_ready_for_startup}" == true ]]; then
      wait_component_ready "px4_mavros" flight_ready /ev_health/flight_ready
    fi
    wait_component_ready "px4_mavros" message /mavros/vision_pose/pose_cov
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

  if [[ "${lio_backend}" == fr_lio ]]; then
    validate_frlio_runtime_overlay
  fi

  prepare_run
  print_configuration
  start_stack

  log_info "通过：完整链路已就绪：MID-360 -> ${lio_backend} -> 重定位(${relocalization_enabled}) -> PX4/MAVROS(${px4_components_enabled}) -> rosbag(${record_bag}) -> 导航。"
  printf 'state=ready\nflight_run_dir=%s\n' "${flight_run_dir}" >"${stack_ready_file}.tmp.$$"
  mv -f "${stack_ready_file}.tmp.$$" "${stack_ready_file}"
  log_info "组件日志：${flight_log_dir}"
  log_info "本终端现正监控运行栈。按 Ctrl+C 将停止本次运行启动的全部组件。"
  if ((auto_stop_after_ready_sec > 0)); then
    log_info "自动停止验证：将在 ${auto_stop_after_ready_sec} 秒后关闭。"
    sleep "${auto_stop_after_ready_sec}"
    return 0
  fi
  monitor_stack
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  main "$@"
fi
