#!/usr/bin/env bash
set -Eeuo pipefail

# Propeller-off Phase 4 recorder for /Odometry/propagated.
# This script never starts MAVROS, PX4, an Offboard node, or a control publisher.
# It does not modify FAST-LIO source or vehicle parameters.

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
session_stamp="${PHASE4_TIMESTAMP:-$(date +%Y%m%d_%H%M%S)}"
session_root="${PHASE4_OUTPUT_DIR:-${script_dir}/phase4_operator_records/phase4_${session_stamp}}"
checkpoint_dir="${PHASE35_CHECKPOINT:-${script_dir}/phase_checkpoints/phase35_signoff_20260724_152912}"
fastlio_ws="${FASTLIO_WS:-/home/robot/livox_mid360_env/ws_fastlio}"
livox_ws="${LIVOX_WS:-/home/robot/livox_mid360_env/ws_livox}"
mid360_env_root="$(dirname "${livox_ws}")"
mid360_setup="${MID360_SETUP:-${mid360_env_root}/setup_mid360.bash}"
fastlio_src="${fastlio_ws}/src/fast_lio"
fastlio_build="${fastlio_ws}/build/fast_lio"
livox_config="${livox_ws}/src/livox_ros_driver2/config/MID360_config.json"
pidstat_bin="${PIDSTAT_BIN:-/home/robot/.local/bin/pidstat}"
gate_program="${script_dir}/phase4_topic_gate.py"

export ROS_DOMAIN_ID="${PHASE4_ROS_DOMAIN_ID:-76}"
export ROS_LOCALHOST_ONLY=1
export RCUTILS_COLORIZED_OUTPUT=0

baseline_seconds="${PHASE4_BASELINE_SECONDS:-60}"
static_seconds="${PHASE4_STATIC_SECONDS:-60}"
final_static_seconds="${PHASE4_FINAL_STATIC_SECONDS:-10}"
fault_observe_seconds="${PHASE4_FAULT_SECONDS:-3}"
recovery_observe_seconds="${PHASE4_RECOVERY_SECONDS:-6}"
cycle_observe_seconds="${PHASE4_CYCLE_SECONDS:-5}"

evidence_dir="${session_root}/evidence"
log_dir="${session_root}/logs"
metrics_dir="${session_root}/metrics"
bag_root="${session_root}/bags"
if [[ -e "${session_root}" ]]; then
  printf 'REFUSED: output directory already exists: %s\n' "${session_root}" >&2
  exit 2
fi
mkdir -p "${evidence_dir}/preflight" "${log_dir}" "${metrics_dir}" "${bag_root}"

set +u
if [[ -f "${mid360_setup}" ]]; then
  # shellcheck disable=SC1090
  source "${mid360_setup}"
else
  source /opt/ros/humble/setup.bash
  source "${livox_ws}/install/setup.bash"
  export LD_LIBRARY_PATH="${mid360_env_root}/local/lib:${LD_LIBRARY_PATH:-}"
fi
set +u
source "${fastlio_ws}/install/setup.bash"
if [[ -f "${script_dir}/install/setup.bash" ]]; then
  source "${script_dir}/install/setup.bash"
fi
set -u

declare -a managed_pids=()
declare -a managed_names=()
current_bag_pid=""
current_fastlio_pid=""
current_livox_pid=""
current_imu_gate_pid=""
current_lidar_gate_pid=""
current_pidstat_pid=""
current_rss_pid=""
last_started_pid=""
cleanup_started=0

timestamp() {
  date --iso-8601=ns
}

log_event() {
  printf '%s %s\n' "$(timestamp)" "$*" | tee -a "${evidence_dir}/events.txt"
}

fail() {
  log_event "FAIL $*"
  printf 'FAILED: %s\nEvidence: %s\n' "$*" "${session_root}" >&2
  exit 1
}

register_pid() {
  managed_pids+=("$1")
  managed_names+=("$2")
}

start_group() {
  local name="$1"
  local logfile="$2"
  shift 2
  setsid "$@" >"${logfile}" 2>&1 &
  local pid=$!
  register_pid "${pid}" "${name}"
  last_started_pid="${pid}"
}

stop_group() {
  local pid="${1:-}"
  local name="${2:-process}"
  local pgid=""
  local index
  [[ -n "${pid}" ]] || return 0
  pgid="$(ps -o pgid= -p "${pid}" 2>/dev/null | tr -d '[:space:]')"
  if kill -0 "${pid}" 2>/dev/null; then
    if [[ "${pgid}" == "${pid}" ]]; then
      kill -INT -- "-${pid}" 2>/dev/null || true
    else
      kill -INT "${pid}" 2>/dev/null || true
    fi
    for _index in $(seq 1 40); do
      kill -0 "${pid}" 2>/dev/null || break
      sleep 0.1
    done
  fi
  if kill -0 "${pid}" 2>/dev/null; then
    if [[ "${pgid}" == "${pid}" ]]; then
      kill -TERM -- "-${pid}" 2>/dev/null || true
    else
      kill -TERM "${pid}" 2>/dev/null || true
    fi
    for _index in $(seq 1 20); do
      kill -0 "${pid}" 2>/dev/null || break
      sleep 0.1
    done
  fi
  if kill -0 "${pid}" 2>/dev/null; then
    if [[ "${pgid}" == "${pid}" ]]; then
      kill -KILL -- "-${pid}" 2>/dev/null || true
    else
      kill -KILL "${pid}" 2>/dev/null || true
    fi
    log_event "FORCED_KILL ${name} pid=${pid}"
  fi
  wait "${pid}" 2>/dev/null || true
  for index in "${!managed_pids[@]}"; do
    if [[ "${managed_pids[index]}" == "${pid}" ]]; then
      managed_pids[index]=""
    fi
  done
}

cleanup() {
  local exit_code=$?
  (( cleanup_started == 0 )) || return
  cleanup_started=1
  trap - EXIT INT TERM
  set +e
  log_event "CLEANUP_BEGIN exit_code=${exit_code}"
  for ((index=${#managed_pids[@]} - 1; index >= 0; index--)); do
    [[ -n "${managed_pids[index]}" ]] &&
      stop_group "${managed_pids[index]}" "${managed_names[index]}"
  done
  ROS_DOMAIN_ID="${ROS_DOMAIN_ID}" ros2 daemon stop >/dev/null 2>&1 || true
  ps -eo pid=,ppid=,pgid=,comm=,args= >"${evidence_dir}/processes_after.txt"
  verify_live_checkpoint >"${evidence_dir}/checkpoint_after.txt" 2>&1 || true
  if find "${bag_root}" -mindepth 2 -maxdepth 2 -name metadata.yaml -print -quit | grep -q .; then
    while IFS= read -r metadata; do
      bag_dir="$(dirname "${metadata}")"
      safe_name="$(basename "${bag_dir}")"
      ros2 bag info "${bag_dir}" >"${evidence_dir}/bag_info_${safe_name}.txt" 2>&1 || true
    done < <(find "${bag_root}" -mindepth 2 -maxdepth 2 -name metadata.yaml -print | sort)
  fi
  {
    printf '# Phase 4 operator collection\n\n'
    printf -- '- Session: `%s`\n' "${session_root}"
    printf -- '- Exit code: `%s`\n' "${exit_code}"
    printf -- '- ROS_DOMAIN_ID: `%s`\n' "${ROS_DOMAIN_ID}"
    printf -- '- MAVROS/PX4/OFFBOARD started by script: `NO`\n'
    printf -- '- FAST-LIO source modified by script: `NO`\n'
    printf -- '- EKF2_EV_CTRL changed by script: `NO`\n'
    printf -- '- Independent analysis/signoff: `PENDING`\n'
  } >"${session_root}/COLLECTION_STATUS.md"
  log_event "CLEANUP_END"
  find "${session_root}" -type f ! -name EVIDENCE.sha256 -print0 |
    sort -z | xargs -0 sha256sum >"${session_root}/EVIDENCE.sha256" 2>/dev/null || true
  printf '\n数据采集目录：%s\n' "${session_root}"
  printf 'rosbag 根目录：%s\n' "${bag_root}"
  printf '请不要修改该目录；将完整目录交回进行独立分析和 Phase 4 审签。\n'
  exit "${exit_code}"
}
trap cleanup EXIT INT TERM

forbidden_process_audit() {
  local output="$1"
  ps -eo pid=,comm=,args= |
    awk '
      tolower($2) ~ /^(mavros|mavros_node|px4|px4_ros|micrortps|microxrceagent|offboard_control|minipc_mavros|minipc_offboard)/ {
        print
      }
    ' >"${output}"
  [[ ! -s "${output}" ]]
}

residual_runtime_audit() {
  local output="$1"
  ps -eo pid=,comm=,args= |
    awk '
      tolower($2) ~ /^(fastlio_mapping|livox_ros_driv|ros2)$/ &&
      tolower($0) ~ /(fastlio_mapping|livox_ros_driver2_node|ros2 bag record)/ {
        print
      }
    ' >"${output}"
  [[ ! -s "${output}" ]]
}

verify_live_checkpoint() {
  [[ -d "${checkpoint_dir}/files" ]] || return 1
  (
    cd "${checkpoint_dir}"
    sha256sum -c CHECKPOINT.sha256
    sha256sum -c MANIFEST.sha256
  )

  local frozen live relative
  while IFS= read -r frozen; do
    relative="${frozen#"${checkpoint_dir}"/files/}"
    case "${relative}" in
      HIGH_RATE_EV_ODOM_DESIGN.md)
        live="${script_dir}/HIGH_RATE_EV_ODOM_DESIGN.md"
        ;;
      build/LastTest.log)
        live="${fastlio_build}/Testing/Temporary/LastTest.log"
        ;;
      build/*)
        live="${fastlio_build}/${relative#build/}"
        ;;
      *)
        live="${fastlio_src}/${relative}"
        ;;
    esac
    [[ -f "${live}" ]] || {
      printf 'MISSING %s\n' "${live}"
      return 1
    }
    frozen_hash="$(sha256sum "${frozen}" | awk '{print $1}')"
    live_hash="$(sha256sum "${live}" | awk '{print $1}')"
    printf '%s  %s  %s\n' "${frozen_hash}" "${live_hash}" "${live}"
    [[ "${frozen_hash}" == "${live_hash}" ]] || return 1
  done < <(find "${checkpoint_dir}/files" -type f | sort)
}

wait_for_topic() {
  local topic="$1"
  local timeout_s="${2:-30}"
  local end=$((SECONDS + timeout_s))
  while (( SECONDS < end )); do
    ros2 topic list 2>/dev/null | grep -Fxq "${topic}" && return 0
    sleep 0.25
  done
  return 1
}

assert_topic_absent() {
  local topic="$1"
  if ros2 topic list 2>/dev/null | grep -Fxq "${topic}"; then
    fail "默认关闭时发现禁止话题 ${topic}"
  fi
}

diag_value_from_file() {
  local key="$1"
  local file="$2"
  awk -v wanted="${key}" '
    $0 ~ "key: " wanted "$" {
      if (getline > 0) {
        sub(/^[[:space:]]*value:[[:space:]]*/, "")
        gsub(/^["'\'']|["'\'']$/, "")
        print
        exit
      }
    }
  ' "${file}"
}

capture_diagnostics() {
  local output="$1"
  timeout 4 ros2 topic echo --once /high_rate_ev/diagnostics >"${output}" 2>&1
}

wait_for_diag() {
  local key="$1"
  local expected="$2"
  local timeout_s="${3:-60}"
  local output="$4"
  local end=$((SECONDS + timeout_s))
  local temp="${output}.tmp"
  while (( SECONDS < end )); do
    if capture_diagnostics "${temp}"; then
      if [[ "$(diag_value_from_file "${key}" "${temp}")" == "${expected}" ]]; then
        mv "${temp}" "${output}"
        return 0
      fi
    fi
    sleep 0.25
  done
  [[ -f "${temp}" ]] && mv "${temp}" "${output}"
  return 1
}

mark() {
  local label="$1"
  ros2 topic pub --once /phase4_validation/marker std_msgs/msg/String \
    "{data: '${label}'}" >/dev/null 2>&1
  printf '%s %s\n' "$(timestamp)" "${label}" >>"${evidence_dir}/markers_wall_time.txt"
  log_event "MARKER ${label}"
}

start_livox() {
  local label="$1"
  local raw_mode="$2"
  local -a remaps=()
  if [[ "${raw_mode}" == "raw" ]]; then
    remaps=(
      -r /livox/imu:=/phase4/raw/imu
      -r /livox/lidar:=/phase4/raw/lidar
    )
  fi
  start_group "${label}_livox" "${log_dir}/${label}_livox.log" \
    ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args \
    -r __node:=livox_lidar_publisher \
    "${remaps[@]}" \
    -p xfer_format:=1 \
    -p multi_topic:=0 \
    -p data_src:=0 \
    -p publish_freq:=10.0 \
    -p output_data_type:=0 \
    -p frame_id:=livox_frame \
    -p lvx_file_path:=/home/livox/livox_test.lvx \
    -p user_config_path:="${livox_config}" \
    -p cmdline_input_bd_code:=livox0000000001
  current_livox_pid="${last_started_pid}"
  sleep 1
  kill -0 "${current_livox_pid}" 2>/dev/null || fail "Livox 启动失败：${log_dir}/${label}_livox.log"
}

start_fastlio() {
  local label="$1"
  local phase2="$2"
  local phase3="$3"
  start_group "${label}_fastlio" "${log_dir}/${label}_fastlio.log" \
    ros2 launch fast_lio mapping.launch.py \
    rviz:=false \
    high_rate_ev_phase2_enabled:="${phase2}" \
    high_rate_ev_phase3_publisher_enabled:="${phase3}"
  current_fastlio_pid="${last_started_pid}"
  sleep 1
  kill -0 "${current_fastlio_pid}" 2>/dev/null || fail "FAST-LIO 启动失败：${log_dir}/${label}_fastlio.log"
}

start_gate() {
  local kind="$1"
  local label="$2"
  local input output
  if [[ "${kind}" == "imu" ]]; then
    input="/phase4/raw/imu"
    output="/livox/imu"
  else
    input="/phase4/raw/lidar"
    output="/livox/lidar"
  fi
  start_group "${label}_${kind}_gate" "${log_dir}/${label}_${kind}_gate.log" \
    python3 "${gate_program}" \
    --kind "${kind}" \
    --input "${input}" \
    --output "${output}" \
    --node-name "phase4_${kind}_gate"
}

start_bag() {
  local label="$1"
  shift
  local bag_dir="${bag_root}/${label}"
  start_group "${label}_bag" "${log_dir}/${label}_rosbag.log" \
    ros2 bag record -o "${bag_dir}" "$@"
  current_bag_pid="${last_started_pid}"
  sleep 1
  kill -0 "${current_bag_pid}" 2>/dev/null || fail "rosbag 启动失败：${log_dir}/${label}_rosbag.log"
}

fastlio_process_pid() {
  ps -eo pid=,pgid=,comm= |
    awk -v group="${current_fastlio_pid}" '$2 == group && $3 == "fastlio_mapping" {print $1; exit}'
}

start_metrics() {
  local label="$1"
  local mapping_pid=""
  for _index in $(seq 1 40); do
    mapping_pid="$(fastlio_process_pid)"
    [[ -n "${mapping_pid}" ]] && break
    sleep 0.25
  done
  [[ -n "${mapping_pid}" ]] || fail "无法定位 fastlio_mapping PID"
  printf '%s\n' "${mapping_pid}" >"${metrics_dir}/${label}_fastlio_pid.txt"
  start_group "${label}_pidstat" "${metrics_dir}/${label}_pidstat_t.txt" \
    "${pidstat_bin}" -h -t -r -u -p "${mapping_pid}" 1
  current_pidstat_pid="${last_started_pid}"
  (
    printf 'wall_time,pid,threads,rss_kib,vsz_kib\n'
    while kill -0 "${mapping_pid}" 2>/dev/null; do
      threads="$(find "/proc/${mapping_pid}/task" -mindepth 1 -maxdepth 1 -type d 2>/dev/null | wc -l)"
      rss="$(awk '/^VmRSS:/ {print $2}' "/proc/${mapping_pid}/status" 2>/dev/null || true)"
      vsz="$(awk '/^VmSize:/ {print $2}' "/proc/${mapping_pid}/status" 2>/dev/null || true)"
      printf '%s,%s,%s,%s,%s\n' "$(timestamp)" "${mapping_pid}" "${threads:-0}" "${rss:-0}" "${vsz:-0}"
      sleep 1
    done
  ) >"${metrics_dir}/${label}_rss_threads.csv" &
  current_rss_pid=$!
  register_pid "${current_rss_pid}" "${label}_rss"
}

stop_metrics() {
  stop_group "${current_pidstat_pid}" "pidstat"
  stop_group "${current_rss_pid}" "rss_sampler"
  current_pidstat_pid=""
  current_rss_pid=""
}

stop_phase() {
  local label="$1"
  stop_metrics
  stop_group "${current_bag_pid}" "${label}_bag"
  stop_group "${current_fastlio_pid}" "${label}_fastlio"
  stop_group "${current_imu_gate_pid}" "${label}_imu_gate"
  stop_group "${current_lidar_gate_pid}" "${label}_lidar_gate"
  stop_group "${current_livox_pid}" "${label}_livox"
  current_bag_pid=""
  current_fastlio_pid=""
  current_imu_gate_pid=""
  current_lidar_gate_pid=""
  current_livox_pid=""
  sleep 2
}

motion_phase() {
  local label="$1"
  local instruction="$2"
  printf '\n%s\n' "${instruction}"
  read -r -p "准备好后按 Enter 开始。"
  mark "${label}_START"
  read -r -p "用约 3 秒完成 0.2–0.3 m 动作；完全停止后按 Enter。"
  mark "${label}_STOPPED"
  sleep 2
  mark "${label}_SETTLED"
}

printf '%s\n' "Phase 4 采集目录：${session_root}"
printf '%s\n' "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}，ROS_LOCALHOST_ONLY=1"
read -r -p "确认已拆除全部桨叶、未连接飞控/MAVROS/PX4，输入 PROPS_REMOVED 继续：" safety_ack
[[ "${safety_ack}" == "PROPS_REMOVED" ]] || fail "操作者未完成安全确认"

[[ -x "${pidstat_bin}" ]] || fail "pidstat 不可用：${pidstat_bin}"
[[ -f "${livox_config}" ]] || fail "MID360 配置不存在：${livox_config}"
[[ -x "${gate_program}" ]] || fail "topic gate 不可执行：${gate_program}"
livox_driver_executable="$(
  ros2 pkg prefix livox_ros_driver2
)/lib/livox_ros_driver2/livox_ros_driver2_node"
livox_driver_library="$(
  ros2 pkg prefix livox_ros_driver2
)/lib/liblivox_ros_driver2.so"
[[ -x "${livox_driver_executable}" ]] ||
  fail "Livox 驱动可执行文件不存在：${livox_driver_executable}"
[[ -r "${livox_driver_library}" ]] ||
  fail "Livox 驱动组件库不存在：${livox_driver_library}"
{
  printf '## executable\n'
  ldd "${livox_driver_executable}"
  printf '\n## component library\n'
  ldd "${livox_driver_library}"
} >"${evidence_dir}/preflight/livox_ldd.txt"
if grep -q 'not found' "${evidence_dir}/preflight/livox_ldd.txt"; then
  fail "Livox 驱动仍有未解析的动态库"
fi
[[ "$(cat /sys/class/net/enp86s0/carrier 2>/dev/null)" == "1" ]] ||
  fail "enp86s0 无物理链路"
ip -4 -o address show dev enp86s0 | grep -Eq '[[:space:]]192\.168\.1\.5/24[[:space:]]' ||
  fail "enp86s0 未配置 192.168.1.5/24"
ping -I enp86s0 -c 3 -W 1 192.168.1.128 >"${evidence_dir}/preflight/ping_mid360.txt" ||
  fail "MID360 192.168.1.128 不可达"
forbidden_process_audit "${evidence_dir}/preflight/forbidden_processes.txt" ||
  fail "发现 MAVROS/PX4/OFFBOARD 进程"
residual_runtime_audit "${evidence_dir}/preflight/residual_runtime.txt" ||
  fail "发现残留 Livox/FAST-LIO/rosbag 进程"
verify_live_checkpoint >"${evidence_dir}/preflight/checkpoint_and_live_sha256.txt" 2>&1 ||
  fail "Phase 3.5 检查点或当前源码/构建产物 SHA-256 不一致"

{
  printf 'EKF2_EV_CTRL=11 (operator-frozen; no PX4 connection made)\n'
  printf 'MAVROS_PX4_ALLOWED=NO\n'
  ip -br link show enp86s0
  ip -br -4 address show enp86s0
  "${pidstat_bin}" -V
} >"${evidence_dir}/preflight/readiness.txt"
ps -eo pid=,ppid=,pgid=,comm=,args= >"${evidence_dir}/preflight/processes_before.txt"
log_event "PREFLIGHT_PASS"

common_topics=(
  /livox/lidar
  /livox/imu
  /Odometry
  /phase4_validation/marker
  /rosout
)
enabled_topics=(
  "${common_topics[@]}"
  /Odometry/propagated
  /high_rate_ev/diagnostics
)

log_event "A_DEFAULT_OFF_BEGIN"
start_livox A_default_off direct
wait_for_topic /livox/imu 20 || fail "A: /livox/imu 未出现"
wait_for_topic /livox/lidar 20 || fail "A: /livox/lidar 未出现"
start_fastlio A_default_off false false
wait_for_topic /Odometry 60 || fail "A: /Odometry 未出现"
sleep 3
assert_topic_absent /Odometry/propagated
assert_topic_absent /high_rate_ev/diagnostics
ros2 topic list -t >"${evidence_dir}/A_topics.txt"
ros2 node list >"${evidence_dir}/A_nodes.txt"
ros2 param get /laserMapping high_rate_ev.phase2_enabled >"${evidence_dir}/A_phase2_param.txt" 2>&1 || true
ros2 param get /laserMapping high_rate_ev.phase3_publisher_enabled >"${evidence_dir}/A_phase3_param.txt" 2>&1 || true
start_bag A_default_off "${common_topics[@]}"
start_metrics A_default_off
mark A_DEFAULT_OFF_START
printf 'A 组默认关闭基线：保持设备静止 %s 秒。\n' "${baseline_seconds}"
sleep "${baseline_seconds}"
mark A_DEFAULT_OFF_END
stop_phase A_default_off
residual_runtime_audit "${evidence_dir}/A_residual_runtime.txt" ||
  fail "A 结束后存在残留运行进程"
log_event "A_DEFAULT_OFF_END"

log_event "B_ENABLED_BEGIN"
start_livox B_enabled direct
wait_for_topic /livox/imu 20 || fail "B: /livox/imu 未出现"
wait_for_topic /livox/lidar 20 || fail "B: /livox/lidar 未出现"
start_fastlio B_enabled true true
wait_for_topic /Odometry 60 || fail "B: /Odometry 未出现"
wait_for_topic /Odometry/propagated 60 || fail "B: /Odometry/propagated 未出现"
wait_for_topic /high_rate_ev/diagnostics 20 || fail "B: diagnostics 未出现"
wait_for_diag state HEALTHY 90 "${evidence_dir}/B_first_healthy.yaml" ||
  fail "B: diagnostics 未在 90 秒内进入 HEALTHY"
start_bag B_enabled_motion "${enabled_topics[@]}"
start_metrics B_enabled
mark B_ENABLED_START
mark STATIC_60S_START
printf 'B 组静止基线：保持设备静止 %s 秒。\n' "${static_seconds}"
sleep "${static_seconds}"
mark STATIC_60S_END
motion_phase FORWARD "机头朝向不变，整机向前平移。"
motion_phase BACKWARD "机头朝向不变，整机向后平移。"
motion_phase LEFT "机头朝向不变，整机向左平移。"
motion_phase RIGHT "机头朝向不变，整机向右平移。"
motion_phase UP "姿态不变，整机向上平移。"
motion_phase DOWN "姿态不变，整机向下平移。"
motion_phase YAW90 "原地进行约 +90° Yaw 旋转，位置尽量不变。"
motion_phase YAW90_FORWARD "保持新的 Yaw 朝向，沿当前机头前方平移。"
mark FINAL_STATIC_START
printf '设备完全停止，静置 %s 秒用于回零检查。\n' "${final_static_seconds}"
sleep "${final_static_seconds}"
mark FINAL_STATIC_END
capture_diagnostics "${evidence_dir}/B_final_diagnostics.yaml" || true
mark B_ENABLED_END
stop_phase B_enabled
residual_runtime_audit "${evidence_dir}/B_residual_runtime.txt" ||
  fail "B 结束后存在残留运行进程"
log_event "B_ENABLED_END"

log_event "C_FAULT_RECOVERY_BEGIN"
start_livox C_fault raw
wait_for_topic /phase4/raw/imu 20 || fail "C: raw IMU 未出现"
wait_for_topic /phase4/raw/lidar 20 || fail "C: raw LiDAR 未出现"
start_gate imu C_fault
current_imu_gate_pid="${last_started_pid}"
start_gate lidar C_fault
current_lidar_gate_pid="${last_started_pid}"
wait_for_topic /livox/imu 10 || fail "C: IMU gate 未输出"
wait_for_topic /livox/lidar 10 || fail "C: LiDAR gate 未输出"
start_fastlio C_fault true true
wait_for_topic /Odometry/propagated 90 || fail "C: propagated 未出现"
wait_for_diag state HEALTHY 90 "${evidence_dir}/C_initial_healthy.yaml" ||
  fail "C: 初始状态未进入 HEALTHY"
start_bag C_fault_recovery \
  /phase4/raw/imu /phase4/raw/lidar "${enabled_topics[@]}"
start_metrics C_fault
mark C_INITIAL_HEALTHY

capture_diagnostics "${evidence_dir}/C_before_imu_fault.yaml"
mark IMU_GATE_STOP
stop_group "${current_imu_gate_pid}" "C_imu_gate"
current_imu_gate_pid=""
wait_for_diag fault_reason IMU_TIMEOUT 10 "${evidence_dir}/C_imu_fault.yaml" ||
  fail "C: 未观察到精确 IMU_TIMEOUT"
sleep "${fault_observe_seconds}"
capture_diagnostics "${evidence_dir}/C_imu_fault_end.yaml"
mark IMU_GATE_RESTART
start_gate imu C_imu_recovery
current_imu_gate_pid="${last_started_pid}"
wait_for_diag state HEALTHY 30 "${evidence_dir}/C_imu_recovered.yaml" ||
  fail "C: IMU timeout 后未完成恢复"
sleep "${recovery_observe_seconds}"
capture_diagnostics "${evidence_dir}/C_imu_recovery_end.yaml"
mark IMU_RECOVERY_HEALTHY

capture_diagnostics "${evidence_dir}/C_before_lidar_fault.yaml"
mark LIDAR_GATE_STOP
stop_group "${current_lidar_gate_pid}" "C_lidar_gate"
current_lidar_gate_pid=""
wait_for_diag fault_reason LIDAR_TIMEOUT 10 "${evidence_dir}/C_lidar_fault.yaml" ||
  fail "C: 未观察到精确 LIDAR_TIMEOUT"
sleep "${fault_observe_seconds}"
capture_diagnostics "${evidence_dir}/C_lidar_fault_end.yaml"
mark LIDAR_GATE_RESTART
start_gate lidar C_lidar_recovery
current_lidar_gate_pid="${last_started_pid}"
wait_for_diag state HEALTHY 30 "${evidence_dir}/C_lidar_recovered.yaml" ||
  fail "C: LiDAR timeout 后未完成恢复"
sleep "${recovery_observe_seconds}"
capture_diagnostics "${evidence_dir}/C_lidar_recovery_end.yaml"
mark LIDAR_RECOVERY_HEALTHY
stop_phase C_fault
residual_runtime_audit "${evidence_dir}/C_residual_runtime.txt" ||
  fail "C 结束后存在残留运行进程"
log_event "C_FAULT_RECOVERY_END"

log_event "D_RESTART_CYCLES_BEGIN"
for cycle in 1 2 3; do
  label="D_cycle_${cycle}"
  start_livox "${label}" direct
  wait_for_topic /livox/imu 20 || fail "${label}: IMU 未出现"
  wait_for_topic /livox/lidar 20 || fail "${label}: LiDAR 未出现"
  start_fastlio "${label}" true true
  wait_for_topic /Odometry/propagated 90 || fail "${label}: propagated 未出现"
  wait_for_diag state HEALTHY 90 "${evidence_dir}/${label}_healthy.yaml" ||
    fail "${label}: 未进入 HEALTHY"
  start_bag "${label}" \
    /Odometry /Odometry/propagated /high_rate_ev/diagnostics /phase4_validation/marker
  mark "${label}_START"
  sleep "${cycle_observe_seconds}"
  ros2 topic info -v /Odometry/propagated >"${evidence_dir}/${label}_publisher_info.txt"
  ros2 node list >"${evidence_dir}/${label}_nodes.txt"
  mapping_pid="$(fastlio_process_pid)"
  if [[ -n "${mapping_pid}" ]]; then
    find "/proc/${mapping_pid}/task" -mindepth 1 -maxdepth 1 -type d |
      wc -l >"${metrics_dir}/${label}_thread_count.txt"
  fi
  mark "${label}_END"
  stop_phase "${label}"
  residual_runtime_audit "${evidence_dir}/${label}_residual_runtime.txt" ||
    fail "${label}: 停止后存在残留进程"
done
log_event "D_RESTART_CYCLES_END"

forbidden_process_audit "${evidence_dir}/forbidden_processes_final.txt" ||
  fail "采集结束时发现 MAVROS/PX4/OFFBOARD 进程"
verify_live_checkpoint >"${evidence_dir}/checkpoint_final.txt" 2>&1 ||
  fail "采集后 Phase 3.5 SHA-256 自校验失败"
log_event "COLLECTION_COMPLETE"
