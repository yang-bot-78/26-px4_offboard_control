#!/usr/bin/env bash
set -Eeuo pipefail

# 拆桨、未解锁的手持验证。该脚本不会启动 Offboard、不会发布 setpoint、不会解锁。

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
frlio_root="$(cd -- "${script_dir}/.." && pwd -P)"
project_root="$(cd -- "${frlio_root}/../.." && pwd -P)"
highodom_ws="${HIGH_ODOM_WS:-${project_root}}"
livox_ws="${LIVOX_WS:-/home/robot/livox_mid360_env/ws_livox}"
livox_env_setup="${LIVOX_ENV_SETUP:-/home/robot/livox_mid360_env/setup_mid360.bash}"
px4_root="${PX4_CONTROL_ROOT:-${project_root}}"
helper="${script_dir}/high_rate_validation_helper.py"
cleanup_helper="${px4_root}/tools/flight/清理运行环境.sh"
lever_arm_config="${MID360_LEVER_ARM_CONFIG:-${px4_root}/src/px4_ros_com/config/mid360_lever_arm.conf}"
livox_launch="${livox_ws}/install/livox_ros_driver2/share/livox_ros_driver2/launch_ROS2/msg_MID360_launch.py"
fcu_url="${FCU_URL:-serial:///dev/serial/by-id/usb-1a86_USB_Serial-if00-port0:921600?ids=255,190}"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-26}"
validation_mode="${VALIDATION_MODE:-full}"
baseline_duration_sec="${BASELINE_DURATION_SEC:-60}"
static_duration_sec="${STATIC_DURATION_SEC:-300}"
cpu_load_workers="${CPU_LOAD_WORKERS:-4}"
cpu_load_duty_percent="${CPU_LOAD_DUTY_PERCENT:-75}"
record_livox_topics="${RECORD_LIVOX_TOPICS:-false}"
stage_delay_sec="${STAGE_DELAY_SEC:-2}"
mavros_state_timeout_sec="${MAVROS_STATE_TIMEOUT_SEC:-8}"
dry_run="${DRY_RUN:-0}"

session_stamp="${VALIDATION_TIMESTAMP:-$(date +%Y%m%d_%H%M%S)}"
case "${validation_mode}" in
  load_only) default_session_root="${highodom_ws}/validation_records/high_rate_load_${session_stamp}" ;;
  fault_only) default_session_root="${highodom_ws}/validation_records/high_rate_fault_${session_stamp}" ;;
  *) default_session_root="${highodom_ws}/validation_records/high_rate_odom_${session_stamp}" ;;
esac
session_root="${VALIDATION_DIR:-${default_session_root}}"
bag_dir="${session_root}/rosbag"
log_dir="${session_root}/logs"
result_dir="${session_root}/results"
snapshot_dir="${session_root}/snapshot"

declare -a managed_names=()
declare -a managed_pids=()
bag_pid=""
cpu_load_pid=""
cleanup_started=0

die() {
  echo "拒绝继续：$*" >&2
  exit 2
}

is_positive_number() {
  [[ "$1" =~ ^[0-9]+([.][0-9]+)?$ ]] && awk -v value="$1" 'BEGIN { exit !(value > 0) }'
}

require_file() {
  [[ -f "$1" ]] || die "找不到文件 $1"
}

source_overlay() {
  local setup="$1"
  require_file "${setup}"
  set +u
  # shellcheck disable=SC1090
  source "${setup}"
  set -u
}

register_process() {
  managed_names+=("$1")
  managed_pids+=("$2")
}

start_process() {
  local name="$1" log_file="$2"
  shift 2
  echo "[启动] ${name}；日志：${log_file}"
  setsid "$@" >"${log_file}" 2>&1 &
  local pid=$!
  register_process "${name}" "${pid}"
  printf '%s\t%s\t%s\n' "$(date --iso-8601=ns)" "${pid}" "${name}" \
    >>"${snapshot_dir}/managed_processes.tsv"
  STARTED_PID="${pid}"
}

stop_process_group() {
  local name="$1" pid="$2"
  kill -0 "${pid}" 2>/dev/null || return 0
  echo "[停止] ${name} (pid=${pid})"
  kill -INT -- "-${pid}" 2>/dev/null || kill -INT "${pid}" 2>/dev/null || true
  local index
  for index in {1..12}; do
    kill -0 "${pid}" 2>/dev/null || { wait "${pid}" 2>/dev/null || true; return 0; }
    sleep 0.5
  done
  kill -TERM -- "-${pid}" 2>/dev/null || kill -TERM "${pid}" 2>/dev/null || true
  for index in {1..6}; do
    kill -0 "${pid}" 2>/dev/null || { wait "${pid}" 2>/dev/null || true; return 0; }
    sleep 0.5
  done
  echo "[警告] ${name} 在 SIGINT/SIGTERM 后仍未退出，发送 SIGKILL。" >&2
  kill -KILL -- "-${pid}" 2>/dev/null || kill -KILL "${pid}" 2>/dev/null || true
  wait "${pid}" 2>/dev/null || true
}

cleanup() {
  local exit_code=$?
  trap - EXIT INT TERM
  ((cleanup_started == 0)) || exit "${exit_code}"
  cleanup_started=1
  echo
  echo "正在保存录包并停止本脚本启动的进程..."

  if [[ -n "${cpu_load_pid}" ]]; then
    stop_process_group "CPU 负载" "${cpu_load_pid}"
  fi

  # The bag and synthetic load are safe to stop regardless of FCU state.
  if [[ -n "${bag_pid}" ]]; then
    stop_process_group "rosbag" "${bag_pid}"
  fi

  # If interruption happened during fault injection, fail open so LIO receives
  # LiDAR again. This does not command the aircraft or alter the IMU stream.
  if command -v ros2 >/dev/null 2>&1; then
    timeout 4 ros2 service call /frlio_validation/lidar_gate std_srvs/srv/SetBool \
      '{data: true}' >/dev/null 2>&1 || true
  fi

  local may_stop_chain=1 cleanup_state_file
  cleanup_state_file="${snapshot_dir}/mavros_state_cleanup.json"
  if [[ -n "${mavros_pid:-}" ]] && kill -0 "${mavros_pid}" 2>/dev/null; then
    if ! /usr/bin/python3 "${helper}" mavros-state --timeout "${mavros_state_timeout_sec}" \
      --wait-connected --output "${cleanup_state_file}" >/dev/null 2>&1 ||
      ! grep -Eq '"armed":[[:space:]]*false' "${cleanup_state_file}"; then
      may_stop_chain=0
      echo "[安全拒绝] 退出时无法明确证明 armed=false；保留 MAVROS、EV、FR-LIO、" >&2
      echo "[安全拒绝] LiDAR 门控和 Livox 驱动运行。请人工上锁后再执行：" >&2
      echo "  ${cleanup_helper} ev sensor mavros" >&2
    fi
  fi

  local index
  if ((may_stop_chain == 1)); then
    for ((index=${#managed_pids[@]} - 1; index >= 0; index--)); do
      [[ "${managed_pids[index]}" == "${cpu_load_pid}" ]] && continue
      [[ "${managed_pids[index]}" == "${bag_pid}" ]] && continue
      stop_process_group "${managed_names[index]}" "${managed_pids[index]}"
    done
  fi

  ros2 node list >"${snapshot_dir}/nodes_after.txt" 2>/dev/null || true
  ros2 topic list -t >"${snapshot_dir}/topics_after.txt" 2>/dev/null || true
  if [[ -d "${bag_dir}" && ! -f "${bag_dir}/metadata.yaml" ]]; then
    echo "[警告] rosbag 没有 metadata.yaml，录包可能未正常收尾。" >&2
  fi
  echo "验证数据目录：${session_root}"
  echo "rosbag：${bag_dir}"
  exit "${exit_code}"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

wait_process_alive() {
  local pid="$1" name="$2"
  kill -0 "${pid}" 2>/dev/null || die "${name} 在就绪前退出，请查看 ${log_dir}"
}

wait_for_topic_message() {
  local topic="$1" timeout_s="$2" pid="${3:-}" qos="${4:-best_effort}"
  local deadline=$((SECONDS + timeout_s))
  echo "[等待] ${topic} 收到消息（最多 ${timeout_s}s）"
  while ((SECONDS < deadline)); do
    [[ -z "${pid}" ]] || wait_process_alive "${pid}" "${topic} 的上游进程"
    if timeout 3 ros2 topic echo --once --qos-reliability "${qos}" "${topic}" \
      >/dev/null 2>&1; then
      echo "[就绪] ${topic}"
      return 0
    fi
    sleep 0.5
  done
  die "等待 ${topic} 超时"
}

wait_for_service() {
  local service="$1" timeout_s="$2" pid="${3:-}"
  local deadline=$((SECONDS + timeout_s))
  echo "[等待] 服务 ${service}（最多 ${timeout_s}s）"
  while ((SECONDS < deadline)); do
    [[ -z "${pid}" ]] || wait_process_alive "${pid}" "${service} 的进程"
    if ros2 service list 2>/dev/null | grep -Fxq "${service}"; then
      echo "[就绪] ${service}"
      return 0
    fi
    sleep 0.5
  done
  die "等待服务 ${service} 超时"
}

wait_for_status() {
  local topic="$1" expected="$2" timeout_s="$3" pid="${4:-}"
  local deadline=$((SECONDS + timeout_s)) message
  echo "[等待] ${topic}=${expected}（最多 ${timeout_s}s）"
  while ((SECONDS < deadline)); do
    [[ -z "${pid}" ]] || wait_process_alive "${pid}" "${topic} 的进程"
    message="$(timeout 3 ros2 topic echo --once \
      --qos-reliability reliable --qos-durability transient_local \
      "${topic}" std_msgs/msg/String 2>/dev/null || true)"
    if printf '%s\n' "${message}" | grep -Fxq "data: ${expected}"; then
      echo "[就绪] ${topic}=${expected}"
      return 0
    fi
    sleep 0.5
  done
  die "等待 ${topic}=${expected} 超时"
}

wait_for_mavros_connected() {
  local timeout_s="$1" pid="${2:-}"
  local deadline=$((SECONDS + timeout_s)) state_file
  state_file="${snapshot_dir}/mavros_state_connected.json"
  echo "[等待] MAVROS 与飞控建立连接（最多 ${timeout_s}s）"
  while ((SECONDS < deadline)); do
    [[ -z "${pid}" ]] || wait_process_alive "${pid}" "MAVROS 与 EV 健康链"
    if /usr/bin/python3 "${helper}" mavros-state --timeout "${mavros_state_timeout_sec}" \
      --wait-connected --output "${state_file}" >/dev/null 2>&1; then
      echo "[就绪] MAVROS connected=true"
      return 0
    fi
    sleep 0.5
  done
  die "MAVROS 在 ${timeout_s}s 内未报告 connected=true；请查看 ${log_dir}/04_mavros_ev.log"
}

publisher_count() {
  ros2 topic info "$1" 2>/dev/null | awk '/Publisher count:/ {print $3; exit}'
}

require_one_publisher() {
  local topic="$1" count
  count="$(publisher_count "${topic}")"
  [[ "${count}" == 1 ]] || die "${topic} 发布者数量应为 1，当前为 ${count:-未知}"
  echo "[唯一性] ${topic} 仅有 1 个发布者"
}

assert_disarmed() {
  local state_file="${snapshot_dir}/mavros_state_disarmed_check.json"
  if ! /usr/bin/python3 "${helper}" mavros-state --timeout "${mavros_state_timeout_sec}" \
    --wait-connected --output "${state_file}" >/dev/null 2>&1; then
    die "MAVROS 未在新消息中明确报告 connected=true"
  fi
  grep -Eq '"armed":[[:space:]]*false' "${state_file}" ||
    die "MAVROS 未在同一条最新状态中明确报告 armed=false"
  if grep -Eq '"mode":[[:space:]]*"OFFBOARD"' "${state_file}"; then
    die "飞控当前处于 OFFBOARD 模式"
  fi
  echo "[安全] MAVROS connected=true、armed=false、模式不是 OFFBOARD"
}

assert_no_control_publishers() {
  local topic info
  for topic in \
    /mavros/setpoint_raw/local \
    /mavros/setpoint_position/local \
    /fmu/in/trajectory_setpoint \
    /fmu/in/vehicle_command; do
    info="$(ros2 topic info "${topic}" 2>/dev/null || true)"
    if printf '%s\n' "${info}" | grep -Eq 'Publisher count:[[:space:]]+[1-9]'; then
      die "控制话题 ${topic} 已有发布者；本验证必须与 Offboard 控制完全隔离"
    fi
  done
}

mark() {
  local label="$1"
  timeout 6 ros2 topic pub --once --qos-reliability reliable \
    /frlio_validation/event std_msgs/msg/String "{data: '${label}'}" >/dev/null
  printf '%s\t%s\n' "$(date --iso-8601=ns)" "${label}" \
    >>"${result_dir}/event_wall_time.tsv"
  echo "[标记] ${label}"
}

prompt_enter() {
  local prompt="$1"
  read -r -p "${prompt}" _
}

motion_phase() {
  local key="$1" instruction="$2" expected="$3"
  echo
  echo "动作：${instruction}"
  echo "预期：${expected}"
  prompt_enter "准备好后按 Enter，随后开始动作："
  mark "${key}_BEGIN"
  prompt_enter "完成该方向低速移动并停稳后，按 Enter："
  mark "${key}_END"
  echo "现在沿原路低速返回初始位置。"
  mark "${key}_RETURN_BEGIN"
  prompt_enter "回到初始位置并静置 3 秒后，按 Enter："
  mark "${key}_RETURN_END"
  sleep 3
}

case "${validation_mode}" in
  full|load_only|fault_only) ;;
  *) die "VALIDATION_MODE 仅支持 full、load_only 或 fault_only，收到 '${validation_mode}'" ;;
esac
case "${record_livox_topics}" in
  true|false) ;;
  *) die "RECORD_LIVOX_TOPICS 仅支持 true 或 false，收到 '${record_livox_topics}'" ;;
esac

for value in "${baseline_duration_sec}" "${static_duration_sec}" "${cpu_load_workers}" "${cpu_load_duty_percent}" "${stage_delay_sec}" "${mavros_state_timeout_sec}"; do
  is_positive_number "${value}" || die "时长、worker 数和负载占空比必须为正数，收到 '${value}'"
done
[[ "${cpu_load_workers}" =~ ^[0-9]+$ ]] || die "CPU_LOAD_WORKERS 必须是正整数"
awk -v duty="${cpu_load_duty_percent}" 'BEGIN { exit !(duty <= 100) }' ||
  die "CPU_LOAD_DUTY_PERCENT 不能大于 100"
[[ "${ROS_DOMAIN_ID}" == 26 ]] ||
  die "本机已验证的隔离域是 ROS_DOMAIN_ID=26，当前为 ${ROS_DOMAIN_ID}"

require_file /opt/ros/humble/setup.bash
require_file "${livox_ws}/install/setup.bash"
require_file "${livox_env_setup}"
require_file "${highodom_ws}/install/setup.bash"
require_file "${px4_root}/install/setup.bash"
require_file "${helper}"
require_file "${cleanup_helper}"
require_file "${lever_arm_config}"
require_file "${livox_launch}"

source_overlay /opt/ros/humble/setup.bash
source_overlay "${livox_env_setup}"
source_overlay "${highodom_ws}/install/setup.bash"
source_overlay "${px4_root}/install/setup.bash"
# This is a repository-owned shell assignment file.
# shellcheck disable=SC1090
source "${lever_arm_config}"

for command in ros2 setsid timeout awk grep tee; do
  command -v "${command}" >/dev/null || die "缺少命令 ${command}"
done
ros2 pkg prefix livox_ros_driver2 >/dev/null || die "找不到 livox_ros_driver2"
ros2 pkg prefix fr_lio >/dev/null || die "找不到 fr_lio"
ros2 pkg prefix px4_ros_com >/dev/null || die "找不到 px4_ros_com"
installed_frlio_launch="$(ros2 pkg prefix fr_lio)/share/fr_lio/launch/lio.launch.py"
require_file "${installed_frlio_launch}"
grep -q "'lid_topic'" "${installed_frlio_launch}" ||
  die "FR-LIO 安装树尚未包含 lid_topic；请先在 ${highodom_ws} 执行 colcon build --packages-select fr_lio"

if [[ "${dry_run}" == 1 ]]; then
  echo "DRY_RUN=1：依赖与参数检查通过，不连接硬件、不启动节点。"
  echo "验证模式=${validation_mode}"
  echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
  echo "FCU_URL=${fcu_url}"
  echo "MAVROS 状态采样超时=${mavros_state_timeout_sec}s"
  if [[ "${validation_mode}" == load_only ]]; then
    echo "无负载基线=${baseline_duration_sec}s，负载测试=${static_duration_sec}s"
    echo "CPU workers=${cpu_load_workers}，duty=${cpu_load_duty_percent}%"
    echo "录制 Livox 点云=${record_livox_topics}"
  elif [[ "${validation_mode}" == fault_only ]]; then
    echo "仅执行 LiDAR 故障注入、里程计门控和恢复检查"
  else
    echo "静置=${static_duration_sec}s，CPU workers=${cpu_load_workers}，duty=${cpu_load_duty_percent}%"
  fi
  echo "场次将保存到：${session_root}"
  trap - EXIT INT TERM
  exit 0
fi

mkdir -p "${log_dir}" "${result_dir}" "${snapshot_dir}"
touch "${snapshot_dir}/managed_processes.tsv" "${result_dir}/event_wall_time.tsv"

if [[ "${validation_mode}" == load_only ]]; then
  cat <<'EOF'

=================== 安全边界 ===================
1. 必须拆除全部螺旋桨，并将机体牢固固定。
2. 测试期间飞控必须始终 armed=false，且不得进入 OFFBOARD。
3. 脚本不发送运动指令、setpoint 或解锁命令。
4. 本模式不要求移动机体，也不进行 LiDAR 断流测试。
================================================
EOF
elif [[ "${validation_mode}" == fault_only ]]; then
  cat <<'EOF'

=================== 安全边界 ===================
1. 必须拆除全部螺旋桨，并将机体牢固固定。
2. 测试期间飞控必须始终 armed=false，且不得进入 OFFBOARD。
3. 脚本不发送运动指令、setpoint 或解锁命令。
4. 本模式仅暂停约 1 秒 LiDAR 转发，不移动机体、不做 CPU 负载。
================================================
EOF
else
  cat <<'EOF'

=================== 安全边界 ===================
1. 必须拆除全部螺旋桨，并保证机体不会意外伤人。
2. 测试期间飞控必须始终 armed=false，且不得进入 OFFBOARD。
3. 动作全部由人手低速完成；脚本不发送任何运动指令或 setpoint。
4. 上移时请防止线缆受力；旋转时以机体中心附近为轴。
================================================
EOF
fi
read -r -p "确认后请输入 我已拆桨并上锁：" safety_confirmation
[[ "${safety_confirmation}" == "我已拆桨并上锁" ]] || die "未得到完整安全确认"

echo "[清理] 检查并清理上次运行残留；该脚本在 armed 状态不明时会拒绝清理。"
"${cleanup_helper}" all

if pgrep -f 'high_rate_validation_helper.py (gate|fault-test|observe|load)' >/dev/null 2>&1 ||
  pgrep -f '(^|/)frlio([[:space:]]|$)' >/dev/null 2>&1; then
  die "仍存在 FR-LIO 或验证辅助进程，请先人工确认并停止"
fi
assert_no_control_publishers

env | sort >"${snapshot_dir}/environment.txt"
git -C "${frlio_root}" status --short >"${snapshot_dir}/frlio_git_status.txt" 2>/dev/null || true
git -C "${px4_root}" status --short >"${snapshot_dir}/px4_git_status.txt" 2>/dev/null || true
cp "${lever_arm_config}" "${snapshot_dir}/mid360_lever_arm.conf"
cp "${frlio_root}/config/indoors.yaml" "${snapshot_dir}/frlio_indoors.yaml"

# 1. MID360 driver
start_process "Livox MID360 驱动" "${log_dir}/01_livox.log" \
  ros2 launch "${livox_launch}"
livox_pid="${STARTED_PID}"
wait_for_topic_message /livox/imu 30 "${livox_pid}" best_effort
wait_for_topic_message /livox/lidar 30 "${livox_pid}" best_effort
require_one_publisher /livox/imu
require_one_publisher /livox/lidar
sleep "${stage_delay_sec}"

# 2. Test-only LiDAR gate. IMU bypasses this process.
start_process "LiDAR 测试门控" "${log_dir}/02_lidar_gate.log" \
  ros2 run fr_lio lidar_validation_gate
gate_pid="${STARTED_PID}"
wait_for_service /frlio_validation/lidar_gate 15 "${gate_pid}"
wait_for_topic_message /validation/livox/lidar 15 "${gate_pid}" best_effort
require_one_publisher /validation/livox/lidar
sleep "${stage_delay_sec}"

# 3. Modified FR-LIO, explicitly fed by the gated LiDAR topic.
start_process "FR-LIO 高频里程计" "${log_dir}/03_frlio.log" \
  ros2 launch fr_lio lio.launch.py rviz:=false lidar_accumulator:=false \
    lid_topic:=/validation/livox/lidar
frlio_pid="${STARTED_PID}"
wait_for_topic_message /Odometry 45 "${frlio_pid}" best_effort
wait_for_status /frlio/high_rate_odom/status HEALTHY 20 "${frlio_pid}"
require_one_publisher /Odometry
sleep "${stage_delay_sec}"

# 4. MAVROS + odometry guard + EV health monitor + vision bridge. No Offboard node.
start_process "MAVROS 与 EV 健康链" "${log_dir}/04_mavros_ev.log" \
  ros2 launch px4_ros_com fastlio_mavros_autofix.launch.py \
    fcu_url:="${fcu_url}" \
    start_odom_guard:=true start_ev_health_monitor:=true \
    start_mavros_vision_bridge:=true start_tf:=true \
    require_frlio_anchor_status:=true \
    body_to_sensor_x_m:="${MID360_BODY_TO_SENSOR_X_M}" \
    body_to_sensor_y_m:="${MID360_BODY_TO_SENSOR_Y_M}" \
    body_to_sensor_z_m:="${MID360_BODY_TO_SENSOR_Z_M}" \
    body_to_fastlio_yaw_rad:="${MID360_BODY_TO_FASTLIO_YAW_RAD}"
mavros_pid="${STARTED_PID}"
wait_for_mavros_connected 45 "${mavros_pid}"
assert_disarmed
assert_no_control_publishers
wait_for_topic_message /Odometry/guarded 20 "${mavros_pid}" best_effort
wait_for_topic_message /Odometry/healthy 30 "${mavros_pid}" best_effort
wait_for_topic_message /mavros/vision_pose/pose_cov 20 "${mavros_pid}" reliable
require_one_publisher /Odometry/healthy
require_one_publisher /mavros/vision_pose/pose_cov
sleep "${stage_delay_sec}"

# 5. One bag spans the selected validation mode.
if [[ "${validation_mode}" == load_only ]]; then
  # Keep the measurement path lightweight. Recording Livox point clouds can
  # materially alter the CPU load being measured.
  bag_topics=(
    /frlio_validation/event
    /livox/imu
    /Odometry
    /Odometry/guarded
    /Odometry/healthy
    /frlio/high_rate_odom/status
    /frlio/high_rate_odom/anchor_age
    /ev_health/status
    /ev_health/fault
    /mavros/state
  )
  if [[ "${record_livox_topics}" == true ]]; then
    bag_topics+=(/livox/lidar /validation/livox/lidar)
  fi
else
  bag_topics=(
    /frlio_validation/event
    /frlio_validation/lidar_gate_enabled
    /livox/imu
    /livox/lidar
    /validation/livox/lidar
    /Odometry
    /Odometry/guarded
    /Odometry/healthy
    /frlio/high_rate_odom/status
    /frlio/high_rate_odom/anchor_age
    /ev_health/status
    /ev_health/fault
    /ev_health/diagnostics
    /ev_health/velocity_ned
    /mavros/state
    /mavros/estimator_status
    /mavros/local_position/pose
    /mavros/local_position/odom
    /mavros/local_position/velocity_local
    /mavros/local_position/velocity_body
    /mavros/vision_pose/pose_cov
    /mavros/vision_speed/speed_twist_cov
    /tf
    /tf_static
    /rosout
  )
fi
start_process "rosbag" "${log_dir}/05_rosbag.log" \
  ros2 bag record -o "${bag_dir}" "${bag_topics[@]}"
bag_pid="${STARTED_PID}"
sleep 4
wait_process_alive "${bag_pid}" rosbag
mark SESSION_BEGIN

if [[ "${validation_mode}" == fault_only ]]; then
  echo
  echo "故障注入链路已就绪：暂停 LiDAR 转发约 1 秒，IMU 和原始 LiDAR 保持运行。"
  echo "预期：HEALTHY -> SUSPECT_STALE_LIDAR -> FAULT_STALE_LIDAR -> HEALTHY。"
  prompt_enter "确认机体已固定且飞控仍为 armed=false，按 Enter 开始："
  assert_disarmed
  mark LIDAR_FAULT_TEST_BEGIN
  set +e
  /usr/bin/python3 "${helper}" fault-test \
    --pause-seconds 1.0 \
    --output "${result_dir}/lidar_fault_test.json" \
    2>&1 | tee "${log_dir}/06_lidar_fault_test.log"
  fault_test_rc=${PIPESTATUS[0]}
  set -e
  mark LIDAR_FAULT_TEST_END
  wait_for_status /frlio/high_rate_odom/status HEALTHY 15 "${frlio_pid}"
  assert_disarmed
  mark SESSION_END
  if ((fault_test_rc == 0)); then
    echo "LiDAR 故障注入、里程计门控与恢复检查通过。"
  else
    echo "LiDAR 故障注入检查未通过，详见 lidar_fault_test.json。" >&2
  fi
  exit "${fault_test_rc}"
fi

if [[ "${validation_mode}" == load_only ]]; then
  echo
  echo "负载测试链路已就绪，机体全程保持固定。"
  echo "先记录 ${baseline_duration_sec} 秒无负载基线，再执行 ${static_duration_sec} 秒 CPU 负载。"
  echo "负载为 ${cpu_load_workers} 个 worker、每个 ${cpu_load_duty_percent}% 占空比。"
  echo "任何时候按 Ctrl+C，脚本都会停止负载、保存 rosbag，再按安全策略关闭链路。"
  prompt_enter "确认机体已固定且飞控仍为 armed=false，按 Enter 开始无负载基线："
  assert_disarmed

  mark LOAD_BASELINE_BEGIN
  set +e
  /usr/bin/python3 "${helper}" observe --lightweight \
    --duration "${baseline_duration_sec}" --print-every 30 \
    --csv "${result_dir}/baseline_anchor_age.csv" \
    --summary "${result_dir}/baseline_summary.json" \
    2>&1 | tee "${log_dir}/06_baseline_observe.log"
  baseline_rc=${PIPESTATUS[0]}
  set -e
  mark LOAD_BASELINE_END
  ((baseline_rc == 0)) || echo "[警告] 无负载基线监测异常退出，请结合 rosbag 分析。" >&2

  assert_disarmed
  mark CPU_LOAD_BEGIN
  start_process "CPU 负载" "${log_dir}/07_cpu_load.log" \
    /usr/bin/python3 "${helper}" load \
      --workers "${cpu_load_workers}" --duty "${cpu_load_duty_percent}"
  cpu_load_pid="${STARTED_PID}"
  sleep 1
  wait_process_alive "${cpu_load_pid}" "CPU 负载"
  set +e
  /usr/bin/python3 "${helper}" observe --lightweight \
    --duration "${static_duration_sec}" --print-every 30 \
    --csv "${result_dir}/load_anchor_age.csv" \
    --summary "${result_dir}/load_summary.json" \
    2>&1 | tee "${log_dir}/08_load_observe.log"
  load_observe_rc=${PIPESTATUS[0]}
  set -e
  stop_process_group "CPU 负载" "${cpu_load_pid}"
  cpu_load_pid=""
  mark CPU_LOAD_END
  ((load_observe_rc == 0)) || echo "[警告] 负载监测异常退出，请结合 rosbag 分析。" >&2

  assert_disarmed
  mark SESSION_END
  echo
  echo "负载测试完成。退出时将先用 SIGINT 保存 rosbag，再停止其他节点。"
  exit 0
fi

echo
echo "全链路已就绪。所有动作都以当前机头方向定义 body/FLU：+X 前、+Y 左、+Z 上。"
echo "请保持飞控上锁；任何时候按 Ctrl+C 都会先结束 CPU 负载、保存 rosbag，再关闭链路。"

mark INITIAL_STATIC_BEGIN
echo "先把机体放稳 10 秒，建立动作前基线。"
sleep 10
mark INITIAL_STATIC_END

motion_phase MOVE_FORWARD \
  "沿机头方向慢速前移约 0.3 m，建议速度 0.1~0.2 m/s，不要同时旋转。" \
  "/Odometry.twist.twist.linear.x 应主要为正。"
motion_phase MOVE_LEFT \
  "保持机头不变，向机体左侧慢速平移约 0.3 m。" \
  "/Odometry.twist.twist.linear.y 应主要为正。"
motion_phase MOVE_UP \
  "保持姿态，沿竖直向上慢速抬高约 0.2 m。" \
  "/Odometry.twist.twist.linear.z 应主要为正。"
motion_phase YAW_POSITIVE \
  "尽量绕机体 Z 轴中心，俯视逆时针慢转约 45~90 度。" \
  "/Odometry.twist.twist.angular.z 应为正，ENU yaw 应增大。"

echo
echo "下一阶段自动暂停 LiDAR 转发约 1 秒。Livox 驱动继续运行，所以 IMU 不会中断。"
echo "预期依次出现 SUSPECT_STALE_LIDAR（锚点 >0.15 s）、FAULT_STALE_LIDAR"
echo "（锚点 >0.40 s），FAULT 后 /Odometry 停止，恢复后回到 HEALTHY。"
prompt_enter "请固定机体并确认仍然 armed=false；按 Enter 开始故障注入："
assert_disarmed
mark LIDAR_FAULT_TEST_BEGIN
set +e
/usr/bin/python3 "${helper}" fault-test \
  --pause-seconds 1.0 \
  --output "${result_dir}/lidar_fault_test.json" \
  2>&1 | tee "${log_dir}/06_lidar_fault_test.log"
fault_test_rc=${PIPESTATUS[0]}
set -e
mark LIDAR_FAULT_TEST_END
if ((fault_test_rc != 0)); then
  echo "[警告] LiDAR 故障测试有未通过项，继续录制静置数据；详见 lidar_fault_test.json。" >&2
fi
wait_for_status /frlio/high_rate_odom/status HEALTHY 15 "${frlio_pid}"

echo
echo "最后阶段：机体固定静置 ${static_duration_sec} 秒，同时施加受控 CPU 负载。"
echo "负载为 ${cpu_load_workers} 个 worker、每个 ${cpu_load_duty_percent}% 占空比；不会占满全部 CPU。"
echo "每 30 秒打印状态、最大锚点年龄和超过 0.15 s 的样本数。"
prompt_enter "把机体牢固放置、不要触碰，按 Enter 开始："
assert_disarmed
mark STATIC_CPU_LOAD_BEGIN
start_process "CPU 负载" "${log_dir}/07_cpu_load.log" \
  /usr/bin/python3 "${helper}" load \
    --workers "${cpu_load_workers}" --duty "${cpu_load_duty_percent}"
cpu_load_pid="${STARTED_PID}"
sleep 1
wait_process_alive "${cpu_load_pid}" "CPU 负载"
set +e
/usr/bin/python3 "${helper}" observe \
  --duration "${static_duration_sec}" --print-every 30 \
  --csv "${result_dir}/static_anchor_age.csv" \
  --summary "${result_dir}/static_anchor_summary.json" \
  2>&1 | tee "${log_dir}/08_static_observe.log"
observe_rc=${PIPESTATUS[0]}
set -e
stop_process_group "CPU 负载" "${cpu_load_pid}"
mark STATIC_CPU_LOAD_END
((observe_rc == 0)) || echo "[警告] 静置监测程序异常退出，请结合 rosbag 分析。" >&2

assert_disarmed
mark SESSION_END
echo
echo "全部提示阶段完成。退出时将先用 SIGINT 正常保存 rosbag，再停止其他节点。"
