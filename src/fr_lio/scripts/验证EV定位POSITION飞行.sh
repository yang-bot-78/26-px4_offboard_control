#!/usr/bin/env bash
set -Eeuo pipefail

# FR-LIO -> EV -> PX4 的低空 POSITION 手飞验证。
# 本脚本不解锁、不切模式、不启动 Offboard/导航，也不发布任何 setpoint。

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

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-26}"
fcu_url="${FCU_URL:-serial:///dev/serial/by-id/usb-1a86_USB_Serial-if00-port0:921600?ids=255,190}"
stable_sec="${POSITION_STABLE_SEC:-10}"
ready_timeout_sec="${POSITION_READY_TIMEOUT_SEC:-120}"
stage_delay_sec="${STAGE_DELAY_SEC:-2}"
mavros_state_timeout_sec="${MAVROS_STATE_TIMEOUT_SEC:-8}"
dry_run="${DRY_RUN:-0}"
session_stamp="${POSITION_TIMESTAMP:-$(date +%Y%m%d_%H%M%S)}"
session_root="${POSITION_RECORD_DIR:-${highodom_ws}/validation_records/ev_position_${session_stamp}}"
bag_dir="${session_root}/rosbag"
log_dir="${session_root}/logs"
result_dir="${session_root}/results"
snapshot_dir="${session_root}/snapshot"
ready_file="${result_dir}/POSITION_READY.json"
alarm_file="${result_dir}/LOCALIZATION_ALARM.json"
stop_file="${result_dir}/STOP_MONITOR"
monitor_events="${result_dir}/monitor_events.jsonl"

declare -a managed_names=()
declare -a managed_pids=()
bag_pid=""
mavros_pid=""
cleanup_started=0
flight_authorized=0
STARTED_PID=""

die() {
  echo "拒绝继续：$*" >&2
  exit 2
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

start_process() {
  local name="$1" log_file="$2"
  shift 2
  echo "[启动] ${name}；日志：${log_file}"
  setsid "$@" >"${log_file}" 2>&1 &
  STARTED_PID=$!
  managed_names+=("${name}")
  managed_pids+=("${STARTED_PID}")
  printf '%s\t%s\t%s\n' "$(date --iso-8601=ns)" "${STARTED_PID}" "${name}" \
    >>"${snapshot_dir}/managed_processes.tsv"
}

start_visible_process() {
  local name="$1" log_file="$2"
  shift 2
  echo "[启动] ${name}；日志和本终端同步显示：${log_file}"
  setsid "$@" > >(tee -a "${log_file}") 2>&1 &
  STARTED_PID=$!
  managed_names+=("${name}")
  managed_pids+=("${STARTED_PID}")
  printf '%s\t%s\t%s\n' "$(date --iso-8601=ns)" "${STARTED_PID}" "${name}" \
    >>"${snapshot_dir}/managed_processes.tsv"
}

stop_process_group() {
  local name="$1" pid="$2" index
  kill -0 "${pid}" 2>/dev/null || return 0
  echo "[停止] ${name} (pid=${pid})"
  kill -INT -- "-${pid}" 2>/dev/null || kill -INT "${pid}" 2>/dev/null || true
  for index in {1..12}; do
    kill -0 "${pid}" 2>/dev/null || { wait "${pid}" 2>/dev/null || true; return 0; }
    sleep 0.5
  done
  kill -TERM -- "-${pid}" 2>/dev/null || kill -TERM "${pid}" 2>/dev/null || true
  for index in {1..6}; do
    kill -0 "${pid}" 2>/dev/null || { wait "${pid}" 2>/dev/null || true; return 0; }
    sleep 0.5
  done
  kill -KILL -- "-${pid}" 2>/dev/null || kill -KILL "${pid}" 2>/dev/null || true
  wait "${pid}" 2>/dev/null || true
}

latest_mavros_state() {
  /usr/bin/python3 "${helper}" mavros-state --timeout "${mavros_state_timeout_sec}" \
    --wait-connected \
    --output "$1" >/dev/null 2>&1
}

is_explicitly_disarmed() {
  local output="${snapshot_dir}/mavros_state_teardown.json"
  latest_mavros_state "${output}" && grep -Eq '"armed":[[:space:]]*false' "${output}"
}

cleanup() {
  local exit_code=$? index
  trap - EXIT INT TERM HUP
  ((cleanup_started == 0)) || exit "${exit_code}"
  cleanup_started=1
  echo
  if ((flight_authorized == 1)) && ! is_explicitly_disarmed; then
    echo "[安全拒绝] 无法明确证明 armed=false，因此保留 rosbag、监视器、MAVROS、FR-LIO 和 Livox。" >&2
    echo "[安全拒绝] 请飞手切回 STABILIZED、人工降落并上锁，然后执行：" >&2
    echo "  ROS_DOMAIN_ID=${ROS_DOMAIN_ID} ${cleanup_helper} ev sensor mavros" >&2
    echo "数据目录：${session_root}" >&2
    exit "${exit_code}"
  fi

  touch "${stop_file}" 2>/dev/null || true
  echo "正在保存 rosbag，并停止本脚本启动的链路..."
  for ((index=${#managed_pids[@]} - 1; index >= 0; index--)); do
    stop_process_group "${managed_names[index]}" "${managed_pids[index]}"
  done
  echo "POSITION 飞行数据目录：${session_root}"
  echo "rosbag：${bag_dir}"
  exit "${exit_code}"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM HUP

wait_process_alive() {
  kill -0 "$1" 2>/dev/null || die "$2 已退出，请查看 ${log_dir}"
}

wait_for_topic() {
  local topic="$1" timeout_s="$2" pid="$3" qos="${4:-best_effort}"
  local deadline=$((SECONDS + timeout_s))
  echo "[等待] ${topic}（最多 ${timeout_s}s）"
  while ((SECONDS < deadline)); do
    wait_process_alive "${pid}" "${topic} 的上游"
    if timeout 3 ros2 topic echo --once --qos-reliability "${qos}" "${topic}" \
      >/dev/null 2>&1; then
      echo "[就绪] ${topic}"
      return 0
    fi
    sleep 0.5
  done
  die "等待 ${topic} 超时"
}

wait_for_transient_status() {
  local topic="$1" timeout_s="$2" pid="$3"
  local message_type="${4:-std_msgs/msg/String}" deadline
  deadline=$((SECONDS + timeout_s))
  echo "[等待] ${topic} 锁存状态（最多 ${timeout_s}s）"
  while ((SECONDS < deadline)); do
    wait_process_alive "${pid}" "${topic} 的上游"
    if timeout 3 ros2 topic echo --once \
      --qos-reliability reliable --qos-durability transient_local \
      "${topic}" "${message_type}" >/dev/null 2>&1; then
      echo "[就绪] ${topic}"
      return 0
    fi
    sleep 0.5
  done
  die "等待 ${topic} 超时"
}

wait_for_service() {
  local service="$1" timeout_s="$2" pid="$3" deadline
  deadline=$((SECONDS + timeout_s))
  echo "[等待] 服务 ${service}（最多 ${timeout_s}s）"
  while ((SECONDS < deadline)); do
    wait_process_alive "${pid}" "${service} 的上游"
    ros2 service list 2>/dev/null | grep -Fxq "${service}" && return 0
    sleep 0.5
  done
  die "等待服务 ${service} 超时"
}

wait_for_mavros_connected() {
  local deadline output
  deadline=$((SECONDS + 45))
  output="${snapshot_dir}/mavros_state_connected.json"
  echo "[等待] MAVROS 与飞控连接（最多 45s）"
  while ((SECONDS < deadline)); do
    wait_process_alive "${mavros_pid}" "MAVROS 与 EV 链"
    if latest_mavros_state "${output}"; then
      echo "[就绪] MAVROS connected=true"
      return 0
    fi
    sleep 0.5
  done
  die "MAVROS 未连接飞控"
}

assert_disarmed_non_offboard() {
  local output="${snapshot_dir}/mavros_state_preflight.json"
  latest_mavros_state "${output}" || die "未收到新的 MAVROS connected=true 状态"
  grep -Eq '"armed":[[:space:]]*false' "${output}" || die "飞控不是 armed=false"
  ! grep -Eq '"mode":[[:space:]]*"OFFBOARD"' "${output}" || die "飞控处于 OFFBOARD"
}

assert_no_control_publishers() {
  local topic info
  for topic in \
    /mavros/setpoint_raw/local \
    /mavros/setpoint_position/local \
    /fmu/in/trajectory_setpoint \
    /fmu/in/vehicle_command; do
    info="$(ros2 topic info "${topic}" 2>/dev/null || true)"
    if grep -Eq 'Publisher count:[[:space:]]+[1-9]' <<<"${info}"; then
      die "控制话题 ${topic} 已有发布者；POSITION 验证必须与 Offboard 隔离"
    fi
  done
  echo "[安全] 未发现 Offboard/setpoint 发布者"
}

check_ekf2_ev_ctrl() {
  local response_file="${snapshot_dir}/EKF2_EV_CTRL.txt" attempt_file response
  local deadline=$((SECONDS + 30)) attempt=0
  # MAVROS 2 exposes FCU parameters as ROS parameters on /mavros/param;
  # it does not provide the ROS 1-era /mavros/param/get service.
  wait_for_service /mavros/param/get_parameters 20 "${mavros_pid}"
  echo "[等待] PX4 参数 EKF2_EV_CTRL 同步（最多 30s，只读）"
  while ((SECONDS < deadline)); do
    attempt=$((attempt + 1))
    attempt_file="${snapshot_dir}/EKF2_EV_CTRL_attempt_${attempt}.txt"
    timeout 8 ros2 param get /mavros/param EKF2_EV_CTRL >"${attempt_file}" 2>&1 || true
    response="$(<"${attempt_file}")"
    if grep -Eq 'Integer value is:[[:space:]]*11([^0-9]|$)' <<<"${response}"; then
      cp "${attempt_file}" "${response_file}"
      echo "[参数] EKF2_EV_CTRL=11（EV 水平位置 + 垂直位置 + yaw；不融合 EV 速度）"
      return 0
    fi
    if grep -Eq 'Integer value is:[[:space:]]*[0-9]+' <<<"${response}"; then
      cp "${attempt_file}" "${response_file}"
      sed -n '1,80p' "${response_file}" >&2
      die "EKF2_EV_CTRL 不是 11；脚本只读参数，不会自动修改飞控"
    fi
    printf '[等待] EKF2_EV_CTRL 尚未同步（第 %s 次）：%s\n' \
      "${attempt}" "${response:-无响应}"
    sleep 1
  done
  cp "${attempt_file}" "${response_file}" 2>/dev/null || true
  sed -n '1,80p' "${response_file}" >&2 || true
  die "30 秒内未能从 PX4 读取 EKF2_EV_CTRL；脚本不会绕过此飞行前检查"
}

mark() {
  local label="$1"
  timeout 6 ros2 topic pub --once --qos-reliability reliable \
    /frlio_position/event std_msgs/msg/String "{data: '${label}'}" >/dev/null
  printf '%s\t%s\n' "$(date --iso-8601=ns)" "${label}" >>"${result_dir}/event_wall_time.tsv"
  echo "[标记] ${label}"
}

for required in /opt/ros/humble/setup.bash "${livox_ws}/install/setup.bash" \
  "${livox_env_setup}" \
  "${highodom_ws}/install/setup.bash" "${px4_root}/install/setup.bash" \
  "${helper}" "${cleanup_helper}" "${lever_arm_config}" "${livox_launch}"; do
  require_file "${required}"
done
[[ "${ROS_DOMAIN_ID}" == 26 ]] || die "本链路已验证的 ROS_DOMAIN_ID 是 26"
[[ "${stable_sec}" =~ ^[0-9]+([.][0-9]+)?$ ]] || die "POSITION_STABLE_SEC 必须是数字"
[[ "${ready_timeout_sec}" =~ ^[0-9]+$ ]] || die "POSITION_READY_TIMEOUT_SEC 必须是整数"
[[ "${stage_delay_sec}" =~ ^[0-9]+([.][0-9]+)?$ ]] || die "STAGE_DELAY_SEC 必须是数字"
[[ "${mavros_state_timeout_sec}" =~ ^[0-9]+([.][0-9]+)?$ ]] || \
  die "MAVROS_STATE_TIMEOUT_SEC 必须是数字"

source_overlay /opt/ros/humble/setup.bash
source_overlay "${livox_env_setup}"
source_overlay "${highodom_ws}/install/setup.bash"
source_overlay "${px4_root}/install/setup.bash"
# shellcheck disable=SC1090
source "${lever_arm_config}"

ros2 pkg prefix fr_lio >/dev/null || die "找不到 fr_lio"
ros2 pkg prefix px4_ros_com >/dev/null || die "找不到 px4_ros_com"
if [[ "${dry_run}" == 1 ]]; then
  echo "DRY_RUN=1：依赖检查通过；未连接硬件、未启动节点。"
  echo "实际拆桨预检：ROS_DOMAIN_ID=26 $0"
  echo "实际放桨验证：ALLOW_PROP_ON_POSITION_TEST=YES ROS_DOMAIN_ID=26 $0"
  trap - EXIT INT TERM HUP
  exit 0
fi
[[ -t 0 ]] || die "必须在交互终端运行"

mkdir -p "${log_dir}" "${result_dir}" "${snapshot_dir}"
touch "${snapshot_dir}/managed_processes.tsv" "${result_dir}/event_wall_time.tsv"

cat <<'EOF'

================ EV POSITION 飞行安全边界 ================
1. 第一次必须先拆桨完整预检；放桨只允许在空旷、可控、无人员区域进行。
2. 飞手必须全程握住遥控器，并能立即切回 STABILIZED 和人工降落。
3. 脚本不解锁、不切模式、不发送 setpoint；只拉起定位链、录包和报警。
4. 首次放桨只做 0.5-0.7 m 低空悬停，不做水平移动，不进入 OFFBOARD。
5. 任一定位报警：立即切回 STABILIZED，人工降落并上锁，不等待自动恢复。
===========================================================
EOF

prop_on_test=false
if [[ "${ALLOW_PROP_ON_POSITION_TEST:-NO}" == YES ]]; then
  prop_on_test=true
  read -r -p "放桨飞行模式。确认遥控接管、空域、桨叶和急停后，输入“我确认低空手飞风险”：" confirmation
  [[ "${confirmation}" == "我确认低空手飞风险" ]] || die "未得到放桨飞行确认"
else
  read -r -p "默认仅允许拆桨预检。确认已拆除全部螺旋桨后，输入“我已拆桨”：" confirmation
  [[ "${confirmation}" == "我已拆桨" ]] || die "未得到拆桨确认"
fi

echo "[清理] 清理旧的 sensor/MAVROS/EV/Offboard 残留"
"${cleanup_helper}" all
assert_no_control_publishers

env | sort >"${snapshot_dir}/environment.txt"
git -C "${frlio_root}" status --short >"${snapshot_dir}/frlio_git_status.txt" 2>/dev/null || true
git -C "${px4_root}" status --short >"${snapshot_dir}/px4_git_status.txt" 2>/dev/null || true
cp "${lever_arm_config}" "${snapshot_dir}/mid360_lever_arm.conf"
cp "${frlio_root}/config/indoors.yaml" "${snapshot_dir}/frlio_indoors.yaml"

start_process "Livox MID360 驱动" "${log_dir}/01_livox.log" ros2 launch "${livox_launch}"
livox_pid="${STARTED_PID}"
wait_for_topic /livox/imu 30 "${livox_pid}" best_effort
wait_for_topic /livox/lidar 30 "${livox_pid}" best_effort
sleep "${stage_delay_sec}"

start_process "FR-LIO 高频里程计" "${log_dir}/02_frlio.log" \
  ros2 launch fr_lio lio.launch.py rviz:=false lidar_accumulator:=false \
  lid_topic:=/livox/lidar
frlio_pid="${STARTED_PID}"
wait_for_topic /Odometry 45 "${frlio_pid}" best_effort
wait_for_transient_status /frlio/high_rate_odom/status 20 "${frlio_pid}"
sleep "${stage_delay_sec}"

start_process "MAVROS 与 EV 健康链" "${log_dir}/03_mavros_ev.log" \
  ros2 launch px4_ros_com fastlio_mavros_autofix.launch.py \
  fcu_url:="${fcu_url}" start_odom_guard:=true start_ev_health_monitor:=true \
  start_mavros_vision_bridge:=true start_tf:=true \
  require_frlio_anchor_status:=true \
  body_to_sensor_x_m:="${MID360_BODY_TO_SENSOR_X_M}" \
  body_to_sensor_y_m:="${MID360_BODY_TO_SENSOR_Y_M}" \
  body_to_sensor_z_m:="${MID360_BODY_TO_SENSOR_Z_M}" \
  body_to_fastlio_yaw_rad:="${MID360_BODY_TO_FASTLIO_YAW_RAD}"
mavros_pid="${STARTED_PID}"
wait_for_mavros_connected
assert_disarmed_non_offboard
assert_no_control_publishers
check_ekf2_ev_ctrl
wait_for_topic /Odometry/healthy 30 "${mavros_pid}" best_effort
wait_for_transient_status /ev_health/flight_ready 20 "${mavros_pid}" std_msgs/msg/Bool
wait_for_topic /mavros/vision_pose/pose_cov 20 "${mavros_pid}" reliable
wait_for_topic /mavros/local_position/pose 30 "${mavros_pid}" best_effort
wait_for_topic /mavros/estimator_status 30 "${mavros_pid}" best_effort

bag_topics=(
  /frlio_position/event
  /livox/imu
  /Odometry
  /Odometry/guarded
  /Odometry/healthy
  /frlio/high_rate_odom/status
  /frlio/high_rate_odom/anchor_age
  /frlio/high_rate_odom/predictor_age
  /frlio/high_rate_odom/ev_usable
  /frlio/high_rate_odom/planner_usable
  /frlio/high_rate_odom/localization_health
  /ev_health/status
  /ev_health/fault
  /ev_health/flight_ready
  /ev_health/diagnostics
  /ev_health/velocity_ned
  /mavros/state
  /mavros/estimator_status
  /mavros/extended_state
  /mavros/local_position/pose
  /mavros/local_position/odom
  /mavros/local_position/velocity_local
  /mavros/local_position/velocity_body
  /mavros/vision_pose/pose_cov
  /mavros/vision_speed/speed_twist_cov
  /mavros/imu/data
  /tf
  /tf_static
  /rosout
)
start_process "飞行 rosbag" "${log_dir}/04_rosbag.log" \
  ros2 bag record -o "${bag_dir}" "${bag_topics[@]}"
bag_pid="${STARTED_PID}"
sleep 4
wait_process_alive "${bag_pid}" rosbag

start_visible_process "POSITION 定位监视器" "${log_dir}/05_position_monitor.log" \
  /usr/bin/python3 "${helper}" flight-monitor \
  --ready-sec "${stable_sec}" --ready-timeout "${ready_timeout_sec}" \
  --ready-file "${ready_file}" --alarm-file "${alarm_file}" \
  --stop-file "${stop_file}" --event-log "${monitor_events}"
monitor_pid="${STARTED_PID}"
mark SESSION_BEGIN

echo "[等待] 定位链连续稳定 ${stable_sec}s；详见 ${log_dir}/05_position_monitor.log"
deadline=$((SECONDS + ${ready_timeout_sec%.*} + 5))
while [[ ! -f "${ready_file}" ]]; do
  wait_process_alive "${monitor_pid}" "POSITION 定位监视器"
  ((SECONDS < deadline)) || die "POSITION 稳定门超时"
  sleep 0.2
done
assert_disarmed_non_offboard
assert_no_control_publishers
mark POSITION_READY

if [[ "${prop_on_test}" == false ]]; then
  cat <<'EOF'

==================== 拆桨预检通过 ====================
FR-LIO、EV 健康链、PX4 水平/垂直位置标志和 150 Hz 以上健康里程计
已经连续稳定 10 秒。此次是拆桨预检，不得解锁或尝试起飞。
======================================================
EOF
  read -r -p "确认飞控仍为 armed=false，按 Enter 保存预检 rosbag 并退出：" _
  assert_disarmed_non_offboard
  mark PROPS_OFF_PREFLIGHT_PASS
  touch "${stop_file}"
  exit 0
fi

# From this point onward, every exit path must prove a fresh armed=false state
# before stopping any part of the localization chain.
flight_authorized=1

cat <<'EOF'

==================== 定位稳定门已通过 ====================
飞手操作顺序：
  1. 保持 STABILIZED，由飞手手动解锁。
  2. 手动起飞至 0.5-0.7 m，先稳住高度。
  3. 飞手手动切换 POSITION/POSCTL，然后缓慢松杆。
  4. 仅悬停 10-20 秒，不做水平移动；随时准备切回 STABILIZED。
  5. 完成后切回 STABILIZED，人工降落并上锁。

终端出现“立即处置”或蜂鸣时：立刻切回 STABILIZED、人工降落并上锁。
脚本不会替飞手切模式或降落。飞行中不要按 Ctrl+C 停止定位链。
==========================================================
EOF

read -r -p "准备开始时按 Enter，脚本只添加 TAKEOFF_AUTHORIZED_BY_PILOT 录包标记：" _
assert_disarmed_non_offboard
assert_no_control_publishers
mark TAKEOFF_AUTHORIZED_BY_PILOT

echo "人工降落并上锁后，输入“已降落上锁”结束并保存 rosbag。"
while true; do
  if ! kill -0 "${monitor_pid}" 2>/dev/null; then
    echo -e "\a[立即处置] POSITION 定位监视器已退出。立即切回 STABILIZED，人工降落并上锁！" >&2
  fi
  action=""
  if read -r -t 1 action; then
    [[ "${action}" == "已降落上锁" ]] || { echo "输入不匹配，继续保持定位链运行。"; continue; }
    if is_explicitly_disarmed; then
      break
    fi
    echo "[拒绝结束] MAVROS 尚未明确报告 armed=false。继续保持全部链路运行。" >&2
  fi
done

mark LANDED_DISARMED
flight_authorized=0
touch "${stop_file}"
echo "[完成] 已确认 armed=false，正在安全保存数据并退出。"
