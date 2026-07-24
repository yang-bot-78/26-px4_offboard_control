#!/usr/bin/env bash
set -euo pipefail

# This script intentionally starts only:
#   1. fastlio_ev_health_monitor.py
#   2. fastlio_mavros_odometry_bridge (the only PX4 EV input)
#   3. static ENU->NED and FLU->FRD frame transforms
#   4. ros2 bag record and std_msgs marker publishers
# It never starts an Offboard node, publishes setpoints, or calls arm/mode services.

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
session_stamp="${EV_VALIDATION_TIMESTAMP:-$(date +%Y%m%d_%H%M%S)}"
session_root="${EV_VALIDATION_DIR:-${script_dir}/validation_records/prop_off_ev_${session_stamp}}"
bag_dir="${session_root}/rosbag"
snapshot_dir="${session_root}/snapshot"
log_dir="${session_root}/logs"

mkdir -p "${snapshot_dir}" "${log_dir}"

set +u
source /opt/ros/humble/setup.bash
source "${script_dir}/install/setup.bash"
set -u

node_list="$(ros2 node list 2>/dev/null || true)"
topic_list="$(ros2 topic list 2>/dev/null || true)"

dangerous_node_pattern='/(minipc_mavros_offboard|minipc_offboard_control|offboard_control|offboard_control_srv)$'
if printf '%s\n' "${node_list}" | grep -Eq "${dangerous_node_pattern}"; then
  echo "REFUSED: an Offboard-capable node is already running:" >&2
  printf '%s\n' "${node_list}" | grep -E "${dangerous_node_pattern}" >&2
  exit 2
fi

for duplicate in /fastlio_ev_health_monitor /fastlio_mavros_odometry_bridge /fastlio_mavros_vision_bridge /fastlio_vehicle_visual_odometry; do
  if printf '%s\n' "${node_list}" | grep -Fxq "${duplicate}"; then
    echo "REFUSED: ${duplicate} is already running; stop the existing EV validation/bridge first." >&2
    echo "If fastlio_mavros_autofix.launch.py is running, stop that whole launch with Ctrl+C," >&2
    echo "then start MAVROS-only using PROP_OFF_EV_VALIDATION.md before retrying." >&2
    exit 2
  fi
done

for required_topic in /Odometry /mavros/state /mavros/local_position/velocity_local; do
  if ! printf '%s\n' "${topic_list}" | grep -Fxq "${required_topic}"; then
    echo "REFUSED: required topic is missing: ${required_topic}" >&2
    echo "Start only the LiDAR driver, FAST-LIO and MAVROS, then retry." >&2
    exit 2
  fi
done

state_text="$(timeout 6 ros2 topic echo --once /mavros/state 2>/dev/null || true)"
if ! printf '%s\n' "${state_text}" | grep -Eq '^armed:[[:space:]]+false$'; then
  echo "REFUSED: MAVROS does not explicitly report armed=false:" >&2
  printf '%s\n' "${state_text}" >&2
  exit 2
fi
if printf '%s\n' "${state_text}" | grep -Eq "^mode:[[:space:]]+['\"]?OFFBOARD['\"]?$"; then
  echo "REFUSED: vehicle is already in OFFBOARD mode." >&2
  exit 2
fi

setpoint_topics=(
  /mavros/setpoint_raw/local
  /mavros/setpoint_position/local
  /fmu/in/trajectory_setpoint
  /fmu/in/vehicle_command
)
for topic in "${setpoint_topics[@]}"; do
  info="$(ros2 topic info -v "${topic}" 2>/dev/null || true)"
  if printf '%s\n' "${info}" | grep -Eq 'Publisher count:[[:space:]]+[1-9]'; then
    echo "REFUSED: a publisher already exists on control topic ${topic}:" >&2
    printf '%s\n' "${info}" >&2
    exit 2
  fi
done

ev_input_topics=(
  /mavros/odometry/out
  /mavros/vision_pose/pose_cov
  /fmu/in/vehicle_visual_odometry
)
for topic in "${ev_input_topics[@]}"; do
  info="$(ros2 topic info -v "${topic}" 2>/dev/null || true)"
  if printf '%s\n' "${info}" | grep -Eq 'Publisher count:[[:space:]]+[1-9]'; then
    echo "REFUSED: an EV publisher already exists on ${topic}:" >&2
    printf '%s\n' "${info}" >&2
    exit 2
  fi
done

printf '%s\n' "${node_list}" >"${snapshot_dir}/nodes_before.txt"
printf '%s\n' "${topic_list}" >"${snapshot_dir}/topics_before.txt"
printf '%s\n' "${state_text}" >"${snapshot_dir}/mavros_state_before.yaml"
git -C "${script_dir}" status --short >"${snapshot_dir}/git_status.txt" 2>/dev/null || true
git -C "${script_dir}" diff --stat >"${snapshot_dir}/git_diff_stat.txt" 2>/dev/null || true

bag_topics=(
  /ev_validation/marker
  /Odometry
  /Odometry/healthy
  /ev_health/status
  /ev_health/fault
  /ev_health/diagnostics
  /ev_health/velocity_ned
  /mavros/state
  /mavros/local_position/pose
  /mavros/local_position/odom
  /mavros/local_position/velocity_local
  /mavros/odometry/out
  /mavros/vision_pose/pose_cov
  /mavros/vision_speed/speed_twist_cov
  /mavros/setpoint_raw/local
  /mavros/setpoint_raw/target_local
  /fmu/out/vehicle_local_position
  /fmu/in/vehicle_visual_odometry
  /fmu/in/trajectory_setpoint
  /fmu/in/vehicle_command
  /rosout
)

bag_pid=""
validation_pid=""
watchdog_pid=""

cleanup() {
  trap - EXIT INT TERM
  if [[ -n "${watchdog_pid}" ]] && kill -0 "${watchdog_pid}" 2>/dev/null; then
    kill -INT "${watchdog_pid}" 2>/dev/null || true
    wait "${watchdog_pid}" 2>/dev/null || true
  fi
  if [[ -n "${validation_pid}" ]] && kill -0 "${validation_pid}" 2>/dev/null; then
    kill -INT "${validation_pid}" 2>/dev/null || true
    wait "${validation_pid}" 2>/dev/null || true
  fi
  if [[ -n "${bag_pid}" ]] && kill -0 "${bag_pid}" 2>/dev/null; then
    kill -INT "${bag_pid}" 2>/dev/null || true
    wait "${bag_pid}" 2>/dev/null || true
  fi
  ros2 node list >"${snapshot_dir}/nodes_after.txt" 2>/dev/null || true
  ros2 topic list -t >"${snapshot_dir}/topics_after.txt" 2>/dev/null || true
  timeout 4 ros2 topic echo --once /mavros/state \
    >"${snapshot_dir}/mavros_state_after.yaml" 2>/dev/null || true
  if [[ -f "${bag_dir}/metadata.yaml" ]]; then
    PYTHONNOUSERSITE=1 /usr/bin/python3 "${script_dir}/analyze_prop_off_ev_validation.py" \
      "${bag_dir}" --output "${session_root}/validation_report.md" \
      >"${log_dir}/analyzer.log" 2>&1 || true
  fi
  echo
  echo "Validation session saved: ${session_root}"
  echo "Rosbag: ${bag_dir}"
}
trap cleanup EXIT INT TERM

mark() {
  local label="$1"
  ros2 topic pub --once /ev_validation/marker std_msgs/msg/String \
    "{data: '${label}'}" >/dev/null
  printf '%s %s\n' "$(date --iso-8601=ns)" "${label}" >>"${snapshot_dir}/markers_wall_time.txt"
}

echo "Safety checks passed: MAVROS armed=false, mode is not OFFBOARD, no known Offboard node"
echo "or control-topic publisher is present. No PX4 parameter will be changed."
echo "Session directory: ${session_root}"

python3 "${script_dir}/prop_off_safety_watchdog.py" --parent-pid "$$" \
  >"${log_dir}/safety_watchdog.log" 2>&1 &
watchdog_pid=$!
sleep 0.5
if ! kill -0 "${watchdog_pid}" 2>/dev/null; then
  echo "Safety watchdog failed to start; inspect ${log_dir}/safety_watchdog.log" >&2
  exit 3
fi

ros2 bag record -o "${bag_dir}" "${bag_topics[@]}" \
  >"${log_dir}/rosbag.log" 2>&1 &
bag_pid=$!
sleep 1

ros2 launch px4_ros_com prop_off_ev_validation.launch.py \
  recovery_healthy_s:=7.5 \
  position_yaw_offset_rad:=0.0 \
  >"${log_dir}/validation_nodes.log" 2>&1 &
validation_pid=$!

for _index in $(seq 1 50); do
  active_nodes="$(ros2 node list 2>/dev/null || true)"
  if printf '%s\n' "${active_nodes}" | grep -Fxq /fastlio_ev_health_monitor && \
     printf '%s\n' "${active_nodes}" | grep -Fxq /fastlio_mavros_odometry_bridge; then
    break
  fi
  sleep 0.2
done

active_nodes="$(ros2 node list 2>/dev/null || true)"
if ! printf '%s\n' "${active_nodes}" | grep -Fxq /fastlio_ev_health_monitor || \
   ! printf '%s\n' "${active_nodes}" | grep -Fxq /fastlio_mavros_odometry_bridge; then
  echo "Validation nodes failed to start; inspect ${log_dir}/validation_nodes.log" >&2
  exit 3
fi
if printf '%s\n' "${active_nodes}" | grep -Eq "${dangerous_node_pattern}"; then
  echo "REFUSED: an Offboard-capable node appeared after validation launch." >&2
  exit 3
fi

ros2 param dump /fastlio_ev_health_monitor \
  >"${snapshot_dir}/fastlio_ev_health_monitor.params.yaml" 2>/dev/null || true
ros2 param dump /fastlio_mavros_odometry_bridge \
  >"${snapshot_dir}/fastlio_mavros_odometry_bridge.params.yaml" 2>/dev/null || true
ros2 node info /fastlio_ev_health_monitor \
  >"${snapshot_dir}/fastlio_ev_health_monitor.node.txt" 2>/dev/null || true
ros2 node info /fastlio_mavros_odometry_bridge \
  >"${snapshot_dir}/fastlio_mavros_odometry_bridge.node.txt" 2>/dev/null || true

for topic in "${ev_input_topics[@]}"; do
  safe_name="${topic//\//_}"
  ros2 topic info -v "${topic}" \
    >"${snapshot_dir}/topic_info${safe_name}.txt" 2>/dev/null || true
done
odom_publishers="$(ros2 topic info -v /mavros/odometry/out 2>/dev/null || true)"
if ! printf '%s\n' "${odom_publishers}" | grep -Eq 'Publisher count:[[:space:]]+1$'; then
  echo "REFUSED: /mavros/odometry/out does not have exactly one publisher." >&2
  printf '%s\n' "${odom_publishers}" >&2
  exit 3
fi
for topic in /mavros/vision_pose/pose_cov /fmu/in/vehicle_visual_odometry; do
  info="$(ros2 topic info -v "${topic}" 2>/dev/null || true)"
  if printf '%s\n' "${info}" | grep -Eq 'Publisher count:[[:space:]]+[1-9]'; then
    echo "REFUSED: duplicate EV path appeared on ${topic}." >&2
    exit 3
  fi
done

mark STATIC_60S_START
echo "Keeping the aircraft motionless for 60 seconds for timing/velocity/covariance acceptance..."
sleep 60
mark STATIC_60S_END
timeout 4 ros2 topic echo --once /ev_health/status \
  >"${snapshot_dir}/health_after_startup.yaml" 2>/dev/null || true
timeout 4 ros2 topic echo --once /ev_health/diagnostics \
  >"${snapshot_dir}/diagnostics_after_startup.yaml" 2>/dev/null || true

move_phase() {
  local label="$1"
  local instruction="$2"
  echo
  read -r -p "${instruction} 准备好后按 Enter 开始。"
  mark "${label}_START"
  read -r -p "缓慢完成动作，用约 3 秒移动 0.2-0.3 m；停止并静置后按 Enter。"
  mark "${label}_STOPPED"
  sleep 2
  mark "${label}_SETTLED"
}

move_phase FORWARD "机头朝向保持不变，整机向前平移"
move_phase BACKWARD "整机向后平移回原位"
move_phase LEFT "整机向左平移"
move_phase RIGHT "整机向右平移回原位"
move_phase UP "整机向上平移"
move_phase DOWN "整机向下平移回原高度"
move_phase YAW90_FORWARD "原地旋转Yaw约90度，然后沿新的机头前方平移"

echo
mark FASTLIO_STOP_REQUESTED
read -r -p "准备验证 FAST-LIO 中断：在 FAST-LIO 终端按 Ctrl+C，确认已停止后回此处按 Enter。"
mark FASTLIO_CONFIRMED_STOPPED
echo "保持其他节点运行并静置 3 秒，记录 SUSPECT -> FAULT 和输出中断..."
sleep 3
mark FASTLIO_FAULT_OBSERVE_END

mark FASTLIO_RESTART_REQUESTED
read -r -p "现在只重启 FAST-LIO；确认 /Odometry 恢复后回此处按 Enter。"
mark FASTLIO_CONFIRMED_RESTARTED
echo "保持静止 10 秒，记录 7.5 秒滞回恢复..."
sleep 10
mark FASTLIO_RECOVERY_OBSERVE_END

timeout 4 ros2 topic echo --once /ev_health/status \
  >"${snapshot_dir}/health_after_recovery.yaml" 2>/dev/null || true
timeout 4 ros2 topic echo --once /ev_health/diagnostics \
  >"${snapshot_dir}/diagnostics_after_recovery.yaml" 2>/dev/null || true

echo
echo "动作序列已完成。请确认飞机仍未解锁，随后按 Enter 保存并停止验证节点。"
read -r
mark VALIDATION_COMPLETE
