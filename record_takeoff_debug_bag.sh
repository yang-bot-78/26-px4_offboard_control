#!/usr/bin/env bash
set -euo pipefail

project_root="${HOME}/ws_offboard_control"
timestamp="${FLIGHT_TIMESTAMP:-$(date +%Y%m%d_%H%M%S)}"
flight_date="${timestamp%%_*}"
base_run_root="${FLIGHT_RUN_DIR:-${project_root}/flight_records/${flight_date}/flight_${timestamp}}"
run_root="${base_run_root}"
snapshot_dir="${run_root}/snapshot"
bag_dir="${run_root}/rosbag"
runtime_dir="${project_root}/runtime"
pid_file="${runtime_dir}/takeoff_debug_bag.pid"
run_dir_file="${runtime_dir}/takeoff_debug_bag_run_dir.txt"
status_file="${runtime_dir}/last_rosbag_status.txt"
history_file="${runtime_dir}/last_rosbag_status_history.log"
run_status_file="${snapshot_dir}/rosbag_status.txt"

mid360_candidates=(
  "${HOME}/livox_mid360_env/ws_fastlio/src/fast_lio/config/mid360.yaml"
  "${HOME}/fastlio2_ws/src/FAST_LIO/config/mid360.yaml"
  "${HOME}/livox_mid360_env/slam_src/FAST_LIO_ROS2-main/config/mid360.yaml"
  "${HOME}/livox_mid360_env/slam_src/FAST_LIO_ROS2-ros2/config/mid360.yaml"
)

copy_if_exists() {
  local src="$1"
  local dst="$2"
  if [[ -f "${src}" ]]; then
    cp "${src}" "${dst}"
    return 0
  fi
  return 1
}

ensure_unique_run_root() {
  local candidate="$1"
  local unique="${candidate}"
  local index=1

  while [[ -e "${unique}/rosbag" || -e "${unique}/snapshot" ]]; do
    unique="${candidate}_r${index}"
    index=$((index + 1))
  done

  printf '%s\n' "${unique}"
}

write_status() {
  local state="$1"
  local detail="${2:-}"
  local now
  now="$(date '+%F %T %z')"

  cat >"${status_file}" <<EOF
state=${state}
timestamp=${now}
flight_timestamp=${timestamp}
run_root=${run_root}
pid=${bag_pid:-}
detail=${detail}
EOF

  cat >"${run_status_file}" <<EOF
state=${state}
timestamp=${now}
flight_timestamp=${timestamp}
run_root=${run_root}
pid=${bag_pid:-}
detail=${detail}
EOF

  printf '%s state=%s flight=%s pid=%s detail=%s\n' \
    "${now}" "${state}" "${timestamp}" "${bag_pid:-}" "${detail}" >>"${history_file}"
}

dump_params_if_node_exists() {
  local node_name="$1"
  local out_file="$2"
  if ros2 node list 2>/dev/null | grep -Fxq "${node_name}"; then
    ros2 param dump "${node_name}" >"${out_file}" 2>/dev/null || true
  fi
}

run_root="$(ensure_unique_run_root "${run_root}")"
snapshot_dir="${run_root}/snapshot"
bag_dir="${run_root}/rosbag"
run_status_file="${snapshot_dir}/rosbag_status.txt"

mkdir -p "${snapshot_dir}"
mkdir -p "${runtime_dir}"

cd "${project_root}"
set +u
source /opt/ros/humble/setup.bash
source "${project_root}/install/setup.bash"
set -u

printf '%s\n' "${timestamp}" >"${snapshot_dir}/flight_timestamp.txt"
printf '%s\n' "${run_root}" >"${snapshot_dir}/flight_run_dir.txt"
env | sort >"${snapshot_dir}/environment.txt"
ros2 topic list -t >"${snapshot_dir}/ros2_topic_list.txt" 2>/dev/null || true
ros2 node list >"${snapshot_dir}/ros2_node_list.txt" 2>/dev/null || true

copy_if_exists "${project_root}/run_takeoff_1m_hold.sh" \
  "${snapshot_dir}/run_takeoff_1m_hold.sh"
copy_if_exists "${project_root}/run_hover_1m_offboard.sh" \
  "${snapshot_dir}/run_hover_1m_offboard.sh"
copy_if_exists "${project_root}/start_takeoff_1m_stack.sh" \
  "${snapshot_dir}/start_takeoff_1m_stack.sh"
copy_if_exists "${project_root}/src/px4_ros_com/launch/fastlio_mavros_autofix.launch.py" \
  "${snapshot_dir}/fastlio_mavros_autofix.launch.py"
copy_if_exists "${project_root}/src/px4_ros_com/src/bridges/fastlio_mavros_odometry_bridge.cpp" \
  "${snapshot_dir}/fastlio_mavros_odometry_bridge.cpp"
copy_if_exists "${project_root}/src/px4_ros_com/src/bridges/fastlio_mavros_vision_bridge.cpp" \
  "${snapshot_dir}/fastlio_mavros_vision_bridge.cpp"
copy_if_exists "${project_root}/src/px4_ros_com/src/bridges/fastlio_odometry_guard.cpp" \
  "${snapshot_dir}/fastlio_odometry_guard.cpp"
copy_if_exists "${project_root}/src/px4_ros_com/scripts/minipc_mavros_offboard.py" \
  "${snapshot_dir}/minipc_mavros_offboard.py"
copy_if_exists "${project_root}/src/px4_ros_com/scripts/check_fastlio_vision_yaw.py" \
  "${snapshot_dir}/check_fastlio_vision_yaw.py"

for candidate in "${mid360_candidates[@]}"; do
  if copy_if_exists "${candidate}" "${snapshot_dir}/mid360.yaml"; then
    printf '%s\n' "${candidate}" >"${snapshot_dir}/mid360_source_path.txt"
    break
  fi
done

git -C "${project_root}" rev-parse HEAD >"${snapshot_dir}/git_commit.txt" 2>/dev/null || true
git -C "${project_root}" status --short >"${snapshot_dir}/git_status.txt" 2>/dev/null || true
git -C "${project_root}" diff --stat >"${snapshot_dir}/git_diff_stat.txt" 2>/dev/null || true

sleep "${PARAM_SNAPSHOT_WAIT_SEC:-2}"

dump_params_if_node_exists "/fastlio_mavros_odometry_bridge" \
  "${snapshot_dir}/fastlio_mavros_odometry_bridge.params.yaml"
dump_params_if_node_exists "/fastlio_mavros_vision_bridge" \
  "${snapshot_dir}/fastlio_mavros_vision_bridge.params.yaml"
dump_params_if_node_exists "/fastlio_odometry_guard" \
  "${snapshot_dir}/fastlio_odometry_guard.params.yaml"
dump_params_if_node_exists "/fix_mavros_odometry_frames" \
  "${snapshot_dir}/fix_mavros_odometry_frames.params.yaml"
dump_params_if_node_exists "/mavros" \
  "${snapshot_dir}/mavros.params.yaml"

echo "Flight run directory: ${run_root}"
echo "Parameter snapshot: ${snapshot_dir}"
echo "Recording rosbag to: ${bag_dir}"
echo "Press Ctrl+C in this terminal to stop recording."

ros2 bag record -o "${bag_dir}" \
  /Odometry \
  /Odometry/guarded \
  /path \
  /cloud_registered \
  /mavros/state \
  /mavros/local_position/pose \
  /mavros/local_position/odom \
  /mavros/vision_pose/pose_cov \
  /mavros/vision_speed/speed_twist_cov \
  /mavros/setpoint_raw/local \
  /mavros/setpoint_raw/target_local \
  /minipc_mavros_offboard/mpc_setpoint_debug \
  /fmu/out/vehicle_local_position \
  /fmu/out/vehicle_local_position_setpoint \
  /fmu/out/vehicle_attitude \
  /fmu/out/vehicle_attitude_setpoint \
  /fmu/out/trajectory_setpoint \
  /fmu/in/trajectory_setpoint &

bag_pid=$!
printf '%s\n' "${bag_pid}" >"${pid_file}"
printf '%s\n' "${run_root}" >"${run_dir_file}"
write_status "recording" "rosbag started"

stop_reason="completed"

cleanup() {
  if [[ -n "${bag_pid:-}" ]] && kill -0 "${bag_pid}" 2>/dev/null; then
    kill -INT "${bag_pid}" 2>/dev/null || true
    wait "${bag_pid}" 2>/dev/null || true
  fi
  case "${stop_reason}" in
    completed)
      write_status "saved_cleanly" "rosbag exited normally"
      ;;
    signal_int)
      write_status "saved_cleanly" "rosbag stopped by SIGINT"
      ;;
    signal_term)
      write_status "saved_cleanly" "rosbag stopped by SIGTERM"
      ;;
    *)
      write_status "interrupted" "rosbag cleanup with reason=${stop_reason}"
      ;;
  esac
  rm -f "${pid_file}" "${run_dir_file}"
}

trap 'stop_reason="signal_int"; cleanup; exit 130' INT
trap 'stop_reason="signal_term"; cleanup; exit 143' TERM HUP

if wait "${bag_pid}"; then
  stop_reason="completed"
else
  stop_reason="bag_process_error"
fi

cleanup
