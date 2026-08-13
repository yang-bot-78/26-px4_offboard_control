#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
project_root="$(cd -- "${script_dir}/../.." && pwd -P)"

timestamp="${FLIGHT_TIMESTAMP:-$(date +%Y%m%d_%H%M%S)}"
flight_date="${timestamp%%_*}"
explicit_run_root=false
if [[ -n "${FLIGHT_RUN_DIR:-}" ]]; then
  base_run_root="${FLIGHT_RUN_DIR}"
  explicit_run_root=true
else
  base_run_root="${project_root}/flight_records/${flight_date}/flight_${timestamp}"
fi
run_root="${base_run_root}"
snapshot_dir="${run_root}/snapshot"
bag_dir="${run_root}/rosbag"
runtime_dir="${project_root}/runtime"
pid_file="${runtime_dir}/takeoff_debug_bag.pid"
run_dir_file="${runtime_dir}/takeoff_debug_bag_run_dir.txt"
status_file="${runtime_dir}/last_rosbag_status.txt"
history_file="${runtime_dir}/last_rosbag_status_history.log"
run_status_file="${snapshot_dir}/rosbag_status.txt"
param_snapshot_enabled="${PARAM_SNAPSHOT_ENABLED:-false}"
record_ego_diagnostics="${RECORD_EGO_DIAGNOSTICS:-false}"

if [[ "${param_snapshot_enabled}" != true && "${param_snapshot_enabled}" != false ]]; then
  echo "PARAM_SNAPSHOT_ENABLED 必须是 true 或 false" >&2
  exit 1
fi
if [[ "${record_ego_diagnostics}" != true && "${record_ego_diagnostics}" != false ]]; then
  echo "RECORD_EGO_DIAGNOSTICS 必须是 true 或 false" >&2
  exit 1
fi

mid360_candidates=(
  "${HOME}/livox_mid360_env/ws_fastlio/src/fast_lio/config/mid360.yaml"
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

copy_optional() {
  copy_if_exists "$1" "$2" || true
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

# 整栈启动脚本会在启动录包器之前先创建日志和组件快照。因此显式传入的场次目录
# 按设计本来就已存在；那种模式下只有"rosbag 已存在"才算冲突。
if [[ "${explicit_run_root}" == true ]]; then
  if [[ -e "${run_root}/rosbag" ]]; then
    run_root="$(ensure_unique_run_root "${run_root}")"
  fi
else
  run_root="$(ensure_unique_run_root "${run_root}")"
fi
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

copy_optional "${project_root}/tools/flight/run_takeoff_1m_hold.sh" \
  "${snapshot_dir}/run_takeoff_1m_hold.sh"
copy_optional "${project_root}/tools/flight/run_hover_1m_offboard.sh" \
  "${snapshot_dir}/run_hover_1m_offboard.sh"
copy_optional "${project_root}/tools/flight/一键启动起飞栈.sh" \
  "${snapshot_dir}/一键启动起飞栈.sh"
copy_optional "${project_root}/tools/flight/一键启动导航栈.sh" \
  "${snapshot_dir}/一键启动导航栈.sh"
copy_optional "${project_root}/src/px4_ros_com/launch/fastlio_mavros_autofix.launch.py" \
  "${snapshot_dir}/fastlio_mavros_autofix.launch.py"
copy_optional "${project_root}/src/px4_ros_com/src/bridges/fastlio_mavros_vision_bridge.cpp" \
  "${snapshot_dir}/fastlio_mavros_vision_bridge.cpp"
copy_optional "${project_root}/src/px4_ros_com/config/mid360_lever_arm.conf" \
  "${snapshot_dir}/mid360_lever_arm.conf"
copy_optional "${project_root}/tools/flight/等待ROS就绪.sh" \
  "${snapshot_dir}/等待ROS就绪.sh"
copy_optional "${project_root}/src/px4_ros_com/src/bridges/fastlio_odometry_guard.cpp" \
  "${snapshot_dir}/fastlio_odometry_guard.cpp"
copy_optional "${project_root}/src/px4_ros_com/scripts/minipc_mavros_offboard.py" \
  "${snapshot_dir}/minipc_mavros_offboard.py"
copy_optional "${project_root}/src/px4_ros_com/scripts/fastlio_ev_health_monitor.py" \
  "${snapshot_dir}/fastlio_ev_health_monitor.py"
copy_optional "${project_root}/src/px4_ros_com/px4_ros_com/ev_health.py" \
  "${snapshot_dir}/ev_health.py"
copy_optional "${project_root}/src/px4_ros_com/scripts/check_fastlio_vision_yaw.py" \
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

if [[ "${param_snapshot_enabled}" == true ]]; then
  sleep "${PARAM_SNAPSHOT_WAIT_SEC:-2}"
  dump_params_if_node_exists "/fastlio_mavros_vision_bridge" \
    "${snapshot_dir}/fastlio_mavros_vision_bridge.params.yaml"
  dump_params_if_node_exists "/fastlio_odometry_guard" \
    "${snapshot_dir}/fastlio_odometry_guard.params.yaml"
  dump_params_if_node_exists "/fastlio_ev_health_monitor" \
    "${snapshot_dir}/fastlio_ev_health_monitor.params.yaml"
  dump_params_if_node_exists "/fix_mavros_odometry_frames" \
    "${snapshot_dir}/fix_mavros_odometry_frames.params.yaml"
  dump_params_if_node_exists "/mavros" \
    "${snapshot_dir}/mavros.params.yaml"
else
  printf '%s\n' \
    'Runtime parameter dumps disabled to avoid disturbing the real-time EV chain.' \
    >"${snapshot_dir}/runtime_params_not_captured.txt"
fi

echo "本场次目录：${run_root}"
echo "参数快照目录：${snapshot_dir}"
echo "录包输出目录：${bag_dir}"
echo "是否录制完整配准点云：${RECORD_POINTCLOUD:-false}"
echo "是否录制 EGO 障碍诊断：${record_ego_diagnostics}"
echo "在本终端按 Ctrl+C 停止录包。"

bag_topics=(
  /velocity_calibration/marker
  /Odometry
  /Odometry/guarded
  /Odometry/healthy
  /planning/odom
  /fastlio_global/relocalized_pose
  /ev_health/status
  /ev_health/fault
  /ev_health/diagnostics
  /ev_health/velocity_ned
  /path
  /mavros/state
  # PX4/MAVROS 速度对比与融合判定：
  #   /Odometry.twist              FAST-LIO child/body FLU 原始速度
  #   /mavros/vision_speed/...     桥接后送入 MAVROS 的世界 ENU 视觉速度
  #   /mavros/local_position/...   PX4 EKF 输出的本地速度
  #   /fmu/out/...                 PX4 原生 uORB 镜像（若当前 XRCE-DDS 已发布）
  /mavros/estimator_status
  /fmu/out/estimator_status_flags
  /fmu/out/vehicle_local_position_v1
  /fmu/out/vehicle_local_position
  /mavros/local_position/pose
  /mavros/local_position/odom
  /mavros/local_position/velocity_local
  /mavros/local_position/velocity_body
  /mavros/vision_pose/pose_cov
  /mavros/vision_speed/speed_twist_cov
  /mavros/setpoint_raw/local
  /mavros/setpoint_raw/target_local
  # 导航栈话题。缺了这些，导航飞行就无法复盘：只留下飞机做了什么的记录，
  # 而没有"当时规划的是什么"的记录。
  /race/odom
  /race/pose
  /race/navigation_setpoint
  /race/global_path
  /race/super_planner/raw_path
  /race/planner/status
  /race/global_planner/status
  /race/flight_altitude_reference
  /race/control/status
  /race/ego/odom
  /race/ego/local_goal
  /race/ego/local_path_reference
  /race/ego/bspline
  /race/ego/validated_bspline
  /race/ego/position_command
  /race/ego/trajectory_setpoint
  /race/ego/predicted_path
  /race/ego/status
  /race/ego/command_local_goal_seq
  /race/mission/zone
  /race/mission/status
  /race/mission/event
  /goal_pose
  /tf
  /tf_static
)

if [[ "${record_ego_diagnostics}" == true ]]; then
  # These are the actual obstacle inputs and collision volume used by EGO.
  # They make a flight replay decisive: compare predicted_path against this
  # fused cloud and its inflated occupancy, rather than against /saved_map alone.
  bag_topics+=(
    /race/ego/cloud
    /race/ego/occupancy
    /race/ego/occupancy_inflate
  )
fi

# PointCloud2 的序列化和 SQLite 写入会在机载计算机上抢占 FAST-LIO 的资源。
# EV 诊断话题已包含常规飞行复盘所需的健康指标；只在专门的地面
# test.
if [[ "${RECORD_POINTCLOUD:-false}" == "true" ]]; then
  # Full registered clouds and the static map can be large. EGO's fused map
  # is controlled separately by RECORD_EGO_DIAGNOSTICS above.
  bag_topics+=(/cloud_registered /saved_map)
fi

ros2 bag record -o "${bag_dir}" "${bag_topics[@]}" &

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
