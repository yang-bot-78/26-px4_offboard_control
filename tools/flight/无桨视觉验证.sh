#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
project_root="$(cd -- "${script_dir}/../.." && pwd -P)"

# 本脚本刻意只启动以下四类进程：
#   1. fastlio_ev_health_monitor.py（EV 健康监视）
#   2. fastlio_mavros_vision_bridge（唯一的 PX4 EV 输入）
#   3. ENU->NED 与 FLU->FRD 静态坐标变换
#   4. ros2 bag 录包与 std_msgs 标记发布器
# 它从不启动 Offboard 节点、从不发布 setpoint、从不调用 arm/mode 服务。

session_stamp="${EV_VALIDATION_TIMESTAMP:-$(date +%Y%m%d_%H%M%S)}"
session_root="${EV_VALIDATION_DIR:-${project_root}/validation_records/prop_off_ev_${session_stamp}}"
bag_dir="${session_root}/rosbag"
snapshot_dir="${session_root}/snapshot"
log_dir="${session_root}/logs"
analyzer="${project_root}/tools/analysis/分析无桨视觉验证.py"
env_cleanup="${project_root}/tools/flight/清理运行环境.sh"
readiness_helper="${project_root}/tools/flight/等待ROS就绪.sh"
strict_coordinate_validation="${STRICT_COORDINATE_VALIDATION:-false}"
world_yaw_alignment_rad="${WORLD_YAW_ALIGNMENT_RAD:-0.0}"
lever_arm_config="${MID360_LEVER_ARM_CONFIG:-${project_root}/src/px4_ros_com/config/mid360_lever_arm.conf}"

case "${strict_coordinate_validation}" in
  true|false) ;;
  *) echo "拒绝启动：STRICT_COORDINATE_VALIDATION 必须是 true 或 false。" >&2; exit 2 ;;
esac
if [[ ! -f "${lever_arm_config}" ]]; then
  echo "拒绝启动：找不到坐标配置 ${lever_arm_config}" >&2
  exit 2
fi
if [[ ! -x "${readiness_helper}" ]]; then
  echo "拒绝启动：找不到可执行的 ROS 就绪检查器 ${readiness_helper}" >&2
  exit 2
fi
# This is a repository-owned shell assignment file, not operator input.
source "${lever_arm_config}"

# 让 /mavros/vision_speed/speed_twist_cov 录到真实内容，而不是一个空话题。
# 这里的 vision_speed 仅用于诊断：EKF2_EV_CTRL=11 不含 bit2(=4)，无论这个开关如何，
# PX4 都不会融合 EV 速度，而且本脚本从不改动 PX4 参数。默认打开的理由是：录了话题
# 却不开发布者的场次根本无法判定 twist 坐标系 —— 那正是 2026-08-09 速度坐标系修复
# 要验证的东西。设 EV_PUBLISH_SPEED=false 可复现旧的"只有位姿"场次。
publish_speed="${EV_PUBLISH_SPEED:-true}"
case "${publish_speed}" in
  true|True|TRUE|1) publish_speed=True ;;
  false|False|FALSE|0) publish_speed=False ;;
  *) echo "拒绝启动：EV_PUBLISH_SPEED 必须是 true 或 false，当前为 '${publish_speed}'" >&2; exit 2 ;;
esac

mkdir -p "${snapshot_dir}" "${log_dir}"

set +u
source /opt/ros/humble/setup.bash
source "${project_root}/install/setup.bash"
set -u

# 启动前清理上一次验证的残留。刻意只清 ev/offboard/rosbag/rviz 四类：
# 雷达驱动、FAST-LIO 和 MAVROS 必须由操作员按 无桨视觉验证.md 预先起好，本脚本
# 不启动它们，也就不能替操作员断开它们 —— 断了之后下面的必需话题检查会直接拒绝启动。
# 清掉这四类正是为了避免中断后重跑时撞上
# "REFUSED: /fastlio_ev_health_monitor is already running"。
if [[ -x "${env_cleanup}" ]]; then
  echo "启动前环境清理（EV / Offboard / 录包 / RViz）..."
  if ! "${env_cleanup}" ev offboard rosbag rviz; then
    echo "拒绝启动：环境清理失败；请处理上面列出的进程后重试。" >&2
    exit 2
  fi
else
  echo "拒绝启动：找不到清理脚本 ${env_cleanup}" >&2
  exit 2
fi

node_list="$(ros2 node list 2>/dev/null || true)"
topic_list="$(ros2 topic list 2>/dev/null || true)"

# offboard_waypoint_node 是导航栈的 Offboard 节点（race_offboard），自
# minipc_mavros_offboard.py 于 2026-08-09 删除后，它是唯一的 setpoint 发布者。
# 旧的 minipc 名字刻意保留在列表里：陈旧的 install 树仍可能提供那个可执行文件，
# 而它会向同一个话题发布。
dangerous_node_pattern='/(minipc_mavros_offboard|minipc_offboard_control|offboard_control|offboard_control_srv|offboard_waypoint_node)$'
if printf '%s\n' "${node_list}" | grep -Eq "${dangerous_node_pattern}"; then
  echo "拒绝启动：已有具备 Offboard 能力的节点在运行：" >&2
  printf '%s\n' "${node_list}" | grep -E "${dangerous_node_pattern}" >&2
  exit 2
fi

# 后两个可执行文件已从工作区删除。它们刻意保留在列表里：如果陈旧的 install 树仍
# 提供其中之一，它就会成为第二个 EV 写入者 —— 宁可拒绝启动，也不能让 PX4 同时从
# 两个来源取外部视觉。
for duplicate in /fastlio_ev_health_monitor /fastlio_mavros_odometry_bridge /fastlio_mavros_vision_bridge /fastlio_vehicle_visual_odometry; do
  if printf '%s\n' "${node_list}" | grep -Fxq "${duplicate}"; then
    echo "拒绝启动：${duplicate} 已在运行；请先停掉现有的 EV 验证或桥接进程。" >&2
    echo "如果 fastlio_mavros_autofix.launch.py 正在运行，先用 Ctrl+C 停掉整个 launch，" >&2
    echo "再按 无桨视觉验证.md 只启动 MAVROS，然后重试。" >&2
    exit 2
  fi
done

for required_topic in /Odometry /mavros/state /mavros/local_position/velocity_local; do
  if ! printf '%s\n' "${topic_list}" | grep -Fxq "${required_topic}"; then
    echo "拒绝启动：缺少必需话题：${required_topic}" >&2
    echo "请只启动雷达驱动、FAST-LIO 和 MAVROS，然后重试。" >&2
    exit 2
  fi
done

state_text="$(timeout 6 ros2 topic echo --once /mavros/state 2>/dev/null || true)"
if ! printf '%s\n' "${state_text}" | grep -Eq '^armed:[[:space:]]+false$'; then
  echo "拒绝启动：MAVROS 没有明确报告 armed=false：" >&2
  printf '%s\n' "${state_text}" >&2
  exit 2
fi
if printf '%s\n' "${state_text}" | grep -Eq "^mode:[[:space:]]+['\"]?OFFBOARD['\"]?$"; then
  echo "拒绝启动：飞机已处于 OFFBOARD 模式。" >&2
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
    echo "拒绝启动：控制话题 ${topic} 上已存在发布者：" >&2
    printf '%s\n' "${info}" >&2
    exit 2
  fi
done

# 这三个都是"送进 PX4 的 EV 输入"话题：MAVROS/PX4 侧只订阅，不发布。启动前它们的
# 发布者数必须为 0，否则说明已经有别的 EV 写入者在向飞控灌位姿。
ev_input_topics=(
  /mavros/odometry/out
  /mavros/vision_pose/pose_cov
  /fmu/in/vehicle_visual_odometry
)
for topic in "${ev_input_topics[@]}"; do
  info="$(ros2 topic info -v "${topic}" 2>/dev/null || true)"
  if printf '%s\n' "${info}" | grep -Eq 'Publisher count:[[:space:]]+[1-9]'; then
    echo "拒绝启动：${topic} 上已存在 EV 发布者：" >&2
    printf '%s\n' "${info}" >&2
    exit 2
  fi
done

printf '%s\n' "${node_list}" >"${snapshot_dir}/nodes_before.txt"
printf '%s\n' "${topic_list}" >"${snapshot_dir}/topics_before.txt"
printf '%s\n' "${state_text}" >"${snapshot_dir}/mavros_state_before.yaml"
cp "${lever_arm_config}" "${snapshot_dir}/mid360_lever_arm.conf"
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
if [[ "${strict_coordinate_validation}" == true ]]; then
  bag_topics+=(
    # FAST-LIO 对 LiDAR 或 IMU header stamp 回退会清空同步缓冲。
    # 严格场次必须录到两个源话题，才能区分时间异常与坐标异常。
    /livox/lidar
    /livox/imu
    /race/odom
    /tf
    /tf_static
  )
fi

bag_pid=""
validation_pid=""
watchdog_pid=""
race_bridge_pid=""
static_tf_pid=""

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
  if [[ -n "${race_bridge_pid}" ]] && kill -0 "${race_bridge_pid}" 2>/dev/null; then
    kill -INT "${race_bridge_pid}" 2>/dev/null || true
    wait "${race_bridge_pid}" 2>/dev/null || true
  fi
  if [[ -n "${static_tf_pid}" ]] && kill -0 "${static_tf_pid}" 2>/dev/null; then
    kill -INT "${static_tf_pid}" 2>/dev/null || true
    wait "${static_tf_pid}" 2>/dev/null || true
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
    # 分析器在 tools/analysis/ 下，不在本脚本旁边。分析失败要报出来而不是藏起来：
    # 这里的 `|| true` 是为了"坏分析器不能毁掉好 bag"，不是为了"缺报告也算干净场次"。
    if [[ -x "${analyzer}" || -f "${analyzer}" ]]; then
      analyzer_args=("${bag_dir}" --output "${session_root}/validation_report.md")
      if [[ "${strict_coordinate_validation}" == true ]]; then
        analyzer_args+=(--strict-coordinate)
      fi
      if ! PYTHONNOUSERSITE=1 /usr/bin/python3 "${analyzer}" "${analyzer_args[@]}" \
        >"${log_dir}/analyzer.log" 2>&1; then
        echo "[警告] 分析器执行失败，但 bag 完好。日志见 ${log_dir}/analyzer.log" >&2
        echo "[警告] 判定本场次之前请手动重跑分析器。" >&2
      fi
    else
      echo "[警告] 找不到分析器：${analyzer}" >&2
    fi
  fi
  echo
  echo "验证场次已保存：${session_root}"
  echo "录包目录：${bag_dir}"
}
trap cleanup EXIT INT TERM

mark() {
  local label="$1"
  ros2 topic pub --once /ev_validation/marker std_msgs/msg/String \
    "{data: '${label}'}" >/dev/null
  printf '%s %s\n' "$(date --iso-8601=ns)" "${label}" >>"${snapshot_dir}/markers_wall_time.txt"
}

echo "安全检查通过：MAVROS armed=false、当前模式不是 OFFBOARD、没有已知的 Offboard"
echo "节点或控制话题发布者。本脚本不会修改任何 PX4 参数。"
echo "场次目录：${session_root}"

if [[ "${strict_coordinate_validation}" == true ]]; then
  if printf '%s\n' "${node_list}" | grep -Fxq /fastlio_bridge_node; then
    echo "拒绝启动：/fastlio_bridge_node 已在运行，无法保证严格验证参数唯一。" >&2
    exit 2
  fi
  race_odom_info="$(ros2 topic info -v /race/odom 2>/dev/null || true)"
  if printf '%s\n' "${race_odom_info}" | grep -Eq 'Publisher count:[[:space:]]+[1-9]'; then
    echo "拒绝启动：/race/odom 已有发布者，无法保证严格验证单一桥。" >&2
    printf '%s\n' "${race_odom_info}" >&2
    exit 2
  fi
  static_tf_yaw_rad="$(awk -v yaw="${MID360_BODY_TO_FASTLIO_YAW_RAD}" 'BEGIN {print -yaw}')"
  ros2 run race_mapping fastlio_bridge_node --ros-args \
    -p "body_to_sensor_x_m:=${MID360_BODY_TO_SENSOR_X_M}" \
    -p "body_to_sensor_y_m:=${MID360_BODY_TO_SENSOR_Y_M}" \
    -p "body_to_sensor_z_m:=${MID360_BODY_TO_SENSOR_Z_M}" \
    -p "body_to_fastlio_yaw_rad:=${MID360_BODY_TO_FASTLIO_YAW_RAD}" \
    -p "world_yaw_alignment_rad:=${world_yaw_alignment_rad}" \
    >"${log_dir}/race_bridge.log" 2>&1 &
  race_bridge_pid=$!
  ros2 run tf2_ros static_transform_publisher \
    --x "${MID360_BODY_TO_SENSOR_X_M}" \
    --y "${MID360_BODY_TO_SENSOR_Y_M}" \
    --z "${MID360_BODY_TO_SENSOR_Z_M}" \
    --roll 0 --pitch 0 --yaw "${static_tf_yaw_rad}" \
    --frame-id base_link --child-frame-id body \
    --ros-args -r __node:=strict_base_link_to_body \
    >"${log_dir}/strict_static_tf.log" 2>&1 &
  static_tf_pid=$!
  # A one-shot `ros2 topic echo` can race DDS discovery immediately after the
  # publisher starts. Retry message reception while also checking that the
  # bridge process is still alive, so a discovery delay is not reported as a
  # coordinate-conversion failure.
  if ! READINESS_PROCESS_PID="${race_bridge_pid}" \
    "${readiness_helper}" message 30 /race/odom \
    >"${log_dir}/race_bridge_readiness.log" 2>&1; then
    echo "拒绝继续：严格验证桥未能发布 /race/odom。" >&2
    sed -n '1,80p' "${log_dir}/race_bridge_readiness.log" >&2 || true
    exit 3
  fi
  echo "严格坐标验证：已启动唯一 fastlio_bridge_node，并发布 base_link -> body 候选 TF。"
  echo "候选参数：x=${MID360_BODY_TO_SENSOR_X_M} y=${MID360_BODY_TO_SENSOR_Y_M} z=${MID360_BODY_TO_SENSOR_Z_M} installation_yaw=${MID360_BODY_TO_FASTLIO_YAW_RAD} world_yaw=${world_yaw_alignment_rad}"
fi

python3 "${script_dir}/无桨安全看门狗.py" --parent-pid "$$" \
  >"${log_dir}/safety_watchdog.log" 2>&1 &
watchdog_pid=$!
sleep 0.5
if ! kill -0 "${watchdog_pid}" 2>/dev/null; then
  echo "安全看门狗启动失败；请查看 ${log_dir}/safety_watchdog.log" >&2
  exit 3
fi

ros2 bag record -o "${bag_dir}" "${bag_topics[@]}" \
  >"${log_dir}/rosbag.log" 2>&1 &
bag_pid=$!
sleep 1

ros2 launch px4_ros_com prop_off_ev_validation.launch.py \
  recovery_healthy_s:=7.5 \
  world_yaw_alignment_rad:="${world_yaw_alignment_rad}" \
  publish_speed:="${publish_speed}" \
  body_to_sensor_x_m:="${MID360_BODY_TO_SENSOR_X_M}" \
  body_to_sensor_y_m:="${MID360_BODY_TO_SENSOR_Y_M}" \
  body_to_sensor_z_m:="${MID360_BODY_TO_SENSOR_Z_M}" \
  body_to_fastlio_yaw_rad:="${MID360_BODY_TO_FASTLIO_YAW_RAD}" \
  >"${log_dir}/validation_nodes.log" 2>&1 &
validation_pid=$!

for _index in $(seq 1 50); do
  active_nodes="$(ros2 node list 2>/dev/null || true)"
  if printf '%s\n' "${active_nodes}" | grep -Fxq /fastlio_ev_health_monitor && \
     printf '%s\n' "${active_nodes}" | grep -Fxq /fastlio_mavros_vision_bridge; then
    break
  fi
  sleep 0.2
done

active_nodes="$(ros2 node list 2>/dev/null || true)"
if ! printf '%s\n' "${active_nodes}" | grep -Fxq /fastlio_ev_health_monitor || \
   ! printf '%s\n' "${active_nodes}" | grep -Fxq /fastlio_mavros_vision_bridge; then
  echo "验证节点启动失败；请查看 ${log_dir}/validation_nodes.log" >&2
  exit 3
fi
if printf '%s\n' "${active_nodes}" | grep -Eq "${dangerous_node_pattern}"; then
  echo "拒绝继续：验证 launch 启动后出现了具备 Offboard 能力的节点。" >&2
  exit 3
fi

ros2 param dump /fastlio_ev_health_monitor \
  >"${snapshot_dir}/fastlio_ev_health_monitor.params.yaml" 2>/dev/null || true
ros2 param dump /fastlio_mavros_vision_bridge \
  >"${snapshot_dir}/fastlio_mavros_vision_bridge.params.yaml" 2>/dev/null || true
ros2 node info /fastlio_ev_health_monitor \
  >"${snapshot_dir}/fastlio_ev_health_monitor.node.txt" 2>/dev/null || true
ros2 node info /fastlio_mavros_vision_bridge \
  >"${snapshot_dir}/fastlio_mavros_vision_bridge.node.txt" 2>/dev/null || true

for topic in "${ev_input_topics[@]}"; do
  safe_name="${topic//\//_}"
  ros2 topic info -v "${topic}" \
    >"${snapshot_dir}/topic_info${safe_name}.txt" 2>/dev/null || true
done
# 启动后，唯一允许存在的 EV 写入者是 vision bridge 的 /mavros/vision_pose/pose_cov，
# 且必须恰好一个。
#
# 这里以前要求 /mavros/odometry/out "恰好一个发布者"，那是 fastlio_mavros_odometry_bridge
# 时代的遗留。该桥已从源码删除，本 launch 也不会启动任何东西去发布它，所以那条检查
# 永远不可能通过 —— 它还和上面"启动前该话题发布者必须为 0"的检查互相矛盾（同一个
# 话题不可能既必须是 0、又必须是 1，而这中间没有任何进程去发布它）。
# /mavros/odometry/out 是 MAVROS 的订阅端，属于 EV 输入话题，0 个发布者才是当前架构
# 的正确状态；它继续留在录包列表里只是为了在有人重新引入该通路时能录到证据。
pose_publishers="$(ros2 topic info -v /mavros/vision_pose/pose_cov 2>/dev/null || true)"
if ! printf '%s\n' "${pose_publishers}" | grep -Eq 'Publisher count:[[:space:]]+1$'; then
  echo "拒绝继续：/mavros/vision_pose/pose_cov 的发布者不是恰好一个。" >&2
  echo "它是当前唯一的 PX4 EV 输入通路：0 个说明 vision bridge 没起来，" >&2
  echo "多于 1 个说明存在重复 EV 写入者，两种情况都不能继续验证。" >&2
  printf '%s\n' "${pose_publishers}" >&2
  exit 3
fi
for topic in /mavros/odometry/out /fmu/in/vehicle_visual_odometry; do
  info="$(ros2 topic info -v "${topic}" 2>/dev/null || true)"
  if printf '%s\n' "${info}" | grep -Eq 'Publisher count:[[:space:]]+[1-9]'; then
    echo "拒绝继续：${topic} 上出现了重复的 EV 通路。" >&2
    printf '%s\n' "${info}" >&2
    exit 3
  fi
done

mark STATIC_60S_START
echo "请保持飞机静止不动 60 秒，用于时序、速度和协方差的验收采样..."
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

if [[ "${strict_coordinate_validation}" == true ]]; then
  echo
  read -r -p "保持飞机原地，准备缓慢旋转 Yaw 360 度；准备好后按 Enter。"
  mark YAW360_START
  read -r -p "用约 15-20 秒完成一整圈，尽量围绕飞机控制中心旋转；停止并静置后按 Enter。"
  mark YAW360_STOPPED
  sleep 2
  mark YAW360_SETTLED

  move_phase LEFT "机头朝向保持不变，整机向左平移"
  move_phase FORWARD "机头朝向保持不变，整机向前平移"
  move_phase UP "机头朝向保持不变，整机向上平移"
  move_phase DOWN "整机向下平移回原高度"
  move_phase YAW90_FORWARD "原地旋转 Yaw 约 90 度，然后沿新的机头前方平移"

  echo
  echo "严格坐标动作序列已完成。请确认飞机仍未解锁，随后按 Enter 保存并停止录包。"
  read -r
  mark VALIDATION_COMPLETE
  exit 0
fi

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
