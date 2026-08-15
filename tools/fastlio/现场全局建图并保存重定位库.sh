#!/usr/bin/env bash
set -euo pipefail

# 生成 FR-LIO 全局重定位数据库，并提供手飞建图所需的定位链路。
# 启动：MID-360、项目内 FR-LIO、全局后端、MAVROS、MAVLink ODOMETRY EV 链路和诊断 rosbag。
# 不启动：Offboard、导航控制、解锁、模式切换或任何 setpoint 发布器。

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" || "${1:-}" == "--帮助" ]]; then
  cat <<'EOF'
用法：
  ./tools/fastlio/现场全局建图并保存重定位库.sh

默认保存目录：
  maps/frlio_global_3d_<保存时间戳>

可选环境变量：
  FRLIO_GLOBAL_MAP_DIR=/绝对路径/地图父目录
  FRLIO_CONFIG=/绝对路径/indoors.yaml
  MID360_FRLIO_DELAY_SEC=4
  GLOBAL_MAP_WAIT_TIMEOUT=90
  FRLIO_GLOBAL_MAP_RESOLUTION=0.15
  FRLIO_GLOBAL_MAP_MAX_Z=2.5
  FCU_URL=serial:///dev/ttyUSB0:921600?ids=255,190
  MAVROS_STABLE_SEC=10
  RECORD_BAG=true
EOF
  exit 0
fi
if (($# > 0)); then
  echo "[错误] 不支持的位置参数：$*" >&2
  exit 2
fi
if [[ ! -t 0 ]]; then
  echo "[错误] 必须在交互终端运行，以便确认安全状态和保存操作。" >&2
  exit 2
fi

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
project_root="$(cd -- "${script_dir}/../.." && pwd -P)"
livox_env="${LIVOX_MID360_ENV:-${HOME}/livox_mid360_env}"
frlio_config="${FRLIO_CONFIG:-${project_root}/src/fr_lio/config/indoors.yaml}"
backend_config="${script_dir}/三关键帧测试后端参数.yaml"
rviz_config="${project_root}/Lin_shi/fastlio_global_slam/config/fastlio_global_slam.rviz"
autofix_launch="${project_root}/src/px4_ros_com/launch/fastlio_mavros_autofix.launch.py"
wait_timeout="${GLOBAL_MAP_WAIT_TIMEOUT:-90}"
driver_delay="${MID360_FRLIO_DELAY_SEC:-${MID360_FASTLIO_DELAY_SEC:-4}}"
resolution="${FRLIO_GLOBAL_MAP_RESOLUTION:-${FASTLIO_GLOBAL_MAP_RESOLUTION:-0.15}}"
fcu_url="${FCU_URL:-serial:///dev/ttyUSB0:921600?ids=255,190}"
odometry_stability_gate="${project_root}/tools/flight/等待flight_ready稳定.py"
bag_script="${project_root}/tools/rosbag/开始录包.sh"
mavros_stable_sec="${MAVROS_STABLE_SEC:-10}"
record_bag="${RECORD_BAG:-true}"
timestamp="$(date +%Y%m%d_%H%M%S)"
flight_date="${timestamp%%_*}"
flight_run_dir="${project_root}/flight_records/${flight_date}/flight_${timestamp}"
map_parent="${FRLIO_GLOBAL_MAP_DIR:-${FASTLIO_GLOBAL_MAP_DIR:-${project_root}/maps}}"
map_dir="${map_parent%/}/frlio_global_3d_${timestamp}"
max_z="${FRLIO_GLOBAL_MAP_MAX_Z:-${FASTLIO_GLOBAL_MAP_MAX_Z:-2.5}}"
log_dir="${project_root}/runtime/全局建图_${timestamp}"

[[ "${map_parent}" == /* ]] || { echo "[错误] FRLIO_GLOBAL_MAP_DIR 必须是绝对路径。" >&2; exit 2; }
if [[ ! "${wait_timeout}" =~ ^[0-9]+$ ]] || ((wait_timeout < 10)); then
  echo "[错误] GLOBAL_MAP_WAIT_TIMEOUT 必须是不小于 10 的整数。" >&2
  exit 2
fi
[[ "${resolution}" =~ ^([0-9]+([.][0-9]*)?|[.][0-9]+)$ ]] || {
  echo "[错误] FRLIO_GLOBAL_MAP_RESOLUTION 必须是正数。" >&2; exit 2;
}
[[ "${max_z}" =~ ^-?[0-9]+([.][0-9]+)?$ ]] || {
  echo "[错误] FRLIO_GLOBAL_MAP_MAX_Z 必须是数字，例如 3.0。" >&2; exit 2;
}
[[ "${mavros_stable_sec}" =~ ^[0-9]+$ ]] || {
  echo "[错误] MAVROS_STABLE_SEC 必须是正整数。" >&2; exit 2;
}
((mavros_stable_sec >= 3)) || {
  echo "[错误] MAVROS_STABLE_SEC 不能小于 3 秒。" >&2; exit 2;
}
case "${record_bag}" in
  true|false) ;;
  *) echo "[错误] RECORD_BAG 必须是 true 或 false。" >&2; exit 2 ;;
esac
for required in \
  "${livox_env}/run_mid360_driver.sh" \
  "${livox_env}/setup_mid360.bash" \
  "${frlio_config}" \
  "${backend_config}" \
  "${rviz_config}" \
  "${project_root}/install/setup.bash" \
  "${project_root}/tools/flight/清理运行环境.sh" \
  "${autofix_launch}" \
  "${odometry_stability_gate}" \
  "${bag_script}" \
  "${project_root}/src/px4_ros_com/config/mid360_lever_arm.conf" \
  "$(command -v pcl_passthrough_filter 2>/dev/null || printf '%s' /missing/pcl_passthrough_filter)"; do
  [[ -f "${required}" ]] || { echo "[错误] 缺少文件：${required}" >&2; exit 2; }
done
if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
  echo "[错误] 当前没有图形桌面环境，无法启动 RViz。" >&2
  exit 2
fi

mkdir -p "${log_dir}"
cat <<EOF
============================================================
          MID-360 全局建图并保存重定位数据库
============================================================
保存目录：${map_dir}
日志目录：${log_dir}
飞行诊断目录：${flight_run_dir}

本流程启动：雷达、项目内 FR-LIO、全局关键帧后端、MAVROS、MAVLink ODOMETRY EV 链路和诊断 rosbag。
本流程不会启动：Offboard、导航、解锁、模式切换或任何 setpoint 发布器。

开始前确认：
  1. 当前没有其他雷达、FR-LIO、MAVROS 或 EV 实例；
  2. 飞手已确认遥控器可接管，起飞区域无人员；
  3. 先等待脚本报告 EV 链路连续稳定，再手动解锁并切 Position；
  4. 建图完成后先手动降落、上锁，再在脚本中保存并退出。

注意：脚本只提供定位数据，不会替飞手控制飞机。若链路异常，立即切回
Stabilized/手动模式；不要在飞机已解锁时按 Ctrl+C 结束脚本。

每次保存都会创建新的时间戳目录，不会沿用或覆盖旧地图；保存后同时生成去顶端点云副本。
============================================================
EOF
read -r -p "确认安全条件后按回车继续（Ctrl+C 取消）：" _confirm

cd "${project_root}"
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# FR-LIO directly subscribes to livox_ros_driver2/CustomMsg, so the parent
# process must expose the MID-360 message typesupport before loading this
# workspace overlay.
# shellcheck disable=SC1091
source "${livox_env}/setup_mid360.bash"
# shellcheck disable=SC1091
export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"
export COLCON_TRACE="${COLCON_TRACE:-}"
# shellcheck disable=SC1091
source "${project_root}/install/setup.bash"
set -u
export PYTHONNOUSERSITE=1
# shellcheck disable=SC1091
source "${project_root}/src/px4_ros_com/config/mid360_lever_arm.conf"

# Verify the package index before starting any FCU process so an older overlay
# cannot select an outdated EV launch file.
expected_px4_prefix="${project_root}/install/px4_ros_com"
export AMENT_PREFIX_PATH="${expected_px4_prefix}${AMENT_PREFIX_PATH:+:${AMENT_PREFIX_PATH}}"
for px4_python_site in \
  "${expected_px4_prefix}"/lib/python*/site-packages \
  "${expected_px4_prefix}"/local/lib/python*/dist-packages \
  "${expected_px4_prefix}"/local/lib/python*/site-packages; do
  [[ -d "${px4_python_site}" ]] || continue
  export PYTHONPATH="${px4_python_site}${PYTHONPATH:+:${PYTHONPATH}}"
done
resolved_px4_share="$(python3 - <<'PY'
from ament_index_python.packages import get_package_share_directory
print(get_package_share_directory("px4_ros_com"))
PY
)"
expected_px4_share="${project_root}/install/px4_ros_com/share/px4_ros_com"
if [[ "${resolved_px4_share}" != "${expected_px4_share}" ]]; then
  echo "[错误] px4_ros_com 被旧 overlay 覆盖：${resolved_px4_share}" >&2
  echo "[错误] 期望当前工作区：${expected_px4_share}" >&2
  exit 2
fi
if grep -Eq 'DeclareLaunchArgument\("start_(bridge|px4_ev_bridge)' "${autofix_launch}" ||
   grep -Eq 'Only one external-vision output may feed PX4' "${autofix_launch}"; then
  echo "[错误] 当前 fastlio_mavros_autofix.launch.py 仍含旧的多 EV 输出路径，拒绝启动。" >&2
  exit 2
fi

"${project_root}/tools/flight/清理运行环境.sh" offboard rosbag ev rviz sensor mavros
# livox_ros_driver2 may need a short interval to release its SDK UDP workers
# after the cleanup script had to terminate an old driver instance.
echo "[等待] 等待 MID-360 驱动 UDP 接收端口释放。"
sleep 5

declare -a names=() pids=() pgids=()
stopping=false
start_component() {
  local name="$1" logfile="$2"; shift 2
  echo "[启动] ${name}"
  setsid "$@" >"${logfile}" 2>&1 &
  names+=("${name}"); pids+=("$!"); pgids+=("$!")
  echo "[启动] ${name} PID=$!，日志=${logfile}"
}
running() {
  local i name="$1"
  for ((i=0; i<${#names[@]}; i++)); do
    [[ "${names[i]}" == "${name}" ]] || continue
    kill -0 "${pids[i]}" 2>/dev/null || kill -0 -- "-${pgids[i]}" 2>/dev/null
    return
  done
  return 1
}

# This process group contains MAVROS, the EV health monitor, and the external
# vision bridge.  Never tear it down while the aircraft state is unknown or
# armed; the operator must land and disarm first.
flight_state_allows_shutdown() {
  local state
  command -v ros2 >/dev/null 2>&1 || return 1
  state="$(timeout 8 ros2 topic echo --once /mavros/state mavros_msgs/msg/State 2>/dev/null || true)"
  [[ -n "${state}" ]] && grep -Eq '^armed:[[:space:]]+false$' <<<"${state}"
}

wait_until_disarmed_for_shutdown() {
  while ! flight_state_allows_shutdown; do
    echo "[安全] 未明确确认 armed=false；保留 EV 健康桥和外部视觉链路。请先降落并上锁。" >&2
    sleep 2
  done
}

stop_named_component() {
  local target_name="$1" timeout_s="$2"
  local i deadline pgid pid
  for ((i=0; i<${#names[@]}; i++)); do
    [[ "${names[i]}" == "${target_name}" ]] || continue
    pgid="${pgids[i]}"
    pid="${pids[i]}"
    if ! kill -0 "${pid}" 2>/dev/null && ! kill -0 -- "-${pgid}" 2>/dev/null; then
      wait "${pid}" 2>/dev/null || true
      return 0
    fi
    echo "[停止] ${target_name}：发送 SIGINT 并等待完成落盘。"
    kill -INT -- "-${pgid}" 2>/dev/null || true
    deadline=$((SECONDS + timeout_s))
    while ((SECONDS < deadline)); do
      if ! kill -0 "${pid}" 2>/dev/null && ! kill -0 -- "-${pgid}" 2>/dev/null; then
        wait "${pid}" 2>/dev/null || true
        echo "[停止] ${target_name} 已干净退出。"
        return 0
      fi
      sleep 1
    done
    echo "[警告] ${target_name} 在 ${timeout_s}s 内未退出，发送 SIGTERM。" >&2
    kill -TERM -- "-${pgid}" 2>/dev/null || true
    deadline=$((SECONDS + 5))
    while ((SECONDS < deadline)); do
      if ! kill -0 "${pid}" 2>/dev/null && ! kill -0 -- "-${pgid}" 2>/dev/null; then
        wait "${pid}" 2>/dev/null || true
        return 1
      fi
      sleep 1
    done
    echo "[警告] ${target_name} 仍未退出，强制回收该录包进程组。" >&2
    kill -KILL -- "-${pgid}" 2>/dev/null || true
    wait "${pid}" 2>/dev/null || true
    return 1
  done
  return 0
}

stop_all() {
  [[ "${stopping}" == false ]] || return
  ((${#pgids[@]} > 0)) || return
  # Do not disconnect a live aircraft.  The operator must land and disarm first.
  trap '' INT TERM HUP
  wait_until_disarmed_for_shutdown
  stopping=true; trap - EXIT INT TERM HUP; set +e
  if [[ "${record_bag}" == true ]]; then
    stop_named_component "诊断 rosbag" 20 || true
  fi
  echo "[停止] 按逆序停止其余全局建图组件。"
  local i pgid
  for ((i=${#pgids[@]}-1; i>=0; i--)); do
    [[ "${names[i]}" == "诊断 rosbag" ]] && continue
    kill -INT -- "-${pgids[i]}" 2>/dev/null || true
  done
  sleep 5
  for ((i=${#pgids[@]}-1; i>=0; i--)); do
    [[ "${names[i]}" == "诊断 rosbag" ]] && continue
    pgid="${pgids[i]}"
    kill -TERM -- "-${pgid}" 2>/dev/null || true
  done
  sleep 2
  for ((i=${#pgids[@]}-1; i>=0; i--)); do
    [[ "${names[i]}" == "诊断 rosbag" ]] && continue
    pgid="${pgids[i]}"
    kill -KILL -- "-${pgid}" 2>/dev/null || true
  done
  for i in "${pids[@]}"; do wait "${i}" 2>/dev/null || true; done
}
trap stop_all EXIT
trap 'exit 130' INT TERM HUP

wait_topic() {
  local topic="$1" name="$2" start="${SECONDS}"
  echo "[等待] ${topic} 真实消息。"
  while ((SECONDS-start < wait_timeout)); do
    running "${name}" || { echo "[错误] ${name} 已退出，请查看日志。" >&2; return 1; }
    timeout 3 ros2 topic echo --once "${topic}" >/dev/null 2>&1 && {
      echo "[就绪] ${topic}"; return 0;
    }
    sleep 1
  done
  echo "[错误] 等待 ${topic} 超时。" >&2
  return 1
}
wait_service() {
  local service="$1" name="$2" start="${SECONDS}"
  echo "[等待] ${service} 服务。"
  while ((SECONDS-start < wait_timeout)); do
    running "${name}" || { echo "[错误] ${name} 已退出，请查看日志。" >&2; return 1; }
    timeout 3 ros2 service type "${service}" >/dev/null 2>&1 && {
      echo "[就绪] ${service}"; return 0;
    }
    sleep 1
  done
  echo "[错误] 等待 ${service} 超时。" >&2
  return 1
}

wait_node() {
  local node="$1" name="$2" start="${SECONDS}"
  echo "[等待] ${node} 节点。"
  while ((SECONDS-start < wait_timeout)); do
    running "${name}" || { echo "[错误] ${name} 已退出，请查看日志。" >&2; return 1; }
    if ros2 node list 2>/dev/null | grep -Fxq "${node}"; then
      echo "[就绪] ${node}"
      return 0
    fi
    sleep 1
  done
  echo "[错误] 等待 ${node} 超时。" >&2
  return 1
}

wait_backend_sync() {
  local start="${SECONDS}"
  local status=""
  echo "[等待] 全局后端收到 FR-LIO 同步里程计和点云。"
  while ((SECONDS-start < wait_timeout)); do
    running "全局关键帧后端" || {
      echo "[错误] 全局关键帧后端已退出，请查看 ${log_dir}/全局后端.log。" >&2
      return 1
    }
    status="$(timeout 5 ros2 topic echo --once /fastlio_global/backend_status --field data 2>/dev/null || true)"
    if grep -Eq 'synced_callbacks=[1-9][0-9]*' <<<"${status}"; then
      echo "[就绪] 全局后端已同步：${status}"
      return 0
    fi
    sleep 1
  done
  echo "[错误] 全局后端未收到 FR-LIO 的同步 /Odometry + /cloud_registered。" >&2
  echo "[错误] 最近状态：${status:-无数据}" >&2
  return 1
}

wait_mavros_stable() {
  echo "[等待] MAVROS/EV 定位链路连续稳定 ${mavros_stable_sec} 秒。"
  if python3 "${odometry_stability_gate}" \
      --timeout-s "${wait_timeout}" \
      --stable-s "${mavros_stable_sec}" \
      --require-disarmed \
      --reject-offboard; then
    return 0
  fi
  echo "[错误] MAVROS/EV 定位链路未通过 MAVLink ODOMETRY 速度稳定门。" >&2
  tail -80 "${log_dir}/MAVROS+EV.log" >&2 || true
  return 1
}

start_component "MID-360 雷达驱动" "${log_dir}/雷达驱动.log" "${livox_env}/run_mid360_driver.sh"
wait_topic /livox/imu "MID-360 雷达驱动"
wait_topic /livox/lidar "MID-360 雷达驱动"
echo "[等待] 雷达数据就绪，${driver_delay} 秒后启动项目内 FR-LIO。"
sleep "${driver_delay}"

start_component "全局建图 FR-LIO" "${log_dir}/FR-LIO.log" \
  ros2 launch fr_lio lio.launch.py "config_file:=${frlio_config}" \
  rviz:=false lidar_accumulator:=false
wait_topic /Odometry "全局建图 FR-LIO"
wait_topic /cloud_registered "全局建图 FR-LIO"
wait_topic /frlio/high_rate_odom/status "全局建图 FR-LIO"

start_component "全局关键帧后端" "${log_dir}/全局后端.log" \
  ros2 launch fastlio_global_slam fastlio_global_slam.launch.py \
  start_fastlio:=false rviz:=false backend_config:="${backend_config}"
wait_service /fastlio_global_backend/save_map "全局关键帧后端"
wait_backend_sync

start_component "MAVROS + EV 定位链" "${log_dir}/MAVROS+EV.log" \
  ros2 launch "${autofix_launch}" \
  "fcu_url:=${fcu_url}" \
  start_mavros_odometry_bridge:=true \
  start_odom_guard:=true \
  start_ev_health_monitor:=true \
  start_tf:=true \
  require_frlio_anchor_status:=true \
  world_yaw_alignment_rad:=0.0 \
  "body_to_sensor_x_m:=${MID360_BODY_TO_SENSOR_X_M}" \
  "body_to_sensor_y_m:=${MID360_BODY_TO_SENSOR_Y_M}" \
  "body_to_sensor_z_m:=${MID360_BODY_TO_SENSOR_Z_M}" \
  "body_to_fastlio_yaw_rad:=${MID360_BODY_TO_FASTLIO_YAW_RAD}"
wait_mavros_stable

if [[ "${record_bag}" == true ]]; then
  start_component "诊断 rosbag" "${log_dir}/rosbag.log" \
    env FLIGHT_TIMESTAMP="${timestamp}" FLIGHT_RUN_DIR="${flight_run_dir}" \
      RECORD_POINTCLOUD=false RECORD_EGO_DIAGNOSTICS=false \
      "${bag_script}"
  wait_node /rosbag2_recorder "诊断 rosbag"
else
  echo "[警告] RECORD_BAG=false：本次 Position 建图不会生成链路诊断 rosbag。" >&2
fi

start_component "全局建图 RViz" "${log_dir}/RViz.log" rviz2 -d "${rviz_config}"
sleep 3
running "全局建图 RViz" || { echo "[错误] RViz 启动失败，请查看 ${log_dir}/RViz.log。" >&2; exit 3; }

cat <<EOF

============================================================
FR-LIO 全局建图和 MAVROS/EV 定位链路已经开始，后端正在接收 /Odometry + /cloud_registered，RViz 正在实时显示。
诊断 rosbag：$([[ "${record_bag}" == true ]] && printf '%s' "${flight_run_dir}/rosbag" || printf '%s' '<已禁用>')

定位稳定门已通过。现在可以由飞手手动解锁、起飞并切换到 Position 进行手飞建图。
脚本不会发送任何解锁、模式切换或飞行控制命令。若 Position 无法定点，立即切回 Stabilized。

请从规划地图起点开始，按与现场地图相同的路线缓慢走一遍，最后回到起点静止。
本次测试最低只要求 3 个关键帧，生成阈值为平移 0.15m 或旋转 5 度。
建议从起点缓慢移动 0.3-0.5m并改变朝向，然后回到起点静止。
  手飞建图完成后，必须先手动降落并上锁，再在此终端输入 1 尝试保存；输入 0 放弃。
若不足 3 个，脚本不会退出，可继续移动后再次输入 1。
============================================================
EOF
while true; do
  read -r -p "请输入操作（1=保存，0=放弃）：" action
  case "${action}" in
    1) ;;
    0) echo "[放弃] 不保存全局重定位库。"; exit 1 ;;
    *) echo "[提示] 只能输入 1 或 0。"; continue ;;
  esac

  state_before_save="$(timeout 8 ros2 topic echo --once /mavros/state mavros_msgs/msg/State 2>/dev/null || true)"
  if [[ -z "${state_before_save}" ]] || ! grep -Eq '^armed:[[:space:]]+false$' <<<"${state_before_save}"; then
    echo "[拒绝保存] 未确认飞机已上锁（armed=false）。请先手动降落并上锁，再输入 1。" >&2
    continue
  fi

  [[ ! -e "${map_dir}" ]] || { echo "[错误] 保存目录已存在，拒绝覆盖：${map_dir}" >&2; exit 4; }
  echo "[保存] 正在保存到新目录：${map_dir}"
  save_output="$(timeout 120 ros2 service call /fastlio_global_backend/save_map \
    fastlio_global_slam/srv/SaveMap \
    "{directory: '${map_dir}', resolution: ${resolution}}" 2>&1)" || {
    echo "${save_output}" >&2; echo "[错误] 保存服务调用失败。" >&2; exit 4;
  }
  printf '%s\n' "${save_output}"
  grep -Eq 'success[=:][[:space:]]*(true|True)' <<<"${save_output}" || {
    echo "[错误] 保存服务未返回 success=true。" >&2; exit 4;
  }

  metadata="${map_dir}/metadata.csv"
  global_map="${map_dir}/GlobalMap.pcd"
  keyframe_count="$(find "${map_dir}/keyframes" -maxdepth 1 -type f -name 'keyframe_*.pcd' 2>/dev/null | wc -l)"
  if [[ ! -s "${metadata}" || ! -s "${global_map}" ]]; then
    echo "[错误] metadata.csv 或 GlobalMap.pcd 不存在或为空。" >&2
    exit 5
  fi
  if ((keyframe_count < 3)); then
    echo "[不足] 当前只有 ${keyframe_count} 个关键帧，至少需要 3 个。"
    echo "[继续] 请再移动至少 0.15m 或转动至少 5 度，然后再次输入 1。"
    continue
  fi

  clipped_map="${map_dir}/GlobalMap_去顶端_z${max_z}m.pcd"
  clipped_tmp="${clipped_map}.tmp.$$.pcd"
  trap 'rm -f -- "${clipped_tmp:-}"; stop_all' EXIT INT TERM HUP
  pcl_passthrough_filter "${global_map}" "${clipped_tmp}" \
    -field z -min -100000 -max "${max_z}" -inside 1 -keep 0 >/dev/null
  mv -- "${clipped_tmp}" "${clipped_map}"
  trap stop_all EXIT
  clipped_points="$(awk '$1 == "POINTS" {print $2; exit}' "${clipped_map}")"
  [[ "${clipped_points}" =~ ^[0-9]+$ && "${clipped_points}" -gt 0 ]] || {
    echo "[错误] 去顶端点云为空或格式无效：${clipped_map}" >&2; exit 5;
  }
  echo "[完成] 去顶端点云：${clipped_map}（z <= ${max_z} m，${clipped_points} 点）"
  break
done

echo "[成功] 重定位库已保存：${map_dir}（${keyframe_count} 个关键帧）"
if [[ "${record_bag}" == true ]]; then
  echo "[证据] 退出脚本时会先干净保存诊断 rosbag：${flight_run_dir}/rosbag"
fi
if ((keyframe_count < 10)); then
  echo "[警告] 当前只有 ${keyframe_count} 个关键帧，仅用于开局单次重定位和规划线测试，不能作为实飞重定位库。"
fi
echo "[提示] 现在可以停止本脚本，再运行 一键重定位并规划验证.sh。"
