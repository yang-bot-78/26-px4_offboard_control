#!/usr/bin/env bash
set -eo pipefail

# 启动导航栈：Super A* 全局规划 + EGO B-spline 局部避障 + Offboard 状态机，
# 全部经由 MAVROS。
#
# 本脚本取代 run_takeoff_1m_hold.sh（已于 2026-08-09 与 minipc_mavros_offboard.py
# 一并删除）。现在 offboard_waypoint_node 是唯一的
# publisher of /mavros/setpoint_raw/local.
#
# 安全提示 —— 飞行前必读：
#   minipc_mavros_offboard.py 提供的水平漂移保护和起飞瞬态守卫**没有**被移植过来。
#   本节点有 EV 健康门禁（解锁前连续 HEALTHY 7.5s，飞行中 FAULT -> AUTO.LAND）和
#   六层高度保护，但没有任何东西会对横向漂移或糟糕的起飞瞬态做出反应。
#   请全程握住遥控器随时接管。详见 docs/NAVIGATION_MERGE.md。

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
project_root="$(cd -- "${script_dir}/../.." && pwd -P)"

cd "${project_root}"
source /opt/ros/humble/setup.bash
source "${project_root}/install/setup.bash"
set -u

# astar_ego：Super A* 做全局规划并持续给 EGO 喂滚动局部目标；EGO 负责局部避障
# 并驱动飞机。ego_planner.enable=false 时，EGO 类请求统一降级为 Super。
requested_planner_backend="${PLANNER_BACKEND:-astar_ego}"
planner_backend="${requested_planner_backend}"
tuning_file="${TUNING_FILE:-${project_root}/src/race_bringup/config/astar_ego_tuning.yaml}"
# 任务序列器执行固定的 A->B->C->D 比赛航点序列。
mission_enabled="${MISSION_ENABLED:-true}"
# 赛场扫描图。race_mapping.launch.py 里的默认值是一张遗留地图，**不是**比赛场地 ——
# 请在这里传入真实的赛场扫描图。
map_file="${MAP_FILE:-}"
map_auto_load="${MAP_AUTO_LOAD:-true}"
rviz="${RVIZ:-true}"
# 目标识别（bs/）。默认关闭：它需要 RealSense 和 YOLO 权重，而缺依赖这种问题
# 不该在飞行途中才暴露出来。
recognition_enabled="${RECOGNITION_ENABLED:-false}"
enable_output="${ENABLE_OUTPUT:-true}"
manual_handover="${MANUAL_HANDOVER:-false}"
ev_fault_auto_land="${EV_FAULT_AUTO_LAND:-true}"
body_to_sensor_x_m="${BODY_TO_SENSOR_X_M:-0.0}"
body_to_sensor_y_m="${BODY_TO_SENSOR_Y_M:-0.0}"
body_to_sensor_z_m="${BODY_TO_SENSOR_Z_M:-0.08}"
body_to_fastlio_yaw_rad="${BODY_TO_FASTLIO_YAW_RAD:-0.0}"
world_yaw_alignment_rad="${WORLD_YAW_ALIGNMENT_RAD:-0.0}"
publish_fastlio_bridge="${PUBLISH_FASTLIO_BRIDGE:-true}"
publish_camera_init_tf="${PUBLISH_CAMERA_INIT_TF:-true}"
# The relocalization bridge owns map -> camera_init.  Its raw /Odometry frame
# remains local, so do not also advertise that raw odom as an identity child of map.
broadcast_map_to_odom="${PUBLISH_MAP_TO_ODOM_TF:-${publish_camera_init_tf}}"
map_frame_id="${MAP_FRAME_ID:-map}"
fast_lio_odom_topic="${FASTLIO_ODOM_TOPIC:-/Odometry}"
require_map_local_alignment="${REQUIRE_MAP_LOCAL_ALIGNMENT:-false}"

for boolean_name in mission_enabled rviz recognition_enabled enable_output map_auto_load manual_handover ev_fault_auto_land broadcast_map_to_odom; do
  boolean_value="${!boolean_name}"
  if [[ "${boolean_value}" != true && "${boolean_value}" != false ]]; then
    echo "${boolean_name^^} must be true or false; got: ${boolean_value}" >&2
    exit 1
  fi
done

case "${requested_planner_backend}" in
  astar_ego|super|ego|ego-shadow) ;;
  *)
    echo "PLANNER_BACKEND 必须是 astar_ego、super、ego 或 ego-shadow" >&2
    exit 1
    ;;
esac

[[ -f "${tuning_file}" ]] || { echo "TUNING_FILE 不存在：${tuning_file}" >&2; exit 1; }
if ! ego_enabled="$(python3 - "${project_root}" "${tuning_file}" <<'PY'
import sys

project_root, tuning_file = sys.argv[1:]
sys.path.insert(0, f'{project_root}/src/race_bringup/launch')
from astar_ego_tuning import load_tuning

print(str(load_tuning(tuning_file)['ego_planner']['enable']).lower())
PY
)"; then
  echo "无法从 TUNING_FILE 读取 EGO 开关：${tuning_file}" >&2
  exit 1
fi
if [[ "${ego_enabled}" == false ]]; then
  case "${requested_planner_backend}" in
    astar_ego|ego|ego-shadow) planner_backend=super ;;
  esac
fi

launch_args=(
  "planner_backend:=${planner_backend}"
  "tuning_file:=${tuning_file}"
  "mission_enabled:=${mission_enabled}"
  "rviz:=${rviz}"
  # 真机：使用墙上时钟，且 MAVROS 由整栈脚本里的
  # fastlio_mavros_autofix.launch.py 单独启动。
  "use_sim_time:=false"
  "recognition_enabled:=${recognition_enabled}"
  "enable_output:=${enable_output}"
  "manual_handover:=${manual_handover}"
  "ev_fault_auto_land:=${ev_fault_auto_land}"
  "body_to_sensor_x_m:=${body_to_sensor_x_m}"
  "body_to_sensor_y_m:=${body_to_sensor_y_m}"
  "body_to_sensor_z_m:=${body_to_sensor_z_m}"
  "body_to_fastlio_yaw_rad:=${body_to_fastlio_yaw_rad}"
  "world_yaw_alignment_rad:=${world_yaw_alignment_rad}"
  "publish_fastlio_bridge:=${publish_fastlio_bridge}"
  "publish_camera_init_tf:=${publish_camera_init_tf}"
  "broadcast_map_to_odom:=${broadcast_map_to_odom}"
  "map_frame_id:=${map_frame_id}"
  "fast_lio_odom_topic:=${fast_lio_odom_topic}"
  "require_map_local_alignment:=${require_map_local_alignment}"
)
if [[ -n "${map_file}" ]]; then
  if [[ ! -f "${map_file}" ]]; then
    echo "MAP_FILE 不存在：${map_file}" >&2
    exit 1
  fi
  launch_args+=("map_file:=${map_file}")
fi
launch_args+=("map_auto_load:=${map_auto_load}")

echo "导航后端：    请求=${requested_planner_backend}，实际=${planner_backend}，EGO开关=${ego_enabled}"
echo "任务序列器：  ${mission_enabled}"
if [[ -z "${map_file}" && "${map_auto_load}" == false ]]; then
  echo "地图文件：    <链路验证模式已关闭地图>"
else
  echo "地图文件：    ${map_file:-<race_mapping 默认图，不是赛场扫描图>}"
fi
echo "地图自动加载：${map_auto_load}"
echo "Recognition:        ${recognition_enabled}"
echo "导航输出：    ${enable_output}"
echo "人工接管：    ${manual_handover}"
echo "EV 故障自动降落：${ev_fault_auto_land}"

exec ros2 launch race_bringup navigation.launch.py "${launch_args[@]}"
