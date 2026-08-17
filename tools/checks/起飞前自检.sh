#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
project_root="$(cd -- "${script_dir}/../.." && pwd -P)"

quiet=false

if [[ "${1:-}" == "--quiet" ]]; then
  quiet=true
fi

say() {
  if [[ "${quiet}" != true ]]; then
    echo "$@"
  fi
}

require_file() {
  local path="$1"
  if [[ ! -f "${path}" ]]; then
    echo "缺少必需文件：${path}" >&2
    exit 1
  fi
}

require_executable() {
  local path="$1"
  require_file "${path}"
  if [[ ! -x "${path}" ]]; then
    echo "必需文件没有可执行权限：${path}" >&2
    exit 1
  fi
}

require_command() {
  local name="$1"
  if ! command -v "${name}" >/dev/null 2>&1; then
    echo "缺少命令：${name}" >&2
    exit 1
  fi
}

# Usage: require_ros_executable <package> <executable>
require_ros_executable() {
  local package="$1"
  local executable="$2"
  if ! ros2 pkg executables "${package}" | awk '{print $2}' | grep -Fxq "${executable}"; then
    echo "${package} 的可执行文件未安装：${executable}" >&2
    exit 1
  fi
}

say "Checking offboard autostart prerequisites..."

require_command gnome-terminal
require_command systemctl
require_command python3
require_command timeout

require_file "/opt/ros/humble/setup.bash"
require_file "${project_root}/install/setup.bash"
require_executable "${project_root}/tools/flight/一键启动导航栈.sh"
require_executable "${project_root}/tools/flight/一键启动起飞栈.sh"
require_executable "${project_root}/tools/flight/开机自启动起飞栈.sh"
require_executable "${project_root}/tools/flight/运行导航控制.sh"
require_executable "${project_root}/tools/flight/检查PX4视觉ULog.sh"
require_executable "${project_root}/tools/rosbag/开始录包.sh"
require_executable "${project_root}/tools/rosbag/停止录包.sh"
require_executable "${project_root}/tools/rosbag/刷新录包状态.sh"
require_file "${project_root}/tools/rosbag/ws_offboard_rosbag_shutdown.service"
require_executable "${HOME}/livox_mid360_env/run_mid360_driver.sh"
lio_backend="${LIO_BACKEND:-fr_lio}"
frlio_config="${FRLIO_CONFIG:-${project_root}/src/fr_lio/config/indoors.yaml}"
case "${lio_backend}" in
  fr_lio) require_file "${frlio_config}" ;;
  fast_lio) require_executable "${HOME}/livox_mid360_env/run_fastlio_mid360.sh" ;;
  *) echo "LIO_BACKEND 必须是 fr_lio 或 fast_lio：${lio_backend}" >&2; exit 1 ;;
esac

launch_source="${project_root}/src/px4_ros_com/launch/fastlio_mavros_autofix.launch.py"
stack_script="${project_root}/tools/flight/一键启动起飞栈.sh"
full_stack_script="${project_root}/tools/flight/一键启动导航栈.sh"
managed_component_runner="${project_root}/tools/flight/受管组件窗口.sh"
ev_contract_helper="${project_root}/src/px4_ros_com/test/ev_entry_contract_static.py"
lever_arm_config="${MID360_LEVER_ARM_CONFIG:-${project_root}/src/px4_ros_com/config/mid360_lever_arm.conf}"
lever_arm_validator="${project_root}/tools/fastlio/校验杆臂配置.sh"
readiness_helper="${project_root}/tools/flight/等待ROS就绪.sh"

require_file "${launch_source}"
require_file "${stack_script}"
require_file "${ev_contract_helper}"
require_executable "${lever_arm_validator}"
require_executable "${readiness_helper}"
require_executable "${managed_component_runner}"
require_file "${lever_arm_config}"
"${lever_arm_validator}" "${lever_arm_config}"
python3 "${ev_contract_helper}" --launch "${launch_source}" --stack "${stack_script}"

# 统一入口刻意单独检查：它的进程编排不是上面 EV AST 契约检查器所消费的那种
# 旧版 open_window 命令形态。安全关键的 launch 绑定要保持显式。
# shellcheck disable=SC2016
for required_token in \
  'ros2 launch px4_ros_com fastlio_mavros_autofix.launch.py' \
  'start_mavros_odometry_bridge:=true' \
  'start_px4_vehicle_odometry:=false' \
  'start_odom_guard:=true' \
  'start_ev_health_monitor:=true' \
  'start_tf:=true' \
  'gnome-terminal --window --wait' \
  '受管组件窗口.sh' \
  'kill -TERM -- "-${pgid}"' \
  'kill -KILL -- "-${pgid}"' \
  'world_yaw_alignment_rad:=' \
  "body_to_sensor_x_m:=\${MID360_BODY_TO_SENSOR_X_M}" \
  "body_to_sensor_y_m:=\${MID360_BODY_TO_SENSOR_Y_M}" \
  "body_to_sensor_z_m:=\${MID360_BODY_TO_SENSOR_Z_M}" \
  "body_to_fastlio_yaw_rad:=\${MID360_BODY_TO_FASTLIO_YAW_RAD}" \
  'wait_component_ready "mid360_driver" message /livox/imu' \
  'wait_component_ready "mid360_driver" message /livox/lidar' \
  'sleep "${mid360_fastlio_delay_sec}"' \
  'wait_component_ready "fastlio" message /Odometry' \
  'wait_component_ready "px4_mavros" message /mavros/local_position/odom' \
  'wait_component_ready "px4_mavros" message /mavros/local_position/velocity_local' \
  'wait_component_ready "px4_mavros" flight_ready /ev_health/flight_ready' \
  'wait_component_ready "px4_mavros" message /Odometry/healthy' \
  'wait_component_ready "px4_mavros" message /mavros/odometry/out' \
  'wait_component_ready "navigation" message /race/odom'; do
  if ! grep -Fq "${required_token}" "${full_stack_script}"; then
    echo "完整栈缺少必需的链路契约：${required_token}" >&2
    exit 1
  fi
done

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "${project_root}/install/setup.bash"
set -u

require_command ros2
# EV 链（px4_ros_com）—— 导航栈融合没有改动这部分。
require_ros_executable px4_ros_com fastlio_odometry_guard
require_ros_executable px4_ros_com fastlio_ev_health_monitor.py
require_ros_executable px4_ros_com fastlio_mavros_odometry_bridge
require_ros_executable fr_lio frlio

# 导航栈。offboard_waypoint_node 已取代 minipc_mavros_offboard.py，成为
# /mavros/setpoint_raw/local 的唯一发布者。
require_ros_executable race_offboard offboard_waypoint_node
require_ros_executable race_offboard mission_sequencer_node.py
require_ros_executable race_super_planner_ros2 super_planner_ros2_node
require_ros_executable race_ego_bridge race_ego_odom_bridge
require_ros_executable race_ego_bridge race_ego_cloud_bridge
require_ros_executable race_ego_bridge race_ego_goal_bridge
require_ros_executable race_ego_bridge race_ego_trajectory_bridge
require_ros_executable ego_planner ego_planner_node
require_ros_executable ego_planner traj_server
require_ros_executable race_mapping fastlio_bridge_node
require_ros_executable race_mapping map_io_node

launch_arguments="$(
  ros2 launch px4_ros_com fastlio_mavros_autofix.launch.py --show-args
)"

if [[ "${launch_arguments}" != *"world_yaw_alignment_rad"* ]]; then
  echo "fastlio_mavros_autofix.launch.py 没有暴露统一的 world_yaw_alignment_rad" >&2
  exit 1
fi

if [[ "${launch_arguments}" != *"body_to_sensor_x_m"* ]]; then
  echo "fastlio_mavros_autofix.launch.py 没有暴露 MID360 杆臂参数" >&2
  exit 1
fi

if [[ "${launch_arguments}" != *"ev_max_velocity_difference_mps"* ]]; then
  echo "fastlio_mavros_autofix.launch.py 没有暴露 EV 健康门限参数" >&2
  exit 1
fi

if [[ "${launch_arguments}" != *"ev_max_internal_velocity_difference_mps"* ]]; then
  echo "fastlio_mavros_autofix.launch.py 没有暴露 EV 内部速度一致性门限参数" >&2
  exit 1
fi

if [[ ! -e /dev/ttyUSB0 ]]; then
  echo "警告：/dev/ttyUSB0 还不存在；在飞控串口出现前 MAVROS 可能一直等待或失败" >&2
fi

"${project_root}/tools/rosbag/刷新录包状态.sh" || true

say "Autostart prerequisites look ready."
