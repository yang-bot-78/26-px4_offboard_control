#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
project_root="$(cd -- "${script_dir}/../.." && pwd -P)"

driver_delay="${DRIVER_DELAY_SEC:-1}"
fastlio_delay="${FASTLIO_DELAY_SEC:-2}"
mavros_delay="${MAVROS_DELAY_SEC:-3}"
bag_delay="${BAG_DELAY_SEC:-2}"
readiness_timeout_sec="${READINESS_TIMEOUT_SEC:-90}"
world_yaw_alignment_rad="${WORLD_YAW_ALIGNMENT_RAD:-1.57079632679}"
flight_timestamp="${FLIGHT_TIMESTAMP:-$(date +%Y%m%d_%H%M%S)}"
flight_date="${flight_timestamp%%_*}"
flight_run_dir="${FLIGHT_RUN_DIR:-${project_root}/flight_records/${flight_date}/flight_${flight_timestamp}}"
flight_log_dir="${flight_run_dir}/logs"
user_systemd_dir="${HOME}/.config/systemd/user"
service_name="ws_offboard_rosbag_shutdown.service"
service_src="${project_root}/tools/rosbag/${service_name}"
service_dst="${user_systemd_dir}/${service_name}"
lever_arm_config="${MID360_LEVER_ARM_CONFIG:-${project_root}/src/px4_ros_com/config/mid360_lever_arm.conf}"
lever_arm_validator="${project_root}/tools/fastlio/校验杆臂配置.sh"
readiness_helper="${project_root}/tools/flight/等待ROS就绪.sh"
env_cleanup="${project_root}/tools/flight/清理运行环境.sh"

require_file() {
  local path="$1"
  if [[ ! -e "$path" ]]; then
    echo "缺少必需文件：$path" >&2
    exit 1
  fi
}

open_window() {
  local title="$1"
  local log_name="$2"
  local cmd="$3"
  gnome-terminal --title="$title" -- bash -lc "export FLIGHT_TIMESTAMP='${flight_timestamp}'; export FLIGHT_RUN_DIR='${flight_run_dir}'; mkdir -p '${flight_log_dir}'; { $cmd; } 2>&1 | tee -a '${flight_log_dir}/${log_name}.log'; status=\${PIPESTATUS[0]}; echo; echo '[${title}] exited with status' \${status}; exec bash" &
}

require_file "${HOME}/livox_mid360_env/run_mid360_driver.sh"
require_file "${HOME}/livox_mid360_env/run_fastlio_mid360.sh"
require_file "${project_root}/tools/rosbag/开始录包.sh"
require_file "${project_root}/tools/rosbag/刷新录包状态.sh"
require_file "${project_root}/tools/rosbag/查看最近录包.sh"
require_file "${project_root}/tools/rosbag/停止录包.sh"
require_file "${project_root}/tools/flight/运行导航控制.sh"
require_file "${project_root}/tools/checks/起飞前自检.sh"
require_file "${lever_arm_validator}"
require_file "${readiness_helper}"
require_file "${env_cleanup}"
require_file "${lever_arm_config}"
require_file "${project_root}/install/setup.bash"
require_file "${service_src}"

"${lever_arm_validator}" "${lever_arm_config}"
source "${lever_arm_config}"

if [[ ! "${readiness_timeout_sec}" =~ ^[0-9]+$ ]] || (( readiness_timeout_sec < 1 )); then
  echo "READINESS_TIMEOUT_SEC 必须是正整数" >&2
  exit 1
fi

if ! command -v gnome-terminal >/dev/null 2>&1; then
  echo "找不到 gnome-terminal" >&2
  exit 1
fi

# 启动前确保环境干净。本脚本会为全部五个组件各开一个窗口，所以按 all 清理：
# 上一次残留的驱动或 FAST-LIO 会让新起的链路接到一个已经积压追赶的里程计上。
echo "启动前环境清理..."
if ! "${env_cleanup}" all; then
  echo "启动前环境清理失败；请处理上面列出的进程后重试。" >&2
  exit 1
fi

mkdir -p "${flight_run_dir}"
mkdir -p "${flight_log_dir}"
mkdir -p "${user_systemd_dir}"
cp "${lever_arm_config}" "${flight_run_dir}/mid360_lever_arm.conf"
cd "${project_root}"

if [[ "${SKIP_PREFLIGHT_CHECK:-0}" != "1" ]]; then
  "${project_root}/tools/checks/起飞前自检.sh"
fi

"${project_root}/tools/rosbag/刷新录包状态.sh"
cp "${service_src}" "${service_dst}"
chmod 644 "${service_dst}"
systemctl --user daemon-reload || true
systemctl --user enable --now "${service_name}" >/dev/null 2>&1 || true
echo "本场次目录：${flight_run_dir}"
echo "本场次日志目录：${flight_log_dir}"
echo "导航栈：按 astar_ego_tuning.yaml 选择有效后端（EGO 关闭时自动使用 Super A*） + Offboard（MAVROS）。漂移保护与起飞瞬态守卫均未生效 —— 请全程握住遥控器。"
echo "MID360 机体到传感器杆臂（FLU）：x=${MID360_BODY_TO_SENSOR_X_M} m y=${MID360_BODY_TO_SENSOR_Y_M} m z=${MID360_BODY_TO_SENSOR_Z_M} m yaw=${MID360_BODY_TO_FASTLIO_YAW_RAD} rad"

open_window "MID360 Driver" "mid360_driver" \
  "cd '${HOME}/livox_mid360_env' && ./run_mid360_driver.sh"

open_window "FAST-LIO" "fastlio" \
  "sleep ${driver_delay}; source /opt/ros/humble/setup.bash && '${readiness_helper}' topic '${readiness_timeout_sec}' /livox/imu && '${readiness_helper}' topic '${readiness_timeout_sec}' /livox/lidar && cd '${HOME}/livox_mid360_env' && ./run_fastlio_mid360.sh"

open_window "PX4 MAVROS" "px4_mavros" \
  "sleep $((driver_delay + fastlio_delay)); source /opt/ros/humble/setup.bash && source '${project_root}/install/setup.bash' && '${readiness_helper}' topic '${readiness_timeout_sec}' /Odometry && cd '${project_root}' && ros2 launch px4_ros_com fastlio_mavros_autofix.launch.py fcu_url:=serial:///dev/ttyUSB0:921600?ids=255,190 start_mavros_vision_bridge:=true world_yaw_alignment_rad:=${world_yaw_alignment_rad} body_to_sensor_x_m:=${MID360_BODY_TO_SENSOR_X_M} body_to_sensor_y_m:=${MID360_BODY_TO_SENSOR_Y_M} body_to_sensor_z_m:=${MID360_BODY_TO_SENSOR_Z_M} body_to_fastlio_yaw_rad:=${MID360_BODY_TO_FASTLIO_YAW_RAD}"

open_window "ROS Bag Debug" "rosbag_debug" \
  "sleep $((driver_delay + fastlio_delay + mavros_delay)); source /opt/ros/humble/setup.bash && '${readiness_helper}' mavros_connected '${readiness_timeout_sec}' && '${readiness_helper}' topic '${readiness_timeout_sec}' /ev_health/status && cd '${project_root}' && '${project_root}/tools/rosbag/开始录包.sh'"

open_window "Navigation" "navigation" \
  "sleep $((driver_delay + fastlio_delay + mavros_delay + bag_delay)); source /opt/ros/humble/setup.bash && '${readiness_helper}' mavros_connected '${readiness_timeout_sec}' && '${readiness_helper}' topic '${readiness_timeout_sec}' /mavros/local_position/odom && '${readiness_helper}' healthy '${readiness_timeout_sec}' /ev_health/status && cd '${project_root}' && WORLD_YAW_ALIGNMENT_RAD='${world_yaw_alignment_rad}' '${project_root}/tools/flight/运行导航控制.sh'"

wait
