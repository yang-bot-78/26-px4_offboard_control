#!/usr/bin/env bash
set -euo pipefail

# 工程最小入口：编译 src 与 Lin_shi 中的软件包，再启动无 MAVROS 的规划入口。
# 启动模式：sim（默认，race_sim）或 planner（只启动 Super Planner）。

project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
livox_ws="${LIVOX_WS:-/home/robot/livox_mid360_env/ws_livox}"
livox_trace_prefix="${LIVOX_TRACE_PREFIX:-/home/robot/ws_offboard_control/install/livox_trace_control_interfaces}"
start_mode="${START_MODE:-sim}"

case "${start_mode}" in
  sim|planner) ;;
  *)
    echo "用法：START_MODE=sim|planner $0" >&2
    exit 2
    ;;
esac

cd "${project_root}"
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
if [[ ! -f "${livox_trace_prefix}/share/livox_trace_control_interfaces/package.bash" ]]; then
  echo "未找到 Livox trace 接口：${livox_trace_prefix}" >&2
  echo "请设置 LIVOX_TRACE_PREFIX 为 livox_trace_control_interfaces 的安装前缀。" >&2
  exit 1
fi
if [[ ! -f "${livox_ws}/install/setup.bash" ]]; then
  echo "未找到 Livox ROS 2 安装树：${livox_ws}/install/setup.bash" >&2
  echo "请设置 LIVOX_WS 为包含 install/setup.bash 的工作区。" >&2
  exit 1
fi
# FR-LIO depends on livox_ros_driver2, which is maintained in this external
# workspace rather than copied into this repository.
source "${livox_trace_prefix}/share/livox_trace_control_interfaces/package.bash"
source "${livox_ws}/install/setup.bash"
export COLCON_TRACE="${COLCON_TRACE:-}"
set -u
export PYTHONNOUSERSITE=1
export WS_OFFBOARD_CONTROL_ROOT="${project_root}"

echo "[编译] 构建 src 与 Lin_shi 中的 ROS 2 软件包。"
PYTHONNOUSERSITE=1 colcon build --base-paths src Lin_shi --symlink-install

set +u
# shellcheck disable=SC1091
source "${project_root}/install/setup.bash"
set -u

echo "[启动] 模式=${start_mode}"
case "${start_mode}" in
  sim)
    echo "[提示] 只启动 race_sim 规划入口，不启动 MAVROS，不连接 PX4。"
    exec ros2 launch race_bringup race_sim.launch.py \
      use_sim_time:=false \
      control_source:=navigation \
      require_strict_local_position_health:=false
    ;;
  planner)
    echo "[提示] 只启动 Super Planner，控制输出关闭。"
    exec ros2 launch race_super_planner_ros2 super_planner_ros2.launch.py \
      use_sim_time:=false \
      enable_output:=false \
      global_only_mode:=true \
      publish_global_path:=true \
      publish_ego_local_goal:=false \
      odom_topic:=/planning/odom \
      fallback_odom_topic:=/planning/odom \
      cloud_topic:=/saved_map \
      fallback_cloud_topic:=/saved_map \
      require_mavros_connected:=false
    ;;
esac
