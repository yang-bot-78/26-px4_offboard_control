#!/usr/bin/env bash
set -euo pipefail

# 备份工程的最小入口：先从 src 编译，再启动无 MAVROS 的规划入口。
# 启动模式：sim（默认，race_sim）或 planner（只启动 Super Planner）。

project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
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
export COLCON_TRACE="${COLCON_TRACE:-}"
set -u
export PYTHONNOUSERSITE=1

echo "[编译] 清洁备份不带 build/install/log，开始编译 src。"
PYTHONNOUSERSITE=1 colcon build --base-paths src --symlink-install

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
