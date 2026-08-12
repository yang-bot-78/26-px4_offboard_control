#!/usr/bin/env bash
set -eo pipefail

cd /home/robot/ws_offboard_control
source /opt/ros/humble/setup.bash
source /home/robot/ws_offboard_control/install/setup.bash
set -u

params_file="${PARAMS_FILE:-}"
publish_odom_tf="${PUBLISH_ODOM_TF:-false}"
start_2_5d="${START_2_5D:-true}"
start_relocalized_tf="${START_RELOCALIZED_TF:-false}"
use_map_server="${USE_MAP_SERVER:-false}"
map_yaml="${MAP_YAML:-}"
map_frame="${MAP_FRAME:-camera_init}"
odom_frame="${ODOM_FRAME:-camera_init}"
rviz="${RVIZ:-true}"

if [[ -z "${params_file}" ]]; then
  params_file="$(ros2 pkg prefix offboard_nav2_planning)/share/offboard_nav2_planning/config/nav2_planner_pointcloud.yaml"
fi

launch_args=(
  "params_file:=${params_file}"
  "publish_odom_tf:=${publish_odom_tf}"
  "start_2_5d:=${start_2_5d}"
  "start_relocalized_tf:=${start_relocalized_tf}"
  "use_map_server:=${use_map_server}"
  "map_frame:=${map_frame}"
  "odom_frame:=${odom_frame}"
  "rviz:=${rviz}"
)

if [[ -n "${map_yaml}" ]]; then
  launch_args+=("map:=${map_yaml}")
fi

exec ros2 launch offboard_nav2_planning nav2_planner_only.launch.py "${launch_args[@]}"
