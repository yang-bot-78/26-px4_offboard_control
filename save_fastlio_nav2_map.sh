#!/usr/bin/env bash
set -eo pipefail

cd /home/robot/ws_offboard_control
source /opt/ros/humble/setup.bash
source /home/robot/ws_offboard_control/install/setup.bash
set -u

cloud_topic="${CLOUD_TOPIC:-/fastlio_global/map}"
output_yaml="${OUTPUT_YAML:-/home/robot/ws_offboard_control/maps/fastlio_nav2_map.yaml}"
resolution="${MAP_RESOLUTION:-0.10}"
min_z="${MAP_MIN_Z:--0.20}"
max_z="${MAP_MAX_Z:-2.00}"
padding_m="${MAP_PADDING_M:-1.0}"
occupied_dilation_m="${MAP_OCCUPIED_DILATION_M:-0.20}"
timeout_s="${MAP_TIMEOUT_S:-10.0}"
ground_filter="${MAP_GROUND_FILTER:-true}"
ground_percentile="${MAP_GROUND_PERCENTILE:-0.15}"
obstacle_min_height="${MAP_OBSTACLE_MIN_HEIGHT_M:-0.25}"
min_points_per_cell="${MAP_MIN_POINTS_PER_CELL:-2}"

exec ros2 run offboard_nav2_planning save_2d_map_from_cloud --ros-args \
  -p cloud_topic:="${cloud_topic}" \
  -p output_yaml:="${output_yaml}" \
  -p resolution:="${resolution}" \
  -p min_z:="${min_z}" \
  -p max_z:="${max_z}" \
  -p padding_m:="${padding_m}" \
  -p occupied_dilation_m:="${occupied_dilation_m}" \
  -p timeout_s:="${timeout_s}" \
  -p ground_filter:="${ground_filter}" \
  -p ground_percentile:="${ground_percentile}" \
  -p obstacle_min_height_above_floor_m:="${obstacle_min_height}" \
  -p min_points_per_cell:="${min_points_per_cell}"
