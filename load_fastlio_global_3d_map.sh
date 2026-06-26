#!/usr/bin/env bash
set -eo pipefail

cd /home/robot/ws_offboard_control
source /opt/ros/humble/setup.bash
source /home/robot/ws_offboard_control/install/setup.bash
set -u

map_dir="${FASTLIO_GLOBAL_MAP_DIR:-/home/robot/ws_offboard_control/maps/fastlio_global_3d}"

exec ros2 service call /fastlio_global_backend/load_map fastlio_global_slam/srv/LoadMap \
  "{directory: '${map_dir}'}"
