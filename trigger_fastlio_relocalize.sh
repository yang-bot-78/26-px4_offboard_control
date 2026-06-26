#!/usr/bin/env bash
set -eo pipefail

cd /home/robot/ws_offboard_control
source /opt/ros/humble/setup.bash
source /home/robot/ws_offboard_control/install/setup.bash
set -u

exec ros2 service call /fastlio_global_backend/relocalize fastlio_global_slam/srv/Relocalize \
  "{use_latest_scan: true}"
