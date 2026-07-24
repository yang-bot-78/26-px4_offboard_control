#!/usr/bin/env bash
set -eo pipefail

cd /home/robot/ws_offboard_control
source /opt/ros/humble/setup.bash
source /home/robot/ws_offboard_control/install/setup.bash
set -u

export BACKEND_CONFIG="${BACKEND_CONFIG:-/home/robot/ws_offboard_control/install/fastlio_global_slam/share/fastlio_global_slam/config/relocalization.yaml}"
# Relocalization backend should not open the FAST-LIO RViz by default.
# Nav2 route validation uses nav2_stage1_planning.rviz, started later by run_nav2_relocalized_map.sh.
export RVIZ="${RVIZ:-false}"
export START_FASTLIO="${START_FASTLIO:-false}"

exec /home/robot/ws_offboard_control/run_fastlio_global_slam.sh
