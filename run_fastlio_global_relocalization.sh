#!/usr/bin/env bash
set -eo pipefail

cd /home/robot/ws_offboard_control
source /opt/ros/humble/setup.bash
source /home/robot/ws_offboard_control/install/setup.bash
set -u

export BACKEND_CONFIG="${BACKEND_CONFIG:-/home/robot/ws_offboard_control/install/fastlio_global_slam/share/fastlio_global_slam/config/relocalization.yaml}"
export RVIZ="${RVIZ:-true}"
export START_FASTLIO="${START_FASTLIO:-false}"

exec /home/robot/ws_offboard_control/run_fastlio_global_slam.sh
