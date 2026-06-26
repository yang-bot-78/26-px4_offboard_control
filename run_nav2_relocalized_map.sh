#!/usr/bin/env bash
set -eo pipefail

cd /home/robot/ws_offboard_control
source /opt/ros/humble/setup.bash
source /home/robot/ws_offboard_control/install/setup.bash
set -u

if [[ "${ALLOW_DUPLICATE_NAV2:-false}" != "true" ]]; then
  if ros2 node list 2>/dev/null | grep -qx "/planner_server"; then
    echo "Nav2 planner stack already appears to be running."
    echo "Stop the old run first, or use ALLOW_DUPLICATE_NAV2=true if you really need another instance."
    exit 2
  fi
fi

export PARAMS_FILE="${PARAMS_FILE:-/home/robot/ws_offboard_control/install/offboard_nav2_planning/share/offboard_nav2_planning/config/nav2_planner_relocalized_map.yaml}"
export USE_MAP_SERVER="${USE_MAP_SERVER:-true}"
export MAP_YAML="${MAP_YAML:-/home/robot/ws_offboard_control/maps/fastlio_nav2_map.yaml}"
export MAP_FRAME="${MAP_FRAME:-map}"
export ODOM_FRAME="${ODOM_FRAME:-camera_init}"
export PUBLISH_ODOM_TF="${PUBLISH_ODOM_TF:-true}"
export START_RELOCALIZED_TF="${START_RELOCALIZED_TF:-true}"
# The current 2.5D lifter samples /cloud_registered in camera_init. Keep it off
# for relocalized-map validation until the map->camera_init TF has been verified.
export START_2_5D="${START_2_5D:-false}"

if [[ -z "${RVIZ:-}" ]]; then
  if pgrep -x rviz2 >/dev/null; then
    export RVIZ=false
  else
    export RVIZ=true
  fi
fi

exec /home/robot/ws_offboard_control/run_nav2_stage1_planning.sh
