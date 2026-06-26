#!/usr/bin/env bash
set -eo pipefail

patterns=(
  "nav2_planner.*planner_server"
  "nav2_map_server.*map_server"
  "nav2_lifecycle_manager.*lifecycle_manager_(planner|map)"
  "offboard_nav2_planning.*goal_to_path"
  "offboard_nav2_planning.*odometry_tf_publisher"
  "offboard_nav2_planning.*relocalized_pose_to_tf"
  "offboard_nav2_planning.*path_2_5d_lifter"
)

if [[ "${STOP_RVIZ:-false}" == "true" ]]; then
  patterns+=("rviz2")
fi

for pattern in "${patterns[@]}"; do
  pkill -f "${pattern}" 2>/dev/null || true
done

echo "Requested stop for Nav2 relocalized-map nodes."
