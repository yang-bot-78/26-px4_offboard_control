#!/usr/bin/env bash
set -eo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
workspace_root="$(cd -- "${script_dir}/../.." && pwd -P)"
cd "${workspace_root}"
source /opt/ros/humble/setup.bash
source "${workspace_root}/install/setup.bash"
set -u
export WS_OFFBOARD_CONTROL_ROOT="${workspace_root}"

params_file="${PARAMS_FILE:-$(ros2 pkg prefix offboard_nav2_planning)/share/offboard_nav2_planning/config/nav2_planner_relocalized_map.yaml}"
map_frame="${MAP_FRAME:-map}"
odom_frame="${ODOM_FRAME:-camera_init}"

cleanup() {
  for pid in "${pids[@]:-}"; do
    if kill -0 "${pid}" 2>/dev/null; then
      kill "${pid}" 2>/dev/null || true
    fi
  done
}
trap cleanup INT TERM EXIT

pids=()

echo "Starting /nav2_stage1_goal_to_path helper..."
ros2 run offboard_nav2_planning goal_to_path --ros-args --params-file "${params_file}" &
pids+=("$!")

echo "Starting /nav2_stage1_odometry_tf_publisher helper..."
ros2 run offboard_nav2_planning odometry_tf_publisher --ros-args --params-file "${params_file}" &
pids+=("$!")

if ! ros2 node list 2>/dev/null | grep -qx "/nav2_relocalized_pose_to_tf"; then
  echo "Starting /nav2_relocalized_pose_to_tf helper..."
  ros2 run offboard_nav2_planning relocalized_pose_to_tf --ros-args \
    -p map_frame:="${map_frame}" \
    -p odom_frame:="${odom_frame}" &
  pids+=("$!")
else
  echo "/nav2_relocalized_pose_to_tf already exists; not starting another one."
fi

echo "Nav2 helper nodes are running. Keep this terminal open."
wait
