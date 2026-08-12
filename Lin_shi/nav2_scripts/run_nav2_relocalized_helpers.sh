#!/usr/bin/env bash
set -eo pipefail

cd /home/robot/ws_offboard_control
source /opt/ros/humble/setup.bash
source /home/robot/ws_offboard_control/install/setup.bash
set -u

params_file="${PARAMS_FILE:-/home/robot/ws_offboard_control/install/offboard_nav2_planning/share/offboard_nav2_planning/config/nav2_planner_relocalized_map.yaml}"
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
python3 /home/robot/ws_offboard_control/src/offboard_nav2_planning/offboard_nav2_planning/goal_to_path.py --ros-args \
  --params-file "${params_file}" &
pids+=("$!")

echo "Starting /nav2_stage1_odometry_tf_publisher helper..."
python3 /home/robot/ws_offboard_control/src/offboard_nav2_planning/offboard_nav2_planning/odometry_tf_publisher.py --ros-args \
  --params-file "${params_file}" &
pids+=("$!")

if ! ros2 node list 2>/dev/null | grep -qx "/nav2_relocalized_pose_to_tf"; then
  echo "Starting /nav2_relocalized_pose_to_tf helper..."
  python3 /home/robot/ws_offboard_control/src/offboard_nav2_planning/offboard_nav2_planning/relocalized_pose_to_tf.py --ros-args \
    -p map_frame:="${map_frame}" \
    -p odom_frame:="${odom_frame}" &
  pids+=("$!")
else
  echo "/nav2_relocalized_pose_to_tf already exists; not starting another one."
fi

echo "Nav2 helper nodes are running. Keep this terminal open."
wait
