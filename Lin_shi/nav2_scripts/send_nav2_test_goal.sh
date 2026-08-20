#!/usr/bin/env bash
set -eo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
workspace_root="$(cd -- "${script_dir}/../.." && pwd -P)"
cd "${workspace_root}"
source /opt/ros/humble/setup.bash
source "${workspace_root}/install/setup.bash"
set -u

goal_x="${GOAL_X:-1.0}"
goal_y="${GOAL_Y:-0.0}"
goal_yaw="${GOAL_YAW_RAD:-0.0}"
goal_frame="${GOAL_FRAME:-map}"

qz="$(python3 - <<PY
import math
print(math.sin(float("${goal_yaw}") / 2.0))
PY
)"
qw="$(python3 - <<PY
import math
print(math.cos(float("${goal_yaw}") / 2.0))
PY
)"

echo "Publishing test goal to /goal_pose: frame=${goal_frame} x=${goal_x} y=${goal_y} yaw=${goal_yaw}"

ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: '${goal_frame}'},
  pose: {
    position: {x: ${goal_x}, y: ${goal_y}, z: 0.0},
    orientation: {x: 0.0, y: 0.0, z: ${qz}, w: ${qw}}
  }
}"
