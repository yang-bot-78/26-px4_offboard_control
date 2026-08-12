#!/usr/bin/env bash
set -eo pipefail

cd /home/robot/ws_offboard_control
source /opt/ros/humble/setup.bash
source /home/robot/ws_offboard_control/install/setup.bash
set -u

echo "== Nav2 planner nodes =="
timeout 3s ros2 node list >/tmp/check_nav2_node_list.txt 2>/dev/null || true
grep -E 'planner_server|map_server|lifecycle_manager|nav2_stage1_goal_to_path|nav2_relocalized_pose_to_tf|nav2_stage1_odometry_tf_publisher' /tmp/check_nav2_node_list.txt || true

echo
echo "== Required helper node status =="
required_nodes=(
  /planner_server
  /map_server
  /nav2_stage1_goal_to_path
  /nav2_stage1_odometry_tf_publisher
  /nav2_relocalized_pose_to_tf
)
for node in "${required_nodes[@]}"; do
  if grep -Fxq "${node}" /tmp/check_nav2_node_list.txt; then
    echo "${node}: ok"
  else
    echo "${node}: MISSING"
  fi
done

echo
echo "== Lifecycle states =="
for node in /planner_server /map_server; do
  if grep -Fxq "${node}" /tmp/check_nav2_node_list.txt; then
    printf '%s: ' "${node}"
    timeout 3s ros2 lifecycle get "${node}" 2>/dev/null || echo "timeout/unresponsive"
  else
    echo "${node}: missing"
  fi
done

echo
echo "== Planner action =="
timeout 3s ros2 action list -t >/tmp/check_nav2_action_list.txt 2>/dev/null || true
grep -E '/compute_path_to_pose|ComputePathToPose' /tmp/check_nav2_action_list.txt || true

echo
echo "== Required topics =="
timeout 3s ros2 topic list -t >/tmp/check_nav2_topic_list.txt 2>/dev/null || true
grep -E '^/(goal_pose|move_base_simple/goal|nav2_stage1/path|map|global_costmap/costmap|tf)( |$)' /tmp/check_nav2_topic_list.txt || true

echo
echo "== Odometry input =="
timeout 5s ros2 topic hz /Odometry >/tmp/check_nav2_odom_hz.txt 2>/dev/null || true
if grep -q "average rate" /tmp/check_nav2_odom_hz.txt 2>/dev/null; then
  tail -n 5 /tmp/check_nav2_odom_hz.txt
else
  echo "/Odometry: no hz sample"
fi
if timeout 3s ros2 topic echo /Odometry --once >/tmp/check_nav2_odom_once.txt 2>/dev/null; then
  grep -E 'frame_id:|child_frame_id:' /tmp/check_nav2_odom_once.txt | head -n 6
else
  echo "/Odometry: no one-shot sample"
fi

echo
echo "== Latest goal/path samples =="
timeout 2s ros2 topic echo /goal_pose --once >/tmp/check_nav2_goal_pose.txt 2>/dev/null && \
  echo "/goal_pose: received" || echo "/goal_pose: no recent sample; click and drag RViz '2D Goal Pose'"
timeout 2s ros2 topic echo /nav2_stage1/path --once >/tmp/check_nav2_stage1_path.txt 2>/dev/null && \
  echo "/nav2_stage1/path: received" || echo "/nav2_stage1/path: no sample"

echo
echo "== TF checks =="
timeout 4s ros2 run tf2_ros tf2_echo map body >/tmp/check_nav2_tf_map_body.txt 2>/dev/null || true
if grep -q "At time" /tmp/check_nav2_tf_map_body.txt 2>/dev/null; then
  echo "map -> body: ok"
else
  echo "map -> body: missing"
fi
timeout 4s ros2 run tf2_ros tf2_echo map camera_init >/tmp/check_nav2_tf_map_camera.txt 2>/dev/null || true
if grep -q "At time" /tmp/check_nav2_tf_map_camera.txt 2>/dev/null; then
  echo "map -> camera_init: ok"
else
  echo "map -> camera_init: missing"
fi
timeout 4s ros2 run tf2_ros tf2_echo camera_init body >/tmp/check_nav2_tf_camera_body.txt 2>/dev/null || true
if grep -q "At time" /tmp/check_nav2_tf_camera_body.txt 2>/dev/null; then
  echo "camera_init -> body: ok"
else
  echo "camera_init -> body: missing"
fi

echo
echo "== Helpful node details =="
for node in /nav2_stage1_goal_to_path /planner_server; do
  if grep -Fxq "${node}" /tmp/check_nav2_node_list.txt; then
    echo "-- ${node}"
    timeout 5s ros2 node info "${node}" 2>/dev/null | sed -n '1,120p' || echo "node info timeout/unresponsive"
  fi
done
