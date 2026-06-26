#!/usr/bin/env bash
set -eo pipefail

cd /home/robot/ws_offboard_control
source /opt/ros/humble/setup.bash
source /home/robot/ws_offboard_control/install/setup.bash
set -u

echo "== Nodes =="
ros2 node list | grep -E 'fastlio|nav2_relocalized|odometry_tf|planner|map_server' || true

echo
echo "== Required services =="
ros2 service list | grep -E '/fastlio_global_backend/(load_map|relocalize|save_map)' || true

echo
echo "== Required topics =="
ros2 topic list | grep -E '^/(Odometry|cloud_registered|fastlio_global/(map|path|relocalized_pose)|tf)$' || true

echo
echo "== One-shot topic availability =="
timeout 3s ros2 topic echo /Odometry --once >/tmp/check_odom.txt 2>/dev/null && echo "/Odometry: ok" || echo "/Odometry: missing"
timeout 3s ros2 topic echo /cloud_registered --once >/tmp/check_cloud.txt 2>/dev/null && echo "/cloud_registered: ok" || echo "/cloud_registered: missing"
timeout 3s ros2 topic echo /fastlio_global/relocalized_pose --once >/tmp/check_relocalized_pose.txt 2>/dev/null && echo "/fastlio_global/relocalized_pose: ok" || echo "/fastlio_global/relocalized_pose: missing"

echo
echo "== TF checks =="
timeout 3s ros2 run tf2_ros tf2_echo camera_init body >/tmp/check_tf_camera_body.txt 2>/dev/null && echo "camera_init -> body: ok" || echo "camera_init -> body: missing"
timeout 3s ros2 run tf2_ros tf2_echo map camera_init >/tmp/check_tf_map_camera.txt 2>/dev/null && echo "map -> camera_init: ok" || echo "map -> camera_init: missing"
