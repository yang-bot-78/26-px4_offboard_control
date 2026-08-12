#!/usr/bin/env bash
set -eo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
project_root="$(cd -- "${script_dir}/../.." && pwd -P)"

cd ${project_root}
source /opt/ros/humble/setup.bash
source ${project_root}/install/setup.bash
set -u

echo "== Nodes =="
ros2 node list | grep -E 'fastlio|nav2_relocalized|odometry_tf|planner|map_server' || true

echo
echo "== 必需服务 =="
ros2 service list | grep -E '/fastlio_global_backend/(load_map|relocalize|save_map)' || true

echo
echo "== 必需话题 =="
ros2 topic list | grep -E '^/(Odometry|cloud_registered|fastlio_global/(map|path|relocalized_pose)|tf)$' || true

echo
echo "== 一次性话题可用性 =="
timeout 3s ros2 topic echo /Odometry --once >/tmp/check_odom.txt 2>/dev/null && echo "/Odometry：正常" || echo "/Odometry：缺失"
timeout 3s ros2 topic echo /cloud_registered --once >/tmp/check_cloud.txt 2>/dev/null && echo "/cloud_registered：正常" || echo "/cloud_registered：缺失"
timeout 3s ros2 topic echo /fastlio_global/relocalized_pose --once >/tmp/check_relocalized_pose.txt 2>/dev/null && echo "/fastlio_global/relocalized_pose：正常" || echo "/fastlio_global/relocalized_pose：缺失"

echo
echo "== TF 检查 =="
timeout 3s ros2 run tf2_ros tf2_echo camera_init body >/tmp/check_tf_camera_body.txt 2>/dev/null && echo "camera_init -> body：正常" || echo "camera_init -> body：缺失"
timeout 3s ros2 run tf2_ros tf2_echo map camera_init >/tmp/check_tf_map_camera.txt 2>/dev/null && echo "map -> camera_init：正常" || echo "map -> camera_init：缺失"
