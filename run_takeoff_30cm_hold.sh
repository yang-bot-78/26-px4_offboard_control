#!/usr/bin/env bash
set -eo pipefail

cd /home/robot/ws_offboard_control
source /home/robot/ws_offboard_control/install/setup.bash
set -u

# Relative target in NED: -0.30 m means climb 30 cm.
exec ros2 run px4_ros_com minipc_mavros_offboard.py --ros-args \
  -p arm_only:=false \
  -p use_rc_offboard:=true \
  -p target_x_m:=0.0 \
  -p target_y_m:=0.0 \
  -p target_z_m:=-0.30 \
  -p hover_seconds:=30.0 \
  -p auto_land:=false \
  -p offboard_stabilize_seconds:=10.0
