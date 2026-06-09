#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="${WS_DIR:-$SCRIPT_DIR}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
WS_SETUP="${WS_SETUP:-$WS_DIR/install/setup.bash}"
FCU_URL="${FCU_URL:-serial:///dev/ttyUSB0:921600}"

usage() {
  cat <<EOF
Usage:
  $(basename "$0") <command>

Commands:
  doctor        Check whether required setup files and ros2 are available
  mavros        Launch MAVROS with FCU_URL=${FCU_URL}
  fix-odom-frames  Reset MAVROS odometry target frames to odom/base_link
  state         Read /mavros/state once
  odom          Read /Odometry once
  bridge        Launch FAST-LIO -> MAVROS odometry bridge
  mavros-odom   Read /mavros/odometry/out once
  print         Print the full multi-terminal command sequence

Environment overrides:
  WS_DIR        Default: $HOME/ws_offboard_control
  ROS_SETUP     Default: /opt/ros/humble/setup.bash
  WS_SETUP      Default: \$WS_DIR/install/setup.bash
  FCU_URL       Default: serial:///dev/ttyUSB0:921600
EOF
}

require_file() {
  local path="$1"
  if [[ ! -f "$path" ]]; then
    echo "Missing required file: $path" >&2
    exit 1
  fi
}

source_ros_only() {
  require_file "$ROS_SETUP"
  # shellcheck disable=SC1090
  source "$ROS_SETUP"
}

source_ros_and_ws() {
  source_ros_only
  require_file "$WS_SETUP"
  # shellcheck disable=SC1090
  source "$WS_SETUP"
}

doctor() {
  echo "ROS setup: $ROS_SETUP"
  [[ -f "$ROS_SETUP" ]] && echo "  ok" || echo "  missing"

  echo "Workspace setup: $WS_SETUP"
  [[ -f "$WS_SETUP" ]] && echo "  ok" || echo "  missing"

  echo "ros2 binary:"
  if command -v ros2 >/dev/null 2>&1; then
    command -v ros2
  else
    echo "  missing"
  fi
}

fix_odom_frames() {
  source_ros_only

  echo "Setting MAVROS odometry frame targets:"
  echo "  /mavros/odometry fcu.map_id_des        = map"
  echo "  /mavros/odometry fcu.odom_parent_id_des = odom"
  echo "  /mavros/odometry fcu.odom_child_id_des  = base_link"

  ros2 param set /mavros/odometry fcu.map_id_des map
  ros2 param set /mavros/odometry fcu.odom_parent_id_des odom
  ros2 param set /mavros/odometry fcu.odom_child_id_des base_link

  echo
  echo "Current values:"
  ros2 param get /mavros/odometry fcu.map_id_des
  ros2 param get /mavros/odometry fcu.odom_parent_id_des
  ros2 param get /mavros/odometry fcu.odom_child_id_des
}

print_sequence() {
  cat <<EOF
Terminal 1: MAVROS
  source ${ROS_SETUP}
  ros2 launch mavros px4.launch fcu_url:=${FCU_URL}

Terminal 2: MAVROS state
  source ${ROS_SETUP}
  ros2 topic echo /mavros/state --once

Terminal 3: Your lidar driver + FAST-LIO
  Start with your normal commands, then verify:
  cd ${WS_DIR}
  source ${ROS_SETUP}
  source ${WS_SETUP}
  ros2 topic echo /Odometry --once

Terminal 4: FAST-LIO -> MAVROS bridge
  cd ${WS_DIR}
  source ${ROS_SETUP}
  source ${WS_SETUP}
  ros2 launch px4_ros_com fastlio_mavros_odometry_bridge.launch.py

Terminal 5: Bridge output
  cd ${WS_DIR}
  source ${ROS_SETUP}
  source ${WS_SETUP}
  ros2 topic echo /mavros/odometry/out --once

QGC MAVLink Console
  listener vehicle_visual_odometry 5
  mavlink status
  param show EKF2_EV_CTRL

After MAVROS starts, if you see errors about camera_init_ned:
  cd ${WS_DIR}
  ./fastlio_mavros_check.sh fix-odom-frames
EOF
}

cmd="${1:-}"

case "$cmd" in
  doctor)
    doctor
    ;;
  mavros)
    source_ros_only
    exec ros2 launch mavros px4.launch "fcu_url:=${FCU_URL}"
    ;;
  fix-odom-frames)
    fix_odom_frames
    ;;
  state)
    source_ros_only
    exec ros2 topic echo /mavros/state --once
    ;;
  odom)
    source_ros_and_ws
    cd "$WS_DIR"
    exec ros2 topic echo /Odometry --once
    ;;
  bridge)
    source_ros_and_ws
    cd "$WS_DIR"
    exec ros2 launch px4_ros_com fastlio_mavros_odometry_bridge.launch.py
    ;;
  mavros-odom)
    source_ros_and_ws
    cd "$WS_DIR"
    exec ros2 topic echo /mavros/odometry/out --once
    ;;
  print)
    print_sequence
    ;;
  *)
    usage
    exit 1
    ;;
esac
