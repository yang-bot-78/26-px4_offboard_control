#!/usr/bin/env bash
set -eo pipefail

cd /home/robot/ws_offboard_control
source /opt/ros/humble/setup.bash
source /home/robot/ws_offboard_control/install/setup.bash
set -u

service_name="/fastlio_global_backend/relocalize"
service_type="fastlio_global_slam/srv/Relocalize"
wait_timeout_s="${RELOCALIZE_SERVICE_WAIT_TIMEOUT_S:-5}"
call_timeout_s="${RELOCALIZE_CALL_TIMEOUT_S:-60}"
map_dir="${FASTLIO_GLOBAL_MAP_DIR:-/home/robot/ws_offboard_control/maps/fastlio_global_3d}"
auto_load_map="${AUTO_LOAD_GLOBAL_MAP:-true}"

if ! timeout "${wait_timeout_s}s" ros2 service type "${service_name}" >/tmp/fastlio_relocalize_service_type.txt 2>/dev/null; then
  echo "Relocalize service is not available: ${service_name}" >&2
  echo "Start the backend first in another terminal:" >&2
  echo "  cd /home/robot/ws_offboard_control" >&2
  echo "  ./run_fastlio_global_relocalization.sh" >&2
  echo "" >&2
  echo "Then check:" >&2
  echo "  ros2 service list | grep fastlio_global_backend" >&2
  echo "  ros2 topic hz /fastlio_global/relocalized_pose" >&2
  exit 2
fi

if [[ "${auto_load_map}" == "true" ]]; then
  if [[ ! -f "${map_dir}/metadata.csv" ]]; then
    echo "Global 3D map metadata is missing: ${map_dir}/metadata.csv" >&2
    echo "Set FASTLIO_GLOBAL_MAP_DIR or save/load a valid FAST-LIO global map first." >&2
    exit 3
  fi

  echo "Loading FAST-LIO global 3D map before relocalization: ${map_dir}"
  timeout "${wait_timeout_s}s" ros2 service call /fastlio_global_backend/load_map fastlio_global_slam/srv/LoadMap \
    "{directory: '${map_dir}'}"
fi

echo "Calling relocalize service with timeout ${call_timeout_s}s..."
exec timeout "${call_timeout_s}s" ros2 service call "${service_name}" "${service_type}" \
  "{use_latest_scan: true}"
