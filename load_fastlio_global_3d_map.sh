#!/usr/bin/env bash
set -eo pipefail

cd /home/robot/ws_offboard_control
source /opt/ros/humble/setup.bash
source /home/robot/ws_offboard_control/install/setup.bash
set -u

map_dir="${FASTLIO_GLOBAL_MAP_DIR:-/home/robot/ws_offboard_control/maps/fastlio_global_3d}"
service_name="/fastlio_global_backend/load_map"
wait_timeout_s="${LOAD_MAP_SERVICE_WAIT_TIMEOUT_S:-5}"

if [[ ! -f "${map_dir}/metadata.csv" ]]; then
  echo "Global 3D map metadata is missing: ${map_dir}/metadata.csv" >&2
  echo "Set FASTLIO_GLOBAL_MAP_DIR or save a valid FAST-LIO global map first." >&2
  exit 3
fi

if ! timeout "${wait_timeout_s}s" ros2 service type "${service_name}" >/tmp/fastlio_load_map_service_type.txt 2>/dev/null; then
  echo "Load-map service is not available: ${service_name}" >&2
  echo "Start the backend first:" >&2
  echo "  cd /home/robot/ws_offboard_control" >&2
  echo "  ./run_fastlio_global_relocalization.sh" >&2
  exit 2
fi

exec ros2 service call "${service_name}" fastlio_global_slam/srv/LoadMap \
  "{directory: '${map_dir}'}"
