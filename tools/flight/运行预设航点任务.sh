#!/usr/bin/env bash
set -euo pipefail

# Pilot-triggered preset mission entrypoint.  This script never arms, changes
# flight mode, or sends MAVROS setpoints.  The pilot takes off manually and
# switches PX4 to OFFBOARD; mission_sequencer_node then publishes B1 -> B2 -> C
# -> D to /goal_pose.

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
project_root="$(cd -- "${script_dir}/../.." && pwd -P)"
mission_file="${MISSION_FILE:-${project_root}/src/race_offboard/config/waypoints/main/mission.yaml}"
rviz="${RVIZ:-false}"

if [[ "${rviz}" != true && "${rviz}" != false ]]; then
  echo "RVIZ must be true or false; got: ${rviz}" >&2
  exit 2
fi
if [[ ! -f "${mission_file}" ]]; then
  echo "MISSION_FILE does not exist: ${mission_file}" >&2
  exit 2
fi

# Do not fly a mission that would merely wait for RViz points.  Presets are
# mandatory for this unattended entrypoint.
if ! python3 - "${mission_file}" <<'PY'
import sys
import yaml

with open(sys.argv[1], encoding='utf-8') as stream:
    document = yaml.safe_load(stream) or {}
points = document.get('preset_points')
if not isinstance(points, list) or len(points) != 4:
    raise SystemExit('MISSION_FILE must define exactly four preset_points: B1, B2, C, D')
PY
then
  exit 2
fi

echo "Preset mission: ${mission_file}"
echo "RViz: ${rviz}"
echo "Pilot procedure: arm and take off manually, stabilize at cruise height, then select OFFBOARD."

export MISSION_ENABLED=true
export MISSION_FILE="${mission_file}"
export MANUAL_HANDOVER=true
export RVIZ="${rviz}"
exec "${script_dir}/运行导航控制.sh"
