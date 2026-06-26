#!/usr/bin/env bash
set -eo pipefail

map_dir="${FASTLIO_GLOBAL_MAP_DIR:-/home/robot/ws_offboard_control/maps/fastlio_global_3d}"
metadata="${map_dir}/metadata.csv"

if [[ ! -f "${metadata}" ]]; then
  echo "metadata.csv not found: ${metadata}"
  exit 1
fi

keyframes=$(( $(wc -l < "${metadata}") - 1 ))
echo "Map directory: ${map_dir}"
echo "Keyframes: ${keyframes}"

if [[ "${keyframes}" -lt 10 ]]; then
  echo "Status: too small for reliable relocalization. Re-scan and save after moving through the area."
  exit 2
fi

echo "Status: usable candidate for relocalization."
