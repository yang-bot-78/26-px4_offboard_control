#!/usr/bin/env bash
set -euo pipefail

project_root="${HOME}/ws_offboard_control"
records_root="${project_root}/flight_records"
mode="${1:-all}"

if [[ ! -d "${records_root}" ]]; then
  echo "flight_records not found: ${records_root}"
  exit 0
fi

list_rosbag_dirs() {
  find "${records_root}" -type d -name rosbag | sort
}

delete_dir() {
  local dir="$1"
  if [[ -d "${dir}" ]]; then
    rm -rf "${dir}"
    echo "deleted: ${dir}"
  fi
}

delete_empty_flight_dirs() {
  find "${records_root}" -mindepth 2 -maxdepth 2 -type d -name 'flight_*' | while read -r flight_dir; do
    if [[ -d "${flight_dir}/snapshot" && ! -d "${flight_dir}/rosbag" ]]; then
      echo "kept snapshot-only dir: ${flight_dir}"
      continue
    fi
    if [[ -z "$(find "${flight_dir}" -mindepth 1 -maxdepth 1 -print -quit)" ]]; then
      rmdir "${flight_dir}" 2>/dev/null || true
    fi
  done
}

case "${mode}" in
  all)
    bag_dirs="$(list_rosbag_dirs)"
    if [[ -z "${bag_dirs}" ]]; then
      echo "no rosbag directories found under ${records_root}"
      exit 0
    fi
    while read -r dir; do
      [[ -n "${dir}" ]] && delete_dir "${dir}"
    done <<< "${bag_dirs}"
    delete_empty_flight_dirs
    ;;
  latest)
    latest_dir="$(list_rosbag_dirs | tail -n 1)"
    if [[ -z "${latest_dir}" ]]; then
      echo "no rosbag directories found under ${records_root}"
      exit 0
    fi
    delete_dir "${latest_dir}"
    delete_empty_flight_dirs
    ;;
  *)
    echo "usage: ./delete_recorded_rosbags.sh [all|latest]"
    exit 1
    ;;
esac
