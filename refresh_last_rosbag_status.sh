#!/usr/bin/env bash
set -euo pipefail

project_root="${HOME}/ws_offboard_control"
runtime_dir="${project_root}/runtime"
pid_file="${runtime_dir}/takeoff_debug_bag.pid"
run_dir_file="${runtime_dir}/takeoff_debug_bag_run_dir.txt"
status_file="${runtime_dir}/last_rosbag_status.txt"
history_file="${runtime_dir}/last_rosbag_status_history.log"

read_kv() {
  local key="$1"
  local file="$2"
  awk -F= -v k="${key}" '$1 == k {sub($1 FS, ""); print; exit}' "${file}" 2>/dev/null || true
}

write_status() {
  local state="$1"
  local detail="$2"
  local now
  local run_root=""
  local bag_pid=""
  now="$(date '+%F %T %z')"

  [[ -f "${run_dir_file}" ]] && run_root="$(tr -d '\r' < "${run_dir_file}")"
  [[ -f "${pid_file}" ]] && bag_pid="$(tr -d '[:space:]' < "${pid_file}")"

  cat >"${status_file}" <<EOF
state=${state}
timestamp=${now}
run_root=${run_root}
pid=${bag_pid}
detail=${detail}
EOF

  if [[ -n "${run_root}" ]]; then
    mkdir -p "${run_root}/snapshot"
    cat >"${run_root}/snapshot/rosbag_status.txt" <<EOF
state=${state}
timestamp=${now}
run_root=${run_root}
pid=${bag_pid}
detail=${detail}
EOF
  fi

  printf '%s state=%s pid=%s detail=%s run_root=%s\n' \
    "${now}" "${state}" "${bag_pid}" "${detail}" "${run_root}" >>"${history_file}"
}

mkdir -p "${runtime_dir}"

if [[ ! -f "${status_file}" ]]; then
  write_status "unknown" "no previous rosbag status file"
  exit 0
fi

current_state="$(read_kv state "${status_file}")"
current_pid="$(read_kv pid "${status_file}")"

if [[ "${current_state}" == "recording" || "${current_state}" == "stopping" ]]; then
  if [[ -n "${current_pid}" ]] && kill -0 "${current_pid}" 2>/dev/null; then
    write_status "recording" "rosbag process still running"
  else
    rm -f "${pid_file}" "${run_dir_file}"
    write_status "interrupted" "previous rosbag ended unexpectedly or system rebooted before clean stop"
  fi
fi
