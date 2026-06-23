#!/usr/bin/env bash
set -euo pipefail

project_root="${HOME}/ws_offboard_control"
runtime_dir="${project_root}/runtime"
pid_file="${runtime_dir}/takeoff_debug_bag.pid"
run_dir_file="${runtime_dir}/takeoff_debug_bag_run_dir.txt"
status_file="${runtime_dir}/last_rosbag_status.txt"
history_file="${runtime_dir}/last_rosbag_status_history.log"

write_status() {
  local state="$1"
  local detail="${2:-}"
  local now
  local run_root=""
  now="$(date '+%F %T %z')"
  if [[ -f "${run_dir_file}" ]]; then
    run_root="$(tr -d '\r' < "${run_dir_file}")"
  fi

  cat >"${status_file}" <<EOF
state=${state}
timestamp=${now}
run_root=${run_root}
pid=${bag_pid:-}
detail=${detail}
EOF

  if [[ -n "${run_root}" ]]; then
    mkdir -p "${run_root}/snapshot"
    cat >"${run_root}/snapshot/rosbag_status.txt" <<EOF
state=${state}
timestamp=${now}
run_root=${run_root}
pid=${bag_pid:-}
detail=${detail}
EOF
  fi

  printf '%s state=%s pid=%s detail=%s run_root=%s\n' \
    "${now}" "${state}" "${bag_pid:-}" "${detail}" "${run_root}" >>"${history_file}"
}

if [[ ! -f "${pid_file}" ]]; then
  echo "rosbag pid file not present: ${pid_file}"
  write_status "not_running" "stop requested but pid file absent"
  exit 0
fi

bag_pid="$(tr -d '[:space:]' < "${pid_file}")"
if [[ -z "${bag_pid}" ]]; then
  echo "rosbag pid file is empty"
  rm -f "${pid_file}"
  write_status "interrupted" "pid file empty during stop request"
  exit 0
fi

if ! kill -0 "${bag_pid}" 2>/dev/null; then
  echo "rosbag process not running: ${bag_pid}"
  rm -f "${pid_file}"
  write_status "interrupted" "pid existed but rosbag process already gone"
  exit 0
fi

echo "Stopping rosbag gracefully: pid=${bag_pid}"
write_status "stopping" "shutdown guard sent SIGINT"
kill -INT "${bag_pid}" 2>/dev/null || true

for _ in $(seq 1 15); do
  if ! kill -0 "${bag_pid}" 2>/dev/null; then
    rm -f "${pid_file}" "${run_dir_file}"
    write_status "saved_cleanly" "shutdown guard stopped rosbag with SIGINT"
    echo "rosbag stopped cleanly"
    exit 0
  fi
  sleep 1
done

echo "rosbag did not exit after SIGINT, sending SIGTERM"
kill -TERM "${bag_pid}" 2>/dev/null || true
sleep 2

rm -f "${pid_file}" "${run_dir_file}"
write_status "interrupted" "shutdown guard escalated to SIGTERM"
