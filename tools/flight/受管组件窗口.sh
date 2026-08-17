#!/usr/bin/env bash
set -euo pipefail

usage() {
  echo "usage: $0 <owner_pid> <owner_start_ticks> <runtime_file> <log_file> <title> -- <command> [args...]" >&2
  exit 2
}

[[ $# -ge 7 ]] || usage

owner_pid="$1"
owner_start_ticks="$2"
runtime_file="$3"
log_file="$4"
title="$5"
shift 5
[[ "${1:-}" == -- ]] || usage
shift
[[ $# -gt 0 ]] || usage

if [[ ! "${owner_pid}" =~ ^[0-9]+$ || ! "${owner_start_ticks}" =~ ^[0-9]+$ ]]; then
  echo "owner PID 和 start ticks 必须是数字" >&2
  exit 2
fi

mkdir -p "$(dirname -- "${runtime_file}")" "$(dirname -- "${log_file}")"
exec > >(tee -a "${log_file}") 2>&1

component_pid=""
component_pgid=""
owner_watchdog_pid=""
cleanup_started=false
component_cpu_affinity="${COMPONENT_CPU_AFFINITY:-}"

owner_is_alive() {
  local current_start_ticks
  [[ -r "/proc/${owner_pid}/stat" ]] || return 1
  current_start_ticks="$(awk '{print $22}' "/proc/${owner_pid}/stat" 2>/dev/null || true)"
  [[ "${current_start_ticks}" == "${owner_start_ticks}" ]]
}

component_group_is_alive() {
  [[ "${component_pgid}" =~ ^[0-9]+$ ]] || return 1
  kill -0 -- "-${component_pgid}" 2>/dev/null
}

write_runtime_state() {
  local state="$1"
  local status="${2:-}"
  local temporary_file="${runtime_file}.tmp.$$"

  {
    printf 'state=%s\n' "${state}"
    printf 'status=%s\n' "${status}"
    printf 'owner_pid=%s\n' "${owner_pid}"
    printf 'pid=%s\n' "${component_pid}"
    printf 'pgid=%s\n' "${component_pgid}"
    printf 'cpu_affinity=%s\n' "${component_cpu_affinity}"
    printf 'title=%s\n' "${title}"
  } >"${temporary_file}"
  mv -f "${temporary_file}" "${runtime_file}"
}

stop_component_group() {
  local signal="${1:-TERM}"
  if component_group_is_alive; then
    kill -"${signal}" -- "-${component_pgid}" 2>/dev/null || true
  fi
}

wait_for_group_exit() {
  local attempts="${1:-25}"
  local _index
  for ((_index=0; _index<attempts; _index++)); do
    component_group_is_alive || return 0
    sleep 0.2
  done
  return 1
}

cleanup() {
  local status=$?
  [[ "${cleanup_started}" == false ]] || return
  cleanup_started=true
  trap - EXIT INT TERM HUP
  set +e

  if [[ -n "${owner_watchdog_pid}" ]] && kill -0 "${owner_watchdog_pid}" 2>/dev/null; then
    kill -TERM "${owner_watchdog_pid}" 2>/dev/null || true
    wait "${owner_watchdog_pid}" 2>/dev/null || true
  fi

  if component_group_is_alive; then
    echo "[${title}] 正在停止进程组 ${component_pgid}"
    stop_component_group TERM
    if ! wait_for_group_exit 25; then
      echo "[${title}] TERM 超时，强制结束进程组 ${component_pgid}"
      stop_component_group KILL
      wait_for_group_exit 10 || true
    fi
  fi
  if [[ "${component_pid}" =~ ^[0-9]+$ ]]; then
    wait "${component_pid}" 2>/dev/null || true
  fi
  write_runtime_state exited "${status}" || true
  echo "[${title}] 已退出，状态码 ${status}"
  exit "${status}"
}

on_signal() {
  local signal="$1"
  echo "[${title}] received ${signal}"
  case "${signal}" in
    INT) exit 130 ;;
    *) exit 143 ;;
  esac
}

trap cleanup EXIT
trap 'on_signal INT' INT
trap 'on_signal TERM' TERM
trap 'on_signal HUP' HUP

echo "[${title}] starting"
printf '[%s] command:' "${title}"
printf ' %q' "$@"
printf '\n'

if [[ -n "${component_cpu_affinity}" ]]; then
  if ! command -v taskset >/dev/null 2>&1; then
    echo "[${title}] COMPONENT_CPU_AFFINITY=${component_cpu_affinity} but taskset is unavailable" >&2
    exit 1
  fi
  if ! taskset --cpu-list "${component_cpu_affinity}" true >/dev/null 2>&1; then
    echo "[${title}] invalid CPU affinity: ${component_cpu_affinity}" >&2
    exit 1
  fi
  echo "[${title}] CPU affinity=${component_cpu_affinity}"
  setsid taskset --cpu-list "${component_cpu_affinity}" "$@" &
else
  setsid "$@" &
fi
component_pid=$!
# 这个子进程在 setsid(1) 之前不是进程组长，所以 setsid 成功后它的 PID 就是新的
# 会话 ID 和进程组 ID。要立刻记下来：短命的 launch 父进程可能在 ps(1) 来得及
# 检查它之前就退出了，而同组的后代进程仍然需要清理。
component_pgid="${component_pid}"
write_runtime_state running ""
echo "[${title}] pid=${component_pid} pgid=${component_pgid} log=${log_file}"

(
  while component_group_is_alive; do
    if ! owner_is_alive; then
      echo "[${title}] owner ${owner_pid} 已消失，正在停止进程组 ${component_pgid}"
      stop_component_group TERM
      if ! wait_for_group_exit 25; then
        stop_component_group KILL
      fi
      exit 0
    fi
    sleep 0.5
  done
) &
owner_watchdog_pid=$!

set +e
wait "${component_pid}"
component_status=$?
set -e
exit "${component_status}"
