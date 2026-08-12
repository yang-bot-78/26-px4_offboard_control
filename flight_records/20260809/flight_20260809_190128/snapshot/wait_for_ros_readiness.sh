#!/usr/bin/env bash
set -euo pipefail

mode="${1:?usage: wait_for_ros_readiness.sh <topic|message|node|healthy|mavros_connected> <timeout_s> [name]}"
timeout_s="${2:?usage: wait_for_ros_readiness.sh <topic|message|node|healthy|mavros_connected> <timeout_s> [name]}"
topic="${3:-}"
readiness_process_pid="${READINESS_PROCESS_PID:-}"
readiness_setup_bash="${READINESS_SETUP_BASH:-}"

if [[ -n "${readiness_setup_bash}" ]]; then
  if [[ ! -f "${readiness_setup_bash}" ]]; then
    echo "READINESS_SETUP_BASH does not exist: ${readiness_setup_bash}" >&2
    exit 2
  fi
  # Livox custom message types live in the driver overlay, not the main
  # workspace overlay used by the navigation processes.
  set +u
  # shellcheck disable=SC1090
  source "${readiness_setup_bash}"
  set -u
fi
if [[ ! "${timeout_s}" =~ ^[0-9]+$ ]] || (( timeout_s < 1 )); then
  echo "timeout_s must be a positive integer" >&2
  exit 2
fi
deadline=$((SECONDS + timeout_s))

case "${mode}" in
  topic)
    [[ -n "${topic}" ]] || { echo "topic mode requires a topic name" >&2; exit 2; }
    ;;
  message)
    [[ -n "${topic}" ]] || { echo "message mode requires a topic name" >&2; exit 2; }
    ;;
  node)
    [[ -n "${topic}" ]] || { echo "node mode requires a node name" >&2; exit 2; }
    ;;
  healthy)
    topic="${topic:-/ev_health/status}"
    ;;
  mavros_connected)
    topic="${topic:-/mavros/state}"
    ;;
  *)
    echo "unknown readiness mode: ${mode}" >&2
    exit 2
    ;;
esac

echo "Waiting for ${mode}: ${topic} (timeout=${timeout_s}s)"
while (( SECONDS < deadline )); do
  if [[ -n "${readiness_process_pid}" ]] &&
    ! kill -0 "${readiness_process_pid}" 2>/dev/null; then
    echo "Readiness owner exited before becoming ready: pid=${readiness_process_pid}, mode=${mode}, name=${topic}" >&2
    exit 1
  fi
  case "${mode}" in
    topic)
      if timeout 5 ros2 topic list -t 2>/dev/null |
        awk -v wanted="${topic}" '$1 == wanted { found=1 } END { exit !found }'; then
        echo "Ready: topic ${topic} has a publisher"
        exit 0
      fi
      ;;
    message)
      # Sensor publishers are commonly BEST_EFFORT; request the compatible
      # subscriber QoS so a real sample, rather than topic discovery, is tested.
      if timeout 5 ros2 topic echo --once --qos-reliability best_effort "${topic}" >/dev/null 2>&1; then
        echo "Ready: received a message from ${topic}"
        exit 0
      fi
      ;;
    node)
      if timeout 5 ros2 node list 2>/dev/null | grep -Fxq "${topic}"; then
        echo "Ready: node ${topic} exists"
        exit 0
      fi
      ;;
    healthy)
      message="$(timeout 5 ros2 topic echo --once "${topic}" std_msgs/msg/String 2>/dev/null || true)"
      if [[ "${message}" == *"data: HEALTHY"* ]]; then
        echo "Ready: ${topic}=HEALTHY"
        exit 0
      fi
      ;;
    mavros_connected)
      message="$(timeout 5 ros2 topic echo --once "${topic}" mavros_msgs/msg/State 2>/dev/null || true)"
      if [[ "${message}" == *"connected: true"* ]]; then
        echo "Ready: MAVROS connected"
        exit 0
      fi
      ;;
  esac
  sleep 1
done

echo "Readiness timeout: mode=${mode}, topic=${topic}, timeout=${timeout_s}s" >&2
if [[ "${mode}" == healthy ]]; then
  echo "Latest EV health diagnostics:" >&2
  timeout 5 ros2 topic echo --once /ev_health/diagnostics \
    diagnostic_msgs/msg/DiagnosticArray --field status 2>/dev/null |
    sed -n '1,100p' >&2 || true
fi
exit 1
