#!/usr/bin/env bash
set -euo pipefail

mode="${1:?usage: wait_for_ros_readiness.sh <topic|healthy|mavros_connected> <timeout_s> [topic]}"
timeout_s="${2:?usage: wait_for_ros_readiness.sh <topic|healthy|mavros_connected> <timeout_s> [topic]}"
topic="${3:-}"
if [[ ! "${timeout_s}" =~ ^[0-9]+$ ]] || (( timeout_s < 1 )); then
  echo "timeout_s must be a positive integer" >&2
  exit 2
fi
deadline=$((SECONDS + timeout_s))

case "${mode}" in
  topic)
    [[ -n "${topic}" ]] || { echo "topic mode requires a topic name" >&2; exit 2; }
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
  case "${mode}" in
    topic)
      if timeout 5 ros2 topic list -t 2>/dev/null |
        awk -v wanted="${topic}" '$1 == wanted { found=1 } END { exit !found }'; then
        echo "Ready: topic ${topic} has a publisher"
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
exit 1
