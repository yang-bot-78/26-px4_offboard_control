#!/usr/bin/env bash
set -euo pipefail

mode="${1:?usage: 等待ROS就绪.sh <topic|message|node|healthy|flight_ready|mavros_connected> <timeout_s> [name]}"
timeout_s="${2:?usage: 等待ROS就绪.sh <topic|message|node|healthy|flight_ready|mavros_connected> <timeout_s> [name]}"
topic="${3:-}"
readiness_process_pid="${READINESS_PROCESS_PID:-}"
readiness_setup_bash="${READINESS_SETUP_BASH:-}"

if [[ -n "${readiness_setup_bash}" ]]; then
  if [[ ! -f "${readiness_setup_bash}" ]]; then
    echo "READINESS_SETUP_BASH 不存在：${readiness_setup_bash}" >&2
    exit 2
  fi
  # Livox 的自定义消息类型在驱动 overlay 里，不在导航进程使用的主工作区 overlay 里。
  set +u
  # shellcheck disable=SC1090
  source "${readiness_setup_bash}"
  set -u
fi
if [[ ! "${timeout_s}" =~ ^[0-9]+$ ]] || (( timeout_s < 1 )); then
  echo "timeout_s 必须是正整数" >&2
  exit 2
fi
deadline=$((SECONDS + timeout_s))

case "${mode}" in
  topic)
    [[ -n "${topic}" ]] || { echo "topic 模式需要提供话题名" >&2; exit 2; }
    ;;
  message)
    [[ -n "${topic}" ]] || { echo "message 模式需要提供话题名" >&2; exit 2; }
    ;;
  node)
    [[ -n "${topic}" ]] || { echo "node 模式需要提供节点名" >&2; exit 2; }
    ;;
  healthy)
    topic="${topic:-/ev_health/status}"
    ;;
  flight_ready)
    topic="${topic:-/ev_health/flight_ready}"
    ;;
  mavros_connected)
    topic="${topic:-/mavros/state}"
    ;;
  *)
    echo "未知的就绪检查模式：${mode}" >&2
    exit 2
    ;;
esac

echo "正在等待 ${mode}：${topic}（超时 ${timeout_s}s）"
while (( SECONDS < deadline )); do
  if [[ -n "${readiness_process_pid}" ]] &&
    ! kill -0 "${readiness_process_pid}" 2>/dev/null; then
    echo "被等待的进程在就绪之前已退出：pid=${readiness_process_pid}, mode=${mode}, name=${topic}" >&2
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
      # 传感器发布者通常是 BEST_EFFORT；这里请求兼容的订阅端 QoS，确保测的是
      # 真实收到样本，而不只是话题被发现。
      if timeout 5 ros2 topic echo --once --qos-reliability best_effort "${topic}" >/dev/null 2>&1; then
        echo "已就绪：收到来自 ${topic} 的真实消息"
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
    flight_ready)
      message="$(timeout 5 ros2 topic echo --once "${topic}" std_msgs/msg/Bool 2>/dev/null || true)"
      if [[ "${message}" == *"data: true"* ]]; then
        echo "Ready: ${topic}=true"
        exit 0
      fi
      ;;
    mavros_connected)
      message="$(timeout 5 ros2 topic echo --once "${topic}" mavros_msgs/msg/State 2>/dev/null || true)"
      if [[ "${message}" == *"connected: true"* ]]; then
        echo "已就绪：MAVROS 已连接"
        exit 0
      fi
      ;;
  esac
  sleep 1
done

echo "就绪等待超时：mode=${mode}, topic=${topic}, timeout=${timeout_s}s" >&2
if [[ "${mode}" == healthy || "${mode}" == flight_ready ]]; then
  echo "最近一次 EV 健康诊断：" >&2
  timeout 5 ros2 topic echo --once /ev_health/diagnostics \
    diagnostic_msgs/msg/DiagnosticArray --field status 2>/dev/null |
    sed -n '1,100p' >&2 || true
fi
exit 1
