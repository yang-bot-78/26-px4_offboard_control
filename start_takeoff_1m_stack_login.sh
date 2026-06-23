#!/usr/bin/env bash
set -euo pipefail

mkdir -p "${HOME}/ws_offboard_control/runtime"
{
  echo "autostart login wrapper started: $(date '+%F %T %z')"
  echo "DISPLAY=${DISPLAY:-}"
  echo "XDG_SESSION_TYPE=${XDG_SESSION_TYPE:-}"
} >>"${HOME}/ws_offboard_control/runtime/autostart_login.log"

sleep "${AUTOSTART_DELAY_SEC:-10}"
exec "${HOME}/ws_offboard_control/start_takeoff_1m_stack.sh" >>"${HOME}/ws_offboard_control/runtime/autostart_login.log" 2>&1
