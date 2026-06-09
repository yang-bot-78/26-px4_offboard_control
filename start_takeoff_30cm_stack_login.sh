#!/usr/bin/env bash
set -euo pipefail

sleep "${AUTOSTART_DELAY_SEC:-10}"
exec "${HOME}/ws_offboard_control/start_takeoff_30cm_stack.sh"
