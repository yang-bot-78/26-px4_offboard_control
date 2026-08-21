#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
venv_activate="/home/robot/rong_ws/ws_offboard_control/bs/yolo/bin/activate"
recognition_allowed_cpus="${RECOGNITION_ALLOWED_CPUS:-}"
recognition_nice_level="${RECOGNITION_NICE_LEVEL:-10}"

if [[ ! -f "$venv_activate" ]]; then
    echo "Python environment activation script not found: $venv_activate" >&2
    exit 1
fi

source "$venv_activate"
if [[ -n "$recognition_allowed_cpus" ]]; then
    export RECOGNITION_ALLOWED_CPUS="$recognition_allowed_cpus"
fi
exec nice -n "$recognition_nice_level" python3 "$script_dir/d435i_si.py" "$@"
