#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
venv_python="$script_dir/yolo/bin/python"

if [[ ! -x "$venv_python" ]]; then
    echo "Python environment not found: $venv_python" >&2
    echo "Install dependencies with: $script_dir/yolo/bin/python -m pip install -r $script_dir/requirements-cpu.txt" >&2
    exit 1
fi

exec "$venv_python" "$script_dir/0812.py" "$@"
