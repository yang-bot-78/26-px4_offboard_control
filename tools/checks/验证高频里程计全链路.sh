#!/usr/bin/env bash
set -Eeuo pipefail

# 用户工具入口；实现保留在 fr_lio 包内，避免两个副本发生漂移。
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
project_root="$(cd -- "${script_dir}/../.." && pwd -P)"

exec "${project_root}/src/fr_lio/scripts/验证高频里程计全链路.sh" "$@"
