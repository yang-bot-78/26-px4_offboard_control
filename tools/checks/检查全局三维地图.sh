#!/usr/bin/env bash
set -eo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
project_root="$(cd -- "${script_dir}/../.." && pwd -P)"

map_dir="${FASTLIO_GLOBAL_MAP_DIR:-${project_root}/maps/fastlio_global_3d}"
metadata="${map_dir}/metadata.csv"

if [[ ! -f "${metadata}" ]]; then
  echo "找不到 metadata.csv：${metadata}"
  exit 1
fi

keyframes=$(( $(wc -l < "${metadata}") - 1 ))
echo "地图目录：${map_dir}"
echo "Keyframes: ${keyframes}"

if [[ "${keyframes}" -lt 10 ]]; then
  echo "状态：地图太小，不足以可靠重定位。请走遍场地重新采图并保存。"
  exit 2
fi

echo "状态：可用于重定位的候选地图。"
