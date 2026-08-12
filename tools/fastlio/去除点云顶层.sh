#!/usr/bin/env bash
set -euo pipefail

# 手动删除点云顶部：只生成输出副本，不修改输入文件。
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
project_root="$(cd -- "${script_dir}/../.." && pwd -P)"
default_input="${project_root}/maps/fastlio_global_3d_20260810/GlobalMap.pcd"

usage() {
  cat <<'EOF'
用法：
  ./tools/fastlio/去除点云顶层.sh [输入.pcd] [最高保留高度_m] [输出.pcd]

默认值：
  输入：maps/fastlio_global_3d_20260810/GlobalMap.pcd
  最高保留高度：3.0
  输出：输入文件名后追加 _去顶端_z<高度>m.pcd

示例：
  ./tools/fastlio/去除点云顶层.sh
  ./tools/fastlio/去除点云顶层.sh 地图.pcd 2.5
  ./tools/fastlio/去除点云顶层.sh 地图.pcd 3.0 新地图_去顶端.pcd
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--帮助" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

input_file="${1:-${default_input}}"
max_z="${2:-3.0}"
if [[ -n "${3:-}" ]]; then
  output_file="$3"
else
  stem="${input_file%.pcd}"
  output_file="${stem}_去顶端_z${max_z}m.pcd"
fi

[[ -f "${input_file}" ]] || { echo "[错误] 输入地图不存在：${input_file}" >&2; exit 2; }
[[ "${input_file}" == *.pcd && "${output_file}" == *.pcd ]] || {
  echo "[错误] 输入和输出必须是 .pcd 文件。" >&2
  exit 2
}
[[ "${max_z}" =~ ^-?[0-9]+([.][0-9]+)?$ ]] || {
  echo "[错误] 最高高度必须是数字，例如 3.0。" >&2
  exit 2
}
if [[ "$(realpath -m -- "${input_file}")" == "$(realpath -m -- "${output_file}")" ]]; then
  echo "[错误] 输出不能覆盖输入，请指定新的输出文件。" >&2
  exit 2
fi

mkdir -p "$(dirname -- "${output_file}")"
tmp_file="${output_file}.tmp.$$.pcd"
trap 'rm -f -- "${tmp_file}"' EXIT INT TERM HUP

echo "[输入] ${input_file}"
echo "[规则] 保留 z <= ${max_z} m（同时保留所有低于该高度的点）"
echo "[输出] ${output_file}"
pcl_passthrough_filter "${input_file}" "${tmp_file}" \
  -field z -min -100000 -max "${max_z}" -inside 1 -keep 0
mv -- "${tmp_file}" "${output_file}"
trap - EXIT INT TERM HUP

points="$(awk '$1 == "POINTS" {print $2; exit}' "${output_file}")"
[[ -n "${points}" ]] || { echo "[错误] 输出 PCD 缺少 POINTS 字段。" >&2; exit 3; }
echo "[完成] 输出点数：${points}"
echo "[提示] 原始地图未修改；规划脚本可通过 PLANNING_MAP_FILE 指向此输出。"
