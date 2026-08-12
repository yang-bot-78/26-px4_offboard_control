#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
project_root="$(cd -- "${script_dir}/../.." && pwd -P)"

config_path="${1:-${project_root}/src/px4_ros_com/config/mid360_lever_arm.conf}"

if [[ ! -f "${config_path}" ]]; then
  echo "缺少 MID360 杆臂配置文件：${config_path}" >&2
  exit 1
fi

# 这是仓库自带的 shell 赋值文件，不是操作员输入。
source "${config_path}"

required_names=(
  MID360_LEVER_ARM_CALIBRATED
  MID360_INSTALLATION_YAW_VERIFIED
  MID360_BODY_TO_SENSOR_X_M
  MID360_BODY_TO_SENSOR_Y_M
  MID360_BODY_TO_SENSOR_Z_M
  MID360_BODY_TO_FASTLIO_YAW_RAD
)
for name in "${required_names[@]}"; do
  if [[ ! -v "${name}" ]]; then
    echo "missing ${name} in ${config_path}" >&2
    exit 1
  fi
done

if [[ "${MID360_LEVER_ARM_CALIBRATED}" != "1" ]]; then
  echo "MID360 杆臂尚未标定；请实测控制中心到 FAST-LIO 原点的 XYZ，并更新 ${config_path}" >&2
  exit 1
fi

if [[ "${MID360_INSTALLATION_YAW_VERIFIED}" != "1" ]]; then
  echo "MID360 安装 yaw 未通过无桨轴向验证；自动飞行仍处于封锁状态" >&2
  exit 1
fi

number_pattern='^-?([0-9]+([.][0-9]*)?|[.][0-9]+)([eE][+-]?[0-9]+)?$'
for name in MID360_BODY_TO_SENSOR_X_M MID360_BODY_TO_SENSOR_Y_M MID360_BODY_TO_SENSOR_Z_M MID360_BODY_TO_FASTLIO_YAW_RAD; do
  value="${!name}"
  if [[ ! "${value}" =~ ${number_pattern} ]]; then
    echo "${name} 必须是有限小数，当前为：${value}" >&2
    exit 1
  fi
done

if ! awk \
  -v x="${MID360_BODY_TO_SENSOR_X_M}" \
  -v y="${MID360_BODY_TO_SENSOR_Y_M}" \
  -v z="${MID360_BODY_TO_SENSOR_Z_M}" \
  'BEGIN { squared=x*x+y*y+z*z; exit !(squared > 1e-8 && squared <= 4.0) }'; then
  echo "MID360 杆臂模长必须大于 0.1 mm 且不超过 2 m" >&2
  exit 1
fi

printf 'MID360 杆臂校验通过（机体 FLU，控制中心 -> FAST-LIO 原点）：x=%s m y=%s m z=%s m；R_FASTLIO_body yaw=%s rad\n' \
  "${MID360_BODY_TO_SENSOR_X_M}" \
  "${MID360_BODY_TO_SENSOR_Y_M}" \
  "${MID360_BODY_TO_SENSOR_Z_M}" \
  "${MID360_BODY_TO_FASTLIO_YAW_RAD}"
