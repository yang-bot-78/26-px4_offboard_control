#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
project_root="$(cd -- "${script_dir}/../.." && pwd -P)"
snapshot_file="${1:-}"
timeout_sec="${PX4_ULOG_PARAM_TIMEOUT_SEC:-30}"
vision_profile_bit=128

if ! [[ "${timeout_sec}" =~ ^[1-9][0-9]*$ ]]; then
  echo "PX4_ULOG_PARAM_TIMEOUT_SEC 必须是正整数：${timeout_sec}" >&2
  exit 2
fi

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "${project_root}/install/setup.bash"
set -u

deadline=$((SECONDS + timeout_sec))
attempt=0
response=""
profile=""

echo "[ULog] 检查 PX4 SDLOG_PROFILE 是否包含视觉与避障位（bit 7）..."
while ((SECONDS < deadline)); do
  attempt=$((attempt + 1))
  response="$(timeout 8 ros2 param get /mavros/param SDLOG_PROFILE 2>&1 || true)"
  profile="$(sed -nE 's/^Integer value is:[[:space:]]*([0-9]+).*$/\1/p' <<<"${response}" | head -n 1)"
  if [[ -n "${profile}" ]]; then
    break
  fi
  sleep 1
done

if [[ -n "${snapshot_file}" ]]; then
  mkdir -p "$(dirname -- "${snapshot_file}")"
  {
    printf 'checked_at=%s\n' "$(date '+%F %T %z')"
    printf 'attempts=%s\n' "${attempt}"
    printf 'response=%s\n' "${response//$'\n'/ }"
  } >"${snapshot_file}"
fi

if [[ -z "${profile}" ]]; then
  echo "[ULog] 无法读取 /mavros/param 的 SDLOG_PROFILE；禁止在缺少 PX4 接收侧证据时开始录包。" >&2
  echo "[ULog] 最后响应：${response:-无响应}" >&2
  exit 1
fi

if (((profile & vision_profile_bit) == 0)); then
  required_profile=$((profile | vision_profile_bit))
  echo "[ULog] SDLOG_PROFILE=${profile} 未开启 bit 7，ULog 不会记录 vehicle_visual_odometry。" >&2
  echo "[ULog] 请在 QGroundControl 将 SDLOG_PROFILE 设为 ${required_profile}，保存并重启飞控后再启动飞行栈。" >&2
  exit 1
fi

echo "[ULog] PASS: SDLOG_PROFILE=${profile}，vehicle_visual_odometry 将以约 33 Hz 写入 ULog。"
