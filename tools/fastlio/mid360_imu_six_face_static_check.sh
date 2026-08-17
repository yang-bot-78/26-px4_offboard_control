#!/usr/bin/env bash
set -euo pipefail

# Collect six stationary MID-360 IMU datasets. This is a non-flight tool:
# it never starts MAVROS, EV, Offboard, or sends a vehicle command.

usage() {
  cat <<'EOF'
用法：
  ./tools/fastlio/mid360_imu_six_face_static_check.sh [options]

按 MID-360 内置 IMU 的六个朝向分别录制一个 rosbag：
  imu_x_up, imu_x_down, imu_y_up, imu_y_down, imu_z_up, imu_z_down

轴名称指 `/livox/imu` 报告的 IMU 坐标系，不是机体坐标系。
摆放雷达前请确认传感器轴向定义。

选项：
  --sample-seconds N   每面录制时长，默认 120 秒。
  --settle-seconds N   每面录制前静置时长，默认 30 秒。
  --warmup-seconds N   开始前传感器预热静置时长，默认 100 秒。
  --output-dir PATH    输出目录；默认在 validation_records/ 下创建时间戳目录。
  -h, --help           显示帮助。

环境变量：
  START_MID360_DRIVER=1  当 `/livox/imu` 尚未发布时，启动
                         ~/livox_mid360_env/run_mid360_driver.sh；只会停止本脚本启动的驱动。
  LIVOX_MID360_ENV=PATH  覆盖 Livox 环境目录。

安全要求：
  拆下全部螺旋桨并保持飞机上锁。本脚本只录制 `/livox/imu`，不会连接飞控或发送控制命令。
EOF
}

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
project_root="$(cd -- "${script_dir}/../.." && pwd -P)"
livox_env="${LIVOX_MID360_ENV:-${HOME}/livox_mid360_env}"
sample_seconds=120
settle_seconds=30
warmup_seconds=100
output_dir=""
start_mid360_driver="${START_MID360_DRIVER:-0}"
driver_started=false
driver_pid=""
driver_pgid=""
recorder_pid=""
recorder_pgid=""
current_bag=""

while (($# > 0)); do
  case "$1" in
    --sample-seconds)
      sample_seconds="${2:-}"
      shift 2
      ;;
    --settle-seconds)
      settle_seconds="${2:-}"
      shift 2
      ;;
    --warmup-seconds)
      warmup_seconds="${2:-}"
      shift 2
      ;;
    --output-dir)
      output_dir="${2:-}"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "错误：不支持的参数：$1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ ! -t 0 ]]; then
  echo "错误：六面采集需要交互式终端。" >&2
  exit 2
fi

is_positive_integer() {
  [[ "$1" =~ ^[0-9]+$ ]] && ((10#$1 > 0))
}

for value_name in sample_seconds settle_seconds warmup_seconds; do
  value="${!value_name}"
  if ! is_positive_integer "${value}"; then
    echo "错误：${value_name} 必须是正整数，当前为 '${value}'。" >&2
    exit 2
  fi
done

case "${start_mid360_driver}" in
  0|1) ;;
  *)
    echo "错误：START_MID360_DRIVER 必须是 0 或 1。" >&2
    exit 2
    ;;
esac

if [[ -z "${output_dir}" ]]; then
  output_dir="${project_root}/validation_records/mid360_imu_six_face_$(date +%Y%m%d_%H%M%S)"
elif [[ "${output_dir}" != /* ]]; then
  output_dir="${project_root}/${output_dir}"
fi

[[ ! -e "${output_dir}" ]] || {
  echo "错误：输出目录已存在：${output_dir}" >&2
  exit 2
}
mkdir -p "$(dirname -- "${output_dir}")"

set +u
# The IMU topic itself uses sensor_msgs, but loading the Livox environment
# keeps driver and rosbag type support consistent with the hardware stack.
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
if [[ -f "${livox_env}/setup_mid360.bash" ]]; then
  # shellcheck disable=SC1091
  source "${livox_env}/setup_mid360.bash"
fi
set -u
command -v ros2 >/dev/null 2>&1 || { echo "错误：ros2 不可用。" >&2; exit 2; }

stop_process_group() {
  local pid="$1" pgid="$2" name="$3" signal="$4"
  [[ -n "${pid}" && -n "${pgid}" ]] || return 0
  if kill -0 "${pid}" 2>/dev/null || kill -0 -- "-${pgid}" 2>/dev/null; then
    echo "[停止] ${name}：发送 ${signal}。"
    kill -s "${signal}" -- "-${pgid}" 2>/dev/null || true
  fi
}

wait_for_process_exit() {
  local pid="$1" timeout_seconds="$2" deadline
  [[ -n "${pid}" ]] || return 0
  deadline=$((SECONDS + timeout_seconds))
  while kill -0 "${pid}" 2>/dev/null && ((SECONDS < deadline)); do
    sleep 1
  done
  if kill -0 "${pid}" 2>/dev/null; then
    return 1
  fi
  wait "${pid}" 2>/dev/null || true
}

stop_recorder() {
  [[ -n "${recorder_pid}" ]] || return 0
  stop_process_group "${recorder_pid}" "${recorder_pgid}" "rosbag recorder" INT
  if ! wait_for_process_exit "${recorder_pid}" 15; then
    stop_process_group "${recorder_pid}" "${recorder_pgid}" "rosbag recorder" TERM
    if ! wait_for_process_exit "${recorder_pid}" 5; then
      echo "警告：rosbag 录制器未正常退出：${current_bag}" >&2
    fi
  fi
  recorder_pid=""
  recorder_pgid=""
  current_bag=""
}

cleanup() {
  local status=$?
  trap - EXIT INT TERM HUP
  stop_recorder
  if [[ -n "${driver_pid}" ]]; then
    stop_process_group "${driver_pid}" "${driver_pgid}" "MID-360 driver started by this script" INT
    if ! wait_for_process_exit "${driver_pid}" 10; then
      stop_process_group "${driver_pid}" "${driver_pgid}" "MID-360 driver started by this script" TERM
    fi
  fi
  exit "${status}"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM HUP

wait_for_imu_message() {
  local timeout_seconds="$1"
  local deadline=$((SECONDS + timeout_seconds))
  while ((SECONDS < deadline)); do
    if timeout 3 ros2 topic echo --once /livox/imu >/dev/null 2>&1; then
      return 0
    fi
    sleep 1
  done
  return 1
}

echo
echo "================================================================"
echo "MID-360 内置 IMU 六面静态采集"
echo "输出目录：${output_dir}"
echo "本脚本只录制 /livox/imu，不启动 MAVROS、EV 或 Offboard。"
echo "请拆下全部螺旋桨，保持飞机上锁，并可靠支撑机体。"
echo "================================================================"
read -r -p "确认以上安全条件后按回车继续（Ctrl-C 取消）：" _confirm
mkdir -p "${output_dir}"

if ! wait_for_imu_message 3; then
  if [[ "${start_mid360_driver}" != 1 ]]; then
    echo "错误：/livox/imu 没有发布消息。" >&2
    echo "请先单独启动 MID-360 驱动，或使用 START_MID360_DRIVER=1 重试。" >&2
    exit 1
  fi
  driver_script="${livox_env}/run_mid360_driver.sh"
  [[ -x "${driver_script}" ]] || {
    echo "错误：MID-360 驱动不可执行：${driver_script}" >&2
    exit 2
  }
  echo "[启动] MID-360 驱动：${driver_script}"
  setsid "${driver_script}" >"${output_dir}/mid360_driver.log" 2>&1 &
  driver_pid="$!"
  driver_pgid="$!"
  if ! wait_for_imu_message 30; then
    echo "错误：等待 30 秒后仍未收到 /livox/imu。" >&2
    echo "请查看：${output_dir}/mid360_driver.log" >&2
    exit 1
  fi
else
  echo "[就绪] 使用现有的 /livox/imu 发布者。"
fi

mkdir -p "${output_dir}/bags"
metadata_file="${output_dir}/session.csv"
notes_file="${output_dir}/README.txt"

cat >"${notes_file}" <<EOF
MID-360 内置 IMU 六面静态采集

这是无桨、非飞行采集，只录制 /livox/imu。
每个 bag 都是在指定 IMU 轴竖直向上并静止后录制的。
标签指 IMU 消息坐标系，不指机体坐标系。

sample_seconds=${sample_seconds}
settle_seconds=${settle_seconds}
warmup_seconds=${warmup_seconds}
driver_start_requested=${start_mid360_driver}
driver_started_by_this_script=${driver_started}
EOF
printf 'face,imu_axis_direction,settle_seconds,sample_seconds,bag_path,started_at,finished_at,imu_messages\n' >"${metadata_file}"

countdown() {
  local seconds="$1" description="$2" remaining
  echo "${description}：${seconds} 秒"
  for ((remaining = seconds; remaining > 0; --remaining)); do
    printf '  剩余 %s 秒\r' "${remaining}"
    sleep 1
  done
  printf '%*s\r' 40 ''
}

record_face() {
  local sequence="$1" face="$2" orientation="$3" bag_path started_at finished_at imu_messages
  bag_path="${output_dir}/bags/${sequence}_${face}"

  echo
  echo "----------------------------------------------------------------"
  echo "第 ${sequence}/06 面：${face}（${orientation}）"
  echo "请将 MID-360 摆放到指定 IMU 轴竖直向上的方向。"
  read -r -p "机体已支撑牢固且完全静止后按回车：" _confirm
  countdown "${settle_seconds}" "${face} 录制前静置"

  echo "[录制] ${bag_path}"
  current_bag="${bag_path}"
  started_at="$(date -Is)"
  setsid ros2 bag record -o "${bag_path}" /livox/imu >"${bag_path}.record.log" 2>&1 &
  recorder_pid="$!"
  recorder_pgid="$!"
  sleep 2
  if ! kill -0 "${recorder_pid}" 2>/dev/null; then
    wait "${recorder_pid}" 2>/dev/null || true
    echo "错误：rosbag 录制器启动失败，请查看 ${bag_path}.record.log" >&2
    exit 1
  fi
  countdown "${sample_seconds}" "正在录制 ${face}"
  stop_recorder
  finished_at="$(date -Is)"

  ros2 bag info "${bag_path}" >"${bag_path}.info.txt" 2>&1 || {
    echo "错误：无法读取 rosbag 元数据：${bag_path}" >&2
    exit 1
  }
  imu_messages="$(sed -nE 's/.*Topic: \/livox\/imu .* Count: ([0-9]+).*/\1/p' "${bag_path}.info.txt" | head -n 1)"
  if [[ ! "${imu_messages}" =~ ^[0-9]+$ ]] || ((imu_messages == 0)); then
    echo "错误：${face} 没有写入 /livox/imu 消息。" >&2
    exit 1
  fi
  printf '%s,%s,%s,%s,%s,%s,%s,%s\n' \
    "${face}" "${orientation}" "${settle_seconds}" "${sample_seconds}" \
    "${bag_path}" "${started_at}" "${finished_at}" "${imu_messages}" >>"${metadata_file}"
  echo "[完成] ${face}：${imu_messages} 条 IMU 消息"
}

echo
echo "请保持 MID-360 静止，进行初始热稳定。"
countdown "${warmup_seconds}" "初始热稳定"

record_face 01 imu_x_up '+X up'
record_face 02 imu_x_down '-X up'
record_face 03 imu_y_up '+Y up'
record_face 04 imu_y_down '-Y up'
record_face 05 imu_z_up '+Z up'
record_face 06 imu_z_down '-Z up'

echo
echo "通过：六个 rosbag 已保存到 ${output_dir}/bags"
echo "采集元数据：${metadata_file}"
echo "下一步：对这些 bag 进行 Allan 方差和六面偏置/比例分析。"
