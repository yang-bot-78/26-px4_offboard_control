#!/usr/bin/env bash
set -euo pipefail

# 真机单目标流程：默认地面切入 OFFBOARD -> 程序自动解锁并起飞到 0.75m
# -> 稳定检查通过后等待 RViz 单目标。使用 --manualtakeoff 时，改为
# 飞手先在 POSITION/POSCTL 起飞到安全高度并悬停，再切 OFFBOARD 直接交接导航。
# 飞手仍然负责用遥控器切入 OFFBOARD，并必须全程保持接管准备。

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
project_root="$(cd -- "${script_dir}/../.." && pwd -P)"
record_bag=true
rosbag_profile=full
bag_option=""
takeoff_mode=auto
no_ego=false

usage() {
  cat <<'EOF'
用法：切Offboard后自动起飞0.75米单目标验证.sh [--manualtakeoff] [--noego] [--norosbag|--lowbag|--lowrosbag|--trosbag]

选项：
  --manualtakeoff  飞手先在 POSITION/POSCTL 手动起飞到安全高度；通过交接门禁后，脚本自动监听
                   OFFBOARD 切换并继续执行。程序不解锁、不起飞。也接受
                   --manual-takeoff。手动目标高度由 MANUAL_TAKEOFF_ALTITUDE_M 设置。
  --noego       仅运行 Super A* 全局规划，不启动 EGO 局部规划器。
  --norosbag  本次不启动 rosbag 录制，其余飞行与安全检查保持不变。
  --lowbag     低负载录制规划/交接、FR-LIO性能和完整EV失效证据链；也接受 --lowrosbag。
  --trosbag   在 lowrosbag 规划/交接内容基础上，追加高度追踪证据链：FR-LIO、PX4 EKF、独立 rangefinder/baro、z/vz/thrust 设定值。
  -h, --help  显示本帮助。
EOF
}

while (($# > 0)); do
  case "$1" in
    --manualtakeoff|--manual-takeoff)
      [[ "${takeoff_mode}" == auto ]] || {
        echo "[错误] 自动起飞与人工起飞模式不能同时使用。" >&2
        exit 2
      }
      takeoff_mode=manual
      ;;
    --noego)
      no_ego=true
      ;;
    --norosbag)
      [[ -z "${bag_option}" ]] || {
        echo "[错误] --norosbag、--lowbag/--lowrosbag 与 --trosbag 只能选择一个。" >&2
        exit 2
      }
      record_bag=false
      bag_option="norosbag"
      ;;
    --lowbag|--lowrosbag)
      [[ -z "${bag_option}" ]] || {
        echo "[错误] --norosbag、--lowbag/--lowrosbag 与 --trosbag 只能选择一个。" >&2
        exit 2
      }
      rosbag_profile=low
      bag_option="lowbag"
      ;;
    --trosbag)
      [[ -z "${bag_option}" ]] || {
        echo "[错误] --norosbag、--lowbag/--lowrosbag 与 --trosbag 只能选择一个。" >&2
        exit 2
      }
      rosbag_profile=trace
      bag_option="trosbag"
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[错误] 不支持的参数：$1。使用 --help 查看用法。" >&2
      exit 2
      ;;
  esac
  shift
done

find_unique_file_by_suffix() {
  local directory="$1"
  local suffix="$2"
  local description="$3"
  local -a matches=()

  [[ -d "${directory}" ]] || {
    echo "[错误] ${description}目录不存在：${directory}" >&2
    return 2
  }
  mapfile -d '' -t matches < <(
    find "${directory}" -maxdepth 1 -type f -iname "*.${suffix}" -print0 | sort -z
  )
  if ((${#matches[@]} != 1)); then
    echo "[错误] ${directory} 根目录必须有且只有一个 .${suffix} ${description}，当前找到 ${#matches[@]} 个。" >&2
    ((${#matches[@]} == 0)) || printf '  %s\n' "${matches[@]}" >&2
    return 2
  fi
  printf '%s\n' "${matches[0]}"
}

stack_script="${script_dir}/一键启动导航栈.sh"
frlio_config="${FRLIO_CONFIG:-${project_root}/src/fr_lio/config/indoors.yaml}"
frlio_source="${project_root}/src/fr_lio/src/laser_mapping.cpp"
frlio_executable="${project_root}/install/fr_lio/lib/fr_lio/frlio"
lever_arm_config="${MID360_LEVER_ARM_CONFIG:-${project_root}/src/px4_ros_com/config/mid360_lever_arm.conf}"
lever_arm_validator="${project_root}/tools/fastlio/校验杆臂配置.sh"
world_yaw_alignment_rad="${WORLD_YAW_ALIGNMENT_RAD:-0.0}"
target_altitude_m="0.75"
manual_handover_min_altitude_m="0.30"
manual_handover_max_altitude_m="2.00"
manual_takeoff_altitude_m="${MANUAL_TAKEOFF_ALTITUDE_M:-${target_altitude_m}}"
if ! awk -v value="${manual_takeoff_altitude_m}" \
  -v min="${manual_handover_min_altitude_m}" -v max="${manual_handover_max_altitude_m}" \
  'BEGIN {exit !(value == value && value >= min && value <= max)}'; then
  echo "[错误] MANUAL_TAKEOFF_ALTITUDE_M 必须在 ${manual_handover_min_altitude_m}-${manual_handover_max_altitude_m}m 安全范围内。" >&2
  exit 2
fi
flight_ready_false_grace_s="${FLIGHT_READY_FALSE_GRACE_S:-0.75}"
default_map_dir="${project_root}/maps/main"
global_map_source_dir="${FRLIO_GLOBAL_MAP_DIR:-${FASTLIO_GLOBAL_MAP_DIR:-${default_map_dir}}}"
map_file="${MAP_FILE:-}"
if [[ -z "${map_file}" ]]; then
  map_file="$(find_unique_file_by_suffix "${default_map_dir}" pcd "规划地图")"
fi
metadata_file="$(find_unique_file_by_suffix "${global_map_source_dir}" csv "重定位元数据")"
global_map_dir="${global_map_source_dir}"
requested_planner_backend="${PLANNER_BACKEND:-astar_ego}"
effective_planner_backend="${requested_planner_backend}"
if [[ "${no_ego}" == true ]]; then
  effective_planner_backend=super
fi
canonical_tuning_file="${TUNING_FILE:-${project_root}/src/race_bringup/config/astar_ego_tuning.yaml}"
rviz_software_rendering="${RVIZ_SOFTWARE_RENDERING:-false}"
child_pid=""
flight_timestamp="${FLIGHT_TIMESTAMP:-$(date +%Y%m%d_%H%M%S)}"
flight_date="${flight_timestamp%%_*}"
flight_run_dir="${FLIGHT_RUN_DIR:-${project_root}/flight_records/${flight_date}/flight_${flight_timestamp}}"
stack_ready_file="${flight_run_dir}/snapshot/stack_ready.state"
tuning_file="${flight_run_dir}/snapshot/astar_ego_tuning_0.75m.yaml"

state_text() {
  local state="" raw=""
  local attempts_remaining=3
  while ((attempts_remaining > 0)); do
    # /mavros/state is a live MAVROS stream (volatile durability).  Requesting
    # transient_local here is incompatible with the publisher and can make a
    # one-shot echo report no sample even while MAVROS is connected.
    # ros2 topic echo can keep running after SIGTERM during DDS teardown.
    # Bound every attempt so an unavailable state stream cannot stall cleanup.
    raw="$(timeout --kill-after=1s 3s ros2 topic echo --once \
      --qos-reliability best_effort \
      /mavros/state mavros_msgs/msg/State 2>/dev/null || true)"
    if ! grep -Eq '^connected:[[:space:]]*(true|false)$' <<<"${raw}" ||
       ! grep -Eq '^armed:[[:space:]]*(true|false)$' <<<"${raw}" ||
       ! grep -Eq '^mode:[[:space:]]*[^[:space:]]+' <<<"${raw}"; then
      # Keep a default-QoS fallback for distributions/RMWs that reject an
      # explicit reliability override for this publisher.
      raw="$(timeout --kill-after=1s 3s ros2 topic echo --once \
        /mavros/state mavros_msgs/msg/State 2>/dev/null || true)"
    fi
    # ros2 topic echo may print "A message was lost!!!" even when the command
    # exits successfully.  Accept only a complete State payload; otherwise a
    # transport diagnostic must not be mistaken for armed=false/true data.
    if grep -Eq '^connected:[[:space:]]*(true|false)$' <<<"${raw}" &&
       grep -Eq '^armed:[[:space:]]*(true|false)$' <<<"${raw}" &&
       grep -Eq '^mode:[[:space:]]*[^[:space:]]+' <<<"${raw}"; then
      state="${raw}"
      printf '%s\n' "${state}"
      return 0
    fi
    attempts_remaining=$((attempts_remaining - 1))
    sleep 0.2
  done
}

cleanup() {
  local status=$?
  local state=""
  trap - EXIT
  trap '' INT TERM HUP
  if [[ -n "${child_pid}" ]] && kill -0 "${child_pid}" 2>/dev/null; then
    echo
    echo "[停止等待] 只有明确确认 armed=false 才允许停止导航栈、EV 健康桥和外部视觉链路。" >&2
    echo "[必须操作] 请先降落并上锁；状态读取失败时同样保持整条链路。" >&2
    while kill -0 "${child_pid}" 2>/dev/null; do
      state="$(state_text)"
      if [[ -n "${state}" ]] && grep -Eq 'armed:[[:space:]]*false' <<<"${state}"; then
        break
      fi
      sleep 1
    done
    kill -TERM "${child_pid}" 2>/dev/null || true
    wait "${child_pid}" 2>/dev/null || true
  fi
  return "${status}"
}

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM HUP

[[ -x "${stack_script}" ]] || { echo "[错误] 缺少启动脚本：${stack_script}" >&2; exit 2; }
[[ -f "${lever_arm_config}" ]] || { echo "[错误] 缺少坐标配置：${lever_arm_config}" >&2; exit 2; }
[[ -f "${frlio_config}" ]] || { echo "[错误] 缺少 FR-LIO 配置：${frlio_config}" >&2; exit 2; }
[[ -f "${frlio_source}" ]] || { echo "[错误] 缺少 FR-LIO 源码：${frlio_source}" >&2; exit 2; }
[[ -x "${frlio_executable}" ]] || { echo "[错误] 缺少已编译的 FR-LIO：${frlio_executable}" >&2; exit 2; }
grep -Fq 'const auto livox_qos = rclcpp::SensorDataQoS().keep_last(5);' "${frlio_source}" || {
  echo "[错误] FR-LIO 未包含 Livox Best Effort QoS 修复，禁止启动自动起飞流程。" >&2
  exit 2
}
if [[ "${frlio_source}" -nt "${frlio_executable}" ]]; then
  echo "[错误] FR-LIO 二进制早于 Livox QoS 修复源码，禁止启动自动起飞流程。" >&2
  echo "[修复] cd ${project_root} && colcon build --packages-select fr_lio --symlink-install" >&2
  exit 2
fi
[[ -x "${lever_arm_validator}" ]] || { echo "[错误] 缺少坐标配置校验器：${lever_arm_validator}" >&2; exit 2; }
[[ -s "${map_file}" ]] || { echo "[错误] 地图不存在或为空：${map_file}" >&2; exit 2; }
[[ -s "${metadata_file}" ]] || { echo "[错误] 重定位 CSV 不存在或为空：${metadata_file}" >&2; exit 2; }
[[ -d "${global_map_source_dir}/keyframes" ]] || { echo "[错误] 重定位关键帧目录不存在。" >&2; exit 2; }
keyframe_count="$(find "${global_map_source_dir}/keyframes" -maxdepth 1 -type f -iname '*.pcd' -printf . | wc -c)"
((keyframe_count >= 10)) || { echo "[错误] 实飞重定位库至少需要 10 个关键帧，当前只有 ${keyframe_count} 个。" >&2; exit 2; }
[[ -f "${canonical_tuning_file}" ]] || { echo "[错误] 调参文件不存在：${canonical_tuning_file}" >&2; exit 2; }
[[ -t 0 ]] || { echo "[错误] 必须在交互终端执行。" >&2; exit 2; }

cd "${project_root}"
set +u
source /opt/ros/humble/setup.bash
source "${project_root}/install/setup.bash"
set -u

"${lever_arm_validator}" "${lever_arm_config}"
# 本场杆臂配置路径可由环境变量选择。
# shellcheck disable=SC1090
source "${lever_arm_config}"
if [[ "${MID360_BODY_TO_SENSOR_X_M}" != "0.0" ||
      "${MID360_BODY_TO_SENSOR_Y_M}" != "0.0" ||
      "${MID360_BODY_TO_SENSOR_Z_M}" != "0.08" ||
      "${MID360_BODY_TO_FASTLIO_YAW_RAD}" != "0.0" ||
      "${world_yaw_alignment_rad}" != "0.0" ]]; then
  echo "[错误] 自动起飞验证要求 x=0.0 y=0.0 z=0.08、安装 yaw=0.0、world yaw=0.0。" >&2
  exit 2
fi

case "${requested_planner_backend}" in
  astar_ego|super|ego|ego-shadow) ;;
  *) echo "[错误] PLANNER_BACKEND 必须是 astar_ego、super、ego 或 ego-shadow。" >&2; exit 2 ;;
esac

mkdir -p "$(dirname -- "${tuning_file}")"
python3 - "${canonical_tuning_file}" "${tuning_file}" "${target_altitude_m}" "${no_ego}" \
  "${manual_handover_min_altitude_m}" "${manual_handover_max_altitude_m}" <<'PY'
import pathlib
import sys
import yaml

source, destination, altitude_text, no_ego_text, min_height_text, max_height_text = sys.argv[1:]
altitude = float(altitude_text)
no_ego = no_ego_text.lower() == "true"
min_height = float(min_height_text)
max_height = float(max_height_text)
with open(source, encoding="utf-8") as stream:
    tuning = yaml.safe_load(stream)
tuning["ego_planner"]["fixed_flight_height"] = altitude
tuning["shared_safety"]["fault_envelope"]["z_min"] = min_height
tuning["shared_safety"]["fault_envelope"]["z_max"] = max_height
tuning["offboard"]["fixed_flight_height_m"] = altitude
tuning["offboard"]["max_safe_height_m"] = max_height
if no_ego:
    tuning["ego_planner"]["enable"] = False
destination_path = pathlib.Path(destination)
destination_path.write_text(
    "# Generated for this flight; canonical source: " + source + "\n" +
    yaml.safe_dump(tuning, allow_unicode=True, sort_keys=False),
    encoding="utf-8",
)
PY

if ! ego_enabled="$(python3 - "${project_root}" "${tuning_file}" <<'PY'
import sys

project_root, tuning_file = sys.argv[1:]
sys.path.insert(0, f"{project_root}/src/race_bringup/launch")
from astar_ego_tuning import load_tuning

tuning = load_tuning(tuning_file)
assert tuning["ego_planner"]["fixed_flight_height"] == 0.75
assert tuning["offboard"]["fixed_flight_height_m"] == 0.75
assert tuning["shared_safety"]["fault_envelope"]["z_min"] == 0.30
assert tuning["shared_safety"]["fault_envelope"]["z_max"] == 2.00
assert tuning["offboard"]["max_safe_height_m"] == 2.00
print(str(tuning["ego_planner"]["enable"]).lower())
PY
)"; then
  echo "[错误] 无法生成或校验 0.75m 本场调参文件。" >&2
  exit 2
fi
if [[ "${no_ego}" == true ]]; then
  [[ "${ego_enabled}" == false ]] || {
    echo "[错误] --noego 未能关闭调参快照中的 EGO。" >&2
    exit 2
  }
  effective_planner_backend=super
elif [[ "${ego_enabled}" == false ]]; then
  case "${requested_planner_backend}" in
    astar_ego|ego|ego-shadow) effective_planner_backend=super ;;
  esac
fi

# 重定位后端固定读取 metadata.csv；非标准文件名在本场目录中转成标准结构。
if [[ "${metadata_file}" != "${global_map_source_dir}/metadata.csv" ]]; then
  global_map_dir="${flight_run_dir}/relocalization_map"
  mkdir -p "${global_map_dir}"
  ln -s "${metadata_file}" "${global_map_dir}/metadata.csv"
  ln -s "${global_map_source_dir}/keyframes" "${global_map_dir}/keyframes"
fi

if [[ "${rviz_software_rendering}" == true ]]; then
  export LIBGL_ALWAYS_SOFTWARE=1
else
  unset LIBGL_ALWAYS_SOFTWARE
fi

control_status_text() {
  timeout 5 ros2 topic echo --once /race/control/status --field data 2>/dev/null || true
}

# 话题字段保持稳定的英文键名供程序解析；终端显示时转换为中文。
format_control_status() {
  sed -E \
    -e 's/state=IDLE_HOLD/状态=空闲悬停/g' \
    -e 's/state=TRACKING/状态=跟踪中/g' \
    -e 's/source=safety_hold/来源=安全悬停/g' \
    -e 's/reason=no active goal/原因=没有活动目标/g' \
    -e 's/manual_handover=/人工交接=/g' \
    -e 's/pilot_override=/飞手接管=/g' \
    -e 's/armed=/已解锁=/g' \
    -e 's/offboard=/外部控制=/g' \
    -e 's/handover_ready=/交接条件就绪=/g' \
    -e 's/handover_accepted=/交接已接受=/g' \
    -e 's/map_local_alignment=/地图本地对齐=/g' \
    -e 's/position_valid=/位置有效=/g' \
    -e 's/ev_ready=/EV就绪=/g' \
    -e 's/position_aligned=/位置已对齐=/g' \
    -e 's/speed_safe=/速度安全=/g' \
    -e 's/height_safe=/高度安全=/g' \
    -e 's/current_height_m=/当前高度米=/g' \
    -e 's/current_agl_m=/当前距地高度米=/g' \
    -e 's/altitude_reference_valid=/高度基准有效=/g' \
    -e 's/hold_height_m=/悬停高度米=/g' \
    -e 's/speed_mps=/速度米每秒=/g' \
    -e 's/vertical_speed_mps=/垂直速度米每秒=/g'
}

health_ok() {
  [[ "$(ev_flight_ready_state)" == "true" ]]
}

ev_flight_ready_state() {
  timeout 5 ros2 topic echo --once /ev_health/flight_ready --field data 2>/dev/null |
    sed -n '/^[[:space:]]*---[[:space:]]*$/d; /^[[:space:]]*$/d; 1p' |
    sed -E 's/^[[:space:]]*data:[[:space:]]*//; s/["'\'']//g'
}

require_state() {
  local expected="$1"
  local state
  state="$(state_text)"
  if [[ -z "${state}" ]]; then
    echo "[错误] 连续 3 次未收到 /mavros/state，无法确认飞控状态。" >&2
    return 1
  fi
  if ! grep -Eq "${expected}" <<<"${state}"; then
    echo "[错误] 当前飞控状态不满足要求：" >&2
    printf '%s\n' "${state}" | format_control_status >&2
    return 1
  fi
}

wait_ev_healthy_stable() {
  local timeout_sec="${1:-30}"
  local stable_sec="${2:-3}"
  python3 "${script_dir}/等待flight_ready稳定.py" \
    --timeout-s "${timeout_sec}" --stable-s "${stable_sec}"
}

wait_stack_ready() {
  local deadline=$((SECONDS + 300))
  echo "[等待] 完整启动栈通过全部检查（包括全局重定位、RViz 和 rosbag）。"
  while ((SECONDS < deadline)); do
    if [[ -f "${stack_ready_file}" ]] && grep -Fxq 'state=ready' "${stack_ready_file}"; then
      echo "[就绪] 完整启动栈已通过全部检查。"
      return 0
    fi
    kill -0 "${child_pid}" 2>/dev/null || { echo "[错误] 启动栈在就绪前退出。" >&2; return 1; }
    sleep 1
  done
  echo "[错误] 等待完整启动栈超时。" >&2
  return 1
}

wait_ground_takeoff_gate() {
  echo "[检查] 复核地面自动起飞条件：未解锁、OFFBOARD、EV/位置/地图对齐有效、飞机静止。"
  set +o pipefail
  if timeout 35 ros2 topic echo /race/control/status --field data 2>/dev/null |
    awk '
      function display(line) {
        gsub("state=IDLE_HOLD", "状态=空闲悬停", line)
        gsub("manual_handover=", "人工交接=", line)
        gsub("armed=", "已解锁=", line)
        gsub("offboard=", "外部控制=", line)
        gsub("map_local_alignment=", "地图本地对齐=", line)
        gsub("position_valid=", "位置有效=", line)
        gsub("ev_ready=", "EV就绪=", line)
        gsub("position_aligned=", "位置已对齐=", line)
        gsub("speed_safe=", "速度安全=", line)
        gsub("height_safe=", "高度安全=", line)
        gsub("current_height_m=", "当前高度米=", line)
        gsub("current_agl_m=", "当前距地高度米=", line)
        gsub("speed_mps=", "速度米每秒=", line)
        return line
      }
      BEGIN {ready_since = 0; next_report = systime(); passed = 0}
      /state=IDLE_HOLD/ && /manual_handover=0/ && /armed=0/ && /offboard=1/ &&
      /map_local_alignment=1/ && /position_valid=1/ && /ev_ready=1/ &&
      /position_aligned=1/ && /speed_safe=1/ {
        height = 999
        if (match($0, /current_height_m=[-+0-9.eE]+/)) {
          height = substr($0, RSTART + 17, RLENGTH - 17) + 0
        }
        if (height >= -0.15 && height <= 0.20) {
          if (ready_since == 0) ready_since = systime()
          if (systime() - ready_since >= 3) {
            print "[就绪] 地面起飞条件已连续稳定 3 秒："
            print display($0)
            fflush()
            passed = 1
            exit 0
          }
          next
        }
      }
      /state=/ {
        ready_since = 0
        if (systime() >= next_report) {
          print "[等待详情] " display($0)
          fflush()
          next_report = systime() + 3
        }
      }
      END {if (!passed) exit 1}
    '
  then
    set -o pipefail
    return 0
  fi
  set -o pipefail
  echo "[错误] 30 秒内未连续满足地面起飞门禁，禁止起飞。" >&2
  control_status_text | format_control_status >&2
  return 1
}

wait_manual_handover_gate() {
  echo "[检查] 复核人工起飞交接条件：POSITION/POSCTL、悬停高度、速度、位置和 EV 全部安全。"
  echo "[范围] 人工交接高度 ${manual_handover_min_altitude_m}-${manual_handover_max_altitude_m}m，水平速度不超过 0.20m/s。"
  set +o pipefail
  if MANUAL_MIN_HEIGHT="${manual_handover_min_altitude_m}" \
    MANUAL_MAX_HEIGHT="${manual_handover_max_altitude_m}" \
    timeout 65 ros2 topic echo /race/control/status --field data 2>/dev/null |
    MANUAL_MIN_HEIGHT="${manual_handover_min_altitude_m}" \
    MANUAL_MAX_HEIGHT="${manual_handover_max_altitude_m}" awk '
      function display(line) {
        gsub("state=IDLE_HOLD", "状态=空闲悬停", line)
        gsub("source=safety_hold", "来源=安全悬停", line)
        gsub("reason=no active goal", "原因=没有活动目标", line)
        gsub("manual_handover=", "人工交接=", line)
        gsub("pilot_override=", "飞手接管=", line)
        gsub("armed=", "已解锁=", line)
        gsub("offboard=", "外部控制=", line)
        gsub("handover_ready=", "交接条件就绪=", line)
        gsub("handover_accepted=", "交接已接受=", line)
        gsub("map_local_alignment=", "地图本地对齐=", line)
        gsub("position_valid=", "位置有效=", line)
        gsub("ev_ready=", "EV就绪=", line)
        gsub("position_aligned=", "位置已对齐=", line)
        gsub("speed_safe=", "速度安全=", line)
        gsub("height_safe=", "高度安全=", line)
        gsub("current_height_m=", "当前高度米=", line)
        gsub("current_agl_m=", "当前距地高度米=", line)
        gsub("speed_mps=", "速度米每秒=", line)
        gsub("vertical_speed_mps=", "垂直速度米每秒=", line)
        return line
      }
      BEGIN {
        ready_since = 0; next_report = systime(); passed = 0
        min_height = ENVIRON["MANUAL_MIN_HEIGHT"] + 0
        max_height = ENVIRON["MANUAL_MAX_HEIGHT"] + 0
      }
      /state=IDLE_HOLD/ && /manual_handover=1/ && /armed=1/ && /offboard=0/ &&
      /position_valid=1/ && /ev_ready=1/ &&
      /position_aligned=1/ && /speed_safe=1/ && /handover_ready=1/ {
        height = 999
        if (match($0, /current_height_m=[-+0-9.eE]+/)) {
          height = substr($0, RSTART + 17, RLENGTH - 17) + 0
        }
        if (height >= min_height && height <= max_height) {
          if (ready_since == 0) ready_since = systime()
          if (systime() - ready_since >= 3) {
            print "[就绪] 人工起飞高度和 Offboard 交接门禁已连续稳定 3 秒："
            print display($0)
            fflush()
            passed = 1
            exit 0
          }
          next
        }
      }
      /state=/ {
        ready_since = 0
        if (systime() >= next_report) {
          print "[等待详情] " display($0)
          fflush()
          next_report = systime() + 3
        }
      }
      END {if (!passed) exit 1}
    '
  then
    set -o pipefail
    return 0
  fi
  set -o pipefail
  echo "[等待] 60 秒内未持续满足人工起飞交接门禁；保持 POSITION/POSCTL 后脚本将继续监听。" >&2
  control_status_text | format_control_status >&2
  return 1
}

wait_manual_position_handover() {
  local state last_report=0
  echo "[等待] 请由飞手手动解锁起飞；切到 POSITION/POSCTL 后脚本会自动确认当前位置。"
  while kill -0 "${child_pid}" 2>/dev/null; do
    state="$(state_text)"
    if [[ -z "${state}" ]]; then
      sleep 0.2
      continue
    fi
    if grep -Eq 'armed:[[:space:]]*true' <<<"${state}" &&
      grep -Eq "mode:[[:space:]]+['\"]?(POSITION|POSCTL)['\"]?" <<<"${state}"; then
      echo "[检测] 已进入 POSITION/POSCTL，开始自动复核交接门禁。"
      if wait_manual_handover_gate; then
        capture_planning_start_position
        return $?
      fi
      echo "[等待] POSITION/POSCTL 交接条件尚未持续满足；保持悬停后脚本会继续监听。" >&2
      sleep 0.5
      continue
    fi
    if ((SECONDS >= last_report)); then
      echo "[等待] 尚未检测到已解锁的 POSITION/POSCTL；请完成手动起飞后切入 POSITION/POSCTL。"
      last_report=$((SECONDS + 3))
    fi
    sleep 0.5
  done
  echo "[错误] 启动栈已退出，无法等待人工 POSITION/POSCTL 交接。" >&2
  return 1
}

capture_planning_start_position() {
  local topic output x y z
  for topic in /race/ego/odom /race/odom; do
    output="$(timeout 5 ros2 topic echo --once "${topic}" nav_msgs/msg/Odometry \
      --field pose.pose.position 2>/dev/null || true)"
    x="$(sed -n -E 's/^[[:space:]]*x:[[:space:]]*([-+0-9.eE]+)[[:space:]]*$/\1/p' <<<"${output}")"
    y="$(sed -n -E 's/^[[:space:]]*y:[[:space:]]*([-+0-9.eE]+)[[:space:]]*$/\1/p' <<<"${output}")"
    z="$(sed -n -E 's/^[[:space:]]*z:[[:space:]]*([-+0-9.eE]+)[[:space:]]*$/\1/p' <<<"${output}")"
    if [[ -n "${x}" && -n "${y}" && -n "${z}" ]] &&
      awk -v x="${x}" -v y="${y}" -v z="${z}" \
        'BEGIN {exit !(x == x && y == y && z == z)}'; then
      echo "[规划起点] 已在 POSITION/POSCTL 采样实时坐标：${topic}，XYZ=(${x}, ${y}, ${z})。"
      echo "[规划起点] Offboard 将固定 POSITION 起飞点 XY，并使用切换前 POSITION 高度；不会改写飞控估计坐标。"
      return 0
    fi
  done
  echo "[错误] 未收到可用的 /race/ego/odom 或 /race/odom，无法确认规划起点。" >&2
  return 1
}

wait_handover_accepted() {
  local state status last_report=0
  echo "[等待] 已通过人工交接门禁，正在监听飞手切入 OFFBOARD。"
  echo "[保护] POSITION 阶段持续跟踪当前位置；仅当位置误差不超过 0.15m 且速度、高度、EV 均安全时，控制节点才锁定当前位置并接受交接。"
  while kill -0 "${child_pid}" 2>/dev/null; do
    state="$(state_text)"
    if [[ -z "${state}" ]]; then
      sleep 0.2
      continue
    fi
    if ! grep -Eq 'armed:[[:space:]]*true' <<<"${state}"; then
      echo "[错误] 飞行器已上锁，取消等待 OFFBOARD 交接。" >&2
      return 1
    fi
    if ! grep -Eq "mode:[[:space:]]+['\"]?OFFBOARD['\"]?" <<<"${state}"; then
      if ((SECONDS >= last_report)); then
        echo "[等待] 当前仍未切入 OFFBOARD；飞机保持在 POSITION 阶段跟踪的当前位置。"
        last_report=$((SECONDS + 3))
      fi
      sleep 0.2
      continue
    fi

    status="$(control_status_text)"
    if grep -Eq 'handover_accepted=1' <<<"${status}"; then
      echo "[就绪] 已检测到 OFFBOARD，控制节点已锁定切换瞬间当前位置并接受人工交接。"
      return 0
    fi
    if ! health_ok; then
      echo "[错误] OFFBOARD 交接期间 flight_ready=false；请立即切回 POSITION。" >&2
      return 1
    fi
    if ((SECONDS >= last_report)); then
      echo "[等待] 已检测到 OFFBOARD，控制节点仍在复核交接安全条件："
      printf '%s\n' "${status}" | format_control_status
      last_report=$((SECONDS + 3))
    fi
    sleep 0.2
  done
  echo "[错误] 启动栈已退出，无法继续等待 OFFBOARD 交接。" >&2
  return 1
}

verify_runtime_altitude() {
  local output value
  output="$(timeout 5 ros2 param get /offboard_waypoint_node cruise_altitude_m 2>/dev/null || true)"
  value="$(sed -n -E 's/.*:[[:space:]]*([-+0-9.eE]+)[[:space:]]*$/\1/p' <<<"${output}")"
  if ! awk -v actual="${value:-nan}" -v expected="${target_altitude_m}" \
    'BEGIN {exit !(actual == actual && actual > expected - 0.0001 && actual < expected + 0.0001)}'; then
    echo "[错误] Offboard 运行时定高不是 ${target_altitude_m}m：${output:-无输出}" >&2
    return 1
  fi
  if [[ "${takeoff_mode}" == manual ]]; then
    echo "[就绪] Offboard 基线高度参数已确认为 ${target_altitude_m}m；人工交接将改用切入前 POSITION 高度。"
  else
    echo "[就绪] 自动起飞默认定高已确认为 ${target_altitude_m}m。"
  fi
}

verify_manual_handover_altitude_reference() {
  MANUAL_MIN_HEIGHT="${manual_handover_min_altitude_m}" \
  MANUAL_MAX_HEIGHT="${manual_handover_max_altitude_m}" python3 - <<'PY'
import math
import os
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from race_msgs.msg import FlightAltitudeReference
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data

minimum = float(os.environ["MANUAL_MIN_HEIGHT"])
maximum = float(os.environ["MANUAL_MAX_HEIGHT"])
rclpy.init()
node = rclpy.create_node("verify_manual_handover_altitude_reference")
latest = {"odom": None, "reference": None}
node.create_subscription(
    Odometry, "/mavros/local_position/odom",
    lambda message: latest.update(odom=message), qos_profile_sensor_data)
node.create_subscription(
    FlightAltitudeReference, "/race/flight_altitude_reference",
    lambda message: latest.update(reference=message),
    QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
               durability=DurabilityPolicy.TRANSIENT_LOCAL))
result = 1
strict_tolerance = 0.15
relaxed_tolerance = 0.30
relaxed_phase = False
start_time = time.monotonic()
strict_deadline = start_time + 5.0
deadline = start_time + 10.0
last_details = None
try:
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        odom = latest["odom"]
        reference = latest["reference"]
        if odom is not None and reference is not None and reference.valid:
            current_z_ned = -odom.pose.pose.position.z
            target_agl = reference.target_agl_m
            height_error = abs(reference.target_z_local_ned - current_z_ned)
            coherent = abs(
                (reference.target_z_map - reference.ground_z_map) - target_agl) <= 1.0e-3
            inside_window = minimum - 1.0e-3 <= target_agl <= maximum + 1.0e-3
            finite = all(math.isfinite(value) for value in (
                current_z_ned, target_agl, reference.target_z_local_ned,
                reference.ground_z_map, reference.target_z_map))
            last_details = (current_z_ned, reference.target_z_local_ned, target_agl, height_error)
            if finite and coherent and inside_window and height_error <= (
                relaxed_tolerance if relaxed_phase else strict_tolerance):
                phase = "放宽门禁" if relaxed_phase else "严格门禁"
                print(
                    f"[就绪] {phase}已通过；锁定 POSITION 当前高度：{target_agl:.3f}m；"
                    f"当前高度偏差 {height_error:.3f}m，规划路径将保持该高度。",
                    flush=True)
                result = 0
                break
        now = time.monotonic()
        if not relaxed_phase and now >= strict_deadline:
            relaxed_phase = True
            print(
                "[警告] 高度偏差在严格门禁 5 秒内未回到 0.15m；"
                "进入第二阶段，临时放宽至 0.30m。",
                file=sys.stderr, flush=True)
    if result:
        if last_details is None:
            print(
                "[错误] 10 秒内未收到有效的人工交接高度参考；请立即切回 POSITION。",
                file=sys.stderr, flush=True)
        else:
            current_z_ned, target_z_ned, target_agl, height_error = last_details
            print(
                f"[错误] 高度参考校验失败：当前NED z={current_z_ned:.3f}，"
                f"锁定NED z={target_z_ned:.3f}，AGL={target_agl:.3f}m，"
                f"偏差={height_error:.3f}m（上限 0.30m）；请立即切回 POSITION。",
                file=sys.stderr, flush=True)
finally:
    node.destroy_node()
    rclpy.shutdown()
sys.exit(result)
PY
}

request_takeoff() {
  local output
  echo "[指令] 起飞门禁已通过，请求自动解锁并垂直起飞到 ${target_altitude_m}m。"
  output="$(timeout 10 ros2 service call /race/takeoff std_srvs/srv/Trigger '{}' 2>&1)" || {
    echo "[错误] /race/takeoff 服务调用失败。" >&2
    return 1
  }
  if ! grep -Eq 'success[=:][[:space:]]*(true|True)' <<<"${output}"; then
    echo "[错误] 起飞服务未返回成功结果。" >&2
    return 1
  fi
  echo "[就绪] 起飞服务已接受请求。"
}

wait_takeoff_stable() {
  TARGET_ALTITUDE_M="${target_altitude_m}" \
  FLIGHT_READY_FALSE_GRACE_S="${flight_ready_false_grace_s}" python3 - <<'PY'
import math
import os
import sys
import time

import rclpy
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from race_msgs.msg import FlightAltitudeReference
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import Bool

target = float(os.environ["TARGET_ALTITUDE_M"])
flight_ready_false_grace_s = float(os.environ["FLIGHT_READY_FALSE_GRACE_S"])
if flight_ready_false_grace_s <= 0.0:
    raise ValueError("FLIGHT_READY_FALSE_GRACE_S 必须大于零")
min_stable_agl_m = 0.55
max_stable_agl_m = 0.80


def 中文飞行模式(mode):
    return {
        "OFFBOARD": "外部控制",
        "POSITION": "位置控制",
        "STABILIZED": "自稳",
    }.get(mode, mode)


rclpy.init()
node = rclpy.create_node("wait_auto_takeoff_075")
latest = {
    "state": None, "state_time": 0.0,
    "odom": None, "odom_time": 0.0,
    "flight_ready": None, "flight_ready_time": 0.0,
    "reference": None, "reference_time": 0.0,
}
node.create_subscription(
    State, "/mavros/state",
    lambda msg: latest.update(state=msg, state_time=time.monotonic()),
    qos_profile_sensor_data)
node.create_subscription(
    Odometry, "/mavros/local_position/odom",
    lambda msg: latest.update(odom=msg, odom_time=time.monotonic()),
    qos_profile_sensor_data)
node.create_subscription(
    Bool, "/ev_health/flight_ready",
    lambda msg: latest.update(flight_ready=bool(msg.data), flight_ready_time=time.monotonic()),
    qos_profile_sensor_data)
node.create_subscription(
    FlightAltitudeReference, "/race/flight_altitude_reference",
    lambda msg: latest.update(reference=msg, reference_time=time.monotonic()),
    QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    ))

deadline = time.monotonic() + 60.0
stable_since = None
armed_seen = False
next_report = 0.0
flight_ready_false_since = None
result = 1
try:
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        state, odom, flight_ready = latest["state"], latest["odom"], latest["flight_ready"]
        reference = latest["reference"]
        now = time.monotonic()
        if flight_ready is False:
            if flight_ready_false_since is None:
                flight_ready_false_since = now
                print(
                    f"[警告] 起飞中 EV 飞行就绪状态暂时为否；"
                    f"持续超过 {flight_ready_false_grace_s:.2f} 秒才中止起飞检查。",
                    file=sys.stderr,
                    flush=True,
                )
            elif now - flight_ready_false_since >= flight_ready_false_grace_s:
                print(
                    f"[错误] 起飞中 EV 飞行就绪状态持续为否超过 "
                    f"{flight_ready_false_grace_s:.2f} 秒；请查看 EV 健康诊断并立即遥控接管。",
                    file=sys.stderr,
                    flush=True,
                )
                break
        else:
            flight_ready_false_since = None
        if state is None or odom is None or reference is None:
            continue
        if state.armed:
            armed_seen = True
        if armed_seen and (not state.armed or state.mode != "OFFBOARD"):
            print(
                f"[错误] 起飞中飞控状态丢失：已解锁={'是' if state.armed else '否'} "
                f"飞行模式={中文飞行模式(state.mode)}；"
                "请立即遥控接管。",
                file=sys.stderr,
                flush=True,
            )
            break
        velocity = odom.twist.twist.linear
        ground_z_local_enu = -reference.ground_z_local_ned
        height = odom.pose.pose.position.z - ground_z_local_enu
        horizontal_speed = math.hypot(velocity.x, velocity.y)
        vertical_speed = abs(velocity.z)
        inputs_fresh = all(
            now - latest[key] <= 1.0
            for key in ("state_time", "odom_time", "flight_ready_time")
        )
        reference_ok = (
            reference.valid and math.isfinite(reference.target_agl_m) and
            abs(reference.target_agl_m - target) <= 0.001
        )
        height_ok = math.isfinite(height) and min_stable_agl_m <= height <= max_stable_agl_m
        horizontal_speed_ok = horizontal_speed <= 0.20
        good = (
            inputs_fresh and state.connected and state.armed and state.mode == "OFFBOARD" and
            flight_ready is True and reference_ok and height_ok and
            horizontal_speed_ok
        )
        if good:
            stable_since = stable_since or now
            if now - stable_since >= 3.0:
                print(
                    f"[就绪] 自动起飞完成并稳定 3 秒："
                    f"距地高度={height:.3f}米 水平速度={horizontal_speed:.3f}米/秒 "
                    f"垂直速度={vertical_speed:.3f}米/秒 EV飞行就绪={'是' if flight_ready else '否'}",
                    flush=True,
                )
                result = 0
                break
        else:
            stable_since = None
        if now >= next_report:
            print(
                f"[起飞检查] 已解锁={'是' if state.armed else '否'} "
                f"飞行模式={中文飞行模式(state.mode)} "
                f"EV飞行就绪={'是' if flight_ready else '否'} "
                f"距地高度={height:.3f}/{min_stable_agl_m:.2f}-{max_stable_agl_m:.2f}米 "
                f"目标高度={target:.2f}米 高度基准有效={'是' if reference_ok else '否'} "
                f"高度合格={'是' if height_ok else '否'} 水平速度={horizontal_speed:.3f}米/秒 "
                f"水平速度合格={'是' if horizontal_speed_ok else '否'} "
                f"垂直速度={vertical_speed:.3f}米/秒（仅供诊断）",
                flush=True,
            )
            next_report = now + 1.0
    else:
        print("[错误] 60 秒内自动起飞未通过稳定性检查；请立即遥控接管。", file=sys.stderr)
finally:
    node.destroy_node()
    rclpy.shutdown()
sys.exit(result)
PY
}

wait_planning_tracking() {
  local state status last_report=0
  echo "[等待] 飞机将保持在已锁定当前位置，直到控制节点收到有效规划路径并进入跟踪。"
  while kill -0 "${child_pid}" 2>/dev/null; do
    status="$(control_status_text)"
    if grep -Eq 'state=TRACKING' <<<"${status}"; then
      echo "[就绪] 控制节点已收到有效规划路径并进入跟踪。"
      return 0
    fi
    state="$(state_text)"
    if [[ -n "${state}" ]] &&
      (! grep -Eq 'armed:[[:space:]]*true' <<<"${state}" ||
       ! grep -Eq "mode:[[:space:]]+['\"]?OFFBOARD['\"]?" <<<"${state}"); then
      echo "[错误] 飞手已退出 OFFBOARD 或已上锁，停止等待规划路径。" >&2
      return 1
    fi
    if ((SECONDS >= last_report)); then
      echo "[等待] 尚未收到有效规划路径；飞机继续在当前位置悬停。"
      printf '%s\n' "${status}" | format_control_status
      last_report=$((SECONDS + 5))
    fi
    sleep 0.5
  done
  echo "[错误] 启动栈已退出，停止等待规划路径。" >&2
  return 1
}

cat <<EOF
============================================================
       地面 OFFBOARD -> $([[ "${takeoff_mode}" == manual ]] && echo "人工起飞交接" || echo "自动起飞 0.75m") -> 单目标
============================================================
本脚本会启动导航、MID-360、项目内 FR-LIO、全局重定位、MAVROS/EV、RViz 和 rosbag。
$(if [[ "${no_ego}" == true ]]; then echo "当前模式：仅 Super A* 全局规划，不启动 EGO 局部规划器。"; fi)
$(if [[ "${takeoff_mode}" == manual ]]; then
  echo "人工模式：先切到 POSITION/POSCTL，在该模式手动起飞到 ${manual_takeoff_altitude_m}m 附近（交接允许 ${manual_handover_min_altitude_m}-${manual_handover_max_altitude_m}m）并悬停，再切 OFFBOARD。"
  echo "程序不会解锁或起飞；控制节点会锁定 POSITION 起飞点 XY，并沿用切入 OFFBOARD 前的 POSITION 高度。"
else
  echo "自动模式：第二次回车后，只有全部地面门禁通过，程序才会调用起飞服务，"
  echo "然后自动解锁、垂直起飞并定高 ${target_altitude_m}m。"
fi)

规划地图：${map_file}
重定位目录：${global_map_source_dir}
关键帧数：${keyframe_count}
规划后端：请求=${requested_planner_backend}，实际=${effective_planner_backend}，EGO开关=${ego_enabled}
本场定高调参：${tuning_file}
录制 rosbag：$([[ "${record_bag}" == true ]] && echo 是 || echo 否)（模式：${rosbag_profile}）
起飞模式：${takeoff_mode}

开始前必须确认：
1. 螺旋桨/电池/遥控器和飞行区域已检查；
2. 遥控器可随时切回 POSITION/STABILIZED，飞手全程握住遥控器；
3. 本次只飞低空、短距离、单目标；
4. 现有控制节点没有水平漂移保护和起飞瞬态保护。
============================================================
[回车 $([[ "${takeoff_mode}" == manual ]] && echo "1/1" || echo "1/2")] 按回车启动完整程序，Ctrl+C 取消：
EOF
read -r

# The stack must not share this terminal's foreground process group: Ctrl+C
# first enters the fail-closed cleanup above and must not directly stop MAVROS.
# --wait retains a single child PID for readiness checks and later safe teardown.
setsid --wait env \
  FLIGHT_TIMESTAMP="${flight_timestamp}" \
  FLIGHT_RUN_DIR="${flight_run_dir}" \
  MID360_LEVER_ARM_CONFIG="${lever_arm_config}" \
  MID360_BODY_TO_SENSOR_X_M="${MID360_BODY_TO_SENSOR_X_M}" \
  MID360_BODY_TO_SENSOR_Y_M="${MID360_BODY_TO_SENSOR_Y_M}" \
  MID360_BODY_TO_SENSOR_Z_M="${MID360_BODY_TO_SENSOR_Z_M}" \
  MID360_BODY_TO_FASTLIO_YAW_RAD="${MID360_BODY_TO_FASTLIO_YAW_RAD}" \
  WORLD_YAW_ALIGNMENT_RAD="${world_yaw_alignment_rad}" \
  MAP_FILE="${map_file}" \
  FRLIO_GLOBAL_MAP_DIR="${global_map_dir}" \
  LIO_BACKEND=fr_lio \
  FRLIO_CONFIG="${frlio_config}" \
  RELOCALIZATION_ENABLED=true \
  PLANNER_BACKEND="${effective_planner_backend}" \
  TUNING_FILE="${tuning_file}" \
  MISSION_ENABLED=false \
  RECOGNITION_ENABLED=false \
  RVIZ=true \
  ENABLE_OUTPUT=true \
  WAYPOINT_FSM_ENABLED="${WAYPOINT_FSM_ENABLED:-false}" \
  MANUAL_HANDOVER="$([[ "${takeoff_mode}" == manual ]] && echo true || echo false)" \
  EV_FAULT_AUTO_LAND=false \
  RECORD_BAG="${record_bag}" \
  ROSBAG_PROFILE="${rosbag_profile}" \
  RECORD_EGO_DIAGNOSTICS=false \
  COMPONENT_WINDOWS=false \
  "${stack_script}" &
child_pid=$!

wait_stack_ready || exit 3
wait_ev_healthy_stable 30 3 || exit 3
require_state 'connected:[[:space:]]*true' || exit 3
require_state 'armed:[[:space:]]*false' || exit 3
verify_runtime_altitude || exit 3

if [[ "${takeoff_mode}" == manual ]]; then
  cat <<EOF

[自动监听]
请手动解锁，切到 POSITION/POSCTL 后在该模式起飞到 ${manual_takeoff_altitude_m}m 附近（允许 ${manual_handover_min_altitude_m}-${manual_handover_max_altitude_m}m），松杆悬停。
无需再按回车；脚本检测到 POSITION/POSCTL 后会自动检查高度与交接门禁，并锁定起飞点 XY 与切换前高度作为 Offboard 参考。
EOF
  wait_manual_position_handover || exit 4

  cat <<'EOF'

[自动监听]
控制节点正在后台连续发送“POSITION 起飞点 XY + 当前高度”悬停设定值。
现在仅由飞手将遥控器切到 OFFBOARD；本脚本会自动检测并继续，无需按回车。
若没有目标点或规划路径，飞机将锁定在 POSITION 起飞点 XY、切换前高度悬停，持续等待。
EOF
  wait_handover_accepted || exit 5
  verify_manual_handover_altitude_reference || exit 5
else
  cat <<'EOF'

[回车 2/2]
请保持飞机在地面且不要解锁，现在用遥控器切到 OFFBOARD。
确认飞控已显示 OFFBOARD 后按回车。
回车后脚本会复核状态；通过后将立即自动解锁并起飞，不再询问。
EOF
  read -r

  require_state 'connected:[[:space:]]*true' || exit 4
  require_state 'armed:[[:space:]]*false' || exit 4
  require_state "mode:[[:space:]]+['\"]?OFFBOARD['\"]?" || exit 4
  wait_ev_healthy_stable 30 3 || exit 4
  wait_ground_takeoff_gate || exit 4
  require_state 'armed:[[:space:]]*false' || exit 4
  require_state "mode:[[:space:]]+['\"]?OFFBOARD['\"]?" || exit 4
  request_takeoff || exit 5
  wait_takeoff_stable || exit 5
fi

cat <<'EOF'

[等待规划]
飞机已锁定在切换至 OFFBOARD 时的位置悬停。
EOF
if [[ "${WAYPOINT_FSM_ENABLED:-false}" == true ]]; then
  echo "航点状态机已接管目标序列：无需在 RViz 手动点目标；等待状态机发布第一个航点。"
else
  echo "现在可在 RViz 使用「2D Goal Pose」点击一个近距离、空旷目标点。"
  echo "建议首次距离不超过 1m；收到目标和有效规划路径后，飞机才开始飞行。"
fi

wait_planning_tracking || exit 6

echo "[已开始] 单目标已进入 TRACKING。全程保持遥控器接管准备。"
echo "[结束] 任务后请手动切回 POSITION/STABILIZED，降落并上锁，然后在本终端按 Ctrl+C。"
while kill -0 "${child_pid}" 2>/dev/null; do
  health_ok || echo "[警告] flight_ready=false；请查看 /ev_health/diagnostics 并立即人工接管。" >&2
  sleep 2
done
