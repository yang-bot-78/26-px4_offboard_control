#!/usr/bin/env bash
set -euo pipefail

# 实飞单目标验证：程序绝不解锁、绝不起飞、绝不切换飞行模式。
# 它只在飞手已手动起飞、POSCTL/POSITION 悬停并手动切入 OFFBOARD 后，
# 才允许 RViz 的一个目标点交给导航器。
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
project_root="$(cd -- "${script_dir}/../.." && pwd -P)"

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
lever_arm_config="${MID360_LEVER_ARM_CONFIG:-${project_root}/src/px4_ros_com/config/mid360_lever_arm.conf}"
lever_arm_validator="${project_root}/tools/fastlio/校验杆臂配置.sh"
world_yaw_alignment_rad="${WORLD_YAW_ALIGNMENT_RAD:-0.0}"
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
tuning_file="${TUNING_FILE:-${project_root}/src/race_bringup/config/astar_ego_tuning.yaml}"
rviz_software_rendering="${RVIZ_SOFTWARE_RENDERING:-false}"
child_pid=""
flight_timestamp="${FLIGHT_TIMESTAMP:-$(date +%Y%m%d_%H%M%S)}"
flight_date="${flight_timestamp%%_*}"
flight_run_dir="${FLIGHT_RUN_DIR:-${project_root}/flight_records/${flight_date}/flight_${flight_timestamp}}"
stack_ready_file="${flight_run_dir}/snapshot/stack_ready.state"

# 重定位后端固定读取 metadata.csv。当用户更换的 CSV 名字不同时，
# 在本次飞行记录中建立标准结构，源地图目录保持不变。
if [[ "${metadata_file}" != "${global_map_source_dir}/metadata.csv" ]]; then
  global_map_dir="${flight_run_dir}/relocalization_map"
  mkdir -p "${global_map_dir}"
  ln -s "${metadata_file}" "${global_map_dir}/metadata.csv"
  ln -s "${global_map_source_dir}/keyframes" "${global_map_dir}/keyframes"
fi

cleanup() {
  trap - EXIT INT TERM HUP
  if [[ -n "${child_pid}" ]] && kill -0 "${child_pid}" 2>/dev/null; then
    echo
    echo "[停止] 请先用遥控器降落并上锁；确认后才会停止导航栈。"
    kill -TERM "${child_pid}" 2>/dev/null || true
    wait "${child_pid}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM HUP

[[ -x "${stack_script}" ]] || { echo "[错误] 缺少启动脚本：${stack_script}" >&2; exit 2; }
[[ -f "${lever_arm_config}" ]] || { echo "[错误] 缺少坐标配置：${lever_arm_config}" >&2; exit 2; }
[[ -f "${frlio_config}" ]] || { echo "[错误] 缺少 FR-LIO 配置：${frlio_config}" >&2; exit 2; }
[[ -x "${lever_arm_validator}" ]] || { echo "[错误] 缺少坐标配置校验器：${lever_arm_validator}" >&2; exit 2; }
[[ -s "${map_file}" ]] || { echo "[错误] 地图不存在或为空：${map_file}" >&2; exit 2; }
[[ -s "${metadata_file}" ]] || { echo "[错误] 重定位 CSV 不存在或为空：${metadata_file}" >&2; exit 2; }
[[ -d "${global_map_source_dir}/keyframes" ]] || { echo "[错误] 重定位关键帧目录不存在：${global_map_source_dir}/keyframes" >&2; exit 2; }
keyframe_count="$(find "${global_map_source_dir}/keyframes" -maxdepth 1 -type f -iname '*.pcd' -printf . | wc -c)"
((keyframe_count >= 10)) || { echo "[错误] 实飞重定位库至少需要 10 个关键帧，当前只有 ${keyframe_count} 个。" >&2; exit 2; }
[[ -f "${tuning_file}" ]] || { echo "[错误] 调参文件不存在：${tuning_file}" >&2; exit 2; }
[[ -t 0 ]] || { echo "[错误] 必须在交互终端执行。" >&2; exit 2; }

cd "${project_root}"
set +u
source /opt/ros/humble/setup.bash
source "${project_root}/install/setup.bash"
set -u

# The validated frame contract is part of this flight entry point.  Validate
# and snapshot the exact file before starting any component; the delegated
# stack script then passes these values to both the EV bridge and navigation.
"${lever_arm_validator}" "${lever_arm_config}"
source "${lever_arm_config}"
if [[ "${MID360_BODY_TO_SENSOR_X_M}" != "0.0" ||
      "${MID360_BODY_TO_SENSOR_Y_M}" != "0.0" ||
      "${MID360_BODY_TO_SENSOR_Z_M}" != "0.08" ||
      "${MID360_BODY_TO_FASTLIO_YAW_RAD}" != "0.0" ||
      "${world_yaw_alignment_rad}" != "0.0" ]]; then
  echo "[错误] 人工起飞验证要求坐标参数：x=0.0 y=0.0 z=0.08、安装 yaw=0.0、world yaw=0.0；当前配置不匹配。" >&2
  exit 2
fi

case "${requested_planner_backend}" in
  astar_ego|super|ego|ego-shadow) ;;
  *) echo "[错误] PLANNER_BACKEND 必须是 astar_ego、super、ego 或 ego-shadow。" >&2; exit 2 ;;
esac
if ! ego_enabled="$(python3 - "${project_root}" "${tuning_file}" <<'PY'
import sys

project_root, tuning_file = sys.argv[1:]
sys.path.insert(0, f'{project_root}/src/race_bringup/launch')
from astar_ego_tuning import load_tuning

print(str(load_tuning(tuning_file)['ego_planner']['enable']).lower())
PY
)"; then
  echo "[错误] 无法从调参文件读取 EGO 开关：${tuning_file}" >&2
  exit 2
fi
if [[ "${ego_enabled}" == false ]]; then
  case "${requested_planner_backend}" in
    astar_ego|ego|ego-shadow) effective_planner_backend=super ;;
  esac
fi

if [[ "${rviz_software_rendering}" == true ]]; then
  # Avoid RViz startup freezes caused by the host OpenGL driver.  This only
  # affects RViz and can be disabled with RVIZ_SOFTWARE_RENDERING=false.
  export LIBGL_ALWAYS_SOFTWARE=1
else
  unset LIBGL_ALWAYS_SOFTWARE
fi

wait_message() {
  local topic="$1"
  local description="$2"
  local deadline=$((SECONDS + 120))
  while ((SECONDS < deadline)); do
    if timeout 3 ros2 topic echo --once "${topic}" >/dev/null 2>&1; then
      return 0
    fi
    kill -0 "${child_pid}" 2>/dev/null || {
      echo "[错误] 启动栈已退出。" >&2
      return 1
    }
  done
  echo "[错误] 等待超时：${description}（${topic}）。" >&2
  return 1
}

wait_tracking() {
  local planner_label="Super A* 路径"
  if [[ "${effective_planner_backend}" != super ]]; then
    planner_label="EGO 局部轨迹"
  fi
  echo "[检查] 等待${planner_label}开始执行，控制状态必须进入 TRACKING。"
  set +o pipefail
  if timeout 30 ros2 topic echo /race/control/status --field data 2>/dev/null |
    awk '
      /state=TRACKING/ {
        print "[就绪] 规划输出已开始执行，控制状态已进入 TRACKING。"
        print $0
        fflush()
        passed = 1
        exit 0
      }
      END {if (!passed) exit 1}
    '
  then
    set -o pipefail
    return 0
  fi
  set -o pipefail
  echo "[错误] 30 秒内规划输出未进入执行状态，飞机仍保持原地悬停。" >&2
  echo "[处理] 请立即切回 POSITION，不要继续给点或等待飞机自行移动。" >&2
  printf '%s\n' "$(control_status_text)" >&2
  return 1
}

wait_stack_ready() {
  local deadline=$((SECONDS + 300))
  echo "[等待] 完整启动栈通过全部检查（包括 RViz 和 rosbag）。"
  while ((SECONDS < deadline)); do
    if [[ -f "${stack_ready_file}" ]] && grep -Fxq 'state=ready' "${stack_ready_file}"; then
      echo "[就绪] 完整启动栈已经 PASS。"
      return 0
    fi
    kill -0 "${child_pid}" 2>/dev/null || {
      echo "[错误] 启动栈在完整就绪前退出。" >&2
      return 1
    }
    sleep 1
  done
  echo "[错误] 等待完整启动栈超时：${stack_ready_file}" >&2
  return 1
}

control_status_text() {
  timeout 5 ros2 topic echo --once /race/control/status --field data 2>/dev/null || true
}

wait_handover_gate() {
  echo "[检查] 等待当前位置、悬停设定值、速度、高度和 EV 全部满足交接条件。"
  echo "[范围] 高度 0.50-0.85m，水平速度不超过 0.20m/s，位置误差不超过 0.15m。"
  echo "[要求] handover_ready 必须连续保持 3 秒；请在 POSITION 中稳定悬停，不要快速升降穿过范围。"

  # Keep one subscription alive so a short readiness window cannot be missed
  # by repeatedly starting `ros2 topic echo --once` and waiting for discovery.
  set +o pipefail
  if timeout 65 ros2 topic echo /race/control/status --field data 2>/dev/null |
    awk '
      BEGIN {ready_since = 0; next_report = systime(); passed = 0}
      /handover_ready=1/ {
        if (ready_since == 0) ready_since = systime()
        if (systime() - ready_since >= 3) {
          print "[就绪] POSITION 悬停和 Offboard 交接门禁已连续稳定 3 秒："
          print $0
          fflush()
          passed = 1
          exit 0
        }
      }
      /handover_ready=0/ {ready_since = 0}
      /state=/ {
        if (systime() >= next_report) {
          print "[等待详情] " $0
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
  echo "[错误] 60 秒内未满足人工交接门禁，禁止切 OFFBOARD。请根据 height_safe、speed_safe 等字段调整。" >&2
  printf '%s\n' "$(control_status_text)" >&2
  return 1
}

wait_handover_accepted() {
  local deadline=$((SECONDS + 10))
  local status
  while ((SECONDS < deadline)); do
    status="$(control_status_text)"
    if grep -Eq 'handover_accepted=1' <<<"${status}"; then
      echo "[就绪] 控制节点已经接受 OFFBOARD 人工交接。"
      return 0
    fi
    if ! health_ok; then
      echo "[错误] OFFBOARD 交接期间 flight_ready=false；请查看 /ev_health/diagnostics 并立即切回 POSITION。" >&2
      return 1
    fi
    sleep 1
  done
  echo "[错误] 控制节点没有接受人工交接，请立即切回 POSITION。" >&2
  printf '%s\n' "$(control_status_text)" >&2
  return 1
}

state_text() {
  timeout 5 ros2 topic echo --once /mavros/state 2>/dev/null || true
}

health_ok() {
  [[ "$(ev_flight_ready_state)" == "true" ]]
}

ev_flight_ready_state() {
  timeout 5 ros2 topic echo --once /ev_health/flight_ready --field data 2>/dev/null |
    sed -n '/^[[:space:]]*---[[:space:]]*$/d; /^[[:space:]]*$/d; 1p' |
    sed -E 's/^[[:space:]]*data:[[:space:]]*//; s/["'\'']//g'
}

wait_ev_healthy_stable() {
  local timeout_sec="${1:-30}"
  local stable_sec="${2:-3}"
  python3 "${script_dir}/等待flight_ready稳定.py" \
    --timeout-s "${timeout_sec}" --stable-s "${stable_sec}"
}

require_state() {
  local expected="$1"
  local state
  state="$(state_text)"
  if ! grep -Eq "${expected}" <<<"${state}"; then
    echo "[错误] 当前飞控状态不满足要求：" >&2
    printf '%s\n' "${state}" >&2
    return 1
  fi
}

cat <<EOF
============================================================
       人工起飞 -> POSITION -> OFFBOARD 单目标实飞验证
============================================================
本脚本会启动：导航、MID-360、项目内 FR-LIO、MAVROS/EV、RViz、rosbag。
在 MAVROS/EV 和人工起飞步骤前，必须完成 Scan Context + ICP 全局重定位。
本脚本不会：解锁、起飞、切 POSITION、切 OFFBOARD、自动执行任务航点。

规划地图：${map_file}
重定位源目录：${global_map_source_dir}
重定位 CSV：${metadata_file}
关键帧数：${keyframe_count}
导航模式：单个 RViz 目标点；未点击目标前只悬停。
规划后端：请求=${requested_planner_backend}，实际=${effective_planner_backend}，EGO开关=${ego_enabled}
RViz 渲染：${rviz_software_rendering}
坐标配置：${lever_arm_config}
坐标参数：body_to_sensor=(x=${MID360_BODY_TO_SENSOR_X_M}, y=${MID360_BODY_TO_SENSOR_Y_M}, z=${MID360_BODY_TO_SENSOR_Z_M}) m，body_to_fastlio_yaw=${MID360_BODY_TO_FASTLIO_YAW_RAD} rad，world_yaw=${world_yaw_alignment_rad} rad

开始前必须确认：
1. 螺旋桨、电池、遥控器和人工接管均已检查；
2. 飞行区域在 y=-3.20m 至 y=12.40m 的规划边界内；
3. 遥控器随时可切回 STABILIZED，且已知降落开关位置；
4. 本次只飞低空、短距离、单目标，禁止任务航点和自动起飞。
============================================================
按回车启动整链，Ctrl+C 取消：
EOF
read -r

# Keep the background flight stack in its own session so terminal Ctrl+C does
# not bypass this entry point's cleanup and terminate MAVROS immediately.
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
  MANUAL_HANDOVER=true \
  RECORD_BAG=true \
  RECORD_EGO_DIAGNOSTICS=true \
  COMPONENT_WINDOWS=false \
  "${stack_script}" &
child_pid=$!

wait_stack_ready || exit 3
wait_ev_healthy_stable 30 3 || exit 3
require_state 'connected:[[:space:]]*true' || exit 3
require_state 'armed:[[:space:]]*false' || exit 3

cat <<'EOF'

[人工步骤 1]
请手动解锁，在 STABILIZED 起飞至 0.60-0.75m，切换到 POSITION/POSCTL，松杆悬停。
确认 30 秒内定位稳定、飞机无明显漂移后，按回车继续。
EOF
read -r
wait_ev_healthy_stable 30 3 || exit 4
require_state 'armed:[[:space:]]*true' || exit 4
require_state "mode:[[:space:]]+['\"]?(POSITION|POSCTL)['\"]?" || exit 4
wait_handover_gate || exit 4

cat <<'EOF'

[人工步骤 2]
控制节点已经在后台连续发送“当前位置悬停”设定值。
现在仅由飞手将遥控器切到 OFFBOARD；本脚本不会替你切换。
切换成功且飞机保持悬停后，按回车继续。
EOF
read -r
require_state 'armed:[[:space:]]*true' || exit 5
require_state "mode:[[:space:]]+['\"]?OFFBOARD['\"]?" || exit 5
wait_handover_accepted || exit 5

cat <<'EOF'

[人工步骤 3]
现在已经是 OFFBOARD 人工交接状态，飞机应保持当前位置。
请在 RViz 使用「2D Goal Pose」点击一个近距离、空旷目标点。
首次目标建议距离不超过 1m；点击后路径生成并开始跟踪。
EOF
wait_message /goal_pose "RViz 单目标" || exit 6
if [[ "${effective_planner_backend}" == super ]]; then
  wait_message /race/super_planner/raw_path "Super A* 规划路径" || exit 6
else
  wait_message /race/global_path "EGO 上游全局规划路径" || exit 6
fi
wait_tracking || exit 6

if [[ "${effective_planner_backend}" == super ]]; then
  echo "[已开始] Super A* 路径已生成，控制状态已进入 TRACKING。全程保持遥控器接管准备。"
else
  echo "[已开始] 全局路径和经过验证的 EGO 局部轨迹均已就绪。全程保持遥控器接管准备。"
fi
echo "[结束] 请手动切回 STABILIZED 并降落、上锁；然后在本终端按 Ctrl+C。"
while kill -0 "${child_pid}" 2>/dev/null; do
  health_ok || echo "[警告] flight_ready=false；请查看 /ev_health/diagnostics 并立即人工接管。" >&2
  sleep 2
done
