#!/usr/bin/env bash
set -euo pipefail

# 真机单目标流程：地面切入 OFFBOARD -> 程序自动解锁并起飞到 0.75m
# -> 稳定检查通过后等待 RViz 单目标。
# 飞手仍然负责用遥控器切入 OFFBOARD，并必须全程保持接管准备。

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
target_altitude_m="0.75"
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
    raw="$(timeout 3 ros2 topic echo --once \
      --qos-history keep_last \
      --qos-depth 1 \
      --qos-reliability reliable \
      --qos-durability transient_local \
      /mavros/state 2>/dev/null || true)"
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
python3 - "${canonical_tuning_file}" "${tuning_file}" "${target_altitude_m}" <<'PY'
import pathlib
import sys
import yaml

source, destination, altitude_text = sys.argv[1:]
altitude = float(altitude_text)
with open(source, encoding="utf-8") as stream:
    tuning = yaml.safe_load(stream)
tuning["ego_planner"]["fixed_flight_height"] = altitude
tuning["offboard"]["fixed_flight_height_m"] = altitude
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
print(str(tuning["ego_planner"]["enable"]).lower())
PY
)"; then
  echo "[错误] 无法生成或校验 0.75m 本场调参文件。" >&2
  exit 2
fi
if [[ "${ego_enabled}" == false ]]; then
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
    printf '%s\n' "${state}" >&2
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
      echo "[就绪] 完整启动栈已经 PASS。"
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
            print $0
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
  echo "[错误] 30 秒内未连续满足地面起飞门禁，禁止起飞。" >&2
  printf '%s\n' "$(control_status_text)" >&2
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
  echo "[就绪] Offboard 运行时定高已确认为 ${target_altitude_m}m。"
}

request_takeoff() {
  local output
  echo "[指令] 起飞门禁已通过，请求自动解锁并垂直起飞到 ${target_altitude_m}m。"
  output="$(timeout 10 ros2 service call /race/takeoff std_srvs/srv/Trigger '{}' 2>&1)" || {
    printf '%s\n' "${output}" >&2
    echo "[错误] /race/takeoff 服务调用失败。" >&2
    return 1
  }
  printf '%s\n' "${output}"
  if ! grep -Eq 'success[=:][[:space:]]*(true|True)' <<<"${output}"; then
    echo "[错误] 起飞服务未返回 success=true。" >&2
    return 1
  fi
}

wait_takeoff_stable() {
  TARGET_ALTITUDE_M="${target_altitude_m}" python3 - <<'PY'
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
min_stable_agl_m = 0.55
max_stable_agl_m = 0.80
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
previous_pose = None
filtered_pose_vertical_speed = 0.0
result = 1
try:
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        state, odom, flight_ready = latest["state"], latest["odom"], latest["flight_ready"]
        reference = latest["reference"]
        now = time.monotonic()
        if flight_ready is False:
            print("[错误] 起飞中 flight_ready=false；请查看 /ev_health/diagnostics 并立即遥控接管。", file=sys.stderr, flush=True)
            break
        if state is None or odom is None or reference is None:
            continue
        if state.armed:
            armed_seen = True
        if armed_seen and (not state.armed or state.mode != "OFFBOARD"):
            print(
                f"[错误] 起飞中飞控状态丢失：armed={int(state.armed)} mode={state.mode}；"
                "请立即遥控接管。",
                file=sys.stderr,
                flush=True,
            )
            break
        position = odom.pose.pose.position
        velocity = odom.twist.twist.linear
        ground_z_local_enu = -reference.ground_z_local_ned
        height = position.z - ground_z_local_enu
        horizontal_speed = math.hypot(velocity.x, velocity.y)
        vertical_speed = abs(velocity.z)
        if previous_pose is not None:
            pose_dt = now - previous_pose[0]
            if 0.005 <= pose_dt <= 0.25:
                pose_rate = (position.z - previous_pose[1]) / pose_dt
                filtered_pose_vertical_speed = (
                    0.15 * pose_rate + 0.85 * filtered_pose_vertical_speed
                )
        previous_pose = (now, position.z)
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
        vertical_speed_ok = vertical_speed <= 0.10
        good = (
            inputs_fresh and state.connected and state.armed and state.mode == "OFFBOARD" and
            flight_ready is True and reference_ok and height_ok and
            horizontal_speed_ok and vertical_speed_ok
        )
        if good:
            stable_since = stable_since or now
            if now - stable_since >= 3.0:
                print(
                    f"[就绪] 自动起飞完成并稳定 3 秒："
                    f"height={height:.3f}m horizontal_speed={horizontal_speed:.3f}m/s "
                    f"vertical_speed={vertical_speed:.3f}m/s flight_ready={flight_ready}",
                    flush=True,
                )
                result = 0
                break
        else:
            stable_since = None
        if now >= next_report:
            print(
                f"[起飞检查] armed={int(state.armed)} mode={state.mode} flight_ready={flight_ready} "
                f"AGL={height:.3f}/{min_stable_agl_m:.2f}-{max_stable_agl_m:.2f}m "
                f"target={target:.2f}m reference_ok={int(reference_ok)} "
                f"height_ok={int(height_ok)} horizontal_speed={horizontal_speed:.3f}m/s "
                f"horizontal_speed_ok={int(horizontal_speed_ok)} "
                f"vertical_speed={vertical_speed:.3f}m/s vertical_speed_ok={int(vertical_speed_ok)} "
                f"pose_vertical_speed={abs(filtered_pose_vertical_speed):.3f}m/s "
                f"velocity_disagreement={abs(vertical_speed - abs(filtered_pose_vertical_speed)):.3f}m/s",
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

wait_message() {
  local topic="$1"
  local description="$2"
  local deadline=$((SECONDS + 120))
  while ((SECONDS < deadline)); do
    if timeout 3 ros2 topic echo --once "${topic}" >/dev/null 2>&1; then
      return 0
    fi
    kill -0 "${child_pid}" 2>/dev/null || { echo "[错误] 启动栈已退出。" >&2; return 1; }
  done
  echo "[错误] 等待超时：${description}（${topic}）。" >&2
  return 1
}

wait_tracking() {
  echo "[检查] 已收到目标点，等待规划输出并进入 TRACKING。"
  set +o pipefail
  if timeout 30 ros2 topic echo /race/control/status --field data 2>/dev/null |
    awk '/state=TRACKING/ {print "[就绪] 控制状态已进入 TRACKING。"; print $0; fflush(); passed=1; exit 0}
         END {if (!passed) exit 1}'
  then
    set -o pipefail
    return 0
  fi
  set -o pipefail
  echo "[错误] 30 秒内未进入 TRACKING；请立即切回 POSITION/STABILIZED 接管。" >&2
  printf '%s\n' "$(control_status_text)" >&2
  return 1
}

cat <<EOF
============================================================
       地面 OFFBOARD -> 自动起飞 0.75m -> 单目标
============================================================
本脚本会启动导航、MID-360、项目内 FR-LIO、全局重定位、MAVROS/EV、RViz 和 rosbag。
第二次回车后，只有全部地面门禁通过，程序才会调用起飞服务，
然后自动解锁、垂直起飞并定高 ${target_altitude_m}m。

规划地图：${map_file}
重定位目录：${global_map_source_dir}
关键帧数：${keyframe_count}
规划后端：请求=${requested_planner_backend}，实际=${effective_planner_backend}，EGO开关=${ego_enabled}
本场定高调参：${tuning_file}

开始前必须确认：
1. 飞机在地面、未解锁，螺旋桨/电池/遥控器和飞行区域已检查；
2. 遥控器可随时切回 POSITION/STABILIZED，飞手全程握住遥控器；
3. 本次只飞低空、短距离、单目标；
4. 现有控制节点没有水平漂移保护和起飞瞬态保护。
============================================================
[回车 1/2] 按回车启动完整程序，Ctrl+C 取消：
EOF
read -r

env \
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
  MANUAL_HANDOVER=false \
  EV_FAULT_AUTO_LAND=false \
  RECORD_BAG=true \
  RECORD_EGO_DIAGNOSTICS=false \
  COMPONENT_WINDOWS=false \
  "${stack_script}" &
child_pid=$!

wait_stack_ready || exit 3
wait_ev_healthy_stable 30 3 || exit 3
require_state 'connected:[[:space:]]*true' || exit 3
require_state 'armed:[[:space:]]*false' || exit 3
verify_runtime_altitude || exit 3

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

cat <<'EOF'

[请给点]
飞机已在 0.75m 定高稳定悬停。
现在请在 RViz 使用「2D Goal Pose」点击一个近距离、空旷目标点。
建议首次距离不超过 1m；无需再按回车，点击后会立即规划并开始飞行。
EOF

wait_message /goal_pose "RViz 单目标" || exit 6
if [[ "${effective_planner_backend}" == super ]]; then
  wait_message /race/super_planner/raw_path "Super A* 规划路径" || exit 6
else
  wait_message /race/global_path "EGO 上游全局规划路径" || exit 6
fi
wait_tracking || exit 6

echo "[已开始] 单目标已进入 TRACKING。全程保持遥控器接管准备。"
echo "[结束] 任务后请手动切回 POSITION/STABILIZED，降落并上锁，然后在本终端按 Ctrl+C。"
while kill -0 "${child_pid}" 2>/dev/null; do
  health_ok || echo "[警告] flight_ready=false；请查看 /ev_health/diagnostics 并立即人工接管。" >&2
  sleep 2
done
