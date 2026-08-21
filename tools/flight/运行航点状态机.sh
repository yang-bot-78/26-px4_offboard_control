#!/usr/bin/env bash
set -euo pipefail

# Saved-waypoint flight entrypoint.
#
# The existing single-goal script owns the real-flight safety gates, takeoff,
# OFFBOARD handover and rosbag. This wrapper starts the route state machine
# alongside it. The state machine publishes only map-frame /goal_pose goals;
# offboard_waypoint_node remains the sole PX4 setpoint publisher.

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
project_root="$(cd -- "${script_dir}/../.." && pwd -P)"
single_goal_script="${script_dir}/切Offboard后自动起飞0.75米单目标验证.sh"
fsm_node="${project_root}/src/race_offboard/mission/waypoint_state_machine_node.py"
exporter="${project_root}/src/race_offboard/mission/export_mission_waypoints.py"
mission_file="${MISSION_FILE:-${project_root}/src/race_offboard/config/waypoints/main/mission.yaml}"
waypoints_file="${WAYPOINTS_FILE:-${project_root}/src/race_offboard/config/waypoints/main/waypoints.yaml}"
fsm_log="${FSM_LOG_FILE:-${project_root}/runtime/waypoint_fsm_$(date +%Y%m%d_%H%M%S).log}"
fsm_pid=""
cleanup_started=false

usage() {
  cat <<'EOF'
用法：运行航点状态机.sh [--manualtakeoff] [--noego] [--norosbag|--lowrosbag|--trosbag]

从 WAYPOINTS_FILE 保存的全部航点生成路线后飞行（至少需要两个点）。参数会原样传给已有的
“切Offboard后自动起飞0.75米单目标验证.sh”，因此人工起飞、规划后端和录包选项保持一致。
人工或自动起飞完成、控制节点报告有效高度参考后，状态机才发布路线第一个航点；
到达每个航点后按 hold_sec 等配置切换状态并发布下一个目标。

默认航点源文件：src/race_offboard/config/waypoints/main/waypoints.yaml
默认任务路线文件：src/race_offboard/config/waypoints/main/mission.yaml
每次启动都会先用最新航点源文件重建任务路线。

可通过环境变量覆盖：MISSION_FILE、WAYPOINTS_FILE、FSM_LOG_FILE。
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

for argument in "$@"; do
  if [[ "${argument}" == "--noego" ]]; then
    echo "错误：航点状态机需要 EGO 局部规划器生成可跟踪轨迹，不能与 --noego 同时使用。" >&2
    exit 2
  fi
done

[[ -x "${single_goal_script}" ]] || {
  echo "错误：找不到单目标安全流程：${single_goal_script}" >&2
  exit 2
}
[[ -f "${fsm_node}" ]] || {
  echo "错误：找不到航点状态机节点：${fsm_node}" >&2
  exit 2
}
[[ -f "${exporter}" ]] || {
  echo "错误：找不到航点导出器：${exporter}" >&2
  exit 2
}
[[ -f "${waypoints_file}" ]] || {
  echo "错误：WAYPOINTS_FILE 不存在：${waypoints_file}" >&2
  exit 2
}

set +u
source /opt/ros/humble/setup.bash
source "${project_root}/install/setup.bash"
set -u

# Rebuild the tracking route from the latest saved waypoint source on every
# launch.  This keeps the flight entrypoint from following a stale mission.yaml
# after a handheld or in-flight waypoint collection.
if ! python3 "${exporter}" \
  --waypoints-file "${waypoints_file}" \
  --mission-file "${mission_file}" \
  --all-waypoints; then
  echo "错误：无法从最新航点生成任务路线，请检查 ${waypoints_file}。" >&2
  exit 2
fi

# Validate with the same parser used at runtime, so a malformed route never
# reaches the real-flight launcher.
if ! python3 - "${fsm_node}" "${mission_file}" <<'PY'
import importlib.util
import sys
from pathlib import Path

module_path, mission_path = map(Path, sys.argv[1:])
spec = importlib.util.spec_from_file_location('waypoint_state_machine_node', module_path)
module = importlib.util.module_from_spec(spec)
assert spec.loader is not None
sys.modules[spec.name] = module
spec.loader.exec_module(module)
points = module.load_route(mission_path)
print(f"航点状态机路线：{len(points)} 个点，顺序：" + " -> ".join(p.name for p in points))
PY
then
  echo "错误：任务文件未通过航点状态机校验：${mission_file}" >&2
  exit 2
fi

mkdir -p "$(dirname -- "${fsm_log}")"

stop_fsm() {
  [[ "${cleanup_started}" == false ]] || return
  cleanup_started=true
  if [[ -n "${fsm_pid}" ]] && kill -0 "${fsm_pid}" 2>/dev/null; then
    kill -INT "${fsm_pid}" 2>/dev/null || true
    wait "${fsm_pid}" 2>/dev/null || true
  fi
}
trap stop_fsm EXIT
trap 'exit 130' INT
trap 'exit 143' TERM HUP

echo "状态机日志：${fsm_log}"
fsm_args=(--mission-file "${mission_file}")
wait_for_takeoff=false
for argument in "$@"; do
  case "${argument}" in
    --manualtakeoff|--manual-takeoff) wait_for_takeoff=true ;;
  esac
done
# Both automatic and manual handover modes must wait for the controller's
# post-takeoff altitude/reference gate.  Publishing the first goal on the
# OFFBOARD mode edge races that gate and the controller correctly rejects it.
fsm_args+=(--wait-for-takeoff)
python3 "${fsm_node}" "${fsm_args[@]}" \
  > >(tee -a "${fsm_log}") 2>&1 &
fsm_pid=$!
# Fail before starting the aircraft workflow if the state-machine process
# cannot initialize.  Without this check a Python/ROS startup exception leaves
# the safety script running with no waypoint publisher, which is easy to miss
# while watching the other terminal windows.
sleep 0.5
fsm_ready=false
for _ in {1..10}; do
  if grep -q '\[WAYPOINT_FSM_READY\]' "${fsm_log}" 2>/dev/null; then
    fsm_ready=true
    break
  fi
  if ! kill -0 "${fsm_pid}" 2>/dev/null; then
    break
  fi
  sleep 0.1
done
if [[ "${fsm_ready}" != true ]]; then
  wait "${fsm_pid}" 2>/dev/null || true
  echo "错误：航点状态机未能启动；请检查日志：${fsm_log}" >&2
  exit 2
fi

# Keep the established real-flight safety workflow and pass through its
# --manualtakeoff/--noego/rosbag arguments unchanged.
set +e
env MISSION_FILE="${mission_file}" WAYPOINTS_FILE="${waypoints_file}" \
  WAYPOINT_FSM_ENABLED=true \
  WAYPOINT_VISUALIZER_ENABLED=true \
  "${single_goal_script}" "$@"
status=$?
set -e
exit "${status}"
