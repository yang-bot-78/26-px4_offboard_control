#!/usr/bin/env bash
# 刻意不用 set -e：本脚本要把所有类别都检查完、把发现的东西都报出来，
# 单个 kill 失败不能让清理半途而废。
set -uo pipefail

# 启动前的环境清理：确认没有上一次遗留的进程，不干净就断开它们。
#
# 用法：
#   清理运行环境.sh <类别>...
#
# 类别：
#   ev        EV 桥接、里程计守护、健康监视、无桨验证 launch 与安全看门狗
#   offboard  Offboard 与导航控制节点
#   rosbag    ros2 bag 录包进程
#   sensor    MID-360 驱动与 FAST-LIO
#   mavros    mavros_node 与 fastlio_mavros_autofix.launch.py
#   rviz      rviz2
#   all       以上全部
#
# 环境变量：
#   SKIP_ENV_CLEANUP=1     跳过整个清理（只用于排查，不建议）
#   ENV_CLEANUP_DRY_RUN=1  只报告发现了什么，不结束任何进程
#
# 退出码：0 环境已干净或已清理干净，2 拒绝清理，3 清理后仍有进程存活。
#
# 安全约束：飞机 armed=true 时本脚本拒绝结束任何进程。断开 MAVROS 或 Offboard
# 节点会让一架已解锁的飞机失去控制来源，这种情况必须由操作员先落地上锁再清理。
# 读不到 /mavros/state 时同样拒绝（失效即拒绝）—— 那时无法证明飞机没有解锁。

if [[ "${SKIP_ENV_CLEANUP:-0}" == 1 ]]; then
  echo "[清理] SKIP_ENV_CLEANUP=1：已跳过启动前环境清理。" >&2
  exit 0
fi

dry_run="${ENV_CLEANUP_DRY_RUN:-0}"

# 各类别对应的进程匹配式。刻意逐个写全，不用 'ros2' 这种宽泛匹配：
# 宽泛匹配会连带杀掉操作员自己开的无关终端命令。
patterns_ev=(
  fastlio_ev_health_monitor
  fastlio_mavros_vision_bridge
  fastlio_odometry_guard
  fastlio_mavros_odometry_bridge
  fastlio_vehicle_visual_odometry
  prop_off_ev_validation.launch.py
  无桨视觉验证.sh
  无桨安全看门狗.py
)
# 后两个可执行文件已从源码删除，仍列在这里：陈旧的 install 树可能还留着它们，
# 而它们会成为第二个 PX4 EV 写入者。
patterns_offboard=(
  offboard_waypoint_node
  minipc_mavros_offboard
  offboard_control_srv
  ego_odom_bridge
  navigation.launch.py
  运行导航控制.sh
)
patterns_rosbag=(
  rosbag2_recorder
  'ros2 bag record'
  开始录包.sh
)
patterns_sensor=(
  livox_ros_driver2_node
  msg_MID360_launch.py
  run_mid360_driver.sh
  fastlio_mapping
  run_fastlio_mid360.sh
)
patterns_mavros=(
  mavros_node
  fastlio_mavros_autofix.launch.py
)
patterns_rviz=(
  rviz2
)

# 收集某个类别下当前存活的 PID。排除自身、父进程和本次清理相关的进程，
# 否则脚本会把自己或调用者杀掉。
# 自身及全部祖先进程的 PID 集合，供 is_ancestor 判断。启动时算一次即可。
declare -A cleanup_self_and_ancestors=()
build_ancestor_set() {
  local pid=$$ guard=0
  while [[ -n "${pid}" ]] && ((pid > 0 && guard < 64)); do
    cleanup_self_and_ancestors["${pid}"]=1
    pid="$(ps -o ppid= -p "${pid}" 2>/dev/null | tr -d '[:space:]')"
    ((guard++))
  done
}
build_ancestor_set

is_ancestor() {
  [[ -n "${cleanup_self_and_ancestors[$1]:-}" ]]
}

collect_pids() {
  # 传入的是数组变量名。参数名刻意与调用方的实参不同名：Bash 的 local -n 在名字
  # 相同时会报"循环的名称引用"并静默返回空数组，那会让清理误报环境干净。
  local -n _cleanup_patterns="$1"
  local pattern pid found=()
  for pattern in "${_cleanup_patterns[@]}"; do
    while read -r pid; do
      [[ -n "${pid}" ]] || continue
      # 排除整条祖先链，不只是直接父进程：调用方可能是被组件窗口再套一层起来的，
      # 那时它的名字仍会匹配上面的模式，杀掉它等于清理脚本把自己的调用者干掉。
      is_ancestor "${pid}" && continue
      # 跳过本清理脚本自身的匹配（pgrep -f 会匹配到我们自己的命令行）。
      case "$(tr '\0' ' ' <"/proc/${pid}/cmdline" 2>/dev/null)" in
        *清理运行环境.sh*) continue ;;
      esac
      found+=("${pid}")
    done < <(pgrep -f -- "${pattern}" 2>/dev/null)
  done
  printf '%s\n' "${found[@]:-}" | awk 'NF' | sort -un
}

# armed=true 时拒绝清理。读不到状态也拒绝：无法证明飞机未解锁就不能动进程。
assert_not_armed() {
  local state
  if ! command -v ros2 >/dev/null 2>&1; then
    return 0
  fi
  if ! pgrep -f mavros_node >/dev/null 2>&1; then
    # MAVROS 没在跑，飞机不可能通过本机 Offboard 被控制。
    return 0
  fi
  state="$(timeout 8 ros2 topic echo --once /mavros/state mavros_msgs/msg/State 2>/dev/null)"
  if [[ -z "${state}" ]]; then
    echo "[清理] 拒绝清理：MAVROS 在运行，但读不到 /mavros/state。" >&2
    echo "[清理] 无法证明飞机未解锁，不能断开任何进程。请人工确认飞机已上锁后重试。" >&2
    return 1
  fi
  if ! printf '%s\n' "${state}" | grep -Eq '^armed:[[:space:]]+false$'; then
    echo "[清理] 拒绝清理：MAVROS 报告飞机已解锁（armed=true）。" >&2
    echo "[清理] 断开 MAVROS 或 Offboard 节点会让已解锁的飞机失去控制来源。" >&2
    echo "[清理] 请先落地并上锁，再重新启动。" >&2
    printf '%s\n' "${state}" >&2
    return 1
  fi
}

describe_pids() {
  local pid
  for pid in "$@"; do
    printf '        %s  %s\n' "${pid}" \
      "$(ps -o cmd= -p "${pid}" 2>/dev/null | cut -c1-100)"
  done
}

# 先 SIGINT（ROS 节点靠它做正常退出、写完 bag 元数据），再 SIGTERM，最后 SIGKILL。
# 直接 SIGKILL 会让 ros2 bag 留下没有 metadata.yaml 的坏录包。
terminate_pids() {
  local label="$1"; shift
  local pids=("$@") pid alive=()

  echo "[清理] ${label}：发现 ${#pids[@]} 个遗留进程，正在断开。"
  describe_pids "${pids[@]}"

  for pid in "${pids[@]}"; do kill -INT "${pid}" 2>/dev/null; done
  for _ in 1 2 3 4 5 6 7 8; do
    alive=()
    for pid in "${pids[@]}"; do kill -0 "${pid}" 2>/dev/null && alive+=("${pid}"); done
    ((${#alive[@]} == 0)) && break
    sleep 1
  done

  if ((${#alive[@]} > 0)); then
    for pid in "${alive[@]}"; do kill -TERM "${pid}" 2>/dev/null; done
    for _ in 1 2 3 4 5; do
      alive=()
      for pid in "${pids[@]}"; do kill -0 "${pid}" 2>/dev/null && alive+=("${pid}"); done
      ((${#alive[@]} == 0)) && break
      sleep 1
    done
  fi

  if ((${#alive[@]} > 0)); then
    echo "[清理] ${label}：SIGINT/SIGTERM 后仍存活，改用 SIGKILL。" >&2
    describe_pids "${alive[@]}" >&2
    for pid in "${alive[@]}"; do kill -KILL "${pid}" 2>/dev/null; done
    sleep 2
  fi

  alive=()
  for pid in "${pids[@]}"; do kill -0 "${pid}" 2>/dev/null && alive+=("${pid}"); done
  if ((${#alive[@]} > 0)); then
    echo "[清理] ${label}：以下进程无法结束（可能属于其他用户或已成僵尸）：" >&2
    describe_pids "${alive[@]}" >&2
    return 1
  fi
  echo "[清理] ${label}：已断开。"
}

resolve_categories() {
  local requested=("$@") category expanded=()
  for category in "${requested[@]}"; do
    case "${category}" in
      all) expanded+=(ev offboard rosbag sensor mavros rviz) ;;
      ev|offboard|rosbag|sensor|mavros|rviz) expanded+=("${category}") ;;
      *)
        echo "[清理] 未知类别：${category}" >&2
        echo "[清理] 可用类别：ev offboard rosbag sensor mavros rviz all" >&2
        return 2
        ;;
    esac
  done
  printf '%s\n' "${expanded[@]}" | awk '!seen[$0]++'
}

main() {
  if (($# == 0)); then
    echo "[清理] 用法：清理运行环境.sh <类别>...（ev offboard rosbag sensor mavros rviz all）" >&2
    exit 2
  fi

  local categories
  if ! mapfile -t categories < <(resolve_categories "$@"); then
    exit 2
  fi
  ((${#categories[@]} > 0)) || exit 2

  local category label pids all_pids=() failed=0
  declare -A labels=(
    [ev]="EV 桥接与健康监视"
    [offboard]="Offboard 与导航控制"
    [rosbag]="rosbag 录包"
    [sensor]="MID-360 驱动与 FAST-LIO"
    [mavros]="MAVROS"
    [rviz]="RViz"
  )

  # 先把所有类别扫一遍，再决定是否动手。这样"环境本来就干净"的常见情况完全不需要
  # 读 /mavros/state，也不会因为读不到状态而误报拒绝。
  declare -A found=()
  for category in "${categories[@]}"; do
    mapfile -t pids < <(collect_pids "patterns_${category}")
    if ((${#pids[@]} > 0)); then
      found["${category}"]="${pids[*]}"
      all_pids+=("${pids[@]}")
    fi
  done

  if ((${#all_pids[@]} == 0)); then
    echo "[清理] 环境干净：所选类别下没有遗留进程。"
    exit 0
  fi

  if [[ "${dry_run}" == 1 ]]; then
    echo "[清理] ENV_CLEANUP_DRY_RUN=1：只报告，不结束任何进程。"
    for category in "${categories[@]}"; do
      [[ -n "${found[${category}]:-}" ]] || continue
      echo "[清理] ${labels[${category}]}："
      # shellcheck disable=SC2086
      describe_pids ${found[${category}]}
    done
    exit 0
  fi

  if ! assert_not_armed; then
    exit 2
  fi

  # 按依赖倒序断开：先断消费者（导航/EV/录包），最后断被依赖的传感器与 MAVROS。
  # 反过来做会让 Offboard 节点在 MAVROS 消失后继续对着空话题发布 setpoint。
  local order=(offboard rosbag ev rviz sensor mavros)
  for category in "${order[@]}"; do
    [[ -n "${found[${category}]:-}" ]] || continue
    label="${labels[${category}]}"
    # shellcheck disable=SC2206
    pids=(${found[${category}]})
    terminate_pids "${label}" "${pids[@]}" || failed=1
  done

  if ((failed != 0)); then
    echo "[清理] 清理未完全成功；请人工处理上面列出的进程后再启动。" >&2
    exit 3
  fi

  echo "[清理] 启动前环境清理完成。"
}

main "$@"
