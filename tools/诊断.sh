#!/usr/bin/env bash
# 只读的启动链路诊断脚本。从不发布话题、从不调用服务、从不修改 PX4 参数、从不解锁。
# 任何时候都可以安全运行，包括整栈正在跑的时候。
#
# 各层按依赖顺序检查，报告只点出**最先坏的那一层**：坏层下游必然连带失败，
# 逐个去追那些症状，正是上次把 FAST-LIO 延迟问题误判成坐标系问题的原因。
#
# Usage:
#   ./诊断.sh                 full diagnostic (static + runtime)
#   ./诊断.sh --static        static checks only; no running stack needed
#   ./诊断.sh --probe-sec 10  longer runtime sampling window (default 5)
#   ./诊断.sh --watch 5       repeat the runtime layers every 5s until Ctrl+C
#
# 退出码：0 全部通过，1 只有警告，2 至少一项失败。

# 刻意不用 `set -e`：诊断脚本必须能在自己的检查失败后继续跑完。
set -uo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
project_root="${script_dir}"
livox_env="${LIVOX_MID360_ENV:-${HOME}/livox_mid360_env}"
livox_setup="${livox_env}/setup_mid360.bash"
fastlio_src="${FASTLIO_SRC:-${livox_env}/ws_fastlio/src/fast_lio/src/laserMapping.cpp}"

static_only=false
probe_sec=5
watch_sec=0

while (($# > 0)); do
  case "$1" in
    --static) static_only=true; shift ;;
    --probe-sec) probe_sec="${2:?--probe-sec needs a value}"; shift 2 ;;
    --watch) watch_sec="${2:?--watch needs a value}"; shift 2 ;;
    -h|--help) sed -n '1,19p' "${BASH_SOURCE[0]}"; exit 0 ;;
    *) printf '未知参数：%s（用 --help）\n' "$1" >&2; exit 2 ;;
  esac
done

for value in "${probe_sec}" "${watch_sec}"; do
  if [[ ! "${value}" =~ ^[0-9]+$ ]]; then
    printf '参数必须是非负整数：%s\n' "${value}" >&2
    exit 2
  fi
done

fail_count=0
warn_count=0
declare -a broken_layers=()
current_layer=""

if [[ -t 1 ]]; then
  c_reset=$'\033[0m'; c_ok=$'\033[32m'; c_warn=$'\033[33m'
  c_fail=$'\033[31m'; c_head=$'\033[1;36m'; c_dim=$'\033[2m'
else
  c_reset=""; c_ok=""; c_warn=""; c_fail=""; c_head=""; c_dim=""
fi

layer() {
  current_layer="$1"
  printf '\n%s=== %s ===%s\n' "${c_head}" "$1" "${c_reset}"
}

ok()   { printf '  %s[ OK ]%s %s\n' "${c_ok}" "${c_reset}" "$*"; }
info() { printf '  %s[    ]%s %s\n' "${c_dim}" "${c_reset}" "$*"; }

warn() {
  printf '  %s[WARN]%s %s\n' "${c_warn}" "${c_reset}" "$*"
  ((warn_count++))
}

fail() {
  printf '  %s[FAIL]%s %s\n' "${c_fail}" "${c_reset}" "$*"
  ((fail_count++))
  local layer
  for layer in "${broken_layers[@]+"${broken_layers[@]}"}"; do
    [[ "${layer}" == "${current_layer}" ]] && return 0
  done
  broken_layers+=("${current_layer}")
}

# 说明某项失败为何重要、该怎么处理。刻意紧挨着失败输出，这样
# report is actionable without cross-referencing 验证方法.md.
hint() {
  printf '         %s↳ %s%s\n' "${c_dim}" "$*" "${c_reset}"
}

topic_has_publisher() {
  printf '%s\n' "${topic_list}" | grep -Fxq "$1"
}

# 只统计发布者数量，不做订阅。`ros2 topic info` 是只读操作。
publisher_count() {
  local topic="$1" out
  out="$(timeout 8 ros2 topic info "${topic}" 2>/dev/null)" || { printf 'ERR\n'; return; }
  printf '%s\n' "${out}" | awk -F': *' '/Publisher count/ {print $2; found=1}
    END {if (!found) print "ERR"}'
}

check_static_layer_0() {
  layer "层 0：静态前提（不需要跑起来）"

  if [[ -f "${project_root}/install/setup.bash" ]]; then
    ok "install/setup.bash 存在"
  else
    fail "install/setup.bash 缺失"
    hint "先 colcon build --packages-up-to race_offboard race_bringup"
    return
  fi

  local script
  for script in tools/flight/一键启动导航栈.sh \
    tools/flight/等待ROS就绪.sh \
    tools/flight/无桨视觉验证.sh \
    tools/checks/起飞前自检.sh; do
    if [[ -x "${project_root}/${script}" ]]; then
      ok "${script}"
    else
      fail "${script} 缺失或不可执行"
    fi
  done

  # The single remaining code-level flight blocker (验证方法.md §4). MAVROS puts
  # odom.twist.linear 里是机体 FLU 速度；桥接却当成世界 ENU 用，只做了 enuToMap，
  # 因此飞机一转向方向就错。
  local bridge="${project_root}/src/race_ego_bridge/src/ego_odom_bridge.cpp"
  if [[ -f "${bridge}" ]]; then
    if grep -q "body_to_world" "${bridge}"; then
      ok "ego_odom_bridge 速度已做 body→world 姿态旋转"
    else
      fail "ego_odom_bridge 仍把 twist.twist.linear 当 world ENU 用（验证方法.md §4）"
      hint "MAVROS 那个字段是 body FLU；缺姿态旋转，转弯后 EGO 收到的速度方向就错了"
      hint "悬停看不出来，只在 yaw≠0 时暴露。这是当前唯一的代码级阻断项"
    fi
  else
    fail "找不到 ego_odom_bridge.cpp"
  fi

  # 上游一跳处的同类缺陷：EV 链交给 PX4 的东西。静态和运行时都要查，因为它在任何
  # yaw≠0 的时候都是错的，而飞机静止时完全看不出来。
  local vision_bridge="${project_root}/src/px4_ros_com/src/bridges/fastlio_mavros_vision_bridge.cpp"
  if [[ -f "${vision_bridge}" ]]; then
    if grep -q "child_to_world_rotation" "${vision_bridge}"; then
      ok "vision_speed 速度已从 FAST-LIO child 帧旋转到 world"
    else
      fail "vision_speed 未做 child→world 旋转，MAVROS 会按 local ENU 误读"
      hint "FAST-LIO 按 REP-147 把 twist 放在 child 帧；直接转发方向错"
    fi
  else
    fail "找不到 fastlio_mavros_vision_bridge.cpp"
  fi

  # /Odometry 落后 1.4s 的根因；已于 2026-08-09 18:59 确认修复。
  # reliable 订阅队列在负载下保留了积压的雷达/IMU 旧样本，而单线程建图循环
  # 永远追不上当前时间。
  if [[ -f "${fastlio_src}" ]]; then
    local qos_count
    qos_count="$(grep -c "SensorDataQoS" "${fastlio_src}")"
    if ((qos_count >= 3)); then
      ok "FAST-LIO 传感器入口用 SensorDataQoS（${qos_count} 处）"
    else
      fail "FAST-LIO 只有 ${qos_count} 处 SensorDataQoS，应为 3（点云×2 + IMU）"
      hint "reliable 队列会保留启动积压的旧样本，/Odometry 会稳定落后约 1.4s"
      hint "不要靠提高 max_input_age_s 或重写时间戳绕过 —— 那是假装看得见"
    fi
    # install/ 是一次性创建的符号链接树，它自己的 mtime 证明不了任何事。
    # 必须解析到真实二进制：那才是实际运行的东西。
    local fastlio_bin
    fastlio_bin="$(readlink -f \
      "${livox_env}/ws_fastlio/install/fast_lio/lib/fast_lio/fastlio_mapping" 2>/dev/null)"
    if [[ -f "${fastlio_bin}" ]]; then
      if [[ "${fastlio_src}" -nt "${fastlio_bin}" ]]; then
        fail "laserMapping.cpp 比编译产物新 —— 源码改动没进二进制"
        hint "源码 $(date -r "${fastlio_src}" '+%m-%d %H:%M:%S')，二进制 $(date -r "${fastlio_bin}" '+%m-%d %H:%M:%S')"
        hint "你读源码看到的修复不等于正在跑的行为。重新编译 ws_fastlio 后再验证"
      else
        ok "FAST-LIO 二进制不早于源码（$(date -r "${fastlio_bin}" '+%m-%d %H:%M')）"
      fi
    else
      warn "解析不到 FAST-LIO 二进制，无法判断是否需要重新编译"
    fi
  else
    warn "读不到 FAST-LIO 源码：${fastlio_src}（预编译二进制在工作区外，属既定约束）"
  fi

  # 静默失败缺陷：分析器在 tools/analysis/ 下，但脚本却在自己所在的 tools/flight/
  # 里找它，还用 `|| true` 把错误吞掉，导致验证报告根本不会生成。
  local runner="${project_root}/tools/flight/无桨视觉验证.sh"
  local analyzer="${project_root}/tools/analysis/分析无桨视觉验证.py"
  if [[ -f "${runner}" ]]; then
    if grep -q 'script_dir}/分析无桨视觉验证.py' "${runner}"; then
      warn "无桨视觉验证.sh 找错分析器路径（tools/flight/ vs tools/analysis/）"
      hint "外面套了 || true，所以自动生成报告会静默失败；手动重跑 tools/analysis/ 下那个"
    elif [[ ! -f "${analyzer}" ]]; then
      fail "找不到分析器 tools/analysis/分析无桨视觉验证.py"
      hint "没有它，无桨场次录完包也判不了 —— 报告是判定依据，不是附属产物"
    else
      ok "无桨脚本的分析器路径正确"
    fi
  fi

  # geofence 是按工程 map 帧（南、东、上）标定的。把它当成标准 ENU 来读，
  # 正是所有安全边界被静默废掉的原因。
  local tuning="${project_root}/src/race_bringup/config/astar_ego_tuning.yaml"
  if [[ -f "${tuning}" ]]; then
    local z_max
    z_max="$(awk '/^ *z_max:/ {print $2; exit}' "${tuning}")"
    info "geofence z_max=${z_max:-?}（已验证起飞高度 0.80m，余量约 5cm）"
    if [[ -n "${z_max}" ]] && awk -v z="${z_max}" 'BEGIN{exit !(z <= 0.85)}'; then
      warn "geofence 仍是仿真场地值，赛场 15×14m 未实测（验证方法.md §8）"
    fi
  else
    fail "找不到 astar_ego_tuning.yaml"
  fi

  local mission="${project_root}/src/race_offboard/config/mission.yaml"
  if [[ -f "${mission}" ]]; then
    if grep -qE '^\s*preset_points:' "${mission}"; then
      ok "mission.yaml 已启用 preset_points"
    else
      info "preset_points 未启用 —— 当天需用 RViz 依次点 B1/B2/C/D"
    fi
  fi
}

check_layer_1_lidar() {
  layer "层 1：MID-360 驱动"

  # Livox CustomMsg 只存在于驱动 overlay 里，所以在主工作区 overlay 下无法 echo
  # /livox/lidar。这里"没有"意味着"解析不了"，不是"雷达没数据" —— 这个误报已经
  # 浪费过一次排查时间。
  local topic
  for topic in /livox/imu /livox/lidar; do
    if topic_has_publisher "${topic}"; then
      ok "${topic} 有发布者"
    else
      fail "${topic} 无发布者"
      hint "先确认驱动窗口是否在跑；CustomMsg 类型在驱动 overlay 里，主 overlay 解析不了"
    fi
  done

  if [[ -f "${livox_setup}" ]]; then
    ok "驱动 overlay 存在：${livox_setup}"
  else
    warn "驱动 overlay 缺失：${livox_setup} —— 就绪检查无法解析 CustomMsg"
  fi
}

check_layer_2_fastlio() {
  layer "层 2：FAST-LIO 里程计"

  if ! topic_has_publisher /Odometry; then
    fail "/Odometry 无发布者"
    hint "FAST-LIO 没起来或已退出。层 1 若已 FAIL，先修层 1"
    return
  fi
  ok "/Odometry 有发布者"

  # 整个脚本里最有用的一个数字：里程计时间戳落后当前时间多少。
  # EV 会拒绝任何超过 max_input_age_s 的数据。
  local age
  age="$(measure_odometry_age /Odometry)"
  if [[ "${age}" == "ERR" ]]; then
    fail "/Odometry 取不到样本（${probe_sec}s 内无消息）"
    hint "话题存在但不出数据：FAST-LIO 卡住了，不是坐标问题"
  elif awk -v a="${age}" 'BEGIN{exit !(a > 0.25)}'; then
    fail "/Odometry 落后当前时间 ${age}s，已超 EV 门限 0.25s"
    hint "先查 FAST-LIO 的 SensorDataQoS 是否生效并已重新编译（层 0）"
    hint "不要提高 max_input_age_s，也不要给里程计重写当前时间戳"
  elif awk -v a="${age}" 'BEGIN{exit !(a > 0.1)}'; then
    warn "/Odometry 落后 ${age}s —— 未超 0.25s 门限但偏高"
    hint "修复后的正常值约 0.016s；偏高说明有调度尖峰"
  else
    ok "/Odometry 延迟 ${age}s（门限 0.25s，回归基线约 0.016s）"
  fi

  if topic_has_publisher /Odometry/healthy; then
    ok "/Odometry/healthy 有发布者（odom guard 在跑）"
  else
    fail "/Odometry/healthy 无发布者 —— fastlio_odometry_guard 没起来"
    hint "vision bridge 的输入就是这个话题，缺了整条 EV 链不通"
  fi
}

check_layer_3_ev() {
  layer "层 3：EV 链与 MAVROS"

  if ! topic_has_publisher /mavros/state; then
    fail "/mavros/state 无发布者 —— MAVROS 没起来"
    return
  fi

  local state
  state="$(timeout 8 ros2 topic echo --once /mavros/state mavros_msgs/msg/State 2>/dev/null)"
  if [[ -z "${state}" ]]; then
    fail "/mavros/state 取不到样本"
  else
    if [[ "${state}" == *"connected: true"* ]]; then
      ok "MAVROS 已连接飞控"
    else
      fail "MAVROS 未连接飞控（connected: false）"
      hint "查 /dev/ttyUSB0 与波特率 921600；EV 数据发出去也没人收"
    fi
    # 只把 armed/mode 作为事实报告出来。本脚本从不修改它们。
    if [[ "${state}" == *"armed: true"* ]]; then
      warn "飞机当前已解锁（armed: true）"
    else
      ok "未解锁（armed: false）"
    fi
    local mode
    mode="$(printf '%s\n' "${state}" | awk -F': *' '/^mode:/ {print $2; exit}')"
    if [[ "${mode}" == *OFFBOARD* ]]; then
      warn "当前模式 OFFBOARD"
    else
      info "当前模式 ${mode:-未知}"
    fi
  fi

  local topic
  for topic in /mavros/local_position/odom /mavros/local_position/velocity_local \
    /mavros/vision_pose/pose_cov /mavros/vision_speed/speed_twist_cov; do
    if topic_has_publisher "${topic}"; then
      ok "${topic}"
    else
      fail "${topic} 无发布者"
    fi
  done

  check_ev_health
  check_vision_speed_frame
  check_single_publisher_invariants
}

# EKF2_EV_CTRL 是位掩码：bit0=1 水平位置，bit1=2 高度，bit3=8 yaw，bit2=4 速度。
# 11 = 1+2+8，所以速度融合是关闭的：PX4 收到 /mavros/vision_speed/speed_twist_cov
# 但会忽略它。ROS 侧的 bag 只能证明我们发了什么，永远证明不了 EKF2 融合了它 ——
# 那需要 PX4
# console (estimator_aid_src_ev_vel / cs_ev_vel).
check_vision_speed_frame() {
  topic_has_publisher /mavros/vision_speed/speed_twist_cov || return 0

  info "EKF2_EV_CTRL=11 → 位置+高度+yaw 融合，速度位(bit2=4)未置位"
  info "所以 PX4 收得到 vision_speed 但不融合；ROS 侧录包只能证明发出去了"
  hint "要证明是否被融合，必须看 PX4 控制台 estimator_aid_src_ev_vel / cs_ev_vel"

  # Frame mismatch, same class as 验证方法.md §4: FAST-LIO puts twist in the
  # 按 REP-147，twist 表达在 child 帧，而 MAVROS 的 vision_speed 把这个向量当成
  # local ENU 读。判断旋转是否存在要看那次调用，而不是看旧的原样拷贝有没有消失：
  # `speed_msg.twist = msg->twist` 合法地保留着，用来带上协方差和刻意置 NaN 的
  # 角速度。
  local bridge="${project_root}/src/px4_ros_com/src/bridges/fastlio_mavros_vision_bridge.cpp"
  if [[ -f "${bridge}" ]]; then
    if grep -q "child_to_world_rotation" "${bridge}"; then
      ok "vision_speed 已把 FAST-LIO child 帧速度旋转到 world"
    else
      fail "vision_speed 送的是 child 帧速度，但 MAVROS 按 local ENU 解释"
      hint "FAST-LIO 按 REP-147 把 twist 放在 child 帧；bridge 未做姿态旋转"
      hint "launch 里 force_twist_frame_id=odom 且注释自称 local ENU —— 两者矛盾"
      hint "yaw≠0 时这个速度方向就是错的。录包前先修，否则采到的数据不能用于判定"
    fi
  fi
}

check_ev_health() {
  if ! topic_has_publisher /ev_health/status; then
    fail "/ev_health/status 无发布者 —— EV 健康监视没起来"
    hint "起飞门禁要连续 HEALTHY 7.5s，缺这个话题控制节点永远不会锁点"
    return
  fi

  local status
  status="$(timeout 8 ros2 topic echo --once /ev_health/status std_msgs/msg/String 2>/dev/null |
    awk -F': *' '/^data:/ {print $2; exit}')"
  case "${status}" in
    HEALTHY)
      ok "/ev_health/status = HEALTHY"
      ;;
    SUSPECT)
      warn "/ev_health/status = SUSPECT —— 应在 7.5s 滞回内自动恢复"
      hint "SUSPECT 一次就重置整个 7.5s 起飞计时，不是暂停"
      ;;
    FAULT)
      fail "/ev_health/status = FAULT"
      ;;
    "")
      fail "/ev_health/status 取不到样本"
      ;;
    *)
      fail "/ev_health/status 未知状态：${status}（按 FAULT 处理）"
      ;;
  esac

  # 这两个原因经常被混淆。px4_velocity_unaligned 的含义是"找不到时间差在 0.1s 内的
  # PX4 速度样本来比对" —— 速度差是 NaN，因为它根本没算出来。它**不是**坐标系错误的
  # 证据，把它当坐标系问题去追会白费一整场。
  local diag
  diag="$(timeout 8 ros2 topic echo --once /ev_health/diagnostics \
    diagnostic_msgs/msg/DiagnosticArray 2>/dev/null)"
  if [[ -n "${diag}" ]]; then
    local reason align input_age
    reason="$(printf '%s\n' "${diag}" | grep -A1 'key: reason' | awk -F': *' '/value:/ {print $2; exit}')"
    align="$(printf '%s\n' "${diag}" | grep -A1 'key: velocity_alignment_s' | awk -F': *' '/value:/ {print $2; exit}')"
    input_age="$(printf '%s\n' "${diag}" | grep -A1 'key: input_age_s' | awk -F': *' '/value:/ {print $2; exit}')"
    [[ -n "${reason}" ]] && info "reason=${reason}"
    [[ -n "${input_age}" ]] && info "input_age_s=${input_age}（门限 0.25）"
    if [[ -n "${align}" ]]; then
      if awk -v a="${align}" 'BEGIN{exit !(a > 0.1)}' 2>/dev/null; then
        warn "velocity_alignment_s=${align} 超 0.1s 门限"
        hint "含义是找不到时间对齐的 PX4 速度样本来比对，不是两个速度方向相反"
        hint "此时速度差是 NaN，别当坐标系错误去查"
      else
        ok "velocity_alignment_s=${align}（门限 0.1）"
      fi
    fi
  fi
}

check_single_publisher_invariants() {
  # 两个 setpoint 发布者意味着 PX4 跟踪最后到达的那一条，飞机会变得不可控。
  # 两个 EV 写入者意味着 EKF2 收到互相矛盾的位置。这两项在节点启动时都是
  # fail-closed 的，但陈旧的 install/ 树仍可能把已删除的节点复活。
  local topic count
  for topic in /mavros/setpoint_raw/local /mavros/vision_pose/pose_cov; do
    topic_has_publisher "${topic}" || continue
    count="$(publisher_count "${topic}")"
    case "${count}" in
      1) ok "${topic} 发布者 = 1" ;;
      ERR) warn "${topic} 查不到发布者数" ;;
      *)
        fail "${topic} 有 ${count} 个发布者，必须是 1"
        hint "两个 setpoint 发布者会让 PX4 跟踪最后到达的那条，飞机不可控"
        hint "用 ros2 topic info -v ${topic} 找出是谁；注意陈旧 install/ 树"
        ;;
    esac
  done

  # minipc_mavros_offboard.py 已于 2026-08-09 删除；现在 offboard_waypoint_node 是
  # 唯一的 Offboard 节点。陈旧的 overlay 仍可能把旧的那个启动起来。
  local node
  for node in /minipc_mavros_offboard /minipc_offboard_control; do
    if printf '%s\n' "${node_list}" | grep -Fxq "${node}"; then
      fail "已删除的旧节点仍在运行：${node}"
      hint "陈旧 install/ 树。停掉它，重新 colcon build"
    fi
  done
}

check_layer_4_navigation() {
  layer "层 4：导航栈"

  if topic_has_publisher /race/odom; then
    ok "/race/odom"
  else
    fail "/race/odom 无发布者"
    hint "常见于 fastlio_bridge_node 的 odom_topic 订阅了错的话题名（应为 /Odometry）"
    hint "该节点订阅错话题时一条消息都收不到，且不报错"
  fi

  if topic_has_publisher /race/control/status; then
    ok "/race/control/status"
  else
    fail "/race/control/status 无发布者 —— 导航栈没起来"
  fi

  for topic in /saved_map /race/ego/trajectory_setpoint /race/navigation_setpoint \
    /race/ego/odom /race/mission/status; do
    if topic_has_publisher "${topic}"; then
      ok "${topic}"
    else
      info "${topic} 无发布者（无地图/ego-shadow/关任务模式下属正常）"
    fi
  done

  if printf '%s\n' "${node_list}" | grep -Fxq /rosbag2_recorder; then
    ok "rosbag 录制中"
  else
    info "rosbag 未录制 —— 没有录包就没有事后判定依据"
  fi
}

# 里程计时间戳相对墙上时钟的滞后量，单位秒。只读：只创建订阅者，不发布任何东西。
# 采样窗口内没收到样本时打印 ERR。
measure_odometry_age() {
  local topic="$1"
  PROBE_TOPIC="${topic}" PROBE_SEC="${probe_sec}" \
    timeout "$((probe_sec + 6))" python3 -c '
import os, sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry

topic = os.environ["PROBE_TOPIC"]
window = float(os.environ["PROBE_SEC"])
ages = []

rclpy.init(args=None)
node = Node("diagnostic_odometry_age_probe")
qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                 history=HistoryPolicy.KEEP_LAST)

def on_msg(msg):
    stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    ages.append(node.get_clock().now().nanoseconds * 1e-9 - stamp)

node.create_subscription(Odometry, topic, on_msg, qos)
deadline = time.monotonic() + window
while time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)
node.destroy_node()
rclpy.shutdown()

if not ages:
    print("ERR")
else:
    print("%.3f" % (sum(ages) / len(ages)))
' 2>/dev/null || printf 'ERR\n'
}

collect_ros_graph() {
  topic_list="$(timeout 15 ros2 topic list 2>/dev/null)"
  node_list="$(timeout 15 ros2 node list 2>/dev/null)"
  if [[ -z "${topic_list}" ]]; then
    printf '\n%s没有发现任何 ROS 话题。%s\n' "${c_fail}" "${c_reset}"
    printf '整栈没起来，或 ROS_DOMAIN_ID 不一致。先跑：\n'
    printf '  NO_MAP_VALIDATION=true ./tools/flight/一键启动导航栈.sh\n'
    printf '只做静态检查：./诊断.sh --static\n'
    return 1
  fi
  return 0
}

run_runtime_layers() {
  collect_ros_graph || return 1
  check_layer_1_lidar
  check_layer_2_fastlio
  check_layer_3_ev
  check_layer_4_navigation
}

print_summary() {
  printf '\n%s=== 结论 ===%s\n' "${c_head}" "${c_reset}"
  if ((fail_count == 0 && warn_count == 0)); then
    printf '  %s全部检查通过。%s\n' "${c_ok}" "${c_reset}"
  else
    printf '  失败 %d 项，警告 %d 项。\n' "${fail_count}" "${warn_count}"
  fi

  # 点出第一个坏掉的**运行时**层就是本脚本的意义：断点下游的一切必然连带失败，
  # 而那些症状看起来像是彼此独立的 bug。第 0 层单独报告 —— 静态代码缺陷不会导致
  # 某个话题没有发布者，把它折进同一条因果链会断言一个并不存在的因果关系。
  local layer
  local -a runtime_broken=()
  local static_broken=false
  for layer in "${broken_layers[@]+"${broken_layers[@]}"}"; do
    if [[ "${layer}" == 层\ 0* ]]; then
      static_broken=true
    else
      runtime_broken+=("${layer}")
    fi
  done

  if ((${#runtime_broken[@]} > 0)); then
    printf '\n  %s最先断的一层：%s%s\n' "${c_fail}" "${runtime_broken[0]}" "${c_reset}"
    if ((${#runtime_broken[@]} > 1)); then
      printf '  下游同时报错（大概率是连带失败，先修上面那层再看）：\n'
      local index
      for ((index = 1; index < ${#runtime_broken[@]}; index++)); do
        printf '    - %s\n' "${runtime_broken[index]}"
      done
    fi
  fi

  if [[ "${static_broken}" == true ]]; then
    printf '\n  %s层 0 另有静态问题，与上面的运行时故障无因果关系，需单独修。%s\n' \
      "${c_warn}" "${c_reset}"
  fi

  printf '\n  %s本脚本只读：未发布话题、未调服务、未改参数、未解锁。%s\n' \
    "${c_dim}" "${c_reset}"
  printf '  %s判定门限与验证步骤见 验证方法.md；本脚本不构成飞行授权。%s\n' \
    "${c_dim}" "${c_reset}"

  ((fail_count > 0)) && return 2
  ((warn_count > 0)) && return 1
  return 0
}

main() {
  printf '%s导航栈诊断%s  %s\n' "${c_head}" "${c_reset}" "$(date '+%Y-%m-%d %H:%M:%S')"
  printf '工程根：%s\n' "${project_root}"

  check_static_layer_0

  if [[ "${static_only}" == true ]]; then
    printf '\n%s--static：跳过运行时层。%s\n' "${c_dim}" "${c_reset}"
    print_summary
    return $?
  fi

  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash 2>/dev/null
  # shellcheck disable=SC1091
  [[ -f "${project_root}/install/setup.bash" ]] &&
    source "${project_root}/install/setup.bash" 2>/dev/null
  set -u

  if ! command -v ros2 >/dev/null 2>&1; then
    layer "运行时"
    fail "ros2 命令不可用"
    print_summary
    return $?
  fi

  if ((watch_sec > 0)); then
    printf '\n%s--watch %ss：Ctrl+C 停止。%s\n' "${c_dim}" "${watch_sec}" "${c_reset}"
    while true; do
      fail_count=0; warn_count=0; broken_layers=()
      printf '\n%s---- %s ----%s\n' "${c_dim}" "$(date '+%H:%M:%S')" "${c_reset}"
      run_runtime_layers
      print_summary
      sleep "${watch_sec}"
    done
  fi

  run_runtime_layers
  print_summary
  return $?
}

main
