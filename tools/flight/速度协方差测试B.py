#!/usr/bin/env python3
"""拆桨状态下引导三维速度校准，并向 rosbag 写入可离线分析的阶段标记。

流程固定为：静止 60 秒、上移约 0.5 m 并静止、下移回原位、沿 ROS ENU X/Y
缓慢往返、缓慢偏航旋转。脚本绝不解锁、切模式或发布控制量。
"""

import argparse
import os
from pathlib import Path
import sys
import threading
import time

import rclpy
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


PROJECT_ROOT = Path(__file__).resolve().parents[2]
RUNTIME_DIR = PROJECT_ROOT / "runtime"
PID_FILE = RUNTIME_DIR / "takeoff_debug_bag.pid"
RUN_DIR_FILE = RUNTIME_DIR / "takeoff_debug_bag_run_dir.txt"
MARKER_TOPIC = "/velocity_calibration/marker"


class TestAborted(RuntimeError):
    """操作员中止或运行时安全条件不满足。"""


class CalibrationNode(Node):
    def __init__(self):
        super().__init__("velocity_calibration_3d")
        reliable = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
        self.marker_publisher = self.create_publisher(String, MARKER_TOPIC, reliable)
        self.create_subscription(State, "/mavros/state", self._state_callback, reliable)
        self.create_subscription(String, "/ev_health/status", self._health_callback, reliable)
        sensor_data = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry, "/Odometry", self._odom_callback, sensor_data)
        self.create_subscription(
            Odometry, "/mavros/odometry/out", self._ev_output_callback, reliable
        )
        self.state = None
        self.health = None
        self.last_odom_monotonic = None
        self.last_ev_output_monotonic = None
        self._last_health_warning = None

    def _state_callback(self, message):
        self.state = message

    def _health_callback(self, message):
        self.health = message.data

    def _odom_callback(self, _message):
        self.last_odom_monotonic = time.monotonic()

    def _ev_output_callback(self, _message):
        self.last_ev_output_monotonic = time.monotonic()

    def readiness_gaps(self):
        gaps = []
        if self.state is None:
            gaps.append("/mavros/state")
        if self.health is None:
            gaps.append("/ev_health/status")
        if self.last_odom_monotonic is None:
            gaps.append("/Odometry")
        if self.last_ev_output_monotonic is None:
            gaps.append("/mavros/odometry/out")
        if self.marker_publisher.get_subscription_count() == 0:
            gaps.append("rosbag:/velocity_calibration/marker")
        return gaps

    def known_offboard_publishers(self):
        publishers = []
        for topic in ("/mavros/setpoint_raw/local", "/fmu/in/trajectory_setpoint"):
            for endpoint in self.get_publishers_info_by_topic(topic):
                publishers.append(f"{topic}: {endpoint.node_namespace}/{endpoint.node_name}")
        return publishers

    def check_runtime(self, require_healthy=False):
        now = time.monotonic()
        if self.state is None:
            raise TestAborted("尚未收到 /mavros/state")
        if not self.state.connected:
            raise TestAborted("MAVROS 已断开")
        if self.state.armed:
            raise TestAborted("检测到飞机已解锁；本校准必须拆桨且未解锁")
        if self.state.mode != "STABILIZED":
            raise TestAborted(f"飞控模式必须是 STABILIZED，当前为 {self.state.mode}")
        if self.health is None:
            raise TestAborted("尚未收到 /ev_health/status")
        if require_healthy and self.health != "HEALTHY":
            raise TestAborted(f"EV 当前不是 HEALTHY，而是 {self.health}")
        if self.health == "FAULT":
            raise TestAborted("EV 已进入 FAULT，请停止移动并检查定位链")
        if self.health != "HEALTHY" and self.health != self._last_health_warning:
            print(f"\n[警告] EV 状态变为 {self.health}，请停止移动并观察。", flush=True)
            self._last_health_warning = self.health
        if self.last_odom_monotonic is None or now - self.last_odom_monotonic > 0.5:
            raise TestAborted("/Odometry 已超过 0.5 秒没有新消息")
        if self.last_ev_output_monotonic is None or now - self.last_ev_output_monotonic > 0.5:
            raise TestAborted("/mavros/odometry/out 已超过 0.5 秒没有新消息")


def parse_args():
    parser = argparse.ArgumentParser(
        description="中文交互式 FAST-LIO 三维速度校准（拆桨、不发控制量）"
    )
    parser.add_argument(
        "--重复次数", "--repeats", dest="repeats", type=int, default=1,
        help="垂直、X、Y 往返重复次数，默认 1",
    )
    parser.add_argument(
        "--静止秒数", "--stationary-seconds", dest="stationary_seconds",
        type=int, default=10, help="移动后的静止段持续秒数，默认 10",
    )
    parser.add_argument(
        "--初始静止秒数", "--initial-stationary-seconds", dest="initial_stationary_seconds",
        type=int, default=60, help="开场静止持续秒数，默认且建议 60",
    )
    return parser.parse_args()


def read_pid():
    try:
        value = PID_FILE.read_text(encoding="utf-8").strip()
        return int(value)
    except (FileNotFoundError, ValueError):
        raise TestAborted("未找到正在运行的录包进程；请先运行 tools/rosbag/开始录包.sh")


def check_rosbag_running():
    pid = read_pid()
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        raise TestAborted(f"录包进程 pid={pid} 已退出")
    except PermissionError:
        raise TestAborted(f"无权检查录包进程 pid={pid}")


def marker_log_path():
    try:
        run_root = Path(RUN_DIR_FILE.read_text(encoding="utf-8").strip())
    except FileNotFoundError:
        raise TestAborted("找不到本场次目录；请先启动录包")
    if not run_root.is_dir():
        raise TestAborted(f"本场次目录不存在：{run_root}")
    snapshot_dir = run_root / "snapshot"
    snapshot_dir.mkdir(parents=True, exist_ok=True)
    return snapshot_dir / "三维速度校准标记.txt"


def wait_until_ready(node, timeout_s=15.0):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if (
            node.state is not None
            and node.health is not None
            and node.last_odom_monotonic is not None
            and node.last_ev_output_monotonic is not None
            and node.marker_publisher.get_subscription_count() > 0
        ):
            node.check_runtime(require_healthy=True)
            return
        time.sleep(0.1)
    gaps = node.readiness_gaps()
    detail = "、".join(gaps) if gaps else "话题已收到但运行状态未通过"
    raise TestAborted(f"前置链路就绪超时；未就绪：{detail}")


def ask(prompt):
    try:
        input(prompt)
    except (EOFError, KeyboardInterrupt):
        raise TestAborted("操作员中止")


def publish_marker(node, log_file, label):
    check_rosbag_running()
    node.check_runtime()
    message = String()
    message.data = label
    node.marker_publisher.publish(message)
    wall_ns = time.time_ns()
    wall_text = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(wall_ns / 1e9))
    with log_file.open("a", encoding="utf-8") as stream:
        stream.write(f"{wall_ns} {wall_text}.{wall_ns % 1_000_000_000:09d} {label}\n")
    print(f"[标记] {label}", flush=True)
    time.sleep(0.1)


def countdown(node, seconds, description):
    print(f"{description}：保持 {seconds} 秒", flush=True)
    for remaining in range(seconds, 0, -1):
        node.check_runtime()
        print(f"  剩余 {remaining} 秒", end="\r", flush=True)
        time.sleep(1.0)
    print(" " * 30, end="\r", flush=True)


def run_axis(node, log_file, axis_name, outward_name, return_name, repeats, stationary_s):
    """执行一个世界 ENU 轴的慢速往返；仅以标记记录人工动作。"""
    publish_marker(node, log_file, f"速度校准|{axis_name}|轴测试开始")
    print(f"\n===== {axis_name}：{outward_name}，再原路{return_name} =====")
    for repeat in range(1, repeats + 1):
        prefix = f"速度校准|{axis_name}|第{repeat:02d}次"
        print(f"\n--- {axis_name} 第 {repeat}/{repeats} 次 ---")

        publish_marker(node, log_file, f"{prefix}|起点静止开始")
        countdown(node, stationary_s, "起点静止")
        publish_marker(node, log_file, f"{prefix}|起点静止结束")

        ask(f"准备好后按回车；回车后以小且平稳的速度{outward_name}，到达终点后再次按回车。")
        publish_marker(node, log_file, f"{prefix}|{outward_name}开始")
        ask(f"到达终点并完全停稳后按回车，结束{outward_name}。")
        publish_marker(node, log_file, f"{prefix}|{outward_name}结束")

        publish_marker(node, log_file, f"{prefix}|终点静止开始")
        countdown(node, stationary_s, "终点静止")
        publish_marker(node, log_file, f"{prefix}|终点静止结束")

        ask(f"准备好后按回车；回车后以小且平稳的速度{return_name}，回到起点后再次按回车。")
        publish_marker(node, log_file, f"{prefix}|{return_name}开始")
        ask(f"回到起点并完全停稳后按回车，结束{ return_name }。")
        publish_marker(node, log_file, f"{prefix}|{return_name}结束")

        publish_marker(node, log_file, f"{prefix}|返回后静止开始")
        countdown(node, stationary_s, "返回后静止")
        publish_marker(node, log_file, f"{prefix}|返回后静止结束")
    publish_marker(node, log_file, f"速度校准|{axis_name}|轴测试结束")


def run_yaw(node, log_file, stationary_s):
    """记录纯偏航段，便于检验 child/body 速度旋转到世界 ENU 的链路。"""
    prefix = "速度校准|偏航"
    publish_marker(node, log_file, f"{prefix}|开始")
    ask("保持位置不变；按回车后缓慢、连续地偏航旋转一整圈，再按回车。")
    publish_marker(node, log_file, f"{prefix}|旋转开始")
    ask("偏航旋转已完成且机身已停稳后按回车。")
    publish_marker(node, log_file, f"{prefix}|旋转结束")
    publish_marker(node, log_file, f"{prefix}|结束后静止开始")
    countdown(node, stationary_s, "偏航后静止")
    publish_marker(node, log_file, f"{prefix}|结束后静止结束")
    publish_marker(node, log_file, f"{prefix}|结束")


def main():
    args = parse_args()
    if args.repeats < 1 or args.repeats > 10:
        print("错误：重复次数必须在 1–10 之间", file=sys.stderr)
        return 2
    if args.stationary_seconds < 3 or args.stationary_seconds > 60:
        print("错误：静止秒数必须在 3–60 秒之间", file=sys.stderr)
        return 2
    if args.initial_stationary_seconds < 60 or args.initial_stationary_seconds > 180:
        print("错误：初始静止秒数必须在 60–180 秒之间", file=sys.stderr)
        return 2

    try:
        check_rosbag_running()
        log_file = marker_log_path()
    except TestAborted as error:
        print(f"拒绝启动：{error}", file=sys.stderr)
        return 2

    print("FAST-LIO 三维速度校准")
    print("本脚本只发布阶段标记，不解锁、不切模式、不发布控制指令。")
    print("动作顺序：静止 60 秒 → 上移约 0.5 m → 下移回原位 → ENU X/Y 往返 → 慢速偏航。")
    print("建议两人配合：一人缓慢移动整机，一人操作本终端。")
    confirmation = input("确认已经拆除全部螺旋桨，请输入“已拆桨”：").strip()
    if confirmation != "已拆桨":
        print("未确认拆桨，测试取消。")
        return 2

    rclpy.init()
    node = CalibrationNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    log_file.touch(exist_ok=True)

    result = 0
    test_started = False
    try:
        wait_until_ready(node)
        publishers = node.known_offboard_publishers()
        if publishers:
            raise TestAborted("检测到 Offboard setpoint 发布者：" + "; ".join(publishers))
        print("安全检查通过：未解锁、非 OFFBOARD、EV=HEALTHY、录包标记订阅正常。")
        ask("确认整机位于测试起点，准备开始后按回车。")
        publish_marker(node, log_file, "速度校准|整场开始")
        test_started = True

        publish_marker(node, log_file, "速度校准|开场静止开始")
        countdown(node, args.initial_stationary_seconds, "开场静止（不得接触机架）")
        publish_marker(node, log_file, "速度校准|开场静止结束")

        axes = [
            ("Z轴", "沿 ROS ENU +Z 上移约 0.5 m", "沿 ROS ENU -Z 下移回原位"),
            ("X轴", "沿 ROS ENU +X 缓慢移动", "沿 ROS ENU -X 缓慢返回"),
            ("Y轴", "沿 ROS ENU +Y 缓慢移动", "沿 ROS ENU -Y 缓慢返回"),
        ]
        for axis in axes:
            run_axis(
                node, log_file, *axis,
                repeats=args.repeats,
                stationary_s=args.stationary_seconds,
            )

        run_yaw(node, log_file, args.stationary_seconds)

        publish_marker(node, log_file, "速度校准|整场结束")
        print("\n三维速度校准已完成。请在录包终端按 Ctrl+C 干净保存 rosbag。")
        print(f"标记备份：{log_file}")
    except (TestAborted, KeyboardInterrupt) as error:
        result = 3
        print(f"\n测试中止：{error}", file=sys.stderr)
        if test_started:
            try:
                publish_marker(node, log_file, f"速度校准|整场中止|{error}")
            except Exception:
                pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)
    return result


if __name__ == "__main__":
    raise SystemExit(main())
