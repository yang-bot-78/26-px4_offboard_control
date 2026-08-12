#!/usr/bin/env python3
"""人工驾驶的 FAST-LIO 垂直起降测试；脚本只检查状态和写 rosbag 阶段标记。"""

import argparse
import os
from pathlib import Path
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import TwistWithCovarianceStamped
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


class VerticalTestNode(Node):
    def __init__(self):
        super().__init__("velocity_covariance_vertical_test")
        reliable = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
        self.marker_publisher = self.create_publisher(String, MARKER_TOPIC, reliable)
        self.create_subscription(State, "/mavros/state", self._state_callback, reliable)
        self.create_subscription(String, "/ev_health/status", self._health_callback, reliable)
        self.create_subscription(Odometry, "/Odometry", self._odom_callback, reliable)
        self.create_subscription(
            TwistWithCovarianceStamped,
            "/mavros/vision_speed/speed_twist_cov",
            self._speed_callback,
            reliable,
        )
        self.state = None
        self.health = None
        self.last_odom_monotonic = None
        self.last_speed_monotonic = None
        self._last_health_warning = None

    def _state_callback(self, message):
        self.state = message

    def _health_callback(self, message):
        self.health = message.data

    def _odom_callback(self, _message):
        self.last_odom_monotonic = time.monotonic()

    def _speed_callback(self, _message):
        self.last_speed_monotonic = time.monotonic()

    def known_offboard_publishers(self):
        publishers = []
        for topic in ("/mavros/setpoint_raw/local", "/fmu/in/trajectory_setpoint"):
            for endpoint in self.get_publishers_info_by_topic(topic):
                publishers.append(f"{topic}: {endpoint.node_namespace}/{endpoint.node_name}")
        return publishers

    def check_runtime(self, require_armed=None):
        now = time.monotonic()
        if self.state is None:
            raise TestAborted("尚未收到 /mavros/state")
        if not self.state.connected:
            raise TestAborted("MAVROS 已断开")
        if require_armed is not None and self.state.armed != require_armed:
            expected = "已解锁" if require_armed else "未解锁"
            raise TestAborted(f"当前必须是{expected}")
        if self.state.mode != "STABILIZED":
            raise TestAborted(f"垂直测试必须保持 STABILIZED，当前为 {self.state.mode}")
        if self.health is None:
            raise TestAborted("尚未收到 /ev_health/status")
        if self.health == "FAULT":
            raise TestAborted("EV 已进入 FAULT，请立即停止飞行并降落")
        if self.health != "HEALTHY" and self.health != self._last_health_warning:
            print(f"\n[警告] EV 状态变为 {self.health}，请停止动作并准备降落。", flush=True)
            self._last_health_warning = self.health
        if self.last_odom_monotonic is None or now - self.last_odom_monotonic > 0.5:
            raise TestAborted("/Odometry 已超过 0.5 秒没有新消息")
        if self.last_speed_monotonic is None or now - self.last_speed_monotonic > 0.5:
            raise TestAborted("vision_speed 已超过 0.5 秒没有新消息")


def parse_args():
    parser = argparse.ArgumentParser(
        description="人工驾驶的 FAST-LIO 垂直起降测试（只写标记，不发控制量）"
    )
    parser.add_argument(
        "--允许有桨飞行", "--allow-props-flight", dest="allow_props_flight",
        action="store_true", help="显式确认本脚本用于有桨人工飞行；默认拒绝启动",
    )
    parser.add_argument(
        "--重复次数", "--repeats", dest="repeats", type=int, default=3,
        help="垂直起降重复次数，默认 3",
    )
    parser.add_argument(
        "--悬停秒数", "--hover-seconds", dest="hover_seconds", type=int, default=10,
        help="每次到达约 1 m 后的悬停秒数，默认 10",
    )
    parser.add_argument(
        "--地面静止秒数", "--ground-seconds", dest="ground_seconds", type=int, default=5,
        help="起飞前和落地后的静止秒数，默认 5",
    )
    parser.add_argument(
        "--起飞高度", "--takeoff-height", dest="takeoff_height", type=float, default=1.0,
        help="提示用目标高度，单位 m，默认 1.0；脚本不会控制高度",
    )
    return parser.parse_args()


def ask(prompt):
    try:
        input(prompt)
    except (EOFError, KeyboardInterrupt):
        raise TestAborted("操作员中止")


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
    return snapshot_dir / "速度协方差垂直起降标记.txt"


def wait_until_ready(node, timeout_s=15.0):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if (
            node.state is not None
            and node.health is not None
            and node.last_odom_monotonic is not None
            and node.last_speed_monotonic is not None
            and node.marker_publisher.get_subscription_count() > 0
        ):
            node.check_runtime(require_armed=False)
            return
        time.sleep(0.1)
    raise TestAborted("等待 MAVROS、EV、Odometry、vision_speed 或 rosbag 标记订阅者超时")


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


def countdown(node, seconds, description, require_armed=None):
    print(f"{description}：保持 {seconds} 秒", flush=True)
    for remaining in range(seconds, 0, -1):
        node.check_runtime(require_armed=require_armed)
        print(f"  剩余 {remaining} 秒", end="\r", flush=True)
        time.sleep(1.0)
    print(" " * 30, end="\r", flush=True)


def run_one_cycle(node, log_file, repeat, args):
    prefix = f"测试垂直|第{repeat:02d}次"
    publish_marker(node, log_file, f"{prefix}|地面静止开始")
    countdown(node, args.ground_seconds, "起飞前地面静止", require_armed=False)
    publish_marker(node, log_file, f"{prefix}|地面静止结束")

    ask(
        f"确认遥控器可接管、模式为 STABILIZED；准备手动解锁并起飞到约 "
        f"{args.takeoff_height:.1f} m 后按回车。脚本不会解锁或控制飞机。"
    )
    node.check_runtime(require_armed=True)
    publish_marker(node, log_file, f"{prefix}|起飞上升开始")
    ask(f"已到约 {args.takeoff_height:.1f} m 且姿态稳定后按回车，开始悬停。")
    node.check_runtime(require_armed=True)
    publish_marker(node, log_file, f"{prefix}|起飞上升结束")

    publish_marker(node, log_file, f"{prefix}|空中悬停开始")
    countdown(node, args.hover_seconds, "空中悬停", require_armed=True)
    publish_marker(node, log_file, f"{prefix}|空中悬停结束")

    ask("准备手动下降；按回车后下降并在原地落地，完全接地后再次按回车。")
    node.check_runtime(require_armed=True)
    publish_marker(node, log_file, f"{prefix}|下降着陆开始")
    ask("确认飞机已经接地且姿态稳定后按回车，结束下降阶段。")
    node.check_runtime(require_armed=True)
    publish_marker(node, log_file, f"{prefix}|下降着陆结束")

    ask("现在手动上锁；确认 /mavros/state 显示 armed=false 后按回车。")
    node.check_runtime(require_armed=False)
    publish_marker(node, log_file, f"{prefix}|落地静止开始")
    countdown(node, args.ground_seconds, "落地后地面静止", require_armed=False)
    publish_marker(node, log_file, f"{prefix}|落地静止结束")


def main():
    args = parse_args()
    if not args.allow_props_flight:
        print(
            "拒绝启动：这是有桨人工飞行标记脚本。确认测试区、安全员和遥控器接管后，"
            "重新添加 --允许有桨飞行。脚本仍不会自动解锁或发布控制量。",
            file=sys.stderr,
        )
        return 2
    if not 1 <= args.repeats <= 10:
        print("错误：重复次数必须在 1–10 之间", file=sys.stderr)
        return 2
    if not 3 <= args.hover_seconds <= 60:
        print("错误：悬停秒数必须在 3–60 之间", file=sys.stderr)
        return 2
    if not 3 <= args.ground_seconds <= 60:
        print("错误：地面静止秒数必须在 3–60 之间", file=sys.stderr)
        return 2
    if not 0.3 <= args.takeoff_height <= 3.0:
        print("错误：目标高度提示必须在 0.3–3.0 m 之间", file=sys.stderr)
        return 2

    try:
        check_rosbag_running()
        log_file = marker_log_path()
    except TestAborted as error:
        print(f"拒绝启动：{error}", file=sys.stderr)
        return 2

    print("FAST-LIO 垂直起降速度协方差测试")
    print("脚本只做状态检查和阶段标记，不解锁、不切模式、不发布控制指令。")
    print("飞行员必须全程保持 STABILIZED，并准备立即手动降落。")
    confirmation = input("确认测试区清空、遥控器可接管、安全员在位，请输入“已确认”：").strip()
    if confirmation != "已确认":
        print("未确认安全条件，测试取消。")
        return 2

    rclpy.init()
    node = VerticalTestNode()
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
        print("安全检查通过：未解锁、STABILIZED、EV=HEALTHY、录包标记订阅正常。")
        ask("确认飞机位于起飞点，准备开始后按回车。")
        publish_marker(node, log_file, "测试垂直|整场开始")
        test_started = True

        for repeat in range(1, args.repeats + 1):
            print(f"\n===== 垂直起降第 {repeat}/{args.repeats} 次 =====")
            run_one_cycle(node, log_file, repeat, args)

        publish_marker(node, log_file, "测试垂直|整场结束")
        print("\n垂直起降测试已完成。请在录包终端按 Ctrl+C 干净保存 rosbag。")
        print(f"标记备份：{log_file}")
    except (TestAborted, KeyboardInterrupt) as error:
        result = 3
        print(f"\n测试中止：{error}", file=sys.stderr)
        if test_started:
            try:
                publish_marker(node, log_file, f"测试垂直|整场中止|{error}")
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
