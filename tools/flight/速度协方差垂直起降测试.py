#!/usr/bin/env python3
"""FR-LIO 拆桨垂直速度协方差测试：静止、上移约 0.5 m、静止、下移回原位。

只发布 /velocity_calibration/marker，绝不解锁、切模式或发布控制量。
标记兼容 ``tools/analysis/分析速度协方差测试B.py --axes z``。
"""

import argparse
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

import rclpy
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


PROJECT_ROOT = Path(__file__).resolve().parents[2]
RUNTIME_DIR = PROJECT_ROOT / "runtime"
PID_FILE = RUNTIME_DIR / "takeoff_debug_bag.pid"
RUN_DIR_FILE = RUNTIME_DIR / "takeoff_debug_bag_run_dir.txt"
MARKER_TOPIC = "/velocity_calibration/marker"
PRECHAIN_SCRIPT = PROJECT_ROOT / "tools" / "flight" / "一键启动导航栈.sh"


class TestAborted(RuntimeError):
    """操作员中止或运行时安全条件不满足。"""


class VerticalCalibrationNode(Node):
    def __init__(self):
        super().__init__("vertical_velocity_covariance_test")
        reliable = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
        sensor_data = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.marker_publisher = self.create_publisher(String, MARKER_TOPIC, reliable)
        self.create_subscription(State, "/mavros/state", self._state_callback, reliable)
        self.create_subscription(String, "/ev_health/status", self._health_callback, reliable)
        self.create_subscription(
            String,
            "/frlio/high_rate_odom/status",
            self._frlio_status_callback,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        # FR-LIO publishes /Odometry with SensorDataQoS (Best Effort).  A
        # Reliable subscription is incompatible and receives no samples.
        self.create_subscription(Odometry, "/Odometry", self._odom_callback, sensor_data)
        self.create_subscription(
            Odometry, "/mavros/odometry/out", self._ev_output_callback, reliable
        )
        self.state = None
        self.health = None
        self.frlio_status = None
        self.last_odom_monotonic = None
        self.last_ev_output_monotonic = None

    def _state_callback(self, message):
        self.state = message

    def _health_callback(self, message):
        self.health = message.data

    def _frlio_status_callback(self, message):
        self.frlio_status = message.data

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
        if self.frlio_status is None:
            gaps.append("/frlio/high_rate_odom/status")
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
            raise TestAborted("检测到飞机已解锁；本测试必须拆桨且未解锁")
        if self.state.mode != "STABILIZED":
            raise TestAborted(f"飞控模式必须是 STABILIZED，当前为 {self.state.mode}")
        if self.health is None:
            raise TestAborted("尚未收到 /ev_health/status")
        if self.health == "FAULT":
            raise TestAborted("EV 已进入 FAULT，请停止移动并检查定位链")
        if require_healthy and self.health != "HEALTHY":
            raise TestAborted(f"EV 当前不是 HEALTHY，而是 {self.health}")
        if self.frlio_status is None:
            raise TestAborted("尚未收到 /frlio/high_rate_odom/status；拒绝在非 FR-LIO 链路上测试")
        if self.frlio_status != "HEALTHY":
            raise TestAborted(f"FR-LIO 高频里程计不是 HEALTHY，而是 {self.frlio_status}")
        if self.last_odom_monotonic is None or now - self.last_odom_monotonic > 0.5:
            raise TestAborted("/Odometry 已超过 0.5 秒没有新消息")
        if self.last_ev_output_monotonic is None or now - self.last_ev_output_monotonic > 0.5:
            raise TestAborted("/mavros/odometry/out 已超过 0.5 秒没有新消息")


def parse_args():
    parser = argparse.ArgumentParser(description="FR-LIO 拆桨垂直速度协方差测试（只写标记，不发控制量）")
    parser.add_argument("--重复次数", "--repeats", dest="repeats", type=int, default=3,
                        help="上移/回位循环次数，默认 3")
    parser.add_argument("--初始静止秒数", "--initial-stationary-seconds", dest="initial_stationary_seconds", type=int, default=60,
                        help="开始前静止时长，默认且最少 60 秒")
    parser.add_argument("--静止秒数", "--stationary-seconds", dest="stationary_seconds", type=int, default=10,
                        help="每次上移和回位后的静止时长，默认 10 秒")
    parser.add_argument("--上移米", "--rise-meters", dest="rise_meters", type=float, default=0.5,
                        help="人工上移高度提示，默认 0.5 m；脚本不会控制运动")
    parser.add_argument(
        "--不启动前置链路", "--no-start-prechain", dest="start_prechain",
        action="store_false",
        help="使用已运行的 FR-LIO/MAVROS/EV/rosbag 链路；默认由本脚本启动",
    )
    parser.set_defaults(start_prechain=True)
    parser.add_argument(
        "--前置链路超时", "--prechain-timeout", dest="prechain_timeout", type=float,
        default=120.0, help="等待自动启动链路就绪的最长秒数，默认 120",
    )
    return parser.parse_args()


def ask(prompt):
    try:
        input(prompt)
    except (EOFError, KeyboardInterrupt):
        raise TestAborted("操作员中止")


def check_rosbag_running():
    try:
        pid = int(PID_FILE.read_text(encoding="utf-8").strip())
    except (FileNotFoundError, ValueError):
        raise TestAborted("未找到正在运行的录包进程；请先运行 tools/rosbag/开始录包.sh")
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        raise TestAborted(f"录包进程 pid={pid} 已退出")
    except PermissionError:
        raise TestAborted(f"无权检查录包进程 pid={pid}")


def topic_has_publisher(topic):
    """用 ROS 图确认已有链路，避免自动启动与操作员手动链路撞车。"""
    try:
        result = subprocess.run(
            ["ros2", "topic", "info", "-v", topic],
            check=False, capture_output=True, text=True, timeout=5.0,
        )
    except (FileNotFoundError, subprocess.TimeoutExpired) as error:
        raise TestAborted(f"无法查询 ROS 图以检查前置链路：{error}")
    return "Publisher count:" in result.stdout and not "Publisher count: 0" in result.stdout


def start_frlio_prechain():
    """启动无桨 FR-LIO 全链路，返回本脚本拥有的 launcher 进程或 None。

    ``一键启动导航栈.sh`` 负责 MID-360、FR-LIO、MAVROS/EV 和 rosbag 的完整
    生命周期。此处明确关闭任务和控制输出，不能把校准变成飞行任务。
    """
    if topic_has_publisher("/frlio/high_rate_odom/status"):
        print("检测到已有 FR-LIO 前置链路；复用它，不重复启动。", flush=True)
        return None
    if topic_has_publisher("/Odometry"):
        raise TestAborted(
            "检测到 /Odometry 发布者但没有 FR-LIO 状态发布者；可能是 FAST-LIO 或残留链路。"
            "请先清理，或用 --不启动前置链路 明确复用已确认的 FR-LIO 链路。"
        )
    if not PRECHAIN_SCRIPT.is_file() or not os.access(PRECHAIN_SCRIPT, os.X_OK):
        raise TestAborted(f"找不到可执行的 FR-LIO 前置链路脚本：{PRECHAIN_SCRIPT}")
    environment = os.environ.copy()
    environment.update({
        "LIO_BACKEND": "fr_lio",
        "NO_MAP_VALIDATION": "true",
        "NAVIGATION_ENABLED": "false",
        "ENABLE_OUTPUT": "false",
        "MISSION_ENABLED": "false",
        "RVIZ": "false",
        "RECORD_BAG": "true",
    })
    print("启动无桨前置链路：MID-360 → FR-LIO → MAVROS/EV → rosbag。", flush=True)
    return subprocess.Popen([str(PRECHAIN_SCRIPT)], env=environment, start_new_session=True)


def stop_owned_prechain(process):
    if process is None or process.poll() is not None:
        return
    print("正在停止本次测试启动的前置链路并保存 rosbag…", flush=True)
    process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=35.0)
    except KeyboardInterrupt:
        print("\n收到再次中止；请求前置链路立即完成清理。", file=sys.stderr)
        process.terminate()
        try:
            process.wait(timeout=10.0)
        except subprocess.TimeoutExpired:
            process.kill()
    except subprocess.TimeoutExpired:
        print("前置链路未在 35 秒内退出，发送 SIGTERM。", file=sys.stderr)
        process.terminate()
        try:
            process.wait(timeout=10.0)
        except subprocess.TimeoutExpired:
            print("前置链路仍未退出，发送 SIGKILL。", file=sys.stderr)
            process.kill()


def marker_log_path():
    try:
        run_root = Path(RUN_DIR_FILE.read_text(encoding="utf-8").strip())
    except FileNotFoundError:
        raise TestAborted("找不到本场次目录；请先启动录包")
    if not run_root.is_dir():
        raise TestAborted(f"本场次目录不存在：{run_root}")
    snapshot_dir = run_root / "snapshot"
    snapshot_dir.mkdir(parents=True, exist_ok=True)
    return snapshot_dir / "垂直速度协方差测试标记.txt"


def wait_until_ready(node, timeout_s=15.0, prechain_process=None):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if prechain_process is not None and prechain_process.poll() is not None:
            raise TestAborted(
                f"自动启动的前置链路已退出（code={prechain_process.returncode}）；"
                "请查看一键启动链路的终端日志。"
            )
        if (node.state is not None and node.health is not None and node.frlio_status is not None and
                node.last_odom_monotonic is not None and node.last_ev_output_monotonic is not None and
                node.marker_publisher.get_subscription_count() > 0):
            node.check_runtime(require_healthy=True)
            return
        time.sleep(0.1)
    gaps = node.readiness_gaps()
    detail = "、".join(gaps) if gaps else "话题已收到但运行状态未通过"
    raise TestAborted(f"前置链路就绪超时；未就绪：{detail}")


def publish_marker(node, log_file, label):
    check_rosbag_running()
    node.check_runtime()
    node.marker_publisher.publish(String(data=label))
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


def run_cycle(node, log_file, repeat, args):
    prefix = f"速度校准|Z轴|第{repeat:02d}次"
    publish_marker(node, log_file, f"{prefix}|起点静止开始")
    countdown(node, args.stationary_seconds, "起点静止")
    publish_marker(node, log_file, f"{prefix}|起点静止结束")

    up = f"沿 ROS ENU +Z 上移约 {args.rise_meters:.2f} m"
    ask(f"准备好后按回车；随后缓慢、平稳地{up}，到达后再次按回车。")
    publish_marker(node, log_file, f"{prefix}|{up}|开始")
    ask("已到达上方位置并完全停稳后按回车。")
    publish_marker(node, log_file, f"{prefix}|{up}|结束")

    publish_marker(node, log_file, f"{prefix}|终点静止开始")
    countdown(node, args.stationary_seconds, "上方静止")
    publish_marker(node, log_file, f"{prefix}|终点静止结束")

    down = "沿 ROS ENU -Z 下移回原位"
    ask(f"准备好后按回车；随后缓慢、平稳地{down}，回到起点后再次按回车。")
    publish_marker(node, log_file, f"{prefix}|{down}|开始")
    ask("已回到原位并完全停稳后按回车。")
    publish_marker(node, log_file, f"{prefix}|{down}|结束")

    publish_marker(node, log_file, f"{prefix}|返回后静止开始")
    countdown(node, args.stationary_seconds, "回位后静止")
    publish_marker(node, log_file, f"{prefix}|返回后静止结束")


def main():
    args = parse_args()
    if not 1 <= args.repeats <= 10:
        print("错误：重复次数必须在 1–10 之间", file=sys.stderr)
        return 2
    if not 60 <= args.initial_stationary_seconds <= 180:
        print("错误：初始静止秒数必须在 60–180 秒之间", file=sys.stderr)
        return 2
    if not 3 <= args.stationary_seconds <= 60:
        print("错误：静止秒数必须在 3–60 秒之间", file=sys.stderr)
        return 2
    if not 0.2 <= args.rise_meters <= 1.0:
        print("错误：上移高度必须在 0.2–1.0 m 之间", file=sys.stderr)
        return 2
    if not 15.0 <= args.prechain_timeout <= 300.0:
        print("错误：前置链路超时必须在 15–300 秒之间", file=sys.stderr)
        return 2

    print("FR-LIO 垂直速度协方差测试（拆桨）")
    print("流程：静止 60 秒 → ROS ENU +Z 上移约 0.5 m → 静止 → -Z 回原位 → 静止。")
    print("脚本只写 rosbag 标记；不解锁、不切模式、不发布控制量。")
    if input("确认已经拆除全部螺旋桨，请输入“已拆桨”：").strip() != "已拆桨":
        print("未确认拆桨，测试取消。")
        return 2

    prechain_process = None
    result, started = 0, False
    node = None
    executor = None
    spin_thread = None
    try:
        if args.start_prechain:
            prechain_process = start_frlio_prechain()
        rclpy.init()
        node = VerticalCalibrationNode()
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()
        wait_until_ready(node, timeout_s=args.prechain_timeout, prechain_process=prechain_process)
        check_rosbag_running()
        log_file = marker_log_path()
        log_file.touch(exist_ok=True)
        publishers = node.known_offboard_publishers()
        if publishers:
            raise TestAborted("检测到 Offboard setpoint 发布者：" + "; ".join(publishers))
        print("安全检查通过：FR-LIO=HEALTHY、EV=HEALTHY、未解锁、STABILIZED、录包标记订阅正常。")
        ask("确认机架位于起点且无人接触，按回车开始 60 秒静止。")
        publish_marker(node, log_file, "速度校准|整场开始")
        started = True
        publish_marker(node, log_file, "速度校准|开场静止开始")
        countdown(node, args.initial_stationary_seconds, "开场静止（不得接触机架）")
        publish_marker(node, log_file, "速度校准|开场静止结束")
        for repeat in range(1, args.repeats + 1):
            print(f"\n===== 垂直循环 {repeat}/{args.repeats} =====")
            run_cycle(node, log_file, repeat, args)
        publish_marker(node, log_file, "速度校准|整场结束")
        print("\n垂直测试完成。请在录包终端按 Ctrl+C 干净保存 rosbag。")
        print(f"标记备份：{log_file}")
    except (TestAborted, KeyboardInterrupt) as error:
        result = 3
        print(f"\n测试中止：{error}", file=sys.stderr)
        if started:
            try:
                publish_marker(node, log_file, f"速度校准|整场中止|{error}")
            except Exception:
                pass
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if spin_thread is not None:
            spin_thread.join(timeout=2.0)
        stop_owned_prechain(prechain_process)
    return result


if __name__ == "__main__":
    raise SystemExit(main())
