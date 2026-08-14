#!/usr/bin/env python3
"""以单个常驻 ROS 2 订阅器检查 EV flight_ready 的连续健康状态。"""

import argparse
import time
from typing import Dict, Optional

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool


class FlightReadyGate(Node):
    def __init__(self, timeout_s: float, stable_s: float, max_message_age_s: float):
        super().__init__("flight_ready_stability_gate")
        self._deadline_s = time.monotonic() + timeout_s
        self._stable_s = stable_s
        self._max_message_age_s = max_message_age_s
        self._true_since_s: Optional[float] = None
        self._last_message_s: Optional[float] = None
        self._last_ready: Optional[bool] = None
        self._diagnostics: Dict[str, str] = {}
        self._done = False
        self._success = False

        status_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        diagnostics_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            Bool, "/ev_health/flight_ready", self._ready_callback, status_qos
        )
        self.create_subscription(
            DiagnosticArray,
            "/ev_health/diagnostics",
            self._diagnostics_callback,
            diagnostics_qos,
        )
        self.create_timer(0.05, self._check_deadline)

    @property
    def done(self) -> bool:
        return self._done

    @property
    def success(self) -> bool:
        return self._success

    def _ready_callback(self, msg: Bool) -> None:
        now_s = time.monotonic()
        self._last_message_s = now_s
        self._last_ready = bool(msg.data)
        if self._last_ready:
            if self._true_since_s is None:
                self._true_since_s = now_s
        else:
            self._true_since_s = None

    def _diagnostics_callback(self, msg: DiagnosticArray) -> None:
        for status in msg.status:
            if status.name != "fastlio_ev_health":
                continue
            self._diagnostics = {item.key: item.value for item in status.values}
            self._diagnostics["message"] = status.message
            return

    def _check_deadline(self) -> None:
        now_s = time.monotonic()
        message_fresh = (
            self._last_message_s is not None
            and now_s - self._last_message_s <= self._max_message_age_s
        )
        if not message_fresh:
            self._true_since_s = None
        if (
            self._true_since_s is not None
            and now_s - self._true_since_s >= self._stable_s
        ):
            self._success = True
            self._done = True
            return
        if now_s >= self._deadline_s:
            self._done = True

    def failure_summary(self) -> str:
        if self._last_message_s is None:
            state = "未收到 /ev_health/flight_ready"
        else:
            age_s = time.monotonic() - self._last_message_s
            state = f"last_flight_ready={self._last_ready} age={age_s:.3f}s"
        diagnostic_keys = (
            "state",
            "reason",
            "flight_ready_reason",
            "frlio_status",
            "frlio_gate_reason",
        )
        details = [
            f"{key}={self._diagnostics[key]}"
            for key in diagnostic_keys
            if key in self._diagnostics
        ]
        return state if not details else f"{state}; " + ", ".join(details)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="等待 /ev_health/flight_ready 连续一段时间为 true"
    )
    parser.add_argument("--timeout-s", type=float, default=30.0)
    parser.add_argument("--stable-s", type=float, default=3.0)
    parser.add_argument("--max-message-age-s", type=float, default=0.5)
    args = parser.parse_args()
    if args.timeout_s <= 0.0 or args.stable_s <= 0.0 or args.max_message_age_s <= 0.0:
        parser.error("所有时间参数必须大于 0")
    return args


def main() -> int:
    args = parse_args()
    print(
        f"[检查] 以常驻订阅器等待 flight_ready 连续 {args.stable_s:g} 秒为 true。",
        flush=True,
    )
    rclpy.init()
    node = FlightReadyGate(args.timeout_s, args.stable_s, args.max_message_age_s)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.success:
            print(f"[就绪] flight_ready 已连续 {args.stable_s:g} 秒为 true。", flush=True)
            return 0
        print(
            f"[错误] flight_ready 未在 {args.timeout_s:g} 秒内连续 "
            f"{args.stable_s:g} 秒为 true：{node.failure_summary()}。",
            flush=True,
        )
        return 1
    except (KeyboardInterrupt, ExternalShutdownException):
        return 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
