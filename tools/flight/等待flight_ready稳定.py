#!/usr/bin/env python3
"""在一个连续 ROS 2 时间窗内检查 MAVLink ODOMETRY 速度融合链路。"""

import argparse
import math
import time
from typing import Dict, Optional

from mavros_msgs.msg import EstimatorStatus, State
from nav_msgs.msg import Odometry
import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from std_msgs.msg import Bool


class FlightReadyGate(Node):
    def __init__(
        self,
        timeout_s: float,
        stable_s: float,
        odometry_max_message_age_s: float,
        status_max_message_age_s: float,
        require_disarmed: bool,
        reject_offboard: bool,
    ):
        super().__init__("mavlink_odometry_stability_gate")
        self._deadline_s = time.monotonic() + timeout_s
        self._stable_s = stable_s
        # MAVROS state 和 estimator_status 实测约 1 Hz。它们是状态量，
        # 不应按高频 ODOMETRY 的时效要求检查；ODOMETRY 仍严格限制在 0.5 s。
        self._odometry_max_message_age_s = odometry_max_message_age_s
        self._status_max_message_age_s = status_max_message_age_s
        self._true_since_s: Optional[float] = None
        self._last_message_s: Optional[float] = None
        self._last_ready: Optional[bool] = None
        self._state: Optional[State] = None
        self._state_message_s: Optional[float] = None
        self._estimator: Optional[EstimatorStatus] = None
        self._estimator_message_s: Optional[float] = None
        self._mavlink_odometry: Optional[Odometry] = None
        self._mavlink_odometry_message_s: Optional[float] = None
        self._fused_odometry: Optional[Odometry] = None
        self._fused_odometry_message_s: Optional[float] = None
        self._require_disarmed = require_disarmed
        self._reject_offboard = reject_offboard
        self._diagnostics: Dict[str, str] = {}
        self._done = False
        self._success = False
        self._last_block_reason: Optional[str] = None
        self._last_block_s: Optional[float] = None

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
        self.create_subscription(
            State, "/mavros/state", self._state_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            EstimatorStatus,
            "/mavros/estimator_status",
            self._estimator_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            "/mavros/odometry/out",
            self._mavlink_odometry_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            "/mavros/local_position/odom",
            self._fused_odometry_callback,
            qos_profile_sensor_data,
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

    def _state_callback(self, msg: State) -> None:
        self._state = msg
        self._state_message_s = time.monotonic()

    def _estimator_callback(self, msg: EstimatorStatus) -> None:
        self._estimator = msg
        self._estimator_message_s = time.monotonic()

    def _mavlink_odometry_callback(self, msg: Odometry) -> None:
        self._mavlink_odometry = msg
        self._mavlink_odometry_message_s = time.monotonic()

    def _fused_odometry_callback(self, msg: Odometry) -> None:
        self._fused_odometry = msg
        self._fused_odometry_message_s = time.monotonic()

    @staticmethod
    def _fresh(
        received_s: Optional[float], now_s: float, max_message_age_s: float
    ) -> bool:
        return received_s is not None and now_s - received_s <= max_message_age_s

    @staticmethod
    def _finite(values) -> bool:
        return all(math.isfinite(value) for value in values)

    def _not_ready_reason(self, now_s: float) -> str:
        if not self._fresh(
            self._last_message_s, now_s, self._odometry_max_message_age_s
        ):
            return "flight_ready_stale"
        if self._last_ready is not True:
            return "flight_ready_false"
        if not self._fresh(
            self._state_message_s, now_s, self._status_max_message_age_s
        ) or self._state is None:
            return "mavros_state_stale"
        if not self._state.connected:
            return "mavros_disconnected"
        if self._require_disarmed and self._state.armed:
            return "vehicle_armed"
        if self._reject_offboard and self._state.mode == "OFFBOARD":
            return "offboard_mode"
        if not self._fresh(
            self._estimator_message_s, now_s, self._status_max_message_age_s
        ) or self._estimator is None:
            return "estimator_status_stale"
        if not self._estimator.pos_horiz_rel_status_flag:
            return "horizontal_position_invalid"
        if not self._estimator.pos_vert_abs_status_flag:
            return "vertical_position_invalid"
        if not self._fresh(
            self._mavlink_odometry_message_s,
            now_s,
            self._odometry_max_message_age_s,
        ) or self._mavlink_odometry is None:
            return "mavlink_odometry_stale"
        source = self._mavlink_odometry
        source_velocity = source.twist.twist.linear
        source_covariance = source.twist.covariance
        if not self._finite((source_velocity.x, source_velocity.y, source_velocity.z)):
            return "mavlink_odometry_velocity_nonfinite"
        if not self._finite((source_covariance[0], source_covariance[7], source_covariance[14])) or any(
            value <= 0.0 for value in (source_covariance[0], source_covariance[7], source_covariance[14])
        ):
            return "mavlink_odometry_velocity_covariance_invalid"
        if not self._fresh(
            self._fused_odometry_message_s,
            now_s,
            self._odometry_max_message_age_s,
        ) or self._fused_odometry is None:
            return "fused_local_odometry_stale"
        fused = self._fused_odometry
        fused_position = fused.pose.pose.position
        fused_velocity = fused.twist.twist.linear
        if not self._finite((fused_position.x, fused_position.y, fused_position.z)):
            return "fused_local_position_nonfinite"
        if not self._finite((fused_velocity.x, fused_velocity.y, fused_velocity.z)):
            return "fused_local_velocity_nonfinite"
        return ""

    def _check_deadline(self) -> None:
        now_s = time.monotonic()
        reason = self._not_ready_reason(now_s)
        if reason:
            self._true_since_s = None
            self._last_block_reason = reason
            self._last_block_s = now_s
        elif self._true_since_s is None:
            self._true_since_s = now_s
        if self._true_since_s is not None and now_s - self._true_since_s >= self._stable_s:
            self._success = True
            self._done = True
            return
        if now_s >= self._deadline_s:
            self._done = True

    @staticmethod
    def _reason_text(reason: str) -> str:
        reasons = {
            "flight_ready_stale": "飞行就绪状态超时",
            "flight_ready_false": "飞行就绪状态为 false",
            "mavros_state_stale": "MAVROS 飞控状态超时",
            "mavros_disconnected": "MAVROS 未连接飞控",
            "vehicle_armed": "飞行器已解锁",
            "offboard_mode": "飞行器处于 OFFBOARD 模式",
            "estimator_status_stale": "EKF 状态超时",
            "horizontal_position_invalid": "PX4 水平位置无效",
            "vertical_position_invalid": "PX4 垂直位置无效",
            "mavlink_odometry_stale": "MAVLink ODOMETRY 输入超时",
            "mavlink_odometry_velocity_nonfinite": "ODOMETRY 速度存在非有限值",
            "mavlink_odometry_velocity_covariance_invalid": "ODOMETRY 速度协方差无效",
            "fused_local_odometry_stale": "PX4 融合里程计超时",
            "fused_local_position_nonfinite": "PX4 融合位置存在非有限值",
            "fused_local_velocity_nonfinite": "PX4 融合速度存在非有限值",
        }
        return reasons.get(reason, reason)

    def failure_summary(self) -> str:
        reason = self._not_ready_reason(time.monotonic())
        if reason:
            state = self._reason_text(reason)
        else:
            elapsed_s = 0.0
            if self._true_since_s is not None:
                elapsed_s = time.monotonic() - self._true_since_s
            state = f"当前条件已恢复，但连续有效仅 {elapsed_s:.1f} 秒"
            if self._last_block_reason is not None and self._last_block_s is not None:
                ago_s = time.monotonic() - self._last_block_s
                state += (
                    f"；最近阻断：{self._reason_text(self._last_block_reason)}"
                    f"（{ago_s:.1f} 秒前）"
                )
        diagnostic_keys = (
            ("state", "诊断状态"),
            ("reason", "诊断原因"),
            ("flight_ready_reason", "飞行就绪原因"),
            ("frlio_status", "FR-LIO 状态"),
            ("frlio_gate_reason", "FR-LIO 门禁原因"),
        )
        details = [
            f"{label}={self._diagnostics[key]}"
            for key, label in diagnostic_keys
            if key in self._diagnostics
        ]
        return state if not details else f"{state}; " + ", ".join(details)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="等待 MAVLink ODOMETRY 输入速度和 PX4 融合速度连续有效"
    )
    parser.add_argument("--timeout-s", type=float, default=30.0)
    parser.add_argument("--stable-s", type=float, default=3.0)
    parser.add_argument(
        "--max-message-age-s",
        type=float,
        default=0.5,
        help="高频 ODOMETRY 和 flight_ready 的最大消息时效（秒）",
    )
    parser.add_argument(
        "--status-max-message-age-s",
        type=float,
        default=2.5,
        help="低频 MAVROS/EKF 状态的最大消息时效（秒）",
    )
    parser.add_argument("--require-disarmed", action="store_true")
    parser.add_argument("--reject-offboard", action="store_true")
    args = parser.parse_args()
    if (
        args.timeout_s <= 0.0
        or args.stable_s <= 0.0
        or args.max_message_age_s <= 0.0
        or args.status_max_message_age_s <= 0.0
    ):
        parser.error("所有时间参数必须大于 0")
    return args


def main() -> int:
    args = parse_args()
    print(
        f"[检查] 等待 MAVLink ODOMETRY 速度和 PX4 融合里程计连续 "
        f"{args.stable_s:g} 秒有效。",
        flush=True,
    )
    rclpy.init()
    node = FlightReadyGate(
        args.timeout_s,
        args.stable_s,
        args.max_message_age_s,
        args.status_max_message_age_s,
        args.require_disarmed,
        args.reject_offboard,
    )
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.success:
            print(
                f"[就绪] MAVLink ODOMETRY 速度和 PX4 融合里程计已连续 "
                f"{args.stable_s:g} 秒有效。",
                flush=True,
            )
            return 0
        print(
            f"[错误] ODOMETRY 速度门未在 {args.timeout_s:g} 秒内连续 "
            f"{args.stable_s:g} 秒满足：{node.failure_summary()}。",
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
