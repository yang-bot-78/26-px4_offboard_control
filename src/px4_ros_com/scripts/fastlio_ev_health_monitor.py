#!/usr/bin/env python3
"""Gate external vision using timestamp and PX4 velocity consistency."""

from __future__ import annotations

import copy
import math
from typing import Iterable, List

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TwistStamped, TwistWithCovarianceStamped
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from px4_msgs.msg import EstimatorAidSource2d, VehicleLocalPosition
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool, Float64, String, UInt8

from px4_ros_com.ev_health import (
    covariance_3x3_is_symmetric_psd,
    EvHealthMonitorCore,
    HealthConfig,
    apply_world_velocity_covariance_floors,
    HealthState,
    ev_enu_to_px4_ned,
    quaternion_rotation_matrix_xyzw,
    rotate_vector,
)


def _stamp_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _vector_text(values: Iterable[float]) -> str:
    return ",".join("nan" if not math.isfinite(value) else f"{value:.6f}" for value in values)


# Statuses that mean "the propagated state itself is unusable".  These are the
# only FR-LIO statuses that may block the EV stream, because they are the only
# ones asserting that there is no trustworthy current state.
_FRLIO_STATE_UNUSABLE_STATUSES = frozenset(
    {
        "FAULT_STATE_UNUSABLE",
        "UNIT_UNCONFIRMED",
        "WAITING_FOR_LIDAR",
        "DISABLED",
    }
)

# Statuses that mean "LiDAR corrections are late, the predictor is still real
# time". EV continues with FR-LIO's propagated covariance; only the planner stops.
_FRLIO_PLANNER_UNUSABLE_STATUSES = frozenset({"FAULT_STALE_LIDAR"})

_FRLIO_DEGRADED_STATUSES = frozenset({"SUSPECT_STALE_LIDAR"})


def _evaluate_frlio_anchor_gate(
    *,
    required: bool,
    status,
    status_seen: bool,
    status_message_age_s: float,
    anchor_age_s: float,
    anchor_message_age_s: float,
    message_timeout_s: float,
    max_anchor_age_s: float,
    predictor_age_s: float = math.nan,
    predictor_message_age_s: float = math.inf,
    max_predictor_age_s: float = 0.05,
    allow_stale_lidar_predictor_degraded: bool = False,
):
    """Return ``(reason, block_output)`` for the FR-LIO safety gate.

    ``block_output`` means "do not hand this sample to PX4 as EV".  It is
    reserved for the cases where the propagated state itself is untrustworthy:

    * FR-LIO reports a STATE_UNUSABLE-class status,
    * the status/predictor stream is missing or timed out (we cannot tell), or
    * the predictor age exceeds ``max_predictor_age_s``.

    A stale *anchor* (late LiDAR posterior) is deliberately NOT a block. It is
    reported as a degraded reason so the planner stops while EV stays continuous.
    FR-LIO already propagates process uncertainty into the source covariance;
    this gate must not encode health state by multiplying that covariance again.
    PX4 retains its own EV innovation/timeout gating, and the offboard node uses
    ``planner_usable`` to enter HOLD.
    """
    if not required:
        return None, False
    if not status_seen:
        return "frlio_status_missing", True
    if status_message_age_s < 0.0 or status_message_age_s > message_timeout_s:
        return f"frlio_status_timeout age={status_message_age_s:.3f}s", True
    if status in _FRLIO_STATE_UNUSABLE_STATUSES:
        return f"frlio_state_unusable status={status}", True

    # The predictor stream is what licenses continued EV publication in every
    # degraded case, so its freshness is validated before any of them.
    def _predictor_reason():
        if not math.isfinite(predictor_message_age_s):
            return "frlio_predictor_age_missing"
        if predictor_message_age_s < 0.0 or predictor_message_age_s > message_timeout_s:
            return f"frlio_predictor_age_timeout age={predictor_message_age_s:.3f}s"
        if (
            not math.isfinite(predictor_age_s)
            or predictor_age_s < 0.0
            or predictor_age_s > max_predictor_age_s
        ):
            return f"frlio_predictor_stale age={predictor_age_s:.3f}s"
        return None

    if status in _FRLIO_PLANNER_UNUSABLE_STATUSES:
        if not allow_stale_lidar_predictor_degraded:
            return "frlio_status_FAULT_STALE_LIDAR", True
        predictor_reason = _predictor_reason()
        if predictor_reason is not None:
            return predictor_reason, True
        return (
            "frlio_stale_lidar_predictor_fresh "
            f"predictor_age={predictor_age_s:.3f}s",
            False,
        )
    if not math.isfinite(anchor_message_age_s):
        return "frlio_anchor_age_missing", True
    if anchor_message_age_s < 0.0 or anchor_message_age_s > message_timeout_s:
        return f"frlio_anchor_age_timeout age={anchor_message_age_s:.3f}s", True
    if not math.isfinite(anchor_age_s) or anchor_age_s < 0.0:
        return "frlio_anchor_age_invalid", True
    if anchor_age_s >= max_anchor_age_s:
        # The anchor crossed the stale threshold before the status message
        # carrying that transition arrived.  Same physical situation as
        # FAULT_STALE_LIDAR, so it gets the same graded treatment rather than a
        # hard block: a message-ordering race must not create an EV dropout.
        if not allow_stale_lidar_predictor_degraded:
            return f"frlio_anchor_stale age={anchor_age_s:.3f}s", True
        predictor_reason = _predictor_reason()
        if predictor_reason is not None:
            return predictor_reason, True
        return (
            f"frlio_stale_lidar_predictor_fresh anchor_age={anchor_age_s:.3f}s "
            f"predictor_age={predictor_age_s:.3f}s",
            False,
        )
    if status in _FRLIO_DEGRADED_STATUSES:
        return f"frlio_anchor_suspect age={anchor_age_s:.3f}s", False
    if status != "HEALTHY":
        # Unrecognised status text still fails closed: we cannot map it to a
        # level, so we must not assume the state is usable.
        return f"frlio_status_{status or 'empty'}", True
    return None, False


def _is_frlio_predictor_degraded_reason(reason) -> bool:
    return bool(reason) and reason.startswith("frlio_stale_lidar_predictor_fresh")


def _is_internal_velocity_only_fault(reason: str) -> bool:
    """Whether a core fault is expected while LiDAR corrections are stale."""
    return reason == "internal_velocity_unaligned" or reason.startswith(
        "internal_velocity_mismatch "
    )


def _is_px4_velocity_bootstrap_reason(reason: str) -> bool:
    """Whether PX4's local-velocity check alone can block EKF2 bootstrap."""
    return reason == "px4_velocity_timeout" or reason == "px4_velocity_unaligned" or (
        reason.startswith("velocity_mismatch ")
    )


def _bootstrap_forward_allowed(
    *,
    accepted: bool,
    core_state,
    reason: str,
    mavros_armed,
    bootstrap_completed: bool,
    now_s: float,
    bootstrap_deadline_s: float,
) -> bool:
    """Permit EV only while a disarmed PX4 EKF2 establishes its local state.

    MAVROS local velocity is produced by the estimator that consumes this EV
    stream. Before that estimator has been seeded it can be missing or drift
    far from the stationary FAST-LIO derivative. Dropping every accepted EV
    sample at that point creates a bootstrap deadlock. Flight readiness remains
    fail-closed; this exception only keeps the input needed to initialize EKF2.
    """
    if (
        not accepted
        or mavros_armed is not False
        or bootstrap_completed
        or now_s > bootstrap_deadline_s
        or core_state == HealthState.HEALTHY
    ):
        return False
    return _is_px4_velocity_bootstrap_reason(reason) or reason.startswith(
        "recovering "
    )


def _copy_with_covariance_floors(
    msg: Odometry,
    *,
    min_position_variance: float,
    min_orientation_variance: float,
    linear_covariance_body,
) -> Odometry:
    """Copy an accepted measurement without encoding health in covariance."""
    output = copy.deepcopy(msg)
    for index in (0, 7, 14):
        output.pose.covariance[index] = max(
            float(output.pose.covariance[index]), min_position_variance
        )
    for index in (21, 28, 35):
        output.pose.covariance[index] = max(
            float(output.pose.covariance[index]), min_orientation_variance
        )
    for row in range(3):
        for column in range(3):
            output.twist.covariance[row * 6 + column] = linear_covariance_body[row][column]
    return output


def _evaluate_flight_ready(
    *,
    core_state,
    frlio_reason,
    frlio_block: bool,
    frlio_predictor_degraded: bool = False,
    now_s: float,
    last_healthy_output_s,
    healthy_output_timeout_s: float,
    dead_reckoning: bool = False,
    px4_local_position_age_s: float = 0.0,
    require_px4_local_position: bool = False,
    px4_local_position_timeout_s: float = 0.30,
    ev_fuse_age_s: float = math.inf,
    require_px4_ev_fusion: bool = False,
    px4_ev_fuse_timeout_s: float = 0.50,
):
    """Return ``(ready, reason)`` for the actual flight-safety gate.

    ``/ev_health/status`` and FR-LIO's high-rate status are raw diagnostic
    state machines.  In particular, the IMU/LiDAR boundary can produce a
    millisecond-scale SUSPECT pulse while the accepted EV stream remains
    continuous.  Do not make that pulse restart the pre-arm continuity timer.
    FAULT, an expired healthy-output stream, and the hard FR-LIO anchor gate
    still fail closed immediately.
    """
    if frlio_predictor_degraded:
        return False, "frlio_predictor_degraded"
    if require_px4_local_position and not math.isfinite(px4_local_position_age_s):
        return False, "px4_local_position_missing"
    if require_px4_local_position and px4_local_position_age_s > px4_local_position_timeout_s:
        return False, f"px4_local_position_stale age={px4_local_position_age_s:.3f}s"
    if dead_reckoning:
        return False, "px4_dead_reckoning"
    if require_px4_ev_fusion and (
        not math.isfinite(ev_fuse_age_s) or ev_fuse_age_s > px4_ev_fuse_timeout_s
    ):
        return False, f"px4_ev_last_fuse_stale age={ev_fuse_age_s:.3f}s"
    if core_state == HealthState.FAULT:
        return False, f"core_state_{core_state.value.lower()}"
    if frlio_block:
        return False, frlio_reason or "frlio_hard_block"
    if last_healthy_output_s is None:
        return False, "healthy_odom_output_missing"
    output_age_s = now_s - last_healthy_output_s
    if output_age_s < 0.0 or output_age_s > healthy_output_timeout_s:
        return (
            False,
            f"healthy_odom_output_stale age={output_age_s:.3f}s",
        )
    return True, "ready"


class FastlioEvHealthMonitor(Node):
    def __init__(self) -> None:
        super().__init__("fastlio_ev_health_monitor")

        # Subscribe before the legacy guard so this safety gate observes the very
        # jump/bad-timestamp frame that the guard may discard from its own output.
        self.input_topic = self.declare_parameter("input_topic", "/Odometry").value
        self.output_topic = self.declare_parameter("output_topic", "/Odometry/healthy").value
        self.px4_velocity_topic = self.declare_parameter(
            "px4_velocity_topic", "/mavros/local_position/velocity_local"
        ).value
        self.mavros_state_topic = self.declare_parameter(
            "mavros_state_topic", "/mavros/state"
        ).value
        self.px4_local_odom_topic = self.declare_parameter(
            "px4_local_odom_topic", "/mavros/local_position/odom"
        ).value
        self.use_local_odom_velocity_fallback = bool(
            self.declare_parameter("use_local_odom_velocity_fallback", False).value
        )
        self.status_topic = self.declare_parameter("status_topic", "/ev_health/status").value
        self.fault_topic = self.declare_parameter("fault_topic", "/ev_health/fault").value
        self.flight_ready_topic = self.declare_parameter(
            "flight_ready_topic", "/ev_health/flight_ready"
        ).value
        self.diagnostic_topic = self.declare_parameter(
            "diagnostic_topic", "/ev_health/diagnostics"
        ).value
        self.velocity_diagnostic_topic = self.declare_parameter(
            "velocity_diagnostic_topic", "/ev_health/velocity_ned"
        ).value
        self.effective_points_ok_topic = self.declare_parameter(
            "effective_points_ok_topic", ""
        ).value
        self.require_frlio_anchor_status = bool(
            self.declare_parameter("require_frlio_anchor_status", False).value
        )
        self.frlio_status_topic = self.declare_parameter(
            "frlio_status_topic", "/frlio/high_rate_odom/status"
        ).value
        self.frlio_anchor_age_topic = self.declare_parameter(
            "frlio_anchor_age_topic", "/frlio/high_rate_odom/anchor_age"
        ).value
        self.frlio_predictor_age_topic = self.declare_parameter(
            "frlio_predictor_age_topic", "/frlio/high_rate_odom/predictor_age"
        ).value
        # Keep aligned with fr_lio's high_rate_odom.max_predictor_age_s. This is
        # the threshold that decides whether EV is taken away, so the two layers
        # must not disagree about when the propagated state stops being usable.
        self.frlio_max_predictor_age_s = float(
            self.declare_parameter("frlio_max_predictor_age_s", 0.15).value
        )
        self.frlio_allow_stale_lidar_predictor_degraded = bool(
            self.declare_parameter(
                "frlio_allow_stale_lidar_predictor_degraded", True
            ).value
        )
        self.flight_ready_output_timeout_s = float(
            self.declare_parameter("flight_ready_output_timeout_s", 0.10).value
        )
        self.px4_bootstrap_max_s = float(
            self.declare_parameter("px4_bootstrap_max_s", 15.0).value
        )
        if self.flight_ready_output_timeout_s <= 0.0:
            raise ValueError("flight_ready_output_timeout_s must be positive")
        if self.frlio_max_predictor_age_s <= 0.0:
            raise ValueError("frlio_max_predictor_age_s must be positive")
        if self.px4_bootstrap_max_s <= 0.0:
            raise ValueError("px4_bootstrap_max_s must be positive")
        self.px4_local_position_topic = self.declare_parameter(
            "px4_local_position_topic", ""
        ).value
        self.require_px4_local_position = bool(
            self.declare_parameter("require_px4_local_position", True).value
        )
        self.px4_local_position_timeout_s = float(
            self.declare_parameter("px4_local_position_timeout_s", 0.30).value
        )
        self.px4_ev_aid_topic = self.declare_parameter(
            "px4_ev_aid_topic", ""
        ).value
        self.require_px4_ev_fusion = bool(
            self.declare_parameter("require_px4_ev_fusion", True).value
        )
        self.px4_ev_fuse_timeout_s = float(
            self.declare_parameter("px4_ev_fuse_timeout_s", 0.50).value
        )
        enable_relocalization = bool(
            self.declare_parameter("enable_relocalization", True).value
        )
        relocalization_stable_s = float(
            self.declare_parameter("relocalization_stable_s", 0.50).value
        )

        config = HealthConfig(
            position_yaw_offset_rad=float(
                self.declare_parameter(
                    "world_yaw_alignment_rad", 0.0
                ).value
            ),
            min_dt_s=float(self.declare_parameter("min_dt_s", 0.001).value),
            max_dt_s=float(self.declare_parameter("max_dt_s", 0.5).value),
            max_input_age_s=float(self.declare_parameter("max_input_age_s", 0.25).value),
            max_future_stamp_s=float(
                self.declare_parameter("max_future_stamp_s", 0.05).value
            ),
            max_position_jump_m=float(
                self.declare_parameter("max_position_jump_m", 0.15).value
            ),
            max_horizontal_velocity_difference_mps=float(
                self.declare_parameter(
                    "max_horizontal_velocity_difference_mps", 0.45
                ).value
            ),
            max_internal_velocity_difference_mps=float(
                self.declare_parameter(
                    "max_internal_velocity_difference_mps", 0.50
                ).value
            ),
            velocity_comparison_window_s=float(
                self.declare_parameter("velocity_comparison_window_s", 0.15).value
            ),
            velocity_difference_min_speed_mps=float(
                self.declare_parameter("velocity_difference_min_speed_mps", 0.05).value
            ),
            anomaly_to_fault_s=float(
                self.declare_parameter("anomaly_to_fault_s", 0.3).value
            ),
            recovery_healthy_s=float(
                self.declare_parameter("recovery_healthy_s", 2.0).value
            ),
            message_timeout_s=float(
                self.declare_parameter("message_timeout_s", 0.5).value
            ),
            px4_velocity_timeout_s=float(
                self.declare_parameter("px4_velocity_timeout_s", 0.5).value
            ),
            px4_velocity_max_input_age_s=float(
                self.declare_parameter(
                    "px4_velocity_max_input_age_s", 0.25
                ).value
            ),
            px4_velocity_max_future_stamp_s=float(
                self.declare_parameter(
                    "px4_velocity_max_future_stamp_s", 0.05
                ).value
            ),
            max_velocity_alignment_s=float(
                self.declare_parameter("max_velocity_alignment_s", 0.1).value
            ),
            max_internal_velocity_alignment_s=float(
                self.declare_parameter("max_internal_velocity_alignment_s", 0.1).value
            ),
            velocity_history_s=float(
                self.declare_parameter("velocity_history_s", 3.0).value
            ),
            internal_velocity_history_s=float(
                self.declare_parameter("internal_velocity_history_s", 3.0).value
            ),
            velocity_lowpass_cutoff_hz=float(
                self.declare_parameter("velocity_lowpass_cutoff_hz", 3.0).value
            ),
            stationary_speed_deadband_mps=float(
                self.declare_parameter("stationary_speed_deadband_mps", 0.02).value
            ),
            min_position_variance=float(
                self.declare_parameter("healthy_position_variance_floor_m2", 0.01).value
            ),
            min_orientation_variance=float(
                self.declare_parameter(
                    "healthy_orientation_variance_floor_rad2", 0.02
                ).value
            ),
            velocity_variance=float(
                self.declare_parameter("velocity_variance_m2ps2", 0.04).value
            ),
            velocity_variance_floor_x_m2ps2=float(
                self.declare_parameter("velocity_variance_floor_x_m2ps2", 0.0016).value
            ),
            velocity_variance_floor_y_m2ps2=float(
                self.declare_parameter("velocity_variance_floor_y_m2ps2", 0.0013).value
            ),
            velocity_variance_floor_z_m2ps2=float(
                self.declare_parameter("velocity_variance_floor_z_m2ps2", 0.0009).value
            ),
            require_effective_points_status=bool(self.effective_points_ok_topic),
            effective_points_timeout_s=float(
                self.declare_parameter("effective_points_timeout_s", 0.5).value
            ),
            require_frlio_anchor_status=self.require_frlio_anchor_status,
            frlio_anchor_status_timeout_s=float(
                self.declare_parameter("frlio_anchor_status_timeout_s", 0.5).value
            ),
            frlio_max_anchor_age_s=float(
                self.declare_parameter("frlio_max_anchor_age_s", 0.40).value
            ),
            enable_relocalization=enable_relocalization,
            relocalization_stable_s=relocalization_stable_s,
        )
        if config.frlio_anchor_status_timeout_s <= 0.0:
            raise ValueError("frlio_anchor_status_timeout_s must be positive")
        if config.frlio_max_anchor_age_s <= 0.0:
            raise ValueError("frlio_max_anchor_age_s must be positive")
        self.core = EvHealthMonitorCore(config)
        self._last_velocity_topic_receive_s = -math.inf
        self._last_output_covariance = (math.nan, math.nan, math.nan, math.nan)
        self._last_internal_velocity_body = (math.nan, math.nan, math.nan)
        self._last_internal_velocity_world = (math.nan, math.nan, math.nan)
        self._last_internal_velocity_ned = (math.nan, math.nan, math.nan)
        self._last_internal_velocity_covariance_body = (
            (math.nan, math.nan, math.nan),
            (math.nan, math.nan, math.nan),
            (math.nan, math.nan, math.nan),
        )
        self._last_applied_covariance_multiplier = math.nan
        self._last_logged_health_key = None
        self._frlio_status = None
        self._frlio_status_receive_s = None
        self._frlio_anchor_age_s = math.nan
        self._frlio_anchor_age_receive_s = None
        self._frlio_predictor_age_s = math.nan
        self._frlio_predictor_age_receive_s = None
        self._frlio_gate_reason = None
        self._frlio_gate_block = False
        self._frlio_predictor_degraded = False
        self._last_healthy_output_publish_s = None
        self._flight_ready = False
        self._flight_ready_reason = "initializing"
        self._last_logged_flight_ready = None
        self._mavros_armed = None
        self._px4_dead_reckoning = None
        self._px4_local_position_receive_s = None
        self._px4_z_reset_counter = None
        self._px4_ev_fuse_age_s = math.inf
        self._px4_ev_aid_receive_s = None
        self._ev_reset_counter = 0
        self._last_published_ev_reset_counter = None
        self._bootstrap_completed = False
        self._bootstrap_deadline_s = self._now_seconds() + self.px4_bootstrap_max_s

        odom_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        status_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.healthy_publisher = self.create_publisher(
            Odometry, self.output_topic, odom_qos
        )
        self.status_publisher = self.create_publisher(
            String, self.status_topic, status_qos
        )
        self.fault_publisher = self.create_publisher(Bool, self.fault_topic, status_qos)
        self.flight_ready_publisher = self.create_publisher(
            Bool, self.flight_ready_topic, status_qos
        )
        self.reset_counter_publisher = self.create_publisher(
            UInt8, "/ev_health/reset_counter", status_qos
        )
        self.diagnostic_publisher = self.create_publisher(
            DiagnosticArray, self.diagnostic_topic, 10
        )
        self.velocity_publisher = self.create_publisher(
            TwistWithCovarianceStamped, self.velocity_diagnostic_topic, 10
        )
        self.ev_subscription = self.create_subscription(
            Odometry, self.input_topic, self._ev_callback, odom_qos
        )
        self.velocity_subscription = self.create_subscription(
            TwistStamped,
            self.px4_velocity_topic,
            self._px4_velocity_callback,
            odom_qos,
        )
        self.mavros_state_subscription = self.create_subscription(
            State,
            self.mavros_state_topic,
            self._mavros_state_callback,
            odom_qos,
        )
        # VehicleLocalPosition and EstimatorAidSource2d are PX4 uORB/DDS
        # interfaces. MAVLink deployments intentionally have no /fmu graph;
        # only create these subscriptions when the corresponding safety gates
        # are explicitly enabled and a topic was configured.
        self.px4_local_position_subscription = None
        if self.require_px4_local_position and self.px4_local_position_topic:
            self.px4_local_position_subscription = self.create_subscription(
                VehicleLocalPosition,
                self.px4_local_position_topic,
                self._px4_local_position_callback,
                odom_qos,
            )
        self.px4_ev_aid_subscription = None
        if self.require_px4_ev_fusion and self.px4_ev_aid_topic:
            self.px4_ev_aid_subscription = self.create_subscription(
                EstimatorAidSource2d,
                self.px4_ev_aid_topic,
                self._px4_ev_aid_callback,
                odom_qos,
            )
        self.local_odom_subscription = None
        if self.use_local_odom_velocity_fallback:
            self.local_odom_subscription = self.create_subscription(
                Odometry,
                self.px4_local_odom_topic,
                self._px4_local_odom_callback,
                odom_qos,
            )
        self.effective_points_subscription = None
        if self.effective_points_ok_topic:
            self.effective_points_subscription = self.create_subscription(
                Bool,
                self.effective_points_ok_topic,
                lambda msg: self.core.update_effective_points(
                    msg.data, self._now_seconds()
                ),
                10,
            )
        self.frlio_status_subscription = None
        self.frlio_anchor_age_subscription = None
        self.frlio_predictor_age_subscription = None
        if self.require_frlio_anchor_status:
            self.frlio_status_subscription = self.create_subscription(
                String,
                self.frlio_status_topic,
                self._frlio_status_callback,
                status_qos,
            )
            self.frlio_anchor_age_subscription = self.create_subscription(
                Float64,
                self.frlio_anchor_age_topic,
                self._frlio_anchor_age_callback,
                odom_qos,
            )
            self.frlio_predictor_age_subscription = self.create_subscription(
                Float64,
                self.frlio_predictor_age_topic,
                self._frlio_predictor_age_callback,
                odom_qos,
            )

        self.timer = self.create_timer(0.1, self._timer_callback)
        self.get_logger().info(
            "EV health gate %s -> %s; compare %s in PX4 local NED; "
            "status=%s, diagnostics=%s"
            % (
                self.input_topic,
                self.output_topic,
                self.px4_velocity_topic,
                self.status_topic,
                self.diagnostic_topic,
            )
        )

    def _now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _frlio_status_callback(self, msg: String) -> None:
        self._frlio_status = str(msg.data).strip()
        self._frlio_status_receive_s = self._now_seconds()

    def _frlio_anchor_age_callback(self, msg: Float64) -> None:
        self._frlio_anchor_age_s = float(msg.data)
        self._frlio_anchor_age_receive_s = self._now_seconds()

    def _frlio_predictor_age_callback(self, msg: Float64) -> None:
        self._frlio_predictor_age_s = float(msg.data)
        self._frlio_predictor_age_receive_s = self._now_seconds()

    def _frlio_anchor_gate(self, now_s: float):
        anchor_message_age_s = (
            math.inf
            if self._frlio_anchor_age_receive_s is None
            else now_s - self._frlio_anchor_age_receive_s
        )
        predictor_message_age_s = (
            math.inf
            if self._frlio_predictor_age_receive_s is None
            else now_s - self._frlio_predictor_age_receive_s
        )
        status_message_age_s = (
            math.inf
            if self._frlio_status_receive_s is None
            else now_s - self._frlio_status_receive_s
        )
        return _evaluate_frlio_anchor_gate(
            required=self.require_frlio_anchor_status,
            status=self._frlio_status,
            status_seen=self._frlio_status_receive_s is not None,
            status_message_age_s=status_message_age_s,
            anchor_age_s=self._frlio_anchor_age_s,
            anchor_message_age_s=anchor_message_age_s,
            message_timeout_s=self.core.config.frlio_anchor_status_timeout_s,
            max_anchor_age_s=self.core.config.frlio_max_anchor_age_s,
            predictor_age_s=self._frlio_predictor_age_s,
            predictor_message_age_s=predictor_message_age_s,
            max_predictor_age_s=self.frlio_max_predictor_age_s,
            allow_stale_lidar_predictor_degraded=(
                self.frlio_allow_stale_lidar_predictor_degraded
            ),
        )

    def _px4_velocity_callback(self, msg: TwistStamped) -> None:
        now_s = self._now_seconds()
        sample_stamp_s = _stamp_seconds(msg.header.stamp)
        linear = msg.twist.linear
        if self.core.update_px4_velocity(
            (linear.x, linear.y, linear.z),
            now_s,
            input_is_enu=True,
            sample_stamp_s=sample_stamp_s,
        ):
            self._last_velocity_topic_receive_s = now_s

    def _mavros_state_callback(self, msg: State) -> None:
        self._mavros_armed = bool(msg.armed)

    def _px4_local_position_callback(self, msg: VehicleLocalPosition) -> None:
        self._px4_local_position_receive_s = self._now_seconds()
        self._px4_dead_reckoning = bool(msg.dead_reckoning)
        self._px4_z_reset_counter = int(msg.z_reset_counter)

    def _px4_ev_aid_callback(self, msg: EstimatorAidSource2d) -> None:
        # PX4 timestamps are both in boot-time microseconds, so the difference
        # is independent of the ROS/PX4 wall-clock offset.
        now_s = self._now_seconds()
        self._px4_ev_aid_receive_s = now_s
        if msg.time_last_fuse == 0 or msg.timestamp < msg.time_last_fuse:
            self._px4_ev_fuse_age_s = math.inf
            return
        self._px4_ev_fuse_age_s = (msg.timestamp - msg.time_last_fuse) * 1e-6

    def _px4_local_odom_callback(self, msg: Odometry) -> None:
        # nav_msgs/Odometry twist is normally expressed in child_frame_id.  This
        # fallback must therefore only be enabled when the installation has
        # verified that MAVROS publishes this particular topic in local ENU.
        now_s = self._now_seconds()
        if now_s - self._last_velocity_topic_receive_s <= self.core.config.px4_velocity_timeout_s:
            return
        linear = msg.twist.twist.linear
        sample_stamp_s = _stamp_seconds(msg.header.stamp)
        self.core.update_px4_velocity(
            (linear.x, linear.y, linear.z),
            now_s,
            input_is_enu=True,
            sample_stamp_s=sample_stamp_s,
        )

    @staticmethod
    def _linear_velocity_covariance(msg: Odometry):
        return tuple(
            tuple(float(msg.twist.covariance[row * 6 + column]) for column in range(3))
            for row in range(3)
        )

    @classmethod
    def _message_is_finite(cls, msg: Odometry) -> bool:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        v = msg.twist.twist.linear
        values: List[float] = [
            p.x,
            p.y,
            p.z,
            q.x,
            q.y,
            q.z,
            q.w,
            v.x,
            v.y,
            v.z,
            *msg.pose.covariance,
        ]
        # Angular velocity may legitimately be NaN (unknown in MAVLink).  Only
        # require the measured linear block and its covariance to be valid.
        linear_covariance = cls._linear_velocity_covariance(msg)
        return all(math.isfinite(value) for value in values) and (
            covariance_3x3_is_symmetric_psd(linear_covariance)
        )

    def _ev_callback(self, msg: Odometry) -> None:
        now_s = self._now_seconds()
        position = msg.pose.pose.position
        quaternion = msg.pose.pose.orientation
        velocity_body = msg.twist.twist.linear
        try:
            body_to_world = quaternion_rotation_matrix_xyzw(
                (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
            )
            internal_velocity_body = (
                float(velocity_body.x),
                float(velocity_body.y),
                float(velocity_body.z),
            )
            internal_velocity_world = rotate_vector(body_to_world, internal_velocity_body)
            internal_velocity_ned = ev_enu_to_px4_ned(
                internal_velocity_world,
                self.core.config.position_yaw_offset_rad,
            )
            linear_covariance_body = apply_world_velocity_covariance_floors(
                body_to_world,
                self._linear_velocity_covariance(msg),
                (
                    self.core.config.velocity_variance_floor_x_m2ps2,
                    self.core.config.velocity_variance_floor_y_m2ps2,
                    self.core.config.velocity_variance_floor_z_m2ps2,
                ),
            )
        except (TypeError, ValueError):
            internal_velocity_body = (math.nan, math.nan, math.nan)
            internal_velocity_world = (math.nan, math.nan, math.nan)
            internal_velocity_ned = (math.nan, math.nan, math.nan)
            linear_covariance_body = (
                (math.nan, math.nan, math.nan),
                (math.nan, math.nan, math.nan),
                (math.nan, math.nan, math.nan),
            )
        self._last_internal_velocity_body = internal_velocity_body
        self._last_internal_velocity_world = internal_velocity_world
        self._last_internal_velocity_ned = internal_velocity_ned
        self._last_internal_velocity_covariance_body = linear_covariance_body
        result = self.core.process_ev(
            stamp_s=_stamp_seconds(msg.header.stamp),
            receive_time_s=now_s,
            raw_position_enu=(position.x, position.y, position.z),
            internal_velocity_px4_ned=internal_velocity_ned,
            covariance_xyz=(
                msg.pose.covariance[0],
                msg.pose.covariance[7],
                msg.pose.covariance[14],
            ),
            finite_payload=self._message_is_finite(msg),
        )
        self._ev_reset_counter = int(result.reset_counter)
        frlio_reason, frlio_block = self._frlio_anchor_gate(now_s)
        self._frlio_gate_reason = frlio_reason
        self._frlio_gate_block = frlio_block
        self._frlio_predictor_degraded = _is_frlio_predictor_degraded_reason(
            frlio_reason
        )
        if frlio_reason is not None and frlio_block:
            result = self.core.report_external_anomaly(
                now_s, frlio_reason, accepted=result.accepted
            )
        # A fresh predictor can continue EV while LiDAR corrections are stale.
        # FR-LIO's source covariance already includes process-noise propagation
        # and stale-anchor growth, so applying health-state inflation here would
        # double count uncertainty and make PX4's EV weight oscillate.
        predictor_degraded_fault = (
            self._frlio_predictor_degraded
            and result.accepted
            and _is_internal_velocity_only_fault(result.reason)
        )

        bootstrap_forward = _bootstrap_forward_allowed(
            accepted=result.accepted,
            core_state=result.state,
            reason=result.reason,
            mavros_armed=self._mavros_armed,
            bootstrap_completed=self._bootstrap_completed,
            now_s=now_s,
            bootstrap_deadline_s=self._bootstrap_deadline_s,
        )
        if (result.publish or predictor_degraded_fault or bootstrap_forward) and not frlio_block:
            output = _copy_with_covariance_floors(
                msg,
                min_position_variance=self.core.config.min_position_variance,
                min_orientation_variance=self.core.config.min_orientation_variance,
                linear_covariance_body=linear_covariance_body,
            )
            self._last_output_covariance = (
                output.pose.covariance[0],
                output.pose.covariance[7],
                output.pose.covariance[14],
                output.pose.covariance[35],
            )
            self._last_applied_covariance_multiplier = 1.0
            # Preserve the original measurement stamp.  Downstream must not turn
            # an old measurement into an apparently current one.
            if result.relocalized:
                # Publish the counter before the first pose in the replacement
                # frame. The native PX4 bridge blocks one sample on this edge,
                # so it cannot send an old-frame pose with the new counter.
                self.reset_counter_publisher.publish(
                    UInt8(data=int(result.reset_counter))
                )
                self.get_logger().warning(
                    "[EV_RELOCALIZED] reset_counter=%d; committed new source frame atomically",
                    int(result.reset_counter),
                )
            self.healthy_publisher.publish(output)
            self._last_healthy_output_publish_s = now_s

        if result.state == HealthState.HEALTHY:
            self._bootstrap_completed = True

        if result.accepted and all(
            math.isfinite(value)
            for value in result.metrics.raw_velocity_px4_ned
        ):
            self._publish_velocity(msg)
        self._publish_status_and_diagnostics()

    def _publish_velocity(self, source: Odometry) -> None:
        velocity = TwistWithCovarianceStamped()
        velocity.header = source.header
        velocity.header.frame_id = "px4_local_ned"
        values = self.core.metrics.velocity_px4_ned
        velocity.twist.twist.linear.x = values[0]
        velocity.twist.twist.linear.y = values[1]
        velocity.twist.twist.linear.z = values[2]
        for index in (0, 7, 14):
            velocity.twist.covariance[index] = self.core.config.velocity_variance
        self.velocity_publisher.publish(velocity)

    def _timer_callback(self) -> None:
        now_s = self._now_seconds()
        self.core.check_timeout(now_s)
        frlio_reason, frlio_block = self._frlio_anchor_gate(now_s)
        self._frlio_gate_reason = frlio_reason
        self._frlio_gate_block = frlio_block
        self._frlio_predictor_degraded = _is_frlio_predictor_degraded_reason(
            frlio_reason
        )
        if frlio_reason is not None and frlio_block:
            self.core.report_external_anomaly(now_s, frlio_reason)
        self._publish_status_and_diagnostics()

    def _publish_status_and_diagnostics(self) -> None:
        # A stale LiDAR posterior with a fresh IMU predictor is explicitly
        # degraded. Do not expose its velocity-only core FAULT as a normal EV
        # hard fault: that would request AUTO.LAND even though FR-LIO and PX4
        # are receiving a continuous prediction with propagated covariance.
        predictor_degraded_fault = (
            self._frlio_predictor_degraded
            and self.core.state == HealthState.FAULT
            and _is_internal_velocity_only_fault(self.core.reason)
        )
        exposed_predictor_degraded = (
            self._frlio_predictor_degraded
            and (self.core.state != HealthState.FAULT or predictor_degraded_fault)
        )
        state = HealthState.SUSPECT if exposed_predictor_degraded else self.core.state
        reason = (
            f"{self._frlio_gate_reason}; {self.core.reason}"
            if exposed_predictor_degraded
            else self.core.reason
        )
        self.status_publisher.publish(String(data=state.value))
        self.fault_publisher.publish(Bool(data=state == HealthState.FAULT))
        now_s = self._now_seconds()
        ev_fuse_age_s = self._px4_ev_fuse_age_s
        if self._px4_ev_aid_receive_s is None:
            ev_fuse_age_s = math.inf
        elif math.isfinite(ev_fuse_age_s):
            ev_fuse_age_s += max(0.0, now_s - self._px4_ev_aid_receive_s)
        (
            self._flight_ready,
            self._flight_ready_reason,
        ) = _evaluate_flight_ready(
            core_state=self.core.state,
            frlio_reason=self._frlio_gate_reason,
            frlio_block=self._frlio_gate_block,
            frlio_predictor_degraded=exposed_predictor_degraded,
            now_s=now_s,
            last_healthy_output_s=self._last_healthy_output_publish_s,
            healthy_output_timeout_s=self.flight_ready_output_timeout_s,
            dead_reckoning=bool(self._px4_dead_reckoning),
            px4_local_position_age_s=(
                math.inf
                if self._px4_local_position_receive_s is None
                else now_s - self._px4_local_position_receive_s
            ),
            require_px4_local_position=self.require_px4_local_position,
            px4_local_position_timeout_s=self.px4_local_position_timeout_s,
            ev_fuse_age_s=ev_fuse_age_s,
            require_px4_ev_fusion=self.require_px4_ev_fusion,
            px4_ev_fuse_timeout_s=self.px4_ev_fuse_timeout_s,
        )
        self.flight_ready_publisher.publish(Bool(data=self._flight_ready))
        # Transient-local publication lets a restarted native PX4 bridge learn
        # the current source-frame counter before its first EV sample.
        if self._last_published_ev_reset_counter != self._ev_reset_counter:
            self.reset_counter_publisher.publish(UInt8(data=self._ev_reset_counter))
            self._last_published_ev_reset_counter = self._ev_reset_counter

        diagnostic = DiagnosticArray()
        diagnostic.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = "fastlio_ev_health"
        status.hardware_id = "fastlio_px4_local"
        status.level = {
            HealthState.HEALTHY: DiagnosticStatus.OK,
            HealthState.SUSPECT: DiagnosticStatus.WARN,
            HealthState.FAULT: DiagnosticStatus.ERROR,
        }[state]
        status.message = reason
        metrics = self.core.metrics
        status.values = [
            KeyValue(key="state", value=state.value),
            KeyValue(key="reason", value=reason),
            KeyValue(key="flight_ready", value=str(self._flight_ready).lower()),
            KeyValue(key="flight_ready_reason", value=self._flight_ready_reason),
            KeyValue(
                key="px4_dead_reckoning",
                value=str(bool(self._px4_dead_reckoning)).lower(),
            ),
            KeyValue(
                key="px4_z_reset_counter",
                value=("unknown" if self._px4_z_reset_counter is None else str(self._px4_z_reset_counter)),
            ),
            KeyValue(key="ev_reset_counter", value=str(self._ev_reset_counter)),
            KeyValue(
                key="px4_local_position_age_s",
                value=(
                    "inf"
                    if self._px4_local_position_receive_s is None
                    else f"{now_s - self._px4_local_position_receive_s:.6f}"
                ),
            ),
            KeyValue(key="px4_ev_last_fuse_age_s", value=f"{ev_fuse_age_s:.6f}"),
            KeyValue(key="px4_ev_fusion_required", value=str(self.require_px4_ev_fusion).lower()),
            KeyValue(
                key="frlio_anchor_gate_required",
                value=str(self.require_frlio_anchor_status).lower(),
            ),
            KeyValue(key="frlio_status", value=str(self._frlio_status)),
            KeyValue(
                key="frlio_gate_reason",
                value=str(self._frlio_gate_reason),
            ),
            KeyValue(
                key="frlio_gate_block",
                value=str(self._frlio_gate_block).lower(),
            ),
            KeyValue(
                key="frlio_anchor_age_s",
                value=(
                    "nan"
                    if not math.isfinite(self._frlio_anchor_age_s)
                    else f"{self._frlio_anchor_age_s:.6f}"
                ),
            ),
            KeyValue(
                key="frlio_predictor_age_s",
                value=(
                    "nan"
                    if not math.isfinite(self._frlio_predictor_age_s)
                    else f"{self._frlio_predictor_age_s:.6f}"
                ),
            ),
            KeyValue(
                key="frlio_predictor_degraded",
                value=str(exposed_predictor_degraded).lower(),
            ),
            KeyValue(key="comparison_frame", value="PX4 local NED"),
            KeyValue(key="ev_raw_position_enu_m", value=_vector_text(metrics.raw_position_enu)),
            KeyValue(key="ev_position_px4_ned_m", value=_vector_text(metrics.position_px4_ned)),
            KeyValue(
                key="ev_raw_differenced_velocity_px4_ned_mps",
                value=_vector_text(metrics.raw_velocity_px4_ned),
            ),
            KeyValue(
                key="ev_filtered_velocity_px4_ned_mps",
                value=_vector_text(metrics.velocity_px4_ned),
            ),
            KeyValue(
                key="ev_internal_velocity_body_flu_mps",
                value=_vector_text(self._last_internal_velocity_body),
            ),
            KeyValue(
                key="ev_internal_velocity_world_enu_mps",
                value=_vector_text(self._last_internal_velocity_world),
            ),
            KeyValue(
                key="ev_internal_velocity_px4_ned_mps",
                value=_vector_text(self._last_internal_velocity_ned),
            ),
            KeyValue(
                key="ev_internal_velocity_covariance_body",
                value=_vector_text(
                    value
                    for row in self._last_internal_velocity_covariance_body
                    for value in row
                ),
            ),
            KeyValue(
                key="px4_velocity_px4_ned_mps",
                value=_vector_text(metrics.px4_velocity_px4_ned),
            ),
            KeyValue(
                key="horizontal_velocity_difference_mps",
                value=f"{metrics.horizontal_velocity_difference_mps:.6f}",
            ),
            KeyValue(
                key="internal_vs_position_velocity_difference_mps",
                value=f"{metrics.internal_velocity_difference_mps:.6f}",
            ),
            KeyValue(
                key="internal_vertical_vs_position_velocity_difference_mps",
                value=f"{metrics.internal_vertical_velocity_difference_mps:.6f}",
            ),
            KeyValue(
                key="single_frame_displacement_m",
                value=f"{metrics.single_frame_displacement_m:.6f}",
            ),
            KeyValue(key="measurement_dt_s", value=f"{metrics.dt_s:.6f}"),
            KeyValue(key="ev_sample_gap_s", value=f"{metrics.sample_gap_s:.6f}"),
            KeyValue(
                key="velocity_comparison_dt_s",
                value=f"{metrics.velocity_comparison_dt_s:.6f}",
            ),
            KeyValue(key="input_age_s", value=f"{metrics.input_age_s:.6f}"),
            KeyValue(key="ev_sample_age_s", value=f"{metrics.sample_age_s:.6f}"),
            KeyValue(
                key="velocity_alignment_s",
                value=f"{metrics.velocity_alignment_s:.6f}",
            ),
            KeyValue(
                key="internal_velocity_alignment_s",
                value=f"{metrics.internal_velocity_alignment_s:.6f}",
            ),
            KeyValue(key="source_covariance_xyz_m2", value=_vector_text(metrics.covariance_xyz)),
            KeyValue(
                key="last_published_covariance_xyz_yaw",
                value=_vector_text(self._last_output_covariance),
            ),
            KeyValue(
                key="applied_covariance_multiplier",
                value=f"{self._last_applied_covariance_multiplier:.6f}",
            ),
        ]
        diagnostic.status = [status]
        self.diagnostic_publisher.publish(diagnostic)

        # Do not turn the raw ~10 ms FR-LIO warning sawtooth into a logging
        # feedback load. The full raw FR-LIO state remains in diagnostics;
        # log only core transitions or entry/exit of a hard FR-LIO block.
        health_log_key = (state, self._frlio_gate_block, exposed_predictor_degraded)
        if health_log_key != self._last_logged_health_key:
            message = f"EV health state={state.value}: {reason}"
            if state == HealthState.HEALTHY:
                self.get_logger().info(message)
            else:
                self.get_logger().warning(message)
            self._last_logged_health_key = health_log_key
        if self._flight_ready != self._last_logged_flight_ready:
            message = (
                f"EV flight_ready={str(self._flight_ready).lower()}: "
                f"{self._flight_ready_reason}"
            )
            if self._flight_ready:
                self.get_logger().info(message)
            else:
                self.get_logger().warning(message)
            self._last_logged_flight_ready = self._flight_ready


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FastlioEvHealthMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
