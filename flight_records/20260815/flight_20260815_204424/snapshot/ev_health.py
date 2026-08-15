"""
Pure EV health monitoring logic shared by the ROS node and unit tests.

FAST-LIO and MAVROS expose ROS ENU coordinates while PX4's estimator uses NED.
This module performs every comparison in PX4 local NED:

    (north, east, down) = (y_enu, x_enu, -z_enu)

``position_yaw_offset_rad`` is applied in ENU before the ENU -> NED change.
Keeping this logic ROS-message independent makes frame/sign and state-machine
behaviour straightforward to test without a running flight stack.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from enum import Enum
import math
from typing import Iterable, Optional, Sequence, Tuple


Vector3 = Tuple[float, float, float]
Matrix3 = Tuple[Tuple[float, float, float], ...]


class HealthState(str, Enum):
    HEALTHY = "HEALTHY"
    SUSPECT = "SUSPECT"
    FAULT = "FAULT"


@dataclass
class HealthConfig:
    position_yaw_offset_rad: float = 0.0
    min_dt_s: float = 0.001
    max_dt_s: float = 0.5
    max_input_age_s: float = 0.25
    max_future_stamp_s: float = 0.05
    max_position_jump_m: float = 0.15
    max_horizontal_velocity_difference_mps: float = 0.45
    max_internal_velocity_difference_mps: float = 0.50
    velocity_comparison_window_s: float = 0.15
    velocity_difference_min_speed_mps: float = 0.05
    anomaly_to_fault_s: float = 0.3
    recovery_healthy_s: float = 2.0
    message_timeout_s: float = 0.5
    px4_velocity_timeout_s: float = 0.5
    px4_velocity_max_input_age_s: float = 0.25
    px4_velocity_max_future_stamp_s: float = 0.05
    max_velocity_alignment_s: float = 0.1
    max_internal_velocity_alignment_s: float = 0.1
    velocity_history_s: float = 3.0
    internal_velocity_history_s: float = 3.0
    velocity_lowpass_cutoff_hz: float = 3.0
    stationary_speed_deadband_mps: float = 0.02
    suspect_covariance_multiplier: float = 100.0
    min_position_variance: float = 0.01
    min_orientation_variance: float = 0.02
    velocity_variance: float = 0.04
    # Floors are expressed in the world ENU frame and come from prop-off,
    # independently referenced velocity calibration.
    velocity_variance_floor_x_m2ps2: float = 0.0016
    velocity_variance_floor_y_m2ps2: float = 0.0013
    velocity_variance_floor_z_m2ps2: float = 0.0009
    require_effective_points_status: bool = False
    effective_points_timeout_s: float = 0.5
    require_frlio_anchor_status: bool = False
    frlio_anchor_status_timeout_s: float = 0.5
    frlio_max_anchor_age_s: float = 0.40


@dataclass
class HealthMetrics:
    raw_position_enu: Vector3 = (math.nan, math.nan, math.nan)
    position_px4_ned: Vector3 = (math.nan, math.nan, math.nan)
    raw_velocity_px4_ned: Vector3 = (math.nan, math.nan, math.nan)
    velocity_px4_ned: Vector3 = (0.0, 0.0, 0.0)
    px4_velocity_px4_ned: Vector3 = (math.nan, math.nan, math.nan)
    horizontal_velocity_difference_mps: float = math.nan
    internal_velocity_difference_mps: float = math.nan
    internal_vertical_velocity_difference_mps: float = math.nan
    single_frame_displacement_m: float = math.nan
    dt_s: float = math.nan
    velocity_comparison_dt_s: float = math.nan
    input_age_s: float = math.nan
    velocity_alignment_s: float = math.nan
    internal_velocity_alignment_s: float = math.nan
    covariance_xyz: Vector3 = (math.nan, math.nan, math.nan)


@dataclass
class FrameResult:
    accepted: bool
    publish: bool
    state: HealthState
    reason: str
    covariance_multiplier: float
    velocity_px4_ned: Vector3
    metrics: HealthMetrics


def _finite(values: Iterable[float]) -> bool:
    return all(math.isfinite(float(value)) for value in values)


def rotate_enu_xy(vector: Sequence[float], yaw_rad: float) -> Vector3:
    """Rotate an ENU vector around +Z by ``yaw_rad``."""
    c = math.cos(yaw_rad)
    s = math.sin(yaw_rad)
    x, y, z = (float(vector[0]), float(vector[1]), float(vector[2]))
    return (c * x - s * y, s * x + c * y, z)


def enu_to_ned(vector: Sequence[float]) -> Vector3:
    """Convert a world-frame ROS ENU vector to PX4 local NED."""
    return (float(vector[1]), float(vector[0]), -float(vector[2]))


def ned_to_enu(vector: Sequence[float]) -> Vector3:
    """Convert a PX4 local NED vector to ROS ENU."""
    return (float(vector[1]), float(vector[0]), -float(vector[2]))


def inverse_rotate_enu_xy(vector: Sequence[float], yaw_rad: float) -> Vector3:
    return rotate_enu_xy(vector, -yaw_rad)


def ev_enu_to_px4_ned(vector: Sequence[float], yaw_offset_rad: float) -> Vector3:
    """Apply the bridge yaw alignment, then express the result in PX4 NED."""
    return enu_to_ned(rotate_enu_xy(vector, yaw_offset_rad))


def px4_ned_velocity_to_ev_enu(vector: Sequence[float], yaw_offset_rad: float) -> Vector3:
    """Inverse of :func:`ev_enu_to_px4_ned`, used for outgoing Odometry.twist."""
    return inverse_rotate_enu_xy(ned_to_enu(vector), yaw_offset_rad)


def quaternion_rotation_matrix_xyzw(quaternion: Sequence[float]) -> Matrix3:
    """Return the child-to-parent rotation for a ROS ``(x,y,z,w)`` quaternion."""
    x, y, z, w = (float(value) for value in quaternion)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm <= 1e-12:
        raise ValueError("invalid quaternion")
    x, y, z, w = (value / norm for value in (x, y, z, w))
    return (
        (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)),
        (2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)),
        (2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)),
    )


def rotate_vector(matrix: Matrix3, vector: Sequence[float]) -> Vector3:
    values = tuple(float(value) for value in vector)
    return tuple(
        sum(matrix[row][column] * values[column] for column in range(3))
        for row in range(3)
    )  # type: ignore[return-value]


def rotate_covariance_3x3(matrix: Matrix3, covariance: Matrix3) -> Matrix3:
    """Compute ``R P R^T`` without adding a runtime NumPy dependency."""
    return tuple(
        tuple(
            sum(
                matrix[row][left]
                * covariance[left][right]
                * matrix[column][right]
                for left in range(3)
                for right in range(3)
            )
            for column in range(3)
        )
        for row in range(3)
    )


def transpose_matrix3(matrix: Matrix3) -> Matrix3:
    """Return the transpose of a 3x3 matrix."""
    return tuple(
        tuple(matrix[column][row] for column in range(3)) for row in range(3)
    )


def apply_world_velocity_covariance_floors(
    child_to_world: Matrix3,
    covariance_child: Matrix3,
    floors_world_enu: Sequence[float],
) -> Matrix3:
    """Apply diagonal variance floors in world ENU and return child-frame data.

    Adding only the missing diagonal variance preserves existing cross terms and
    keeps the matrix PSD.  The final inverse rotation preserves the ROS
    Odometry contract: its twist covariance remains expressed in the child
    frame, while the MAVROS bridge rotates it to world ENU later.
    """
    floors = tuple(float(value) for value in floors_world_enu)
    if len(floors) != 3 or not _finite(floors) or any(value < 0.0 for value in floors):
        raise ValueError("velocity covariance floors must be three non-negative finite values")

    world = [list(row) for row in rotate_covariance_3x3(child_to_world, covariance_child)]
    for axis, floor in enumerate(floors):
        world[axis][axis] += max(0.0, floor - world[axis][axis])

    floored_world = tuple(
        tuple(0.5 * (world[row][column] + world[column][row]) for column in range(3))
        for row in range(3)
    )
    child = rotate_covariance_3x3(
        transpose_matrix3(child_to_world), floored_world
    )
    return tuple(
        tuple(0.5 * (child[row][column] + child[column][row]) for column in range(3))
        for row in range(3)
    )


def covariance_3x3_is_symmetric_psd(covariance: Matrix3, tolerance: float = 1e-9) -> bool:
    """Finite symmetric positive-semidefinite check for a 3x3 covariance."""
    values = tuple(value for row in covariance for value in row)
    if not _finite(values):
        return False
    for row in range(3):
        if covariance[row][row] <= 0.0:
            return False
        for column in range(row + 1, 3):
            if abs(covariance[row][column] - covariance[column][row]) > tolerance:
                return False
    for first, second in ((0, 1), (0, 2), (1, 2)):
        minor = (
            covariance[first][first] * covariance[second][second]
            - covariance[first][second] * covariance[second][first]
        )
        if minor < -tolerance:
            return False
    determinant = (
        covariance[0][0]
        * (covariance[1][1] * covariance[2][2] - covariance[1][2] * covariance[2][1])
        - covariance[0][1]
        * (covariance[1][0] * covariance[2][2] - covariance[1][2] * covariance[2][0])
        + covariance[0][2]
        * (covariance[1][0] * covariance[2][1] - covariance[1][1] * covariance[2][0])
    )
    return determinant >= -tolerance


def compensate_sensor_to_body_lever_arm(
    velocity_sensor: Sequence[float],
    angular_velocity_sensor: Sequence[float],
    sensor_to_body_translation: Sequence[float],
) -> Vector3:
    """Shift velocity from sensor origin to body origin using ``v + omega x r``."""
    velocity = tuple(float(value) for value in velocity_sensor)
    omega = tuple(float(value) for value in angular_velocity_sensor)
    translation = tuple(float(value) for value in sensor_to_body_translation)
    if not _finite((*velocity, *omega, *translation)):
        raise ValueError("lever-arm compensation requires finite inputs")
    return (
        velocity[0] + omega[1] * translation[2] - omega[2] * translation[1],
        velocity[1] + omega[2] * translation[0] - omega[0] * translation[2],
        velocity[2] + omega[0] * translation[1] - omega[1] * translation[0],
    )


def _subtract(a: Vector3, b: Vector3) -> Vector3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _norm(vector: Vector3) -> float:
    return math.sqrt(sum(value * value for value in vector))


def _horizontal_norm(vector: Vector3) -> float:
    return math.hypot(vector[0], vector[1])


class EvHealthMonitorCore:
    """
    Timestamp-aware EV/PX4 consistency monitor.

    Invalid frames are never accepted or published.  A discontinuous position
    never becomes the new derivative baseline: recovery requires data to return
    close to the last trusted pose.  FAULT never emits a position sample;
    notably, this class has no mechanism that can repeat the last good position.
    """

    def __init__(self, config: Optional[HealthConfig] = None) -> None:
        self.config = config or HealthConfig()
        if self.config.velocity_comparison_window_s <= self.config.min_dt_s:
            raise ValueError(
                "velocity_comparison_window_s must be greater than min_dt_s"
            )
        self.state = HealthState.SUSPECT
        self.reason = "initializing"
        self.metrics = HealthMetrics()

        self._last_stamp_s: Optional[float] = None
        self._last_receive_s: Optional[float] = None
        self._last_position_ned: Optional[Vector3] = None
        self._position_history = deque()
        self._filtered_velocity_ned: Vector3 = (0.0, 0.0, 0.0)
        self._px4_velocity_ned: Optional[Vector3] = None
        self._px4_receive_s: Optional[float] = None
        self._px4_velocity_history = deque()
        self._last_px4_velocity_stamp_s: Optional[float] = None
        self._internal_velocity_history = deque()
        self._effective_points_ok: Optional[bool] = None
        self._effective_points_receive_s: Optional[float] = None
        self._anomaly_since_s: Optional[float] = None
        self._healthy_since_s: Optional[float] = None
        self._anomaly_start_covariance_fraction = 0.0
        # Startup is deliberately low-confidence until the recovery interval has
        # contained only aligned, healthy samples.
        self._recovery_start_covariance_fraction = 1.0

    def update_px4_velocity(
        self,
        velocity: Sequence[float],
        receive_time_s: float,
        *,
        input_is_enu: bool = True,
        sample_stamp_s: Optional[float] = None,
    ) -> bool:
        sample_stamp_s = receive_time_s if sample_stamp_s is None else sample_stamp_s
        if not math.isfinite(float(receive_time_s)):
            return False
        now_s = float(receive_time_s)
        if not _finite((*velocity, sample_stamp_s)):
            self._set_anomaly(now_s, "non_finite_px4_velocity")
            return False
        sample_stamp_s = float(sample_stamp_s)
        sample_age_s = now_s - sample_stamp_s
        if sample_stamp_s <= 0.0:
            self._set_anomaly(now_s, "invalid_px4_velocity_stamp")
            return False
        if sample_age_s > self.config.px4_velocity_max_input_age_s:
            self._set_anomaly(
                now_s,
                f"stale_px4_velocity age={sample_age_s:.3f}s",
            )
            return False
        if sample_age_s < -self.config.px4_velocity_max_future_stamp_s:
            self._set_anomaly(
                now_s,
                f"future_px4_velocity age={sample_age_s:.3f}s",
            )
            return False
        if (
            self._last_px4_velocity_stamp_s is not None
            and sample_stamp_s <= self._last_px4_velocity_stamp_s
        ):
            self._set_anomaly(
                now_s,
                "non_monotonic_px4_velocity_stamp "
                f"dt={sample_stamp_s - self._last_px4_velocity_stamp_s:.6f}s",
            )
            return False
        parsed = (float(velocity[0]), float(velocity[1]), float(velocity[2]))
        self._px4_velocity_ned = enu_to_ned(parsed) if input_is_enu else parsed
        self._px4_receive_s = now_s
        self._last_px4_velocity_stamp_s = sample_stamp_s
        self._px4_velocity_history.append(
            (sample_stamp_s, self._px4_velocity_ned, now_s)
        )
        newest_stamp = sample_stamp_s
        while (
            self._px4_velocity_history
            and newest_stamp - self._px4_velocity_history[0][0]
            > self.config.velocity_history_s
        ):
            self._px4_velocity_history.popleft()
        self.metrics.px4_velocity_px4_ned = self._px4_velocity_ned
        return True

    def update_effective_points(self, ok: bool, receive_time_s: float) -> None:
        self._effective_points_ok = bool(ok)
        self._effective_points_receive_s = float(receive_time_s)

    def report_external_anomaly(
        self, receive_time_s: float, reason: str, *, accepted: bool = False
    ) -> FrameResult:
        """Apply a node-owned safety gate to the common health state machine."""
        now_s = float(receive_time_s)
        self._set_anomaly(now_s, str(reason))
        return self._result(accepted, now_s)

    def _effective_points_reason(self, now_s: float) -> Optional[str]:
        if not self.config.require_effective_points_status:
            return None
        if self._effective_points_receive_s is None:
            return "effective_points_status_missing"
        age_s = now_s - self._effective_points_receive_s
        if age_s < 0.0 or age_s > self.config.effective_points_timeout_s:
            return f"effective_points_status_timeout age={age_s:.3f}s"
        if self._effective_points_ok is not True:
            return "no_effective_points"
        return None

    def _set_anomaly(self, now_s: float, reason: str) -> None:
        if self._anomaly_since_s is None:
            self._anomaly_start_covariance_fraction = self._covariance_fraction(
                now_s
            )
            self._anomaly_since_s = now_s
        self._healthy_since_s = None
        self.reason = reason
        if self.state == HealthState.FAULT:
            return
        if now_s - self._anomaly_since_s + 1e-9 >= self.config.anomaly_to_fault_s:
            self.state = HealthState.FAULT
        else:
            self.state = HealthState.SUSPECT

    def _set_healthy_sample(self, now_s: float) -> None:
        if self.state == HealthState.HEALTHY:
            self._anomaly_since_s = None
            self._healthy_since_s = None
            self.reason = "ok"
            return
        if self._healthy_since_s is None:
            if self._anomaly_since_s is not None:
                self._recovery_start_covariance_fraction = (
                    self._covariance_fraction(now_s)
                )
            self._healthy_since_s = now_s
        self._anomaly_since_s = None
        elapsed = now_s - self._healthy_since_s
        if elapsed + 1e-9 >= self.config.recovery_healthy_s:
            self.state = HealthState.HEALTHY
            self.reason = "ok"
        else:
            self.reason = f"recovering ({elapsed:.2f}/{self.config.recovery_healthy_s:.2f}s)"

    def _covariance_fraction(self, now_s: float) -> float:
        if self.state == HealthState.HEALTHY:
            return 0.0
        if self.state == HealthState.FAULT:
            return 1.0
        if self._anomaly_since_s is not None:
            if self.config.anomaly_to_fault_s <= 0.0:
                return 1.0
            progress = min(
                1.0,
                max(0.0, (now_s - self._anomaly_since_s) / self.config.anomaly_to_fault_s),
            )
            return self._anomaly_start_covariance_fraction + progress * (
                1.0 - self._anomaly_start_covariance_fraction
            )
        if self._healthy_since_s is not None:
            if self.config.recovery_healthy_s <= 0.0:
                return 0.0
            progress = min(
                1.0,
                max(0.0, (now_s - self._healthy_since_s) / self.config.recovery_healthy_s),
            )
            return self._recovery_start_covariance_fraction * (1.0 - progress)
        return self._recovery_start_covariance_fraction

    def _covariance_multiplier(self, now_s: float) -> float:
        fraction = self._covariance_fraction(now_s)
        return 1.0 + fraction * (self.config.suspect_covariance_multiplier - 1.0)

    def _result(self, accepted: bool, now_s: float) -> FrameResult:
        return FrameResult(
            accepted=accepted,
            publish=accepted and self.state != HealthState.FAULT,
            state=self.state,
            reason=self.reason,
            covariance_multiplier=self._covariance_multiplier(now_s),
            velocity_px4_ned=self._filtered_velocity_ned,
            metrics=self.metrics,
        )

    def process_ev(
        self,
        *,
        stamp_s: float,
        receive_time_s: float,
        raw_position_enu: Sequence[float],
        internal_velocity_px4_ned: Optional[Sequence[float]] = None,
        covariance_xyz: Sequence[float] = (math.nan, math.nan, math.nan),
        finite_payload: bool = True,
    ) -> FrameResult:
        now_s = float(receive_time_s)
        raw = (float(raw_position_enu[0]), float(raw_position_enu[1]), float(raw_position_enu[2]))
        covariance = (float(covariance_xyz[0]), float(covariance_xyz[1]), float(covariance_xyz[2]))
        self.metrics.raw_position_enu = raw
        self.metrics.covariance_xyz = covariance
        self.metrics.position_px4_ned = (math.nan, math.nan, math.nan)
        self.metrics.raw_velocity_px4_ned = (math.nan, math.nan, math.nan)
        self.metrics.horizontal_velocity_difference_mps = math.nan
        self.metrics.internal_velocity_difference_mps = math.nan
        self.metrics.internal_vertical_velocity_difference_mps = math.nan
        self.metrics.single_frame_displacement_m = math.nan
        self.metrics.dt_s = math.nan
        self.metrics.velocity_comparison_dt_s = math.nan
        self.metrics.input_age_s = math.nan
        self.metrics.velocity_alignment_s = math.nan
        self.metrics.internal_velocity_alignment_s = math.nan

        if not finite_payload or not _finite((*raw, stamp_s, receive_time_s)):
            self._set_anomaly(now_s, "non_finite_ev_message")
            return self._result(False, now_s)

        # Populate the common-frame diagnostic even when this finite sample is
        # subsequently rejected for age or timestamp reasons.
        position_ned = ev_enu_to_px4_ned(
            raw, self.config.position_yaw_offset_rad
        )
        self.metrics.position_px4_ned = position_ned

        if self._last_stamp_s is not None:
            source_dt = float(stamp_s) - self._last_stamp_s
            self.metrics.dt_s = source_dt
            if source_dt <= 0.0:
                self._set_anomaly(
                    now_s, f"non_monotonic_stamp dt={source_dt:.6f}s"
                )
                # Never move a derivative baseline backwards.
                return self._result(False, now_s)
            if source_dt <= self.config.min_dt_s:
                # A positive but closely spaced source stamp is scheduler or
                # sensor burst jitter, not a clock regression.  Drop it without
                # moving the derivative baseline or poisoning health recovery.
                return self._result(False, now_s)

        input_age_s = now_s - float(stamp_s)
        self.metrics.input_age_s = input_age_s
        if input_age_s > self.config.max_input_age_s:
            self._set_anomaly(now_s, f"stale_input age={input_age_s:.3f}s")
            # A delayed queue must never drag the trusted baseline along an old
            # trajectory one frame at a time.
            return self._result(False, now_s)
        if input_age_s < -self.config.max_future_stamp_s:
            self._set_anomaly(now_s, f"future_input age={input_age_s:.3f}s")
            return self._result(False, now_s)

        if self._last_stamp_s is None or self._last_position_ned is None:
            self._record_internal_velocity(float(stamp_s), internal_velocity_px4_ned)
            self._record_position(float(stamp_s), position_ned)
            self._last_stamp_s = float(stamp_s)
            self._last_receive_s = now_s
            self._last_position_ned = position_ned
            self.metrics.dt_s = math.nan
            self.metrics.single_frame_displacement_m = 0.0
            aligned_px4 = self._interpolated_px4_velocity(float(stamp_s), now_s)
            effective_points_reason = self._effective_points_reason(now_s)
            if aligned_px4 is not None and effective_points_reason is None:
                self.metrics.px4_velocity_px4_ned = aligned_px4
                self._set_healthy_sample(now_s)
            else:
                reason = (
                    "px4_velocity_unaligned"
                    if aligned_px4 is None
                    else effective_points_reason
                )
                self._set_anomaly(now_s, reason)
            return self._result(True, now_s)

        dt = float(stamp_s) - self._last_stamp_s
        self.metrics.dt_s = dt
        delta = _subtract(position_ned, self._last_position_ned)
        displacement = _norm(delta)
        self.metrics.single_frame_displacement_m = displacement
        if dt > self.config.max_dt_s:
            self._set_anomaly(now_s, f"invalid_dt dt={dt:.3f}s")
            # A fresh stream may resume after a scheduling gap, but rebaseline
            # only when it is still spatially continuous with the trusted pose.
            if displacement <= self.config.max_position_jump_m:
                self._resync(float(stamp_s), now_s, raw)
            return self._result(False, now_s)

        if displacement > self.config.max_position_jump_m:
            self._set_anomaly(now_s, f"position_jump displacement={displacement:.3f}m")
            return self._result(False, now_s)

        self._last_stamp_s = float(stamp_s)
        self._last_receive_s = now_s
        self._last_position_ned = position_ned
        self._record_internal_velocity(float(stamp_s), internal_velocity_px4_ned)
        self._record_position(float(stamp_s), position_ned)

        window = self._windowed_position_velocity(float(stamp_s))
        if window is None:
            aligned_px4 = self._interpolated_px4_velocity(float(stamp_s), now_s)
            effective_points_reason = self._effective_points_reason(now_s)
            if aligned_px4 is not None and effective_points_reason is None:
                self.metrics.px4_velocity_px4_ned = aligned_px4
                self._set_healthy_sample(now_s)
            else:
                reason = (
                    "px4_velocity_unaligned"
                    if aligned_px4 is None
                    else effective_points_reason
                )
                self._set_anomaly(now_s, reason)
            return self._result(True, now_s)

        interval_start_stamp_s, raw_velocity = window
        comparison_dt = float(stamp_s) - interval_start_stamp_s
        self.metrics.velocity_comparison_dt_s = comparison_dt
        self.metrics.raw_velocity_px4_ned = raw_velocity
        cutoff = max(0.0, self.config.velocity_lowpass_cutoff_hz)
        alpha = (
            1.0
            if cutoff == 0.0
            else (2.0 * math.pi * cutoff * dt)
            / (1.0 + 2.0 * math.pi * cutoff * dt)
        )
        filtered = tuple(
            previous + alpha * (current - previous)
            for previous, current in zip(self._filtered_velocity_ned, raw_velocity)
        )
        if _horizontal_norm(raw_velocity) <= self.config.stationary_speed_deadband_mps:
            # Preserve low-pass continuity, but prevent a tiny quantisation bias from
            # appearing as a permanent velocity while stationary.
            filtered = (
                0.0
                if abs(filtered[0]) <= self.config.stationary_speed_deadband_mps
                else filtered[0],
                0.0
                if abs(filtered[1]) <= self.config.stationary_speed_deadband_mps
                else filtered[1],
                0.0
                if abs(filtered[2]) <= self.config.stationary_speed_deadband_mps
                else filtered[2],
            )
        self._filtered_velocity_ned = filtered  # type: ignore[assignment]
        self.metrics.velocity_px4_ned = self._filtered_velocity_ned

        anomaly_reason: Optional[str] = None
        effective_points_reason = self._effective_points_reason(now_s)
        if effective_points_reason is not None:
            anomaly_reason = effective_points_reason
        elif internal_velocity_px4_ned is not None:
            internal_velocity = tuple(
                float(value) for value in internal_velocity_px4_ned
            )
            if not _finite(internal_velocity):
                anomaly_reason = "non_finite_internal_velocity"
            else:
                aligned_internal = self._average_internal_velocity(
                    interval_start_stamp_s,
                    float(stamp_s),
                )
                if aligned_internal is None:
                    anomaly_reason = "internal_velocity_unaligned"
                else:
                    # Compare the position derivative with the internal velocity
                    # averaged over the same source-time interval.  Comparing the
                    # current instantaneous twist against the interval derivative
                    # creates false faults during acceleration and braking.
                    internal_difference = _norm(
                        _subtract(aligned_internal, raw_velocity)  # type: ignore[arg-type]
                    )
                    internal_horizontal_difference = _horizontal_norm(
                        _subtract(aligned_internal, raw_velocity)  # type: ignore[arg-type]
                    )
                    internal_vertical_difference = abs(
                        aligned_internal[2] - raw_velocity[2]
                    )
                    self.metrics.internal_velocity_difference_mps = internal_difference
                    self.metrics.internal_vertical_velocity_difference_mps = (
                        internal_vertical_difference
                    )
                    # EKF2_EV_CTRL=11 does not fuse EV velocity.  FAST-LIO's
                    # vertical twist is therefore retained as a diagnostic, but
                    # a vertical-only transient must not force an EV FAULT and
                    # automatic landing.  Horizontal consistency remains gated.
                    if (
                        internal_horizontal_difference
                        > self.config.max_internal_velocity_difference_mps
                    ):
                        anomaly_reason = (
                            "internal_velocity_mismatch "
                            f"difference={internal_horizontal_difference:.3f}m/s"
                        )
        if anomaly_reason is None:
            aligned_px4 = self._average_px4_velocity(
                interval_start_stamp_s,
                float(stamp_s),
                now_s,
            )
            if aligned_px4 is None:
                anomaly_reason = "px4_velocity_unaligned"
                self.metrics.horizontal_velocity_difference_mps = math.nan
            else:
                # EV differencing represents the whole source-time interval.
                # Compare it with PX4 velocity averaged over that same interval;
                # filtering is reserved for the published vision speed.
                velocity_difference = _subtract(raw_velocity, aligned_px4)
                horizontal_difference = _horizontal_norm(velocity_difference)
                self.metrics.horizontal_velocity_difference_mps = horizontal_difference
                self.metrics.px4_velocity_px4_ned = aligned_px4
                either_moving = max(
                    _horizontal_norm(raw_velocity),
                    _horizontal_norm(aligned_px4),
                ) >= self.config.velocity_difference_min_speed_mps
                if (
                    either_moving
                    and horizontal_difference
                    > self.config.max_horizontal_velocity_difference_mps
                ):
                    anomaly_reason = f"velocity_mismatch difference={horizontal_difference:.3f}m/s"

        if anomaly_reason is None:
            self._set_healthy_sample(now_s)
        else:
            self._set_anomaly(now_s, anomaly_reason)
        return self._result(True, now_s)

    def _px4_velocity_at(
        self, sample_stamp_s: float, now_s: float
    ) -> Optional[Tuple[Vector3, float]]:
        """Interpolate PX4 velocity at a source timestamp, with bounded fallback."""
        if not self._px4_velocity_is_fresh(now_s) or not self._px4_velocity_history:
            return None

        samples = list(self._px4_velocity_history)
        before = None
        after = None
        for sample in samples:
            if sample[0] <= sample_stamp_s:
                before = sample
            if sample[0] >= sample_stamp_s:
                after = sample
                break

        if before is not None and after is not None:
            before_error = sample_stamp_s - before[0]
            after_error = after[0] - sample_stamp_s
            alignment_s = max(before_error, after_error)
            if alignment_s <= self.config.max_velocity_alignment_s + 1e-9:
                span_s = after[0] - before[0]
                if span_s <= 1e-12:
                    return before[1], alignment_s
                fraction = before_error / span_s
                interpolated = tuple(
                    left + fraction * (right - left)
                    for left, right in zip(before[1], after[1])
                )
                return interpolated, alignment_s  # type: ignore[return-value]

        nearest = min(samples, key=lambda sample: abs(sample[0] - sample_stamp_s))
        alignment_s = abs(nearest[0] - sample_stamp_s)
        # Preserve the measured miss even when it exceeds the safety gate.  A
        # NaN here made intermittent cross-stream timestamp failures impossible
        # to distinguish from a missing or stale PX4 velocity stream offline.
        self.metrics.velocity_alignment_s = alignment_s
        if alignment_s > self.config.max_velocity_alignment_s + 1e-9:
            return None
        return nearest[1], alignment_s

    def _record_position(self, sample_stamp_s: float, position_ned: Vector3) -> None:
        if self._position_history and sample_stamp_s <= self._position_history[-1][0]:
            return
        self._position_history.append((sample_stamp_s, position_ned))
        window_s = self.config.velocity_comparison_window_s
        while (
            len(self._position_history) > 2
            and sample_stamp_s - self._position_history[1][0] >= window_s
        ):
            self._position_history.popleft()

    def _windowed_position_velocity(
        self, end_stamp_s: float
    ) -> Optional[Tuple[float, Vector3]]:
        target_stamp_s = end_stamp_s - self.config.velocity_comparison_window_s
        samples = list(self._position_history)
        if len(samples) < 2 or target_stamp_s < samples[0][0] - 1e-9:
            return None

        before = None
        after = None
        for sample in samples:
            if sample[0] <= target_stamp_s + 1e-12:
                before = sample
            if sample[0] >= target_stamp_s - 1e-12:
                after = sample
                break
        if before is None or after is None:
            return None

        if after[0] - before[0] <= 1e-12:
            start_position = before[1]
        else:
            fraction = (target_stamp_s - before[0]) / (after[0] - before[0])
            start_position = tuple(
                left + fraction * (right - left)
                for left, right in zip(before[1], after[1])
            )
        end_position = samples[-1][1]
        interval_s = end_stamp_s - target_stamp_s
        velocity = tuple(
            (end - start) / interval_s
            for start, end in zip(start_position, end_position)
        )
        return target_stamp_s, velocity  # type: ignore[return-value]

    def _interpolated_px4_velocity(
        self, sample_stamp_s: float, now_s: float
    ) -> Optional[Vector3]:
        aligned = self._px4_velocity_at(sample_stamp_s, now_s)
        if aligned is None:
            return None
        velocity, alignment_s = aligned
        self.metrics.velocity_alignment_s = alignment_s
        return velocity

    def _average_px4_velocity(
        self, start_stamp_s: float, end_stamp_s: float, now_s: float
    ) -> Optional[Vector3]:
        """Time-average PX4 velocity over the EV differencing interval."""
        if end_stamp_s <= start_stamp_s:
            return None
        start = self._px4_velocity_at(start_stamp_s, now_s)
        end = self._px4_velocity_at(end_stamp_s, now_s)
        if start is None or end is None:
            return None

        points = [(start_stamp_s, start[0])]
        points.extend(
            (sample_stamp, velocity)
            for sample_stamp, velocity, _receive_stamp in self._px4_velocity_history
            if start_stamp_s < sample_stamp < end_stamp_s
        )
        points.append((end_stamp_s, end[0]))

        integral = [0.0, 0.0, 0.0]
        for (left_stamp, left_velocity), (right_stamp, right_velocity) in zip(
            points, points[1:]
        ):
            segment_dt = right_stamp - left_stamp
            for axis in range(3):
                integral[axis] += (
                    0.5
                    * (left_velocity[axis] + right_velocity[axis])
                    * segment_dt
                )
        interval_dt = end_stamp_s - start_stamp_s
        self.metrics.velocity_alignment_s = max(start[1], end[1])
        return tuple(value / interval_dt for value in integral)  # type: ignore[return-value]

    def _record_internal_velocity(
        self,
        sample_stamp_s: float,
        velocity: Optional[Sequence[float]],
    ) -> None:
        if velocity is None:
            return
        parsed = tuple(float(value) for value in velocity)
        if not _finite((*parsed, sample_stamp_s)):
            return
        if self._internal_velocity_history and sample_stamp_s <= self._internal_velocity_history[-1][0]:
            return
        self._internal_velocity_history.append((sample_stamp_s, parsed))
        while (
            self._internal_velocity_history
            and sample_stamp_s - self._internal_velocity_history[0][0]
            > self.config.internal_velocity_history_s
        ):
            self._internal_velocity_history.popleft()

    def _internal_velocity_at(
        self,
        sample_stamp_s: float,
    ) -> Optional[Tuple[Vector3, float]]:
        if not self._internal_velocity_history:
            return None
        samples = list(self._internal_velocity_history)
        before = None
        after = None
        for sample in samples:
            if sample[0] <= sample_stamp_s:
                before = sample
            if sample[0] >= sample_stamp_s:
                after = sample
                break
        if before is not None and after is not None:
            before_error = sample_stamp_s - before[0]
            after_error = after[0] - sample_stamp_s
            alignment_s = max(before_error, after_error)
            if alignment_s <= self.config.max_internal_velocity_alignment_s + 1e-9:
                span_s = after[0] - before[0]
                if span_s <= 1e-12:
                    return before[1], alignment_s
                fraction = before_error / span_s
                interpolated = tuple(
                    left + fraction * (right - left)
                    for left, right in zip(before[1], after[1])
                )
                return interpolated, alignment_s  # type: ignore[return-value]
        nearest = min(samples, key=lambda sample: abs(sample[0] - sample_stamp_s))
        alignment_s = abs(nearest[0] - sample_stamp_s)
        self.metrics.internal_velocity_alignment_s = alignment_s
        if alignment_s > self.config.max_internal_velocity_alignment_s + 1e-9:
            return None
        return nearest[1], alignment_s

    def _average_internal_velocity(
        self,
        start_stamp_s: float,
        end_stamp_s: float,
    ) -> Optional[Vector3]:
        """Time-average FAST-LIO velocity over the EV differencing interval."""
        if end_stamp_s <= start_stamp_s:
            return None
        start = self._internal_velocity_at(start_stamp_s)
        end = self._internal_velocity_at(end_stamp_s)
        if start is None or end is None:
            return None
        points = [(start_stamp_s, start[0])]
        points.extend(
            (sample_stamp, velocity)
            for sample_stamp, velocity in self._internal_velocity_history
            if start_stamp_s < sample_stamp < end_stamp_s
        )
        points.append((end_stamp_s, end[0]))
        integral = [0.0, 0.0, 0.0]
        for (left_stamp, left_velocity), (right_stamp, right_velocity) in zip(
            points, points[1:]
        ):
            segment_dt = right_stamp - left_stamp
            for axis in range(3):
                integral[axis] += (
                    0.5
                    * (left_velocity[axis] + right_velocity[axis])
                    * segment_dt
                )
        interval_dt = end_stamp_s - start_stamp_s
        self.metrics.internal_velocity_alignment_s = max(start[1], end[1])
        return tuple(value / interval_dt for value in integral)  # type: ignore[return-value]

    def _resync(self, stamp_s: float, receive_time_s: float, raw_position_enu: Vector3) -> None:
        self._last_stamp_s = stamp_s
        self._last_receive_s = receive_time_s
        self._last_position_ned = ev_enu_to_px4_ned(
            raw_position_enu, self.config.position_yaw_offset_rad
        )
        self._filtered_velocity_ned = (0.0, 0.0, 0.0)
        self.metrics.velocity_px4_ned = self._filtered_velocity_ned
        self._internal_velocity_history.clear()
        self._position_history.clear()
        self._record_position(stamp_s, self._last_position_ned)

    def _px4_velocity_is_fresh(self, now_s: float) -> bool:
        return (
            self._px4_velocity_ned is not None
            and self._px4_receive_s is not None
            and 0.0 <= now_s - self._px4_receive_s <= self.config.px4_velocity_timeout_s
        )

    def check_timeout(self, now_s: float) -> HealthState:
        now_s = float(now_s)
        if self._last_receive_s is None:
            self._set_anomaly(now_s, "waiting_for_ev")
        elif now_s - self._last_receive_s > self.config.message_timeout_s:
            self._set_anomaly(now_s, f"ev_timeout age={now_s - self._last_receive_s:.3f}s")
        elif not self._px4_velocity_is_fresh(now_s):
            self._set_anomaly(now_s, "px4_velocity_timeout")
        elif self._effective_points_reason(now_s) is not None:
            self._set_anomaly(now_s, self._effective_points_reason(now_s))
        elif self._anomaly_since_s is not None:
            # A timer tick must also advance an already active anomaly.  Without
            # this, one bad frame followed by a short input pause could remain
            # SUSPECT until another EV callback arrived, exceeding the configured
            # anomaly_to_fault_s deadline.
            self._set_anomaly(now_s, self.reason)
        return self.state
