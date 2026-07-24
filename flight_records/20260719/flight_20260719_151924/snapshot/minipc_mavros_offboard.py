#!/usr/bin/env python3

import math
import sys
from collections import deque
from enum import Enum, auto
from typing import Optional

import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion
from mavros_msgs.msg import ExtendedState, PositionTarget, State
from mavros_msgs.srv import CommandBool, SetMode
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class Phase(Enum):
    WAIT_FOR_CONNECTION = auto()
    WAIT_FOR_LOCAL_POSE = auto()
    STREAM_SETPOINTS = auto()
    WAIT_FOR_RC_OFFBOARD = auto()
    REQUEST_MODE = auto()
    WAIT_FOR_OFFBOARD = auto()
    REQUEST_ARM = auto()
    ARM_REJECTED = auto()
    HOLD = auto()
    ROUTE_TRACKING = auto()
    FINAL_HOLD = auto()
    OFFBOARD_LAND = auto()
    LANDED_HOLD = auto()
    REQUEST_LAND = auto()
    WAIT_FOR_LAND = auto()
    REQUEST_DISARM = auto()
    WAIT_FOR_DISARM = auto()
    DONE = auto()


def yaw_to_quaternion(yaw_rad: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw_rad / 2.0)
    q.w = math.cos(yaw_rad / 2.0)
    return q


def ned_delta_to_enu(dx_ned: float, dy_ned: float, dz_ned: float) -> tuple[float, float, float]:
    return dy_ned, dx_ned, -dz_ned


def clamp(value: float, lower: float, upper: float) -> float:
    return min(max(value, lower), upper)


def vector_norm(x: float, y: float) -> float:
    return math.hypot(x, y)


class RampSetpointGenerator:
    def make_target_setpoint(
        self,
        *,
        node: Node,
        target_pose: PoseStamped,
        final_target_z: float,
        arm_only: bool,
        ignore_z_in_arm_only: bool,
        hold_elapsed_s: float,
        z_ramp_seconds: float,
    ) -> PositionTarget:
        msg = self._base_position_target(node)

        if arm_only and ignore_z_in_arm_only:
            msg.type_mask |= PositionTarget.IGNORE_PZ

        msg.position.x = float(target_pose.pose.position.x)
        msg.position.y = float(target_pose.pose.position.y)

        if arm_only:
            z_cmd = target_pose.pose.position.z
        else:
            z_start = target_pose.pose.position.z
            alpha = clamp(hold_elapsed_s / z_ramp_seconds, 0.0, 1.0)
            z_cmd = z_start + alpha * (final_target_z - z_start)
        msg.position.z = float(z_cmd)
        msg.yaw = self._yaw_from_pose(target_pose)
        return msg

    def make_landing_setpoint(
        self,
        *,
        node: Node,
        target_pose: PoseStamped,
        reference_pose: PoseStamped,
        start_z: float,
        elapsed_s: float,
        land_speed_mps: float,
    ) -> PositionTarget:
        msg = self._base_position_target(node)
        msg.position.x = float(target_pose.pose.position.x)
        msg.position.y = float(target_pose.pose.position.y)

        z_end = float(reference_pose.pose.position.z)
        if start_z >= z_end:
            z_cmd = max(z_end, start_z - land_speed_mps * elapsed_s)
        else:
            z_cmd = min(z_end, start_z + land_speed_mps * elapsed_s)
        msg.position.z = float(z_cmd)
        msg.yaw = self._yaw_from_pose(target_pose)
        return msg

    @staticmethod
    def _base_position_target(node: Node) -> PositionTarget:
        msg = PositionTarget()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        msg.type_mask = (
            PositionTarget.IGNORE_VX
            | PositionTarget.IGNORE_VY
            | PositionTarget.IGNORE_VZ
            | PositionTarget.IGNORE_AFX
            | PositionTarget.IGNORE_AFY
            | PositionTarget.IGNORE_AFZ
            | PositionTarget.IGNORE_YAW_RATE
        )
        return msg

    @staticmethod
    def _yaw_from_pose(target_pose: PoseStamped) -> float:
        q = target_pose.pose.orientation
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )


class MpcSetpointGenerator(RampSetpointGenerator):
    """Lightweight constrained double-integrator receding-horizon setpoint generator."""

    def __init__(
        self,
        *,
        dt: float,
        horizon_steps: int,
        max_vz: float,
        max_az: float,
        max_vxy: float,
        max_axy: float,
        xy_hold_weight: float,
        z_tracking_weight: float,
        velocity_weight: float,
        accel_weight: float,
        terminal_weight: float,
    ) -> None:
        self.dt = max(0.01, dt)
        self.horizon_steps = max(1, horizon_steps)
        self.max_vz = max(0.02, max_vz)
        self.max_az = max(0.02, max_az)
        self.max_vxy = max(0.02, max_vxy)
        self.max_axy = max(0.02, max_axy)
        self.xy_hold_weight = max(0.01, xy_hold_weight)
        self.z_tracking_weight = max(0.01, z_tracking_weight)
        self.velocity_weight = max(0.01, velocity_weight)
        self.accel_weight = max(0.0, accel_weight)
        self.terminal_weight = max(0.01, terminal_weight)
        self.last_debug_ns: Optional[int] = None
        self.last_debug_msg: Optional[PositionTarget] = None

    def make_target_setpoint(
        self,
        *,
        node: Node,
        target_pose: PoseStamped,
        final_target_z: float,
        arm_only: bool,
        ignore_z_in_arm_only: bool,
        hold_elapsed_s: float,
        z_ramp_seconds: float,
        local_odom: Optional[Odometry] = None,
    ) -> PositionTarget:
        if arm_only:
            return super().make_target_setpoint(
                node=node,
                target_pose=target_pose,
                final_target_z=final_target_z,
                arm_only=arm_only,
                ignore_z_in_arm_only=ignore_z_in_arm_only,
                hold_elapsed_s=hold_elapsed_s,
                z_ramp_seconds=z_ramp_seconds,
            )

        z_start = target_pose.pose.position.z
        alpha = clamp(hold_elapsed_s / z_ramp_seconds, 0.0, 1.0)
        z_ref = z_start + alpha * (final_target_z - z_start)
        return self._make_mpc_setpoint(
            node=node,
            target_pose=target_pose,
            ref_x=target_pose.pose.position.x,
            ref_y=target_pose.pose.position.y,
            ref_z=z_ref,
            yaw=RampSetpointGenerator._yaw_from_pose(target_pose),
            local_odom=local_odom,
            max_vz=self.max_vz,
            phase_name="hold",
        )

    def make_landing_setpoint(
        self,
        *,
        node: Node,
        target_pose: PoseStamped,
        reference_pose: PoseStamped,
        start_z: float,
        elapsed_s: float,
        land_speed_mps: float,
        local_odom: Optional[Odometry] = None,
    ) -> PositionTarget:
        z_end = float(reference_pose.pose.position.z)
        limited_land_speed = min(self.max_vz, max(0.02, land_speed_mps))
        if start_z >= z_end:
            z_ref = max(z_end, start_z - limited_land_speed * elapsed_s)
        else:
            z_ref = min(z_end, start_z + limited_land_speed * elapsed_s)

        return self._make_mpc_setpoint(
            node=node,
            target_pose=target_pose,
            ref_x=target_pose.pose.position.x,
            ref_y=target_pose.pose.position.y,
            ref_z=z_ref,
            yaw=RampSetpointGenerator._yaw_from_pose(target_pose),
            local_odom=local_odom,
            max_vz=limited_land_speed,
            phase_name="land",
        )

    def _make_mpc_setpoint(
        self,
        *,
        node: Node,
        target_pose: PoseStamped,
        ref_x: float,
        ref_y: float,
        ref_z: float,
        yaw: float,
        local_odom: Optional[Odometry],
        max_vz: float,
        phase_name: str,
    ) -> PositionTarget:
        if local_odom is None:
            return super().make_target_setpoint(
                node=node,
                target_pose=target_pose,
                final_target_z=ref_z,
                arm_only=False,
                ignore_z_in_arm_only=False,
                hold_elapsed_s=1.0,
                z_ramp_seconds=1.0,
            )

        p = local_odom.pose.pose.position
        v = local_odom.twist.twist.linear

        ax = self._axis_accel(
            pos=float(p.x),
            vel=float(v.x),
            ref=float(ref_x),
            max_accel=self.max_axy,
            max_vel=self.max_vxy,
            position_weight=self.xy_hold_weight,
        )
        ay = self._axis_accel(
            pos=float(p.y),
            vel=float(v.y),
            ref=float(ref_y),
            max_accel=self.max_axy,
            max_vel=self.max_vxy,
            position_weight=self.xy_hold_weight,
        )
        az = self._axis_accel(
            pos=float(p.z),
            vel=float(v.z),
            ref=float(ref_z),
            max_accel=self.max_az,
            max_vel=max_vz,
            position_weight=self.z_tracking_weight,
        )

        axy_norm = vector_norm(ax, ay)
        if axy_norm > self.max_axy:
            scale = self.max_axy / axy_norm
            ax *= scale
            ay *= scale

        vx_cmd = clamp(float(v.x) + ax * self.dt, -self.max_vxy, self.max_vxy)
        vy_cmd = clamp(float(v.y) + ay * self.dt, -self.max_vxy, self.max_vxy)
        vxy_norm = vector_norm(vx_cmd, vy_cmd)
        if vxy_norm > self.max_vxy:
            scale = self.max_vxy / vxy_norm
            vx_cmd *= scale
            vy_cmd *= scale
        vz_cmd = clamp(float(v.z) + az * self.dt, -max_vz, max_vz)

        x_cmd = float(p.x) + vx_cmd * self.dt + 0.5 * ax * self.dt * self.dt
        y_cmd = float(p.y) + vy_cmd * self.dt + 0.5 * ay * self.dt * self.dt
        z_cmd = float(p.z) + vz_cmd * self.dt + 0.5 * az * self.dt * self.dt

        # Keep the generated setpoint on the safe side of the moving reference.
        x_cmd = self._limit_step_toward(float(p.x), x_cmd, float(ref_x))
        y_cmd = self._limit_step_toward(float(p.y), y_cmd, float(ref_y))
        z_cmd = self._limit_step_toward(float(p.z), z_cmd, float(ref_z))

        msg = PositionTarget()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        msg.type_mask = PositionTarget.IGNORE_YAW_RATE
        msg.position.x = x_cmd
        msg.position.y = y_cmd
        msg.position.z = z_cmd
        msg.velocity.x = vx_cmd
        msg.velocity.y = vy_cmd
        msg.velocity.z = vz_cmd
        msg.acceleration_or_force.x = ax
        msg.acceleration_or_force.y = ay
        msg.acceleration_or_force.z = az
        msg.yaw = yaw

        self.last_debug_msg = msg
        self._log_debug_1hz(
            node=node,
            phase_name=phase_name,
            px=float(p.x),
            py=float(p.y),
            pz=float(p.z),
            vx=float(v.x),
            vy=float(v.y),
            vz=float(v.z),
            ref_x=float(ref_x),
            ref_y=float(ref_y),
            ref_z=float(ref_z),
            vx_cmd=vx_cmd,
            vy_cmd=vy_cmd,
            vz_cmd=vz_cmd,
            ax=ax,
            ay=ay,
            az=az,
        )
        return msg

    def _axis_accel(
        self,
        *,
        pos: float,
        vel: float,
        ref: float,
        max_accel: float,
        max_vel: float,
        position_weight: float,
    ) -> float:
        error = ref - pos
        lookahead = max(self.dt, self.dt * self.horizon_steps)
        kp = (position_weight + self.terminal_weight) / (lookahead * lookahead)
        kd = self.velocity_weight / lookahead
        raw_accel = kp * error - kd * vel
        if self.accel_weight > 0.0:
            raw_accel /= 1.0 + self.accel_weight

        stopping_distance = vel * vel / (2.0 * max_accel)
        if abs(error) <= stopping_distance and error * vel > 0.0:
            raw_accel = -math.copysign(max_accel, vel)

        next_vel = clamp(vel + raw_accel * self.dt, -max_vel, max_vel)
        raw_accel = (next_vel - vel) / self.dt
        return clamp(raw_accel, -max_accel, max_accel)

    @staticmethod
    def _limit_step_toward(current: float, command: float, ref: float) -> float:
        if current <= ref:
            return clamp(command, current, ref)
        return clamp(command, ref, current)

    def _log_debug_1hz(
        self,
        *,
        node: Node,
        phase_name: str,
        px: float,
        py: float,
        pz: float,
        vx: float,
        vy: float,
        vz: float,
        ref_x: float,
        ref_y: float,
        ref_z: float,
        vx_cmd: float,
        vy_cmd: float,
        vz_cmd: float,
        ax: float,
        ay: float,
        az: float,
    ) -> None:
        now_ns = node.get_clock().now().nanoseconds
        if self.last_debug_ns is not None and (now_ns - self.last_debug_ns) < 1_000_000_000:
            return
        self.last_debug_ns = now_ns
        node.get_logger().info(
            "MPC setpoint "
            f"phase={phase_name}, ref=({ref_x:.2f}, {ref_y:.2f}, {ref_z:.2f}), "
            f"pos_err=({ref_x - px:.2f}, {ref_y - py:.2f}, {ref_z - pz:.2f}), "
            f"vel_err=({vx_cmd - vx:.2f}, {vy_cmd - vy:.2f}, {vz_cmd - vz:.2f}), "
            f"accel=({ax:.2f}, {ay:.2f}, {az:.2f})"
        )


class RoutePathFollower:
    def __init__(
        self,
        *,
        node: Node,
        path_topic: str,
        fixed_height_m: float,
        start_tolerance_m: float,
        goal_tolerance_m: float,
        lookahead_m: float,
        max_vxy_mps: float,
        max_vz_mps: float,
        path_timeout_s: float,
        map_frame: str = "map",
        body_frame: str = "body",
    ) -> None:
        self.node = node
        self.path_topic = path_topic
        self.fixed_height_m = max(0.05, fixed_height_m)
        self.start_tolerance_m = max(0.02, start_tolerance_m)
        self.goal_tolerance_m = max(0.02, goal_tolerance_m)
        self.lookahead_m = max(0.05, lookahead_m)
        self.max_vxy_mps = max(0.02, max_vxy_mps)
        self.max_vz_mps = max(0.02, max_vz_mps)
        self.path_timeout_s = max(0.2, path_timeout_s)
        self.map_frame = map_frame
        self.body_frame = body_frame

        self.latest_path: Optional[Path] = None
        self.path_received_ns: Optional[int] = None
        self.reference_points: list[tuple[float, float]] = []
        self.map_to_local_xy_offset: Optional[tuple[float, float]] = None
        self.last_debug_ns: Optional[int] = None
        self.last_error: str = "no path received"

        transient_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.reference_path_pub = node.create_publisher(
            Path, "/route_tracker/reference_path_3d", transient_qos
        )
        self.lookahead_pub = node.create_publisher(
            PoseStamped, "/route_tracker/lookahead_target", 10
        )
        node.create_subscription(Path, path_topic, self._path_cb, transient_qos)

    def _path_cb(self, msg: Path) -> None:
        self.latest_path = msg
        self.path_received_ns = self.node.get_clock().now().nanoseconds
        self.map_to_local_xy_offset = None
        if not msg.poses:
            self.reference_points = []
            self.last_error = "path is empty"
            self.node.get_logger().warning("Route path is empty; route tracking will not start")
            return

        frame_id = msg.header.frame_id or self.map_frame
        if frame_id != self.map_frame:
            self.reference_points = []
            self.last_error = f"path frame {frame_id!r} is not {self.map_frame!r}"
            self.node.get_logger().warning(
                f"Route path frame is {frame_id!r}; first version expects {self.map_frame!r}"
            )
            return

        raw_points: list[tuple[float, float]] = []
        for pose in msg.poses:
            p = pose.pose.position
            if all(math.isfinite(v) for v in (p.x, p.y)):
                raw_points.append((float(p.x), float(p.y)))
        if len(raw_points) < 2:
            self.reference_points = []
            self.last_error = "path has fewer than 2 finite points"
            self.node.get_logger().warning("Route path has fewer than 2 finite points")
            return

        self.reference_points = self._resample(raw_points)
        self.last_error = ""
        self._publish_reference_path()
        self.node.get_logger().info(
            f"Received route path: raw_points={len(raw_points)}, "
            f"resampled_points={len(self.reference_points)}, frame={frame_id}"
        )

    def _resample(self, points: list[tuple[float, float]]) -> list[tuple[float, float]]:
        step = min(0.20, max(0.05, self.lookahead_m * 0.5))
        out = [points[0]]
        for start, end in zip(points[:-1], points[1:]):
            sx, sy = start
            ex, ey = end
            length = math.hypot(ex - sx, ey - sy)
            if length < 1e-6:
                continue
            count = max(1, int(math.ceil(length / step)))
            for i in range(1, count + 1):
                ratio = i / count
                out.append((sx + (ex - sx) * ratio, sy + (ey - sy) * ratio))
        return out

    def _publish_reference_path(self) -> None:
        path = Path()
        path.header.frame_id = self.map_frame
        path.header.stamp = self.node.get_clock().now().to_msg()
        for x, y in self.reference_points:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = self.fixed_height_m
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.reference_path_pub.publish(path)

    def path_ready(self) -> bool:
        if self.path_received_ns is None or self.latest_path is None:
            self.last_error = "no path received"
            return False
        age_s = (self.node.get_clock().now().nanoseconds - self.path_received_ns) / 1e9
        if age_s > self.path_timeout_s:
            self.last_error = f"path timeout {age_s:.2f}s > {self.path_timeout_s:.2f}s"
            return False
        if len(self.reference_points) < 2:
            if not self.last_error:
                self.last_error = "path has fewer than 2 points"
            return False
        return True

    def lookup_map_position(self, tf_buffer: tf2_ros.Buffer) -> Optional[tuple[float, float]]:
        try:
            transform = tf_buffer.lookup_transform(
                self.map_frame,
                self.body_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.02),
            )
        except Exception as exc:  # noqa: BLE001
            self.last_error = f"TF {self.map_frame}->{self.body_frame} unavailable: {exc}"
            return None
        t = transform.transform.translation
        if not all(math.isfinite(v) for v in (t.x, t.y)):
            self.last_error = "TF map position is not finite"
            return None
        return float(t.x), float(t.y)

    def prepare_start(
        self,
        *,
        current_map_xy: tuple[float, float],
        current_local_xy: tuple[float, float],
    ) -> bool:
        if not self.path_ready():
            return False

        start_x, start_y = self.reference_points[0]
        start_error = math.hypot(current_map_xy[0] - start_x, current_map_xy[1] - start_y)
        if start_error > self.start_tolerance_m:
            self.last_error = (
                f"path start too far: {start_error:.2f}m > {self.start_tolerance_m:.2f}m"
            )
            return False

        self.map_to_local_xy_offset = (
            current_local_xy[0] - current_map_xy[0],
            current_local_xy[1] - current_map_xy[1],
        )
        self.node.get_logger().info(
            "Route map/local alignment locked: "
            f"current_map=({current_map_xy[0]:.2f}, {current_map_xy[1]:.2f}), "
            f"current_local=({current_local_xy[0]:.2f}, {current_local_xy[1]:.2f}), "
            f"map_to_local_xy_offset=({self.map_to_local_xy_offset[0]:.2f}, "
            f"{self.map_to_local_xy_offset[1]:.2f})"
        )
        return True

    def make_setpoint(
        self,
        *,
        current_map_xy: tuple[float, float],
        current_local_odom: Odometry,
        yaw: float,
        dt_s: float,
    ) -> tuple[PositionTarget, bool]:
        if self.map_to_local_xy_offset is None:
            raise RuntimeError("route map/local alignment has not been initialized")
        if not self.path_ready():
            raise RuntimeError(self.last_error)

        nearest_idx = self._nearest_index(current_map_xy)
        lookahead_idx = self._lookahead_index(nearest_idx)
        target_map_xy = self.reference_points[lookahead_idx]
        goal_map_xy = self.reference_points[-1]

        offset_x, offset_y = self.map_to_local_xy_offset
        target_local_x = target_map_xy[0] + offset_x
        target_local_y = target_map_xy[1] + offset_y

        p = current_local_odom.pose.pose.position
        dx = target_local_x - float(p.x)
        dy = target_local_y - float(p.y)
        dist = math.hypot(dx, dy)
        max_step = self.max_vxy_mps * max(0.01, dt_s)
        if dist > max_step and dist > 1e-6:
            scale = max_step / dist
            cmd_x = float(p.x) + dx * scale
            cmd_y = float(p.y) + dy * scale
            vx_cmd = dx / dist * self.max_vxy_mps
            vy_cmd = dy / dist * self.max_vxy_mps
        else:
            cmd_x = target_local_x
            cmd_y = target_local_y
            vx_cmd = dx / max(0.01, dt_s)
            vy_cmd = dy / max(0.01, dt_s)

        vxy = math.hypot(vx_cmd, vy_cmd)
        if vxy > self.max_vxy_mps:
            scale = self.max_vxy_mps / vxy
            vx_cmd *= scale
            vy_cmd *= scale

        target_z_ned = -self.fixed_height_m
        dz = target_z_ned - float(p.z)
        vz_cmd = clamp(dz / max(0.01, dt_s), -self.max_vz_mps, self.max_vz_mps)
        cmd_z = float(p.z) + vz_cmd * max(0.01, dt_s)
        if abs(target_z_ned - cmd_z) < 0.02:
            cmd_z = target_z_ned

        msg = PositionTarget()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        msg.type_mask = (
            PositionTarget.IGNORE_AFX
            | PositionTarget.IGNORE_AFY
            | PositionTarget.IGNORE_AFZ
            | PositionTarget.IGNORE_YAW_RATE
        )
        msg.position.x = cmd_x
        msg.position.y = cmd_y
        msg.position.z = cmd_z
        msg.velocity.x = vx_cmd
        msg.velocity.y = vy_cmd
        msg.velocity.z = vz_cmd
        msg.yaw = yaw

        self._publish_lookahead(target_map_xy)
        self._log_debug(current_map_xy, current_local_odom, target_map_xy, (cmd_x, cmd_y, cmd_z))

        goal_dist = math.hypot(current_map_xy[0] - goal_map_xy[0], current_map_xy[1] - goal_map_xy[1])
        finished = goal_dist <= self.goal_tolerance_m
        return msg, finished

    def make_hold_setpoint(self, *, current_local_odom: Odometry, yaw: float) -> PositionTarget:
        p = current_local_odom.pose.pose.position
        msg = RampSetpointGenerator._base_position_target(self.node)
        msg.position.x = float(p.x)
        msg.position.y = float(p.y)
        msg.position.z = float(p.z)
        msg.yaw = yaw
        return msg

    def _nearest_index(self, current_map_xy: tuple[float, float]) -> int:
        best_idx = 0
        best_dist = float("inf")
        for idx, (x, y) in enumerate(self.reference_points):
            dist = (current_map_xy[0] - x) ** 2 + (current_map_xy[1] - y) ** 2
            if dist < best_dist:
                best_idx = idx
                best_dist = dist
        return best_idx

    def _lookahead_index(self, nearest_idx: int) -> int:
        total = 0.0
        last = self.reference_points[nearest_idx]
        for idx in range(nearest_idx + 1, len(self.reference_points)):
            point = self.reference_points[idx]
            total += math.hypot(point[0] - last[0], point[1] - last[1])
            if total >= self.lookahead_m:
                return idx
            last = point
        return len(self.reference_points) - 1

    def _publish_lookahead(self, target_map_xy: tuple[float, float]) -> None:
        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = self.node.get_clock().now().to_msg()
        pose.pose.position.x = target_map_xy[0]
        pose.pose.position.y = target_map_xy[1]
        pose.pose.position.z = self.fixed_height_m
        pose.pose.orientation.w = 1.0
        self.lookahead_pub.publish(pose)

    def _log_debug(
        self,
        current_map_xy: tuple[float, float],
        current_local_odom: Odometry,
        target_map_xy: tuple[float, float],
        local_setpoint_xyz: tuple[float, float, float],
    ) -> None:
        now_ns = self.node.get_clock().now().nanoseconds
        if self.last_debug_ns is not None and (now_ns - self.last_debug_ns) < 1_000_000_000:
            return
        self.last_debug_ns = now_ns
        p = current_local_odom.pose.pose.position
        offset = self.map_to_local_xy_offset or (float("nan"), float("nan"))
        self.node.get_logger().info(
            "Route tracking setpoint: "
            f"current_map=({current_map_xy[0]:.2f}, {current_map_xy[1]:.2f}), "
            f"current_local=({p.x:.2f}, {p.y:.2f}), "
            f"map_to_local_xy_offset=({offset[0]:.2f}, {offset[1]:.2f}), "
            f"lookahead_map=({target_map_xy[0]:.2f}, {target_map_xy[1]:.2f}), "
            f"local_setpoint=({local_setpoint_xyz[0]:.2f}, {local_setpoint_xyz[1]:.2f}, "
            f"{local_setpoint_xyz[2]:.2f})"
        )


class MiniPcMavrosOffboard(Node):
    def __init__(self) -> None:
        super().__init__("minipc_mavros_offboard")

        self.declare_parameter("arm_only", True)
        self.declare_parameter("takeoff_height_m", 0.8)
        self.declare_parameter("hover_seconds", 10.0)
        self.declare_parameter("prestream_count", 30)
        self.declare_parameter("offboard_stabilize_seconds", 1.0)
        self.declare_parameter("use_rc_offboard", False)
        self.declare_parameter("hold_after_arm_reject", True)
        self.declare_parameter("ignore_z_in_arm_only", True)
        self.declare_parameter("debug_hold_after_mode_drop", True)
        self.declare_parameter("recent_pose_samples", 30)
        self.declare_parameter("require_vision_pose", True)
        self.declare_parameter("vision_freshness_s", 1.0)
        self.declare_parameter("lift_only_seconds", 2.5)
        self.declare_parameter("z_ramp_seconds", 4.0)
        self.declare_parameter("target_x_m", 0.0)
        self.declare_parameter("target_y_m", 0.0)
        self.declare_parameter("target_z_m", 0.0)
        self.declare_parameter("xy_move_unlock_height_m", 0.0)
        self.declare_parameter("yaw_deg", 0.0)
        self.declare_parameter("auto_land", True)
        self.declare_parameter("land_on_offboard_loss", True)
        self.declare_parameter("offboard_land", False)
        self.declare_parameter("offboard_land_speed_mps", 0.10)
        self.declare_parameter("offboard_land_auto_handoff_height_m", 0.10)
        self.declare_parameter("auto_disarm_after_land", True)
        self.declare_parameter("disarm_after_land_delay_s", 2.0)
        self.declare_parameter("land_wait_timeout_s", 45.0)
        self.declare_parameter("landed_ground_tolerance_m", 0.06)
        self.declare_parameter("target_reached_tolerance_m", 0.15)
        self.declare_parameter("landed_hold_seconds", 0.0)
        self.declare_parameter("drift_guard_enabled", True)
        self.declare_parameter("drift_warning_m", 0.20)
        self.declare_parameter("drift_land_m", 0.30)
        self.declare_parameter("drift_emergency_m", 0.50)
        self.declare_parameter("max_horizontal_drift_m", 0.5)
        self.declare_parameter("mavros_state_freshness_s", 1.5)
        self.declare_parameter("control_mode", "ramp")
        self.declare_parameter("mpc_dt", 0.05)
        self.declare_parameter("mpc_horizon_steps", 20)
        self.declare_parameter("mpc_max_vz", 0.30)
        self.declare_parameter("mpc_max_az", 0.50)
        self.declare_parameter("mpc_max_vxy", 0.30)
        self.declare_parameter("mpc_max_axy", 0.50)
        self.declare_parameter("mpc_xy_hold_weight", 2.0)
        self.declare_parameter("mpc_z_tracking_weight", 3.0)
        self.declare_parameter("mpc_velocity_weight", 1.5)
        self.declare_parameter("mpc_accel_weight", 0.2)
        self.declare_parameter("mpc_terminal_weight", 4.0)
        self.declare_parameter("enable_route_tracking", False)
        self.declare_parameter("route_path_topic", "/nav2_stage1/path")
        self.declare_parameter("route_fixed_flight_height_m", 1.0)
        self.declare_parameter("route_start_tolerance_m", 0.40)
        self.declare_parameter("route_goal_tolerance_m", 0.20)
        self.declare_parameter("route_lookahead_m", 0.40)
        self.declare_parameter("route_max_vxy_mps", 0.25)
        self.declare_parameter("route_max_vz_mps", 0.15)
        self.declare_parameter("route_setpoint_rate_hz", 30.0)
        self.declare_parameter("route_path_timeout_s", 2.0)
        self.declare_parameter("route_fail_action", "hold")
        self.declare_parameter("route_final_hold_seconds", 3.0)

        self.arm_only = bool(self.get_parameter("arm_only").value)
        self.takeoff_height_m = float(self.get_parameter("takeoff_height_m").value)
        self.hover_seconds = float(self.get_parameter("hover_seconds").value)
        self.prestream_count = int(self.get_parameter("prestream_count").value)
        self.offboard_stabilize_seconds = float(
            self.get_parameter("offboard_stabilize_seconds").value
        )
        self.use_rc_offboard = bool(self.get_parameter("use_rc_offboard").value)
        self.hold_after_arm_reject = bool(self.get_parameter("hold_after_arm_reject").value)
        self.ignore_z_in_arm_only = bool(self.get_parameter("ignore_z_in_arm_only").value)
        self.debug_hold_after_mode_drop = bool(self.get_parameter("debug_hold_after_mode_drop").value)
        self.recent_pose_samples = max(5, int(self.get_parameter("recent_pose_samples").value))
        self.require_vision_pose = bool(self.get_parameter("require_vision_pose").value)
        self.vision_freshness_s = max(0.1, float(self.get_parameter("vision_freshness_s").value))
        self.lift_only_seconds = max(0.0, float(self.get_parameter("lift_only_seconds").value))
        self.z_ramp_seconds = max(0.1, float(self.get_parameter("z_ramp_seconds").value))
        self.target_x_m = float(self.get_parameter("target_x_m").value)
        self.target_y_m = float(self.get_parameter("target_y_m").value)
        self.target_z_m = float(self.get_parameter("target_z_m").value)
        self.xy_move_unlock_height_m = max(
            0.0, float(self.get_parameter("xy_move_unlock_height_m").value)
        )
        self.yaw_deg = float(self.get_parameter("yaw_deg").value)
        self.auto_land = bool(self.get_parameter("auto_land").value)
        self.land_on_offboard_loss = bool(self.get_parameter("land_on_offboard_loss").value)
        self.offboard_land = bool(self.get_parameter("offboard_land").value)
        self.offboard_land_speed_mps = max(
            0.02, float(self.get_parameter("offboard_land_speed_mps").value)
        )
        self.offboard_land_auto_handoff_height_m = max(
            0.02, float(self.get_parameter("offboard_land_auto_handoff_height_m").value)
        )
        self.auto_disarm_after_land = bool(self.get_parameter("auto_disarm_after_land").value)
        self.disarm_after_land_delay_s = max(
            0.0, float(self.get_parameter("disarm_after_land_delay_s").value)
        )
        self.land_wait_timeout_s = max(
            5.0, float(self.get_parameter("land_wait_timeout_s").value)
        )
        self.landed_ground_tolerance_m = max(
            0.02, float(self.get_parameter("landed_ground_tolerance_m").value)
        )
        self.target_reached_tolerance_m = max(
            0.02, float(self.get_parameter("target_reached_tolerance_m").value)
        )
        self.landed_hold_seconds = max(0.0, float(self.get_parameter("landed_hold_seconds").value))
        self.drift_guard_enabled = bool(self.get_parameter("drift_guard_enabled").value)
        self.drift_warning_m = max(0.01, float(self.get_parameter("drift_warning_m").value))
        self.drift_land_m = max(0.05, float(self.get_parameter("drift_land_m").value))
        self.drift_emergency_m = max(0.05, float(self.get_parameter("drift_emergency_m").value))
        self.max_horizontal_drift_m = max(
            0.05, float(self.get_parameter("max_horizontal_drift_m").value)
        )
        self.mavros_state_freshness_s = max(
            0.2, float(self.get_parameter("mavros_state_freshness_s").value)
        )
        self.control_mode = str(self.get_parameter("control_mode").value).strip().lower()
        self.mpc_dt = max(0.01, float(self.get_parameter("mpc_dt").value))
        self.mpc_horizon_steps = max(1, int(self.get_parameter("mpc_horizon_steps").value))
        self.mpc_max_vz = max(0.02, float(self.get_parameter("mpc_max_vz").value))
        self.mpc_max_az = max(0.02, float(self.get_parameter("mpc_max_az").value))
        self.mpc_max_vxy = max(0.02, float(self.get_parameter("mpc_max_vxy").value))
        self.mpc_max_axy = max(0.02, float(self.get_parameter("mpc_max_axy").value))
        self.mpc_xy_hold_weight = max(0.01, float(self.get_parameter("mpc_xy_hold_weight").value))
        self.mpc_z_tracking_weight = max(
            0.01, float(self.get_parameter("mpc_z_tracking_weight").value)
        )
        self.mpc_velocity_weight = max(0.01, float(self.get_parameter("mpc_velocity_weight").value))
        self.mpc_accel_weight = max(0.0, float(self.get_parameter("mpc_accel_weight").value))
        self.mpc_terminal_weight = max(0.01, float(self.get_parameter("mpc_terminal_weight").value))
        self.enable_route_tracking = bool(self.get_parameter("enable_route_tracking").value)
        self.route_path_topic = str(self.get_parameter("route_path_topic").value)
        self.route_fixed_flight_height_m = max(
            0.05, float(self.get_parameter("route_fixed_flight_height_m").value)
        )
        self.route_start_tolerance_m = max(
            0.02, float(self.get_parameter("route_start_tolerance_m").value)
        )
        self.route_goal_tolerance_m = max(
            0.02, float(self.get_parameter("route_goal_tolerance_m").value)
        )
        self.route_lookahead_m = max(0.05, float(self.get_parameter("route_lookahead_m").value))
        self.route_max_vxy_mps = max(0.02, float(self.get_parameter("route_max_vxy_mps").value))
        self.route_max_vz_mps = max(0.02, float(self.get_parameter("route_max_vz_mps").value))
        self.route_setpoint_rate_hz = max(
            20.0, float(self.get_parameter("route_setpoint_rate_hz").value)
        )
        self.route_path_timeout_s = max(0.2, float(self.get_parameter("route_path_timeout_s").value))
        self.route_fail_action = str(self.get_parameter("route_fail_action").value).strip().lower()
        self.route_final_hold_seconds = max(
            0.0, float(self.get_parameter("route_final_hold_seconds").value)
        )
        if self.route_fail_action not in ("hold", "offboard_land"):
            self.get_logger().warning(
                f"Unknown route_fail_action={self.route_fail_action!r}; falling back to hold"
            )
            self.route_fail_action = "hold"
        if self.control_mode not in ("ramp", "mpc"):
            self.get_logger().warning(
                f"Unknown control_mode={self.control_mode!r}; falling back to ramp"
            )
            self.control_mode = "ramp"
        if self.drift_land_m <= self.drift_warning_m:
            self.drift_land_m = self.drift_warning_m + 0.05
        if self.drift_emergency_m <= self.drift_land_m:
            self.drift_emergency_m = self.drift_land_m + 0.10

        if self.control_mode == "mpc":
            self.setpoint_generator = MpcSetpointGenerator(
                dt=self.mpc_dt,
                horizon_steps=self.mpc_horizon_steps,
                max_vz=self.mpc_max_vz,
                max_az=self.mpc_max_az,
                max_vxy=self.mpc_max_vxy,
                max_axy=self.mpc_max_axy,
                xy_hold_weight=self.mpc_xy_hold_weight,
                z_tracking_weight=self.mpc_z_tracking_weight,
                velocity_weight=self.mpc_velocity_weight,
                accel_weight=self.mpc_accel_weight,
                terminal_weight=self.mpc_terminal_weight,
            )
        else:
            self.setpoint_generator = RampSetpointGenerator()

        self.route_tracker = RoutePathFollower(
            node=self,
            path_topic=self.route_path_topic,
            fixed_height_m=self.route_fixed_flight_height_m,
            start_tolerance_m=self.route_start_tolerance_m,
            goal_tolerance_m=self.route_goal_tolerance_m,
            lookahead_m=self.route_lookahead_m,
            max_vxy_mps=self.route_max_vxy_mps,
            max_vz_mps=self.route_max_vz_mps,
            path_timeout_s=self.route_path_timeout_s,
        )
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.setpoint_pub = self.create_publisher(PositionTarget, "/mavros/setpoint_raw/local", 10)
        self.mpc_debug_pub = self.create_publisher(
            PositionTarget, "/minipc_mavros_offboard/mpc_setpoint_debug", 10
        )
        self.create_subscription(State, "/mavros/state", self._state_cb, qos)
        self.create_subscription(
            ExtendedState,
            "/mavros/extended_state",
            self._extended_state_cb,
            qos,
        )
        self.create_subscription(PoseStamped, "/mavros/local_position/pose", self._pose_cb, qos)
        self.create_subscription(Odometry, "/mavros/local_position/odom", self._odom_cb, qos)
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/mavros/vision_pose/pose_cov",
            self._vision_pose_cb,
            qos,
        )

        self.arming_client = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.mode_client = self.create_client(SetMode, "/mavros/set_mode")

        self.state = State()
        self.extended_state = ExtendedState()
        self._last_offboard_state_ns: Optional[int] = None
        self._last_armed_state_ns: Optional[int] = None
        self._last_extended_state_ns: Optional[int] = None
        self.local_pose = PoseStamped()
        self.local_odom = Odometry()
        self._got_pose = False
        self._got_odom = False
        self._last_odom_ns: Optional[int] = None
        self._got_vision_pose = False
        self._last_vision_pose_ns: Optional[int] = None
        self._pose_samples = deque(maxlen=self.recent_pose_samples)
        self._odom_samples = deque(maxlen=self.recent_pose_samples)
        self._target_pose: Optional[PoseStamped] = None
        self._target_position_ned: Optional[tuple[float, float, float]] = None
        self._reference_pose: Optional[PoseStamped] = None
        self._final_target_z = 0.0

        self.phase = Phase.WAIT_FOR_CONNECTION
        self._phase_logged = None
        self._prestream_counter = 0
        self._mode_future = None
        self._arm_future = None
        self._disarm_future = None
        self._hold_start_ns = None
        self._target_reached_ns = None
        self._offboard_land_start_ns = None
        self._offboard_land_start_z = None
        self._landed_hold_start_ns = None
        self._land_request_done_ns = None
        self._landed_seen_ns = None
        self._landing_fallback_logged = False
        self._offboard_seen_since_ns = None
        self._hold_timeout_logged = False
        self._drift_guard_triggered = False
        self._drift_warning_logged = False
        self._drift_emergency_logged = False
        self._route_start_pending_logged = False
        self._route_start_aborted = False
        self._route_fail_count = 0
        self._route_fail_limit = 15
        self._route_final_hold_start_ns = None
        self._last_route_setpoint_ns: Optional[int] = None

        timer_period_s = min(0.05, 1.0 / self.route_setpoint_rate_hz)
        self.timer = self.create_timer(timer_period_s, self._timer_cb)

        self.get_logger().info(
            "Started minipc_mavros_offboard with "
            f"arm_only={self.arm_only}, takeoff_height_m={self.takeoff_height_m}, "
            f"hover_seconds={self.hover_seconds}, offboard_stabilize_seconds={self.offboard_stabilize_seconds}, "
            f"use_rc_offboard={self.use_rc_offboard}, hold_after_arm_reject={self.hold_after_arm_reject}, "
            f"ignore_z_in_arm_only={self.ignore_z_in_arm_only}, "
            f"debug_hold_after_mode_drop={self.debug_hold_after_mode_drop}, "
            f"recent_pose_samples={self.recent_pose_samples}, require_vision_pose={self.require_vision_pose}, "
            f"vision_freshness_s={self.vision_freshness_s}, lift_only_seconds={self.lift_only_seconds}, "
            f"z_ramp_seconds={self.z_ramp_seconds}, "
            f"target_xyz=({self.target_x_m}, {self.target_y_m}, {self.target_z_m}), "
            f"xy_move_unlock_height_m={self.xy_move_unlock_height_m}, "
            f"yaw_deg={self.yaw_deg}, auto_land={self.auto_land}, "
            f"land_on_offboard_loss={self.land_on_offboard_loss}, "
            f"offboard_land={self.offboard_land}, "
            f"offboard_land_speed_mps={self.offboard_land_speed_mps}, "
            f"offboard_land_auto_handoff_height_m={self.offboard_land_auto_handoff_height_m}, "
            f"auto_disarm_after_land={self.auto_disarm_after_land}, "
            f"disarm_after_land_delay_s={self.disarm_after_land_delay_s}, "
            f"land_wait_timeout_s={self.land_wait_timeout_s}, "
            f"landed_ground_tolerance_m={self.landed_ground_tolerance_m}, "
            f"target_reached_tolerance_m={self.target_reached_tolerance_m}, "
            f"landed_hold_seconds={self.landed_hold_seconds}, "
            f"drift_guard_enabled={self.drift_guard_enabled}, "
            f"drift_warning_m={self.drift_warning_m}, "
            f"drift_land_m={self.drift_land_m}, "
            f"drift_emergency_m={self.drift_emergency_m}, "
            f"legacy_max_horizontal_drift_m={self.max_horizontal_drift_m}, "
            f"mavros_state_freshness_s={self.mavros_state_freshness_s}, "
            f"control_mode={self.control_mode}, "
            f"enable_route_tracking={self.enable_route_tracking}, "
            f"route_path_topic={self.route_path_topic}, "
            f"route_fixed_flight_height_m={self.route_fixed_flight_height_m}, "
            f"route_start_tolerance_m={self.route_start_tolerance_m}, "
            f"route_goal_tolerance_m={self.route_goal_tolerance_m}, "
            f"route_lookahead_m={self.route_lookahead_m}, "
            f"route_max_vxy_mps={self.route_max_vxy_mps}, "
            f"route_max_vz_mps={self.route_max_vz_mps}, "
            f"route_setpoint_rate_hz={self.route_setpoint_rate_hz}, "
            f"route_path_timeout_s={self.route_path_timeout_s}, "
            f"route_fail_action={self.route_fail_action}, "
            f"route_final_hold_seconds={self.route_final_hold_seconds}"
        )
        if self.control_mode == "mpc":
            self.get_logger().info(
                "MPC parameters: "
                f"dt={self.mpc_dt}, horizon_steps={self.mpc_horizon_steps}, "
                f"max_vz={self.mpc_max_vz}, max_az={self.mpc_max_az}, "
                f"max_vxy={self.mpc_max_vxy}, max_axy={self.mpc_max_axy}, "
                f"xy_hold_weight={self.mpc_xy_hold_weight}, "
                f"z_tracking_weight={self.mpc_z_tracking_weight}, "
                f"velocity_weight={self.mpc_velocity_weight}, "
                f"accel_weight={self.mpc_accel_weight}, "
                f"terminal_weight={self.mpc_terminal_weight}"
            )

    def _state_cb(self, msg: State) -> None:
        self.state = msg
        now_ns = self.get_clock().now().nanoseconds
        if str(msg.mode).upper() == "OFFBOARD":
            self._last_offboard_state_ns = now_ns
        if msg.armed:
            self._last_armed_state_ns = now_ns

    def _extended_state_cb(self, msg: ExtendedState) -> None:
        self.extended_state = msg
        self._last_extended_state_ns = self.get_clock().now().nanoseconds

    def _pose_cb(self, msg: PoseStamped) -> None:
        self.local_pose = msg
        self._got_pose = True
        p = msg.pose.position
        if all(math.isfinite(v) for v in (p.x, p.y, p.z)):
            self._pose_samples.append((p.x, p.y, p.z))

    def _odom_cb(self, msg: Odometry) -> None:
        self.local_odom = msg
        self._got_odom = True
        self._last_odom_ns = self.get_clock().now().nanoseconds
        p = msg.pose.pose.position
        if all(math.isfinite(v) for v in (p.x, p.y, p.z)):
            self._odom_samples.append((p.x, p.y, p.z))

    def _vision_pose_cb(self, msg: PoseWithCovarianceStamped) -> None:
        p = msg.pose.pose.position
        if all(math.isfinite(v) for v in (p.x, p.y, p.z)):
            self._got_vision_pose = True
            self._last_vision_pose_ns = self.get_clock().now().nanoseconds

    def _log_phase(self, text: str) -> None:
        if self._phase_logged != text:
            self._phase_logged = text
            self.get_logger().info(text)

    def _connected(self) -> bool:
        return self.state.connected

    def _offboard_active(self) -> bool:
        if str(self.state.mode).upper() == "OFFBOARD":
            return True
        return self._recent_state_seen(self._last_offboard_state_ns)

    def _armed_active(self) -> bool:
        if self.state.armed:
            return True
        return self._recent_state_seen(self._last_armed_state_ns)

    def _recent_state_seen(self, seen_ns: Optional[int]) -> bool:
        if seen_ns is None:
            return False
        age_s = (self.get_clock().now().nanoseconds - seen_ns) / 1e9
        return age_s <= self.mavros_state_freshness_s

    def _extended_state_fresh(self) -> bool:
        return self._recent_state_seen(self._last_extended_state_ns)

    def _estimate_ready(self) -> bool:
        if not self._got_odom:
            return False

        p = self.local_odom.pose.pose.position
        if not (all(math.isfinite(v) for v in (p.x, p.y, p.z)) and len(self._odom_samples) >= 5):
            return False

        return self._vision_pose_fresh()

    def _vision_pose_fresh(self) -> bool:
        if not self.require_vision_pose:
            return True

        if not self._got_vision_pose or self._last_vision_pose_ns is None:
            return False

        age_s = (self.get_clock().now().nanoseconds - self._last_vision_pose_ns) / 1e9
        return age_s <= self.vision_freshness_s

    def _local_odom_fresh(self) -> bool:
        if not self._got_odom or self._last_odom_ns is None:
            return False
        age_s = (self.get_clock().now().nanoseconds - self._last_odom_ns) / 1e9
        return age_s <= self.vision_freshness_s

    def _averaged_odom_pose(self) -> PoseStamped:
        pose = PoseStamped()
        pose.header = self.local_odom.header
        pose.pose.orientation = self.local_odom.pose.pose.orientation

        xs = [sample[0] for sample in self._odom_samples]
        ys = [sample[1] for sample in self._odom_samples]
        zs = [sample[2] for sample in self._odom_samples]
        pose.pose.position.x = sum(xs) / len(xs)
        pose.pose.position.y = sum(ys) / len(ys)
        pose.pose.position.z = sum(zs) / len(zs)
        return pose

    def _ensure_target_pose(self) -> None:
        if self._target_pose is not None:
            return

        averaged_pose = self._averaged_odom_pose()
        self._reference_pose = averaged_pose
        current = averaged_pose.pose
        target = PoseStamped()
        target.header.frame_id = "map"
        delta_x_enu, delta_y_enu, delta_z_enu = ned_delta_to_enu(
            self.target_x_m, self.target_y_m, self.target_z_m
        )
        target.pose.position.x = current.position.x + delta_x_enu
        target.pose.position.y = current.position.y + delta_y_enu

        if abs(self.target_z_m) > 1e-6:
            self._final_target_z = current.position.z + delta_z_enu
        elif self.arm_only:
            self._final_target_z = current.position.z
        else:
            self._final_target_z = current.position.z + self.takeoff_height_m
        target.pose.position.z = current.position.z

        if abs(self.yaw_deg) > 1e-6:
            target.pose.orientation = yaw_to_quaternion(math.radians(self.yaw_deg))
        else:
            target.pose.orientation = current.orientation

        self._target_pose = target
        self._target_position_ned = (
            target.pose.position.x,
            target.pose.position.y,
            self._final_target_z,
        )
        self.get_logger().info(
            "Averaged MAVROS local odometry at lock "
            f"x={current.position.x:.2f}, y={current.position.y:.2f}, z={current.position.z:.2f}"
        )
        self.get_logger().info(
            "Target setpoint locked to ENU for MAVROS local setpoint "
            f"x={target.pose.position.x:.2f}, y={target.pose.position.y:.2f}, "
            f"z_start={target.pose.position.z:.2f}, z_final={self._final_target_z:.2f}"
        )

    def _publish_target_pose(self) -> None:
        if self._target_pose is None:
            return
        hold_elapsed = 0.0
        if self._hold_start_ns is not None:
            hold_elapsed = (self.get_clock().now().nanoseconds - self._hold_start_ns) / 1e9

        target_pose = self._target_pose
        if self._hold_start_ns is None and self._got_odom:
            target_pose = PoseStamped()
            target_pose.header = self._target_pose.header
            target_pose.pose = self._target_pose.pose
            target_pose.pose.orientation = self.local_odom.pose.pose.orientation

        kwargs = {
            "node": self,
            "target_pose": target_pose,
            "final_target_z": self._final_target_z,
            "arm_only": self.arm_only,
            "ignore_z_in_arm_only": self.ignore_z_in_arm_only,
            "hold_elapsed_s": hold_elapsed,
            "z_ramp_seconds": self.z_ramp_seconds,
        }
        if isinstance(self.setpoint_generator, MpcSetpointGenerator):
            kwargs["local_odom"] = self.local_odom if self._got_odom else None
        msg = self.setpoint_generator.make_target_setpoint(**kwargs)
        self._apply_xy_stage_gate(msg)
        self._publish_setpoint(msg)

    def _publish_offboard_land_pose(self) -> None:
        if self._target_pose is None or self._reference_pose is None:
            return

        now_ns = self.get_clock().now().nanoseconds
        if self._offboard_land_start_ns is None:
            self._offboard_land_start_ns = now_ns
            current_z = self.local_odom.pose.pose.position.z if self._got_odom else self._final_target_z
            self._offboard_land_start_z = float(current_z)
            self.get_logger().info(
                "Starting OFFBOARD controlled landing: "
                f"z_start={self._offboard_land_start_z:.2f}, "
                f"z_end={self._reference_pose.pose.position.z:.2f}, "
                f"speed={self.offboard_land_speed_mps:.2f} m/s"
            )

        elapsed = (now_ns - self._offboard_land_start_ns) / 1e9
        kwargs = {
            "node": self,
            "target_pose": self._target_pose,
            "reference_pose": self._reference_pose,
            "start_z": float(self._offboard_land_start_z),
            "elapsed_s": elapsed,
            "land_speed_mps": self.offboard_land_speed_mps,
        }
        if isinstance(self.setpoint_generator, MpcSetpointGenerator):
            kwargs["local_odom"] = self.local_odom if self._got_odom else None
        msg = self.setpoint_generator.make_landing_setpoint(**kwargs)
        self._publish_setpoint(msg)

    def _publish_setpoint(self, msg: PositionTarget) -> None:
        self.setpoint_pub.publish(msg)
        if self.control_mode == "mpc":
            self.mpc_debug_pub.publish(msg)

    def _apply_xy_stage_gate(self, msg: PositionTarget) -> None:
        if (
            self._target_pose is None
            or self._reference_pose is None
            or self.xy_move_unlock_height_m <= 0.0
            or not self._got_odom
        ):
            return

        target_xy_offset = vector_norm(
            self._target_pose.pose.position.x - self._reference_pose.pose.position.x,
            self._target_pose.pose.position.y - self._reference_pose.pose.position.y,
        )
        if target_xy_offset <= 1e-6:
            return

        current_z = float(self.local_odom.pose.pose.position.z)
        gate_z = float(self._reference_pose.pose.position.z) + self.xy_move_unlock_height_m
        if current_z >= gate_z:
            return

        msg.position.x = float(self._reference_pose.pose.position.x)
        msg.position.y = float(self._reference_pose.pose.position.y)

        if msg.type_mask & PositionTarget.IGNORE_VX == 0:
            msg.velocity.x = 0.0
        if msg.type_mask & PositionTarget.IGNORE_VY == 0:
            msg.velocity.y = 0.0
        if msg.type_mask & PositionTarget.IGNORE_AFX == 0:
            msg.acceleration_or_force.x = 0.0
        if msg.type_mask & PositionTarget.IGNORE_AFY == 0:
            msg.acceleration_or_force.y = 0.0

    def _target_reached(self) -> bool:
        if not self._got_odom:
            return False

        p = self.local_odom.pose.pose.position
        error_z = abs(p.z - self._final_target_z)
        horizontal_error = self._horizontal_error_from_target()
        if horizontal_error is None:
            return error_z <= self.target_reached_tolerance_m

        return (
            error_z <= self.target_reached_tolerance_m
            and horizontal_error <= self.drift_warning_m
        )

    def _offboard_land_auto_handoff_reached(self) -> bool:
        if not self._got_odom or self._reference_pose is None:
            return False

        current_z = self.local_odom.pose.pose.position.z
        ground_z = self._reference_pose.pose.position.z
        return abs(current_z - ground_z) <= self.offboard_land_auto_handoff_height_m

    def _local_altitude_at_ground(self) -> bool:
        if not self._got_odom or self._reference_pose is None:
            return False
        current_z = float(self.local_odom.pose.pose.position.z)
        ground_z = float(self._reference_pose.pose.position.z)
        return abs(current_z - ground_z) <= self.landed_ground_tolerance_m

    def _landed_ready_for_disarm(self) -> bool:
        if self._land_request_done_ns is None:
            return False

        elapsed = (self.get_clock().now().nanoseconds - self._land_request_done_ns) / 1e9
        if self._extended_state_fresh():
            if self.extended_state.landed_state == ExtendedState.LANDED_STATE_ON_GROUND:
                return True
            if elapsed < self.land_wait_timeout_s or not self._local_altitude_at_ground():
                return False
            if not self._landing_fallback_logged:
                self._landing_fallback_logged = True
                self.get_logger().warning(
                    "MAVROS extended_state has not reported ON_GROUND after timeout "
                    f"(landed_state={self.extended_state.landed_state}); using local altitude near ground as disarm fallback"
                )
            return True

        if elapsed < self.land_wait_timeout_s:
            return False
        if self._local_altitude_at_ground():
            if not self._landing_fallback_logged:
                self._landing_fallback_logged = True
                self.get_logger().warning(
                    "MAVROS extended_state is unavailable/stale; using local altitude near ground as disarm fallback"
                )
            return True
        return False

    def _request_offboard(self) -> None:
        req = SetMode.Request()
        req.base_mode = 0
        req.custom_mode = "OFFBOARD"
        self._mode_future = self.mode_client.call_async(req)
        self.get_logger().info("Requested OFFBOARD mode")

    def _request_arm(self) -> None:
        req = CommandBool.Request()
        req.value = True
        self._arm_future = self.arming_client.call_async(req)
        self.get_logger().info("Requested arming")

    def _request_disarm(self) -> None:
        req = CommandBool.Request()
        req.value = False
        self._disarm_future = self.arming_client.call_async(req)
        self.get_logger().info("Requested disarm after landing")

    def _start_hold_phase(self) -> None:
        if self._target_pose is not None and self._got_odom:
            self._target_pose.pose.orientation = self.local_odom.pose.pose.orientation
            yaw = RampSetpointGenerator._yaw_from_pose(self._target_pose)
            self.get_logger().info(f"Locked hold yaw to current MAVROS local yaw {math.degrees(yaw):.1f} deg")
        self._hold_start_ns = self.get_clock().now().nanoseconds
        self._target_reached_ns = None
        self._hold_timeout_logged = False
        self.phase = Phase.HOLD

    def _request_land(self) -> None:
        req = SetMode.Request()
        req.base_mode = 0
        req.custom_mode = "AUTO.LAND"
        self._mode_future = self.mode_client.call_async(req)
        self.get_logger().info("Requested AUTO.LAND mode")

    def _current_yaw(self) -> float:
        if self._target_pose is not None:
            return RampSetpointGenerator._yaw_from_pose(self._target_pose)
        if self._got_odom:
            q = self.local_odom.pose.pose.orientation
            return math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            )
        return math.radians(self.yaw_deg)

    def _start_route_tracking_or_finish_hold(self) -> None:
        if not self.enable_route_tracking:
            self._go_to_post_hover_landing()
            return
        if self._route_start_aborted:
            if not self._hold_timeout_logged:
                self._hold_timeout_logged = True
                self.get_logger().warning(
                    "Route tracking startup was aborted earlier; continuing to hold setpoint"
                )
            return

        current_map_xy = self.route_tracker.lookup_map_position(self.tf_buffer)
        if current_map_xy is None:
            self.get_logger().warning(
                f"Route tracking not started: {self.route_tracker.last_error}"
            )
            self._handle_route_failure(startup=True)
            return

        p = self.local_odom.pose.pose.position
        current_local_xy = (float(p.x), float(p.y))
        if not self.route_tracker.prepare_start(
            current_map_xy=current_map_xy,
            current_local_xy=current_local_xy,
        ):
            self.get_logger().warning(
                f"Route tracking not started: {self.route_tracker.last_error}"
            )
            self._handle_route_failure(startup=True)
            return

        self._route_fail_count = 0
        self._route_final_hold_start_ns = None
        self._last_route_setpoint_ns = self.get_clock().now().nanoseconds
        self.get_logger().info("Route tracking enabled; entering ROUTE_TRACKING")
        self.phase = Phase.ROUTE_TRACKING

    def _go_to_post_hover_landing(self) -> None:
        if self.offboard_land:
            self.phase = Phase.OFFBOARD_LAND
        elif self.auto_land:
            self.phase = Phase.REQUEST_LAND
        else:
            if not self._hold_timeout_logged:
                self._hold_timeout_logged = True
                self.get_logger().info(
                    "Hover timeout reached; continuing to hold setpoint until OFFBOARD/armed state changes or node is interrupted"
                )

    def _handle_route_failure(self, *, startup: bool = False) -> None:
        if self.route_fail_action == "offboard_land":
            self.get_logger().error(
                f"Route tracking failure; route_fail_action=offboard_land: {self.route_tracker.last_error}"
            )
            self.phase = Phase.OFFBOARD_LAND if self.offboard_land else Phase.REQUEST_LAND
            return

        if startup:
            self._route_start_aborted = True
            self.get_logger().warning(
                "Route tracking unavailable at startup; holding current takeoff setpoint because route_fail_action=hold"
            )
            self._hold_timeout_logged = True
            self.phase = Phase.HOLD
            return

        self._route_fail_count += 1
        if self._got_odom:
            self._publish_setpoint(
                self.route_tracker.make_hold_setpoint(
                    current_local_odom=self.local_odom,
                    yaw=self._current_yaw(),
                )
            )
        if self._route_fail_count >= self._route_fail_limit:
            self.get_logger().error(
                "Route tracking had consecutive failures; entering OFFBOARD land path"
            )
            self.phase = Phase.OFFBOARD_LAND if self.offboard_land else Phase.REQUEST_LAND

    def _publish_route_tracking_setpoint(self) -> None:
        if not self._local_odom_fresh():
            self.route_tracker.last_error = "MAVROS local odometry is stale during route tracking"
            self._handle_route_failure()
            return
        if not self._vision_pose_fresh():
            self.route_tracker.last_error = "MAVROS vision pose is stale during route tracking"
            self._handle_route_failure()
            return

        current_map_xy = self.route_tracker.lookup_map_position(self.tf_buffer)
        if current_map_xy is None:
            self._handle_route_failure()
            return

        now_ns = self.get_clock().now().nanoseconds
        if self._last_route_setpoint_ns is None:
            dt_s = 1.0 / self.route_setpoint_rate_hz
        else:
            dt_s = max(0.001, (now_ns - self._last_route_setpoint_ns) / 1e9)
            if dt_s > 0.05:
                self.get_logger().warning(
                    f"Route setpoint interval {dt_s:.3f}s is below 20 Hz requirement"
                )
        self._last_route_setpoint_ns = now_ns

        try:
            msg, finished = self.route_tracker.make_setpoint(
                current_map_xy=current_map_xy,
                current_local_odom=self.local_odom,
                yaw=self._current_yaw(),
                dt_s=dt_s,
            )
        except Exception as exc:  # noqa: BLE001
            self.route_tracker.last_error = f"route tracker exception: {exc}"
            self.get_logger().warning(self.route_tracker.last_error)
            self._handle_route_failure()
            return

        self._route_fail_count = 0
        self._publish_setpoint(msg)

        if finished:
            self.get_logger().info("Route goal reached; entering FINAL_HOLD")
            self._route_final_hold_start_ns = self.get_clock().now().nanoseconds
            self.phase = Phase.FINAL_HOLD

    def _horizontal_error_from_target(self) -> Optional[float]:
        if self._target_pose is None or not self._got_odom:
            return None

        target = self._target_pose.pose.position
        current = self.local_odom.pose.pose.position
        if not all(math.isfinite(v) for v in (target.x, target.y, current.x, current.y)):
            return None

        dx = current.x - target.x
        dy = current.y - target.y
        return math.hypot(dx, dy)

    def _drift_guard_allows_hold(self) -> bool:
        if not self.drift_guard_enabled:
            return True

        error_m = self._horizontal_error_from_target()
        if error_m is None:
            return True

        if error_m >= self.drift_emergency_m and not self._drift_emergency_logged:
            self._drift_emergency_logged = True
            self.get_logger().fatal(
                "Horizontal error emergency threshold exceeded: "
                f"error={error_m:.2f} m >= emergency={self.drift_emergency_m:.2f} m. "
                "Manual takeover recommended immediately."
            )

        if error_m >= self.drift_warning_m and not self._drift_warning_logged:
            self._drift_warning_logged = True
            self.get_logger().warning(
                "Horizontal error warning: "
                f"error={error_m:.2f} m >= warning={self.drift_warning_m:.2f} m"
            )

        if self._drift_guard_triggered or error_m < self.drift_land_m:
            return True

        self._drift_guard_triggered = True
        self.get_logger().error(
            "Horizontal drift guard triggered: "
            f"error={error_m:.2f} m >= limit={self.drift_land_m:.2f} m. "
            "Automatic landing on drift is disabled; continuing to publish setpoints."
        )
        return True

    def _log_mavros_state(self, prefix: str) -> None:
        self.get_logger().info(
            f"{prefix}: connected={self.state.connected}, armed={self.state.armed}, "
            f"guided={self.state.guided}, manual_input={self.state.manual_input}, mode={self.state.mode}"
        )

    def _debug_mode_drop(self) -> None:
        self.get_logger().info(
            f"Debug hold after OFFBOARD/armed drop: connected={self.state.connected}, armed={self.state.armed}, "
            f"guided={self.state.guided}, manual_input={self.state.manual_input}, mode={self.state.mode}"
        )

    def _timer_cb(self) -> None:
        if self.phase == Phase.WAIT_FOR_CONNECTION:
            self._log_phase("Waiting for MAVROS connection")
            if self._connected():
                self.phase = Phase.WAIT_FOR_LOCAL_POSE
            return

        if self.phase == Phase.WAIT_FOR_LOCAL_POSE:
            self._log_phase("Waiting for MAVROS local odometry before arming")
            if self._estimate_ready():
                self._ensure_target_pose()
                self.phase = Phase.STREAM_SETPOINTS
            return

        if self.phase == Phase.STREAM_SETPOINTS:
            self._log_phase("Streaming pre-arm MAVROS setpoints")
            self._publish_target_pose()
            self._prestream_counter += 1
            if self._prestream_counter >= self.prestream_count:
                self._offboard_seen_since_ns = None
                self.phase = Phase.WAIT_FOR_RC_OFFBOARD if self.use_rc_offboard else Phase.REQUEST_MODE
            return

        if self.phase == Phase.WAIT_FOR_RC_OFFBOARD:
            self._log_phase("Streaming setpoints and waiting for RC to switch OFFBOARD")
            self._publish_target_pose()

            if not self._offboard_active():
                self._offboard_seen_since_ns = None
                return

            now_ns = self.get_clock().now().nanoseconds
            if self._offboard_seen_since_ns is None:
                self._offboard_seen_since_ns = now_ns
                self.get_logger().info("OFFBOARD reported by MAVROS state")
                return

            elapsed = (now_ns - self._offboard_seen_since_ns) / 1e9
            if elapsed >= self.offboard_stabilize_seconds:
                self.phase = Phase.REQUEST_ARM
            return

        if self.phase == Phase.REQUEST_MODE:
            self._log_phase("Switching to OFFBOARD")
            self._publish_target_pose()

            if self._mode_future is None:
                if not self.mode_client.wait_for_service(timeout_sec=1.0):
                    self.get_logger().warning("Waiting for /mavros/set_mode service")
                    return
                self._request_offboard()
                return

            if not self._mode_future.done():
                return

            result = self._mode_future.result()
            if result is None:
                raise RuntimeError("OFFBOARD mode request failed: empty response")

            if not result.mode_sent:
                raise RuntimeError(f"OFFBOARD mode request rejected: {result}")

            self._mode_future = None
            self._offboard_seen_since_ns = None
            self.phase = Phase.WAIT_FOR_OFFBOARD
            return

        if self.phase == Phase.WAIT_FOR_OFFBOARD:
            self._log_phase("Waiting for OFFBOARD mode to become active")
            self._publish_target_pose()

            if not self._offboard_active():
                self._offboard_seen_since_ns = None
                return

            now_ns = self.get_clock().now().nanoseconds
            if self._offboard_seen_since_ns is None:
                self._offboard_seen_since_ns = now_ns
                self.get_logger().info("OFFBOARD reported by MAVROS state")
                return

            elapsed = (now_ns - self._offboard_seen_since_ns) / 1e9
            if elapsed >= self.offboard_stabilize_seconds:
                self.phase = Phase.REQUEST_ARM
            return

        if self.phase == Phase.REQUEST_ARM:
            self._log_phase("Arming vehicle")
            self._publish_target_pose()

            if not self._vision_pose_fresh():
                self.get_logger().warning(
                    "Waiting for fresh MAVROS vision pose before arming"
                )
                return

            if self._arm_future is None:
                if not self.arming_client.wait_for_service(timeout_sec=1.0):
                    self.get_logger().warning("Waiting for /mavros/cmd/arming service")
                    return
                self._request_arm()
                return

            if not self._arm_future.done():
                return

            result = self._arm_future.result()
            if result is None:
                raise RuntimeError("Arm request failed: empty response")

            if not result.success:
                self._log_mavros_state("MAVROS state before/at arm reject")
                self.get_logger().error(f"Arm request rejected: {result}")
                self._arm_future = None

                if self._armed_active() and self._offboard_active():
                    self.get_logger().warning(
                        "Arm service response was rejected, but MAVROS state is already armed/OFFBOARD; continuing to HOLD"
                    )
                    self._start_hold_phase()
                    return

                if self.hold_after_arm_reject:
                    self.phase = Phase.ARM_REJECTED
                    return

                raise RuntimeError(f"Arm request rejected: {result}")

            self._arm_future = None
            self._start_hold_phase()
            return

        if self.phase == Phase.ARM_REJECTED:
            self._log_phase("Arm rejected; continuing to hold setpoint for debugging")
            self._publish_target_pose()
            if self._armed_active() and self._offboard_active():
                self.get_logger().warning(
                    "Vehicle became armed/OFFBOARD after arm rejection; continuing to HOLD"
                )
                self._start_hold_phase()
            return

        if self.phase == Phase.HOLD:
            self._log_phase("Holding MAVROS setpoint")
            if not self._drift_guard_allows_hold():
                return
            self._publish_target_pose()

            if self.arm_only:
                return

            if not self._armed_active() or not self._offboard_active():
                if self._armed_active() and self.land_on_offboard_loss:
                    self._log_mavros_state(
                        "Vehicle left OFFBOARD while armed; requesting AUTO.LAND"
                    )
                    self._mode_future = None
                    self.phase = Phase.REQUEST_LAND
                    return

                if self.debug_hold_after_mode_drop:
                    self._log_phase("Vehicle left armed/OFFBOARD state; continuing debug hold")
                    self._debug_mode_drop()
                    return
                self._log_mavros_state("Hold ended because vehicle left armed/OFFBOARD state")
                self.phase = Phase.DONE
                return

            now_ns = self.get_clock().now().nanoseconds
            if self._target_reached_ns is None:
                if self._target_reached():
                    self._target_reached_ns = now_ns
                    self.get_logger().info(
                        "Target reached; starting hover timer before landing"
                    )
                return

            elapsed = (now_ns - self._target_reached_ns) / 1e9
            if elapsed >= self.hover_seconds:
                self._start_route_tracking_or_finish_hold()
            return

        if self.phase == Phase.ROUTE_TRACKING:
            self._log_phase("Tracking Nav2 stage1 path with MAVROS setpoints")

            if not self._armed_active() or not self._offboard_active():
                if self._armed_active() and self.land_on_offboard_loss:
                    self._log_mavros_state(
                        "Vehicle left OFFBOARD during route tracking; requesting AUTO.LAND"
                    )
                    self._mode_future = None
                    self.phase = Phase.REQUEST_LAND
                    return

                if self.debug_hold_after_mode_drop:
                    self._log_phase("Vehicle left armed/OFFBOARD state during route tracking")
                    self._debug_mode_drop()
                    return
                self._log_mavros_state("Route tracking ended because vehicle left armed/OFFBOARD state")
                self.phase = Phase.DONE
                return

            self._publish_route_tracking_setpoint()
            return

        if self.phase == Phase.FINAL_HOLD:
            self._log_phase("Holding final route setpoint before landing")

            if not self._armed_active() or not self._offboard_active():
                if self._armed_active() and self.land_on_offboard_loss:
                    self._log_mavros_state(
                        "Vehicle left OFFBOARD during route final hold; requesting AUTO.LAND"
                    )
                    self._mode_future = None
                    self.phase = Phase.REQUEST_LAND
                    return
                if self.debug_hold_after_mode_drop:
                    self._debug_mode_drop()
                    return
                self.phase = Phase.DONE
                return

            if self._got_odom:
                self._publish_setpoint(
                    self.route_tracker.make_hold_setpoint(
                        current_local_odom=self.local_odom,
                        yaw=self._current_yaw(),
                    )
                )

            if self._route_final_hold_start_ns is None:
                self._route_final_hold_start_ns = self.get_clock().now().nanoseconds
                return

            elapsed = (self.get_clock().now().nanoseconds - self._route_final_hold_start_ns) / 1e9
            if elapsed >= self.route_final_hold_seconds:
                self._go_to_post_hover_landing()
            return

        if self.phase == Phase.OFFBOARD_LAND:
            self._log_phase("Landing with OFFBOARD position setpoints")
            self._publish_offboard_land_pose()

            if not self._armed_active() or not self._offboard_active():
                if self.debug_hold_after_mode_drop:
                    self._log_phase("Vehicle left armed/OFFBOARD state during OFFBOARD landing")
                    self._debug_mode_drop()
                    return
                self._log_mavros_state("OFFBOARD landing ended because vehicle left armed/OFFBOARD state")
                self.phase = Phase.DONE
                return

            if self._offboard_land_auto_handoff_reached():
                self.get_logger().info(
                    "OFFBOARD landing reached AUTO.LAND handoff height; requesting AUTO.LAND"
                )
                self._mode_future = None
                self.phase = Phase.REQUEST_LAND
            return

        if self.phase == Phase.LANDED_HOLD:
            self._log_phase("OFFBOARD landing completed; holding landed setpoint")
            self._publish_offboard_land_pose()

            if self.landed_hold_seconds <= 0.0:
                return

            elapsed = (self.get_clock().now().nanoseconds - self._landed_hold_start_ns) / 1e9
            if elapsed >= self.landed_hold_seconds:
                self.phase = Phase.DONE
            return

        if self.phase == Phase.REQUEST_LAND:
            self._log_phase("Landing")
            if self._mode_future is None:
                self._request_land()
                return

            if not self._mode_future.done():
                return

            result = self._mode_future.result()
            if result is None or not result.mode_sent:
                raise RuntimeError("AUTO.LAND request failed")

            self._mode_future = None
            self._land_request_done_ns = self.get_clock().now().nanoseconds
            self._landed_seen_ns = None
            self._landing_fallback_logged = False
            self.phase = Phase.WAIT_FOR_LAND
            return

        if self.phase == Phase.WAIT_FOR_LAND:
            self._log_phase("Waiting for landing completion before disarm")
            if not self.state.armed:
                self.get_logger().info("Vehicle is already disarmed after AUTO.LAND")
                self.phase = Phase.DONE
                return

            if not self.auto_disarm_after_land:
                return

            now_ns = self.get_clock().now().nanoseconds
            if not self._landed_ready_for_disarm():
                return

            if self._landed_seen_ns is None:
                self._landed_seen_ns = now_ns
                self.get_logger().info(
                    "Landing detected; waiting before disarm request"
                )
                return

            elapsed = (now_ns - self._landed_seen_ns) / 1e9
            if elapsed >= self.disarm_after_land_delay_s:
                self.phase = Phase.REQUEST_DISARM
            return

        if self.phase == Phase.REQUEST_DISARM:
            self._log_phase("Disarming after landing")
            if not self.state.armed:
                self.phase = Phase.DONE
                return

            if self._disarm_future is None:
                if not self.arming_client.wait_for_service(timeout_sec=1.0):
                    self.get_logger().warning("Waiting for /mavros/cmd/arming service to disarm")
                    return
                self._request_disarm()
                return

            if not self._disarm_future.done():
                return

            result = self._disarm_future.result()
            self._disarm_future = None
            if result is None or not result.success:
                self.get_logger().warning(f"Disarm request rejected or empty: {result}; retrying")
                return

            self.phase = Phase.WAIT_FOR_DISARM
            return

        if self.phase == Phase.WAIT_FOR_DISARM:
            self._log_phase("Waiting for disarm confirmation")
            if not self.state.armed:
                self.get_logger().info("Vehicle disarmed; motors should be stopped")
                self.phase = Phase.DONE
                return
            return

        if self.phase == Phase.DONE:
            self._log_phase("MAVROS offboard control completed; stopping node")
            raise SystemExit(0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MiniPcMavrosOffboard()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:  # noqa: BLE001
        print(exc, file=sys.stderr)
        sys.exit(1)
