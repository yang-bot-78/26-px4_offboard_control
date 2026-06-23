#!/usr/bin/env python3

import math
import sys
from collections import deque
from enum import Enum, auto
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import CommandBool, SetMode
from nav_msgs.msg import Odometry
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
    REQUEST_LAND = auto()
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
        self.declare_parameter("yaw_deg", 0.0)
        self.declare_parameter("auto_land", True)
        self.declare_parameter("drift_guard_enabled", True)
        self.declare_parameter("max_horizontal_drift_m", 0.5)
        self.declare_parameter("drift_guard_action", "land")

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
        self.yaw_deg = float(self.get_parameter("yaw_deg").value)
        self.auto_land = bool(self.get_parameter("auto_land").value)
        self.drift_guard_enabled = bool(self.get_parameter("drift_guard_enabled").value)
        self.max_horizontal_drift_m = max(
            0.05, float(self.get_parameter("max_horizontal_drift_m").value)
        )
        self.drift_guard_action = str(self.get_parameter("drift_guard_action").value).lower()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.setpoint_pub = self.create_publisher(PositionTarget, "/mavros/setpoint_raw/local", 10)
        self.create_subscription(State, "/mavros/state", self._state_cb, qos)
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
        self.local_pose = PoseStamped()
        self.local_odom = Odometry()
        self._got_pose = False
        self._got_odom = False
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
        self._hold_start_ns = None
        self._offboard_seen_since_ns = None
        self._hold_timeout_logged = False
        self._drift_guard_triggered = False

        self.timer = self.create_timer(0.05, self._timer_cb)

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
            f"yaw_deg={self.yaw_deg}, auto_land={self.auto_land}, "
            f"drift_guard_enabled={self.drift_guard_enabled}, "
            f"max_horizontal_drift_m={self.max_horizontal_drift_m}, "
            f"drift_guard_action={self.drift_guard_action}"
        )

    def _state_cb(self, msg: State) -> None:
        self.state = msg

    def _pose_cb(self, msg: PoseStamped) -> None:
        self.local_pose = msg
        self._got_pose = True
        p = msg.pose.position
        if all(math.isfinite(v) for v in (p.x, p.y, p.z)):
            self._pose_samples.append((p.x, p.y, p.z))

    def _odom_cb(self, msg: Odometry) -> None:
        self.local_odom = msg
        self._got_odom = True
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
        return str(self.state.mode).upper() == "OFFBOARD"

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
        msg = PositionTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
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

        hold_elapsed = 0.0
        if self._hold_start_ns is not None:
            hold_elapsed = (self.get_clock().now().nanoseconds - self._hold_start_ns) / 1e9

        if self.arm_only and self.ignore_z_in_arm_only:
            msg.type_mask |= PositionTarget.IGNORE_PZ

        msg.position.x = float(self._target_pose.pose.position.x)
        msg.position.y = float(self._target_pose.pose.position.y)

        if self.arm_only:
            z_cmd = self._target_pose.pose.position.z
        else:
            z_start = self._target_pose.pose.position.z
            alpha = min(max(hold_elapsed / self.z_ramp_seconds, 0.0), 1.0)
            z_cmd = z_start + alpha * (self._final_target_z - z_start)
        msg.position.z = float(z_cmd)

        q = self._target_pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        msg.yaw = float(yaw)
        self.setpoint_pub.publish(msg)

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

    def _request_land(self) -> None:
        req = SetMode.Request()
        req.base_mode = 0
        req.custom_mode = "AUTO.LAND"
        self._mode_future = self.mode_client.call_async(req)
        self.get_logger().info("Requested AUTO.LAND mode")

    def _horizontal_drift_from_reference(self) -> Optional[float]:
        if self._reference_pose is None or not self._got_odom:
            return None

        ref = self._reference_pose.pose.position
        current = self.local_odom.pose.pose.position
        if not all(math.isfinite(v) for v in (ref.x, ref.y, current.x, current.y)):
            return None

        dx = current.x - ref.x
        dy = current.y - ref.y
        return math.hypot(dx, dy)

    def _drift_guard_allows_hold(self) -> bool:
        if not self.drift_guard_enabled or self._drift_guard_triggered:
            return True

        drift_m = self._horizontal_drift_from_reference()
        if drift_m is None or drift_m <= self.max_horizontal_drift_m:
            return True

        self._drift_guard_triggered = True
        self.get_logger().error(
            "Horizontal drift guard triggered: "
            f"drift={drift_m:.2f} m > limit={self.max_horizontal_drift_m:.2f} m. "
            f"action={self.drift_guard_action}"
        )

        if self.state.armed and self._offboard_active() and self.drift_guard_action != "exit":
            self._mode_future = None
            self.phase = Phase.REQUEST_LAND
        else:
            self.phase = Phase.DONE
        return False

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

                if self.hold_after_arm_reject:
                    self.phase = Phase.ARM_REJECTED
                    return

                raise RuntimeError(f"Arm request rejected: {result}")

            self._arm_future = None
            self._hold_start_ns = self.get_clock().now().nanoseconds
            self._hold_timeout_logged = False
            self.phase = Phase.HOLD
            return

        if self.phase == Phase.ARM_REJECTED:
            self._log_phase("Arm rejected; continuing to hold setpoint for debugging")
            self._publish_target_pose()
            return

        if self.phase == Phase.HOLD:
            self._log_phase("Holding MAVROS setpoint")
            if not self._drift_guard_allows_hold():
                return
            self._publish_target_pose()

            if self.arm_only:
                return

            if not self.state.armed or not self._offboard_active():
                if self.debug_hold_after_mode_drop:
                    self._log_phase("Vehicle left armed/OFFBOARD state; continuing debug hold")
                    self._debug_mode_drop()
                    return
                self._log_mavros_state("Hold ended because vehicle left armed/OFFBOARD state")
                self.phase = Phase.DONE
                return

            elapsed = (self.get_clock().now().nanoseconds - self._hold_start_ns) / 1e9
            if elapsed >= self.hover_seconds:
                if self.auto_land:
                    self.phase = Phase.REQUEST_LAND
                else:
                    if not self._hold_timeout_logged:
                        self._hold_timeout_logged = True
                        self.get_logger().info(
                            "Hover timeout reached; continuing to hold setpoint until OFFBOARD/armed state changes or node is interrupted"
                        )
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

            self.phase = Phase.DONE
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
