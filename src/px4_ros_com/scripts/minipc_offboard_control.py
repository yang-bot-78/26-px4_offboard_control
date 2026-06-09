#!/usr/bin/env python3

import math
import sys
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus


class FlightPhase(Enum):
    WAIT_FOR_ESTIMATE = auto()
    STREAM_SETPOINTS = auto()
    ARM_AND_OFFBOARD = auto()
    HOVER = auto()
    LAND = auto()
    DONE = auto()


class MiniPcOffboardControl(Node):
    def __init__(self) -> None:
        super().__init__("minipc_offboard_control")

        self.declare_parameter("arm_only", True)
        self.declare_parameter("takeoff_height_m", 0.8)
        self.declare_parameter("hover_seconds", 10.0)
        self.declare_parameter("prestream_count", 20)
        self.declare_parameter("target_x_m", 0.0)
        self.declare_parameter("target_y_m", 0.0)
        self.declare_parameter("yaw_deg", 0.0)
        self.declare_parameter("auto_land", True)

        self.arm_only = self.get_parameter("arm_only").value
        self.takeoff_height_m = float(self.get_parameter("takeoff_height_m").value)
        self.hover_seconds = float(self.get_parameter("hover_seconds").value)
        self.prestream_count = int(self.get_parameter("prestream_count").value)
        self.target_x_m = float(self.get_parameter("target_x_m").value)
        self.target_y_m = float(self.get_parameter("target_y_m").value)
        self.yaw_rad = math.radians(float(self.get_parameter("yaw_deg").value))
        self.auto_land = bool(self.get_parameter("auto_land").value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", qos
        )
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos
        )
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", qos
        )

        self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position",
            self._vehicle_local_position_cb,
            qos,
        )
        self.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_status",
            self._vehicle_status_cb,
            qos,
        )

        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self._got_local_position = False
        self._got_vehicle_status = False

        self.phase = FlightPhase.WAIT_FOR_ESTIMATE
        self._phase_logged = None
        self._prestream_sent = 0
        self._command_sent = False
        self._hover_start_us = None
        self._target_position = None

        self.timer = self.create_timer(0.1, self._timer_cb)

        self.get_logger().info(
            "Started minipc_offboard_control with "
            f"arm_only={self.arm_only}, takeoff_height_m={self.takeoff_height_m}, "
            f"hover_seconds={self.hover_seconds}, target_xy=({self.target_x_m}, {self.target_y_m}), "
            f"yaw_deg={math.degrees(self.yaw_rad):.1f}, auto_land={self.auto_land}"
        )

    def _vehicle_local_position_cb(self, msg: VehicleLocalPosition) -> None:
        self.vehicle_local_position = msg
        self._got_local_position = True

    def _vehicle_status_cb(self, msg: VehicleStatus) -> None:
        self.vehicle_status = msg
        self._got_vehicle_status = True

    def _log_phase(self, text: str) -> None:
        if self._phase_logged != text:
            self._phase_logged = text
            self.get_logger().info(text)

    def _estimate_ready(self) -> bool:
        lp = self.vehicle_local_position
        return (
            self._got_local_position
            and self._got_vehicle_status
            and lp.xy_valid
            and lp.z_valid
            and lp.v_xy_valid
            and lp.v_z_valid
            and lp.heading_good_for_control
            and not lp.dead_reckoning
        )

    def _publish_offboard_heartbeat(self) -> None:
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self._now_us()
        self.offboard_control_mode_pub.publish(msg)

    def _publish_position_setpoint(self, x: float, y: float, z: float, yaw: float) -> None:
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = yaw
        msg.timestamp = self._now_us()
        self.trajectory_setpoint_pub.publish(msg)

    def _publish_vehicle_command(self, command: int, **params: float) -> None:
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = float(params.get("param1", 0.0))
        msg.param2 = float(params.get("param2", 0.0))
        msg.param3 = float(params.get("param3", 0.0))
        msg.param4 = float(params.get("param4", 0.0))
        msg.param5 = float(params.get("param5", 0.0))
        msg.param6 = float(params.get("param6", 0.0))
        msg.param7 = float(params.get("param7", 0.0))
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self._now_us()
        self.vehicle_command_pub.publish(msg)

    def _arm(self) -> None:
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0
        )
        self.get_logger().info("Arm command sent")

    def _engage_offboard(self) -> None:
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0
        )
        self.get_logger().info("Offboard mode command sent")

    def _land(self) -> None:
        self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Land command sent")

    def _now_us(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1000)

    def _ensure_target_position(self) -> None:
        if self._target_position is not None:
            return

        current_x = self.vehicle_local_position.x
        current_y = self.vehicle_local_position.y
        current_z = self.vehicle_local_position.z

        target_x = current_x + self.target_x_m
        target_y = current_y + self.target_y_m

        if self.arm_only:
            target_z = current_z
        else:
            # Local position is NED, so climbing means moving z in the negative direction.
            target_z = current_z - self.takeoff_height_m

        self._target_position = (target_x, target_y, target_z)
        self.get_logger().info(
            f"Target setpoint locked to x={target_x:.2f}, y={target_y:.2f}, z={target_z:.2f}"
        )

    def _timer_cb(self) -> None:
        if self.phase == FlightPhase.WAIT_FOR_ESTIMATE:
            self._log_phase("Waiting for valid local position estimate before arming")
            if not self._estimate_ready():
                return

            self._ensure_target_position()
            self.phase = FlightPhase.STREAM_SETPOINTS

        if self.phase == FlightPhase.STREAM_SETPOINTS:
            self._log_phase("Streaming pre-arm offboard setpoints")
            self._publish_offboard_heartbeat()
            self._publish_position_setpoint(*self._target_position, self.yaw_rad)
            self._prestream_sent += 1

            if self._prestream_sent >= self.prestream_count:
                self.phase = FlightPhase.ARM_AND_OFFBOARD
            return

        if self.phase == FlightPhase.ARM_AND_OFFBOARD:
            self._log_phase("Sending offboard mode and arm command")
            self._publish_offboard_heartbeat()
            self._publish_position_setpoint(*self._target_position, self.yaw_rad)

            if not self._command_sent:
                self._engage_offboard()
                self._arm()
                self._command_sent = True
                self._hover_start_us = self._now_us()
                self.phase = FlightPhase.HOVER
            return

        if self.phase == FlightPhase.HOVER:
            self._log_phase("Holding target setpoint")
            self._publish_offboard_heartbeat()
            self._publish_position_setpoint(*self._target_position, self.yaw_rad)

            if self.arm_only:
                return

            if self._hover_start_us is not None:
                elapsed_sec = (self._now_us() - self._hover_start_us) / 1e6
                if elapsed_sec >= self.hover_seconds:
                    if self.auto_land:
                        self.phase = FlightPhase.LAND
                    else:
                        self.phase = FlightPhase.DONE
            return

        if self.phase == FlightPhase.LAND:
            self._log_phase("Auto-landing")
            self._land()
            self.phase = FlightPhase.DONE
            return

        if self.phase == FlightPhase.DONE:
            self._log_phase("Control script completed; stopping node")
            raise SystemExit(0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MiniPcOffboardControl()

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
