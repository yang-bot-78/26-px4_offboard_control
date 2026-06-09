#!/usr/bin/env python3

import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient


class MavrosOdometryFrameFixer(Node):
    def __init__(self) -> None:
        super().__init__("fix_mavros_odometry_frames")

        self.declare_parameter("target_node", "/mavros/odometry")
        self.declare_parameter("map_id_des", "map")
        self.declare_parameter("odom_parent_id_des", "odom")
        self.declare_parameter("odom_child_id_des", "base_link")
        self.declare_parameter("timeout_sec", 15.0)

        target_node = self.get_parameter("target_node").get_parameter_value().string_value
        self._timeout_sec = self.get_parameter("timeout_sec").get_parameter_value().double_value
        self._client = AsyncParameterClient(self, target_node)

        self._target_values = [
            Parameter(
                "fcu.map_id_des",
                value=self.get_parameter("map_id_des").get_parameter_value().string_value,
            ),
            Parameter(
                "fcu.odom_parent_id_des",
                value=self.get_parameter("odom_parent_id_des").get_parameter_value().string_value,
            ),
            Parameter(
                "fcu.odom_child_id_des",
                value=self.get_parameter("odom_child_id_des").get_parameter_value().string_value,
            ),
        ]

    def run(self) -> int:
        deadline = time.monotonic() + self._timeout_sec
        self.get_logger().info("Waiting for /mavros/odometry parameter service...")

        while time.monotonic() < deadline:
            if self._client.wait_for_service(timeout_sec=1.0):
                break
        else:
            self.get_logger().error("Timed out waiting for /mavros/odometry parameter service.")
            return 1

        self.get_logger().info("Setting MAVROS odometry target frames to odom/base_link.")
        future = self._client.set_parameters(self._target_values)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self._timeout_sec)

        if not future.done() or future.result() is None:
            self.get_logger().error("Failed to set MAVROS odometry parameters.")
            return 1

        results = future.result()
        if not all(result.successful for result in results):
            for result in results:
                if not result.successful:
                    self.get_logger().error(f"Parameter set failed: {result.reason}")
            return 1

        self.get_logger().info("MAVROS odometry frame targets updated successfully.")
        return 0


def main() -> int:
    rclpy.init()
    node = MavrosOdometryFrameFixer()
    try:
        return node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
