#!/usr/bin/env python3

import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient


class MavrosIdentityFixer(Node):
    def __init__(self) -> None:
        super().__init__("fix_mavros_identity")

        self.declare_parameter("target_node", "/mavros/mavros")
        self.declare_parameter("system_id", 255)
        self.declare_parameter("component_id", 190)
        self.declare_parameter("target_system_id", 1)
        self.declare_parameter("target_component_id", 1)
        self.declare_parameter("timeout_sec", 15.0)

        target_node = self.get_parameter("target_node").get_parameter_value().string_value
        self._timeout_sec = self.get_parameter("timeout_sec").get_parameter_value().double_value
        self._client = AsyncParameterClient(self, target_node)

        self._target_values = [
            Parameter(
                "system_id",
                value=int(self.get_parameter("system_id").get_parameter_value().integer_value),
            ),
            Parameter(
                "component_id",
                value=int(self.get_parameter("component_id").get_parameter_value().integer_value),
            ),
            Parameter(
                "target_system_id",
                value=int(self.get_parameter("target_system_id").get_parameter_value().integer_value),
            ),
            Parameter(
                "target_component_id",
                value=int(self.get_parameter("target_component_id").get_parameter_value().integer_value),
            ),
        ]

    def run(self) -> int:
        deadline = time.monotonic() + self._timeout_sec
        self.get_logger().info("Waiting for /mavros/mavros parameter service...")

        while time.monotonic() < deadline:
            if self._client.wait_for_service(timeout_sec=1.0):
                break
        else:
            self.get_logger().error("Timed out waiting for /mavros/mavros parameter service.")
            return 1

        self.get_logger().info("Setting MAVROS system/component identity.")
        future = self._client.set_parameters(self._target_values)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self._timeout_sec)

        if not future.done() or future.result() is None:
            self.get_logger().error("Failed to set MAVROS identity parameters.")
            return 1

        results = future.result()
        if not all(result.successful for result in results):
            for result in results:
                if not result.successful:
                    self.get_logger().error(f"Parameter set failed: {result.reason}")
            return 1

        self.get_logger().info("MAVROS identity parameters updated successfully.")
        return 0


def main() -> int:
    rclpy.init()
    node = MavrosIdentityFixer()
    try:
        return node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
