#!/usr/bin/env python3
"""Validation-only ROS 2 topic gate used to isolate one MID360 stream."""

from __future__ import annotations

import argparse
import signal
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Forward one sensor topic until this process is stopped."
    )
    parser.add_argument("--kind", choices=("imu", "lidar"), required=True)
    parser.add_argument("--input", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--node-name", required=True)
    return parser.parse_args()


class TopicGate(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__(args.node_name)
        if args.kind == "imu":
            from sensor_msgs.msg import Imu

            message_type = Imu
        else:
            from livox_ros_driver2.msg import CustomMsg

            message_type = CustomMsg

        qos = QoSProfile(depth=20)
        self._publisher = self.create_publisher(message_type, args.output, qos)
        self._subscription = self.create_subscription(
            message_type,
            args.input,
            self._publisher.publish,
            qos,
        )
        self.get_logger().info(
            f"forwarding {args.kind}: {args.input} -> {args.output}"
        )


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = TopicGate(args)

    def request_shutdown(_signum: int, _frame: object) -> None:
        if rclpy.ok():
            rclpy.shutdown()

    signal.signal(signal.SIGINT, request_shutdown)
    signal.signal(signal.SIGTERM, request_shutdown)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
