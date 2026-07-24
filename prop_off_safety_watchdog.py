#!/usr/bin/env python3
"""Stop the parent validation script if MAVROS reports arm or OFFBOARD."""

from __future__ import annotations

import argparse
import os
import signal

from mavros_msgs.msg import State
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class PropOffSafetyWatchdog(Node):
    """Observe MAVROS state without publishing or calling any service."""

    def __init__(self, parent_pid: int) -> None:
        super().__init__("prop_off_safety_watchdog")
        self.parent_pid = parent_pid
        self.create_subscription(
            State,
            "/mavros/state",
            self._state_callback,
            qos_profile_sensor_data,
        )

    def _state_callback(self, message: State) -> None:
        unsafe = bool(message.armed) or message.mode.strip().upper() == "OFFBOARD"
        if not unsafe:
            return
        self.get_logger().error(
            f"Unsafe state during prop-off validation: armed={message.armed}, "
            f"mode={message.mode}; stopping validation"
        )
        try:
            os.kill(self.parent_pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        rclpy.shutdown()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--parent-pid", type=int, required=True)
    args = parser.parse_args()
    rclpy.init()
    node = PropOffSafetyWatchdog(args.parent_pid)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
