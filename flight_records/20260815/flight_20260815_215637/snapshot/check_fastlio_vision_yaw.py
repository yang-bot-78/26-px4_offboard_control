#!/usr/bin/env python3

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


def yaw_deg(q: Quaternion) -> float:
    yaw = math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )
    return math.degrees(yaw)


def wrap_deg(angle: float) -> float:
    return (angle + 180.0) % 360.0 - 180.0


class FastlioVisionYawCheck(Node):
    def __init__(self) -> None:
        super().__init__("check_fastlio_vision_yaw")

        self.declare_parameter("fastlio_topic", "/Odometry")
        self.declare_parameter("vision_topic", "/mavros/vision_pose/pose_cov")
        self.declare_parameter("local_odom_topic", "/mavros/local_position/odom")
        self.declare_parameter("print_period_s", 0.5)

        self.fastlio_topic = str(self.get_parameter("fastlio_topic").value)
        self.vision_topic = str(self.get_parameter("vision_topic").value)
        self.local_odom_topic = str(self.get_parameter("local_odom_topic").value)
        print_period_s = max(0.1, float(self.get_parameter("print_period_s").value))

        self.fastlio_yaw: Optional[float] = None
        self.vision_yaw: Optional[float] = None
        self.local_yaw: Optional[float] = None

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(Odometry, self.fastlio_topic, self.fastlio_cb, qos)
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.vision_topic,
            self.vision_cb,
            qos,
        )
        self.create_subscription(Odometry, self.local_odom_topic, self.local_odom_cb, qos)
        self.create_timer(print_period_s, self.print_yaws)

        self.get_logger().info(
            "Watching yaw: "
            f"fastlio={self.fastlio_topic}, vision={self.vision_topic}, "
            f"local_odom={self.local_odom_topic}"
        )

    def fastlio_cb(self, msg: Odometry) -> None:
        self.fastlio_yaw = yaw_deg(msg.pose.pose.orientation)

    def vision_cb(self, msg: PoseWithCovarianceStamped) -> None:
        self.vision_yaw = yaw_deg(msg.pose.pose.orientation)

    def local_odom_cb(self, msg: Odometry) -> None:
        self.local_yaw = yaw_deg(msg.pose.pose.orientation)

    def print_yaws(self) -> None:
        waiting = []
        if self.fastlio_yaw is None:
            waiting.append(self.fastlio_topic)
        if self.vision_yaw is None:
            waiting.append(self.vision_topic)

        if waiting:
            print("waiting for " + ", ".join(waiting))
            return

        vision_delta = wrap_deg(self.vision_yaw - self.fastlio_yaw)
        line = (
            f"FAST-LIO yaw={self.fastlio_yaw:7.1f} deg | "
            f"vision yaw={self.vision_yaw:7.1f} deg | "
            f"vision-fastlio={vision_delta:7.1f} deg"
        )

        if self.local_yaw is not None:
            local_delta = wrap_deg(self.local_yaw - self.vision_yaw)
            line += (
                f" | PX4 local yaw={self.local_yaw:7.1f} deg | "
                f"local-vision={local_delta:7.1f} deg"
            )

        print(line)


def main() -> None:
    rclpy.init()
    node = FastlioVisionYawCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
