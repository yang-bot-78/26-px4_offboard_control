#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class OdometryTfPublisher(Node):
    def __init__(self) -> None:
        super().__init__("nav2_stage1_odometry_tf_publisher")

        self.declare_parameter("odom_topic", "/Odometry")
        self.declare_parameter("frame_id", "camera_init")
        self.declare_parameter("child_frame_id", "body")
        self.declare_parameter("force_frame_ids", False)

        self._broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            Odometry,
            self.get_parameter("odom_topic").value,
            self._odom_cb,
            50,
        )
        self.get_logger().info(
            "Publishing TF from odometry topic %s"
            % self.get_parameter("odom_topic").value
        )

    def _odom_cb(self, msg: Odometry) -> None:
        force_frame_ids = bool(self.get_parameter("force_frame_ids").value)

        transform = TransformStamped()
        transform.header = msg.header
        transform.child_frame_id = msg.child_frame_id

        if force_frame_ids or not transform.header.frame_id:
            transform.header.frame_id = self.get_parameter("frame_id").value
        if force_frame_ids or not transform.child_frame_id:
            transform.child_frame_id = self.get_parameter("child_frame_id").value

        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation
        self._broadcaster.sendTransform(transform)


def main() -> None:
    rclpy.init()
    node = OdometryTfPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
