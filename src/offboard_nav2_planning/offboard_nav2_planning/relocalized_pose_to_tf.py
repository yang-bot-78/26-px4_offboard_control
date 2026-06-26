#!/usr/bin/env python3
import math
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

Quaternion = Tuple[float, float, float, float]
Vector3 = Tuple[float, float, float]


class RelocalizedPoseToTf(Node):
    def __init__(self) -> None:
        super().__init__("nav2_relocalized_pose_to_tf")

        self.declare_parameter("relocalized_pose_topic", "/fastlio_global/relocalized_pose")
        self.declare_parameter("odom_topic", "/Odometry")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "camera_init")
        self.declare_parameter("publish_rate_hz", 20.0)

        self._latest_odom: Optional[Odometry] = None
        self._latest_tf: Optional[TransformStamped] = None
        self._broadcaster = TransformBroadcaster(self)

        self.create_subscription(
            Odometry,
            self.get_parameter("odom_topic").value,
            self._odom_cb,
            30,
        )
        self.create_subscription(
            PoseStamped,
            self.get_parameter("relocalized_pose_topic").value,
            self._relocalized_pose_cb,
            10,
        )
        publish_rate_hz = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.create_timer(1.0 / publish_rate_hz, self._publish_latest_tf)

        self.get_logger().info(
            "Waiting for relocalized pose on %s and odometry on %s"
            % (
                self.get_parameter("relocalized_pose_topic").value,
                self.get_parameter("odom_topic").value,
            )
        )

    def _odom_cb(self, msg: Odometry) -> None:
        self._latest_odom = msg

    def _relocalized_pose_cb(self, msg: PoseStamped) -> None:
        if self._latest_odom is None:
            self.get_logger().warn("Relocalized pose received before odometry; ignoring.")
            return

        map_frame = str(self.get_parameter("map_frame").value)
        odom_frame = str(self.get_parameter("odom_frame").value)

        t_map_body, q_map_body = self._pose_to_transform(msg.pose)
        t_odom_body, q_odom_body = self._pose_to_transform(self._latest_odom.pose.pose)

        q_body_odom = self._quat_inverse(q_odom_body)
        q_map_odom = self._quat_multiply(q_map_body, q_body_odom)
        rotated_odom_body = self._quat_rotate(q_map_odom, t_odom_body)
        t_map_odom = (
            t_map_body[0] - rotated_odom_body[0],
            t_map_body[1] - rotated_odom_body[1],
            t_map_body[2] - rotated_odom_body[2],
        )

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = map_frame
        transform.child_frame_id = odom_frame
        transform.transform.translation.x = t_map_odom[0]
        transform.transform.translation.y = t_map_odom[1]
        transform.transform.translation.z = t_map_odom[2]
        transform.transform.rotation.x = q_map_odom[0]
        transform.transform.rotation.y = q_map_odom[1]
        transform.transform.rotation.z = q_map_odom[2]
        transform.transform.rotation.w = q_map_odom[3]
        self._latest_tf = transform

        self.get_logger().info(
            "Updated %s -> %s TF from relocalization."
            % (map_frame, odom_frame)
        )

    def _publish_latest_tf(self) -> None:
        if self._latest_tf is None:
            return
        self._latest_tf.header.stamp = self.get_clock().now().to_msg()
        self._broadcaster.sendTransform(self._latest_tf)

    @staticmethod
    def _pose_to_transform(pose: Pose) -> Tuple[Vector3, Quaternion]:
        translation = (
            float(pose.position.x),
            float(pose.position.y),
            float(pose.position.z),
        )
        q = (
            float(pose.orientation.x),
            float(pose.orientation.y),
            float(pose.orientation.z),
            float(pose.orientation.w),
        )
        return translation, RelocalizedPoseToTf._quat_normalize(q)

    @staticmethod
    def _quat_normalize(q: Quaternion) -> Quaternion:
        norm = math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])
        if norm < 1e-9:
            return (0.0, 0.0, 0.0, 1.0)
        return (q[0] / norm, q[1] / norm, q[2] / norm, q[3] / norm)

    @staticmethod
    def _quat_inverse(q: Quaternion) -> Quaternion:
        return (-q[0], -q[1], -q[2], q[3])

    @staticmethod
    def _quat_multiply(a: Quaternion, b: Quaternion) -> Quaternion:
        ax, ay, az, aw = a
        bx, by, bz, bw = b
        return RelocalizedPoseToTf._quat_normalize(RelocalizedPoseToTf._quat_multiply_raw(a, b))

    @staticmethod
    def _quat_multiply_raw(a: Quaternion, b: Quaternion) -> Quaternion:
        ax, ay, az, aw = a
        bx, by, bz, bw = b
        return (
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        )

    @staticmethod
    def _quat_rotate(q: Quaternion, v: Vector3) -> Vector3:
        vq = (v[0], v[1], v[2], 0.0)
        rotated = RelocalizedPoseToTf._quat_multiply_raw(
            RelocalizedPoseToTf._quat_multiply_raw(q, vq),
            RelocalizedPoseToTf._quat_inverse(q),
        )
        return (rotated[0], rotated[1], rotated[2])


def main() -> None:
    rclpy.init()
    node = RelocalizedPoseToTf()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
