#!/usr/bin/env python3
"""把 FR-LIO 里程计对齐到重定位地图，仅供无飞行规划验证。"""

import copy
import math
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import TransformBroadcaster

Quaternion = Tuple[float, float, float, float]
Vector3 = Tuple[float, float, float]


def quat_normalize(q: Quaternion) -> Quaternion:
    norm = math.sqrt(sum(value * value for value in q))
    if norm < 1e-9:
        return (0.0, 0.0, 0.0, 1.0)
    return tuple(value / norm for value in q)  # type: ignore[return-value]


def quat_inverse(q: Quaternion) -> Quaternion:
    return (-q[0], -q[1], -q[2], q[3])


def quat_multiply_raw(a: Quaternion, b: Quaternion) -> Quaternion:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quat_multiply(a: Quaternion, b: Quaternion) -> Quaternion:
    return quat_normalize(quat_multiply_raw(a, b))


def quat_rotate(q: Quaternion, vector: Vector3) -> Vector3:
    rotated = quat_multiply_raw(
        quat_multiply_raw(q, (vector[0], vector[1], vector[2], 0.0)),
        quat_inverse(q),
    )
    return (rotated[0], rotated[1], rotated[2])


def pose_parts(pose: Pose) -> Tuple[Vector3, Quaternion]:
    return (
        (pose.position.x, pose.position.y, pose.position.z),
        quat_normalize(
            (
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            )
        ),
    )


def rotate_pose_covariance(covariance, rotation: Quaternion):
    yaw = math.atan2(
        2.0 * (rotation[3] * rotation[2] + rotation[0] * rotation[1]),
        1.0 - 2.0 * (rotation[1] * rotation[1] + rotation[2] * rotation[2]),
    )
    c, s = math.cos(yaw), math.sin(yaw)
    transform = [[float(row == column) for column in range(6)] for row in range(6)]
    for offset in (0, 3):
        transform[offset][offset] = c
        transform[offset][offset + 1] = -s
        transform[offset + 1][offset] = s
        transform[offset + 1][offset + 1] = c
    source = [[float(covariance[row * 6 + column]) for column in range(6)] for row in range(6)]
    output = [0.0] * 36
    for row in range(6):
        for column in range(6):
            output[row * 6 + column] = sum(
                transform[row][left]
                * source[left][right]
                * transform[column][right]
                for left in range(6)
                for right in range(6)
            )
    return output


class RelocalizationFrameBridge(Node):
    def __init__(self) -> None:
        super().__init__("planning_relocalization_frame_bridge")
        self.declare_parameter("odom_topic", "/Odometry")
        self.declare_parameter(
            "relocalized_pose_topic", "/fastlio_global/relocalized_pose"
        )
        self.declare_parameter("output_odom_topic", "/planning/odom")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("body_frame", "body")
        self.declare_parameter("publish_odom_body_tf", True)

        self._latest_odom: Optional[Odometry] = None
        self._pending_relocalized_pose: Optional[PoseStamped] = None
        self._map_to_odom: Optional[Tuple[Vector3, Quaternion]] = None
        self._tf_broadcaster = TransformBroadcaster(self)
        self._odom_pub = self.create_publisher(
            Odometry, self.get_parameter("output_odom_topic").value, 20
        )
        # Match FR-LIO's BEST_EFFORT publisher and retain only the newest frame
        # so slow Python callbacks cannot accumulate stale odometry messages.
        frlio_odom_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.create_subscription(
            Odometry,
            self.get_parameter("odom_topic").value,
            self._odom_callback,
            frlio_odom_qos,
        )
        self.create_subscription(
            PoseStamped,
            self.get_parameter("relocalized_pose_topic").value,
            self._relocalized_pose_callback,
            10,
        )
        self.get_logger().info(
            "等待 FR-LIO 里程计和重定位结果；成功后发布 /planning/odom。"
        )

    def _odom_callback(self, msg: Odometry) -> None:
        self._latest_odom = msg
        if self._pending_relocalized_pose is not None:
            pending_pose = self._pending_relocalized_pose
            self._pending_relocalized_pose = None
            self._apply_relocalized_pose(pending_pose)
        odom_translation, odom_rotation = pose_parts(msg.pose.pose)
        odom_frame = str(self.get_parameter("odom_frame").value)
        body_frame = str(self.get_parameter("body_frame").value)

        if bool(self.get_parameter("publish_odom_body_tf").value):
            self._publish_transform(
                odom_frame,
                body_frame,
                odom_translation,
                odom_rotation,
                msg.header.stamp,
            )
        if self._map_to_odom is None:
            return

        map_to_odom_translation, map_to_odom_rotation = self._map_to_odom
        rotated_translation = quat_rotate(map_to_odom_rotation, odom_translation)
        map_to_body_translation = tuple(
            map_to_odom_translation[index] + rotated_translation[index]
            for index in range(3)
        )
        map_to_body_rotation = quat_multiply(map_to_odom_rotation, odom_rotation)

        output = Odometry()
        output.header.stamp = msg.header.stamp
        output.header.frame_id = str(self.get_parameter("map_frame").value)
        output.child_frame_id = body_frame
        # ROS Python 消息字段赋值可能保留对象引用；必须复制，避免把缓存的原始
        # camera_init 里程计原地改成 map 位姿，导致下一次重定位计算坐标漂移。
        output.pose = copy.deepcopy(msg.pose)
        output.pose.pose.position.x = map_to_body_translation[0]
        output.pose.pose.position.y = map_to_body_translation[1]
        output.pose.pose.position.z = map_to_body_translation[2]
        output.pose.pose.orientation.x = map_to_body_rotation[0]
        output.pose.pose.orientation.y = map_to_body_rotation[1]
        output.pose.pose.orientation.z = map_to_body_rotation[2]
        output.pose.pose.orientation.w = map_to_body_rotation[3]
        output.pose.covariance = rotate_pose_covariance(
            msg.pose.covariance, map_to_odom_rotation
        )
        output.twist = copy.deepcopy(msg.twist)
        self._odom_pub.publish(output)

        self._publish_transform(
            str(self.get_parameter("map_frame").value),
            odom_frame,
            map_to_odom_translation,
            map_to_odom_rotation,
            msg.header.stamp,
        )

    def _relocalized_pose_callback(self, msg: PoseStamped) -> None:
        if self._latest_odom is None:
            # One-shot relocalization must not be lost just because the first
            # FR-LIO odometry message has not arrived yet.
            self._pending_relocalized_pose = copy.deepcopy(msg)
            self.get_logger().warning(
                "收到重定位结果时尚无 FR-LIO 里程计；将等待首帧后应用。"
            )
            return
        self._apply_relocalized_pose(msg)

    def _apply_relocalized_pose(self, msg: PoseStamped) -> None:
        if self._latest_odom is None:
            return

        map_to_body_translation, map_to_body_rotation = pose_parts(msg.pose)
        odom_to_body_translation, odom_to_body_rotation = pose_parts(
            self._latest_odom.pose.pose
        )
        map_to_odom_rotation = quat_multiply(
            map_to_body_rotation, quat_inverse(odom_to_body_rotation)
        )
        rotated_odom_to_body = quat_rotate(
            map_to_odom_rotation, odom_to_body_translation
        )
        map_to_odom_translation = tuple(
            map_to_body_translation[index] - rotated_odom_to_body[index]
            for index in range(3)
        )
        self._map_to_odom = (map_to_odom_translation, map_to_odom_rotation)
        self.get_logger().info(
            "重定位坐标对齐完成，开始发布 map -> camera_init 和 /planning/odom。"
        )

    def _publish_transform(
        self,
        parent: str,
        child: str,
        translation: Vector3,
        rotation: Quaternion,
        stamp,
    ) -> None:
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = parent
        transform.child_frame_id = child
        transform.transform.translation.x = translation[0]
        transform.transform.translation.y = translation[1]
        transform.transform.translation.z = translation[2]
        transform.transform.rotation.x = rotation[0]
        transform.transform.rotation.y = rotation[1]
        transform.transform.rotation.z = rotation[2]
        transform.transform.rotation.w = rotation[3]
        self._tf_broadcaster.sendTransform(transform)


def main() -> None:
    rclpy.init()
    node = RelocalizationFrameBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
