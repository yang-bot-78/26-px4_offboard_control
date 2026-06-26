#!/usr/bin/env python3
from copy import deepcopy
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import Odometry, Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class GoalToPath(Node):
    def __init__(self) -> None:
        super().__init__("nav2_stage1_goal_to_path")

        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("legacy_goal_topic", "/move_base_simple/goal")
        self.declare_parameter("odom_topic", "/Odometry")
        self.declare_parameter("path_topic", "/nav2_stage1/path")
        self.declare_parameter("planner_action", "/compute_path_to_pose")
        self.declare_parameter("planner_id", "GridBased")
        self.declare_parameter("global_frame", "camera_init")
        self.declare_parameter("use_odom_start", True)
        self.declare_parameter("wait_for_action_timeout_s", 2.0)

        self._planner_id = self.get_parameter("planner_id").value
        self._global_frame = self.get_parameter("global_frame").value
        self._use_odom_start = bool(self.get_parameter("use_odom_start").value)
        self._latest_start: Optional[PoseStamped] = None
        self._active_goal = None

        transient_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self._path_pub = self.create_publisher(
            Path,
            self.get_parameter("path_topic").value,
            transient_qos,
        )
        self.create_subscription(
            PoseStamped,
            self.get_parameter("goal_topic").value,
            self._goal_cb,
            10,
        )
        self.create_subscription(
            PoseStamped,
            self.get_parameter("legacy_goal_topic").value,
            self._goal_cb,
            10,
        )
        self.create_subscription(
            Odometry,
            self.get_parameter("odom_topic").value,
            self._odom_cb,
            20,
        )

        self._action_client = ActionClient(
            self,
            ComputePathToPose,
            self.get_parameter("planner_action").value,
        )
        self.get_logger().info(
            "Waiting for RViz goals on %s or %s; publishing planned paths on %s"
            % (
                self.get_parameter("goal_topic").value,
                self.get_parameter("legacy_goal_topic").value,
                self.get_parameter("path_topic").value,
            )
        )

    def _odom_cb(self, msg: Odometry) -> None:
        start = PoseStamped()
        start.header = msg.header
        start.pose = msg.pose.pose
        if not start.header.frame_id:
            start.header.frame_id = self._global_frame
        self._latest_start = start

    def _goal_cb(self, msg: PoseStamped) -> None:
        goal = deepcopy(msg)
        if not goal.header.frame_id:
            goal.header.frame_id = self._global_frame
        goal.header.stamp = self.get_clock().now().to_msg()

        timeout_s = float(self.get_parameter("wait_for_action_timeout_s").value)
        if not self._action_client.wait_for_server(timeout_sec=timeout_s):
            self.get_logger().warn("Nav2 planner action is not available yet.")
            return

        request = ComputePathToPose.Goal()
        request.goal = goal
        request.planner_id = self._planner_id

        if self._use_odom_start:
            if self._latest_start is None:
                self.get_logger().warn("No odometry received yet; cannot seed planner start.")
                return
            request.start = deepcopy(self._latest_start)
            request.start.header.stamp = self.get_clock().now().to_msg()
            request.use_start = True
        else:
            request.use_start = False

        if self._active_goal is not None:
            self.get_logger().info("A previous planning request is still running; ignoring new goal.")
            return

        self.get_logger().info(
            "Planning to x=%.2f y=%.2f in frame %s"
            % (goal.pose.position.x, goal.pose.position.y, goal.header.frame_id)
        )
        future = self._action_client.send_goal_async(request)
        future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Planner rejected the goal.")
            self._active_goal = None
            return

        self._active_goal = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future) -> None:
        try:
            result = future.result().result
        except Exception as exc:
            self.get_logger().error("Planning request failed: %s" % exc)
            self._active_goal = None
            return

        path = result.path
        path.header.stamp = self.get_clock().now().to_msg()
        self._path_pub.publish(path)
        self.get_logger().info("Published path with %d poses." % len(path.poses))
        self._active_goal = None


def main() -> None:
    rclpy.init()
    node = GoalToPath()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
