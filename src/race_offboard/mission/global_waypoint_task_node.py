#!/usr/bin/env python3
"""Plan the saved B1 -> B2 -> C -> D route without PX4 control nodes."""

from __future__ import annotations

import math
import sys
from pathlib import Path
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from mission_sequencer_node import MissionConfigError, Point, load_settings, point_is_reached


POINT_NAMES = ('B1', 'B2', 'C', 'D')


class GlobalWaypointTaskNode(Node):
    """Publish one planner goal at a time and advance from map-frame odometry."""

    def __init__(self) -> None:
        super().__init__('global_waypoint_task_node')
        self.declare_parameter('mission_file', '')
        self.declare_parameter('odom_topic', '/race/odom')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('state_topic', '/race/global_waypoint_task/state')
        mission_file = self.get_parameter('mission_file').value
        if not mission_file:
            raise MissionConfigError('mission_file parameter must not be empty')
        settings = load_settings(Path(mission_file))
        if len(settings.preset_points) != len(POINT_NAMES):
            raise MissionConfigError('mission_file must define B1, B2, C, D preset_points')
        self._points: tuple[Point, ...] = settings.preset_points
        self._arrive_radius = settings.arrive_radius
        self._next_index = 0
        self._initial_goal_sent = False
        self._finished = False

        transient_reliable = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT)
        self._goal_publisher = self.create_publisher(
            PoseStamped, self.get_parameter('goal_topic').value, 10)
        self._state_publisher = self.create_publisher(
            String, self.get_parameter('state_topic').value, transient_reliable)
        self._event_publisher = self.create_publisher(
            String, '/race/global_waypoint_task/event', 10)
        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value,
            self._odom_callback, sensor_qos)
        self.create_timer(0.25, self._start_when_planner_ready)
        self.get_logger().info(
            '[GLOBAL_TASK_READY] 等待全局规划器订阅目标；路线：B1 -> B2 -> C -> D')

    def _start_when_planner_ready(self) -> None:
        if self._initial_goal_sent or self._goal_publisher.get_subscription_count() == 0:
            return
        self._publish_goal(self._points[0])
        self._initial_goal_sent = True

    def _odom_callback(self, message: Odometry) -> None:
        if not self._initial_goal_sent or self._finished:
            return
        if message.header.frame_id != 'map':
            self.get_logger().warning(
                f'忽略非 map 坐标系里程计：{message.header.frame_id}', throttle_duration_sec=5.0)
            return
        position = message.pose.pose.position
        if not (math.isfinite(position.x) and math.isfinite(position.y)):
            return
        point = self._points[self._next_index]
        if not point_is_reached(position.x, position.y, point, self._arrive_radius):
            return
        self.get_logger().info(
            f'[GLOBAL_TASK_ARRIVED] 已到达 {POINT_NAMES[self._next_index]}，'
            f'半径={self._arrive_radius:.2f}m')
        if self._next_index in (0, 3):
            self._switch_state(POINT_NAMES[self._next_index])
        self._next_index += 1
        if self._next_index >= len(self._points):
            self._finished = True
            self._event_publisher.publish(String(data='event=route_complete point=D'))
            self.get_logger().info('[GLOBAL_TASK_DONE] 已到达 D，未发布后续目标。')
            return
        self._publish_goal(self._points[self._next_index])

    def _switch_state(self, state: str) -> None:
        self._state_publisher.publish(String(data=state))
        self._event_publisher.publish(String(data=f'event=state_changed state={state}'))
        self.get_logger().info(f'[GLOBAL_TASK_STATE] 状态已切换成功：{state}')

    def _publish_goal(self, point: Point) -> None:
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = 'map'
        goal.pose.position.x = point.x
        goal.pose.position.y = point.y
        # The global planner applies its configured fixed flight height.  This
        # node never publishes a vehicle setpoint.
        goal.pose.orientation.w = 1.0
        self._goal_publisher.publish(goal)
        name = POINT_NAMES[self._next_index]
        self._event_publisher.publish(
            String(data=(
                f'event=global_plan_requested point={name} '
                f'x={point.x:.3f} y={point.y:.3f}')))
        self.get_logger().info(
            f'[GLOBAL_TASK_PLAN] 请求到 {name} 的全局路径：x={point.x:.3f} y={point.y:.3f}')


def main() -> int:
    rclpy.init()
    node: Optional[GlobalWaypointTaskNode] = None
    try:
        node = GlobalWaypointTaskNode()
        rclpy.spin(node)
    except MissionConfigError as error:
        print(f'global_waypoint_task_node configuration error: {error}', file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        return 0
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
