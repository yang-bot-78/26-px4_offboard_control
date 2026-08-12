#!/usr/bin/env python3
"""
ROS integration check for the ground-height Goal Bridge gate.

This deliberately starts only the real Goal Bridge: no PX4, Gazebo, or
planner is required. It proves that ground goals are cached, only the latest
one survives, and a stable hover anywhere in the configured safety height
window releases that goal.
"""

import subprocess
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


def pose(x, y=0.0, z=0.78):
    msg = PoseStamped()
    msg.header.frame_id = 'map'
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.position.z = z
    msg.pose.orientation.w = 1.0
    return msg


def odom(height, horizontal_speed=0.0, vertical_speed=0.0):
    msg = Odometry()
    msg.header.frame_id = 'map'
    msg.pose.pose.position.z = height
    msg.pose.pose.orientation.w = 1.0
    msg.twist.twist.linear.x = horizontal_speed
    msg.twist.twist.linear.z = vertical_speed
    return msg


def spin_until(node, predicate, timeout_sec):
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            return True
    return False


def main():
    bridge = subprocess.Popen([
        'ros2', 'run', 'race_ego_bridge', 'race_ego_goal_bridge', '--ros-args',
        '-p', 'input_topic:=/test/takeoff_gate/local_goal',
        '-p', 'ego_topic:=/test/takeoff_gate/ego_goal',
        '-p', 'odom_topic:=/test/takeoff_gate/odom',
        '-p', 'release_height:=0.50',
        '-p', 'flight_height:=0.78',
        '-p', 'stable_duration_sec:=0.15',
    ])
    rclpy.init()
    node = rclpy.create_node('test_goal_bridge_takeoff_release')
    released = []
    goals = node.create_publisher(PoseStamped, '/test/takeoff_gate/local_goal', 10)
    odoms = node.create_publisher(Odometry, '/test/takeoff_gate/odom', 10)
    release_subscription = node.create_subscription(
        PoseStamped, '/test/takeoff_gate/ego_goal', lambda msg: released.append(msg), 10)
    try:
        if not spin_until(node, lambda: goals.get_subscription_count() >= 1, 15.0):
            raise AssertionError('real Goal Bridge did not subscribe')
        odoms.publish(odom(0.0))
        goals.publish(pose(1.0))
        time.sleep(0.1)
        goals.publish(pose(2.0))
        spin_until(node, lambda: False, 0.5)
        if released:
            raise AssertionError('Goal Bridge released a ground-level local_goal')
        # The handover gate is a safety-height range, not a fixed 0.78 m
        # cruise-altitude check. Test both boundaries and an interior height.
        for index, height in enumerate((0.50, 0.65, 0.85), start=1):
            # Dropping below release_height earns a fresh handover for the
            # next simulated takeoff.
            odoms.publish(odom(0.49))
            spin_until(node, lambda: False, 0.1)
            goal_x = 2.0 + index
            goals.publish(pose(goal_x))
            spin_until(node, lambda: False, 0.1)
            odoms.publish(odom(height))
            spin_until(node, lambda: False, 0.1)
            odoms.publish(odom(height))
            spin_until(node, lambda: False, 0.1)
            odoms.publish(odom(height))
            if not spin_until(node, lambda: len(released) == index, 3.0):
                raise AssertionError(
                    f'Goal Bridge did not release a stable {height:.2f} m local_goal')
            if abs(released[index - 1].pose.position.x - goal_x) > 1e-6:
                raise AssertionError('Goal Bridge released the wrong cached local_goal')

        # Heights outside the safety window must remain gated, even when the
        # vehicle is otherwise stationary.
        odoms.publish(odom(0.49))
        spin_until(node, lambda: False, 0.1)
        goals.publish(pose(6.0))
        odoms.publish(odom(0.49))
        spin_until(node, lambda: False, 0.3)
        if len(released) != 3:
            raise AssertionError('Goal Bridge released below the safety height range')
        odoms.publish(odom(0.86))
        spin_until(node, lambda: False, 0.3)
        if len(released) != 3:
            raise AssertionError('Goal Bridge released above the safety height range')

        # A stable height still requires safe horizontal and vertical speed.
        odoms.publish(odom(0.49))
        spin_until(node, lambda: False, 0.1)
        goals.publish(pose(7.0))
        odoms.publish(odom(0.65, horizontal_speed=0.09))
        spin_until(node, lambda: False, 0.3)
        if len(released) != 3:
            raise AssertionError('Goal Bridge released with excessive horizontal speed')
        odoms.publish(odom(0.65))
        spin_until(node, lambda: False, 0.1)
        odoms.publish(odom(0.65))
        spin_until(node, lambda: False, 0.1)
        odoms.publish(odom(0.65))
        if not spin_until(node, lambda: len(released) == 4, 3.0):
            raise AssertionError('Goal Bridge did not release after speed became safe')

        # Rolling local goals must not wait through another stable-duration
        # window once the initial takeoff handover has completed.
        goals.publish(pose(1.0))
        if not spin_until(node, lambda: len(released) == 5, 0.5):
            raise AssertionError('Goal Bridge gated a rolling local_goal after takeoff handover')
        if abs(released[4].pose.position.x - 1.0) > 1e-6:
            raise AssertionError('Goal Bridge did not forward the rolling local_goal')
        print('PASS goal_bridge_ground_cache_then_latest_release')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        bridge.terminate()
        try:
            bridge.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            bridge.kill()
            bridge.wait()
        del release_subscription


if __name__ == '__main__':
    main()
