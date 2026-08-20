#!/usr/bin/env python3
"""
ROS integration check for the Goal Bridge background-planning height gate.

This deliberately starts only the real Goal Bridge: no PX4, Gazebo, or
planner is required. It proves that ground goals are cached, only the latest
one survives, and 0.40 m AGL releases planning during an armed takeoff without
depending on horizontal or vertical speed.
"""

import subprocess
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from race_msgs.msg import FlightAltitudeReference
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


GROUND_Z_MAP = -0.14
TARGET_AGL = 0.78
TARGET_Z_MAP = GROUND_Z_MAP + TARGET_AGL


def pose(x, y=0.0, z=0.78):
    msg = PoseStamped()
    msg.header.frame_id = 'map'
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.position.z = z
    msg.pose.orientation.w = 1.0
    return msg


def odom(agl, horizontal_speed=0.0, vertical_speed=0.0):
    msg = Odometry()
    msg.header.frame_id = 'map'
    msg.pose.pose.position.z = GROUND_Z_MAP + agl
    msg.pose.pose.orientation.w = 1.0
    msg.twist.twist.linear.x = horizontal_speed
    msg.twist.twist.linear.z = vertical_speed
    return msg


def altitude_reference(flight_id, valid=True):
    msg = FlightAltitudeReference()
    msg.flight_id = flight_id
    msg.valid = valid
    msg.target_agl_m = TARGET_AGL
    msg.min_agl_m = 0.50
    msg.max_agl_m = 0.90
    msg.ground_z_local_ned = -GROUND_Z_MAP
    msg.target_z_local_ned = msg.ground_z_local_ned - TARGET_AGL
    msg.ground_z_map = GROUND_Z_MAP
    msg.target_z_map = TARGET_Z_MAP
    return msg


def vehicle_state(armed):
    msg = State()
    msg.connected = True
    msg.armed = armed
    msg.mode = 'OFFBOARD' if armed else 'STABILIZED'
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
        '-p', 'velocity_topic:=/test/takeoff_gate/velocity_odom',
        '-p', 'release_height:=0.40',
        '-p', 'flight_height:=0.78',
        '-p', 'stable_duration_sec:=0.15',
    ])
    rclpy.init()
    node = rclpy.create_node('test_goal_bridge_takeoff_release')
    released = []
    goals = node.create_publisher(PoseStamped, '/test/takeoff_gate/local_goal', 10)
    odoms = node.create_publisher(Odometry, '/test/takeoff_gate/odom', 10)
    velocity_odoms = node.create_publisher(
        Odometry, '/test/takeoff_gate/velocity_odom', 10)
    states = node.create_publisher(State, '/mavros/state', 10)
    reference_qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    references = node.create_publisher(
        FlightAltitudeReference, '/race/flight_altitude_reference', reference_qos)
    release_subscription = node.create_subscription(
        PoseStamped, '/test/takeoff_gate/ego_goal', lambda msg: released.append(msg), 10)
    try:
        if not spin_until(node, lambda: goals.get_subscription_count() >= 1, 15.0):
            raise AssertionError('real Goal Bridge did not subscribe')
        if not spin_until(node, lambda: references.get_subscription_count() >= 1, 5.0):
            raise AssertionError('real Goal Bridge did not subscribe to altitude reference')
        if not spin_until(node, lambda: velocity_odoms.get_subscription_count() >= 1, 5.0):
            raise AssertionError('real Goal Bridge did not subscribe to PX4 velocity odometry')

        def publish_kinematics(pose_odom, velocity_odom=None):
            odoms.publish(pose_odom)
            velocity_odoms.publish(velocity_odom or pose_odom)

        references.publish(altitude_reference(1))
        spin_until(node, lambda: False, 0.1)
        states.publish(vehicle_state(False))
        publish_kinematics(odom(0.0))
        goals.publish(pose(1.0))
        time.sleep(0.1)
        goals.publish(pose(2.0))
        spin_until(node, lambda: False, 0.5)
        if released:
            raise AssertionError('Goal Bridge released a ground-level local_goal')
        # The background planner must not start below 0.40 m, even with a fresh
        # velocity source. It also must not start while disarmed.
        publish_kinematics(odom(0.399), odom(0.399))
        spin_until(node, lambda: False, 0.3)
        if released:
            raise AssertionError('Goal Bridge released below 0.40 m AGL')
        states.publish(vehicle_state(False))
        publish_kinematics(
            odom(0.40, horizontal_speed=0.50, vertical_speed=0.80),
            odom(0.40, horizontal_speed=0.50, vertical_speed=0.80))
        spin_until(node, lambda: False, 0.3)
        if released:
            raise AssertionError('Goal Bridge released while disarmed')

        # During armed takeoff, reaching 0.40 m releases the latest cached goal.
        # Large vertical speed is normal here and deliberately does not gate
        # background planning.
        states.publish(vehicle_state(True))
        publish_kinematics(
            odom(0.40, horizontal_speed=0.50, vertical_speed=0.80),
            odom(0.40, horizontal_speed=0.50, vertical_speed=0.80))
        if not spin_until(node, lambda: len(released) == 1, 3.0):
            raise AssertionError('Goal Bridge did not release at 0.40 m during takeoff')
        if abs(released[0].pose.position.x - 2.0) > 1e-6:
            raise AssertionError('Goal Bridge released the wrong cached local_goal')
        if abs(released[0].pose.position.z - TARGET_Z_MAP) > 1e-6:
            raise AssertionError('Goal Bridge treated AGL as absolute map altitude')

        # Rolling local goals must not wait through another stable-duration
        # window once the initial takeoff handover has completed.
        goals.publish(pose(1.0))
        if not spin_until(node, lambda: len(released) == 2, 0.5):
            raise AssertionError('Goal Bridge gated a rolling local_goal after takeoff handover')
        if abs(released[1].pose.position.x - 1.0) > 1e-6:
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
