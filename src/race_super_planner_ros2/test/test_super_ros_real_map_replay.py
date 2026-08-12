#!/usr/bin/env python3
# flake8: noqa
"""Launch the real Super node and replay the recorded start/goal against the saved PCD."""
import argparse
import json
import struct
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as RosPath
from sensor_msgs.msg import PointCloud2, PointField


ROOT = Path(__file__).resolve().parents[3]
PCD = ROOT / 'src/FAST_LIO/PCD/scans_20260603_1916.pcd'
PARAMS = ROOT / 'src/race_super_planner_ros2/config/navigation.yaml'
EXECUTABLE = ROOT / 'build/race_super_planner_ros2/super_planner_ros2_node'


def load_xyz():
    data = PCD.read_bytes()
    marker = b'DATA binary\n'
    offset = data.index(marker) + len(marker)
    header = data[:offset].decode()
    width = int(next(line.split()[1] for line in header.splitlines() if line.startswith('POINTS ')))
    # Saved PCD has eight float fields; x/y/z are the first three.
    return b''.join(struct.pack('<fff', *struct.unpack_from('<fff', data, offset + i * 32))
                    for i in range(width))


def parse_args():
    parser = argparse.ArgumentParser(
        description='Replay one recorded Super start/goal with the production node and saved PCD.')
    parser.add_argument('--start', nargs=3, type=float, metavar=('X', 'Y', 'Z'),
                        default=(0.02, 0.0, 0.78))
    parser.add_argument('--goal', nargs=3, type=float, metavar=('X', 'Y', 'Z'),
                        default=(10.923, 0.012, 0.78))
    parser.add_argument('--record', type=Path,
                        help='write a self-contained offline replay record (JSON)')
    parser.add_argument('--require-local-goal', action='store_true',
                        help='also require the production rolling local_goal output')
    return parser.parse_args()


def main():
    args = parse_args()
    if not EXECUTABLE.is_file() or not PCD.is_file():
        raise RuntimeError('build Super first and keep the saved PCD at its canonical path')
    command = [str(EXECUTABLE), '--ros-args', '--params-file', str(PARAMS),
               '-p', 'odom_topic:=/super_replay/odom', '-p', 'fallback_odom_topic:=/super_replay/odom',
               '-p', 'cloud_topic:=/super_replay/cloud', '-p', 'fallback_cloud_topic:=/super_replay/cloud',
               '-p', 'goal_topic:=/super_replay/goal', '-p', 'odom_input_frame:=map',
               '-p', 'global_path_topic:=/super_replay/global_path',
               '-p', 'prefer_px4_local_position:=false', '-p', 'global_only_mode:=true',
               '-p', 'publish_global_path:=true',
               '-p', f'publish_ego_local_goal:={str(args.require_local_goal).lower()}',
               '-p', 'ego_local_goal_topic:=/super_replay/local_goal',
               '-p', 'enable_output:=false', '-p', 'planning_resolution:=0.05',
               '-p', 'required_center_clearance:=0.466',
               # Tests stay on the production-safe A* baseline even if a
               # developer has a local parameter-file override.
               '-p', 'allow_direct_path:=false',
               '-p', 'shared_bounds/x_min:=-0.85', '-p', 'shared_bounds/x_max:=11.90',
               '-p', 'shared_bounds/y_min:=-1.30', '-p', 'shared_bounds/y_max:=12.40',
               '-p', 'shared_bounds/z_min:=0.50', '-p', 'shared_bounds/z_max:=0.85']
    proc = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    rclpy.init()
    node = rclpy.create_node('super_real_map_replay')
    cloud_pub = node.create_publisher(PointCloud2, '/super_replay/cloud', 10)
    odom_pub = node.create_publisher(Odometry, '/super_replay/odom', 10)
    goal_pub = node.create_publisher(PoseStamped, '/super_replay/goal', 10)
    received = []
    local_goals = []
    path_subscription = node.create_subscription(
        RosPath, '/super_replay/global_path', lambda msg: received.append(msg), 10)
    local_goal_subscription = node.create_subscription(
        PoseStamped, '/super_replay/local_goal', lambda msg: local_goals.append(msg), 10)
    cloud = PointCloud2(); cloud.header.frame_id = 'map'; cloud.height = 1; cloud.width = 109147
    cloud.point_step = 12; cloud.row_step = cloud.width * cloud.point_step; cloud.is_dense = True
    cloud.fields = [PointField(name=n, offset=i * 4, datatype=PointField.FLOAT32, count=1)
                    for i, n in enumerate(('x', 'y', 'z'))]
    cloud.data = load_xyz()
    odom = Odometry(); odom.header.frame_id = 'map'; odom.pose.pose.position.x = args.start[0]
    odom.pose.pose.position.y = args.start[1]; odom.pose.pose.position.z = args.start[2]
    odom.pose.pose.orientation.w = 1.0
    goal = PoseStamped(); goal.header.frame_id = 'map'; goal.pose.position.x = args.goal[0]
    goal.pose.position.y = args.goal[1]; goal.pose.position.z = args.goal[2]
    goal.pose.orientation.w = 1.0
    deadline = time.monotonic() + 12.0
    while time.monotonic() < deadline and (not received or (args.require_local_goal and not local_goals)):
        cloud.header.stamp = node.get_clock().now().to_msg(); odom.header.stamp = cloud.header.stamp
        goal.header.stamp = cloud.header.stamp
        cloud_pub.publish(cloud); odom_pub.publish(odom); goal_pub.publish(goal)
        rclpy.spin_once(node, timeout_sec=0.1)
    proc.terminate(); output = proc.communicate(timeout=5)[0]
    del path_subscription
    del local_goal_subscription
    node.destroy_node(); rclpy.shutdown()
    record = {
        'schema': 'super_direct_repro_record/v1',
        'start': list(args.start), 'goal': list(args.goal),
        'pcd_path': str(PCD), 'pcd_point_count': 109147,
        'parameters': {
            'allow_direct_path': False, 'global_only_mode': True,
            'planning_resolution': 0.05, 'required_center_clearance': 0.466,
            'query_height': args.goal[2],
            'shared_bounds': {'x': [-0.85, 11.90], 'y': [-1.30, 12.40], 'z': [0.50, 0.85]},
        },
        'filtered_cloud_summary': 'production Super PointCloud2 callback; see node_log_tail',
        'first_failure_sample': None, 'nearest_obstacle': None,
        'node_log_tail': output[-4000:],
    }
    if '[SUPER_DIRECT_MODE] enabled=false' not in output:
        raise RuntimeError('Super did not confirm DIRECT=false runtime mode\n' + output[-4000:])
    if not received or len(received[-1].poses) < 2:
        if args.record:
            args.record.write_text(json.dumps(record, indent=2) + '\n')
        raise RuntimeError('real Super node did not publish a non-empty /race/global_path\n' + output[-4000:])
    if args.require_local_goal and not local_goals:
        raise RuntimeError('real Super node published a path but no rolling local_goal\n' + output[-4000:])
    endpoint = received[-1].poses[-1].pose.position
    error = ((endpoint.x - goal.pose.position.x) ** 2 + (endpoint.y - goal.pose.position.y) ** 2) ** 0.5
    if error > 0.05:
        raise RuntimeError(f'global endpoint error {error:.3f}m > 0.05m')
    record['result'] = {
        'published_global_path': True, 'path_points': len(received[-1].poses),
        'endpoint': [endpoint.x, endpoint.y, endpoint.z], 'endpoint_error_m': error,
        'global_path': [
            [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]
            for pose in received[-1].poses
        ],
        'first_local_goal': ([local_goals[0].pose.position.x, local_goals[0].pose.position.y,
                              local_goals[0].pose.position.z] if local_goals else None),
        'local_goals': [
            [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]
            for pose in local_goals
        ],
    }
    if args.record:
        args.record.write_text(json.dumps(record, indent=2) + '\n')
    print(f'PASS global_path_points={len(received[-1].poses)} endpoint_error={error:.4f}m '
          f'start={tuple(args.start)} goal={tuple(args.goal)} direct=false')


if __name__ == '__main__':
    main()
