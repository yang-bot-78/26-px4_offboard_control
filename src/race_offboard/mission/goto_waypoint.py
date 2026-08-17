#!/usr/bin/env python3
"""Publish a saved waypoint to the existing /goal_pose planner input."""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped

from waypoint_common import default_waypoints_file, load_waypoints, resolve_map_file, sha256_file


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("name")
    parser.add_argument("--waypoints-file", default=None)
    parser.add_argument("--map-file", default=None)
    parser.add_argument("--topic", default="/goal_pose")
    parser.add_argument("--publish-count", type=int, default=3)
    args = parser.parse_args()
    path = Path(args.waypoints_file).expanduser().resolve() if args.waypoints_file else default_waypoints_file()
    try:
        data = load_waypoints(path)
        point = data["waypoints"][args.name]
    except Exception as error:
        print(f"[拒绝执行] 无法读取航点 {args.name!r}: {error}", file=sys.stderr)
        return 2
    if data.get("frame_id", "map") != "map":
        print(f"[拒绝执行] waypoint frame_id={data.get('frame_id')!r}，要求 map", file=sys.stderr)
        return 2
    recorded_sha = data.get("map_pcd_sha256")
    map_path = resolve_map_file(args.map_file)
    current_sha = sha256_file(map_path) if map_path else None
    if recorded_sha and not current_sha:
        print("[拒绝执行] 航点带有 PCD SHA256，但当前地图文件不可确定，无法验证地图身份。", file=sys.stderr)
        return 2
    if recorded_sha and current_sha != recorded_sha:
        print(f"[拒绝执行] 地图 SHA256 不一致：记录={recorded_sha} 当前={current_sha}", file=sys.stderr)
        return 2
    if not recorded_sha:
        print("[警告] 航点没有 PCD SHA256，无法进行地图身份校验。", file=sys.stderr)
    try:
        x, y, z, yaw = (float(point[key]) for key in ("x", "y", "z", "yaw"))
    except (KeyError, TypeError, ValueError) as error:
        print(f"[拒绝执行] 航点数值无效: {error}", file=sys.stderr)
        return 2
    if not all(math.isfinite(value) for value in (x, y, z, yaw)):
        print("[拒绝执行] 航点包含非有限数值。", file=sys.stderr)
        return 2
    rclpy.init()
    node = rclpy.create_node("goto_waypoint")
    publisher = node.create_publisher(PoseStamped, args.topic, 10)
    message = PoseStamped()
    message.header.frame_id = "map"
    message.pose.position.x, message.pose.position.y, message.pose.position.z = x, y, z
    message.pose.orientation.z = math.sin(yaw / 2.0)
    message.pose.orientation.w = math.cos(yaw / 2.0)
    try:
        for _ in range(max(1, args.publish_count)):
            message.header.stamp = node.get_clock().now().to_msg()
            publisher.publish(message)
            rclpy.spin_once(node, timeout_sec=0.05)
            time.sleep(0.05)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    print(f"已发布航点 {args.name} 到 {args.topic}: x={x:.3f} y={y:.3f} z={z:.3f} yaw={yaw:.3f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
