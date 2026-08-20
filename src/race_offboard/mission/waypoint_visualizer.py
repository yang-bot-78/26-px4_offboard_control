#!/usr/bin/env python3
"""Publish saved map-frame waypoints as persistent RViz markers."""

from __future__ import annotations

import argparse
import math
import re
from pathlib import Path
from typing import Any

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray

from waypoint_common import default_waypoints_file, load_waypoints


_FIXED_ROUTE_NAMES = ('B1', 'B2', 'C', 'D')
_FIXED_ROUTE_ORDER = {name: index for index, name in enumerate(_FIXED_ROUTE_NAMES)}
_NUMBERED_WAYPOINT = re.compile(r'^P([0-9]+)$', re.IGNORECASE)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--waypoints-file', default=None)
    parser.add_argument('--topic', default='/race/saved_waypoints')
    parser.add_argument('--refresh-sec', type=float, default=1.0)
    # launch_ros appends ``--ros-args`` for the node name/remappings. Keep
    # those arguments for rclpy.init() instead of rejecting the visualizer.
    return parser.parse_known_args()[0]


def _ordered_waypoint_items(waypoints: dict[str, Any]) -> list[tuple[str, Any]]:
    """Return the display/route order independently of YAML insertion history."""
    fixed: list[tuple[int, int, str, Any]] = []
    numbered: list[tuple[int, int, str, Any]] = []
    custom: list[tuple[str, Any]] = []
    for insertion_order, (name, value) in enumerate(waypoints.items()):
        text_name = str(name)
        fixed_order = _FIXED_ROUTE_ORDER.get(text_name)
        if fixed_order is not None:
            fixed.append((fixed_order, insertion_order, text_name, value))
            continue
        match = _NUMBERED_WAYPOINT.fullmatch(text_name)
        if match:
            numbered.append((int(match.group(1)), insertion_order, text_name, value))
        else:
            custom.append((text_name, value))
    fixed.sort(key=lambda item: (item[0], item[1]))
    numbered.sort(key=lambda item: (item[0], item[1]))
    return ([(name, value) for _, _, name, value in fixed]
            + [(name, value) for _, _, name, value in numbered]
            + custom)


def valid_waypoints(document: dict[str, Any]) -> list[tuple[str, float, float, float]]:
    """Return only finite map-frame waypoints in the tracked-route order."""
    if document.get('frame_id') != 'map':
        raise ValueError("waypoint file frame_id must be 'map'")
    points: list[tuple[str, float, float, float]] = []
    for name, value in _ordered_waypoint_items(document['waypoints']):
        if not isinstance(value, dict):
            raise ValueError(f'waypoint {name!r} is not a mapping')
        coordinates = tuple(float(value.get(axis)) for axis in ('x', 'y', 'z'))
        if not all(math.isfinite(coordinate) for coordinate in coordinates):
            raise ValueError(f'waypoint {name!r} has a non-finite coordinate')
        points.append((str(name), *coordinates))
    return points


class WaypointVisualizer(Node):
    def __init__(self, waypoints_file: Path, topic: str, refresh_sec: float) -> None:
        super().__init__('waypoint_visualizer')
        self._waypoints_file = waypoints_file
        self._last_signature: tuple[int, int] | None = None
        self._last_error: str | None = None
        self._publisher = self.create_publisher(
            MarkerArray, topic,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(refresh_sec, self._refresh)
        self._refresh()

    def _refresh(self) -> None:
        try:
            status = self._waypoints_file.stat()
            signature = (status.st_mtime_ns, status.st_size)
            if signature == self._last_signature:
                return
            points = valid_waypoints(load_waypoints(self._waypoints_file))
        except (OSError, ValueError, TypeError) as error:
            message = str(error)
            if message != self._last_error:
                self.get_logger().warning(f'无法读取保存航点，已清空 RViz 标记：{message}')
            self._last_error = message
            self._last_signature = None
            self._publish([])
            return

        self._last_signature = signature
        self._last_error = None
        self._publish(points)
        self.get_logger().info(f'已在 RViz 更新 {len(points)} 个保存航点。')

    def _publish(self, points: list[tuple[str, float, float, float]]) -> None:
        now = self.get_clock().now().to_msg()
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        if len(points) >= 2:
            route = Marker()
            route.header.frame_id = 'map'
            route.header.stamp = now
            route.ns = 'saved_waypoints_route'
            route.id = 0
            route.type = Marker.LINE_STRIP
            route.action = Marker.ADD
            route.scale.x = 0.045
            route.color.r = 1.0
            route.color.g = 0.65
            route.color.b = 0.0
            route.color.a = 0.95
            route.points = [Point(x=x, y=y, z=z + 0.03) for _, x, y, z in points]
            markers.markers.append(route)

        for index, (name, x, y, z) in enumerate(points):
            point = Marker()
            point.header.frame_id = 'map'
            point.header.stamp = now
            point.ns = 'saved_waypoints_point'
            point.id = index
            point.type = Marker.SPHERE
            point.action = Marker.ADD
            point.pose.position.x = x
            point.pose.position.y = y
            point.pose.position.z = z
            point.pose.orientation.w = 1.0
            point.scale.x = 0.22
            point.scale.y = 0.22
            point.scale.z = 0.22
            point.color.r = 0.0
            point.color.g = 0.9
            point.color.b = 1.0
            point.color.a = 1.0
            markers.markers.append(point)

            label = Marker()
            label.header.frame_id = 'map'
            label.header.stamp = now
            label.ns = 'saved_waypoints_label'
            label.id = index
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = x
            label.pose.position.y = y
            label.pose.position.z = z + 0.20
            label.pose.orientation.w = 1.0
            label.scale.z = 0.18
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 1.0
            label.text = name
            markers.markers.append(label)

        self._publisher.publish(markers)


def main() -> int:
    args = parse_args()
    if args.refresh_sec <= 0.0:
        raise SystemExit('--refresh-sec must be positive')
    waypoints_file = (Path(args.waypoints_file).expanduser().resolve()
                      if args.waypoints_file else default_waypoints_file())
    rclpy.init()
    node = WaypointVisualizer(waypoints_file, args.topic, args.refresh_sec)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
