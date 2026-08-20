#!/usr/bin/env python3
"""Save one stable map/ENU waypoint for later mission export."""

from __future__ import annotations

import argparse
import math
import statistics
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

from waypoint_common import atomic_write_yaml, default_waypoints_file, load_waypoints, sha256_file


class SampleCollector:
    def __init__(self, args: argparse.Namespace) -> None:
        self.node = rclpy.create_node('save_waypoint')
        self.args = args
        self.samples: list[tuple[float, float, float, float]] = []
        self.last_stamp: float | None = None
        self.last_receive = 0.0
        self.invalid_frame = False
        self.invalid_stamp = False
        self.flight_ready = False
        self.flight_ready_receive = 0.0
        self.node.create_subscription(
            Odometry, args.odom_topic, self._odom_callback,
            QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE))
        self.node.create_subscription(
            Bool, args.flight_ready_topic, self._ready_callback,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def now(self) -> float:
        return self.node.get_clock().now().nanoseconds / 1e9

    def _odom_callback(self, message: Odometry) -> None:
        stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
        if message.header.frame_id != 'map':
            self.invalid_frame = True
            return
        if stamp <= 0 or (self.last_stamp is not None and stamp <= self.last_stamp):
            self.invalid_stamp = True
            return
        point = message.pose.pose.position
        self.last_stamp = stamp
        self.last_receive = self.now()
        self.samples.append((stamp, point.x, point.y, point.z))
        self.samples = [sample for sample in self.samples if stamp - sample[0] <= self.args.window]

    def _ready_callback(self, message: Bool) -> None:
        self.flight_ready = bool(message.data)
        self.flight_ready_receive = self.now()

    def close(self) -> None:
        self.node.destroy_node()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--name', required=True,
        help='B1, B2, C, D or another unique saved-point name')
    parser.add_argument('--mode', choices=('xy', 'xyz'), default='xy')
    parser.add_argument('--flight-z', type=float, default=0.78)
    parser.add_argument('--waypoints-file', default=None)
    parser.add_argument('--map-file', default=None)
    parser.add_argument('--odom-topic', default='/race/odom')
    parser.add_argument('--flight-ready-topic', default='/ev_health/flight_ready')
    parser.add_argument(
        '--require-flight-ready', action='store_true',
        help='reject saving unless the flight-ready topic is true and fresh')
    parser.add_argument('--window', type=float, default=1.0)
    parser.add_argument('--timeout', type=float, default=5.0)
    parser.add_argument('--max-age', type=float, default=0.40)
    parser.add_argument('--max-stddev', type=float, default=0.05)
    parser.add_argument('--min-samples', type=int, default=5)
    parser.add_argument('--force', action='store_true')
    return parser.parse_args()


def fail(reason: str) -> int:
    print(f'[WAYPOINT_SAVE_REJECTED] {reason}', file=sys.stderr)
    return 2


def main() -> int:
    args = parse_args()
    if (not args.name or any(character in args.name for character in '/\\') or
            args.window <= 0 or args.timeout < args.window or args.min_samples < 2 or
            not math.isfinite(args.flight_z)):
        return fail('invalid name or sampling parameters')
    destination = (Path(args.waypoints_file).expanduser().resolve()
                   if args.waypoints_file else default_waypoints_file())
    try:
        document = load_waypoints(destination)
    except (OSError, ValueError) as error:
        return fail(str(error))
    if document['frame_id'] != 'map':
        return fail(f'waypoint file frame_id={document["frame_id"]!r}, expected map')
    if args.name in document['waypoints'] and not args.force:
        return fail(f'waypoint {args.name!r} exists; use --force to replace it')
    map_path = Path(args.map_file).expanduser().resolve() if args.map_file else None
    if map_path is not None and not map_path.is_file():
        return fail(f'map file does not exist: {map_path}')

    collector = SampleCollector(args)
    try:
        started = time.monotonic()
        while time.monotonic() - started < args.timeout:
            rclpy.spin_once(collector.node, timeout_sec=0.1)
            if (collector.samples and
                    collector.samples[-1][0] - collector.samples[0][0] >= args.window):
                break
        now = collector.now()
        if collector.invalid_frame:
            return fail('/race/odom must use frame_id=map')
        if collector.invalid_stamp or collector.last_stamp is None:
            return fail('/race/odom timestamps are missing or non-monotonic')
        if len(collector.samples) < args.min_samples:
            return fail(f'insufficient samples: {len(collector.samples)} < {args.min_samples}')
        if (now - collector.last_receive > args.max_age or
                now - collector.last_stamp > args.max_age):
            return fail('/race/odom is stale')
        if (args.require_flight_ready and
                (not collector.flight_ready or now - collector.flight_ready_receive > 1.0)):
            return fail('/ev_health/flight_ready is false or stale')
        if any(not math.isfinite(value) for sample in collector.samples for value in sample):
            return fail('/race/odom contains a non-finite coordinate')
        values = [
            statistics.median(sample[index] for sample in collector.samples)
            for index in (1, 2, 3)]
        standard_deviation = max(
            statistics.pstdev(sample[index] for sample in collector.samples)
            for index in (1, 2, 3))
        if standard_deviation > args.max_stddev:
            return fail(f'position standard deviation {standard_deviation:.3f}m exceeds limit')
        if args.mode == 'xy':
            values[2] = args.flight_z
        document.update({
            'frame_id': 'map',
            'map_pcd_file': str(map_path) if map_path else None,
            'map_pcd_sha256': sha256_file(map_path),
            'default_flight_height': args.flight_z,
        })
        document['waypoints'][args.name] = {
            'mode': args.mode, 'x': values[0], 'y': values[1], 'z': values[2],
            'recorded_at': datetime.now(timezone.utc).isoformat(),
            'sample_count': len(collector.samples),
            'position_stddev_m': standard_deviation,
        }
        atomic_write_yaml(destination, document)
        print(f'航点保存成功：名称={args.name} x={values[0]:.3f} y={values[1]:.3f}')
        return 0
    finally:
        collector.close()


if __name__ == '__main__':
    rclpy.init()
    try:
        raise SystemExit(main())
    finally:
        rclpy.shutdown()
