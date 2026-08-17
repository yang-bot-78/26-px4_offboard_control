#!/usr/bin/env python3
"""Save a stable, health-gated waypoint from /race/odom."""

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
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Bool, String

from waypoint_common import (atomic_write_yaml, default_waypoints_file, load_waypoints,
                             resolve_map_file, sha256_file, quaternion_to_yaw, wrap_yaw)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--name", required=True)
    parser.add_argument("--mode", choices=("xy", "xyz"), required=True)
    parser.add_argument("--flight-z", type=float, default=0.78)
    parser.add_argument("--force", action="store_true")
    parser.add_argument("--window", type=float, default=1.0)
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--max-age", type=float, default=0.40)
    parser.add_argument("--max-speed", type=float, default=0.15)
    parser.add_argument("--max-stddev", type=float, default=0.05)
    parser.add_argument("--min-samples", type=int, default=5)
    parser.add_argument("--map-file", default=None)
    parser.add_argument("--odom-topic", default="/race/odom")
    parser.add_argument("--health-topic", default="/frlio/high_rate_odom/status")
    parser.add_argument("--planner-usable-topic", default="/frlio/high_rate_odom/planner_usable")
    parser.add_argument("--flight-ready-topic", default="/ev_health/flight_ready")
    parser.add_argument("--health-max-age", type=float, default=1.0)
    parser.add_argument("--waypoints-file", default=None)
    return parser.parse_args()


class Collector:
    def __init__(self, args: argparse.Namespace) -> None:
        self.node = rclpy.create_node("save_waypoint")
        self.args = args
        self.samples = []
        self.last_stamp = None
        self.timestamp_violation = False
        self.frame_violation = False
        self.last_receive = 0.0
        self.health = None
        self.health_receive = 0.0
        self.planner_usable = None
        self.planner_receive = 0.0
        self.flight_ready = None
        self.flight_receive = 0.0
        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
        status_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(Odometry, args.odom_topic, self.odom_cb, qos)
        self.node.create_subscription(String, args.health_topic, self.health_cb, status_qos)
        self.node.create_subscription(Bool, args.planner_usable_topic, self.planner_cb, status_qos)
        self.node.create_subscription(Bool, args.flight_ready_topic, self.ready_cb, status_qos)

    def now(self) -> float:
        return self.node.get_clock().now().nanoseconds / 1e9

    def odom_cb(self, msg: Odometry) -> None:
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        receive = self.now()
        if stamp <= 0:
            self.timestamp_violation = True
            return
        if msg.header.frame_id != "map":
            self.frame_violation = True
            self.node.get_logger().error(f"拒绝：/race/odom frame_id={msg.header.frame_id!r}，要求 map")
            return
        if self.last_stamp is not None and stamp <= self.last_stamp:
            self.timestamp_violation = True
            return
        self.last_stamp = stamp
        self.last_receive = receive
        p = msg.pose.pose.position
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        v = msg.twist.twist.linear
        speed = math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)
        self.samples.append((stamp, p.x, p.y, p.z, yaw, speed))
        cutoff = stamp - self.args.window
        self.samples = [sample for sample in self.samples if sample[0] >= cutoff]

    def health_cb(self, msg: String) -> None:
        self.health, self.health_receive = msg.data, self.now()

    def planner_cb(self, msg: Bool) -> None:
        self.planner_usable, self.planner_receive = bool(msg.data), self.now()

    def ready_cb(self, msg: Bool) -> None:
        self.flight_ready, self.flight_receive = bool(msg.data), self.now()

    def close(self) -> None:
        self.node.destroy_node()


def fail(message: str) -> int:
    print(f"[拒绝保存] {message}", file=sys.stderr)
    return 2


def main() -> int:
    args = parse_args()
    if not args.name or any(char in args.name for char in "/\\"):
        return fail("waypoint 名称不能为空且不能包含路径分隔符")
    if args.window <= 0 or args.timeout < args.window or args.flight_z != args.flight_z:
        return fail("window/timeout/flight-z 参数无效")
    path = Path(args.waypoints_file).expanduser().resolve() if args.waypoints_file else default_waypoints_file()
    try:
        data = load_waypoints(path)
    except Exception as error:
        return fail(str(error))
    if args.name in data["waypoints"] and not args.force:
        return fail(f"已存在同名航点 {args.name!r}，需要 --force 才能覆盖")
    map_path = resolve_map_file(args.map_file)
    map_sha = sha256_file(map_path) if map_path else None
    if not map_path:
        print("[警告] 未能可靠确定当前 PCD（请设置 MAP_FILE 或 --map-file），将记录空 SHA。", file=sys.stderr)
    collector = Collector(args)
    start = time.monotonic()
    try:
        while time.monotonic() - start < args.timeout:
            rclpy.spin_once(collector.node, timeout_sec=0.10)
            if collector.samples and collector.samples[-1][0] - collector.samples[0][0] >= args.window:
                break
        samples = collector.samples
        if len(samples) < args.min_samples:
            return fail(f"/race/odom 样本不足：{len(samples)} < {args.min_samples}")
        now = collector.now()
        if collector.frame_violation:
            return fail("/race/odom 存在非 map frame 的样本")
        if collector.timestamp_violation:
            return fail("/race/odom 时间戳不单调或无效")
        if now - collector.last_receive > args.max_age:
            return fail(f"/race/odom 不新鲜：{now - collector.last_receive:.3f}s")
        stamp_age = now - collector.last_stamp
        if stamp_age > args.max_age or stamp_age < -args.max_age:
            return fail(f"/race/odom header 时间戳不新鲜：age={stamp_age:.3f}s")
        if any(samples[index][0] <= samples[index - 1][0] for index in range(1, len(samples))):
            return fail("/race/odom 时间戳不单调")
        if any(not math.isfinite(value) for sample in samples for value in sample):
            return fail("/race/odom 包含非有限数值")
        health_age = now - collector.health_receive
        if collector.health is None or health_age > args.health_max_age:
            return fail(f"FR-LIO health 不新鲜或缺失（age={health_age:.3f}s）")
        if collector.health not in ("HEALTHY", "SUSPECT_STALE_LIDAR", "DEGRADED"):
            return fail(f"FR-LIO health={collector.health!r}")
        if collector.planner_usable is not True or now - collector.planner_receive > args.health_max_age:
            return fail("FR-LIO planner_usable 不为 true 或已过期")
        if collector.flight_ready is not True or now - collector.flight_receive > args.health_max_age:
            return fail("/ev_health/flight_ready 不为 true 或已过期")
        max_speed = max(sample[5] for sample in samples)
        for previous, current in zip(samples, samples[1:]):
            dt = current[0] - previous[0]
            if dt <= 0:
                return fail("/race/odom 时间戳不单调")
            max_speed = max(max_speed, math.sqrt(sum((current[i] - previous[i]) ** 2 for i in (1, 2, 3))) / dt)
        if max_speed > args.max_speed:
            return fail(f"飞机未静止：最大速度 {max_speed:.3f}m/s > {args.max_speed:.3f}m/s")
        means = [statistics.median(sample[index] for sample in samples) for index in (1, 2, 3)]
        stddev = max(statistics.pstdev(sample[index] for sample in samples) for index in (1, 2, 3))
        if stddev > args.max_stddev:
            return fail(f"位置标准差过大：{stddev:.4f}m > {args.max_stddev:.4f}m")
        yaw = math.atan2(statistics.fmean(math.sin(sample[4]) for sample in samples),
                         statistics.fmean(math.cos(sample[4]) for sample in samples))
        if args.mode == "xy":
            means[2] = args.flight_z
        result = {"frame_id": "map", "map_pcd_file": str(map_path) if map_path else None,
                  "map_pcd_filename": map_path.name if map_path else None,
                  "map_pcd_sha256": map_sha, "default_flight_height": args.flight_z,
                  "waypoints": data["waypoints"]}
        result["waypoints"][args.name] = {
            "mode": args.mode, "x": float(means[0]), "y": float(means[1]), "z": float(means[2]),
            "yaw": float(wrap_yaw(yaw)), "recorded_at": datetime.now(timezone.utc).isoformat(),
            "sample_count": len(samples), "position_stddev_m": float(stddev),
            "max_speed_mps": float(max_speed),
        }
        atomic_write_yaml(path, result)
        print(f"已保存航点 {args.name}: x={means[0]:.3f} y={means[1]:.3f} z={means[2]:.3f} yaw={yaw:.3f}")
        return 0
    finally:
        collector.close()


if __name__ == "__main__":
    rclpy.init()
    try:
        raise SystemExit(main())
    finally:
        rclpy.shutdown()
