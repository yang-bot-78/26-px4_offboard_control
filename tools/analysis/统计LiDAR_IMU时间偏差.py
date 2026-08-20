#!/usr/bin/env python3
"""Measure sensor-clock offset between Livox LiDAR frames and IMU samples.

The default LiDAR timestamp is CustomMsg.timebase (the first-point sensor
timestamp), not the ROS receive time.  Each LiDAR frame is paired with the
nearest IMU header stamp.  P95/P99 are reported for absolute offset magnitude;
the signed mean and standard deviation are reported separately.
"""

from __future__ import annotations

import argparse
from bisect import bisect_left
from collections import deque
import json
import math
import statistics
import time
from typing import Deque, List, Optional, Tuple


def _percentile(values: List[float], fraction: float) -> float:
    if not values:
        return math.nan
    ordered = sorted(values)
    index = (len(ordered) - 1) * fraction
    lower = int(math.floor(index))
    upper = int(math.ceil(index))
    if lower == upper:
        return ordered[lower]
    weight = index - lower
    return ordered[lower] + weight * (ordered[upper] - ordered[lower])


def _stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _summarize(
    args: argparse.Namespace,
    values: List[float],
    elapsed: List[float],
    unmatched: int,
    out_of_order_lidar: int,
    source: Optional[str] = None,
) -> dict:
    abs_values = [abs(value) for value in values]
    if values:
        mean_s = statistics.fmean(values)
        std_s = statistics.pstdev(values) if len(values) > 1 else 0.0
        signed_p95_s = _percentile(values, 0.95)
        signed_p99_s = _percentile(values, 0.99)
        abs_p95_s = _percentile(abs_values, 0.95)
        abs_p99_s = _percentile(abs_values, 0.99)
    else:
        mean_s = std_s = signed_p95_s = signed_p99_s = math.nan
        abs_p95_s = abs_p99_s = math.nan

    slope_s_per_s = math.nan
    drift_total_s = math.nan
    if len(values) >= 2 and elapsed[-1] > elapsed[0]:
        x_mean = statistics.fmean(elapsed)
        y_mean = statistics.fmean(values)
        denominator = sum((x - x_mean) ** 2 for x in elapsed)
        if denominator > 0.0:
            slope_s_per_s = sum(
                (x - x_mean) * (y - y_mean)
                for x, y in zip(elapsed, values)
            ) / denominator
            drift_total_s = slope_s_per_s * (elapsed[-1] - elapsed[0])

    result = {
        "source": source,
        "lidar_topic": args.lidar_topic,
        "imu_topic": args.imu_topic,
        "lidar_timestamp": args.lidar_stamp,
        "samples": len(values),
        "unmatched": unmatched,
        "out_of_order_lidar": out_of_order_lidar,
        "duration_s": (elapsed[-1] - elapsed[0]) if values else 0.0,
        "mean_dt_ms": mean_s * 1e3,
        "std_dt_ms": std_s * 1e3,
        "p95_abs_dt_ms": abs_p95_s * 1e3,
        "p99_abs_dt_ms": abs_p99_s * 1e3,
        "p95_signed_dt_ms": signed_p95_s * 1e3,
        "p99_signed_dt_ms": signed_p99_s * 1e3,
        "drift_slope_us_per_s": slope_s_per_s * 1e6,
        "fitted_drift_us": drift_total_s * 1e6,
    }
    return result


class TimestampMonitor:
    def __init__(self, ros, args: argparse.Namespace) -> None:
        self.ros = ros
        self.args = args
        self.node = ros["Node"]("frlio_lidar_imu_timestamp_monitor")
        self.started = time.monotonic()
        self.active = True
        self.pending: Deque[Tuple[float, float]] = deque()
        self.imu_stamps: Deque[int] = deque()
        self.deltas_s: List[float] = []
        self.elapsed_s: List[float] = []
        self.unmatched = 0
        self.out_of_order_lidar = 0
        self.last_lidar_ns: Optional[int] = None
        qos = ros["QoSProfile"](
            reliability=ros["ReliabilityPolicy"].BEST_EFFORT,
            durability=ros["DurabilityPolicy"].VOLATILE,
            history=ros["HistoryPolicy"].KEEP_LAST,
            depth=100,
        )
        self.node.create_subscription(
            ros["Imu"], args.imu_topic, self._on_imu, qos
        )
        self.node.create_subscription(
            ros["CustomMsg"], args.lidar_topic, self._on_lidar, qos
        )
        self.node.create_timer(0.01, self._flush_pending)

    def _on_imu(self, message) -> None:
        if not self.active:
            return
        stamp_ns = _stamp_ns(message.header.stamp)
        if stamp_ns <= 0:
            return
        if self.imu_stamps and stamp_ns < self.imu_stamps[-1]:
            # Keep the deque sorted; a rare out-of-order sample should not
            # corrupt nearest-neighbour pairing.
            values = list(self.imu_stamps)
            values.insert(bisect_left(values, stamp_ns), stamp_ns)
            self.imu_stamps = deque(values)
        else:
            self.imu_stamps.append(stamp_ns)
        newest = self.imu_stamps[-1]
        cutoff = newest - int(self.args.history_s * 1e9)
        while self.imu_stamps and self.imu_stamps[0] < cutoff:
            self.imu_stamps.popleft()

    def _on_lidar(self, message) -> None:
        if not self.active:
            return
        header_ns = _stamp_ns(message.header.stamp)
        if self.args.lidar_stamp == "timebase":
            stamp_ns = int(getattr(message, "timebase", 0))
            if stamp_ns <= 0:
                stamp_ns = header_ns
        else:
            stamp_ns = header_ns
        if stamp_ns <= 0:
            self.unmatched += 1
            return
        if self.last_lidar_ns is not None and stamp_ns < self.last_lidar_ns:
            self.out_of_order_lidar += 1
        self.last_lidar_ns = stamp_ns
        self.pending.append((stamp_ns / 1e9, time.monotonic()))

    def _flush_pending(self) -> None:
        now = time.monotonic()
        while self.pending:
            lidar_s, received_s = self.pending[0]
            if now - received_s < self.args.pair_delay_s and self.active:
                break
            self.pending.popleft()
            if not self.imu_stamps:
                if now - received_s < self.args.max_wait_s:
                    self.pending.appendleft((lidar_s, received_s))
                    break
                self.unmatched += 1
                continue
            imu_values = list(self.imu_stamps)
            index = bisect_left(imu_values, int(lidar_s * 1e9))
            candidates = []
            if index < len(imu_values):
                candidates.append(imu_values[index])
            if index > 0:
                candidates.append(imu_values[index - 1])
            imu_s = min(candidates, key=lambda value: abs(value / 1e9 - lidar_s)) / 1e9
            delta_s = lidar_s - imu_s
            if abs(delta_s) > self.args.max_pair_gap_s:
                self.unmatched += 1
                continue
            self.deltas_s.append(delta_s)
            self.elapsed_s.append(received_s - self.started)

    def finish(self) -> None:
        self.active = False
        deadline = time.monotonic() + max(self.args.pair_delay_s, self.args.max_wait_s)
        while self.pending and time.monotonic() < deadline:
            self._flush_pending()
            time.sleep(0.005)
        self._flush_pending()

    def report(self) -> dict:
        return _summarize(
            self.args,
            self.deltas_s,
            self.elapsed_s,
            self.unmatched,
            self.out_of_order_lidar,
        )


def _import_ros():
    import rclpy
    from livox_ros_driver2.msg import CustomMsg
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Imu

    return {
        "rclpy": rclpy,
        "CustomMsg": CustomMsg,
        "Imu": Imu,
        "Node": Node,
        "QoSProfile": QoSProfile,
        "ReliabilityPolicy": ReliabilityPolicy,
        "DurabilityPolicy": DurabilityPolicy,
        "HistoryPolicy": HistoryPolicy,
    }


def _read_bag(args: argparse.Namespace, ros: dict) -> dict:
    """Read only the raw LiDAR and IMU topics from a rosbag2 SQLite bag."""
    from pathlib import Path

    import rosbag2_py
    from rclpy.serialization import deserialize_message

    bag_path = Path(args.bag).expanduser().resolve()
    if bag_path.is_file() and bag_path.suffix == ".db3":
        bag_uri = bag_path.parent
    else:
        bag_uri = bag_path
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_uri), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", "cdr"),
    )
    topic_types = {
        item.name: item.type for item in reader.get_all_topics_and_types()
    }
    required = {args.lidar_topic, args.imu_topic}
    missing = sorted(topic for topic in required if topic not in topic_types)
    if missing:
        raise RuntimeError(
            "rosbag 缺少原始传感器话题：" + ", ".join(missing)
        )

    lidar_stamps: List[int] = []
    imu_stamps: List[int] = []
    while reader.has_next():
        topic, serialized, _recorded_ns = reader.read_next()
        if topic == args.imu_topic:
            message = deserialize_message(serialized, ros["Imu"])
            stamp_ns = _stamp_ns(message.header.stamp)
            if stamp_ns > 0:
                imu_stamps.append(stamp_ns)
        elif topic == args.lidar_topic:
            message = deserialize_message(serialized, ros["CustomMsg"])
            header_ns = _stamp_ns(message.header.stamp)
            if args.lidar_stamp == "timebase":
                stamp_ns = int(getattr(message, "timebase", 0))
                if stamp_ns <= 0:
                    stamp_ns = header_ns
            else:
                stamp_ns = header_ns
            if stamp_ns > 0:
                lidar_stamps.append(stamp_ns)

    imu_stamps.sort()
    deltas_s: List[float] = []
    elapsed_s: List[float] = []
    unmatched = 0
    out_of_order = sum(
        current < previous
        for previous, current in zip(lidar_stamps, lidar_stamps[1:])
    )
    if lidar_stamps:
        first_lidar_s = lidar_stamps[0] / 1e9
        for lidar_ns in lidar_stamps:
            index = bisect_left(imu_stamps, lidar_ns)
            candidates = []
            if index < len(imu_stamps):
                candidates.append(imu_stamps[index])
            if index > 0:
                candidates.append(imu_stamps[index - 1])
            if not candidates:
                unmatched += 1
                continue
            imu_ns = min(candidates, key=lambda value: abs(value - lidar_ns))
            delta_s = (lidar_ns - imu_ns) / 1e9
            if abs(delta_s) > args.max_pair_gap_s:
                unmatched += 1
                continue
            deltas_s.append(delta_s)
            elapsed_s.append(lidar_ns / 1e9 - first_lidar_s)

    return _summarize(
        args,
        deltas_s,
        elapsed_s,
        unmatched,
        out_of_order,
        source=str(bag_uri),
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--bag",
        help="Read a rosbag2 directory or its rosbag_0.db3 directly instead of live topics",
    )
    parser.add_argument("--seconds", type=float, default=60.0)
    parser.add_argument("--lidar-topic", default="/livox/lidar")
    parser.add_argument("--imu-topic", default="/livox/imu")
    parser.add_argument(
        "--lidar-stamp", choices=("timebase", "header"), default="timebase"
    )
    parser.add_argument("--pair-delay-s", type=float, default=0.05)
    parser.add_argument("--max-wait-s", type=float, default=0.50)
    parser.add_argument("--max-pair-gap-s", type=float, default=0.10)
    parser.add_argument("--history-s", type=float, default=10.0)
    parser.add_argument("--json", dest="json_path")
    args = parser.parse_args()
    if args.seconds <= 0.0 or args.max_pair_gap_s <= 0.0:
        parser.error("seconds and max-pair-gap-s must be positive")

    ros = _import_ros()
    if args.bag:
        try:
            result = _read_bag(args, ros)
        except Exception as exc:
            parser.error(str(exc))
        print(json.dumps(result, ensure_ascii=False, indent=2))
        if args.json_path:
            with open(args.json_path, "w", encoding="utf-8") as stream:
                json.dump(result, stream, ensure_ascii=False, indent=2)
        return 0

    rclpy = ros["rclpy"]
    rclpy.init()
    monitor = TimestampMonitor(ros, args)
    end = time.monotonic() + args.seconds
    try:
        while time.monotonic() < end:
            rclpy.spin_once(monitor.node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.finish()
        result = monitor.report()
        print(json.dumps(result, ensure_ascii=False, indent=2))
        if args.json_path:
            with open(args.json_path, "w", encoding="utf-8") as stream:
                json.dump(result, stream, ensure_ascii=False, indent=2)
        monitor.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
