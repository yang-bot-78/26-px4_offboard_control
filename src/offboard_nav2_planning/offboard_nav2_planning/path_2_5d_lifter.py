#!/usr/bin/env python3
import math
from collections import defaultdict
from copy import deepcopy
from typing import DefaultDict, Iterable, List, Optional, Tuple

import rclpy
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

Point3 = Tuple[float, float, float]


class Path25DLifter(Node):
    def __init__(self) -> None:
        super().__init__("nav2_stage2_path_2_5d_lifter")

        self.declare_parameter("input_path_topic", "/nav2_stage1/path")
        self.declare_parameter("cloud_topic", "/cloud_registered")
        self.declare_parameter("output_path_topic", "/nav2_stage2/path_2_5d")
        self.declare_parameter("global_frame", "camera_init")
        self.declare_parameter("default_z_m", 1.0)
        self.declare_parameter("min_z_m", 0.5)
        self.declare_parameter("max_z_m", 2.0)
        self.declare_parameter("sample_radius_m", 0.6)
        self.declare_parameter("grid_cell_m", 0.4)
        self.declare_parameter("floor_percentile", 0.15)
        self.declare_parameter("flight_height_above_floor_m", 1.0)
        self.declare_parameter("obstacle_clearance_m", 0.4)
        self.declare_parameter("min_obstacle_height_above_floor_m", 0.15)
        self.declare_parameter("max_z_step_m", 0.20)
        self.declare_parameter("max_cloud_points", 60000)
        self.declare_parameter("cloud_stride", 4)

        transient_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self._cloud_frame: Optional[str] = None
        self._bins: DefaultDict[Tuple[int, int], List[float]] = defaultdict(list)

        self._path_pub = self.create_publisher(
            Path,
            self.get_parameter("output_path_topic").value,
            transient_qos,
        )
        self.create_subscription(
            PointCloud2,
            self.get_parameter("cloud_topic").value,
            self._cloud_cb,
            2,
        )
        self.create_subscription(
            Path,
            self.get_parameter("input_path_topic").value,
            self._path_cb,
            10,
        )
        self.get_logger().info(
            "2.5D lifter: %s -> %s using cloud %s"
            % (
                self.get_parameter("input_path_topic").value,
                self.get_parameter("output_path_topic").value,
                self.get_parameter("cloud_topic").value,
            )
        )

    def _cloud_cb(self, msg: PointCloud2) -> None:
        frame = msg.header.frame_id or self.get_parameter("global_frame").value
        global_frame = self.get_parameter("global_frame").value
        if frame != global_frame:
            self.get_logger().warn(
                "Cloud frame is %s but global_frame is %s; no TF transform is applied."
                % (frame, global_frame),
                throttle_duration_sec=5.0,
            )

        grid_cell = float(self.get_parameter("grid_cell_m").value)
        max_points = int(self.get_parameter("max_cloud_points").value)
        stride = max(1, int(self.get_parameter("cloud_stride").value))

        bins: DefaultDict[Tuple[int, int], List[float]] = defaultdict(list)
        kept = 0
        cloud_points = point_cloud2.read_points(
            msg,
            field_names=("x", "y", "z"),
            skip_nans=True,
        )
        for i, point in enumerate(cloud_points):
            if i % stride != 0:
                continue
            x, y, z = self._point_tuple(point)
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue
            key = (math.floor(x / grid_cell), math.floor(y / grid_cell))
            bins[key].append(z)
            kept += 1
            if kept >= max_points:
                break

        self._cloud_frame = frame
        self._bins = bins

    def _path_cb(self, msg: Path) -> None:
        if not msg.poses:
            return

        default_z = float(self.get_parameter("default_z_m").value)
        min_z = float(self.get_parameter("min_z_m").value)
        max_z = float(self.get_parameter("max_z_m").value)
        max_step = float(self.get_parameter("max_z_step_m").value)

        out = Path()
        out.header = msg.header
        if not out.header.frame_id:
            out.header.frame_id = self.get_parameter("global_frame").value
        out.header.stamp = self.get_clock().now().to_msg()

        last_z: Optional[float] = None
        for pose in msg.poses:
            lifted = deepcopy(pose)
            x = pose.pose.position.x
            y = pose.pose.position.y
            target_z = self._target_z(x, y, default_z)
            target_z = min(max(target_z, min_z), max_z)
            if last_z is not None:
                target_z = min(max(target_z, last_z - max_step), last_z + max_step)
            lifted.pose.position.z = target_z
            lifted.header.stamp = out.header.stamp
            if not lifted.header.frame_id:
                lifted.header.frame_id = out.header.frame_id
            out.poses.append(lifted)
            last_z = target_z

        self._path_pub.publish(out)
        self.get_logger().info(
            "Published 2.5D path with %d poses, z %.2f..%.2f m."
            % (
                len(out.poses),
                min(p.pose.position.z for p in out.poses),
                max(p.pose.position.z for p in out.poses),
            )
        )

    def _target_z(self, x: float, y: float, default_z: float) -> float:
        zs = self._nearby_z_values(x, y)
        if not zs:
            return default_z

        zs.sort()
        percentile = min(max(float(self.get_parameter("floor_percentile").value), 0.0), 1.0)
        floor_idx = min(len(zs) - 1, max(0, int(percentile * (len(zs) - 1))))
        floor_z = zs[floor_idx]

        min_obstacle_height = float(self.get_parameter("min_obstacle_height_above_floor_m").value)
        obstacles = [z for z in zs if z >= floor_z + min_obstacle_height]

        floor_target = floor_z + float(self.get_parameter("flight_height_above_floor_m").value)
        if not obstacles:
            return floor_target

        obstacle_target = max(obstacles) + float(self.get_parameter("obstacle_clearance_m").value)
        return max(floor_target, obstacle_target)

    def _nearby_z_values(self, x: float, y: float) -> List[float]:
        grid_cell = float(self.get_parameter("grid_cell_m").value)
        radius = float(self.get_parameter("sample_radius_m").value)
        radius_cells = max(1, math.ceil(radius / grid_cell))
        center_key = (math.floor(x / grid_cell), math.floor(y / grid_cell))

        values: List[float] = []
        for ix in range(center_key[0] - radius_cells, center_key[0] + radius_cells + 1):
            for iy in range(center_key[1] - radius_cells, center_key[1] + radius_cells + 1):
                for z in self._bins.get((ix, iy), []):
                    # The XY distance was quantized into grid cells; include neighbor bins
                    # and let the local map density absorb the small approximation.
                    values.append(z)
        return values if values else []

    @staticmethod
    def _point_tuple(point: Iterable[float]) -> Point3:
        try:
            x = point["x"]
            y = point["y"]
            z = point["z"]
        except (IndexError, KeyError, TypeError, ValueError):
            x, y, z = point
        return float(x), float(y), float(z)


def main() -> None:
    rclpy.init()
    node = Path25DLifter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
