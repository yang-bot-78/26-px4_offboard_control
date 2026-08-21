#!/usr/bin/env python3
import math
import sys
from collections import defaultdict
from pathlib import Path
from typing import DefaultDict, Iterable, List, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

Point2 = Tuple[float, float]
GridKey = Tuple[int, int]


class Save2DMapFromCloud(Node):
    def __init__(self) -> None:
        super().__init__("save_2d_map_from_cloud")

        self.declare_parameter("cloud_topic", "/fastlio_global/map")
        self.declare_parameter("output_yaml", "/home/robot/ws_offboard_control/maps/fastlio_nav2_map.yaml")
        self.declare_parameter("resolution", 0.10)
        self.declare_parameter("min_z", -0.20)
        self.declare_parameter("max_z", 2.00)
        self.declare_parameter("padding_m", 1.0)
        self.declare_parameter("occupied_dilation_m", 0.20)
        self.declare_parameter("timeout_s", 10.0)
        self.declare_parameter("free_unknown", True)
        self.declare_parameter("ground_filter", True)
        self.declare_parameter("ground_percentile", 0.15)
        self.declare_parameter("obstacle_min_height_above_floor_m", 0.25)
        self.declare_parameter("min_points_per_cell", 2)

        self._cloud_received = False
        self.done = False
        self.exit_code = 0
        self.create_subscription(
            PointCloud2,
            self.get_parameter("cloud_topic").value,
            self._cloud_cb,
            1,
        )
        self.create_timer(float(self.get_parameter("timeout_s").value), self._timeout_cb)
        self.get_logger().info(
            "Waiting for PointCloud2 on %s"
            % self.get_parameter("cloud_topic").value
        )

    def _timeout_cb(self) -> None:
        if not self._cloud_received:
            self.get_logger().error("Timed out waiting for cloud.")
            self.exit_code = 1
            self.done = True

    def _cloud_cb(self, msg: PointCloud2) -> None:
        if self._cloud_received:
            return
        self._cloud_received = True

        min_z = float(self.get_parameter("min_z").value)
        max_z = float(self.get_parameter("max_z").value)
        points: List[Tuple[float, float, float]] = []

        for point in point_cloud2.read_points(
            msg,
            field_names=("x", "y", "z"),
            skip_nans=True,
        ):
            x, y, z = self._point_tuple(point)
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue
            if min_z <= z <= max_z:
                points.append((x, y, z))

        if not points:
            self.get_logger().error("No points survived z filtering; map was not written.")
            self.exit_code = 1
            self.done = True
            return

        output_yaml = Path(str(self.get_parameter("output_yaml").value)).expanduser()
        resolution = float(self.get_parameter("resolution").value)
        padding = float(self.get_parameter("padding_m").value)
        dilation = float(self.get_parameter("occupied_dilation_m").value)
        free_unknown = bool(self.get_parameter("free_unknown").value)
        ground_filter = bool(self.get_parameter("ground_filter").value)
        ground_percentile = float(self.get_parameter("ground_percentile").value)
        obstacle_min_height = float(self.get_parameter("obstacle_min_height_above_floor_m").value)
        min_points_per_cell = int(self.get_parameter("min_points_per_cell").value)

        self._write_map(
            points,
            output_yaml,
            resolution,
            padding,
            dilation,
            free_unknown,
            ground_filter,
            ground_percentile,
            obstacle_min_height,
            min_points_per_cell,
        )
        self.get_logger().info("Wrote %s" % output_yaml)
        self.done = True

    def _write_map(
        self,
        points: List[Tuple[float, float, float]],
        output_yaml: Path,
        resolution: float,
        padding: float,
        dilation: float,
        free_unknown: bool,
        ground_filter: bool,
        ground_percentile: float,
        obstacle_min_height: float,
        min_points_per_cell: int,
    ) -> None:
        min_x = min(p[0] for p in points) - padding
        max_x = max(p[0] for p in points) + padding
        min_y = min(p[1] for p in points) - padding
        max_y = max(p[1] for p in points) + padding

        width = max(1, int(math.ceil((max_x - min_x) / resolution)))
        height = max(1, int(math.ceil((max_y - min_y) / resolution)))
        occupied = 0

        unknown = 205
        free = 254
        occ = 0
        pixels = [free if free_unknown else unknown] * (width * height)
        dilation_cells = max(0, int(math.ceil(dilation / resolution)))
        z_bins: DefaultDict[GridKey, List[float]] = defaultdict(list)

        for x, y, z in points:
            gx = int(math.floor((x - min_x) / resolution))
            gy = int(math.floor((y - min_y) / resolution))
            if 0 <= gx < width and 0 <= gy < height:
                z_bins[(gx, gy)].append(z)

        occupied_cells: List[GridKey] = []
        for key, zs in z_bins.items():
            if len(zs) < min_points_per_cell:
                continue
            if ground_filter and not self._cell_has_obstacle(zs, ground_percentile, obstacle_min_height):
                continue
            occupied_cells.append(key)

        for gx, gy in occupied_cells:
            for dx in range(-dilation_cells, dilation_cells + 1):
                for dy in range(-dilation_cells, dilation_cells + 1):
                    if dx * dx + dy * dy > dilation_cells * dilation_cells:
                        continue
                    nx = gx + dx
                    ny = gy + dy
                    if 0 <= nx < width and 0 <= ny < height:
                        # PGM rows start at top, ROS map origin starts at bottom-left.
                        row = height - 1 - ny
                        idx = row * width + nx
                        if pixels[idx] != occ:
                            occupied += 1
                        pixels[idx] = occ

        output_yaml.parent.mkdir(parents=True, exist_ok=True)
        pgm_path = output_yaml.with_suffix(".pgm")
        with pgm_path.open("wb") as f:
            f.write(("P5\n%d %d\n255\n" % (width, height)).encode("ascii"))
            f.write(bytes(pixels))

        with output_yaml.open("w", encoding="ascii") as f:
            f.write("image: %s\n" % pgm_path.name)
            f.write("mode: trinary\n")
            f.write("resolution: %.6f\n" % resolution)
            f.write("origin: [%.6f, %.6f, 0.0]\n" % (min_x, min_y))
            f.write("negate: 0\n")
            f.write("occupied_thresh: 0.65\n")
            f.write("free_thresh: 0.25\n")

        self.get_logger().info(
            "Map size %dx%d at %.3f m/cell, obstacle source cells: %d, occupied cells after dilation: %d"
            % (width, height, resolution, len(occupied_cells), occupied)
        )

    @staticmethod
    def _cell_has_obstacle(
        zs: List[float],
        ground_percentile: float,
        obstacle_min_height: float,
    ) -> bool:
        zs.sort()
        percentile = min(max(ground_percentile, 0.0), 1.0)
        floor_idx = min(len(zs) - 1, max(0, int(percentile * (len(zs) - 1))))
        floor_z = zs[floor_idx]
        return max(zs) >= floor_z + obstacle_min_height

    @staticmethod
    def _point_tuple(point: Iterable[float]) -> Tuple[float, float, float]:
        try:
            x = point["x"]
            y = point["y"]
            z = point["z"]
        except (IndexError, KeyError, TypeError, ValueError):
            x, y, z = point
        return float(x), float(y), float(z)


def main() -> None:
    rclpy.init(args=sys.argv)
    node = Save2DMapFromCloud()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        exit_code = node.exit_code
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
