#!/usr/bin/env python3
"""Verify exact or cropped-derived planning-map identity for saved waypoints."""

from __future__ import annotations

import argparse
import math
import re
import shutil
import subprocess
import tempfile
import sys
from pathlib import Path
from typing import Iterable

from waypoint_common import load_waypoints, sha256_file


_SHA256 = re.compile(r'^[0-9a-f]{64}$')
_PCD_POINT_TOLERANCE_M = 0.002


class WaypointMapBindingError(ValueError):
    """Raised when a waypoint store cannot be tied to the requested map."""


def _pcd_xyz_lines(path: Path) -> Iterable[tuple[float, float, float]]:
    """Yield xyz points from an ASCII PCD file."""
    lines = path.read_text(encoding='ascii', errors='strict').splitlines()
    data_index = next(
        (index for index, line in enumerate(lines) if line.upper().startswith('DATA ')),
        None)
    if data_index is None or lines[data_index].split(maxsplit=1)[1].lower() != 'ascii':
        raise ValueError(f'PCD is not ASCII: {path}')
    fields = next(
        (line.split()[1:] for line in lines[:data_index]
         if line.upper().startswith('FIELDS ')), None)
    if not fields or not all(axis in fields for axis in ('x', 'y', 'z')):
        raise ValueError(f'PCD has no x/y/z fields: {path}')
    indexes = tuple(fields.index(axis) for axis in ('x', 'y', 'z'))
    for line in lines[data_index + 1:]:
        values = line.split()
        if len(values) <= max(indexes):
            continue
        yield tuple(float(values[index]) for index in indexes)


def _ascii_pcd_points(path: Path) -> list[tuple[float, float, float]]:
    """Read any supported PCD through PCL when it is not already ASCII."""
    try:
        return list(_pcd_xyz_lines(path))
    except (UnicodeDecodeError, StopIteration, ValueError):
        converter = shutil.which('pcl_convert_pcd_ascii_binary')
        if converter is None:
            raise ValueError(
                f'cannot inspect binary PCD without pcl_convert_pcd_ascii_binary: {path}')
        with tempfile.TemporaryDirectory(prefix='waypoint_map_binding_') as directory:
            converted = Path(directory) / 'converted_ascii.pcd'
            subprocess.run(
                [converter, str(path), str(converted), '0'],
                check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            return list(_pcd_xyz_lines(converted))


def _is_verified_point_cloud_subset(recorded_map: Path, active_map: Path) -> tuple[int, int]:
    """Return point counts if active_map is a non-empty subset of recorded_map."""
    recorded_points = _ascii_pcd_points(recorded_map)
    active_points = _ascii_pcd_points(active_map)
    if not recorded_points or not active_points or len(active_points) > len(recorded_points):
        raise ValueError('empty or larger point-cloud subset')
    tolerance = _PCD_POINT_TOLERANCE_M
    buckets: dict[tuple[int, int, int], list[tuple[float, float, float]]] = {}
    for point in recorded_points:
        bucket = tuple(math.floor(value / tolerance) for value in point)
        buckets.setdefault(bucket, []).append(point)
    tolerance_squared = tolerance * tolerance
    for point in active_points:
        bucket = tuple(math.floor(value / tolerance) for value in point)
        matched = False
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for dz in (-1, 0, 1):
                    neighbor_bucket = (bucket[0] + dx, bucket[1] + dy, bucket[2] + dz)
                    for candidate in buckets.get(neighbor_bucket, []):
                        distance_squared = sum(
                            (point[index] - candidate[index]) ** 2 for index in range(3))
                        if distance_squared <= tolerance_squared:
                            matched = True
                            break
                    if matched:
                        break
                if matched:
                    break
            if matched:
                break
        if not matched:
            raise ValueError('active point cloud contains points absent from recorded map')
    return len(active_points), len(recorded_points)


def validate_waypoint_map_binding(
        waypoints_file: str | Path, map_file: str | Path) -> str:
    """Return the verified map digest or raise a precise binding error."""
    waypoint_path = Path(waypoints_file).expanduser().resolve()
    map_path = Path(map_file).expanduser().resolve()
    saved = load_waypoints(waypoint_path)
    if saved.get('frame_id') != 'map':
        raise WaypointMapBindingError(
            f'waypoint frame_id={saved.get("frame_id")!r}, expected map')
    expected = saved.get('map_pcd_sha256')
    if not isinstance(expected, str) or not _SHA256.fullmatch(expected):
        raise WaypointMapBindingError(
            f'waypoint map_pcd_sha256 is missing or invalid: {expected!r}')
    actual = sha256_file(map_path)
    if actual is None:
        raise WaypointMapBindingError(f'planning map does not exist or is empty: {map_path}')
    if actual != expected:
        recorded = saved.get('map_pcd_file') or '<not recorded>'
        recorded_path = Path(recorded).expanduser()
        if not recorded_path.is_absolute():
            recorded_path = waypoint_path.parent / recorded_path
        recorded_path = recorded_path.resolve()
        if recorded_path.is_file():
            try:
                _is_verified_point_cloud_subset(recorded_path, map_path)
            except (OSError, ValueError, subprocess.SubprocessError) as error:
                subset_error = str(error)
            else:
                return actual
        else:
            subset_error = f'recorded map does not exist: {recorded_path}'
        raise WaypointMapBindingError(
            f'planning map SHA-256 mismatch: waypoint={expected}, active={actual}; '
            f'waypoints were recorded against {recorded}; '
            f'active map is not a verified cropped subset ({subset_error})')
    return actual


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--waypoints-file', required=True)
    parser.add_argument('--map-file', required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        digest = validate_waypoint_map_binding(args.waypoints_file, args.map_file)
        expected = load_waypoints(Path(args.waypoints_file).expanduser().resolve()).get(
            'map_pcd_sha256')
    except (OSError, ValueError) as error:
        print(f'[WAYPOINT_MAP_BINDING_REJECTED] {error}', file=sys.stderr)
        return 2
    if digest != expected:
        print(
            f'[WAYPOINT_MAP_BINDING_DERIVED_SUBSET] map_sha256={digest} '
            f'source_sha256={expected}')
    else:
        print(f'[WAYPOINT_MAP_BINDING_OK] map_sha256={digest}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
