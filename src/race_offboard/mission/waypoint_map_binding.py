#!/usr/bin/env python3
"""Reject saved waypoints that do not belong to the active planning map."""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

from waypoint_common import load_waypoints, sha256_file


_SHA256 = re.compile(r'^[0-9a-f]{64}$')


class WaypointMapBindingError(ValueError):
    """Raised when a waypoint store cannot be tied to the requested map."""


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
        raise WaypointMapBindingError(
            f'planning map SHA-256 mismatch: waypoint={expected}, active={actual}; '
            f'waypoints were recorded against {recorded}')
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
    except (OSError, ValueError) as error:
        print(f'[WAYPOINT_MAP_BINDING_REJECTED] {error}', file=sys.stderr)
        return 2
    print(f'[WAYPOINT_MAP_BINDING_OK] map_sha256={digest}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
