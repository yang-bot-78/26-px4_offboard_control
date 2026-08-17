#!/usr/bin/env python3
"""List saved map-frame waypoints."""

from __future__ import annotations

import argparse
from pathlib import Path

from waypoint_common import default_waypoints_file, load_waypoints


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--waypoints-file", default=None)
    args = parser.parse_args()
    path = Path(args.waypoints_file).expanduser().resolve() if args.waypoints_file else default_waypoints_file()
    try:
        data = load_waypoints(path)
    except Exception as error:
        parser.error(str(error))
    print(f"file: {path}")
    print(f"frame_id: {data.get('frame_id', 'map')}")
    for name, point in data.get("waypoints", {}).items():
        print(f"{name}: mode={point.get('mode', '?')} x={point.get('x')} y={point.get('y')} z={point.get('z')} yaw={point.get('yaw')}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
