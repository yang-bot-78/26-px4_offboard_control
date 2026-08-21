#!/usr/bin/env python3
"""
Convert checked saved waypoints into a state-machine mission YAML.

The legacy ``preset_points`` field remains available for consumers that use
it.  ``route_points`` preserves every saved waypoint in tracking order
(B1/B2/C/D followed by naturally numbered P points).  A route needs at least
two saved points; C and D are optional.
"""

from __future__ import annotations

import argparse
import math
import re
import sys
from pathlib import Path
from typing import Any

from waypoint_common import atomic_write_yaml, default_waypoints_file, load_waypoints


DEFAULT_NAMES = ('B1', 'B2', 'C', 'D')
_NUMBERED_WAYPOINT = re.compile(r'^P([0-9]+)$', re.IGNORECASE)


def _ordered_waypoint_names(
        saved: dict[str, Any], names: tuple[str, ...]) -> list[str]:
    """Return the tracking order independently of YAML insertion history.

    B1/B2/C/D are the fixed lead-in.  Numbered points are sorted by their
    numeric suffix so rewriting an older point cannot move P010 before P009.
    Any custom names remain supported and follow the numbered points in their
    original insertion order.
    """
    fixed_after_names = [
        name for name in DEFAULT_NAMES
        if name in saved['waypoints'] and name not in names
    ]
    extras = [
        name for name in saved['waypoints']
        if name not in names and name not in fixed_after_names
    ]
    numbered = []
    custom = []
    for position, name in enumerate(extras):
        match = _NUMBERED_WAYPOINT.fullmatch(str(name))
        if match:
            numbered.append((int(match.group(1)), position, name))
        else:
            custom.append(name)
    numbered.sort(key=lambda item: (item[0], item[1]))
    return list(names) + fixed_after_names + [item[2] for item in numbered] + custom


def _select_names(saved: dict[str, Any]) -> tuple[str, ...]:
    """Choose the saved route lead-in without requiring all four legacy names."""
    waypoints = saved.get('waypoints')
    if not isinstance(waypoints, dict):
        raise ValueError('waypoints must be a mapping')
    names = [name for name in DEFAULT_NAMES if name in waypoints]
    if len(names) < 2:
        names.extend(name for name in waypoints if name not in names)
    if len(names) < 2:
        raise ValueError('at least two saved waypoints are required')
    return tuple(names)


def _finite(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f'{label} must be numeric')
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f'{label} must be finite')
    return result


def build_mission_document(
        saved: dict[str, Any], names: tuple[str, ...],
        arrive_radius: float, b2_hold_sec: float, d_hold_sec: float,
        source_file: Path) -> dict[str, Any]:
    """Validate the saved store and return exactly the mission schema consumed by the FSM."""
    if saved.get('frame_id') != 'map':
        raise ValueError(f'waypoint frame_id={saved.get("frame_id")!r}, expected map')
    if len(names) < 2:
        raise ValueError('at least two waypoint names are required')
    if len(set(names)) != len(names):
        raise ValueError('waypoint names must be unique')
    points = []
    for name in names:
        entry = saved['waypoints'].get(name)
        if not isinstance(entry, dict):
            raise ValueError(f'missing saved waypoint {name!r}')
        mode = entry.get('mode')
        if mode not in ('xy', 'xyz'):
            raise ValueError(f'waypoint {name!r} mode must be xy or xyz')
        points.append([_finite(entry.get('x'), f'{name}.x'), _finite(entry.get('y'), f'{name}.y')])
    route_points = []
    ordered_names = _ordered_waypoint_names(saved, names)
    for name in ordered_names:
        entry = saved['waypoints'].get(name)
        if not isinstance(entry, dict):
            raise ValueError(f'waypoint {name!r} must be a mapping')
        route_entry = {
            'name': name,
            'x': _finite(entry.get('x'), f'{name}.x'),
            'y': _finite(entry.get('y'), f'{name}.y'),
            'z': _finite(entry.get('z', saved.get('default_flight_height', 0.75)), f'{name}.z'),
        }
        # These optional fields are intentionally copied through.  They are
        # published as events by waypoint_state_machine_node for later aircraft
        # behavior modules; unknown fields stay in the source waypoint store.
        for key in ('state', 'behavior'):
            if key in entry:
                route_entry[key] = str(entry[key])
        if 'behavior_params' in entry:
            if not isinstance(entry['behavior_params'], dict):
                raise ValueError(f'{name}.behavior_params must be a mapping')
            route_entry['behavior_params'] = dict(entry['behavior_params'])
        if 'hold_sec' in entry:
            route_entry['hold_sec'] = _finite(entry['hold_sec'], f'{name}.hold_sec')
        route_points.append(route_entry)
    return {
        'arrive_radius': _finite(arrive_radius, 'arrive_radius'),
        'b2_hold_sec': _finite(b2_hold_sec, 'b2_hold_sec'),
        'd_hold_sec': _finite(d_hold_sec, 'd_hold_sec'),
        'preset_points': points,
        'route_points': route_points,
        # Traceability fields are ignored by MissionSequencer.load_settings.
        'source_waypoints_file': str(source_file),
        'source_waypoint_names': list(names),
        'source_frame_id': 'map',
        'source_map_pcd_file': saved.get('map_pcd_file'),
        'source_map_pcd_sha256': saved.get('map_pcd_sha256'),
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--waypoints-file', default=None)
    parser.add_argument('--mission-file', required=True)
    parser.add_argument(
        '--names', nargs='+', metavar='NAME', default=None,
        help='route lead-in names; omit to use all saved B1/B2/C/D points (at least two)')
    parser.add_argument('--arrive-radius', type=float, default=0.40)
    parser.add_argument('--b2-hold-sec', type=float, default=3.0)
    parser.add_argument('--d-hold-sec', type=float, default=3.0)
    parser.add_argument(
        '--all-waypoints', action='store_true',
        help='retain every saved waypoint in route_points (default output also includes it)')
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    source = (Path(args.waypoints_file).expanduser().resolve()
              if args.waypoints_file else default_waypoints_file())
    destination = Path(args.mission_file).expanduser().resolve()
    try:
        saved = load_waypoints(source)
        names = tuple(args.names) if args.names is not None else _select_names(saved)
        document = build_mission_document(
            saved, names, args.arrive_radius, args.b2_hold_sec,
            args.d_hold_sec, source)
        if document['arrive_radius'] <= 0 or min(
                document['b2_hold_sec'], document['d_hold_sec']) < 0:
            raise ValueError('arrival radius must be positive and holds must be non-negative')
        atomic_write_yaml(destination, document)
    except (OSError, ValueError) as error:
        print(f'[MISSION_EXPORT_REJECTED] {error}', file=sys.stderr)
        return 2
    print(
        f'[MISSION_EXPORTED] file={destination} order={"->".join(names)} '
        f'map_sha256={document["source_map_pcd_sha256"] or "unrecorded"}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
