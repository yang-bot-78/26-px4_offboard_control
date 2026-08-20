#!/usr/bin/env python3
"""Regression coverage for RViz saved-waypoint display order."""

import importlib.util
import sys
import unittest
from pathlib import Path


MISSION_DIR = Path(__file__).resolve().parents[1] / 'mission'
COMMON_SPEC = importlib.util.spec_from_file_location(
    'waypoint_common', MISSION_DIR / 'waypoint_common.py')
COMMON = importlib.util.module_from_spec(COMMON_SPEC)
assert COMMON_SPEC.loader is not None
sys.modules[COMMON_SPEC.name] = COMMON
COMMON_SPEC.loader.exec_module(COMMON)
VISUALIZER_SPEC = importlib.util.spec_from_file_location(
    'waypoint_visualizer', MISSION_DIR / 'waypoint_visualizer.py')
VISUALIZER = importlib.util.module_from_spec(VISUALIZER_SPEC)
assert VISUALIZER_SPEC.loader is not None
sys.modules[VISUALIZER_SPEC.name] = VISUALIZER
VISUALIZER_SPEC.loader.exec_module(VISUALIZER)


class WaypointVisualizerTest(unittest.TestCase):
    def test_route_line_follows_fixed_lead_in_and_natural_p_order(self):
        document = {
            'frame_id': 'map',
            'waypoints': {
                'B1': {'x': 1.0, 'y': 0.0, 'z': 0.8},
                'B2': {'x': 2.0, 'y': 0.0, 'z': 0.8},
                'D': {'x': 4.0, 'y': 0.0, 'z': 0.8},
                'P010': {'x': 10.0, 'y': 0.0, 'z': 0.8},
                'P005': {'x': 5.0, 'y': 0.0, 'z': 0.8},
                'C': {'x': 3.0, 'y': 0.0, 'z': 0.8},
                'inspection': {'x': 20.0, 'y': 0.0, 'z': 0.8},
            },
        }

        points = VISUALIZER.valid_waypoints(document)

        self.assertEqual(
            [name for name, _, _, _ in points],
            ['B1', 'B2', 'C', 'D', 'P005', 'P010', 'inspection'])


if __name__ == '__main__':
    unittest.main()
