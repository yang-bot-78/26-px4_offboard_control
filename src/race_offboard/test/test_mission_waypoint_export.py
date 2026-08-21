#!/usr/bin/env python3
"""Pure validation coverage for waypoint-store to mission conversion."""

import importlib.util
import sys
import unittest
from pathlib import Path
from unittest import mock


MISSION_DIR = Path(__file__).resolve().parents[1] / 'mission'
COMMON_SPEC = importlib.util.spec_from_file_location(
    'waypoint_common', MISSION_DIR / 'waypoint_common.py')
COMMON = importlib.util.module_from_spec(COMMON_SPEC)
assert COMMON_SPEC.loader is not None
sys.modules[COMMON_SPEC.name] = COMMON
COMMON_SPEC.loader.exec_module(COMMON)
EXPORT_SPEC = importlib.util.spec_from_file_location(
    'export_mission_waypoints', MISSION_DIR / 'export_mission_waypoints.py')
EXPORT = importlib.util.module_from_spec(EXPORT_SPEC)
assert EXPORT_SPEC.loader is not None
sys.modules[EXPORT_SPEC.name] = EXPORT
EXPORT_SPEC.loader.exec_module(EXPORT)


class MissionWaypointExportTest(unittest.TestCase):
    def saved(self):
        return {
            'frame_id': 'map',
            'map_pcd_file': '/tmp/course.pcd',
            'map_pcd_sha256': 'abc123',
            'waypoints': {
                'B1': {'mode': 'xy', 'x': 1.0, 'y': 2.0},
                'B2': {'mode': 'xy', 'x': 3.0, 'y': 4.0},
                'C': {'mode': 'xyz', 'x': 5.0, 'y': 6.0},
                'D': {'mode': 'xy', 'x': 7.0, 'y': 8.0},
            },
        }

    def test_export_has_exact_state_machine_schema_and_order(self):
        result = EXPORT.build_mission_document(
            self.saved(), ('B1', 'B2', 'C', 'D'), 0.4, 3.0, 3.0, Path('/tmp/waypoints.yaml'))
        self.assertEqual(result['preset_points'], [[1.0, 2.0], [3.0, 4.0], [5.0, 6.0], [7.0, 8.0]])
        self.assertEqual(result['source_waypoint_names'], ['B1', 'B2', 'C', 'D'])

    def test_export_default_holds_are_half_a_second(self):
        with mock.patch.object(sys, 'argv', ['export_mission_waypoints.py', '--mission-file', '/tmp/mission.yaml']):
            args = EXPORT.parse_args()
        self.assertEqual(args.b2_hold_sec, 0.5)
        self.assertEqual(args.d_hold_sec, 0.5)
        self.assertEqual(args.default_hold_sec, 0.5)

    def test_export_writes_half_second_hold_for_every_point_by_default(self):
        result = EXPORT.build_mission_document(
            self.saved(), ('B1', 'B2', 'C', 'D'), 0.4, 0.5, 0.5,
            Path('/tmp/waypoints.yaml'))
        self.assertTrue(all(point['hold_sec'] == 0.5 for point in result['route_points']))

    def test_export_rejects_missing_point_wrong_frame_and_non_finite_values(self):
        for mutate in (
                lambda document: document['waypoints'].pop('D'),
                lambda document: document.update({'frame_id': 'odom'}),
                lambda document: document['waypoints']['C'].update({'x': float('nan')})):
            document = self.saved()
            mutate(document)
            with self.assertRaises(ValueError):
                EXPORT.build_mission_document(
                    document, ('B1', 'B2', 'C', 'D'), 0.4, 3.0, 3.0,
                    Path('/tmp/waypoints.yaml'))

    def test_export_sorts_numbered_points_after_fixed_lead_in(self):
        document = self.saved()
        document['waypoints'].update({
            'P010': {'mode': 'xy', 'x': 10.0, 'y': 0.0},
            'P005': {'mode': 'xy', 'x': 5.0, 'y': 0.0},
            'P002': {'mode': 'xy', 'x': 2.0, 'y': 0.0},
            'inspection': {'mode': 'xy', 'x': 20.0, 'y': 0.0},
        })
        result = EXPORT.build_mission_document(
            document, ('B1', 'B2', 'C', 'D'), 0.4, 3.0, 3.0,
            Path('/tmp/waypoints.yaml'))
        self.assertEqual(
            [point['name'] for point in result['route_points']],
            ['B1', 'B2', 'C', 'D', 'P002', 'P005', 'P010', 'inspection'])

    def test_export_accepts_two_points_and_keeps_later_points(self):
        document = {
            'frame_id': 'map',
            'waypoints': {
                'B1': {'mode': 'xy', 'x': 1.0, 'y': 2.0},
                'B2': {'mode': 'xy', 'x': 3.0, 'y': 4.0},
                'P005': {'mode': 'xy', 'x': 5.0, 'y': 6.0},
                'inspection': {'mode': 'xy', 'x': 7.0, 'y': 8.0},
            },
        }
        names = EXPORT._select_names(document)
        result = EXPORT.build_mission_document(
            document, names, 0.4, 3.0, 3.0, Path('/tmp/waypoints.yaml'))
        self.assertEqual(names, ('B1', 'B2'))
        self.assertEqual(
            [point['name'] for point in result['route_points']],
            ['B1', 'B2', 'P005', 'inspection'])

    def test_export_rejects_fewer_than_two_points(self):
        document = {
            'frame_id': 'map',
            'waypoints': {'B1': {'mode': 'xy', 'x': 1.0, 'y': 2.0}},
        }
        with self.assertRaises(ValueError):
            EXPORT.build_mission_document(
                document, ('B1',), 0.4, 3.0, 3.0, Path('/tmp/waypoints.yaml'))


if __name__ == '__main__':
    unittest.main()
