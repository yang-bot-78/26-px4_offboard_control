"""Regression coverage for saved-waypoint planning-map identity checks."""

import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path


MISSION_DIR = Path(__file__).resolve().parents[1] / 'mission'
COMMON_SPEC = importlib.util.spec_from_file_location(
    'waypoint_common', MISSION_DIR / 'waypoint_common.py')
COMMON = importlib.util.module_from_spec(COMMON_SPEC)
assert COMMON_SPEC.loader is not None
sys.modules[COMMON_SPEC.name] = COMMON
COMMON_SPEC.loader.exec_module(COMMON)
BINDING_SPEC = importlib.util.spec_from_file_location(
    'waypoint_map_binding', MISSION_DIR / 'waypoint_map_binding.py')
BINDING = importlib.util.module_from_spec(BINDING_SPEC)
assert BINDING_SPEC.loader is not None
sys.modules[BINDING_SPEC.name] = BINDING
BINDING_SPEC.loader.exec_module(BINDING)


class WaypointMapBindingTest(unittest.TestCase):
    def write_waypoints(self, path: Path, digest: str, frame: str = 'map') -> None:
        path.write_text(
            f'frame_id: {frame}\nmap_pcd_sha256: {digest}\nwaypoints: {{}}\n',
            encoding='utf-8')

    def write_pcd(self, path: Path, points: list[tuple[float, float, float]]) -> None:
        rows = '\n'.join(f'{x} {y} {z}' for x, y, z in points)
        path.write_text(
            '\n'.join([
                '# .PCD v0.7 - Point Cloud Data file format',
                'VERSION 0.7',
                'FIELDS x y z',
                'SIZE 4 4 4',
                'TYPE F F F',
                'COUNT 1 1 1',
                f'WIDTH {len(points)}',
                'HEIGHT 1',
                'VIEWPOINT 0 0 0 1 0 0 0',
                f'POINTS {len(points)}',
                'DATA ascii',
                rows,
            ]),
            encoding='ascii')

    def test_accepts_identical_map_content_after_file_move(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            saved_map = root / 'saved-map.pcd'
            active_map = root / 'active-map.pcd'
            saved_map.write_bytes(b'point-cloud')
            active_map.write_bytes(b'point-cloud')
            waypoints = root / 'waypoints.yaml'
            self.write_waypoints(waypoints, COMMON.sha256_file(saved_map))
            self.assertEqual(
                BINDING.validate_waypoint_map_binding(waypoints, active_map),
                COMMON.sha256_file(active_map))

    def test_rejects_different_map_content(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            recorded_map = root / 'recorded-map.pcd'
            active_map = root / 'active-map.pcd'
            recorded_map.write_bytes(b'recorded')
            active_map.write_bytes(b'active')
            waypoints = root / 'waypoints.yaml'
            self.write_waypoints(waypoints, COMMON.sha256_file(recorded_map))
            with self.assertRaisesRegex(BINDING.WaypointMapBindingError, 'SHA-256 mismatch'):
                BINDING.validate_waypoint_map_binding(waypoints, active_map)

    def test_accepts_cropped_point_cloud_derived_from_recorded_map(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            recorded_map = root / 'recorded-map.pcd'
            active_map = root / 'cropped-map.pcd'
            points = [(0.0, 0.0, 0.0), (1.0, 2.0, 0.5), (-1.0, 3.0, 1.0)]
            self.write_pcd(recorded_map, points)
            self.write_pcd(active_map, points[1:])
            waypoints = root / 'waypoints.yaml'
            waypoints.write_text(
                f'frame_id: map\nmap_pcd_file: {recorded_map}\n'
                f'map_pcd_sha256: {COMMON.sha256_file(recorded_map)}\nwaypoints: {{}}\n',
                encoding='utf-8')
            self.assertEqual(
                BINDING.validate_waypoint_map_binding(waypoints, active_map),
                COMMON.sha256_file(active_map))

    def test_rejects_point_cloud_with_foreign_point(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            recorded_map = root / 'recorded-map.pcd'
            active_map = root / 'foreign-map.pcd'
            self.write_pcd(recorded_map, [(0.0, 0.0, 0.0), (1.0, 1.0, 1.0)])
            self.write_pcd(active_map, [(0.0, 0.0, 0.0), (9.0, 9.0, 9.0)])
            waypoints = root / 'waypoints.yaml'
            waypoints.write_text(
                f'frame_id: map\nmap_pcd_file: {recorded_map}\n'
                f'map_pcd_sha256: {COMMON.sha256_file(recorded_map)}\nwaypoints: {{}}\n',
                encoding='utf-8')
            with self.assertRaisesRegex(
                    BINDING.WaypointMapBindingError, 'not a verified cropped subset'):
                BINDING.validate_waypoint_map_binding(waypoints, active_map)

    def test_rejects_missing_map_fingerprint_and_non_map_frame(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            active_map = root / 'active-map.pcd'
            active_map.write_bytes(b'active')
            waypoints = root / 'waypoints.yaml'
            self.write_waypoints(waypoints, 'not-a-digest')
            with self.assertRaisesRegex(BINDING.WaypointMapBindingError, 'missing or invalid'):
                BINDING.validate_waypoint_map_binding(waypoints, active_map)
            self.write_waypoints(waypoints, COMMON.sha256_file(active_map), frame='odom')
            with self.assertRaisesRegex(BINDING.WaypointMapBindingError, 'expected map'):
                BINDING.validate_waypoint_map_binding(waypoints, active_map)


if __name__ == '__main__':
    unittest.main()
