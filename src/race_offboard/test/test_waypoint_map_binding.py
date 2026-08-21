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
