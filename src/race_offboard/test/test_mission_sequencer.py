#!/usr/bin/env python3
"""Pure logic coverage for the four-point RViz mission state machine."""

import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path


MODULE_PATH = Path(__file__).resolve().parents[1] / 'mission' / 'mission_sequencer_node.py'
SPEC = importlib.util.spec_from_file_location('mission_sequencer_node', MODULE_PATH)
MISSION = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MISSION
SPEC.loader.exec_module(MISSION)


def write_settings(document):
    stream = tempfile.NamedTemporaryFile('w', suffix='.yaml', delete=False, encoding='utf-8')
    stream.write(document)
    stream.close()
    return Path(stream.name)


class MissionSequencerTest(unittest.TestCase):
    def settings(self):
        return MISSION.MissionSettings(arrive_radius=0.4, b2_hold_sec=3.0, d_hold_sec=3.0)

    def collect_four_points(self, sequencer):
        for x, y in ((1.0, 0.0), (2.0, 0.0), (3.0, 0.0), (4.0, 0.0)):
            sequencer.add_rviz_point(x, y)

    def test_settings_validation_and_point_validation(self):
        for document in ('arrive_radius: -0.1\n', 'b2_hold_sec: -1\n'):
            path = write_settings(document)
            self.addCleanup(path.unlink)
            with self.assertRaises(MISSION.MissionConfigError):
                MISSION.load_settings(path)

        sequencer = MISSION.MissionSequencer(self.settings())
        sequencer.start()
        sequencer.add_rviz_point(1.0, 0.0)
        with self.assertRaises(MISSION.MissionConfigError):
            sequencer.add_rviz_point(1.04, 0.0)

    def test_rviz_coordinates_allow_negative_enu_values(self):
        sequencer = MISSION.MissionSequencer(self.settings())
        sequencer.start()
        point = sequencer.add_rviz_point(-1.5, -2.25)
        self.assertEqual(point, MISSION.Point('b1', -1.5, -2.25))

    def test_arrival_radius_boundary(self):
        point = MISSION.Point('b1', 1.0, 0.0)
        self.assertTrue(MISSION.point_is_reached(1.4, 0.0, point, 0.4))
        self.assertFalse(MISSION.point_is_reached(1.4001, 0.0, point, 0.4))

    def test_four_points_wait_for_takeoff_then_follow_b1_b2_c_d_sequence(self):
        sequencer = MISSION.MissionSequencer(self.settings())
        sequencer.start()
        self.collect_four_points(sequencer)
        self.assertEqual(sequencer.state, MISSION.MissionState.WAIT_TAKEOFF)
        self.assertIsNone(sequencer.on_position(1.0, 0.0, 0.0))

        action = sequencer.on_control_status('PREFLIGHT_STABILIZING', 0.0)
        self.assertEqual(action.goal.name, 'b1')
        self.assertEqual(sequencer.zone, 'A')

        action = sequencer.on_position(1.0, 0.0, 1.0)
        self.assertEqual(action.goal.name, 'b2')
        self.assertEqual(action.zone, 'B')
        self.assertTrue(action.b_recognition)

        action = sequencer.on_control_hold(2.0)
        self.assertEqual(action.event, 'arrived_b2 hold_started')
        self.assertEqual(sequencer.state, MISSION.MissionState.HOLD_B2)
        self.assertIsNone(sequencer.on_timer(4.99))
        action = sequencer.on_timer(5.0)
        self.assertEqual(action.goal.name, 'c')
        self.assertEqual(action.zone, 'C')
        self.assertFalse(action.b_recognition)

        action = sequencer.on_position(3.0, 0.0, 7.0)
        self.assertEqual(action.goal.name, 'd')
        self.assertIsNone(action.zone)
        self.assertEqual(sequencer.zone, 'C')

        action = sequencer.on_control_hold(8.0)
        self.assertEqual(action.zone, 'D')
        self.assertTrue(action.d_recognition)
        self.assertEqual(sequencer.state, MISSION.MissionState.HOLD_D)
        done = sequencer.on_timer(11.0)
        self.assertTrue(done.mission_done)
        self.assertTrue(sequencer.d_recognition)
        self.assertEqual(sequencer.state, MISSION.MissionState.MISSION_DONE)


class PresetPointsTest(unittest.TestCase):
    """Preset waypoints let a race run start without an operator in RViz."""

    PRESET = """
arrive_radius: 0.40
b2_hold_sec: 3.0
d_hold_sec: 3.0
preset_points:
  - [1.0, 0.0]
  - [2.0, 0.0]
  - [3.0, 0.0]
  - [4.0, 0.0]
"""

    def load_preset(self):
        return MISSION.load_settings(write_settings(self.PRESET))

    def test_preset_points_are_loaded_in_order(self):
        settings = self.load_preset()
        self.assertEqual(len(settings.preset_points), 4)
        self.assertEqual(
            [p.name for p in settings.preset_points], ['b1', 'b2', 'c', 'd'])
        self.assertEqual(settings.preset_points[0].x, 1.0)
        self.assertEqual(settings.preset_points[3].x, 4.0)

    def test_start_skips_collection_when_presets_exist(self):
        sequencer = MISSION.MissionSequencer(self.load_preset())
        self.assertTrue(sequencer.using_preset_points)
        self.assertEqual(sequencer.collected_count, 4)
        sequencer.start()
        self.assertEqual(sequencer.state, MISSION.MissionState.WAIT_TAKEOFF)

    def test_mission_starts_on_takeoff_without_any_rviz_click(self):
        sequencer = MISSION.MissionSequencer(self.load_preset())
        sequencer.start()
        action = sequencer.on_control_status('state=IDLE_HOLD', 0.0)
        self.assertIsNotNone(action)
        self.assertEqual(action.goal.name, 'b1')
        self.assertEqual(sequencer.state, MISSION.MissionState.GOTO_B1)

    def test_rviz_click_replaces_a_preset_course_before_takeoff(self):
        sequencer = MISSION.MissionSequencer(self.load_preset())
        sequencer.start()
        # First click discards the preset instead of being rejected.
        sequencer.add_rviz_point(9.0, 9.0)
        self.assertEqual(sequencer.collected_count, 1)
        self.assertEqual(sequencer.state, MISSION.MissionState.COLLECTING_POINTS)
        for x in (8.0, 7.0, 6.0):
            sequencer.add_rviz_point(x, 9.0)
        self.assertEqual(sequencer.collected_count, 4)
        self.assertEqual(sequencer.points[0].x, 9.0)

    def test_no_preset_still_requires_rviz_points(self):
        settings = MISSION.load_settings(
            write_settings('arrive_radius: 0.4\nb2_hold_sec: 1.0\nd_hold_sec: 1.0\n'))
        sequencer = MISSION.MissionSequencer(settings)
        self.assertFalse(sequencer.using_preset_points)
        sequencer.start()
        self.assertEqual(sequencer.state, MISSION.MissionState.COLLECTING_POINTS)
        self.assertIsNone(sequencer.on_control_status('state=IDLE_HOLD', 0.0))

    def test_wrong_preset_count_is_rejected(self):
        with self.assertRaises(MISSION.MissionConfigError):
            MISSION.load_settings(
                write_settings('preset_points:\n  - [1.0, 0.0]\n  - [2.0, 0.0]\n'))

    def test_coincident_preset_points_are_rejected(self):
        with self.assertRaises(MISSION.MissionConfigError):
            MISSION.load_settings(
                write_settings(
                    'preset_points:\n  - [1.0, 0.0]\n  - [1.0, 0.0]\n'
                    '  - [3.0, 0.0]\n  - [4.0, 0.0]\n'))

    def test_malformed_preset_entry_is_rejected(self):
        with self.assertRaises(MISSION.MissionConfigError):
            MISSION.load_settings(
                write_settings(
                    'preset_points:\n  - [1.0]\n  - [2.0, 0.0]\n'
                    '  - [3.0, 0.0]\n  - [4.0, 0.0]\n'))


if __name__ == '__main__':
    unittest.main()
