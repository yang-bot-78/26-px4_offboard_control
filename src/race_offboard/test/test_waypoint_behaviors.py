#!/usr/bin/env python3
"""Pure tests for the archived waypoint behavior contracts."""

import importlib.util
import math
import sys
import unittest
from pathlib import Path


MODULE_PATH = Path(__file__).resolve().parents[1] / 'mission' / 'waypoint_behaviors.py'
SPEC = importlib.util.spec_from_file_location('waypoint_behaviors_test', MODULE_PATH)
BEHAVIORS = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = BEHAVIORS
SPEC.loader.exec_module(BEHAVIORS)


class WaypointBehaviorsTest(unittest.TestCase):
    def test_camera_and_planner_commands_are_canonical(self):
        self.assertEqual(BEHAVIORS.build_behavior('打开相机').name, 'open_camera')
        self.assertEqual(BEHAVIORS.build_behavior('关闭相机').name, 'close_camera')
        self.assertEqual(
            BEHAVIORS.build_behavior('关闭ego改为A*接管路径').params,
            {
                'planner_backend': 'super',
                'ego_enabled': False,
                'requires_navigation_restart': True,
            })

    def test_speed_requires_positive_mps(self):
        command = BEHAVIORS.build_behavior(
            'set_global_speed', {'speed_mps': 1.25})
        self.assertEqual(command.params['speed_mps'], 1.25)
        with self.assertRaises(BEHAVIORS.BehaviorConfigError):
            BEHAVIORS.build_behavior('set_global_speed', {'speed_mps': 0.0})

    def test_yaw_sweep_returns_to_arrival_heading(self):
        arrival = math.radians(170.0)
        command = BEHAVIORS.build_behavior(
            'yaw_sweep', {
                'n_deg': 30.0, 'm_sec': 2.0, 'cycles': 2, 'direction': 'left'}, arrival)
        self.assertEqual(len(command.phases), 2)
        self.assertEqual(command.phases[0].direction, 'left')
        self.assertAlmostEqual(
            command.phases[-1].target_yaw_rad,
            BEHAVIORS.wrap_angle(arrival + math.radians(60.0)))
        self.assertEqual(command.phases[-1].hold_sec, 2.0)

    def test_yaw_sweep_can_accumulate_right_turns_without_restore(self):
        command = BEHAVIORS.build_behavior(
            'yaw_sweep', {'n': 45, 'm': 2, 'x': 2, 'direction': 'right'}, 0.0)
        self.assertEqual([phase.direction for phase in command.phases], ['right', 'right'])
        self.assertAlmostEqual(command.phases[-1].target_yaw_rad, math.radians(-90.0))


if __name__ == '__main__':
    unittest.main()
