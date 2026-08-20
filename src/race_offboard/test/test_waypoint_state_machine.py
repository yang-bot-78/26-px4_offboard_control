#!/usr/bin/env python3
"""Pure logic tests for the extensible saved-waypoint state machine."""

import importlib.util
import sys
import unittest
from pathlib import Path


MODULE_PATH = Path(__file__).resolve().parents[1] / 'mission' / 'waypoint_state_machine_node.py'
SPEC = importlib.util.spec_from_file_location('waypoint_state_machine_node_test', MODULE_PATH)
FSM = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = FSM
SPEC.loader.exec_module(FSM)


class WaypointStateMachineTest(unittest.TestCase):
    def points(self):
        return (
            FSM.RoutePoint('TAKEOFF', 0.0, 0.0, 0.75, 'TAKEOFF', 'arm_camera', 0.5),
            FSM.RoutePoint('SCAN_A', 1.0, 0.0, 0.75, 'SCAN_A', 'scan_left', 0.0),
            FSM.RoutePoint('LAND', 1.0, 1.0, 0.75, 'LAND', 'prepare_land', 0.0),
        )

    def test_offboard_and_arrival_advance_in_order(self):
        machine = FSM.WaypointStateMachine(self.points(), 0.2)
        action = machine.on_offboard(True)
        self.assertEqual(action.goal.name, 'TAKEOFF')
        self.assertEqual(machine.state, FSM.State.GOTO)
        action = machine.on_position(0.0, 0.0, 10.0)
        self.assertEqual(machine.state, FSM.State.HOLD)
        self.assertEqual(action.arrived.name, 'TAKEOFF')
        self.assertIsNone(machine.on_timer(10.4))
        action = machine.on_timer(10.5)
        self.assertEqual(action.goal.name, 'SCAN_A')
        action = machine.on_position(1.0, 0.0, 11.0)
        self.assertEqual(action.goal.name, 'LAND')
        action = machine.on_position(1.0, 1.0, 12.0)
        self.assertTrue(action.done)
        self.assertEqual(machine.state, FSM.State.DONE)

    def test_leaving_offboard_pauses_without_silent_resume(self):
        machine = FSM.WaypointStateMachine(self.points(), 0.2)
        machine.on_offboard(True)
        action = machine.on_offboard(False)
        self.assertEqual(action.event, 'mission_paused_pilot_left_offboard')
        action = machine.on_offboard(True)
        self.assertEqual(action.event, 'mission_paused_restart_required')

    def test_automatic_mode_waits_for_takeoff_before_horizontal_goal(self):
        machine = FSM.WaypointStateMachine(self.points(), 0.2, wait_for_takeoff=True)
        action = machine.on_offboard(True)
        self.assertEqual(action.event, 'offboard_waiting_for_takeoff')
        self.assertIsNone(machine.on_control_status(
            'state=IDLE_HOLD armed=1 offboard=1 current_height_m=0.30'))
        action = machine.on_control_status(
            'state=IDLE_HOLD armed=1 offboard=1 current_height_m=0.56')
        self.assertEqual(action.goal.name, 'TAKEOFF')


if __name__ == '__main__':
    unittest.main()
