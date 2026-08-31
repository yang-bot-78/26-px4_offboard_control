#!/usr/bin/env python3
"""Pure logic tests for the extensible saved-waypoint state machine."""

import importlib.util
import math
import sys
import tempfile
import textwrap
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
        action = machine.on_position(0.0, 0.0, 0.75, 10.0)
        self.assertEqual(machine.state, FSM.State.HOLD)
        self.assertEqual(action.arrived.name, 'TAKEOFF')
        self.assertIsNone(machine.on_timer(10.4))
        action = machine.on_timer(10.5)
        self.assertEqual(action.goal.name, 'SCAN_A')
        action = machine.on_position(1.0, 0.0, 0.75, 11.0)
        self.assertEqual(action.goal.name, 'LAND')
        action = machine.on_position(1.0, 1.0, 0.75, 12.0)
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
            'state=IDLE_HOLD armed=1 offboard=1 current_height_m=0.30 altitude_reference_valid=0'))
        self.assertIsNone(machine.on_control_status(
            'state=IDLE_HOLD armed=1 offboard=1 current_height_m=0.56 altitude_reference_valid=0'))
        action = machine.on_control_status(
            'state=IDLE_HOLD armed=1 offboard=1 current_height_m=0.56 altitude_reference_valid=1')
        self.assertEqual(action.goal.name, 'TAKEOFF')

    def test_early_offboard_does_not_publish_goal_before_handover_gate(self):
        machine = FSM.WaypointStateMachine(self.points(), 0.2, wait_for_takeoff=True)
        machine.on_navigation_ready()
        self.assertEqual(machine.on_offboard(True).event, 'offboard_waiting_for_takeoff')
        self.assertIsNone(machine.on_control_status(
            'state=IDLE_HOLD armed=1 offboard=1 current_height_m=0.75 '
            'altitude_reference_valid=0'))
        action = machine.on_control_status(
            'state=IDLE_HOLD armed=1 offboard=1 current_height_m=0.75 '
            'altitude_reference_valid=1')
        self.assertEqual(action.goal.name, 'TAKEOFF')

    def test_arrival_waits_for_waypoint_height(self):
        machine = FSM.WaypointStateMachine(self.points(), 0.2)
        machine.on_offboard(True)
        self.assertIsNone(machine.on_position(0.0, 0.0, 0.40, 10.0))
        self.assertEqual(machine.state, FSM.State.GOTO)

    def test_arrival_does_not_require_a_velocity_sample(self):
        machine = FSM.WaypointStateMachine(self.points(), 0.2)
        machine.on_offboard(True)
        action = machine.on_position(0.0, 0.0, 0.75, 10.0)
        self.assertEqual(action.arrived.name, 'TAKEOFF')

    def test_load_route_defaults_each_point_to_half_a_second_hold(self):
        with tempfile.TemporaryDirectory() as directory:
            mission = Path(directory) / 'mission.yaml'
            mission.write_text(textwrap.dedent('''\
                route_points:
                - name: B1
                  x: 0.0
                  y: 0.0
                  z: 0.75
                - name: B2
                  x: 1.0
                  y: 0.0
                  z: 0.75
            '''), encoding='utf-8')
            points = FSM.load_route(mission)
        self.assertEqual([point.hold_sec for point in points], [0.5, 0.5])

    def test_preplan_b1_only_after_navigation_and_position_handover_ready(self):
        machine = FSM.WaypointStateMachine(
            self.points(), 0.2, wait_for_takeoff=True, preplan_first_waypoint=True)
        self.assertIsNone(machine.on_navigation_ready(False))
        self.assertIsNone(machine.on_control_status(
            'state=IDLE_HOLD armed=1 offboard=0 handover_ready=1')
        )
        action = machine.on_navigation_ready()
        self.assertEqual(action.preplan_goal.name, 'TAKEOFF')
        self.assertIsNone(action.goal)
        self.assertEqual(machine.state, FSM.State.WAIT_OFFBOARD)
        self.assertEqual(machine.index, -1)
        self.assertIsNone(machine.on_control_status(
            'state=IDLE_HOLD armed=1 offboard=0 handover_ready=1'))
        self.assertEqual(machine.on_offboard(True).event, 'offboard_waiting_for_takeoff')
        action = machine.on_control_status(
            'state=IDLE_HOLD armed=1 offboard=1 current_height_m=0.56 '
            'altitude_reference_valid=1')
        self.assertEqual(action.goal.name, 'TAKEOFF')

    def test_recognition_results_append_to_one_log_across_process_restarts(self):
        with tempfile.TemporaryDirectory() as directory:
            launcher = Path(directory) / 'recognition.sh'
            result_log = Path(directory) / 'recognition_results.log'
            launcher.write_text(textwrap.dedent('''\
                #!/usr/bin/env bash
                printf 'timestamp=2026-08-21T12:00:00.000Z result=plane\\n' >> \
                    "$RECOGNITION_RESULT_LOG_FILE"
                exec sleep 60
            '''), encoding='utf-8')
            launcher.chmod(0o755)
            controller = FSM.RecognitionProcessController(str(launcher), result_log)
            self.assertTrue(controller.start())
            self.assertFalse(controller.start())
            self.assertTrue(controller.stop())
            self.assertTrue(controller.start())
            self.assertTrue(controller.stop())
            self.assertFalse(controller.stop())
            self.assertEqual(
                result_log.read_text(encoding='utf-8').splitlines(),
                [
                    'timestamp=2026-08-21T12:00:00.000Z result=plane',
                    'timestamp=2026-08-21T12:00:00.000Z result=plane',
                ])

    def test_yaw_behavior_waits_at_target_then_returns_to_arrival_yaw(self):
        phases = (
            FSM.YawPhase(1, 'left', math.radians(30.0), 5.0),
            FSM.YawPhase(2, 'right', 0.0, 0.0),
        )
        execution = FSM.YawBehaviorExecution('B2', phases)

        target, completed = execution.step(0.0, 10.0)
        self.assertAlmostEqual(target, math.radians(30.0))
        self.assertFalse(completed)
        target, completed = execution.step(math.radians(30.0), 11.0)
        self.assertAlmostEqual(target, math.radians(30.0))
        self.assertFalse(completed)
        target, completed = execution.step(math.radians(30.0), 15.9)
        self.assertAlmostEqual(target, math.radians(30.0))
        self.assertFalse(completed)
        target, completed = execution.step(math.radians(30.0), 16.0)
        self.assertAlmostEqual(target, 0.0)
        self.assertFalse(completed)
        target, completed = execution.step(0.0, 17.0)
        self.assertIsNone(target)
        self.assertTrue(completed)

    def test_yaw_behavior_resets_hold_when_heading_leaves_tolerance(self):
        phase = FSM.YawPhase(1, 'left', math.radians(30.0), 2.0)
        execution = FSM.YawBehaviorExecution('B2', (phase,))
        execution.step(math.radians(30.0), 1.0)
        execution.step(math.radians(20.0), 2.0)
        _, completed = execution.step(math.radians(30.0), 3.0)
        self.assertFalse(completed)
        _, completed = execution.step(math.radians(30.0), 5.0)
        self.assertTrue(completed)


if __name__ == '__main__':
    unittest.main()
