#!/usr/bin/env python3
import copy
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'src' / 'race_bringup' / 'launch'))

from astar_ego_tuning import TuningError, load_tuning, node_parameter_overlays  # noqa: E402


class AstarEgoTuningTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.path = ROOT / 'src/race_bringup/config/astar_ego_tuning.yaml'
        cls.tuning = load_tuning(cls.path)

    def test_default_file_loads(self):
        self.assertEqual(1, self.tuning['schema_version'])

    def test_constrained_clearance_is_explicit_and_reaches_ego_and_bridge(self):
        overlays = node_parameter_overlays(self.tuning)
        ego = self.tuning['ego_planner']
        self.assertIsInstance(ego['constrained_clearance_enable'], bool)
        self.assertEqual(
            ego['constrained_clearance_enable'],
            overlays['ego']['fsm/constrained_clearance/enable'])
        self.assertEqual(
            ego['constrained_clearance_enable'],
            overlays['trajectory_bridge']['constrained_clearance_enable'])
        self.assertEqual(
            ego['constrained_clearance_m'],
            overlays['ego']['fsm/constrained_clearance/clearance_m'])
        self.assertEqual(
            ego['constrained_optimization_clearance_m'],
            overlays['ego']['fsm/constrained_clearance/optimization_m'])
        self.assertEqual(
            ego['constrained_clearance_m'],
            overlays['trajectory_bridge']['constrained_clearance_m'])
        self.assertLess(ego['constrained_clearance_m'], ego['planning_clearance_m'])

    def test_constrained_clearance_stays_disabled(self):
        # The constrained channel lowers the only hard anti-collision distance
        # below required_center_clearance and pushes the reduced value into the
        # bridge as well, so the shipped configuration must keep it off.  See
        # the rationale block above constrained_clearance_enable in the YAML for
        # the two prerequisites that must be met before re-enabling it.
        self.assertFalse(self.tuning['ego_planner']['constrained_clearance_enable'])

    def test_far_end_clearance_extra_reaches_the_optimizer(self):
        overlays = node_parameter_overlays(self.tuning)
        ego = self.tuning['ego_planner']
        self.assertEqual(0.06, ego['far_end_clearance_extra_m'])
        self.assertEqual(
            ego['far_end_clearance_extra_m'],
            overlays['ego']['optimization/far_end_clearance_extra'])
        # Soft target only: it must stay above the optimizer's uniform target and
        # far above the hard floor, never at or below it.
        self.assertGreater(
            ego['optimization_clearance_m'] + ego['far_end_clearance_extra_m'],
            self.tuning['shared_safety']['required_center_clearance'])

    def test_far_end_clearance_extra_is_bounded(self):
        bad = copy.deepcopy(self.tuning)
        bad['ego_planner']['far_end_clearance_extra_m'] = 0.16
        with self.assertRaisesRegex(TuningError, 'far_end_clearance_extra_m'):
            self._validate(bad)
        bad['ego_planner']['far_end_clearance_extra_m'] = -0.01
        with self.assertRaisesRegex(TuningError, 'far_end_clearance_extra_m'):
            self._validate(bad)

    def test_far_end_target_cannot_run_away_from_the_hard_floor(self):
        bad = copy.deepcopy(self.tuning)
        floor = bad['shared_safety']['required_center_clearance']
        bad['ego_planner']['optimization_clearance_m'] = floor + 0.15
        bad['ego_planner']['far_end_clearance_extra_m'] = 0.15
        with self.assertRaisesRegex(TuningError, 'far_end_clearance_extra_m'):
            self._validate(bad)

    def test_lateral_offset_must_reach_past_a_pillar(self):
        # A 0.5m box on the reference line blocks lateral offsets up to
        # half-width + planning_clearance_m + extra_clearance_m.  A ladder that
        # cannot exceed that reach makes the fallback useless, so the launch
        # validation has to reject it rather than fail silently in flight.
        bad = copy.deepcopy(self.tuning)
        ego = bad['ego_planner']
        reach = 0.25 + ego['planning_clearance_m'] + \
            bad['ego_recovery']['extra_clearance_m']
        bad['ego_recovery']['max_lateral_offset_m'] = reach - 0.01
        with self.assertRaisesRegex(TuningError, 'max_lateral_offset_m'):
            self._validate(bad)

    def test_lateral_offset_cannot_exceed_recovery_deviation_gate(self):
        # Searching further sideways than build_recovery_seed allows would only
        # produce candidates that are rejected later.
        bad = copy.deepcopy(self.tuning)
        bad['ego_recovery']['max_lateral_offset_m'] = \
            bad['ego_recovery']['max_reference_deviation_m'] + 0.01
        with self.assertRaisesRegex(TuningError, 'max_lateral_offset_m'):
            self._validate(bad)

    def test_lateral_offset_step_stays_within_the_ladder(self):
        bad = copy.deepcopy(self.tuning)
        bad['ego_recovery']['lateral_offset_step_m'] = \
            bad['ego_recovery']['max_lateral_offset_m'] + 0.01
        with self.assertRaisesRegex(TuningError, 'lateral_offset_step_m'):
            self._validate(bad)
        bad['ego_recovery']['lateral_offset_step_m'] = 0.04
        with self.assertRaisesRegex(TuningError, 'lateral_offset_step_m'):
            self._validate(bad)

    def test_constrained_clearance_cannot_cross_physical_floor(self):
        bad = copy.deepcopy(self.tuning)
        bad['ego_planner']['constrained_clearance_m'] = 0.20
        with self.assertRaisesRegex(TuningError, 'physical envelope'):
            self._validate(bad)

    def test_constrained_clearance_cannot_be_used_as_normal_clearance(self):
        bad = copy.deepcopy(self.tuning)
        bad['ego_planner']['constrained_clearance_m'] = 0.30
        with self.assertRaisesRegex(TuningError, 'required_center_clearance'):
            self._validate(bad)

    def test_constrained_optimization_clearance_has_margin_but_stays_below_normal(self):
        bad = copy.deepcopy(self.tuning)
        bad['ego_planner']['constrained_optimization_clearance_m'] = (
            bad['ego_planner']['constrained_clearance_m'] - 0.001)
        with self.assertRaisesRegex(TuningError, 'constrained_optimization_clearance_m'):
            self._validate(bad)
        bad['ego_planner']['constrained_optimization_clearance_m'] = (
            bad['ego_planner']['planning_clearance_m'])
        with self.assertRaisesRegex(TuningError, 'constrained_optimization_clearance_m'):
            self._validate(bad)

    def test_default_map_source_fuses_fixed_walls_and_live_obstacles(self):
        self.assertEqual('static_live_px4', self.tuning['ego_map_source'])

    def test_real_fastlio_topics_reach_cloud_bridge(self):
        overlays = node_parameter_overlays(self.tuning)
        expected = {
            'live_topic': '/cloud_registered',
            'body_topic': '/cloud_registered_body',
            'pose_topic': '/race/pose',
        }
        for key, value in expected.items():
            self.assertEqual(value, self.tuning['ego_map'][key])
            self.assertEqual(value, overlays['cloud_bridge'][key])

    def test_live_cloud_topics_must_be_absolute(self):
        for key in ('live_topic', 'body_topic', 'pose_topic'):
            with self.subTest(key=key):
                bad = copy.deepcopy(self.tuning)
                bad['ego_map'][key] = 'relative/topic'
                with self.assertRaisesRegex(TuningError, key):
                    self._validate(bad)

    def test_flight_scripts_gate_on_real_body_cloud_and_tracking(self):
        stack = (ROOT / 'tools/flight/一键启动导航栈.sh').read_text(encoding='utf-8')
        flight = (
            ROOT / 'tools/flight/人工起飞后Offboard单目标验证.sh'
        ).read_text(encoding='utf-8')
        self.assertIn(
            'wait_component_ready "fastlio" message /cloud_registered_body', stack)
        self.assertIn(
            'wait_component_ready "navigation" message /race/ego/cloud', stack)
        self.assertIn('effective_planner_backend', stack)
        self.assertIn(
            '[[ "${effective_planner_backend}" != super', stack)
        self.assertIn('/state=TRACKING/', flight)
        self.assertIn('wait_tracking || exit 6', flight)
        self.assertIn('/race/super_planner/raw_path', flight)
        self.assertIn('RECORD_EGO_DIAGNOSTICS=true', flight)
        bag = (ROOT / 'tools/rosbag/开始录包.sh').read_text(encoding='utf-8')
        self.assertIn('RECORD_EGO_DIAGNOSTICS', bag)
        self.assertIn('ROSBAG_PROFILE', bag)
        self.assertIn('low_bag_topics', bag)
        self.assertIn('merge_unique_topics', bag)
        self.assertIn(
            '"${low_bag_topics[@]}" "${trace_bag_topics[@]}"', bag)
        low_topics = bag.split('low_bag_topics=(', 1)[1].split(
            'full_bag_topics=(', 1)[0]
        # Low mode must be sufficient to explain a handover/recovery flight:
        # commanded trajectory, measured state, terminal status and EV anchor
        # health. It must not reintroduce the heavy raw streams it is intended
        # to avoid.
        for topic in (
                '/rosout', '/race/ego/odom', '/race/navigation_setpoint',
                '/race/ego/planner_safety_status',
                '/frlio/high_rate_odom/status',
                '/frlio/high_rate_odom/anchor_age',
                '/frlio/performance', '/Odometry', '/Odometry/guarded',
                '/fastlio_global/relocalized_pose',
                '/fastlio_global/backend_status',
                '/planning/odom', '/Odometry/healthy',
                '/ev_health/status', '/ev_health/fault',
                '/ev_health/flight_ready', '/ev_health/diagnostics',
                '/mavros/vision_pose/pose_cov',
                '/mavros/local_position/odom',
                '/mavros/local_position/velocity_local',
                '/fmu/out/estimator_status_flags',
                '/fmu/out/vehicle_local_position'):
            self.assertIn(topic, low_topics)
        for topic in (
                '/livox/lidar', '/livox/imu', '/cloud_registered',
                '/saved_map', '/tf_static'):
            self.assertNotIn(topic, low_topics)
        for topic in (
                '/race/ego/cloud', '/race/ego/occupancy',
                '/race/ego/occupancy_inflate', '/race/ego/predicted_path'):
            self.assertIn(topic, bag)
        # Flight bags must retain enough upstream evidence to distinguish a
        # MID-360 input interruption from FR-LIO/EV processing starvation.
        for topic in (
                '/livox/lidar', '/livox/imu',
                '/frlio/high_rate_odom/status',
                '/frlio/high_rate_odom/anchor_age',
                '/Odometry', '/planning/odom', '/Odometry/healthy'):
            self.assertIn(topic, bag)
        auto_takeoff = (
            ROOT / 'tools/flight/切Offboard后自动起飞0.75米单目标验证.sh'
        ).read_text(encoding='utf-8')
        self.assertIn('--lowrosbag', auto_takeoff)
        self.assertIn('--lowbag', auto_takeoff)
        self.assertIn('ROSBAG_PROFILE="${rosbag_profile}"', auto_takeoff)
        self.assertIn('manual_handover_min_altitude_m="0.50"', auto_takeoff)
        self.assertIn('manual_handover_max_altitude_m="1.20"', auto_takeoff)
        self.assertIn('tuning["shared_safety"]["fault_envelope"]["z_max"] = max_height', auto_takeoff)
        self.assertIn('wait_manual_position_handover || exit 4', auto_takeoff)
        self.assertIn('capture_planning_start_position', auto_takeoff)
        self.assertIn('wait_handover_accepted || exit 5', auto_takeoff)
        self.assertIn('wait_planning_tracking || exit 6', auto_takeoff)
        self.assertIn('setsid --wait env \\', auto_takeoff)
        manual_takeoff = (
            ROOT / 'tools/flight/人工起飞后Offboard单目标验证.sh'
        ).read_text(encoding='utf-8')
        self.assertIn('setsid --wait env \\', manual_takeoff)
        manual_block = auto_takeoff.split(
            'if [[ "${takeoff_mode}" == manual ]]; then\n  cat <<EOF', 1)[1].split(
            'else\n', 1)[0]
        self.assertNotIn('read -r', manual_block)
        for name in ('race_click_planner.rviz', 'race_mission_click_planner.rviz'):
            rviz = (ROOT / 'src/race_bringup/rviz' / name).read_text(encoding='utf-8')
            self.assertIn('Value: /cloud_registered', rviz)
            self.assertNotIn('/fast_lio/cloud_registered', rviz)

    def test_manual_handover_preserves_position_altitude_across_the_planning_chain(self):
        offboard = (ROOT / 'src/race_offboard/src/offboard_waypoint_node.cpp').read_text(
            encoding='utf-8')
        self.assertIn('lockManualHandoverFlightAltitudeReference()', offboard)
        self.assertIn('target_z_local_ned_ = static_cast<double>(current_z_);', offboard)
        self.assertIn('flight_target_agl_m_ = -static_cast<double>(current_z_);', offboard)

        consumers = (
            ROOT / 'src/race_super_planner_ros2/src/super_planner_ros2_node.cpp',
            ROOT / 'src/race_ego_bridge/src/ego_goal_bridge.cpp',
            ROOT / 'src/race_ego_bridge/src/ego_trajectory_bridge.cpp',
            ROOT / 'src/ego_planner_upstream/plan_manage/src/ego_replan_fsm.cpp',
        )
        for source in consumers:
            text = source.read_text(encoding='utf-8')
            self.assertIn('target_agl_m <', text)
            self.assertIn('target_agl_m >', text)
        script = (ROOT / 'tools/flight/切Offboard后自动起飞0.75米单目标验证.sh').read_text(
            encoding='utf-8')
        self.assertIn('verify_manual_handover_altitude_reference || exit 5', script)
        self.assertIn('规划路径将保持该高度', script)

    def test_pillar_area_tuning_reaches_every_target_node(self):
        overlays = node_parameter_overlays(self.tuning)
        self.assertEqual(
            self.tuning['ego_planner']['replan_start_position_error_m'],
            overlays['ego']['fsm/replan_start_position_error_m'])
        for key in (
                'handover_position_tolerance_m', 'handover_velocity_tolerance_mps',
                'handover_acceleration_tolerance_mps2'):
            self.assertEqual(
                self.tuning['ego_planner'][key], overlays['ego'][f'manager/{key}'])
            self.assertEqual(
                self.tuning['ego_planner'][key], overlays['ego'][f'fsm/{key}'])
        self.assertEqual(
            self.tuning['global_planner']['max_local_goal_distance'],
            overlays['super']['ego_local_goal_max_distance_m'])
        self.assertEqual(
            self.tuning['global_planner']['first_commit_max_path_error_m'],
            overlays['super']['pending_path_max_cross_track_error_m'])
        self.assertEqual(
            self.tuning['global_planner']['stale_start_max_replans'],
            overlays['super']['pending_path_max_replans'])
        self.assertEqual(
            self.tuning['ego_map']['live_obstacle_memory_sec'],
            overlays['cloud_bridge']['live_obstacle_memory_sec'])
        self.assertEqual(
            self.tuning['ego_map']['live_obstacle_voxel_size_m'],
            overlays['cloud_bridge']['live_obstacle_voxel_size_m'])
        self.assertEqual(
            self.tuning['ego_map']['pose_sync_max_skew_sec'],
            overlays['cloud_bridge']['max_pose_age_sec'])
        self.assertEqual(
            self.tuning['trajectory_bridge']['braking_deceleration_mps2'],
            overlays['offboard']['braking_max_acc'])
        self.assertEqual(
            self.tuning['ego_map']['minimum_reliable_detection_range_m'],
            overlays['trajectory_bridge']['minimum_reliable_detection_range_m'])
        self.assertEqual(
            self.tuning['trajectory_bridge']['active_recheck_history_sec'],
            overlays['trajectory_bridge']['active_recheck_history_sec'])
        self.assertEqual(
            self.tuning['offboard']['same_goal_retry_on_safety_latch'],
            overlays['offboard']['same_goal_retry_on_safety_latch'])

    def test_pillar_area_replan_threshold_stays_in_a_safe_band(self):
        for value in (0.14, 0.31):
            with self.subTest(value=value):
                bad = copy.deepcopy(self.tuning)
                bad['ego_planner']['replan_start_position_error_m'] = value
                with self.assertRaisesRegex(TuningError, 'replan_start_position_error_m'):
                    self._validate(bad)

    def test_pose_buffer_covers_both_sides_of_allowed_timestamp_skew(self):
        bad = copy.deepcopy(self.tuning)
        bad['ego_map']['pose_buffer_duration_sec'] = 0.59
        with self.assertRaisesRegex(TuningError, 'pose_buffer_duration_sec'):
            self._validate(bad)

    def test_maximum_speed_must_stop_inside_reliable_detection_range(self):
        bad = copy.deepcopy(self.tuning)
        bad['ego_map']['minimum_reliable_detection_range_m'] = 0.60
        with self.assertRaisesRegex(TuningError, 'braking horizon'):
            self._validate(bad)

    def test_live_obstacle_memory_must_cover_stopping_time(self):
        bad = copy.deepcopy(self.tuning)
        bad['ego_map']['live_obstacle_memory_sec'] = 0.60
        with self.assertRaisesRegex(TuningError, 'stopping time'):
            self._validate(bad)

    def test_unknown_map_source_fails(self):
        bad = copy.deepcopy(self.tuning)
        bad['ego_map_source'] = 'static_plus_live_typo'
        with self.assertRaisesRegex(TuningError, 'ego_map_source'):
            self._validate(bad)

    def test_missing_field_fails(self):
        bad = copy.deepcopy(self.tuning)
        del bad['global_planner']['grid_resolution']
        with self.assertRaisesRegex(TuningError, 'grid_resolution'):
            self._validate(bad)

    def test_invalid_x_bounds_fail(self):
        bad = copy.deepcopy(self.tuning)
        bad['shared_safety']['fault_envelope']['x_min'] = 31.0
        with self.assertRaisesRegex(TuningError, 'x_min'):
            self._validate(bad)

    def test_fixed_height_outside_bounds_fails(self):
        bad = copy.deepcopy(self.tuning)
        bad['ego_planner']['fixed_flight_height'] = 0.91
        with self.assertRaisesRegex(TuningError, 'fixed_flight_height'):
            self._validate(bad)

    def test_min_inflation_above_hard_fails(self):
        bad = copy.deepcopy(self.tuning)
        bad['global_planner']['min_planning_inflation_radius'] = 0.37
        with self.assertRaisesRegex(TuningError, 'min_planning'):
            self._validate(bad)

    def test_all_node_bounds_are_identical(self):
        overlays = node_parameter_overlays(self.tuning)
        expected = (-30.0, 30.0, -30.0, 30.0, 0.50, 0.90)
        for name in ('ego', 'goal_bridge', 'trajectory_bridge'):
            values = overlays[name]
            self.assertEqual(expected, (
                values['min_x'], values['max_x'], values['min_y'], values['max_y'],
                values['min_height'], values['max_height']))
        self.assertEqual(expected[:4], (
            overlays['super']['shared_bounds/x_min'], overlays['super']['shared_bounds/x_max'],
            overlays['super']['shared_bounds/y_min'], overlays['super']['shared_bounds/y_max']))
        self.assertEqual(expected[4:], (
            overlays['super']['shared_bounds/z_min'], overlays['super']['shared_bounds/z_max']))
        self.assertEqual(expected, (
            overlays['offboard']['shared_bounds/x_min'],
            overlays['offboard']['shared_bounds/x_max'],
            overlays['offboard']['shared_bounds/y_min'],
            overlays['offboard']['shared_bounds/y_max'],
            overlays['offboard']['shared_bounds/z_min'],
            overlays['offboard']['shared_bounds/z_max']))

    def test_ego_grid_resolution_is_explicit_and_positive(self):
        overlays = node_parameter_overlays(self.tuning)
        self.assertEqual(
            self.tuning['global_planner']['grid_resolution'],
            overlays['ego']['grid_map/resolution'])
        self.assertGreater(overlays['ego']['grid_map/resolution'], 0.0)

    def test_super_planning_resolution_is_independent_of_ego_map_resolution(self):
        overlays = node_parameter_overlays(self.tuning)
        self.assertEqual(0.05, overlays['super']['planning_resolution'])
        self.assertEqual(0.15, overlays['ego']['grid_map/resolution'])

    def test_hard_and_optimization_clearances_are_separate(self):
        overlays = node_parameter_overlays(self.tuning)
        required = self.tuning['shared_safety']['required_center_clearance']
        planning = self.tuning['ego_planner']['planning_clearance_m']
        target = self.tuning['ego_planner']['optimization_clearance_m']
        self.assertEqual(required, overlays['ego']['required_center_clearance'])
        self.assertEqual(target, overlays['ego']['optimization/dist0'])
        self.assertEqual(required, overlays['ego']['grid_map/obstacles_inflation'])
        self.assertEqual(planning, overlays['ego']['grid_map/planning_inflation'])
        self.assertGreater(planning, required)
        self.assertGreater(target, planning)
        self.assertNotIn('obstacles_inflation', self.tuning['ego_planner'])
        self.assertNotIn('dist0', self.tuning['ego_planner'])

    def test_required_center_clearance_maps_to_super_and_bridge(self):
        overlays = node_parameter_overlays(self.tuning)
        required = self.tuning['shared_safety']['required_center_clearance']
        self.assertEqual(required, overlays['super']['required_center_clearance'])
        self.assertEqual(required, overlays['trajectory_bridge']['obstacle_clearance'])

    def test_inflight_continuity_gate_clears_the_ego_replan_period(self):
        overlays = node_parameter_overlays(self.tuning)
        timeout = self.tuning['offboard']['ego_setpoint_timeout_sec']
        # The node declares this as "trajectory_timeout". Nothing used to map it,
        # so its 0.50 built-in default silently governed a latching safety gate
        # while sitting below EGO's 0.516s median replan period.
        self.assertEqual(timeout, overlays['offboard']['trajectory_timeout'])
        self.assertGreater(timeout, 0.981)
        self.assertLessEqual(timeout, 3.0)

    def test_handover_and_continuity_gates_stay_independent(self):
        overlays = node_parameter_overlays(self.tuning)
        # The strict handover gate only delays EGO takeover; the continuity gate
        # latches the whole flight. Collapsing them onto one value is what made a
        # normal replan gap indistinguishable from a planner failure.
        self.assertEqual(
            self.tuning['trajectory_bridge']['bspline_timeout_sec'],
            overlays['offboard']['ego_setpoint_timeout_sec'])
        self.assertLess(
            overlays['offboard']['ego_setpoint_timeout_sec'],
            overlays['offboard']['trajectory_timeout'])

    def test_inflight_continuity_gate_is_bounded(self):
        for value in (0.0, -0.1, 0.5, 1.0, 3.1):
            with self.subTest(value=value):
                bad = copy.deepcopy(self.tuning)
                bad['offboard']['ego_setpoint_timeout_sec'] = value
                with self.assertRaisesRegex(
                        TuningError, 'ego_setpoint_timeout_sec'):
                    self._validate(bad)

    def test_optimization_clearance_is_bounded_above_hard_limit(self):
        for value in (0.28, 0.441):
            with self.subTest(value=value):
                bad = copy.deepcopy(self.tuning)
                bad['ego_planner']['optimization_clearance_m'] = value
                with self.assertRaisesRegex(
                        TuningError, 'optimization_clearance_m'):
                    self._validate(bad)

    def test_planning_clearance_stays_between_hard_and_optimization_limits(self):
        for value in (0.28, 0.421, 0.431):
            with self.subTest(value=value):
                bad = copy.deepcopy(self.tuning)
                bad['ego_planner']['planning_clearance_m'] = value
                with self.assertRaisesRegex(TuningError, 'planning_clearance_m'):
                    self._validate(bad)

    def test_super_and_ego_share_local_reference_topic(self):
        overlays = node_parameter_overlays(self.tuning)
        topic = self.tuning['ego_recovery']['reference_path_topic']
        self.assertEqual(topic, overlays['super']['ego_reference_path_topic'])
        self.assertEqual(topic, overlays['ego']['fsm/ego_recovery/reference_path_topic'])

    def test_required_center_clearance_must_be_positive(self):
        bad = copy.deepcopy(self.tuning)
        bad['shared_safety']['required_center_clearance'] = 0.0
        with self.assertRaisesRegex(TuningError, 'required_center_clearance'):
            self._validate(bad)

    def test_required_center_clearance_must_be_finite(self):
        for value in (float('nan'), float('inf'), float('-inf')):
            with self.subTest(value=value):
                bad = copy.deepcopy(self.tuning)
                bad['shared_safety']['required_center_clearance'] = value
                with self.assertRaisesRegex(TuningError, 'required_center_clearance'):
                    self._validate(bad)

    def test_command_continuity_limits_are_positive_and_mapped(self):
        overlays = node_parameter_overlays(self.tuning)
        self.assertEqual(0.80, overlays['trajectory_bridge']['max_yaw_rate_rad_s'])
        self.assertEqual(0.80, overlays['offboard']['ego_yaw_rate_limit_rad_s'])
        self.assertEqual(2.0, overlays['offboard']['ego_setpoint_max_lead_m'])
        bad = copy.deepcopy(self.tuning)
        bad['offboard']['ego_setpoint_max_lead_m'] = 0.0
        with self.assertRaisesRegex(TuningError, 'ego_setpoint_max_lead_m'):
            self._validate(bad)

    def test_ego_occupancy_output_covers_the_flight_band(self):
        overlays = node_parameter_overlays(self.tuning)
        self.assertGreaterEqual(
            overlays['ego']['grid_map/visualization_truncate_height'],
            self.tuning['shared_safety']['fault_envelope']['z_max'])

    def test_calibrated_target_is_inside_fault_envelope(self):
        bounds = self.tuning['shared_safety']['fault_envelope']
        target = (10.9922, 3.58093, 0.78)
        self.assertGreaterEqual(target[0], bounds['x_min'])
        self.assertLessEqual(target[0], bounds['x_max'])
        self.assertGreaterEqual(target[1], bounds['y_min'])
        self.assertLessEqual(target[1], bounds['y_max'])
        self.assertGreaterEqual(target[2], bounds['z_min'])
        self.assertLessEqual(target[2], bounds['z_max'])

    def test_old_hard_boundary_neighborhood_is_allowed_but_extreme_faults_are_not(self):
        bounds = self.tuning['shared_safety']['fault_envelope']
        for point in ((-0.86, -3.21), (11.91, 12.41), (20.0, -20.0)):
            with self.subTest(point=point):
                self.assertGreaterEqual(point[0], bounds['x_min'])
                self.assertLessEqual(point[0], bounds['x_max'])
                self.assertGreaterEqual(point[1], bounds['y_min'])
                self.assertLessEqual(point[1], bounds['y_max'])
        self.assertGreater(30.01, bounds['x_max'])
        self.assertLess(-30.01, bounds['y_min'])

    def test_fault_envelope_is_not_an_ego_local_map_size_requirement(self):
        bounds = self.tuning['shared_safety']['fault_envelope']
        self.assertGreater(abs(bounds['x_max']),
                           self.tuning['ego_planner']['occupancy_map_size_x'] * 0.5)
        self.assertGreater(abs(bounds['y_max']),
                           self.tuning['ego_planner']['occupancy_map_size_y'] * 0.5)
        self.assertEqual(
            bounds['planning_grid_padding_m'],
            node_parameter_overlays(self.tuning)['super'][
                'fault_envelope/planning_grid_padding_m'])

    def test_recovery_framework_is_enabled_and_bounded(self):
        overlays = node_parameter_overlays(self.tuning)
        self.assertTrue(self.tuning['ego_planner']['use_distinctive_trajectories'])
        self.assertTrue(overlays['ego']['manager/use_distinctive_trajs'])
        self.assertEqual(
            self.tuning['ego_planner']['max_reference_deviation_m'],
            overlays['ego']['manager/max_reference_deviation_m'])
        self.assertTrue(self.tuning['ego_recovery']['enable'])
        self.assertFalse(self.tuning['ego_recovery']['diagnostic_only'])
        self.assertEqual(1, self.tuning['ego_recovery']['max_recovery_attempts'])
        self.assertEqual(0.5, self.tuning['ego_recovery']['cooldown_sec'])
        self.assertEqual(2.0, self.tuning['ego_recovery']['exhausted_backoff_sec'])
        self.assertEqual(0.6, self.tuning['ego_recovery']['rejoin_search_distance_m'])
        self.assertEqual(0.15, self.tuning['ego_recovery']['minimum_forward_progress_m'])
        self.assertEqual(0.06, self.tuning['ego_recovery']['extra_clearance_m'])
        self.assertEqual(1.50, self.tuning['ego_recovery']['max_reference_deviation_m'])
        self.assertEqual(0.75, self.tuning['ego_recovery']['max_lateral_offset_m'])
        self.assertEqual(0.15, self.tuning['ego_recovery']['lateral_offset_step_m'])
        self.assertEqual(
            0.75, overlays['ego']['fsm/ego_recovery/max_lateral_offset_m'])
        self.assertEqual(
            0.15, overlays['ego']['fsm/ego_recovery/lateral_offset_step_m'])
        self.assertTrue(overlays['ego']['fsm/ego_recovery/enable'])
        self.assertFalse(overlays['ego']['fsm/ego_recovery/diagnostic_only'])
        self.assertEqual(
            0.6, overlays['ego']['fsm/ego_recovery/rejoin_search_distance_m'])
        self.assertEqual(
            0.15, overlays['ego']['fsm/ego_recovery/minimum_forward_progress_m'])
        self.assertEqual(
            0.06, overlays['ego']['fsm/ego_recovery/extra_clearance_m'])
        self.assertEqual(
            1.50, overlays['ego']['fsm/ego_recovery/max_reference_deviation_m'])
        self.assertEqual(
            2.0, overlays['ego']['fsm/ego_recovery/exhausted_backoff_sec'])

    def test_replan_hold_timeout_reaches_the_trajectory_bridge(self):
        overlays = node_parameter_overlays(self.tuning)
        self.assertEqual(
            self.tuning['trajectory_bridge']['replan_hold_timeout_sec'],
            overlays['trajectory_bridge']['replan_hold_timeout_sec'])

    def test_reference_deviation_guard_is_bounded(self):
        for value in (0.0, 0.349, 1.001):
            with self.subTest(value=value):
                bad = copy.deepcopy(self.tuning)
                bad['ego_planner']['max_reference_deviation_m'] = value
                with self.assertRaisesRegex(TuningError, 'max_reference_deviation_m'):
                    self._validate(bad)

    def test_rejoin_search_distance_is_bounded(self):
        for value in (0.0, 0.59, 2.01):
            with self.subTest(value=value):
                bad = copy.deepcopy(self.tuning)
                bad['ego_recovery']['rejoin_search_distance_m'] = value
                with self.assertRaisesRegex(TuningError, 'rejoin_search_distance_m'):
                    self._validate(bad)

    def test_forward_progress_is_bounded_by_rejoin_search(self):
        for value in (0.0, 0.049, 0.401):
            with self.subTest(value=value):
                bad = copy.deepcopy(self.tuning)
                bad['ego_recovery']['minimum_forward_progress_m'] = value
                with self.assertRaisesRegex(TuningError, 'minimum_forward_progress_m'):
                    self._validate(bad)
        bad = copy.deepcopy(self.tuning)
        bad['ego_recovery']['minimum_forward_progress_m'] = 0.61
        with self.assertRaisesRegex(TuningError, 'minimum_forward_progress_m'):
            self._validate(bad)

    def test_recovery_geometry_limits_are_bounded(self):
        for value in (-0.01, 0.101):
            with self.subTest(extra_clearance_m=value):
                bad = copy.deepcopy(self.tuning)
                bad['ego_recovery']['extra_clearance_m'] = value
                with self.assertRaisesRegex(TuningError, 'extra_clearance_m'):
                    self._validate(bad)
        bad = copy.deepcopy(self.tuning)
        bad['ego_recovery']['extra_clearance_m'] = 0.09
        with self.assertRaisesRegex(TuningError, 'extra_clearance_m'):
            self._validate(bad)

        corridor_limit = self.tuning['shared_safety']['fault_envelope'][
            'mission_corridor_max_error_m']
        for value in (0.449, corridor_limit + 0.001):
            with self.subTest(max_reference_deviation_m=value):
                bad = copy.deepcopy(self.tuning)
                bad['ego_recovery']['max_reference_deviation_m'] = value
                with self.assertRaisesRegex(TuningError, 'max_reference_deviation_m'):
                    self._validate(bad)

    def test_handover_tolerances_are_bounded(self):
        limits = {
            'handover_position_tolerance_m': 0.10,
            'handover_velocity_tolerance_mps': 0.14,
            'handover_acceleration_tolerance_mps2': 0.10,
        }
        for key, limit in limits.items():
            for value in (0.0, -0.001, limit + 0.001):
                with self.subTest(key=key, value=value):
                    bad = copy.deepcopy(self.tuning)
                    bad['ego_planner'][key] = value
                    with self.assertRaisesRegex(TuningError, key):
                        self._validate(bad)

    def test_handover_tolerances_match_bridge(self):
        ego = self.tuning['ego_planner']
        bridge = self.tuning['trajectory_bridge']
        self.assertEqual(
            ego['handover_position_tolerance_m'],
            bridge['switch_position_tolerance_m'])
        self.assertEqual(
            ego['handover_velocity_tolerance_mps'],
            bridge['switch_velocity_tolerance_mps'])
        self.assertEqual(
            ego['handover_acceleration_tolerance_mps2'],
            bridge['switch_acceleration_tolerance_mps2'])

        bad = copy.deepcopy(self.tuning)
        bad['trajectory_bridge']['switch_velocity_tolerance_mps'] = 0.13
        with self.assertRaisesRegex(TuningError, 'must match'):
            self._validate(bad)

    def test_goal_bridge_background_release_and_legacy_stability_are_plumbed(self):
        overlays = node_parameter_overlays(self.tuning)
        offboard = self.tuning['offboard']
        self.assertEqual(0.40, offboard['ego_goal_release_height_m'])
        self.assertEqual(
            offboard['ego_goal_release_height_m'],
            overlays['goal_bridge']['release_height'])
        self.assertEqual(
            offboard['ego_goal_stable_duration_sec'],
            overlays['goal_bridge']['stable_duration_sec'])
        self.assertEqual(
            offboard['ego_goal_stable_height_tolerance_m'],
            overlays['goal_bridge']['stable_height_tolerance_m'])

    def test_background_release_height_is_bounded_but_may_precede_safety_window(self):
        # 0.40 m intentionally precedes the 0.50 m command safety window: it
        # starts planning only, while Offboard keeps the horizontal takeover
        # gated at takeoff_height_m.
        bounds = self.tuning['shared_safety']['fault_envelope']
        offboard = self.tuning['offboard']
        self.assertLess(offboard['ego_goal_release_height_m'], bounds['z_min'])
        for value in (0.0, offboard['fixed_flight_height_m'] + 0.001):
            with self.subTest(value=value):
                bad = copy.deepcopy(self.tuning)
                bad['offboard']['ego_goal_release_height_m'] = value
                with self.assertRaisesRegex(TuningError, 'ego_goal_release_height_m'):
                    self._validate(bad)

    def test_trajectory_lifecycle_and_switch_gates_are_plumbed(self):
        overlays = node_parameter_overlays(self.tuning)
        planner = self.tuning['global_planner']
        bridge = self.tuning['trajectory_bridge']
        for key in (
                'trajectory_prefetch_sec', 'trajectory_stall_timeout_sec',
                'trajectory_recovery_confirmation_sec'):
            self.assertEqual(planner[key], overlays['super'][key])
        for key in (
                'switch_position_tolerance_m', 'switch_velocity_tolerance_mps',
                'switch_acceleration_tolerance_mps2', 'handover_max_rejections',
                'handover_transaction_timeout_sec'):
            self.assertEqual(bridge[key], overlays['trajectory_bridge'][key])

    def test_recovery_can_be_disabled_without_changing_other_tuning(self):
        disabled = copy.deepcopy(self.tuning)
        disabled['ego_recovery']['enable'] = False
        overlays = self._validate(disabled)
        self.assertFalse(node_parameter_overlays(overlays)['ego']['fsm/ego_recovery/enable'])

    def test_recovery_has_exactly_one_fallback_attempt(self):
        for value in (0, 2):
            with self.subTest(value=value):
                bad = copy.deepcopy(self.tuning)
                bad['ego_recovery']['max_recovery_attempts'] = value
                with self.assertRaisesRegex(TuningError, 'max_recovery_attempts'):
                    self._validate(bad)

    def test_stale_global_start_has_exactly_one_replan(self):
        for value in (0, 2):
            with self.subTest(value=value):
                bad = copy.deepcopy(self.tuning)
                bad['global_planner']['stale_start_max_replans'] = value
                with self.assertRaisesRegex(TuningError, 'stale_start_max_replans'):
                    self._validate(bad)

    def test_first_commit_path_error_is_bounded(self):
        for value in (0.049, 0.501):
            with self.subTest(value=value):
                bad = copy.deepcopy(self.tuning)
                bad['global_planner']['first_commit_max_path_error_m'] = value
                with self.assertRaisesRegex(TuningError, 'first_commit_max_path_error_m'):
                    self._validate(bad)

    def test_expensive_ego_diagnostics_are_explicitly_disabled_by_default(self):
        overlays = node_parameter_overlays(self.tuning)
        self.assertFalse(self.tuning['ego_diagnostics']['enable'])
        self.assertFalse(overlays['ego']['ego_diagnostics.enable'])

    def test_ego_diagnostics_enable_must_be_boolean(self):
        bad = copy.deepcopy(self.tuning)
        bad['ego_diagnostics']['enable'] = 1
        with self.assertRaisesRegex(TuningError, 'ego_diagnostics.enable'):
            self._validate(bad)

    def _validate(self, document):
        import tempfile
        import yaml
        with tempfile.NamedTemporaryFile('w', suffix='.yaml') as stream:
            yaml.safe_dump(document, stream)
            stream.flush()
            return load_tuning(stream.name)


if __name__ == '__main__':
    unittest.main()
