"""Shared astar_ego tuning loader used by the canonical launch and offline checks."""

import math
from pathlib import Path

import yaml


class TuningError(ValueError):
    """Raised when the user-editable tuning file violates a safety contract."""


EGO_MAP_SOURCES = ('static', 'live', 'live_px4', 'static_live_px4')

REQUIRED = {
    'schema_version': int,
    'ego_map_source': str,
    'ego_map': dict,
    'shared_safety': dict,
    'global_planner': dict,
    'ego_planner': dict,
    'ego_recovery': dict,
    '动态放宽': dict,
    'ego_diagnostics': dict,
    'trajectory_bridge': dict,
    'offboard': dict,
    'diagnostics': dict,
}


def _require(mapping, key, expected_type, path):
    if key not in mapping:
        raise TuningError(f'missing required field: {path}.{key}')
    value = mapping[key]
    if expected_type is float:
        if not isinstance(value, (int, float)) or isinstance(value, bool):
            raise TuningError(f'{path}.{key} must be a number')
    elif not isinstance(value, expected_type):
        raise TuningError(f'{path}.{key} must be {expected_type.__name__}')
    return value


def _number(mapping, key, path):
    return float(_require(mapping, key, float, path))


def _non_negative(mapping, key, path):
    value = _number(mapping, key, path)
    if value < 0.0:
        raise TuningError(f'{path}.{key} must be >= 0')
    return value


def load_tuning(path):
    path = Path(path)
    if not path.is_file():
        raise TuningError(f'tuning file does not exist: {path}')
    with path.open(encoding='utf-8') as stream:
        tuning = yaml.safe_load(stream)
    if not isinstance(tuning, dict):
        raise TuningError('tuning document must be a mapping')
    for key, expected_type in REQUIRED.items():
        _require(tuning, key, expected_type, 'root')
    if tuning['schema_version'] != 1:
        raise TuningError('schema_version must be 1')
    # Fail closed on a typo: an unknown source must abort startup rather than
    # silently fall back to a map the operator did not ask for.
    if tuning['ego_map_source'] not in EGO_MAP_SOURCES:
        raise TuningError(
            'ego_map_source must be one of '
            f"{', '.join(EGO_MAP_SOURCES)}")

    shared = tuning['shared_safety']
    _require(shared, 'frame_id', str, 'shared_safety')
    required_center_clearance = _number(
        shared, 'required_center_clearance', 'shared_safety')
    if not math.isfinite(required_center_clearance) or required_center_clearance <= 0.0:
        raise TuningError(
            'shared_safety.required_center_clearance must be finite and > 0')
    bounds = _require(shared, 'fault_envelope', dict, 'shared_safety')
    for axis in ('x', 'y', 'z'):
        lower = _number(bounds, f'{axis}_min', 'shared_safety.fault_envelope')
        upper = _number(bounds, f'{axis}_max', 'shared_safety.fault_envelope')
        if lower >= upper:
            raise TuningError(
                f'shared_safety.fault_envelope.{axis}_min must be < {axis}_max')
    for key in ('position_max_speed_mps', 'position_jump_allowance_m',
                'planning_grid_padding_m',
                'mission_corridor_max_error_m'):
        if _non_negative(bounds, key, 'shared_safety.fault_envelope') <= 0.0:
            raise TuningError(f'shared_safety.fault_envelope.{key} must be > 0')

    global_planner = tuning['global_planner']
    for key in (
            'grid_resolution', 'planning_resolution', 'hard_inflation_radius',
            'min_planning_inflation_radius', 'soft_obstacle_cost_radius',
            'clearance_cost_weight', 'tracking_lookahead_distance',
            'max_local_goal_distance', 'smoothing_min_clearance',
            'first_commit_max_path_error_m',
            'trajectory_prefetch_sec', 'trajectory_stall_timeout_sec',
            'trajectory_recovery_confirmation_sec'):
        _non_negative(global_planner, key, 'global_planner')
    wind_recovery = _require(global_planner, 'wind_recovery', dict, 'global_planner')
    _require(wind_recovery, 'enable', bool, 'global_planner.wind_recovery')
    for key in ('stall_sec', 'stage_sec', 'min_final_distance_m',
                'stalled_progress_mps', 'recovered_progress_mps'):
        _non_negative(wind_recovery, key, 'global_planner.wind_recovery')
    if not 3.0 <= wind_recovery['stall_sec'] <= 5.0:
        raise TuningError('global_planner.wind_recovery.stall_sec must be in [3.0, 5.0]')
    if not 3.0 <= wind_recovery['stage_sec'] <= 5.0:
        raise TuningError('global_planner.wind_recovery.stage_sec must be in [3.0, 5.0]')
    if wind_recovery['recovered_progress_mps'] <= wind_recovery['stalled_progress_mps']:
        raise TuningError(
            'global_planner.wind_recovery.recovered_progress_mps must exceed stalled_progress_mps')
    terminal_slowdown = _require(
        global_planner, 'terminal_slowdown', dict, 'global_planner')
    _require(terminal_slowdown, 'enable', bool, 'global_planner.terminal_slowdown')
    slowdown_distance = _non_negative(
        terminal_slowdown, 'distance_m', 'global_planner.terminal_slowdown')
    stop_distance = _non_negative(
        terminal_slowdown, 'stop_distance_m', 'global_planner.terminal_slowdown')
    if not 0.05 <= stop_distance <= 0.30:
        raise TuningError(
            'global_planner.terminal_slowdown.stop_distance_m must be in [0.05, 0.30]')
    if slowdown_distance <= stop_distance:
        raise TuningError(
            'global_planner.terminal_slowdown.distance_m must exceed stop_distance_m')
    if slowdown_distance < global_planner['tracking_lookahead_distance']:
        raise TuningError(
            'global_planner.terminal_slowdown.distance_m must be >= tracking_lookahead_distance')
    for key in ('allow_direct_path', 'enable_path_shortcut'):
        _require(global_planner, key, bool, 'global_planner')
    stale_start_max_replans = _require(
        global_planner, 'stale_start_max_replans', int, 'global_planner')
    if not 0.05 <= global_planner['first_commit_max_path_error_m'] <= 0.50:
        raise TuningError(
            'global_planner.first_commit_max_path_error_m must be in [0.05, 0.50]')
    if stale_start_max_replans != 1:
        raise TuningError('global_planner.stale_start_max_replans must be 1')
    if global_planner['min_planning_inflation_radius'] > global_planner['hard_inflation_radius']:
        raise TuningError(
            'global_planner.min_planning_inflation_radius must be <= hard_inflation_radius')
    if global_planner['soft_obstacle_cost_radius'] < global_planner['hard_inflation_radius']:
        raise TuningError(
            'global_planner.soft_obstacle_cost_radius must be >= hard_inflation_radius')
    if global_planner['trajectory_prefetch_sec'] < 0.5:
        raise TuningError('global_planner.trajectory_prefetch_sec must be >= 0.5')
    if not 0.5 <= global_planner['trajectory_stall_timeout_sec'] <= 3.0:
        raise TuningError(
            'global_planner.trajectory_stall_timeout_sec must be in [0.5, 3.0]')
    if not 0.2 <= global_planner['trajectory_recovery_confirmation_sec'] <= \
            global_planner['trajectory_stall_timeout_sec']:
        raise TuningError(
            'global_planner.trajectory_recovery_confirmation_sec must be in '
            '[0.2, trajectory_stall_timeout_sec]')

    ego = tuning['ego_planner']
    _require(ego, 'enable', bool, 'ego_planner')
    _require(ego, 'use_distinctive_trajectories', bool, 'ego_planner')
    _require(ego, 'constrained_clearance_enable', bool, 'ego_planner')
    planning_clearance = _number(
        ego, 'planning_clearance_m', 'ego_planner')
    optimization_clearance = _number(
        ego, 'optimization_clearance_m', 'ego_planner')
    constrained_clearance = _number(
        ego, 'constrained_clearance_m', 'ego_planner')
    constrained_optimization_clearance = _number(
        ego, 'constrained_optimization_clearance_m', 'ego_planner')
    for key in (
            'planning_horizon', 'local_update_range_x',
            'local_update_range_y', 'local_update_range_z', 'max_velocity',
            'max_acceleration', 'control_point_spacing', 'fixed_flight_height',
            'occupancy_publish_max_height', 'occupancy_map_size_x',
            'occupancy_map_size_y', 'replan_start_position_error_m',
            'max_reference_deviation_m', 'handover_prediction_initial_sec'):
        _non_negative(ego, key, 'ego_planner')
    for key in (
            'handover_position_tolerance_m', 'handover_velocity_tolerance_mps',
            'handover_acceleration_tolerance_mps2'):
        if _non_negative(ego, key, 'ego_planner') <= 0.0:
            raise TuningError(f'ego_planner.{key} must be > 0')
    if ego['handover_position_tolerance_m'] > 0.10:
        raise TuningError('ego_planner.handover_position_tolerance_m must be <= 0.10')
    if ego['handover_velocity_tolerance_mps'] > 0.14:
        raise TuningError('ego_planner.handover_velocity_tolerance_mps must be <= 0.14')
    if ego['handover_acceleration_tolerance_mps2'] > 0.10:
        raise TuningError(
            'ego_planner.handover_acceleration_tolerance_mps2 must be <= 0.10')
    if not 0.15 <= ego['replan_start_position_error_m'] <= 0.30:
        raise TuningError(
            'ego_planner.replan_start_position_error_m must be in [0.15, 0.30]')
    if not 0.10 <= ego['handover_prediction_initial_sec'] <= 1.00:
        raise TuningError(
            'ego_planner.handover_prediction_initial_sec must be in [0.10, 1.00]')
    if not 0.35 <= ego['max_reference_deviation_m'] <= 1.00:
        raise TuningError(
            'ego_planner.max_reference_deviation_m must be in [0.35, 1.00]')
    if global_planner['max_local_goal_distance'] > ego['planning_horizon']:
        raise TuningError(
            'global_planner.max_local_goal_distance must be <= ego_planner.planning_horizon')
    if planning_clearance < required_center_clearance:
        raise TuningError(
            'ego_planner.planning_clearance_m must be >= '
            'shared_safety.required_center_clearance')
    if not math.isfinite(constrained_clearance) or constrained_clearance < 0.207132:
        raise TuningError(
            'ego_planner.constrained_clearance_m must be finite and >= physical envelope 0.207132')
    if constrained_clearance >= planning_clearance:
        raise TuningError(
            'ego_planner.constrained_clearance_m must be < ego_planner.planning_clearance_m')
    if constrained_clearance >= required_center_clearance:
        raise TuningError(
            'ego_planner.constrained_clearance_m must be < '
            'shared_safety.required_center_clearance')
    if constrained_optimization_clearance < constrained_clearance:
        raise TuningError(
            'ego_planner.constrained_optimization_clearance_m must be >= '
            'ego_planner.constrained_clearance_m')
    if constrained_optimization_clearance >= planning_clearance:
        raise TuningError(
            'ego_planner.constrained_optimization_clearance_m must be < '
            'ego_planner.planning_clearance_m')
    if planning_clearance > required_center_clearance + 0.12:
        raise TuningError(
            'ego_planner.planning_clearance_m must be <= '
            'required_center_clearance + 0.12')
    if optimization_clearance < planning_clearance:
        raise TuningError(
            'ego_planner.optimization_clearance_m must be >= '
            'ego_planner.planning_clearance_m')
    if optimization_clearance > required_center_clearance + 0.15:
        raise TuningError(
            'ego_planner.optimization_clearance_m must be <= '
            'required_center_clearance + 0.15')
    # Soft-only tail bias.  It cannot weaken collision avoidance, but an
    # unbounded value would make the far end unsatisfiable in narrow sections
    # and turn every replan into a recovery attempt.
    far_end_extra = _number(ego, 'far_end_clearance_extra_m', 'ego_planner')
    if not 0.0 <= far_end_extra <= 0.15:
        raise TuningError(
            'ego_planner.far_end_clearance_extra_m must be in [0.0, 0.15]')
    if optimization_clearance + far_end_extra > required_center_clearance + 0.25:
        raise TuningError(
            'ego_planner.optimization_clearance_m + far_end_clearance_extra_m '
            'must be <= required_center_clearance + 0.25')

    ego_map = tuning['ego_map']
    for key in ('live_topic', 'body_topic', 'pose_topic'):
        topic = _require(ego_map, key, str, 'ego_map')
        if not topic.startswith('/') or topic == '/':
            raise TuningError(f'ego_map.{key} must be an absolute ROS topic')
    live_memory = _non_negative(ego_map, 'live_obstacle_memory_sec', 'ego_map')
    if live_memory > 1.5:
        raise TuningError('ego_map.live_obstacle_memory_sec must be <= 1.5')
    live_voxel = _number(ego_map, 'live_obstacle_voxel_size_m', 'ego_map')
    if not 0.05 <= live_voxel <= 0.25:
        raise TuningError('ego_map.live_obstacle_voxel_size_m must be in [0.05, 0.25]')
    live_observations = _require(
        ego_map, 'live_obstacle_min_observations', int, 'ego_map')
    if not 1 <= live_observations <= 5:
        raise TuningError(
            'ego_map.live_obstacle_min_observations must be in [1, 5]')
    for key in ('minimum_reliable_detection_range_m', 'pose_sync_max_skew_sec',
                'pose_buffer_duration_sec'):
        if _non_negative(ego_map, key, 'ego_map') <= 0.0:
            raise TuningError(f'ego_map.{key} must be > 0')
    if ego_map['pose_buffer_duration_sec'] < 2.0 * ego_map['pose_sync_max_skew_sec']:
        raise TuningError(
            'ego_map.pose_buffer_duration_sec must be >= 2 * pose_sync_max_skew_sec')

    recovery = tuning['ego_recovery']
    for key in ('enable', 'diagnostic_only'):
        _require(recovery, key, bool, 'ego_recovery')
    _require(recovery, 'reference_path_topic', str, 'ego_recovery')
    max_attempts = _require(recovery, 'max_recovery_attempts', int, 'ego_recovery')
    if max_attempts != 1:
        raise TuningError(
            'ego_recovery.max_recovery_attempts must be 1: normal EGO plus one fallback')
    _non_negative(recovery, 'cooldown_sec', 'ego_recovery')
    exhausted_backoff = _non_negative(
        recovery, 'exhausted_backoff_sec', 'ego_recovery')
    if exhausted_backoff < recovery['cooldown_sec']:
        raise TuningError(
            'ego_recovery.exhausted_backoff_sec must be >= cooldown_sec')
    rejoin_search_distance = _number(
        recovery, 'rejoin_search_distance_m', 'ego_recovery')
    if not 0.60 <= rejoin_search_distance <= 2.0:
        raise TuningError(
            'ego_recovery.rejoin_search_distance_m must be in [0.60, 2.0]')
    minimum_forward_progress = _number(
        recovery, 'minimum_forward_progress_m', 'ego_recovery')
    if not 0.05 <= minimum_forward_progress <= 0.40:
        raise TuningError(
            'ego_recovery.minimum_forward_progress_m must be in [0.05, 0.40]')
    if minimum_forward_progress > rejoin_search_distance:
        raise TuningError(
            'ego_recovery.minimum_forward_progress_m must be <= '
            'rejoin_search_distance_m')
    extra_clearance = _number(recovery, 'extra_clearance_m', 'ego_recovery')
    if not 0.0 <= extra_clearance <= 0.10:
        raise TuningError(
            'ego_recovery.extra_clearance_m must be in [0.0, 0.10]')
    if planning_clearance + extra_clearance > optimization_clearance + 1.0e-9:
        raise TuningError(
            'ego_planner.planning_clearance_m + ego_recovery.extra_clearance_m '
            'must be <= ego_planner.optimization_clearance_m')
    recovery_reference_deviation = _number(
        recovery, 'max_reference_deviation_m', 'ego_recovery')
    if not 0.45 <= recovery_reference_deviation <= bounds['mission_corridor_max_error_m']:
        raise TuningError(
            'ego_recovery.max_reference_deviation_m must be in [0.45, '
            'shared_safety.fault_envelope.mission_corridor_max_error_m]')
    max_lateral_offset = _number(
        recovery, 'max_lateral_offset_m', 'ego_recovery')
    lateral_offset_step = _number(
        recovery, 'lateral_offset_step_m', 'ego_recovery')
    # The lateral rejoin fallback needs to reach past a pillar sitting on the
    # reference line.  A 0.5m box at planning_clearance_m + extra_clearance_m
    # blocks lateral offsets up to half-width + that clearance, so the ladder
    # must be able to exceed it or the fallback cannot help.
    required_lateral_reach = 0.25 + planning_clearance + extra_clearance
    if not 0.0 < max_lateral_offset <= recovery_reference_deviation:
        raise TuningError(
            'ego_recovery.max_lateral_offset_m must be in (0, '
            'ego_recovery.max_reference_deviation_m]')
    if max_lateral_offset < required_lateral_reach:
        raise TuningError(
            'ego_recovery.max_lateral_offset_m must be >= '
            f'{required_lateral_reach:.3f} to clear a 0.5m pillar at '
            'planning_clearance_m + extra_clearance_m')
    if not 0.05 <= lateral_offset_step <= max_lateral_offset:
        raise TuningError(
            'ego_recovery.lateral_offset_step_m must be in [0.05, '
            'ego_recovery.max_lateral_offset_m]')
    active_preplan_lookahead = _number(
        recovery, 'active_preplan_lookahead_m', 'ego_recovery')
    if not 0.30 <= active_preplan_lookahead <= 1.50:
        raise TuningError(
            'ego_recovery.active_preplan_lookahead_m must be in [0.30, 1.50]')

    # 飞手可读的中文动态放宽配置。它只能放宽软代价和搜索范围，所有硬防撞
    # 判据仍由 shared_safety 和 trajectory_bridge 独立执行。
    dynamic = tuning['动态放宽']
    _require(dynamic, '启用', bool, '动态放宽')
    for key in ('软放宽等待秒', '扩大绕行等待秒', '软放宽障碍代价半径米',
                '软放宽靠障碍代价权重', '软放宽恢复额外净空米',
                '扩大后重接搜索距离米', '扩大后最大横向搜索距离米',
                '扩大后最小前进距离米'):
        if _non_negative(dynamic, key, '动态放宽') <= 0.0:
            raise TuningError(f'动态放宽.{key} must be > 0')
    if dynamic['软放宽等待秒'] < 3.0 or dynamic['扩大绕行等待秒'] < 3.0:
        raise TuningError('动态放宽的两个等待时间不得小于 3 秒')
    if not global_planner['hard_inflation_radius'] <= \
            dynamic['软放宽障碍代价半径米'] <= global_planner['soft_obstacle_cost_radius']:
        raise TuningError('动态放宽.软放宽障碍代价半径米 must stay between hard and normal radius')
    if dynamic['软放宽靠障碍代价权重'] > global_planner['clearance_cost_weight']:
        raise TuningError('动态放宽.软放宽靠障碍代价权重 must not exceed normal weight')
    if dynamic['软放宽恢复额外净空米'] > recovery['extra_clearance_m']:
        raise TuningError('动态放宽.软放宽恢复额外净空米 must not exceed normal extra clearance')
    if dynamic['扩大后重接搜索距离米'] < recovery['rejoin_search_distance_m'] or \
            dynamic['扩大后重接搜索距离米'] > 2.0:
        raise TuningError('动态放宽.扩大后重接搜索距离米 must be within normal..2.0m')
    if dynamic['扩大后最大横向搜索距离米'] < recovery['max_lateral_offset_m'] or \
            dynamic['扩大后最大横向搜索距离米'] > recovery_reference_deviation:
        raise TuningError('动态放宽.扩大后最大横向搜索距离米 must be within normal..最大参考偏离')
    if not 0.05 <= dynamic['扩大后最小前进距离米'] <= recovery['minimum_forward_progress_m']:
        raise TuningError('动态放宽.扩大后最小前进距离米 must be in [0.05, normal minimum]')

    ego_diagnostics = tuning['ego_diagnostics']
    _require(ego_diagnostics, 'enable', bool, 'ego_diagnostics')

    bridge = tuning['trajectory_bridge']
    for key in (
            'occupancy_timeout_sec', 'bspline_timeout_sec', 'command_timeout_sec',
            'active_recheck_history_sec',
            'dynamic_invalid_grace_sec', 'collision_sample_spacing_m',
            'replan_hold_timeout_sec',
            'switch_position_tolerance_m', 'switch_velocity_tolerance_mps',
            'switch_acceleration_tolerance_mps2',
            'handover_transaction_timeout_sec',
            'dynamic_limit_margin', 'max_yaw_rate_rad_s',
            'braking_deceleration_mps2',
            'reaction_time_sec'):
        if _non_negative(bridge, key, 'trajectory_bridge') <= 0.0:
            raise TuningError(f'trajectory_bridge.{key} must be > 0')
    if bridge['collision_sample_spacing_m'] > global_planner['grid_resolution']:
        raise TuningError(
            'trajectory_bridge.collision_sample_spacing_m must be <= '
            'global_planner.grid_resolution')
    if not 0.10 <= bridge['active_recheck_history_sec'] <= 1.0:
        raise TuningError(
            'trajectory_bridge.active_recheck_history_sec must be in [0.10, 1.0]')
    if bridge['max_yaw_rate_rad_s'] <= 0.0:
        raise TuningError('trajectory_bridge.max_yaw_rate_rad_s must be > 0')
    if not 0.50 <= bridge['replan_hold_timeout_sec'] <= 3.0:
        raise TuningError(
            'trajectory_bridge.replan_hold_timeout_sec must be in [0.50, 3.0]')
    if bridge['switch_position_tolerance_m'] > 0.10:
        raise TuningError(
            'trajectory_bridge.switch_position_tolerance_m must be <= 0.10')
    if bridge['switch_velocity_tolerance_mps'] > 0.14:
        raise TuningError(
            'trajectory_bridge.switch_velocity_tolerance_mps must be <= 0.14')
    if bridge['switch_acceleration_tolerance_mps2'] > 0.10:
        raise TuningError(
            'trajectory_bridge.switch_acceleration_tolerance_mps2 must be <= 0.10')
    for ego_key, bridge_key in (
            ('handover_position_tolerance_m', 'switch_position_tolerance_m'),
            ('handover_velocity_tolerance_mps', 'switch_velocity_tolerance_mps'),
            ('handover_acceleration_tolerance_mps2',
             'switch_acceleration_tolerance_mps2')):
        if not math.isclose(ego[ego_key], bridge[bridge_key], abs_tol=1.0e-9):
            raise TuningError(
                f'ego_planner.{ego_key} must match '
                f'trajectory_bridge.{bridge_key}')
    handover_max_rejections = bridge.get('handover_max_rejections')
    if not isinstance(handover_max_rejections, int) or handover_max_rejections not in (1, 2, 3):
        raise TuningError('trajectory_bridge.handover_max_rejections must be 1, 2, or 3')
    maximum_speed = ego['max_velocity']
    stopping_time = (
        bridge['reaction_time_sec'] +
        maximum_speed / bridge['braking_deceleration_mps2'])
    if (tuning['ego_map_source'] != 'static' and
            ego_map['live_obstacle_memory_sec'] + 1.0e-9 < stopping_time):
        raise TuningError(
            'ego_map.live_obstacle_memory_sec must cover reaction plus '
            'maximum-speed stopping time')
    braking_horizon = (
        required_center_clearance +
        maximum_speed * bridge['reaction_time_sec'] +
        maximum_speed * maximum_speed / (2.0 * bridge['braking_deceleration_mps2']))
    if braking_horizon > ego_map['minimum_reliable_detection_range_m']:
        raise TuningError(
            'maximum-speed braking horizon exceeds '
            'ego_map.minimum_reliable_detection_range_m')

    offboard = tuning['offboard']
    for key in (
            'takeoff_height_m', 'ego_goal_release_height_m', 'fixed_flight_height_m',
            'max_safe_height_m', 'goal_tolerance_xy_m', 'goal_tolerance_z_m',
            'setpoint_rate_hz', 'ego_goal_stable_height_tolerance_m',
            'ego_goal_stable_horizontal_speed_mps', 'ego_goal_stable_vertical_speed_mps',
            'ego_goal_stable_duration_sec', 'trusted_position_max_speed_mps',
            'trusted_position_jump_allowance_m', 'ego_setpoint_max_lead_m',
            'ego_yaw_rate_limit_rad_s', 'initial_position_stabilization_sec',
            'initial_position_max_spread_m', 'initial_position_max_speed_mps',
            'ego_setpoint_timeout_sec'):
        _non_negative(offboard, key, 'offboard')
    _require(offboard, 'same_goal_retry_on_safety_latch', bool, 'offboard')
    # Exceeding this timeout latches PLANNER_FAILURE_HOLD for the remainder of
    # the flight, so it has to clear EGO's measured replan period (median 0.516s,
    # max 0.981s) with margin. Anything at or below one second reintroduces the
    # false latch that stopped 20260801_182422 at age=0.543 status=PLANNING.
    if offboard['ego_setpoint_timeout_sec'] <= 1.0:
        raise TuningError(
            'offboard.ego_setpoint_timeout_sec must be > 1.0 to clear the '
            'measured EGO replan period')
    # The gate is only useful if it still expires well inside a trajectory, or
    # the vehicle could keep tracking a trajectory that has fully run out.
    if offboard['ego_setpoint_timeout_sec'] > 3.0:
        raise TuningError('offboard.ego_setpoint_timeout_sec must be <= 3.0')
    z_min = bounds['z_min']
    z_max = bounds['z_max']
    if ego['occupancy_publish_max_height'] < z_max:
        raise TuningError(
            'ego_planner.occupancy_publish_max_height must cover shared z_max')
    for name, value in (
            ('ego_planner.fixed_flight_height', ego['fixed_flight_height']),
            ('offboard.fixed_flight_height_m', offboard['fixed_flight_height_m']),
            ('offboard.takeoff_height_m', offboard['takeoff_height_m'])):
        if not z_min <= value <= z_max:
            raise TuningError(
                f'{name} must be inside shared_safety.fault_envelope.z_min/z_max')
    if offboard['ego_goal_release_height_m'] > offboard['fixed_flight_height_m']:
        raise TuningError('offboard.ego_goal_release_height_m must be <= fixed_flight_height_m')
    if offboard['ego_goal_release_height_m'] <= 0.0:
        raise TuningError('offboard.ego_goal_release_height_m must be > 0')
    if offboard['max_safe_height_m'] > z_max:
        raise TuningError(
            'offboard.max_safe_height_m must be <= shared_safety.fault_envelope.z_max')
    for key in ('trusted_position_max_speed_mps', 'trusted_position_jump_allowance_m',
                'ego_setpoint_max_lead_m', 'ego_yaw_rate_limit_rad_s',
                'initial_position_stabilization_sec', 'initial_position_max_spread_m',
                'initial_position_max_speed_mps'):
        if offboard[key] <= 0.0:
            raise TuningError(f'offboard.{key} must be > 0')
    if not isinstance(tuning['diagnostics'].get('print_effective_tuning'), bool):
        raise TuningError('diagnostics.print_effective_tuning must be bool')
    if _non_negative(tuning['diagnostics'], 'tuning_summary_rate_hz', 'diagnostics') <= 0.0:
        raise TuningError('diagnostics.tuning_summary_rate_hz must be > 0')
    return tuning


def node_parameter_overlays(tuning):
    """Return the exact node parameter overrides built from the one YAML source."""
    bounds = tuning['shared_safety']['fault_envelope']
    required_center_clearance = tuning['shared_safety']['required_center_clearance']
    global_planner = tuning['global_planner']
    ego = tuning['ego_planner']
    constrained_clearance = ego['constrained_clearance_m']
    constrained_optimization_clearance = ego['constrained_optimization_clearance_m']
    ego_map = tuning['ego_map']
    recovery = tuning['ego_recovery']
    dynamic = tuning['动态放宽']
    ego_diagnostics = tuning['ego_diagnostics']
    bridge = tuning['trajectory_bridge']
    offboard = tuning['offboard']
    shared = {
        'min_x': bounds['x_min'], 'max_x': bounds['x_max'],
        'min_y': bounds['y_min'], 'max_y': bounds['y_max'],
        'min_height': bounds['z_min'], 'max_height': bounds['z_max'],
    }
    return {
        'super': {
            'shared_bounds/x_min': bounds['x_min'], 'shared_bounds/x_max': bounds['x_max'],
            'shared_bounds/y_min': bounds['y_min'], 'shared_bounds/y_max': bounds['y_max'],
            'shared_bounds/z_min': bounds['z_min'], 'shared_bounds/z_max': bounds['z_max'],
            'fault_envelope/max_speed_mps': bounds['position_max_speed_mps'],
            'fault_envelope/jump_allowance_m': bounds['position_jump_allowance_m'],
            'fault_envelope/planning_grid_padding_m': bounds['planning_grid_padding_m'],
            'mission_corridor_max_error_m': bounds['mission_corridor_max_error_m'],
            'world_frame': tuning['shared_safety']['frame_id'],
            'fixed_flight_height': ego['fixed_flight_height'],
            'min_safe_height': bounds['z_min'], 'max_safe_height': bounds['z_max'],
            'grid_resolution': global_planner['grid_resolution'],
            'planning_resolution': global_planner['planning_resolution'],
            'required_center_clearance': required_center_clearance,
            'obstacle_inflation_radius': global_planner['hard_inflation_radius'],
            'min_planning_inflation_radius': global_planner['min_planning_inflation_radius'],
            'soft_obstacle_cost_radius': global_planner['soft_obstacle_cost_radius'],
            'clearance_cost_weight': global_planner['clearance_cost_weight'],
            'tracking_lookahead_distance': global_planner['tracking_lookahead_distance'],
            'terminal_slowdown/enable': global_planner['terminal_slowdown']['enable'],
            'terminal_slowdown/distance_m': global_planner['terminal_slowdown']['distance_m'],
            'terminal_slowdown/stop_distance_m':
                global_planner['terminal_slowdown']['stop_distance_m'],
            'wind_recovery/enable': global_planner['wind_recovery']['enable'],
            'wind_recovery/stall_sec': global_planner['wind_recovery']['stall_sec'],
            'wind_recovery/stage_sec': global_planner['wind_recovery']['stage_sec'],
            'wind_recovery/min_final_distance_m':
                global_planner['wind_recovery']['min_final_distance_m'],
            'wind_recovery/stalled_progress_mps':
                global_planner['wind_recovery']['stalled_progress_mps'],
            'wind_recovery/recovered_progress_mps':
                global_planner['wind_recovery']['recovered_progress_mps'],
            'pending_path_max_cross_track_error_m':
                global_planner['first_commit_max_path_error_m'],
            'pending_path_max_replans': global_planner['stale_start_max_replans'],
            'ego_local_goal_max_distance_m': global_planner['max_local_goal_distance'],
            'trajectory_prefetch_sec': global_planner['trajectory_prefetch_sec'],
            'trajectory_stall_timeout_sec':
                global_planner['trajectory_stall_timeout_sec'],
            'trajectory_recovery_confirmation_sec':
                global_planner['trajectory_recovery_confirmation_sec'],
            'smoothing_min_clearance': global_planner['smoothing_min_clearance'],
            'allow_direct_path': global_planner['allow_direct_path'],
            'enable_path_shortcut': global_planner['enable_path_shortcut'],
            'ego_reference_path_topic': recovery['reference_path_topic'],
            # Recovery searches along this continuation, so the preview must
            # cover the configured rejoin distance.
            'ego_reference_preview_distance_m': max(
                0.60, recovery['rejoin_search_distance_m'],
                dynamic['扩大后重接搜索距离米']),
            'dynamic_relaxation/enable': dynamic['启用'],
            'dynamic_relaxation/soft_wait_sec': dynamic['软放宽等待秒'],
            'dynamic_relaxation/expand_wait_sec': dynamic['扩大绕行等待秒'],
            'dynamic_relaxation/soft_cost_radius_m': dynamic['软放宽障碍代价半径米'],
            'dynamic_relaxation/soft_cost_weight': dynamic['软放宽靠障碍代价权重'],
        },
        'ego': {
            **shared,
            'required_center_clearance': required_center_clearance,
            'fsm/constrained_clearance/enable': ego['constrained_clearance_enable'],
            'fsm/constrained_clearance/clearance_m': constrained_clearance,
            'fsm/constrained_clearance/optimization_m': constrained_optimization_clearance,
            'optimization/dist0': ego['optimization_clearance_m'],
            'optimization/far_end_clearance_extra':
                ego['far_end_clearance_extra_m'],
            # GridMap has no usable default resolution.  Leaving this unset makes
            # its default -1.0 produce negative voxel dimensions at startup.
            'grid_map/resolution': global_planner['grid_resolution'],
            # GridMap retains its upstream parameter name, but its sole value
            # is the shared centre-clearance contract above.
            'grid_map/obstacles_inflation': required_center_clearance,
            'grid_map/planning_inflation': ego['planning_clearance_m'],
            'fsm/planning_horizon': ego['planning_horizon'],
            'manager/planning_horizon': ego['planning_horizon'],
            'grid_map/local_update_range_x': ego['local_update_range_x'],
            'grid_map/local_update_range_y': ego['local_update_range_y'],
            'grid_map/local_update_range_z': ego['local_update_range_z'],
            'grid_map/map_size_x': ego['occupancy_map_size_x'],
            'grid_map/map_size_y': ego['occupancy_map_size_y'],
            'grid_map/visualization_truncate_height': ego['occupancy_publish_max_height'],
            'manager/max_vel': ego['max_velocity'],
            'optimization/max_vel': ego['max_velocity'],
            'manager/max_acc': ego['max_acceleration'],
            'optimization/max_acc': ego['max_acceleration'],
            'manager/control_points_distance': ego['control_point_spacing'],
            'manager/use_distinctive_trajs': ego['use_distinctive_trajectories'],
            'manager/max_reference_deviation_m': ego['max_reference_deviation_m'],
            'manager/handover_position_tolerance_m': ego['handover_position_tolerance_m'],
            'manager/handover_velocity_tolerance_mps': ego['handover_velocity_tolerance_mps'],
            'manager/handover_acceleration_tolerance_mps2':
                ego['handover_acceleration_tolerance_mps2'],
            'fsm/validation_flight_height': ego['fixed_flight_height'],
            'fsm/replan_start_position_error_m': ego['replan_start_position_error_m'],
            'fsm/handover_prediction_initial_sec':
                ego['handover_prediction_initial_sec'],
            'fsm/handover_position_tolerance_m': ego['handover_position_tolerance_m'],
            'fsm/handover_velocity_tolerance_mps': ego['handover_velocity_tolerance_mps'],
            'fsm/handover_acceleration_tolerance_mps2':
                ego['handover_acceleration_tolerance_mps2'],
            'fsm/ego_recovery/enable': recovery['enable'],
            'fsm/ego_recovery/diagnostic_only': recovery['diagnostic_only'],
            'fsm/ego_recovery/rejoin_search_distance_m':
                recovery['rejoin_search_distance_m'],
            'fsm/ego_recovery/minimum_forward_progress_m':
                recovery['minimum_forward_progress_m'],
            'fsm/ego_recovery/extra_clearance_m': recovery['extra_clearance_m'],
            'fsm/ego_recovery/max_reference_deviation_m':
                recovery['max_reference_deviation_m'],
            'fsm/ego_recovery/max_lateral_offset_m':
                recovery['max_lateral_offset_m'],
            'fsm/ego_recovery/lateral_offset_step_m':
                recovery['lateral_offset_step_m'],
            'fsm/ego_recovery/max_recovery_attempts': recovery['max_recovery_attempts'],
            'fsm/ego_recovery/cooldown_sec': recovery['cooldown_sec'],
            'fsm/ego_recovery/exhausted_backoff_sec':
                recovery['exhausted_backoff_sec'],
            'fsm/active_preplan_lookahead_m': recovery['active_preplan_lookahead_m'],
            'fsm/ego_recovery/reference_path_topic': recovery['reference_path_topic'],
            'dynamic_relaxation/enable': dynamic['启用'],
            'dynamic_relaxation/soft_recovery_extra_clearance_m':
                dynamic['软放宽恢复额外净空米'],
            'dynamic_relaxation/expanded_rejoin_search_distance_m':
                dynamic['扩大后重接搜索距离米'],
            'dynamic_relaxation/expanded_max_lateral_offset_m':
                dynamic['扩大后最大横向搜索距离米'],
            'dynamic_relaxation/expanded_minimum_forward_progress_m':
                dynamic['扩大后最小前进距离米'],
            'ego_diagnostics.enable': ego_diagnostics['enable'],
        },
        'cloud_bridge': {
            'live_topic': ego_map['live_topic'],
            'body_topic': ego_map['body_topic'],
            'pose_topic': ego_map['pose_topic'],
            'live_obstacle_memory_sec': ego_map['live_obstacle_memory_sec'],
            'live_obstacle_voxel_size_m': ego_map['live_obstacle_voxel_size_m'],
            'live_obstacle_min_observations': ego_map['live_obstacle_min_observations'],
            'max_pose_age_sec': ego_map['pose_sync_max_skew_sec'],
            'pose_buffer_duration_sec': ego_map['pose_buffer_duration_sec'],
        },
        'goal_bridge': {**shared, 'frame_id': tuning['shared_safety']['frame_id'],
                        'flight_height': ego['fixed_flight_height'],
                        'release_height': offboard['ego_goal_release_height_m'],
                        'stable_height_tolerance_m':
                            offboard['ego_goal_stable_height_tolerance_m'],
                        'stable_horizontal_speed_mps':
                            offboard['ego_goal_stable_horizontal_speed_mps'],
                        'stable_vertical_speed_mps':
                            offboard['ego_goal_stable_vertical_speed_mps'],
                        'stable_duration_sec': offboard['ego_goal_stable_duration_sec']},
        'trajectory_bridge': {**shared, 'frame_id': tuning['shared_safety']['frame_id'],
                              'obstacle_clearance': required_center_clearance,
                              'constrained_clearance_enable': ego['constrained_clearance_enable'],
                              'constrained_clearance_m': constrained_clearance,
                              'flight_height': ego['fixed_flight_height'],
                              'occupancy_timeout_sec': bridge['occupancy_timeout_sec'],
                              'active_recheck_history_sec':
                                  bridge['active_recheck_history_sec'],
                              'command_timeout_sec': bridge['command_timeout_sec'],
                              'bspline_timeout_sec': bridge['bspline_timeout_sec'],
                              'dynamic_invalid_grace_sec': bridge['dynamic_invalid_grace_sec'],
                              'replan_hold_timeout_sec': bridge['replan_hold_timeout_sec'],
                              'switch_position_tolerance_m':
                                  bridge['switch_position_tolerance_m'],
                              'switch_velocity_tolerance_mps':
                                  bridge['switch_velocity_tolerance_mps'],
                              'switch_acceleration_tolerance_mps2':
                                  bridge['switch_acceleration_tolerance_mps2'],
                              'handover_max_rejections': bridge['handover_max_rejections'],
                              'handover_transaction_timeout_sec':
                                  bridge['handover_transaction_timeout_sec'],
                              'trajectory_sample_spacing': bridge['collision_sample_spacing_m'],
                              'braking_deceleration_mps2':
                                  bridge['braking_deceleration_mps2'],
                              'reaction_time_sec': bridge['reaction_time_sec'],
                              'minimum_reliable_detection_range_m':
                                  ego_map['minimum_reliable_detection_range_m'],
                              'feasibility_tolerance': bridge['dynamic_limit_margin'],
                              'max_yaw_rate_rad_s': bridge['max_yaw_rate_rad_s'],
                              'max_velocity': ego['max_velocity'],
                              'max_acceleration': ego['max_acceleration']},
        'offboard': {
            'shared_bounds/x_min': bounds['x_min'], 'shared_bounds/x_max': bounds['x_max'],
            'shared_bounds/y_min': bounds['y_min'], 'shared_bounds/y_max': bounds['y_max'],
            'shared_bounds/z_min': bounds['z_min'], 'shared_bounds/z_max': bounds['z_max'],
            'cruise_altitude_m': offboard['fixed_flight_height_m'],
            'takeoff_complete_height_m': offboard['takeoff_height_m'],
            'min_command_height_m': bounds['z_min'],
            'max_command_height_m': offboard['max_safe_height_m'],
            'goal_reached_position_tolerance': offboard['goal_tolerance_xy_m'],
            # Two different gates, deliberately not the same value.
            # ego_setpoint_timeout_sec is the strict handover gate: EGO only
            # takes over after takeoff once a setpoint this fresh exists, and
            # failing it merely keeps holding XY.
            'ego_setpoint_timeout_sec': bridge['bspline_timeout_sec'],
            # trajectory_timeout is the in-flight continuity gate, and exceeding
            # it latches PLANNER_FAILURE_HOLD for the rest of the flight. It must
            # therefore sit above EGO's normal replan period, which the node's own
            # 0.50 default did not (measured median 0.516s).
            'trajectory_timeout': offboard['ego_setpoint_timeout_sec'],
            'braking_max_acc': bridge['braking_deceleration_mps2'],
            'setpoint_rate_hz': offboard['setpoint_rate_hz'],
            'trusted_position_max_speed_mps': offboard['trusted_position_max_speed_mps'],
            'trusted_position_jump_allowance_m': offboard['trusted_position_jump_allowance_m'],
            'ego_setpoint_max_lead_m': offboard['ego_setpoint_max_lead_m'],
            'ego_yaw_rate_limit_rad_s': offboard['ego_yaw_rate_limit_rad_s'],
            'initial_position_stabilization_sec': offboard['initial_position_stabilization_sec'],
            'initial_position_max_spread_m': offboard['initial_position_max_spread_m'],
            'initial_position_max_speed_mps': offboard['initial_position_max_speed_mps'],
            'same_goal_retry_on_safety_latch': offboard['same_goal_retry_on_safety_latch'],
        },
    }


def summary(tuning, path, effective_ego_map_source=None):
    bounds = tuning['shared_safety']['fault_envelope']
    # Report the source that is actually in effect. When --ego-map-source
    # overrides the YAML for one run, printing only the YAML value would send
    # every later diagnostic reading the wrong way.
    yaml_source = tuning['ego_map_source']
    if effective_ego_map_source is None or effective_ego_map_source == yaml_source:
        map_source_line = f'ego_map_source={yaml_source}'
    else:
        map_source_line = (
            f'ego_map_source={effective_ego_map_source} '
            f'(launch override; tuning yaml says {yaml_source})')
    return [
        '[TUNING_FILE]', f'path={path}', f"schema_version={tuning['schema_version']}",
        '[EGO_MAP_SOURCE]', map_source_line,
        '[EGO_MAP_TUNING]', str(tuning['ego_map']),
        '[FAULT_ENVELOPE]',
        f"x=[{bounds['x_min']},{bounds['x_max']}] y=[{bounds['y_min']},{bounds['y_max']}] "
        f"z=[{bounds['z_min']},{bounds['z_max']}] jump=max_speed*dt+"
        f"{bounds['position_jump_allowance_m']}m grid_padding="
        f"{bounds['planning_grid_padding_m']}m corridor="
        f"{bounds['mission_corridor_max_error_m']}m",
        '[GLOBAL_PLANNER_TUNING]', str(tuning['global_planner']),
        '[EGO_TUNING]', str(tuning['ego_planner']),
        '[BRIDGE_TUNING]', str(tuning['trajectory_bridge']),
        '[OFFBOARD_TUNING]', str(tuning['offboard']),
    ]
