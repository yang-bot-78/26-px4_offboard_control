import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

sys.path.insert(0, os.path.join(get_package_share_directory('race_bringup'), 'launch'))
from astar_ego_tuning import TuningError, load_tuning, node_parameter_overlays


def launch_setup(context):
    try:
        overlay = node_parameter_overlays(
            load_tuning(LaunchConfiguration('tuning_file').perform(context)))
    except TuningError as error:
        raise RuntimeError(f'SHARED_BOUNDARY_MISMATCH: {error}') from error

    control_enabled = LaunchConfiguration('control_enabled')
    shadow_mode = LaunchConfiguration('shadow_mode')
    map_source = LaunchConfiguration('map_source')
    flight_mode = LaunchConfiguration('flight_mode')
    goal_input_topic = LaunchConfiguration('goal_input_topic')
    require_strict_health = LaunchConfiguration('require_strict_local_position_health')
    # Every node that timestamps or ages sensor data must read the same clock as
    # the source of that data. Gazebo, the mid360 plugin, FAST-LIO and
    # frame_transform_node all run on /clock (race_mapping.launch.py sets
    # use_sim_time explicitly). Leaving these nodes on the wall clock put a
    # ~1.79e9 s offset between the cloud stamps EGO consumes and the TF/pose
    # times it compares them against: the TF lookup in ego_cloud_bridge.cpp:133
    # keys on cloud.header.stamp, and the 1.0 s freshness gates (EGO
    # fsm/validation_map_timeout_sec, trajectory_bridge occupancy_timeout_sec)
    # compare stamps across that gap. Measured after the fix: the
    # cloud_registered_body -> /race/ego/cloud stamp gap fell from 1.79e9 s
    # to 0.078 s.
    #
    # This is NOT the cause of the ~0.5 m live-cloud/map misalignment seen
    # earlier; that was the mid360 plugin dropping minDist_ from each point's
    # radius (fixed in mid360_points_plugin.cpp) and survived this change
    # unchanged until that fix landed. The clock setting stands on its own
    # merits above.
    clock = {'use_sim_time': LaunchConfiguration('use_sim_time')}

    goal_bridge = Node(
        package='race_ego_bridge', executable='race_ego_goal_bridge',
        name='race_ego_goal_bridge', output='screen',
        parameters=[clock, {
            'flight_mode': flight_mode,
            'input_topic': goal_input_topic,
            'retry_goal_on_ego_failure': False,
            'ego_failure_retry_period_sec': 0.75,
            'duplicate_goal_position_tolerance_m': 0.03,
            'duplicate_goal_time_window_sec': 1.0,
        }, overlay['goal_bridge']],
    )
    ego_planner = Node(
        package='ego_planner', executable='ego_planner_node',
        name='race_ego_planner', output='screen',
        remappings=[
            ('odom_world', '/race/ego/odom'), ('grid_map/odom', '/race/ego/odom'),
            ('grid_map/cloud', '/race/ego/cloud'), ('/move_base_simple/goal', '/race/ego/goal_input'),
            ('planning/bspline', '/race/ego/bspline'), ('planning/data_display', '/race/ego/data_display'),
            ('planning/broadcast_bspline_from_planner', '/race/ego/broadcast_bspline'),
            ('planning/broadcast_bspline_to_planner', '/race/ego/broadcast_bspline'),
            ('grid_map/occupancy', '/race/ego/occupancy'),
            ('grid_map/occupancy_inflate', '/race/ego/occupancy_inflate'),
            ('goal_point', '/race/ego/goal_marker'), ('global_list', '/race/ego/global_path'),
            ('init_list', '/race/ego/initial_path'), ('optimal_list', '/race/ego/optimal_path'),
            ('a_star_list', '/race/ego/a_star_path'),
        ],
        parameters=[clock, {
            'fsm/flight_type': 1, 'fsm/thresh_replan_time': 0.5,
            'fsm/thresh_no_replan_meter': 0.35, 'fsm/planning_horizen_time': 3.0,
            'fsm/emergency_time': 0.8, 'fsm/realworld_experiment': False,
            'fsm/fail_safe': True,
            'fsm/shadow_mode': ParameterValue(shadow_mode, value_type=bool),
            'fsm/validation_flat_mode': ParameterValue(PythonExpression([
                "'true' if '", flight_mode, "' == 'flat' else 'false'"]), value_type=bool),
            'fsm/validation_sample_spacing': 0.04, 'fsm/validation_map_timeout_sec': 1.0,
            'fsm/replan_start_position_error_m': 0.10, 'fsm/replan_candidate_trials': 3,
            'fsm/validation_failure_retry_limit': 3, 'fsm/waypoint_num': 0,
            'grid_map/map_size_y': 14.0,
            'grid_map/map_size_z': 2.5, 'grid_map/local_map_margin': 5,
            'grid_map/ground_height': 0.0, 'grid_map/cx': 321.0, 'grid_map/cy': 243.0,
            'grid_map/fx': 387.0, 'grid_map/fy': 387.0, 'grid_map/use_depth_filter': False,
            'grid_map/depth_filter_tolerance': 0.15, 'grid_map/depth_filter_maxdist': 5.0,
            'grid_map/depth_filter_mindist': 0.2, 'grid_map/depth_filter_margin': 2,
            'grid_map/k_depth_scaling_factor': 1000.0, 'grid_map/skip_pixel': 2,
            'grid_map/p_hit': 0.65, 'grid_map/p_miss': 0.35, 'grid_map/p_min': 0.12,
            'grid_map/p_max': 0.90, 'grid_map/p_occ': 0.80, 'grid_map/min_ray_length': 0.1,
            'grid_map/max_ray_length': 5.0, 'grid_map/local_map_margin': 5,
            'grid_map/virtual_ceil_yp': 7.0, 'grid_map/virtual_ceil_yn': -7.0,
            'grid_map/show_occ_time': False, 'grid_map/pose_type': 2, 'grid_map/frame_id': 'map',
            'grid_map/odom_depth_timeout': 0.5, 'manager/max_jerk': 2.0,
            'manager/feasibility_tolerance': 0.05, 'manager/use_distinctive_trajs': True,
            'manager/drone_id': -1, 'optimization/lambda_smooth': 1.0,
            'optimization/lambda_collision': 1.2, 'optimization/lambda_feasibility': 0.4,
            'optimization/lambda_fitness': 1.0, 'optimization/swarm_clearance': 0.5,
            'optimization/order': 3, 'prediction/obj_num': 0, 'prediction/lambda': 1.0,
            'prediction/predict_rate': 1.0,
        }, overlay['ego']],
    )
    trajectory_bridge = Node(
        package='race_ego_bridge', executable='race_ego_trajectory_bridge',
        name='race_ego_trajectory_bridge', output='screen',
        parameters=[clock, {
            'publish_setpoint': True, 'control_enabled': control_enabled, 'map_source': map_source,
            'require_strict_local_position_health': require_strict_health,
            # Use the same raw input cloud indexed by EGO's continuous final
            # recheck. /occupancy is regenerated occupied voxel centres.
            'occupancy_topic': '/race/ego/cloud', 'occupancy_is_inflated': False,
            'flight_mode': flight_mode, 'dynamic_invalid_grace_sec': 0.75,
            'obstacle_clearance': overlay['ego']['grid_map/obstacles_inflation'],
        }, overlay['trajectory_bridge']],
    )
    return [
        Node(package='race_ego_bridge', executable='race_ego_odom_bridge',
             name='race_ego_odom_bridge', output='screen',
             parameters=[clock, {'require_strict_health': require_strict_health}]),
        Node(package='race_ego_bridge', executable='race_ego_cloud_bridge',
             name='race_ego_cloud_bridge', output='screen',
             parameters=[clock, {'map_source': map_source}, overlay['cloud_bridge']]),
        goal_bridge,
        ego_planner,
        Node(package='ego_planner', executable='traj_server', name='race_ego_traj_server', output='screen',
             remappings=[('planning/bspline', '/race/ego/validated_bspline'),
                         ('/position_cmd', '/race/ego/position_command')],
             parameters=[clock,
                         {'traj_server/time_forward': 0.8, 'traj_server/frame_id': 'map'}]),
        trajectory_bridge,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('control_enabled', default_value='false'),
        DeclareLaunchArgument('shadow_mode', default_value='false'),
        DeclareLaunchArgument('map_source', default_value='static'),
        # Retained only so existing launcher invocations remain accepted. The tuning YAML
        # is applied last and is the authoritative user-editable source.
        DeclareLaunchArgument('max_vel', default_value='0.60'),
        DeclareLaunchArgument('max_acc', default_value='0.80'),
        DeclareLaunchArgument('inflation', default_value='0.30'),
        DeclareLaunchArgument('resolution', default_value='0.15'),
        DeclareLaunchArgument('local_map_x', default_value='6.0'),
        DeclareLaunchArgument('local_map_y', default_value='6.0'),
        DeclareLaunchArgument('local_map_z', default_value='1.25'),
        DeclareLaunchArgument('flight_mode', default_value='flat'),
        DeclareLaunchArgument('flight_height', default_value='0.78'),
        DeclareLaunchArgument('max_height', default_value='0.90'),
        DeclareLaunchArgument('goal_input_topic', default_value='/goal_pose'),
        DeclareLaunchArgument('require_strict_local_position_health', default_value='false'),
        # Defaults to true: this stack only ever runs against Gazebo SITL, whose
        # /clock every upstream sensor node already follows.
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'tuning_file', default_value=PathJoinSubstitution([
                FindPackageShare('race_bringup'), 'config', 'astar_ego_tuning.yaml'])),
        OpaqueFunction(function=launch_setup),
    ])
