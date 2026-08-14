import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo,
    OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitution import Substitution
from launch.substitutions import (
    IfElseSubstitution, LaunchConfiguration, PathJoinSubstitution, PythonExpression)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

sys.path.insert(0, os.path.dirname(__file__))
from astar_ego_tuning import TuningError, load_tuning, summary  # noqa: E402


class EffectivePlannerBackend(Substitution):
    """Resolve the requested backend against the canonical EGO enable switch."""

    _EGO_BACKENDS = ('astar_ego', 'ego', 'ego-shadow')

    def __init__(self, tuning_file, requested_backend):
        self._tuning_file = tuning_file
        self._requested_backend = requested_backend

    def describe(self):
        return 'EffectivePlannerBackend(tuning_file, planner_backend)'

    def perform(self, context):
        requested = self._requested_backend.perform(context)
        if requested not in self._EGO_BACKENDS:
            return requested
        path = self._tuning_file.perform(context)
        try:
            ego_enabled = load_tuning(path)['ego_planner']['enable']
        except (TuningError, KeyError) as error:
            raise RuntimeError(f'SHARED_BOUNDARY_MISMATCH: {error}') from error
        return requested if ego_enabled else 'super'


def print_tuning_summary(context):
    path = LaunchConfiguration('tuning_file').perform(context)
    try:
        tuning = load_tuning(path)
    except TuningError as error:
        raise RuntimeError(f'SHARED_BOUNDARY_MISMATCH: {error}') from error
    # Resolved value, so the banner matches what the nodes actually run with
    # even when ego_map_source:= overrides the yaml for this one run.
    effective = LaunchConfiguration('ego_map_source').perform(context)
    requested_backend = LaunchConfiguration('planner_backend').perform(context)
    effective_backend = EffectivePlannerBackend(
        LaunchConfiguration('tuning_file'),
        LaunchConfiguration('planner_backend')).perform(context)
    backend_line = (
        f'[PLANNER_BACKEND] requested={requested_backend} '
        f'effective={effective_backend} ego_enabled={tuning["ego_planner"]["enable"]}')
    tuning_lines = [LogInfo(msg=line) for line in summary(tuning, path, effective)]
    return [LogInfo(msg=backend_line)] + tuning_lines


class TuningEgoMapSource(Substitution):
    """Resolve ego_map_source from the one canonical tuning YAML."""

    # Used as the declared default so the YAML is the single source of truth,
    # while an explicit --ego-map-source still overrides it for one run.

    def __init__(self, tuning_file):
        self._tuning_file = tuning_file

    def describe(self):
        return 'TuningEgoMapSource(tuning_file)'

    def perform(self, context):
        path = self._tuning_file.perform(context)
        try:
            return load_tuning(path)['ego_map_source']
        except TuningError as error:
            raise RuntimeError(f'SHARED_BOUNDARY_MISMATCH: {error}') from error


def generate_launch_description():
    bringup_dir = get_package_share_directory('race_bringup')
    mapping_dir = get_package_share_directory('race_mapping')
    planner_dir = get_package_share_directory('race_super_planner_ros2')

    rviz = LaunchConfiguration('rviz')
    enable_output = LaunchConfiguration('enable_output')
    require_strict_health = LaunchConfiguration('require_strict_local_position_health')
    ev_fault_auto_land = LaunchConfiguration('ev_fault_auto_land')
    map_file = LaunchConfiguration('map_file')
    map_auto_load = LaunchConfiguration('map_auto_load')
    planner_config = LaunchConfiguration('planner_config')
    offboard_config = LaunchConfiguration('offboard_config')
    requested_planner_backend = LaunchConfiguration('planner_backend')
    ego_map_source = LaunchConfiguration('ego_map_source')
    ego_max_vel = LaunchConfiguration('ego_max_vel')
    ego_max_acc = LaunchConfiguration('ego_max_acc')
    ego_inflation = LaunchConfiguration('ego_inflation')
    ego_resolution = LaunchConfiguration('ego_resolution')
    ego_local_map_x = LaunchConfiguration('ego_local_map_x')
    ego_local_map_y = LaunchConfiguration('ego_local_map_y')
    ego_local_map_z = LaunchConfiguration('ego_local_map_z')
    ego_flight_mode = LaunchConfiguration('ego_flight_mode')
    ego_flight_height = LaunchConfiguration('ego_flight_height')
    ego_max_height = LaunchConfiguration('ego_max_height')
    tuning_file = LaunchConfiguration('tuning_file')
    planner_backend = EffectivePlannerBackend(tuning_file, requested_planner_backend)
    mission_enabled = LaunchConfiguration('mission_enabled')
    recognition_enabled = LaunchConfiguration('recognition_enabled')
    mission_file = LaunchConfiguration('mission_file')
    mission_rviz_config = PathJoinSubstitution([
        FindPackageShare('race_bringup'), 'rviz', 'race_mission_click_planner.rviz',
    ])
    standard_rviz_config = PathJoinSubstitution([
        FindPackageShare('race_bringup'), 'rviz', 'race_click_planner.rviz',
    ])
    # One clock for the whole stack. race_mapping already forces use_sim_time on
    # Gazebo, the mid360 plugin, FAST-LIO and frame_transform_node; the planning
    # and control side had no setting at all and silently ran on the wall clock.
    use_sim_time = LaunchConfiguration('use_sim_time')
    body_to_sensor_x_m = LaunchConfiguration('body_to_sensor_x_m')
    body_to_sensor_y_m = LaunchConfiguration('body_to_sensor_y_m')
    body_to_sensor_z_m = LaunchConfiguration('body_to_sensor_z_m')
    body_to_fastlio_yaw_rad = LaunchConfiguration('body_to_fastlio_yaw_rad')
    world_yaw_alignment_rad = LaunchConfiguration('world_yaw_alignment_rad')
    publish_fastlio_bridge = LaunchConfiguration('publish_fastlio_bridge')
    publish_camera_init_tf = LaunchConfiguration('publish_camera_init_tf')
    map_frame_id = LaunchConfiguration('map_frame_id')
    fast_lio_odom_topic = LaunchConfiguration('fast_lio_odom_topic')

    offboard_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'launch', 'race_sim.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'config_file': offboard_config,
            'require_strict_local_position_health': require_strict_health,
            'ev_fault_auto_land': ev_fault_auto_land,
            'control_source': PythonExpression([
                "'ego' if '", planner_backend,
                "' in ['ego', 'astar_ego'] else 'navigation'"
            ]),
            'require_global_planner_final_goal': PythonExpression([
                "'true' if '", planner_backend, "' == 'astar_ego' else 'false'"
            ]),
            'manual_handover': LaunchConfiguration('manual_handover'),
            'require_map_local_alignment': LaunchConfiguration(
                'require_map_local_alignment'),
            'tuning_file': tuning_file,
        }.items(),
    )

    mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mapping_dir, 'launch', 'race_mapping.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'rviz': rviz,
            # A mission click must not go to /goal_pose before all four points are collected.
            'rviz_config_file': IfElseSubstitution(
                mission_enabled, mission_rviz_config, standard_rviz_config),
            'map_file': map_file,
            'map_auto_load': map_auto_load,
            'fast_lio_cloud_registered_topic': '/cloud_registered',
            'fast_lio_cloud_registered_body_topic': '/cloud_registered_body',
            'body_to_sensor_x_m': body_to_sensor_x_m,
            'body_to_sensor_y_m': body_to_sensor_y_m,
            'body_to_sensor_z_m': body_to_sensor_z_m,
            'body_to_fastlio_yaw_rad': body_to_fastlio_yaw_rad,
            'world_yaw_alignment_rad': world_yaw_alignment_rad,
            'publish_fastlio_bridge': publish_fastlio_bridge,
            'publish_camera_init_tf': publish_camera_init_tf,
            'map_frame_id': map_frame_id,
            'fast_lio_odom_topic': fast_lio_odom_topic,
        }.items(),
    )

    planner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(planner_dir, 'launch', 'super_planner_ros2.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'config_file': planner_config,
            'enable_output': enable_output,
            'global_only_mode': PythonExpression([
                "'true' if '", planner_backend, "' == 'astar_ego' else 'false'"
            ]),
            'publish_global_path': PythonExpression([
                "'true' if '", planner_backend,
                "' in ['ego-shadow', 'astar_ego'] else 'false'"
            ]),
            'publish_ego_local_goal': PythonExpression([
                "'true' if '", planner_backend,
                "' in ['ego-shadow', 'astar_ego'] else 'false'"
            ]),
            # /race/odom is the single map-frame pose after applying the shared
            # world yaw. Planner goals and the loaded map use this same frame.
            'odom_topic': '/race/odom',
            'fallback_odom_topic': '/race/odom',
            'odom_input_frame': 'map',
            'tuning_file': tuning_file,
        }.items(),
        condition=IfCondition(PythonExpression([
            "'", planner_backend, "' in ['super', 'ego-shadow', 'astar_ego']"
        ])),
    )

    ego_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('race_ego_bridge'), 'launch', 'ego_race.launch.py'
        ])),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'control_enabled': PythonExpression([
                "'true' if '", planner_backend,
                "' in ['ego', 'astar_ego'] and '", enable_output,
                "' == 'true' else 'false'"
            ]),
            'shadow_mode': PythonExpression([
                "'true' if '", planner_backend, "' == 'ego-shadow' else 'false'"
            ]),
            'goal_input_topic': PythonExpression([
                "'/race/ego/local_goal' if '", planner_backend,
                "' in ['ego-shadow', 'astar_ego'] else '/goal_pose'"
            ]),
            'map_source': ego_map_source,
            'max_vel': ego_max_vel,
            'max_acc': ego_max_acc,
            'inflation': ego_inflation,
            'resolution': ego_resolution,
            'local_map_x': ego_local_map_x,
            'local_map_y': ego_local_map_y,
            'local_map_z': ego_local_map_z,
            'flight_mode': ego_flight_mode,
            'flight_height': ego_flight_height,
            'max_height': ego_max_height,
            'require_strict_local_position_health': require_strict_health,
            'tuning_file': tuning_file,
        }.items(),
        condition=IfCondition(PythonExpression(["'", planner_backend, "' != 'super'"])),
    )

    mission_sequencer = Node(
        package='race_offboard',
        executable='mission_sequencer_node.py',
        name='mission_sequencer_node',
        output='screen',
        parameters=[{
            'mission_file': mission_file,
            'use_sim_time': use_sim_time,
        }],
        condition=IfCondition(mission_enabled),
    )

    # Target recognition. Lives in bs/, outside the colcon tree, because it is
    # a standalone Python program with heavy vision dependencies; run it by
    # absolute path rather than packaging it. It idles until the mission
    # sequencer turns recognition on for zone B or D.
    recognition_node = ExecuteProcess(
        cmd=[
            'python3',
            os.path.join(
                os.path.expanduser('~'), 'rong_ws', 'ws_offboard_control',
                'bs', 'code', 'bs_recognition_node.py'),
        ],
        output='screen',
        condition=IfCondition(recognition_enabled),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[
            '-d', IfElseSubstitution(
                mission_enabled, mission_rviz_config, standard_rviz_config),
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        # Target detection (bs/). Off by default: it needs the RealSense stack
        # and the YOLO weights present, and a missing dependency should not be
        # discovered during a flight.
        DeclareLaunchArgument('recognition_enabled', default_value='false'),
        DeclareLaunchArgument('enable_output', default_value='false'),
        DeclareLaunchArgument('require_strict_local_position_health', default_value='false'),
        DeclareLaunchArgument(
            'manual_handover', default_value='false',
            description='Pilot controls arming, takeoff, and OFFBOARD handover.'),
        DeclareLaunchArgument('require_map_local_alignment', default_value='false'),
        # astar_ego: Super A* plans globally and hands EGO a rolling local
        # goal; EGO does local avoidance and drives the vehicle. This is the
        # competition configuration. ego-shadow (the old default) runs EGO in
        # parallel without letting it control, which is a diagnostic mode.
        DeclareLaunchArgument('planner_backend', default_value='astar_ego'),
        # The whole stack runs against Gazebo SITL, so /clock is authoritative.
        # Set this to false only for a replay/bag run that has no /clock.
        # Real hardware: wall clock, not a Gazebo /clock.
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('body_to_sensor_x_m', default_value='0.0'),
        DeclareLaunchArgument('body_to_sensor_y_m', default_value='0.0'),
        DeclareLaunchArgument('body_to_sensor_z_m', default_value='0.08'),
        DeclareLaunchArgument(
            'body_to_fastlio_yaw_rad', default_value='0.0'),
        DeclareLaunchArgument(
            'world_yaw_alignment_rad', default_value='0.0',
            description='FAST-LIO camera_init world to project map ENU yaw alignment'),
        DeclareLaunchArgument('publish_fastlio_bridge', default_value='true'),
        DeclareLaunchArgument('publish_camera_init_tf', default_value='true'),
        DeclareLaunchArgument('map_frame_id', default_value='map'),
        DeclareLaunchArgument('fast_lio_odom_topic', default_value='/Odometry'),
        # The A->B->C->D race sequence. On by default: without it the stack
        # flies to a single RViz goal and never runs the course.
        DeclareLaunchArgument('mission_enabled', default_value='true'),
        DeclareLaunchArgument(
            'map_auto_load', default_value='true',
            description='Load the PCD at startup; set false for no-map validation.'),
        DeclareLaunchArgument(
            'mission_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('race_offboard'), 'config', 'mission.yaml',
            ]),
        ),
        DeclareLaunchArgument(
            'tuning_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('race_bringup'), 'config', 'astar_ego_tuning.yaml',
            ]),
        ),
        # Default comes from the one canonical tuning YAML (ego_map_source at
        # its top). An explicit ego_map_source:= still overrides for one run.
        DeclareLaunchArgument(
            'ego_map_source', default_value=TuningEgoMapSource(tuning_file)),
        DeclareLaunchArgument('ego_max_vel', default_value='0.60'),
        DeclareLaunchArgument('ego_max_acc', default_value='0.80'),
        DeclareLaunchArgument('ego_inflation', default_value='0.30'),
        DeclareLaunchArgument('ego_resolution', default_value='0.15'),
        DeclareLaunchArgument('ego_local_map_x', default_value='6.0'),
        DeclareLaunchArgument('ego_local_map_y', default_value='6.0'),
        DeclareLaunchArgument('ego_local_map_z', default_value='1.25'),
        DeclareLaunchArgument('ego_flight_mode', default_value='flat'),
        DeclareLaunchArgument('ego_flight_height', default_value='0.78'),
        DeclareLaunchArgument('ego_max_height', default_value='0.90'),
        DeclareLaunchArgument(
            'map_file',
            default_value=os.path.join(
                os.path.expanduser('~'),
                'rong_ws',
                'ws_offboard_control',
                'maps',
                'fastlio_global_3d',
                'GlobalMap.pcd',
            ),
        ),
        DeclareLaunchArgument(
            'planner_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('race_super_planner_ros2'),
                'config',
                'navigation.yaml',
            ]),
        ),
        DeclareLaunchArgument(
            'offboard_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('race_offboard'),
                'config',
                'navigation.yaml',
            ]),
        ),
        DeclareLaunchArgument('ev_fault_auto_land', default_value='true'),
        OpaqueFunction(function=print_tuning_summary),
        offboard_launch,
        mapping_launch,
        planner_launch,
        ego_launch,
        mission_sequencer,
        recognition_node,
        rviz_node,
    ])
