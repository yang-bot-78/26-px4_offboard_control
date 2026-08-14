import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

sys.path.insert(0, os.path.dirname(__file__))
from astar_ego_tuning import TuningError, load_tuning, node_parameter_overlays  # noqa: E402


def launch_setup(context):
    config_file = LaunchConfiguration('config_file')
    tuning_file = LaunchConfiguration('tuning_file')
    try:
        tuning = load_tuning(tuning_file.perform(context))
    except TuningError as error:
        raise RuntimeError(f'SHARED_BOUNDARY_MISMATCH: {error}') from error
    overlay = node_parameter_overlays(tuning)['offboard']
    return [Node(
        package='race_offboard', executable='offboard_waypoint_node',
        name='offboard_waypoint_node', output='screen',
        parameters=[
            config_file,
            overlay,
            {
                'require_strict_local_position_health': LaunchConfiguration(
                    'require_strict_local_position_health'),
                'ev_fault_auto_land': LaunchConfiguration('ev_fault_auto_land'),
                'control_source': LaunchConfiguration('control_source'),
                'require_global_planner_final_goal': LaunchConfiguration(
                    'require_global_planner_final_goal'),
                'manual_handover': LaunchConfiguration('manual_handover'),
                'require_map_local_alignment': LaunchConfiguration(
                    'require_map_local_alignment'),
            },
        ],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('race_offboard'),
                'config',
                'navigation.yaml',
            ]),
        ),
        DeclareLaunchArgument('require_strict_local_position_health', default_value='false'),
        DeclareLaunchArgument('ev_fault_auto_land', default_value='true'),
        DeclareLaunchArgument('control_source', default_value='navigation'),
        DeclareLaunchArgument('require_global_planner_final_goal', default_value='false'),
        DeclareLaunchArgument('manual_handover', default_value='false'),
        DeclareLaunchArgument('require_map_local_alignment', default_value='false'),
        DeclareLaunchArgument(
            'tuning_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('race_bringup'), 'config', 'astar_ego_tuning.yaml',
            ]),
        ),
        OpaqueFunction(function=launch_setup),
    ])
