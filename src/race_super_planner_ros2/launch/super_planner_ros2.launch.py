import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

sys.path.insert(0, os.path.join(get_package_share_directory('race_bringup'), 'launch'))
from astar_ego_tuning import TuningError, load_tuning, node_parameter_overlays  # noqa: E402


def launch_setup(context):
    tuning_file = LaunchConfiguration('tuning_file').perform(context)
    try:
        overlay = node_parameter_overlays(load_tuning(tuning_file))['super']
    except TuningError as error:
        raise RuntimeError(f'SHARED_BOUNDARY_MISMATCH: {error}') from error
    return [Node(
        package='race_super_planner_ros2',
        executable='super_planner_ros2_node',
        name='super_planner_ros2_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            overlay,
            {
                # use_sim_time must NOT be its own leading dict entry. launch_ros
                # writes each dict to a temp params file keyed on '/**', and rcl
                # gives a named-node section ('super_planner_ros2_node:' in
                # config_file) priority over every '/**' section that follows it
                # once a '/**' file has already been seen. Measured directly on
                # the node binary: `--params-file navigation.yaml --params-file
                # <flags>` yields global_only=true local_goal=/race/ego/local_goal,
                # while prefixing a '/**'-only file makes the same flags come back
                # false/disabled. Keeping the clock in this last dict keeps a
                # single '/**' file after config_file, so the overrides win.
                # Same clock as Gazebo and the mapping chain; see the note in
                # race_ego_bridge/launch/ego_race.launch.py.
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'enable_output': LaunchConfiguration('enable_output'),
                'global_only_mode': LaunchConfiguration('global_only_mode'),
                'publish_global_path': LaunchConfiguration('publish_global_path'),
                'publish_ego_local_goal': LaunchConfiguration('publish_ego_local_goal'),
                'odom_topic': LaunchConfiguration('odom_topic'),
                'fallback_odom_topic': LaunchConfiguration('fallback_odom_topic'),
                'odom_input_frame': LaunchConfiguration('odom_input_frame'),
                'cloud_topic': LaunchConfiguration('cloud_topic'),
                'fallback_cloud_topic': LaunchConfiguration('fallback_cloud_topic'),
                'require_mavros_connected': LaunchConfiguration('require_mavros_connected'),
            },
        ],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('race_super_planner_ros2'),
                'config',
                'navigation.yaml',
            ]),
        ),
        DeclareLaunchArgument('enable_output', default_value='false'),
        DeclareLaunchArgument('global_only_mode', default_value='false'),
        DeclareLaunchArgument('publish_global_path', default_value='false'),
        DeclareLaunchArgument('publish_ego_local_goal', default_value='false'),
        DeclareLaunchArgument('odom_topic', default_value='/race/odom'),
        DeclareLaunchArgument(
            'fallback_odom_topic', default_value='/race/odom'
        ),
        DeclareLaunchArgument('odom_input_frame', default_value='map'),
        DeclareLaunchArgument('cloud_topic', default_value='/saved_map'),
        DeclareLaunchArgument('fallback_cloud_topic', default_value='/saved_map'),
        DeclareLaunchArgument('require_mavros_connected', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'tuning_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('race_bringup'), 'config', 'astar_ego_tuning.yaml',
            ]),
        ),
        OpaqueFunction(function=launch_setup),
    ])
