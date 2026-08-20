import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    start_fastlio = LaunchConfiguration("start_fastlio")
    fastlio_config_path = LaunchConfiguration("fastlio_config_path")
    fastlio_config_file = LaunchConfiguration("fastlio_config_file")
    backend_config = LaunchConfiguration("backend_config")
    rviz = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    map_directory = LaunchConfiguration("map_directory")
    package_share = get_package_share_directory("fastlio_global_slam")
    workspace_root = os.environ.get(
        "WS_OFFBOARD_CONTROL_ROOT", str(Path(package_share).resolve().parents[3])
    )

    fastlio_node = Node(
        package="fast_lio",
        executable="fastlio_mapping",
        output="screen",
        condition=IfCondition(start_fastlio),
        parameters=[
            PathJoinSubstitution([fastlio_config_path, fastlio_config_file]),
        ],
    )

    backend_node = Node(
        package="fastlio_global_slam",
        executable="fastlio_global_backend",
        name="fastlio_global_backend",
        output="screen",
        parameters=[backend_config, {
            "map_save_directory": map_directory,
            "map_load_directory": map_directory,
        }],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="fastlio_global_rviz",
        output="screen",
        condition=IfCondition(rviz),
        arguments=["-d", rviz_config],
    )

    return LaunchDescription([
        DeclareLaunchArgument("start_fastlio", default_value="false"),
        DeclareLaunchArgument(
            "fastlio_config_path",
            default_value=PathJoinSubstitution(
                [FindPackageShare("fastlio_global_slam"), "config"]
            ),
        ),
        DeclareLaunchArgument(
            "fastlio_config_file", default_value="fastlio_mid360_global.yaml"
        ),
        DeclareLaunchArgument(
            "backend_config",
            default_value=PathJoinSubstitution([FindPackageShare("fastlio_global_slam"), "config", "backend.yaml"]),
        ),
        DeclareLaunchArgument(
            "map_directory",
            default_value=PathJoinSubstitution([
                workspace_root,
                "maps",
                "fastlio_global_3d",
            ]),
        ),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=PathJoinSubstitution([FindPackageShare("fastlio_global_slam"), "config", "fastlio_global_slam.rviz"]),
        ),
        fastlio_node,
        backend_node,
        rviz_node,
    ])
