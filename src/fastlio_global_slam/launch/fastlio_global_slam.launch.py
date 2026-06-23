from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    start_fastlio = LaunchConfiguration("start_fastlio")
    fastlio_config_path = LaunchConfiguration("fastlio_config_path")
    fastlio_config_file = LaunchConfiguration("fastlio_config_file")
    backend_config = LaunchConfiguration("backend_config")
    rviz = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")

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
        parameters=[backend_config],
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
            default_value=TextSubstitution(text="/home/robot/fastlio2_ws/src/FAST_LIO/config"),
        ),
        DeclareLaunchArgument("fastlio_config_file", default_value="mid360.yaml"),
        DeclareLaunchArgument(
            "backend_config",
            default_value=PathJoinSubstitution([FindPackageShare("fastlio_global_slam"), "config", "backend.yaml"]),
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
