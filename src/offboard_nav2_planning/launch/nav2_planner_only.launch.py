from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    rviz = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    autostart = LaunchConfiguration("autostart")
    use_map_server = LaunchConfiguration("use_map_server")
    map_yaml = LaunchConfiguration("map")
    map_frame = LaunchConfiguration("map_frame")
    odom_frame = LaunchConfiguration("odom_frame")
    publish_odom_tf = LaunchConfiguration("publish_odom_tf")
    start_2_5d = LaunchConfiguration("start_2_5d")
    start_relocalized_tf = LaunchConfiguration("start_relocalized_tf")
    log_level = LaunchConfiguration("log_level")

    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[params_file],
        arguments=["--ros-args", "--log-level", log_level],
        remappings=remappings,
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_planner",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[
            {"autostart": autostart},
            {"node_names": ["planner_server"]},
        ],
    )

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        condition=IfCondition(use_map_server),
        parameters=[
            params_file,
            {
                "yaml_filename": map_yaml,
                "frame_id": map_frame,
            },
        ],
        arguments=["--ros-args", "--log-level", log_level],
        remappings=remappings,
    )

    map_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map",
        output="screen",
        condition=IfCondition(use_map_server),
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[
            {"autostart": autostart},
            {"node_names": ["map_server"]},
        ],
    )

    goal_to_path = Node(
        package="offboard_nav2_planning",
        executable="goal_to_path",
        name="nav2_stage1_goal_to_path",
        output="screen",
        parameters=[params_file],
    )

    odometry_tf_publisher = Node(
        package="offboard_nav2_planning",
        executable="odometry_tf_publisher",
        name="nav2_stage1_odometry_tf_publisher",
        output="screen",
        condition=IfCondition(publish_odom_tf),
        parameters=[params_file],
        remappings=remappings,
    )

    relocalized_pose_to_tf = Node(
        package="offboard_nav2_planning",
        executable="relocalized_pose_to_tf",
        name="nav2_relocalized_pose_to_tf",
        output="screen",
        condition=IfCondition(start_relocalized_tf),
        parameters=[
            {
                "map_frame": map_frame,
                "odom_frame": odom_frame,
            }
        ],
        remappings=remappings,
    )

    path_2_5d_lifter = Node(
        package="offboard_nav2_planning",
        executable="path_2_5d_lifter",
        name="nav2_stage2_path_2_5d_lifter",
        output="screen",
        condition=IfCondition(start_2_5d),
        parameters=[params_file],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="nav2_stage1_rviz",
        output="screen",
        condition=IfCondition(rviz),
        arguments=["-d", rviz_config],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("offboard_nav2_planning"),
                        "config",
                        "nav2_planner_pointcloud.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("offboard_nav2_planning"),
                        "rviz",
                        "nav2_stage1_planning.rviz",
                    ]
                ),
            ),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("use_map_server", default_value="false"),
            DeclareLaunchArgument("map", default_value=""),
            DeclareLaunchArgument("map_frame", default_value="camera_init"),
            DeclareLaunchArgument("odom_frame", default_value="camera_init"),
            DeclareLaunchArgument("publish_odom_tf", default_value="false"),
            DeclareLaunchArgument("start_2_5d", default_value="true"),
            DeclareLaunchArgument("start_relocalized_tf", default_value="false"),
            DeclareLaunchArgument("log_level", default_value="info"),
            planner_server,
            lifecycle_manager,
            map_server,
            map_lifecycle_manager,
            goal_to_path,
            odometry_tf_publisher,
            relocalized_pose_to_tf,
            path_2_5d_lifter,
            rviz_node,
        ]
    )
