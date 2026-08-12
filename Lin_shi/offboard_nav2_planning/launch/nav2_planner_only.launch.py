from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
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

    source_dir = "/home/robot/ws_offboard_control/src/offboard_nav2_planning/offboard_nav2_planning"

    goal_to_path = ExecuteProcess(
        cmd=[
            "python3",
            f"{source_dir}/goal_to_path.py",
            "--ros-args",
            "--params-file",
            params_file,
        ],
        output="screen",
    )

    odometry_tf_publisher = ExecuteProcess(
        cmd=[
            "python3",
            f"{source_dir}/odometry_tf_publisher.py",
            "--ros-args",
            "--params-file",
            params_file,
        ],
        output="screen",
        condition=IfCondition(publish_odom_tf),
    )

    relocalized_pose_to_tf = ExecuteProcess(
        cmd=[
            "python3",
            f"{source_dir}/relocalized_pose_to_tf.py",
            "--ros-args",
            "-p",
            ["map_frame:=", map_frame],
            "-p",
            ["odom_frame:=", odom_frame],
        ],
        output="screen",
        condition=IfCondition(start_relocalized_tf),
    )

    path_2_5d_lifter = ExecuteProcess(
        cmd=[
            "python3",
            f"{source_dir}/path_2_5d_lifter.py",
            "--ros-args",
            "--params-file",
            params_file,
        ],
        output="screen",
        condition=IfCondition(start_2_5d),
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
