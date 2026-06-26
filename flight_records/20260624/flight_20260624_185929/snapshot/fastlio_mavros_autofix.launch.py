from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    fcu_url = LaunchConfiguration("fcu_url")
    tgt_system = LaunchConfiguration("tgt_system")
    tgt_component = LaunchConfiguration("tgt_component")
    start_bridge = LaunchConfiguration("start_bridge")
    start_mavros_vision_bridge = LaunchConfiguration("start_mavros_vision_bridge")
    start_px4_ev_bridge = LaunchConfiguration("start_px4_ev_bridge")
    start_odom_guard = LaunchConfiguration("start_odom_guard")
    start_tf = LaunchConfiguration("start_tf")
    fix_timeout_sec = LaunchConfiguration("fix_timeout_sec")
    camera_init_ned_yaw = LaunchConfiguration("camera_init_ned_yaw")
    attitude_yaw_offset_rad = LaunchConfiguration("attitude_yaw_offset_rad")
    body_yaw_offset_rad = LaunchConfiguration("body_yaw_offset_rad")
    vision_yaw_offset_rad = LaunchConfiguration("vision_yaw_offset_rad")
    vision_position_yaw_offset_rad = LaunchConfiguration("vision_position_yaw_offset_rad")
    ev_quality = LaunchConfiguration("ev_quality")
    fastlio_odom_topic = LaunchConfiguration("fastlio_odom_topic")
    guarded_odom_topic = LaunchConfiguration("guarded_odom_topic")

    fastlio_odometry_guard = Node(
        package="px4_ros_com",
        executable="fastlio_odometry_guard",
        name="fastlio_odometry_guard",
        output="screen",
        condition=IfCondition(start_odom_guard),
        parameters=[
            {
                "input_topic": fastlio_odom_topic,
                "output_topic": guarded_odom_topic,
                "max_dt_s": 1.0,
                "max_position_jump_m": 0.30,
                "max_xy_jump_m": 0.20,
                "max_z_jump_m": 0.10,
                "max_computed_speed_mps": 1.2,
                "max_computed_z_speed_mps": 0.7,
            }
        ],
    )

    mavros_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("mavros"), "launch", "node.launch"])
        ),
        launch_arguments={
            "fcu_url": fcu_url,
            "gcs_url": "",
            "tgt_system": tgt_system,
            "tgt_component": tgt_component,
            "pluginlists_yaml": PathJoinSubstitution(
                [FindPackageShare("mavros"), "launch", "px4_pluginlists.yaml"]
            ),
            "config_yaml": PathJoinSubstitution(
                [FindPackageShare("px4_ros_com"), "config", "mavros_px4_identity_override.yaml"]
            ),
        }.items(),
    )

    odometry_frame_fixer = Node(
        package="px4_ros_com",
        executable="fix_mavros_odometry_frames.py",
        name="fix_mavros_odometry_frames",
        output="screen",
        condition=IfCondition(start_bridge),
        parameters=[
            {
                "target_node": "/mavros/odometry",
                "map_id_des": "map",
                "odom_parent_id_des": "odom",
                "odom_child_id_des": "base_link",
                "timeout_sec": fix_timeout_sec,
            }
        ],
    )

    fastlio_mavros_vision_bridge = Node(
        package="px4_ros_com",
        executable="fastlio_mavros_vision_bridge",
        name="fastlio_mavros_vision_bridge",
        output="screen",
        condition=IfCondition(start_mavros_vision_bridge),
        parameters=[
            {
                "input_topic": guarded_odom_topic,
                "pose_topic": "/mavros/vision_pose/pose_cov",
                "speed_topic": "/mavros/vision_speed/speed_twist_cov",
                "force_pose_frame_id": "odom",
                "force_twist_frame_id": "base_link",
                "restamp_message": True,
                "publish_speed": False,
                "yaw_offset_rad": vision_yaw_offset_rad,
                "position_yaw_offset_rad": vision_position_yaw_offset_rad,
            }
        ],
    )

    fastlio_px4_ev_bridge = Node(
        package="px4_ros_com",
        executable="fastlio_vehicle_visual_odometry",
        name="fastlio_vehicle_visual_odometry",
        output="screen",
        condition=IfCondition(start_px4_ev_bridge),
        parameters=[
            {
                "input_topic": guarded_odom_topic,
                "output_topic": "/fmu/in/vehicle_visual_odometry",
                "quality": ev_quality,
            }
        ],
    )

    fastlio_bridge = Node(
        package="px4_ros_com",
        executable="fastlio_mavros_odometry_bridge",
        name="fastlio_mavros_odometry_bridge",
        output="screen",
        condition=IfCondition(start_bridge),
        parameters=[
            {
                "input_topic": guarded_odom_topic,
                "output_topic": "/mavros/odometry/out",
                "frame_id": "camera_init",
                "child_frame_id": "body",
                "force_frame_ids": False,
                "attitude_yaw_offset_rad": attitude_yaw_offset_rad,
                "body_yaw_offset_rad": body_yaw_offset_rad,
            }
        ],
    )

    tf_odom_to_camera_init = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="odom_to_camera_init",
        output="screen",
        condition=IfCondition(start_tf),
        arguments=[
            "--x", "0",
            "--y", "0",
            "--z", "0",
            "--roll", "0",
            "--pitch", "0",
            "--yaw", "0",
            "--frame-id", "odom",
            "--child-frame-id", "camera_init",
        ],
    )

    tf_base_link_to_body = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_body",
        output="screen",
        condition=IfCondition(start_tf),
        arguments=[
            "--x", "0",
            "--y", "0",
            "--z", "0",
            "--roll", "0",
            "--pitch", "0",
            "--yaw", "0",
            "--frame-id", "base_link",
            "--child-frame-id", "body",
        ],
    )

    # MAVROS 2.14 derives helper frames like "<frame>_ned" and "<frame>_frd"
    # for odometry conversion. Publish them explicitly for camera_init/body.
    # Default to 0 rad yaw here; adjust only after verifying the full TF and
    # odometry frame chain end-to-end.
    tf_camera_init_to_camera_init_ned = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_init_to_camera_init_ned",
        output="screen",
        condition=IfCondition(start_tf),
        arguments=[
            "--x", "0",
            "--y", "0",
            "--z", "0",
            "--roll", "3.141592653589793",
            "--pitch", "0",
            "--yaw", camera_init_ned_yaw,
            "--frame-id", "camera_init",
            "--child-frame-id", "camera_init_ned",
        ],
    )

    tf_body_to_body_frd = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="body_to_body_frd",
        output="screen",
        condition=IfCondition(start_tf),
        arguments=[
            "--x", "0",
            "--y", "0",
            "--z", "0",
            "--roll", "3.141592653589793",
            "--pitch", "0",
            "--yaw", "0",
            "--frame-id", "body",
            "--child-frame-id", "body_frd",
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("fcu_url", default_value="serial:///dev/ttyUSB0:921600?ids=255,190"),
            DeclareLaunchArgument("tgt_system", default_value="1"),
            DeclareLaunchArgument("tgt_component", default_value="1"),
            DeclareLaunchArgument("start_bridge", default_value="false"),
            DeclareLaunchArgument("start_mavros_vision_bridge", default_value="true"),
            DeclareLaunchArgument("start_px4_ev_bridge", default_value="false"),
            DeclareLaunchArgument("start_odom_guard", default_value="true"),
            DeclareLaunchArgument("start_tf", default_value="true"),
            DeclareLaunchArgument("fix_timeout_sec", default_value="15.0"),
            DeclareLaunchArgument("camera_init_ned_yaw", default_value="0.0"),
            DeclareLaunchArgument("attitude_yaw_offset_rad", default_value="0.0"),
            DeclareLaunchArgument("body_yaw_offset_rad", default_value="0.0"),
            DeclareLaunchArgument("vision_yaw_offset_rad", default_value="0.0"),
            DeclareLaunchArgument("vision_position_yaw_offset_rad", default_value="1.57079632679"),
            DeclareLaunchArgument("ev_quality", default_value="100"),
            DeclareLaunchArgument("fastlio_odom_topic", default_value="/Odometry"),
            DeclareLaunchArgument("guarded_odom_topic", default_value="/Odometry/guarded"),
            mavros_launch,
            fastlio_odometry_guard,
            odometry_frame_fixer,
            fastlio_mavros_vision_bridge,
            fastlio_px4_ev_bridge,
            fastlio_bridge,
            tf_odom_to_camera_init,
            tf_base_link_to_body,
            tf_camera_init_to_camera_init_ned,
            tf_body_to_body_frd,
        ]
    )
