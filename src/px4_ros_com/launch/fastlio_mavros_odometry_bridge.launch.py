from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    attitude_yaw_offset_rad = LaunchConfiguration("attitude_yaw_offset_rad")
    body_yaw_offset_rad = LaunchConfiguration("body_yaw_offset_rad")
    fastlio_odom_topic = LaunchConfiguration("fastlio_odom_topic")
    guarded_odom_topic = LaunchConfiguration("guarded_odom_topic")

    return LaunchDescription(
        [
            DeclareLaunchArgument("attitude_yaw_offset_rad", default_value="0.0"),
            DeclareLaunchArgument("body_yaw_offset_rad", default_value="0.0"),
            DeclareLaunchArgument("fastlio_odom_topic", default_value="/Odometry"),
            DeclareLaunchArgument("guarded_odom_topic", default_value="/Odometry/guarded"),
            Node(
                package="px4_ros_com",
                executable="fastlio_odometry_guard",
                name="fastlio_odometry_guard",
                output="screen",
                parameters=[
                    {
                        "input_topic": fastlio_odom_topic,
                        "output_topic": guarded_odom_topic,
                        "max_position_jump_m": 0.30,
                        "max_xy_jump_m": 0.20,
                        "max_z_jump_m": 0.10,
                        "max_computed_speed_mps": 1.2,
                        "max_computed_z_speed_mps": 0.7,
                    }
                ],
            ),
            Node(
                package="px4_ros_com",
                executable="fastlio_mavros_odometry_bridge",
                name="fastlio_mavros_odometry_bridge",
                output="screen",
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
        ]
    )
