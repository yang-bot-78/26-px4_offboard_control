from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    attitude_yaw_offset_rad = LaunchConfiguration("attitude_yaw_offset_rad")
    body_yaw_offset_rad = LaunchConfiguration("body_yaw_offset_rad")

    return LaunchDescription(
        [
            DeclareLaunchArgument("attitude_yaw_offset_rad", default_value="0.0"),
            DeclareLaunchArgument("body_yaw_offset_rad", default_value="0.0"),
            Node(
                package="px4_ros_com",
                executable="fastlio_mavros_odometry_bridge",
                name="fastlio_mavros_odometry_bridge",
                output="screen",
                parameters=[
                    {
                        "input_topic": "/Odometry",
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
