from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    camera_init_ned_yaw = LaunchConfiguration("camera_init_ned_yaw")

    return LaunchDescription(
        [
            DeclareLaunchArgument("camera_init_ned_yaw", default_value="0.0"),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="odom_to_camera_init",
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
                output="screen",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_link_to_body",
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
                output="screen",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="camera_init_to_camera_init_ned",
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
                output="screen",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="body_to_body_frd",
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
                output="screen",
            ),
        ]
    )
