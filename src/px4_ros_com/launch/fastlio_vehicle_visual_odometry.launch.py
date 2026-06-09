from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    quality = LaunchConfiguration("quality")
    return LaunchDescription(
        [
            DeclareLaunchArgument("quality", default_value="100"),
            Node(
                package="px4_ros_com",
                executable="fastlio_vehicle_visual_odometry",
                name="fastlio_vehicle_visual_odometry",
                output="screen",
                parameters=[
                    {
                        "input_topic": "/Odometry",
                        "output_topic": "/fmu/in/vehicle_visual_odometry",
                        "quality": quality,
                    }
                ],
            )
        ]
    )
