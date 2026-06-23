from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    quality = LaunchConfiguration("quality")
    fastlio_odom_topic = LaunchConfiguration("fastlio_odom_topic")
    guarded_odom_topic = LaunchConfiguration("guarded_odom_topic")
    return LaunchDescription(
        [
            DeclareLaunchArgument("quality", default_value="100"),
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
                executable="fastlio_vehicle_visual_odometry",
                name="fastlio_vehicle_visual_odometry",
                output="screen",
                parameters=[
                    {
                        "input_topic": guarded_odom_topic,
                        "output_topic": "/fmu/in/vehicle_visual_odometry",
                        "quality": quality,
                    }
                ],
            )
        ]
    )
