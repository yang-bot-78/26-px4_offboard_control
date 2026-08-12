"""Start only the EV observation path for a disarmed, propeller-off hand test.

EV chain: /Odometry -> fastlio_ev_health_monitor -> /Odometry/healthy
          -> fastlio_mavros_vision_bridge -> /mavros/vision_pose/pose_cov -> MAVROS
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    input_topic = LaunchConfiguration("input_topic")
    healthy_topic = LaunchConfiguration("healthy_topic")
    world_yaw_alignment_rad = LaunchConfiguration("world_yaw_alignment_rad")
    recovery_healthy_s = LaunchConfiguration("recovery_healthy_s")
    publish_speed = LaunchConfiguration("publish_speed")
    body_to_sensor_x_m = LaunchConfiguration("body_to_sensor_x_m")
    body_to_sensor_y_m = LaunchConfiguration("body_to_sensor_y_m")
    body_to_sensor_z_m = LaunchConfiguration("body_to_sensor_z_m")
    body_to_fastlio_yaw_rad = LaunchConfiguration("body_to_fastlio_yaw_rad")

    ev_health_monitor = Node(
        package="px4_ros_com",
        executable="fastlio_ev_health_monitor.py",
        name="fastlio_ev_health_monitor",
        output="screen",
        parameters=[
            {
                "input_topic": input_topic,
                "output_topic": healthy_topic,
                "px4_velocity_topic": "/mavros/local_position/velocity_local",
                "world_yaw_alignment_rad": world_yaw_alignment_rad,
                "max_input_age_s": 0.25,
                "max_position_jump_m": 0.15,
                "max_horizontal_velocity_difference_mps": 0.45,
                "max_internal_velocity_difference_mps": 0.50,
                "anomaly_to_fault_s": 0.3,
                "recovery_healthy_s": recovery_healthy_s,
                "message_timeout_s": 0.5,
                "velocity_lowpass_cutoff_hz": 3.0,
                "max_velocity_alignment_s": 0.1,
                "px4_velocity_max_input_age_s": 0.25,
                "healthy_position_variance_floor_m2": 0.01,
                "healthy_orientation_variance_floor_rad2": 0.02,
                "velocity_variance_m2ps2": 0.04,
            }
        ],
    )

    vision_bridge = Node(
        package="px4_ros_com",
        executable="fastlio_mavros_vision_bridge",
        name="fastlio_mavros_vision_bridge",
        output="screen",
        parameters=[
            {
                "input_topic": healthy_topic,
                "pose_topic": "/mavros/vision_pose/pose_cov",
                "restamp_message": False,
                "publish_speed": publish_speed,
                "world_yaw_alignment_rad": world_yaw_alignment_rad,
                "body_to_sensor_x_m": body_to_sensor_x_m,
                "body_to_sensor_y_m": body_to_sensor_y_m,
                "body_to_sensor_z_m": body_to_sensor_z_m,
                "body_to_fastlio_yaw_rad": body_to_fastlio_yaw_rad,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("input_topic", default_value="/Odometry"),
            DeclareLaunchArgument("healthy_topic", default_value="/Odometry/healthy"),
            DeclareLaunchArgument(
                "world_yaw_alignment_rad", default_value="0.0"
            ),
            DeclareLaunchArgument("body_to_sensor_x_m", default_value="0.0"),
            DeclareLaunchArgument("body_to_sensor_y_m", default_value="0.0"),
            DeclareLaunchArgument("body_to_sensor_z_m", default_value="0.08"),
            DeclareLaunchArgument("body_to_fastlio_yaw_rad", default_value="0.0"),
            # Validation-only override; production recovery is 2.0 s and the
            # Offboard node independently requires 7.5 s continuous HEALTHY.
            DeclareLaunchArgument("recovery_healthy_s", default_value="7.5"),
            # vision_speed is diagnostic-only: EKF2_EV_CTRL=11 has no bit2 (=4),
            # so PX4 does not fuse EV velocity regardless of this flag.  It stays
            # False by default because the default session should not add a
            # publisher it will not judge; set it True to record the topic and
            # verify the child->world twist rotation, which is the whole point of
            # a velocity-frame session.  Turning it on never enables fusion --
            # only changing EKF2_EV_CTRL would, and that is out of scope here.
            DeclareLaunchArgument("publish_speed", default_value="False"),
            ev_health_monitor,
            vision_bridge,
        ]
    )
