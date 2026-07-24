"""Start only the EV observation path for a disarmed, propeller-off hand test."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Create a launch description with no arming, mode, or setpoint node."""
    input_topic = LaunchConfiguration("input_topic")
    healthy_topic = LaunchConfiguration("healthy_topic")
    position_yaw_offset_rad = LaunchConfiguration("position_yaw_offset_rad")
    recovery_healthy_s = LaunchConfiguration("recovery_healthy_s")

    health_monitor = Node(
        package="px4_ros_com",
        executable="fastlio_ev_health_monitor.py",
        name="fastlio_ev_health_monitor",
        output="screen",
        parameters=[
            {
                "input_topic": input_topic,
                "output_topic": healthy_topic,
                "px4_velocity_topic": "/mavros/local_position/velocity_local",
                "position_yaw_offset_rad": position_yaw_offset_rad,
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

    odometry_bridge = Node(
        package="px4_ros_com",
        executable="fastlio_mavros_odometry_bridge",
        name="fastlio_mavros_odometry_bridge",
        output="screen",
        parameters=[
            {
                "input_topic": healthy_topic,
                "output_topic": "/mavros/odometry/out",
                "frame_id": "camera_init",
                "child_frame_id": "body",
                "force_frame_ids": False,
                "restamp_message": False,
                "derive_missing_twist": False,
                "input_linear_velocity_frame": "child",
                "attitude_yaw_offset_rad": 0.0,
                "body_yaw_offset_rad": 0.0,
                "position_yaw_offset_rad": position_yaw_offset_rad,
            }
        ],
    )

    world_enu_to_ned = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_init_to_camera_init_ned_ev_validation",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "3.141592653589793", "--pitch", "0",
            "--yaw", "1.5707963267948966",
            "--frame-id", "camera_init", "--child-frame-id", "camera_init_ned",
        ],
    )

    body_flu_to_frd = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="body_to_body_frd_ev_validation",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "3.141592653589793", "--pitch", "0", "--yaw", "0",
            "--frame-id", "body", "--child-frame-id", "body_frd",
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("input_topic", default_value="/Odometry"),
            DeclareLaunchArgument("healthy_topic", default_value="/Odometry/healthy"),
            DeclareLaunchArgument("position_yaw_offset_rad", default_value="0.0"),
            # Validation-only override. Production health recovery is 2.0 s and
            # the Offboard node independently requires 7.5 s continuous HEALTHY.
            DeclareLaunchArgument("recovery_healthy_s", default_value="7.5"),
            health_monitor,
            odometry_bridge,
            world_enu_to_ned,
            body_flu_to_frd,
        ]
    )
