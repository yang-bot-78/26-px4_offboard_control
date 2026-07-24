"""Start MAVROS only for the disarmed propeller-off EV validation."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Include MAVROS without any EV bridge or Offboard controller."""
    mavros = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("mavros"), "launch", "node.launch"])
        ),
        launch_arguments={
            "fcu_url": LaunchConfiguration("fcu_url"),
            # node.launch requires the argument but ros2 launch CLI rejects an
            # empty name:= value. Passing it here safely disables GCS forwarding.
            "gcs_url": "",
            "tgt_system": LaunchConfiguration("tgt_system"),
            "tgt_component": LaunchConfiguration("tgt_component"),
            "pluginlists_yaml": PathJoinSubstitution(
                [FindPackageShare("mavros"), "launch", "px4_pluginlists.yaml"]
            ),
            "config_yaml": PathJoinSubstitution(
                [
                    FindPackageShare("px4_ros_com"),
                    "config",
                    "mavros_px4_identity_override.yaml",
                ]
            ),
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "fcu_url",
                default_value="serial:///dev/ttyUSB0:921600?ids=255,190",
            ),
            DeclareLaunchArgument("tgt_system", default_value="1"),
            DeclareLaunchArgument("tgt_component", default_value="1"),
            mavros,
        ]
    )
