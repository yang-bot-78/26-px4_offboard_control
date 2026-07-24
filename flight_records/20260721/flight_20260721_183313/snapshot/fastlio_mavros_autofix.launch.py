from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _validate_single_ev_output(context):
    names = (
        "start_mavros_vision_bridge",
        "start_px4_ev_bridge",
        "start_bridge",
    )
    enabled = [
        name
        for name in names
        if LaunchConfiguration(name).perform(context).strip().lower()
        in ("1", "true", "yes", "on")
    ]
    if len(enabled) > 1:
        raise RuntimeError(
            "Only one external-vision output may feed PX4 at a time; enabled: "
            + ", ".join(enabled)
        )
    return []


def generate_launch_description():
    fcu_url = LaunchConfiguration("fcu_url")
    tgt_system = LaunchConfiguration("tgt_system")
    tgt_component = LaunchConfiguration("tgt_component")
    start_bridge = LaunchConfiguration("start_bridge")
    start_mavros_vision_bridge = LaunchConfiguration("start_mavros_vision_bridge")
    start_px4_ev_bridge = LaunchConfiguration("start_px4_ev_bridge")
    start_odom_guard = LaunchConfiguration("start_odom_guard")
    start_ev_health_monitor = LaunchConfiguration("start_ev_health_monitor")
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
    healthy_odom_topic = LaunchConfiguration("healthy_odom_topic")
    ev_max_input_age_s = LaunchConfiguration("ev_max_input_age_s")
    ev_max_position_jump_m = LaunchConfiguration("ev_max_position_jump_m")
    ev_max_velocity_difference_mps = LaunchConfiguration(
        "ev_max_velocity_difference_mps"
    )
    ev_anomaly_to_fault_s = LaunchConfiguration("ev_anomaly_to_fault_s")
    ev_recovery_healthy_s = LaunchConfiguration("ev_recovery_healthy_s")
    ev_message_timeout_s = LaunchConfiguration("ev_message_timeout_s")
    ev_velocity_lowpass_cutoff_hz = LaunchConfiguration(
        "ev_velocity_lowpass_cutoff_hz"
    )
    ev_max_velocity_alignment_s = LaunchConfiguration("ev_max_velocity_alignment_s")
    ev_px4_velocity_max_input_age_s = LaunchConfiguration(
        "ev_px4_velocity_max_input_age_s"
    )
    ev_effective_points_ok_topic = LaunchConfiguration(
        "ev_effective_points_ok_topic"
    )
    ev_effective_points_timeout_s = LaunchConfiguration(
        "ev_effective_points_timeout_s"
    )

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
                "max_computed_z_speed_mps": 1.2,
            }
        ],
    )

    # Safety path: inspect raw FAST-LIO so a frame rejected by the legacy guard
    # is still visible to the health state machine.  /Odometry/guarded remains a
    # parallel diagnostic/compatibility stream; every flight-controller bridge
    # below consumes only /Odometry/healthy.
    fastlio_ev_health_monitor = Node(
        package="px4_ros_com",
        executable="fastlio_ev_health_monitor.py",
        name="fastlio_ev_health_monitor",
        output="screen",
        condition=IfCondition(start_ev_health_monitor),
        parameters=[
            {
                "input_topic": fastlio_odom_topic,
                "output_topic": healthy_odom_topic,
                "px4_velocity_topic": "/mavros/local_position/velocity_local",
                "position_yaw_offset_rad": vision_position_yaw_offset_rad,
                "max_input_age_s": ev_max_input_age_s,
                "max_position_jump_m": ev_max_position_jump_m,
                "max_horizontal_velocity_difference_mps": ev_max_velocity_difference_mps,
                "anomaly_to_fault_s": ev_anomaly_to_fault_s,
                "recovery_healthy_s": ev_recovery_healthy_s,
                "message_timeout_s": ev_message_timeout_s,
                "velocity_lowpass_cutoff_hz": ev_velocity_lowpass_cutoff_hz,
                "max_velocity_alignment_s": ev_max_velocity_alignment_s,
                "px4_velocity_max_input_age_s": ev_px4_velocity_max_input_age_s,
                "effective_points_ok_topic": ev_effective_points_ok_topic,
                "effective_points_timeout_s": ev_effective_points_timeout_s,
                "healthy_position_variance_floor_m2": 0.01,
                "healthy_orientation_variance_floor_rad2": 0.02,
                "velocity_variance_m2ps2": 0.04,
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
                "input_topic": healthy_odom_topic,
                "pose_topic": "/mavros/vision_pose/pose_cov",
                "speed_topic": "/mavros/vision_speed/speed_twist_cov",
                "force_pose_frame_id": "odom",
                # MAVROS vision_speed_estimate interprets the vector as local ENU.
                "force_twist_frame_id": "odom",
                # Preserve FAST-LIO's measurement time.  The health gate rejects
                # stale data before it reaches MAVROS; never disguise old data as now.
                "restamp_message": False,
                # Publish for hand-held validation. EKF2_EV_CTRL remains 11, so
                # PX4 receives but does not fuse EV velocity unless explicitly
                # changed later by the operator.
                "publish_speed": True,
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
                "input_topic": healthy_odom_topic,
                "output_topic": "/fmu/in/vehicle_visual_odometry",
                "quality": ev_quality,
                "position_yaw_offset_rad": vision_position_yaw_offset_rad,
                "yaw_offset_rad": vision_yaw_offset_rad,
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
                "input_topic": healthy_odom_topic,
                "output_topic": "/mavros/odometry/out",
                "frame_id": "camera_init",
                "child_frame_id": "body",
                "force_frame_ids": False,
                "restamp_message": False,
                # /Odometry/healthy carries the validated differenced linear
                # velocity in its original world ENU axes for the vision-speed path.
                # Convert it to child/body before using MAVROS odometry input.
                "input_linear_velocity_frame": "world",
                "position_yaw_offset_rad": vision_position_yaw_offset_rad,
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
            DeclareLaunchArgument(
                "fcu_url",
                default_value="serial:///dev/ttyUSB0:921600?ids=255,190",
            ),
            DeclareLaunchArgument("tgt_system", default_value="1"),
            DeclareLaunchArgument("tgt_component", default_value="1"),
            DeclareLaunchArgument("start_bridge", default_value="false"),
            DeclareLaunchArgument("start_mavros_vision_bridge", default_value="true"),
            DeclareLaunchArgument("start_px4_ev_bridge", default_value="false"),
            DeclareLaunchArgument("start_odom_guard", default_value="true"),
            DeclareLaunchArgument("start_ev_health_monitor", default_value="true"),
            DeclareLaunchArgument("start_tf", default_value="true"),
            DeclareLaunchArgument("fix_timeout_sec", default_value="15.0"),
            DeclareLaunchArgument("camera_init_ned_yaw", default_value="0.0"),
            DeclareLaunchArgument("attitude_yaw_offset_rad", default_value="0.0"),
            DeclareLaunchArgument("body_yaw_offset_rad", default_value="0.0"),
            DeclareLaunchArgument("vision_yaw_offset_rad", default_value="0.0"),
            # The current flight logs empirically validate zero offset at this
            # interface (raw +X -> PX4 NED +Y).  The MID360 housing points toward
            # body-right, so retain zero only while the hand-held axis test shows
            # FAST-LIO's output axes are already aligned by its IMU/driver setup.
            DeclareLaunchArgument("vision_position_yaw_offset_rad", default_value="0.0"),
            DeclareLaunchArgument("ev_quality", default_value="100"),
            DeclareLaunchArgument("fastlio_odom_topic", default_value="/Odometry"),
            DeclareLaunchArgument("guarded_odom_topic", default_value="/Odometry/guarded"),
            DeclareLaunchArgument("healthy_odom_topic", default_value="/Odometry/healthy"),
            DeclareLaunchArgument("ev_max_input_age_s", default_value="0.25"),
            DeclareLaunchArgument("ev_max_position_jump_m", default_value="0.15"),
            DeclareLaunchArgument("ev_max_velocity_difference_mps", default_value="0.45"),
            DeclareLaunchArgument("ev_anomaly_to_fault_s", default_value="0.3"),
            DeclareLaunchArgument("ev_recovery_healthy_s", default_value="2.0"),
            DeclareLaunchArgument("ev_message_timeout_s", default_value="0.5"),
            DeclareLaunchArgument("ev_velocity_lowpass_cutoff_hz", default_value="3.0"),
            DeclareLaunchArgument("ev_max_velocity_alignment_s", default_value="0.1"),
            DeclareLaunchArgument(
                "ev_px4_velocity_max_input_age_s", default_value="0.25"
            ),
            DeclareLaunchArgument("ev_effective_points_ok_topic", default_value=""),
            DeclareLaunchArgument(
                "ev_effective_points_timeout_s", default_value="0.5"
            ),
            OpaqueFunction(function=_validate_single_ev_output),
            mavros_launch,
            fastlio_odometry_guard,
            fastlio_ev_health_monitor,
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
