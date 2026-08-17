from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


# EV 链生产路径：
#   MID-360 -> FAST-LIO -> /Odometry -> EV health gate
#     -> fastlio_px4_vehicle_odometry -> /fmu/in/vehicle_visual_odometry
#     -> PX4 EKF2.  MAVROS ODOMETRY remains an explicit, disabled-by-default
#     diagnostic path and must not be enabled together with the native writer.
#
# 曾经的 start_bridge(/mavros/odometry/out) 与
# start_px4_ev_bridge(/fmu/in/vehicle_visual_odometry) 两条备选 EV 通路已删除，
# 连同其运行期互斥校验 —— 源头唯一比运行期检测更可靠。
# 新增任何 PX4 EV 写者前先核对当前 launch 参数、桥接实现和静态契约测试。


def generate_launch_description():
    fcu_url = LaunchConfiguration("fcu_url")
    tgt_system = LaunchConfiguration("tgt_system")
    tgt_component = LaunchConfiguration("tgt_component")
    start_mavros_odometry_bridge = LaunchConfiguration("start_mavros_odometry_bridge")
    start_px4_vehicle_odometry = LaunchConfiguration("start_px4_vehicle_odometry")
    start_odom_guard = LaunchConfiguration("start_odom_guard")
    start_ev_health_monitor = LaunchConfiguration("start_ev_health_monitor")
    start_tf = LaunchConfiguration("start_tf")
    fix_timeout_sec = LaunchConfiguration("fix_timeout_sec")
    camera_init_ned_yaw = LaunchConfiguration("camera_init_ned_yaw")
    body_yaw_offset_rad = LaunchConfiguration("body_yaw_offset_rad")
    min_linear_velocity_variance = LaunchConfiguration(
        "min_linear_velocity_variance"
    )
    world_yaw_alignment_rad = LaunchConfiguration("world_yaw_alignment_rad")
    body_to_sensor_x_m = LaunchConfiguration("body_to_sensor_x_m")
    body_to_sensor_y_m = LaunchConfiguration("body_to_sensor_y_m")
    body_to_sensor_z_m = LaunchConfiguration("body_to_sensor_z_m")
    body_to_fastlio_yaw_rad = LaunchConfiguration("body_to_fastlio_yaw_rad")
    ev_quality = LaunchConfiguration("ev_quality")
    fastlio_odom_topic = LaunchConfiguration("fastlio_odom_topic")
    guarded_odom_topic = LaunchConfiguration("guarded_odom_topic")
    healthy_odom_topic = LaunchConfiguration("healthy_odom_topic")
    ev_max_input_age_s = LaunchConfiguration("ev_max_input_age_s")
    guard_velocity_window_s = LaunchConfiguration("guard_velocity_window_s")
    guard_rebaseline_after_rejections = LaunchConfiguration(
        "guard_rebaseline_after_rejections"
    )
    ev_max_velocity_difference_mps = LaunchConfiguration(
        "ev_max_velocity_difference_mps"
    )
    ev_max_internal_velocity_difference_mps = LaunchConfiguration(
        "ev_max_internal_velocity_difference_mps"
    )
    ev_velocity_comparison_window_s = LaunchConfiguration(
        "ev_velocity_comparison_window_s"
    )
    ev_anomaly_to_fault_s = LaunchConfiguration("ev_anomaly_to_fault_s")
    ev_internal_velocity_unaligned_grace_s = LaunchConfiguration(
        "ev_internal_velocity_unaligned_grace_s"
    )
    ev_recovery_healthy_s = LaunchConfiguration("ev_recovery_healthy_s")
    ev_message_timeout_s = LaunchConfiguration("ev_message_timeout_s")
    ev_velocity_lowpass_cutoff_hz = LaunchConfiguration(
        "ev_velocity_lowpass_cutoff_hz"
    )
    ev_max_velocity_alignment_s = LaunchConfiguration("ev_max_velocity_alignment_s")
    ev_max_internal_velocity_alignment_s = LaunchConfiguration(
        "ev_max_internal_velocity_alignment_s"
    )
    ev_internal_velocity_history_s = LaunchConfiguration(
        "ev_internal_velocity_history_s"
    )
    ev_px4_velocity_max_input_age_s = LaunchConfiguration(
        "ev_px4_velocity_max_input_age_s"
    )
    ev_effective_points_ok_topic = LaunchConfiguration(
        "ev_effective_points_ok_topic"
    )
    ev_effective_points_timeout_s = LaunchConfiguration(
        "ev_effective_points_timeout_s"
    )
    ev_publish_rate_hz = LaunchConfiguration("ev_publish_rate_hz")
    timing_alignment_enabled = LaunchConfiguration("timing_alignment_enabled")
    require_frlio_anchor_status = LaunchConfiguration(
        "require_frlio_anchor_status"
    )
    frlio_anchor_status_timeout_s = LaunchConfiguration(
        "frlio_anchor_status_timeout_s"
    )
    frlio_max_anchor_age_s = LaunchConfiguration("frlio_max_anchor_age_s")
    frlio_max_predictor_age_s = LaunchConfiguration("frlio_max_predictor_age_s")
    frlio_allow_stale_lidar_predictor_degraded = LaunchConfiguration(
        "frlio_allow_stale_lidar_predictor_degraded"
    )
    flight_ready_output_timeout_s = LaunchConfiguration(
        "flight_ready_output_timeout_s"
    )
    require_px4_local_position = LaunchConfiguration("require_px4_local_position")
    px4_local_position_timeout_s = LaunchConfiguration("px4_local_position_timeout_s")
    require_px4_ev_fusion = LaunchConfiguration("require_px4_ev_fusion")
    px4_ev_aid_topic = LaunchConfiguration("px4_ev_aid_topic")
    px4_ev_fuse_timeout_s = LaunchConfiguration("px4_ev_fuse_timeout_s")
    px4_bootstrap_max_s = LaunchConfiguration("px4_bootstrap_max_s")
    velocity_variance_floor_x_m2ps2 = LaunchConfiguration(
        "velocity_variance_floor_x_m2ps2"
    )
    velocity_variance_floor_y_m2ps2 = LaunchConfiguration(
        "velocity_variance_floor_y_m2ps2"
    )
    velocity_variance_floor_z_m2ps2 = LaunchConfiguration(
        "velocity_variance_floor_z_m2ps2"
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
                "velocity_window_s": guard_velocity_window_s,
                "rebaseline_after_rejections": ParameterValue(
                    guard_rebaseline_after_rejections, value_type=int
                ),
                # A flight-time position jump must remain rejected; a ground
                # relocalization requires an explicit guarded restart.
                "allow_rebaseline": False,
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
                "world_yaw_alignment_rad": world_yaw_alignment_rad,
                "max_input_age_s": ev_max_input_age_s,
                "max_horizontal_velocity_difference_mps": ev_max_velocity_difference_mps,
                "max_internal_velocity_difference_mps":
                    ev_max_internal_velocity_difference_mps,
                "velocity_comparison_window_s": ev_velocity_comparison_window_s,
                "max_internal_velocity_alignment_s": ev_max_internal_velocity_alignment_s,
                "internal_velocity_history_s": ev_internal_velocity_history_s,
                "anomaly_to_fault_s": ev_anomaly_to_fault_s,
                "internal_velocity_unaligned_grace_s":
                    ev_internal_velocity_unaligned_grace_s,
                "recovery_healthy_s": ev_recovery_healthy_s,
                "message_timeout_s": ev_message_timeout_s,
                "velocity_lowpass_cutoff_hz": ev_velocity_lowpass_cutoff_hz,
                "max_velocity_alignment_s": ev_max_velocity_alignment_s,
                "px4_velocity_max_input_age_s": ev_px4_velocity_max_input_age_s,
                "effective_points_ok_topic": ev_effective_points_ok_topic,
                "effective_points_timeout_s": ev_effective_points_timeout_s,
                "require_frlio_anchor_status": ParameterValue(
                    require_frlio_anchor_status, value_type=bool
                ),
                "frlio_anchor_status_timeout_s": frlio_anchor_status_timeout_s,
                "frlio_max_anchor_age_s": frlio_max_anchor_age_s,
                "frlio_max_predictor_age_s": frlio_max_predictor_age_s,
                "frlio_allow_stale_lidar_predictor_degraded": ParameterValue(
                    frlio_allow_stale_lidar_predictor_degraded, value_type=bool
                ),
                "flight_ready_output_timeout_s": flight_ready_output_timeout_s,
                "require_px4_local_position": ParameterValue(
                    require_px4_local_position, value_type=bool
                ),
                "px4_local_position_timeout_s": px4_local_position_timeout_s,
                "require_px4_ev_fusion": ParameterValue(
                    require_px4_ev_fusion, value_type=bool
                ),
                "px4_ev_aid_topic": px4_ev_aid_topic,
                "px4_ev_fuse_timeout_s": px4_ev_fuse_timeout_s,
                "px4_bootstrap_max_s": px4_bootstrap_max_s,
                "healthy_position_variance_floor_m2": 0.01,
                "healthy_orientation_variance_floor_rad2": 0.02,
                "velocity_variance_m2ps2": 0.04,
                "velocity_variance_floor_x_m2ps2": velocity_variance_floor_x_m2ps2,
                "velocity_variance_floor_y_m2ps2": velocity_variance_floor_y_m2ps2,
                "velocity_variance_floor_z_m2ps2": velocity_variance_floor_z_m2ps2,
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
                [
                    FindPackageShare("px4_ros_com"),
                    "config",
                    "mavros_navigation_pluginlists.yaml",
                ]
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

    fastlio_mavros_odometry_bridge = Node(
        package="px4_ros_com",
        executable="fastlio_mavros_odometry_bridge",
        name="fastlio_mavros_odometry_bridge",
        output="screen",
        condition=IfCondition(start_mavros_odometry_bridge),
        parameters=[
            {
                "input_topic": healthy_odom_topic,
                "output_topic": "/mavros/odometry/out",
                "world_frame_id": "odom",
                "body_frame_id": "base_link",
                # Preserve FAST-LIO's measurement time. The health gate rejects
                # stale data before it reaches MAVROS; never disguise old data as now.
                "restamp_message": False,
                "max_publish_rate_hz": ParameterValue(
                    ev_publish_rate_hz, value_type=float
                ),
                "timing_alignment_enabled": ParameterValue(
                    timing_alignment_enabled, value_type=bool
                ),
                "world_yaw_alignment_rad": world_yaw_alignment_rad,
                "body_to_sensor_x_m": body_to_sensor_x_m,
                "body_to_sensor_y_m": body_to_sensor_y_m,
                "body_to_sensor_z_m": body_to_sensor_z_m,
                "body_to_fastlio_yaw_rad": body_to_fastlio_yaw_rad,
            }
        ],
    )

    # The native PX4 message is the production writer because it preserves
    # VehicleOdometry.reset_counter.  The expression prevents two EV writers
    # when an old validation command explicitly enables the MAVROS bridge.
    fastlio_px4_vehicle_odometry = Node(
        package="px4_ros_com",
        executable="fastlio_px4_vehicle_odometry",
        name="fastlio_px4_vehicle_odometry",
        output="screen",
        condition=IfCondition(PythonExpression([
            "'", start_px4_vehicle_odometry, "' == 'true' and '",
            start_mavros_odometry_bridge, "' == 'false'",
        ])),
        parameters=[
            {
                "input_topic": healthy_odom_topic,
                "output_topic": "/fmu/in/vehicle_visual_odometry",
                "reset_counter_topic": "/ev_health/reset_counter",
                "position_yaw_offset_rad": world_yaw_alignment_rad,
                "max_publish_rate_hz": ParameterValue(
                    ev_publish_rate_hz, value_type=float
                ),
            }
        ],
    )

    tf_odom_to_odom_ned = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="odom_to_odom_ned",
        output="screen",
        condition=IfCondition(start_tf),
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "3.141592653589793", "--pitch", "0",
            "--yaw", "1.5707963267948966",
            "--frame-id", "odom", "--child-frame-id", "odom_ned",
        ],
    )

    tf_base_link_to_base_link_frd = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_base_link_frd",
        output="screen",
        condition=IfCondition(start_tf),
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "3.141592653589793", "--pitch", "0", "--yaw", "0",
            "--frame-id", "base_link", "--child-frame-id", "base_link_frd",
        ],
    )

    tf_base_link_to_body = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_body",
        output="screen",
        condition=IfCondition(start_tf),
        # Physical pose of the FAST-LIO IMU child frame F in the aircraft
        # control-center FLU frame B. Translation is r_B; the orientation is
        # R_B_F = R_F_B^T, so its yaw is the negative installation yaw used by
        # the vision bridge.
        arguments=[
            "--x", body_to_sensor_x_m,
            "--y", body_to_sensor_y_m,
            "--z", body_to_sensor_z_m,
            "--roll", "0",
            "--pitch", "0",
            "--yaw", PythonExpression(["-1.0 * (", body_to_fastlio_yaw_rad, ")"]),
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
                default_value=(
                    "serial:///dev/serial/by-id/usb-1a86_USB_Serial-if00-port0:"
                    "921600?ids=255,190"
                ),
            ),
            DeclareLaunchArgument("tgt_system", default_value="1"),
            DeclareLaunchArgument("tgt_component", default_value="1"),
            DeclareLaunchArgument("start_mavros_odometry_bridge", default_value="true"),
            DeclareLaunchArgument("start_px4_vehicle_odometry", default_value="false"),
            DeclareLaunchArgument("start_odom_guard", default_value="true"),
            DeclareLaunchArgument("start_ev_health_monitor", default_value="true"),
            DeclareLaunchArgument("start_tf", default_value="true"),
            DeclareLaunchArgument("fix_timeout_sec", default_value="30.0"),
            DeclareLaunchArgument(
                "camera_init_ned_yaw", default_value="1.5707963267948966"
            ),
            DeclareLaunchArgument("body_yaw_offset_rad", default_value="0.0"),
            DeclareLaunchArgument(
                "min_linear_velocity_variance", default_value="0.0001"
            ),
            # Position and attitude are coordinates in the same world frame.
            # A single launch argument prevents inconsistent world yaw alignment.
            DeclareLaunchArgument(
                "world_yaw_alignment_rad", default_value="0.0"
            ),
            # This is a world-frame alignment, not the MID360 installation yaw.
            # Housing direction cannot determine it; retain zero until the
            # documented propeller-off world-axis test establishes otherwise.
            DeclareLaunchArgument("body_to_sensor_x_m", default_value="0.0"),
            DeclareLaunchArgument("body_to_sensor_y_m", default_value="0.0"),
            # Latest propeller-off validation measured the FAST-LIO pose origin
            # 0.08 m above the aircraft control center.  Keep direct launch
            # defaults identical to the production lever-arm configuration.
            DeclareLaunchArgument("body_to_sensor_z_m", default_value="0.08"),
            DeclareLaunchArgument(
                "body_to_fastlio_yaw_rad", default_value="0.0"),
            DeclareLaunchArgument("ev_quality", default_value="100"),
            DeclareLaunchArgument("fastlio_odom_topic", default_value="/Odometry"),
            DeclareLaunchArgument("guarded_odom_topic", default_value="/Odometry/guarded"),
            DeclareLaunchArgument("healthy_odom_topic", default_value="/Odometry/healthy"),
            DeclareLaunchArgument("ev_max_input_age_s", default_value="0.25"),
            DeclareLaunchArgument("guard_velocity_window_s", default_value="0.12"),
            DeclareLaunchArgument("guard_rebaseline_after_rejections", default_value="8"),
            DeclareLaunchArgument("ev_max_velocity_difference_mps", default_value="0.45"),
            DeclareLaunchArgument(
                "ev_max_internal_velocity_difference_mps", default_value="0.50"
            ),
            DeclareLaunchArgument(
                "ev_velocity_comparison_window_s", default_value="0.15"
            ),
            DeclareLaunchArgument("ev_anomaly_to_fault_s", default_value="0.3"),
            DeclareLaunchArgument(
                "ev_internal_velocity_unaligned_grace_s", default_value="0.20"
            ),
            DeclareLaunchArgument("ev_recovery_healthy_s", default_value="2.0"),
            DeclareLaunchArgument("ev_message_timeout_s", default_value="0.5"),
            DeclareLaunchArgument("ev_velocity_lowpass_cutoff_hz", default_value="3.0"),
            DeclareLaunchArgument("ev_max_velocity_alignment_s", default_value="0.1"),
            DeclareLaunchArgument(
                "ev_max_internal_velocity_alignment_s", default_value="0.1"
            ),
            DeclareLaunchArgument(
                "ev_internal_velocity_history_s", default_value="3.0"
            ),
            DeclareLaunchArgument(
                "ev_px4_velocity_max_input_age_s", default_value="0.25"
            ),
            DeclareLaunchArgument("ev_effective_points_ok_topic", default_value=""),
            DeclareLaunchArgument(
                "ev_effective_points_timeout_s", default_value="0.5"
            ),
            DeclareLaunchArgument("ev_publish_rate_hz", default_value="50.0"),
            DeclareLaunchArgument("timing_alignment_enabled", default_value="true"),
            DeclareLaunchArgument(
                "require_frlio_anchor_status", default_value="true"
            ),
            DeclareLaunchArgument(
                "frlio_anchor_status_timeout_s", default_value="0.5"
            ),
            DeclareLaunchArgument("frlio_max_anchor_age_s", default_value="0.40"),
            # Must match fr_lio's high_rate_odom.max_predictor_age_s. This gate
            # is the one that can take EV away, so a value tighter than FR-LIO's
            # own L3 threshold would block EV while FR-LIO still reports the
            # state as usable -- the two layers would disagree about the same
            # fact. The measured high-rate publish gap reaches 135 ms, so the
            # previous 0.05 s also fired during healthy operation.
            DeclareLaunchArgument("frlio_max_predictor_age_s", default_value="0.15"),
            DeclareLaunchArgument(
                "frlio_allow_stale_lidar_predictor_degraded", default_value="true"
            ),
            DeclareLaunchArgument(
                # Tolerate short FR-LIO LiDAR-correction stalls without dropping
                # flight_ready; EV fault and hard anchor gates still fail closed.
                "flight_ready_output_timeout_s", default_value="0.8"
            ),
            # The MAVLink transport does not expose PX4 uORB VehicleLocalPosition
            # to ROS 2. Use MAVROS local odometry instead; do not subscribe to
            # the unavailable /fmu topic.
            DeclareLaunchArgument("require_px4_local_position", default_value="false"),
            DeclareLaunchArgument("px4_local_position_timeout_s", default_value="0.30"),
            DeclareLaunchArgument("require_px4_ev_fusion", default_value="false"),
            DeclareLaunchArgument("px4_ev_aid_topic", default_value=""),
            DeclareLaunchArgument("px4_ev_fuse_timeout_s", default_value="0.50"),
            DeclareLaunchArgument("px4_bootstrap_max_s", default_value="15.0"),
            DeclareLaunchArgument(
                "velocity_variance_floor_x_m2ps2", default_value="0.0016"
            ),
            DeclareLaunchArgument(
                "velocity_variance_floor_y_m2ps2", default_value="0.0013"
            ),
            DeclareLaunchArgument(
                # 2026-08-15 FR-LIO prop-off vertical calibration:
                # static p95=0.0158 m/s; dynamic PX4 residual p95=0.0388 m/s.
                # The evidence-derived 1.5-sigma variance is 8.81629e-4;
                # round upward rather than understating the floor.
                "velocity_variance_floor_z_m2ps2", default_value="0.0009"
            ),
            mavros_launch,
            fastlio_odometry_guard,
            fastlio_ev_health_monitor,
            odometry_frame_fixer,
            fastlio_mavros_odometry_bridge,
            fastlio_px4_vehicle_odometry,
            tf_odom_to_odom_ned,
            tf_base_link_to_base_link_frd,
            tf_base_link_to_body,
            tf_camera_init_to_camera_init_ned,
            tf_body_to_body_frd,
        ]
    )
