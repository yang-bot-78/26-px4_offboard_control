import os
import sys
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import PackageNotFoundError
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition


def generate_launch_description():
    # ================================================================
    # Launch Arguments
    # ================================================================
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock'
    )
    enable_demo_cloud_arg = DeclareLaunchArgument(
        'enable_demo_cloud', default_value='false',
        description='Enable demo cloud node (for testing without real PCD)'
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Start RViz with FAST-LIO visualization config'
    )
    rviz_config_file_arg = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=PathJoinSubstitution([
            get_package_share_directory('race_bringup'), 'rviz', 'race_click_planner.rviz',
        ]),
        description='RViz configuration file; mission mode supplies its isolated click topic config'
    )
    map_file_arg = DeclareLaunchArgument(
        'map_file',
        # Was PCD/scans_nopillar.pcd inside the fast_lio share directory, which
        # does not exist here (FAST-LIO is an external prebuilt workspace), and
        # that PCD was a scan of the *simulation* arena in any case.
        #
        # This points at the one map that actually exists in this workspace so a
        # bring-up does not sit in NO_MAP forever.  It is NOT the competition
        # map: the rules allow scanning the real arena the day before, and that
        # scan must be passed here explicitly (map_file:=...).  Verify the PCD
        # matches the arena before trusting the planner's obstacle set.
        default_value=os.path.join(
            os.path.expanduser('~'), 'rong_ws', 'ws_offboard_control', 'maps',
            'fastlio_global_3d', 'GlobalMap.pcd'),
        description='Path to PCD map file for map_io_node. Override with the '
                    'arena scan before flying: map_file:=/path/to/arena.pcd'
    )
    start_fast_lio_arg = DeclareLaunchArgument(
        'start_fast_lio', default_value='false',
        description='Include the fast_lio package launch. False on the real '
                    'aircraft, where FAST-LIO runs from ~/livox_mid360_env.'
    )
    # When scanning a brand new map, map_file names the file that will be
    # written, and it does not exist yet. Leaving auto_load on makes map_io_node
    # retry the load once a second forever and flood the log with
    # "PCD file not found".
    map_auto_load_arg = DeclareLaunchArgument(
        'map_auto_load', default_value='true',
        description='Load map_file at startup. Set false when scanning a new map.'
    )
    fast_lio_cloud_registered_topic_arg = DeclareLaunchArgument(
        'fast_lio_cloud_registered_topic',
        default_value='/cloud_registered',
        description='Remap FAST-LIO /cloud_registered output'
    )
    fast_lio_cloud_registered_body_topic_arg = DeclareLaunchArgument(
        'fast_lio_cloud_registered_body_topic',
        default_value='/cloud_registered_body',
        description='Remap FAST-LIO /cloud_registered_body output'
    )
    fast_lio_pcd_save_arg = DeclareLaunchArgument(
        'fast_lio_pcd_save_en', default_value='false',
        description='Allow FAST-LIO to overwrite PCD/scans.pcd on shutdown'
    )
    body_to_sensor_x_arg = DeclareLaunchArgument(
        'body_to_sensor_x_m', default_value='0.0',
        description='Control-center to FAST-LIO pose origin lever arm in body FLU'
    )
    body_to_sensor_y_arg = DeclareLaunchArgument(
        'body_to_sensor_y_m', default_value='0.0',
        description='Control-center to FAST-LIO pose origin lever arm in body FLU'
    )
    body_to_sensor_z_arg = DeclareLaunchArgument(
        'body_to_sensor_z_m', default_value='0.08',
        description='Control-center to FAST-LIO pose origin lever arm in body FLU'
    )
    body_to_fastlio_yaw_arg = DeclareLaunchArgument(
        'body_to_fastlio_yaw_rad', default_value='0.0',
        description='Fixed body-to-FAST-LIO child-frame installation yaw'
    )
    world_yaw_alignment_arg = DeclareLaunchArgument(
        'world_yaw_alignment_rad', default_value='0.0',
        description='FAST-LIO camera_init world to project map ENU yaw alignment'
    )
    publish_fastlio_bridge_arg = DeclareLaunchArgument(
        'publish_fastlio_bridge', default_value='true',
        description='Publish the raw FAST-LIO -> /race/odom bridge'
    )
    publish_camera_init_tf_arg = DeclareLaunchArgument(
        'publish_camera_init_tf', default_value='true',
        description='Publish the fixed map -> camera_init TF'
    )
    broadcast_map_to_odom_arg = DeclareLaunchArgument(
        'broadcast_map_to_odom', default_value='true',
        description='Publish identity map -> odom TF; disable with external relocalization'
    )
    map_frame_id_arg = DeclareLaunchArgument(
        'map_frame_id', default_value='camera_init',
        description='Frame used for the static planning map'
    )
    fast_lio_odom_topic_arg = DeclareLaunchArgument(
        'fast_lio_odom_topic', default_value='/Odometry/healthy',
        description='Odometry input for the race mapping bridge'
    )

    # Evaluated in Python because a missing package must not break building the
    # description; IfCondition would still import the launch file first.
    start_fast_lio_requested = 'start_fast_lio:=true' in ' '.join(sys.argv)
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_demo_cloud = LaunchConfiguration('enable_demo_cloud')
    rviz = LaunchConfiguration('rviz')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    map_file = LaunchConfiguration('map_file')
    # map_io_node reads auto_load with as_bool(), so the substitution has to be
    # declared as a bool. Passing the raw LaunchConfiguration would deliver the
    # string "false" and throw InvalidParameterTypeException at startup.
    map_auto_load = ParameterValue(
        LaunchConfiguration('map_auto_load'), value_type=bool)
    fast_lio_cloud_registered_topic = LaunchConfiguration('fast_lio_cloud_registered_topic')
    fast_lio_cloud_registered_body_topic = LaunchConfiguration('fast_lio_cloud_registered_body_topic')
    fast_lio_pcd_save_en = LaunchConfiguration('fast_lio_pcd_save_en')

    # ================================================================
    # 1. FAST-LIO2 (optional)
    # ================================================================
    # On this workspace FAST-LIO is NOT a colcon package: the real aircraft runs
    # the prebuilt binary from ~/livox_mid360_env/ws_fastlio via
    # run_fastlio_mid360.sh, started by tools/flight/一键启动起飞栈.sh.
    # So the fast_lio package is normally absent and including its launch file
    # would raise PackageNotFoundError while merely building this description.
    # start_fast_lio stays for a workspace that does vendor the package.
    fast_lio_actions = []
    if start_fast_lio_requested:
        from ament_index_python.packages import PackageNotFoundError
        try:
            fast_lio_dir = get_package_share_directory('fast_lio')
            fast_lio_launch_path = os.path.join(fast_lio_dir, 'launch', 'mapping.launch.py')
            fast_lio_actions.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(fast_lio_launch_path),
                    launch_arguments={
                        'use_sim_time': use_sim_time,
                        'config_file': 'mid360.yaml',
                        'rviz': rviz,
                        'rviz_cfg': rviz_config_file,
                        'cloud_registered_topic': fast_lio_cloud_registered_topic,
                        'cloud_registered_body_topic': fast_lio_cloud_registered_body_topic,
                        'pcd_save_en': fast_lio_pcd_save_en,
                    }.items()
                ))
        except PackageNotFoundError:
            fast_lio_actions.append(
                LogInfo(
                    msg='[RACE_MAPPING] start_fast_lio:=true but the fast_lio package '
                        'is not installed; expecting an externally started FAST-LIO.'))

    # ================================================================
    # 2. fastlio_bridge_node - FAST-LIO ENU/body -> map/base_link relay
    # ================================================================
    bridge_node = Node(
        package='race_mapping',
        executable='fastlio_bridge_node',
        name='fastlio_bridge_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'body_to_sensor_x_m': LaunchConfiguration('body_to_sensor_x_m'),
            'body_to_sensor_y_m': LaunchConfiguration('body_to_sensor_y_m'),
            'body_to_sensor_z_m': LaunchConfiguration('body_to_sensor_z_m'),
            'body_to_fastlio_yaw_rad': LaunchConfiguration('body_to_fastlio_yaw_rad'),
            'world_yaw_alignment_rad': LaunchConfiguration('world_yaw_alignment_rad'),
            'odom_topic': LaunchConfiguration('fast_lio_odom_topic'),
            'health_status_topic': '/frlio/high_rate_odom/status',
            'planner_usable_topic': '/frlio/high_rate_odom/planner_usable',
            'ev_health_planner_usable_topic': '/ev_health/planner_usable',
            'require_health_status': True,
            'recovery_healthy_sec': 1.0,
            'max_speed_mps': 1.5,
            'max_dt_s': 0.5,
        }]
        , condition=IfCondition(LaunchConfiguration('publish_fastlio_bridge'))
    )

    # ================================================================
    # 3. frame_transform_node - 静态 TF 发布
    #    (发布 map→odom, camera_init→map；不发布未经标定的雷达 TF)
    # ================================================================
    frame_tf_node = Node(
        package='race_mapping',
        executable='frame_transform_node',
        name='frame_transform_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'world_yaw_alignment_rad': LaunchConfiguration('world_yaw_alignment_rad'),
            'publish_camera_init_tf': LaunchConfiguration('publish_camera_init_tf'),
            'broadcast_map_to_odom': LaunchConfiguration('broadcast_map_to_odom'),
        }]
    )

    # ================================================================
    # 4. map_io_node - PCD 地图加载/保存/发布
    #    默认加载 C 区无柱静态主地图 scans_nopillar.pcd → /saved_map
    # ================================================================
    map_io_node = Node(
        package='race_mapping',
        executable='map_io_node',
        name='map_io_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'map_file': map_file,
            'auto_load': map_auto_load,
            # /save_map_pcd 保存的是这个 topic 最后收到的那一条消息。
            # 节点默认值 /cloud_registered_body 是单帧机体系点云（实测约
            # 3000~5000 点），存出来是"最后一帧雷达看到的东西"，不是地图 ——
            # 一次实际保存只得到 3231 点 / 52KB，而同期 /Laser_map 已累积
            # 47 万点。/Laser_map 是 FAST-LIO 在 map_en:=true 下发布的全量
            # 累积地图，字段与前者完全一致（x/y/z/intensity/normal_*/curvature），
            # PointXYZI 可直接读取。
            'save_subscription_topic': '/Laser_map',
            'publish_topic': '/saved_map',
            # The PCD is stored in FAST-LIO camera_init coordinates.  Publish
            # that source frame and let camera_init -> map TF apply the single
            # world_yaw_alignment rotation.
            'frame_id': LaunchConfiguration('map_frame_id'),
            'display_leaf_size': 0.10,
            'max_display_points': 3000000,
        }]
    )

    # ================================================================
    # 5. mapping_manager_node - 建图管理 (自动加载/保存)
    # ================================================================
    mapping_mgr_node = Node(
        package='race_mapping',
        executable='mapping_manager_node',
        name='mapping_manager_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'auto_load_on_start': False,
            'auto_save_on_land': True,
            'load_delay_seconds': 5.0,  # 比 map_io_node 晚启动
        }]
    )

    # ================================================================
    # 6. race_mapping_demo_node (可选: 仅调试用)
    #    当没有真实 PCD 时, 生成虚拟点云测试 RViz 显示
    # ================================================================
    demo_node = Node(
        package='race_mapping',
        executable='race_mapping_demo_node',
        name='race_mapping_demo_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'num_points': 5000,
            'radius': 20.0,
            'height': 5.0,
        }],
        condition=IfCondition(enable_demo_cloud)  # 默认不启动
    )

    return LaunchDescription([
        start_fast_lio_arg,
        use_sim_time_arg,
        enable_demo_cloud_arg,
        rviz_arg,
        rviz_config_file_arg,
        map_file_arg,
        map_auto_load_arg,
        fast_lio_cloud_registered_topic_arg,
        fast_lio_cloud_registered_body_topic_arg,
        fast_lio_pcd_save_arg,
        body_to_sensor_x_arg,
        body_to_sensor_y_arg,
        body_to_sensor_z_arg,
        body_to_fastlio_yaw_arg,
        world_yaw_alignment_arg,
        publish_fastlio_bridge_arg,
        publish_camera_init_tf_arg,
        broadcast_map_to_odom_arg,
        map_frame_id_arg,
        fast_lio_odom_topic_arg,
        *fast_lio_actions,
        bridge_node,
        frame_tf_node,
        map_io_node,
        mapping_mgr_node,
        demo_node,
    ])
