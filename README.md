# ws_offboard_control

本文档记录当前导航无人机工程的自动起降进度、默认链路、启动方法和已知边界。当前记录时间：2026-06-17。

## 工程目标

本工程用于把 `MID-360 + FAST-LIO` 的里程计送入 `PX4`，通过 `MAVROS` 给 PX4 提供外部视觉定位，并在此基础上验证室内 Offboard 自动起飞、悬停、降落和飞行数据记录。

当前主链路：

```text
MID-360
  -> livox_ros_driver2
  -> FAST-LIO
  -> /Odometry
  -> fastlio_mavros_vision_bridge
  -> /mavros/vision_pose/pose_cov
  -> MAVROS
  -> PX4 EKF2
  -> MAVROS setpoint_raw/local
  -> Offboard 起飞 / 悬停 / 降落
```

当前默认方案：

- 飞控通信：`MAVROS + MAVLink`
- 外部视觉输入：`MAVROS vision_pose`
- 默认不使用 `uXRCE-DDS` 作为 PX4 EV 输入
- 默认不使用 `MAVROS odometry` 作为 PX4 EV 输入

## 当前自动起降进度

当前自动起降链路已经完成到“可进入 Offboard、可解锁、可到达目标高度、可悬停、可执行 Offboard 控制下降、切换 AUTO.LAND、落地后自动上锁”的阶段。

已完成：

- `MID-360 -> FAST-LIO -> MAVROS vision_pose -> PX4 EKF2` 链路已打通。
- `vehicle_visual_odometry` 能稳定进入 PX4。
- `vehicle_visual_odometry.pose_frame = 1`。
- `vehicle_local_position.heading_good_for_control = True`。
- `estimator_aid_src_ev_yaw.fused = True`。
- `estimator_aid_src_ev_yaw.innovation_rejected = False`。
- `heading` 与 `unaided_heading` 可以对齐。
- 静置后 `reset counter` 不再持续增长。
- Offboard 起飞节点已具备状态机流程：等待 MAVROS、等待视觉/本地里程计、预发送 setpoint、等待遥控器切 OFFBOARD、稳定等待、解锁、爬升、悬停、降落。
- 当前主脚本加入视觉新鲜度检查、起飞前平均里程计锁点、Z 轴爬升斜坡、水平漂移告警/保护、Offboard 控制下降、低高度后切 `AUTO.LAND`、落地后自动 disarm。
- 一键启动脚本已能顺序拉起 `MID360 -> FAST-LIO -> MAVROS -> rosbag -> 起飞脚本`，并把日志集中保存到 `flight_records/`。
- rosbag 记录脚本已能保存飞行时的话题、节点列表、参数快照、脚本快照、git 状态和 MID-360 配置快照。

旧日志中 `AUTO.LAND` 请求成功后节点会立即退出；更新后预期收尾日志为：

```text
OFFBOARD reported by MAVROS state
Arming vehicle
Target reached; starting hover timer before landing
Landing with OFFBOARD position setpoints
Starting OFFBOARD controlled landing
OFFBOARD landing reached AUTO.LAND handoff height; requesting AUTO.LAND
Waiting for landing completion before disarm
Vehicle disarmed; motors should be stopped
MAVROS offboard control completed; stopping node
```

当前注意事项：

- 当前主起降脚本已统一为 `run_takeoff_1m_hold.sh`，目标为相对起点上升约 `0.5 m`。
- `30cm/0p3m` 旧脚本已经删除，所有起飞、悬停、一键启动和自启动入口均使用 `1m` 命名。
- 当前 `auto_land:=false`，但 `offboard_land:=true`，所以逻辑是先 Offboard 控制下降，到接近起点高度后再请求 `AUTO.LAND`。
- 最近 `runtime/last_rosbag_status.txt` 显示上一轮 rosbag 状态为 `interrupted`，含义是上次录包未被脚本干净停止，可能是系统重启或进程异常退出导致。飞行目录中的历史 rosbag 和 snapshot 仍可用于排查。

## 当前默认起飞参数

主脚本：[run_takeoff_1m_hold.sh](run_takeoff_1m_hold.sh)

关键参数：

```text
arm_only=false
use_rc_offboard=true
require_vision_pose=true
vision_freshness_s=1.0
prestream_count=100
recent_pose_samples=30
lift_only_seconds=0.0
z_ramp_seconds=1.0
MPC_TKO_RAMP_T=1.0
target_x_m=0.0
target_y_m=0.0
target_z_m=-0.50
hover_seconds=10.0
auto_land=false
offboard_land=true
offboard_land_speed_mps=0.10
offboard_land_auto_handoff_height_m=0.10
auto_disarm_after_land=true
disarm_after_land_delay_s=2.0
land_wait_timeout_s=45.0
landed_ground_tolerance_m=0.06
target_reached_tolerance_m=0.15
land_on_offboard_loss=true
offboard_stabilize_seconds=20.0
drift_warning_m=0.20
drift_land_m=0.30
drift_emergency_m=0.50
```

说明：

- `target_z_m` 使用 NED 语义，`-0.50` 表示向上爬升约 `0.5 m`。
- PX4 `MPC_TKO_RAMP_T=1.0 s` 由 QGC 配置；脚本不在启动时重复写飞控参数。Offboard 高度设定点在 `1.0 s` 内受限爬升到 `0.5 m`，可用 `Z_RAMP_SECONDS` 环境变量调整。
- `use_rc_offboard=true` 表示脚本持续发 setpoint，但等待遥控器/外部操作切入 `OFFBOARD` 后再进入解锁流程。
- `offboard_stabilize_seconds=20.0` 表示进入 `OFFBOARD` 后稳定等待 20 秒再请求解锁。
- 水平漂移达到 `0.20 m` 会报警，达到 `0.30 m` 会触发保护日志；当前代码记录风险并持续发送 setpoint，不会直接强制中断控制。

## 自动起降状态机

控制节点：[minipc_mavros_offboard.py](src/px4_ros_com/scripts/minipc_mavros_offboard.py)

主要阶段：

```text
WAIT_FOR_CONNECTION
  -> WAIT_FOR_LOCAL_POSE
  -> STREAM_SETPOINTS
  -> WAIT_FOR_RC_OFFBOARD
  -> REQUEST_ARM
  -> HOLD
  -> OFFBOARD_LAND
  -> REQUEST_LAND
  -> WAIT_FOR_LAND
  -> REQUEST_DISARM
  -> WAIT_FOR_DISARM
  -> DONE
```

新增的 Nav2 2.5D 路线跟踪阶段默认关闭。显式设置
`enable_route_tracking=true` 后，悬停完成才会尝试进入：

```text
HOLD
  -> ROUTE_TRACKING
  -> FINAL_HOLD
  -> OFFBOARD_LAND
  -> REQUEST_LAND
  -> WAIT_FOR_LAND
  -> REQUEST_DISARM
  -> WAIT_FOR_DISARM
  -> DONE
```

该阶段只订阅 Nav2 planner-only 输出的 `nav_msgs/Path`，默认话题为
`/nav2_stage1/path`，不会启动或接入 Nav2 controller，也不会使用
`/cmd_vel` 控制无人机。实际控制量仍由同一个 Offboard 节点发布到
`/mavros/setpoint_raw/local`，同一时刻保持单一 setpoint 发布逻辑。

第一版路线跟踪采用固定高度 2.5D lookahead 跟踪：路径在 `map` 坐标系中，
进入路线前通过当前 `map -> body` TF 和 `/mavros/local_position/odom`
建立一次 `map_to_local_xy_offset`，再把 lookahead 目标点转换到 MAVROS local
setpoint。高度参数 `route_fixed_flight_height_m=1.0` 使用 ROS ENU 向上语义，
发布到 MAVROS `FRAME_LOCAL_NED` 前会转换为 `position.z=-1.0`。

可通过 [run_takeoff_1m_hold.sh](run_takeoff_1m_hold.sh) 的环境变量开启：

```bash
ENABLE_ROUTE_TRACKING=true ROUTE_PATH_TOPIC=/nav2_stage1/path ./run_takeoff_1m_hold.sh
```

route 相关参数默认保守：

```text
enable_route_tracking=false
route_path_topic=/nav2_stage1/path
route_fixed_flight_height_m=1.0
route_start_tolerance_m=0.40
route_goal_tolerance_m=0.20
route_lookahead_m=0.40
route_max_vxy_mps=0.25
route_max_vz_mps=0.15
route_setpoint_rate_hz=30.0
route_path_timeout_s=2.0
route_fail_action=hold
route_final_hold_seconds=3.0
```

核心行为：

- 起飞前要求 MAVROS 已连接。
- 起飞前要求 `/mavros/local_position/odom` 有效。
- 默认要求 `/mavros/vision_pose/pose_cov` 在 `1.0 s` 内有新数据。
- 使用最近 `30` 个里程计样本平均值锁定起飞参考点。
- 发布 `/mavros/setpoint_raw/local`，坐标帧为 `FRAME_LOCAL_NED`。
- 爬升过程中对 Z 轴 setpoint 做斜坡，减少突变。
- 到达目标高度并满足水平误差条件后开始悬停计时。
- 降落阶段先继续使用 Offboard 位置 setpoint 缓慢下降，再在接近起点高度时请求 `AUTO.LAND`。
- 请求 `AUTO.LAND` 后节点不再立即退出；它会等待 `/mavros/extended_state` 报告 `LANDED_STATE_ON_GROUND`，延迟 `2 s` 后调用 `/mavros/cmd/arming` 发送 `value=false` 主动上锁。如果 extended state 长时间不可用，会在 `45 s` 后用本地高度接近起点高度作为备用判据并打印 warning。

## 已验证有效配置

### FAST-LIO 外参

文件：

- `~/livox_mid360_env/ws_fastlio/src/fast_lio/config/mid360.yaml`

当前有效值：

```yaml
extrinsic_R: [ 1., 0., 0.,
                0., 1., 0.,
                0.,  0., 1.]
```

### MAVROS vision 安装方向修正

相关文件：

- [fastlio_mavros_vision_bridge.cpp](src/px4_ros_com/src/bridges/fastlio_mavros_vision_bridge.cpp)
- [fastlio_mavros_autofix.launch.py](src/px4_ros_com/launch/fastlio_mavros_autofix.launch.py)

当前默认值：

```text
vision_yaw_offset_rad = 0.0
vision_position_yaw_offset_rad = 0.0
```

说明：

- MID360 使用内置 IMU，整机安装方向改变时不改 FAST-LIO 的 `extrinsic_R`。
- 当前整机坐标约定为 `+X` 向前、`+Y` 向左、`+Z` 向上，因此不旋转 FAST-LIO 水平位置。
- 旧值 `1.57079632679` 会把 `(x, y)` 旋转成 `(-y, x)`，导致机头向前移动表现为 `Y` 增加。

### PX4 参数

当前已验证有效：

```bash
param set EKF2_EV_CTRL 11
param save
```

说明：

- `11` = EV 水平位置 + EV 垂直位置 + EV yaw
- 当前不把 `15` 作为默认值

## 启动方式

### 一键启动

```bash
cd ~/ws_offboard_control
./start_takeoff_1m_stack.sh
```

该脚本会打开 5 个终端：

1. `MID360 Driver`
2. `FAST-LIO`
3. `PX4 MAVROS`
4. `ROS Bag Debug`
5. `Takeoff 0.5m`

一键启动前会运行：

```bash
./check_takeoff_autostart_ready.sh
```

用于检查 `gnome-terminal`、ROS 环境、构建产物、关键脚本、MID-360/FAST-LIO 启动脚本和 MAVROS launch 参数是否存在。

### 手动四终端启动

终端 1：MID-360 驱动

```bash
~/livox_mid360_env/run_mid360_driver.sh
```

终端 2：FAST-LIO

```bash
~/livox_mid360_env/run_fastlio_mid360.sh
```

终端 3：MAVROS + PX4 链路

```bash
cd ~/ws_offboard_control
source /opt/ros/humble/setup.bash
source ~/ws_offboard_control/install/setup.bash
ros2 launch px4_ros_com fastlio_mavros_autofix.launch.py \
  fcu_url:=serial:///dev/ttyUSB0:921600?ids=255,190
```

终端 4：起飞控制

```bash
cd ~/ws_offboard_control
./run_takeoff_1m_hold.sh
```

## 数据记录

录包脚本：[record_takeoff_debug_bag.sh](record_takeoff_debug_bag.sh)

输出目录：

```text
flight_records/YYYYMMDD/flight_YYYYMMDD_HHMMSS/
```

每次飞行目录包含：

- `logs/`：各终端日志
- `rosbag/`：飞行 rosbag
- `snapshot/`：脚本、参数、节点、话题、git 状态和环境快照

默认录制话题包括：

- `/Odometry`
- `/Odometry/guarded`
- `/path`
- `/cloud_registered`
- `/mavros/state`
- `/mavros/local_position/pose`
- `/mavros/local_position/odom`
- `/mavros/vision_pose/pose_cov`
- `/mavros/vision_speed/speed_twist_cov`
- `/mavros/setpoint_raw/local`
- `/mavros/setpoint_raw/target_local`
- `/fmu/out/vehicle_local_position`
- `/fmu/out/vehicle_local_position_setpoint`
- `/fmu/out/vehicle_attitude`
- `/fmu/out/vehicle_attitude_setpoint`
- `/fmu/out/trajectory_setpoint`
- `/fmu/in/trajectory_setpoint`

查看最近 rosbag 状态：

```bash
cd ~/ws_offboard_control
./show_last_rosbag_status.sh
```

手动停止录包：

```bash
cd ~/ws_offboard_control
./stop_takeoff_debug_bag.sh
```

清理录包：

```bash
cd ~/ws_offboard_control
./delete_recorded_rosbags.sh
./delete_recorded_rosbags.sh latest
```

## 开机/登录后自启动

打开 GNOME 登录后自动启动：

```bash
cd ~/ws_offboard_control
./enable_autostart.sh
```

关闭自启动：

```bash
cd ~/ws_offboard_control
./disable_autostart.sh
```

自启动入口会调用：

```bash
./start_takeoff_1m_stack_login.sh
```

该脚本登录后等待约 `10 s`，再执行 `start_takeoff_1m_stack.sh`。

## 全局建图后端

当前工程新增了独立后端包：

- [fastlio_global_slam](src/fastlio_global_slam)

该包挂在现有 FAST-LIO2 后面，默认不影响当前 `MAVROS + PX4` 控制链路。

目标链路：

```text
FAST-LIO2
  -> Scan Context 回环候选
  -> ICP 精配准
  -> 可选 GTSAM 因子图优化
  -> 全局地图 / 优化路径
  -> 后续可扩展到 Scan Context 重定位
```

当前已落地：

- 订阅 `/Odometry` 和 `/cloud_registered`
- 按位姿增量抽取关键帧
- 生成 Scan Context 描述子并做回环候选检索
- 对候选回环执行 ICP 精配准
- 提供地图保存服务
- 提供地图加载服务
- 提供基于最新扫描的重定位服务
- 发布全局地图 `/fastlio_global/map`
- 发布优化路径 `/fastlio_global/path`
- 发布回环边可视化 `/fastlio_global/loop_markers`
- 发布重定位结果 `/fastlio_global/relocalized_pose`

当前边界：

- `GTSAM` 在本机未装系统开发包时，会自动退化成无因子图模式。
- 本机当前还没有安装 `libgtsam-dev`，所以因子图代码已接入但未实际启用。

## Nav2 路径规划与旧地图复用

当前工程已在不接入 PX4 控制的前提下，完成 `FAST-LIO + 3D 重定位 + Nav2 planner-only` 验证。

当前已跑通链路：

```text
FAST-LIO
  -> /Odometry + /cloud_registered
  -> fastlio_global_slam 加载旧 3D 关键帧地图
  -> Scan Context + ICP 重定位
  -> /fastlio_global/relocalized_pose
  -> relocalized_pose_to_tf 发布 map -> camera_init
  -> odometry_tf_publisher 发布 camera_init -> body
  -> Nav2 map_server 加载旧 2D 地图
  -> Nav2 planner_server 生成 /nav2_stage1/path
```

当前边界：

- 该阶段只做路径规划和 RViz 验证，不向 PX4 发布控制指令。
- 重启后复用旧地图时，只有 2D `map.yaml/.pgm` 不够，必须同时有 `fastlio_global_slam` 保存的 3D 关键帧地图。
- `2D Pose Estimate` 当前不会校正 FAST-LIO 位姿；定位依赖 `trigger_fastlio_relocalize.sh` 的 3D 重定位结果。

### 地图文件

默认 3D 重定位地图目录：

```text
/home/robot/ws_offboard_control/maps/fastlio_global_3d
```

默认 2D Nav2 地图：

```text
/home/robot/ws_offboard_control/maps/fastlio_nav2_map.yaml
/home/robot/ws_offboard_control/maps/fastlio_nav2_map.pgm
```

保存 3D 重定位地图：

```bash
cd ~/ws_offboard_control
./save_fastlio_global_3d_map.sh
```

检查 3D 地图关键帧数量：

```bash
cd ~/ws_offboard_control
./check_fastlio_global_3d_map.sh
```

建议至少 `10+` 个 keyframes，最好 `30+`。如果只显示 `1` 个 keyframe，说明扫图时几乎没有移动，重定位很难可靠。

保存 2D Nav2 地图的当前推荐参数：

```bash
cd ~/ws_offboard_control
CLOUD_TOPIC=/Laser_map \
MAP_RESOLUTION=0.05 \
MAP_OCCUPIED_DILATION_M=0.10 \
MAP_OBSTACLE_MIN_HEIGHT_M=0.30 \
./save_fastlio_nav2_map.sh
```

### 重启后复用旧地图的确定启动顺序

先清理旧的 Nav2/RViz，避免重复节点导致 planner/action 混乱：

```bash
cd ~/ws_offboard_control
STOP_RVIZ=true ./stop_nav2_relocalized_map.sh
```

终端 1：启动 FAST-LIO，使用原来的 FAST-LIO 启动方式。启动后确认：

```bash
source /opt/ros/humble/setup.bash
source ~/ws_offboard_control/install/setup.bash
ros2 topic hz /Odometry
ros2 topic hz /cloud_registered
```

终端 2：启动 3D 重定位后端：

```bash
cd ~/ws_offboard_control
./run_fastlio_global_relocalization.sh
```

终端 3：加载旧 3D 地图：

```bash
cd ~/ws_offboard_control
./load_fastlio_global_3d_map.sh
```

返回 `success=True` 后，轻微移动或转动雷达/飞机，触发第一次重定位：

```bash
./trigger_fastlio_relocalize.sh
```

需要看到类似：

```text
success=True
Relocalization succeeded
```

终端 4：启动 Nav2 复用 2D 地图：

```bash
cd ~/ws_offboard_control
RVIZ=true ./run_nav2_relocalized_map.sh
```

如果已有 RViz，只启动 Nav2 节点：

```bash
RVIZ=false ./run_nav2_relocalized_map.sh
```

再触发一次重定位，让 Nav2 的 TF 桥接节点收到新的 `/fastlio_global/relocalized_pose`：

```bash
cd ~/ws_offboard_control
./trigger_fastlio_relocalize.sh
```

检查 TF：

```bash
ros2 run tf2_ros tf2_echo map camera_init
ros2 run tf2_ros tf2_echo map body
```

能连续输出 transform 即表示：

```text
map -> camera_init -> body
```

链路已连通。

### RViz 路径规划验证

Nav2 RViz 配置：

```bash
ros2 run rviz2 rviz2 -d ~/ws_offboard_control/install/offboard_nav2_planning/share/offboard_nav2_planning/rviz/nav2_stage1_planning.rviz
```

RViz 左侧设置：

```text
Global Options -> Fixed Frame = map
```

应能看到以下显示项：

- `Static Map`
- `Nav2 Global Costmap`
- `Nav2 Planned Path`
- `Nav2 2.5D Path`
- `FAST-LIO Odometry`
- `TF`

使用顶部 `2D Goal Pose` 点目标。成功后会发布：

```text
/nav2_stage1/path
```

可用命令确认：

```bash
ros2 topic echo /goal_pose --once
ros2 topic echo /nav2_stage1/path --once
```

如果 `/goal_pose` 有数据但 `/nav2_stage1/path` 没有，优先检查：

- 是否存在重复 Nav2 节点：

  ```bash
  ros2 node list | grep -E 'planner_server|map_server|nav2_stage1_goal_to_path'
  ```

- `planner_server` 是否 active：

  ```bash
  ros2 lifecycle get /planner_server
  ```

- 目标点和飞机当前位置是否落在 costmap 可通行区域，而不是障碍或未知区域。

### 相关脚本

- [run_fastlio_global_relocalization.sh](run_fastlio_global_relocalization.sh)：启动 3D 重定位后端
- [load_fastlio_global_3d_map.sh](load_fastlio_global_3d_map.sh)：加载旧 3D 关键帧地图
- [trigger_fastlio_relocalize.sh](trigger_fastlio_relocalize.sh)：触发一次 3D 重定位
- [run_nav2_relocalized_map.sh](run_nav2_relocalized_map.sh)：启动 Nav2 静态地图规划和 TF 桥
- [stop_nav2_relocalized_map.sh](stop_nav2_relocalized_map.sh)：停止 Nav2 规划栈，避免重复节点
- [save_fastlio_global_3d_map.sh](save_fastlio_global_3d_map.sh)：保存 3D 重定位地图
- [save_fastlio_nav2_map.sh](save_fastlio_nav2_map.sh)：从点云投影保存 2D Nav2 地图
- [check_relocalization_status.sh](check_relocalization_status.sh)：检查重定位相关话题/TF
- [check_fastlio_global_3d_map.sh](check_fastlio_global_3d_map.sh)：检查 3D 地图 keyframe 数量

## 构建

修改 `px4_ros_com` 后：

```bash
cd ~/ws_offboard_control
source /opt/ros/humble/setup.bash
colcon build --packages-select px4_ros_com
source ~/ws_offboard_control/install/setup.bash
```

同时构建全局建图后端：

```bash
cd ~/ws_offboard_control
source /opt/ros/humble/setup.bash
colcon build --packages-select px4_ros_com fastlio_global_slam
source ~/ws_offboard_control/install/setup.bash
```

如需启用 GTSAM 因子图优化：

```bash
sudo apt-get update
sudo apt-get install -y libgtsam-dev

cd ~/ws_offboard_control
source /opt/ros/humble/setup.bash
colcon build --packages-select fastlio_global_slam
source ~/ws_offboard_control/install/setup.bash
```

## 核心文件

- MAVROS/PX4 默认 launch：[fastlio_mavros_autofix.launch.py](src/px4_ros_com/launch/fastlio_mavros_autofix.launch.py)
- Offboard 起降控制：[minipc_mavros_offboard.py](src/px4_ros_com/scripts/minipc_mavros_offboard.py)
- MAVROS vision bridge：[fastlio_mavros_vision_bridge.cpp](src/px4_ros_com/src/bridges/fastlio_mavros_vision_bridge.cpp)
- 里程计保护节点：[fastlio_odometry_guard.cpp](src/px4_ros_com/src/bridges/fastlio_odometry_guard.cpp)
- 旧 MAVROS odometry bridge：[fastlio_mavros_odometry_bridge.cpp](src/px4_ros_com/src/bridges/fastlio_mavros_odometry_bridge.cpp)
- 起飞脚本：[run_takeoff_1m_hold.sh](run_takeoff_1m_hold.sh)
- 悬停脚本：[run_hover_1m_offboard.sh](run_hover_1m_offboard.sh)
- 一键启动脚本：[start_takeoff_1m_stack.sh](start_takeoff_1m_stack.sh)
- 录包脚本：[record_takeoff_debug_bag.sh](record_takeoff_debug_bag.sh)
- 自启动检查：[check_takeoff_autostart_ready.sh](check_takeoff_autostart_ready.sh)
- 脚本速查：[ROOT_SCRIPTS_QUICK_REF.md](ROOT_SCRIPTS_QUICK_REF.md)
- 脚本指南：[SCRIPTS_GUIDE.md](SCRIPTS_GUIDE.md)
- 全局建图 launch：[fastlio_global_slam.launch.py](src/fastlio_global_slam/launch/fastlio_global_slam.launch.py)
- 全局建图后端：[fastlio_global_backend.cpp](src/fastlio_global_slam/src/fastlio_global_backend.cpp)

## 下一步建议

短期建议：

- 对最近一次完整起降 rosbag 做一次高度、水平误差、setpoint 与 EKF local position 的曲线复盘。
- 明确漂移保护策略：当前达到 `drift_land_m` 后只是记录错误并继续发 setpoint，后续可改成进入 Offboard Land 或要求人工接管。
- 再做 3 到 5 次同参数重复起降，记录最大水平误差、目标高度误差、降落切 `AUTO.LAND` 高度和落地稳定性。

中期建议：

- 增加一份标准试飞检查单：桨叶/电池/定位/遥控器模式/PX4 参数/rosbag 状态/紧急接管流程。
- 将 `show_last_rosbag_status.sh` 纳入一键启动前提示，避免上一次录包异常被忽略。
- 在 README 中维护一个飞行记录表，把每次有效试飞的高度、耗时、是否完整降落、异常现象和 rosbag 路径记录下来。
