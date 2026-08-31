# EgoHX PX4 Offboard Navigation

> 工程状态整理：2026-08-31
> 工作区：`/home/robot/egohx_ws/26-px4_offboard_control-main`
> 当前结论：`FLIGHT_GO_NO_GO=GO`，当前配置已通过实飞验证

本工程是一套面向室内无人机的 ROS 2 Humble 导航与 PX4 Offboard 控制工作区。系统使用
MID-360 激光雷达完成激光惯性里程计和全局重定位，通过 MAVROS 向 PX4 提供外部视觉定位，
并由 Super A*、EGO B-spline、航点状态机和 Offboard 控制节点完成路径规划、局部避障、
任务行为与飞控指令输出。

本文只描述当前工作区，不再记录历次 README 修改。旧方案和阶段性记录统一保存在
[`docs/archive/`](docs/archive/)；它们不能覆盖当前源码、配置和验证结论。

## 当前进度

| 模块 | 当前状态 |
| --- | --- |
| MID-360 与 LIO | 主启动脚本默认选择仓库内 `FR-LIO`，同时保留外部 `FAST-LIO` 入口 |
| PX4 外部视觉 | `/Odometry` 经健康守卫后，由唯一 vision bridge 写入 MAVROS `vision_pose` |
| 全局定位 | 已接入 Scan Context + ICP 单次重定位，并生成统一的 `map` 坐标里程计 |
| 地图与航点 | 航点文件记录规划地图 SHA-256，地图不匹配时任务入口拒绝启动 |
| 全局规划 | Super A* 支持静态 PCD、路径平滑、窄通道、逆风恢复和终点减速 |
| 局部规划 | EGO B-spline 支持静态地图与实时点云融合，当前默认 `static_live_px4` |
| Offboard 控制 | `offboard_waypoint_node` 是 PX4 local setpoint 的唯一发布者，负责起飞交接、跟踪、悬停和故障降落 |
| 航点任务 | 支持保存任意数量航点、顺序飞行、到点停留、识别开关、A* 接管、全局调速和航向扫描 |
| 数据记录 | 一键栈可保存 rosbag、日志、参数、话题、节点、Git 与环境快照 |
| 验证结论 | 当前配置已通过实飞验证，定位、重定位、规划、航点任务和 Offboard 控制链路可用 |

当前已完成并经实飞验证的重点能力：

- `yaw_sweep` 已通过受保护的 `/race/mission/yaw_override` 请求接入唯一 Offboard 控制器；
  每个航向阶段按 `/race/pose` 实测角度确认，完成前状态机不会切换到下一航点。
- Super 直控路径在终点前缩短前视距离并降低速度前馈，减少高速穿越终点的风险。
- 航点和地图绑定校验支持规划地图由同一重定位库派生的受控子集，同时拒绝无关地图。
- 当前应用任务位于 `src/race_offboard/config/waypoints/main/`，包含 13 个路线点；
  `P006` 配置了左转 90 度、停留 5 秒并返回到达航向的行为。

## 系统链路

```text
MID-360
  -> livox_ros_driver2
  -> FR-LIO / FAST-LIO
  -> /Odometry
  -> fastlio_odometry_guard
  -> /Odometry/healthy
  -> fastlio_mavros_vision_bridge
  -> /mavros/vision_pose/pose_cov
  -> MAVROS -> PX4 EKF2

/Odometry + 关键帧库
  -> Scan Context + ICP
  -> /planning/odom + map -> camera_init
  -> /race/odom (标准 ROS ENU)

静态 PCD + 实时点云 + /race/odom
  -> Super A* 全局路径
  -> EGO B-spline 局部轨迹
  -> /race/navigation_setpoint 或 /race/ego/trajectory_setpoint
  -> offboard_waypoint_node
  -> /mavros/setpoint_raw/local
  -> MAVROS -> PX4
```

`map` 是统一的标准 ROS ENU 坐标系。世界 yaw 对齐只在定位/地图边界应用一次，默认
`world_yaw_alignment_rad=0.0`；MAVROS 与 PX4 NED 的转换集中在 Offboard 边界完成。

## 软件包

| 路径 | 职责 |
| --- | --- |
| `src/fr_lio` | 仓库内高频 LIO、点云输入门禁和高频里程计状态 |
| `src/px4_ros_com` | 里程计健康守卫、EV 健康监控、MAVROS vision bridge 与坐标补偿 |
| `Lin_shi/fastlio_global_slam` | Scan Context、ICP、关键帧地图保存/加载和全局重定位 |
| `src/race_mapping` | 地图加载、LIO 到统一 `map` 坐标的桥接与 TF |
| `src/race_super_planner_ros2` | Super A* 全局规划与路径跟踪策略 |
| `src/ego_planner_upstream` | EGO B-spline 局部规划算法 |
| `src/race_ego_bridge` | 里程计、点云、目标和 EGO 轨迹桥接 |
| `src/race_offboard` | MAVROS Offboard 控制、航点保存、任务状态机和任务行为 |
| `src/race_bringup` | 全导航栈 launch 与统一调参分发 |
| `src/race_msgs` | 规划和控制之间的自定义 ROS 消息 |

`Lin_shi/offboard_nav2_planning` 是已退役的 Nav2 路线，仅供历史参考，不属于当前导航主链。

## 运行环境

已知工作环境：

- Ubuntu + ROS 2 Humble
- PX4 + MAVROS 2，默认串口为 `serial:///dev/ttyUSB0:921600?ids=255,190`
- Livox MID-360 与 `livox_ros_driver2`
- 外部 Livox 工作区默认位于 `/home/robot/livox_mid360_env`
- `colcon`、PCL、Eigen3、OpenCV、`cv_bridge`、`message_filters`
- GNOME Terminal，用于一键真机栈的分组件窗口

FR-LIO 的 `ikd_tree v0.1.0` 已随源码固定，不需要单独安装。完整来源和许可证见
[`src/fr_lio/UPSTREAM.md`](src/fr_lio/UPSTREAM.md)。

## 构建

最简构建入口会构建 `src` 与 `Lin_shi` 中的软件包，并启动不连接 MAVROS/PX4 的入口：

```bash
cd /home/robot/egohx_ws/26-px4_offboard_control-main
LIVOX_TRACE_PREFIX=/home/robot/livox_mid360_env/ws_livox/install/livox_trace_control_interfaces \
./tools/一键编译并启动.sh
```

只启动控制输出关闭的 Super Planner：

```bash
LIVOX_TRACE_PREFIX=/home/robot/livox_mid360_env/ws_livox/install/livox_trace_control_interfaces \
START_MODE=planner ./tools/一键编译并启动.sh
```

手动构建时必须先加载 ROS 2、Livox trace 接口和 Livox 驱动 overlay：

```bash
cd /home/robot/egohx_ws/26-px4_offboard_control-main
source /opt/ros/humble/setup.bash
source /home/robot/livox_mid360_env/ws_livox/install/livox_trace_control_interfaces/share/livox_trace_control_interfaces/package.bash
source /home/robot/livox_mid360_env/ws_livox/install/setup.bash
PYTHONNOUSERSITE=1 colcon build --base-paths src Lin_shi --symlink-install
source install/setup.bash
```

当前一键构建脚本仍保留旧的 trace 接口默认路径，因此在本工作区需要像上例一样设置
`LIVOX_TRACE_PREFIX`。如 Livox 工作区也不在默认位置，再通过 `LIVOX_WS` 覆盖。

## 主要入口

### 1. 输出关闭的导航检查

该模式会访问真实传感器和 MAVROS，但禁止任务与控制输出，适合拆桨联调：

```bash
NO_MAP_VALIDATION=true \
ENABLE_OUTPUT=false \
MISSION_ENABLED=false \
REQUIRE_FLIGHT_READY_FOR_STARTUP=false \
RVIZ=false \
./tools/flight/一键启动导航栈.sh
```

设置 `LIO_BACKEND=fast_lio` 可切换到外部 FAST-LIO；未设置时当前默认是 `fr_lio`。

### 2. 建图与重定位库

生成现场 PCD：

```bash
./tools/fastlio/现场扫图并保存地图.sh
```

生成包含 `metadata.csv` 和 `keyframes/` 的全局重定位库：

```bash
./tools/fastlio/现场全局建图并保存重定位库.sh
```

仅验证重定位和规划，不连接飞行控制输出：

```bash
./tools/fastlio/一键重定位并规划验证.sh
```

### 3. 保存航点

```bash
./tools/flight/一键保存航点.sh
```

启动后按提示重定位，再在目标位置按回车保存。前四个采样点命名为 `B1`、`B2`、`C`、
`D`，后续为 `P005`、`P006` 等；任务路线会使用所有有效航点。每次采集保存在日期目录，
当前应用同步到 `src/race_offboard/config/waypoints/main/`。

如规划地图不在 `maps/main`，必须同时指定同一地图和重定位库：

```bash
MAP_FILE=/absolute/path/course.pcd \
FRLIO_GLOBAL_MAP_DIR=/absolute/path/map_database \
./tools/flight/一键保存航点.sh
```

详细说明见 [`docs/航点状态机飞行.md`](docs/航点状态机飞行.md)。

### 4. 航点状态机

当前推荐的任务入口由飞手先在 `POSITION/POSCTL` 起飞，随后手动切入 `OFFBOARD`：

```bash
./tools/flight/运行航点状态机.sh --manualtakeoff --trosbag --noego
```

该命令使用 Super A* 直控路线并录制任务专项 rosbag。去掉 `--noego` 可使用默认
`astar_ego` 后端；`--norosbag`、`--lowrosbag` 和 `--trosbag` 可选择录包策略。

状态机只发布 `/goal_pose` 和受保护的航向请求，不直接解锁、切换模式或发布 PX4
setpoint。离开 `OFFBOARD` 后任务暂停，重新进入时不会静默续飞。

### 5. 单目标验证

```bash
./tools/flight/人工起飞后Offboard单目标验证.sh
```

该入口强制执行全局重定位；成功并锁定 `map -> PX4 local` 变换后，才允许飞手切入
`OFFBOARD` 并在 RViz 发布一个近距离目标。

### 6. 数据记录

一键流程默认将记录写入：

```text
flight_records/YYYYMMDD/flight_YYYYMMDD_HHMMSS/
├── logs/
├── rosbag/
└── snapshot/
```

常用命令：

```bash
./tools/rosbag/查看最近录包.sh
./tools/rosbag/停止录包.sh
./tools/rosbag/刷新录包状态.sh
```

## 统一配置

| 文件 | 用途 |
| --- | --- |
| `src/race_bringup/config/astar_ego_tuning.yaml` | Super、EGO、桥接和 Offboard 的统一规划/安全参数 |
| `src/race_offboard/config/navigation.yaml` | Offboard 状态机、健康门禁、高度与 setpoint 参数 |
| `src/race_offboard/config/waypoints/main/waypoints.yaml` | 当前采集航点、地图路径和地图哈希 |
| `src/race_offboard/config/waypoints/main/mission.yaml` | 当前路线顺序、停留时间和到点行为 |
| `src/px4_ros_com/config/mid360_lever_arm.conf` | MID-360 安装位置与方向补偿 |
| `src/fr_lio/config/indoors.yaml` | FR-LIO 室内参数 |

`astar_ego_tuning.yaml` 是规划链的参数单一来源。修改后必须完整重启，禁止在飞行中修改
安全边界、障碍膨胀或动力学限制。

## 实飞使用边界

- 当前配置已经通过实飞验证，工程状态为 `FLIGHT_GO_NO_GO=GO`。
- 实飞结论只适用于当前硬件安装、源码、地图、航点和参数组合；修改其中任一项后应重新执行对应级别的验证。
- `/mavros/setpoint_raw/local` 必须只有 `offboard_waypoint_node` 一个发布者；节点发现冲突会拒绝启动。
- 起飞前要求 MAVROS、PX4 本地位置、EV 健康和 `/ev_health/flight_ready` 连续满足门禁。
- 全局重定位失败、地图/航点哈希不匹配、时间戳回退、位置跳变或关键输入过期时，流程应失败关闭。
- 飞行中 EV `FAULT` 默认请求 `AUTO.LAND`；高度边界、点云失效和 setpoint 超时还有独立保护。
- `ENABLE_OUTPUT=false` 才表示规划控制输出关闭；启动真实雷达或 MAVROS 不等于仿真。
- 每次实飞仍须完成飞前检查，并由飞手全程握持遥控器准备接管；硬件或关键配置变更后的首次验证应先拆桨进行。

后续维护重点：

- 为已通过的实飞补充固定格式的验收报告，记录飞行日期、源码提交、地图与航点哈希、参数快照、rosbag 和关键指标。
- 在继续修改任务、规划或控制代码前冻结当前可飞版本，确保后续改动可以与实飞基线准确比较和回退。
- 持续维护功能测试与代码质量检查，后续规划或控制改动必须保持测试通过。
- 历史状态文档仍包含旧工作区路径和旧 `NO_GO` 阶段结论，仅作为历史证据，不代表当前工程状态。

## 验证

当前验证结论与保留证据：

- 当前硬件、地图、航点和参数组合已经通过实飞验证，工程状态为 `GO`。
- `flight_records/20260822/` 保留了多轮运行与实飞的日志、rosbag 和运行快照；航点状态机记录包含多航点到达和 `P006` 航向行为完成事件。
- `validation_records/prop_off_ev_20260812_075818/validation_report.md`：实飞前的拆桨 EV 坐标、健康链与静态检查总体 `PASS`；其中记录的速率门禁属于当时阶段结果，不覆盖当前实飞 `GO` 结论。
- 当前构建树的功能测试覆盖 FR-LIO、EV 健康、坐标转换、起飞交接、航点状态机、地图绑定、航向覆盖、Super 路径策略和终点减速。
- 当前相关软件包的功能测试与代码质量检查均已通过。

常用验证命令：

```bash
colcon test --packages-select \
  fr_lio px4_ros_com race_bringup race_ego_bridge race_offboard race_super_planner_ros2
colcon test-result --all --verbose
```

调参文件可单独做结构校验：

```bash
python3 -c "import sys; sys.path.insert(0, 'src/race_bringup/launch'); from astar_ego_tuning import load_tuning; load_tuning('src/race_bringup/config/astar_ego_tuning.yaml'); print('OK')"
```

## 文档索引

- [`docs/航点状态机飞行.md`](docs/航点状态机飞行.md)：保存航点、状态机和任务行为
- [`docs/预设航点任务.md`](docs/预设航点任务.md)：预设航点任务配置与入口
- [`docs/全局路径首次交接修改总结与回退说明.md`](docs/全局路径首次交接修改总结与回退说明.md)：全局路径首次交接策略
- [`start.md`](start.md)：建图、重定位和单目标操作说明
- [`tools/脚本清单.md`](tools/脚本清单.md)：脚本用途与安全分类
- [`docs/坐标转换.md`](docs/坐标转换.md)：坐标、安装方向和杆臂说明
- [`docs/VALIDATION_MATRIX.md`](docs/VALIDATION_MATRIX.md)：历史验证证据边界
- [`docs/archive/`](docs/archive/)：已退役方案和 README 历史

## 目录说明

```text
src/                 当前 ROS 2 源码
Lin_shi/             重定位后端及已退役 Nav2 包
tools/               构建、飞行、建图、检查、录包和分析脚本
docs/                当前专题文档与历史归档
maps/                本机地图和关键帧库，不进入 Git
flight_records/      飞行日志、rosbag 和快照，不进入 Git
runtime/             临时运行状态，不进入 Git
validation_records/  保留的验证报告
rollback_backups/    历史回退材料，不参与当前运行
```
