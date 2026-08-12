# ws_offboard_control

> 更新时间：2026-08-12
> 适用工作区：`~/rong_ws/ws_offboard_control`
> 当前判定：`FLIGHT_GO_NO_GO = NO`（判定门限与依据见 [docs/VALIDATION_MATRIX.md](docs/VALIDATION_MATRIX.md)）

> **当前状态（2026-08-09）**
>
> EV 速度链的两处 twist 坐标系缺陷已修（vision bridge 与 ego bridge），
> 代码已重编并通过功能测试，但**尚未实测确认**；`EKF2_EV_CTRL=11` 保持不变，
> 仍不融合 EV 速度。详见 [视觉速度融合验收.md](视觉速度融合验收.md)。
>
> EV 链已收敛为唯一通路，多余的两个 PX4 EV 写者已从源码删除：
>
> ```text
> MID-360 -> FAST-LIO -> /Odometry -> fastlio_odometry_guard
>   -> /Odometry/healthy -> fastlio_mavros_vision_bridge
>   -> /mavros/vision_pose/pose_cov -> MAVROS -> PX4 EKF2
> ```
>
> 唯一性现在由源码保证（只有一个 EV 写者可被构建和返回），并由
> `test_ev_odometry_acceptance.py` 的结构化契约校验，不再依赖 launch 参数互斥。
>
> `坐标转换.md` 不再是权威文档。坐标与安装参数以当前 launch/config、桥接实现和
> 静态契约测试为准。脚本分类见 [脚本清单.md](脚本清单.md)。安全边界见
> [docs/CURRENT_STATE.md](docs/CURRENT_STATE.md)。
>
> 高频里程计那条路已判定不采用（`HIGH_RATE_PROMOTION_AUTHORIZED=NO`、B2 的
> 90 秒验证 FAIL），相关产物与脚本已清理，历史记录见
> `docs/archive/README_history.md`。
>
> 本文是工程说明，不构成飞行授权。实飞前按 脚本清单的安全分类逐项确认。

本文档记录当前导航无人机工程的自动起降进度、默认链路、启动方法和已知边界。

## 最新关键改动（2026-08-12）

- **EV 速度坐标系已修正。** `fastlio_mavros_vision_bridge` 现在将 FAST-LIO 按
  REP-147 定义、位于 child frame 的线速度及其协方差旋转到局部世界 ENU 后，再发布
  `/mavros/vision_speed/speed_twist_cov`；该话题的 `frame_id` 同步为 `odom`。非有限
  速度样本会被丢弃，不会送入 MAVROS。`EKF2_EV_CTRL` 仍为 `11`，因此 PX4 目前不融合
  EV 速度，速度链仅用于诊断和后续拆桨验收。
- **EV 健康链加强。** 健康监控继续从原始 `/Odometry` 生成唯一的
  `/Odometry/healthy`，并新增内部速度一致性、时间对齐历史和世界系速度协方差下限检查；
  FAST-LIO 未知角速度在具有足够大协方差时可通过守卫，避免将“未知”误判为故障。
- **FAST-LIO 机体安装 yaw 已由严格无桨录包定标。** 2026-08-12 的完整动作录包实测
  原始 `/Odometry.twist` 为 `+X` 机头、`+Y` 左、`+Z` 上；此前按外壳安装图加入的
  `pi` 补偿会把 `/race/odom` 前进和左移同时取反。默认
  `body_to_fastlio_yaw_rad` 已改为 `0`，静态 `base_link -> body` 也使用单位旋转。
- **世界 yaw 已统一为一个参数。** `world_yaw_alignment_rad` 同时作用于
  `/race/odom` 的位置、姿态和协方差，`map -> camera_init` TF、
  `/mavros/vision_pose/pose_cov` 以及地图/规划器使用的 `map` 坐标。`+1.57079632679 rad`
  候选已被 2026-08-12 的 RViz 实测否决：`Vehicle Odom` 箭头会指向真实机头左侧。
  当前默认值恢复为 `0.0 rad`，且不再保留 EGO 内部旧的固定 `+90 deg` 转换。机体安装参数仍独立为 `body_to_sensor_{x,y,z}_m` 和
  `body_to_fastlio_yaw_rad=0`，不能用安装 yaw 代替世界对齐。
- **唯一写者与放行结论不变。** PX4 的 EV 输入仍只有
  `/Odometry/healthy -> fastlio_mavros_vision_bridge -> /mavros/vision_pose/pose_cov`；
  `offboard_waypoint_node` 仍是 `/mavros/setpoint_raw/local` 的唯一发布者。以上改动
  已有源码契约和功能测试覆盖，但尚未完成实机验证，`FLIGHT_GO_NO_GO` 维持 `NO`。
- **17:13/17:15 场次不能用于坐标放行。** 两个启动快照都被旧环境变量
  覆盖为 `WORLD_YAW_ALIGNMENT_RAD=+1.57079632679`；包内证明位置和姿态确实同时
  旋转了 `+90 deg`，不存在“只转位置”的 TF 实现错误。17:15 场次中 FAST-LIO
  连续报告传感器时间回退并清空缓冲，后续位置发散到千米级。下一轮严格无桨
  录包已增加 `/livox/lidar` 和 `/livox/imu` 源时间戳检查；任意回退都会判定失败。
- **全局重定位 ICP 现在使用 Scan Context 粗航向。** `sector_shift` 按描述子的
  `360 / scan_context_sectors` 角度分辨率转换为 yaw，作为当前扫描到候选关键帧子图的
  ICP 初值。日志同时输出 `sector_shift`、粗 yaw、精配准 yaw/平移和 fitness；
  `relocalization_fitness_threshold=0.45` 保持不变，不通过放宽阈值接受错误配准。
- **人工起飞单目标入口现已加入强制全局重定位。** 执行
  `./tools/flight/人工起飞后Offboard单目标验证.sh` 后，整栈只启动一套 MID-360 和
  FAST-LIO，加载 `maps/fastlio_global_3d_20260810` 的关键帧库并调用一次 Scan Context +
  ICP。只有服务返回 `success=true` 后，重定位桥才发布 `/planning/odom`，导航桥再由它
  生成 `/race/odom`；动态 `map -> camera_init` 取代原固定 TF，静态规划地图直接标记为
  `map`。MAVROS/EV 同样读取 `/planning/odom`，避免规划与 PX4 使用不同位姿源。
- **重定位实飞增加两级失败门禁。** ICP 失败时 MAVROS/EV 不会启动，脚本也不会显示
  人工起飞步骤；ICP 成功后 Offboard 会由同步的 `/race/odom` 与
  `/mavros/local_position/pose` 锁定静态 `map -> PX4 local` 平面刚体变换，并将 Super/EGO
  的位置、速度、加速度和 yaw 统一转换后才发布。该变换未就绪时整栈不会 PASS。rosbag
  新增 `/planning/odom`、`/fastlio_global/relocalized_pose`、`/tf` 和 `/tf_static`。
  该链路目前只完成代码与构建验证，尚未完成新的拆桨现场重定位验证，因此顶部
  `FLIGHT_GO_NO_GO = NO` 不变。

## 工程目标

本工程用于把 `MID-360 + FAST-LIO` 的里程计送入 `PX4`，通过 `MAVROS` 给 PX4 提供外部视觉定位，并在此基础上验证室内 Offboard 自动起飞、悬停、降落和飞行数据记录。

当前主链路：

```text
MID-360
  -> livox_ros_driver2
  -> FAST-LIO
  -> /Odometry
  -> Scan Context + ICP（人工起飞单目标入口强制启用）
  -> /planning/odom (map)
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

## 当前起飞与外部降落控制链路（2026-08-07）

本节描述当前源码的控制接口，不改变本文档顶部的历史门禁结论，也不构成飞行授权。

当前完整链主入口为 [一键启动导航栈.sh](tools/flight/一键启动导航栈.sh)，控制脚本为
[运行导航控制.sh](tools/flight/运行导航控制.sh)，节点为
[offboard_waypoint_node.cpp](src/race_offboard/src/offboard_waypoint_node.cpp)。脚本名保留
`1m` 历史命名，实际默认相对起飞高度为 `0.8 m`。

```text
MID-360 -> FAST-LIO -> /Odometry -> MAVROS vision_pose -> PX4 EKF2
                                                    |
                                                    v
                      offboard_waypoint_node -> /mavros/setpoint_raw/local
                                                    |
                                                    v
                                           PX4 OFFBOARD: 0.8 m 定点
                                                    |
                         外部 RC / GCS / 监督节点 -> PX4 AUTO.LAND
```

### 启动与起飞流程

`./tools/flight/一键启动导航栈.sh` 先预启动导航控制节点，再依次启动 MID-360、
FAST-LIO、MAVROS/EV 和 rosbag，并在每一级收到真实消息或健康状态后才继续。导航预热必须
早于实时传感器链，避免集中创建节点使 FAST-LIO 积压旧扫描。MID-360 启动后
默认至少等待 `4 s`，且 IMU 和点云话题均就绪后才启动 FAST-LIO；延时可用
`MID360_FASTLIO_DELAY_SEC` 覆盖。控制节点完成以下状态流转：

```text
WAIT_FOR_CONNECTION
  -> WAIT_FOR_LOCAL_POSE
  -> STREAM_SETPOINTS
  -> WAIT_FOR_RC_OFFBOARD
  -> REQUEST_ARM
  -> HOLD (相对起点 0.8 m 定点)
```

- `WAIT_FOR_RC_OFFBOARD` 持续发布预发送 setpoint，等待外部操作将飞控切入 `OFFBOARD`。
- 确认 OFFBOARD 后等待 `20 s`，再请求解锁。
- 节点以最近 `30` 个 `/mavros/local_position/odom` 样本建立飞控本地起飞参考点，锁定 XY 与当前航向；
  `target_z_m=-0.80` 使用 NED 语义，表示向上 `0.8 m`。
- 到达目标后持续发布 `/mavros/setpoint_raw/local` 的固定位置设定值。当前没有终端
  `land` 命令，也没有正常流程中的定时下降或自旋。

当前主脚本的控制参数如下：

```text
target_x_m=0.0
target_y_m=0.0
target_z_m=-0.80
z_ramp_seconds=2.5
prestream_count=100
recent_pose_samples=30
use_rc_offboard=true
offboard_stabilize_seconds=20.0
require_vision_pose=true
vision_freshness_s=1.0
require_ev_health=true
ev_health_required_s=7.5
ev_health_freshness_s=1.0
auto_land=false
offboard_land=false
land_on_offboard_loss=true
```

### 外部降落接口

正常降落由外部控制端直接让 PX4 进入 `AUTO.LAND`。对人工操控，优先使用遥控器飞行
模式开关或地面站；对程序化操控，外部监督节点可调用 MAVROS：

```bash
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode \
  "{base_mode: 0, custom_mode: 'AUTO.LAND'}"
```

外部 `AUTO.LAND` 从当前高度由 PX4 接管下降。控制节点检测到模式切换后会维持无爬升
setpoint 心跳、等待 `/mavros/extended_state` 报告落地，并在落地后等待 `2 s` 请求上锁；
扩展状态不可用时，`45 s` 后使用本地高度接近起飞参考地面的备用判据。

当前脚本的 `offboard_land=false`，因此 `offboard_land_auto_handoff_height_m=0.30` 不在
正常流程中生效。若外部控制器需要“外部触发、先缓降至相对地面 `0.3 m`、再切
`AUTO.LAND`”，它必须成为唯一的下降 setpoint 发布者，或为现有控制节点增加显式的
受控降落请求接口；不要与本节点同时向 `/mavros/setpoint_raw/local` 发布。

`land_on_offboard_loss=true` 是安全策略：飞机已解锁时离开 OFFBOARD，控制节点会请求
`AUTO.LAND`。外部控制端应显式请求 `AUTO.LAND`，不能把切换到手动模式当作通用控制权
交接。

### 接入接口与安全边界

| 方向 | ROS 接口 | 当前用途 |
|---|---|---|
| 输入 | `/mavros/state`、`/mavros/extended_state` | 连接、飞行模式、解锁与落地确认 |
| 输入 | `/mavros/local_position/odom` | PX4 本地起飞参考点、定点误差、落地备用判据（飞控边界） |
| 输入 | `/mavros/vision_pose/pose_cov` | 起飞前视觉新鲜度检查 |
| 输入 | `/ev_health/status` | 起飞前连续健康门禁与飞行中 FAULT 保护 |
| 输出 | `/mavros/setpoint_raw/local` | 唯一的 OFFBOARD 位置设定值发布者 |
| 服务 | `/mavros/set_mode` | 请求 OFFBOARD 或 AUTO.LAND |
| 服务 | `/mavros/cmd/arming` | 解锁与落地后上锁 |

- 起飞前必须满足 MAVROS 已连接、局部里程计与视觉位姿新鲜，并持续 `HEALTHY` 至少
  `7.5 s`；`SUSPECT` 会重新开始该预检计时。首次地面保持点和起飞 WARMUP 也使用同一个
  `ev_health_.ready()` 门禁，不能在 EV 刚变为 `HEALTHY` 时提前锁定 PX4 旧局部原点。
- PX4 局部原点发生大跳变时，只允许在 `armed=false`、主状态 `IDLE`、控制状态
  `IDLE_HOLD` 且 EV 仍 ready 的条件下候选重锁；新坐标还必须低速并连续稳定 `1.0 s`。
  armed 后禁止因局部原点跳变自动重锁，异常位置继续由飞行保护处理。
- 飞行中仅 `/ev_health/status=FAULT` 触发 EV 安全 `AUTO.LAND`；`SUSPECT` 或健康状态
  超时不会单独触发 EV 自动降落。
- 水平漂移保护仍独立生效：告警 `0.20 m`，持续 `0.30 s` 超过 `0.50 m` 请求降落，持续
  `0.10 s` 超过 `0.60 m` 进入紧急降落。
- 起飞后的前 `3 s` 还检查位移、水平速度、横滚和俯仰；任一指标持续超限时请求
  `AUTO.LAND`。默认阈值分别为 `0.50 m`、`0.50 m/s`、`10 deg`、`10 deg`。

节点内的 Nav2 路线跟踪（`RoutePathFollower`）已删除：融合后由 Super A* + EGO 承担
路径规划与跟踪。节点仍是 `/mavros/setpoint_raw/local` 的唯一发布者，这条不变式在
接入新规划器时必须保持。

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
world_yaw_alignment_rad = 0.0
body_to_fastlio_yaw_rad = 0.0
```

说明：

- MID360 使用内置 IMU，整机安装方向改变时不改 FAST-LIO 的 `extrinsic_R`。
- 位置和姿态必须属于同一个世界坐标系，因此共用一个世界 yaw 对齐参数。
- MID360 的固定安装 yaw 由 `body_to_fastlio_yaw_rad` 单独处理，不能写入世界 yaw 对齐。
- `/race/odom` 的 RViz 蓝色箭头表示补偿后的 `base_link +X`，应与真实机头一致；它不是
  TF Display 的坐标轴箭头。
- `map` 是统一的标准 ROS ENU 工作坐标。静态 PCD 和 FAST-LIO 点云保持
  `camera_init` 源 frame，由 `map -> camera_init` TF 变换到 `map`；目标点、Super、EGO
  均直接使用该 `map`，不再各自增加 yaw。
- `+90 deg` 候选已在实机 RViz 中造成 `Vehicle Odom` 箭头指向真实机头左侧，故已撤销。
  严格无桨录包在 `world_yaw=0` 时验证了前进 `x>0`、左移 `y>0`，因此当前默认保持 `0.0`。
  人工向机头前方平移时，RViz 蓝色箭头与位移方向仍必须同时对应真实机头前方才算通过。
- 通用导航/起飞入口会拒绝由旧终端环境继承的非零 `WORLD_YAW_ALIGNMENT_RAD`。
  起飞入口不提供覆盖；通用导航入口只在 `MISSION_ENABLED=false`、
  `ENABLE_OUTPUT=false` 且 `ALLOW_UNVALIDATED_WORLD_YAW=true` 时允许拆桨坐标实验。

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
cd ~/rong_ws/ws_offboard_control
MAP_FILE=/path/to/arena.pcd ./tools/flight/一键启动导航栈.sh
```

该脚本在原终端中按依赖顺序编排完整链路，并为 Navigation、MID-360、FAST-LIO、
MAVROS/EV 和 rosbag 分别打开独立 GNOME Terminal 窗口。每个窗口显示实时输出，同时把
日志写入本次 `flight_records/<date>/flight_<timestamp>/logs/`。

原终端是整栈 owner：在其中按 `Ctrl+C` 会按 PGID 清理全部组件及窗口。每个组件窗口也有
owner watchdog；即使 owner 被强制结束，也会自动终止本窗口管理的完整进程组，避免 ROS
launch 组长退出后留下孤立子节点。

一键启动前会运行：

```bash
./tools/checks/起飞前自检.sh
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
cd ~/rong_ws/ws_offboard_control
source /opt/ros/humble/setup.bash
source ~/rong_ws/ws_offboard_control/install/setup.bash
ros2 launch px4_ros_com fastlio_mavros_autofix.launch.py \
  fcu_url:=serial:///dev/ttyUSB0:921600?ids=255,190
```

终端 4：起飞控制

```bash
cd ~/rong_ws/ws_offboard_control
./tools/flight/运行导航控制.sh
```

## 数据记录

录包脚本：[开始录包.sh](tools/rosbag/开始录包.sh)

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
- `/race/odom`
- `/race/pose`
- `/tf`
- `/tf_static`
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
cd ~/rong_ws/ws_offboard_control
./tools/rosbag/查看最近录包.sh
```

手动停止录包：

```bash
cd ~/rong_ws/ws_offboard_control
./tools/rosbag/停止录包.sh
```

清理录包：

```bash
cd ~/rong_ws/ws_offboard_control
./tools/rosbag/删除历史录包.sh
./tools/rosbag/删除历史录包.sh latest
```

## 开机/登录后自启动

打开 GNOME 登录后自动启动：

```bash
cd ~/rong_ws/ws_offboard_control
./tools/autostart/启用开机自启.sh
```

关闭自启动：

```bash
cd ~/rong_ws/ws_offboard_control
./tools/autostart/禁用开机自启.sh
```

自启动入口会调用：

```bash
./tools/flight/开机自启动起飞栈.sh
```

该脚本登录后等待约 `10 s`，再执行 `tools/flight/一键启动起飞栈.sh`。

## 全局建图后端

当前工程新增了独立后端包：

- [fastlio_global_slam](Lin_shi/fastlio_global_slam)

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

### 重定位后实飞入口

人工起飞单目标验证脚本现在会按以下顺序运行：

```text
MID-360 -> FAST-LIO -> 加载关键帧库 -> Scan Context + ICP
        -> /planning/odom + 动态 map -> camera_init
        -> /race/odom -> MAVROS/EV -> Super/EGO -> Offboard
```

运行命令：

```bash
cd ~/rong_ws/ws_offboard_control
source /opt/ros/humble/setup.bash
source install/setup.bash
./tools/flight/人工起飞后Offboard单目标验证.sh
```

脚本不会同时调用两个完整启动脚本，也不会重复启动雷达或 FAST-LIO。重定位服务失败时
会在启动 MAVROS 前退出；成功后 Offboard 节点先以静止的 `/race/odom` 和 PX4 local pose
锁定 `map -> PX4 local` 平面变换，再允许脚本显示人工起飞、切 OFFBOARD 和点击目标的步骤。
实飞前仍必须先拆桨运行该入口，确认日志中的 `Relocalization succeeded`、
`[MAP_LOCAL_ALIGNMENT_LOCKED]`、实时点云/地图重合以及坐标方向正确。

## Nav2 路径规划与旧地图复用（已退役）

Nav2 这条线已整体退役：融合后导航由 Super A* 全局规划 + EGO B-spline 局部规划承担。

- `offboard_nav2_planning` 与相关脚本已移至 `Lin_shi/`
- `fastlio_global_slam` 也移至 `Lin_shi/`；它提供地图保存/加载/重定位，
  比赛规则允许赛前一天采图，该能力保留待评估
- 原操作步骤见 `docs/archive/README_nav2_history.md`

## 构建

修改 `px4_ros_com` 后：

```bash
cd ~/rong_ws/ws_offboard_control
source /opt/ros/humble/setup.bash
PYTHONNOUSERSITE=1 colcon build --packages-select px4_ros_com
source ~/rong_ws/ws_offboard_control/install/setup.bash
```

修改 `race_ego_bridge`（EGO 轨迹到 MAVROS 的桥）后同理换包名。`PYTHONNOUSERSITE=1`
是必需的：用户 site-packages 里有版本冲突的包会让 colcon 的 Python 扩展加载失败。

改了 FAST-LIO（`~/livox_mid360_env/ws_fastlio`）则必须限定包发现范围：

```bash
cd ~/livox_mid360_env/ws_fastlio
source /opt/ros/humble/setup.bash
source ~/livox_mid360_env/setup_mid360.bash
PYTHONNOUSERSITE=1 colcon build --base-paths src --symlink-install --packages-select fast_lio
```

不加 `--base-paths src`，遗留的 sanitizer 构建目录会让 colcon 报
`Duplicate package names not supported` 并中止。编译后用 `readlink -f` 核对真实
二进制的时间戳，`install/` 是符号链接树、它自己的 mtime 证明不了任何事：

```bash
ls -l "$(readlink -f ~/livox_mid360_env/ws_fastlio/install/fast_lio/lib/fast_lio/fastlio_mapping)" \
      ~/livox_mid360_env/ws_fastlio/src/fast_lio/src/laserMapping.cpp
```

`./诊断.sh --static` 已内置这项新旧检查。

`fastlio_global_slam` 已移至 `Lin_shi/`，不参与默认构建；它的 GTSAM 因子图需要
`libgtsam-dev`，本机未安装，代码会自动退化成无因子图模式。

## 核心文件

- MAVROS/PX4 默认 launch：[fastlio_mavros_autofix.launch.py](src/px4_ros_com/launch/fastlio_mavros_autofix.launch.py)
- Offboard 起降控制：[offboard_waypoint_node.cpp](src/race_offboard/src/offboard_waypoint_node.cpp)（`race_offboard` 包，由 `race_bringup/navigation.launch.py` 拉起，已不是 px4_ros_com 下的 Python 脚本）
- EGO 轨迹到 MAVROS 桥：[ego_odom_bridge.cpp](src/race_ego_bridge/src/ego_odom_bridge.cpp)
- MAVROS vision bridge：[fastlio_mavros_vision_bridge.cpp](src/px4_ros_com/src/bridges/fastlio_mavros_vision_bridge.cpp)
- 里程计保护节点：[fastlio_odometry_guard.cpp](src/px4_ros_com/src/bridges/fastlio_odometry_guard.cpp)
- 起飞脚本：[运行导航控制.sh](tools/flight/运行导航控制.sh)
- 悬停脚本：[运行导航控制.sh](tools/flight/运行导航控制.sh)
- 一键启动脚本：[一键启动导航栈.sh](tools/flight/一键启动导航栈.sh)
- 录包脚本：[开始录包.sh](tools/rosbag/开始录包.sh)
- 自启动检查：[起飞前自检.sh](tools/checks/起飞前自检.sh)
- 坐标参数契约：`mid360_lever_arm.conf`、MAVROS launch 与 EV 静态契约测试
- 脚本清单：[脚本清单.md](脚本清单.md)
- 验证方法与判定门限：[验证方法.md](验证方法.md)
- 无桨 EV 验证流程：[无桨视觉验证.md](无桨视觉验证.md)
- 视觉速度融合验收：[视觉速度融合验收.md](视觉速度融合验收.md)
- 速度坐标系取证结果：[速度坐标系取证.md](速度坐标系取证.md)
- twist 旋转数值校验器：[校验速度旋转.py](tools/analysis/校验速度旋转.py)
- 启动链路诊断：[诊断.sh](诊断.sh)
- 全局建图 launch：[fastlio_global_slam.launch.py](Lin_shi/fastlio_global_slam/launch/fastlio_global_slam.launch.py)
- 全局建图后端：[fastlio_global_backend.cpp](Lin_shi/fastlio_global_slam/src/fastlio_global_backend.cpp)

## 下一步建议

短期建议：

- 对最近一次完整起降 rosbag 做一次高度、水平误差、setpoint 与 EKF local position 的曲线复盘。
- 在受控测试中复核漂移保护阈值；当前达到 `drift_land_m` 后会请求安全 `AUTO.LAND`，不只是记录告警。
- 再做 3 到 5 次同参数重复起降，记录最大水平误差、目标高度误差、外部 `AUTO.LAND` 请求时刻和落地稳定性。

中期建议：

- 增加一份标准试飞检查单：桨叶/电池/定位/遥控器模式/PX4 参数/rosbag 状态/紧急接管流程。
- 将 `tools/rosbag/查看最近录包.sh` 纳入一键启动前提示，避免上一次录包异常被忽略。
- 在 README 中维护一个飞行记录表，把每次有效试飞的高度、耗时、是否完整降落、异常现象和 rosbag 路径记录下来。
