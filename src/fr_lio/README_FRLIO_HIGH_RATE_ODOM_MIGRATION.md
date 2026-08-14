# FR-LIO 高频 Odometry 接入 26-px4_offboard_control 修改方案

## 1. 目标与推荐边界

目标是在保留 `26-px4_offboard_control` 现有飞行安全链路的前提下，将 FR-LIO 的高频里程计能力接入当前系统：每次 LiDAR/IESKF 更新产生一个可信状态锚点，两帧 LiDAR 之间由 IMU 将状态传播到当前测量时刻，并以 IMU 频率输出 `nav_msgs/msg/Odometry`。

推荐的数据链路如下：

```text
MID-360 LiDAR + IMU
        |
        v
FAST-LIO/FR-LIO（LiDAR 校正 + IMU 高频传播）
        |
        | /Odometry，ROS ENU + body FLU，保留传感器时间戳
        v
fastlio_ev_health_monitor
        |
        | /Odometry/healthy
        v
fastlio_mavros_vision_bridge
        |
        | 杆臂补偿、安装角修正、ENU/FLU -> NED/FRD
        v
MAVROS -> PX4 EKF2
```

推荐只移植高频状态传播能力，不在第一阶段引入 FR-LIO 的回环、GTSAM、影子地图和地图修正。PX4 应使用连续的局部 `odom`，不能使用可能因回环而跳变的 `odom_map`。

## 2. 修改总览

| 项目 | 推荐方案 | 主要修改位置 |
|---|---|---|
| Odometry 话题 | 高频输出直接使用 `/Odometry`，同时把话题改为参数 | FR-LIO/FAST-LIO 发布节点、启动文件 |
| QoS | 整条实时 odom 链统一为 `SensorDataQoS`，depth 5 | 发布节点、guard、health monitor、MAVROS bridge |
| IMU 单位 | 内部统一 SI；增加显式参数，默认 `m/s^2`，禁止无条件乘 9.81 | IMU 回调和配置 |
| frame 命名 | 上游固定 `odom -> body`，均表示 ENU/FLU | LIO 配置、TF 启动文件 |
| 坐标转换 | LIO 内不做 NED/FRD；只在 MAVROS/PX4 边界转换一次 | `fastlio_mavros_vision_bridge` |
| 杆臂 | 继续由现有 vision bridge 补偿，LIO 内不重复补偿 | bridge 和 `mid360_lever_arm.conf` |
| 协方差/速度 | pose 在 ENU，twist 在 child/body FLU；填满线速度 3x3 协方差 | 高频传播和 health monitor |
| 激光累积 | 默认先关闭额外累积；确需累积时用时间窗和锚点年龄约束 | accumulator 和 launch |
| 构建依赖 | 第一阶段不引入 GTSAM；只移植独立传播模块 | 现有 FAST-LIO 包/CMake |
| 时间戳 | 永远保留测量时间；增加 LiDAR 锚点年龄检查，禁止 restamp | LIO、health monitor、bridge |

## 3. `/odom` 到 `/Odometry` 的话题接口

### 修改方案

在 LIO 节点增加参数：

```yaml
publish:
  odom_topic: /Odometry
  odom_unfiltered_topic: /Odometry/unfiltered
  map_odom_topic: /Odometry/map
```

不要在 C++ 中写死 `"odom"`。创建 publisher 时读取参数：

```cpp
pub_odom_ = create_publisher<nav_msgs::msg::Odometry>(
    odom_topic_, odom_qos);
```

`26-px4_offboard_control` 继续保持：

```text
fastlio_odom_topic=/Odometry
healthy_odom_topic=/Odometry/healthy
```

短期也可以通过 launch remap 将 `/odom` 映射到 `/Odometry`，但推荐参数化，因为诊断脚本、录包和起飞入口已经把 `/Odometry` 作为正式契约。

### 验证

```bash
ros2 topic info -v /Odometry
ros2 topic hz /Odometry
ros2 topic echo /Odometry --once
```

验收条件：只有一个正式 `/Odometry` 发布者，不允许旧 FAST-LIO 与新高频发布器同时写入。

## 4. Publisher/Subscriber QoS

### 修改方案

高频数据应优先保证新鲜，避免可靠队列积压后把旧位姿送入 PX4。推荐整条实时链使用：

```cpp
auto odom_qos = rclcpp::SensorDataQoS().keep_last(5);
```

需要同步修改：

1. LIO `/Odometry` publisher。
2. `fastlio_odometry_guard` 的输入和输出。
3. `fastlio_ev_health_monitor.py` 的 `/Odometry` 输入和 `/Odometry/healthy` 输出。
4. `fastlio_mavros_vision_bridge` 的输入。

Python 使用 `qos_profile_sensor_data` 或等价的 `BEST_EFFORT + KEEP_LAST` 配置。诊断、状态和故障话题仍可使用 reliable；这里只统一高频 odom 数据面。

队列深度建议为 5，而不是 20。200 Hz 输入下 depth 20 代表最多 100 ms 排队空间，不适合外部视觉实时入口。

### 验证

用 `ros2 topic info -v` 确认每一对 publisher/subscriber 的 reliability、durability 和 depth 兼容；同时测量：

```text
now - /Odometry.header.stamp
now - /Odometry/healthy.header.stamp
```

正常运行建议 P99 小于 50 ms，硬门限继续保持 250 ms。

## 5. IMU 加速度单位

### 问题

FR-LIO 当前在 IMU 回调中无条件执行：

```cpp
linear_acceleration *= 9.81;
```

如果 Livox 驱动已经按 ROS `sensor_msgs/Imu` 规范输出 `m/s^2`，该操作会把加速度错误放大 9.81 倍。

### 修改方案

内部传播模型统一使用 SI：

```text
加速度：m/s^2
角速度：rad/s
时间：s
位置：m
速度：m/s
```

增加显式参数：

```yaml
imu:
  acceleration_unit: mps2   # 可选 mps2 或 g
```

只有配置为 `g` 时才乘标准重力。启动时打印单位和静止加速度模长；若静止均值不在合理范围内，应拒绝启用高频输出，而不是自动猜测。

建议判据：

```text
mps2 模式：静止 |a| 应接近 9.81
g 模式：原始静止 |a| 应接近 1.0，换算后接近 9.81
```

注意：单位转换必须只做一次，主 IESKF 和高频传播要读取同一份已归一化 IMU 数据。

## 6. body/frame 命名

### 推荐契约

```text
Odometry.header.frame_id = odom
Odometry.child_frame_id  = body
pose：body 原点在 odom/ENU 世界系中的位姿
twist：body/FLU 坐标系表达的速度
```

`body` 在这里是 FAST-LIO 的状态原点/IMU 原点，不是经过杆臂补偿后的飞控控制中心。现有 vision bridge 再将它变换到 `base_link` 控制原点。

不要同时把 LIO 的 `body_frame` 改成 `base_link` 又在 bridge 中应用 `body_to_sensor` 杆臂，否则会补偿两次。

建议整理 TF：

```text
odom -> body                 LIO 动态 TF，可选发布
base_link -> body            已标定的静态安装 TF
body -> body_frd             PX4 边界辅助 TF
```

必须保证 `odom -> body` 只有一个发布者。若规划栈或 MAVROS 已发布同一 TF，则关闭 LIO 的 `publish.tf_en`。

## 7. ENU/NED 与 FLU/FRD 转换边界

### 唯一转换原则

LIO、规划和 ROS 侧统一使用：

```text
世界系：ENU，X 东、Y 北、Z 上
机体系：FLU，X 前、Y 左、Z 上
```

PX4 使用：

```text
世界系：NED，X 北、Y 东、Z 下
机体系：FRD，X 前、Y 右、Z 下
```

转换只允许发生在 `fastlio_mavros_vision_bridge/MAVROS` 边界。LIO 发布 `/Odometry` 时不得提前做 ENU -> NED 或 FLU -> FRD，否则 MAVROS 会再次转换。

高频 odom 的速度约定保持：

```text
v_body_flu = R_world_body^T * v_world_enu
```

现有 vision bridge 再把 body 速度旋转到 world ENU，交给 MAVROS 的 vision speed 接口。不要把 `/Odometry.twist` 改成 world ENU 而不同时修改 health monitor 和 bridge。

### 验证动作

无桨固定机体做三个方向动作：向前、向左、向上。原始 `/Odometry.twist.linear` 应分别主要表现为 `+X`、`+Y`、`+Z`；PX4 NED 侧应对应前向机体速度以及世界轴的正确换算。

## 8. 杆臂补偿

### 修改方案

继续使用当前唯一标定源：

```text
src/px4_ros_com/config/mid360_lever_arm.conf
```

参数含义保持为：从飞控控制中心到 FAST-LIO 状态原点的向量，表达在 aircraft body FLU 中。

FR-LIO 的 `mapping.extrinsic_T/R` 是 LiDAR 与 IMU 之间的外参，用于点云去畸变和 LIO 状态估计；`body_to_sensor_*` 是飞控控制中心与 LIO 状态原点之间的杆臂。两者不是同一个量，不能相加后在两处分别使用。

推荐职责：

```text
LIO：只处理 LiDAR <-> IMU 外参
vision bridge：只处理控制中心 <-> LIO 状态原点杆臂和安装 yaw
```

高频传播输出仍表示 LIO/IMU 原点。杆臂补偿后的位置应由现有 bridge 生成。

## 9. 协方差和速度语义

### Pose 协方差

从最近一次 LiDAR 校正状态的 EKF 协方差出发，将误差传播到当前 IMU 时刻：

```text
P_now = F * P_anchor * F^T + Q_imu
```

推荐实际实现连续传播完整误差状态协方差，而不是只在发布时构造一个 6x23 近似 Jacobian。最终抽取 `[position, attitude]` 的 6x6 块，按 ROS 顺序写入 `pose.covariance`：

```text
x, y, z, roll, pitch, yaw
```

协方差必须与 pose 使用相同表达坐标；进行世界对齐、安装角或杆臂变换时，bridge 必须同步用 Jacobian 转换协方差。

### Twist 语义

```text
twist.linear：body/FLU 中的线速度
twist.angular：body/FLU 中、已去陀螺零偏的角速度
```

必须填充 `twist.covariance` 的线速度 3x3 块，不能让默认全零表示“无限可信”。推荐从传播状态的速度协方差旋转到 body：

```text
P_v_body = R_world_body^T * P_v_world * R_world_body
```

并保留交叉项。角速度有两种合法做法：

1. 发布有限角速度，并给出陀螺噪声与 bias 协方差推导出的有限方差。
2. 不提供角速度时发布 `NaN`，并把角速度方差设为大值，例如 `1e6`。

不能发布有限角速度却保留零协方差，也不能发布 `NaN` 而不设置高方差。现有 guard 同时兼容“有限角速度”以及“NaN + 高方差”。

### 滤波

建议 `/Odometry` 首先发布未额外低通的滤波状态；如确需平滑，应使用时间常数或截止频率计算每帧 alpha，不能用一个固定 alpha 同时处理姿态、线速度和角速度。任何滤波造成的群延迟都要进入时延验收。

## 10. 激光累积策略

### 推荐方向

第一阶段关闭 FR-LIO 额外的 10 包累积，直接沿用当前生产 FAST-LIO 的 MID-360 输入节奏。原因是累积会同时增加：

- LiDAR 校正周期；
- 高频传播锚点年龄；
- 点云处理峰值负载；
- 外部视觉的模型外推距离。

如果现场点密度不足，累积器应改为可配置的时间窗，而不只是固定包数：

```yaml
lidar_accumulator:
  enabled: false
  max_packets: 1
  max_window_ms: 50
```

满足任一上限就发布，并记录：实际窗口、点数、LiDAR 结束时间和处理延迟。输出消息的时间基准必须覆盖全部点，不能把累积包错误标记为最后一个包的单一开始时间。

高频传播另设 `max_anchor_age_s`。即使 IMU 仍在更新，只要最后一次有效 LiDAR 校正过旧，也必须停止健康发布或显著增大协方差。

## 11. GTSAM、ikd-tree、Livox 等构建依赖

### 推荐的第一阶段

不要把整个 FR-LIO 包复制进 `26-px4_offboard_control`。当前生产入口从外部环境启动 FAST-LIO；高频传播应做成一个与 GTSAM 无关的小模块，合入实际运行的 FAST-LIO 包：

```text
HighRateOdomPropagator
  resetFromCorrectedState(...)
  propagateImu(...)
  buildOdometry(...)
```

最小依赖应只有：

```text
rclcpp
sensor_msgs
nav_msgs
geometry_msgs
Eigen3
FAST-LIO 已有的状态和 SO(3) 工具
```

这样不会为了高频 odom 引入 GTSAM、TBB、FR-LIO 回环线程和地图修正依赖。

### 若后续完整引入 FR-LIO

应将依赖按功能拆分：

```text
fr_lio_core：LiDAR/IMU IESKF、PCL、Eigen、ikd-tree、Livox
fr_lio_high_rate_odom：只依赖 core 状态接口和 ROS 消息
fr_lio_loop_closure：GTSAM、TBB，可选构建
```

使用 CMake 选项控制回环：

```cmake
option(FR_LIO_ENABLE_LOOP_CLOSURE "Build GTSAM loop closure" OFF)
```

只有该选项开启时才 `find_package(GTSAM REQUIRED)`。Livox 消息依赖应与当前 MID-360 驱动版本保持一致，不能在同一进程混用不兼容的 `livox_ros_driver` 和 `livox_ros_driver2` 消息。

## 12. MAVROS 健康链路的时间戳约束

### 时间戳原则

```text
/Odometry.header.stamp = 当前被积分 IMU 样本的测量时间
禁止使用 now() 覆盖旧测量时间
禁止 bridge restamp
所有状态传播都使用传感器时间差
```

现有 `restamp_message=false` 必须保留。健康检查继续拒绝：

- 非单调时间戳；
- 输入年龄大于 0.25 s；
- 时间戳领先系统时间超过 0.05 s；
- 超过 0.5 s 没有消息。

### 必须新增的锚点年龄约束

仅检查 odom 消息时间戳不够。IMU 可以持续产生“当前时间”的消息，但它可能来自一个已经失去 LiDAR 校正很久的旧锚点。因此高频发布器必须计算：

```text
anchor_age = imu_stamp - last_valid_lidar_update_stamp
```

推荐初值：

```yaml
high_rate_odom:
  warn_anchor_age_s: 0.15
  max_anchor_age_s: 0.40
```

处理建议：

```text
anchor_age <= warn：正常发布
warn < anchor_age <= max：继续发布，但按时间增长协方差并标记 SUSPECT
anchor_age > max：停止 `/Odometry/healthy`，触发 FAULT，不得仅滚动 IMU 锚点后继续伪装成 LiDAR 有效
```

建议新增诊断话题：

```text
/frlio/high_rate_odom/status
/frlio/high_rate_odom/anchor_age
```

并让 `fastlio_ev_health_monitor` 可选订阅状态，作为有效 LiDAR 更新门控。FR-LIO 当前 `dt > 0.5 s` 时将纯 IMU传播结果写回锚点的逻辑不能作为“LiDAR 锚点刷新”。

## 13. 推荐实现顺序

### 阶段 A：离线实现

1. 在实际生产 FAST-LIO 分支加入独立传播器，不启用 PX4 输出。
2. 增加参数化 `/Odometry/high_rate_test`，与当前 `/Odometry` 同时录包比较。
3. 统一 SI 单位、frame 和 twist 语义。
4. 实现完整顺序 IMU 积分：每条 IMU 从上一传播时刻积分；每次 LiDAR 校正后原子地重置传播状态。
5. 实现 pose/twist 协方差和 anchor-age 诊断。

### 阶段 B：无桨链路验证

1. 将健康链路输入临时切换到 `/Odometry/high_rate_test`。
2. 验证 QoS、时间戳、轴向、杆臂和协方差。
3. 静置、单轴平移、单轴转动、遮挡 LiDAR、停止 IMU，逐项确认故障状态。
4. 确认没有重复 `/Odometry` 发布者和重复 TF。

### 阶段 C：替换正式入口

1. 高频输出切换到 `/Odometry`。
2. 保留 `/Odometry/healthy -> MAVROS -> PX4` 单一 EV 写入链。
3. 更新起飞前自检和 rosbag 记录项，加入 anchor age、输入频率、P99 时延和协方差范围。
4. 先做无桨验证，再做系留/低风险飞行验证，不直接进入自主任务。

## 14. 验收标准

| 检查项 | 建议标准 |
|---|---|
| `/Odometry` 频率 | 跟随有效 IMU 频率，频率稳定且无重复时间戳 |
| 源消息时延 | P99 < 50 ms，任何样本 < 250 ms |
| 时间戳 | 严格单调，不 restamp，不明显超前系统时钟 |
| LiDAR 锚点 | 正常时低于 warn；超过 max 必须退出 HEALTHY |
| 静置漂移 | 不因单位错误产生持续加速或快速位置漂移 |
| 速度坐标 | body FLU：前/左/上分别为 +X/+Y/+Z |
| 协方差 | 有限、对称、半正定，不能默认全零 |
| QoS | 所有实时 odom pub/sub 兼容，无启动积压追赶 |
| 杆臂 | 只补偿一次，绕轴转动时控制中心位置无明显圆周摆动 |
| PX4 输入 | 只有 `/Odometry/healthy` 一条正式 EV 路径 |
| LiDAR 中断 | 在设定 anchor 超时后进入 SUSPECT/FAULT |
| IMU 中断 | 在 message timeout 后进入 FAULT |

## 15. 已确认方向与当前实现状态

已确认：`1A 2A 3C 4A 5A 6A 7A 8A 9A 10A`。

当前 `fr-lio` 工作区补丁已完成：

- 新增独立的 18 状态高频传播器，逐条中值积分 IMU，并传播完整协方差。
- LiDAR 校正后按校正时刻插值 IMU，并回放该时刻之后的历史样本。
- `/Odometry` 参数化为正式输出，使用 `SensorDataQoS().keep_last(5)`。
- 输出固定为 `odom -> body`、ENU/FLU；线速度和去 bias 角速度均为 body/FLU。
- IMU 单位默认 `unconfirmed`，静置统计 10 秒后只报告建议，不自动切换、不发布高频 odom。
- LiDAR 锚点 0.15 秒进入 SUSPECT，0.40 秒进入 FAULT 并停止 `/Odometry`。
- 默认关闭额外 LiDAR 累积、低通滤波、回环、影子地图、地图修正和工作树重建。
- GTSAM 回环改为 `FR_LIO_ENABLE_LOOP_CLOSURE=ON` 时才构建，默认不依赖 GTSAM。
- 已通过 ROS 包构建；地图修正 9 项测试和高频传播 13 项测试全部通过。
- 生产 odom 数据面已统一为 `BEST_EFFORT + KEEP_LAST(5)`：FR-LIO、guard、
  health monitor 和 MAVROS vision bridge 使用兼容 QoS。
- health monitor 可选订阅瞬态 `/frlio/high_rate_odom/status` 和实时
  `/frlio/high_rate_odom/anchor_age`；FR-LIO 生产入口强制开启该门控。未知状态、
  状态/年龄超时或锚点超过 0.40 秒时停止 `/Odometry/healthy`。

`10A` 的飞行授权边界仍然有效：生产 health monitor、guard 和 MAVROS bridge
已接入上述 QoS 与锚点门控，但仍必须完成无桨、遮挡 LiDAR、停止 IMU、轴向、
杆臂和时延验收。当前 `/Odometry` 不应直接绕过健康检查送入 PX4。

以下原始选项保留作为决策记录。

下面选择会决定实际修改范围。请回复每项编号和选项，例如：`1A 2A 3B 4A 5A 6A`。

### 1. 移植范围

- **1A（推荐）**：只把高频传播模块移入当前实际运行的 FAST-LIO，保留现有建图与 PX4 桥。
- 1B：用完整 FR-LIO 替换当前 FAST-LIO，包括 GTSAM 回环、影子地图和新启动系统。

### 2. 高频传播算法

- **2A（推荐）**：实现逐条 IMU 顺序积分；LiDAR 校正后重置传播状态。这比 FR-LIO 当前“锚点 + 当前单条 IMU 一步外推”更适合飞行。
- 2B：严格照搬 FR-LIO 当前单步外推，改动较少，但快速运动和锚点变旧时误差更大。

### 3. IMU 原始加速度单位

- 3A：`/livox/imu.linear_acceleration` 静置模长约为 `9.81`，选择 `mps2`，不乘 9.81。
- 3B：静置模长约为 `1.0`，选择 `g`，进入传播器前乘 9.81。
- **3C（未测量时推荐）**：先录取 10 秒静置数据自动生成报告，确认后再编译启用，不允许运行期自动猜测。

### 4. 实时 odom QoS

- **4A（推荐）**：`/Odometry` 和 `/Odometry/healthy` 全链使用 best-effort SensorDataQoS，depth 5。
- 4B：全链 reliable，depth 5；网络或消费者阻塞时更容易产生排队延迟。

### 5. 激光累积

- **5A（推荐）**：第一阶段关闭额外累积，使用当前生产 FAST-LIO 的输入策略。
- 5B：保留固定包数累积，需要你提供目标包数和实测原始 LiDAR 消息频率。
- 5C：实现最大包数 + 最大时间窗双条件，需要你确认目标窗口，建议先从 50 ms 开始。

### 6. 高频输出的姿态/速度平滑

- **6A（推荐）**：正式 `/Odometry` 不增加额外低通，保留真实滤波状态和最小延迟；平滑输出另开诊断话题。
- 6B：对速度和姿态增加按截止频率配置的低通，需要根据飞行日志确定截止频率。

### 7. LiDAR 锚点超时策略

- **7A（推荐）**：0.15 s 后 SUSPECT，0.30 s 后 FAULT，并停止向 MAVROS 发布健康 odom。
- 7B：使用其他门限；请给出正常 LiDAR 更新频率及允许的最大纯惯性维持时间。

### 8. frame 与杆臂职责

- **8A（推荐）**：LIO 输出 `odom -> body`（IMU/LIO 原点），现有 vision bridge 唯一负责 `body -> base_link` 杆臂和 PX4 坐标转换。
- 8B：LIO 直接输出控制中心 `base_link` 位姿，并删除 vision bridge 的杆臂补偿。该方向修改面更大，需要重新做现有无桨轴向与杆臂验收。

### 9. 角速度发布语义

- **9A（推荐）**：发布去 bias 的有限 body/FLU 角速度，并计算相应协方差。
- 9B：角速度发布 `NaN`，角速度协方差设为 `1e6`，PX4 不使用该测量。

### 10. 实施目标仓库

- **10A（推荐）**：先在当前工作空间做可评审补丁/独立模块，再按确认结果移入实际生产 FAST-LIO 源码。
- 10B：直接修改 `26-px4_offboard_control` 的启动、bridge 和健康链路，同时修改外部 FAST-LIO 环境。需要先确认实际生产 FAST-LIO 源码路径，因为启动脚本当前引用的是 `${HOME}/livox_mid360_env/run_fastlio_mid360.sh`。
