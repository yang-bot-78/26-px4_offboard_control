# 导航栈融合总结

> 更新时间：2026-08-09
>
> 本文说明 `0807BFR`（仿真）的导航能力如何接入 `ws_offboard_control`（真机），
> 改了什么、为什么这么改、哪些地方还不能飞。
>
> **本文不构成飞行授权。** §11 列出的未关闭项全部关闭前，不要装桨。
>
> **两条最要紧的**：飞行中没有水平漂移保护和起飞瞬态保护（§11.4，已接受的
> 缺口，必须有人握遥控器）；geofence 和高度边界仍是仿真场地标定值（§11.3，
> 必须现场重标）。

---

## 1. 一句话概括

把仿真工作区里已经跑通完整避障的两个规划器（Super A\* 全局 + EGO B-spline 局部）
搬到真机工作区，通信层从 PX4 DDS 全部换成 MAVROS，并补上仿真里不需要、真机
必须有的外部视觉健康门禁。

EV 定位链路（MID-360 → FAST-LIO → MAVROS vision_pose → EKF2）**一行未改**。
旧的起飞/悬停基线 `minipc_mavros_offboard.py` 已删除，`offboard_waypoint_node`
是现在唯一的 Offboard 节点（§7）。

---

## 2. 为什么要做

`ws_offboard_control` 之前只能做三件事：起飞、定点悬停、外部触发降落。
它没有任何路径规划或避障能力 —— 飞机不知道障碍在哪，也不会自己去某个地方。

`0807BFR` 有完整的导航能力，但它跑在仿真里，全程用 PX4 DDS
（`px4_msgs` + Micro-XRCE-DDS Agent，UDP 8888），而真机这边是 MAVROS + 串口。
两套通信机制不兼容，不能直接拿来用。

目标是 2026 智能无人系统挑战赛·飞行避障 4.0。

---

## 3. 关键判断：算法本身不需要改

移植前做了一次耦合度普查，结论比预期乐观得多：

| 组成部分 | 与 PX4 的耦合 | 处理方式 |
|---|---|---|
| Super A\* 的 6 个策略头文件（A\* 搜索、Chaikin 平滑、净空校验、窄通道居中） | **零** | 原样拷贝 |
| EGO 全部 6 个子包（B-spline 优化器、体素地图、路径搜索） | **零** | 原样拷贝 |
| `race_msgs` 三个消息定义 | **零** | 原样拷贝 |
| Super A\* 节点主体 | 只用了 1 个 PX4 消息类型 | 改约 20 行 |
| EGO 的 4 个桥接节点 | 4 个里 1 个零改动、1 个可删、2 个换消息类型 | 改几十行 |
| `race_mapping` 5 个节点 | 只有 1 个用到，且那段默认关闭 | 删掉那段 |
| `race_offboard` Offboard 节点 | 41 处引用，arm/mode/setpoint/心跳全套 | **重写通信层** |

也就是说，**规划算法完全不知道 PX4 存在**。真正的工作量集中在一个文件：
`race_offboard/src/offboard_waypoint_node.cpp`（1561 行）。

这一点很重要，因为它意味着避障逻辑没有被改动过，仿真里验证过的行为在真机上
应当一致 —— 前提是喂给它的坐标和地图是对的。

---

## 4. 最容易搞错的地方：三套坐标系

工程里现在**同时存在三个不同的坐标约定**。这是全部改动中最危险的部分，
因为轴搞反了飞机会朝错误方向飞，而集成测试发现不了。

| 名字 | 轴向 | 谁在用 |
|---|---|---|
| 项目 `map` 帧 | **(South, East, Up)** | 规划器内部、geofence、点云地图 |
| `px4_ned` | 标准 NED (North, East, Down) | `NavigationSetpoint` 消息、Offboard 节点内部状态 |
| MAVROS ENU | 标准 ENU (East, North, Up) | `/mavros/local_position/*`、`/mavros/setpoint_raw/local` |

`map` 帧**不是** ENU。它由 `race_ego_bridge/frame_utils.hpp` 里
`nedToMap = {-x, y, -z}` 定义，展开后是 (South, East, Up)，与标准 ENU 相差
绕 Up 轴 −90°。

### 两个反直觉的坑

**坑一：`FRAME_LOCAL_NED` 其实要填 ENU。**
`/mavros/setpoint_raw/local` 的 `coordinate_frame` 常量名叫 `FRAME_LOCAL_NED`，
但 MAVROS 插件内部会自己做 ENU→NED 再发 MAVLink。所以这里必须填 **ENU** 数值。
把 NED 原样填进去 → North 和 East 互换 → 飞机朝 90° 偏的方向飞。

**坑二：不要"修正" `nedToMap`。**
看到 `{-x, y, -z}` 会很想把它改成标准的 XY 交换。**不要改。**
`astar_ego_tuning.yaml` 里 `shared_safety` 整块（geofence、0.29m 净空）
都是按 (South, East, Up) 标定的，改约定会静默废掉所有安全边界，
而且不会有任何报错。

### 怎么防住

转换只发生在一个地方 —— `race_offboard` 的 MAVROS 边界，集中在
`race_offboard/include/race_offboard/mavros_frame_utils.hpp` 的 4 个函数里。
配套 11 个 gtest（`test_mavros_frame_utils.cpp`），除了往返恒等和跨 ±π 环绕，
还**交叉验证了与 ego_bridge 约定的一致性** —— 如果两边约定哪天漂移了，测试会红。

---

## 5. Offboard 节点重写了什么

状态机逻辑一行没动：两套状态机（`State` 7 态 + `ControlState` 8 态）、
`runActive()` 的 10 级优先级链、六层高度保护，全部保留原样。只换了通信层。

| 原 PX4 DDS | 换成 MAVROS |
|---|---|
| `OffboardControlMode` 心跳发布 | **整体删除** —— MAVROS 自己维护心跳 |
| `TrajectorySetpoint` × 3 处 | `mavros_msgs/PositionTarget` → `/mavros/setpoint_raw/local` |
| `VehicleCommand` ARM(400) | `/mavros/cmd/arming` 服务 |
| `VehicleCommand` DO_SET_MODE(176) | `/mavros/set_mode` 服务 |
| `VehicleCommand` NAV_LAND(21) | `/mavros/set_mode` custom_mode=`AUTO.LAND` |
| `VehicleLocalPosition` 订阅 | `/mavros/local_position/pose` + `velocity_local` |
| `VehicleStatus` 订阅 | `mavros_msgs/State` |

### 三个不只是"换个名字"的改动

**命令从发布变成服务调用。** PX4 DDS 的命令是 fire-and-forget 发布，
MAVROS 全是服务。在 20Hz 定时器回调里同步 `call()` 会死锁执行器，
所以改成 `async_send_request` + pending 标志跟踪。这是本次最大的结构性改动。

**"忽略某轴"的表达方式变了。** PX4 用 NaN 标记忽略的轴，MAVROS 用 `type_mask` 位掩码。
不能用固定掩码：超高软守卫只把 Z 轴前馈置 NaN，要保留 XY 前馈，
所以掩码必须**逐轴动态生成**。另外 `PositionTarget` 没有 jerk 字段，
EGO 的 jerk 前馈会丢失（只影响 ego 路径）。

**定位健康标志位没有等价物。** PX4 的 `VehicleLocalPosition` 带 4 个估计器有效性
标志（`xy_valid`/`z_valid`/`dead_reckoning`/`heading_good_for_control`），
MAVROS 的 pose 消息什么都不带。采用保守等价：有限、单调、通过跳变检查的
pose 就视为有效；同时把 MAVROS 独有的 `State.connected` 纳入守卫
（这是 PX4 DDS 没有的信号，能检测串口链路断开）。
因为 `require_strict_local_position_health` 本来默认 false，这与原行为等价。

---

## 6. 补上了仿真里没有的东西

`race_offboard` 完全没有 EV/vision 健康检查 —— 穷尽检索确认过。仿真里不需要，
因为 PX4 SITL 的位置估计永远有效。

真机不一样：EKF2 的位置来自 FAST-LIO 经 `/mavros/vision_pose/pose_cov`，
这条链路可能在 MAVROS 报告链路健康的同时悄悄退化。**在退化的外部视觉估计上解锁**
就是这个门禁要防的事故。

从已飞过的 `minipc_mavros_offboard.py` 移植了 `EvHealthTracker`（改写为 C++，
header-only 便于单测），行为一致：

- **起飞前**：连续 `HEALTHY` ≥ 7.5s 才允许解锁。`SUSPECT` 一次就**重置整个计时**，
  不是暂停 —— 已累积的 7 秒不予保留。消息中断超过新鲜度窗口也重置。
- **飞行中**：只有 `FAULT` 触发 `AUTO.LAND`。`SUSPECT` 和状态超时会记录但不单独降落
  （与已飞行为一致）。
- 无法识别的状态字符串按 `FAULT` 处理，不按"大概没事"。

配套 11 个 gtest（`test_ev_health_tracker.cpp`）覆盖这些边界。

### 同时去掉了一个东西

`force_arm_in_sim`（PX4 magic 21196）**已删除**。它绕过 PX4 预飞检查，
仿真调试方便，真机上误开会跳过安全检查。真机必须让预飞检查生效。

---

## 7. 真正替换：minipc 已删除

`offboard_waypoint_node` 现在是**唯一**的 Offboard 节点，也是
`/mavros/setpoint_raw/local` 的唯一发布者。

2026-08-09 删除的文件：

| 文件 | 说明 |
|---|---|
| `src/px4_ros_com/scripts/minipc_mavros_offboard.py` | 旧起飞/悬停节点 |
| `src/px4_ros_com/test/test_minipc_mavros_offboard_ev_safety.py` | 它的 24 个测试 |
| `tools/flight/run_takeoff_1m_hold.sh` | 旧起飞脚本 |
| `tools/flight/run_hover_1m_offboard.sh` | 旧悬停脚本 |

替代入口是新的 **`tools/flight/运行导航控制.sh`**，完整一键栈入口为
`一键启动导航栈.sh`。`起飞前自检.sh`
的可执行文件检查也已改为逐个校验导航栈的 11 个节点（该辅助函数原先硬编码
`px4_ros_com` 包名，现改为接收包名参数）。

代价见 §11.4：漂移保护和起飞瞬态守卫随之永久失去。

保留的防护：`offboard_waypoint_node` 启动时自检 `/mavros/setpoint_raw/local`
的发布者数量，发现冲突就抛异常拒绝启动（fail closed）。这仍有意义 ——
防止误启动两个 `offboard_waypoint_node` 实例，也防止陈旧 install 树里残留的
旧节点被启动。`无桨视觉验证.sh` 的危险节点列表同时保留新旧两个名字。

---

## 8. 顺手修掉的两个会导致启动即崩的问题

这两个不是移植引入的，是移植过程中暴露出来的：

**`race_mapping.launch.py` 不再硬依赖 `fast_lio` 包。**
它在构建 launch description 时调用 `get_package_share_directory('fast_lio')`。
原先工作区没有移植 FAST-LIO，真机使用外部预编译二进制；该包现在仍保持可选，
而高频 FR-LIO 已作为 `src/fr_lio` 随仓库版本控制。默认实飞链路仍使用已验证
的 FAST-LIO，FR-LIO 必须通过 `LIO_BACKEND=fr_lio` 显式选择且仅用于无桨验证。

**5 处硬编码 `use_sim_time: True`。**
真机没有 Gazebo `/clock`，节点会永久等待一个永远不来的时钟。
已全部改为参数驱动，默认 false。

---

## 9. 当前数据流

```text
                    ┌─ 定位（未改动，已验证）──────────────────┐
MID-360 → FAST-LIO → /Odometry → fastlio_odometry_guard
                       → /Odometry/healthy → fastlio_ev_health_monitor
                       → fastlio_mavros_vision_bridge
                       → /mavros/vision_pose/pose_cov → MAVROS → PX4 EKF2
                    └──────────────────────────────────────────┘
                                      │
                          /mavros/local_position/*
                                      │
        ┌─────────────────────────────┼─────────────────────────────┐
        ↓                             ↓                             ↓
   Super A*                    EGO B-spline                  race_offboard
（全局路径, 2Hz 重规划）      （局部避障, B-spline）        （状态机 + 高度保护
        │                             │                        + EV 健康门禁）
        │  /race/ego/local_goal        │                             │
        ├────────────────────────────→ │                             │
        │                             │                             │
        │ /race/navigation_setpoint    │ /race/ego/trajectory_setpoint│
        └─────────────────────────────┴────────────────────────────→ │
                                                                     ↓
                                              /mavros/setpoint_raw/local (ENU)
                                                                     ↓
                                                            MAVROS → PX4
```

障碍来源：Super A\* 用预加载的静态 PCD 地图（`/saved_map`）；
EGO 用静态地图 + 实时点云融合（`static_live_px4` 模式）。

任务链与识别（§11.3b 接通）：

```text
mission_sequencer  ──/goal_pose──────────→ Super A*
（A→B1→B2→C→D）    ──/race/mission/zone──→ 诊断/录包
                   ──/race/mission/recognition (latched)
                                          ↓
                              bs_recognition_node
                            （urgb.py 四模型级联，仅在
                              B/D 区识别开启时运行）
                                          ↓
                              /race/recognition/result
```

航点来自 `mission.yaml` 的 `preset_points`（无人值守）或 RViz 手点（当天覆盖）。
识别节点默认不启动，需 `recognition_enabled:=true`。

---

## 10. 验证状态

| 项目 | 状态 |
|---|---|
| 14 个包编译 | 通过 |
| 7 个 launch 文件可构建 | 通过 |
| 坐标转换单测（11 项，含与 ego_bridge 交叉验证） | 通过 |
| EV 健康门禁单测（11 项） | 通过 |
| 任务序列器单测（12 项，含预设航点 8 项） | 通过 |
| `起飞前自检.sh` 预检 | exit=0 |
| Super A\* 策略测试（68 项） | 通过 |
| EGO 桥接安全测试（29 项） | 通过 |
| 全工作区功能测试 | 26 passed, 0 failed |
| **原有 EV 链路测试（66 个，原 90 减去随 minipc 删除的 24 个）** | **仍全部通过** |
| 三个不变式 I1/I2/I3 | 复核通过 |

lint 仍有失败，全部在逐字节未改动的上游 EGO 包和本次未触及的 `px4_ros_com` 里，
不是本次引入。移植的 6 个包（`race_*`）lint 全绿。

---

## 11. 还不能飞 —— 实飞前必须关闭的项目

按严重程度排序。**这一节是本文最重要的部分。**

### 11.1 已修：话题名不匹配

`fastlio_bridge_node.cpp` 曾硬编码订阅 `/fast_lio/odometry`，而真机 FAST-LIO
（`~/livox_mid360_env/ws_fastlio`）实际发布 `/Odometry`。这个节点会静默地
一条消息都收不到，`/race/odom` 与 `/race/pose` 永不发布，且不报错。

已参数化为 `odom_topic`，默认 `/Odometry`；启动日志会打印实际订阅的话题名，
便于现场确认接对了。

### 11.2 部分已修：静态地图

`map_file` 默认值现在指向 `maps/fastlio_global_3d/GlobalMap.pcd` —— 这个文件
确实存在，所以 bring-up 不会卡在 `NO_MAP`。

**但它不是比赛地图。** 比赛规则允许赛前一天现场采图，那张图必须显式传入：

```bash
MAP_FILE=/path/to/arena.pcd ./tools/flight/运行导航控制.sh
```

未传入时 `运行导航控制.sh` 会打印
`<race_mapping default; NOT the arena scan>` 提醒。飞行前务必确认地图与场地一致，
否则规划器的障碍集是错的。

### 11.3 阻断级：geofence 是仿真场地的，与真实赛场不符

当前 geofence：x ∈ [−0.85, 11.90]、y ∈ [−1.30, 12.40]，即 12.75m × 13.70m。
竞赛场地是 15m × 14m。这组数值来自仿真赛场模型标定，**不是真实场地实测**。

更要紧的是 z ∈ [0.50, 0.85]，巡航高度 0.78m。而现有已验证的真机起飞基线是
**0.80m**（`target_z_m=-0.80`）—— 恰好在 0.85 上界附近，余量只有 5cm。
超高软守卫在 1.05m 触发。这组高度数值需要按真机实际标定复核。

**这是安全合同，必须现场实测重标，不能沿用仿真值。**

### 11.3b 已打通：竞赛任务链与识别集成

融合初期只搬了「导航与避障」，竞赛任务链是断的。现已接上：

| 原状态 | 现状态 |
|---|---|
| `planner_backend` 默认 `ego-shadow`（EGO 并行但不控制，诊断模式） | 默认 `astar_ego`（比赛配置） |
| `mission_enabled` 默认 `false`（任务序列器不启动） | 默认 `true` |
| 四个航点只能 RViz 手点 → 无人值守跑不了 | 支持 `mission.yaml` 的 `preset_points` 预设；RViz 点击仍可当天覆盖预设 |
| 识别触发信号 `b_recognition`/`d_recognition` 无任何订阅者（`_log_recognition` 只打日志，注释明写 "no external process was started"） | 改为发布 latched `/race/mission/recognition`，新增 `bs/code/bs_recognition_node.py` 消费 |

`bs_recognition_node.py` 复用 `urgb.py` 的四模型级联（不复制推理代码，阈值不变），
仅在识别开启时跑，结果发布到 `/race/recognition/result`。它默认**不随
launch 启动**（`recognition_enabled:=false`）—— RealSense 和 YOLO 权重缺失
不应该在飞行中才发现。

`preset_points` 当前在 `mission.yaml` 里是**注释掉的占位坐标**（来自仿真场地），
必须换成实测坐标才有意义；不填就退回 RViz 手点。坐标是项目 map 帧 (South, East, Up)，
不是标准 ENU，填写前先读 §4。

注意 `urgb.py` 在 import 时会 `sched_setaffinity` 把进程钉到 3 个核，
这会影响整个 ROS 进程（含执行器）而不只是推理。节点已把
`cpu_threads`/`cpu_limit` 暴露为参数并在 import 前设置环境变量，
`cpu_limit:=0` 可完全关闭钉核。

### 11.4 缺失的真机保护（已接受的缺口）

`minipc_mavros_offboard.py` 已于 2026-08-09 删除（真正替换，见 §7），随之
**永久失去**以下两层保护 —— 它们从未移植进 `race_offboard`：

- **水平漂移保护**：告警 `0.20m` / 持续 `0.30s` 超 `0.50m` 请求降落 /
  持续 `0.10s` 超 `0.60m` 紧急降落
- **起飞瞬态守卫**：起飞后 `3.0s` 窗口内检查位移 `0.50m`、水平速度 `0.50m/s`、
  横滚 `10°`、俯仰 `10°`，任一持续 `0.20s` 超限则请求 `AUTO.LAND`

同时删除了验证这两层的 8 个单元测试（原 `test_minipc_mavros_offboard_ev_safety.py`
共 24 个测试，其余 16 个覆盖的 EV 门禁/降落时序行为已由
`race_offboard` 的 `test_ev_health_tracker.cpp` 等重新覆盖）。

**这是明确接受的缺口，不是遗漏。** 现在的替代手段只有：
`race_offboard` 保留的六层高度保护、EV 健康门禁，以及**操作者用遥控器随时接管**。
飞行中出现水平漂移或起飞抖动时，系统不会自己反应。实飞必须有人握着遥控器。

若后续要补，参考实现仍可在 flight_records 的历史快照里找到
（如 `flight_records/20260807/flight_20260807_125941/snapshot/minipc_mavros_offboard.py`）。

### 11.5 未验证：MAVROS odom 的 twist 到底在哪个坐标系

`ego_odom_bridge` 假设 `/mavros/local_position/odom` 的 `twist.linear` 在
**世界 ENU** 系（机体系速度是单独的 `velocity_body` 话题）。
这个假设**没有实测确认**，代码里已标注注释。如果实际是机体系，
EGO 收到的速度会在转弯时出错。

### 11.6 当前坐标参数契约

`坐标转换.md` 不再是权威文档，不能用它决定启动参数或判定验证通过。当前依据是
`src/px4_ros_com/config/mid360_lever_arm.conf`、
`src/px4_ros_com/launch/fastlio_mavros_autofix.launch.py`、桥接实现，以及
`src/px4_ros_com/test/ev_entry_contract_static.py` 的静态契约检查。

### 11.7 已修：录包覆盖导航话题

`开始录包.sh` 现在录制 `/race/*` 全套（规划器输出、EGO 轨迹与
状态、任务区/事件、`/goal_pose`）。点云类话题（`/saved_map`、`/race/ego/cloud`、
`/race/ego/occupancy`）体积大，与 `/cloud_registered` 一起放在
`RECORD_POINTCLOUD=true` 开关后面。

### 11.8 端到端 setpoint 透传无自动化覆盖

两个按 PX4 DDS 话题编写的集成测试（`test_ego_setpoint_passthrough.py`、
`test_3c28_first_gate_multinode_replay.py`）已删除 —— 它们驱动的 `/fmu/*` 话题
不再存在，且引用了导航栈已经不依赖的 `px4_msgs`。

**后果**：EGO→Offboard 的端到端透传行为目前只有单元测试覆盖
（`test_mavros_frame_utils.cpp` 覆盖逐轴 type_mask 与坐标转换），
没有跑真实节点的回放测试。重写需要 MAVROS 话题集加一个 `/ev_health/status`
发布者来满足新的起飞前门禁。

---

## 12. 剩余待办

已完成：§11.1 话题名、§11.2 地图路径（默认可用，比赛图仍需现场采）、
§11.3b 任务链与识别集成、§11.4 决定接受保护缺口、§11.7 录包话题。

**仍需场地才能关闭**：

1. **§11.3 现场实测重标 geofence 和高度** —— 安全前提，当前值来自仿真场地
2. **赛前一天现场采图**，用 `MAP_FILE=` 传入
3. **无桨地面验证**：
   - 手持移动确认 `/race/navigation_setpoint` 的 XY 方向与实际移动一致
     （验证 North/East 没搞反 —— 本次改动最可能出错的地方）
   - geofence 边界拒绝生效
   - `/mavros/setpoint_raw/local` 与 `/mavros/vision_pose/pose_cov` 各只有一个发布者
4. **§11.5 验证 MAVROS odom 的 twist 坐标系**（当前假设世界 ENU，未实测）
5. **填 `preset_points`** 为实测坐标，或确认用 RViz 手点

**不需要场地**：

6. **维护当前坐标参数契约** —— 修改 launch/config/桥接实现后重跑静态契约检查，不把
   `坐标转换.md` 当作启动或验收依据
7. 装桨前重跑 `tools/checks/起飞前自检.sh`

---

## 13. 没有移植的东西

- `FAST_LIO`（默认真机仍使用外部 `~/livox_mid360_env/ws_fastlio` 二进制；高频
  FR-LIO 源码已作为 `src/fr_lio` 纳入）
- `livox_ros_driver2`（真机已有）
- `mid360_simulation`、`race_world`（仿真件）
- `PX4-Autopilot`、`Micro-XRCE-DDS-Agent`（真机走 MAVROS 串口）
- `Lin_shi/`（Nav2 线已退役，不参与构建）
