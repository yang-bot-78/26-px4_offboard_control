# 本次起飞与 EV 健康链路修改记录

## 2026-08-18 换轨门限修改前备份

修改来源：`src/race_bringup/config/astar_ego_tuning.yaml`。以下为修改前实际运行配置，保留用于回退和飞行对比。

| 门限 | 修改前值 |
|---|---:|
| `handover_position_tolerance_m` | `0.015m` |
| `handover_velocity_tolerance_mps` | `0.005m/s` |
| `handover_acceleration_tolerance_mps2` | `0.01m/s²` |
| `switch_position_tolerance_m` | `0.05m` |
| `switch_velocity_tolerance_mps` | `0.10m/s` |
| `switch_acceleration_tolerance_mps2` | `0.20m/s²` |
| `handover_max_rejections` | `2` |
| `handover_transaction_timeout_sec` | `1.50s` |
| `replan_start_position_error_m` | `0.25m` |

本次计划修改为：`0.08m / 0.13m/s / 0.09m/s²` 的 EGO 内部交接门限，及 `0.10m / 0.14m/s / 0.23m/s²` 的 Bridge 交接门限；最多拒绝 `3` 次，事务超时和重新锚定距离保持不变。

## 自动起飞检查

- 脚本：`tools/flight/切Offboard后自动起飞0.75米单目标验证.sh`
- 起飞检查、稳定完成和故障提示改为中文字段输出。
- 删除位姿差分垂直速度及“速度差异”的计算和输出；该差异不再可能被误解为起飞稳定门限。
- 起飞完成后的稳定检查保留飞行模式、EV 飞行就绪、高度基准、距地高度和水平速度；垂直速度仅作为诊断信息，不再参与完成判定。
- 起飞稳定高度范围保持 `0.55-0.80 米`，目标高度保持 `0.75 米`。
- 起飞过程为定高爬升，垂直速度不设“合格”门限；日志仍输出垂直速度供排障使用。
- 高度超过 `0.80 米` 时“高度合格”为否，不会判定为起飞完成；该条件不会改变全局 `/ev_health/flight_ready`。
- 对起飞过程中的单次 `flight_ready=false` 增加 `0.75 秒`宽限期。短暂抖动仅告警，持续超过宽限期才中止起飞检查。可通过环境变量 `FLIGHT_READY_FALSE_GRACE_S` 覆盖。

## FR-LIO 锚点年龄容错

- FR-LIO 配置中的 `high_rate_odom.max_anchor_age_s` 已统一为 `0.60 秒`：
  - `src/fr_lio/config/indoors.yaml`
  - `src/fr_lio/config/outdoors.yaml`
  - `src/fr_lio/config/indoors_baseline.yaml`
  - `src/fr_lio/config/outdoors_baseline.yaml`
- EV 健康监控器的 `frlio_max_anchor_age_s` 默认值和启动参数默认值同步为 `0.60 秒`：
  - `src/px4_ros_com/px4_ros_com/ev_health.py`
  - `src/px4_ros_com/scripts/fastlio_ev_health_monitor.py`
  - `src/px4_ros_com/launch/fastlio_mavros_autofix.launch.py`
- EV 侧锚点过期判断由 `>=` 改为 `>`，与 FR-LIO 的边界语义一致；锚点年龄恰好等于上限时两侧均不判为硬过期。
- `0.15-0.60 秒`仍为 FR-LIO 降级区间：继续发布传播里程计并使协方差随传播时间增加；超过 `0.60 秒`才停止健康输出并进入硬故障处理。

## 本次锚点超时排查结论

场次：`flight_records/20260817/flight_20260817_224757`。该场次发生在 `0.40 秒`旧门限生效时。

- 在 `1786978212.616-1786978213.018`，LiDAR 与 IMU 源时间戳同时存在约 `0.40 秒`和 `0.34 秒`空窗，属于 MID-360 驱动、网络传输或系统调度层的共同暂停。
- 起飞阶段 `1786978230`，LiDAR 约 `10 Hz`、IMU 持续到达，高速 `/Odometry` 的最大到达延迟仅 `24 ms`；但 LiDAR 校正锚点年龄升至 `0.35-0.55 秒`。结论是 FR-LIO 的 LiDAR 校正/匹配路径未及时完成，属于处理积压或锁竞争，而非传感器原始数据断流。
- 现有日志不足以区分计算耗时、锁竞争和系统资源竞争。后续应记录每帧点云排队、预处理、匹配求解及 `reset_from_lidar` 的耗时，并分别关闭 rosbag、RViz、导航/重定位进行对照测试。

## 不录包运行

自动起飞脚本支持：

```bash
./tools/flight/切Offboard后自动起飞0.75米单目标验证.sh --norosbag
```

- 默认仍录制 rosbag。
- `--norosbag` 只将本次 `RECORD_BAG` 设为 `false`，不改变飞行、导航或 EV 安全检查。
- 可使用 `--help` 查看参数说明。

## EGO 同目标换轨修复

- 飞机解锁且距地高度达到 `0.40 米`后，Goal Bridge 即放行 rolling goal，让 EGO 在起飞过程中后台规划；起飞垂直速度不参与这个放行条件。
- `0.40 米`不代表开始水平飞行。Offboard 仍保持原有 `0.55 米`起飞完成门，达到后才允许接管 EGO 轨迹；固定飞行高度仍为 `0.78 米`。
- 正常 EGO 是第一次规划；失败后只允许一次后备。第一次若为交接状态不连续，第二次从实测状态重新锚定；若为路线/碰撞不可行，第二次使用局部 A* 种子。两者不会连续都执行。
- 恢复 A* 可在 `1.50 米`任务走廊内绕开旧参考线，不要求先直线返回旧轨迹；防撞净空、动力学限制、地理围栏和定高约束不变。
- Bridge 第一次发现新旧轨迹不连续时，仅在旧轨迹剩余部分仍无碰撞时继续执行旧轨迹；否则立即进入保持/刹车路径。
- 同一局部目标最多拒绝两次，事务最长 `1.50 秒`。实际截止时间还会扣除按当前速度计算的刹车时间和 `0.15 秒`余量，因此旧轨迹快结束时会更早刹车。
- 次数耗尽或超时后发布 `EGO_REPLAN_TRANSACTION_EXHAUSTED`，迟到轨迹直接丢弃；普通地图版本刷新和等待时间不会重开同一事务。只有明确的新局部目标/新全局路径才能建立新事务。
- 已删除只修改 `start_time` 的伪连续修复。正常换轨按实测规划耗时预测旧轨迹未来状态，预测仍受严格位置、速度、加速度交接门复核。

验证结果：相关 3 个包编译通过；EGO 功能测试 5/5、Bridge 功能测试 2/2、配置测试 58/58、0.40 米 Goal Bridge 真实节点集成测试均通过。尚未进行实飞验证。

## 待确认事项

“距地高度超过 `1.5 米` 时需要 false”尚未改动。当前超过 `0.80 米`已会令自动起飞检查的“高度合格”为否；若需要将全局 `/ev_health/flight_ready` 也设为 `false`，需明确确认，因为这会扩大 EV 健康信号的语义和影响范围。
