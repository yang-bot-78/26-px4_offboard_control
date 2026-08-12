# Offboard 起飞、悬停和自动降落判定报告

## 数据

- ROS bag：`flight_records/20260810/flight_20260810_201111/rosbag`
- rosbag 时长：273.47 s；67014 条消息
- 保存状态：`saved_cleanly`
- 本场没有比 `log_dagan.ulg` 更新的 PX4 ULog，因此模式和解锁以 rosbag 的 `/mavros/state` 为准。
- 当前参数仍为 `EKF2_EV_CTRL=11`，本场没有启用 EV velocity 融合。

## 状态时间线

| 时间（bag 相对秒） | connected | armed | mode | 判定 |
| ---: | :---: | :---: | --- | --- |
| 1.295 | true | false | STABILIZED | 地面待命 |
| 36.233 | true | false | OFFBOARD | 进入 Offboard |
| 37.232 | true | true | OFFBOARD | 解锁成功 |
| 61.224 | true | true | AUTO.LAND | 自动降落开始 |
| 67.222 | true | false | OFFBOARD | 已落地并上锁 |
| 92.220 | true | false | STABILIZED | 回到手动模式 |

随后还有一段 `STABILIZED/POSCTL` 手动操作（约 `96.220-107.215 s`），不属于本次 Offboard 轨迹验证。

## Offboard 起降

### 结果

```text
OFFBOARD_ENTRY: PASS
ARMING: PASS
AUTO_LAND_REQUEST: PASS
LANDING_DISARM: PASS
```

`/mavros/setpoint_raw/local` 共 644 条，发送时间约 `28.895-61.034 s`，平均约 `20.01 Hz`，最大间隔约 `55 ms`。满足 PX4 进入 Offboard 所需的连续 setpoint 条件。

### EV 健康

在实际 Offboard 解锁飞行窗口 `37.232-61.224 s` 内，`/ev_health/status` 全部为 `HEALTHY`（480 个样本）。本窗口没有 `SUSPECT` 或 `FAULT`。

录包后段 `154.760-163.558 s` 出现过 `SUSPECT/FAULT`，发生在本次飞行已经结束之后，不影响本次 Offboard 起降判定。诊断内容为：

```text
154.760 s  SUSPECT  px4_velocity_unaligned
155.104 s  FAULT    px4_velocity_timeout
161.553 s  recovering（持续约 2 s）
163.558 s  HEALTHY
```

异常期间 `input_age_s` 仍约 `0.01-0.05 s`，因此没有证据表明 FAST-LIO 输入停止；更可能是录包后手动操作阶段 PX4 本地速度与 EV 速度不一致，触发了健康监视器的一致性门限。下一场应在飞行结束、切回手动后仍保持静止，分别录制 `/ev_health/velocity_ned`、`/mavros/local_position/velocity_local` 和 `/mavros/vision_speed/speed_twist_cov`，确认是坐标/符号差异还是确有延迟。

## 悬停判定

以解锁前 `34-36 s` 的本地位置作为起飞基准，计算 Offboard 解锁窗口内的位置和速度。

| 指标 | 结果 | 判定 |
| --- | ---: | --- |
| 最大水平漂移 | `0.221 m` | 满足 `<0.30 m` 的工程门槛 |
| Offboard 解锁飞行时长 | `23.99 s` | 已完成短时悬停 |
| 降落前约 5 s 的相对高度增量 | `0.687-0.739 m`，均值约 `0.714 m` | 接近但略低于 `0.78 m` 目标 |
| 降落前约 5 s 水平速度均值 | `0.072 m/s` | 高于严格 `0.05 m/s` 停稳门槛 |
| 降落前约 5 s 水平速度 P95 | `0.112 m/s` | 未达到严格停稳标准 |

因此本场结论为：

```text
水平定点：工程上基本通过
严格悬停停稳：未完全通过
```

需要注意：本次高度是通过本地位置相对起飞基准计算的。setpoint 目标为 `0.78 m`，但本地位置显示的绝对 z 原点存在约 `-0.11 m` 偏置；下一场应继续同时记录地面基准、目标高度和实际高度，不能只看单个绝对 z 数值。

## 轨迹跟踪

本场 rosbag 没有以下轨迹输入或规划输出话题：

```text
/race/navigation_setpoint
/race/ego/trajectory_setpoint
/race/ego/status
/goal_pose
```

`/race/control/status` 只出现了 `IDLE_HOLD` 和 `WAITING_FOR_ODOM`，没有 `PLANNING`、`TRACKING` 或 `GOAL_REACHED_HOLD`。

因此：

```text
轨迹规划输出：未启动
轨迹跟踪能力：本场未测试
```

## 下一步

1. 保持 `EKF2_EV_CTRL=11`，重复一次 Offboard 悬停；起飞后至少保持 30 s，并等待高度和速度连续 10 s 稳定后再降落。
2. 用实际现场地图启动完整导航栈，设置 `MISSION_ENABLED=false`，先只发送一个约 0.8-1.0 m 的安全目标。
3. 录制并检查 `/goal_pose`、`/race/navigation_setpoint`、`/race/ego/trajectory_setpoint`、`/race/ego/status` 和 `/race/control/status`。
4. 只有确认 `TRACKING`、目标到达误差和速度满足门槛后，才增加反向、侧向和小矩形轨迹。
5. 继续关注录包后段出现的 `FAULT`，不能因为它不在飞行窗口内就忽略；先完成 PX4 本地速度与 EV 速度的同时间对齐检查，再决定是否调整健康门限或坐标转换。

## 总结

```text
Offboard 进入：通过
自动解锁：通过
短时水平悬停：基本通过
严格速度停稳：未完全通过
自动降落并上锁：通过
轨迹跟踪：未测试
```
