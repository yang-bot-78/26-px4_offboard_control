# POSITION 60 秒及四向打杆判定报告

## 数据

- ROS bag：`flight_records/20260810/flight_20260810_190646/rosbag`
- PX4 ULog：`/home/robot/下载/rizi/log_dagan.ulg`
- rosbag 时长：156.94 s；ULog 时长：125.75 s
- 两条时间轴通过 `/mavros/local_position/odom` 与 ULog `vehicle_local_position` 对齐，偏移约 `28.661 s`
- `EKF2_EV_CTRL=11`

## 飞行状态和融合状态

- ULog 中 `POSCTL`（QGC Position）区间约为 `14.421-117.783 s`。
- POSCTL 区间没有 `OFFBOARD`，没有 failsafe，`dead_reckoning=false`。
- `estimator_aid_src_ev_pos`、`ev_hgt`、`ev_yaw` 在 POSCTL 区间均为 `fused=true`，`innovation_rejected=false`。
- `estimator_status_flags.cs_ev_vel=false`，本场没有启用 EV velocity 融合，这是预期结果。
- EV 健康在 POSCTL 测试区间保持 HEALTHY；录包前的起飞/稳定阶段和落地后的阶段分别出现过短暂 `SUSPECT/FAULT`，原因是 velocity mismatch / position jump，不属于本次四向动作区间。

## 60 秒 POSITION 悬停

### 判定窗口

松杆后的稳定窗口取 ULog `18.170-78.170 s`，正好 60 s；该窗口位于 POSCTL 内，且第一段水平打杆从约 `78.354 s` 才开始。

以窗口前 5 s 的平均位置作为参考点：

| 指标 | 结果 |
|---|---:|
| 水平最大漂移 | 0.289 m |
| 水平漂移 P95 | 0.198 m |
| 窗口结束漂移 | 0.095 m |
| X/Y 位置标准差 | 0.055 / 0.059 m |
| Z 位置标准差 | 0.024 m |
| 60 s 内 PX4 位置 reset | 0 |

按“稳定后的 60 s”计算，最大漂移低于 0.3 m，POSITION 定点可用。

### 严格边界

如果从刚切入 POSCTL 的第一帧 `14.42 s` 就开始计时，前几秒仍包含模式切换和高度/位置收敛，60 s 窗口最大水平偏差约 `0.343 m`。因此本场应记录为：

```text
POSITION 定点：基本通过（稳定窗口通过）
严格从模式切换第一帧计算：边界，不应宣称完全无过渡
```

## 前后打杆

机头航向约 `89.4 deg`，即机头指向世界 East；以下使用 PX4 NED 位置解释。

| 动作 | ULog 时间 | 摇杆 | 主要位移 | 结果 |
|---|---:|---:|---:|---|
| 前 | 78.354-83.362 s | pitch `+0.56` | East/NED-Y 约 `+1.56 m` | 方向正确、响应正常 |
| 后 | 85.370-91.574 s | pitch `-0.51` | West/NED-Y 约 `-1.86 m` | 方向正确 |

前后反向后，在约 `91.77 s` 的居中段，相对悬停参考点水平残差约 `0.17 m`，前后轴回程通过。

## 左右打杆

在当前航向下，机体右侧对应 NED-X 减小，机体左侧对应 NED-X 增大。

| 动作 | ULog 时间 | 摇杆 | 主要位移 | 结果 |
|---|---:|---:|---:|---|
| 右 | 93.970-100.374 s | roll `+0.51` | NED-X 约 `-1.34 m` | 方向正确、响应正常 |
| 左 | 102.370-109.770 s | roll `-0.44` | NED-X 约 `+1.59 m` | 方向正确 |

左右反向后：

- 相对右移前，NED-X 轴返回误差约 `0.03 m`；
- 但交叉轴仍有约 `0.36 m` 的 NED-Y 残差；
- 相对最初悬停参考点的总水平残差约 `0.52 m`。

因此左右动作的方向和控制响应通过，但“反向后完全回到原位”未通过严格标准。

需要注意：POSITION 模式中，单次松杆的含义是保持当前位置，不会自动返回动作前位置；回到原位必须依靠反向杆量。该场前后反向回程较好，左右反向回程仍需复测。

## 总结

```text
60 s POSITION 定点：稳定窗口通过，整体基本通过
前后方向响应：通过
左右方向符号和响应：通过
左右反向回原位：未完全通过，存在约 0.36 m 横向残差
EV position/height/yaw 融合：通过
EV velocity 融合：未启用，不能由本场判定
```

本场可以作为 POSITION 定点可用的证据，但不建议仅凭本场进入 Offboard 或启用 `EKF2_EV_CTRL=15`。建议先复测左右方向：减小杆量到约 10%-15%，每次动作后居中等待至少 5 s，确认速度降到 `0.05 m/s` 以下再进行反向动作。
