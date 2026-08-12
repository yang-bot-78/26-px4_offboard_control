# 速度协方差修改后 Shadow 录包分析

## 结论

本场录包已干净保存，新的 X/Y 协方差下限已经实际进入健康输出和 MAVROS vision speed：

```text
Odometry/healthy X 方差：均值 0.00160000 m2/s2
Odometry/healthy Y 方差：均值 0.00129994 m2/s2
vision_speed X 方差：    恒为 0.00160000 m2/s2
vision_speed Y 方差：    恒为 0.00130000 m2/s2
```

Z 方差保持 FAST-LIO 原始值，没有根据未完成的垂直测试人为放大。

```text
COVARIANCE_FLOOR_APPLIED = PASS
EV_HEALTH = HEALTHY
EKF2_EV_CTRL = 仍应保持 11
EV_VELOCITY_FUSION = 未启用
```

## 数据身份

- 场次：`flight_20260810_182908`
- 时长：82.90 s
- 消息总数：20088
- 保存状态：`saved_cleanly`
- 状态：MAVROS 已连接、未解锁、`STABILIZED`
- `/mavros/setpoint_raw/target_local`：0 条

## 链路健康

| 项目 | 结果 |
| --- | ---: |
| `/ev_health/status` | 1659/1659 `HEALTHY` |
| `/Odometry` | 10.000 Hz |
| `/Odometry/healthy` | 10.000 Hz |
| `/mavros/vision_speed/speed_twist_cov` | 10.000 Hz |
| 输入年龄 mean/p95/max | 16.3/41.9/76.6 ms |
| PX4 速度对齐 mean/p95/max | 24.4/32.8/36.0 ms |
| 水平速度差 mean/p95/max | 0.0304/0.0628/0.1184 m/s |
| 内部速度差分误差 mean/p95/max | 0.0277/0.0505/0.0847 m/s |

没有 `SUSPECT`、`FAULT`、持续断流或明显速度失配。

## 重要边界

这场录包证明的是“协方差修改后的 EV 链 shadow”已经生效，不是实际导航飞行 shadow：

- 快照节点中只有 MID360、FAST-LIO、MAVROS、健康节点和 bridge；没有 Navigation/Offboard 控制节点。
- 飞控全程 `STABILIZED` 且未解锁。
- 没有位置 setpoint，因此不能用本场评估导航控制跟踪，也不能评估实际飞行振动。

因此本场通过后，下一步可进行独立的 POSITION 手动悬停验证；不要把本场结果解释为已经可以打开 `EKF2_EV_CTRL=15`。

## 原始证据

```text
rosbag：flight_records/20260810/flight_20260810_182908/rosbag
快照：flight_records/20260810/flight_20260810_182908/snapshot
状态：flight_records/20260810/flight_20260810_182908/snapshot/rosbag_status.txt
```
