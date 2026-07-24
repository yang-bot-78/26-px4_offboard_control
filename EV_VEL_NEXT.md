# EV velocity acceptance handoff

## 已确认结论
- `READY_FOR_EV_VEL_FUSION = NO`，保持 `EKF2_EV_CTRL=11`。
- 无桨验证全程未解锁、未进入 OFFBOARD、无控制话题消息。
- 唯一输入为 `/mavros/odometry/out`；旧 `vision_pose` 和直接 PX4 EV 发布均为零。
- 时间戳单调、source age 合格；静止噪声、六方向符号、停止回零、FAULT 抑制和 7.5 s 恢复通过。
- 内部速度对位置差分最佳偏移 `+50 ms`，相关系数 `0.752`，RMSE `0.0247 m/s`。

## 当前硬阻塞项
- 桥接器以 `min_linear_velocity_variance=0.01` 钳制速度方差，未保留 FAST-LIO 动态协方差。
- 实际 FAST-LIO 频率 `9.45 Hz`，判定 `FUSION_RATE_READINESS_FAIL`；禁止重复旧样本提频。
- 速度与位置差分相关性仅中等，尚未达到“高度相关”。
- 缺少 PX4 `vehicle_visual_odometry`、EV aid source 和 `cs_ev_vel=false` 的 uORB 证据。

## 安全边界
- 不修改任何 PX4 参数，不得将 `EKF2_EV_CTRL` 改成 `15`。
- 不装桨、不解锁、不启动电机、不 Autotune、不实飞。
- 不启动 Offboard、起飞、模式切换、解锁或控制设定值节点。

## 下一步需要读取的文件
- `EV_VEL_NEXT.md`
- `validation_records/prop_off_ev_20260722_152742/validation_report.md`
- `src/px4_ros_com/src/bridges/fastlio_mavros_odometry_bridge.cpp`
- `src/px4_ros_com/scripts/fastlio_ev_health_monitor.py`
- `/home/robot/livox_mid360_env/ws_fastlio/src/fast_lio/src/laserMapping.cpp`

## 下一次验收标准
- 输出携带有限、对称、半正定且随FAST-LIO变化的真实三维速度协方差。
- 三维速度符号正确、停止快速回零，与时间对齐位置差分高度相关且无尖峰/旧帧。
- 只有一路 ODOMETRY 输入，原始时间戳保留，SUSPECT 膨胀协方差，FAULT 停止输出。
- PX4 收到有限且随运动变化的速度，同时 `cs_ev_vel=false`，位置/高度/航向融合无退化。
- 真实发布率满足要求或有当前 PX4 版本的低频安全证据后，才可重新判定 YES/NO。
