# ws_offboard_control

当前工程用于把 `MID-360 + FAST-LIO` 的里程计送入 `PX4`，并基于 `MAVROS` 做室内外部视觉定位与 Offboard/起飞验证。

## 当前默认链路

```text
MID-360
  -> livox_ros_driver2
  -> FAST-LIO
  -> /Odometry
  -> fastlio_mavros_vision_bridge
  -> /mavros/vision_pose/pose_cov
  -> MAVROS
  -> PX4
```

当前默认方案：

- 飞控通信：`MAVROS + MAVLink`
- 外部视觉输入：`MAVROS vision_pose`
- 不使用 `uXRCE-DDS` 作为默认 PX4 EV 输入
- 不使用 `MAVROS odometry` 作为默认 PX4 EV 输入

## 当前已验证有效的配置

### FAST-LIO 外参

文件：

- `~/livox_mid360_env/ws_fastlio/src/fast_lio/config/mid360.yaml`

当前有效值：

```yaml
extrinsic_R: [ 0., -1., 0.,
                1.,  0., 0.,
                0.,  0., 1.]
```

### MAVROS vision yaw 修正

文件：

- [fastlio_mavros_vision_bridge.cpp](src/px4_ros_com/src/bridges/fastlio_mavros_vision_bridge.cpp)
- [fastlio_mavros_autofix.launch.py](src/px4_ros_com/launch/fastlio_mavros_autofix.launch.py)

当前默认值：

```text
vision_yaw_offset_rad = 1.5707963267948966
```

### PX4 参数

当前已验证有效：

```bash
param set EKF2_EV_CTRL 11
param save
```

说明：

- `11` = EV 水平位置 + EV 垂直位置 + EV yaw
- 当前不把 `15` 作为默认值

## 当前验证结论

当前已经确认：

- `vehicle_visual_odometry` 能稳定进入 PX4
- `vehicle_visual_odometry.pose_frame = 1`
- `vehicle_local_position.heading_good_for_control = True`
- `estimator_aid_src_ev_yaw.fused = True`
- `estimator_aid_src_ev_yaw.innovation_rejected = False`
- `heading` 与 `unaided_heading` 可以对齐
- `reset counter` 在静置后不会持续增长

也就是说：

- `VIO/MAVROS/PX4` 链路已经打通
- `yaw` 参考已经可用于控制

## 当前状态更新

此前“有桨起飞左倾更像动力链/映射问题”的问题，当前已解决。

因此当前结论更新为：

- `VIO/MAVROS/PX4` 链路已打通
- `yaw` 参考已可用于控制
- 当前起飞链路可继续用于后续飞行与控制验证
- README 不再把“左倾/滚转失控”作为当前阻塞项

## 核心文件

- 默认 launch：
  [fastlio_mavros_autofix.launch.py](src/px4_ros_com/launch/fastlio_mavros_autofix.launch.py)
- MAVROS vision bridge：
  [fastlio_mavros_vision_bridge.cpp](src/px4_ros_com/src/bridges/fastlio_mavros_vision_bridge.cpp)
- 旧的 MAVROS odometry bridge：
  [fastlio_mavros_odometry_bridge.cpp](src/px4_ros_com/src/bridges/fastlio_mavros_odometry_bridge.cpp)
- 30 cm 起飞脚本：
  [run_takeoff_30cm_hold.sh](run_takeoff_30cm_hold.sh)
- 一键启动脚本：
  [start_takeoff_30cm_stack.sh](start_takeoff_30cm_stack.sh)

## 构建

第一次使用或修改 `px4_ros_com` 后：

```bash
cd ~/ws_offboard_control
source /opt/ros/humble/setup.bash
colcon build --packages-select px4_ros_com
source ~/ws_offboard_control/install/setup.bash
```

## 启动方式

### 手动四终端启动

#### 终端 1：MID-360 驱动

```bash
~/livox_mid360_env/run_mid360_driver.sh
```

#### 终端 2：FAST-LIO

```bash
~/livox_mid360_env/run_fastlio_mid360.sh
```

#### 终端 3：MAVROS + PX4 链路

```bash
cd ~/ws_offboard_control
source /opt/ros/humble/setup.bash
source ~/ws_offboard_control/install/setup.bash
ros2 launch px4_ros_com fastlio_mavros_autofix.launch.py \
  fcu_url:=serial:///dev/ttyUSB0:921600?ids=255,190
```

#### 终端 4：30 cm 起飞脚本

```bash
cd ~/ws_offboard_control
./run_takeoff_30cm_hold.sh
```

### 一键启动

```bash
cd ~/ws_offboard_control
./start_takeoff_30cm_stack.sh
```

它会按顺序拉起四个独立终端窗口：

1. `run_mid360_driver.sh`
2. `run_fastlio_mid360.sh`
3. `fastlio_mavros_autofix.launch.py`
4. `run_takeoff_30cm_hold.sh`

## 起飞前最小检查

### ROS 侧

```bash
source /opt/ros/humble/setup.bash
source ~/ws_offboard_control/install/setup.bash

ros2 topic echo /Odometry --once
ros2 topic echo /mavros/state --once
ros2 topic echo /mavros/vision_pose/pose_cov --once
```

目标：

- `/Odometry` 有数据
- `/mavros/state` 中 `connected: true`
- `/mavros/vision_pose/pose_cov` 有数据

### PX4 侧

在 QGC 的 MAVLink Console 执行：

```bash
listener vehicle_visual_odometry 5
listener vehicle_local_position 5
listener estimator_aid_src_ev_yaw 5
```

目标：

- `vehicle_visual_odometry.pose_frame: 1`
- `vehicle_local_position.xy_valid: True`
- `vehicle_local_position.z_valid: True`
- `vehicle_local_position.v_xy_valid: True`
- `vehicle_local_position.v_z_valid: True`
- `vehicle_local_position.heading_good_for_control: True`
- `vehicle_local_position.dead_reckoning: False`
- `estimator_aid_src_ev_yaw.fused: True`
- `estimator_aid_src_ev_yaw.innovation_rejected: False`

## 30 cm 起飞脚本

文件：

- [run_takeoff_30cm_hold.sh](run_takeoff_30cm_hold.sh)

当前参数：

```bash
-p target_z_m:=-0.30
```

说明：

- 目标是相对当前点起飞 `30 cm`
- 当前 `auto_land:=false`
- 需要 RC 随时接管

## 常用检查命令

### 检查 MAVROS

```bash
cd ~/ws_offboard_control
./fastlio_mavros_check.sh doctor
./fastlio_mavros_check.sh print
```

### 分项检查

```bash
cd ~/ws_offboard_control
./fastlio_mavros_check.sh mavros
./fastlio_mavros_check.sh state
./fastlio_mavros_check.sh odom
./fastlio_mavros_check.sh bridge
./fastlio_mavros_check.sh mavros-odom
```

## 出问题时优先回传

1. `ros2 topic echo /Odometry --once`
2. `ros2 topic echo /mavros/state --once`
3. `ros2 topic echo /mavros/vision_pose/pose_cov --once`
4. `listener vehicle_visual_odometry 5`
5. `listener vehicle_local_position 5`
6. `listener estimator_aid_src_ev_yaw 5`

## 当前不再作为主流程的路线

下面这些内容不要再当作默认主链路：

- `FAST-LIO -> /mavros/odometry/out -> MAVROS -> PX4`
- `FAST-LIO -> /fmu/in/vehicle_visual_odometry -> PX4`
- 旧的 `camera_init_ned_yaw = -pi/2` 试错结论
