# Root Scripts Quick Ref

范围：只统计 `~/ws_offboard_control` 根目录当前直接可执行、且日常会手动调用的脚本。

## 一眼总览

| 脚本 | 作用 | 直接执行 |
|---|---|---|
| `run_takeoff_1m_hold.sh` | 1 m 起飞、悬停 10 s、Offboard 控制下降并切 `AUTO.LAND` | `./run_takeoff_1m_hold.sh` |
| `run_hover_1m_offboard.sh` | 1 m 起飞并保持悬停，不主动降落 | `./run_hover_1m_offboard.sh` |
| `start_takeoff_1m_stack.sh` | 打开 5 个终端并启动整套 1 m 起降链路 | `./start_takeoff_1m_stack.sh` |
| `start_takeoff_1m_stack_login.sh` | 登录后延时再启动五终端脚本 | `./start_takeoff_1m_stack_login.sh` |
| `record_takeoff_debug_bag.sh` | 录起飞排障 rosbag 并保存快照 | `./record_takeoff_debug_bag.sh` |
| `show_last_rosbag_status.sh` | 查看最近一次 rosbag 状态 | `./show_last_rosbag_status.sh` |
| `stop_takeoff_debug_bag.sh` | 手动停止当前录包 | `./stop_takeoff_debug_bag.sh` |
| `refresh_last_rosbag_status.sh` | 刷新最近录包状态文件 | `./refresh_last_rosbag_status.sh` |
| `delete_recorded_rosbags.sh` | 删除已录 rosbag，默认保留 snapshot | `./delete_recorded_rosbags.sh` |
| `check_takeoff_autostart_ready.sh` | 检查一键启动/自启动前置条件 | `./check_takeoff_autostart_ready.sh` |
| `enable_autostart.sh` | 打开 GNOME 登录后自启动 | `./enable_autostart.sh` |
| `disable_autostart.sh` | 关闭 GNOME 登录后自启动 | `./disable_autostart.sh` |

## 主流程脚本

### `run_takeoff_1m_hold.sh`

一句话：

- 当前主起降脚本：相对起点爬升约 `1 m`，悬停 `10 s`，然后用 Offboard setpoint 缓慢下降，接近起点高度后切 `AUTO.LAND`。

关键参数：

- `target_z_m:=-1.00`
- `hover_seconds:=10.0`
- `offboard_land:=true`
- `offboard_land_speed_mps:=0.10`
- `offboard_stabilize_seconds:=20.0`
- `require_vision_pose:=true`
- `vision_freshness_s:=1.0`

### `run_hover_1m_offboard.sh`

一句话：

- 1 m 悬停测试脚本：目标高度同样是 `1 m`，悬停时间 `30 s`，不主动执行 Offboard 降落流程。

适用：

- 需要较长时间观察定点悬停、水平漂移或姿态稳定性时使用。

### `start_takeoff_1m_stack.sh`

一句话：

- 用 `gnome-terminal` 顺序拉起整套试飞链路。

打开的终端：

1. `MID360 Driver`
2. `FAST-LIO`
3. `PX4 MAVROS`
4. `ROS Bag Debug`
5. `Takeoff 1m`

默认等待：

- `1 s -> 2 s -> 3 s -> 2 s`

输出：

- 日志和 rosbag 默认进入 `flight_records/YYYYMMDD/flight_YYYYMMDD_HHMMSS/`。

## 录包和状态

### `record_takeoff_debug_bag.sh`

一句话：

- 录制起飞排障所需话题，并自动保存本次飞行参数、脚本、节点、话题、环境和 git 快照。

输出目录：

- `~/ws_offboard_control/flight_records/日期/flight_时间戳/`

### `show_last_rosbag_status.sh`

一句话：

- 查看最近一次录包是否正在录制、干净保存或异常中断。

### `stop_takeoff_debug_bag.sh`

一句话：

- 给当前录包进程发送停止信号，让 rosbag 尽量干净落盘。

### `delete_recorded_rosbags.sh`

一句话：

- 删除 `flight_records` 中的 rosbag 数据，默认保留 `snapshot/` 方便复盘配置。

常用：

```bash
./delete_recorded_rosbags.sh
./delete_recorded_rosbags.sh latest
```

## 自启动

### `enable_autostart.sh`

一句话：

- 创建 `~/.config/autostart/ws_offboard_takeoff_stack.desktop`，让 GNOME 登录后自动执行 `start_takeoff_1m_stack_login.sh`。

### `disable_autostart.sh`

一句话：

- 删除 GNOME 自启动入口，并关闭 rosbag shutdown guard 用户服务。

## 最常用的 3 个

```bash
cd ~/ws_offboard_control
./start_takeoff_1m_stack.sh
./run_takeoff_1m_hold.sh
./show_last_rosbag_status.sh
```
