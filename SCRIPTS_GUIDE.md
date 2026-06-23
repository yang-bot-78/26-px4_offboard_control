# Scripts Quick Guide

这份文档只保留最核心信息：脚本做什么、什么时候用、怎么执行。

## 当前主脚本

| 脚本 | 作用 | 什么时候用 |
|---|---|---|
| `start_takeoff_1m_stack.sh` | 一键启动 `MID360 -> FAST-LIO -> MAVROS -> rosbag -> 1 m 起降` | 正常试飞主入口 |
| `run_takeoff_1m_hold.sh` | 1 m 起飞、悬停、Offboard 下降、切 `AUTO.LAND` | 已手动启动其它链路时 |
| `run_hover_1m_offboard.sh` | 1 m 起飞并保持悬停，不主动降落 | 悬停稳定性测试 |
| `record_takeoff_debug_bag.sh` | 录制排障 rosbag 和配置快照 | 需要回看高度/漂移/控制问题 |
| `show_last_rosbag_status.sh` | 查看最近一次录包状态 | 起飞前或排障时 |
| `stop_takeoff_debug_bag.sh` | 手动停止当前录包 | 结束测试或关机前 |
| `check_takeoff_autostart_ready.sh` | 检查自启动/一键启动依赖 | 自启动前、改脚本后 |
| `enable_autostart.sh` | 打开 GNOME 登录后自启动 | 需要登录后自动拉起五终端 |
| `disable_autostart.sh` | 关闭 GNOME 登录后自启动 | 暂停自动启动 |

## 1. 一键启动

### `start_takeoff_1m_stack.sh`

```bash
cd ~/ws_offboard_control
./start_takeoff_1m_stack.sh
```

它会打开：

1. `MID360 Driver`
2. `FAST-LIO`
3. `PX4 MAVROS`
4. `ROS Bag Debug`
5. `Takeoff 1m`

## 2. 单独起飞 / 悬停

### `run_takeoff_1m_hold.sh`

```bash
cd ~/ws_offboard_control
./run_takeoff_1m_hold.sh
```

当前关键行为：

- 相对起点上升约 `1 m`
- 悬停 `10 s`
- Offboard 控制下降
- 接近起点高度后请求 `AUTO.LAND`

### `run_hover_1m_offboard.sh`

```bash
cd ~/ws_offboard_control
./run_hover_1m_offboard.sh
```

当前关键行为：

- 相对起点上升约 `1 m`
- 悬停 `30 s`
- 不主动进入 Offboard 降落流程

## 3. 录制 rosbag

### `record_takeoff_debug_bag.sh`

```bash
cd ~/ws_offboard_control
./record_takeoff_debug_bag.sh
```

输出目录：

```text
~/ws_offboard_control/flight_records/日期/flight_时间戳/
```

查看最近状态：

```bash
./show_last_rosbag_status.sh
```

手动停止：

```bash
./stop_takeoff_debug_bag.sh
```

## 4. 登录后自动启动

### `enable_autostart.sh`

```bash
cd ~/ws_offboard_control
./enable_autostart.sh
```

它会创建 GNOME 自启动入口，登录后调用：

```bash
./start_takeoff_1m_stack_login.sh
```

### `disable_autostart.sh`

```bash
cd ~/ws_offboard_control
./disable_autostart.sh
```

## 最常用的 3 个

```bash
./start_takeoff_1m_stack.sh
./run_takeoff_1m_hold.sh
./show_last_rosbag_status.sh
```
