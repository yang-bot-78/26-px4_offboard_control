# Root Scripts Quick Ref

> **Phase 1B-I-P 安全横幅（2026-08-03）**  
> `VISION_POSE` 稳定入口源码已静态收敛到 `/Odometry/healthy`：vision ON，
> MAVROS ODOMETRY/direct PX4 EV OFF。该结论仅有 textual-not-structural 静态
> 断言，runtime-global writer 唯一性未验证，入口不可据此运行。`204248/618c2b`
> 仅为 shadow，active/runner 仍为 B1。`C10_FLIGHT_PROFILE_INTENT=UNRESOLVED`；
> 后续高度、悬停和 spin 内容均为历史描述。`NEXT_GATE_AUTHORIZED=NO`、
> `READY_FOR_DIAGNOSTIC_LIVE=NO`、`FLIGHT_GO_NO_GO=NO`。

## 安全分类（权威）

| 类别 | 脚本 | 当前处置 |
| --- | --- | --- |
| `FLIGHT` | `run_takeoff_1m_hold.sh`, `run_hover_1m_offboard.sh`, `一键启动起飞栈.sh`, `开机自启动起飞栈.sh` | 禁止执行；可发布 setpoint 或进入 arm/mode/起降链 |
| `FLIGHT_CONFIG` | `启用开机自启.sh`, `禁用开机自启.sh` | 禁止执行；会修改持久入口，且后者范围大于本次授权 |
| `NONFLIGHT_HARDWARE` | `开始录包.sh`, `停止录包.sh` | 仅在未来明确运行授权下使用 |
| `OFFLINE` | `查看最近录包.sh`, `刷新录包状态.sh`, `起飞前自检.sh` | 本阶段未执行；preflight 会解析 ROS 安装入口 |
| `OFFLINE_DESTRUCTIVE` | `删除历史录包.sh` | 禁止执行；会删除历史 bag |

以下内容为历史脚本说明，C10 未解决前不作为权威飞行参数表。

范围：只统计 `~/ws_offboard_control` 根目录当前直接可执行、且日常会手动调用的脚本。

## 一眼总览（历史描述）

| 脚本 | 作用 | 直接执行 |
|---|---|---|
| `run_takeoff_1m_hold.sh` | 1 m 起飞、悬停 10 s、Offboard 控制下降并切 `AUTO.LAND` | `./run_takeoff_1m_hold.sh` |
| `run_hover_1m_offboard.sh` | 1 m 起飞并保持悬停，不主动降落 | `./run_hover_1m_offboard.sh` |
| `一键启动起飞栈.sh` | 打开 5 个终端并启动整套 1 m 起降链路 | `./一键启动起飞栈.sh` |
| `开机自启动起飞栈.sh` | 登录后延时再启动五终端脚本 | `./开机自启动起飞栈.sh` |
| `开始录包.sh` | 录起飞排障 rosbag 并保存快照 | `./开始录包.sh` |
| `查看最近录包.sh` | 查看最近一次 rosbag 状态 | `./查看最近录包.sh` |
| `停止录包.sh` | 手动停止当前录包 | `./停止录包.sh` |
| `刷新录包状态.sh` | 刷新最近录包状态文件 | `./刷新录包状态.sh` |
| `删除历史录包.sh` | 删除已录 rosbag，默认保留 snapshot | `./删除历史录包.sh` |
| `起飞前自检.sh` | 检查一键启动/自启动前置条件 | `./起飞前自检.sh` |
| `启用开机自启.sh` | 打开 GNOME 登录后自启动 | `./启用开机自启.sh` |
| `禁用开机自启.sh` | 关闭 GNOME 登录后自启动 | `./禁用开机自启.sh` |

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

### `一键启动起飞栈.sh`

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

### `开始录包.sh`

一句话：

- 录制起飞排障所需话题，并自动保存本次飞行参数、脚本、节点、话题、环境和 git 快照。

输出目录：

- `~/ws_offboard_control/flight_records/日期/flight_时间戳/`

### `查看最近录包.sh`

一句话：

- 查看最近一次录包是否正在录制、干净保存或异常中断。

### `停止录包.sh`

一句话：

- 给当前录包进程发送停止信号，让 rosbag 尽量干净落盘。

### `删除历史录包.sh`

一句话：

- 删除 `flight_records` 中的 rosbag 数据，默认保留 `snapshot/` 方便复盘配置。

常用：

```bash
./删除历史录包.sh
./删除历史录包.sh latest
```

## 自启动

### `启用开机自启.sh`

一句话：

- 创建 `~/.config/autostart/ws_offboard_takeoff_stack.desktop`，让 GNOME 登录后自动执行 `开机自启动起飞栈.sh`。

### `禁用开机自启.sh`

一句话：

- 删除 GNOME 自启动入口，并关闭 rosbag shutdown guard 用户服务。

## 最常用的 3 个

```bash
cd ~/ws_offboard_control
./一键启动起飞栈.sh
./run_takeoff_1m_hold.sh
./查看最近录包.sh
```
