# Scripts Quick Guide

> **Phase 1B-I-P 安全横幅（2026-08-03）**  
> `VISION_POSE` 稳定入口源码已静态收敛到 `/Odometry/healthy`：vision ON，
> MAVROS ODOMETRY/direct PX4 EV OFF，源时间戳保留。独立审查仅通过源码静态
> 收敛；断言为 textual-not-structural，runtime-global writer 唯一性未验证，
> 禁止据此运行起飞、一键栈或 autostart 配置脚本。
> `204248/618c2b` 仍仅为 shadow，active/runner 仍为 B1。
> `C10_FLIGHT_PROFILE_INTENT=UNRESOLVED`，下文参数均为历史描述。
> `NEXT_GATE_AUTHORIZED=NO`、`READY_FOR_DIAGNOSTIC_LIVE=NO`、
> `FLIGHT_GO_NO_GO=NO`。

## 安全分类（权威）

| 类别 | 脚本 | 当前处置 |
| --- | --- | --- |
| `FLIGHT` | `一键启动起飞栈.sh`, `run_takeoff_1m_hold.sh`, `run_hover_1m_offboard.sh`, `开机自启动起飞栈.sh` | 禁止执行 |
| `FLIGHT_CONFIG` | `启用开机自启.sh`, `禁用开机自启.sh` | 禁止执行；会修改持久入口 |
| `NONFLIGHT_HARDWARE` | `开始录包.sh`, `停止录包.sh` | 需要未来明确授权 |
| `OFFLINE` | `查看最近录包.sh`, `刷新录包状态.sh`, `起飞前自检.sh` | 本阶段未执行 |
| `OFFLINE_DESTRUCTIVE` | `删除历史录包.sh` | 禁止执行 |

以下原文为历史快速指南，C10 未解决前不作为权威飞行参数。

这份文档只保留最核心信息：脚本做什么、什么时候用、怎么执行。

## 当前主脚本（历史描述）

| 脚本 | 作用 | 什么时候用 |
|---|---|---|
| `一键启动起飞栈.sh` | 一键启动 `MID360 -> FAST-LIO -> MAVROS -> rosbag -> 0.5 m 起降` | 正常试飞主入口 |
| `run_takeoff_1m_hold.sh` | 0.5 m 起飞、悬停、Offboard 下降、切 `AUTO.LAND`、落地后上锁 | 已手动启动其它链路时 |
| `run_hover_1m_offboard.sh` | 1 m 起飞并保持悬停，不主动降落 | 悬停稳定性测试 |
| `开始录包.sh` | 录制排障 rosbag 和配置快照 | 需要回看高度/漂移/控制问题 |
| `查看最近录包.sh` | 查看最近一次录包状态 | 起飞前或排障时 |
| `停止录包.sh` | 手动停止当前录包 | 结束测试或关机前 |
| `起飞前自检.sh` | 检查自启动/一键启动依赖 | 自启动前、改脚本后 |
| `启用开机自启.sh` | 打开 GNOME 登录后自启动 | 需要登录后自动拉起五终端 |
| `禁用开机自启.sh` | 关闭 GNOME 登录后自启动 | 暂停自动启动 |

## 1. 一键启动

### `一键启动起飞栈.sh`

```bash
cd ~/ws_offboard_control
./一键启动起飞栈.sh
```

它会打开：

1. `MID360 Driver`
2. `FAST-LIO`
3. `PX4 MAVROS`
4. `ROS Bag Debug`
5. `Takeoff 0.5m`

## 2. 单独起飞 / 悬停

### `run_takeoff_1m_hold.sh`

```bash
cd ~/ws_offboard_control
./run_takeoff_1m_hold.sh
```

当前关键行为：

- 相对起点上升约 `0.5 m`
- 悬停 `10 s`
- Offboard 控制下降
- 接近起点高度后请求 `AUTO.LAND`
- 等待落地后自动 disarm 停桨

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

### `开始录包.sh`

```bash
cd ~/ws_offboard_control
./开始录包.sh
```

输出目录：

```text
~/ws_offboard_control/flight_records/日期/flight_时间戳/
```

查看最近状态：

```bash
./查看最近录包.sh
```

手动停止：

```bash
./停止录包.sh
```

## 4. 登录后自动启动

### `启用开机自启.sh`

```bash
cd ~/ws_offboard_control
./启用开机自启.sh
```

它会创建 GNOME 自启动入口，登录后调用：

```bash
./开机自启动起飞栈.sh
```

### `禁用开机自启.sh`

```bash
cd ~/ws_offboard_control
./禁用开机自启.sh
```

## 最常用的 3 个

```bash
./一键启动起飞栈.sh
./run_takeoff_1m_hold.sh
./查看最近录包.sh
```
