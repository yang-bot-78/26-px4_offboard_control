# 最近新增脚本启动说明

所有命令默认在以下工程中执行：

```bash
cd /home/robot/rong_ws/ws_offboard_control
```

涉及雷达和飞机的脚本运行前必须拆下全部螺旋桨、确认飞机已上锁，并关闭其他 MID-360、FAST-LIO、MAVROS、Offboard 和 RViz 实例。

## 1. 现场扫图并保存普通 PCD

用途：启动 MID-360、FAST-LIO 和 RViz，实时查看扫图结果，最后保存一份规划和显示使用的 `现场地图.pcd`。

```bash
./tools/fastlio/现场扫图并保存地图.sh
```

走场完成后在脚本终端输入：

```text
保存地图
```

默认输出：

```text
maps/现场扫图/YYYYMMDD/扫图_YYYYMMDD_HHMMSS/现场地图.pcd
```

该 PCD 可用于 RViz 和规划器障碍地图，但不能代替全局重定位数据库。

## 2. 生成全局重定位数据库

用途：启动 MID-360、FAST-LIO、全局关键帧后端和 RViz，实时查看点云、全局地图与轨迹，并生成重定位需要的 CSV、全局点云和关键帧。

```bash
./tools/fastlio/现场全局建图并保存重定位库.sh
```

手动去除地图顶层（不修改原图，默认保留 z <= 3.0 m）：

```bash
./tools/fastlio/去除点云顶层.sh
```

自定义最高高度和输出文件：

```bash
./tools/fastlio/去除点云顶层.sh 输入地图.pcd 2.5 输出地图_去顶端_z2.5m.pcd
```

沿与规划地图相同的路线缓慢走场，最后回到起点并静止，然后在脚本终端输入：

```text
保存关键帧
```

默认输出：

```text
maps/fastlio_global_3d_20260810/
├── metadata.csv
├── GlobalMap.pcd
└── keyframes/
```

本次测试最低只要求 3 个明显不同的关键帧，生成阈值为平移 `0.15m` 或旋转 `5°`。建议从起点缓慢移动 `0.3-0.5m` 并改变朝向后回到起点。输入 `1` 尝试保存，输入 `0` 放弃；不足 3 个时可继续移动并再次输入 `1`，节点不会退出。

3 个关键帧仅用于开局单次重定位和单目标规划线检查；正式实飞仍应采集至少 10 个、推荐 20 个以上的关键帧。

## 3. 开局重定位一次并验证一条规划路线

用途：启动 MID-360 和 FAST-LIO，加载全局重定位库，开局只重定位一次；成功后加载静态规划地图并启动只规划 Super Planner 和 RViz。

前置文件：

```text
规划地图：maps/fastlio_global_3d_20260810/GlobalMap_去顶端_z3m.pcd
重定位库：maps/fastlio_global_3d_20260810/
```

启动：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
./tools/fastlio/一键重定位并规划验证.sh
```

重定位和 TF 检查通过后，RViz 会自动打开。使用工具栏 `2D Goal Pose` 只点击一个空旷目标点，脚本会等待：

```text
/goal_pose
/race/global_path
```

确认路径起点接近机体且不穿过障碍后，在脚本终端输入：

```text
结束测试
```

该流程固定 `enable_output=false`，不会启动 MAVROS、EV、Offboard、解锁或起飞。

## 4. 人工起飞后的 Offboard 单目标实飞验证

仅在已完成 Shadow 路径验证、手动 POSITION 悬停验证后使用。脚本不会解锁、起飞或切换模式；
飞手手动起飞并切到 POSITION 后，再手动切入 OFFBOARD，随后在 RViz 点击一个近距离目标点。

```bash
./tools/flight/人工起飞后Offboard单目标验证.sh
```

首次只点击一个距离不超过 1m 的空旷目标点。结束时先人工切回 STABILIZED、降落并上锁，再在脚本终端按 Ctrl+C。

## 查看刚保存的全局重定位地图

只加载 `maps/fastlio_global_3d_20260810/GlobalMap_去顶端_z3m.pcd` 并打开 RViz：

```bash
./tools/fastlio/查看全局重定位地图.sh
```

该脚本发布 `/fastlio_global/map`，RViz 的 Fixed Frame 为 `camera_init`。关闭 RViz 窗口后地图发布节点会自动退出。

## 4. 清洁备份的一键编译和启动

备份目录：

```text
/home/robot/rong_ws/0811bf
```

默认从空的 `build/install` 全量编译，然后启动不连接 MAVROS 的 `race_sim` 入口：

```bash
cd /home/robot/rong_ws/0811bf
./tools/一键编译并启动.sh
```

只启动控制输出关闭的 Super Planner：

```bash
cd /home/robot/rong_ws/0811bf
START_MODE=planner ./tools/一键编译并启动.sh
```

## 辅助文件

以下文件由主脚本自动调用，不需要单独运行：

```text
tools/fastlio/重定位坐标桥.py
tools/fastlio/只重定位一次后端参数.yaml
```

查看任一主脚本的参数说明：

```bash
./tools/fastlio/现场扫图并保存地图.sh --帮助
./tools/fastlio/现场全局建图并保存重定位库.sh --帮助
./tools/fastlio/一键重定位并规划验证.sh --帮助
```
