# Nav2 路径规划与旧地图复用（已退役，归档）

融合后导航由 Super A* 全局规划 + EGO B-spline 局部规划承担，Nav2 这条线整体退役。
`offboard_nav2_planning` 与相关脚本已移至 `Lin_shi/`，`fastlio_global_slam` 也移至
`Lin_shi/`（其地图保存/加载/重定位能力可能对复用赛前地图有用，保留待评估）。
本文保留当时的操作步骤以便需要时取回。

---

## Nav2 路径规划与旧地图复用

当前工程已在不接入 PX4 控制的前提下，完成 `FAST-LIO + 3D 重定位 + Nav2 planner-only` 验证。

当前已跑通链路：

```text
FAST-LIO
  -> /Odometry + /cloud_registered
  -> fastlio_global_slam 加载旧 3D 关键帧地图
  -> Scan Context + ICP 重定位
  -> /fastlio_global/relocalized_pose
  -> relocalized_pose_to_tf 发布 map -> camera_init
  -> odometry_tf_publisher 发布 camera_init -> body
  -> Nav2 map_server 加载旧 2D 地图
  -> Nav2 planner_server 生成 /nav2_stage1/path
```

当前边界：

- 该阶段只做路径规划和 RViz 验证，不向 PX4 发布控制指令。
- 重启后复用旧地图时，只有 2D `map.yaml/.pgm` 不够，必须同时有 `fastlio_global_slam` 保存的 3D 关键帧地图。
- `2D Pose Estimate` 当前不会校正 FAST-LIO 位姿；定位依赖 `触发重定位.sh` 的 3D 重定位结果。

### 地图文件

默认 3D 重定位地图目录：

```text
/home/robot/rong_ws/ws_offboard_control/maps/fastlio_global_3d
```

默认 2D Nav2 地图：

```text
/home/robot/rong_ws/ws_offboard_control/maps/fastlio_nav2_map.yaml
/home/robot/rong_ws/ws_offboard_control/maps/fastlio_nav2_map.pgm
```

保存 3D 重定位地图：

```bash
cd ~/rong_ws/ws_offboard_control
./保存三维地图.sh
```

检查 3D 地图关键帧数量：

```bash
cd ~/rong_ws/ws_offboard_control
./检查全局三维地图.sh
```

建议至少 `10+` 个 keyframes，最好 `30+`。如果只显示 `1` 个 keyframe，说明扫图时几乎没有移动，重定位很难可靠。

保存 2D Nav2 地图的当前推荐参数：

```bash
cd ~/rong_ws/ws_offboard_control
CLOUD_TOPIC=/Laser_map \
MAP_RESOLUTION=0.05 \
MAP_OCCUPIED_DILATION_M=0.10 \
MAP_OBSTACLE_MIN_HEIGHT_M=0.30 \
./保存二维地图.sh
```

### 重启后复用旧地图的确定启动顺序

先清理旧的 Nav2/RViz，避免重复节点导致 planner/action 混乱：

```bash
cd ~/rong_ws/ws_offboard_control
STOP_RVIZ=true ./stop_nav2_relocalized_map.sh
```

终端 1：启动 FAST-LIO，使用原来的 FAST-LIO 启动方式。启动后确认：

```bash
source /opt/ros/humble/setup.bash
source ~/rong_ws/ws_offboard_control/install/setup.bash
ros2 topic hz /Odometry
ros2 topic hz /cloud_registered
```

终端 2：启动 3D 重定位后端：

```bash
cd ~/rong_ws/ws_offboard_control
./运行全局重定位.sh
```

终端 3：加载旧 3D 地图：

```bash
cd ~/rong_ws/ws_offboard_control
./加载三维地图.sh
```

返回 `success=True` 后，轻微移动或转动雷达/飞机，触发第一次重定位：

```bash
./触发重定位.sh
```

需要看到类似：

```text
success=True
Relocalization succeeded
```

终端 4：启动 Nav2 复用 2D 地图：

```bash
cd ~/rong_ws/ws_offboard_control
RVIZ=true ./run_nav2_relocalized_map.sh
```

如果已有 RViz，只启动 Nav2 节点：

```bash
RVIZ=false ./run_nav2_relocalized_map.sh
```

再触发一次重定位，让 Nav2 的 TF 桥接节点收到新的 `/fastlio_global/relocalized_pose`：

```bash
cd ~/rong_ws/ws_offboard_control
./触发重定位.sh
```

检查 TF：

```bash
ros2 run tf2_ros tf2_echo map camera_init
ros2 run tf2_ros tf2_echo map body
```

能连续输出 transform 即表示：

```text
map -> camera_init -> body
```

链路已连通。

### RViz 路径规划验证

Nav2 RViz 配置：

```bash
ros2 run rviz2 rviz2 -d ~/rong_ws/ws_offboard_control/install/offboard_nav2_planning/share/offboard_nav2_planning/rviz/nav2_stage1_planning.rviz
```

RViz 左侧设置：

```text
Global Options -> Fixed Frame = map
```

应能看到以下显示项：

- `Static Map`
- `Nav2 Global Costmap`
- `Nav2 Planned Path`
- `Nav2 2.5D Path`
- `FAST-LIO Odometry`
- `TF`

使用顶部 `2D Goal Pose` 点目标。成功后会发布：

```text
/nav2_stage1/path
```

可用命令确认：

```bash
ros2 topic echo /goal_pose --once
ros2 topic echo /nav2_stage1/path --once
```

如果 `/goal_pose` 有数据但 `/nav2_stage1/path` 没有，优先检查：

- 是否存在重复 Nav2 节点：

  ```bash
  ros2 node list | grep -E 'planner_server|map_server|nav2_stage1_goal_to_path'
  ```

- `planner_server` 是否 active：

  ```bash
  ros2 lifecycle get /planner_server
  ```

- 目标点和飞机当前位置是否落在 costmap 可通行区域，而不是障碍或未知区域。

### 相关脚本

- [运行全局重定位.sh](运行全局重定位.sh)：启动 3D 重定位后端
- [加载三维地图.sh](加载三维地图.sh)：加载旧 3D 关键帧地图
- [触发重定位.sh](触发重定位.sh)：触发一次 3D 重定位
- [run_nav2_relocalized_map.sh](run_nav2_relocalized_map.sh)：启动 Nav2 静态地图规划和 TF 桥
- [stop_nav2_relocalized_map.sh](stop_nav2_relocalized_map.sh)：停止 Nav2 规划栈，避免重复节点
- [保存三维地图.sh](保存三维地图.sh)：保存 3D 重定位地图
- [保存二维地图.sh](保存二维地图.sh)：从点云投影保存 2D Nav2 地图
- [检查重定位状态.sh](检查重定位状态.sh)：检查重定位相关话题/TF
- [检查全局三维地图.sh](检查全局三维地图.sh)：检查 3D 地图 keyframe 数量

