# README 历史段落（已归档）

以下内容是 2026-08-01~08-03 高频里程计与 durable trace control 实验的门禁记录。
该实验结论为不采用（`HIGH_RATE_PROMOTION_AUTHORIZED=NO`、B2 的 90 秒验证 FAIL），
相关产物与脚本已清理。保留此文件仅为追溯当时的判断依据。

---

## 当前 Durable Trace / Perf 门禁状态（2026-08-03）

高频 odom 的 production trace durable watermark/ACK 控制面仍处于非飞行
闭环阶段。权威状态和全部当前 SHA-256 位于
[DURABLE_TRACE_CONTROL_STATUS.md](DURABLE_TRACE_CONTROL_STATUS.md)。以下状态
覆盖该文档其余历史 live 记录；它不授权新的诊断 live 或飞行。

- 三路 driver/relay/probe durable-ACK 在隔离无硬件夹具中通过；production
  hardware-chain tail closure 尚无完整证据。
- 已消耗的最新无硬件 `--check` 均在 self-test 期间 fail-closed，未启动 ROS
  节点、Livox、FAST-LIO、MAVROS、硬件或 live。最新失败 handoff 为
  `relay_exit_1785705461202930138_21484`，原始 perf control 输出为
  `failed to mmap with 12 (Cannot allocate memory)`。
- 根因是 perf 默认对每个 CPU 建 mmap ring。本机有 16 个 CPU，固定 32768 页
  ring 因而被按 CPU 申请。内核策略
  `/etc/sysctl.d/70-relay-blocking-capture-perf.conf` 已由管理员加载，值为
  `kernel.perf_event_mlock_kb=132000`，文件为 `root:root 0644`；该策略只在
  单线程 mmap 模式下满足容量要求。
- 已离线验证的新版 root helper 显式使用 `perf record --per-thread`，保持既有
  trace events、32768 页、AIO=4、Dwarf 调用栈、受控退出和固定 8192-yield
  负载不变。其 SHA-256 为
  `ba78b283349c235c1619be769e45e8573cfdd64a95aec0d539bf4d4838c979ff`。
  当前 `/usr/local` 安装 helper 仍为旧的
  `d647a8081e4f1143dd446d83a837a0857b54d5d5dfc5b0e8b33bb7d53c904a50`，不含
  `--per-thread`。
- 新 helper 的 Release 构建、24 个 helper 单元测试、C++ trace-control 测试、
  Python control/closure 夹具、Bash 语法和 Python 编译均已通过，且无
  collector/perf/runner 残留。

下一步必须先按照
[ADMIN_INSTALL.md](privileged/relay_blocking_capture/ADMIN_INSTALL.md) 使用新的
root-owned staging 目录 `relay_blocking_capture_20260803_per_thread_mmap`
部署该 helper；之后还需要新的、明确授权的一次无硬件 `--check`。在获得直接的
零 LOST completion 证据、完整 production tail closure 和全部后续门禁前：

```text
READY_FOR_DIAGNOSTIC_LIVE=NO
FLIGHT_GO_NO_GO=NO
```

> **历史代际标签：以下“当前 active”描述属于 `191236/b5a32c` 代际；
> 它不是 Phase 1B-D 选定的未来 shadow 身份。**

## 当前高频 odom 问题（2026-08-01）

高频 odom 是独立诊断链路，不是下文默认起飞链路。当前 active 为：

```text
/home/robot/ws_high_rate_odom_20260801_191236_event_sample_limited
FAST-LIO SHA-256: b5a32c066507bd633c693721909e07f35169fa8c95955042f38ea9bec337cafa
ELF build-id: 3d4c035bd598a40fbbfc45240379c77ec7d59d2b
```

该 active 已完成受控晋升；旧 teardown-fixed active 保留。新 active 的
Release 构建、`10/10` CTest、无硬件探针、项目 manifest/运行时身份校验和
wrapper 静态检查均通过。`/high_rate_ev/events` 已从 Reliable 深队列改为：

```text
BEST_EFFORT + KEEP_LAST(1) + VOLATILE
```

匹配的事件捕获端使用相同 QoS；历史门禁会在 JSON 无效、事件序号丢失、
重复、乱序或出现历史 FAULT 时 fail-closed。因此当前已消除可靠事件 writer
等待 ACK/历史队列造成的反压路径。新 active 还加入了事件负载控制：非关键事件
每 8 条保留 1 条，IMU ingress、状态/故障转换、连续性和 lineage 事件保持完整；
事件环容量为 `8192`，导出批量为 `128`。诊断 wrapper 现在以默认 `nice 10`
启动 MAVROS 和 EV bridge；可用 `HIGH_RATE_START_NICE=0..19` 调整。

最新一次授权的拆桨、未解锁 90 秒诊断 live 位于：

- [relay_diagnostic_nonflight_90s_20260801_203326](runtime/relay_diagnostic_nonflight_90s_20260801_203326)
- [最新诊断审计](runtime/relay_diagnostic_nonflight_90s_20260801_203326/DIAGNOSTIC_AUDIT.md)

本次在第 3 阶段的 `/Odometry/propagated` 频率门禁处 fail-closed，测得
`7.895 Hz`，最大间隔 `2.366 s`，未启动 MAVROS/EV bridge，90 秒窗口运行
时间为 `0 s`。因此本轮没有验证 `nice 10` 对 bridge/MAVROS 启动调度的影响；
该次授权已消耗且没有重试。

上一轮授权的拆桨、未解锁 90 秒诊断 live 位于：

- [relay_diagnostic_nonflight_90s_20260801_201504](runtime/relay_diagnostic_nonflight_90s_20260801_201504)
- [完整诊断审计](runtime/relay_diagnostic_nonflight_90s_20260801_201504/DIAGNOSTIC_AUDIT.md)

上一轮结果为 fail-closed，未进入 90 秒计时窗口，窗口运行时间为 `0 s`：

- 传感器和 fresh relay 初始新鲜度检查通过；`/Odometry/propagated` 初测
  `29.999 Hz`。
- MAVROS 已连接，始终未解锁且不在 `OFFBOARD`；`EKF2_EV_CTRL=11`，EV
  velocity 未融合。
- 三个 pre-bridge 事件历史门禁全部通过；最终 31,090 条事件没有 JSON 错误、
  事件序号断裂、recorder drop 或 schema reject。
- 事件捕获在约 55.5 秒内产生 `79,618,421 bytes`，约 `1.43 MB/s`；相比上一轮
  约 `6.5 MB/s` 降低约 78%，证明采样/批量限制有效。
- 覆盖故障的 358.647 ms 调度区间内，FAST-LIO、fresh relay、MAVROS 的 process
  runqueue wait 分别为 `17.468 ms`、`20.509 ms`、`38.632 ms`。
- 同期 relay 在 0.500 秒内收到并转发 93 条 IMU，正常值应约为 100；没有
  stale、future、invalid-stamp 或 relay reject。
- FAST-LIO 相邻有效 IMU 出现 `25.056 ms` 源时间戳空洞和 `29.265 ms`
  callback 空洞，仍超过 `max_imu_dt_s=0.020`，因此正确触发
  `fault_reason=IMU_GAP`。
- `IMU_GAP` 发生在 EV health monitor 完成初始化前约 1.028 秒；高频 propagated
  输出已经停止，所以最终看到的 `waiting_for_ev` 仍是下游现象。

安全与 teardown 正常：46 条 MAVROS 状态全部 connected/disarmed/non-OFFBOARD，
没有 setpoint publisher，所有组件退出，目标进程残留为 0，active 生成日志
未改变，teardown 后项目校验仍为 PASS。

当前结论和边界：

- BestEffort/浅队列和事件采样修复有效，明显降低了事件导出负载和调度等待，
  并已使最新候选链路进入 90 秒窗口，但不能保证窗口内持续 HEALTHY。
- 当前 active 的剩余问题仍是运行期间 200 Hz IMU 完整性偶发越过 20 ms 门限。
  序列化/导出移出 FAST-LIO 和启动期 `nice 10` 隔离均已完成硬件 live 验证；两项
  调度减载尚未消除自然 IMU 缺口。未晋升候选已通过一次受控 30 ms 缺口的
  `IMU_GAP -> RESTART -> INITIALIZING -> HEALTHY -> 恢复发布` 硬件验证和一次无
  MAVROS 90 秒稳定性门禁；但最新 MAVROS/PX4 90 秒非飞行验证因 `IMU_TIMEOUT`、
  约 9.25 秒 EV health 中断和事件断序失败，尚无端到端 90/600 秒稳定性证据。
- 在新的明确授权前，不应继续 90 秒或 600 秒 live。
- 当前 `FLIGHT_GO_NO_GO=NO`，该结果不授权装桨、解锁、OFFBOARD 或飞行。

### 序列化/导出移出 FAST-LIO（静态候选）

已实现一个未晋升的候选根：
`/home/robot/ws_high_rate_odom_20260801_204248_event_export_outproc`。
FAST-LIO 现在只向 `/high_rate_ev/events_raw` 发布带固定 40 字节版本头的
`std_msgs/msg/UInt8MultiArray` 批次；独立进程 `high_rate_event_exporter` 在进程
外执行 JSON 序列化，并继续向 `/high_rate_ev/events` 提供原有 JSONL 协议。原有
事件 QoS（BestEffort、KeepLast(1)、Volatile）保持不变。

候选验证结果：

- Release 构建成功，CTest `11/11` 通过，包含 round-trip、坏 magic/版本、截断和
  长度不匹配测试。
- `fastlio_mapping` 不再包含 `serialize_phase3_event_batch` 符号或实时 JSON
  构造；该符号仅存在于 `high_rate_event_exporter`。
- 无硬件探针启动 `/laser_mapping` 和 `/high_rate_event_exporter`，确认 raw/JSON
  节点拓扑；向两者发送 SIGINT 后均 `exit=0`，无残留目标进程。
- 最新受控 restart bootstrap 修复候选 SHA-256：
  `618c2b4db8b1f9df648f31cd51af3bc25511f3d94c2b8f707ec33fc3befdafd1`，ELF
  build-id：`4f2caf70e476e70868067e0e42f4797bc0665892`；exporter SHA-256 仍为：
  `158cc01e8205d12fe3a6528eb41a06fb52eefc835b8b99f3262f6d9f72a3ef3d`。
- 下述硬件 A/B 使用的是 bootstrap 修复前的候选：SHA-256
  `6f44097d8248192ab421a0704228d7998c83337382634ebd8947ef301133d1bc`，
  build-id `464ea0efc3cc32c4b95e46cbe04fb67ee644714b`。因此 A/B 可以证明触发源，
  但不能证明最新 bootstrap 修复的 live 恢复结果。

### relay depth 1/8 硬件 A/B（2026-08-02）

在新的明确授权下执行了 depth 1、depth 8 和一次明确授权的 depth 8 重测。三次均为
拆桨台架，脚本未启动 arm、mode switch、setpoint 或起飞节点，teardown 后目标进程
残留均为 0。depth 1 和 depth 8 重测到达 MAVROS 阶段，均确认 connected、disarmed、
non-Offboard 和 `EKF2_EV_CTRL=11`；首次 depth 8 在 MAVROS 前由速率门禁终止。三次都
未进入正式 90 秒窗口，窗口运行时间均为 `0 s`，不能作为 90 秒稳定性通过证据。

| 运行 | probe 样本 / >20 ms gap / 最大 source 间隔 | relay IMU 最大 source 间隔 | 结果 |
|---|---|---|---|
| [depth 1](runtime/relay_diagnostic_nonflight_90s_20260802_depth1_124000) | `16718 / 0 / 11.093 ms` | `26.118 ms` | `IMU_GAP`；EV health 未就绪，fail-closed |
| [depth 8](runtime/relay_diagnostic_nonflight_90s_20260802_depth8_124353) | `7433 / 0 / 8.125 ms` | `7.819 ms` | 无 FAST-LIO fault transition；propagated 短窗速率 `28.696 Hz`，fail-closed |
| [depth 8 重测](runtime/relay_diagnostic_nonflight_90s_20260802_depth8_repeat_124651) | `16479 / 0 / 12.660 ms` | `18.060 ms` | 后段出现 `IMU_TIMESTAMP_STALE`、`IMU_TIMEOUT`，EV 输入停顿约 `332 ms`，fail-closed |

该硬件同源对照已经区分了最初的两种假设：depth 1 故障时，直接订阅
`/livox/imu` 的 KeepLast(64) probe source timestamp 连续，而 relay depth 1 输入出现
`26.118 ms` source 空洞；改为 depth 8 后，两次 relay source 最大间隔都低于 `20 ms`。
因此原 `IMU_GAP` 的主因是 relay 输入侧 BestEffort KeepLast(1) 覆盖，不是 Livox
driver/设备未发布。

depth 8 仍不是端到端修复。重测中 probe source 和 relay source 都连续，但 probe
arrival 最大间隔为 `34.650 ms`，随后 FAST-LIO 发生 timestamp stale/timeout，EV health
以 `stale_input age=0.332s` 转回 FAULT。这表明深队列能保住 source 序列，却不能消除
CPU 调度停顿和消息过龄；下一修复优先级应转向隔离 FAST-LIO/relay 的调度资源与启动期
负载，而不是继续增大队列。

受控 restart 也获得了首次 live 证据。depth 1 的事件 `18413` 为
`HEALTHY -> FAULT(IMU_GAP)`；连续 IMU 满足门槛后，事件 `18423` 进入
`FAULT -> INITIALIZING`，证明内部 `restart()` 已被调用。但约 `36.5 ms` 后，事件
`18439` 又以 `MISSING_PREDECESSOR` 进入 FAULT；之后新 posterior 被拒绝，未恢复到
HEALTHY。当前问题已从“IMU_GAP 后不会触发 restart”收敛为“新 epoch 尚无 posterior
baseline 时，advance/watchdog 提前把缺少 predecessor 判为硬故障”。

该竞争窗口现已在最新候选中修复。受控 restart 会设置仅限新 epoch 的 bootstrap
等待状态；校正时间早于新 epoch 首个 IMU、因而依赖已清除旧轨迹的 posterior 会被
拒绝，但保持 `INITIALIZING`、publication admission 关闭，且不会再触发
`MISSING_PREDECESSOR`。首个可由新 IMU buffer 重放的 posterior 成功安装 baseline 后
退出 bootstrap 等待；只有下一 generation 完成 continuity 比较并进入 `HEALTHY` 后
才重新开放 admission 和恢复发布。普通首次启动、人工 restart 和真正丢失所需
predecessor 的硬故障语义未放宽。

新增回归测试按 live 顺序覆盖：`IMU_GAP -> 连续 IMU streak -> restart -> watchdog
先运行 -> 旧 epoch posterior 被拒绝但不 FAULT -> 新 posterior baseline -> 下一
generation HEALTHY -> admission reopen`。Release 全量构建和 CTest `11/11` 通过；
新安装候选的无硬件双进程探针位于
`/home/robot/ws_high_rate_odom_20260801_204248_event_export_outproc/evidence/restart_bootstrap_no_hardware_20260802_130415`，
`fastlio_mapping` 与 exporter 均 `exit=0` 且无残留。后文记录的受控 30 ms 注入
已补齐该候选的单次硬件恢复证据，但尚未完成 90/600 秒稳定性验证，候选不得晋升
active。

随后授权执行了一次最新二进制的 depth 8 硬件恢复验证：
[depth8_bootstrap_recovery_131133](runtime/relay_diagnostic_nonflight_90s_20260802_depth8_bootstrap_recovery_131133)。
该运行在启动 MAVROS 前由 fault-history 门禁终止，正式 90 秒窗口为 `0 s`；没有启动
MAVROS/EV bridge、arm、Offboard 或 setpoint 节点，所有组件正常退出且残留为 0。

本次确实观察到一次完整的软故障恢复，但没有触发 IMU_GAP bootstrap：

```text
event 2117: HEALTHY -> FAULT(IMU_TIMEOUT)
约 311 ms 后: RECOVERING
再约 2.000 s 后: HEALTHY
event 3112 起: publication resumed（共观察到 8 个后续 PUBLISHED decision）
RESTART event: 0
IMU_GAP transition: 0
```

source probe 的 `7551` 个样本没有 >20 ms source gap，最大 source 间隔为
`8.821 ms`；但其 callback arrival 曾停顿 `334.510 ms`。同一停顿期间 depth 8 relay
输入队列不足以容纳约 67 个 200 Hz IMU，relay 最大 source 间隔达到 `54.065 ms`。
因此本次验证了 `IMU_TIMEOUT -> RECOVERING -> HEALTHY -> 恢复发布`，同时再次证明
depth 8 只能覆盖约 40 ms 排队，无法抵御数百毫秒调度停顿。由于 fault 先以
`IMU_TIMEOUT` 清理状态，后续 source 空洞没有进入 `IMU_GAP` 受控 restart 路径；最新
bootstrap 修复在该次自然停顿运行中仍为未覆盖，而不是失败。其后已用严格位于
20..50 ms 之间的受控单次注入补齐，结果如下。

### depth 8 受控 IMU_GAP 恢复验证（2026-08-02）

在新的单次 live 授权下，使用 IMU/LiDAR relay `KeepLast(8)`、独立 source probe 和
`30 ms` 目标执行专用恢复验证。证据位于
[imu_gap_recovery_depth8_20260802_054347](../ws_high_rate_odom_20260801_204248_event_export_outproc/runtime/imu_gap_recovery_depth8_20260802_054347)。
该路径只启动 Livox、source probe、fresh relay、FAST-LIO、进程外事件 exporter 和
证据捕获；未启动 MAVROS、EV bridge、arm、Offboard、setpoint 或起飞节点。

relay Trigger 成功执行一次，以 source timestamp 为准形成 `30.144052 ms` 缺口并
主动丢弃 `5` 条 IMU；目标严格位于 `20 ms` IMU gap 与 `50 ms` timeout 之间。独立
probe 共记录 `7684` 条原始 IMU，source gap 计数为 `0`，最大 source/arrival 间隔
分别为 `8.274545 ms` 和 `9.601613 ms`，因此本次 fault 可确定归因于受控注入，而非
Livox 自然漏发或本轮调度停顿。

恢复事件门禁通过，关键序列为：

```text
event 2031: HEALTHY -> FAULT(IMU_GAP)
event 2041: writer_site=RESTART
event 2042: FAULT -> INITIALIZING
event 2131: first new HEALTHY observation
event 2149: first TIMER_FINAL_DECISION/PUBLISHED after recovery
```

HEALTHY 后共捕获 `29` 个 PUBLISHED decision；`3908` 条 trigger 后事件中无无效 JSON、
事件序号断裂、额外 fault transition 或 `MISSING_PREDECESSOR`。最终
`RUN_RESULT.txt` 为 `FINAL_EXIT_CODE=0`，所有目标组件正常退出、残留为 `0`，active
生成日志未变化。由此，最新 bootstrap 候选的
`IMU_GAP -> RESTART -> INITIALIZING -> HEALTHY -> 恢复发布` 已完成一次硬件闭环验证。

该结果只证明确定性单次故障恢复，不证明长期无自然 gap、90 秒链路稳定性或飞行可用性；
候选仍未晋升 active，`FLIGHT_GO_NO_GO=NO`，并且本次授权不允许自动重试。

### 候选冻结与无 MAVROS 90 秒稳定性（2026-08-02）

受控恢复通过后已冻结当前候选。冻结记录位于
[frozen_candidate_20260802_135631](../ws_high_rate_odom_20260801_204248_event_export_outproc/verification/frozen_candidate_20260802_135631)，
覆盖 `97` 个 FAST-LIO 源码文件、`9` 个 relay/现有门禁文件和 `6` 个运行产物。
运行前后共 `112` 项 SHA-256 复核全部通过；本轮没有修改 FAST-LIO、relay、source
probe、现有健康/事件门禁、配置或安装二进制。冻结身份仍为：

```text
fastlio_mapping: 618c2b4db8b1f9df648f31cd51af3bc25511f3d94c2b8f707ec33fc3befdafd1
build-id: 4f2caf70e476e70868067e0e42f4797bc0665892
event exporter: 158cc01e8205d12fe3a6528eb41a06fb52eefc835b8b99f3262f6d9f72a3ef3d
livox_fresh_relay: bf5b145dd1158e44c90de0c8b451cac4e1a4223756a907aeb87dbbf32991e92d
```

随后完成一次不注入缺口、不启动 MAVROS/EV bridge/Offboard/arm/setpoint 的 depth 8
硬件稳定性验证，完整证据和审计位于
[stability_no_mavros_90s_20260802_135631](../ws_high_rate_odom_20260801_204248_event_export_outproc/runtime/stability_no_mavros_90s_20260802_135631)。
runner 的周期安全检查使原始采集持续 `124.871750207 s`；门禁固定评估 HEALTHY
准入点之后精确前 `90.000000000 s`，其余原始记录作为补充证据保留。

精确 90 秒结果：

| 指标 | 结果 |
|---|---:|
| propagated odom 样本 | `2700` |
| 接收 / header timestamp 频率 | `30.001258 / 30.000337 Hz` |
| 最大接收 / timestamp 空窗 | `37.527541 / 44.234807 ms` |
| 平均 / 最大延迟 | `9.717370 / 15.716110 ms` |
| 事件记录 / JSON 错误 / 序号断裂 | `64399 / 0 / 0` |
| fault transition / RESTART | `0 / 0` |
| source probe 样本 / >20 ms gap | `18001 / 0` |
| 最大 source / callback 间隔 | `10.304214 / 12.241075 ms` |

窗口内 `2701` 条 high-rate diagnostics 全部 HEALTHY，fault/timeout/reject 计数均为
`0`；relay IMU/LiDAR 各 `180` 条 diagnostics 全部 FORWARDING，注入状态始终为
DISABLED，注入 request/drop/actual gap 均为 `0`。所有目标组件 `exit=0`，残留为
`0`，active 生成日志未变化。

初始 assessor 因 diagnostics 实际保存在 `logs/*.log`、却只查找
`captures/*.yaml` 而返回 `2`；原始流完整存在，初始失败文件已保留。修正后的外部
离线 assessor 仅调整证据路径和精确 90 秒截取，`4/4` 合成测试及同一份 raw 证据
复核通过，没有重跑 live，也没有修改冻结运行组件。

结论：冻结候选已通过本次“无注入、无 MAVROS”的严格 90 秒稳定性门禁；仍未验证
MAVROS/PX4/Offboard 或飞行链路，`FLIGHT_GO_NO_GO=NO`。

### MAVROS/PX4 90 秒非飞行验证（2026-08-02）

随后在新的单次 live 授权下，使用同一冻结候选、relay depth 8、source probe 开启、
注入关闭，执行了 MAVROS/PX4 90 秒非飞行验证。完整证据和审计位于
[mavros_px4_nonflight_90s_20260802_142709](../ws_high_rate_odom_20260801_204248_event_export_outproc/runtime/mavros_px4_nonflight_90s_20260802_142709)。
窗口完整运行 `90.337702142 s`，未自动重试。

安全边界通过：窗口内 `90` 条 MAVROS 状态全部 connected、disarmed、`STABILIZED`；
`53` 次持久安全检查全部 SAFE，最大状态年龄 `0.998473 s`，低于 `3 s` 门限；
`EKF2_EV_CTRL=11` 已通过 MAVROS 参数 API 只读确认。没有 arm、Offboard、mode switch、
setpoint、vehicle command 或 takeoff 节点/发布者。所有目标组件 `exit=0`，残留为 `0`，
active 日志未改变，冻结文件运行后复核仍全部通过。

功能稳定性未通过。窗口开始约 `1.448 s` 后，事件 `32372` 出现：

```text
HEALTHY -> FAULT(IMU_TIMEOUT), fault_reason=11
```

FAST-LIO 约 `2.067 s` 后恢复 HEALTHY；下游 EV health 因 `stale_input age=0.332s`
进入 FAULT，并因 `7.5 s` 连续健康恢复门槛，在约 `9.249 s` 后才重新开放
`/Odometry/high_rate_healthy -> /mavros/odometry/out`。窗口结束后的 propagated
短窗频率只有 `28.710 Hz`，最大观测空窗 `0.310 s`，低于 `29.5..30.5 Hz` 门禁。

事件捕获共 `102480` 条有效 JSON，但存在两段序号断裂：

```text
32347 -> 32368  （缺 20 个序号）
52214 -> 52289  （缺 74 个序号）
```

source probe 在窗口内记录 `18068` 条 IMU，source gap 计数增量为 `0`，最大 source
间隔 `10.616766 ms`；relay 最终 received/published 均为 `30956`，无 stale/future/
invalid/injected/policy drop，最大 source 间隔同为 `10.616766 ms`，注入状态始终
DISABLED。一次 probe callback arrival 间隔达到 `147.133834 ms`，但对应 source
间隔仅 `3.980842 ms`，只能证明 probe 进程调度停顿，不能归因于 Livox 漏发。

当前证据确认 FAST-LIO watchdog timeout、恢复过程和 EV 输出中断，也排除了 source
timestamp 缺口及 relay policy drop；但 fault 前事件恰有断序，调度采样也不足以确定
FAST-LIO ingress 中断的精确上游边界。MAVROS 日志另有 AUTOPILOT_VERSION 请求超时后
回退默认 capability，以及未解码 FCU EVENT；连接未中断，但这些仍需在飞行前解决。

结论：本轮“PX4 连接与非飞行安全约束”通过，“MAVROS/PX4 外部视觉 90 秒稳定性”
未通过，`FLIGHT_GO_NO_GO=NO`，不授权装桨、解锁、Offboard 或飞行。

此前在明确授权下完成的一次拆桨、未解锁硬件端到端验证位于：
[relay_diagnostic_nonflight_90s_20260801_210139](runtime/relay_diagnostic_nonflight_90s_20260801_210139)。
raw -> exporter -> JSON 事件链正常工作，进入了 MAVROS/EV bridge；事件门禁在
三个 pre-bridge 阶段均通过，捕获 `74,901` 条有效 JSON 事件、`196,219,954`
字节，序号无断裂。约 39 秒后 FAST-LIO 发生一次 `IMU_GAP`，健康状态进入
`FAULT`，90 秒窗口 fail-closed；该结果不是 exporter 解析或序号错误。MAVROS
全程 connected/disarmed/non-OFFBOARD，`EKF2_EV_CTRL=11`，无 setpoint，teardown
残留为 0，项目校验仍为 PASS。详见[诊断审计](runtime/relay_diagnostic_nonflight_90s_20260801_210139/DIAGNOSTIC_AUDIT.md)。

### 旧 active 中 IMU_GAP 不自动恢复的原因

本节描述 2026-08-01 active 的历史行为，不适用于上文新增受控 restart 的
`install_restart` 候选。旧状态机将 `IMU_GAP` 视为丢失必需积分区间的硬故障，
而不是短暂超时。检测到
interval integrity failure 时会以 `restart_required=true` 调用 `enter_fault()`；
随后清空 active generation、pending posterior、candidate 和 IMU buffer，并设置：

```text
restart_required=true
recovery_required=false
state=FAULT
```

此后 IMU ingest、LiDAR posterior accept 和 propagator process 都会因
`restart_required` 在入口直接拒绝，因此无法生成新的恢复 generation，也不会进入
`RECOVERING`。虽然 propagator 提供 `restart()`，当前 `laserMapping.cpp` 没有运行时
调用路径，实际只能通过重启 FAST-LIO 进程解除锁存。

本次 live 的事件证据与源码行为一致：首次 `IMU_GAP` 出现在事件 `41373`；之后
`16,494` 次 IMU ingress 被标记为 `IMU_INGEST_REJECTED`，`102` 个 posterior 被
标记为 `POSTERIOR_REJECTED`，`imu_buffer_count=0`，`recovery_healthy_s=0`。因此
“未自行恢复”是当前 fail-closed 设计的确定行为，不是 exporter 导致的恢复失败。

因此候选实现选择受控内部 restart：丢弃故障前 generation，从新的连续 IMU 区间和
新 LiDAR posterior 重新初始化，不能跨越缺失 IMU 区间继续旧状态。修复前 live 暴露的
`MISSING_PREDECESSOR` 竞争窗口及最新静态修复见上文 2026-08-02 A/B 结果。

### IMU_GAP 所在层级（2026-08-01 live 证据）

本次故障的首个 watchdog FAULT 为事件 `41373`。故障前最后两个有效
`IMU_INGRESS` 的 FAST-LIO source timestamp 为：

```text
event 41358: 1785589394560044541 ns
event 41370: 1785589394600382129 ns
source gap: 40.337588 ms
```

FAST-LIO 的 `max_imu_dt_s` 为 `20 ms`，因此 propagator 在
`interval_integrity_failure()` 的 `advance()` 路径中检测并报告 `IMU_GAP`。
这说明 propagator 是故障的检测层，不是缺口的产生层。

同一对样本的 middleware receive 间隔为 `49.132038 ms`，callback entry 间隔为
`49.434678 ms`；source timestamp 本身已经跳过约 `40 ms`，所以不能归因于
FAST-LIO 内部 timestamp 重写或单纯 JSON exporter 延迟。

故障前后约 `0.5 s` 的 relay 诊断窗口中：

```text
fresh_relay IMU received:  +90
fresh_relay IMU published: +90
stale/future/invalid drop: +0
FAST-LIO middleware ingress: 90
```

正常 200 Hz 应约为 100 条。relay 收到多少就发布多少，FAST-LIO middleware
也收到同样的 90 条，因此 relay freshness policy 和 relay -> FAST-LIO 输出段
没有额外丢包。最可能的缺口边界是：

```text
Livox driver publish / DDS BestEffort KeepLast(1)
        -> fresh_relay 的 /livox/imu 输入 callback
```

该区间调度采样还显示 Livox driver、fresh relay、FAST-LIO、MAVROS 的 runqueue
等待分别约 `431 ms`、`179 ms`、`498 ms`、`468 ms`（故障前后约 `0.5 s` 窗口），
支持系统 CPU 调度饥饿导致输入侧 BestEffort 深度 1 覆盖/漏取的判断。当时证据
尚不能进一步区分“Livox driver 未发布”与“driver 已发布但 relay 输入 DDS
覆盖”；上文 2026-08-02 source probe + depth 1/8 对照已完成该层区分。

### 后续实现状态：输入 A/B 与 IMU_GAP 受控重启

已在源码中加入两项候选改动并完成上述硬件 A/B，尚未晋升 active：

- `livox_fresh_relay` 新增 `imu_input_queue_depth` 和 `lidar_input_queue_depth`
  参数，范围 `[1,64]`，默认仍为 `1`。wrapper 可通过
  `HIGH_RATE_RELAY_IMU_INPUT_DEPTH=8`（以及对应 LiDAR 参数）做更深输入队列
  A/B；relay diagnostics 同时记录 `last_source_timestamp_ns`、
  `max_source_interarrival_ns` 和 `source_gap_over_threshold`。
- 新增可选 `livox_imu_source_probe.py`，使用独立 BestEffort 深队列直接订阅
  `/livox/imu`，逐条记录 `probe_sequence`、source timestamp、monotonic arrival、
  source/arrival interarrival 和累计 gap 数。它默认不启动；A/B 时使用：

  ```bash
  cd /home/robot/ws_high_rate_odom_20260801_204248_event_export_outproc
  HIGH_RATE_LIVOX_IMU_SOURCE_PROBE=YES \
  HIGH_RATE_RELAY_IMU_INPUT_DEPTH=8 \
  ./run_event_export_live.sh --relay-diagnostic-90s
  ```

  wrapper 会把 probe 输出保存为当前 run 的
  `captures/livox_imu_source_probe.jsonl`，并纳入统一 teardown。
- FAST-LIO propagator 在 `IMU_GAP` 后保持停止发布，拒绝旧 generation 的输入；
  只有收到默认 `5` 个（可由
  `high_rate_ev.restart_recovery_min_imu_samples` 调整、最小 `2` 个）连续且
  source timestamp 间隔不超过 `max_imu_dt_s` 的新 IMU，才调用受控 `restart()`。
  restart 清除旧 generation/积分 buffer，当前样本作为新 epoch 首样本；设计上之后必须
  重新收到 LiDAR posterior，经历 `INITIALIZING` 和新的 cross-generation 比较后
  才重新进入 `HEALTHY` 并恢复发布。修复前 live 暴露的新 posterior baseline 前
  `MISSING_PREDECESSOR` 竞争窗口已由 bootstrap 门控处理；其他 hard fault 不走此
  自动路径。

候选状态机单元测试已覆盖：`IMU_GAP -> 连续 IMU streak -> restart -> 新 posterior
baseline -> cross-generation HEALTHY`。A/B 队列参数和源间隔 diagnostics 也已通过
`px4_ros_com` relay 单元测试。最新回归已覆盖 restart 后、首个新 posterior 到达前
watchdog/process 先运行和旧 epoch posterior 到达的竞争窗口；完整 FAST-LIO CTest
`11/11` 通过；上文受控 depth 8 live 已补齐最新二进制的单次硬件恢复验证。

A/B 判定规则固定如下：

| source probe | relay depth1 | relay depth8 | 结论 |
|---|---|---|---|
| source timestamp 已有 gap | 同样有 gap | 同样有 gap | Livox driver/设备发布前已缺样本 |
| source 连续 | 有 gap | 无 gap | relay 输入 BestEffort KeepLast(1) 被覆盖 |
| source 连续 | depth1/8 均有 gap | depth1/8 均有 gap | relay callback/系统调度仍不足，需进一步隔离 CPU |

probe 专项测试、无硬件启动和 SIGINT 清理均已通过。硬件对照选择第二行结论：source
连续、depth 1 有 gap、depth 8 无 source gap，确认 KeepLast(1) 覆盖；同时新增的调度
过龄问题意味着 depth 8 只能作为必要缓冲，不能据此晋升或授权飞行。

候选尚未替换 active，也不构成飞行授权；当前飞行结论仍为
`FLIGHT_GO_NO_GO=NO`。

