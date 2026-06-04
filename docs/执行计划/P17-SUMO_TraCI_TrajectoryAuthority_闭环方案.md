# P17 - SUMO TraCI Trajectory Authority 闭环方案


## Current Implementation Status

P17 current implementation status: implemented_green. The active closed-loop runner `run_sumo_trajectory_authority_simulation(...)` and the formal artifact wrapper `run_p17_sumo_artifact_bundle(...)` are wired into the public API. The formal bundle writes `trajectory.csv`, `events.jsonl`, `sanity.jsonl`, `realization.jsonl`, `artifact_manifest.json`, `scenario_report.json`, `run_report.md`, and the SUMO network/config files under `sumo/`. This remains a minimal SUMO coupling closure and does not include the P18 dual-track paper experiment grid.
## 0. 文档定位

本文档用于冻结 P17 的大框架方案，作为后续拆分代码执行计划、实现 SUMO 接入、复查方案取舍和修改控制模式的依据。

P17 的主目标不是把 CORMC 场景简单导出给 SUMO 自行仿真，也不是把 CORMC 轨迹单向 replay 给 SUMO 看动画，而是建立：

```text
SUMO realized state
    -> CORMC 算法计算 active controlled vehicles 的控制轨迹
    -> TraCI executor 写回 SUMO
    -> SUMO 推进背景交通与交互环境
    -> 记录 commanded vs realized、碰撞、teleport、轨迹和可视化证据
```

P17 当前主方案为：

```text
trajectory-authority TraCI closed loop
```

也就是：

```text
CORMC 强控 active controlled vehicles；
SUMO 控制背景交通、路网、碰撞/teleport 检测、可视化和 realized state；
TraCI 只存在于 SUMO adapter / executor 层，不进入 APS、CUC、CMC、纵向模型或横向模型内部。
```

## 1. 方案总原则

### 1.1 不让 SUMO 变成纯 replay

P17 不控制所有车辆。若所有车辆都由 CORMC 每步强制轨迹，SUMO 只剩可视化器和几何检查器，闭环实验价值会显著下降。

P17 的有意义边界是：

```text
active controlled vehicles:
    CORMC 控制其纵向/横向轨迹。

background vehicles:
    SUMO 控制其 car-following、lane-changing、响应和交通环境演化。
```

active controlled vehicles 与 background vehicles 之间发生交互。如果 CORMC 强控车辆轨迹导致与背景车碰撞，应记录为 CORMC failure，而不是让 SUMO 静默修正。

### 1.2 不把 TraCI 命令写进算法

P17 必须增加一层执行器接口。CORMC 算法输出中性控制命令，不输出 TraCI API 调用。

错误设计：

```text
APS / CUC / CMC / Step7 / Step8 直接调用 traci.vehicle.moveToXY(...)
```

正确设计：

```text
CORMC algorithm
    -> ControlledTrajectoryCommand
    -> SumoTrajectoryExecutor
    -> traci.vehicle.moveToXY / setSpeed / setPreviousSpeed
```

这样做的目的：

1. 保持 CORMC 算法不依赖 SUMO。
2. 允许未来切换 executor，例如 `moveToXY`、`changeSublane`、SUMO-native control。
3. 允许同一套 CORMC 输出继续服务 P11/P14/P16 artifact。
4. 便于记录 commanded vs realized，而不是把 SUMO 结果误当成算法内部状态。

## 2. P17 分层架构

建议新增 SUMO 接入层，不改 APS/CUC/CMC 等算法模块的职责。

```text
cormc/
    sumo/
        network.py          # PlainXML / net generation, edge/lane metadata
        mapping.py          # x_global/y <-> edge/lane/pos/xy 映射
        commands.py         # 中性 command dataclass
        adapter.py          # SUMO realized state -> SimulationState
        executor.py         # command -> TraCI API
        spawn.py            # P16 seeded spawn -> TraCI insertion
        loop.py             # closed-loop runner
        monitoring.py       # collision, teleport, mismatch, gate
        artifacts.py        # manifest/report/trace 输出
```

核心数据流：

```text
SUMO realized state at t
    -> SumoStateAdapter
    -> CORMC SimulationState(t)
    -> CormcEngine.advance_one_step()
    -> CommandAdapter
    -> ControlledTrajectoryCommand / SpawnCommand
    -> SumoExecutor
    -> simulationStep(t + dt)
    -> RealizationMonitor
```

## 3. CORMC 与 SUMO 职责边界

### 3.1 CORMC 提供的能力

当前 CORMC 已经具备以下 P17 所需基础能力：

```text
SimulationState:
    t, step, dt
    vehicle_states
    vehicle_specs
    active_maneuvers
    aps_assignment_cache

VehicleState:
    x_global
    y
    v
    a
    physical_lane
    road_role
    lane_change_state
    merge_state

CandidateKinematics:
    x_global
    y
    v
    a

ManeuverTrajectoryState:
    maneuver_type
    start_x_global
    start_y
    target_lane
    target_y
    planned_length
    progress
    assigned_clv_id
    assigned_cfv_id
```

这意味着 CORMC 当前已经能为 TraCI executor 提供每步的：

```text
x_global, y, v, a
```

当前缺少显式 `yaw / heading`，但 P17 第一版不应把 yaw 强行塞回算法主状态。executor 可以根据上一帧和本帧 `(x, y)` 推导 heading。

### 3.2 SUMO 提供的能力

SUMO 在 P17 中提供：

```text
1. 路网拓扑、lane、edge、route 和 GUI 可视化。
2. background vehicles 的 car-following 和 lane-changing。
3. 每步 realized state，包括 position、speed、lane、road。
4. 碰撞、teleport、插车失败、route/mapping 异常检测。
5. active controlled vehicles 对 background vehicles 的物理占位和交互影响。
```

SUMO 不负责：

```text
1. 替 CORMC active controlled vehicles 重新规划轨迹。
2. 静默覆盖 active controlled vehicles 的 CORMC 命令。
3. 决定 APS/CUC/CMC 语义。
4. 解释 CORMC 算法是否成功。
```

## 4. 控制模式

### 4.1 P17 主模式：trajectory_authority_mode

P17 默认实现：

```text
trajectory_authority_mode
```

语义：

```text
纵向:
    CORMC 输出 x_global / v / a。

横向:
    CORMC 输出 y，并通过 sine trajectory 或后续 MPC trajectory 表达横向运动。

TraCI:
    对 active controlled vehicles 使用 moveToXY 同步位置。
    配合 setSpeed / setPreviousSpeed 同步速度语义。

SUMO:
    不静默覆盖 active controlled vehicles 的轨迹。
    负责让 background vehicles 响应这些强控车辆。
```

适用目标：

```text
复现论文中正弦横向轨迹 + 下层跟踪控制的执行语义；
验证 CORMC 控制关键车辆时，能否在 SUMO 背景交通环境中完成合流。
```

代价：

```text
CORMC 必须承担 active controlled vehicles 的安全、连续性、碰撞风险和边界处理责任。
```

### 4.2 非主模式：decision_control_mode

`decision_control_mode` 暂不作为 P17 主线。

该模式可能使用：

```text
changeLane
changeSublane
slowDown
setSpeed
```

它更像把 CORMC 当成高层决策器，让 SUMO 执行底层运动。此模式与论文中的正弦横向轨迹、MPC tracking 语义不完全一致，且容易导致 CORMC command 与 SUMO realized behavior 混在一起。

P17 可以保留 executor 抽象，使未来能切换到该模式做对照实验，但第一版不以此为主线。

### 4.3 replay_mode

`replay_mode` 不是最终验证模式，但可作为 P17-B gate。

语义：

```text
CORMC 已经离线生成 trajectory；
SUMO 只验证 executor、路网映射、GUI、碰撞输出和 commanded vs realized 记录。
```

价值：

```text
先验证 moveToXY executor、edge/lane/pos 映射、heading 推导、speed 设置和 collision output；
避免一上来闭环失败时分不清是算法问题还是 SUMO 接口问题。
```

replay_mode 不得被解释为 P17 的最终算法验证。

## 5. SUMO 路网设计与 x 映射

### 5.1 路网设计目标

P17 路网首先服务稳定映射，而不是追求复杂真实几何。

核心要求：

```text
1. x_global 与 SUMO edge/lane/pos 能稳定互转。
2. CORMC 的 y = +3.5 / 0 / -3.5 能落在 SUMO 可解释的道路面上。
3. on-ramp -> lane_2 的合流横向轨迹在 merge zone 内连续。
4. background vehicles 能在主线和匝道 route 上正常行驶。
```

### 5.2 建议 edge 切分

基于当前 CORMC 道路几何：

```text
mainline_start_global = 0 m
x0_m_global           = 6950 m
x_ramp_end_global     = 7250 m
mainline_end_global   = 10000 m
lane_width            = 3.5 m
```

建议 SUMO edge：

```text
main_pre:
    x = 0 -> 6950
    lanes = 2

merge_zone:
    x = 6950 -> 7250
    lanes = 3

main_post:
    x = 7250 -> 10000
    lanes = 2

ramp_pre:
    x = 6850 -> 6950
    lanes = 1
```

`ramp_pre` 的上游长度第一版可以取 100 m，用于支持 P16 on-ramp boundary insertion。若后续需要更长匝道或 warm-up，可扩展为参数。

### 5.3 lane 映射

建议固定 lane role map：

```text
main_pre:
    laneIndex 0 -> lane_2,   y = 0
    laneIndex 1 -> lane_1,   y = +3.5

merge_zone:
    laneIndex 0 -> on_ramp,  y = -3.5
    laneIndex 1 -> lane_2,   y = 0
    laneIndex 2 -> lane_1,   y = +3.5

main_post:
    laneIndex 0 -> lane_2,   y = 0
    laneIndex 1 -> lane_1,   y = +3.5

ramp_pre:
    laneIndex 0 -> on_ramp,  y = -3.5
```

说明：

```text
SUMO laneIndex 通常从右向左编号。
上述映射与 CORMC y 方向一致：on_ramp 在最右/下侧，lane_2 居中，lane_1 在左/上侧。
```

### 5.4 edge connections

建议连接：

```text
main_pre lane_2 -> merge_zone lane_2
main_pre lane_1 -> merge_zone lane_1
ramp_pre        -> merge_zone on_ramp

merge_zone lane_2   -> main_post lane_2
merge_zone lane_1   -> main_post lane_1
merge_zone on_ramp  -> main_post lane_2
```

`merge_zone on_ramp -> main_post lane_2` 的连接用于让非强控匝道背景车也能完成 SUMO 原生合流。active MV 则由 CORMC 在 `merge_zone` 中通过 `moveToXY` 从 `y=-3.5` 推到 `y=0`。

### 5.5 x_global -> SUMO road position

主线车辆映射：

```text
if 0 <= x_global < 6950:
    edge = main_pre
    pos = x_global - 0

if 6950 <= x_global < 7250:
    edge = merge_zone
    pos = x_global - 6950

if 7250 <= x_global <= 10000:
    edge = main_post
    pos = x_global - 7250
```

匝道车辆映射：

```text
if road_role == on_ramp and x_global < 6950:
    edge = ramp_pre
    pos = x_global - ramp_pre_start_global

if road_role == on_ramp and 6950 <= x_global < 7250:
    edge = merge_zone
    pos = x_global - 6950
```

lane 选择：

```text
physical_lane -> laneIndex
```

### 5.6 SUMO road position -> x_global

反向映射：

```text
x_global = edge_start_x_global + lanePosition
```

P17 realized state 读取时，纵向真值优先来自：

```text
edgeID + lanePosition + edge_start_x_global
```

而不是直接用 GUI 坐标猜 `x_global`。GUI 坐标可作为 sanity check，用于发现路网几何或 mapping bug。

### 5.7 SUMO xy 坐标约定

P17 内部保持 CORMC 坐标：

```text
x_global: 沿行驶方向递增
y:        横向位置，lane_1 = +3.5, lane_2 = 0, on_ramp = -3.5
```

建议 SUMO net 坐标直接使用：

```text
sumo_x = x_global
sumo_y = y
```

这样 `moveToXY` 可直接使用 CORMC `(x_global, y)`，减少额外坐标变换。

## 6. active controlled vehicle 控制权规则

### 6.1 control authority 状态

P17 应引入 execution-layer control authority registry，而不是把控制权写进核心算法。

建议记录：

```text
vehicle_id
authority_state: background_controlled | cormc_controlled | post_hold | released
authority_reason
source_maneuver_type
source_command_id
entered_step / entered_t
exit_candidate_step / exit_candidate_t
```

### 6.2 进入 active control

车辆进入 CORMC active control 的规则：

```text
MV:
    位于 on-ramp/control corridor 且 merge_state != merged。
    merge_state 为 waiting 或 executing 时必须 active。

CV / CLV / CFV:
    被 APS/CUC/CMC 选中，并且本步存在以下任一条件：
        1. cooperation longitudinal command
        2. speed-cap command
        3. lane-change command
        4. active maneuver continuation

compliant CHV:
    P17 第一版可按 control-capable 处理。
    这是执行层简化，不等于声称现实 CHV 完全自动驾驶。

non-compliant CHV:
    不强控。
```

### 6.3 退出 active control

退出规则：

```text
MV:
    merge_state == merged
    且 post_merge_hold 达到阈值后退出。

CV lane-change:
    lane_change_state == normal
    active_maneuver 已清除
    且无新的 cooperation / speed-cap command 后退出。

CLV / CFV longitudinal cooperation:
    对应 MV 已 merged 或 assignment invalid
    且本步无 speed-cap / cooperation command 后退出。
```

建议 `post_merge_hold` 第一版：

```text
post_merge_hold_steps = 10
```

若 `dt = 0.1s`，即 1.0 s。该参数属于 SUMO execution layer，不改变 CORMC 算法状态机。

### 6.4 失败时不做温和退出

如果 active controlled vehicle 出现：

```text
1. collision
2. teleport
3. moveToXY realization mismatch 超阈值
4. route/mapping 不一致导致无法定位
```

则本次 run 进入 failure 记录，不通过“释放给 SUMO 控制”掩盖问题。

## 7. 中性 command 接口

### 7.1 ControlledTrajectoryCommand

建议中性命令字段：

```text
ControlledTrajectoryCommand:
    vehicle_id: str
    step: int
    t: float
    target_t: float
    x_global: float
    y: float
    v: float
    a: float
    yaw: float | None
    physical_lane: str
    road_role: str
    authority_mode: "trajectory_authority"
    authority_reason: str
    source_candidate_id: str | None
    source_command_id: str | None
    source_maneuver_type: str | None
    assigned_clv_id: str | None
    assigned_cfv_id: str | None
```

`yaw` 第一版可以为 `None`，由 executor 推导。

### 7.2 SpawnCommand

建议 spawn 命令字段：

```text
SpawnCommand:
    vehicle_id: str
    step: int
    t: float
    route_id: str
    edge_id: str
    lane_index: int
    depart_pos: float
    x_global: float
    y: float
    v: float
    vehicle_type: str
    compliance_state: str
    source_queue_seed: int
    source_profile_id: str
    source_spawn_reason: str
```

### 7.3 RealizationRecord

建议每步记录 commanded vs realized：

```text
RealizationRecord:
    vehicle_id
    step
    t
    command_x_global
    command_y
    command_v
    command_a
    realized_x_global
    realized_y
    realized_v
    realized_edge_id
    realized_lane_id
    realized_lane_position
    dx_abs
    dy_abs
    dv_abs
    result: matched | mismatch | missing | collided | teleported
```

## 8. TraCI executor 主流程

### 8.1 每步闭环时序

建议主时序：

```text
loop at SUMO time t:

1. read realized SUMO state at t
2. convert to CORMC SimulationState(t)
3. CORMC computes command for t + dt
4. determine active controlled vehicles
5. apply spawn commands if needed
6. for each active controlled vehicle:
       setPreviousSpeed(v_realized_at_t)
       setSpeed(v_cmd_t_plus_dt)
       moveToXY(edge_hint, lane_hint, x_cmd, y_cmd, angle, keepRoute=3)
7. simulationStep(t + dt)
8. read realized SUMO state
9. compare commanded vs realized
10. record collision / teleport / mismatch / trajectory / events
```

### 8.2 setPreviousSpeed / setSpeed / moveToXY 顺序

建议顺序：

```text
setPreviousSpeed
setSpeed
moveToXY
simulationStep
```

理由：

```text
1. setPreviousSpeed 用于让 SUMO 本步其他车辆感知 active 车辆上一时刻速度。
2. setSpeed 给出本步目标速度语义。
3. moveToXY 给出本步最终位置覆盖。
4. simulationStep 推进 SUMO，并让 background vehicles 对 active vehicles 的占位和运动做出响应。
```

不建议把 `moveToXY` 放在 `simulationStep` 后作为单纯纠偏，因为那样 background vehicles 在本步没有看到 CORMC 车辆的目标运动，闭环语义会错位。

### 8.3 heading 推导

若 command 未提供 yaw，executor 根据上一帧和本帧目标位置推导：

```text
dx = x_cmd - x_prev
dy = y_cmd - y_prev
heading_math = atan2(dy, dx)
sumo_angle = 90 - degrees(heading_math)
```

然后归一化到 `[0, 360)`。

若 `dx` 和 `dy` 都接近 0，则沿用上一帧 heading 或默认主线方向。

### 8.4 active vehicle SUMO mode

active controlled vehicles 建议：

```text
speedMode:
    不让 SUMO 静默压低 CORMC speed command。
    具体 bitmask 在 P17-B executor gate 中固定并记录。

laneChangeMode:
    禁止 SUMO autonomous lane change。
    不让 SUMO 替 active vehicle 决策横向行为。

moveToXY keepRoute:
    使用保 route 且允许精确 lateral position 的设置。
```

具体 bitmask 不在本文写死为常量，避免在未做 P17-B gate 前误固定。实现时必须把 bitmask 写入 manifest，并用 replay gate 验证。

## 9. P16 seeded spawn 到 TraCI insertion

### 9.1 总原则

P17 不使用 SUMO random flow 作为主要到达流。

P16 seeded queue 继续作为唯一到达流来源：

```text
same seed -> same queue -> same spawn decision sequence
```

SUMO 只负责把 CORMC 已经判定 generated 的车辆插入路网。

### 9.2 插车流程

流程：

```text
1. CORMC 在 Step 1 pre-freeze 计算 SpawnDecision。

2. generated == False:
       不调用 TraCI insertion。
       记录 cormc_spawn_blocked。
       queue item 下一步继续等待。

3. generated == True:
       转成 SpawnCommand。
       调用 traci.vehicle.add / addFull。
       必要时立即 moveToXY 到精确 x_global/y。
       确认车辆出现在 SUMO realized state。
       SpawnRegistry 标记 inserted。
```

### 9.3 route 设计

建议路线：

```text
route_main:
    main_pre -> merge_zone -> main_post

route_ramp:
    ramp_pre -> merge_zone -> main_post
```

lane_1 / lane_2 的初始 lane 由 `departLane` 指定；on-ramp 车辆使用 `route_ramp`。

### 9.4 插车失败分类

分类：

```text
CORMC blocked spawn:
    CORMC 正常行为。

CORMC generated but SUMO insertion failed:
    integration failure 或 mapping failure。

inserted active vehicle later collides with background:
    CORMC failure。
```

不要把 SUMO route/mapping 插入失败误判为 CORMC 算法合流失败。

## 10. 碰撞、teleport 与失败判定

### 10.1 active collision 规则

用户决策已固定：

```text
只要 collision 涉及 active controlled vehicle，即记录为 CORMC failure，并立即停止或标记本 run 失败。
```

### 10.2 failure 分类

建议分类：

```text
active_vs_background_collision:
    CORMC failure

active_vs_active_collision:
    CORMC failure

background_vs_background_collision:
    SUMO environment failure/warning

active_vehicle_teleported:
    CORMC failure 或 realization failure，需记录具体原因

spawn_add_failed:
    integration failure

mapping_inconsistent:
    integration failure

commanded_realized_mismatch:
    realization mismatch；超过阈值则 gate fail
```

### 10.3 记录字段

collision / failure event 至少记录：

```text
step
t
collision_type
vehicle_ids
active_vehicle_ids
background_vehicle_ids
commanded_x/y/v/a
realized_x/y/v
edge/lane/pos
source_candidate_id
source_command_id
source_maneuver_type
failure_status
```

## 11. 车辆模型不一致与处理

### 11.1 问题

CORMC 当前车辆模型与 SUMO 默认车辆模型不完全一致：

```text
CORMC:
    CAV cruising / gap-regulating / CPID
    CHV stochastic IDM
    CUC/CMC 协作逻辑
    sine lateral trajectory
    第一版未实现完整 MPC tracking

SUMO:
    默认 car-following / lane-changing 由 vType 参数决定
    常见模型包括 Krauss、IDM、LC2013、SL2015 等
```

这种差异会影响 background vehicles 对 active controlled vehicles 的响应，从而影响合流成功率、速度波动、碰撞风险和队列形态。

### 11.2 处理原则

P17 不追求 SUMO background model 与 CORMC 内部模型完全等价。

职责划分：

```text
active controlled vehicles:
    CORMC 模型权威。
    SUMO vType 只保留 length、width、minGap、显示和 route 等属性。
    运动由 moveToXY / setSpeed command 强制实现。

background vehicles:
    SUMO 模型权威。
    尽量使用 SUMO IDM 近似 CORMC CHV IDM 参数。
    lane-changing 使用 SUMO 模型，但参数必须记录。
```

### 11.3 参数记录

P17 artifact manifest 必须记录：

```text
SUMO version
SUMO_HOME
net file
route file
dt / step-length
lateral-resolution
collision.action
vType parameters
carFollowModel
laneChangeModel
speedMode bitmask
laneChangeMode bitmask
executor mode
random seed
P16 profile_id
```

这样后续解释实验结果时，可以区分：

```text
CORMC algorithm effect
SUMO background model effect
executor realization effect
mapping / integration effect
```

## 12. P17 阶段拆分建议

### 12.1 P17-A: SUMO scene closure

目标：

```text
生成 SUMO net/route/config。
跑通 Windows 原生 SUMO + TraCI。
验证 edge/lane/pos/x_global/y 映射。
验证 GUI 可视化。
验证 realized state read。
```

gate：

```text
1. netconvert 成功。
2. sumo / sumo-gui 可启动。
3. lane role map 与 CORMC road geometry 一致。
4. x_global -> edge/lane/pos -> x_global 往返误差在阈值内。
5. mainline / on-ramp routes 可用。
```

### 12.2 P17-B: trajectory executor replay gate

目标：

```text
用已有 CORMC trajectory 驱动 SUMO。
验证 moveToXY executor、setSpeed、setPreviousSpeed、heading 推导。
验证 commanded vs realized 记录。
验证 collision / teleport / mismatch 输出链。
```

说明：

```text
P17-B 是执行器和路网 gate，不是最终算法验证。
```

gate：

```text
1. active trajectory replay 能在 SUMO 中复现 CORMC x/y/v。
2. realized mismatch 低于阈值。
3. collision output 可被捕获。
4. manifest/report 可解释 executor mode 和参数。
```

### 12.3 P17-C: active controlled closed loop

目标：

```text
SUMO realized state -> CORMC -> ControlledTrajectoryCommand -> SUMO executor -> SUMO step。
```

gate：

```text
1. MV / selected CLV / CFV / CV 能进入 active control。
2. background vehicles 由 SUMO 控制。
3. active controlled vehicles 按 CORMC trajectory authority 执行。
4. P16 seeded spawn 可插入 SUMO。
5. active collision 立即记录为 CORMC failure。
6. same seed 可复现，different seed 有差异。
```

## 13. Windows 环境结论

当前不建议迁移到 WSL 作为 P17 默认路线。

原因：

```text
1. Windows 原生 SUMO 已安装并可用。
2. SUMO_HOME 已配置。
3. sumo.exe / sumo-gui.exe / netconvert.exe / duarouter.exe 可用。
4. Python 环境可通过 SUMO_HOME/tools import traci / sumolib。
5. P17 主要风险在路网、映射、executor 时序和 gate，不在操作系统。
```

WSL 可以作为后续可选环境，但 P17 第一版应优先跑通 Windows 原生 SUMO。

## 14. 待验证点

以下内容不影响方案冻结，但实现前必须通过 P17-A/B gate 固定：

```text
1. moveToXY keepRoute 的具体取值。
2. active vehicle speedMode bitmask。
3. active vehicle laneChangeMode bitmask。
4. setPreviousSpeed / setSpeed / moveToXY 顺序在本项目 SUMO 1.22.0 下的实际 realized 行为。
5. generated spawn 后 add / addFull 与立即 moveToXY 的最稳定组合。
6. merge_zone 三车道网络对 background ramp vehicle 的自然合流表现。
7. sublane / lateral-resolution 对 active moveToXY 和 background lane-changing 的影响。
8. collision.action、teleport 设置是否会隐藏 active collision。
```

这些点应由小场景自动 gate 固定，不能只靠 GUI 观察。

## 15. 当前固定结论

P17 当前固定为：

```text
主线方案:
    trajectory-authority TraCI closed loop

核心职责:
    CORMC 控 active controlled vehicles
    SUMO 控 background vehicles 和交互环境

默认 executor:
    moveToXY + setSpeed + setPreviousSpeed

接口原则:
    TraCI 不进入算法模块
    通过中性 command / executor 层解耦

路网原则:
    edge 切分贴合 x_global 边界
    merge_zone 使用三车道道路面支持连续 y 轨迹

spawn 原则:
    P16 seeded queue 是唯一到达流来源
    SUMO 只执行 CORMC generated spawn insertion

失败原则:
    active controlled vehicle collision = CORMC failure
```

后续代码执行计划应围绕 P17-A、P17-B、P17-C 拆分，不再回到 SUMO-native random flow 或 decision-only control 作为主线。

