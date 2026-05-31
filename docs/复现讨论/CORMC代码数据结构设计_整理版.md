# CORMC 代码数据结构设计（整理版）

本文档是 CORMC 第一版复现的代码数据结构规格，也是 enum、state、command、next-state、record、config、ScenarioConfig 与 expected_* 结构的字段权威。它把状态接口、日志验证、参数规格、道路几何和车辆模型中已经确认的概念，整理成后续 Python 微观仿真可以直接落地的 dataclass / enum / buffer / config / record 候选形态。

本文档不实现 Python 代码，不重新定义算法时间步，不推导论文公式，不重复参数表，不决定任何 MVS 场景中每辆车的具体初始 `x / y / v / type / compliance`。若后续实现发现缺少必要字段，应先修订本文档，再进入代码实现或执行计划。

本文档主要输入：

```text
docs/复现讨论/CORMC时间步执行顺序梳理.md
docs/复现讨论/CORMC状态与模块接口规格.md
docs/复现讨论/CORMC输出指标与日志验证规格.md
docs/复现讨论/CORMC参数规格.md
docs/复现讨论/CORMC道路几何与区域规格.md
docs/复现讨论/CORMC车辆模型规格.md
docs/复现讨论/CORMC论文公式与实现映射.md
```

## 1. 文档定位与边界

代码数据结构设计回答以下问题：

1. 第一版实现中哪些概念需要成为稳定 dataclass / enum / buffer / record。
2. 哪些字段属于冻结状态 `S(t)`，哪些属于本步派生结果。
3. command 与 next-state 如何分开承载，避免模块中途改写真实车辆状态。
4. APS assignment cache、active maneuver、CUC choice、CMC decision 如何表达生命周期。
5. 日志、轨迹、sanity check 和 PNG 输出需要哪些代码级记录形态。
6. 后续最小验证场景如何用结构化配置加载，而不是只写自然语言车辆初值。
7. `ScenarioConfig` 如何表达 module overrides、preloaded state、expected_* 与 tolerances。

本文档不负责：

1. 不写可执行 Python 代码。
2. 不选择具体 CSV / JSONL / Parquet 文件写出实现。
3. 不重写 Table I 参数值。
4. 不设定任何 `MVS-*` 场景的车辆初始数值；具体场景清单由 `CORMC最小验证场景执行规格.md` 维护。
5. 不改变 `S(t) -> command / next-state -> commit -> S(t+dt)` 的主循环。
6. 不把工程补丁写成论文原生算法。

后续实现中，本文档的类名和字段名可以作为默认命名。若代码层因工程原因需要小改字段名，应保持语义一一对应，并同步修订本文档。

## 2. 设计总原则

### 2.1 六类数据归属

第一版实现必须把数据分成六类，不能混用。

| 数据类别 | 生命周期 | 典型结构 | 说明 |
| --- | --- | --- | --- |
| `S(t)` 持久状态 | 跨时间步存在，冻结后只读 | `SimulationState`, `VehicleState`, `VehicleSpec`, `APSAssignment`, `ManeuverTrajectoryState`, `LongitudinalControllerMemory` | 本步所有模块的共同输入 |
| 本步派生状态 | 每步生成，每步消费 | `RelationsSnapshot`, `EffectiveAssignmentThisStep`, `SameStepManeuverRelationOverlay` | 不直接进入下一步，除非 commit 阶段状态化 |
| command | 本步意图和约束 | `CommandBuffer`, lane-change / merge / speed-cap command | 不直接改真实位置、速度或 lane |
| next-state | 候选下一状态 | `NextStateBuffer`, `CandidateLongitudinalKinematics`, `CandidateLateralKinematics`, `CandidateKinematics`, `CandidateManeuverProgress` | commit 前不得反写 `S(t)` |
| history / output | commit 后记录 | `TrajectoryRecord`, `EventRecord`, `SanityCheckRecord` | 用于调试、验收、PNG 和后续论文级指标 |
| config | 只读配置 | `RoadGeometryConfig`, `ParameterConfig`, `ControlPolicyConfig`, `ScenarioConfig`, `OutputConfig` | 参数值来源仍归参数规格管理；工程策略单独配置 |

硬约束：

```text
所有 APS / CUC / CMC / 纵向 / 横向模块只读冻结的 S(t)。
所有模块只写 command、next-state、cache update request 或 event candidate。
commit 是唯一生成 S(t+dt) 的阶段。
每辆车每个时间步最多提交一次最终状态。
同一车辆同一时间最多只有一个 active maneuver。
lane_change_state == executing 与 merge_state == executing 不应同时为真。
日志、sanity check 和 PNG 输出不反向改变车辆运动。
```

### 2.2 坐标与单位口径

所有算法状态结构统一使用 `x_global`，不得在 `VehicleState`、`SimulationState`、`RelationsSnapshot`、APS、CUC、CMC 或 commit 中保存或使用 `x_plot` 作为算法状态。

```text
x_global:
    算法内部纵向坐标，以主线 starting boundary 为 0。

x_plot:
    绘图派生坐标，x_plot = x_global - warmup_length。
    只在 PNG 渲染或图表导出时临时计算。
```

字段命名默认使用参数规格中的单位口径，例如 `m`、`s`、`m/s`、`m/s^2`。本文档不在字段名中重复单位后缀，单位解释由字段说明和参数规格共同约束。

### 2.3 标识符口径

第一版建议采用以下标识符口径：

| 标识符 | 建议类型 | 说明 |
| --- | --- | --- |
| `vehicle_id` | string 或 int | 全局唯一，车辆生成后不复用 |
| `command_id` | string 或 int | 本步 command 唯一标识，便于 event 追溯 |
| `event_id` | string 或 int | event history 唯一标识 |
| `scenario_id` | string | 最小验证场景或论文级实验场景标识 |
| `config_id` | string | road / parameter / output config 的版本标识 |

若第一版代码为了简单使用递增 int，也必须保证在一次仿真运行中不复用 active 或已退出车辆的 `vehicle_id`。

### 2.4 空值与缺失口径

本文档中写作 `optional` 的字段，在 Python 实现中可以用 `None` 或显式 sentinel enum 表达。推荐规则：

```text
不存在某个对象:
    用 None。

状态存在但当前不适用:
    用 not_applicable / empty 等 enum。

计算失败或被工程兜底拦截:
    用 failed / invalid，并提供 reason。
```

例如，`CFV` 不存在不能和 `APSAssignmentStatus.failed` 混为一谈；前者是对象缺失，后者是一次 APS assignment 尝试的结果状态。

### 2.5 工程补丁来源标记

以下内容属于第一版工程补丁或实现约束，相关结构必须能携带 `source`、`reason` 或 `is_engineering_patch` 概念：

```text
first_APS(MV)
assignment invalid
immediate_APS_refresh
多 MV 共享 CV 仲裁
same-step maneuver relation overlay
每车每步只提交一次
boundary speed cap 不可行时的保守处理入口
unexpected ordinary lane-change attempt
```

这些字段只用于追踪工程兜底，不得在文档或代码中描述为论文已经给出的原生机制。

## 3. Enum 与 State Machine

### 3.1 必选枚举

| Enum 名称 | 取值 | 使用位置 | 说明 |
| --- | --- | --- | --- |
| `VehicleType` | `cav`, `chv` | `VehicleSpec` | CAV / CHV 车辆类型 |
| `ComplianceState` | `not_applicable`, `compliant`, `non_compliant` | `VehicleSpec`, `TrajectoryRecord` | CAV 使用 `not_applicable`；CHV 使用 compliance 状态 |
| `LaneId` | `lane_1`, `lane_2`, `on_ramp` | `VehicleState`, relations, commands | `lane_2` 是 MV 合流目标，`lane_1` 是 CUC 换道目标 |
| `RoadRole` | `mainline`, `on_ramp` | `VehicleState`, `TrajectoryRecord` | 表达车辆当前主线 / 匝道身份 |
| `LaneChangeState` | `normal`, `executing` | `VehicleState` | 不长期保留 `completed` |
| `MergeState` | `none`, `not_started`, `waiting`, `executing`, `merged` | `VehicleState` | `normal` 不属于 merge state |
| `APSCase` | `case_1`, `case_2`, `case_3`, `case_4` | `APSAssignment` | 对应论文 APS 四种 case |
| `APSAssignmentStatus` | `valid`, `invalid`, `failed`, `empty` | `APSAssignment` | cache 状态与失败状态 |
| `CUCChoice` | `change_to_lane_1`, `stay_lane_2`, `not_applicable` | `CUCDecision`, `EventRecord` | CUC choice 是本步 command / event |
| `ManeuverType` | `lane_change`, `merge` | `ManeuverTrajectoryState`, commands | CUC 换道或 CMC 合流 |
| `CommandType` | `longitudinal`, `cooperation`, `lane_change`, `merge`, `speed_cap`, `state_transition`, `cache_update` | `CommandBuffer` | command 分组键 |
| `EventType` | 见 3.2 | `EventRecord` | 事件类型 |
| `SanityCheckType` | 见 3.3 | `SanityCheckRecord` | sanity check 类型 |

### 3.2 `EventType`

第一版 `EventType` 至少覆盖：

| 取值 | 用途 |
| --- | --- |
| `boundary_generation` | 入口队列检查、新车生成、入口安全失败 |
| `relation_refresh` | lane ordering、leader/follower、TLV/TFV/LV/FV |
| `aps` | APS 触发、候选集合、case、assignment |
| `assignment_invalid` | CMC 前 assignment 有效性检查失败 |
| `cooperative_request` | Step 5 从 assignment 生成 CV 协同请求 |
| `conflict_resolution` | 多 MV 共享 CV 仲裁 |
| `cuc` | CUC utility、安全回退、choice |
| `cmc` | Eq.53 gap、boundary speed cap、merge transition |
| `longitudinal_model` | 纵向模式、candidate acceleration / speed、speed cap 消费 |
| `lateral_trajectory` | 正弦轨迹、front-collision fallback、progress |
| `commit` | 单车提交、lane 更新、state transition、cache cleanup |
| `vehicle_exit` | 车辆驶出仿真路段 |
| `sanity_check` | 碰撞、越界、状态机异常等检查 |
| `engineering_patch` | first_APS、overlay、工程仲裁等补丁说明 |

### 3.3 `SanityCheckType`

第一版 `SanityCheckType` 至少覆盖：

```text
collision
near_collision
boundary_violation
assignment_invalid
state_machine_inconsistency
geometry_inconsistency
unexpected_ordinary_lane_change_attempt
multiple_commit_for_one_vehicle
```

其中 `assignment_invalid` 既可以作为 `EventType.assignment_invalid` 记录决策链，也可以作为 sanity check 的汇总结果。二者用途不同：event 记录发生过程，sanity check 记录验收状态。

### 3.4 辅助 reason 枚举

为了避免后续实现临时拼字符串，第一版建议预留以下 reason code 组。本文档只定义语义集合，具体是否实现为 enum 由代码阶段决定。

| reason 组 | 建议取值 |
| --- | --- |
| `APSTriggerReason` | `first_aps`, `aps_due`, `reuse_cache` |
| `APSFailureReason` | `insufficient_candidates`, `no_insert_pair`, `all_predicted_positive`, `all_predicted_negative`, `mv_speed_near_zero`, `mv_already_in_merging_zone`, `unknown` |
| `AssignmentInvalidReason` | `clv_missing`, `cfv_missing`, `clv_not_lane_2`, `cfv_not_lane_2`, `vehicle_exited`, `wrong_order`, `unsafe_gap_boundary`, `stale_assignment`, `unknown` |
| `CUCFallbackReason` | `utility_not_better`, `target_lane_unsafe`, `non_compliant_chv`, `already_executing_lane_change`, `not_active_cv`, `not_applicable` |
| `BoundaryCapReason` | `normal_cap`, `cap_infeasible`, `cap_negative`, `cap_too_low`, `not_applicable` |
| `CommitRejectReason` | `missing_candidate`, `duplicate_candidate`, `conflicting_transition`, `invalid_state_machine`, `not_applicable` |

这些 reason code 不得替代论文公式，只用于日志、调试和工程兜底追踪。

## 4. 核心状态结构

### 4.1 `VehicleSpec`

`VehicleSpec` 表示车辆生成后跨步不重采样的属性。它不是本步运动状态。

| 字段 | 类型 / 取值 | 归属 | 说明 |
| --- | --- | --- | --- |
| `vehicle_id` | id | 持久 | 全局唯一车辆标识 |
| `vehicle_type` | `VehicleType` | 持久 | CAV 或 CHV |
| `compliance_state` | `ComplianceState` | 持久 | CAV 为 `not_applicable` |
| `desired_speed` | float | 持久 | CHV desired speed 或车辆生成得到的期望速度属性 |
| `desired_time_gap` | optional float | 持久 | 车辆级期望时距；可由车辆类型默认值或场景覆盖得到 |
| `desired_time_gap_class` | optional text | 持久 | 车辆生成阶段分配的 time gap class；第一版可为空 |
| `assigned_arrival_headway` | optional float | 持久 | 车辆生成阶段分配的 arrival headway，用于生成复盘和论文级验证 |
| `inertial_lag` | optional float | 持久 | CAV 惯性滞后；CHV 可为空 |
| `length` | float | 持久 | 来自参数规格的车辆长度引用值 |
| `source_lane_at_generation` | `LaneId` | 持久 | 车辆生成时的入口 lane |
| `generation_step` | int | 持久 | 车辆进入 active set 的 step |
| `generation_t` | float | 持久 | 车辆进入 active set 的时间 |

约束：

```text
VehicleSpec 不保存 x / y / v / a。
VehicleSpec 不保存 CUC choice、APS case、merge_state。
desired_speed、desired_time_gap、assigned_arrival_headway、inertial_lag 等随机属性生成后不在每步重采样。
assigned_arrival_headway 不要求运动模型每步消费，但必须能用于边界车辆生成复盘。
```

### 4.2 `VehicleState`

`VehicleState` 表示 `S(t)` 中每辆 active vehicle 的真实状态。它是冻结快照的一部分。

| 字段 | 类型 / 取值 | 归属 | 说明 |
| --- | --- | --- | --- |
| `vehicle_id` | id | `S(t)` | 关联 `VehicleSpec` |
| `x_global` | float | `S(t)` | 算法内部唯一纵向坐标 |
| `y` | float | `S(t)` | 横向物理位置 |
| `v` | float | `S(t)` | 当前速度 |
| `a` | float | `S(t)` | 当前加速度 |
| `physical_lane` | `LaneId` | `S(t)` | commit 后正式 lane 归属 |
| `road_role` | `RoadRole` | `S(t)` | `mainline` 或 `on_ramp` |
| `lane_change_state` | `LaneChangeState` | `S(t)` | 主线 CV active lane change 状态 |
| `merge_state` | `MergeState` | `S(t)` | MV CMC 状态 |
| `is_active` | bool | `S(t)` | 是否仍在仿真 active set |

禁止字段：

```text
x_plot
CUC choice
本步 candidate speed
本步 command id
本步 APS effective assignment
日志或 sanity check 结果
```

说明：

```text
physical_lane 只在 commit 阶段正式更新。
正在换道或合流时，y 可以位于两条 centerline 之间，但 physical_lane 仍按状态机语义保持原 lane，直到 commit 完成。
merge_state == merged 可在下一时间步压缩为 none，但不能与 lane_change_state.normal 混用。
```

### 4.3 `ManeuverTrajectoryState`

`ManeuverTrajectoryState` 表示 active lane-change / merge 跨步继续执行所需状态。它属于跨步持久状态。

| 字段 | 类型 / 取值 | 归属 | 说明 |
| --- | --- | --- | --- |
| `vehicle_id` | id | 持久 | 执行 maneuver 的车辆 |
| `maneuver_type` | `ManeuverType` | 持久 | `lane_change` 或 `merge` |
| `source_command_id` | optional id | 持久 | 初始化该 maneuver 的 command |
| `source_event_id` | optional id | 持久 | 初始化该 maneuver 的 event |
| `start_step` | int | 持久 | maneuver 初始化 step |
| `start_t` | float | 持久 | maneuver 初始化时间 |
| `start_x_global` | float | 持久 | 起点纵向坐标 |
| `start_y` | float | 持久 | 起点横向坐标 |
| `target_lane` | `LaneId` | 持久 | 目标 lane |
| `target_y` | float | 持久 | 目标 centerline |
| `planned_length` | optional float | 持久 | 正弦轨迹长度语义；公式由车辆模型规格负责 |
| `progress` | float | 持久 | 轨迹进度概念，范围建议为 `0..1` |
| `last_planning_speed` | optional float | 历史必要状态 | 上一步使用的 planning speed，供回退或日志使用 |

约束：

```text
执行过程中不因每步 relations 变化重置 start_x_global / start_y / target_y。
如果 lane_change_state == executing 或 merge_state == executing，下一步继续 maneuver 的依据是该结构，而不是历史 CUC choice。
同一车辆同一时间最多存在一个 ManeuverTrajectoryState。
lane_change_state == executing 与 merge_state == executing 不应同时为真；若出现，commit 或 sanity check 必须记录 state_machine_inconsistency。
```

### 4.4 `LongitudinalControllerMemory`

`LongitudinalControllerMemory` 表示纵向控制器跨步记忆。它主要服务 CAV gap-regulating / CPID 语义，也可以为后续其他需要历史误差的纵向控制器预留入口。

它属于跨步持久状态，不属于日志，不得每步重置。若第一版某辆车不使用 CPID，可保留空 memory 或不为该车创建 memory，但结构必须存在。

| 字段 | 类型 / 取值 | 归属 | 说明 |
| --- | --- | --- | --- |
| `vehicle_id` | id | 持久 | 对应车辆 |
| `spacing_error_integral` | float | 持久 | 外环 spacing error 积分项 |
| `last_spacing_error` | optional float | 持久 | 上一控制步 spacing error |
| `inner_error_integral` | float | 持久 | 内环 error 积分项 |
| `last_inner_error` | optional float | 持久 | 上一控制步 inner loop error |
| `last_controller_update_step` | optional int | 持久 | 最近一次控制器更新 step |
| `last_controller_update_t` | optional float | 持久 | 最近一次控制器更新时间 |
| `controller_mode` | optional text | 持久 | `cav_cpid`, `idm`, `none` 等实现标记 |

约束：

```text
LongitudinalControllerMemory 不进入 TrajectoryRecord。
LongitudinalControllerMemory 不保存车辆真实 x / y / v / a。
CPID 积分项和上一误差不得用 EventRecord 或临时变量替代，否则 gap-regulating 会退化为伪 PID。
若控制器状态被重置，必须记录 EventRecord，说明 reset reason。
```

### 4.5 `SimulationState`

`SimulationState` 是冻结状态 `S(t)` 的代码承载。

| 字段 | 类型 / 取值 | 归属 | 说明 |
| --- | --- | --- | --- |
| `t` | float | `S(t)` | 当前仿真时间 |
| `step` | int | `S(t)` | 当前时间步 |
| `dt` | float | `S(t)` / config ref | 时间步长引用 |
| `active_vehicle_ids` | list[id] | `S(t)` | 本步 active vehicles |
| `vehicle_states` | map[id, `VehicleState`] | `S(t)` | 每辆 active vehicle 的真实状态 |
| `vehicle_specs` | map[id, `VehicleSpec`] | 持久 | active 和必要历史车辆属性 |
| `controller_memory_by_vehicle` | map[id, `LongitudinalControllerMemory`] | 持久 | 纵向控制器跨步记忆 |
| `aps_assignment_cache` | map[id, `APSAssignment`] | 持久 cache | key 为 MV id |
| `active_maneuvers` | map[id, `ManeuverTrajectoryState`] | 持久 | key 为执行 maneuver 的 vehicle id |
| `road_config_ref` | id 或引用 | config | 只读 road config |
| `parameter_config_ref` | id 或引用 | config | 只读 parameter config |
| `scenario_config_ref` | optional id 或引用 | config | 当前 scenario |
| `output_config_ref` | optional id 或引用 | config | 输出配置 |

约束：

```text
SimulationState 不包含 CommandBuffer。
SimulationState 不包含 NextStateBuffer。
SimulationState 不包含本步 RelationsSnapshot。
SimulationState 不包含 trajectory/event/sanity records。
SimulationState 可以包含 LongitudinalControllerMemory，因为它是跨步控制器状态，不是日志。
```

边界车辆生成发生在冻结 `SimulationState` 之前。冻结后，不允许再向 `active_vehicle_ids` 插入新车影响本步 APS / CUC / CMC / 纵向 / 横向计算。

## 5. Relations / APS / CUC / CMC 中间结构

### 5.1 `RelationsSnapshot`

`RelationsSnapshot` 是每步基于冻结 `S(t)` 派生的只读关系快照。

| 字段 | 类型 / 取值 | 生命周期 | 说明 |
| --- | --- | --- | --- |
| `step` | int | 本步派生 | 生成关系的 step |
| `t` | float | 本步派生 | 生成关系的时间 |
| `lane_ordering` | map[`LaneId`, list[id]] | 本步派生 | 每条 lane 按 `x_global` 排序的车辆 |
| `leader_by_vehicle` | map[id, optional id] | 本步派生 | 普通纵向主 leader |
| `follower_by_vehicle` | map[id, optional id] | 本步派生 | 普通纵向 follower |
| `lane_change_neighborhood` | map[id, `LaneChangeNeighborhood`] | 本步派生 | CUC / active lane-change 所需 TLV / TFV / LV / FV |
| `active_maneuver_relation` | map[id, `ActiveManeuverRelation`] | 本步派生 | 正在换道或合流车辆的 logical relation |

约束：

```text
lane_ordering 使用 x_global，不使用 x_plot。
relations 不按 physical y 最近 centerline 连续切换 leader/follower。
relations snapshot 每步生成、每步消费、下一步重建。
```

### 5.2 `LaneChangeNeighborhood`

`LaneChangeNeighborhood` 表达 CUC 和 lane-changing subsystem 需要的目标车道 / 原车道邻接关系。

| 字段 | 类型 / 取值 | 说明 |
| --- | --- | --- |
| `vehicle_id` | id | 被评估或正在换道的 CV |
| `source_lane` | `LaneId` | 第一版 CUC 为 `lane_2` |
| `target_lane` | `LaneId` | 第一版 CUC 为 `lane_1` |
| `tlv_id` | optional id | target lane leader |
| `tfv_id` | optional id | target lane follower |
| `lv_id` | optional id | source lane leader |
| `fv_id` | optional id | source lane follower |
| `snapshot_source` | text | 来自 Step 3 relations snapshot |

### 5.3 `ActiveManeuverRelation`

`ActiveManeuverRelation` 表达 active maneuver 中车辆的 logical longitudinal role。

| 字段 | 类型 / 取值 | 说明 |
| --- | --- | --- |
| `vehicle_id` | id | active maneuver 车辆 |
| `primary_leader_id` | optional id | 车辆纵向模型主要 leader |
| `affected_target_follower_id` | optional id | 目标车道 TFV |
| `affected_source_follower_id` | optional id | 原车道 FV |
| `relation_source` | text | `active_lane_change`, `active_merge`, `normal_following` 等 |

### 5.4 `SameStepManeuverRelationOverlay`

`SameStepManeuverRelationOverlay` 是第一版工程实现约束。它只用于 Step 6 本步新启动 CUC 换道后，让 Step 7 纵向模型消费正确的 TLV / TFV / FV 关系。

| 字段 | 类型 / 取值 | 生命周期 | 说明 |
| --- | --- | --- | --- |
| `vehicle_id` | id | 本步派生 | 本步新启动 lane-change 的 CV |
| `source_command_id` | id | 本步派生 | lane-change command |
| `basis_snapshot_step` | int | 本步派生 | 引用 Step 3 relations snapshot |
| `tlv_id` | optional id | 本步派生 | 目标车道 leader |
| `tfv_id` | optional id | 本步派生 | 目标车道 follower |
| `fv_id` | optional id | 本步派生 | 原车道 follower |
| `target_lane` | `LaneId` | 本步派生 | 第一版为 `lane_1` |
| `is_engineering_patch` | bool | 本步派生 | 必须为 true 或等价标记 |

约束：

```text
overlay 不改变 physical_lane。
overlay 不进入跨步持久状态。
commit 后若换道仍在执行，下一步 relations refresh 根据 state machine 和 ManeuverTrajectoryState 重新派生关系。
```

### 5.5 `APSAssignment`

`APSAssignment` 是每个 MV 的跨步 cache。key 建议为 MV `vehicle_id`。

| 字段 | 类型 / 取值 | 生命周期 | 说明 |
| --- | --- | --- | --- |
| `mv_id` | id | 持久 cache | 对应 MV |
| `status` | `APSAssignmentStatus` | 持久 cache | `valid / invalid / failed / empty` |
| `aps_case` | optional `APSCase` | 持久 cache | APS case |
| `clv_id` | optional id | 持久 cache | assigned CLV |
| `cfv_id` | optional id | 持久 cache | assigned CFV |
| `col_clv` | optional bool | 持久 cache | CLV 是否需要协同 |
| `col_cfv` | optional bool | 持久 cache | CFV 是否需要协同 |
| `eq10_desired_spacing` | optional float 或概念对象 | 持久 cache | case 2 / 4 中 CFV desired spacing 语义 |
| `t_mv_star` | optional float | 调试 / cache | APS 到达预测时间 |
| `last_update_step` | optional int | 持久 cache | 最近一次 APS 更新 step |
| `last_update_t` | optional float | 持久 cache | 最近一次 APS 更新时间 |
| `trigger_reason` | optional `APSTriggerReason` | 持久 cache / event | `first_aps / aps_due / reuse_cache` |
| `failure_reason` | optional `APSFailureReason` | 持久 cache / event | APS 失败原因 |
| `invalid_reason` | optional `AssignmentInvalidReason` | 持久 cache / event | CMC 前验证失败原因 |
| `source` | text | 持久 cache | `paper_algorithm`, `engineering_patch` 等来源标记 |

生命周期：

```text
MV 首次进入 APS 适用阶段:
    必须生成 valid assignment、failed assignment 或 empty assignment。

APS_due:
    更新该 MV 的 APSAssignment cache。

非 APS 周期:
    沿用当前 APSAssignment cache。

CMC 前验证失败:
    status 变为 invalid，并记录 invalid_reason。

MV merged:
    清理该 MV 的 APSAssignment cache。
```

### 5.6 `EffectiveAssignmentThisStep`

`EffectiveAssignmentThisStep` 是 Step 5 消费的本步派生 assignment。

| 字段 | 类型 / 取值 | 生命周期 | 说明 |
| --- | --- | --- | --- |
| `mv_id` | id | 本步派生 | 来源 MV |
| `assignment` | `APSAssignment` 或引用 | 本步派生 | 本步有效 assignment 内容 |
| `source` | enum/text | 本步派生 | `aps_updated_this_step` 或 `cache_reused` |
| `is_valid_for_request` | bool | 本步派生 | 是否可进入 cooperative request 汇总 |

该结构不跨步持久化。跨步持久化仍由 `APSAssignment` cache 负责。

### 5.7 `CooperativeRequest`

`CooperativeRequest` 从有效 assignment 中抽取，供 Step 5 仲裁和 Step 6 CUC 使用。

| 字段 | 类型 / 取值 | 生命周期 | 说明 |
| --- | --- | --- | --- |
| `request_id` | id | 本步派生 | 协同请求标识 |
| `source_mv_id` | id | 本步派生 | 请求来自哪个 MV |
| `cv_id` | id | 本步派生 | 被请求协同的 CV |
| `cv_role` | text | 本步派生 | `clv` 或 `cfv` |
| `col` | bool | 本步派生 | 是否需要协同 |
| `aps_case` | `APSCase` | 本步派生 | 来源 APS case |
| `t_mv_star` | optional float | 本步派生 | 仲裁优先级可能使用 |
| `mv_distance_to_x0_m` | optional float | 本步派生 | 仲裁优先级可能使用 |
| `mv_in_merging_zone` | bool | 本步派生 | 仲裁优先级使用 |

### 5.8 `ConflictResolutionResult`

`ConflictResolutionResult` 表达多 MV 共享 CV 仲裁结果。该结构必须标注为工程补丁。

| 字段 | 类型 / 取值 | 生命周期 | 说明 |
| --- | --- | --- | --- |
| `conflict_id` | id | 本步派生 / event | 冲突组标识 |
| `cv_id` | id | 本步派生 / event | 被多个 MV 请求的 CV |
| `request_ids` | list[id] | 本步派生 / event | 参与冲突的请求 |
| `winner_request_id` | optional id | 本步派生 / event | 获胜请求 |
| `loser_request_ids` | list[id] | 本步派生 / event | 未获胜请求 |
| `priority_basis` | text 或结构 | 本步派生 / event | merging zone > smaller `T*_MV` > closer to `x0^m` |
| `is_engineering_patch` | bool | 本步派生 / event | 必须为 true 或等价标记 |

### 5.9 `CUCDecision`

`CUCDecision` 是本步 CUC 模块输出，不是下一时间步真实控制状态。

| 字段 | 类型 / 取值 | 生命周期 | 说明 |
| --- | --- | --- | --- |
| `vehicle_id` | id | 本步派生 / command | 被决策的 CV |
| `source_request_id` | id | 本步派生 / command | 来源 cooperative request |
| `recommended_choice` | `CUCChoice` | 本步派生 / event | CUC utility 与安全检查得到的模型建议 |
| `effective_choice` | `CUCChoice` | 本步派生 / command | compliance / fallback 后真正用于生成 command 的选择 |
| `utility_choice_1` | optional float | 调试 | choice 1 utility |
| `utility_choice_2` | optional float | 调试 | choice 2 utility |
| `target_lane_safety_pass` | optional bool | 调试 / command | TT 安全约束结果 |
| `fallback_reason` | optional `CUCFallbackReason` | event | 回退到 stay lane 2 的原因 |
| `accepted_by_vehicle` | bool | 本步派生 | CAV / compliant CHV 为 true，non-compliant CHV 为 false |
| `source` | text | 本步派生 | `paper_algorithm` 或工程约束说明 |

约束：

```text
recommended_choice 用于记录论文 CUC 模型建议。
effective_choice 用于决定本步是否生成 LaneChangeCommand 或留 lane 2 cooperation command。
如果 recommended_choice == change_to_lane_1 但车辆是 non-compliant CHV，则 accepted_by_vehicle = false，effective_choice = stay_lane_2。
如果 recommended_choice == change_to_lane_1 但目标车道安全约束失败，则 effective_choice = stay_lane_2，并记录 fallback_reason。
只有 effective_choice == change_to_lane_1 时，才能由 lane-change command 初始化 maneuver。
下一步继续换道的依据是 lane_change_state 与 ManeuverTrajectoryState，不是历史 CUCDecision。
```

### 5.10 `CMCDecision`

`CMCDecision` 是本步 CMC 模块输出。

| 字段 | 类型 / 取值 | 生命周期 | 说明 |
| --- | --- | --- | --- |
| `mv_id` | id | 本步派生 / command | 被处理的 MV |
| `assignment_status` | `APSAssignmentStatus` | 本步派生 | CMC 前 assignment 验证结果 |
| `assignment_invalid_reason` | optional `AssignmentInvalidReason` | event | assignment invalid 原因 |
| `dynamic_acceptable_gap` | optional float | 调试 | Eq.52 结果概念 |
| `eq53_gap_pass` | optional bool | 本步派生 | Eq.53 是否满足 |
| `boundary_speed_cap` | optional float | speed-cap command | Eq.56 速度上限概念 |
| `boundary_cap_reason` | optional `BoundaryCapReason` | event | speed cap 状态 |
| `merge_transition_request` | optional `MergeState` | command | 请求进入 waiting / executing / merged 等 |
| `merge_command_id` | optional id | command | 若开始或继续合流，对应 command |

约束：

```text
CMCDecision 不直接改 MV 的 x / y / lane。
assignment invalid 不等价于每步重查 actual leader/follower 并替代 APS assignment。
merge_state == executing 后，不因短时 gap 变化撤销本次合流。
```

## 6. CommandBuffer 与 NextStateBuffer

### 6.1 `CommandBuffer`

`CommandBuffer` 只承载本步模块意图和约束。

| 字段 | 类型 / 取值 | 说明 |
| --- | --- | --- |
| `step` | int | command 所属 step |
| `t` | float | command 所属时间 |
| `longitudinal_commands` | map[id, `LongitudinalCommand`] | 纵向意图 |
| `cooperation_commands` | map[id, `CooperationCommand`] | 协同意图 |
| `lane_change_commands` | map[id, `LaneChangeCommand`] | CUC 换道命令 |
| `merge_commands` | map[id, `MergeCommand`] | CMC 合流命令 |
| `speed_cap_commands` | map[id, list[`SpeedCapCommand`]] | 速度上限或回退速度约束 |
| `state_transition_commands` | map[id, list[`StateTransitionCommand`]] | 状态机转移请求 |
| `cache_update_commands` | list[`CacheUpdateCommand`] | APS cache 更新或清理请求 |
| `same_step_overlays` | map[id, `SameStepManeuverRelationOverlay`] | 本步新启动换道关系覆盖 |

通用 command 字段建议：

```text
command_id
command_type
module
vehicle_id
source_step
source_t
source_object_id
reason
is_engineering_patch
```

### 6.2 Command 子结构

| 结构 | 必要字段 | 说明 |
| --- | --- | --- |
| `LongitudinalCommand` | `vehicle_id`, `mode`, `leader_id`, `desired_spacing_override`, `speed_cap_refs` | 指示纵向模型使用的模式、leader 和约束 |
| `CooperationCommand` | `vehicle_id`, `source_mv_id`, `cv_role`, `aps_case`, `eq10_desired_spacing` | 表达 CUC / APS 协同语义 |
| `LaneChangeCommand` | `vehicle_id`, `target_lane`, `target_y`, `cuc_decision_id`, `overlay_id`, `init_maneuver` | 初始化或继续 CUC lane 2 -> lane 1 换道 |
| `MergeCommand` | `mv_id`, `target_lane`, `target_y`, `cmc_decision_id`, `init_or_continue_maneuver` | 初始化或继续 on-ramp -> lane 2 合流 |
| `SpeedCapCommand` | `vehicle_id`, `cap_value`, `cap_source`, `cap_reason`, `applies_to` | boundary speed cap 或 front-collision fallback |
| `StateTransitionCommand` | `vehicle_id`, `state_name`, `old_state`, `requested_new_state`, `reason` | lane_change_state / merge_state 转移请求 |
| `CacheUpdateCommand` | `cache_name`, `owner_vehicle_id`, `operation`, `assignment`, `reason` | APS cache update / invalidate / cleanup |

约束：

```text
command 不直接修改 VehicleState。
command 可以被纵向、横向、commit 模块消费。
同一车辆多个 speed cap 同时存在时，纵向模型使用更保守 planning speed。
```

### 6.3 `NextStateBuffer`

`NextStateBuffer` 只承载基于 `S(t)` 和 command 计算出的候选下一状态。

| 字段 | 类型 / 取值 | 说明 |
| --- | --- | --- |
| `step` | int | 所属 step |
| `t` | float | 所属时间 |
| `candidate_longitudinal` | map[id, `CandidateLongitudinalKinematics`] | 纵向候选 `x_global / v / a` 和 planning speed |
| `candidate_lateral` | map[id, `CandidateLateralKinematics`] | 横向候选 `y` 和横向约束结果 |
| `candidate_kinematics` | map[id, `CandidateKinematics`] | commit preparation 合成后的完整候选 `x_global / y / v / a` |
| `candidate_maneuver_progress` | map[id, `CandidateManeuverProgress`] | active maneuver 进度 |
| `candidate_lane_state` | map[id, `CandidateLaneState`] | lane / road role 候选更新 |
| `candidate_state_transitions` | map[id, list[`CandidateStateTransition`]] | 状态机候选转移 |
| `candidate_cache_updates` | list[`CandidateCacheUpdate`] | cache 候选更新 |
| `commit_warnings` | list[`CommitWarning`] | commit 前发现的冲突或缺失 |

### 6.4 Next-state 子结构

| 结构 | 必要字段 | 说明 |
| --- | --- | --- |
| `CandidateLongitudinalKinematics` | `vehicle_id`, `x_global`, `v`, `a`, `candidate_speed`, `planning_speed`, `constraints_applied`, `source_commands` | 纵向模型唯一写入；不写 `y` |
| `CandidateLateralKinematics` | `vehicle_id`, `y`, `target_y`, `front_collision_fallback`, `source_commands` | 横向轨迹模块唯一写入；不写 `x_global / v / a` |
| `CandidateKinematics` | `vehicle_id`, `x_global`, `y`, `v`, `a`, `source_longitudinal_candidate`, `source_lateral_candidate`, `constraints_applied` | 仅由 candidate assembly / commit preparation 合成 |
| `CandidateManeuverProgress` | `vehicle_id`, `maneuver_type`, `progress`, `completed`, `target_y_reached`, `source_command_id` | 换道 / 合流进度 |
| `CandidateLaneState` | `vehicle_id`, `physical_lane`, `road_role`, `reason` | lane 和主线 / 匝道身份候选 |
| `CandidateStateTransition` | `vehicle_id`, `state_name`, `old_state`, `new_state`, `reason` | 状态机候选转移 |
| `CandidateCacheUpdate` | `cache_name`, `owner_vehicle_id`, `operation`, `new_value`, `reason` | APS cache 候选更新 |
| `CommitWarning` | `vehicle_id`, `warning_type`, `reason` | 多候选冲突、缺失候选等 |

约束：

```text
next-state 不反写 S(t)。
next-state 不作为本步其他车辆的真实输入。
纵向模型只能写 CandidateLongitudinalKinematics。
横向轨迹模块只能写 CandidateLateralKinematics 和 CandidateManeuverProgress。
CandidateKinematics 只能由 candidate assembly / commit preparation 合成。
commit 阶段必须对每辆 active vehicle 合成唯一 CandidateKinematics。
```

### 6.5 Commit 输入与输出

commit 阶段读取：

```text
S(t)
CommandBuffer
NextStateBuffer
CandidateCacheUpdate
CandidateStateTransition
```

commit 阶段输出：

```text
S(t+dt)
CommitEvent
TrajectoryRecord candidates
SanityCheckRecord candidates
```

commit 必须保证：

1. 每辆车本步最多提交一次。
2. CV 完成换道后正式更新 `physical_lane = lane_1`，`lane_change_state = normal`。
3. MV 到达 lane 2 centerline 后正式更新 `physical_lane = lane_2`，`road_role = mainline`，`merge_state = merged`，并清理 APS assignment cache。
4. 驶出车辆记录 exit event，再从下一步 active set 移除。
5. `CUCDecision` 不进入下一步控制状态，只能作为 event/history。

## 7. Config 与 ScenarioConfig

### 7.1 `RoadGeometryConfig`

`RoadGeometryConfig` 承载道路几何字段名。字段值来源仍由道路几何与参数规格决定。

| 字段 | 说明 |
| --- | --- |
| `config_id` | 几何配置版本 |
| `mainline_start_global` | 主线起点 |
| `mainline_end_global` | 主线终点 |
| `warmup_length` | warm-up section 长度 |
| `x0_m_global` | merging zone 起点 |
| `l_merging` | merging zone 长度 |
| `x_ramp_end_global` | on-ramp downstream boundary |
| `l_coop_fixed` | fixed cooperative zone 长度 |
| `communication_range` | APS candidate window 使用的 `L_cr` |
| `lane_centerlines` | map[`LaneId`, y] |
| `observation_region` | 论文级指标观测区概念 |

约束：

```text
APS candidate window 使用 communication_range。
fixed cooperative zone 不得替代 APS candidate window。
x_plot 不作为 config 字段进入算法状态，可在 output renderer 中由 warmup_length 派生。
```

### 7.2 `ParameterConfig`

`ParameterConfig` 组织参数字段，但不复制参数表数值。

建议按模块分组：

```text
global_time
vehicle_basic
cav_longitudinal
cpid_defaults
idm
aps
cuc
cmc
lane_changing
vehicle_generation
experiment_grid
```

每个参数项建议包含：

| 字段 | 说明 |
| --- | --- |
| `name` | 实现建议名 |
| `value` | 参数值或分布对象 |
| `unit` | 单位 |
| `source_status` | `paper`, `paper-derived`, `first-version-default`, `to-review` |
| `source_doc` | 来源文档或论文位置 |
| `notes` | 待审阅说明 |

约束：

```text
ParameterConfig 的字段名应引用 CORMC参数规格.md。
如果参数规格更新，ParameterConfig 的字段组同步更新。
assignment invalid 后如何处理、越界保守策略等工程策略不写入 ParameterConfig。
```

### 7.3 `ControlPolicyConfig`

`ControlPolicyConfig` 承载第一版实现开关和工程策略。它不是论文参数表，不属于 `ParameterConfig`。它的职责是让第一版关闭项和工程补丁可审计、可复现、可在后续消融实验中显式调整。

| 字段 | 说明 |
| --- | --- |
| `config_id` | policy config 版本 |
| `assignment_invalid_policy` | assignment invalid 后等待、保守减速或 immediate APS refresh 的策略名 |
| `immediate_aps_refresh_enabled` | 是否允许 assignment invalid 后触发 APS 语义刷新 |
| `ordinary_mainline_lane_change_enabled` | 第一版默认 false |
| `mpc_tracking_enabled` | 第一版默认 false |
| `cmc_platoon_enabled` | 第一版默认 false |
| `boundary_cap_infeasible_policy_ref` | boundary speed cap 不可行时的保守策略引用 |
| `front_collision_avoidance_scope` | front-collision-avoidance 适用范围，例如 CUC lane-change、MV merge |
| `single_active_maneuver_enforced` | 是否强制同一车辆最多一个 active maneuver；第一版应为 true |
| `engineering_patch_logging_enabled` | 是否记录工程补丁事件；第一版应为 true |

约束：

```text
ControlPolicyConfig 可以保存工程策略和第一版关闭项。
ControlPolicyConfig 不保存 Table I 参数。
ControlPolicyConfig 不得把工程补丁描述成论文原生算法。
ParameterConfig 与 ControlPolicyConfig 必须分离。
```

### 7.4 `VehicleGenerationConfig`

`VehicleGenerationConfig` 承载初始化和边界生成所需的配置入口。

| 字段 | 说明 |
| --- | --- |
| `generation_mode` | `paper_random`, `deterministic_smoke`, `explicit_scenario` 等 |
| `boundary_queues` | map[`LaneId`, list[`BoundaryQueueItem`]] |
| `arrival_headway_policy` | shifted negative exponential 或显式 headway |
| `entrance_safety_policy_ref` | 入口安全间隙策略引用 |
| `vehicle_type_sampling_policy` | CAV / CHV 抽样策略 |
| `compliance_sampling_policy` | CHV compliance 抽样策略 |
| `random_seed` | 随机种子 |

`BoundaryQueueItem` 建议字段：

| 字段 | 说明 |
| --- | --- |
| `planned_vehicle_id` | 预生成车辆 id 或占位 id |
| `target_lane` | 入口 lane |
| `scheduled_arrival_t` | 计划到达时间 |
| `assigned_headway` | assigned arrival headway |
| `vehicle_spec_template` | 待生成车辆属性模板 |
| `initial_state_template` | 初始 `x_global / y / v / a` 模板，可由 scenario 覆盖 |

本文档不决定入口安全间隙具体公式，也不决定 on-ramp 初始 `x_global` 数值。

### 7.5 `ScenarioConfig`

`ScenarioConfig` 支撑后续最小验证场景、smoke suite 和论文级实验加载。本文定义 `ScenarioConfig` 如何表达场景，不定义有哪些 `MVS-*` 场景；具体 `MVS-*` 场景清单、车辆初始状态和 expected 值由 `CORMC最小验证场景执行规格.md` 维护。

#### 7.5.1 `ScenarioConfig` 顶层字段

| 字段 | 类型 / 取值 | 说明 |
| --- | --- | --- |
| `scenario_id` | string | 场景标识，例如 `MVS-E2E-1` |
| `scenario_name` | string | 场景短名称 |
| `description` / `purpose` | string | 场景目的，供报告和测试输出使用 |
| `test_level` | enum/string | `unit` / `integration` / `smoke` / `probe` / `deferred` |
| `status` | enum/string | `required` / `optional` / `probe` / `deferred` |
| `derivation_ref` | string 或 list[string] | 指向设计论证稿、公式复核位置或备注 |
| `road_config_ref` | id/ref | road geometry config |
| `parameter_config_ref` | id/ref | parameter config |
| `control_policy_config_ref` | id/ref | control policy config |
| `vehicle_generation_config_ref` | optional id/ref | boundary generation 或随机生成配置；确定性 MVS 场景通常关闭 |
| `output_config_ref` | id/ref | output config |
| `initial_time` | `InitialTimeConfig` | 初始 `t`、`step`、`dt` |
| `initial_vehicles` | list[`InitialVehicleConfig`] | 显式车辆初始状态 |
| `module_overrides` | `ModuleOverrideSpec` | 针对该场景关闭随机、打开测试钩子或固定纵向 |
| `preloaded_assignments` | list[`PreloadedAssignmentSpec`] | 预加载 APS assignment cache / effective assignment |
| `preloaded_state_machine_states` | list[`PreloadedStateMachineStateSpec`] | 预加载 lane-change / merge / cache 状态机状态 |
| `preloaded_maneuver_trajectory_states` | list[`PreloadedManeuverTrajectoryStateSpec`] | 预加载 active lane-change / merge trajectory |
| `expected_events` | list[`ExpectedEventSpec`] | 场景预期事件 |
| `expected_sanity_checks` | list[`ExpectedSanityCheckSpec`] | 场景预期 sanity check |
| `expected_png_features` | list[`ExpectedPNGFeatureSpec`] | 场景预期 PNG 可见特征 |
| `tolerances` | `ScenarioToleranceSpec` | 场景级数值容差 |
| `notes` | optional text | 实现注意事项，不作为 loader 必需字段 |

约束：

```text
ScenarioConfig 可以显式覆盖随机属性以便 smoke 验证。
ScenarioConfig 不改变参数规格中的论文参数来源。
ScenarioConfig 不承载日志字段全集；日志字段由 record 结构定义。
ScenarioConfig 不承载场景设计长篇理由；长篇理由保留在设计论证稿。
```

#### 7.5.2 `InitialTimeConfig`

| 字段 | 说明 |
| --- | --- |
| `t` | 初始仿真时间 |
| `step` | 初始时间步 |
| `dt` | 时间步长引用或显式覆盖；默认应与参数规格一致 |

#### 7.5.3 `InitialVehicleConfig`

`InitialVehicleConfig` 建议字段：

| 字段 | 说明 |
| --- | --- |
| `vehicle_id` | 显式车辆 id |
| `vehicle_type` | `VehicleType` |
| `compliance_state` | `ComplianceState` |
| `initial_x_global` | 初始全局坐标 |
| `initial_y` | 初始横向坐标 |
| `initial_v` | 初始速度 |
| `initial_a` | 初始加速度 |
| `physical_lane` | 初始 lane |
| `road_role` | 初始主线 / 匝道身份 |
| `lane_change_state` | 初始换道状态 |
| `merge_state` | 初始合流状态 |
| `spec_overrides` | desired speed、inertial lag、length 等显式覆盖 |

#### 7.5.4 `ModuleOverrideSpec`

`ModuleOverrideSpec` 表达测试场景对默认模块开关或测试钩子的覆盖。建议字段：

| 字段 | 说明 |
| --- | --- |
| `boundary_generation_enabled` | 是否启用边界车辆生成；MVS 默认 false |
| `random_arrival_enabled` | 是否启用随机到达；MVS 默认 false |
| `random_vehicle_attributes_enabled` | 是否启用随机车辆属性；MVS 默认 false |
| `ordinary_mainline_lane_change_enabled` | 是否启用普通主线主动换道；第一版 MVS 默认 false |
| `platoon_cmc_enabled` | 是否启用 platoon CMC；第一版 MVS 默认 false |
| `mpc_lateral_tracking_enabled` | 是否启用 MPC lateral tracking；第一版 MVS 默认 false |
| `quasi_static_longitudinal_override` | 是否在指定窗口锁定或近似锁定纵向运动，用于隔离验证 APS -> CMC 等链路 |
| `test_harness_overrides` | CUC utility override、直接加载 effective assignment 等测试钩子；必须醒目标注为测试钩子 |

约束：

```text
module_overrides 不得伪装成论文原生机制。
quasi_static_longitudinal_override 只用于执行规格明确允许的场景。
```

#### 7.5.5 `PreloadedAssignmentSpec`

| 字段 | 说明 |
| --- | --- |
| `mv_id` | assignment 归属 MV |
| `clv_id` | assigned CLV |
| `cfv_id` | assigned CFV |
| `aps_case` | `APSCase` |
| `col_clv` / `col_cfv` | APS 协同标志 |
| `desired_spacing_override` | Eq.10 spacing 或空 |
| `status` | `APSAssignmentStatus` |
| `created_at_t` / `created_at_step` | 创建时间 |
| `source` | `aps_cache` / `effective_assignment` / `test_preload` |
| `valid_until_next_aps` | 是否允许非 APS 周期沿用 |
| `staleness_policy` | failure 或 invalid 后保留、stale、invalid 等策略记录 |

#### 7.5.6 `PreloadedStateMachineStateSpec`

| 字段 | 说明 |
| --- | --- |
| `vehicle_id` | 车辆 id |
| `lane_change_state` | 初始换道状态覆盖 |
| `merge_state` | 初始合流状态覆盖 |
| `last_aps_time` | APS 周期测试需要的上次 APS 时间 |
| `active_request_state` | CUC / conflict 单元测试可直接加载 active request |
| `notes` | 说明该状态为何预加载 |

#### 7.5.7 `PreloadedManeuverTrajectoryStateSpec`

建议直接复用或映射到 `ManeuverTrajectoryState` 的最小字段：

| 字段 | 说明 |
| --- | --- |
| `vehicle_id` | 执行 maneuver 的车辆 |
| `maneuver_type` | `lane_change` / `merge` |
| `start_t` / `start_step` | 起始时间或 step |
| `start_x_global` | 起点纵向坐标 |
| `start_y` | 起点横向坐标 |
| `target_lane` | 目标 lane |
| `target_y` | 目标中心线 |
| `planned_length` | 正弦轨迹长度语义，可选 |
| `progress` | 已完成进度，建议 `0..1` |
| `assigned_clv_id` / `assigned_cfv_id` | merge trajectory 可选关联 assignment |

#### 7.5.8 `ExpectedEventSpec`

| 字段 | 说明 |
| --- | --- |
| `event_type` | 期望出现的 `EventType` |
| `required` | 是否为场景通过所必需 |
| `time_window` | 允许出现的时间或 step 窗口 |
| `vehicle_ids` | 相关车辆集合 |
| `match` | 需要匹配的语义键值，例如 `case=case_1`、`eq53_pass=true` |
| `numeric_expectations` | 需要按容差比较的数值，如 `h_tilde`、`boundary_speed_cap` |
| `reason_code` | 期望 reason |
| `source` | `paper_formula` / `first_version_engineering_patch` / `test_harness_override` 等 |

#### 7.5.9 `ExpectedSanityCheckSpec`

| 字段 | 说明 |
| --- | --- |
| `check_type` | 期望检查类型，对应 `SanityCheckType` |
| `required` | 是否必须检查 |
| `expected_status` | `pass` / `fail` / `warning` / `not_applicable` |
| `vehicle_ids` | 相关车辆集合 |
| `time_window` | 允许出现的时间或 step 窗口 |
| `reason_code` | 期望 reason |

#### 7.5.10 `ExpectedPNGFeatureSpec`

| 字段 | 说明 |
| --- | --- |
| `feature_type` | 例如 `aps_assignment_marker`、`lane_change_progress`、`merge_start_marker`、`boundary_warning_marker` |
| `required` | 是否必须可见 |
| `vehicle_ids` | 相关车辆集合 |
| `time_window` | 期望出现的时间或 step 窗口 |
| `expected_visibility` | `visible` / `not_visible` / `optional` |
| `notes` | 人工验收说明 |

#### 7.5.11 `ScenarioToleranceSpec`

| 字段 | 说明 |
| --- | --- |
| `position_abs_m` | 位置绝对误差容差 |
| `speed_abs_mps` | 速度绝对误差容差 |
| `time_abs_s` | 时间绝对误差容差 |
| `derived_formula_abs` | 派生公式结果绝对误差容差 |

第一版默认建议：

```text
position_abs_m = 0.05
speed_abs_mps = 0.05
time_abs_s = 0.1
derived_formula_abs = 0.01
```

### 7.6 `OutputConfig`

`OutputConfig` 控制日志、轨迹和 PNG 输出。

| 字段 | 说明 |
| --- | --- |
| `output_dir` | 输出目录 |
| `run_id` | 运行标识 |
| `enable_trajectory_history` | 是否记录 trajectory |
| `enable_event_history` | 是否记录 event |
| `enable_sanity_check` | 是否记录 sanity check |
| `enable_png_output` | 是否输出 PNG |
| `trajectory_output_format` | 第一版默认表格型记录 |
| `event_output_format` | 第一版默认结构化事件记录 |
| `sanity_output_format` | 第一版默认结构化事件记录 |
| `png_time_space_enabled` | 是否输出主 time-space PNG |
| `png_xy_enabled` | 是否输出可选 x-y 图 |
| `plot_use_x_plot` | PNG 渲染时是否使用 `x_global - warmup_length` |

约束：

```text
OutputConfig 只控制记录和渲染。
OutputConfig 不改变仿真算法状态。
plot_use_x_plot 只影响绘图，不影响 VehicleState 或 RelationsSnapshot。
```

## 8. 输出记录结构

### 8.1 `TrajectoryRecord`

`TrajectoryRecord` 是每步每车轨迹记录。第一版推荐表格型记录，每行对应一个 `step, vehicle_id`。

| 字段 | 说明 |
| --- | --- |
| `run_id` | 运行标识 |
| `scenario_id` | 场景标识 |
| `step` | 时间步 |
| `t` | 时间 |
| `vehicle_id` | 车辆 id |
| `vehicle_type` | CAV / CHV |
| `compliance_state` | compliance 状态 |
| `x_global` | 原始算法坐标 |
| `y` | 横向位置 |
| `v` | 速度 |
| `a` | 加速度 |
| `physical_lane` | lane 归属 |
| `road_role` | mainline / on-ramp |
| `primary_leader_id` | 本步纵向 leader |
| `lane_change_state` | 换道状态 |
| `merge_state` | 合流状态 |
| `active_event_tags` | 本步相关事件标签 |

禁止：

```text
不保存作为算法状态的 x_plot。
如需绘图坐标，由渲染层临时计算 x_plot = x_global - warmup_length。
```

### 8.2 `EventRecord`

`EventRecord` 是异构事件统一容器。第一版推荐结构化事件记录。

| 字段 | 说明 |
| --- | --- |
| `event_id` | 事件 id |
| `run_id` | 运行标识 |
| `scenario_id` | 场景标识 |
| `step` | 时间步 |
| `t` | 时间 |
| `module` | 事件来源模块 |
| `event_type` | `EventType` |
| `vehicle_id` | 主相关车辆，可为空 |
| `related_vehicle_ids` | 其他相关车辆 |
| `source_command_id` | 相关 command |
| `source_candidate_id` | 相关 next-state candidate |
| `reason` | reason code 或说明 |
| `result` | 事件结果 |
| `is_engineering_patch` | 是否工程补丁 |
| `payload` | 事件专用结构化内容 |

`payload` 可承载 APS candidate ids、CUC utility、CMC gap result、conflict winner / loser 等专用内容。代码实现中可以用 dict 或 typed record，但必须保持事件主字段稳定。

### 8.3 `SanityCheckRecord`

`SanityCheckRecord` 记录 sanity check 结果，不反向改变本步运动。

| 字段 | 说明 |
| --- | --- |
| `check_id` | 检查记录 id |
| `run_id` | 运行标识 |
| `scenario_id` | 场景标识 |
| `step` | 时间步 |
| `t` | 时间 |
| `check_type` | `SanityCheckType` |
| `severity` | `info`, `warning`, `error` 等 |
| `vehicle_ids` | 涉及车辆 |
| `lane_id` | 涉及 lane，可为空 |
| `x_global` | 涉及纵向位置，可为空 |
| `result` | pass / fail / warning |
| `reason` | reason code 或说明 |
| `source_event_id` | 关联事件 |
| `payload` | 检查专用细节 |

第一版至少记录：

```text
collision
near_collision
boundary_violation
assignment_invalid
state_machine_inconsistency
lane / geometry inconsistency
unexpected ordinary lane-change attempt
multiple commit for one vehicle in one step
```

### 8.4 `OutputHistory`

第一版实现可以用 `OutputHistory` 聚合输出记录。

| 字段 | 说明 |
| --- | --- |
| `trajectory_records` | list[`TrajectoryRecord`] |
| `event_records` | list[`EventRecord`] |
| `sanity_check_records` | list[`SanityCheckRecord`] |
| `png_artifacts` | list[`OutputArtifactRecord`] |

`OutputArtifactRecord` 建议包含 artifact path、artifact type、source records、created step range 等信息。具体文件写出由执行计划决定。

## 9. 模块读写映射表

| 模块 | 读取结构 | 写入结构 | 禁止行为 |
| --- | --- | --- | --- |
| Boundary generation | `VehicleGenerationConfig`, boundary queues, road config | pre-freeze `VehicleSpec`, `VehicleState`, boundary event | 冻结 `SimulationState` 后再插入新车 |
| Freeze state | active vehicle set, config refs | `SimulationState` | 把 command / event 写入 `S(t)` |
| Relation refresh | `SimulationState` | `RelationsSnapshot`, relation events | 按 `x_plot` 排序，按 `y` 连续切换 leader |
| APS | `SimulationState`, `RelationsSnapshot`, road / parameter config, APS cache | `APSAssignment`, `EffectiveAssignmentThisStep`, cache update command, APS event | 写车辆位置或替代 CMC actual leader/follower |
| Request arbitration | `EffectiveAssignmentThisStep`, `CooperativeRequest` | `ConflictResolutionResult`, active request, conflict event | 把工程仲裁写成论文原生机制 |
| CUC | active request, `RelationsSnapshot`, `VehicleSpec`, `VehicleState`, parameter config | `CUCDecision`, `LaneChangeCommand`, `SameStepManeuverRelationOverlay`, CUC event | active lane-change 时重新选择 CUC |
| CMC | MV state, APS cache, road / parameter config, relations | `CMCDecision`, `MergeCommand`, `SpeedCapCommand`, state transition command, CMC event | 模块内部正式改 lane 或撤销 executing merge |
| Longitudinal model | `SimulationState`, relations, command buffer, specs, parameters, controller memory | `CandidateLongitudinalKinematics`, controller memory update candidate, speed-cap consumption event | 反写 `S(t)` 或直接提交速度；写 `y` |
| Lateral trajectory | active maneuver state, planning speed, lane-change / merge command | `CandidateLateralKinematics`, `CandidateManeuverProgress`, lateral event | 正式改 lane、写 `x_global / v / a` 或单独提交真实状态 |
| Commit | `SimulationState`, `CommandBuffer`, `NextStateBuffer` | `S(t+dt)`, commit event, trajectory candidates | 一车多次提交，遗漏 cache cleanup |
| Information integration | `S(t+dt)`, command / candidate / event candidates | `TrajectoryRecord`, `EventRecord`, `SanityCheckRecord`, PNG artifact record | 重新改写已提交车辆运动 |

## 10. 后续文档如何消费本文档

### 10.1 最小验证场景规格

`CORMC最小验证场景执行规格.md` 应使用本文档中的以下结构：

```text
ScenarioConfig
InitialTimeConfig
InitialVehicleConfig
ModuleOverrideSpec
PreloadedAssignmentSpec
PreloadedStateMachineStateSpec
PreloadedManeuverTrajectoryStateSpec
VehicleSpec
VehicleState
ExpectedEventSpec
ExpectedSanityCheckSpec
ExpectedPNGFeatureSpec
ScenarioToleranceSpec
```

最小验证场景可以显式指定车辆初始状态、随机属性覆盖、module_overrides、preloaded assignment 与 active trajectory，但必须说明这些覆盖只服务 smoke 验证，不改变论文参数来源。

### 10.2 执行计划文档

后续执行计划可以按本文档拆分实现阶段：

```text
P01:
    config、enum、VehicleSpec、VehicleState、SimulationState。

P02:
    RoadGeometryConfig、VehicleGenerationConfig、BoundaryQueueItem。

P03:
    CommandBuffer、NextStateBuffer、commit。

P04:
    LongitudinalCommand、LongitudinalControllerMemory、CandidateLongitudinalKinematics。

P05:
    APSAssignment、EffectiveAssignmentThisStep、APS event。

P06:
    CooperativeRequest、ConflictResolutionResult、CUCDecision、LaneChangeCommand。

P07:
    CMCDecision、MergeCommand、SpeedCapCommand。

P08:
    ManeuverTrajectoryState、CandidateLateralKinematics、CandidateManeuverProgress、SameStepManeuverRelationOverlay。

P09:
    TrajectoryRecord、EventRecord、SanityCheckRecord、PNG artifact。

P10:
    ScenarioConfig 和 smoke scenario 加载。
```

执行计划不得首次决定核心字段归属。如需新增字段，先修订本文档。

### 10.3 代码实现约束

代码实现默认采用本文档命名。允许小范围重命名，但必须保持以下不变量：

1. `VehicleState` 不包含 `x_plot`。
2. `SimulationState` 不包含 command、next-state 或 history。
3. `CommandBuffer` 不直接改真实状态。
4. `NextStateBuffer` 不反写 `S(t)`。
5. `CUCDecision` 不作为下一步控制状态。
6. APS cache 生命周期能表达 update、reuse、invalid、failed、cleanup。
7. 工程补丁相关记录能追溯 `reason` 和来源。

## 11. 验收检查

本文档完成后应满足以下检查：

1. 能从 `S(t)`、command、next-state、cache、history、config 六类中定位每个字段归属。
2. 状态接口规格中的每个模块输入输出，都能映射到本文档中的一个结构。
3. 输出指标与日志验证规格中的 trajectory history、event history、sanity check，都有对应 record。
4. `x_global` 是算法和日志原始坐标；`x_plot` 不进入 `VehicleState` 或 `SimulationState`。
5. APS cache 生命周期能表达 update、reuse、invalid、failed、cleanup。
6. CUC choice 只作为本步 command / event，不作为下一步控制状态。
7. `merge_state == executing` 后继续合流，不被 CMC decision 反复重置。
8. 工程补丁相关字段必须带 source / reason，不写成论文原生机制。
9. 最小验证场景执行规格可以直接引用 `ScenarioConfig`、`InitialVehicleConfig`、`ModuleOverrideSpec`、preloaded state 和 expected_* 结构来写可加载场景。
10. 执行计划可以直接按 `CommandBuffer`、`NextStateBuffer`、`SimulationState` 拆实现阶段。
11. 日志与 PNG 输出实现可以直接消费 `TrajectoryRecord`、`EventRecord`、`SanityCheckRecord`。
12. 文档没有复制参数表数值。
13. 文档没有实现 Python 代码。
14. 文档没有设定具体 `MVS-*` 车辆初始数值。
15. 文档没有改变 `CORMC时间步执行顺序梳理.md` 的主循环。

## 12. 待后续修订入口

以下问题不阻止本文档作为第一版代码数据结构设计使用，但后续实现或最小验证场景阶段可能需要修订：

1. CSV / JSONL / Parquet 的最终文件格式和目录结构。
2. `payload` 是否保持 dict，还是拆成 typed event payload。
3. `planned_length`、`progress` 和正弦轨迹完成容差的最终字段细化。
4. boundary speed cap 不可行时的保守策略字段。
5. collision / near-collision 的几何检测粒度和阈值字段。
6. 论文级实验批量运行时的 run metadata、random seed set 和 aggregation record。

这些修订不得反向改变状态读写原则、`x_global` 坐标口径和 commit 唯一真实写入点。
