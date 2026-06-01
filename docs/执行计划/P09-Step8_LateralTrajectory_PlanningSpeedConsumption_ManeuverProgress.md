# P09 - Step8: Lateral Trajectory / Planning Speed Consumption / Maneuver Progress

> 本文档是 P09 的完整执行计划 spec。它只定义后续 P09 red-before-green implementation 的行为合同、测试计划、验收证据和边界消歧；本轮不实现 P09 lateral trajectory / planning speed consumption / maneuver progress / completion detector 代码，不新增 P09 实际测试文件，不修改业务算法。
>
> P09 只覆盖一次 `S(t) + relations / overlay + P07 lane-change command + P05 merge command + P08 planning speed / longitudinal candidate + active ManeuverTrajectoryState -> Step 8 lateral candidate / maneuver progress / completion candidate / event / sanity / PNG marker -> P10 commit` 的时间步切片。P09 不是 APS、CMC、cooperative request、CUC、纵向模型、planning speed composition、commit、artifact export 或论文级实验入口。

## 0. Slice Identity

- Algorithm-Step Coverage:
  - Primary Step: Step 8, lateral trajectory, planning speed consumption, active maneuver progress and completion candidates.
  - Secondary Steps:
    - 消费 P07 `CommandBuffer.lane_change_commands[cv_id]` 和 `same_step_overlays[cv_id]`，为 CUC choice 1 的 lane 2 -> lane 1 生成横向候选。
    - 消费 P05 `CommandBuffer.merge_commands[mv_id]`，为 MV on-ramp -> lane 2 merge start / continuation 生成横向候选。
    - 消费 P08 `NextStateBuffer.candidate_longitudinal[vehicle_id].planning_speed` 或等价 `planning_speeds[vehicle_id]`，推进正弦参考轨迹。
    - 消费冻结 `SimulationState.active_maneuvers[vehicle_id]`，继续 active lane-change / merge，不因本步 relations 变化重置起点或目标。
    - 生成 `CandidateLateralKinematics`、`CandidateManeuverProgress`、completion candidate / `CandidateLaneState` / `CandidateStateTransition`，供 P10 / commit 消费。
    - 生成 canonical `lateral_trajectory` event；maneuver progress、completion、planning-speed consumption、front-collision fallback 只能作为 `module` / `reason` / `payload` 子语义表达，除非先修订权威 `EventType` schema。
    - 生成 boundary-risk sanity 和 PNG marker evidence。
- MVS Acceptance Gate:
  - required:
    - `MVS-SAFE-1B_executing_cap_lateral_consumption`: executing lane-change / merge 消费 P08 capped planning speed，生成 lateral candidate、maneuver progress、lateral trajectory event，不重新做 P08 speed cap composition。
    - `MVS-SAFE-2`: boundary cap 不可行 / boundary risk 在 P09 可观测。当前 P09 不发明完整保守运动策略；required gate 只要求 risk / infeasible evidence、candidate 不直接提交真实状态、event / sanity / marker 可定位。
    - `MVS-CUC-1A_override_choice1` P09 lateral consumption gate: P07 已为该 scenario_id 产出 lane-change command / same-step overlay；P09 只验证该 handoff 被横向轨迹消费，生成 lane 2 -> lane 1 lateral candidate / maneuver progress，不重新证明 CUC choice。
    - P05 merge-start / merge-continuation lateral targeted gate: P05 merge command 被 P09 消费，生成 on-ramp -> lane 2 lateral candidate / maneuver progress。
  - probe:
    - improved sine trajectory numeric diagnostic。
    - front-collision fallback lateral diagnostic。
    - MPC tracking omitted diagnostic。
    - active maneuver progress clipping diagnostic。
    - completion threshold diagnostic。
  - deferred:
    - `MVS-E2E-1`。
    - `MVS-COMMIT-1-full`。
    - P10 full step4-9 integration pass。
    - P11 full smoke suite aggregation、formal PNG renderer、artifact export、regression report。
    - P12 random boundary generation、random attributes、paper-level experiment grid。
    - strict MPC lateral tracking。
    - ordinary mainline autonomous lane change。
- 本阶段解锁的能力:
  - CUC lane 2 -> lane 1 正弦横向候选可观测。
  - MV on-ramp -> lane 2 合流横向候选可观测。
  - P08 capped planning speed 被横向轨迹稳定消费且可追溯。
  - active maneuver progress 可观测，且不会因本步 relations / overlay 变化重置。
  - completion detector 可观测，但只写 candidate / transition request，不直接提交真实 lane / state。
  - front-collision fallback 与 boundary risk 作为 Step 8 safety correction / diagnostic 可观测。
  - P09 event / sanity / PNG marker 不等待 P11 补齐。
- 本阶段不要求通过的后续场景:
  - 不要求 P10 将 P08 longitudinal + P09 lateral assemble 成完整 `CandidateKinematics` 并 commit。
  - 不要求 P11 formal PNG 或 artifact record 存在。
  - 不要求 P12 论文级随机流量、capacity、aggregate metric。

当前成熟度声明:

- 本文档新增后，P09 仍按 P00 追踪矩阵保持 `trace_registered`。
- 本文档不自动把 P09 标记为 `spec_ready` 或 `implementation_ready`。
- P09 进入 red-before-green implementation 前，需要人工审阅本文档，确认 runner route、candidate source guard、front-collision fallback hook、MVS-SAFE-2 risk policy 和 ScenarioConfig handoff gap 的处理顺序。

## 1. 本阶段目标

P09 的目标不是“实现一个泛化横向模型模块”，而是固定 Step 8 在 CORMC 单步流程中的时间步职责:

```text
freeze S(t)
    + Step 3 relations / active_maneuver_relation
    + P07 lane-change command / same-step overlay
    + P05 merge command / merge-state transition request
    + P08 planning speed / longitudinal candidate
    + active ManeuverTrajectoryState
    + road geometry / lane centerline / parameter config
-> Step 8 lateral candidate
    + maneuver progress
    + completion candidate / transition request
    + lateral_trajectory event
    + lateral_trajectory event payload for maneuver progress / completion
    + sanity / PNG marker
-> P10 candidate assembly / commit later decides final S(t+dt)
```

本阶段需要让以下行为从不可观测变为可测试:

1. `MVS-CUC-1A_override_choice1` / P07 choice 1 输出 lane-change command 后，P09 消费该 command 和 same-step overlay，初始化 lane 2 -> lane 1 正弦轨迹 candidate。
2. P05 merge-start command 输出后，P09 消费该 command，初始化 on-ramp -> lane 2 正弦合流轨迹 candidate。
3. `lane_change_state == executing` 或 `merge_state == executing` 且存在 active `ManeuverTrajectoryState` 时，P09 继续既有轨迹，不重跑 CUC / CMC，不重置 `start_x_global / start_y / target_y`。
4. P09 使用 P08 输出的 `planning_speed` 推进横向轨迹，不使用 `VehicleState.v` 偷代，也不重新计算 base speed / speed cap / front fallback composition。
5. `MVS-SAFE-1B_executing_cap_lateral_consumption` 中，executing MV / CV 的 lateral trajectory event 必须记录 consumed speed 来自 P08 capped planning speed。
6. P09 completion detector 可生成 `CandidateManeuverProgress(completed=True)`、`CandidateLaneState`、`CandidateStateTransition` 或等价 next-state candidate，但真实 `physical_lane / road_role / lane_change_state / merge_state / y` 只能由 P10 / commit 写入。
7. P09 记录 `lateral_trajectory` event；maneuver progress、completion、front-collision diagnostic、planning-speed consumption 作为该 canonical event 的 payload / reason / module 子语义记录；boundary risk 使用已登记 sanity 类型或结构化 diagnostic，PNG marker 同步登记。
8. 普通主线主动换道保持关闭，无 P07 lane-change command / active maneuver 时不得创建 ordinary lane-change lateral candidate。
9. 第一版直接按车辆模型规格中 Eq.33-Eq.36 improved sine reference trajectory 的已批准语义推进，不实现 strict MPC lateral tracking。
10. P09 不直接修改冻结 `S(t)` 中任何真实车辆状态。

正弦轨迹公式来源约束:

- P09 spec 不重新发明横向轨迹公式或新增轨迹参数。
- 后续 implementation 必须以 `docs/复现讨论/CORMC车辆模型规格.md` 第 7 节和已批准 Eq.33-Eq.36 口径为准，使用 `start_x_global / start_y / target_y / lateral_displacement / planning_speed` 等既有语义。
- 如果 implementation 发现当前 Markdown 规格不足以写出代码级 Eq.33-Eq.36 数值公式，必须先补车辆模型规格 / 公式映射或记录 formula gap；不得临时自创 sine shape、轨迹长度、完成阈值或新参数来让测试变绿。

当前上游实现能给 P09 的实际输入:

- P05 `CommandBuffer.merge_commands`:
  - merge start command: `command_id`、`vehicle_id`、`command_type=merge`、`init_or_continue_maneuver=init`、`target_lane=lane_2`、`target_y`、`assigned_clv_id`、`assigned_cfv_id`、`source_speed_cap_command_id`、`source`、`is_engineering_patch`。
  - merge continuation command: `command_id`、`vehicle_id`、`command_type=merge`、`init_or_continue_maneuver=continue`、`target_lane`、`target_y`、`active_maneuver_present`、`no_new_eq53_start_decision=True`、`does_not_rejudge_merge_start=True`、`source_speed_cap_command_id`。
  - P05 also exposes `state_transition_commands[mv_id]` for `merge_state -> executing`, and `speed_cap_commands[mv_id]` as P08 input, not as P09 composition input.
- P07 `CommandBuffer.lane_change_commands`:
  - `command_id`、`command_type=lane_change`、`vehicle_id`、`source_request_id`、`source_mv_id`、`source_lane=lane_2`、`target_lane=lane_1`、`target_y`、`cuc_decision_id`、`overlay_id`、`init_maneuver=True`。
  - P07 also exposes `state_transition_commands[cv_id]` for `lane_change_state` request and `same_step_overlays[cv_id]` with `overlay_id`、target lane neighbors、`reason=same_step_cuc_lane_change_relation_overlay`、`is_engineering_patch=True`。
- P08 `Step7LongitudinalRunResult`:
  - `next_state_buffer.candidate_longitudinal[vehicle_id]` with `candidate_id`、`x_global`、`v`、`a`、`candidate_speed`、`planning_speed`、`source`、`constraints_applied`、`source_commands`。
  - `planning_speeds[vehicle_id]` convenience mapping。
  - `longitudinal_model` / `speed_cap` events and `planning_speed_marker` / `speed_cap_consumption_marker` PNG registrations。

当前代码 / runner 事实消歧:

- `cormc/mvs/runner.py` 还没有 P09 route；首轮 P09 red tests 可先用 Python helper 构造 frozen state、relations、P05/P07/P08 handoff 后直接调用 P09 runner。
- `ScenarioConfig` 当前没有 preloaded command buffer、preloaded P08 longitudinal candidate 或 preloaded planning speed 字段；正式接入 MVS runner 前需要修订 loader / runner 合同，或继续使用 Python helper targeted tests。
- `CandidateLateralKinematics`、`CandidateManeuverProgress`、`CandidateLaneState`、`CandidateStateTransition`、`NextStateBuffer.candidate_lateral` 等基础结构已经存在；但 P03 `assemble_candidate_kinematics` source guard 仍只允许 identity / test harness source。P09 首轮可以先验证 P09 component candidates；若要 assemble / commit，必须先修订 P03 source contract。

## 2. 非目标 / 禁止事项

P09 不得实现或重做以下内容:

- 不重做 P04 APS trigger、candidate search、case classifier、assignment cache 或 Eq.10 source creation。
- 不重做 P05 CMC branch selection、assignment validation、Eq.52、Eq.53、boundary speed cap calculation、waiting / merge command creation。
- 不重做 P06 cooperative request collection 或 conflict resolution。
- 不重做 P07 CUC choice、utility、target lane safety、CHV compliance、lane-change command 或 same-step overlay。
- 不自己决定 CUC choice 1 / choice 2。
- 不自己决定 MV 是否开始合流；只能消费 P05 merge command / state transition request / active maneuver state。
- 不重做 P08 longitudinal model、Eq.10 consumption、IDM、CPID、base candidate speed、boundary speed cap composition、front fallback speed composition。
- 不从 P05 speed cap command 重新合成 planning speed；P09 只能读取 P08 已经输出的 final planning speed。
- 不生成 `CandidateLongitudinalKinematics`。
- 不直接改真实 `x / y / v / a / physical_lane / road_role / lane_change_state / merge_state`。
- 不把 lateral candidate 直接提交为真实 `y`。
- 不把 completion detector 直接提交为真实 completed state。
- 不实现 P10 E2E integration、candidate assembly 或 Step 9 commit。
- 不实现 P11 full smoke suite、formal PNG renderer、artifact export 或 regression report。
- 不实现 P12 random boundary generation、random attributes 或 paper-level experiment grid。
- 不使用 `x_plot` 做任何算法判断。
- 不把 event / sanity / PNG marker 推迟到 P11。
- 不实现 ordinary mainline autonomous lane change。
- 不实现 strict MPC lateral tracking；第一版只需要车辆模型规格批准的 sine reference trajectory 和可追溯证据链。
- 不自创 Eq.33-Eq.36 以外的横向轨迹公式、轨迹长度参数、completion threshold 或 lateral progress 计算口径。
- 不把 front-collision fallback 写成完整论文级 Eq.42-Eq.46 复现，除非已有 approved formula / schema / command source；缺口必须记录为 diagnostic / schema gap。
- 不为 MVS-SAFE-2 临时自创完整保守运动策略。当前 P09 只需记录 boundary risk / infeasible cap 可观测证据，并证明不提前提交真实状态。

## 3. 上游 spec 引用

- `docs/执行计划/CORMC执行计划spec设计总纲v1.md`
  - 引用 P09 Step 8 定位：围绕冻结 `S(t)` 与 P08 planning speed 的横向候选生成，消费 P07 lane-change command / same-step overlay 与 P05 merge command。
  - 引用 `MVS-SAFE-1B_executing_cap_lateral_consumption` 和 `MVS-SAFE-2` 为 P09 gate。
  - 引用 P09 消费 P08 planning speed 推进正弦轨迹，不重新计算 P08 纵向控制、不重跑 P07 CUC、不重判 P05 Eq.53。
  - 引用 completion detector 只生成 candidate / state transition request，完成换道 / 合流只在 commit 阶段正式更新 lane / role / state。
- `docs/执行计划/P00-Spec宪法_权威边界与二维追踪矩阵.md`
  - 引用 P09 row: Step 8 正弦横向轨迹 / active maneuver progress / safety correction。
  - 引用 required gate: `MVS-SAFE-1B_executing_cap_lateral_consumption`、`MVS-SAFE-2`。
  - 引用 event evidence: `lateral_trajectory`、maneuver progress、completion candidate event。
  - 引用 sanity evidence: no reset active trajectory、no ordinary lane-change、boundary risk sanity。
  - 引用 P09 不得把 completion 提前写入真实状态。
- `docs/执行计划/P01-MVS_Runner_ScenarioConfig与验收断言语言.md`
  - 引用 expected_events、forbidden_events、expected_event_counts、expected_sanity_checks、expected_png_features、required / probe / deferred matcher 语义。
  - 当前 runner 没有 P09 route；首轮 P09 tests 可以先使用 Python helper，正式接入 runner 需要修订 route / loader contract。
- `docs/执行计划/P02-Step0-3_清理冻结关系与几何口径.md`
  - 引用冻结 `S(t)`、relations snapshot、lane ordering、lane centerline、active maneuver relation 和 `x_global` 算法口径。
  - P09 只读冻结 state / relations；`x_plot` 只允许 PNG / renderer 派生层使用。
- `docs/执行计划/P03-Step9-10_Command_NextState_Commit_Event_Sanity_Trajectory闭环.md`
  - 引用 `CommandBuffer` / `NextStateBuffer` / `CandidateLateralKinematics` / `CandidateManeuverProgress` / `CandidateLaneState` / `CandidateStateTransition` / `EventRecord` / `SanityCheckRecord` / no-write-before-commit。
  - 引用 commit 是唯一真实状态写入点；P09 只写 candidates，不提交真实 state。
  - 引用 `CandidateKinematics` 只能由 candidate assembly / commit preparation 合成，P09 不直接 assemble final committed state。
- `docs/执行计划/P04-Step4A_APS_Cache_EffectiveAssignment.md`
  - P09 不读取 APS internals，也不重跑 APS；只可能在 PNG / event evidence 中间接引用 assignment id / source ids。
- `docs/执行计划/P05-Step4B_CMC_AssignmentValidation_Eq53_BoundaryCap.md`
  - 引用 P05 输出 `merge_commands`、`speed_cap_commands`、`state_transition_commands`。
  - P05 为 P09 提供 `MergeCommand` 或等价 merge trajectory start / continuation command。
  - P09 不重做 P05 CMC branch、assignment validation、Eq.53 或 boundary cap calculation。
  - boundary speed cap 由 P05 产生，P08 施加到 planning speed，P09 只消费已约束 planning speed。
- `docs/执行计划/P06-Step5_CooperativeRequest_ConflictResolution.md`
  - P09 不读取 P06 loser / suppressed request，也不重做 conflict resolution。
- `docs/执行计划/P07-Step6_CUCChoice_Compliance_LaneChangeCommand_SameStepOverlay.md`
  - 引用 `MVS-CUC-1A_override_choice1` 中 P07 choice 1 输出 lane-change command / state transition request / same-step overlay，供 P09 消费。
  - P07 不提前计算正弦轨迹或 lane-change progress。
  - active lane-change 后续继续换道由 P09 / P10 使用已有 lane_change_state / trajectory state，不再重选 maneuver。
- `docs/执行计划/P08-Step7_LongitudinalModel_Eq10SpacingOverride_SpeedCapComposition.md`
  - 引用 P08 -> P09 handoff: P09 消费 P08 planning speed 推进 lateral trajectory。
  - 引用 P08 不创建 lateral candidate / maneuver progress。
  - 引用 speed cap、front fallback、base candidate speed 合成属于 P08；P09 不得搬运这部分逻辑。
- `docs/复现讨论/CORMC时间步执行顺序梳理.md`
  - 引用 Step 8: CUC lane 2 -> lane 1 和 MV merge 都按正弦参考轨迹更新横向位置，MV 使用已受 speed cap 约束后的 planning speed，第一版不做 ordinary mainline lane change。
- `docs/复现讨论/CORMC论文公式与实现映射.md`
  - 引用 lane-changing Eq.33-Eq.36 sine trajectory、front-collision-avoidance Eq.42-Eq.46、MPC Eq.47-Eq.51 第一版关闭 / 简化边界；P09 不得在实现阶段自创新横向公式。
- `docs/复现讨论/CORMC车辆模型规格.md`
  - 引用 speed cap 先作为纵向速度约束施加，再由横向轨迹消费最终 planning speed。
  - 引用 active trajectory 不因每步 relations 变化重置起点或目标 centerline。
  - 引用 completion 条件和 first-version no strict MPC tracking。
- `docs/复现讨论/CORMC状态与模块接口规格.md`
  - 引用 Lateral trajectory input: active maneuver state、planning speed、target centerline、front-collision fallback。
  - 引用 Lateral trajectory output: candidate y、maneuver progress、continue / delay maneuver result；不得正式改 lane，不得单独提交真实状态。
- `docs/复现讨论/CORMC代码数据结构设计_整理版.md`
  - 引用 `LaneChangeCommand`、`MergeCommand`、`CandidateLateralKinematics`、`CandidateManeuverProgress`、`CandidateLaneState`、`CandidateStateTransition`、`NextStateBuffer.candidate_lateral`、`candidate_maneuver_progress`。
  - 若 P09 implementation 需要新增 typed command 或 payload 字段，必须先修订数据结构设计，而不是在代码中暗增。
- `docs/复现讨论/CORMC输出指标与日志验证规格_整理版.md`
  - 引用 Step 8 event / evidence: active trajectory、candidate y、front-collision fallback、boundary speed cap consumption、closed feature guard。
  - 引用 lateral event: maneuver type、progress、target lane、completion、fallback。
- `docs/复现讨论/CORMC最小验证场景执行规格.md`
  - 引用 `MVS-SAFE-1B_executing_cap_lateral_consumption`: executing 状态横向轨迹消费 capped speed。
  - 引用 `MVS-SAFE-2`: cap 不可行 / boundary risk 可记录；本文档消歧为 P09 required 可观测 gate，不要求 P09 发明完整保守策略。

旧 `P09-日志输出与轨迹图` 类临时蓝图不是当前权威入口。P09 边界以 P00 追踪矩阵、总纲、P03/P05/P07/P08 完整 spec、复现讨论权威规格和当前代码事实为准。

## 4. 行为契约 Given / When / Then

### 4.1 Step 8 输入边界

- Given: 冻结 `S(t)`、Step 3 relations、road geometry、VehicleSpec、P05 / P07 command buffer、P08 longitudinal candidate / planning speed、active maneuver state 已存在。
- When: P09 Step 8 调度。
- Then: P09 只读取这些输入，生成 lateral candidate / maneuver progress / completion candidate / event / sanity / PNG marker；不得修改 `S(t)`。

### 4.2 P08 planning speed handoff

- Given: `NextStateBuffer.candidate_longitudinal[vehicle_id].planning_speed` 或 `Step7LongitudinalRunResult.planning_speeds[vehicle_id]` 存在。
- When: P09 计算 lateral trajectory progress。
- Then: 使用该 planning speed 推进正弦轨迹，并在 event payload 记录 `trajectory_consumed_speed`、`trajectory_consumed_speed_source=p08_planning_speed` 或等价字段。
- Then: 若 longitudinal candidate 带有 `constraints_applied=("boundary_speed_cap", ...)` 或 source command 指向 P05 speed cap，P09 只记录 capped speed 被消费，不重新计算 cap。

### 4.3 `MVS-CUC-1A_override_choice1` / P07 choice 1 lane-change start

- Given: P07 输出 `CommandBuffer.lane_change_commands[cv_id]`，其中 `command_type=lane_change`、`source_lane=lane_2`、`target_lane=lane_1`、`target_y` 非空、`init_maneuver=True`，并存在 same-step overlay。
- When: P09 计算该 CV 的 Step 8 lateral trajectory。
- Then: P09 初始化 lane 2 -> lane 1 maneuver candidate，生成 `CandidateLateralKinematics(vehicle_id=cv_id, target_y=lane_1_y, source_commands=(lane_change_command_id, overlay_id, ...))`。
- Then: P09 生成 `CandidateManeuverProgress(maneuver_type=lane_change, progress>0 or progress initialized, completed=False unless target reached)`。
- Then: P09 生成 canonical `lateral_trajectory` event，记录 `maneuver_type=lane_change`、`source_command_id`、`source_overlay_id`、`target_lane=lane_1`、`planning_speed`。
- Then: `source_overlay_id` / `source_commands` 中的 overlay evidence 必须来自实际存在的 `CommandBuffer.same_step_overlays[cv_id]`；若 lane-change command 只声明 `overlay_id` 但 same-step overlay 缺失，只能记录 `declared_overlay_id` / missing diagnostic，不得伪装成 overlay 已被消费。
- Then: `source_scenario_id` 只能来自 command / overlay 显式 trace；不得为了通过 `MVS-CUC-1A_override_choice1` gate 在 P09 内默认补写该 scenario id。
- Then: P09 不执行 CUC utility / target lane safety / compliance，不产生 `CUC` event。

### 4.4 P07 active lane-change continuation

- Given: `VehicleState.lane_change_state == executing` 或 active `ManeuverTrajectoryState(maneuver_type=lane_change)` 已存在。
- When: P09 执行。
- Then: P09 继续已有 maneuver trajectory state，使用 `start_x_global / start_y / target_y / planned_length / progress`，不得因本步 relations 或 overlay 变化重置 start / target。
- Then: 若本步仍有 P06 request 或 P07 history evidence，P09 不重跑 CUC，不重复生成 lane-change init command。

### 4.5 P05 merge-start consumption

- Given: P05 输出 `CommandBuffer.merge_commands[mv_id]`，其中 `command_type=merge`、`init_or_continue_maneuver=init`、`target_lane=lane_2`、`target_y` 非空。
- When: P09 计算该 MV 的 Step 8 lateral trajectory。
- Then: P09 初始化 on-ramp -> lane 2 merge maneuver candidate，生成 `CandidateLateralKinematics(vehicle_id=mv_id, target_y=lane_2_y)`。
- Then: P09 生成 `CandidateManeuverProgress(maneuver_type=merge, progress>0 or initialized)`。
- Then: P09 生成 canonical `lateral_trajectory` event，并在 payload / reason / module 中记录 merge progress、`source_merge_command_id`、`target_lane=lane_2`、`assigned_clv_id` / `assigned_cfv_id` 如命令中存在。
- Then: P09 不重判 Eq.53，不重算 boundary speed cap。

### 4.6 P05 merge continuation / active merge

- Given: `VehicleState.merge_state == executing` 或 active `ManeuverTrajectoryState(maneuver_type=merge)` 已存在，P05 可能输出 `init_or_continue_maneuver=continue` merge command。
- When: P09 执行。
- Then: P09 继续已有 on-ramp -> lane 2 sine merge trajectory，不重新判断是否开始合流，不因短时 gap 变化退回 waiting。
- Then: 如果 P05 command 包含 `does_not_rejudge_merge_start=True`，P09 event 必须保留或等价记录该 evidence。

### 4.7 `MVS-SAFE-1B_executing_cap_lateral_consumption`

- Given: executing MV / CV 已有 P08 capped planning speed，例如 source command 包含 P05 speed cap 或 constraints 包含 `boundary_speed_cap`。
- When: P09 更新 lateral trajectory。
- Then: lateral trajectory event 记录 `trajectory_consumed_speed <= boundary_speed_cap + tolerance` 或记录 consumed speed 等于 P08 planning speed。
- Then: event 必须表明 P09 没有重做 `base_candidate_speed / boundary_speed_cap / front_fallback_speed` 合成；P08 speed cap composition event 是上游证据，P09 只是消费 result。
- Then: P09 生成 lateral candidate / maneuver progress / PNG marker，不生成 P08 `speed_cap` composition event。

### 4.8 `MVS-SAFE-2` boundary risk / infeasible cap

- Given: 上游 P05 / P08 已记录 cap infeasible、negative cap、boundary risk、boundary_violation warning 或等价 risk evidence，且 P09 正在处理相关 MV maneuver。
- When: P09 执行。
- Then: P09 必须保留 boundary risk 可观测证据，生成 `boundary_violation` sanity warning / fail / not_applicable according to upstream policy，并记录 P09 是否延迟、继续或仅输出 diagnostic。
- Then: 若当前 schema / formula 不足以决定完整保守动作，P09 不得发明运动策略；只能记录 schema gap / first-version diagnostic，同时保证不直接提交真实 state。
- Then: `MVS-SAFE-2` 在 P09 只代表可观测 risk / diagnostic / no-write-before-commit gate；除非先修订车辆模型、状态接口和执行计划，不得升级为 P09 full conservative motion policy。
- Then: 若 P09 选择继续产生 candidate，event 必须说明 risk source 和 candidate status；若选择 delay candidate，必须有结构化 event / progress evidence。

### 4.9 Front-collision fallback diagnostic

- Given: front-collision fallback hook / upstream diagnostic / explicit test helper input 可用。
- When: P09 计算 lateral trajectory。
- Then: P09 记录 fallback consumed / not_applicable / schema_gap，必要时设置 `CandidateLateralKinematics.front_collision_fallback=True`。
- Then: 若公式 / 参数 / schema 不足，不得声称完整 Eq.42-Eq.46 论文级复现。

### 4.10 Completion detector

- Given: 正弦轨迹推进后 `candidate_y` 到达 target lane centerline tolerance，或 progress 达到 completion threshold。
- When: P09 运行 completion detector。
- Then: P09 只生成 completion candidate / event:
  - `CandidateManeuverProgress(completed=True, target_y_reached=True)`。
  - `CandidateLaneState(physical_lane=target_lane, road_role=mainline or current role)` if schema supports。
  - `CandidateStateTransition(state_name=lane_change_state or merge_state, new_state=normal / merged)` if schema supports。
- Then: 真实 `VehicleState.physical_lane / road_role / lane_change_state / merge_state / y` 不得被 P09 修改。

### 4.11 Ordinary mainline lane-change closed

- Given: 主线车辆没有 P07 lane-change command，也没有 active `ManeuverTrajectoryState`。
- When: P09 执行。
- Then: 不创建 ordinary lane-change lateral candidate；记录 `unexpected_ordinary_lane_change_attempt` sanity pass 或 closed feature guard evidence。

### 4.12 P09 不重做上游 / 后续

- Given: P04 / P05 / P06 / P07 / P08 event history 或 command buffers 已存在。
- When: P09 执行。
- Then: P09 不产生 `APS`、`APS_candidate`、`CMC` Eq.53 / boundary cap calculation、`assignment_validation`、`cooperative_request`、`conflict_resolution`、`CUC` choice / safety / compliance、`longitudinal_model`、`spacing_override_consumption`、P08 `speed_cap` composition、`commit` event。

## 5. 允许实现的代码对象

后续 P09 implementation 允许新增或修改的代码对象必须服务 Step 8 时间步切片，不得越界实现 P10-P12 或重做 P04-P08。

### 5.1 domain / state objects

- 可新增 P09 本步派生对象，例如:
  - `LateralTrajectoryMode`
  - `PlanningSpeedHandoff`
  - `LateralTrajectoryConsumption`
  - `ManeuverProgressUpdate`
  - `Step8LateralRunResult`
  - `FrontCollisionFallbackDiagnostic`
  - `BoundaryRiskDiagnostic`
- 若这些对象需要成为跨模块正式 schema，必须先更新 `CORMC代码数据结构设计_整理版.md`；否则只能作为 P09 内部 helper / derived result。

### 5.2 command / next-state objects

- 复用 `CommandBuffer.lane_change_commands` 读取 P07 lane-change command。
- 复用 `CommandBuffer.same_step_overlays` 读取 P07 same-step overlay。
- 复用 `CommandBuffer.merge_commands` 读取 P05 merge start / continuation command。
- 复用 `CommandBuffer.state_transition_commands` 读取 P05 / P07 state transition request evidence，但不直接提交。
- 复用 `NextStateBuffer.candidate_longitudinal` 或 P08 result 的 `planning_speeds` 读取 planning speed。
- 复用 `CandidateLateralKinematics` 写 P09 lateral candidate。
- 复用 `CandidateManeuverProgress` 写 P09 maneuver progress。
- 复用 `CandidateLaneState` 和 `CandidateStateTransition` 写 completion candidate if schema supports。
- 复用 `NextStateBuffer.candidate_lateral`、`candidate_maneuver_progress`、`candidate_lane_state`、`candidate_state_transitions` 承载 P09 output。
- P09 不直接写 `CandidateKinematics`，除非 P03 / P10 source guard 已修订并明确允许 Step8 component source 被 assembly 消费。

### 5.3 step runner / service functions

建议新增:

- `cormc/step8_lateral.py`
- `run_step8_lateral_trajectory_planning_speed_progress(...)`
- `run_step8_lateral_for_scenario(...)` 或 test helper，是否接入 MVS runner 由 P09 implementation 阶段决定。
- `select_lateral_maneuver_source(...)`
- `consume_p08_planning_speed(...)`
- `consume_p07_lane_change_command(...)`
- `consume_p05_merge_command(...)`
- `resolve_or_initialize_maneuver_state(...)`
- `compute_sine_lateral_candidate(...)`
- `compute_maneuver_progress(...)`
- `detect_maneuver_completion(...)`
- `build_lateral_candidate(...)`
- `build_maneuver_progress_candidate(...)`
- `build_completion_candidates(...)`

### 5.4 event / sanity helpers

建议新增:

- `emit_lateral_trajectory_event`
- `build_maneuver_progress_payload`
- `build_maneuver_completion_payload`
- `build_planning_speed_consumption_payload`
- `build_front_collision_fallback_payload`
- `run_p09_no_write_before_commit_sanity`
- `run_p09_no_ordinary_lane_change_sanity`
- `run_p09_no_upstream_rerun_sanity`
- `run_p09_boundary_risk_sanity`
- `run_p09_no_longitudinal_candidate_sanity`
- `register_p09_png_features`

Event / sanity schema guard:

- 当前权威 `EventType` 已包含 `lateral_trajectory`，其用途覆盖正弦轨迹、front-collision fallback 和 progress。
- P09 首轮不得把 `maneuver_progress`、`maneuver_completion`、`planning_speed_consumption`、`front_collision_fallback` 暗增为新的 canonical `event_type`。
- progress、completion、planning speed consumption、front fallback 必须放在 `event_type=lateral_trajectory` 的 `module` / `reason` / `payload` 中，或先修订 `CORMC代码数据结构设计_整理版.md`、loader / matcher 和 MVS 场景规格。
- sanity check 只能使用已登记 `SanityCheckType`：`no_write_before_commit`、`x_plot_used_in_algorithm_path`、`state_machine_inconsistency`、`boundary_violation`、`unexpected_ordinary_lane_change_attempt` 等。
- `active_maneuver_not_reset` 不是当前正式 `SanityCheckType`；首轮应放入 `lateral_trajectory` event payload 或现有 sanity payload。若要升级成正式 sanity enum，必须先修订权威数据结构和 loader / matcher。

若要新增 canonical enum，必须先修订数据结构规格和 loader / matcher 合同。

### 5.5 scenario tests

P09 首轮不应依赖尚未登记的 built-in `MVS-SAFE-1B` / `MVS-SAFE-2` route。可以先用 Python helper 构造:

- frozen `SimulationState`
- `RelationsSnapshot`
- active `ManeuverTrajectoryState`
- P05 `CommandBuffer.merge_commands`
- P07 `CommandBuffer.lane_change_commands` / `same_step_overlays`
- P08 `NextStateBuffer.candidate_longitudinal` 或 `planning_speeds`
- optional front fallback / boundary risk diagnostic input

然后直接调用 P09 runner。后续再将 fixtures 登记进 MVS runner / built-in scenario。

### 5.6 regression tests

- P00 static traceability。
- P01 runner / matcher baseline。
- P02 freeze / relations baseline。
- P03 command / next-state / commit / no-write-before-commit baseline。
- P04 APS no rerun baseline。
- P05 merge command / speed cap command baseline。
- P06 active request / conflict baseline。
- P07 lane-change command / overlay baseline。
- P08 planning speed / no lateral candidate baseline。

## 6. 先写失败测试

本次只写 P09 spec，不新增实际 P09 测试文件。后续执行 P09 implementation 时，必须先写 red tests，再实现最小 Step 8。

### 6.1 Red-before-green 顺序

1. 新增 P09 failing tests / test skeleton。
2. 运行 P09 targeted tests，确认红灯发生在 expected event / sanity / lateral candidate / maneuver progress / PNG matcher 层。
3. 红灯不得是 loader error、unknown enum、unknown field、ImportError、AttributeError 或自然语言断言。
4. 同轮实现最小 P09 Step 8 lateral trajectory / planning speed consumption / maneuver progress / completion candidate。
5. 运行 P09 targeted green tests。
6. 运行 P00-P08 回归。
7. 返回 red-before-green 证据，包括红灯失败原因和绿灯 event / sanity / lateral candidate / progress / PNG marker 样例。

### 6.2 Required targeted tests

- `test_p09_mvs_safe_1b_executing_cap_consumes_p08_planning_speed_for_lateral_progress`
  - 构造 executing MV 或 CV，P08 longitudinal candidate 中 `planning_speed=2.63` 且 constraints/source_commands 显示 boundary speed cap 已施加。
  - 断言 P09 lateral trajectory event `trajectory_consumed_speed=2.63`。
  - 断言生成 `CandidateLateralKinematics` 和 `CandidateManeuverProgress`。
  - 断言没有 P08 `speed_cap` composition event、没有 `longitudinal_model` event。

- `test_p09_mvs_safe_2_boundary_risk_is_observable_without_inventing_commit_strategy`
  - 构造 cap infeasible / boundary risk diagnostic。
  - 断言 P09 生成 `boundary_violation` sanity warning / fail / structured diagnostic。
  - 断言 P09 不直接提交真实 lane / y / state。
  - 若当前 schema 无法表达完整 delay / continue policy，断言 schema gap / first-version diagnostic 被结构化记录。

- `test_p09_mvs_cuc_1a_lateral_consumption_creates_lane2_to_lane1_lateral_candidate`
  - 构造 P07 lane-change command 和 same-step overlay。
  - command / overlay 来源必须标明 `source_scenario_id=MVS-CUC-1A_override_choice1` 或等价 trace；P09 只消费该 handoff，不重跑 CUC。
  - 断言 `target_y=lane_1_y`。
  - 断言 `source_commands` 包含 P07 lane-change command id / overlay id。
  - 断言生成 `lateral_trajectory` event 和 lane-change progress marker。
- `test_p09_lane_change_overlay_id_is_not_consumed_when_same_step_overlay_missing`
  - 构造 P07 lane-change command 声明 `overlay_id`，但 `same_step_overlays[cv_id]` 缺失。
  - 断言 P09 可以继续生成 lane-change lateral candidate，但 `source_commands` 不包含 overlay id，event payload 只记录 declared overlay / missing diagnostic。
- `test_p09_source_scenario_id_is_not_defaulted_without_explicit_trace`
  - 构造 P07 lane-change command 和 same-step overlay 均不携带 `source_scenario_id`。
  - 断言 P09 不默认补写 `MVS-CUC-1A_override_choice1`，只在显式 trace 存在时记录 source scenario。

- `test_p09_p05_merge_start_command_creates_on_ramp_to_lane2_lateral_candidate`
  - 构造 P05 merge-start command。
  - 断言 `target_y=lane_2_y`。
  - 断言 `source_commands` 包含 P05 merge command id。
  - 断言生成带 merge progress payload 的 `lateral_trajectory` event / marker。

### 6.3 Unit / integration tests

- `test_p09_active_lane_change_continuation_uses_existing_maneuver_state_without_reset`
  - 构造已有 `ManeuverTrajectoryState`。
  - 本步 relations 或 overlay 改变时，断言 `start_x_global / start_y / target_y` 没有重置。

- `test_p09_active_merge_continuation_does_not_rejudge_eq53_or_boundary_cap`
  - 构造 `merge_state=executing` 和 P05 continue command。
  - 断言 P09 不产生 `CMC` / `assignment_validation` / Eq.53 event。
  - 断言 event payload 保留 `does_not_rejudge_merge_start=True` 或等价 evidence。

- `test_p09_planning_speed_handoff_uses_p08_planning_speed_not_vehicle_v`
  - 设置 `VehicleState.v` 与 P08 `planning_speed` 不同。
  - 断言 lateral progress 使用 P08 planning speed。

- `test_p09_completion_detector_writes_candidates_only`
  - 构造接近 target_y 的 active maneuver。
  - 断言 `CandidateManeuverProgress.completed=True`、`target_y_reached=True`。
  - 若 schema 支持，断言生成 `CandidateLaneState` / `CandidateStateTransition`。
  - 断言真实 `VehicleState.physical_lane / road_role / lane_change_state / merge_state` 未变。

- `test_p09_no_ordinary_mainline_lane_change_without_p07_command`
  - 主线车辆无 P07 command / active maneuver。
  - 断言不创建 lateral candidate。
  - 断言 `unexpected_ordinary_lane_change_attempt` pass。

- `test_p09_does_not_rerun_aps_cmc_p06_p07_or_p08`
  - forbidden events: `APS`、`APS_candidate`、`CMC`、`assignment_validation`、`cooperative_request`、`conflict_resolution`、`CUC`、`longitudinal_model`、`spacing_override_consumption`、P08 `speed_cap` composition。

- `test_p09_does_not_create_longitudinal_candidates`
  - 断言 P09 output 中没有新增 `CandidateLongitudinalKinematics`。

- `test_p09_no_write_before_commit`
  - 深拷贝或冻结 `S(t)`，P09 后逐字段比较真实车辆状态未变。

- `test_p09_does_not_execute_p10_commit`
  - 断言没有 `commit` event。
  - 断言没有 `candidate_kinematics` final assembled candidate，除非 P10 / assembly explicitly invoked by separate test。

- `test_p09_expected_png_features_register_lane_change_and_merge_progress_markers`
  - 断言 `lane_change_trajectory_marker`、`merge_trajectory_marker`、`maneuver_progress_marker`、`planning_speed_consumption_marker` 注册为 renderer deferred。

### 6.4 Static / matcher tests

- P09 expected_events 缺失时，失败必须为 `missing_event` / `event_mismatch`。
- P09 expected_sanity_checks 缺失时，失败必须为 `missing_sanity_check` / `sanity_check_mismatch`。
- P09 expected_png_features 必须可注册为 renderer deferred，不要求真实 PNG。
- P09 首轮红灯不得由 `maneuver_progress` / `maneuver_completion` / `planning_speed_consumption` / `front_collision_fallback` unknown event enum 触发；若需要这些 canonical event_type，必须先单独完成 schema revision。
- 如果正式接入 MVS runner:
  - `_is_p09_lateral_scenario(...)` route 或等价机制必须明确。
  - `MVS-SAFE-1B` 的 P08 prereq 与 P09 required gate 不得相互覆盖:
    - P08 证明 capped planning speed 已形成。
    - P09 证明 capped planning speed 被 lateral trajectory 消费。
  - `MVS-SAFE-2` 不得被写成 P05/P08 单独 full pass；P09 至少要证明 boundary risk 在横向阶段可定位且未提前提交真实状态。

## 7. 验收证据

未来 P09 implementation 完成后，不能只返回 pytest 数字，必须返回以下证据样例或报告摘录。

### 7.1 Required green evidence

- `MVS-SAFE-1B_executing_cap_lateral_consumption` targeted green evidence:
  - P05 speed cap command id。
  - P08 planning speed / candidate id。
  - P08 evidence: constraints include `boundary_speed_cap` or source command points to P05 speed cap。
  - P09 lateral trajectory event shows `trajectory_consumed_speed = P08 planning_speed`。
  - P09 candidate `vehicle_id=MV_SAFE_EXEC` or equivalent。
  - no P08 speed cap recomposition event produced by P09。
  - no P10 commit event。

- `MVS-SAFE-2` boundary risk / infeasible cap evidence:
  - upstream infeasible / risk source id。
  - P09 boundary risk sanity / diagnostic event。
  - candidate status: continued / delayed / diagnostic_only / schema_gap。
  - no direct true state mutation。
  - if strategy is not implemented, explicit first-version schema gap / policy gap evidence。

- `MVS-CUC-1A_override_choice1` P09 lateral consumption evidence:
  - P07 lane-change command id。
  - P07 same-step overlay id。
  - source scenario id / trace = `MVS-CUC-1A_override_choice1`，且 P09 不重新证明 CUC utility / target lane safety。
  - same-step overlay evidence 必须来自实际 `same_step_overlays` 记录，而不是仅来自 command 声明的 `overlay_id`。
  - scenario trace 必须由 P07 command / overlay 显式提供，P09 不默认伪造 `MVS-CUC-1A_override_choice1`。
  - P09 lateral candidate target lane = `lane_1` and `target_y=lane_1_y`。
  - maneuver type = `lane_change`。
  - no CUC rerun event。

- P05 merge-start / merge-continuation lateral consumption evidence:
  - P05 merge command id。
  - `init_or_continue_maneuver=init` or `continue`。
  - P09 lateral candidate target lane = `lane_2` and `target_y=lane_2_y`。
  - maneuver type = `merge`。
  - no Eq.53 / assignment validation rerun event。

- P08 planning speed handoff evidence:
  - `CandidateLongitudinalKinematics.candidate_id`。
  - `planning_speed` value。
  - P09 consumed speed matches P08 planning speed, not `VehicleState.v` when they differ。

### 7.2 Event samples

All samples below intentionally use canonical `event_type = lateral_trajectory`. Progress, completion, planning-speed consumption and front-fallback are payload semantics unless the authoritative `EventType` schema is revised first.

Lateral trajectory event sample:

```text
event_type = lateral_trajectory
module = Step8LateralTrajectory
vehicle_id = MV_SAFE_EXEC
reason = merge_continuation
payload:
    maneuver_type = merge
    source_merge_command_id = p05:0:merge_continue:MV_SAFE_EXEC
    source_longitudinal_candidate_id = p08:0:MV_SAFE_EXEC:longitudinal
    trajectory_consumed_speed = 2.63
    trajectory_consumed_speed_source = p08_planning_speed
    target_lane = lane_2
    target_y = 0.0
    candidate_y = ...
    progress = ...
    active_maneuver_was_reset = false
```

Lane-change event sample:

```text
event_type = lateral_trajectory
module = Step8LateralTrajectory
vehicle_id = CFV_X
reason = lane_change_start
payload:
    maneuver_type = lane_change
    source_lane_change_command_id = p07:0:CFV_X:lane_change
    source_overlay_id = p07:0:CFV_X:same_step_overlay
    target_lane = lane_1
    target_y = 3.5
    trajectory_consumed_speed_source = p08_planning_speed
```

Maneuver progress event sample:

```text
event_type = lateral_trajectory
module = Step8ManeuverProgress
vehicle_id = CFV_X
reason = progress_updated
payload:
    maneuver_type = lane_change
    previous_progress = 0.20
    candidate_progress = 0.27
    completed = false
    target_y_reached = false
```

Completion event sample:

```text
event_type = lateral_trajectory
module = Step8ManeuverCompletion
vehicle_id = MV_SAFE_EXEC
reason = target_y_reached
payload:
    maneuver_type = merge
    completed = true
    target_y_reached = true
    candidate_lane_state = lane_2
    candidate_merge_state_transition = merged
    true_state_written_by_p09 = false
```

### 7.3 Candidate samples

Lateral candidate sample:

```text
CandidateLateralKinematics(
    candidate_id = p09:0:MV_SAFE_EXEC:lateral
    vehicle_id = MV_SAFE_EXEC
    y = ...
    target_y = 0.0
    source = step8_lateral_trajectory
    front_collision_fallback = false
    source_commands = (p05:0:merge_continue:MV_SAFE_EXEC, p08:0:MV_SAFE_EXEC:longitudinal)
)
```

Maneuver progress sample:

```text
CandidateManeuverProgress(
    candidate_id = p09:0:MV_SAFE_EXEC:maneuver_progress
    vehicle_id = MV_SAFE_EXEC
    maneuver_type = merge
    progress = ...
    completed = false
    target_y_reached = false
    source_command_id = p05:0:merge_continue:MV_SAFE_EXEC
)
```

Completion candidate sample:

```text
CandidateLaneState(
    candidate_id = p09:0:MV_SAFE_EXEC:lane_state
    vehicle_id = MV_SAFE_EXEC
    physical_lane = lane_2
    road_role = mainline
    reason = merge_target_y_reached
)

CandidateStateTransition(
    candidate_id = p09:0:MV_SAFE_EXEC:merge_state
    vehicle_id = MV_SAFE_EXEC
    state_name = merge_state
    old_state = executing
    new_state = merged
    reason = merge_target_y_reached
)
```

If `source=step8_lateral_trajectory` is not yet allowed by P03 candidate assembly guards, implementation must either keep these as component candidates only or first revise the candidate source contract. It must not bypass P03 by using `test_harness_preloaded_candidate` for real P09 output.

### 7.4 Sanity samples

- `no_write_before_commit`: pass for every P09 targeted fixture。
- `x_plot_used_in_algorithm_path`: pass。
- `state_machine_inconsistency`: pass / fail according to candidate transition conflict。
- `boundary_violation`: warning / fail / pass according to MVS-SAFE-2 risk evidence。
- `unexpected_ordinary_lane_change_attempt`: pass when no ordinary lane-change candidate is created。
- active maneuver not reset: express as `lateral_trajectory` event payload or approved existing sanity payload. Do not use `active_maneuver_not_reset` as a new `check_type` unless schema is revised first。

### 7.5 PNG marker samples

- `lane_change_trajectory_marker`: visible for CUC lane-change candidate。
- `merge_trajectory_marker`: visible for MV merge candidate。
- `maneuver_progress_marker`: visible for active maneuver progress。
- `planning_speed_consumption_marker`: visible / optional for P08 planning speed handoff evidence。
- `completion_candidate_marker`: visible when completion candidate exists。
- `front_collision_fallback_marker`: visible / optional depending on fallback diagnostic readiness。
- `boundary_risk_marker`: visible for MVS-SAFE-2 risk evidence。

### 7.6 Negative evidence

P09 completion report must show:

- no APS event produced by P09。
- no CMC Eq.53 / assignment validation / boundary cap calculation event produced by P09。
- no cooperative_request / conflict_resolution event produced by P09。
- no CUC choice / safety / compliance event produced by P09。
- no `longitudinal_model` / `spacing_override_consumption` / P08 `speed_cap` composition event produced by P09。
- no `CandidateLongitudinalKinematics` produced by P09。
- no `commit` event produced by P09。
- no direct mutation of `SimulationState.vehicle_states`。
- P00-P08 regression green evidence, or failures explicitly attributed to unrelated existing changes / upstream unresolved gap。

## 8. 完成标准

P09 只有同时满足以下条件，才能声称完成:

- `MVS-SAFE-1B_executing_cap_lateral_consumption` 的 P09 targeted gate 通过。
- `MVS-SAFE-2` 的 P09 责任被清晰处理：required 可观测 risk / diagnostic gate 通过；完整保守运动策略若 schema / policy 不足，必须被明确列为 gap，不得伪装完成。
- P07 lane-change command / same-step overlay 能被 P09 稳定消费。
- P05 merge-start / merge-continuation command 能被 P09 稳定消费。
- P08 planning speed 能被 P09 稳定消费。
- CUC lane 2 -> lane 1 横向候选可观测。
- MV on-ramp -> lane 2 合流横向候选可观测。
- active maneuver progress 可观测，且不会因 relations 变化重置。
- completion detector 可观测，但只写 candidate / transition request。
- P09 只输出 lateral candidate / maneuver progress / completion candidate / event / sanity / PNG marker。
- P09 不重做 APS。
- P09 不重做 CMC、Eq.53 或 boundary cap calculation。
- P09 不重做 P06 cooperative request / conflict。
- P09 不重做 P07 CUC。
- P09 不重做 P08 longitudinal model / planning speed composition。
- P09 不实现 P10 commit。
- P09 不直接写真实车辆状态。
- P09 不实现 ordinary mainline lane change。
- P09 不实现 strict MPC lateral tracking。
- event / sanity / PNG marker 不等 P11 才补。
- required / probe / deferred 语义仍由 P01 runner / matcher 保持。
- P00-P08 回归通过，或任何失败被明确证明为用户已有改动 / 上游未解决 gap，不得忽略。

## 9. 回归保护

后续阶段不得破坏以下 invariant:

- 所有算法内部使用 `x_global`；`x_plot` 只用于 PNG / renderer 派生层。
- P09 输入只读冻结 `S(t)`、relations、P05/P07/P08 handoff 和 active maneuver state。
- P09 不修改真实 vehicle state。
- P09 不修改 assignment cache。
- P09 不修改真实 lane-change / merge state。
- P09 不创建 P08 longitudinal candidate。
- P09 不把 planning speed 反写成真实 speed。
- P09 不把 lateral candidate 反写成真实 `y`。
- P09 不把 completion detector 直接提交为真实 `physical_lane / road_role / lane_change_state / merge_state`。
- P09 只消费 P08 已合成 planning speed，不重新计算 speed cap composition。
- P09 只消费 P05 已产生 merge command，不重新执行 CMC / Eq.53。
- P09 只消费 P07 已产生 lane-change command / overlay，不重新执行 CUC。
- P09 不从 P06 request history 反推出 lane-change command。
- active lane-change / merge 不因 relations 或 overlay 变化重置 trajectory start / target。
- ordinary mainline autonomous lane change 保持关闭。
- strict MPC lateral tracking 保持关闭，除非先修订 P09 spec 和车辆模型规格。
- Eq.33-Eq.36 improved sine trajectory 只能来自车辆模型规格和批准公式；若公式不足，先记录 formula gap，不得在 P09 实现中临时发明轨迹公式或参数。
- front-collision fallback 若无完整 schema / formula，只能记录 diagnostic / not_applicable / schema gap，不得宣称完整复现。
- P09 candidate source 必须通过 P03 / P10 批准，不得借用 test harness source 表示真实模型输出。
- P09 event / sanity / PNG marker 必须在本阶段产生，不得等待 P11 补齐。
- `MVS-SAFE-1B` 的 P08 prereq 和 P09 required gate 必须在报告中区分:
  - P08 证明 capped planning speed 已产生。
  - P09 证明 capped planning speed 被 lateral trajectory 消费。
- `MVS-SAFE-2` 的 boundary risk evidence 不得被 P05/P08/P09 互相覆盖:
  - P05 可证明 cap infeasible / boundary cap command source。
  - P08 可证明 planning speed composition / warning。
  - P09 可证明 lateral stage boundary risk 可定位且未提前提交真实状态。

## 10. 当前代码 / schema gap 清单

这些 gap 是进入 P09 implementation 前必须处理或显式降级的事项；已由首轮 P09 implementation 关闭的条目保留为状态说明，避免后续误判:

1. `cormc/step8_lateral.py` 已由首轮 P09 implementation 新增，当前只覆盖 helper-based Step 8 component candidate / event / sanity / PNG marker，不代表 MVS runner route 已接入。
2. MVS runner 当前没有 P09 route。
3. `MVS-SAFE-1B_executing_cap_lateral_consumption` 和 `MVS-SAFE-2` required built-in scenario 当前未正式接入 runner；首轮应使用 Python helper targeted tests，或先修订 runner / loader。
4. `ScenarioConfig` 当前没有 preloaded command buffer、preloaded P08 longitudinal candidate、preloaded planning speed、preloaded P09 front fallback diagnostic 字段。
5. `CandidateLateralKinematics` 当前字段较薄: `candidate_id`、`vehicle_id`、`y`、`target_y`、`source`、`front_collision_fallback`、`source_commands`。如果 P09 需要记录 `planning_speed`、`progress`、`target_lane`、`source_longitudinal_candidate_id`，应放在 event / progress candidate payload，或先修订数据结构。
6. `CandidateManeuverProgress` 当前没有 `previous_progress`、`planning_speed_used`、`start_x_global`、`start_y`、`target_lane` 字段；这些可在 event payload 中表达，若要状态化需修订数据结构。
7. `ManeuverTrajectoryState` 已存在于 `SimulationState.active_maneuvers`，但当前没有 P09 更新后的持久化结构；P09 首轮可输出 `CandidateManeuverProgress`，跨步持久化由 P10 / P03 commit 边界承接。
8. P03 candidate assembly / commit source guard 当前只允许 identity / test harness source；P09 真实 component source 若要被 assemble，必须先修订 allowed source contract。
9. front-collision fallback 的完整公式、参数和 command schema 可能不足；P09 首轮只能作为 diagnostic / hook，不得宣称论文级完整复现。
10. `MVS-SAFE-2` 的完整保守策略尚未由车辆模型 / 状态接口 / 日志规格拍板；P09 首轮 required gate 只锁定可观测 risk evidence 和 no-write-before-commit。
11. `CommandBuffer.lane_change_commands`、`merge_commands` 当前以 dict payload 承载。若 P09 需要 typed dataclass，应先修订数据结构规格。
12. P09 expected event / sanity check type 若要新增 canonical enum，必须先修订 loader / matcher / 数据结构规格；首轮只能使用 canonical `event_type=lateral_trajectory` 表达 P09 event 语义，把 progress / completion / planning-speed consumption / front fallback 放入 payload / module / reason。
13. `active_maneuver_not_reset` 当前不是正式 `SanityCheckType`；首轮不能作为 `expected_sanity_checks.check_type` 暗增。
14. Eq.33-Eq.36 的代码级数值公式若在车辆模型规格中仍不足，需要先补公式规格或标记 formula gap；不得由 implementation agent 临时想象。

## 11. Implementation Entry Checklist

本节是下一轮 P09 red-before-green implementation 的硬前置，不是建议清单。下一轮 Codex prompt 必须显式复制或等价回答本节的每个决策；若缺失，implementation agent 应先停止并补齐决策，再写测试或代码。

默认进入策略如下；除非人工审阅明确改写，否则下一轮按这些默认值执行:

- runner 路线: 首轮使用 Python helper targeted tests 直接构造 P05/P07/P08 handoff 和 Step8 fixtures，不先改 MVS runner / loader。
- planning speed source: 只读 P08 `CandidateLongitudinalKinematics.planning_speed` 或 `Step7LongitudinalRunResult.planning_speeds`；不得读取 P05 speed cap command 重新合成 planning speed。
- candidate source guard: 首轮可以先验证 `NextStateBuffer.candidate_lateral`、`candidate_maneuver_progress`、`candidate_lane_state`、`candidate_state_transitions`；若要 assemble / commit `CandidateKinematics`，必须先修订 P03 allowed candidate source，批准 `step8_lateral_trajectory` 或等价 source。
- front fallback: 首轮若缺少 approved formula / schema，记录 not_applicable / diagnostic / schema gap；不得实现伪完整 front-collision-avoidance。
- MVS-SAFE-2: 首轮 required gate 验证 boundary risk 可观测和 no-write-before-commit；不自创 full conservative motion policy。
- event / sanity enum: 首轮只使用 canonical `event_type=lateral_trajectory` 和已有 `SanityCheckType`；不得暗增 `maneuver_progress`、`maneuver_completion`、`planning_speed_consumption`、`front_collision_fallback` 或 `active_maneuver_not_reset`。
- sine formula: 首轮实现必须按车辆模型规格批准的 Eq.33-Eq.36 improved sine trajectory；若缺少足够公式细节，先补规格或记录 formula gap，不得临时自创轨迹公式。
- P05/P07 payload: 首轮消费当前 dict payload，不新增 typed dataclass；如需 typed schema，先修订数据结构规格。
- strict MPC: 继续关闭；不实现 Eq.47-Eq.51 tracking。
- ordinary mainline lane change: 继续关闭；无 P07 command / active maneuver 不创建 lateral candidate。

进入 P09 red-before-green implementation 前，执行者还必须确认:

- 已阅读本文档和所有上游 spec。
- 已确认旧临时蓝图不再作为 P09 权威。
- 已决定 P09 首轮是使用 Python helper targeted tests，还是先修订 MVS runner / loader。
- 已决定如何处理 P03 candidate source guard。
- 已决定 front-collision fallback 首轮是实现、probe 还是 schema gap。
- 已决定 `MVS-SAFE-2` 是仅可观测 required gate，还是先修订车辆模型 / 状态接口以实现完整策略。
- 已确认 P05/P07/P08 当前 command / candidate payload 可以覆盖 P09 required gate 的输入。
- 已准备好在完成后返回 event / sanity / lateral candidate / maneuver progress / PNG marker 样例，而不仅是 pytest 数字。
