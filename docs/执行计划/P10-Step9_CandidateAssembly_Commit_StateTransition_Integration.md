# P10 - Step4-9 Integration Slice: APS / CMC / CUC / Longitudinal / Lateral / Commit Closure

> 本文档是 P10 red-before-green implementation 的执行计划 spec。执行者应按本文先写失败测试，再实现或修订 P10 范围内的 candidate assembly / commit closure / state transition consumption / in-memory OutputHistory evidence / targeted runner or helper gate，并返回红灯、绿灯和验收证据。
>
> P10 与总纲 `P10 - Step4-9 集成切片：APS / CMC / CUC / 纵向 / 横向 / commit 同步闭环` 对齐：它验证 P04-P09 多个时间步切片组合后仍保持冻结输入、command / next-state 分离、唯一 commit、cache 生命周期和 active trajectory 生命周期。P10 不新增或重写 APS、CMC、P06 conflict、P07 CUC、P08 longitudinal model 或 P09 lateral trajectory 的核心算法；它消费这些阶段已经产生的 command / candidate / event / sanity evidence，并把闭环提交、组合验收和回归保护落地。

## 0. Slice Identity

- Algorithm-Step Coverage:
  - Primary Coverage: Step 4-9 integration slice per 总纲；implementation touchpoints are Step 9 candidate assembly / synchronized commit, minimal Step 10 in-memory information-integration evidence, and P10 targeted runner / matcher aggregation.
  - Secondary Steps:
    - 消费 P08 `NextStateBuffer.candidate_longitudinal[vehicle_id]`，将 `x_global / v / a / planning_speed evidence / constraints_applied / source_commands` 作为纵向来源。
    - 消费 P09 `NextStateBuffer.candidate_lateral[vehicle_id]`，将 `y / target_y / source_commands / front_collision_fallback` 作为横向来源。
    - 消费 P09 `candidate_maneuver_progress`、`candidate_lane_state`、`candidate_state_transitions`，决定 active maneuver persistence / completion cleanup 和真实 lane / road_role / state machine 写入。
    - 消费 P04-P07 / P05 cache update request、state transition request、command source 和 event evidence，但不重跑任何上游选择或公式。
    - 为每辆 active vehicle 组装至多一个 final `CandidateKinematics`，执行 duplicate guard，不允许 silent last-write-wins。
    - 在 Step 9 commit 中唯一地写入真实 `S(t+dt)`。
    - 在 Step 10 记录 `OutputHistory`、`TrajectoryRecord`、`EventRecord`、`SanityCheckRecord` 和 renderer-deferred PNG markers，不反向修改已提交状态。
- MVS Acceptance Gate:
  - required:
    - `MVS-COMMIT-1-full` P10 targeted subset / P10 responsibility: 每车每步唯一 commit；非 APS 周期 cache reuse 可追踪；P08/P09 component candidates 被组装；active maneuver / cache / state transition / trajectory / event history 生命周期正确。P10 首轮不要求一次性补齐 full runner / full route。
    - `MVS-E2E-1` P10 integration gate: APS case 1 -> no CUC -> CMC Eq.53 pass -> merge-start command -> P08 longitudinal candidate -> P09 lateral candidate -> P10 commit，主链路能在 P10 层形成真实 `S(t+dt)`。
    - `MVS-CUC-1A_override_choice1` P10 commit gate: P07 choice 1 + P09 lane-change candidate 被 P10 commit 成真实 `y` 和 active lane-change state / completion state，不重跑 CUC。
    - `MVS-SAFE-1B_executing_cap_lateral_consumption` P10 commit gate: P09 已消费 capped planning speed，P10 只提交该 candidate，不重新做 speed cap composition。
    - P09 completion commit gate: P09 completion candidate 只能在 P10 commit 后成为真实 `physical_lane / road_role / lane_change_state / merge_state`。
  - probe:
    - candidate source guard diagnostic。
    - missing longitudinal / missing lateral candidate diagnostic。
    - identity fallback / no-lateral normal vehicle diagnostic。
    - active maneuver persistence / cleanup diagnostic。
    - APS cache cleanup after merge completion diagnostic。
    - trajectory / OutputHistory completeness diagnostic。
    - PNG quicklook marker completeness diagnostic。
  - deferred:
    - P11 full required smoke suite aggregation。
    - formal PNG renderer / artifact export / artifact record。
    - regression report。
    - P12 random boundary generation、random vehicle attributes、paper-level experiment grid。
    - strict paper-level numeric equality / aggregate metric。
- 本阶段解锁的能力:
  - P08 longitudinal component candidate 与 P09 lateral / progress / completion component candidates 可被稳定合成为 final `CandidateKinematics`。
  - completion candidate 只在 commit 阶段正式写入真实 lane / role / state。
  - active maneuver progress 可跨步持久化，completion 后可清理。
  - APS assignment cache 可在 merge completion 后经 cache update / cleanup request 清理。
  - P04-P09 的 event / sanity / PNG evidence 在 P10 commit event / history 中可追溯。
  - Step 10 information integration 成为记录层，不是状态回写层。
- 本阶段不要求通过的后续场景:
  - 不要求 P11 formal PNG 文件、artifact record 或 regression report 存在。
  - 不要求 P12 随机边界生成或论文级实验入口。
  - 不要求未完成的 MVS runner full route 在本 spec 写作阶段可直接加载 P08/P09/P10 handoff。

总纲一致性声明:

- P10 是 Step4-9 integration / combination acceptance stage，不是 P04-P09 核心算法重写阶段。
- P10 必须落地或以 helper-targeted 形式证明总纲列出的 E2E event chain matcher、cross-step sanity aggregation、`MVS-E2E-1` targeted regression runner、`MVS-COMMIT-1-full` targeted regression runner 和 engineering patch trace checker。
- 如果当前 MVS runner / ScenarioConfig 尚不足以承载 full route，P10 首轮应使用 helper-based targeted tests 证明 handoff 和 commit closure，并把 runner / loader gap 作为后续修订项；不得让红灯落在 unknown field / loader 层。
- P10 文档和后续 P10 tests 不得把 `on_ramp_mv` 当作 `RoadRole` 新枚举。权威 `RoadRole` 只有 `mainline` / `on_ramp`；MV 身份必须由 vehicle role / `physical_lane=on_ramp` / `merge_state` / source scenario 语义表达。

P00 / 总纲消歧:

- P00 / 总纲把 P10 登记为“Step 4-9 集成：APS / CMC / CUC / longitudinal / lateral / commit 同步闭环”。
- 本文档接受该覆盖范围：P10 的验证责任覆盖 Step 4-9 组合链路。
- 本文档同时收紧实现边界：P10 不重跑 Step 4-8 核心算法，只消费 Step 4-8 已产生的 command / candidate / event / sanity / cache evidence，并在 Step 9 commit closure 与 P10 targeted evidence 中证明组合闭环。
- 因此 `MVS-E2E-1` 在 P10 的责任是验证上游产物能被组装并 commit 成 `S(t+dt)`，同时 event chain / sanity / cache / active trajectory lifecycle 可追踪；不是在 P10 内重新实现 APS / CMC / CUC / P08 / P09。

## 1. 本阶段目标

P10 的目标不是“实现一个新的仿真主循环”，而是按总纲验证 Step4-9 组合切片在 CORMC 时间步流程中闭合。它的新增实现面主要服务以下职责:

- 组合验收 APS / CMC / CUC / P08 / P09 的上游产物链，而不重新计算这些上游算法。
- 将 P08 longitudinal candidate 与 P09 lateral / progress / completion candidates 组装为每车唯一 final `CandidateKinematics`。
- 在 Step 9 commit 中生成真实 `S(t+dt)`，并正式应用 lane / role / state transition / active maneuver / cache lifecycle。
- 产出 P10 所需 in-memory trajectory / event / sanity / PNG marker evidence，供 P11 formal artifact aggregation 使用。
- 落地或以 helper-targeted 方式证明 `MVS-E2E-1`、`MVS-COMMIT-1-full`、active lane-change no-CUC-rerun、merge executing no-rejudge-start、active trajectory continuation 和 engineering patch trace。

```text
freeze S(t)
    + P04 APS / cache update request / effective assignment evidence
    + P05 merge / waiting / speed cap / state transition / cache evidence
    + P06 cooperative request / conflict evidence
    + P07 lane-change command / spacing handoff / same-step overlay
    + P08 candidate_longitudinal / planning_speed / longitudinal events
    + P09 candidate_lateral / maneuver_progress / lane_state / state_transition
    + active ManeuverTrajectoryState
-> Step 9 candidate assembly
    + per-vehicle final CandidateKinematics
    + one-commit-per-vehicle guard
    + candidate lane / state / cache / active maneuver lifecycle application
-> Step 9 commit
    + S(t+dt)
    + commit events / sanity / expected_png_features
-> Step 10 information integration
    + TrajectoryRecord
    + EventRecord / SanityCheckRecord aggregation
    + OutputHistory in-memory record
-> P11 full smoke aggregation / formal artifacts later
```

本阶段需要让以下行为从“P03 lite 能力”升级为“P10 full integration contract”:

1. 同一 vehicle 的 final `CandidateKinematics.x_global / v / a` 来自 P08 longitudinal candidate。
2. 同一 vehicle 的 final `CandidateKinematics.y` 来自 P09 lateral candidate；无 lateral candidate 的普通车辆保留当前 `y / physical_lane / road_role` 或走已授权 identity fallback。
3. `CandidateManeuverProgress` 的 `completed=False` 时，真实 `x / y / v / a` 可随 candidate 推进，但 lane / role / state 保持 current / executing。
4. `CandidateManeuverProgress.completed=True` 且 P09 产出 `CandidateLaneState` / `CandidateStateTransition` 时，P10 才正式写入真实 lane / role / state。
5. lane-change completion 后，真实 `physical_lane=lane_1`，`lane_change_state=normal`，active maneuver 被 cleanup。
6. merge completion 后，真实 `physical_lane=lane_2`，`road_role=mainline`，`merge_state=merged` 或上游状态机批准状态，active maneuver cleanup，并消费 APS cache cleanup / invalidation request。
7. 未完成 lane-change / merge 的 active maneuver progress 必须持久化到 `S(t+dt).active_maneuvers`，供下一步 P02/P09 继续。
8. 每辆车每步最多一次 final candidate 和一次 commit；duplicate candidate / duplicate transition 必须产生 warning 或 sanity fail，不得 silent overwrite。
9. missing longitudinal candidate 不得默认用 `VehicleState.v` 偷代形成不透明 final candidate；若使用 identity fallback，必须有 approved source、event / warning / sanity evidence。
10. Step 10 只记录 OutputHistory / trajectory / event / sanity / PNG marker，不反向修改 `S(t+dt)`。
11. P10 同步产出 commit event、state transition / maneuver completion payload、trajectory record、sanity 和 PNG marker；不得等待 P11 首次补齐。

当前代码事实消歧:

- `cormc/step9_11.py` 已有 P03 lite:
  - `CommandBuffer`、`NextStateBuffer`、`CandidateKinematics`、`CandidateLaneState`、`CandidateStateTransition`、`CandidateManeuverProgress`、`CandidateCacheUpdate`、`CommitWarning`、`CommitResult`、`OutputHistory`、`EventRecord`、`SanityCheckRecord`、`TrajectoryRecord` 已存在。
  - `assemble_candidate_kinematics(...)` 能把 longitudinal / lateral / progress / transition / cache component 组装为 `CandidateKinematics`。
  - `commit_step(...)` 能根据 final candidate 生成 next `SimulationState`，并记录 commit / information integration / trajectory / sanity。
  - `build_next_simulation_state(...)` 能应用 `candidate_lane_state`、`candidate_state_transitions`、`candidate_cache_updates`。
- P03 lite 仍不足以代表 P10 full:
  - `select_final_candidate_per_vehicle(...)` 当前只读取 `NextStateBuffer.candidate_kinematics`，不会自动从 P08/P09 component candidates 组装 final candidate。
  - `ALLOWED_CANDIDATE_SOURCES` 当前只允许 `identity_candidate_for_commit_infrastructure` 和 `test_harness_preloaded_candidate`；P08 `step7_longitudinal_model`、P09 `step8_lateral_trajectory` 真实 component source 不能直接通过 P03 guard。
  - `build_next_simulation_state(...)` 当前保持 `active_maneuvers=state.active_maneuvers`，还没有根据 `CandidateManeuverProgress` 做 persistence / cleanup。
  - commit event payload 已含 `state_transitions`、`cache_cleanup_vehicle_ids`、source component ids，但还没有完整 maneuver lifecycle payload。
  - MVS runner 当前只有 `MVS-COMMIT-1-lite` route，没有 P08/P09/P10 full integration route。
- P08 当前能给 P10 的输入:
  - `NextStateBuffer.candidate_longitudinal[vehicle_id]`。
  - `CandidateLongitudinalKinematics(candidate_id, vehicle_id, x_global, v, a, candidate_speed, planning_speed, source, constraints_applied, source_commands)`。
  - `planning_speeds` convenience mapping、`longitudinal_model` / `speed_cap` events 和 PNG marker evidence。
- P09 当前能给 P10 的输入:
  - `NextStateBuffer.candidate_lateral[vehicle_id]`。
  - `CandidateLateralKinematics(candidate_id, vehicle_id, y, target_y, source, front_collision_fallback, source_commands)`。
  - `CandidateManeuverProgress(candidate_id, vehicle_id, maneuver_type, progress, completed, target_y_reached, source_command_id)`。
  - `CandidateLaneState` / `CandidateStateTransition` completion candidates。
  - `lateral_trajectory` event、boundary / no-write / ordinary lane-change sanity、PNG markers。

## 2. 非目标 / 禁止事项

P10 不得实现或重做以下内容:

- 不重做 P04 APS trigger、candidate search、case classifier、assignment cache selection 或 Eq.10 source creation。
- 不重做 P05 CMC branch、assignment validation、Eq.52、Eq.53、boundary speed cap calculation、merge-start decision 或 waiting decision。
- 不重做 P06 cooperative request collection / conflict resolution。
- 不重做 P07 CUC choice、target lane safety、CHV compliance、lane-change command 或 same-step overlay。
- 不重做 P08 longitudinal model、Eq.10 consumption、IDM、CPID、planning speed composition、speed cap / front fallback composition。
- 不重做 P09 lateral trajectory、sine progress、completion detection 或 front-collision lateral diagnostic。
- 不在 P10 中重新决定 lane-change start、merge start、CUC choice、spacing override 或 speed cap。
- 不用 `VehicleState.v / y` 偷代缺失的 P08/P09 candidate，除非走已批准 identity fallback，并且 event / warning / sanity 可追踪。
- 不在 Step 10 OutputHistory 中反向修改已提交状态。
- 不实现 P11 formal PNG renderer、artifact export、full smoke suite aggregation 或 regression report。
- 不实现 P12 随机边界生成、随机车辆属性或论文级实验。
- 不使用 `x_plot` 做任何算法判断。
- 不暗增字段、enum、ScenarioConfig 字段或 expected_* 结构。
- 不把 P03 `test_harness_preloaded_candidate` 借用为真实 P08/P09 模型输出 source。
- 不把 duplicate candidate / missing component / source guard failure 隐藏成正常 commit。

P10 是唯一允许把 candidates 写成真实 `S(t+dt)` 的阶段；这个权限只来自 commit 语义，不代表 P10 可以重新计算上游算法。P10 的正确性重点是候选组装、唯一提交、状态机转换、active maneuver 生命周期、cache cleanup、OutputHistory 和可追溯证据链。

## 3. 上游 spec 引用

- `docs/执行计划/CORMC执行计划spec设计总纲v1.md`
  - 引用 P10 Step4-9 集成登记；本文接受 Step4-9 验证覆盖，并明确实现触点以 Step9/10 commit closure、targeted runner / matcher aggregation 和 evidence chain 为主。
  - 引用 P04-P10 每个算法切片必须同步产出本阶段 event / sanity / targeted MVS / PNG marker，不得等待 P11。
  - 引用 commit 是唯一生成 `S(t+dt)` 的阶段。
- `docs/执行计划/P00-Spec宪法_权威边界与二维追踪矩阵.md`
  - 引用 P10 row: `MVS-E2E-1`、`MVS-COMMIT-1-full`。
  - 引用 P10 event evidence: end-to-end event chain、cache lifecycle、active maneuver event。
  - 引用 P10 sanity evidence: one commit per vehicle、no rerun CUC、executing merge no rejudge。
  - 引用 P10 当前仍为 `trace_registered`。
- `docs/执行计划/P01-MVS_Runner_ScenarioConfig与验收断言语言.md`
  - 引用 expected_events、forbidden_events、expected_event_counts、expected_sanity_checks、expected_png_features、required / probe / deferred matcher 语义。
  - 当前 runner 没有 P10 full route；首轮 P10 tests 可以用 Python helper 串接 P05/P08/P09/P10 handoff，或先修订 runner / loader contract。
- `docs/执行计划/P02-Step0-3_清理冻结关系与几何口径.md`
  - 引用冻结 `S(t)`、relations snapshot、active maneuver relation 和 `x_global` 算法坐标。
  - P10 commit 后下一步 P02/P03 会重建 relations；P10 不复用本步 relations 作为下一步真实输入。
- `docs/执行计划/P03-Step9-10_Command_NextState_Commit_Event_Sanity_Trajectory闭环.md`
  - 引用 `CommandBuffer` / `NextStateBuffer` / `CandidateKinematics` / commit_step / duplicate guard / OutputHistory / EventRecord / SanityCheckRecord / TrajectoryRecord。
  - 引用 P03 source guard 当前只允许 identity / test harness；P10 必须正式批准 P08/P09 component source，或把 source-contract gap 列为 blocker。
  - 引用 Step 10 information integration 不得重写 committed state。
- `docs/执行计划/P04-Step4A_APS_Cache_EffectiveAssignment.md`
  - 引用 APS cache update / reuse / failure evidence；P10 只消费 cache update / cleanup request，不重跑 APS。
  - 非 APS 周期 cache reuse 需要在 P10 `MVS-COMMIT-1-full` gate 中保持可追溯。
- `docs/执行计划/P05-Step4B_CMC_AssignmentValidation_Eq53_BoundaryCap.md`
  - 引用 P05 merge command、state transition request、speed cap command、cache / assignment evidence。
  - P10 只消费 P05/P08/P09 产物，不重判 Eq.53 或 boundary cap。
- `docs/执行计划/P06-Step5_CooperativeRequest_ConflictResolution.md`
  - P10 不重做 active request 或 conflict resolution；只在 commit event / OutputHistory 中保留上游 source chain。
- `docs/执行计划/P07-Step6_CUCChoice_Compliance_LaneChangeCommand_SameStepOverlay.md`
  - 引用 `MVS-CUC-1A_override_choice1` 的 lane-change command / same-step overlay handoff。
  - P10 不重跑 CUC，不默认伪造 scenario trace 或 overlay consumption。
- `docs/执行计划/P08-Step7_LongitudinalModel_Eq10SpacingOverride_SpeedCapComposition.md`
  - 引用 P08 才计算 longitudinal candidate / planning speed。
  - P10 消费 P08 `CandidateLongitudinalKinematics`，不重新计算 P08 longitudinal model / speed cap composition。
- `docs/执行计划/P09-Step8_LateralTrajectory_PlanningSpeedConsumption_ManeuverProgress.md`
  - 引用 P09 才计算 lateral candidate / maneuver progress / completion candidate。
  - P10 才把 P09 candidate commit 为真实 `y / lane / state`。
  - 引用 P09 current gap: P03 candidate source guard 需要 P10 修订。
- `docs/复现讨论/CORMC时间步执行顺序梳理.md`
  - 引用 Step 9 同步提交和 Step 10 information integration 的流程位置。
- `docs/复现讨论/CORMC论文公式与实现映射.md`
  - 引用 Step 9-10 同步提交是工程实现约束，不是重新定义车辆模型公式。
- `docs/复现讨论/CORMC状态与模块接口规格.md`
  - 引用只有 commit 阶段能生成 `S(t+dt)`、正式写车辆下一状态、正式更新 lane 归属和主线 / 匝道身份。
  - 引用 active maneuver 和 APS assignment cache 的跨步持久化 / cleanup 语义。
- `docs/复现讨论/CORMC代码数据结构设计_整理版.md`
  - 引用 enum、dataclass、buffer、record、ScenarioConfig 与 expected_* 字段权威。
  - 引用 `CandidateKinematics` 只能由 candidate assembly / commit preparation 合成。
  - 引用 `RoadRole` 权威取值只有 `mainline` 和 `on_ramp`；`on_ramp_mv` 不能作为 P10 `VehicleState.road_role` / `TrajectoryRecord.road_role` 样例或新增 enum。
  - 若 P10 需要新增 formal source、event type、sanity type 或 ScenarioConfig handoff 字段，必须先修订本文档和 loader / matcher，而不是代码暗增。
- `docs/复现讨论/CORMC输出指标与日志验证规格_整理版.md`
  - 引用 Step 9 commit event、Step 10 trajectory / information integration、cache cleanup、lane update、multiple commit sanity 和 PNG quicklook marker。
- `docs/复现讨论/CORMC最小验证场景执行规格.md`
  - 引用 `MVS-E2E-1`、`MVS-COMMIT-1-full`、`MVS-CUC-1A_override_choice1`、`MVS-SAFE-1B_executing_cap_lateral_consumption` 的 P10 责任。

旧 P10 临时蓝图或“最小验证场景跑通”类名称不是当前权威入口。P10 边界以 P00 追踪矩阵、总纲、P03/P08/P09 当前实现、复现讨论权威规格和本文档消歧为准。

## 4. 行为契约 Given / When / Then

### 4.1 Step 9 / Step 10 输入边界

- Given: 冻结 `S(t)`、P04-P09 的 `CommandBuffer`、P08 `candidate_longitudinal`、P09 `candidate_lateral / candidate_maneuver_progress / candidate_lane_state / candidate_state_transitions`、candidate cache updates、upstream events / sanity / PNG markers 已存在。
- When: P10 执行 candidate assembly / commit。
- Then: P10 只消费这些输入，组装 final candidate 并在 commit 阶段写 `S(t+dt)`；不得重新调用 P04-P09 算法。

### 4.2 P08 longitudinal + P09 lateral candidate assembly

- Given: 同一 vehicle 同时存在 P08 longitudinal candidate 和 P09 lateral candidate。
- When: P10 组装 final `CandidateKinematics`。
- Then: final candidate 的 `x_global / v / a / constraints_applied` 来自 P08 longitudinal candidate。
- Then: final candidate 的 `y` 来自 P09 lateral candidate。
- Then: final candidate 的 `source_longitudinal_candidate`、`source_lateral_candidate`、`source_maneuver_progress`、`source_state_transition` 必须记录 component candidate ids。
- Then: 如果 P08 / P09 component source 尚未获 P03 source guard 批准，P10 必须先修订 source contract 或返回 schema gap；不得使用 test harness source 冒充真实模型输出。

### 4.3 No-lateral normal vehicle

- Given: 普通车辆有 P08 longitudinal candidate，但没有 P09 lateral candidate，也没有 active maneuver / lane-change / merge state。
- When: P10 组装 final candidate。
- Then: final candidate 使用 P08 `x_global / v / a`，并保留 current `y / physical_lane / road_role / lane_change_state / merge_state`。
- Then: 不得伪造 lateral maneuver、progress 或 lane transition。

### 4.4 Missing longitudinal candidate

- Given: active vehicle 缺少 P08 longitudinal candidate。
- When: P10 尝试组装 final candidate。
- Then: 不得透明地用 `VehicleState.v` 推进形成看似正常的 final candidate。
- Then: 若当前场景允许 identity fallback，必须使用 `identity_candidate_for_commit_infrastructure` 或 P10 批准的 diagnostic fallback source，并记录 missing longitudinal warning / sanity。
- Then: 若场景不允许 fallback，P10 必须让 targeted gate 在 candidate assembly / sanity 层失败，而不是 loader / AttributeError / natural-language assertion。

### 4.5 Unique final candidate / duplicate guard

- Given: 同一 vehicle 存在多个 final `CandidateKinematics` 或互斥 component assembly path。
- When: P10 执行 final candidate selection。
- Then: 触发 `multiple_commit_for_one_vehicle` sanity fail 或 `CommitWarning`。
- Then: 不允许 silent last-write-wins；不允许同一 vehicle 生成多个 commit event。

### 4.6 Step 9 commit is the only true state write

- Given: final candidates 已组装完成。
- When: P10 commit。
- Then: 只在 `S(t+dt)` 中写真实 `x_global / y / v / a / physical_lane / road_role / lane_change_state / merge_state / aps_assignment_cache / active_maneuvers`。
- Then: 冻结 `S(t)` 不得被反写。
- Then: commit event 必须证明 `commit_is_unique_state_writer=True` 或等价 evidence。

### 4.7 Lane-change in-progress commit

- Given: lane-change vehicle 有 P08 longitudinal candidate、P09 lateral candidate、`CandidateManeuverProgress(completed=False)`。
- When: P10 commit。
- Then: 真实 `x_global / y / v / a` 更新为 final candidate。
- Then: `physical_lane` 保持当前 lane，`lane_change_state` 保持 `executing`，`road_role` 保持 current。
- Then: active maneuver progress 持久化到 `S(t+dt).active_maneuvers[vehicle_id]`。
- Then: 不产生 lane-change completion state transition。

### 4.8 Lane-change completion commit

- Given: P09 输出 `CandidateManeuverProgress(completed=True, target_y_reached=True)`，并输出 `CandidateLaneState(physical_lane=lane_1, road_role=mainline)` 与 `CandidateStateTransition(state_name=lane_change_state, new_state=normal)`。
- When: P10 commit。
- Then: `S(t+dt).vehicle_states[cv_id].physical_lane == lane_1`。
- Then: `lane_change_state == normal`。
- Then: active maneuver 被 cleanup，不再进入下一步 `active_maneuvers`。
- Then: commit event / payload 记录 lane-change completion、source P09 progress candidate、source lane state candidate、source transition candidate。

### 4.9 Merge in-progress commit

- Given: merge vehicle 有 P08 longitudinal candidate、P09 lateral candidate、`CandidateManeuverProgress(completed=False)`。
- When: P10 commit。
- Then: 真实 `x_global / y / v / a` 更新为 final candidate。
- Then: `physical_lane`、`road_role`、`merge_state` 保持 current / on_ramp / executing。
- Then: active merge progress 持久化到 `S(t+dt).active_maneuvers[mv_id]`。
- Then: P10 不重判 Eq.53、不重新决定 merge start、不重新计算 boundary cap。

### 4.10 Merge completion commit / cache cleanup

- Given: P09 输出 merge completion progress、`CandidateLaneState(physical_lane=lane_2, road_role=mainline)`、`CandidateStateTransition(state_name=merge_state, new_state=merged)`，并有 P05/P10 cache cleanup request 或 approved cleanup policy。
- When: P10 commit。
- Then: `S(t+dt).vehicle_states[mv_id].physical_lane == lane_2`。
- Then: `road_role == mainline`。
- Then: `merge_state == merged` 或上游状态机批准的完成态。
- Then: active maneuver 被 cleanup。
- Then: APS assignment cache 对该 MV 被 cleanup / invalidated，前提是 `CandidateCacheUpdate` 或等价 approved request 存在；若 schema 不足，记录 cache cleanup schema gap，不得静默留下错误 cache。

### 4.11 Active maneuver persistence / cleanup

- Given: P09 `CandidateManeuverProgress` 存在。
- When: P10 commit。
- Then: `completed=False` 时，P10 需要将 updated `progress`、`last_planning_speed`、start / target 等已批准字段持久化到 `ManeuverTrajectoryState`。
- Then: `completed=True` 时，P10 需要从 `active_maneuvers` 移除该 vehicle 的 maneuver。
- Then: 如果当前 `ManeuverTrajectoryState` 或 `CandidateManeuverProgress` 字段不足以无损持久化，P10 必须在 spec / implementation report 中列 schema gap，而不是用 event payload 伪装跨步 state。

### 4.12 CandidateLaneState / CandidateStateTransition consumption

- Given: `candidate_lane_state[vehicle_id]` 或 `candidate_state_transitions[vehicle_id]` 存在。
- When: P10 commit。
- Then: 这些 candidate 只能在 Step 9 commit 被应用；P08/P09 不得提前改真实状态。
- Then: duplicate or conflicting transitions for same `state_name` 必须触发 state-machine sanity / warning。
- Then: transition old_state mismatch 必须可观测，不得静默覆盖。

### 4.13 `MVS-SAFE-1B_executing_cap_lateral_consumption` P10 commit gate

- Given: P08 longitudinal candidate `planning_speed` 已受 boundary speed cap 约束，P09 lateral candidate 已消费该 speed。
- When: P10 commit。
- Then: P10 只提交 P08/P09 candidates，不重新做 `speed_cap` composition。
- Then: commit event source chain 必须能追到 P08 candidate、P09 lateral candidate、P05 speed cap command。
- Then: forbidden evidence 必须显示没有新的 `longitudinal_model` / `speed_cap` / `lateral_trajectory` recalculation event 由 P10 产生。

### 4.14 `MVS-CUC-1A_override_choice1` P10 commit gate

- Given: P07 choice 1 明确输出 lane-change command / same-step overlay，P09 已生成 lane 2 -> lane 1 lateral candidate。
- When: P10 commit。
- Then: P10 将 candidate `y` 写入真实 state，并根据 progress 决定 in-progress 或 completion state。
- Then: P10 不重跑 CUC，不默认补写 source scenario，不把 command 声明的 overlay 当成实际 consumed overlay。

### 4.15 `MVS-E2E-1` P10 integration gate

- Given: upstream helper or runner route 已产出 APS case 1 -> CMC Eq.53 pass -> merge-start command -> P08 longitudinal candidate -> P09 lateral candidate。
- When: P10 commit。
- Then: MV 的真实 `S(t+dt)` 反映 P08/P09 candidates。
- Then: commit / trajectory / sanity / PNG marker 能证明主链路已在 P10 层闭合。
- Then: 如果当前 MVS runner / ScenarioConfig 不足以 full route，首轮 P10 implementation 必须使用 helper-based targeted gate，并明确 runner / loader gap。

### 4.16 Step 10 OutputHistory

- Given: Step 9 已生成 `S(t+dt)`。
- When: Step 10 information integration 运行。
- Then: 为每个 committed active vehicle 写 `TrajectoryRecord`。
- Then: 写 commit event、state transition / maneuver completion payload、sanity checks 和 expected PNG marker。
- Then: Step 10 不得修改 `S(t+dt)`。

### 4.17 P10 不重做上游

- Given: P04-P09 events / command / candidates 已存在。
- When: P10 执行。
- Then: P10 不产生新的 `APS`、`APS_candidate`、`CMC`、`assignment_validation`、`cooperative_request`、`conflict_resolution`、`CUC`、`longitudinal_model`、`spacing_override_consumption`、`speed_cap`、`lateral_trajectory` recalculation event。
- Then: P10 可在 commit event payload 中引用这些 upstream event / command / candidate ids。

### 4.18 x_global / x_plot

- Given: P10 记录 trajectory / commit state。
- When: 需要坐标。
- Then: 算法和 record 使用 `x_global`；`x_plot` 只允许未来 PNG renderer 派生，不得进入 P10 commit decision。

## 5. 允许实现的代码对象

P10 implementation 允许新增或修改的代码对象必须服务 Step4-9 integration validation 与 Step 9 / Step 10 集成提交切片，不得越界实现 P11-P12 或重做 P04-P09。

### 5.1 domain / state objects

- 复用并可能扩展:
  - `CommitResult`
  - `CommitWarning`
  - `OutputHistory`
  - `TrajectoryRecord`
  - `EventRecord`
  - `SanityCheckRecord`
- 可以新增 P10 内部派生对象，例如:
  - `CandidateAssemblyDiagnostic`
  - `ManeuverLifecycleUpdate`
  - `CommitSourceTrace`
  - `Step9AssemblyRunResult`
- 若新增对象需要成为跨模块正式 schema，必须先修订 `CORMC代码数据结构设计_整理版.md`。

### 5.2 command / next-state objects

- 复用 `CommandBuffer` 读取 P04-P09 command / overlay / transition / cache evidence。
- 复用 `NextStateBuffer.candidate_longitudinal` 读取 P08 output。
- 复用 `NextStateBuffer.candidate_lateral`、`candidate_maneuver_progress`、`candidate_lane_state`、`candidate_state_transitions` 读取 P09 output。
- 复用 `NextStateBuffer.candidate_cache_updates` 承载 APS cache update / cleanup / invalidation。
- 复用 `CandidateKinematics` 作为 final assembled candidate。
- P10 若要批准 `step7_longitudinal_model`、`step8_lateral_trajectory` 或 `step9_candidate_assembly` 作为真实 source，必须修订 P03 candidate source contract 和相关 tests；不得借用 `test_harness_preloaded_candidate`。

### 5.3 step runner / service functions

建议新增或升级:

- `run_step9_candidate_assembly_commit_integration(...)`
- `assemble_final_candidates_from_components(...)`
- `assemble_candidate_from_p08_p09_components(...)`
- `resolve_identity_or_missing_longitudinal_fallback(...)`
- `validate_candidate_source_contract(...)`
- `select_unique_final_candidate_per_vehicle(...)`
- `apply_maneuver_progress_lifecycle(...)`
- `apply_lane_state_and_state_transition_candidates(...)`
- `apply_cache_update_candidates(...)`
- `emit_p10_commit_event(...)` 或扩展 `emit_commit_event(...)`
- `register_p10_png_features(...)`
- `run_p10_no_upstream_rerun_sanity(...)`
- `run_p10_missing_candidate_sanity(...)`
- `run_p10_active_maneuver_lifecycle_sanity(...)`

### 5.4 event / sanity helpers

Event / sanity schema guard:

- `event_type=commit` 是 P10 首轮可直接使用的 canonical commit event。
- `event_type=information_integration` 当前由 P03 `record_information_integration_v0(...)` 代码实际写入，`tests/test_p03_step9_11.py` 和 P01 matcher 也能按字符串 / module 匹配消费；但权威数据结构文档当前未明确把它列为 canonical `EventType`。P10 可以沿用这个现有代码兼容 event，不得把它解释为已经完成的权威 enum 修订。
- 如果后续 implementation 的 schema / loader / matcher preflight 收紧并拒绝 `information_integration`，Step 10 记录语义必须放入 `event_type=commit` 的 payload，或先修订 `EventType` / loader / matcher / 场景规格；红灯不得落在 unknown `EventType`。
- state transition、maneuver completion、cache cleanup、source-chain consumption 应放在 commit event payload 中，除非先修订权威 `EventType` schema。
- 首轮不得暗增 `maneuver_completion_commit`、`state_transition_commit`、`cache_cleanup_commit`、`candidate_assembly` 等 canonical `event_type`。
- sanity check 优先使用已登记或已在 P03 代码中使用的:
  - `multiple_commit_for_one_vehicle`
  - `no_write_before_commit`
  - `state_machine_inconsistency`
  - `x_plot_used_in_algorithm_path`
  - `boundary_violation`
  - `information_integration_does_not_rewrite_state` 仅在沿用 P03 code / matcher 已接受 check 时可用；如果要写入 strict `ScenarioConfig.expected_sanity_checks` 且 schema 未登记，必须先修订权威数据结构或改放 commit payload。
- 若要新增 `missing_longitudinal_candidate`、`missing_lateral_candidate`、`active_maneuver_lifecycle_mismatch`、`candidate_source_guard_failure` 等 formal `SanityCheckType`，必须先修订数据结构规格、loader / matcher 和 MVS 场景规格。

### 5.5 scenario / runner strategy

P10 首轮不应依赖尚未登记的 full built-in `MVS-E2E-1` / `MVS-COMMIT-1-full` route。可以先用 Python helper 构造:

- frozen `SimulationState`
- `CommandBuffer`
- P08 `NextStateBuffer.candidate_longitudinal`
- P09 `NextStateBuffer.candidate_lateral`
- P09 `candidate_maneuver_progress`
- P09 `candidate_lane_state`
- P09 `candidate_state_transitions`
- optional `candidate_cache_updates`

然后直接调用 P10 assembly / commit runner。将 fixtures 登记进 MVS runner / built-in scenario 是独立后续修订项，不是首轮 P10 green 的必要条件。

### 5.6 regression tests

- P00 static traceability。
- P01 runner / matcher baseline。
- P02 freeze / relations baseline。
- P03 commit lite baseline。
- P04 APS no rerun / cache baseline。
- P05 merge command / speed cap / Eq.53 baseline。
- P06 request / conflict baseline。
- P07 lane-change command / overlay baseline。
- P08 longitudinal candidate baseline。
- P09 lateral candidate / completion candidate baseline。

## 6. 先写失败测试

执行 P10 implementation 时，必须先写 red tests，再实现最小 Step4-9 integration validation 与 Step 9 / Step 10 commit closure。

### 6.1 Red-before-green 顺序

1. 新增 P10 failing tests / test skeleton。
2. 运行 P10 targeted tests，确认红灯发生在 expected commit result / candidate assembly / state transition / trajectory / event / sanity / PNG matcher 层。
3. 红灯不得是 loader error、unknown enum、unknown field、ImportError、AttributeError 或自然语言断言。
4. 同轮实现最小 P10 candidate assembly / commit closure / state transition consumption / OutputHistory evidence / targeted gate。
5. 运行 P10 targeted green tests。
6. 运行 P00-P09 回归。
7. 返回 red-before-green 证据，包括红灯失败原因和绿灯 final candidate / commit / state transition / trajectory / event / sanity / PNG marker 样例。

### 6.2 Required targeted tests

- `test_p10_assembles_p08_longitudinal_and_p09_lateral_candidate`
  - 构造同一 vehicle 的 P08 longitudinal candidate 和 P09 lateral candidate。
  - 断言 final `CandidateKinematics.x_global / v / a` 来自 P08。
  - 断言 final `CandidateKinematics.y` 来自 P09。
  - 断言 source component ids 完整。

- `test_p10_normal_vehicle_without_lateral_candidate_keeps_current_y_and_lane`
  - 构造普通主线车只有 P08 longitudinal candidate。
  - 断言 final candidate 使用 P08 `x / v / a`，保留 current `y / lane / role`。
  - 断言没有 lateral maneuver / progress / transition 被伪造。

- `test_p10_missing_longitudinal_candidate_requires_identity_or_diagnostic_fallback`
  - 构造缺失 P08 candidate 的 active vehicle。
  - 断言不能用 `VehicleState.v` 偷代。
  - 若允许 identity fallback，断言 fallback source / warning / sanity 可追踪；否则断言 targeted gate 在 candidate assembly 层失败。

- `test_p10_mvs_commit_1_full_one_commit_per_vehicle_and_duplicate_guard`
  - 构造每车唯一 final candidate 的 full commit fixture。
  - 构造同车 duplicate final candidate / duplicate transition fixture。
  - 断言正常场景 `multiple_commit_for_one_vehicle=pass`。
  - 断言 duplicate 场景 fail / warning，且不双写真实状态。

- `test_p10_lane_change_in_progress_commit_persists_active_maneuver`
  - 构造 lane-change executing + P09 progress `completed=False`。
  - 断言 `x / y / v / a` 更新。
  - 断言 `physical_lane / lane_change_state` 保持 current / executing。
  - 断言 active maneuver progress 持久化。

- `test_p10_lane_change_completion_commit_applies_lane_state_and_cleans_active_maneuver`
  - 构造 P09 lane-change completion candidate。
  - 断言 P10 后真实 `physical_lane=lane_1`、`lane_change_state=normal`。
  - 断言 active maneuver cleanup。
  - 断言 commit event payload 引用 `CandidateLaneState` / `CandidateStateTransition`。

- `test_p10_merge_in_progress_commit_persists_active_maneuver_without_rejudging_eq53`
  - 构造 merge executing + P09 progress `completed=False`。
  - 断言真实 `x / y / v / a` 更新。
  - 断言 `road_role / merge_state` 保持 on-ramp / executing。
  - 断言 P10 不产生 CMC / Eq.53 event。

- `test_p10_merge_completion_commit_applies_mainline_state_and_cache_cleanup`
  - 构造 P09 merge completion candidate 和 APS cache cleanup request。
  - 断言真实 `physical_lane=lane_2`、`road_role=mainline`、`merge_state=merged`。
  - 断言 active maneuver cleanup。
  - 断言 APS cache cleanup / invalidation 被应用或 schema gap 被结构化记录。

- `test_p10_mvs_safe_1b_commits_p09_candidate_without_speed_cap_recomposition`
  - 构造 P08 capped planning speed + P09 lateral consumption。
  - 断言 P10 final candidate / commit 引用 P08/P09 candidates。
  - 断言 P10 不产生新的 `speed_cap` / `longitudinal_model` / `lateral_trajectory` 计算事件。

- `test_p10_mvs_e2e_1_helper_chain_commits_merge_start_handoff`
  - 若 runner route 不足，使用 helper chain 构造 P05/P08/P09/P10 handoff。
  - 断言 MV 在 P10 后产生真实 `S(t+dt)`。
  - 断言报告中列 runner / loader gap。

- `test_p10_step10_output_history_records_commit_trajectory_event_sanity`
  - 断言 `TrajectoryRecord` 记录 commit 后 state。
  - 断言 `EventRecord` 包含 commit / information integration。
  - 断言 `SanityCheckRecord` 包含 duplicate / no-write / state-machine / x_plot。
  - 断言 Step 10 不改 `S(t+dt)`。

- `test_p10_does_not_rerun_aps_cmc_p06_p07_p08_or_p09`
  - forbidden events: `APS`、`APS_candidate`、`CMC`、`assignment_validation`、`cooperative_request`、`conflict_resolution`、`CUC`、`longitudinal_model`、`spacing_override_consumption`、`speed_cap`、`lateral_trajectory`。

- `test_p10_expected_png_features_register_commit_trajectory_and_completion_markers`
  - 断言 `commit_marker`、`trajectory_quicklook`、`lane_change_completed_marker`、`merge_completed_marker`、`active_maneuver_marker` 注册为 renderer deferred。

### 6.3 Static / matcher tests

- P10 expected_events 缺失时，失败必须为 `missing_event` / `event_mismatch`。
- P10 expected_sanity_checks 缺失时，失败必须为 `missing_sanity_check` / `sanity_check_mismatch`。
- P10 expected_png_features 必须可注册为 renderer deferred，不要求真实 PNG。
- P10 首轮红灯不得由 unknown `EventType` / `SanityCheckType` 触发；若需要新的 canonical enum，必须先单独完成 schema revision。
- 如果正式接入 MVS runner:
  - `_is_p10_commit_integration_scenario(...)` route 或等价机制必须明确。
  - `ScenarioConfig` 若需要 preloaded command / P08 / P09 candidates，必须先修订 loader contract。
  - `MVS-E2E-1` 的 upstream algorithm evidence 与 P10 commit evidence 不得互相覆盖。

## 7. 验收证据

P10 implementation 完成后，不能只返回 pytest 数字，必须返回以下证据样例或报告摘录。

### 7.1 Required green evidence

- `MVS-COMMIT-1-full` P10 targeted subset / P10 responsibility green evidence:
  - final candidate count = active vehicle count, except explicitly failed duplicate fixture。
  - 每车 exactly one commit event。
  - `multiple_commit_for_one_vehicle=pass`。
  - active maneuver in-progress persistence evidence。
  - active maneuver completion cleanup evidence。
  - APS cache cleanup / retained cache lifecycle evidence。

- `MVS-E2E-1` P10 integration evidence:
  - APS / CMC / P08 / P09 upstream source chain ids。
  - final `CandidateKinematics` sample。
  - `CommitResult` sample。
  - `S(t)` vs `S(t+dt)` diff sample。
  - If runner route is unavailable, helper-based targeted gate plus runner / loader gap statement。
  - 不要求 P11 formal artifact、full smoke aggregation report 或正式 PNG 文件；只要求 P10 层 commit / trajectory / sanity / marker evidence 证明闭合。

- `MVS-CUC-1A_override_choice1` P10 commit evidence:
  - P07 lane-change command id。
  - P09 lateral candidate id。
  - P10 final candidate id。
  - committed `y` and lane-change state behavior。
  - no CUC rerun event。

- `MVS-SAFE-1B_executing_cap_lateral_consumption` P10 commit evidence:
  - P05 speed cap command id。
  - P08 capped longitudinal candidate id and constraints。
  - P09 lateral consumption candidate id。
  - P10 final candidate / commit event source chain。
  - no P10 speed-cap recomposition event。

### 7.2 Candidate assembly samples

P08/P09 component input sample:

```text
CandidateLongitudinalKinematics(
    candidate_id = p08:0:MV_E2E:longitudinal
    vehicle_id = MV_E2E
    x_global = 6841.20
    v = 12.0
    a = -1.0
    planning_speed = 12.0
    source = step7_longitudinal_model
    constraints_applied = (boundary_speed_cap,)
)

CandidateLateralKinematics(
    candidate_id = p09:0:MV_E2E:lateral
    vehicle_id = MV_E2E
    y = -3.42
    target_y = 0.0
    source = step8_lateral_trajectory
)
```

Final candidate sample:

```text
CandidateKinematics(
    candidate_id = p10:0:MV_E2E:final
    vehicle_id = MV_E2E
    x_global = 6841.20
    y = -3.42
    v = 12.0
    a = -1.0
    source = step9_candidate_assembly
    source_longitudinal_candidate = p08:0:MV_E2E:longitudinal
    source_lateral_candidate = p09:0:MV_E2E:lateral
    source_maneuver_progress = p09:0:MV_E2E:maneuver_progress
    constraints_applied = (boundary_speed_cap,)
)
```

If `step9_candidate_assembly` or P08/P09 component source is not yet approved by source guards, this sample is a target contract, not permission to bypass P03.

### 7.3 CommitResult / state diff sample

```text
CommitResult(
    previous_state = S(t)
    next_state = S(t+dt)
    final_candidates = {MV_E2E: p10:0:MV_E2E:final, ...}
    warnings = ()
    history = OutputHistory(...)
)

S(t).vehicle_states[MV_E2E]:
    x_global = 6840.00
    y = -3.50
    physical_lane = on_ramp
    road_role = on_ramp
    merge_state = executing

S(t+dt).vehicle_states[MV_E2E]:
    x_global = 6841.20
    y = -3.42
    physical_lane = on_ramp
    road_role = on_ramp
    merge_state = executing
```

`RoadRole` 样例必须使用 `on_ramp` / `mainline`。即使当前 loader / 历史测试中存在 `road_role=on_ramp_mv` 漂移，P10 implementation 不得据此暗增 `RoadRole.on_ramp_mv`；MV 身份应通过 `physical_lane=on_ramp`、`merge_state`、vehicle role / source scenario trace 表达。

Completion diff sample:

```text
S(t+dt).vehicle_states[MV_E2E]:
    physical_lane = lane_2
    road_role = mainline
    merge_state = merged
active_maneuvers:
    MV_E2E removed
aps_assignment_cache:
    MV_E2E cleaned up or invalidated
```

### 7.4 Event samples

Commit event sample:

```text
event_type = commit
module = commit
vehicle_id = MV_E2E
reason = commit_step_final_candidate
payload:
    candidate_source = step9_candidate_assembly
    source_longitudinal_candidate = p08:0:MV_E2E:longitudinal
    source_lateral_candidate = p09:0:MV_E2E:lateral
    source_maneuver_progress = p09:0:MV_E2E:maneuver_progress
    source_state_transition = p09:0:MV_E2E:merge_state
    constraints_applied = [boundary_speed_cap]
    final_state = {x_global: ..., y: ..., physical_lane: lane_2, road_role: mainline, merge_state: merged}
    previous_state = {...}
    state_transitions = [...]
    cache_cleanup_vehicle_ids = [MV_E2E]
    active_maneuver_cleanup_vehicle_ids = [MV_E2E]
    commit_is_unique_state_writer = true
```

Information integration event sample，仅在沿用 P03 当前 accepted event string 时使用:

```text
event_type = information_integration
module = information_integration
reason = step10_record_only
payload:
    trajectory_records_added = <active vehicle count>
    event_history_updated = true
    sanity_history_updated = true
    step10_does_not_rewrite_committed_state = true
```

If strict schema preflight rejects `information_integration`, do not add a private enum. Move the Step 10 record-only fields into the commit event payload or revise the authoritative `EventType` / loader / matcher first.

### 7.5 TrajectoryRecord sample

```text
TrajectoryRecord(
    step = 0
    t = 0.0
    vehicle_id = MV_E2E
    x_global = <committed x>
    y = <committed y>
    v = <committed v>
    a = <committed a>
    physical_lane = lane_2
    road_role = mainline
    merge_state = merged
    active_event_tags = (commit,)
)
```

No `x_plot` may appear in algorithm state or trajectory record.

### 7.6 Sanity samples

- `multiple_commit_for_one_vehicle`: pass in normal full commit; fail / warning in duplicate fixture。
- `no_write_before_commit`: pass comparing frozen `S(t)` before and after assembly / commit invocation。
- `state_machine_inconsistency`: pass unless conflicting lane-change / merge state exists。
- `x_plot_used_in_algorithm_path`: pass。
- `information_integration_does_not_rewrite_state`: pass if accepted by P03 / matcher contract。
- candidate source guard diagnostic: event / sanity / warning if source not approved; formal check type requires schema revision。
- missing longitudinal / missing lateral diagnostic: event / sanity / warning if fallback is used; formal check type requires schema revision。

### 7.7 PNG marker samples

- `commit_marker`: optional / visible for committed vehicles。
- `trajectory_quicklook`: optional / visible for committed trajectory points。
- `lane_change_completed_marker`: visible when lane-change completion commit occurs。
- `merge_completed_marker`: visible when merge completion commit occurs。
- `active_maneuver_marker`: visible / optional for in-progress maneuver persistence。
- `cache_cleanup_marker`: visible / optional when merge completion cleans APS cache。
- `source_chain_marker`: optional for P08/P09 component consumption evidence。

### 7.8 Negative evidence

P10 completion report must show:

- no APS event produced by P10。
- no CMC Eq.53 / assignment validation / boundary cap calculation event produced by P10。
- no cooperative_request / conflict_resolution event produced by P10。
- no CUC choice / safety / compliance event produced by P10。
- no `longitudinal_model` / `spacing_override_consumption` / `speed_cap` recomposition event produced by P10。
- no `lateral_trajectory` recomputation event produced by P10。
- no direct mutation of frozen `S(t)` before commit。
- no Step 10 mutation of `S(t+dt)`。
- P00-P09 regression green evidence, or failures explicitly attributed to unrelated existing changes / upstream unresolved gap。

Forbidden-event evidence must be structured test / matcher evidence, not log text:

- `test_p10_does_not_rerun_aps_cmc_p06_p07_p08_or_p09` must assert `forbidden_events` / actual event set excludes `APS`、`APS_candidate`、`CMC`、`assignment_validation`、`cooperative_request`、`conflict_resolution`、`CUC`、`longitudinal_model`、`spacing_override_consumption`、`speed_cap`、`lateral_trajectory` produced by P10。
- `test_p10_mvs_safe_1b_commits_p09_candidate_without_speed_cap_recomposition` must specifically bind `forbidden_events` to `speed_cap` / `longitudinal_model` / `lateral_trajectory` recomputation while allowing source-chain references in commit payload。
- `test_p10_mvs_cuc_1a_commit_does_not_rerun_cuc` must bind `forbidden_events` to `CUC` while allowing P07 command ids and same-step overlay ids as payload references。
- If using P01 matcher, these checks should use `forbidden_events` and `expected_event_counts` fields where possible; otherwise the test must inspect `result.history.event_dicts()` or P10 actual event dicts directly。

## 8. 完成标准

P10 只有同时满足以下条件，才能声称完成:

- P08 longitudinal candidate 能被 P10 稳定消费。
- P09 lateral / maneuver progress / completion candidate 能被 P10 稳定消费。
- P10 能为每车形成唯一 final candidate，且 duplicate candidate 不会 silent overwrite。
- P10 是唯一真实状态写入点；P04-P09 仍只写 command / candidate / event。
- lane-change in-progress commit 行为正确。
- lane-change completion commit 行为正确。
- merge in-progress commit 行为正确。
- merge completion commit 行为正确。
- active maneuver state 能正确 persistence / cleanup。
- CandidateStateTransition / CandidateLaneState 被正确应用，或明确列为 schema gap。
- APS cache cleanup / invalidation 被正确应用，或明确列为 schema gap。
- `MVS-COMMIT-1-full` 的 P10 targeted subset / P10 responsibility 被清晰处理；不要求首轮补齐 full runner / full route。
- `MVS-E2E-1` 的 P10 responsibility 被清晰处理；若 full runner route 仍不足，必须列 gap 与 targeted 替代证据。
- OutputHistory / TrajectoryRecord / EventRecord / SanityCheckRecord 能记录 P10 结果。
- P10 不重做 APS。
- P10 不重做 CMC / Eq.53 / boundary cap calculation。
- P10 不重做 P06。
- P10 不重做 P07 CUC。
- P10 不重做 P08 longitudinal model / planning speed composition。
- P10 不重做 P09 lateral trajectory。
- P10 不实现 P11 formal PNG / artifact export / regression report。
- event / sanity / PNG marker 不等 P11 才补。
- required / probe / deferred 语义仍由 P01 runner / matcher 保持。
- P00-P09 回归通过，或任何失败被明确证明为用户已有改动 / 上游未解决 gap，不得忽略。

## 9. 回归保护

后续阶段不得破坏以下 invariant:

- 所有算法内部使用 `x_global`；`x_plot` 只用于 PNG / renderer 派生层。
- 所有 P10 输入来自冻结 `S(t)`、CommandBuffer、NextStateBuffer 和上游 evidence。
- P10 不重跑上游算法。
- P10 不修改冻结 `S(t)`。
- P10 只在 Step 9 commit 生成 `S(t+dt)`。
- Step 10 information integration 不修改 `S(t+dt)`。
- 每辆 active vehicle 每步最多一个 final candidate。
- 每辆 active vehicle 每步最多一个 commit event。
- P08 longitudinal source 与 P09 lateral source 必须可追踪。
- P08/P09真实 model source 必须通过 source guard；不得借用 test harness source。
- no-lateral normal vehicle 不得伪造 lateral maneuver。
- missing longitudinal candidate 不得透明偷代。
- completion candidate 只在 P10 commit 后正式改 lane / role / state。
- active maneuver in-progress 必须持久化；completion 必须 cleanup。
- merge completion 后 APS cache cleanup / invalidation 不能被遗忘；若 schema 不足必须显式 gap。
- duplicate transition / conflicting transition 必须触发 warning / sanity。
- `lane_change_state == executing` 与 `merge_state == executing` 不得同车同时为真。
- P10 commit event / sanity / PNG marker 必须在本阶段产生，不得等待 P11 补齐。
- P11 只做 aggregation / formal artifact，不补 P10 首次证据。

## 10. 当前代码 / schema gap 清单

这些 gap 是进入 P10 implementation 前必须处理或显式降级的事项:

1. `select_final_candidate_per_vehicle(...)` 当前只读取 `NextStateBuffer.candidate_kinematics`，不会自动从 P08 `candidate_longitudinal` 和 P09 `candidate_lateral` 组装 final candidate。
2. `assemble_candidate_kinematics(...)` 当前默认 source 为 `test_harness_preloaded_candidate`，真实 P10 implementation 需要正式 source contract，例如批准 `step9_candidate_assembly`。
3. P03 component source guard 当前拒绝 P08 `step7_longitudinal_model` 和 P09 `step8_lateral_trajectory`；P10 必须先修订 `ALLOWED_CANDIDATE_SOURCES` / source policy 或列为 blocker。
4. `build_next_simulation_state(...)` 当前不会根据 `CandidateManeuverProgress` 更新 `active_maneuvers`；它直接保留 `state.active_maneuvers`。
5. `CandidateManeuverProgress` 当前字段较薄，缺少 `start_x_global`、`start_y`、`target_lane`、`planned_length`、`last_planning_speed` 等持久化所需字段。P10 可从 existing active maneuver + progress candidate 合成；如不足必须修订 schema。
6. `ManeuverTrajectoryState` 有 `last_planning_speed` 字段，但当前 P09 没有直接输出持久化候选结构；P10 需要定义如何从 P09 progress candidate 更新它。
7. merge completion cache cleanup 需要 `CandidateCacheUpdate` 或 approved command source；当前 P09 不自动生成 cache cleanup candidate，P10 / P05 / P03 需明确来源。
8. commit event 当前有 `cache_cleanup_vehicle_ids`，但没有正式 `active_maneuver_cleanup_vehicle_ids` 字段；可先放 payload，若要 schema 化需修订数据结构。
9. 权威 `RoadRole` 只有 `mainline` / `on_ramp`，但当前 loader / 历史 fixtures / 上游代码中仍存在 `road_role=on_ramp_mv` 漂移。P10 新样例和新测试必须使用 `road_role=on_ramp` 表达匝道身份；不得暗增 `on_ramp_mv` enum。
10. `information_integration` event_type 当前由 P03 code 写入，且 P03 tests / P01 matcher 能消费；但权威数据结构文档可能未列为正式 `EventType`。P10 若写入 strict expected_events，应先确认当前兼容口径或修订权威文档；否则将 Step 10 record-only 语义放入 commit payload。
11. `information_integration_does_not_rewrite_state` 当前由 P03 code 使用，但数据结构文档的正式 `SanityCheckType` 列表可能未列出。若 P10 将其写入 ScenarioConfig expected_sanity_checks，应先确认 / 修订权威文档。
12. MVS runner 当前没有 P08 / P09 / P10 full route；`MVS-E2E-1` 与 `MVS-COMMIT-1-full` 首轮可能需要 helper-based targeted tests。
13. `ScenarioConfig` 当前没有 preloaded command buffer、preloaded P08 candidate、preloaded P09 candidate、preloaded cache update candidate 字段；不得让 red test 失败在 unknown field / loader 层。
14. `expected_events` / `expected_sanity_checks` 当前只支持有限字段；P10 新语义应放 payload match 中，或先修订 loader / matcher。
15. `MVS-COMMIT-1-full` built-in route 尚未与 P04-P09 helper outputs 串接；P10 首轮只要求 targeted subset / P10 responsibility。
16. `MVS-E2E-1` 端到端 runner route 尚不足以证明 APS->CMC->P08->P09->P10 full chain；P10 spec 允许首轮 targeted helper 替代，但必须报告 gap。
17. full collision / near-collision post-commit sanity 若需要精确矩形碰撞，当前可能仍是 baseline / probe；不得把它伪装为 P10 required full safety policy。

## 11. Implementation Entry Checklist

本节是执行 P10 red-before-green implementation 的硬前置，不是建议清单。执行 prompt 必须显式复制或等价回答本节的每个决策；若缺失，implementation agent 应先停止并补齐决策，再写测试或代码。

默认进入策略如下；除非人工审阅明确改写，否则按这些默认值执行:

Implementation decision locks:

- Lock 1 - source guard: P10 首轮必须先修订 / 明确 P03 candidate source policy，使真实 P08/P09/P10 handoff source `step7_longitudinal_model`、`step8_lateral_trajectory`、`step9_candidate_assembly` 能被合法消费；若不修订，则该实现轮必须停在 source-contract blocker。不得用 `test_harness_preloaded_candidate` 表示真实 P08/P09 model output。
- Lock 2 - active maneuver lifecycle: P10 首轮必须实现或显式结构化降级 active maneuver in-progress persistence 与 completion cleanup。实现时优先用现有 `ManeuverTrajectoryState` + P09 `CandidateManeuverProgress` 更新 progress / source command / last planning speed；若字段不足以无损持久化，必须返回 schema gap，不能只写 event payload 冒充跨步 state。
- Lock 3 - merge completion cache cleanup source: merge completion 后 APS cache cleanup / invalidation 只能由 `CandidateCacheUpdate`、approved cache / state transition command source 或显式 diagnostic gap 承载。P09 不自动生成 cleanup candidate 时，P10 不得静默 cleanup，也不得静默保留错误 cache。
- Lock 4 - runner route vs helper tests: P10 首轮使用 Python helper targeted tests 直接构造 frozen state、CommandBuffer、P08 component candidates、P09 component candidates 和 P10 fixtures；不先扩展 `ScenarioConfig` / loader 接受未登记 preloaded fields，不让红灯落在 unknown field / loader 层。full MVS runner route 留给后续单独修订。

- runner 路线: 按 Lock 4。
- source guard: 按 Lock 1。
- candidate assembly: 先实现 component -> final `CandidateKinematics` 的最小 assembly，不重写 P08/P09。
- missing candidate policy: 普通 no-lateral 可以保留 current y；missing longitudinal 只能 identity fallback with evidence 或 fail，不透明偷代。
- active maneuver lifecycle: 按 Lock 2。
- cache cleanup: 按 Lock 3。
- event / sanity enum: 首轮直接使用 canonical `commit`；`information_integration` 只能作为 P03 code / matcher 已接受的兼容 event string 使用，不得声称已登记为权威 `EventType`。若 strict schema 不接受，改放 commit payload 或先修订权威数据结构。`information_integration_does_not_rewrite_state` 同理只能沿用已接受检查或先修订 `SanityCheckType`。
- P11 boundary: 不实现 formal PNG / artifact export / regression report。
- P12 boundary: 不实现随机边界或论文级实验。

开始 P10 red-before-green implementation 前，执行者还必须确认:

- 已阅读本文档和所有上游 spec。
- 已确认 P10 是 Step4-9 integration validation；具体实现触点集中在 Step9/10 commit closure、targeted runner / matcher aggregation 和 evidence chain，不重跑 P04-P09。
- 已确认 P03 lite 与 P10 full 的差距。
- 已决定 candidate source guard 修订策略。
- 已决定 runner route vs helper targeted tests。
- 已决定 active maneuver persistence / cleanup schema。
- 已决定 APS cache cleanup source。
- 已确认 P08/P09 当前 candidate payload 足以覆盖 required P10 gate，或已列 schema gap。
- 已准备好在完成后返回 final candidate / CommitResult / S(t) vs S(t+dt) / event / sanity / trajectory / PNG marker 样例，而不仅是 pytest 数字。
