# P08 - Step7: Longitudinal Model / Eq.10 Spacing Override / Speed Cap Composition

> P08 只覆盖一次 `S(t) + relations + P07 cooperation / spacing command + P05 speed cap command -> Step 7 longitudinal candidate / planning speed / event / sanity / PNG marker -> P09 / P10 / P11` 的时间步切片。P08 不是 APS、CMC、cooperative request、CUC、横向轨迹或 commit 的实现入口。

## 0. Slice Identity

- Algorithm-Step Coverage:
  - Primary Step: Step 7, longitudinal model and planning speed composition.
  - Secondary Steps:
    - 消费 Step 4A / Step 6 产生的 Eq.10 desired spacing handoff。
    - 消费 Step 4B 产生的 boundary speed cap command。
    - 为 Step 8 lateral trajectory 提供 planning speed / capped speed。
    - 为 Step 9 / Step 10 / Step 11 提供 longitudinal candidate、event、sanity、PNG marker 证据。
- MVS Acceptance Gate:
  - required:
    - `MVS-CUC-2` 的 P08 longitudinal consumption gate。
    - `MVS-CUC-3` 的 P08 non-consumption gate。
    - `MVS-SAFE-1A_waiting_cap`。
  - probe:
    - CPID memory / controller clipping diagnostic。
    - IDM numeric diagnostic。
    - front-collision conservative fallback diagnostic。
    - speed cap non-binding diagnostic。
  - deferred:
    - `MVS-SAFE-1B_executing_cap_lateral_consumption`。
    - `MVS-SAFE-2`。
    - `MVS-E2E-1`。
    - `MVS-COMMIT-1-full`。
    - P09-P12 的任何 full pass。
    - 论文级随机流量、capacity、aggregate metric。
- 本阶段解锁的能力:
  - 基于冻结 `S(t)` 和 relations 为车辆生成 longitudinal candidate 或等价 planning speed candidate。
  - CAV cruising / CAV gap-regulating / CPID 分支可观测。
  - CHV / IDM 分支可观测。
  - P07 choice 2 spacing handoff 被稳定消费，并记录 Eq.10 consumption evidence。
  - non-compliant CHV 不消费 Eq.10 override。
  - P05 boundary speed cap 被施加到 P08 planning speed。
  - base candidate speed、boundary cap、front-collision conservative fallback 合成取最保守可行速度。
- 本阶段不要求通过的后续场景:
  - 不要求 P09 正弦横向轨迹、lane-change progress、merge progress 通过。
  - 不要求 P10 E2E / full commit 通过。
  - 不要求 P11 full smoke suite、formal PNG renderer 或 artifact export。
  - 不要求 P12 随机边界生成或论文级实验入口。

当前成熟度声明:

- 本文档新增后，P08 仍按 P00 追踪矩阵保持 `trace_registered`。
- 本文档不自动把 P08 标记为 `spec_ready` 或 `implementation_ready`。
- P08 进入 red-before-green implementation 前，需要人工审阅本文档、确认 schema gap 的处理顺序，并决定是否同步修订上游数据结构 / runner 合同。

## 1. 本阶段目标

P08 的目标不是“实现一个纵向模型模块”，而是固定 Step 7 在 CORMC 单步流程中的时间步职责:

```text
freeze S(t)
    + Step 3 relations
    + P04 / P07 Eq.10 desired spacing source or cooperation command
    + P05 boundary speed cap command
    + vehicle specs / parameters / controller memory
-> Step 7 longitudinal candidate / planning speed / event / sanity / PNG marker
-> P09 lateral trajectory consumes planning speed
-> P10 / commit later decides final S(t+dt)
```

本阶段需要让以下行为从不可观测变为可测试:

1. CAV 无 leader 或 spacing 足够大时使用 cruising，生成 longitudinal candidate。
2. CAV 有 leader 且 spacing 不足时使用 gap-regulating / CPID，生成 longitudinal candidate。
3. CHV 使用 IDM，且 compliance 只影响 P07 是否接受 CUC 建议，不改变 P08 的 IDM 本质。
4. MV 未进入 merging zone 或没有 boundary cap 时，仍按 on-ramp logical longitudinal relation 生成 base longitudinal candidate。
5. P07 choice 2 输出 `CommandBuffer.cooperation_commands[cv_id]` 后，P08 才允许消费 Eq.10 desired spacing override。
6. `MVS-CUC-2` 中，P07 target lane unsafe fallback 后，CFV stay lane 2，P08 消费 Eq.10 desired spacing override，生成 longitudinal candidate / `longitudinal_model` event。
7. `MVS-CUC-3` 中，non-compliant CHV 不消费 Eq.10 override，不生成 spacing consumption event。
8. APS case 2 / 4 中，只有 CFV 能消费 Eq.10；case 3 不得给 CLV 套 Eq.10。
9. P05 的 boundary speed cap command 进入 P08 planning speed 合成。
10. `MVS-SAFE-1A_waiting_cap` 中，waiting MV 的 final planning speed 不超过 boundary speed cap。
11. P08 输出给 P09 的 planning speed 可追踪来源，不直接推进横向轨迹。
12. P08 不直接写真实 `x / y / v / a / physical_lane / road_role / lane_change_state / merge_state`。

## 2. 非目标 / 禁止事项

P08 不得实现或重做以下内容:

- 不重做 P04 APS trigger、candidate search、case classifier、assignment cache 或 Eq.10 source creation。
- 不重做 P05 CMC branch selection、assignment validation、Eq.53、boundary speed cap calculation、merge command 或 waiting command。
- 不重做 P06 cooperative request collection 或 conflict resolution。
- 不重做 P07 CUC choice、utility、target lane safety、CHV compliance、lane-change command 或 same-step overlay。
- 不自己决定 CUC choice 1 / choice 2。
- 不为 non-compliant CHV 伪造 Eq.10 spacing consumption。
- 不为 P07 未输出 spacing handoff 的车辆消费 Eq.10。
- 不给 APS case 3 的 CLV 套 Eq.10。
- 不实现 P09 sine lateral trajectory、lateral candidate、lane-change progress、merge progress 或 maneuver completion。
- 不初始化或推进 lane-change / merge 的横向位置。
- 不实现 P10 E2E integration 或 Step 9 commit。
- 不实现 P11 full smoke suite、formal PNG renderer、artifact export 或 regression report。
- 不实现 P12 random boundary generation、random attributes 或 paper-level experiment grid。
- 不直接写真实车辆 `x / y / v / a / physical_lane / road_role / lane_change_state / merge_state`。
- 不把 planning speed 直接当成真实速度提交。
- 不写 Step 9 commit result。
- 不使用 `x_plot` 做任何算法判断。
- 不把 event / sanity / PNG marker 推迟到 P11。

第一版关闭或待审阅项:

- CPID 参数来自参数规格中的 first-version default / to-review，不得写成 CORMC 论文本篇 Table I 明确给出。
- `LongitudinalControllerMemory` 当前在文档中有权威语义，但代码中尚未形成完整持久结构。P08 首轮若无法安全落地 CPID memory，必须把 memory 行为降级为 probe / diagnostic 或先修订数据结构。
- front-collision conservative fallback 当前可作为 P08 planning speed 合成中的第一版 hook / diagnostic；若缺少完整公式、参数或 command 字段，不得暗增字段伪装完整实现。
- boundary speed cap 不可行、为负或过低时，P08 只允许执行上游规格批准的保守策略；若无批准策略，应记录 warning / schema gap，不得临时自创运动规则。

## 3. 上游 spec 引用

- `docs/执行计划/CORMC执行计划spec设计总纲v1.md`
  - 引用 P08 的 Step 7 定位、required / probe / deferred gate、P04-P10 每个算法切片同步产出 event / sanity / targeted MVS / PNG marker 的规则。
  - 引用 P08 只写 longitudinal candidate / planning speed / event / sanity，不提交真实状态。
  - 引用 speed cap 先进入 P08 planning speed，再由 P09 lateral trajectory 消费。
- `docs/执行计划/P00-Spec宪法_权威边界与二维追踪矩阵.md`
  - 引用 P08 row: Step 7 longitudinal model / Eq.10 spacing override / speed cap composition。
  - 引用 required gate: `MVS-CUC-2`、`MVS-CUC-3`、`MVS-SAFE-1A_waiting_cap`。
  - 引用 event evidence: `longitudinal_model`、speed cap consumption event。
  - 引用 sanity evidence: Eq.10 only CFV、non-compliant no Eq.10、`planning_speed = min(caps)`。
  - 引用 P08 当前仍为 `trace_registered`，不得提前隐藏实现纵向模型。
- `docs/执行计划/P01-MVS_Runner_ScenarioConfig与验收断言语言.md`
  - 引用 ScenarioConfig loader、expected_events、forbidden_events、expected_event_counts、expected_sanity_checks、expected_png_features、required / probe / deferred matcher 语义。
  - 当前 runner 还没有 P08 route。P08 首轮 red tests 可以先用 Python helper 构造 P05/P07 handoff 和 Step7 fixtures；若要把 `MVS-CUC-2`、`MVS-CUC-3`、`MVS-SAFE-1A_waiting_cap` 正式接入 built-in runner，需要先修订 runner / loader 合同。
- `docs/执行计划/P02-Step0-3_清理冻结关系与几何口径.md`
  - 引用冻结 `S(t)`、relations snapshot、lane ordering、logical longitudinal role、`x_global` 几何口径。
  - P08 只读 Step 3 relations，不读本步中途产生的真实 next-state。
  - 正在换道 / 合流车辆的 longitudinal role 不得仅按 physical `y` 最近车道切换。
- `docs/执行计划/P03-Step9-10_Command_NextState_Commit_Event_Sanity_Trajectory闭环.md`
  - 引用 `CommandBuffer` / `NextStateBuffer` / `CandidateLongitudinalKinematics` / `CandidateKinematics` / `EventRecord` / `SanityCheckRecord` / no-write-before-commit 边界。
  - 当前代码已有 `CandidateLongitudinalKinematics` 和 `NextStateBuffer.candidate_longitudinal`，但 candidate assembly / commit source 白名单只允许 `identity_candidate_for_commit_infrastructure` 和 `test_harness_preloaded_candidate`。P08 若要产生正式 Step7 candidate source，必须先修订候选 source 合同或在 P08 spec 中保留为 schema gap，不得绕过 P03 guard。
- `docs/执行计划/P04-Step4A_APS_Cache_EffectiveAssignment.md`
  - 引用 APS case 2 / 4 中 Eq.10 desired spacing source 只绑定 CFV。
  - 引用 case 3 不给 CLV 套 Eq.10。
  - P08 不重算 APS，只消费经 P07 handoff 或 P04/P06 派生输入传来的 Eq.10 source。
- `docs/执行计划/P05-Step4B_CMC_AssignmentValidation_Eq53_BoundaryCap.md`
  - 引用 P05 产生 `CommandBuffer.speed_cap_commands[mv_id]`，command dict 当前包含 `command_id`、`vehicle_id`、`command_type=speed_cap`、`speed_cap`、`cap_source`、`cap_reason`、`cap_feasible`、`cap_binding`、`source`。
  - 引用 P05 可能同时产生 waiting / merge command，但 P08 只消费纵向意图和 speed cap，不重判 Eq.53。
  - `MVS-SAFE-1A_waiting_cap` 在 P05 只是 boundary speed cap command 前置证据；P08 才验 full planning speed composition。
- `docs/执行计划/P06-Step5_CooperativeRequest_ConflictResolution.md`
  - 引用 P06 只产出 active cooperative request / conflict result。
  - P08 不读取 loser / suppressed request 做纵向模型输入。
- `docs/执行计划/P07-Step6_CUCChoice_Compliance_LaneChangeCommand_SameStepOverlay.md`
  - 引用 P07 choice 2 输出 `CommandBuffer.cooperation_commands[cv_id]` 作为 P08 spacing handoff。
  - 当前 P07 spacing command dict 包含 `command_id`、`vehicle_id`、`source_request_id`、`source_mv_id`、`cv_role`、`aps_case`、`eq10_desired_spacing`、`consumed_by=P08`、`p07_longitudinal_candidate_created=False`、`cuc_decision_id`。
  - 引用 P07 不产生 longitudinal candidate，`MVS-CUC-2` 在 P07 只验 target lane unsafe fallback + spacing handoff；完整 longitudinal consumption 属于 P08。
  - 引用 P07 对 non-compliant CHV 不生成 P08 spacing command。
  - P08 不得把旧 MVS 文档中的 `target_lane_TT_unsafe` 字符串写死为 matcher 条件；P08 只依赖 P07 的 effective choice `stay_lane_2` 和 spacing handoff。P07 canonical fallback reason 为 `target_lane_unsafe`。
- `docs/复现讨论/CORMC时间步执行顺序梳理.md`
  - 引用 Step 7 中 CAV / CHV / MV 纵向行为、waiting 不是停车、boundary speed cap 由纵向阶段施加、横向阶段消费 planning speed。
- `docs/复现讨论/CORMC论文公式与实现映射.md`
  - 引用 CAV longitudinal Eq.17-Eq.27、CHV / IDM Eq.28-Eq.29、CMC boundary speed cap Eq.54-Eq.56、case 2 / 4 CFV Eq.10 desired spacing。
  - 引用 front-collision / boundary speed cap / planning speed 合成需要保守速度并记录触发原因。
- `docs/复现讨论/CORMC车辆模型规格.md`
  - 引用 CAV cruising、CAV gap-regulating / CPID、CHV / IDM、MV on-ramp longitudinal behavior、Eq.10 desired spacing override、speed cap 施加顺序、planning speed 合成。
- `docs/复现讨论/CORMC状态与模块接口规格.md`
  - 引用 Longitudinal model 输入输出边界: `S(t)`、relations、vehicle type、commands、desired spacing override、speed cap -> candidate acceleration / speed / planning result，不直接提交真实状态。
- `docs/复现讨论/CORMC代码数据结构设计_整理版.md`
  - 引用 `CommandBuffer.longitudinal_commands`、`CommandBuffer.cooperation_commands`、`CommandBuffer.speed_cap_commands`、`NextStateBuffer.candidate_longitudinal`、`CandidateLongitudinalKinematics`、`EventRecord`、`SanityCheckRecord`、`ExpectedEventSpec`、`ExpectedSanityCheckSpec`、`ExpectedPNGFeatureSpec`。
  - 若实现发现 `LongitudinalCommand`、`SpeedCapCommand`、controller memory、front fallback command 或 candidate source 字段不足，必须先修订上游数据结构规格或明确使用等价 dict payload，不得在代码里暗增。
- `docs/复现讨论/CORMC输出指标与日志验证规格_整理版.md`
  - 引用 longitudinal model event: mode、leader、desired spacing、speed cap、planning speed。
  - 引用 desired spacing override consumption、speed cap composition、most conservative source、no write-before-commit、PNG marker 的证据语义。
- `docs/复现讨论/CORMC最小验证场景执行规格.md`
  - 引用 `MVS-CUC-2`、`MVS-CUC-3`、`MVS-SAFE-1A_waiting_cap` 的场景意图。
  - 注意消歧: `MVS-CUC-2` 的 CUC fallback 属于 P07；其中 `longitudinal_model` consumption 属于 P08。
  - 注意消歧: `MVS-SAFE-1A_waiting_cap` 当前 built-in loader 中作为 P05 probe / prereq 登记；P08 spec 将其定义为 P08 required targeted gate，但不在本文档中修改 loader。

旧 P04-P07 临时蓝图已退役，不作为 P08 上游权威。P08 边界以 P00 追踪矩阵、总纲、当前 P04 / P05 / P06 / P07 完整 spec、复现讨论权威规格和当前代码事实为准。

## 4. 行为契约

### 4.1 Step 7 输入边界

- Given: 冻结 `S(t)`、Step 3 relations、VehicleSpec、road / parameter config、可选 longitudinal controller memory、P05 speed cap command、P07 cooperation command 已存在。
- When: P08 Step 7 调度。
- Then: P08 只读取这些输入，生成 longitudinal candidate / planning speed / event / sanity / PNG marker；不得修改 `S(t)`。

### 4.2 CAV cruising

- Given: 车辆为 CAV，relations 中无有效 leader，或 actual spacing 足够大，不需要跟驰调节。
- When: P08 计算纵向模型。
- Then: 选择 `longitudinal_mode=cav_cruising`，按论文 cruising 语义或第一版已批准参数计算 candidate acceleration / candidate speed / planning speed，写 `CandidateLongitudinalKinematics` 或等价 candidate，记录 `longitudinal_model` event。

### 4.3 CAV gap-regulating / CPID

- Given: 车辆为 CAV，relations 中存在有效 leader，且 spacing 需要跟驰调节。
- When: P08 计算纵向模型。
- Then: 选择 `longitudinal_mode=cav_gap_regulating` 或 `cav_cpid`，读取 / 更新 controller memory 的候选结果，计算 candidate acceleration / speed，记录 CPID diagnostic event。
- Then: 即使 controller memory schema 尚不足，P08 也必须生成最小 gap-regulating longitudinal candidate；schema gap / probe 只能描述跨步 memory 持久化不足，不得让 CAV gap-regulating candidate 缺席。

### 4.4 CHV / IDM

- Given: 车辆为 CHV。
- When: P08 计算纵向模型。
- Then: 使用 `longitudinal_mode=chv_idm`，按 IDM 语义生成 longitudinal candidate。
- Then: compliant / non-compliant 只影响 P07 是否接受 CUC 建议；P08 不因 compliance 改变 CHV 的基础模型类型。

### 4.5 MV on-ramp longitudinal behavior without boundary cap

- Given: MV 位于 on-ramp，尚未进入 merging zone 或本步没有 P05 boundary speed cap command，`merge_state` 为 `not_started` 或等价非 merging branch。
- When: P08 计算纵向模型。
- Then: P08 仍按 on-ramp logical longitudinal relation 和车辆类型生成 base longitudinal candidate。
- Then: 若没有 boundary cap 且没有 front fallback，`planning_speed` 等于 base candidate speed 或经普通纵向模型裁剪后的 candidate speed。
- Then: P08 不因此触发 APS、CMC Eq.53、merge command 或 lateral merge progress。

### 4.6 Eq.10 spacing override 正向消费

- Given: P07 对 active CV 输出了 `CommandBuffer.cooperation_commands[cv_id]`，其中 `consumed_by=P08`，`eq10_desired_spacing` 非空，`cv_role=cfv`，`aps_case` 为 `case_2` 或 `case_4`，车辆不是 non-compliant CHV。
- When: P08 计算该 CV 的纵向模型。
- Then: P08 使用该 Eq.10 desired spacing override 作为本步 desired spacing source，记录 spacing override consumption event，生成 longitudinal candidate。
- Then: `MVS-CUC-2` 的 P08 gate 必须看到 `desired_spacing_source=Eq10`、`source_command_id`、`source_mv_id`、`cv_role=cfv`、`planning_speed` 或 candidate 结果。

### 4.7 Eq.10 spacing override 禁止消费

- Given: P07 没有输出 P08 spacing command，或 spacing command 的 `eq10_desired_spacing` 为空。
- When: P08 计算纵向模型。
- Then: 不得消费 Eq.10 override，不得生成 spacing override consumption event。

- Given: 只有 suppressed / loser request、历史 CUC event 或 stale active request evidence，但没有 P07 `cooperation_commands[cv_id]` spacing handoff。
- When: P08 计算纵向模型。
- Then: 不得消费 Eq.10 override；P08 不得从 P06 loser / suppressed request 或历史 event 反推出 spacing handoff。

- Given: active CV 是 non-compliant CHV。
- When: P08 计算纵向模型。
- Then: 不得消费 Eq.10 override，使用普通 IDM，记录 non-consumption evidence；`MVS-CUC-3` 必须通过。

- Given: APS case 为 `case_3` 且协同对象为 CLV。
- When: P08 计算纵向模型。
- Then: 不得给 CLV 套 Eq.10；若出现 CLV Eq.10 consumption，`Eq10_applied_to_wrong_vehicle` sanity 必须失败。

### 4.8 P05 boundary speed cap consumption

- Given: P05 输出 `CommandBuffer.speed_cap_commands[mv_id]`，speed cap command 中 `cap_feasible=True`，`speed_cap` 为有效上界。
- When: P08 计算该 MV 的 longitudinal candidate / planning speed。
- Then: P08 将 base candidate speed、boundary speed cap、front-collision conservative fallback 组成 applicable safe speeds，取最保守可行速度作为 `planning_speed`。
- Then: P08 记录 speed cap consumption event，payload 至少能表达 `base_candidate_speed`、`boundary_speed_cap`、`front_fallback_speed` 或 not_applicable、`planning_speed`、`most_conservative_source`、`source_speed_cap_command_id`。

### 4.9 `MVS-SAFE-1A_waiting_cap`

- Given: waiting MV 位于 on-ramp downstream boundary 附近，P05 已输出 binding boundary speed cap，例如约 `2.63 m/s`。
- When: P08 计算 planning speed。
- Then: `planning_speed <= boundary_speed_cap + tolerance`，`most_conservative_source=boundary_speed_cap`，并生成 longitudinal candidate / speed cap consumption event。
- Then: speed cap consumption event 必须结构化记录 `base_candidate_speed`、`boundary_speed_cap`、`front_fallback_speed`、`planning_speed`、`most_conservative_source`。即使 front fallback 不适用，也必须显式记录 `front_fallback_speed=not_applicable` 或等价字段。
- Then: P08 不生成 lateral trajectory event；waiting 状态不推进横向合流。

### 4.10 front-collision conservative fallback

- Given: 当前 step 中 front-collision conservative fallback 可由上游 approved hook 或 P08 内部 proxy 计算。
- When: P08 合成 planning speed。
- Then: front fallback speed 与 base speed、boundary cap 一起取最保守速度，并记录 diagnostic。
- Then: 若公式 / 参数 / schema 不足，P08 首轮只能记录 not_applicable 或 schema gap，不得暗增字段宣称完整 front-collision-avoidance；但 composition event 仍必须保留 `front_fallback_speed` 字段或等价结构化位置。

### 4.11 P08 -> P09 handoff

- Given: P08 已生成 planning speed。
- When: P09 后续执行。
- Then: P09 消费 P08 planning speed 推进 lateral trajectory。
- Then: P08 不计算 `CandidateLateralKinematics`、不更新 `ManeuverTrajectoryState` progress、不写 completion candidate。

### 4.12 P08 -> P10 / commit boundary

- Given: P08 生成 longitudinal candidate。
- When: Step 9 / P10 commit later runs。
- Then: 只有 commit 才能把 candidate 变为 `S(t+dt)` 的真实 `x / v / a`。
- Then: P08 不直接更新 `SimulationState.vehicle_states`。

### 4.13 P08 不重做上游

- Given: P04 / P05 / P06 / P07 event history 或 command buffers 已存在。
- When: P08 执行。
- Then: P08 不产生 `APS`、`APS_candidate`、`CMC` Eq.53、`assignment_validation`、`cooperative_request`、`conflict_resolution`、`CUC` choice / safety / compliance event。

## 5. 允许实现的代码对象

后续 P08 implementation 允许新增或修改的代码对象必须服务 Step 7 时间步切片，不得越界实现 P09-P12。

### 5.1 domain / state objects

- 可新增 `LongitudinalControllerMemory` 的最小代码结构，前提是先确认或修订 `CORMC代码数据结构设计_整理版.md`。
- 可新增 P08 本步派生对象，例如:
  - `LongitudinalModelMode`
  - `DesiredSpacingSource`
  - `SpacingOverrideConsumption`
  - `PlanningSpeedComposition`
  - `Step7LongitudinalRunResult`
- 若这些对象需要成为跨模块正式 schema，必须先更新数据结构规格；否则只能作为 P08 内部 helper / derived result。

### 5.2 command / next-state objects

- 复用 `CommandBuffer.longitudinal_commands` 读取 P05 waiting / on-ramp longitudinal intent。
- 复用 `CommandBuffer.cooperation_commands` 读取 P07 spacing override command。
- 复用 `CommandBuffer.speed_cap_commands` 读取 P05 speed cap command。
- 复用 `CandidateLongitudinalKinematics` 写 P08 longitudinal candidate。
- 复用 `NextStateBuffer.candidate_longitudinal` 承载 P08 output。
- P08 如要输出 assembled `CandidateKinematics` 给 commit，必须先解决 P03 allowed candidate source gap。

### 5.3 step runner / service functions

建议新增:

- `cormc/step7_longitudinal.py`
- `run_step7_longitudinal_model_spacing_speedcap(...)`
- `run_step7_longitudinal_model_for_scenario(...)` 或测试 helper，是否接入 MVS runner 由 P08 implementation 阶段决定。
- `select_longitudinal_mode(...)`
- `compute_cav_cruising_candidate(...)`
- `compute_cav_gap_regulating_candidate(...)`
- `compute_chv_idm_candidate(...)`
- `consume_spacing_override_command(...)`
- `compose_planning_speed(...)`
- `build_longitudinal_candidate(...)`

### 5.4 event / sanity helpers

建议新增:

- `emit_longitudinal_model_event`
- `emit_spacing_override_consumption_event`
- `emit_speed_cap_consumption_event`
- `emit_front_fallback_composition_event`
- `run_p08_eq10_consumption_sanity`
- `run_p08_speed_cap_consumption_sanity`
- `run_p08_no_write_before_commit_sanity`
- `run_p08_no_lateral_candidate_sanity`

Event / sanity 类型优先复用当前 matcher 可表达的 string:

- event_type: `longitudinal_model`
- event_type: `speed_cap`
- event_type: `spacing_override_consumption` 或在 `longitudinal_model` payload 中表达
- sanity check: `Eq10_applied_to_wrong_vehicle`
- sanity check: `no_write_before_commit`
- sanity check: `x_plot_used_in_algorithm_path`
- sanity check: `state_machine_inconsistency`
- sanity check: `boundary_violation`

若要新增 canonical enum，必须先修订数据结构规格和 loader / matcher 合同。

### 5.5 scenario tests

P08 首轮不应依赖尚未登记的 built-in `MVS-CUC-2` / `MVS-CUC-3` route。可以先用 Python helper 构造:

- frozen `SimulationState`
- `RelationsSnapshot`
- P05 `CommandBuffer.speed_cap_commands`
- P07 `CommandBuffer.cooperation_commands`
- optional controller memory

然后直接调用 P08 runner。后续再将这些 fixtures 登记进 MVS runner / built-in scenario。

### 5.6 regression tests

- P00 static traceability。
- P01 runner / matcher baseline。
- P02 freeze / relations baseline。
- P03 commit / no-write-before-commit baseline。
- P04 APS case 2 / 4 / case 3 Eq.10 source baseline。
- P05 boundary cap command baseline。
- P06 active request / conflict baseline。
- P07 spacing handoff / non-compliant no handoff baseline。

## 6. 先写失败测试

本次只写 P08 spec，不新增实际 P08 测试文件。后续执行 P08 implementation 时，必须先写 red tests，再实现最小 Step 7。

### 6.1 Red-before-green 顺序

1. 新增 P08 failing tests / test skeleton。
2. 运行 P08 targeted tests，确认红灯发生在 expected event / sanity / longitudinal candidate / planning speed / PNG matcher 层。
3. 红灯不得是 loader error、unknown enum、unknown field、ImportError、AttributeError 或自然语言断言。
4. 同轮实现最小 P08 Step 7 longitudinal model / Eq.10 consumption / speed cap composition。
5. 运行 P08 targeted green tests。
6. 运行 P00-P07 回归。
7. 返回 red-before-green 证据，包括红灯失败原因和绿灯事件 / sanity / candidate 样例。

### 6.2 Required targeted tests

- `test_p08_mvs_cuc_2_consumes_eq10_after_p07_unsafe_fallback`
  - 构造 P07 unsafe fallback 后的 `cooperation_commands["CFV_X"]`。
  - 断言 P08 消费 `eq10_desired_spacing=58.0`。
  - 断言生成 `CandidateLongitudinalKinematics`。
  - 断言生成 `longitudinal_model` / spacing consumption event。
  - 断言没有 lane-change command 或 lateral candidate。

- `test_p08_mvs_cuc_3_non_compliant_chv_no_eq10_consumption`
  - 构造 non-compliant CHV。
  - 断言 P07 不提供 spacing command，或 P08 即使看到非法 command 也拒绝消费。
  - 断言使用 `longitudinal_mode=chv_idm`。
  - 断言没有 spacing override consumption event。

- `test_p08_mvs_safe_1a_waiting_cap_composes_planning_speed`
  - 构造 P05 binding boundary speed cap command。
  - 断言 final `planning_speed <= speed_cap`。
  - 断言 `most_conservative_source=boundary_speed_cap`。
  - 断言 speed cap composition event 同时记录 `base_candidate_speed`、`boundary_speed_cap`、`front_fallback_speed`、`planning_speed`。
  - 若 front fallback 不适用，断言 `front_fallback_speed=not_applicable` 或等价结构化值存在。
  - 断言 P08 不生成 lateral trajectory。

### 6.3 Unit / integration tests

- `test_p08_cav_cruising_without_leader_or_large_spacing`
  - 断言 CAV 使用 cruising，生成 longitudinal candidate 和 event。

- `test_p08_cav_gap_regulating_with_leader_and_small_spacing`
  - 断言 CAV 使用 gap-regulating / CPID，生成 longitudinal candidate。
  - 若 memory 未实现，仍必须生成最小 gap-regulating candidate；只允许对跨步 memory persistence 断言 schema gap / probe diagnostic。

- `test_p08_mv_on_ramp_longitudinal_candidate_without_boundary_cap`
  - 构造 on-ramp MV，未进入 merging zone 或本步没有 P05 speed cap command。
  - 断言 P08 生成 on-ramp base longitudinal candidate。
  - 断言没有 boundary cap consumption event。
  - 断言没有 CMC Eq.53 / merge command / lateral progress。

- `test_p08_chv_uses_idm_and_compliance_does_not_change_model`
  - compliant CHV 和 non-compliant CHV 均使用 IDM。
  - compliance 只影响 Eq.10 consumption permission。

- `test_p08_case_2_and_case_4_cfv_only_eq10_consumption`
  - 断言 `aps_case in {case_2, case_4}` 且 `cv_role=cfv` 才允许 Eq.10 consumption。

- `test_p08_case_3_clv_no_eq10_consumption`
  - 断言 case 3 CLV 不消费 Eq.10。

- `test_p08_speed_cap_front_fallback_base_speed_min_composition`
  - 断言 base speed、boundary cap、front fallback 中取最保守可行速度。
  - 若 front fallback schema 不足，断言 `front_fallback_speed=not_applicable` 或 schema gap 以结构化字段出现，而不是省略 composition 证据。

- `test_p08_does_not_rerun_aps_cmc_p06_or_p07`
  - forbidden events: `APS`、`APS_candidate`、`CMC` Eq.53、`assignment_validation`、`cooperative_request`、`conflict_resolution`、`CUC` choice / safety / compliance。

- `test_p08_ignores_suppressed_or_historical_request_without_p07_spacing_command`
  - 构造 suppressed / loser request 或历史 CUC event，但不提供 P07 `cooperation_commands[cv_id]`。
  - 断言 P08 不消费 Eq.10。
  - 断言没有 spacing override consumption event。

- `test_p08_does_not_create_lateral_candidates_or_maneuver_progress`
  - 断言 `NextStateBuffer.candidate_lateral == {}`。
  - 断言无 `CandidateManeuverProgress`。

- `test_p08_no_write_before_commit`
  - 深拷贝或冻结 `S(t)`，P08 后逐字段比较真实车辆状态未变。
  - 断言 planning speed 只在 candidate / event 中，不反写真实 `VehicleState.v`。

- `test_p08_planning_speed_handoff_for_p09_without_lateral_progress`
  - 断言 P08 output 能被 P09 后续读取。
  - 断言 P08 不推进横向轨迹。

### 6.4 Static / matcher tests

- P08 expected_events 缺失时，失败必须为 `missing_event` / `event_mismatch`。
- P08 expected_sanity_checks 缺失时，失败必须为 `missing_sanity_check` / `sanity_check_mismatch`。
- P08 expected_png_features 必须可注册为 renderer deferred，不要求真实 PNG。
- 如果正式接入 MVS runner:
  - `_is_p08_longitudinal_scenario(...)` route 或等价机制必须明确。
  - `MVS-SAFE-1A_waiting_cap` 的 P05 prereq 与 P08 required gate 不得相互覆盖。

## 7. 验收证据

未来 P08 implementation 完成后，不能只返回 pytest 数字，必须返回以下证据样例或报告摘录。

### 7.1 Required green evidence

- `MVS-CUC-2` P08 longitudinal consumption targeted green evidence:
  - P07 fallback = `stay_lane_2`。
  - P07 spacing command `consumed_by=P08`。
  - P08 不依赖 `target_lane_TT_unsafe` / `target_lane_unsafe` 字符串差异作为 matcher 条件。
  - P08 event `desired_spacing_source=Eq10`。
  - P08 candidate `vehicle_id=CFV_X`。
  - no lateral candidate。

- `MVS-CUC-3` P08 non-consumption targeted green evidence:
  - non-compliant CHV uses ordinary IDM。
  - no spacing override consumption event。
  - no Eq.10 applied to wrong vehicle sanity failure。

- `MVS-SAFE-1A_waiting_cap` targeted green evidence:
  - P05 speed cap command id。
  - P08 speed cap consumption event。
  - event payload includes `base_candidate_speed`、`boundary_speed_cap`、`front_fallback_speed`、`planning_speed`、`most_conservative_source`。
  - if front fallback is unavailable, `front_fallback_speed=not_applicable` or equivalent structured value is present。
  - `planning_speed <= boundary_speed_cap + tolerance`。
  - no lateral trajectory event。

### 7.2 Event samples

Longitudinal model event sample:

```text
event_type = longitudinal_model
module = Step7LongitudinalModel
vehicle_id = CFV_X
reason = cav_gap_regulating
payload:
    longitudinal_mode = cav_gap_regulating
    leader_id = CLV_Y
    desired_spacing_source = Eq10
    desired_spacing_override = 58.0
    base_candidate_speed = ...
    planning_speed = ...
    source_spacing_command_id = p07:0:CFV_X:spacing_override
```

CHV IDM event sample:

```text
event_type = longitudinal_model
module = Step7LongitudinalModel
vehicle_id = CFV_X
reason = chv_idm
payload:
    vehicle_type = CHV
    compliance_state = non_compliant
    spacing_override_consumed = false
    desired_spacing_source = ordinary_idm
```

Speed cap composition event sample:

```text
event_type = speed_cap
module = Step7SpeedCapComposition
vehicle_id = MV_SAFE_1A
reason = speed_cap_consumed
payload:
    base_candidate_speed = 16.0
    boundary_speed_cap = 2.63
    front_fallback_speed = not_applicable
    planning_speed = 2.63
    most_conservative_source = boundary_speed_cap
    source_speed_cap_command_id = p05:0:speed_cap:MV_SAFE_1A
```

### 7.3 Candidate samples

Longitudinal candidate sample:

```text
CandidateLongitudinalKinematics(
    candidate_id = p08:0:CFV_X:longitudinal
    vehicle_id = CFV_X
    x_global = ...
    v = ...
    a = ...
    candidate_speed = ...
    planning_speed = ...
    source = step7_longitudinal_model
    constraints_applied = (eq10_spacing_override, ...)
    source_commands = (p07:0:CFV_X:spacing_override, ...)
)
```

If `source=step7_longitudinal_model` is not yet allowed by P03 candidate source guards, implementation must first revise the candidate source contract or keep this as a documented schema gap. It must not bypass P03 by using `test_harness_preloaded_candidate` for real P08 output.

### 7.4 Sanity samples

- `Eq10_applied_to_wrong_vehicle`: pass for case 2 / 4 CFV and case 3 CLV no consumption。
- `no_write_before_commit`: pass for every P08 targeted fixture。
- `x_plot_used_in_algorithm_path`: pass。
- `boundary_violation`: pass / warning according to speed cap feasibility。
- `state_machine_inconsistency`: pass, P08 must not change lane-change / merge state。

### 7.5 PNG marker samples

- `longitudinal_candidate_marker`: visible for vehicles with P08 candidate。
- `eq10_spacing_consumption_marker`: visible for `MVS-CUC-2` CFV。
- `speed_cap_consumption_marker`: visible for `MVS-SAFE-1A_waiting_cap` MV。
- `planning_speed_marker`: visible / optional depending on renderer readiness。
- `no_lateral_progress_marker`: not visible for P08-only waiting cap case。

### 7.6 Negative evidence

P08 completion report must show:

- no APS event produced by P08。
- no CMC Eq.53 / boundary cap calculation event produced by P08。
- no cooperative_request / conflict_resolution event produced by P08。
- no CUC choice / safety / compliance event produced by P08。
- no lateral_trajectory event produced by P08。
- no `CandidateLateralKinematics` / `CandidateManeuverProgress` produced by P08。
- no direct mutation of `SimulationState.vehicle_states`。

## 8. 完成标准

P08 只有同时满足以下条件，才能声称完成:

- `MVS-CUC-2` 的 P08 longitudinal consumption gate 通过。
- `MVS-CUC-3` 的 P08 non-consumption gate 通过。
- `MVS-SAFE-1A_waiting_cap` 通过。
- CAV cruising 可观测，并产生 longitudinal candidate。
- CAV gap-regulating / CPID 可观测，并产生最小 longitudinal candidate；schema gap / probe 只能用于 CPID cross-step memory persistence，不得让 gap-regulating candidate 缺席。
- on-ramp MV 在无 boundary cap 的普通纵向场景中可观测，并产生 base longitudinal candidate。
- CHV IDM 可观测，并产生 longitudinal candidate。
- P07 choice 2 spacing handoff 能被 P08 稳定消费。
- non-compliant CHV 不消费 Eq.10。
- case 2 / 4 的 CFV 才消费 Eq.10。
- case 3 不给 CLV 套 Eq.10。
- P05 boundary speed cap 被消费并进入 planning speed。
- speed cap、front fallback、base candidate speed 合成取最保守可行速度。
- P08 只输出 longitudinal candidate / planning speed / event / sanity / PNG marker。
- P08 不重做 APS。
- P08 不重做 CMC 或 boundary cap calculation。
- P08 不重做 P06 cooperative request / conflict。
- P08 不重做 P07 CUC。
- P08 不实现 P09 横向轨迹。
- P08 不直接写真实车辆状态。
- event / sanity / PNG marker 不等 P11 才补。
- required / probe / deferred 语义仍由 P01 runner / matcher 保持。
- P00-P07 回归通过，或任何失败被明确证明为用户已有改动 / 上游未解决 gap，不得忽略。

## 9. 回归保护

后续阶段不得破坏以下 invariant:

- 所有算法内部使用 `x_global`；`x_plot` 只用于 PNG / renderer 派生层。
- 所有 P08 输入只读冻结 `S(t)` 和 Step 3 relations。
- P08 不修改真实 vehicle state。
- P08 不修改 assignment cache。
- P08 不修改 lane-change / merge state。
- P08 不创建 P09 lateral candidate。
- P08 不把 planning speed 反写成真实 speed。
- P08 只消费 P05 已计算的 speed cap，不计算 Eq.54-Eq.56 boundary cap。
- P08 只消费 P07 已输出的 spacing handoff，不重新执行 CUC。
- P08 不从 P06 loser / suppressed request、历史 CUC event 或 stale evidence 反推出 spacing handoff。
- Eq.10 override 只允许 case 2 / 4 CFV 且 P07 / active request 明确 handoff 时消费。
- non-compliant CHV 不消费 Eq.10。
- CHV compliance 不改变 IDM 模型本质。
- CAV gap-regulating / CPID memory 不得用 EventRecord 替代。
- P08 candidate source 必须通过 P03 / P10 批准，不得借用 test harness source 表示真实模型输出。
- P08 event / sanity / PNG marker 必须在本阶段产生，不得等待 P11 补齐。
- `MVS-SAFE-1A_waiting_cap` 的 P05 prereq 和 P08 required gate 必须在报告中区分:
  - P05 证明 speed cap command 已产生。
  - P08 证明 speed cap 被消费并形成 planning speed。

## 10. 当前代码 / schema gap 清单

这些 gap 是进入 P08 implementation 前必须处理或显式降级的事项:

1. `cormc/step7_longitudinal.py` 尚不存在。
2. MVS runner 当前没有 P08 route。
3. `MVS-CUC-2` / `MVS-CUC-3` required built-in scenario 当前未正式接入 runner；P07 仍使用 Python helper targeted tests。
4. `MVS-SAFE-1A_waiting_cap` 当前 built-in loader 中作为 P05 prereq / probe 表达；P08 implementation 需要将 P08 gate 与 P05 prereq 清晰拆开。
5. `ScenarioConfig` 当前没有 preloaded command buffer 字段；P08 red tests 若要直接输入 P05/P07 command，应使用 Python helper，或先修订 loader 合同。
6. `CandidateLongitudinalKinematics` 已存在，但 P03 candidate assembly / commit source guard 当前只允许 identity / test harness source。P08 真实 model source 需要规格和代码共同批准。
7. `LongitudinalControllerMemory` 在数据结构文档中有语义，但当前代码未见完整实现。CPID memory 不能用 event 或临时局部变量伪装跨步记忆。
8. front-collision conservative fallback 的完整公式、参数和 command schema 可能不足；首轮可作为 diagnostic / hook，但不得宣称论文级完整复现。
9. `CommandBuffer.cuc_decisions` 在代码中存在，但 P07 spec 不把它作为正式承载点。P08 不应依赖该字段获取 CUC decision，只应消费 P07 cooperation / lane-change command 和 event payload 可追踪 id。
10. `CommandBuffer.longitudinal_commands`、`cooperation_commands`、`speed_cap_commands` 当前以 dict payload 承载。若 P08 需要 typed dataclass，应先修订数据结构规格。

## 11. Implementation Entry Checklist

本节是下一轮 P08 red-before-green implementation 的硬前置，不是建议清单。下一轮 Codex prompt 必须显式复制或等价回答本节的每个决策；若缺失，implementation agent 应先停止并补齐决策，再写测试或代码。

默认进入策略如下；除非人工审阅明确改写，否则下一轮按这些默认值执行:

- runner 路线: 首轮使用 Python helper targeted tests 直接构造 P05/P07 handoff 和 Step7 fixtures，不先改 MVS runner / loader。
- candidate source guard: 首轮可以先验证 `NextStateBuffer.candidate_longitudinal`；若要 assemble / commit `CandidateKinematics`，必须先修订 P03 allowed candidate source，批准 `step7_longitudinal_model` 或等价 source。
- CPID memory: 首轮必须实现最小 gap-regulating candidate；cross-step CPID memory persistence 可以作为 probe / schema gap，但不能跳过 candidate。
- front fallback: 首轮若缺少 approved formula / schema，记录 `front_fallback_speed=not_applicable` 或等价结构化值；planning speed composition event 仍必须完整保留 base speed、boundary cap、front fallback、planning speed、most conservative source。
- P05/P07 payload: 首轮消费当前 dict payload，不新增 typed dataclass；如需 typed schema，先修订数据结构规格。

进入 P08 red-before-green implementation 前，执行者还必须确认:

- 已阅读本文档和所有上游 spec。
- 已确认旧临时蓝图不再作为 P08 权威。
- 已决定 P08 首轮是使用 Python helper targeted tests，还是先修订 MVS runner / loader。
- 已决定如何处理 P03 candidate source guard。
- 已决定 CPID memory 首轮是实现、probe 还是 schema gap。
- 已决定 front-collision conservative fallback 首轮是实现、probe 还是 schema gap。
- 已确认 P05/P07 当前 command payload 可以覆盖 P08 required gate 的输入。
- 已准备好在完成后返回 event / sanity / candidate / PNG marker 样例，而不仅是 pytest 数字。
