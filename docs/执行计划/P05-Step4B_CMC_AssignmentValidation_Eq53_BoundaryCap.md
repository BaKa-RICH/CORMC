# P05 - Step 4B CMC / Assignment Validation / Eq.53 / Boundary Cap

> 本文档是 P05 的完整执行计划 spec。它只定义后续实现计划与验收合同；本轮不实现 P05 CMC 代码，不新增 P05 实际测试文件，不修改业务算法。
>
> 后续执行 P05 时应在同一轮中完成 red-before-green：先新增 P05 failing tests / test skeleton 并确认红灯停在 expected event / sanity / matcher 层，再实现最小 CMC，使 P05 required MVS targeted gate 转绿。

## 0. Slice Identity

- Algorithm-Step Coverage:
  - Primary Step: Step 4B，处理 on-ramp MV 已进入 merging zone 后的 CMC 分支。
  - Secondary Steps:
    - CMC branch resolver。
    - MV region / `merge_state` 判断。
    - P04 `EffectiveAssignmentThisStep` / APS cache 消费。
    - assignment validity checker。
    - assignment invalid 处理。
    - Eq.52 dynamic acceptable gap 计算。
    - Eq.53 gap checker。
    - boundary speed cap calculator / command source。
    - waiting command。
    - merge-start command / merge state transition request。
    - executing merge continuation command / event。
    - assignment invalid event / sanity。
    - CMC decision event。
    - Eq.53 pass / fail event。
    - boundary cap command event。
    - P05 expected_png_features / marker registration。
- MVS Acceptance Gate:
  - required:
    - `MVS-CMC-1`
    - `MVS-CMC-2`
    - `MVS-ASSIGN-1`
  - prerequisite capability for later required gate:
    - 为 `MVS-SAFE-1A_waiting_cap` 提供可被 P08 消费的 boundary speed cap command / event / sanity 前置证据。
  - probe:
    - CMC 数值诊断可观测，例如 boundary cap binding / non-binding、Eq.52 h_tilde 数值、Eq.53 两侧 gap 数值。
    - boundary cap 不可行、为负或过低时的事件与 sanity 可观测，但完整保守策略不在 P05 强验收。
  - deferred:
    - CMC platoon。
    - front-collision-avoidance 与 boundary speed cap 的最终合成。
    - 正弦横向轨迹推进。
    - full E2E smoke suite。
    - 正式 PNG renderer 与 artifact export。
- 本阶段解锁的能力:
  - Step 4B 在 MV 已进入 merging zone 时能够消费 P04 assignment / cache，验证 assignment，计算 Eq.52 / Eq.53，产出 waiting 或 merge-start command。
  - `merge_state == executing` 时能够继续 CMC 合流，不重新判断是否开始合流，并刷新或沿用 boundary speed cap command。
  - CMC 产生的 boundary speed cap 只作为 command / speed constraint 输出，供 P08 纵向模型消费，不直接改 MV 真实速度。
  - P05 required MVS 的 event、sanity、PNG marker 数据在本阶段同步产生，不等待 P11。
- 本阶段不要求通过的后续场景:
  - 不要求 `MVS-SAFE-1A_waiting_cap` 在 P05 单独 full pass；P05 只提供其 speed cap command / event / sanity 前置能力，最终 planning speed 合成由 P08 验收。
  - 不要求 `MVS-SAFE-1B_executing_cap_lateral_consumption`、`MVS-SAFE-2`、`MVS-E2E-1`、`MVS-COMMIT-1-full` 通过。
  - 不要求 `MVS-CUC-*`、`MVS-CONFLICT-*`、P08 / P09 / P10 / P11 / P12 相关 gate 通过。

## 1. 本阶段目标

P05 聚焦一次 `S(t) -> Step 4B CMC command / event / sanity -> commit boundary` 的算法切片：当 on-ramp MV 已进入 merging zone，或已经处于 `merge_state == executing`，Step 4 不再由 P04 运行 APS，而由 P05 承接 CMC waiting / executing 分支。

P05 覆盖 CORMC 时间步主循环中的以下分支：

```text
for each MV:

    if merge_state == executing:
        继续 CMC 合流轨迹
        使用 CMC 已计算或刷新的 boundary speed cap
        写入 MV 的 merge command
        continue

    if MV 不在 merging zone:
        hand off to P04 Step4A APS
        continue

    if MV 已在 merging zone and merge_state != executing:
        进入 CMC
        计算 dynamic acceptable gap
        验证 APS assignment 中 CLV / CFV 是否仍有效
        计算或刷新 boundary speed cap

        if assignment invalid:
            本步不开始合流
            写入 waiting / on-ramp longitudinal command
        else if Eq.53 gap 满足:
            写入 merge-start command
            写入 merge_state executing transition request
        else:
            不横向合流
            写入 waiting / on-ramp longitudinal command
```

本阶段让以下 MVS 场景从 P05 缺失行为变为 targeted 可通过：

- `MVS-CMC-1`：assignment valid 且 Eq.53 pass，开始合流，产生 merge-start command / transition request。
- `MVS-CMC-2`：assignment valid 但 Eq.53 fail，继续 waiting，不产生横向合流 command。
- `MVS-ASSIGN-1`：assignment invalid 时不偷换 actual leader / follower，不伪造新 CLV / CFV，不开始合流。

P05 为后续阶段提供以下稳定输入：

- 给 P08 的 `SpeedCapCommand` 或等价 speed constraint。
- 给 P09 的 `MergeCommand` 或等价 merge trajectory start / continuation command。
- 给 P10 / P11 的 CMC decision、assignment validation、Eq.53、boundary cap、waiting / merge command、executing continuation event chain。
- 给 targeted MVS matcher 的 expected_events、expected_sanity_checks、expected_png_features 数据。

P05 不提交真实车辆状态。它只能写 command、speed cap command、merge command、state transition request、cache cleanup / invalidate request、event candidate、sanity check、PNG marker registration 或本步派生结果。真实 `x / y / v / a / lane / merge_state` 只能由 P03 / P09 / P10 约束下的 commit 阶段写入 `S(t+dt)`。

## 2. 非目标 / 禁止事项

- 不实现 P04 APS。
- 不重跑 APS。
- 不在 assignment invalid 时静默选择新的 CLV / CFV。
- 不用本步 actual lane 2 leader / follower 替代 P04 assignment。
- 不把 `assignment invalid`、`immediate APS refresh`、invalid assignment 后等待 / conservative handling、boundary speed cap 不可行时的保守处理入口、executing continuation 记录约束写成论文原公式。
- 不实现 P06 cooperative request conflict resolution。
- 不实现 P07 CUC utility / compliance / lane-change command。
- 不实现 P08 纵向模型、planning speed 合成、Eq.10 消费。
- 不实现 P09 正弦横向轨迹推进、front-collision fallback、merge progress。
- 不实现 P10 E2E integration。
- 不实现 P11 full smoke suite、正式 PNG renderer、artifact export、regression report。
- 不实现 P12 随机边界车辆生成或论文级实验入口。
- 不实现 CMC platoon。
- 不直接写真实车辆 `x / y / v / a`。
- 不直接写真实 `physical_lane`。
- 不直接写真实 `merge_state`。
- 不写 Step 9 commit result。
- 不写横向轨迹 next-state。
- 不写纵向模型 candidate。
- 不重新定义道路几何、merging zone、on-ramp downstream boundary、lane centerline 或参数值。
- 不使用 `x_plot` 参与 CMC branch、assignment validation、Eq.52、Eq.53、boundary cap 或 command 生成。
- 不新增核心字段；若发现字段、enum、ScenarioConfig、EventRecord、SanityCheckRecord 或 expected_* 口径缺失，必须先修订 `docs/复现讨论/CORMC代码数据结构设计_整理版.md` 或对应上游规格，不得在代码中暗增。
- 不新增细粒度 `EventType`，例如 `eq53_gap`、`boundary_cap`、`merge_start`、`executing_continuation`。Eq.52 / Eq.53 / waiting / merge-start / boundary cap / executing continuation 均使用既有 `EventType.cmc`，细节放入 payload、`ExpectedEventSpec.match`、`numeric_expectations`、`reason_code`。
- 不新增细粒度 `SanityCheckType`，例如 `boundary_cap_infeasible`、`p05_no_write_before_commit`、`no_same_step_actual_leader_follower_replacement`，除非先修订权威数据结构规格。P05 可用 helper 函数组织 sanity 逻辑，但落到 `ExpectedSanityCheckSpec.check_type` 时必须使用既有类型或先完成规格修订。

## 3. 上游 Spec 引用

- `docs/执行计划/CORMC执行计划spec设计总纲v1.md`
  - 引用“时间步流程切片 + MVS 验收门禁 + 可追溯证据链”的执行计划组织原则。
  - 引用 P04-P10 每个算法切片必须同步产出 event、sanity、targeted MVS 验收断言的要求。
  - 引用所有 Pxx 围绕一次或一段 `S(t) -> S(t+dt)` 展开的模板约束。
- `docs/执行计划/P00-Spec宪法_权威边界与二维追踪矩阵.md`
  - 引用 P05 trace row：Step 4B CMC / assignment validation / Eq.53 / boundary cap command。
  - 引用工程补丁必须携带 `source`、`reason`、`is_engineering_patch` 的规则。
  - 引用 P05 仍为 `trace_registered` 的当前成熟度；本文档不把 P05 标记为 implementation-ready，后续仍需人工审阅后再进入实现。
- `docs/执行计划/P01-MVS_Runner_ScenarioConfig与验收断言语言.md`
  - 引用 ScenarioConfig loader、expected_events、forbidden_events、expected_event_counts、expected_sanity_checks、expected_png_features、required / probe / deferred report 的 matcher 口径。
  - P05 后续 failing tests 必须失败在 matcher 层 missing / mismatch event 或 sanity，不得失败在 loader、unknown field、unknown enum 或自然语言断言。
- `docs/执行计划/P02-Step0-3_清理冻结关系与几何口径.md`
  - 引用冻结 `S(t)`、relations snapshot、region resolver、lane ordering by `x_global`。
  - P05 只读 P02 产出的冻结状态和 relations，不读中途 next-state。
- `docs/执行计划/P03-Step9-10_Command_NextState_Commit_Event_Sanity_Trajectory闭环.md`
  - 引用 `CommandBuffer` / `NextStateBuffer` / `EventRecord` / `SanityCheckRecord` / `OutputHistory` 边界。
  - P05 不得直接提交车辆真实状态；它只写 command / event / sanity。
- `docs/执行计划/P04-P07_总切片蓝图_dependency_sketch.md`
  - 引用 P05 消费 P04 effective assignment / cache 的依赖。
  - 该蓝图中若只写 P05 处理 `merge_state != executing`，P05 本文档按时间步总纲修正为 waiting / executing 分支全覆盖：`merge_state == executing` 必须继续 CMC merge command / boundary cap command，并记录 continuation event。
- `docs/执行计划/P04-Step4A_APS_Cache_EffectiveAssignment.md`
  - 引用 P04 已提供 APS assignment / failure / cache action / `EffectiveAssignmentThisStep`。
  - P05 只能消费这些 assignment，不得重算 APS，不得偷换 leader / follower。
- `docs/复现讨论/CORMC时间步执行顺序梳理.md`
  - 引用 Step 4 分流：MV 未进入 merging zone 执行 APS；MV 已进入 merging zone 执行 CMC。
  - 引用 `merge_state == executing` 后继续合流轨迹，不重新判断是否开始合流。
  - 引用所有模块只读冻结 `S(t)`、只写 command / next-state、最后统一 commit。
- `docs/复现讨论/CORMC论文公式与实现映射.md`
  - 引用 CMC Eq.52 dynamic time gap acceptance。
  - 引用 CMC Eq.53 merging gap constraints。
  - 引用 Eq.54-Eq.56 boundary-collision-avoidance 与 speed cap。
  - 引用 assignment validity 是第一版工程补丁或实现约束，不得写成论文原公式。
- `docs/复现讨论/CORMC车辆模型规格.md`
  - 引用 waiting 不是停车，waiting / executing MV 仍沿 on-ramp 纵向语义运动，并额外叠加 boundary speed cap。
  - 引用 Eq.53 actual spacing 口径：中心点纵向坐标并扣减前车长度。
  - 引用 `merge_state == executing` 后不撤销、不重新判断、不退回 waiting。
  - 引用 boundary speed cap 由 CMC 产生，P08 纵向阶段施加，P09 横向轨迹消费最终 planning speed。
- `docs/复现讨论/CORMC状态与模块接口规格.md`
  - 引用 APS assignment cache 生命周期、assignment invalid policy、`effective_assignment_this_step`、CMC state / merge state 语义。
  - 引用 `merge_state = none / not_started / waiting / executing / merged`。
  - 引用 boundary speed cap、assignment valid、merge command 的接口关系。
- `docs/复现讨论/CORMC代码数据结构设计_整理版.md`
  - 引用 enum、dataclass、buffer、record、ScenarioConfig、expected_* 字段权威。
  - 当前该文档已登记 P05 所需概念：`AssignmentInvalidReason`、`BoundaryCapReason`、`CMCDecision`、`CommandBuffer.merge_commands`、`CommandBuffer.speed_cap_commands`、`MergeCommand`、`SpeedCapCommand`、`StateTransitionCommand`、`CacheUpdateCommand`、`EventRecord`、`SanityCheckRecord`、`ExpectedEventSpec`、`ExpectedSanityCheckSpec`、`ExpectedPNGFeatureSpec`。
  - 本阶段引用该文档的 schema / enum / buffer / record 定义作为字段权威；其中 §10.2 的旧阶段拆分表若把 `APSAssignment`、`EffectiveAssignmentThisStep`、APS event 写到 P05，并把 `CMCDecision`、`MergeCommand`、`SpeedCapCommand` 写到 P07，则与当前阶段边界冲突。该拆分表在 P05 中视为过期的执行阶段建议；以当前 P04 implementation-ready 事实、P04-P07 蓝图中 P05 输入输出依赖、以及本文 P05 Step4B 行为契约为准。P05 不得因 §10.2 将 CMC command 延后到 P07。
  - 若后续实现发现 typed `CMCDecision` / command dataclass、scenario built-in、reason code 或 expected matcher 字段不足，必须先修订本上游文档或 loader 合同，不能在代码里暗增。
- `docs/复现讨论/CORMC输出指标与日志验证规格_整理版.md`
  - 引用 Step 4 必须记录 MV 分支判断、CMC assignment validation、CMC gap decision、boundary speed cap、merge transition。
  - 引用 assignment invalid、boundary cap、Eq.53 pass/fail、waiting / merge command、executing continuation 等 event / sanity / PNG feature 证据。
  - 引用日志和 sanity 不得反向改变车辆运动；`x_plot` 只能用于 PNG / renderer 派生层。
- `docs/复现讨论/CORMC道路几何与区域规格.md`
  - 引用 `x_global` 为算法内部唯一纵向坐标。
  - 引用 `x0_m_global = 6950 m`、`L_merging = 300 m`、`x_ramp_end_global = 7250 m`。
  - 引用 merging zone 判定：`x < x0_m_global` 走 APS；进入 `[x0_m_global, x_ramp_end_global]` 走 CMC；越过 ramp end 需记录 boundary risk / violation 或保守处理。
- `docs/复现讨论/CORMC参数规格.md`
  - 引用 CMC 参数：`cmc_upper_merge_time_gap = 1.2 s`、`cmc_dynamic_gap_xi = 2/3`。
  - 引用 boundary cap 相关参数：`vehicle_length = 4 m`、`standstill_spacing = 2 m`、`lane_change_planned_acceleration = 0.1 m/s^2`、`lane_width = 3.5 m`。
  - 引用工程策略不写入 `ParameterConfig`，应属于 `ControlPolicyConfig` 或 event / policy trace。
- `docs/复现讨论/CORMC最小验证场景执行规格.md`
  - 引用 P05 required / related MVS：`MVS-CMC-1`、`MVS-CMC-2`、`MVS-ASSIGN-1`、`MVS-SAFE-1A_waiting_cap`。
  - 引用每个场景的 setup、key numeric derivation、expected_events、expected_sanity_checks、expected_png_features。

## 4. 行为契约 Given / When / Then

- Given：冻结 `S(t)` 中 MV 为 on-ramp vehicle，`x_MV_global < x0_m_global` 且 `merge_state != executing`。When：Step 4 调度该 MV。Then：P05 不处理该 MV，必须 hand off 给 P04 Step4A APS；不得生成 CMC event、Eq.53 event、merge command 或 boundary cap command。
- Given：冻结 `S(t)` 中 MV 位于 `x0_m_global <= x_MV_global <= x_ramp_end_global` 且 `merge_state in {not_started, waiting}`。When：Step 4B 调度该 MV。Then：进入 CMC waiting decision branch，读取 P04 effective assignment 或 APS cache，执行 assignment validation、Eq.52、boundary cap 和 Eq.53；不得重跑 APS。
- Given：冻结 `S(t)` 中 MV 的 `merge_state == executing`。When：Step 4B 调度该 MV。Then：不执行 assignment validation 作为撤销条件，不重新判断 Eq.53 是否开始合流；只生成或刷新 boundary speed cap command，写入 continue merge command，记录 `executing_continuation` event。
- Given：P04 本步产出 `EffectiveAssignmentThisStep`。When：P05 需要 assignment。Then：优先消费该本步派生 assignment，并记录 `assignment_source=effective_assignment_this_step`；不得读取本步中途新 state。
- Given：P04 本步未更新 assignment，但 `S(t).aps_assignment_cache[mv_id]` 存在且 status 可用。When：P05 需要 assignment。Then：消费 cache-derived assignment，并记录 `assignment_source=aps_cache` 或 `cache_reused`。
- Given：P05 找不到 valid assignment，或 assignment status 为 `failed / empty / invalid`。When：assignment validation 执行。Then：生成 `assignment_invalid` event / sanity，reason 为 `clv_missing / cfv_missing / stale_assignment / unknown` 等结构化 reason；不执行 Eq.53 或记录 `Eq53_evaluated=false_or_skipped`；生成 waiting command / conservative handling command；不得伪造 CLV / CFV。
- Given：assigned CLV / CFV 在 `S(t)` 中不存在或已非 active。When：assignment validation 执行。Then：标记 assignment invalid，reason 为 `clv_missing`、`cfv_missing` 或 `vehicle_exited`；不开始合流。
- Given：assigned CLV / CFV 仍存在，但 assigned CLV 或 CFV 已不在 lane 2，且没有处于允许继续作为目标 gap 边界的状态。When：assignment validation 执行。Then：标记 assignment invalid，reason 为 `clv_not_lane_2` 或 `cfv_not_lane_2`；不使用 actual lane 2 leader / follower 替代。
- Given：assigned CLV / MV / assigned CFV 的纵向顺序不满足 `x_CLV > x_MV > x_CFV`。When：assignment validation 执行。Then：标记 assignment invalid，reason 为 `wrong_order` 或 `unsafe_gap_boundary`；不开始合流。
- Given：assignment valid，且 MV 位于 merging zone。When：Eq.52 执行。Then：按上游车辆模型规格计算 `h_tilde_MV_CM(t)`，至少在 event payload 中记录 `h_tilde`、`h_upper_cm`、`xi`、`x0_m_global`、`x_ramp_end_global`、`x_mv_global`；若公式 OCR 尚需复核，代码实现前必须以车辆模型规格 / PDF 复核结果为准。
- Given：assignment valid 且 Eq.52 已得到 `h_tilde`。When：Eq.53 checker 执行。Then：使用 assigned CLV / CFV 计算：

```text
d_MV_to_CLV = x_CLV - x_MV - L_CLV
d_CFV_to_MV = x_MV - x_CFV - L_MV
threshold = v_MV_or_specified_speed_basis * h_tilde
```

并记录 `d_MV_to_CLV`、`d_CFV_to_MV`、`threshold`、`eq53_gap_pass`。
- Given：`d_MV_to_CLV >= threshold` 且 `d_CFV_to_MV >= threshold`。When：Eq.53 checker 完成。Then：生成 CMC decision event `eq53_pass=true`，写入 merge-start command，写入 `merge_state: not_started/waiting -> executing` state transition request；不得直接修改真实 `merge_state`。
- Given：任一 Eq.53 gap 不满足。When：Eq.53 checker 完成。Then：生成 CMC decision event `eq53_pass=false`，记录 `fail_side=CLV_gap / CFV_gap / both`；写入 waiting command 和 speed cap command；不得生成 merge-start command。
- Given：assignment valid 或 invalid，MV 位于 waiting / not_started。When：boundary speed cap calculator 执行。Then：按 Eq.54-Eq.56 语义计算或刷新 boundary speed cap，生成 `SpeedCapCommand`，记录 `boundary_speed_cap`、`cap_reason`、`cap_feasible`、`cap_binding` 或等价字段。
- Given：boundary speed cap 结果非绑定，例如 `cap_value > current_or_candidate_speed`。When：speed cap command 写入。Then：仍允许记录 non-binding speed cap event，供 `MVS-CMC-1` 和 P08 追踪；不得因此直接改 MV 真实速度。
- Given：boundary speed cap 结果绑定，例如 `cap_value < current_or_candidate_speed`。When：speed cap command 写入。Then：记录 binding cap event，供 `MVS-SAFE-1A_waiting_cap` 的 P08 planning speed 合成消费；P05 不计算最终 planning speed。
- Given：boundary speed cap 不可行、为负或过低。When：speed cap calculator 执行。Then：记录 `boundary_cap_reason=cap_infeasible / cap_negative / cap_too_low`，并记录工程补丁 source / reason / is_engineering_patch；P05 不拍板最终保守运动策略。
- Given：assignment invalid policy 为 `wait_until_next_APS`。When：assignment invalid 发生。Then：写入 cache invalidate / wait-next-APS request 或等价 event；标注为第一版工程策略；不得本步假装 APS 成功。
- Given：assignment invalid policy 允许 `immediate_APS_refresh`。When：assignment invalid 发生。Then：只能写入 APS refresh request 或下步刷新标记；该策略必须标注为 `first_version_engineering_patch`，且仍必须通过 APS 语义重新生成 assignment，不得直接替换 actual leader / follower。
- Given：P01 matcher 消费 P05 输出。When：缺少 P05 required event / sanity / PNG feature。Then：后续 P05 red tests 必须报告 missing expected event / missing sanity / missing feature registration，而不是 loader error、unknown field、ImportError、AttributeError 或自然语言断言失败。
- Given：Step 4B 完成。When：比较冻结 `S(t)` 签名。Then：车辆真实 `x / y / v / a / physical_lane / road_role / merge_state` 未被 P05 改写。

## 5. 允许实现的代码对象

后续执行 P05 时，必须先新增 failing tests / test skeleton 并确认 red phase，再在同一轮中实现以下最小 CMC 对象。允许实现范围仅限 P05 Step 4B；不得越界实现 P06-P12。

- domain / state objects:
  - 复用 P04 `EffectiveAssignmentThisStep` 与 APS assignment cache。
  - 复用 P02 `SimulationState`、`RelationsSnapshot`、region resolver、`x_global` 几何口径。
  - 实现或映射 `CMCDecision`，字段以 `CORMC代码数据结构设计_整理版.md` 为准。
  - 实现或映射 assignment validation result，必须能表达 valid / invalid、invalid reason、source。
- command / next-state objects:
  - 复用 `CommandBuffer.merge_commands`。
  - 复用 `CommandBuffer.speed_cap_commands`。
  - 复用 `CommandBuffer.state_transition_commands`。
  - 复用 `CommandBuffer.cache_update_commands`。
  - 实现或映射 `MergeCommand`、`SpeedCapCommand`、`StateTransitionCommand`、`CacheUpdateCommand`。
  - P05 不写 `NextStateBuffer.candidate_kinematics`，不生成纵向 / 横向 candidate。
- step runner / service functions:
  - `resolve_step4b_cmc_branch`
  - `resolve_cmc_assignment_source`
  - `validate_cmc_assignment`
  - `compute_cmc_dynamic_acceptable_gap`
  - `compute_eq53_gap_inputs`
  - `check_eq53_gap`
  - `compute_boundary_speed_cap`
  - `build_waiting_command`
  - `build_merge_start_command`
  - `build_executing_merge_continuation_command`
  - `build_boundary_speed_cap_command`
  - `build_cmc_state_transition_request`
  - `build_assignment_cache_invalidate_request`
  - `run_step4b_cmc`
- event / sanity helpers:
  - `emit_cmc_branch_event`
  - `emit_assignment_validation_event`
  - `emit_assignment_invalid_event`
  - `emit_eq52_dynamic_gap_event`
  - `emit_eq53_gap_event`
  - `emit_boundary_cap_event`
  - `emit_waiting_command_event`
  - `emit_merge_start_command_event`
  - `emit_executing_continuation_event`
  - `run_cmc_assignment_sanity`
  - `run_cmc_boundary_cap_sanity`
  - `run_p05_no_write_before_commit_sanity`
  - `register_p05_png_features`
  - 以上 `emit_*` / `run_*` 名称只是实现 helper 名，不是 schema enum。实际 `event_type` 应使用既有 `cmc`、`assignment_invalid`、必要时 `engineering_patch`；实际 `check_type` 应使用既有 `assignment_invalid`、`boundary_violation`、`state_machine_inconsistency`、`multiple_commit_for_one_vehicle` 等，或先修订数据结构规格。
- scenario tests:
  - `test_mvs_cmc_1_eq53_pass_starts_merge_contract`
  - `test_mvs_cmc_2_eq53_fail_waiting_contract`
  - `test_mvs_assign_1_invalid_assignment_does_not_swap_actual_leader_follower_contract`
  - `test_waiting_boundary_speed_cap_command_for_safe_1a_prereq_contract`
  - `test_executing_merge_continuation_does_not_rejudge_merge_start_contract`
  - `test_p05_consumes_p04_effective_assignment_without_rerunning_aps`
  - `test_p05_does_not_write_vehicle_state_before_commit`
  - `test_assignment_invalid_marked_engineering_patch`
- regression tests:
  - P00 static traceability remains unchanged unless P00 maturity row is intentionally updated after human review。
  - P01 matcher tests remain green。
  - P02 freeze / relation / geometry tests remain green。
  - P03 command / commit / event / sanity tests remain green。
  - P04 targeted APS tests remain green。

字段与结构核对：

```text
当前上游数据结构文档已经登记 P05 所需核心概念。
当前代码已有 CommandBuffer 的 merge_commands / speed_cap_commands / state_transition_commands 分组，以及通用 EventRecord / SanityCheckRecord / matcher。
当前代码尚未实现 P05 CMC runner、内置 MVS-CMC / MVS-ASSIGN 场景、typed CMCDecision / MergeCommand / SpeedCapCommand。
这些是后续 P05 implementation gap，不是本轮要补的代码。
若后续发现 loader、enum、ExpectedEventSpec 或 ExpectedSanityCheckSpec 不足以表达 P05 场景，必须先修订权威数据结构规格，再实现代码。
P05 场景不得绕过 schema/matcher preflight；细粒度 CMC 断言必须通过既有 event_type + payload/match/numeric_expectations 表达。
```

## 6. 先写失败测试

后续执行 P05 时，必须在同一轮中完成 red-before-green，不得把 P05 再拆成“只写 spec / 只写 failing tests / 再实现”多个执行阶段。步骤如下：

1. 先做 schema / matcher preflight：确认 P05 场景只使用已有 EventType、SanityCheckType、ExpectedEventSpec、ExpectedSanityCheckSpec 字段；若不满足，先修订权威数据结构规格。
2. 新增 P05 failing tests / test skeleton。
3. 先运行 P05 targeted tests，确认红灯发生在 expected event / sanity / matcher 层。
4. 红灯不得是 loader error、unknown enum、unknown field、ImportError、AttributeError、自然语言断言失败或测试配置结构错误。
5. 立即实现 P05 最小 CMC，使 P05 required targeted tests 转绿。
6. 再运行 P05 targeted green tests 和 P00-P04 回归，尤其保护 P04 不被 P05 重跑 APS 污染。
7. 返回 red-before-green 证据：初始 red report、green report、回归报告。

测试治理规则：

```text
1. 若测试是“真实红灯测试”，即 pytest failure 用于证明 P05 尚未实现，则必须使用
   pytest.mark.xfail(strict=True, reason="P05 CMC not implemented")，或放入独立 red-test 命令。
2. 若测试是“合同测试”，即 runner/report 当前应返回 required_failed 且 failure reason 停在 matcher 层，
   则该测试可以在主线保持绿色。
3. 主线全量测试不得因为 P05 尚未实现而变红，除非该轮目标明确是 red phase 且随后同轮转绿。
4. 如果新增 sanity check 名称尚未出现在 `CORMC代码数据结构设计_整理版.md` 的权威集合中，
   只能先作为 spec 修订建议或 matcher 需求，不得直接在代码中暗增。
```

未来 P05 tests 至少包括：

- `test_mvs_cmc_1_eq53_pass_starts_merge_contract`
  - 加载或构造 `MVS-CMC-1`。
  - 断言 assignment valid、`h_tilde ≈ 1.0667`、`eq53_pass=true`、boundary cap non-binding、merge-start command、state transition request。
  - 断言真实 `merge_state` 尚未被 P05 直接改写。
- `test_mvs_cmc_2_eq53_fail_waiting_contract`
  - 加载或构造 `MVS-CMC-2`。
  - 断言 assignment valid、`eq53_pass=false`、`fail_side=CLV_gap`、waiting command、speed cap command。
  - 断言无 merge-start command。
- `test_mvs_assign_1_invalid_assignment_does_not_swap_actual_leader_follower_contract`
  - 加载或构造 `MVS-ASSIGN-1`。
  - 断言 assigned CFV 不在 lane 2 时 reason 为 `cfv_not_lane_2`。
  - 断言 `Eq53_evaluated=false_or_skipped`、`merge_command_created=false`。
  - 断言无 replacement assignment arrow / no actual leader-follower replacement。
- `test_waiting_boundary_speed_cap_command_for_safe_1a_prereq_contract`
  - 使用 `MVS-SAFE-1A_waiting_cap` 的 P05 前置部分。
  - 断言 P05 生成 `boundary_speed_cap ≈ 2.63` 的 `SpeedCapCommand` / event / sanity。
  - 不要求 P08 planning_speed 已合成。
  - 不得把 `MVS-SAFE-1A_waiting_cap` 写成 P05 full pass；P05 只验 speed cap command / event / sanity 前置证据。
- `test_executing_merge_continuation_does_not_rejudge_merge_start_contract`
  - 使用 `merge_state == executing` 的 MV。
  - 应预加载 `merge_state=executing`，并在需要时预加载对应 `ManeuverTrajectoryState` / `preloaded_maneuver_trajectory_states`；若测试不预加载 active maneuver，则必须明确断言 P05 只生成 continuation `MergeCommand`，不负责 lateral trajectory progress。
  - 断言无新 Eq.53 start decision、无 assignment validation 撤销、存在 `executing_continuation` event / merge continuation command / speed cap command。
- `test_p05_consumes_p04_effective_assignment_without_rerunning_aps`
  - 预置 P04 effective assignment 或 cache。
  - 断言 P05 event 中 `assignment_source=effective_assignment_this_step / aps_cache`。
  - 断言 `APS` fresh calculation count 为 0。
- `test_p05_does_not_write_vehicle_state_before_commit`
  - 比较 P05 前后冻结 `S(t)` 签名。
  - 断言真实 `x / y / v / a / lane / merge_state` 未变。
- `test_assignment_invalid_marked_engineering_patch`
  - 断言 assignment invalid event 携带 `source=first_version_engineering_patch`、`reason`、`is_engineering_patch=true`。

这些测试未来执行时必须由 P01 runner / matcher 消费，不能只依赖函数单测。函数单测可以辅助定位数值，但 P05 完成证据必须来自 targeted MVS matcher report。

## 7. 验收证据

P05 implementation 完成后，必须返回以下证据：

- Targeted MVS green evidence:
  - `MVS-CMC-1` targeted green evidence。
  - `MVS-CMC-2` targeted green evidence。
  - `MVS-ASSIGN-1` targeted green evidence。
  - `MVS-SAFE-1A_waiting_cap` 前置 speed cap command evidence。
- EventRecord / event dict:
  - CMC branch event：
    - `branch=cmc_waiting_decision / cmc_executing_continuation / handed_off_to_p04`
    - `mv_id`
    - `x_global`
    - `merge_state`
    - `zone_state`
  - assignment validation event：
    - `assignment_source=effective_assignment_this_step / aps_cache / test_preload`
    - `assignment_valid=true/false`
    - `assigned_clv_id`
    - `assigned_cfv_id`
    - `invalid_reason`
  - assignment invalid event：
    - `reason=cfv_not_lane_2 / clv_not_lane_2 / clv_missing / cfv_missing / vehicle_exited / wrong_order / stale_assignment / unknown`
    - `source=first_version_engineering_patch`
    - `is_engineering_patch=true`
  - Eq.52 / Eq.53 event：
    - `h_tilde`
    - `d_MV_to_CLV`
    - `d_CFV_to_MV`
    - `threshold`
    - `eq53_pass`
    - `fail_side`
  - boundary cap command event：
    - `boundary_speed_cap`
    - `cap_source=boundary_collision_avoidance`
    - `cap_reason=normal_cap / cap_infeasible / cap_negative / cap_too_low / not_applicable`
    - `cap_feasible`
    - `cap_binding`
  - waiting command event：
    - `merge_command_created=false`
    - `longitudinal_mode=cmc_waiting` 或等价 command reason。
  - merge-start command event：
    - `init_or_continue_maneuver=init`
    - `target_lane=lane_2`
    - `target_y=0.0`
    - `state_transition_request=executing`
  - executing continuation event / command：
    - `init_or_continue_maneuver=continue`
    - `no_new_eq53_start_decision=true`
    - `does_not_rejudge_merge_start=true`
- SanityCheckRecord / sanity dict:
  - `assignment_invalid=pass/warning/fail/not_applicable`，按场景语义。
  - `no_same_step_actual_leader_follower_replacement=pass` 或等价 sanity；若该 check type 尚未进入数据结构规格，先修订规格。
  - `boundary_violation=pass/warning/fail/not_applicable`，按场景语义。
  - `p05_no_write_before_commit=pass` 或复用 P03 `no_write_before_commit`。
  - `x_plot_used_in_algorithm_path=pass`。
  - `state_machine_inconsistency=pass`。
- PNG feature / marker:
  - `cmc_decision_marker`
  - `merge_start_marker`
  - `waiting_marker`
  - `assigned_clv_cfv_marker`
  - `assignment_invalid_marker`
  - `boundary_cap_marker`
  - `executing_continuation_marker`
  - `no_replacement_assignment_arrow`
- Scenario matcher report:
  - expected_events / forbidden_events / expected_event_counts / expected_sanity_checks / expected_png_features 均由 P01 matcher 消费。
  - red phase 初始失败原因应为 missing expected event / sanity / feature registration。
  - green phase required 场景应为 required_passed。
- No-write evidence:
  - P05 运行前后冻结 `S(t)` 签名一致。
  - P05 输出只存在于 command buffer、event / sanity history、PNG feature registration 或本步派生结构。
- No-rerun APS evidence:
  - P05 event payload 明确 assignment source。
  - P05 不产生 `APS_candidate`、fresh APS assignment 或 APS cache update event，除非只是写入 assignment invalid 后的 APS refresh request。
- No leader / follower substitution evidence:
  - `MVS-ASSIGN-1` 中 assigned CFV 位于 lane 1 时，P05 记录 invalid，不选 actual lane 2 follower 作为替代。
- No P06-P12 overreach evidence:
  - P05 不产生 cooperative request conflict event。
  - P05 不产生 CUC choice event。
  - P05 不产生 lane-change command。
  - P05 不产生 longitudinal planning speed。
  - P05 不产生 lateral trajectory progress。
  - P05 不产生 full smoke suite artifact。

## 8. 完成标准

P05 进入完成状态时必须满足：

- `MVS-CMC-1` required gate 通过。
- `MVS-CMC-2` required gate 通过。
- `MVS-ASSIGN-1` required gate 通过。
- 为 `MVS-SAFE-1A_waiting_cap` 提供可被 P08 消费的 speed cap command / event / sanity 前置证据。
- P05 消费 P04 effective assignment / cache。
- P05 不重跑 APS。
- P05 不偷换 leader / follower。
- P05 不在 assignment invalid 时生成 fake assignment。
- Eq.53 pass 时只生成 merge-start command / transition request，不直接提交真实 lane / y / merge_state。
- Eq.53 fail 时生成 waiting command / speed cap command，不直接改真实速度。
- assignment invalid 时结构化记录 reason，不伪造 assignment。
- executing merge continuation 不重新判断 merge start，不因短时 gap 变化退回 waiting。
- boundary speed cap 只作为 command / speed constraint 输出，不直接提交 MV 真实速度。
- P05 event / sanity / PNG feature 不等待 P11 才补。
- 所有工程补丁保留 `source` / `reason` / `is_engineering_patch`。
- required / probe / deferred 报告语义仍由 P01 runner 保持。
- P00-P04 回归保持绿色。
- 若新增字段、enum、ScenarioConfig、EventRecord、SanityCheckRecord 或 expected_* 口径，已有上游规格先行修订并可追溯。

## 9. 回归保护

- 所有算法内部使用 `x_global`；`x_plot` 只用于 PNG / renderer 派生层。
- 所有模块只读冻结 `S(t)`。
- command / next-state 不反写真实状态。
- commit 是唯一生成 `S(t+dt)` 的阶段。
- P05 只处理 MV 已进入 merging zone 或 `merge_state == executing` 的 CMC 分支。
- P05 不得重跑 APS。
- P05 不得重算或替换 CLV / CFV。
- P05 不得用 actual leader / follower 兜底替代 assignment invalid。
- assignment invalid 不得静默产生 fake assignment。
- assignment invalid、immediate APS refresh、invalid 后等待 / conservative handling、boundary cap 不可行处理入口、executing continuation 记录约束必须标记为工程补丁或第一版实现约束。
- boundary speed cap 只作为 command / constraint，不直接修改速度。
- merge start / waiting / executing continuation 都必须等 commit 后才成为真实状态变化。
- `merge_state == executing` 后继续既有 merge trajectory，不重新判断是否开始合流，不撤销，不退回 waiting。
- P05 不得实现 CUC、cooperative conflict、纵向模型、横向轨迹或正式 PNG。
- P05 不得等待 P11 才补 event / sanity / PNG feature。
- P05 必须保留 P04 targeted APS gate 行为，不破坏 `MVS-APS-*`。
- P05 必须保留 P03 no-write-before-commit、one-final-candidate-per-vehicle、state-machine consistency 约束。
- 若 boundary cap 不可行、为负或过低，P05 只能记录 speed cap reason / warning / engineering patch trace；最终运动保守策略由后续车辆模型 / P08-P09 规格承接。
- 若后续实现需要新增或收紧 sanity check type，例如 `no_same_step_actual_leader_follower_replacement`、`p05_no_write_before_commit`、`boundary_cap_command_present`，必须先修订 `CORMC代码数据结构设计_整理版.md` 或以既有通用 sanity payload 表达，不得暗增不可追溯字段。
