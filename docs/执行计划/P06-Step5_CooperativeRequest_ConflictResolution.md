# P06 - Step 5 Cooperative Request / Conflict Resolution

> 本文档是 P06 的完整执行计划 spec。它只定义后续 P06 red-before-green implementation 的行为合同、测试计划和验收证据；本轮不实现 P06 cooperative request / conflict resolution 代码，不新增 P06 实际测试文件，不修改业务算法。
>
> P06 只覆盖一次 `S(t) -> Step 5 cooperative request / conflict result / event / sanity -> P07` 的时间步切片。P06 不是泛化冲突处理模块，也不是 CUC / lane-change / longitudinal / lateral / commit 的实现入口。

## 0. Slice Identity

- Algorithm-Step Coverage:
  - Primary Step: Step 5，汇总 P04 / P05 handoff 中有效 assignment 派生出的 cooperative request，并对同一 CV 被多个 MV 请求的情况执行第一版工程仲裁。
  - Secondary Steps:
    - Step 4A / Step 4B handoff reader。
    - valid assignment filter。
    - `col_CLV / col_CFV` request collector。
    - cooperative request event candidate。
    - conflict grouping by `cv_id`。
    - priority resolver。
    - active cooperative request selector。
    - loser / suppressed request trace。
    - conflict resolution event candidate。
    - one-active-request-per-CV sanity。
    - P06 expected_png_features / marker registration。
- MVS Acceptance Gate:
  - required:
    - `MVS-CONFLICT-1A`
    - `MVS-CONFLICT-1B`
  - probe:
    - conflict priority basis diagnostic：记录每个 request 的 `mv_in_merging_zone`、`t_mv_star`、`mv_distance_to_x0_m` 或等价优先级输入。
    - P05 handoff diagnostic：当 P05 assignment validation 结果存在时，P06 能说明该 assignment 被保留或被过滤。
    - deterministic tie-breaker diagnostic：若三层优先级仍完全相同，P06 使用稳定排序键 `(cv_id, source_mv_id, request_id)` 选择 winner；该规则必须标记为 `first_version_engineering_patch`，不得写成论文原生规则。
  - deferred:
    - `MVS-CUC-1A_override_choice1`
    - `MVS-CUC-1B_real_utility_probe`
    - `MVS-CUC-1C_real_utility_choice1_locked`
    - `MVS-CUC-2`
    - `MVS-CUC-3`
    - `MVS-E2E-1`
    - `MVS-COMMIT-1-full`
    - 全局多 MV gap 优化或全局合流顺序优化。
    - P07-P12 的任何业务实现。
- 本阶段解锁的能力:
  - 从有效 APS assignment / `EffectiveAssignmentThisStep` / P05 validated assignment evidence 中收集 `col = 1` 的 CLV / CFV cooperative request。
  - 过滤 failed / invalid / empty / not available assignment，保证 invalid assignment 不会伪装成 active request。
  - 对同一 CV 的多个 request 形成 conflict group，并输出唯一 active cooperative request。
  - 为 P07 提供稳定、可追溯的 active request 输入。
  - 保留 winner / loser / suppressed reason / priority basis / engineering patch 证据，供 P10 / P11 聚合。
- 本阶段不要求通过的后续场景:
  - P06 不要求任何 `MVS-CUC-*` 场景 full pass；这些场景只能把 P06 active request 作为前置输入。
  - P06 不要求 `MVS-E2E-1` 或 `MVS-COMMIT-1-full` 通过。
  - P06 不要求正式 PNG renderer 存在，只要求 registered marker / expected_png_features 可被 P01 matcher 报告。

## 1. 本阶段目标

P06 聚焦一次冻结状态下的 Step 5：

```text
S(t)
  + P04 effective assignment evidence
  + P05 assignment validation / CMC handoff evidence
    -> Step 5 cooperative request collector
    -> conflict resolution result
    -> active cooperative request for P07
    -> event / sanity / PNG marker
```

P06 的最小目标是把 APS / CMC 上游已经产生的 assignment 证据转换成 P07 可消费的 active cooperative request。P06 不重新计算 APS，也不重新验证 CMC Eq.53；它只判断某个上游 assignment 是否可作为本步 request 来源，以及多个 request 指向同一 CV 时谁在本步生效。

P04 已提供给 P06 的证据包括：

- `EffectiveAssignmentThisStep` 或等价本步派生 assignment。
- assignment source：`aps_updated_this_step` / `cache_reused`。
- APS assignment payload：`mv_id`、`clv_id`、`cfv_id`、`aps_case`、`col_clv`、`col_cfv`、`desired_spacing_override`、`status`、cache / source metadata。
- APS event 证据：case、CLV、CFV、`col_CLV`、`col_CFV`、`T*_MV`、failure reason、cache action。
- no-write-before-commit sanity 和 `x_plot_used_in_algorithm_path` sanity。

P05 已提供给 P06 的证据包括：

- assignment validation event / result，说明 assignment source、assigned CLV / CFV、valid / invalid、invalid reason。
- assignment invalid event / sanity，说明 invalid assignment 不得进入后续协同。
- CMC branch event，说明 MV 是否在 merging zone、是否 executing continuation、是否 handed off。
- P05 no-rerun APS 证据，说明 P06 不能把 P05 阶段重新当 APS 入口。

P06 需要给 P07 的输出是：

- active cooperative request：每个 `cv_id` 在同一时间步最多一个 active winner。
- loser / suppressed request trace：未获胜 request 不能静默丢失。
- conflict resolution result：包含 conflict group、winner、loser、priority basis、reason、`source`、`is_engineering_patch`。
- cooperative_request event 和 conflict_resolution event。
- sanity / matcher evidence：用 `conflict_resolution` / `cooperative_request` event payload 和 `expected_event_counts` 证明 one-active-request-per-CV、invalid assignment suppressed；正式 `SanityCheckType` 仅使用当前已授权类型，例如 `no_write_before_commit`。不得把 helper 名直接当成新 sanity enum。
- expected_png_features：至少注册 request marker、conflict group marker、winner marker、loser / suppressed marker。

`MVS-CONFLICT-1A` 必须证明：当同一 CV 被一个已经在 merging zone 的 MV 和一个上游 MV 同时请求时，P06 选择 merging zone MV 为 winner，并将 loser request 记录为 suppressed / conflict。

`MVS-CONFLICT-1B` 必须证明：当两个 MV 都未进入 merging zone 且请求同一 CV 时，P06 选择 `T*_MV` 更小者为 winner，并记录 priority basis 为 `smaller_T_star_MV`。

P06 不能把 `MVS-CUC-1A_override_choice1`、`MVS-CUC-2`、`MVS-CUC-3`、`MVS-E2E-1` 误写成本阶段 full pass。P06 只产出 active request；P07 才消费 active request 并执行 CUC choice / compliance / lane-change command / same-step overlay。

## 2. 非目标 / 禁止事项

- 不重做 P04 APS。
- 不重跑 APS candidate collector、`T*_MV` 预测、CLV / CFV selector 或 APS case classifier。
- 不重新选择 CLV / CFV。
- 不把 failed / invalid / empty assignment 伪装成 cooperative request。
- 不重做 P05 CMC。
- 不重新执行 assignment validation、Eq.52、Eq.53 或 boundary speed cap。
- 不用 actual lane 2 leader / follower 替换 upstream assignment。
- 不执行 P07 CUC utility。
- 不判断 CHV compliance。
- 不检查 CUC target lane TT safety。
- 不生成 lane-change command。
- 不生成 `SameStepManeuverRelationOverlay`。
- 不生成 longitudinal command、speed cap consumption、planning speed 或 Eq.10 consumption。
- 不生成 lateral trajectory candidate。
- 不执行 commit，不写 Step 9 commit result。
- 不实现 P08 纵向模型、P09 横向轨迹、P10 E2E integration、P11 artifact export 或 P12 随机边界生成。
- 不直接写真实车辆 `x / y / v / a / physical_lane / road_role / lane_change_state / merge_state`。
- 不修改 `SimulationState.aps_assignment_cache`；若后续需要 loser 触发 cache / APS refresh，只能作为 event / future request，不能在 P06 直接改 cache。
- 不新增核心字段、enum、ScenarioConfig 字段、EventRecord 字段或 SanityCheckRecord 字段；若发现现有 schema 不足，先修订 `docs/复现讨论/CORMC代码数据结构设计_整理版.md` 或相关上游规格。
- 不新增细粒度 `SanityCheckType`，例如 `conflicting_commands_to_same_CV` 或 `one_active_request_per_cv`，除非先完成上游数据结构修订；实现 helper 名可以细分，但落到 expected_sanity_checks 时必须使用已有 check_type 或先修订规格。
- 不把多 MV 共享 CV 仲裁写成论文原算法。该仲裁必须标记为第一版工程补丁，并保留 `source / reason / is_engineering_patch / priority_basis`。

## 3. 上游 spec 引用

- `docs/执行计划/CORMC执行计划spec设计总纲v1.md`
  - 引用统一模板、P04-P10 每个算法切片同步产出 event / sanity / expected_png_features 的规则。
  - 引用 P06 的 Step 5 定位：cooperative request 汇总与多 MV 冲突仲裁。
- `docs/执行计划/P00-Spec宪法_权威边界与二维追踪矩阵.md`
  - 引用 P06 trace row：`MVS-CONFLICT-1A`、`MVS-CONFLICT-1B`，event evidence 为 cooperative_request / conflict_resolution，sanity evidence 为 one active request per CV。
  - 引用工程补丁必须携带 `source`、`reason`、`is_engineering_patch` 的规则。
  - P00 当前静态矩阵仍把 P06 标为 `trace_registered`；本文档新增后不自动修改 P00 静态测试口径。P06 是否进入 implementation-ready 仍需人工审阅。
- `docs/执行计划/P01-MVS_Runner_ScenarioConfig与验收断言语言.md`
  - 引用 ScenarioConfig loader、expected_events、forbidden_events、expected_event_counts、expected_sanity_checks、expected_png_features、required / probe / deferred matcher。
  - P06 后续 red tests 必须失败在 expected event / sanity / matcher 层，不得失败在 loader error、unknown enum、unknown field、ImportError、AttributeError 或自然语言断言。
- `docs/执行计划/P02-Step0-3_清理冻结关系与几何口径.md`
  - 引用冻结 `S(t)`、relations snapshot、region resolver、`x_global` 几何口径。
  - P06 只读 P02 产出的冻结状态和区域判断，不读中途 next-state。
- `docs/执行计划/P03-Step9-10_Command_NextState_Commit_Event_Sanity_Trajectory闭环.md`
  - 引用 CommandBuffer / OutputHistory / EventRecord / SanityCheckRecord / no-write-before-commit 边界。
  - P06 不得直接提交车辆真实状态，event / sanity / marker 不反写运动。
- `docs/执行计划/P04-P07_总切片蓝图_dependency_sketch.md`
  - 引用 P06 的输入输出依赖：读取 P04 / P05 effective assignment、`col_CLV / col_CFV`、`T*_MV`、MV region、active vehicle / CV 状态；写 cooperative_request event、conflict_resolution event、active cooperative request、conflict loser result、conflict sanity、PNG marker。
  - 该蓝图中 P05-P07 maturity 仍是 skeleton / trace_registered 口径；P05 当前已有完整 spec 和实现测试，P06 本文档以 P04 / P05 已有 spec 与当前代码为上游事实。
- `docs/执行计划/P04-Step4A_APS_Cache_EffectiveAssignment.md`
  - 引用 P04 提供的 `EffectiveAssignmentThisStep`、`col_CLV / col_CFV`、APS case、Eq.10 desired spacing source、cache reuse / APS updated source、failure 不产生活跃 request 的边界。
  - P06 只能消费这些结果，不得重算 APS。
- `docs/执行计划/P05-Step4B_CMC_AssignmentValidation_Eq53_BoundaryCap.md`
  - 引用 P05 提供的 assignment validation、assignment invalid、CMC branch / zone evidence、no rerun APS evidence。
  - P06 应过滤 P05 已判 invalid 的 assignment，不得恢复为 active request。
- `docs/复现讨论/CORMC时间步执行顺序梳理.md`
  - 引用 Step 5 主循环：对所有有效 APS assignment 收集 `col = 1` CLV / CFV 请求；同一 CV 多请求时按第一版工程安全仲裁消解。
  - 引用优先级：已在 merging zone 的 MV > `T*_MV` 更小 > 距离 `x0^m` 更近。
- `docs/复现讨论/CORMC论文公式与实现映射.md`
  - 引用 Step 5 的公式边界：论文未完整定义多 MV 共享 CV 仲裁；该规则是工程补丁。
- `docs/复现讨论/CORMC状态与模块接口规格.md`
  - 引用 `active cooperative request`、`conflict request`、winner / loser、P07 只消费 active request 的接口语义。
  - 引用 Request arbitration 模块只能写本步派生结果、event、sanity，不得写车辆位置。
- `docs/复现讨论/CORMC代码数据结构设计_整理版.md`
  - 引用 `CooperativeRequest`、`ConflictResolutionResult`、`EventRecord`、`SanityCheckRecord`、`ExpectedEventSpec`、`ExpectedSanityCheckSpec`、`ExpectedPNGFeatureSpec`。
  - 引用 `EventType.cooperative_request` 与 `EventType.conflict_resolution` 的 canonical lower-case 语义。P04 / P05 当前代码仍使用 `"APS"` / `"CMC"` 兼容旧 runner；P06 不应引入第三套 casing。
  - §10.2 的旧阶段拆分表把 `CUCDecision` / `LaneChangeCommand` 放在 P06，把 `CMCDecision` / `MergeCommand` / `SpeedCapCommand` 放在 P07。该拆分表与当前 P04-P07 执行计划边界冲突，在 P06 中视为过期的执行阶段建议：P06 只实现 `CooperativeRequest` / `ConflictResolutionResult`，CUC 与 lane-change command 属于 P07，CMC command 属于 P05。
- `docs/复现讨论/CORMC输出指标与日志验证规格_整理版.md`
  - 引用 Step 5 日志需求：收集请求、冲突检测、仲裁结果、loser 状态、工程补丁标记。
  - 引用 conflict smoke 验收：多个 MV 请求同一 CV、winner / loser、仲裁依据、loser waiting / conflict、工程补丁标记。
- `docs/复现讨论/CORMC最小验证场景执行规格.md`
  - 引用 `MVS-CONFLICT-1A` 和 `MVS-CONFLICT-1B` setup、expected_events、expected_sanity_checks、expected_png_features。
  - 引用 `MVS-CONFLICT-1A` 是 conflict arbitration 单元测试，不是 APS 端到端测试。
- `docs/复现讨论/CORMC道路几何与区域规格.md`
  - 引用 `x0_m_global = 6950 m`、merging zone `[x0_m_global, x_ramp_end_global]`、`x_global` 不使用 `x_plot`。
- `docs/复现讨论/CORMC参数规格.md`
  - 引用 `dt`、`x0_m_global`、APS / CMC 参数来源；P06 不重新定义参数，不把仲裁权重写入参数规格。
- `docs/复现讨论/CORMC车辆模型规格.md`
  - 引用 CUC 只对 active cooperative request 中 `col = 1` 的 CV 执行，Eq.10 由后续 P07 / P08 消费，不在 P06 消费。

## 4. 行为契约 Given / When / Then

- Given：冻结 `S(t)`、Step 3 relations、P04 effective assignments、P05 assignment validation / CMC event evidence 已存在。When：执行 Step 5。Then：P06 只读取这些输入，输出 request / conflict 派生结构、event、sanity、PNG marker；不得修改冻结车辆状态。
- Given：某 MV 的 assignment status 为 `failed`、`invalid`、`empty`，或 P04 handoff 标记不可进入 cooperative request。When：P06 collector 扫描该 assignment。Then：不生成 active cooperative request；若需要可记录 suppressed / filtered event，reason 为 `invalid_assignment_filtered`、`failed_assignment_filtered` 或等价结构化 reason。
- Given：P05 已记录 assignment invalid，reason 例如 `cfv_not_lane_2`、`clv_missing`、`wrong_order`。When：P06 处理同一 MV 的 assignment evidence。Then：P06 必须以 P05 validation 为准过滤该 request，不得从 APS cache 恢复成 active request。
- Given：assignment 有效且 `col_clv = true`。When：P06 collector 执行。Then：生成一个 CLV cooperative request，`cv_id = clv_id`，`cv_role = clv`，`col = true`，保留 `source_mv_id`、`aps_case`、`t_mv_star` 或 schema gap 标记。
- Given：assignment 有效且 `col_cfv = true`。When：P06 collector 执行。Then：生成一个 CFV cooperative request，`cv_id = cfv_id`，`cv_role = cfv`，`col = true`，保留 `source_mv_id`、`aps_case`、Eq.10 desired spacing reference 或 source。
- Given：assignment 有效但 `col_clv = false` 且 `col_cfv = false`。When：P06 collector 执行。Then：不生成 cooperative request；可记录 `no_request_due_to_col_false` 作为 debug event，但不得产生 active request。
- Given：assignment 中 CLV / CFV id 缺失、对应 CV 不在 `S(t).active_vehicle_ids`，或 CV 已被 P05 判定失效。When：P06 collector 执行。Then：该 request 不进入 active set；若记录 event，必须说明 `cv_missing`、`cv_inactive` 或 upstream invalid reason。
- Given：多个 request 指向不同 `cv_id`。When：P06 resolver 执行。Then：每个 `cv_id` 分别输出一个 active request，无 conflict group；sanity 记录 one-active-request-per-CV pass。
- Given：同一 `cv_id` 被多个 request 请求。When：P06 resolver 分组。Then：生成 conflict group，并按以下优先级选择 winner：
  1. `mv_in_merging_zone = true` 的 request 优先。
  2. 若都不在 merging zone 或都在同类 zone，`T*_MV` 更小者优先。
  3. 若 `T*_MV` 相同，距离 `x0_m_global` 更近者优先。
  4. 若以上三层仍完全平局，按稳定排序键 `(cv_id, source_mv_id, request_id)` 升序选第一个 request；`cv_id` 在同一 conflict group 内通常相同，但仍保留在排序键中用于跨组审计一致性。
- Given：`MVS-CONFLICT-1A` 中 `MV_A` 已在 merging zone，`MV_B` 在 upstream，二者请求 `CV_X`。When：P06 resolver 执行。Then：`MV_A` 的 request 成为 active winner，`MV_B` 的 request 进入 loser trace；conflict event `priority_basis = MV_in_merging_zone`。
- Given：`MVS-CONFLICT-1B` 中 `MV_G1` 与 `MV_G2` 都 upstream 且请求 `SHARED_CLV_G`。When：P06 resolver 执行。Then：`T*_MV` 更小的 `MV_G1` 成为 active winner，`MV_G2` 进入 loser trace；conflict event `priority_basis = smaller_T_star_MV`。
- Given：三层优先级仍完全平局。When：P06 resolver 执行。Then：不得随机选择；winner 由 `(cv_id, source_mv_id, request_id)` 稳定排序决定，conflict event / result 必须记录 `priority_basis = deterministic_tie_breaker`、`source = first_version_engineering_patch`、`reason = deterministic_tie_breaker_after_equal_priority`、`is_engineering_patch = true`。P06 required gate 不要求覆盖该 tie case，但实现若触发该分支必须可观测。
- Given：winner 已选出。When：P06 输出 active request。Then：同一 `cv_id` 在本步最多有一个 active request；loser request 必须带 `suppressed_by_request_id`、`suppressed_reason`、`conflict_id` 或等价 payload，不得静默丢弃。
- Given：P07 后续执行。When：P07 读取 Step 5 输出。Then：只消费 P06 active cooperative request；P06 不为 P07 预先决定 CUC utility、compliance、target lane safety 或 lane-change command。
- Given：P06 完成。When：比较冻结 `S(t)` 签名。Then：真实 `x / y / v / a / physical_lane / road_role / lane_change_state / merge_state / aps_assignment_cache` 未被 P06 修改。

## 5. 允许实现的代码对象

后续执行 P06 时，必须先新增 P06 failing tests / test skeleton 并确认 red phase，再在同一轮中实现以下最小 Step 5 对象。允许实现范围仅限 P06 Step 5；不得越界实现 P07-P12。

- domain / state objects:
  - `CooperativeRequest`
  - `ConflictResolutionResult`
  - `ActiveCooperativeRequest` 或等价 active request view；若作为独立结构新增，必须先确认 `CORMC代码数据结构设计_整理版.md` 已授权或同步修订。
  - `SuppressedCooperativeRequest` / loser trace 可以作为 `ConflictResolutionResult.payload` 表达，不必新增核心 dataclass。
- command / next-state objects:
  - 可复用 `CommandBuffer.cooperation_commands` 承载 active request reference，但不得生成 lane-change / merge / longitudinal / speed-cap command。
  - 不写 `NextStateBuffer`。
  - 不写 `lane_change_commands`、`merge_commands`、`speed_cap_commands`、`state_transition_commands`。
- step runner / service functions:
  - `collect_cooperative_requests`
  - `filter_valid_request_assignments`
  - `build_cooperative_request_from_assignment`
  - `group_requests_by_cv`
  - `resolve_request_conflicts`
  - `select_conflict_winner`
  - `build_active_cooperative_requests`
  - `run_step5_cooperative_request_conflict_resolution`
- event / sanity helpers:
  - `emit_cooperative_request_event`
  - `emit_conflict_resolution_event`
  - `emit_suppressed_request_event`
  - `assert_p06_one_active_request_per_cv_event_payload`
  - `run_p06_invalid_assignment_suppressed_sanity`
  - `run_p06_no_write_before_commit_sanity`
  - `register_p06_png_features`
  - 以上 helper 名不是 schema enum。实际 event_type 应使用既有 `cooperative_request`、`conflict_resolution`、必要时 `engineering_patch`；actual sanity check_type 应使用既有类型或先修订数据结构规格。
  - 第一版不新增 `conflicting_commands_to_same_CV` / `one_active_request_per_cv` sanity enum。相关断言落在 `conflict_resolution` event payload、`cooperative_request` event payload、`expected_event_counts` 和 `no_write_before_commit` sanity 上。
- scenario tests:
  - `test_mvs_conflict_1a_merging_zone_mv_priority_contract`
  - `test_mvs_conflict_1b_smaller_t_star_priority_contract`
  - `test_p06_collects_col_true_requests_from_p04_effective_assignment`
  - `test_p06_keeps_or_filters_request_based_on_p05_assignment_validation`
  - `test_failed_invalid_empty_assignment_does_not_create_active_request`
  - `test_loser_request_has_suppressed_trace`
  - `test_conflict_resolution_marked_engineering_patch`
  - `test_same_cv_has_at_most_one_active_request`
  - `test_p06_does_not_execute_cuc_or_lane_change_command`
  - `test_p06_does_not_write_vehicle_state_before_commit`
  - `test_p06_does_not_rerun_aps_or_cmc`
- regression tests:
  - P01 matcher tests remain green。
  - P02 freeze / relation / geometry tests remain green。
  - P03 command / commit / event / sanity tests remain green。
  - P04 targeted APS tests remain green。
  - P05 targeted CMC tests remain green。

字段与结构核对：

```text
当前上游数据结构文档已登记 CooperativeRequest、ConflictResolutionResult、EventType.cooperative_request、EventType.conflict_resolution。
当前代码尚未实现 P06 runner、P06 built-in scenarios、P06 request / conflict resolver。
当前 ScenarioConfig 没有显式 preloaded_effective_assignments 字段，P06 tests 不得暗增该顶层字段。
当前 P04 code 的 EffectiveAssignmentThisStep 字段名是 available_for_cooperative_request；本轮将该名称冻结为实现字段名，并把 is_valid_for_request 仅保留为旧文档别名。
当前 P04 assignment cache value 未稳定暴露 t_mv_star；P04 APS event payload 暴露 t_star_mv。
当前 SanityCheckType 权威集合未包含 conflicting_commands_to_same_CV / one_active_request_per_cv。
这些是后续 P06 implementation 前必须处理的 schema / handoff gap；不得在代码中暗增不可追溯字段。
```

P06 handoff preflight 冻结口径：

| 项 | 冻结口径 | 实现前必须满足的失败边界 |
| --- | --- | --- |
| effective assignment 加载 | P06 red tests 只能使用当前 loader 已授权的 `preloaded_assignments`，或在 Python test helper 中显式构造 `EffectiveAssignmentThisStep`；不使用 `preloaded_effective_assignments` 顶层字段。 | 若需要新顶层字段，先修订数据结构文档、MVS 文档和 loader，再写 P06 implementation。 |
| request 可用字段 | canonical 实现字段名为 `available_for_cooperative_request`；旧文档名 `is_valid_for_request` 只能作为兼容别名，不得让 P06 同时猜两个来源。 | 若传入对象两个字段不一致，P06 preflight 必须失败并报告 handoff mismatch。 |
| `t_mv_star` 权威来源 | 优先级为：1. `EffectiveAssignmentThisStep.assignment["t_mv_star"]` 或 `["t_star_mv"]`；2. `preloaded_assignments` / APS cache 中同名字段，前提是 schema 和 loader 已同步允许；3. 显式关联 `source_event_id` 的 P04 APS event payload `t_star_mv`；4. 否则 P06 preflight 失败，不重算 APS。 | `MVS-CONFLICT-1B` 必须在 inline helper 或 preloaded assignment 中携带可观察 `t_mv_star/t_star_mv`，否则红灯应停在 handoff preflight，而不是 matcher 层伪失败。 |
| 同 CV 唯一 active request | 第一版通过 `conflict_resolution.payload.active_request_count_for_cv = 1`、`payload.one_active_request_per_cv = true`、`payload.conflicting_commands_to_same_CV = false` 以及 `expected_event_counts` 表达。 | 不新增未授权 `SanityCheckType`；`expected_sanity_checks` 只使用现有 check_type，例如 `no_write_before_commit`。 |

## 6. 先写失败测试

后续执行 P06 时，必须在同一轮中完成 red-before-green，不得把 P06 再拆成“只写 spec / 只写 failing tests / 再实现”多个执行阶段。步骤如下：

1. 先做 schema / matcher preflight：
   - 确认 P06 场景只使用已有 ScenarioConfig 顶层字段。
   - 确认 expected_events 使用 `cooperative_request`、`conflict_resolution` 或既有兼容 event_type，不新增私有 casing。
   - 确认 expected_sanity_checks 不使用未授权 check_type；`conflicting_commands_to_same_CV` / `one_active_request_per_cv` 第一版只能作为 event payload / matcher 语义键，若要升级为 sanity enum，先修订上游数据结构规格。
   - 确认 `MVS-CONFLICT-1B` 的 `T*_MV` 来源已经在 `EffectiveAssignmentThisStep.assignment`、`preloaded_assignments` / cache，或带 `source_event_id` 的 P04 APS event payload 中可追溯；不得由 P06 重算 APS 得到。
2. 新增 P06 failing tests / test skeleton。
3. 先运行 P06 targeted tests，确认红灯发生在 expected event / sanity / matcher 层。
4. 红灯不得是 loader error、unknown built-in scenario、unknown enum、unknown field、ImportError、AttributeError、自然语言断言失败或测试配置结构错误。
5. 立即实现 P06 最小 request collector / conflict resolver，使 `MVS-CONFLICT-1A`、`MVS-CONFLICT-1B` targeted gate 转绿。
6. 再运行 P06 targeted green tests 和 P00-P05 回归。
7. 返回 red-before-green 证据：初始 red report、green report、回归报告、winner / loser / priority_basis 样例。

测试治理规则：

```text
1. 若测试是“真实红灯测试”，即 pytest failure 用于证明 P06 尚未实现，则必须使用
   pytest.mark.xfail(strict=True, reason="P06 cooperative request conflict resolver not implemented")
   或放入单独 red-test 命令。
2. 若测试是“合同测试”，即 runner/report 当前应返回 required_failed 且 failure reason 停在 matcher 层，
   则该测试可以在主线保持绿色。
3. 主线全量测试不得因为 P06 尚未实现而变红，除非该轮目标明确是 red phase 且随后同轮转绿。
4. 如果新增 sanity check 名称尚未出现在 `CORMC代码数据结构设计_整理版.md` 的权威集合中，
   只能先作为 schema gap / upstream revision requirement，不得直接在代码中暗增。
```

P05 validated assignment evidence 的最小字段：

| 字段 | 用途 | P06 口径 |
| --- | --- | --- |
| `mv_id` | 关联 request source MV | 必需。 |
| `assignment_id` | 关联 P04 assignment / cache / event | 若当前实现尚无独立 id，可用 `mv_id + created_at_step + source` 的稳定组合键，但必须在 event payload 中可追溯。 |
| `validation_status` | valid / invalid / failed / not_applicable | 只有 valid 可进入 request collector。 |
| `invalid_reason` | 过滤原因 | invalid / failed 时必须保留到 suppressed / filtered trace。 |
| `cmc_branch` | P05 分支 | 用于证明 P06 未重跑 CMC，只消费 P05 结果。 |
| `mv_in_merging_zone` | 第一优先级输入 | P06 priority resolver 使用该值或等价 region evidence。 |
| `source_event_id` | 追溯 P05 event | 若 P06 从 event payload 取验证证据，该字段必需。 |

未来 P06 tests 至少包括：

- `test_mvs_conflict_1a_merging_zone_mv_priority_contract`
  - 加载或构造 `MVS-CONFLICT-1A`。
  - 可直接构造 P04 / P05 handoff evidence；该场景不是 APS end-to-end test。
  - 断言 `CV_X` 的 request group 包含 `MV_A` 与 `MV_B`。
  - 断言 winner 为 `MV_A`，loser 为 `MV_B`。
  - 断言 `priority_basis = MV_in_merging_zone`。
  - 断言 `source = first_version_engineering_patch` 且 `is_engineering_patch = true`。
- `test_mvs_conflict_1b_smaller_t_star_priority_contract`
  - 加载或构造 `MVS-CONFLICT-1B`。
  - 断言 `SHARED_CLV_G` 的 request group 包含 `MV_G1` 与 `MV_G2`。
  - 断言 winner 为 `MV_G1`，loser 为 `MV_G2`。
  - 断言 `priority_basis = smaller_T_star_MV`，并能观测 `MV_G1.t_mv_star = 5.5`、`MV_G2.t_mv_star = 6.0` 或等价 numeric expectation。
- `test_p06_collects_col_true_requests_from_p04_effective_assignment`
  - 构造有效 P04 `EffectiveAssignmentThisStep`。
  - case 2：只应从 CFV 生成 request。
  - case 3：只应从 CLV 生成 request。
  - case 4：CLV 与 CFV 各生成一个 request。
  - case 1：不生成 request。
- `test_p06_keeps_or_filters_request_based_on_p05_assignment_validation`
  - P05 validation valid：保留对应 assignment 的 request 输入。
  - P05 validation invalid：过滤 request，并保留 suppressed reason。
  - 断言 P06 不自行重跑 P05 assignment validation。
- `test_failed_invalid_empty_assignment_does_not_create_active_request`
  - `status = failed / invalid / empty` 均不生成 active request。
  - expected_event_counts 中 active request count 为 0。
- `test_loser_request_has_suppressed_trace`
  - conflict 后 loser request 必须可在 event payload 或 result structure 中找到。
  - 断言 loser 不静默丢失。
- `test_conflict_resolution_marked_engineering_patch`
  - conflict_resolution event 必须有 `source=first_version_engineering_patch`、`reason`、`is_engineering_patch=true`、`priority_basis`。
- `test_same_cv_has_at_most_one_active_request`
  - 对每个 `cv_id`，active request count 最大为 1。
  - 若 check_type 缺口未修订，先用 event_count / payload 断言，不得新增未授权 sanity enum。
- `test_p06_does_not_execute_cuc_or_lane_change_command`
  - 断言 P06 没有 `cuc` event、没有 `lane_change_command`、没有 same-step overlay。
  - `MVS-CUC-*` 不被 P06 标为 passed。
- `test_p06_does_not_write_vehicle_state_before_commit`
  - 比较 P06 前后冻结 `S(t)` 签名。
  - 断言真实 `x / y / v / a / lane / merge_state / lane_change_state / aps_assignment_cache` 不变。
- `test_p06_does_not_rerun_aps_or_cmc`
  - 断言 P06 不产生 `APS_candidate`、fresh `APS` assignment、`CMC` Eq.53、boundary speed cap 或 assignment validation event。
  - 若 P06 为 priority 读取 `T*_MV`，来源必须是 upstream assignment / event / validated priority payload；若需要从 frozen state 计算单个 priority scalar，必须显式标注为 `priority_feature_from_frozen_state`，不得写作 fresh APS。

测试配置注意：

```text
1. 不要直接调用尚未登记的 built-in scenario 字符串导致 unknown built-in scenario 红灯。
2. 若 built-in `MVS-CONFLICT-1A/1B` 尚未添加，red tests 应使用 inline ScenarioConfig 或测试 helper 构造 runtime handoff evidence。
3. ScenarioConfig 当前没有 preloaded_effective_assignments 字段；不要暗增该字段。
4. 可使用 preloaded_assignments + Python test helper 生成 EffectiveAssignmentThisStep，或先修订 ScenarioConfig 权威规格；若使用 preloaded_assignments 表达 `MVS-CONFLICT-1B`，必须同步允许 `t_mv_star` / `t_star_mv` 字段，否则只能在 Python helper 中补齐 handoff evidence。
```

## 7. 验收证据

P06 implementation 完成后，必须返回以下证据，而不是只给 pytest 数字。

- Targeted MVS green evidence:
  - `MVS-CONFLICT-1A` targeted green evidence。
  - `MVS-CONFLICT-1B` targeted green evidence。
- cooperative_request event 样例：

```text
event_type = cooperative_request
module = Step5CooperativeRequest
vehicle_id = CV_X
related_vehicle_ids = [MV_A, CV_X]
reason = col_cfv_request / col_clv_request
source = paper_formula 或 first_version_engineering_patch
is_engineering_patch = false for simple request extraction, true only when the source is an engineering fallback
payload:
    request_id
    source_mv_id
    cv_id
    cv_role = clv / cfv
    col = true
    aps_case
    assignment_source
    t_mv_star
    mv_in_merging_zone
    mv_distance_to_x0_m
```

- conflict_resolution event 样例：

```text
event_type = conflict_resolution
module = Step5ConflictResolver
vehicle_id = CV_X
related_vehicle_ids = [CV_X, MV_A, MV_B]
reason = MV_in_merging_zone / smaller_T_star_MV / closer_to_x0_m / deterministic_tie_breaker_after_equal_priority
source = first_version_engineering_patch
is_engineering_patch = true
payload:
    conflict_id
    cv_id
    request_ids
    winner_request_id
    loser_request_ids
    winner_mv_id
    loser_mv_ids
    priority_basis
    priority_values_by_request
    active_request_count_for_cv = 1
    one_active_request_per_cv = true
    conflicting_commands_to_same_CV = false
```

- winner active request 样例：

```text
active_request:
    request_id = p06:step:CV_X:MV_A:cfv
    source_mv_id = MV_A
    cv_id = CV_X
    cv_role = cfv
    active = true
    source_conflict_id = p06:step:conflict:CV_X
```

- loser suppressed / loser trace 样例：

```text
suppressed_request:
    request_id = p06:step:CV_X:MV_B:clv
    source_mv_id = MV_B
    cv_id = CV_X
    active = false
    suppressed_by_request_id = p06:step:CV_X:MV_A:cfv
    suppressed_reason = MV_in_merging_zone / smaller_T_star_MV
    conflict_id = p06:step:conflict:CV_X
```

- priority_basis 样例：

```text
priority_basis:
    ordered_rules = [
        "MV_in_merging_zone",
        "smaller_T_star_MV",
        "closer_to_x0_m",
        "deterministic_tie_breaker"
    ]
    selected_rule = "smaller_T_star_MV"
    values:
        MV_G1.t_mv_star = 5.5
        MV_G2.t_mv_star = 6.0
    deterministic_tie_breaker_key = ["cv_id", "source_mv_id", "request_id"]
```

- engineering_patch 标记样例：

```text
source = first_version_engineering_patch
reason = multi_mv_shared_cv_conflict_resolution
is_engineering_patch = true
```

- relevant sanity check 样例：

```text
check_type = no_write_before_commit
result = pass
reason = p06_no_write_before_commit
payload:
    state_unchanged = true
    aps_assignment_cache_unchanged = true
    p06_outputs_are_derived_only = true
```

如果实现需要 `conflicting_commands_to_same_CV` 或 `one_active_request_per_cv` 作为正式 `SanityCheckType`，必须先修订代码数据结构规格；不能在实现中直接新增。

第一版推荐验收拆分：

| 断言 | 推荐承载位置 |
| --- | --- |
| `cooperative_request` 已生成且每个 active request 带 `request_id/source_mv_id/cv_id/cv_role/col` | `expected_events` 匹配 event payload。 |
| 同一 `cv_id` 最多一个 active request | `conflict_resolution` payload 的 `active_request_count_for_cv = 1`、`one_active_request_per_cv = true`，必要时加 `expected_event_counts`。 |
| loser request 没有静默丢失 | `conflict_resolution` payload 的 `loser_request_ids` / `suppressed_reason`。 |
| P06 不执行 CUC / lane-change / CMC / fresh APS | `forbidden_events` 和 command buffer inspection。 |
| P06 不写冻结 `S(t)` / cache | 现有 `no_write_before_commit` sanity 或状态签名对比。 |
| `conflicting_commands_to_same_CV = false` | event payload 语义键；不是第一版 `SanityCheckType`。 |

- P06 PNG marker / expected_png_features 样例：

```text
cooperative_request_marker visible
conflict_group_marker visible
active_request_marker visible
suppressed_request_marker visible
```

- P06 没有重做 APS 的证据：
  - P06 不产生 `APS_candidate` event。
  - P06 不产生 fresh APS assignment event。
  - P06 不更新 APS cache。
  - P06 只引用 P04 assignment source / event / handoff evidence。
- P06 没有重做 CMC 的证据：
  - P06 不产生 Eq.52 / Eq.53 / boundary speed cap event。
  - P06 不产生 assignment validation event。
  - P06 只读取 P05 validation evidence。
- P06 没有执行 CUC / lane-change command 的证据：
  - P06 不产生 CUC utility event。
  - P06 不生成 lane-change command。
  - P06 不生成 same-step overlay。
- no-write-before-commit 证据：
  - P06 前后冻结 `S(t)` 签名一致。
  - P06 输出只存在于本步派生结构、CommandBuffer.cooperation_commands 或 event / sanity / marker 中。
- Regression evidence:
  - P00 static traceability green。
  - P01-P05 targeted / regression green。
  - P06 targeted green。

## 8. 完成标准

P06 进入完成状态时必须满足：

- `MVS-CONFLICT-1A` required gate 通过。
- `MVS-CONFLICT-1B` required gate 通过。
- P06 能从有效 assignment 中收集 `col = 1` request。
- P06 能从 P04 effective assignment / cache reused evidence 中收集 request。
- P06 能尊重 P05 assignment validation evidence：valid 可继续，invalid 被过滤。
- failed / invalid / empty assignment 不产生 active request。
- `col_clv=false` / `col_cfv=false` 不产生对应 CLV / CFV request。
- 同一 CV 被多个 MV 请求时，只输出一个 active winner。
- loser request 有 trace，不能静默丢失。
- priority basis 可观测，至少覆盖 `MV_in_merging_zone` 与 `smaller_T_star_MV`。
- conflict resolution 明确标记为 first-version engineering patch。
- P06 不重做 APS。
- P06 不重做 CMC。
- P06 不执行 CUC。
- P06 不判断 compliance。
- P06 不生成 lane-change command。
- P06 不生成 same-step overlay。
- P06 不生成 longitudinal / lateral candidate。
- P06 不直接写真实车辆状态。
- P06 不直接修改 APS cache。
- cooperative_request event、conflict_resolution event、sanity、expected_png_features 不等待 P11 才补。
- required / probe / deferred 语义仍由 P01 runner 保持。
- P00-P05 回归保持绿色。
- 若新增字段、enum、ScenarioConfig、EventRecord、SanityCheckRecord 或 expected_* 口径，已有上游规格先行修订并可追溯。

## 9. 回归保护

- 所有算法内部使用 `x_global`；`x_plot` 只用于 PNG / renderer 派生层。
- 所有模块只读冻结 `S(t)`。
- command / next-state 不反写真实状态。
- commit 是唯一生成 `S(t+dt)` 的阶段。
- P06 只处理 Step 5 cooperative request 汇总与多 MV / CV conflict resolution。
- P06 不得重算 APS，不得重选 CLV / CFV。
- P06 不得重算 CMC，不得重判 Eq.53。
- P06 不得把 P05 invalid assignment 恢复为 active request。
- P06 不得把 loser request 静默丢弃。
- P06 不得输出同一 CV 的多个 active request。
- P06 不得执行 CUC utility、compliance、target lane safety、lane-change command 或 same-step overlay。
- P06 不得直接修改 `VehicleState`、`SimulationState`、APS cache 或 active maneuver state。
- 多 MV 共享 CV 仲裁必须继续标记为工程补丁，不得写成论文原算法。
- `MVS-CONFLICT-1A/1B` 是 P06 required gate；`MVS-CUC-*`、`MVS-E2E-1`、`MVS-COMMIT-1-full` 不得被 P06 误报为 full pass。
- P06 event / sanity / PNG feature 不得等待 P11 才补。
- P06 必须保留 P04 targeted APS gate 行为，不破坏 `MVS-APS-*`。
- P06 必须保留 P05 targeted CMC / assignment validation 行为，不破坏 `MVS-CMC-*`、`MVS-ASSIGN-1`。
- 若后续实现需要扩展 ScenarioConfig 以直接加载 effective assignments 或 conflict inputs，必须先修订 `CORMC代码数据结构设计_整理版.md` 与 MVS 文档，再实现代码。
