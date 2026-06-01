# P04 - Step 4A APS / Cache / Effective Assignment（修订版）


> 修订说明：本版已从 P04 spec / failing-test 准备口径，统一为 P04 red-before-green implementation 口径。本轮必须先新增 failing tests 并确认 matcher 层红灯，然后允许同轮实现 P04 Step 4A 最小 APS，使 P04 required MVS targeted gate 转绿；不得越界实现 P05-P12。

## 0. Slice Identity

- Algorithm-Step Coverage:
  - Primary Step: Step 4A，MV 未进入 merging zone 时的 APS 分支。
  - Secondary Steps: APS trigger resolver、APS candidate collector、`T*_MV` 预测、CLV / CFV selector、APS case classifier、`col_CLV / col_CFV`、Eq.10 desired spacing command source、assignment cache action、`EffectiveAssignmentThisStep` 或等价本步有效 assignment、APS event / sanity / PNG feature 证据。
- MVS Acceptance Gate:
  - required:
    - `MVS-APS-FAIL-EMPTY`
    - `MVS-APS-FAIL-CACHE`
    - `MVS-APS-1`
    - `MVS-APS-2`
    - `MVS-APS-3`
    - `MVS-APS-4`
  - probe:
    - APS 边界情况诊断可继续细分，例如无插入对、全部 `D*` 为正、全部 `D*` 为负、`v_MV` 接近 0。
  - deferred:
    - CMC Eq.53。
    - cooperative request conflict resolution。
    - CUC utility。
    - 纵向 / 横向模型。
    - 正式 PNG renderer 与 artifact record。
- 本阶段解锁的能力:
  - 让 Step 4A 可在 targeted MVS 中从 missing APS event 的红灯合同，推进到可验证的 APS assignment / failure / cache 行为。
  - 固定 APS 只读取冻结 `S(t)` 与 Step 3 relations / P02 geometry resolver。
  - 固定供 P05 Step4B assignment validation / CMC 分支消费、并供 P06 Step5 cooperative request 抽取使用的本步有效 assignment 表达，不让后续 CMC / CUC 偷重算 APS。
  - 固定 Eq.10 在 P04 只作为 desired spacing command source / assignment payload，不在本阶段消费纵向模型。
- 本阶段不要求通过的后续场景:
  - 不要求 `MVS-E2E-1` 通过。
  - 不要求 `MVS-CMC-*`、`MVS-CUC-*`、`MVS-CONFLICT-*`、`MVS-SAFE-*` 通过。
  - 不要求真实 PNG 文件存在。

## 1. 本阶段目标

P04 聚焦一次 `S(t) -> Step 4A APS output` 的算法切片：当 on-ramp MV 尚未进入 merging zone，即 `x_MV_global < x0_m_global` 且 `merge_state != executing` 时，本阶段负责判断本步是否执行 APS、是否沿用 cache，并产出本步可追溯的 APS 结果。

本阶段覆盖以下行为：

1. APS trigger resolver:
   - `first_APS`
   - `APS_due`
   - `reuse_cache`
2. APS candidate collector:
   - 只使用 P02 的 `x_global` / `L_cr` candidate window。
3. `T*_MV` 与候选车辆预测。
4. CLV / CFV selector。
5. APS case classifier。
6. `col_CLV` / `col_CFV`。
7. case 2 / 4 中 CFV 的 Eq.10 desired spacing command source。
8. APS failure reason。
9. assignment cache update / retain / invalid / cleanup request。
10. `EffectiveAssignmentThisStep` 或等价本步有效 assignment 结构。
11. APS trigger / assignment / failure / cache action event。
12. targeted MVS matcher 需要的 event / sanity / PNG feature。

P04 不提交真实车辆状态。它只写 command、cache update request、event candidate、sanity check 或本步 effective assignment。若 P04 输出需要在后续步骤改变车辆真实状态，必须等 Step 9 commit。

P04 必须复用 P01-P03 的能力：

```text
P01:
    ScenarioConfig loader、expected_events、forbidden_events、expected_event_counts、
    expected_sanity_checks、expected_png_features matcher、required / probe / deferred report。

P02:
    冻结 S(t)、relations snapshot、lane ordering、region resolver、APS candidate window resolver。

P03:
    CommandBuffer / NextStateBuffer 边界、EventRecord / SanityCheckRecord / OutputHistory、
    commit 唯一真实写入点。
```

## 2. 非目标 / 禁止事项

- 不实现 CMC Eq.53。
- 不实现 boundary speed cap。
- 不实现 cooperative request conflict。
- 不实现 CUC utility。
- 不实现 lane-change command。
- 不实现 longitudinal / lateral candidate。
- 不实现 CAV / IDM 纵向模型消费 Eq.10。
- 不实现正弦横向轨迹。
- 不直接 commit 车辆真实状态。
- 不新增字段；如缺字段，先提出对 `docs/复现讨论/CORMC代码数据结构设计_整理版.md` 的修订建议，不得在实现中暗增。
- 不重新定义道路几何、merging zone、candidate window、lane centerline 或参数值。
- 不使用 `x_plot` 参与 APS candidate、排序、预测、case 判断或 cache。
- 不用 fixed cooperative zone 或 dynamic cooperative window 替代 P02 的 `L_cr` candidate window。
- 不把 `first_APS(MV)` 写成论文原公式；它必须标记为第一版工程补丁。
- 不把 APS failure 伪装成 valid assignment。
- 不在 APS failure 时用失败结果静默覆盖旧 cache。
- 不让 P04 failing tests 因 loader 崩溃、字段缺失、enum 不一致或自然语言断言失败。

## 3. 上游 Spec 引用

- `docs/复现讨论/CORMC时间步执行顺序梳理.md`：
  - Step 4 分流：MV 未进入 merging zone 时执行 APS；MV 已进入 merging zone 时执行 CMC。
  - 所有模块只读冻结 `S(t)`，只写 command / next-state，最后统一 commit。
  - 非 APS 周期沿用上一轮 assignment cache。
- `docs/复现讨论/CORMC论文公式与实现映射.md`：
  - APS candidate set、`T*_MV`、候选预测、CLV / CFV 选择、APS case 1-4、Eq.10 的论文依据。
  - `first_APS(MV)` 是工程补丁，不是论文原公式。
  - 第一版简化 / 第一版关闭 / 工程补丁的分类边界。
- `docs/复现讨论/CORMC代码数据结构设计_整理版.md`：
  - `APSAssignment`、`EffectiveAssignmentThisStep`、`CommandBuffer`、`CacheUpdateCommand`、`EventRecord`、`SanityCheckRecord`、ScenarioConfig、expected_* 字段权威。
  - CAV 的 `compliance_state` 应为 `not_applicable`，CHV 才使用 `compliant / non_compliant`。
  - 如果 P04 发现缺字段，先修订本文档，不在代码里暗增。
- `docs/复现讨论/CORMC输出指标与日志验证规格_整理版.md`：
  - P04 必须产生 APS trigger、assignment、failure、cache action、Eq.10 command source 等 event / sanity / PNG feature 证据。
  - 日志和 sanity 不得反向改变车辆运动。
  - `x_plot` 只能用于 PNG / renderer 派生层。
- `docs/复现讨论/CORMC最小验证场景执行规格.md`：
  - P04 required MVS：`MVS-APS-FAIL-EMPTY`、`MVS-APS-FAIL-CACHE`、`MVS-APS-1`、`MVS-APS-2`、`MVS-APS-3`、`MVS-APS-4`。
  - 每个 APS 场景的 setup、key numeric derivation、expected_events、expected_sanity_checks、expected_png_features。
- `docs/复现讨论/CORMC道路几何与区域规格.md`：
  - `x_global` 是算法内部唯一纵向坐标。
  - `x_plot = x_global - warmup_length` 只用于绘图。
  - `x0_m_global = 6950 m`、`x_ramp_end_global = 7250 m`。
  - APS candidate window 使用 `[x_MV_global - L_cr, x_MV_global + L_cr]`。
- `docs/复现讨论/CORMC参数规格.md`：
  - `L_cr = 300 m`、`T_APS = 5 s`、`dt = 0.1 s`、`L = 4 m`、`g_min_CM = 1.2 s` 等参数来源、单位和数值。
- `docs/复现讨论/CORMC车辆模型规格.md`：
  - Eq.10 只用于 case 2 / 4 中需要协同的 CFV。
  - P04 不实现纵向 / 横向模型，只标记 desired spacing command source。
- `docs/复现讨论/CORMC状态与模块接口规格.md`：
  - 模块读写边界、assignment cache、command / next-state / state transition / cache update request 语义。
  - `EffectiveAssignmentThisStep` 是本步派生结果，不跨步持久化。
- `docs/执行计划/P00-Spec宪法_权威边界与二维追踪矩阵.md`：
  - P04 已完成 trace_registered、完整 P04 spec 与 failing tests 准备；当前执行允许进入 P04 red-before-green implementation，但必须先写失败测试并保留 matcher 层红灯证据。
  - P04 implementation_ready 只限 Step 4A APS / cache / effective assignment，不得扩展到 P05-P12；P05-P07 仍不得被误写成 implementation_ready。
- `docs/执行计划/P01-MVS_Runner_ScenarioConfig与验收断言语言.md`：
  - P04 failing tests 应失败在 matcher 层。
- `docs/执行计划/P02-Step0-3_清理冻结关系与几何口径.md`：
  - P04 必须复用 P02 的 freeze、relations、region resolver 和 APS candidate window resolver。
- `docs/执行计划/P03-Step9-10_Command_NextState_Commit_Event_Sanity_Trajectory闭环.md`：
  - P04 不得直接提交车辆真实状态。
  - event / sanity / trajectory 的记录边界由 P03 收口。

总纲资料记录：

```text
docs/执行计划/CORMC执行计划spec设计总纲v1.md
    本 P04 使用该总纲的统一模板、P04-P10 分层日志规则和 P11 交付级收口边界；
    不自行补参数或公式。
```

## 4. 行为契约 Given / When / Then

- Given：冻结 `S(t)` 中 MV 为 on-ramp vehicle，`x_MV_global < x0_m_global`，`merge_state != executing`。When：Step 4A 调度该 MV。Then：进入 APS branch；不得执行 CMC Eq.53 或横向合流。
- Given：MV 已进入 merging zone，或 `merge_state == executing`。When：Step 4A APS runner 遇到该 MV。Then：P04 不执行 APS，不修改 cache，不产生新的 APS assignment；该 MV hand off 给 P05 CMC branch，必要时记录 `aps_not_applicable` / `handed_off_to_cmc` event 或 sanity `not_applicable`，供调度链路审计。
- Given：MV 首次进入 APS 适用阶段且无 cache。When：执行 trigger resolver。Then：trigger reason 为 `first_APS`，并记录 `source=first_version_engineering_patch`、`reason=first_aps`、`is_engineering_patch=true` 或等价字段。
- Given：MV 已有 `last_update_t` 且 `t - last_update_t >= T_APS`。When：执行 trigger resolver。Then：trigger reason 为 `APS_due`，允许执行 APS 并请求更新 cache。
- Given：MV 已有 valid cache 且未到 APS 周期。When：执行 trigger resolver。Then：trigger reason 为 `reuse_cache`，不重新计算 APS；本步 effective assignment 来源为 cache。
- Given：MV 需要执行 APS。When：收集候选车辆。Then：只从 lane 2 中选择 `x_global` 位于 `[x_MV_global - L_cr, x_MV_global + L_cr]` 的车辆；不得使用 `x_plot`、fixed cooperative zone 或 dynamic cooperative window。
- Given：候选车辆少于两个，且无可沿用 cache。When：执行 APS。Then：产生 APS failure event，`failure_reason=insufficient_candidates`；不创建 CLV / CFV assignment；不产生 cooperative request；PNG feature 注册 `aps_failure_marker` 与 no assignment arrow。
- Given：候选车辆少于两个，但存在旧 cache。When：执行 APS。Then：产生 APS failure event；不得用失败结果静默覆盖旧 cache；cache action 必须显式记录 retain / stale / invalid 策略来源；不得从失败结果产生新 cooperative request。
- Given：候选车辆充足。When：计算 `T*_MV`。Then：使用 `T*_MV = (x0_m_global - x_MV_global) / v_MV` 或上游公式语义；若 `v_MV` 接近 0，应记录 APS failure reason，不进行除零。
- Given：`T*_MV` 已计算。When：预测候选 lane 2 车辆位置。Then：使用冻结 `S(t)` 中候选车辆的 `x_global` / `v` 进行预测，不读取中途 next-state。
- Given：候选预测位置形成插入间隙。When：选择 CLV / CFV。Then：按 APS 正文 / Algorithm 1 语义选择 `D*_j > 0` 且 `D*_{j+1} < 0` 的 pair；若不存在可用 pair，应记录 failure reason。
- Given：CLV / CFV 已选出。When：case classifier 执行。Then：按 Eq.7-Eq.9 与 `D_min_CLV` / `D_min_CFV` 判定 case 1-4。
- Given：case 1。When：写 APS assignment。Then：`col_CLV=false`、`col_CFV=false`，不产生 Eq.10 desired spacing source。
- Given：case 2。When：写 APS assignment。Then：`col_CLV=false`、`col_CFV=true`，Eq.10 desired spacing source 只绑定 CFV。
- Given：case 3。When：写 APS assignment。Then：`col_CLV=true`、`col_CFV=false`，不得给 CLV 套用 Eq.10。
- Given：case 4。When：写 APS assignment。Then：`col_CLV=true`、`col_CFV=true`，Eq.10 desired spacing source 只绑定 CFV。
- Given：APS 成功。When：生成 cache action。Then：写入 cache update request 或等价 candidate，供 commit 或后续状态更新边界处理；不得直接改 `S(t).aps_assignment_cache`。
- Given：APS 本步成功或 cache 被沿用。When：Step 4A 输出本步结果。Then：生成 `EffectiveAssignmentThisStep` 或等价结构，标记 source 为 `aps_updated_this_step` 或 `cache_reused`，并标记是否可进入 cooperative request 汇总。
- Given：P01 matcher 消费 P04 输出。When：缺少 APS event、sanity 或 PNG feature。Then：测试失败原因必须是 matcher 层 missing event / missing sanity / feature registration，不得是 loader 崩溃或字段缺失。
- Given：Step 4A 完成。When：比较冻结 `S(t)`。Then：车辆真实 `x / y / v / a / lane / merge_state` 未被 P04 改写。

## 5. 允许实现的代码对象

当前执行已进入 P04 red-before-green implementation 阶段。本阶段必须先新增 P04 failing tests / test skeleton，并确认红灯发生在 expected event / sanity / matcher 层；随后允许在同一轮中实现以下 P04 Step 4A 最小 APS 对象，使 required MVS targeted gate 转绿。

本阶段允许实现的对象仅限 P04 Step 4A，不得越界实现 P05-P12。

- domain / state objects:
  - 复用 `APSAssignment` 概念。
  - 复用 `EffectiveAssignmentThisStep` 概念。
  - 复用 P02 `SimulationState`、`RelationsSnapshot`、`APSCandidateWindowResult`。
- command / next-state objects:
  - 复用 `CommandBuffer`。
  - 复用 cache update request / `CandidateCacheUpdate` 或等价上游已定义结构。
  - 可写 Eq.10 desired spacing command source，但不消费纵向模型。
- step runner / service functions:
  - `resolve_aps_trigger`
  - `collect_aps_candidates`
  - `compute_t_star_mv`
  - `predict_aps_candidate_positions`
  - `select_clv_cfv`
  - `classify_aps_case`
  - `build_aps_assignment`
  - `build_effective_assignment_this_step`
  - `build_aps_cache_action`
  - `run_step4a_aps`
- event / sanity helpers:
  - `emit_aps_trigger_event`
  - `emit_aps_assignment_event`
  - `emit_aps_failure_event`
  - `emit_assignment_cache_event`
  - `run_aps_assignment_sanity`
  - `register_aps_png_features`
- scenario tests:
  - `test_mvs_aps_fail_empty_p04_required_fails_until_aps_implemented`
  - `test_mvs_aps_fail_cache_p04_required_fails_until_aps_implemented`
  - `test_mvs_aps_case_1_contract`
  - `test_mvs_aps_case_2_eq10_to_cfv_only_contract`
  - `test_mvs_aps_case_3_no_eq10_to_clv_contract`
  - `test_mvs_aps_case_4_eq10_to_cfv_only_contract`
  - `test_first_aps_event_marked_engineering_patch`
  - `test_non_aps_period_reuses_cache_without_mutating_state`
  - `test_p04_does_not_write_vehicle_state_before_commit`

若以上对象需要字段超出 `CORMC代码数据结构设计_整理版.md`，必须先修订该上游规格。

## 6. 先写失败测试

本阶段必须先新增 P04 failing tests / test skeleton，并先运行一次确认 red phase。red phase 的失败必须发生在 expected event / sanity / matcher 层，不得是 loader 崩溃、字段缺失、enum 不一致、ImportError、AttributeError 或自然语言断言。

确认 red phase 后，本轮应立即进入 P04 Step 4A 最小 APS implementation，使 P04 required targeted tests 转绿，并保留 red-before-green 摘要。

测试治理规则：

```text
1. 若测试本身是“真实红灯测试”，即 pytest 失败用于表示 P04 APS 尚未实现，则必须使用
   pytest.mark.xfail(strict=True, reason="P04 APS not implemented")，或放入单独 red-test 命令。
2. 若测试是“合同测试”，即断言 runner/report 当前应返回 required_failed 且 failure reason
   停在 expected event / sanity matcher 层，则该测试应在主线 `python -m pytest -q` 中保持绿色。
3. 主线全量测试不得因为 P04 尚未实现而变红，除非本轮目标明确是 red phase。
4. 如果新增 sanity check 名称尚未出现在 `CORMC代码数据结构设计_整理版.md` 的
   `SanityCheckType` 权威集合中，只能作为合同测试的 expected matcher 需求或 spec 提案；
   进入实现前必须先修订代码数据结构规格。
```

测试清单：

- `test_mvs_aps_fail_empty_p04_required_fails_until_aps_implemented`
  - 加载 `MVS-APS-FAIL-EMPTY`。
  - 运行 targeted scenario。
  - 期望 report 为 `required_failed`，且 failure reason 包含 missing expected APS event / missing sanity，而不是 loader error。
- `test_mvs_aps_fail_cache_p04_required_fails_until_aps_implemented`
  - 使用等价 ScenarioConfig 表达 `MVS-APS-FAIL-CACHE`。
  - 期望缺少 APS failure / cache action event。
- `test_mvs_aps_case_1_contract`
  - 使用等价 ScenarioConfig 表达 `MVS-APS-1`。
  - 期望缺少 APS assignment event，expected match 包含 `case_1`、`col_clv=false`、`col_cfv=false`。
- `test_mvs_aps_case_2_eq10_to_cfv_only_contract`
  - 使用等价 ScenarioConfig 表达 `MVS-APS-2`。
  - 期望缺少 APS assignment / Eq.10 source event，expected match 包含 `case_2`、`col_cfv=true`、`eq10_vehicle_role=cfv`、`desired_spacing_override=58`。
- `test_mvs_aps_case_3_no_eq10_to_clv_contract`
  - 使用等价 ScenarioConfig 表达 `MVS-APS-3`。
  - 期望 forbidden event 可表达 `eq10_vehicle_role=clv`，且缺少 APS assignment event。
- `test_mvs_aps_case_4_eq10_to_cfv_only_contract`
  - 使用等价 ScenarioConfig 表达 `MVS-APS-4`。
  - 期望缺少 APS assignment / Eq.10 source event，expected match 包含 `desired_spacing_override=52`。
- `test_first_aps_event_marked_engineering_patch`
  - expected event 要求 `event_type=engineering_patch` 或 APS trigger event，`source=first_version_engineering_patch`，`reason_code=first_aps`。
- `test_non_aps_period_reuses_cache_without_mutating_state`
  - expected event 要求 `trigger=reuse_cache`、`effective_assignment_source=cache_reused`。
  - 同时断言 P04 未修改冻结状态；现阶段不应因 APS 未实现而出现状态突变。
- `test_p04_does_not_write_vehicle_state_before_commit`
  - 只使用 P02 freeze / P01 runner 边界验证 P04 red test 不会提交状态。
  - 可通过状态签名比较确保当前 skeleton 未实现状态写入。
- `test_p04_hands_off_merging_zone_or_executing_mv_to_cmc_contract`
  - 使用已进入 merging zone 或 `merge_state == executing` 的 MV。
  - expected event 要求 `aps_not_applicable` / `handed_off_to_cmc` 或等价 handoff 证据。
  - 断言 P04 不修改 cache、不创建 APS assignment，并把该 MV 交给 P05 CMC branch。

这些测试必须避免：

```text
1. 因未知 ScenarioConfig 字段失败。
2. 因 enum 不一致失败。
3. 因 loader 缺内置场景失败而掩盖 matcher red。
4. 使用自然语言断言替代 expected_* matcher。
```

## 7. 验收证据

P04 implementation 完成后应提供以下证据。red phase 阶段仅新增 failing tests / test skeleton；进入 green phase 后，本轮允许实现 P04 Step4A 最小 APS，并补齐以下验收证据：

- EventRecord / event dict:
  - APS trigger event：`first_APS` / `APS_due` / `reuse_cache`。
  - APS candidate event：candidate window、candidate ids、使用 `x_global` 与 `L_cr`。
  - APS assignment event：MV、CLV、CFV、case、`col_CLV`、`col_CFV`、`T*_MV`。
  - APS failure event：failure reason，例如 `insufficient_candidates`。
  - cache action event：update / retain / invalid / cleanup request。
  - Eq.10 command source event：case 2 / 4 只绑定 CFV。
- SanityCheckRecord / sanity dict:
  - `assignment_invalid=not_applicable/pass/warning` 按场景语义。
  - `assignment_cache_overwrite_by_failed_APS=false`，若该 check type 尚未进入代码数据结构规格，则先修订规格。
  - `Eq10_applied_to_wrong_vehicle=false`，若该 check type 尚未进入代码数据结构规格，则先修订规格。
  - `x_plot_used_in_algorithm_path=pass`。
  - `p04_no_write_before_commit=pass` 或复用 P03 no write guard；若新增 P04 专用 check type，则先修订规格。
  - `aps_not_applicable` / `handed_off_to_cmc` 可作为 event 或 sanity `not_applicable` 表达；进入实现前必须做字段权威检查。
- PNG feature:
  - `aps_failure_marker`
  - `assignment_arrow` visible / not_visible
  - `aps_assignment_marker`
  - `eq10_spacing_marker`
  - `cache_reuse_marker`
- Scenario matcher report:
  - required 场景通过时，expected_events / forbidden_events / expected_event_counts / expected_sanity_checks / expected_png_features 均由 P01 matcher 消费。
  - failing tests 初始红灯应报告 missing expected event / sanity，而非 loader error。
- 工程补丁追踪:
  - `first_APS(MV)` 事件必须保留 `source` / `reason` / `is_engineering_patch`。
  - failure cache retain / stale / invalid 策略如果属于工程兜底，也必须保留来源标记。

## 8. 完成标准

P04 进入完成状态时必须满足：

- `MVS-APS-FAIL-EMPTY` required gate 通过。
- `MVS-APS-FAIL-CACHE` required gate 通过。
- `MVS-APS-1` / `2` / `3` / `4` required gate 通过。
- APS candidate collector 只使用 P02 `x_global` / `L_cr` candidate window。
- `T*_MV`、候选预测、CLV / CFV、case、`col_CLV` / `col_CFV` 可从 event payload 追溯。
- case 2 / 4 中 Eq.10 desired spacing source 只绑定 CFV。
- case 3 不给 CLV 套 Eq.10。
- failure reason 可结构化匹配。
- APS failure 不伪造 assignment。
- APS failure 不用无效新结果静默覆盖旧 cache。
- `EffectiveAssignmentThisStep` 或等价本步有效 assignment 可供 P05 Step4B assignment validation / CMC 分支消费，并供 P06 Step5 cooperative request 抽取使用。
- P04 不直接提交车辆真实状态。
- P04 不实现 CMC、CUC、纵向模型、横向模型或正式 PNG renderer。
- 所有工程补丁保留 `source` / `reason` / `is_engineering_patch`。
- required / probe / deferred 报告语义仍由 P01 runner 保持。

## 9. 回归保护

- 所有算法内部使用 `x_global`；`x_plot` 只在 PNG / renderer 派生层出现。
- P04 只处理 MV 未进入 merging zone 的 APS 分支。
- MV 已进入 merging zone 或 `merge_state == executing` 时，P04 必须 skip APS 并 hand off 给 P05；不得修改 cache 或创建新的 APS assignment。
- APS candidate window 不得被 fixed cooperative zone 或 dynamic cooperative window 替代。
- `first_APS(MV)` 必须继续标注为工程补丁。
- `APS_due` 使用真实时间或等价 step-time 映射，不能使用 `time + 1` 伪时间。
- 非 APS 周期沿用 cache，不重新计算 APS。
- failure event 必须有结构化 `failure_reason`。
- failure 不创建 fake CLV / CFV assignment。
- failure 不静默覆盖旧 cache。
- `EffectiveAssignmentThisStep` 是本步派生，不跨步持久化。
- cache 跨步更新必须走 cache update request / commit 边界。
- P04 event / sanity / PNG feature 不得等 P11 才补。
- P04 不得绕过 P03 的 command / next-state / commit / OutputHistory 边界。
- 若后续新增字段需求，先修订 `CORMC代码数据结构设计_整理版.md`，再实现代码。
