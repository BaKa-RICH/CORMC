# P11 - Step10 Output Export / Formal PNG / Full MVS Regression Closure

> 本文档是 P13.5 校准后的 P11 权威说明。P11 覆盖 Step 10 的交付级收口：导出 P01-P10 已产生的 trajectory / event / sanity / PNG marker evidence，渲染正式 PNG，生成 artifact manifest，聚合全部 required MVS smoke suite，并输出 regression report。P11 不是 P04-P10 算法、日志、sanity、targeted runner 或 PNG marker 的首次实现阶段；P14 才负责把这些基础能力升级为“跑一次仿真自然生成正式输出包”的交付路径。

## 1. Slice Identity

- Algorithm-Step Coverage:
  - Primary Coverage: Step 10 delivery closure: output export, formal PNG rendering, artifact manifest, required MVS smoke aggregation, regression report.
  - Consumed upstream coverage:
    - P01: `ScenarioConfig` loader、matcher、required / probe / deferred 分类、expected event / sanity / PNG matcher v0。
    - P02: frozen `S(t)`、relations snapshot、geometry / lane / region / x_global 口径、renderer 可用几何信息。
    - P03: `CommandBuffer` / `NextStateBuffer` 分离、`CommitResult`、`OutputHistory`、`TrajectoryRecord`、`EventRecord`、`SanityCheckRecord`、Step 10 record-only evidence。
    - P04-P09: APS / CMC / P06 conflict / CUC / longitudinal / lateral 阶段各自产生的 event、sanity、expected_png_features、targeted MVS evidence。
    - P10: P08/P09 component candidate assembly、唯一 commit、真实 `S(t+dt)`、state transition / active maneuver / cache lifecycle、commit event、OutputHistory、renderer-deferred P10 markers。
  - Explicit non-coverage:
    - 不重新实现 APS、CMC、P06 conflict、CUC、P08 longitudinal、P09 lateral、P10 candidate assembly / commit。
    - 不实现 P16 seeded random simulation、random vehicle attributes、P17 paper-level experiment grid、SUMO comparison。

- MVS Acceptance Gate:
  - required:
    - P11 full required MVS smoke aggregation must enumerate the exact first-version target required suite and must fail, not skip, any required scenario that has no registered ScenarioConfig, runner route, or correct required status.
    - The first-version P11 required denominator is the 20 `scenario_id` values listed in section 5.1. Family wildcards such as `MVS-APS-*` or `MVS-SAFE-*` are explanatory shortcuts only and are not acceptable registry evidence.
    - P11 must export trajectory / event / sanity histories for scenarios that produce `OutputHistory` or equivalent actual records.
    - P11 must render formal PNG files from `TrajectoryRecord` / `EventRecord` / `expected_png_features` / road geometry without mutating algorithm state.
    - P11 must produce artifact manifest and regression report for required / probe / deferred groups.
  - probe:
    - PNG visual quality diagnostics.
    - export format diagnostics.
    - smoke suite runtime / completeness diagnostics.
    - `MVS-CUC-1B_real_utility_probe` and any additional diagnostic scenario explicitly marked probe by the upstream MVS index. A final required safety scenario must not be demoted to probe because of future loader drift.
  - deferred:
    - P16 seeded random generation.
    - P17 paper-level experiment grid.
    - SUMO comparison.
    - strict paper-level capacity / aggregate metrics.
    - strict MPC tracking unless upstream specs promote it.

- Current-state statement consumed by P11:
  - P11 exporter / renderer / manifest / regression report 基础能力已经实现，full required MVS smoke aggregation 已经能聚合 required / probe / deferred 分组。
  - P12 deterministic full simulation loop 已完成：固定场景、关闭随机边界车辆生成，Step0-11 可连续推进多时间步，并能生成 demo PNG。
  - P13 required MVS closure 已完成：`MVS-E2E-1`、`MVS-COMMIT-1-full`、CUC required 和 SAFE required 场景均通过 official loader、deterministic runner 和 P11 matcher 进入验收。
  - 当前 suite fact：`suite_status="passed"`，20 required green，`required_failed=[]`，`required_blocked=[]`，`runner_gaps=[]`。
  - 当前 probe 为 `MVS-CUC-1B_real_utility_probe`；当前 deferred 为 `MVS-CUC-1C_real_utility_choice1_locked`。
  - `MVS-SAFE-1A_waiting_cap` 当前是 required green，不再是 probe/status mismatch。
  - P11 与 P14 的边界：P11 提供 exporter / renderer / manifest / regression report 与 suite aggregation 基础；P14 才负责正式 artifact bundle 的自然输出路径，至少包含 trajectory / event / sanity / PNG / manifest / regression report，用作 P15 重构前后对比基线。

## 2. 本阶段目标

P11 的目标是把 P01-P10 已经逐步落地的 in-memory evidence 变成交付级 artifacts。P11 成功后，人工审阅者应能从一个 run output directory 中看到每个 scenario 的 trajectory export、event export、sanity export、formal PNG、artifact manifest entry 和 regression result，并能确认 required / probe / deferred 的 gating 语义正确。

最小目标：

1. 维护 full required MVS smoke suite registry，列出目标 required suite 与当前可执行状态。
2. 对 required / probe / deferred 做分层：
   - required: 任何缺场景、缺 runner route、matcher fail、required sanity fail 都阻塞 suite。
   - probe: 可执行并报告 observation / failure，但不阻塞 required suite。
   - deferred: 明确 skipped / not required，不进入第一版 required pass 条件。
3. 聚合或执行 P01-P10 已有 targeted runner / helper evidence，不在 P11 首次补 P04-P10 event、sanity 或 PNG marker。
4. 导出 `TrajectoryRecord` history，保持 `x_global` 为权威坐标；`x_plot` 只允许 renderer 派生。
5. 导出 `EventRecord` history，保留 `source`、`reason`、`is_engineering_patch`、`payload`。
6. 导出 `SanityCheckRecord` history，并让 required sanity fail 影响 regression report。
7. 使用 trajectory / event / expected_png_features / road geometry 渲染正式 PNG；PNG 是人工复核证据，不反向影响算法。
8. 生成 artifact manifest，记录每个 scenario 的 input config、run id、status、history exports、PNG paths、report paths、pass/fail/gap 状态。
9. 生成 regression report，汇总 required pass/fail、probe observed、deferred skipped、schema gap、runner gap、artifact path。
10. 明确 P11 不等同于 P14；P15 engine consolidation 需要先有 P14 正式 artifact baseline，P16 random generation 和 P17 paper experiment grid 不应被 P11 偷偷引入。

P11 的交付流应形如：

```text
P01-P10 targeted evidence
    + ScenarioConfig registry / matcher expectations
    + OutputHistory / CommitResult / actual event & sanity dicts
    + expected_png_features
    + road geometry and trajectory records
-> P11 smoke suite aggregator
    + required/probe/deferred partition
    + missing-runner / missing-scenario blockers
    + matcher and sanity result collection
-> P11 exporters
    + trajectory export
    + event export
    + sanity export
    + artifact manifest
-> P11 formal PNG renderer
    + x_plot derived only for plotting
    + markers from expected_png_features
-> P11 regression report
    + required suite verdict
    + probe observations
    + deferred skipped
    + gaps / blockers
```

## 3. 非目标 / 禁止事项

P11 不得做以下事情：

- 不重做 P04 APS trigger、candidate selection、effective assignment 或 assignment cache selection。
- 不重做 P05 CMC branch、assignment validation、Eq.52、Eq.53、boundary speed cap、merge-start / waiting decision。
- 不重做 P06 cooperative request collection 或 conflict resolution。
- 不重做 P07 CUC choice、target lane safety、CHV compliance、lane-change command 或 same-step overlay。
- 不重做 P08 longitudinal model、Eq.10 consumption、IDM/CPID、planning speed composition、speed cap composition。
- 不重做 P09 lateral trajectory、sine progress、completion detection、front-collision lateral diagnostic。
- 不重做 P10 candidate assembly、unique commit、state transition、active maneuver lifecycle、cache cleanup。
- 不在 P11 首次生成 P04-P10 本应产生的 event、sanity、targeted MVS evidence 或 expected_png_features marker。
- 不修改 `SimulationState`、`VehicleState`、`OutputHistory` 输入对象或任何算法 record。
- 不让 PNG renderer 的 `x_plot` 回写 trajectory authority、algorithm state 或 matcher input。
- 不把 formal PNG / artifact export 当成算法判断依据。
- 不将缺失 required scenario / runner route 标记为 deferred 或 silent skip。
- 不暗增字段、enum、ScenarioConfig 字段、OutputConfig 字段、OutputArtifactRecord 字段、EventRecord 字段、SanityCheckRecord 字段、TrajectoryRecord 字段、OutputHistory 字段或 expected_* 结构；若 schema 不足，先在 implementation report 中列为 schema gap。
- 不实现 P16 random arrival、random vehicle attributes、seeded random simulation、P17 capacity / aggregate metrics 或 paper-level experiment grid。

## 4. 上游 spec 引用

- `docs/执行计划/CORMC执行计划spec设计总纲v1.md`
  - 引用 P11 定位：交付级日志导出、PNG 渲染、artifact record、全部 required smoke suite 聚合、regression report。
  - 引用分层原则：P04-P10 不得等待 P11 才补 event / sanity / PNG marker；P11 只做聚合和正式 artifact。

- `docs/执行计划/P00-Spec宪法_权威边界与二维追踪矩阵.md`
  - 引用 P11 row: required gate 是全部 required MVS 聚合执行与报告；probe 非阻塞；deferred 不进入第一版强验收。
  - 引用 P11 不是首次实现日志 / sanity / MVS runner / PNG 口径。

- `docs/执行计划/P01-MVS_Runner_ScenarioConfig与验收断言语言.md`
  - 引用 `ScenarioConfig` loader、expected_events、forbidden_events、expected_event_counts、expected_sanity_checks、expected_png_features、tolerances、required / probe / deferred matcher 语义。
  - P11 不重写 P01 matcher 语义，只聚合多个 scenario 的结果。

- `docs/执行计划/P02-Step0-3_清理冻结关系与几何口径.md`
  - 引用 `x_global` 权威、`x_plot` renderer-only、road geometry、lane centerline、merging zone 和 quicklook 所需几何信息。

- `docs/执行计划/P03-Step9-10_Command_NextState_Commit_Event_Sanity_Trajectory闭环.md`
  - 引用 `OutputHistory`、`TrajectoryRecord`、`EventRecord`、`SanityCheckRecord`、commit event、information integration record-only、no-write-before-commit、time advance evidence。
  - P11 消费这些记录并导出，不重新提交状态。

- `docs/执行计划/P04-Step4A_APS_Cache_EffectiveAssignment.md`
  - 引用 APS failure / success cases、cache reuse / invalidation evidence、APS expected PNG marker。
  - P11 只聚合 P04 targeted evidence，不重新选择 APS assignment。

- `docs/执行计划/P05-Step4B_CMC_AssignmentValidation_Eq53_BoundaryCap.md`
  - 引用 CMC required gates、Eq.53 pass/fail、boundary cap、merge-start / waiting command evidence、cache invalidation request。
  - P11 不重新计算 Eq.53 或 boundary cap。

- `docs/执行计划/P06-Step5_CooperativeRequest_ConflictResolution.md`
  - 引用 cooperative request、conflict winner、suppressed request evidence。
  - P11 不重做 active request 或 conflict resolution。

- `docs/执行计划/P07-Step6_CUCChoice_Compliance_LaneChangeCommand_SameStepOverlay.md`
  - 引用 CUC required/probe/deferred cases、lane-change command、same-step overlay、no-CUC-rerun evidence。
  - P13 后 CUC required routes 已通过 official loader / deterministic runner / matcher 接入 P11 required suite；后续不得重新描述为 missing built-in route。

- `docs/执行计划/P08-Step7_LongitudinalModel_Eq10SpacingOverride_SpeedCapComposition.md`
  - 引用 P08 longitudinal candidate、planning speed、constraints_applied、source_commands、PNG markers。
  - P11 不重算 planning speed。

- `docs/执行计划/P09-Step8_LateralTrajectory_PlanningSpeedConsumption_ManeuverProgress.md`
  - 引用 P09 lateral candidate、maneuver progress、completion candidates、boundary risk sanity、PNG markers。
  - P11 不重算 lateral trajectory。

- `docs/执行计划/P10-Step9_CandidateAssembly_Commit_StateTransition_Integration.md`
  - 引用 P10 的 Step4-9 integration responsibility、official `MVS-E2E-1` / `MVS-COMMIT-1-full` deterministic route evidence、OutputHistory evidence、renderer-deferred P10 markers。
  - P13 后 `MVS-E2E-1` 与 `MVS-COMMIT-1-full` 已接入 official runner route；后续不得重新描述为 helper-only 或 full route gap。

- `docs/复现讨论/CORMC时间步执行顺序梳理.md`
  - 引用 Step 10 information integration 和 Step 11 time advance 的流程位置。

- `docs/复现讨论/CORMC状态与模块接口规格.md`
  - 引用 commit 是真实状态写入点，输出 / PNG / report 不反向改变状态。

- `docs/复现讨论/CORMC代码数据结构设计_整理版.md`
  - 引用 `TrajectoryRecord`、`EventRecord`、`SanityCheckRecord`、`OutputHistory`、`ExpectedPNGFeatureSpec`、`OutputConfig`、`OutputArtifactRecord` 的权威边界。
  - 若现有代码未实现 OutputArtifactRecord / OutputConfig，P11 implementation 必须先以 schema gap / spec revision 处理。

- `docs/复现讨论/CORMC输出指标与日志验证规格_整理版.md`
  - 引用 trajectory history、event history、sanity check result、PNG output、smoke acceptance 的通用语义。

- `docs/复现讨论/CORMC最小验证场景执行规格.md`
  - 引用 required / probe / deferred MVS 场景清单、setup、expected_events、expected_sanity_checks、expected_png_features。
  - P11 当前以 20 required green 为事实基线；未来若目标清单与 built-in registry 再出现差异，必须报告为 blocker / gap，而不是静默跳过。

## 5. 行为契约 Given / When / Then

### 5.1 Required suite registry completeness

- Given: P11 target required suite denominator is exactly the first-version 20 IDs from the MVS index and master plan:
  - `MVS-APS-FAIL-EMPTY`
  - `MVS-APS-FAIL-CACHE`
  - `MVS-APS-1`
  - `MVS-APS-2`
  - `MVS-APS-3`
  - `MVS-APS-4`
  - `MVS-E2E-1`
  - `MVS-COMMIT-1-lite`
  - `MVS-CMC-1`
  - `MVS-CMC-2`
  - `MVS-CUC-1A_override_choice1`
  - `MVS-CUC-2`
  - `MVS-CUC-3`
  - `MVS-SAFE-1A_waiting_cap`
  - `MVS-SAFE-1B_executing_cap_lateral_consumption`
  - `MVS-SAFE-2`
  - `MVS-ASSIGN-1`
  - `MVS-CONFLICT-1A`
  - `MVS-CONFLICT-1B`
  - `MVS-COMMIT-1-full`
- When: P11 builds the full smoke suite registry.
- Then: every target required `scenario_id` must be present, loadable and executable in the official deterministic runner.
- Then: registry completeness is checked against the exact 20 IDs above; wildcard family labels are not sufficient evidence.
- Then: `MVS-E2E-1` and `MVS-COMMIT-1-full` are current official required routes; if a future change removes either route, that is a regression blocker.
- Then: `MVS-SAFE-1A_waiting_cap` must remain in required; any future downgrade to probe is a classification regression.
- Then: `P05-EXECUTING-CONTINUATION` may be reported as an extra regression / diagnostic route, but it is not in the 20-ID required pass denominator.

### 5.2 Required / probe / deferred partition

- Given: P01 supports required / probe / deferred status.
- When: P11 aggregates scenario results.
- Then: required failures block the suite.
- Then: probe failures or observations are reported but do not block required suite pass; current MVS probe denominator includes `MVS-CUC-1B_real_utility_probe` and only scenarios explicitly marked probe by upstream authority.
- Then: deferred scenarios are reported as skipped / not required for first-version gate; current MVS deferred denominator includes `MVS-CUC-1C_real_utility_choice1_locked`.
- Then: no scenario in the exact 20-ID required denominator may appear in probe or deferred groups unless an upstream authority spec explicitly revises the denominator.

### 5.3 P10 / P13 closure guard

- Given: P10 has `MVS-E2E-1` chain evidence and P13 has promoted `MVS-E2E-1` / `MVS-COMMIT-1-full` into official deterministic required routes.
- When: P11 evaluates readiness for `MVS-E2E-1`.
- Then: P11 must cite official loader / deterministic runner / matcher evidence, not helper-only evidence.
- Then: P11 must report no runner gap for `MVS-E2E-1` or `MVS-COMMIT-1-full` in the current suite.
- Then: if a future runner change breaks either route, regression report must surface the route loss as a blocker.

### 5.4 Duplicate commit fail-state guard

- Given: P10 duplicate final candidate fixture produces `multiple_commit_for_one_vehicle=fail`.
- When: P11 aggregates required scenario results.
- Then: any duplicate commit sanity fail must block required suite.
- Then: P11 must not pass the resulting missing-vehicle next_state into later scenario steps as a valid state.

### 5.5 Trajectory export

- Given: `OutputHistory.trajectory_records` contains `TrajectoryRecord` values from P03/P10.
- When: P11 exports trajectory history.
- Then: each committed vehicle / step record is exported with `step`、`t`、`vehicle_id`、`x_global`、`y`、`v`、`a`、`physical_lane`、`road_role`、state fields and event tags.
- Then: `x_plot` must not be added to the authoritative trajectory export unless it is explicitly marked as renderer-derived and excluded from algorithm authority.

### 5.6 Event export

- Given: `OutputHistory.event_records` contains `EventRecord` values.
- When: P11 exports event history.
- Then: export preserves `event_id`、`run_id`、`scenario_id`、`step`、`t`、`module`、`event_type`、`vehicle_id`、`source_command_id`、`source_candidate_id`、`reason`、`result`、`is_engineering_patch`、`source`、`payload`.
- Then: P11 may summarize events but must not create missing P04-P10 algorithm events as if they were produced upstream.

### 5.7 Sanity export

- Given: `OutputHistory.sanity_check_records` contains `SanityCheckRecord` values.
- When: P11 exports sanity history and builds regression report.
- Then: required sanity `fail` blocks required suite.
- Then: probe sanity observations are reported but non-blocking unless promoted by upstream spec.

### 5.8 Formal PNG rendering

- Given: trajectory history, event history, expected_png_features and road geometry are available.
- When: P11 renders formal PNG.
- Then: PNG includes lane / region / trajectory / key markers required by expected_png_features.
- Then: visible / optional / not_visible evaluation must be bound to `expected_png_features` marker evidence and renderer-produced feature status, not to subjective statements such as "looks correct".
- Then: the renderer may compute `x_plot = x_global - warmup_length` or equivalent local plotting coordinates only inside renderer scope.
- Then: renderer output is a file artifact and cannot mutate `SimulationState`、`VehicleState`、`TrajectoryRecord` or matcher inputs.

### 5.9 Artifact manifest

- Given: P11 exports files and renders PNGs.
- When: P11 writes artifact manifest.
- Then: each scenario has entries for scenario_id, run_id, status, input config reference, trajectory export path, event export path, sanity export path, PNG path(s), scenario report path, regression report reference, pass/fail/gap status.
- Then: if `OutputArtifactRecord` / manifest schema is not yet authoritative in code, implementation must either introduce it through a proper schema revision or report the gap before writing ad hoc fields.

### 5.10 Regression report

- Given: P11 has all scenario reports, matcher results, exports, PNG render results and gaps.
- When: P11 builds regression report.
- Then: report groups required green / required failed / required blocked / probe observed / deferred skipped.
- Then: report includes fail reasons, missing scenario ids, missing runner routes, schema gaps and artifact paths.
- Then: report is not a substitute for targeted tests; it is a closure artifact.

### 5.11 No state mutation

- Given: committed states and OutputHistory are passed to P11 exporters / renderer / report builder.
- When: P11 executes export, render and report.
- Then: input signatures are unchanged before and after P11.
- Then: P11 writes files and manifest only.

### 5.12 P16 / P17 boundary

- Given: P16 owns random boundary generation / random vehicle attributes / seeded random simulation, and P17 owns paper-level experiments.
- When: P11 builds smoke suite and artifacts.
- Then: random arrival and random vehicle attributes remain disabled for required smoke.
- Then: capacity / aggregate metric outputs are not required P11 pass conditions.

## 6. 代码对象边界

P11 已落地或后续维护时只允许触碰 delivery-layer objects。允许边界包括：

- smoke aggregation:
  - `SmokeSuiteRegistry`
  - `SmokeSuiteRunResult`
  - `ScenarioAggregationResult`
  - `RequiredScenarioBlocker`
  - `ProbeObservation`
  - `DeferredScenarioRecord`
  - `run_full_required_mvs_smoke_suite(...)`
  - `aggregate_targeted_scenario_reports(...)`

- export helpers:
  - `export_trajectory_history(...)`
  - `export_event_history(...)`
  - `export_sanity_history(...)`
  - `serialize_trajectory_record(...)`
  - `serialize_event_record(...)`
  - `serialize_sanity_record(...)`

- PNG renderer:
  - `render_time_space_png(...)`
  - `render_expected_png_features(...)`
  - `derive_x_plot_for_renderer(...)`
  - `render_lane_and_region_guides(...)`
  - `render_event_markers(...)`

- artifact / report:
  - `ArtifactManifest`
  - `ArtifactManifestEntry`
  - `OutputArtifactRecord` if first revised in the authoritative data-structure spec.
  - `RegressionReport`
  - `ScenarioArtifactBundle`
  - `write_artifact_manifest(...)`
  - `write_regression_report(...)`

- tests:
  - P11 targeted tests for registry completeness, required/probe/deferred semantics, export, PNG rendering, artifact manifest, regression report, x_plot boundary and no-state-mutation.

P11 must not add algorithm objects for APS / CMC / P06 / CUC / P08 / P09 / P10. If existing `ScenarioConfig`、`OutputConfig`、`OutputArtifactRecord` or expected_* schema becomes insufficient in a future change, the regression should surface a structured schema / contract gap, not an unknown field / loader crash.

## 7. 回归测试语义

历史 red-before-green implementation 已完成。后续维护 P11 时仍需保持以下回归顺序和失败语义：

1. Add or update P11 targeted tests before changing P11 behavior.
2. Run P11 targeted tests and confirm failures, if any, occur in smoke suite aggregation, export, PNG rendering, artifact manifest or regression report contract layers.
3. Failures must not be ImportError, AttributeError, unknown enum, unknown field, loader crash, or natural-language assertion.
4. Keep implementation changes within P11 export / renderer / suite aggregation / report boundaries.
5. Run P11 targeted green tests and P12 deterministic loop regression.
6. Return evidence with examples and suite summary, not only pytest counts.

Required test plan:

- `test_p11_required_mvs_registry_contains_target_suite`
  - Asserts the exact 20 target required ids from section 5.1 are enumerated.
  - Asserts family wildcards / ellipsis are not accepted as registry completeness evidence.
  - Any future missing built-in required route is reported as a blocker, not skipped.

- `test_p11_safe_1a_waiting_cap_is_required_not_probe`
  - Asserts `MVS-SAFE-1A_waiting_cap` is in the required denominator.
  - Asserts current loader / runner / matcher classification keeps `MVS-SAFE-1A_waiting_cap` required green, not probe.

- `test_p11_p05_executing_continuation_is_extra_diagnostic_not_required_denominator`
  - Asserts `P05-EXECUTING-CONTINUATION` may appear in extra regression / diagnostic output.
  - Asserts it is excluded from required pass/fail/blocked denominator counts.

- `test_p11_required_smoke_suite_blocks_on_required_failure`
  - Constructs one passing required result and one failing required result.
  - Asserts suite status is fail and fail reason is preserved.

- `test_p11_probe_results_are_reported_but_non_blocking`
  - Asserts probe failure appears in report but does not block required suite.

- `test_p11_deferred_scenarios_are_skipped_not_required`
  - Asserts deferred ids appear as skipped / deferred and not in required pass denominator.

- `test_p11_exports_trajectory_history_for_committed_vehicles`
  - Uses a P03/P10 `OutputHistory`.
  - Asserts exported trajectory records include committed vehicle states and no authoritative `x_plot`.

- `test_p11_exports_event_history_with_source_reason_patch_payload`
  - Asserts event export preserves `source`、`reason`、`is_engineering_patch`、`payload`.

- `test_p11_exports_sanity_history_and_required_fail_affects_report`
  - Asserts sanity fail is exported and blocks required report when required.

- `test_p11_formal_png_renderer_consumes_expected_features`
  - Uses trajectory / event / expected_png_features and road geometry.
  - Asserts PNG file is created and manifest lists expected markers as visible / optional / not_visible per spec.
  - Asserts PNG feature pass/fail is bound to expected_png_features evidence, not subjective visual text.

- `test_p11_x_plot_is_renderer_derived_only`
  - Asserts renderer may use `x_plot`, but `SimulationState` and `TrajectoryRecord` authority remain `x_global`.

- `test_p11_artifact_manifest_records_each_scenario_bundle`
  - Asserts manifest records input config, trajectory/event/sanity exports, PNG paths, report path and status.

- `test_p11_regression_report_groups_required_probe_deferred_and_gaps`
  - Asserts report includes required pass/fail, probe observed, deferred skipped, runner gap and schema gap.

- `test_p11_export_png_report_do_not_mutate_state_or_history`
  - Compares signatures of input committed state and OutputHistory before/after P11.

- `test_p11_p10_p13_closure_guard_keeps_e2e_official_route`
  - Asserts official `MVS-E2E-1` deterministic route can be executed through loader / runner / matcher.
  - Asserts `MVS-E2E-1` and `MVS-COMMIT-1-full` do not appear in `runner_gaps` in the current required suite.

- `test_p11_duplicate_commit_sanity_blocks_runner_continuation`
  - Asserts `multiple_commit_for_one_vehicle=fail` blocks continuation rather than feeding missing-vehicle state to later steps.

- `test_p11_does_not_enable_p12_random_or_paper_metrics`
  - Asserts random arrival, random vehicle attributes and paper-level metrics are absent from first-version required pass criteria.

Static / matcher tests:

- P00 traceability must continue to assert P11 is delivery aggregation and not first log / sanity / MVS / PNG stage.
- P01 matcher semantics must remain unchanged; P11 aggregates matcher results rather than redefining them.
- If P11 adds artifact manifest schema, authoritative data-structure docs and code must be revised together.

## 8. 验收证据

P11 completion evidence includes at least:

- Full required MVS suite report sample:

```text
required:
  required_ids:
    - MVS-APS-FAIL-EMPTY
    - MVS-APS-FAIL-CACHE
    - MVS-APS-1
    - MVS-APS-2
    - MVS-APS-3
    - MVS-APS-4
    - MVS-E2E-1
    - MVS-COMMIT-1-lite
    - MVS-CMC-1
    - MVS-CMC-2
    - MVS-CUC-1A_override_choice1
    - MVS-CUC-2
    - MVS-CUC-3
    - MVS-SAFE-1A_waiting_cap
    - MVS-SAFE-1B_executing_cap_lateral_consumption
    - MVS-SAFE-2
    - MVS-ASSIGN-1
    - MVS-CONFLICT-1A
    - MVS-CONFLICT-1B
    - MVS-COMMIT-1-full
  passed: 20
  failed: 0
  blocked: 0
  missing_runner_routes: []
  classification_blockers: []
  suite_status: passed
```

- Required / probe / deferred grouping sample:

```text
required: [
  MVS-APS-FAIL-EMPTY,
  MVS-APS-FAIL-CACHE,
  MVS-APS-1,
  MVS-APS-2,
  MVS-APS-3,
  MVS-APS-4,
  MVS-E2E-1,
  MVS-COMMIT-1-lite,
  MVS-CMC-1,
  MVS-CMC-2,
  MVS-CUC-1A_override_choice1,
  MVS-CUC-2,
  MVS-CUC-3,
  MVS-SAFE-1A_waiting_cap,
  MVS-SAFE-1B_executing_cap_lateral_consumption,
  MVS-SAFE-2,
  MVS-ASSIGN-1,
  MVS-CONFLICT-1A,
  MVS-CONFLICT-1B,
  MVS-COMMIT-1-full
]
probe: [MVS-CUC-1B_real_utility_probe]
deferred: [MVS-CUC-1C_real_utility_choice1_locked]
extra_non_mvs_deferred_or_out_of_scope: [P16-seeded-random-simulation, P17-paper-grid]
```

- `MVS-E2E-1` artifact sample:
  - official deterministic runner evidence id.
  - P10 commit / state transition evidence id.
  - commit event / trajectory / sanity / PNG marker references.

- `MVS-COMMIT-1-full` artifact sample:
  - one-commit-per-vehicle evidence.
  - duplicate sanity fail blocker sample.
  - active maneuver / cache lifecycle evidence.

- Trajectory export sample:

```text
scenario_id,run_id,step,t,vehicle_id,x_global,y,v,a,physical_lane,road_role,lane_change_state,merge_state,active_event_tags
MVS-E2E-1,p11-run,0,0.0,MV_CMC_1,7002.0,-3.48,20.0,0.0,on_ramp,on_ramp,normal,executing,commit
```

- Event export sample:

```text
event_type=commit
module=commit
vehicle_id=MV_CMC_1
source=first_version_engineering_patch
is_engineering_patch=true
payload.source_longitudinal_candidate=p08:0:MV_CMC_1:longitudinal
payload.source_lateral_candidate=p09:0:MV_CMC_1:lateral
```

- Sanity export sample:

```text
check_type=multiple_commit_for_one_vehicle
result=pass
vehicle_ids=[MV_CMC_1]
```

- Formal PNG file path sample:

```text
artifacts/MVS-E2E-1/p11-run/time_space.png
markers:
  - commit_marker
  - trajectory_quicklook
  - active_maneuver_marker
  - source_chain_marker
```

- Artifact manifest sample:

```text
scenario_id: MVS-E2E-1
run_id: p11-run
status: required_green
inputs:
  scenario_config: official built-in ScenarioConfig
exports:
  trajectory: artifacts/MVS-E2E-1/p11-run/trajectory.csv
  events: artifacts/MVS-E2E-1/p11-run/events.jsonl
  sanity: artifacts/MVS-E2E-1/p11-run/sanity.jsonl
  png:
    - artifacts/MVS-E2E-1/p11-run/time_space.png
reports:
  scenario: artifacts/MVS-E2E-1/p11-run/scenario_report.json
  regression: artifacts/regression_report.json
gaps: []
```

- Regression report sample:

```text
summary:
  suite_status: passed
  required_green: 20
  required_failed: []
  required_blocked: []
  runner_gaps: []
  probe_observed: [MVS-CUC-1B_real_utility_probe]
  deferred_skipped: [MVS-CUC-1C_real_utility_choice1_locked]
```

- Additional evidence:
  - probe non-blocking report sample.
  - deferred skipped report sample.
  - x_plot-only-renderer evidence.
  - no-state-mutation evidence.
  - P04-P10 targeted evidence references showing P11 cites, not first-generates, upstream event / sanity / markers.
  - P00-P10 regression green evidence.
  - explicit runner / loader / ScenarioConfig / OutputArtifactRecord / OutputConfig schema gap list if still present.

## 9. 完成标准

P11 is considered complete when all are true:

- The exact 20 target required MVS ids from section 5.1 appear in the P11 suite registry; wildcard family names or ellipsis do not count as completeness evidence.
- Every required MVS is executable in the official loader / deterministic runner / matcher path; no required scenario is silently skipped.
- `MVS-SAFE-1A_waiting_cap` is counted as required green, not probe.
- `P05-EXECUTING-CONTINUATION` is excluded from the required pass denominator unless an upstream authority spec promotes it.
- Required suite pass/fail is determined by matcher results, sanity results and required evidence, not by log text.
- Probe scenarios are reported but non-blocking.
- Deferred scenarios are skipped / not required for first-version gate.
- Trajectory, event and sanity histories are exported as stable artifacts.
- Formal PNG renderer consumes expected_png_features, emits structured marker evidence / feature status, and produces human-reviewable image files.
- Artifact manifest traces scenario_id, run_id, status, input, history exports, PNG files, report files and gaps.
- Regression report summarizes required / probe / deferred, failure reasons, gaps and artifact paths.
- Export / PNG / report do not mutate `SimulationState`, `OutputHistory` or algorithm records.
- `x_plot` is renderer-only.
- P11 does not first-generate P04-P10 logs, sanity checks, targeted MVS evidence or PNG markers.
- P11 does not re-run or reimplement P04-P10 algorithms.
- P11 does not implement P16 random vehicle generation or P17 paper-level experiments.
- Duplicate commit sanity fail blocks runner continuation.
- `MVS-E2E-1` and `MVS-COMMIT-1-full` remain official required runner routes and do not appear in `runner_gaps`.
- P11 implementation returns evidence samples, not only pytest counts.

## 10. 回归保护

Future changes must preserve these invariants:

- `x_global` remains the authoritative algorithm and trajectory coordinate.
- `x_plot` may be derived only in renderer / figure export scope.
- P11 export, renderer and report are side-effect-free with respect to algorithm state.
- Required scenario missing route is a blocker, never deferred by convenience.
- The 20-ID required denominator cannot be replaced by wildcard family expansion or incomplete samples.
- `MVS-SAFE-1A_waiting_cap` cannot be downgraded to probe by loader status drift.
- `P05-EXECUTING-CONTINUATION` cannot inflate required pass counts.
- Probe does not block required suite unless upstream promotes it.
- Deferred does not enter first-version required pass criteria.
- P11 cannot repair missing P04-P10 evidence by inventing new upstream events.
- P11 cannot reinterpret official deterministic E2E evidence as helper-only runner gap.
- P11 cannot continue from duplicate-commit fail-state.
- P11 cannot introduce untracked schema fields; schema gaps must be explicit.
- Formal PNG is a delivery artifact, not an algorithm input; its gate is expected_png_features marker evidence, not subjective visual approval.
- Artifact manifest and regression report must be reproducible from structured records.
- P16 / P17 remain separate: random boundary generation, random vehicle attributes and paper-level metrics must not become hidden P11 dependencies.

## 11. P13.5 当前状态校准

This section records current project facts after P12 / P13 closure:

1. `MVS-E2E-1` is registered through official loader / deterministic runner / matcher and is required green.
2. `MVS-COMMIT-1-full` is registered through official loader / deterministic runner / matcher and is required green.
3. CUC required scenarios are registered through official loader / deterministic runner / matcher and are required green.
4. SAFE required scenarios, including `MVS-SAFE-1A_waiting_cap`, are registered through official loader / deterministic runner / matcher and are required green.
5. Current suite status is `suite_status="passed"` with 20 required green, `required_failed=[]`, `required_blocked=[]` and `runner_gaps=[]`.
6. Current probe list is `MVS-CUC-1B_real_utility_probe`.
7. Current deferred list is `MVS-CUC-1C_real_utility_choice1_locked`.
8. P11 is not the formal natural-output delivery phase. P14 must create the formal artifact bundle path and regression baseline before P15 engine consolidation starts.
9. P11 must not introduce P16 random generation or P17 paper experiment grid as hidden dependencies.

P13.5 verification record:

- `python -m pytest tests\test_p11_output_export.py tests\test_p12_deterministic_simulation_loop.py` -> `30 passed`
- P11 suite summary -> `suite_status=passed`, `required_green=20`, `required_failed=[]`, `required_blocked=[]`, `runner_gaps=[]`, `probe=[MVS-CUC-1B_real_utility_probe]`, `deferred=[MVS-CUC-1C_real_utility_choice1_locked]`
