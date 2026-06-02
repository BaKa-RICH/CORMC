# P11 - Step10 Output Export / Formal PNG / Full MVS Regression Closure

> 本文档是 P11 red-before-green implementation 的执行计划 spec。P11 只覆盖 Step 10 的交付级收口：导出 P01-P10 已产生的 trajectory / event / sanity / PNG marker evidence，渲染正式 PNG，生成 artifact manifest，聚合全部 required MVS smoke suite，并输出 regression report。P11 不是 P04-P10 算法、日志、sanity、targeted runner 或 PNG marker 的首次实现阶段。

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
    - 不实现 P12 random boundary generation、random vehicle attributes、paper-level capacity / aggregate metrics、SUMO comparison。

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
    - `MVS-CUC-1B_real_utility_probe` and any additional diagnostic scenario explicitly marked probe by the upstream MVS index. A final required safety scenario must not be demoted to probe because of current loader status.
  - deferred:
    - P12 random generation and paper-level experiment grid.
    - SUMO comparison.
    - strict paper-level capacity / aggregate metrics.
    - strict MPC tracking unless upstream specs promote it.

- Current-state statement consumed by P11:
  - P10 helper-targeted `MVS-E2E-1` chain is closed: P05 merge-start evidence -> P08 longitudinal candidate -> P09 lateral/progress candidate -> P10 commit into real `S(t+dt)`.
  - P10 consumes P05/P08/P09 handoff and has evidence for true `S(t+dt)` commit.
  - Built-in `MVS-E2E-1` runner / loader route is still not registered. P11 must not report "`P10 full runner route ready`" as fact.
  - Duplicate final candidate currently produces `multiple_commit_for_one_vehicle` warning / sanity fail and no committed next vehicle for that failed fixture. A P11 runner must treat this as a blocking required failure, never as a state that may continue into later steps.

- Current code / schema facts:
  - Existing loadable built-in routes with code status `required` currently include APS, CMC, ASSIGN, CONFLICT, `MVS-COMMIT-1-lite`, and `P05-EXECUTING-CONTINUATION`. This is a code fact, not the P11 denominator: `P05-EXECUTING-CONTINUATION` is an extra P05 diagnostic / continuation route and must not count in the first-version required pass denominator unless an upstream authority spec promotes it.
  - Current loader status marks `MVS-SAFE-1A_waiting_cap` as `probe`, but the upstream MVS index and master plan list it as required. P11 must treat this as a classification gap / blocker until reconciled, not move `MVS-SAFE-1A_waiting_cap` into probe.
  - Several target required scenarios named by upstream specs are not yet registered as built-in runner routes or not registered with the required status needed by P11: `MVS-E2E-1`, `MVS-COMMIT-1-full`, `MVS-CUC-1A_override_choice1`, `MVS-CUC-2`, `MVS-CUC-3`, `MVS-SAFE-1A_waiting_cap`, `MVS-SAFE-1B_executing_cap_lateral_consumption`, `MVS-SAFE-2`.
  - `OutputHistory.png_artifacts` exists, but formal `OutputArtifactRecord` / manifest schema and output path contract are not implemented as authoritative code objects.
  - `OutputConfig` is described in data-structure documentation, but current code does not yet provide a strict file-output config object for P11.
  - `information_integration` is accepted by current P03/P10 code and matcher tests, but authority docs may still need confirmation if strict enum validation is introduced.

## 2. 本阶段目标

P11 的目标是把 P01-P10 已经逐步落地的 in-memory evidence 变成交付级 artifacts。P11 成功后，人工审阅者应能从一个 run output directory 中看到每个 scenario 的 trajectory export、event export、sanity export、formal PNG、artifact manifest entry 和 regression result，并能确认 required / probe / deferred 的 gating 语义正确。

最小目标：

1. 建立 full required MVS smoke suite registry，列出目标 required suite 与当前可执行状态。
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
10. 明确 P11 完成后才允许进入 P12；P12 的随机边界生成、随机属性和论文级指标不应被 P11 偷偷引入。

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
- 不实现 P12 random arrival、random vehicle attributes、capacity / aggregate metrics、paper-level experiment grid。

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
  - 当前 built-in CUC required routes 仍需 P11 registry guard 明确，不得 silent skip。

- `docs/执行计划/P08-Step7_LongitudinalModel_Eq10SpacingOverride_SpeedCapComposition.md`
  - 引用 P08 longitudinal candidate、planning speed、constraints_applied、source_commands、PNG markers。
  - P11 不重算 planning speed。

- `docs/执行计划/P09-Step8_LateralTrajectory_PlanningSpeedConsumption_ManeuverProgress.md`
  - 引用 P09 lateral candidate、maneuver progress、completion candidates、boundary risk sanity、PNG markers。
  - P11 不重算 lateral trajectory。

- `docs/执行计划/P10-Step9_CandidateAssembly_Commit_StateTransition_Integration.md`
  - 引用 P10 的 Step4-9 integration responsibility、helper-targeted E2E gate、MVS-COMMIT-1-full targeted subset、OutputHistory evidence、renderer-deferred P10 markers。
  - P11 必须保留当前结论：P10 helper-targeted chain closed；built-in `MVS-E2E-1` full runner route not ready。

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
  - P11 implementation 必须对目标清单与当前 built-in registry 的差异给出 blocker / gap，而不是静默跳过。

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
- Then: every target required `scenario_id` must be present as loadable, executable, or explicitly reported as a required blocker.
- Then: registry completeness is checked against the exact 20 IDs above; wildcard family labels are not sufficient evidence.
- Then: missing built-in routes such as current `MVS-E2E-1` / `MVS-COMMIT-1-full` must fail the P11 required suite readiness gate, not be treated as deferred.
- Then: `MVS-SAFE-1A_waiting_cap` must remain in required even if a current loader entry marks it as probe; that mismatch is a classification blocker.
- Then: `P05-EXECUTING-CONTINUATION` may be reported as an extra regression / diagnostic route, but it is not in the 20-ID required pass denominator.

### 5.2 Required / probe / deferred partition

- Given: P01 supports required / probe / deferred status.
- When: P11 aggregates scenario results.
- Then: required failures block the suite.
- Then: probe failures or observations are reported but do not block required suite pass; current MVS probe denominator includes `MVS-CUC-1B_real_utility_probe` and only scenarios explicitly marked probe by upstream authority.
- Then: deferred scenarios are reported as skipped / not required for first-version gate; current MVS deferred denominator includes `MVS-CUC-1C_real_utility_choice1_locked`.
- Then: no scenario in the exact 20-ID required denominator may appear in probe or deferred groups unless an upstream authority spec explicitly revises the denominator.

### 5.3 P10 prerequisite guard

- Given: P10 currently has helper-targeted `MVS-E2E-1` chain evidence but not a built-in full runner route.
- When: P11 evaluates readiness for `MVS-E2E-1`.
- Then: P11 may cite the P10 helper-chain evidence as targeted evidence.
- Then: P11 must still report built-in runner / loader route gap until `MVS-E2E-1` is registered and executable in the smoke suite.
- Then: P11 must not claim "`P10 full runner route ready`".

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

### 5.12 P12 boundary

- Given: P12 owns random boundary generation, random vehicle attributes and paper-level experiments.
- When: P11 builds smoke suite and artifacts.
- Then: random arrival and random vehicle attributes remain disabled for required smoke.
- Then: capacity / aggregate metric outputs are not required P11 pass conditions.

## 6. 允许实现的代码对象

P11 implementation may add or modify delivery-layer objects only. Allowed objects include:

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

P11 implementation must not add algorithm objects for APS / CMC / P06 / CUC / P08 / P09 / P10. If existing `ScenarioConfig`、`OutputConfig`、`OutputArtifactRecord` or expected_* schema is insufficient, the red test should fail at a structured schema / contract gap, not at unknown field / loader crash.

## 7. 先写失败测试

本次创建文档时不新增 P11 实际测试文件。未来 P11 implementation must follow this red-before-green order:

1. Add P11 failing tests / test skeletons first.
2. Run P11 targeted tests and confirm red failures occur in smoke suite aggregation, export, PNG rendering, artifact manifest or regression report contract layers.
3. Red failures must not be ImportError, AttributeError, unknown enum, unknown field, loader crash, or natural-language assertion.
4. Implement minimal P11 export / renderer / suite aggregation / report in the same round.
5. Run P11 targeted green tests and P00-P10 regression.
6. Return red-before-green evidence with examples, not only pytest counts.

Required test plan:

- `test_p11_required_mvs_registry_contains_target_suite`
  - Asserts the exact 20 target required ids from section 5.1 are enumerated.
  - Asserts family wildcards / ellipsis are not accepted as registry completeness evidence.
  - Missing built-in required routes are reported as blockers, not skipped.

- `test_p11_safe_1a_waiting_cap_is_required_not_probe`
  - Asserts `MVS-SAFE-1A_waiting_cap` is in the required denominator.
  - Asserts any current loader `probe` status for `MVS-SAFE-1A_waiting_cap` is reported as a classification blocker, not accepted as P11 probe grouping.

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

- `test_p11_p10_prerequisite_guard_reports_e2e_runner_gap`
  - Asserts helper-targeted P10 E2E evidence can be referenced.
  - Asserts missing built-in `MVS-E2E-1` runner route remains a blocker / gap.

- `test_p11_duplicate_commit_sanity_blocks_runner_continuation`
  - Asserts `multiple_commit_for_one_vehicle=fail` blocks continuation rather than feeding missing-vehicle state to later steps.

- `test_p11_does_not_enable_p12_random_or_paper_metrics`
  - Asserts random arrival, random vehicle attributes and paper-level metrics are absent from first-version required pass criteria.

Static / matcher tests:

- P00 traceability must continue to assert P11 is delivery aggregation and not first log / sanity / MVS / PNG stage.
- P01 matcher semantics must remain unchanged; P11 aggregates matcher results rather than redefining them.
- If P11 adds artifact manifest schema, authoritative data-structure docs and code must be revised together.

## 8. 验收证据

P11 implementation completion report must include at least:

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
  passed: <count>
  failed: <count>
  blocked: <count>
  missing_runner_routes:
    - MVS-E2E-1
    - MVS-COMMIT-1-full
  classification_blockers:
    - MVS-SAFE-1A_waiting_cap: current loader status probe conflicts with required MVS index
  extra_diagnostics_not_in_required_denominator:
    - P05-EXECUTING-CONTINUATION
  suite_status: failed_until_all_required_routes_registered
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
extra_non_mvs_deferred_or_out_of_scope: [P12-paper-grid]
```

- `MVS-E2E-1` artifact sample:
  - P10 helper-targeted chain evidence id.
  - built-in runner gap statement until route is registered.
  - commit event / trajectory / sanity / PNG marker references.

- `MVS-COMMIT-1-full` artifact sample:
  - one-commit-per-vehicle evidence.
  - duplicate sanity fail blocker sample.
  - active maneuver / cache lifecycle evidence.
  - built-in route gap if still missing.

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
status: required_blocked_until_runner_route_registered
inputs:
  scenario_config: built-in-or-helper-reference
exports:
  trajectory: artifacts/MVS-E2E-1/p11-run/trajectory.csv
  events: artifacts/MVS-E2E-1/p11-run/events.jsonl
  sanity: artifacts/MVS-E2E-1/p11-run/sanity.jsonl
  png:
    - artifacts/MVS-E2E-1/p11-run/time_space.png
reports:
  scenario: artifacts/MVS-E2E-1/p11-run/scenario_report.json
  regression: artifacts/regression_report.json
gaps:
  - built-in MVS-E2E-1 runner route not registered
```

- Regression report sample:

```text
summary:
  required_green: [...]
  required_failed: [...]
  required_blocked:
    - scenario_id: MVS-E2E-1
      reason: missing_runner_route
  probe_observed: [...]
  deferred_skipped: [...]
  schema_gaps:
    - OutputArtifactRecord authority missing in code
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

P11 may be called complete only when all are true:

- The exact 20 target required MVS ids from section 5.1 appear in the P11 suite registry; wildcard family names or ellipsis do not count as completeness evidence.
- Every required MVS is executable or explicitly reported as a prerequisite blocker; no required scenario is silently skipped.
- `MVS-SAFE-1A_waiting_cap` is counted as required, not probe; any loader status mismatch is reported as a classification blocker.
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
- P11 does not implement P12 random vehicle generation or paper-level experiments.
- Duplicate commit sanity fail blocks runner continuation.
- P10 built-in full runner route readiness is not claimed until `MVS-E2E-1` and `MVS-COMMIT-1-full` are registered and executable in the suite.
- P11 implementation returns evidence samples, not only pytest counts.

## 10. 回归保护

Future changes must preserve these invariants:

- `x_global` remains the authoritative algorithm and trajectory coordinate.
- `x_plot` may be derived only in renderer / figure export scope.
- P11 export, renderer and report are side-effect-free with respect to algorithm state.
- Required scenario missing route is a blocker, never deferred by convenience.
- The 20-ID required denominator cannot be replaced by wildcard family expansion or incomplete samples.
- `MVS-SAFE-1A_waiting_cap` cannot be downgraded to probe by current loader status.
- `P05-EXECUTING-CONTINUATION` cannot inflate required pass counts.
- Probe does not block required suite unless upstream promotes it.
- Deferred does not enter first-version required pass criteria.
- P11 cannot repair missing P04-P10 evidence by inventing new upstream events.
- P11 cannot reinterpret P10 helper-targeted E2E evidence as built-in full runner readiness.
- P11 cannot continue from duplicate-commit fail-state.
- P11 cannot introduce untracked schema fields; schema gaps must be explicit.
- Formal PNG is a delivery artifact, not an algorithm input; its gate is expected_png_features marker evidence, not subjective visual approval.
- Artifact manifest and regression report must be reproducible from structured records.
- P12 remains separate: random boundary generation, random vehicle attributes and paper-level metrics must not become hidden P11 dependencies.

## 11. 当前 P11 入口 gap 清单

This section records current project facts that P11 implementation must treat as blockers or explicit gaps:

1. Built-in `MVS-E2E-1` runner / loader route is not registered. P10 helper-targeted chain evidence exists, but full runner route readiness is not established.
2. Built-in `MVS-COMMIT-1-full` route is not registered. P10 targeted commit responsibility exists, but full route aggregation remains a P11 / later runner task.
3. Several target required scenarios from upstream specs are not currently loadable built-ins: `MVS-CUC-1A_override_choice1`, `MVS-CUC-2`, `MVS-CUC-3`, `MVS-SAFE-1B_executing_cap_lateral_consumption`, `MVS-SAFE-2`.
4. `MVS-SAFE-1A_waiting_cap` is loadable in current code but marked `probe`; upstream authority marks it required. P11 must report this classification mismatch as a blocker until loader / runner status is reconciled.
5. `P05-EXECUTING-CONTINUATION` is loadable and currently marked required in code, but it is not part of the final 20-ID P11 required denominator. It may appear only as extra regression / diagnostic evidence.
6. Current runner executes targeted routes by scenario family. A one-shot full required smoke suite aggregator is not implemented.
7. Formal PNG renderer is not implemented; current expected_png_features are renderer-deferred.
8. Artifact manifest / formal `OutputArtifactRecord` code schema is not implemented, although data-structure docs reserve `OutputHistory.png_artifacts`.
9. OutputConfig exists as a documented concept, but current code does not yet expose a strict file-output config used by exporters / renderer.
10. `information_integration` event string is current-code compatible; if strict enum validation is introduced, authority docs / loader / matcher must be reconciled before P11 writes strict expected events.
11. Duplicate commit fail-state must be blocked by P11 suite logic; current P10 helper returns a fail-state fixture rather than raising.
12. P11 must not use the current absence of full runner routes as a reason to pass required suite.
