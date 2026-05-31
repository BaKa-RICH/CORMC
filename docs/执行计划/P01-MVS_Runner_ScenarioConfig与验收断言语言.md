# P01 - Step 范围：测试执行层、ScenarioConfig 与验收断言语言 / MVS Runner v0 Gate

## 0. Slice Identity

- Algorithm-Step Coverage:
  - Primary Steps: 测试执行层；服务所有 Step；不改变仿真算法调度。
  - Secondary Steps: Step 0-11 的 targeted scenario runner 入口、ScenarioConfig 加载、expected_events / expected_sanity_checks / expected_png_features matcher、required / probe / deferred 报告。
- MVS Acceptance Gate:
  - required:
    - `MVS-APS-FAIL-EMPTY` 能作为单场景 failing test 加载和执行到预期失败断言层。
    - `MVS-COMMIT-1-lite` 能作为单场景 failing test 加载和执行到预期失败断言层。
    - expected_events matcher 能表达 required event 缺失、reason mismatch、vehicle mismatch、numeric tolerance mismatch。
    - expected_sanity_checks matcher 能表达 pass / fail / warning / not_applicable。
    - expected_png_features matcher v0 能注册可人工复核特征，不要求正式 PNG 渲染。
    - targeted scenario report 显示 scenario_id、test_level、status、失败原因。
  - probe:
    - `MVS-CUC-1B_real_utility_probe` 可加载、可报告为 probe，不阻塞 required suite。
  - deferred:
    - `MVS-CUC-1C_real_utility_choice1_locked` 可加载、可报告为 deferred，不进入第一版强验收。
    - 全量 smoke suite runner、正式 PNG renderer、artifact record、regression report。
- 本阶段解锁的能力:
  - MVS 场景从自然语言清单变成可执行验收合同。
  - required / probe / deferred 在 runner 输出中成为强语义。
  - 后续 P04-P10 可以用 targeted gate 先写失败测试再实现。
  - PNG feature 可以先作为 expected_png_features 注册和报告，不绑定正式渲染。
- 本阶段不要求通过的后续场景:
  - 不要求 APS、CUC、CMC、纵向、横向、commit 算法正确。
  - 不要求 `MVS-APS-FAIL-EMPTY` 或 `MVS-COMMIT-1-lite` 的业务断言通过；本阶段接受它们作为 failing test 暴露缺失实现。
  - 不要求运行全量 required suite。

## 1. 本阶段目标

本阶段只落地 MVS Runner v0 的验收语言，使后续执行计划可以先写失败测试，再实现对应算法时间步切片。它不修复 APS、CMC、CUC 或 commit 逻辑，只要求 runner 能加载 ScenarioConfig 或等价测试配置，并把 expected_events、expected_sanity_checks、expected_png_features 与运行结果进行结构化匹配。

本阶段覆盖的时间步流程是测试执行层对 Step 0-11 的外部驱动：从 ScenarioConfig 读取初始时间、车辆、module_overrides、preloaded assignment / state / maneuver、tolerances 和 expected_*；启动 targeted scenario；收集已有或占位运行结果；生成 scenario report。

本阶段让以下 MVS 场景从“不可执行自然语言”变为“可作为失败测试执行”：

- `MVS-APS-FAIL-EMPTY`：验证 APS 候选不足且无 cache 时不得伪造 assignment 的断言语言。
- `MVS-COMMIT-1-lite`：验证每车每步一次 commit 的断言语言。

本阶段为后续阶段提供稳定输入：ScenarioConfig loader、matcher、report schema、tolerance 比较、required / probe / deferred 分流、PNG feature 注册口径。

本阶段需要的 targeted 证据包括：runner report 中出现 expected event 缺失原因、expected sanity 缺失或状态不匹配原因、expected_png_features 未渲染但已注册的说明、probe / deferred 不阻塞 required 的 suite 结果。

## 2. 非目标 / 禁止事项

- 不实现 APS、CUC、CMC、纵向、横向、commit 的业务算法。
- 不新增 ScenarioConfig 字段；loader 只能消费代码数据结构 spec 已定义或允许的字段。
- 不把 `test_harness_overrides` 伪装成论文原生机制。
- 不要求正式 PNG renderer；expected_png_features v0 只表达可复核特征。
- 不把 probe / deferred 场景计入 required failure。
- 不吞掉失败原因；failing test 必须报告缺失 event、缺失 sanity、数值 mismatch、状态分类错误或 loader 错误。
- 不允许 runner 直接修改 `S(t)`、command、next-state 或 commit 结果来让场景通过。
- 不允许以自然语言检查替代 expected_* matcher。

## 3. 上游 spec 引用

- 时间步总纲：引用 Step 0-11 的调度顺序，runner 只能驱动时间步，不改变 `S(t) -> command / next-state -> commit -> S(t+dt)`。
- 公式映射：引用 MVS 断言中 `source` 可标注为 `paper_formula`、`first_version_engineering_patch` 或 `test_harness_override`，不得混淆论文公式与工程补丁。
- 状态与模块接口：引用 command / next-state / cache / state transition 的语义边界；runner 不能绕过状态提交规则。
- 代码数据结构：引用 `ScenarioConfig`、`InitialTimeConfig`、`InitialVehicleConfig`、`ModuleOverrideSpec`、`PreloadedAssignmentSpec`、`PreloadedStateMachineStateSpec`、`PreloadedManeuverTrajectoryStateSpec`、`ExpectedEventSpec`、`ExpectedSanityCheckSpec`、`ExpectedPNGFeatureSpec`、`ScenarioToleranceSpec` 的字段权威。
- 道路几何：引用 `x_global` 为算法坐标、`x_plot` 仅用于 PNG 派生；MVS loader 不得把 x_plot 载入算法状态。
- 参数规格：引用默认 `dt = 0.1 s` 与场景 tolerance，不重新定义参数来源。
- 车辆模型：引用本阶段不实现车辆模型，只允许后续场景用 overrides 隔离验证。
- 输出日志：引用 event、sanity、trajectory、PNG 的通用验收语义，matcher 消费这些记录而不反向改运动。
- MVS：引用 `MVS-APS-FAIL-EMPTY`、`MVS-COMMIT-1-lite`、`MVS-CUC-1B_real_utility_probe`、`MVS-CUC-1C_real_utility_choice1_locked` 的 scenario_id、test_level、status、expected_*。
- 复现讨论对齐：引用第一版 smoke 优先、不追求论文全量数值复现的边界。

## 4. 行为契约

- Given：ScenarioConfig 包含 scenario_id、test_level、status、initial_time、initial_vehicles、module_overrides、preloaded state、expected_events、expected_sanity_checks、expected_png_features、tolerances。When：MVS Runner v0 加载单个场景。Then：生成只读 ScenarioRuntimeContext，并保留 expected_* 供 matcher 消费。
- Given：`MVS-APS-FAIL-EMPTY` 的 expected_events 要求 APS failure、failure_reason、无新 assignment、无 cooperative request。When：runner 在 APS 实现缺失或结果不匹配时执行 matcher。Then：报告缺失或 mismatch 的 event_type、vehicle_ids、reason_code、required 状态。
- Given：`MVS-COMMIT-1-lite` 的 expected_sanity_checks 包含 `multiple_commit_for_one_vehicle` 相关断言。When：runner 无法找到对应 SanityCheckRecord 或状态不匹配。Then：报告 check_type、expected_status、actual_status、vehicle_ids 和 time_window。
- Given：expected_png_features 只在 v0 注册。When：runner 执行本阶段场景。Then：报告 feature_type、vehicle_ids、expected_visibility 和“renderer deferred / feature registered”，不要求 PNG 文件存在。
- Given：场景 status 为 required。When：required expected event 或 sanity mismatch。Then：targeted scenario report 标记 failed，并阻塞 required suite。
- Given：场景 status 为 probe。When：probe expected mismatch。Then：报告 probe_observation 或 probe_failed，但不阻塞 required suite。
- Given：场景 status 为 deferred。When：runner 发现该场景。Then：报告 skipped_deferred 或 loaded_deferred，不执行强验收。
- Given：expected_* 包含 numeric_expectations。When：actual record 提供数值。Then：按 ScenarioToleranceSpec 比较，不由 matcher 临时发明容差。
- Given：test_harness_overrides 用于隔离场景。When：runner 输出 report。Then：必须将其标为测试钩子，不写成论文原生机制。

## 5. 允许实现的代码对象

只允许实现测试执行层结构、函数、模块和测试文件；不实现 CORMC 算法逻辑。

- domain / state objects
  - `ScenarioRuntimeContext`
  - `ScenarioRunResult`
  - `ScenarioReport`
  - `MatcherResult`
- command / next-state objects
  - 不新增业务 command 或 next-state 字段。
  - 只允许读取已有 `CommandBuffer` / `NextStateBuffer` 结果用于报告。
- step runner / service functions
  - `load_scenario_config`
  - `run_targeted_scenario`
  - `classify_scenario_status`
  - `build_scenario_report`
  - `compare_with_tolerance`
- event / sanity helpers
  - `match_expected_events`
  - `match_expected_sanity_checks`
  - `register_expected_png_features`
  - `match_expected_png_features_v0`
- scenario tests
  - `test_mvs_aps_fail_empty_failing_contract`
  - `test_mvs_commit_1_lite_failing_contract`
  - `test_probe_does_not_block_required_suite`
  - `test_deferred_does_not_enter_required_suite`
- regression tests
  - `test_scenario_config_loader_rejects_unknown_core_fields`
  - `test_matcher_reports_required_failure_reason`
  - `test_tolerance_comes_from_scenario_config`

## 6. 先写失败测试

- unit tests:
  - loader 缺少 `scenario_id` 时失败。
  - loader 遇到未在代码数据结构 spec 中定义的核心字段时失败。
  - matcher 对 required expected_event 找不到 actual event 时失败，并输出 event_type 和 reason。
  - matcher 对 reason_code mismatch 时失败。
  - matcher 对 numeric_expectations 超出 tolerance 时失败。
  - matcher 对 expected_sanity_check 状态不匹配时失败。
- integration tests:
  - `MVS-APS-FAIL-EMPTY` 可加载，且在 APS 尚未实现时以可解释原因失败，而不是 loader 崩溃。
  - `MVS-COMMIT-1-lite` 可加载，且在 commit 尚未实现时以缺失 commit / sanity 记录失败。
  - targeted scenario report 必须包含 scenario_id、test_level、status、required/probe/deferred 分类、失败原因列表。
- MVS scenario tests:
  - `MVS-APS-FAIL-EMPTY` 绑定 expected_events、expected_sanity_checks、expected_png_features。
  - `MVS-COMMIT-1-lite` 绑定 commit event、duplicate commit sanity、trajectory expectation 的占位消费能力。
  - `MVS-CUC-1B_real_utility_probe` 被标记为 probe，不阻塞 required suite。
  - `MVS-CUC-1C_real_utility_choice1_locked` 被标记为 deferred，不进入 required suite。
- event log assertions:
  - expected_events 可以匹配 event_type、vehicle_ids、time_window、match、numeric_expectations、reason_code、source。
  - 工程补丁 source 若为 `first_version_engineering_patch`，report 中必须保留 source / reason。
- sanity check assertions:
  - expected_sanity_checks 可以匹配 check_type、expected_status、vehicle_ids、time_window、reason_code。
  - baseline sanity 支持 `collision=false`、`near_collision=false`、`state_machine_inconsistency=false`、`unexpected_ordinary_lane_change_attempt=false`、`multiple_commit_for_one_vehicle=false`、`x_plot_used_in_algorithm_path=false`。
- PNG / artifact assertions:
  - expected_png_features v0 可以注册 `aps_failure_marker`、`no assignment arrow visible`、`commit marker` 等 feature。
  - 本阶段不要求 artifact path；若 runner 误要求正式 PNG 文件则失败。

## 7. 验收证据

- ScenarioConfig loader report：证明 `MVS-APS-FAIL-EMPTY` 与 `MVS-COMMIT-1-lite` 能加载。
- Scenario matcher report：证明 event / sanity / PNG feature matcher 能报告 required failure。
- Required suite v0 report：证明 required 失败会阻塞 required suite。
- Probe / deferred report：证明 `MVS-CUC-1B_real_utility_probe` 不阻塞 required suite，`MVS-CUC-1C_real_utility_choice1_locked` 不进入第一版强验收。
- EventRecord 证据：本阶段可使用空 actual records 触发 expected event 缺失失败；不要求业务 event 真实产生。
- SanityCheckRecord 证据：本阶段可使用空 actual records 触发 expected sanity 缺失失败；不要求 baseline sanity runner 已实现。
- PNG feature 证据：expected_png_features 在 report 中注册为待渲染 / 可人工复核特征。
- `source / reason / is_engineering_patch` 证据：matcher 和 report 必须能保留这些键，不要求业务模块产生补丁事件。

## 8. 完成标准

- `MVS-APS-FAIL-EMPTY` 和 `MVS-COMMIT-1-lite` 均能以 targeted failing-test 方式运行到 matcher 层。
- expected_events matcher 可表达 required event 缺失、字段 mismatch、reason mismatch、数值容差 mismatch。
- expected_sanity_checks matcher 可表达 pass / fail / warning / not_applicable。
- expected_png_features matcher v0 可表达 required / optional 可见特征，不要求正式 PNG。
- required / probe / deferred 在 ScenarioConfig、runner 和 report 中一致可见。
- `MVS-CUC-1B_real_utility_probe` 不阻塞 required suite。
- `MVS-CUC-1C_real_utility_choice1_locked` 不进入第一版强验收。
- runner 不新增核心字段，不改变算法状态，不绕过 commit。
- 所有失败输出都有可读失败原因，而不是只显示程序异常。

## 9. 回归保护

- ScenarioConfig 字段权威来自 `CORMC代码数据结构设计_整理版.md`。
- MVS 场景具体 expected_* 来自 `CORMC最小验证场景执行规格.md`。
- matcher 只能消费 record / report，不得修改车辆运动状态。
- required / probe / deferred 分类不得被后续 suite 聚合逻辑抹平。
- tolerance 不得由实现 agent 临时决定，必须来自 ScenarioToleranceSpec 或上游 spec。
- `x_global` 是算法坐标；`x_plot` 只允许在 PNG feature 或 renderer 层出现。
- `test_harness_overrides` 必须标为测试钩子，不得写成论文原算法。
- P04-P10 的 targeted MVS gate 必须复用本阶段 matcher，不得另写一套自然语言验收。
- P11 不得重写 P01 的 runner 语义，只能聚合已通过 gate。
