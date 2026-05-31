# P03 - Step 范围：Step 9-10 Command / NextState / Commit / Event / Sanity / Trajectory 最小闭环 / MVS-COMMIT-1-lite Gate

## 0. Slice Identity

- Algorithm-Step Coverage:
  - Primary Steps: Step 9、Step 10。
  - Secondary Steps: CommandBuffer / NextStateBuffer 最小闭环、candidate selection rule、commit_step、duplicate commit guard、immutable snapshot guard、OutputHistory in-memory v0、event / sanity assertion helper v0。
- MVS Acceptance Gate:
  - required:
    - `MVS-COMMIT-1-lite` 通过。
    - 每车每步最多一个 final candidate。
    - duplicate candidate 触发 `multiple_commit_for_one_vehicle` sanity failure 或 commit warning，不得静默提交两次。
    - next-state 不反写 `S(t)`。
    - commit 是唯一真实状态写入点。
    - SimulationState 不包含 command、next-state 或 history。
    - `CUCDecision` 只能进入 command / event / history，不进入下一步控制状态。
  - probe:
    - `MVS-COMMIT-1-full` 的 cache reuse、active maneuver、PNG feature 只要求预留可观测入口，不阻塞本阶段。
  - deferred:
    - 稳定文件导出格式。
    - 完整 PNG renderer。
    - 全量 smoke suite runner。
    - regression report。
- 本阶段解锁的能力:
  - 在 APS / CUC / CMC 算法复杂化前，证明状态提交规则闭环。
  - 让 event、sanity、trajectory 最小内存记录能力可被 P04-P10 targeted matcher 消费。
  - 固定 commit 是唯一生成 `S(t+dt)` 的阶段。
  - 固定 CUCDecision 的生命周期：command / event / history，而非跨步控制状态。
- 本阶段不要求通过的后续场景:
  - 不要求 `MVS-COMMIT-1-full` 通过。
  - 不要求 APS、CUC、CMC、longitudinal、lateral 业务 candidate 完整正确。
  - 不要求正式 PNG、artifact record 或文件导出。

## 1. 本阶段目标

本阶段落地 Step 9-10 的最小闭环：从本步 command / next-state 候选中为每辆车选择唯一 final candidate，经 commit 生成 `S(t+dt)`，再把提交结果、事件、sanity 和轨迹写入内存 OutputHistory v0。它不实现复杂算法，只证明状态边界、一次提交、日志不反写和 CUCDecision 非持久化。

本阶段覆盖 CORMC 时间步流程中的 Step 9 和 Step 10：

- Step 9：读取冻结 `S(t)`、CommandBuffer、NextStateBuffer、CandidateCacheUpdate、CandidateStateTransition；合成每辆 active vehicle 的唯一 CandidateKinematics；提交真实下一状态；正式更新 lane / road_role / state machine / cache；记录 commit event。
- Step 10：读取 commit 后 `S(t+dt)`、本步 command / candidate / event candidates，生成 TrajectoryRecord、EventRecord、SanityCheckRecord、OutputHistory in-memory v0；不得重新改写车辆运动状态。

本阶段让 `MVS-COMMIT-1-lite` 从 P01 的 failing contract 变为 required pass：每辆车每步只提交一次，duplicate candidate 触发 sanity，next-state 不污染冻结状态，SimulationState 不携带临时或 history 结构。

本阶段为后续阶段提供稳定输入：CommandBuffer、NextStateBuffer、candidate selection rule、commit_step、duplicate guard、immutable snapshot guard、EventRecord / SanityCheckRecord / TrajectoryRecord 内存记录、event / sanity assertion helper v0。

本阶段需要 event / sanity / PNG 证据：commit event 记录 candidate source、final state、state transition、cache cleanup；sanity 记录 duplicate commit、state machine consistency、x_plot path；trajectory 记录每步每车的 commit 后状态；PNG 只注册 commit marker / trajectory quicklook 所需数据，不要求正式渲染。

## 2. 非目标 / 禁止事项

- 不实现 APS assignment、CUC utility、CMC Eq.53、纵向 / 横向模型公式。
- 不实现稳定文件导出格式。
- 不实现完整 PNG renderer。
- 不运行全量 smoke suite。
- 不生成 regression report。
- 不新增核心字段；缺字段先修订代码数据结构 spec。
- 不允许任何模块在 commit 前直接写真实 `x / y / v / a`。
- 不允许 Step 10 information integration 反向修改已提交状态。
- 不允许 `CUCDecision` 写入下一步持久控制状态。
- 不允许 duplicate candidate 被 silent last-write-wins 处理。
- 不允许 SimulationState 包含 CommandBuffer、NextStateBuffer、RelationsSnapshot、TrajectoryRecord、EventRecord 或 SanityCheckRecord。

## 3. 上游 spec 引用

- 时间步总纲：引用 Step 9 同步提交、每车本步只提交一次、CV 完成换道正式更新 lane、MV 到达 lane 2 centerline 后转为 mainline 并清理 APS assignment、Step 10 记录轨迹与事件。
- 公式映射：引用 Step 9-10 的同步提交为第一版实现约束；每车每步只提交一次是工程补丁 / 实现约束，不能写成论文公式。
- 状态与模块接口：引用 commit 是唯一真实写入点、information integration 不是状态回写、command / next-state / cache / state transition 的边界。
- 代码数据结构：引用 `CommandBuffer`、`NextStateBuffer`、`CandidateLongitudinalKinematics`、`CandidateLateralKinematics`、`CandidateKinematics`、`CandidateManeuverProgress`、`CandidateLaneState`、`CandidateStateTransition`、`CandidateCacheUpdate`、`CommitWarning`、`TrajectoryRecord`、`EventRecord`、`SanityCheckRecord`、`OutputHistory` 字段权威。
- 道路几何：引用 `x_global` 为 trajectory 原始算法坐标，`x_plot` 只在 PNG 渲染时派生。
- 参数规格：引用 `dt`、车辆长度、lane centerline 等只作为状态和 sanity 判断参数来源，不在本阶段重新定义。
- 车辆模型：引用车辆模型只输出 candidate，等待同步提交；speed cap / lateral consumption 的具体算法 deferred 到后续阶段。
- 输出日志：引用 Step 9 commit event、Step 10 trajectory / event / sanity 记录、日志不反写、CUC choice 不持久化。
- MVS：引用 `MVS-COMMIT-1-lite` 的 setup、expected_events、expected_sanity_checks、expected_png_features；引用 `MVS-COMMIT-1-full` 为 probe / deferred 后续承接。
- 复现讨论对齐：引用第一版优先跑通主链路、日志可定位、非论文全量实验的目标。

## 4. 行为契约

- Given：冻结 `S(t)`、CommandBuffer、NextStateBuffer 均已存在。When：执行 commit preparation。Then：为每辆 active vehicle 合成至多一个 final CandidateKinematics；若候选缺失或冲突，生成 CommitWarning 或 SanityCheckRecord candidate。
- Given：同一 vehicle 在 NextStateBuffer 中存在多个互斥 final candidate。When：执行 duplicate commit guard。Then：触发 `multiple_commit_for_one_vehicle` sanity fail / warning，并阻止 silent double commit。
- Given：NextStateBuffer 中存在 candidate x_global、y、v、a。When：commit 读取 candidate。Then：只在 commit output 中生成 `S(t+dt)`；冻结 `S(t)` 中对应 VehicleState 不得变化。
- Given：lane-change candidate 表明 CV 完成换道。When：commit 处理 state transition。Then：在 `S(t+dt)` 中正式更新 `physical_lane=lane_1`、`lane_change_state=normal`，并记录 commit event；不在 lateral 模块中提前改 lane。
- Given：merge candidate 表明 MV 到达 lane 2 centerline。When：commit 处理 merge completion。Then：在 `S(t+dt)` 中正式更新 `physical_lane=lane_2`、`road_role=mainline`、`merge_state=merged`，并生成 APS cache cleanup candidate / event。
- Given：CommandBuffer 中有 `CUCDecision`。When：commit 和 information integration 执行。Then：CUCDecision 可进入 EventRecord / history tag，不进入下一步 `VehicleState` 或跨步控制状态。
- Given：commit 已生成 `S(t+dt)`。When：Step 10 information integration 记录 trajectory、event、sanity。Then：只读取 `S(t+dt)` 和本步候选，不重新改写真实运动状态。
- Given：TrajectoryRecord 需要记录坐标。When：Step 10 写轨迹。Then：记录 `x_global` 和 `y`，不保存算法状态用 `x_plot`。
- Given：工程补丁事件如 duplicate commit guard。When：写 EventRecord 或 SanityCheckRecord。Then：必须保留 `source`、`reason`、`is_engineering_patch` 或等价来源标记。

## 5. 允许实现的代码对象

- domain / state objects
  - `OutputHistory` in-memory v0
  - `TrajectoryRecord`
  - `EventRecord`
  - `SanityCheckRecord`
  - `CommitResult`
- command / next-state objects
  - `CommandBuffer`
  - `NextStateBuffer`
  - `CandidateLongitudinalKinematics`
  - `CandidateLateralKinematics`
  - `CandidateKinematics`
  - `CandidateManeuverProgress`
  - `CandidateLaneState`
  - `CandidateStateTransition`
  - `CandidateCacheUpdate`
  - `CommitWarning`
- step runner / service functions
  - `select_final_candidate_per_vehicle`
  - `assemble_candidate_kinematics`
  - `commit_step`
  - `apply_state_transition_candidates`
  - `apply_cache_update_candidates`
  - `build_next_simulation_state`
  - `record_information_integration_v0`
- event / sanity helpers
  - `emit_commit_event`
  - `run_duplicate_commit_guard`
  - `run_immutable_snapshot_guard`
  - `run_state_machine_consistency_guard`
  - `append_trajectory_record`
  - `assert_event_records`
  - `assert_sanity_records`
- scenario tests
  - `test_mvs_commit_1_lite_passes`
  - `test_duplicate_candidate_triggers_multiple_commit_sanity`
  - `test_next_state_does_not_mutate_frozen_state`
  - `test_cuc_decision_not_persisted_after_commit`
- regression tests
  - `test_simulation_state_excludes_command_next_state_history`
  - `test_step10_does_not_rewrite_committed_state`
  - `test_commit_event_contains_candidate_source_and_final_state`

## 6. 先写失败测试

- unit tests:
  - NextStateBuffer 反写冻结 `S(t)` 时失败。
  - 同一 vehicle 两个 final CandidateKinematics 未触发 `multiple_commit_for_one_vehicle` 时失败。
  - commit 前 vehicle真实 `x / y / v / a` 被修改时失败。
  - SimulationState 包含 CommandBuffer、NextStateBuffer 或 history 时失败。
  - `CUCDecision` 出现在下一步 VehicleState 或跨步控制状态时失败。
  - Step 10 记录过程修改 `S(t+dt)` 时失败。
- integration tests:
  - `MVS-COMMIT-1-lite`：一个最小多车场景中每车 exactly one final commit event；trajectory_records 每步每车一条；baseline sanity 通过。
  - duplicate candidate 注入场景：同一车两个互斥 candidate，必须产生 `multiple_commit_for_one_vehicle` sanity fail 或 warning，并且不双写真实状态。
  - immutable snapshot 场景：commit 后比较 freeze snapshot，确认 `S(t)` 未被 next-state 或 Step 10 改写。
- MVS scenario tests:
  - `MVS-COMMIT-1-lite` required 通过。
  - `MVS-COMMIT-1-full` 仅注册为 probe / later targeted gate，不阻塞 P03。
- event log assertions:
  - commit event 必须包含 vehicle_id、source_candidate_id、final state summary、state transition、cache cleanup reason。
  - duplicate guard event 若出现，必须标注 `is_engineering_patch=true` 或 equivalent source。
- sanity check assertions:
  - `multiple_commit_for_one_vehicle=false` 在正常 lite 场景通过。
  - duplicate 注入场景中 `multiple_commit_for_one_vehicle` 为 fail / warning。
  - `state_machine_inconsistency=false`。
  - `x_plot_used_in_algorithm_path=false`。
- PNG / artifact assertions:
  - expected_png_features v0 注册 trajectory quicklook data。
  - expected_png_features v0 注册 commit marker。
  - 不要求正式 PNG 文件、artifact path 或导出格式。

## 7. 验收证据

- EventRecord:
  - `commit`：记录 candidate source、final x_global / y / v / a、lane / road_role / state transition、cache cleanup。
  - `engineering_patch` 或 commit warning：记录 duplicate commit guard 的 source、reason、is_engineering_patch。
- SanityCheckRecord:
  - `multiple_commit_for_one_vehicle`。
  - `state_machine_inconsistency`。
  - `geometry_inconsistency` 或 `x_plot_used_in_algorithm_path`。
  - collision / near_collision baseline 可运行，但复杂碰撞逻辑可在后续扩展。
- TrajectoryRecord:
  - 每个 committed active vehicle 每步一条，包含 `step`、`t`、`vehicle_id`、`x_global`、`y`、`v`、`a`、`physical_lane`、`road_role`、`lane_change_state`、`merge_state`。
  - 不记录作为算法状态的 `x_plot`。
- PNG feature:
  - trajectory quicklook 数据已可由 TrajectoryRecord 派生。
  - commit marker 可由 commit event 派生。
- commit invariant:
  - 每辆车每步最多一次最终提交。
  - commit 是唯一真实状态写入点。
  - next-state 不作为本步其他车辆真实输入。
- ScenarioConfig expected matcher:
  - `MVS-COMMIT-1-lite` expected_events 和 expected_sanity_checks 被 P01 matcher 消费并通过。
- `source / reason / is_engineering_patch`:
  - duplicate commit guard、每车每步一次提交、CUCDecision 非持久化若以 event 记录，必须标注第一版实现约束或工程补丁来源。

## 8. 完成标准

- `MVS-COMMIT-1-lite` required gate 通过。
- 正常场景中每辆车每步最多一个 final candidate 和一个 commit event。
- duplicate candidate 场景显式记录 `multiple_commit_for_one_vehicle`，不得 silent pass。
- next-state 不反写冻结 `S(t)`。
- commit 是唯一真实状态写入点。
- SimulationState 不包含 command、next-state、relations 或 history。
- `CUCDecision` 只进入 command / event / history，不进入下一步控制状态。
- OutputHistory in-memory v0 可保存 TrajectoryRecord、EventRecord、SanityCheckRecord。
- event / sanity assertion helper v0 可被后续 P04-P10 targeted MVS 复用。
- probe 场景 `MVS-COMMIT-1-full` 不阻塞本阶段。
- deferred 的正式 PNG、artifact record、全量 suite 和 regression report 不进入本阶段强验收。

## 9. 回归保护

- commit 是唯一生成 `S(t+dt)` 的阶段。
- 每辆车每步最多提交一次最终状态。
- CommandBuffer 表示意图、约束或 maneuver choice；不得直接改真实状态。
- NextStateBuffer 表示候选下一状态；不得反写 `S(t)`。
- Step 10 information integration 不得改写已提交车辆运动。
- `CUCChoice` 不是跨步持久控制状态。
- `lane_change_state == executing` 与 `merge_state == executing` 不得同车同时为真。
- `x_global` 写入 TrajectoryRecord；`x_plot` 只由 PNG renderer 临时派生。
- 工程补丁不得写成论文原算法；duplicate guard 和每车每步一次提交必须保留来源标记。
- P04-P10 不得绕过 P03 的 commit / OutputHistory / sanity helper 另建状态写入路径。
