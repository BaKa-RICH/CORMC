# P02 - Step 范围：Step 0-3 清理、pre-freeze、冻结 S(t)、relations snapshot 与几何口径 / Geometry & Snapshot Sanity Gate

## 0. Slice Identity

- Algorithm-Step Coverage:
  - Primary Steps: Step 0、Step 1、Step 2、Step 3。
  - Secondary Steps: 为 Step 4 APS、Step 6 CUC、Step 7 纵向、Step 8 横向和 Step 9 commit 提供冻结状态、relations snapshot、lane ordering、region resolver、lane centerline resolver、APS candidate window resolver。
- MVS Acceptance Gate:
  - required:
    - 通用 sanity baseline 可运行并记录：`collision=false`、`near_collision=false`、`state_machine_inconsistency=false`、`unexpected_ordinary_lane_change_attempt=false`、`multiple_commit_for_one_vehicle=false`、`x_plot_used_in_algorithm_path=false`。
    - Step 0 清空本步 command / next-state buffer，但不清空 APS assignment cache 或 active maneuver trajectory state。
    - Step 1 boundary generation hook 只发生在 freeze 前；freeze 后不得插入新车影响本步 APS / CUC / CMC / 纵向 / 横向。
    - Step 2 freeze 后所有后续模块共享同一 `S(t)` snapshot。
    - Step 3 lane ordering 使用 `x_global`，不使用 `x_plot`。
    - APS candidate window resolver 使用 `[x_MV_global - L_cr, x_MV_global + L_cr]`，不得使用 fixed cooperative zone 替代。
  - probe:
    - 为后续 `MVS-APS-*`、`MVS-CUC-*` 输出 relations event 与 geometry sanity，暂不要求 APS / CUC 业务通过。
    - active lane-change / merge relation basis 可观测，供后续 same-step overlay 和 active maneuver 关系验证。
  - deferred:
    - APS assignment case 判定。
    - CUC utility、CMC Eq.53、纵向 / 横向 candidate、commit。
    - 正式 PNG renderer 与 artifact record。
- 本阶段解锁的能力:
  - 形成本步稳定 `S(t)`。
  - 形成每步派生 `RelationsSnapshot`。
  - 固定 `x_global` / `x_plot` 口径，防止绘图坐标污染算法。
  - 固定 lane centerline、region、merging zone、APS candidate window 的基础服务。
  - 为 `MVS-APS-*` 和 `MVS-CUC-*` 提供可复用的 candidate / neighbor 输入。
- 本阶段不要求通过的后续场景:
  - 不要求 `MVS-APS-FAIL-EMPTY` 的 APS failure event 由 APS 模块产生。
  - 不要求 `MVS-CUC-1A_override_choice1` 的 lane-change command 产生。
  - 不要求 `MVS-COMMIT-1-lite` 的 commit 通过。

## 1. 本阶段目标

本阶段落地 CORMC 时间步主链路的空间底座和状态冻结底座。它只覆盖一次 `S_pre(t) -> S(t) + relations snapshot` 的准备切片：先清理本步临时数据，允许 freeze 前的边界生成 hook，再冻结当前状态，最后基于冻结 `S(t)` 生成 relations snapshot 和几何 resolver 输出。

本阶段覆盖 Step 0-3：

- Step 0：移除已驶出车辆，清空本步 command / next-state buffer，保留 APS assignment cache 与 active maneuver trajectory state。
- Step 1：提供 pre-freeze boundary generation hook；在 MVS 默认关闭随机生成时，该 hook 仍要可被观测为 disabled / skipped。
- Step 2：冻结 `S(t)`，后续 APS / CUC / CMC / 纵向 / 横向只读该 snapshot。
- Step 3：刷新 lane ordering、leader / follower、TLV / TFV / LV / FV、active maneuver logical role 与 overlay basis。

本阶段让通用 sanity baseline 从不可观测变成可记录；为后续 `MVS-APS-*` 提供正确 lane 2 candidates 和 APS candidate window；为后续 `MVS-CUC-*` 提供 TLV / TFV / LV / FV；为后续 longitudinal / lateral 提供不按 physical y 跳变的 logical role。

本阶段需要 targeted MVS event / sanity / PNG 证据证明：`relation_refresh` event 存在，lane ordering payload 使用 x_global，geometry sanity 证明 x_plot 未进入算法路径，freeze event 证明冻结后 active_vehicle_ids 不再变化，expected_png_features 可注册 lane centerline、merging zone 和 APS candidate window quicklook。

## 2. 非目标 / 禁止事项

- 不实现 APS assignment 选择、APS case、Eq.10 消费。
- 不实现 cooperative request arbitration、CUC utility 或 lane-change command。
- 不实现 CMC Eq.53、boundary speed cap、merge transition。
- 不实现纵向 / 横向车辆模型 candidate。
- 不实现 commit。
- 不新增 `VehicleState`、`SimulationState`、`RelationsSnapshot`、ScenarioConfig、record 字段。
- 不使用 `x_plot` 进行排序、区域判断、APS candidate window、leader / follower、TLV / TFV / LV / FV 计算。
- 不用 fixed cooperative zone 替代 APS candidate window。
- 不在 freeze 后插入新车影响本步算法。
- 不按正在换道车辆的 physical `y` 最近车道连续切换 longitudinal role。
- 不在 Step 3 创建 same-step maneuver relation overlay；Step 3 只提供可引用的 relation basis，overlay 由后续 CUC lane-change command 创建。

## 3. 上游 spec 引用

- 时间步总纲：引用 Step 0 清理、Step 1 边界生成、Step 2 冻结、Step 3 关系刷新，以及所有模块只读冻结 `S(t)` 的规则。
- 公式映射：引用 Step 3 中 TLV / TFV / LV / FV、active lane-change relation、不按 physical y 连续切换关系的论文语义与第一版约束。
- 状态与模块接口：引用 `SimulationState`、`RelationsSnapshot` 的生命周期，command / next-state 不进入 `S(t)`，relations 每步生成、每步消费、下一步重建。
- 代码数据结构：引用 `VehicleSpec`、`VehicleState`、`SimulationState`、`RelationsSnapshot`、`LaneChangeNeighborhood`、`ActiveManeuverRelation`、`SameStepManeuverRelationOverlay` basis、EventRecord、SanityCheckRecord 字段权威。
- 道路几何：引用 `x_global` 为算法内部唯一纵向坐标，`x_plot = x_global - warmup_length` 只用于绘图；引用 lane 1 / lane 2 / on-ramp centerline、merging zone、APS candidate window。
- 参数规格：引用 `dt`、`L_cr`、`x0_m_global`、`x_ramp_end_global`、lane centerline y 值、vehicle length 等参数来源和单位。
- 车辆模型：引用车辆模型只读冻结 `S(t)`、logical longitudinal role 不按 physical y 跳变、纵向先于横向的后续消费前提。
- 输出日志：引用 Step 0-3 需要记录的 cleanup、boundary generation、freeze、relation_refresh、geometry consistency、x_plot sanity、PNG quicklook 语义。
- MVS：引用通用 sanity baseline、MVS 默认 module_overrides、`MVS-APS-*` 的 candidate window 输入、`MVS-CUC-*` 的 TLV / TFV / LV / FV 输入。
- 复现讨论对齐：引用第一版不做普通主线主动换道、不做 MPC tracking、不做 platoon 的边界。

## 4. 行为契约

- Given：上一时间步结束后的 persistent state、APS assignment cache、active maneuver trajectory state、command buffer、next-state buffer。When：执行 Step 0 cleanup。Then：移除已驶出车辆，清空本步 command / next-state buffer，生成 cleanup event candidate，不清空 APS cache 或 active maneuver state。
- Given：ScenarioConfig 的 module_overrides 中 `boundary_generation_enabled=false`。When：执行 Step 1 boundary generation hook。Then：不生成随机车辆，记录 boundary generation skipped / disabled event candidate，active vehicle set 保持 ScenarioConfig 初始集合。
- Given：boundary generation enabled 且入口条件满足。When：执行 Step 1。Then：只允许在 freeze 前向 active set 加入新车，并记录 boundary_generation event；不得在 Step 2 后插入。
- Given：pre-freeze active vehicle set 与配置引用。When：执行 Step 2 freeze。Then：生成 `SimulationState` snapshot，包含 t、step、dt、active_vehicle_ids、vehicle_states、vehicle_specs、APS cache、active_maneuvers 和 config refs；不包含 command、next-state、relations 或 history。
- Given：冻结 `S(t)` 已生成。When：Step 3 lane ordering。Then：每条 lane 按 `x_global` 排序，输出 `RelationsSnapshot` 与 `relation_refresh` event candidate，并设置 `x_plot_used_in_algorithm_path=false` sanity。
- Given：车辆处于 `lane_change_state == executing`。When：Step 3 生成 active lane-change relation。Then：CV 以 TLV 为主 leader，TFV 与 FV 仍可把该 CV 视为 leader；不得按 physical y 最近 lane 切换关系。
- Given：on-ramp MV 与道路参数 `L_cr`。When：APS candidate window resolver 被调用。Then：输出 `[x_MV_global - L_cr, x_MV_global + L_cr]` 的窗口结果，供后续 APS 消费；不得用 fixed cooperative zone。
- Given：lane centerline resolver 收到 lane id。When：解析 lane centerline。Then：返回上游几何规格中的 lane 1、lane 2、on-ramp y 值，并记录 geometry consistency sanity。
- Given：region resolver 收到车辆 `x_global` 与 road role。When：判断 merging zone / mainline / on-ramp downstream boundary。Then：输出派生区域结果，不修改 VehicleState。

## 5. 允许实现的代码对象

- domain / state objects
  - `SimulationState` freeze builder 的实现入口
  - `RelationsSnapshot`
  - `LaneChangeNeighborhood`
  - `ActiveManeuverRelation`
  - geometry resolver result objects；仅复用已定义字段
- command / next-state objects
  - 本阶段只清空 `CommandBuffer` 与 `NextStateBuffer`；不新增业务 command。
- step runner / service functions
  - `step0_cleanup_and_prepare`
  - `step1_prefreeze_boundary_generation_hook`
  - `freeze_simulation_state`
  - `refresh_relations_snapshot`
  - `resolve_lane_ordering_by_x_global`
  - `resolve_lane_centerline`
  - `resolve_region`
  - `resolve_aps_candidate_window`
- event / sanity helpers
  - `emit_cleanup_event_candidate`
  - `emit_boundary_generation_event_candidate`
  - `emit_freeze_event_candidate`
  - `emit_relation_refresh_event_candidate`
  - `run_geometry_sanity_baseline`
  - `assert_x_plot_not_used_in_algorithm_path`
- scenario tests
  - `test_step0_clears_buffers_retains_cache_and_maneuver_state`
  - `test_step1_disabled_boundary_generation_does_not_insert_vehicle`
  - `test_step2_freeze_is_immutable_to_late_vehicle_insert`
  - `test_step3_lane_ordering_uses_x_global`
  - `test_aps_candidate_window_uses_lcr_not_fixed_cooperative_zone`
  - `test_active_lane_change_relation_not_switched_by_physical_y`
- regression tests
  - `test_simulation_state_excludes_command_next_state_history_relations`
  - `test_x_plot_is_absent_from_algorithm_state_and_relations`

## 6. 先写失败测试

- unit tests:
  - Step 0 清空 command / next-state buffer 失败时测试失败。
  - Step 0 清空 APS assignment cache 或 active maneuver state 时测试失败。
  - Step 1 在 `boundary_generation_enabled=false` 下生成车辆时测试失败。
  - Step 2 生成的 SimulationState 包含 CommandBuffer、NextStateBuffer、RelationsSnapshot、TrajectoryRecord、EventRecord 或 SanityCheckRecord 时测试失败。
  - Step 3 lane ordering 使用 x_plot 或输入顺序时测试失败。
  - APS candidate window 不是 `[x_MV_global - L_cr, x_MV_global + L_cr]` 时测试失败。
  - lane centerline 与几何规格不一致时测试失败。
- integration tests:
  - 一个包含 lane 1、lane 2、on-ramp 多车的静态场景执行 Step 0-3 后，active_vehicle_ids 与 freeze snapshot 一致。
  - freeze 后尝试插入新车，不得影响本步 relations snapshot。
  - 正在换道车辆 y 更接近目标 lane 时，logical longitudinal role 仍按状态机关系表达，而不是 physical y 最近 lane。
- MVS scenario tests:
  - 使用 `MVS-APS-FAIL-EMPTY` 初始车辆，仅执行到 Step 3，expected pre-APS lane 2 candidates 可被 relations / geometry 输出支撑。
  - 使用 `MVS-CUC-1A_override_choice1` 的邻接车辆结构，仅执行到 Step 3，TLV / TFV / LV / FV basis 可观测。
- event log assertions:
  - 必须产生 cleanup event、freeze event、relation_refresh event。
  - boundary generation disabled 时必须产生 disabled / skipped 或 equivalent event，不得静默跳过。
  - relation_refresh event payload 必须包含 lane id、ordered vehicle ids、ordered `x_global`。
- sanity check assertions:
  - `x_plot_used_in_algorithm_path=false`。
  - `geometry_inconsistency=false`。
  - baseline sanity 中 collision、near_collision、state_machine_inconsistency、unexpected_ordinary_lane_change_attempt、multiple_commit_for_one_vehicle 均记录为 false 或 not_applicable。
- PNG / artifact assertions:
  - expected_png_features v0 注册 lane centerline quicklook。
  - expected_png_features v0 注册 merging zone boundary quicklook。
  - expected_png_features v0 注册 APS candidate window quicklook。
  - 本阶段不要求正式 PNG 文件存在。

## 7. 验收证据

- EventRecord:
  - `boundary_generation`：记录 disabled / skipped 或生成车辆结果；发生在 freeze 前。
  - `relation_refresh`：记录 lane ordering、leader / follower、TLV / TFV / LV / FV、active maneuver relation basis。
  - `sanity_check` 或 geometry event：记录 `x_plot_used_in_algorithm_path=false`。
- SanityCheckRecord:
  - `geometry_inconsistency=false`。
  - `x_plot_used_in_algorithm_path=false`。
  - 通用 sanity baseline 记录。
- TrajectoryRecord:
  - 本阶段不要求 commit 后 trajectory；若 runner 需要，可仅检查初始 snapshot 不被 Step 0-3 日志反写。
- PNG feature:
  - lane centerline visible / registered。
  - merging zone boundary visible / registered。
  - APS candidate window visible / registered。
- commit invariant:
  - 本阶段未执行 commit；但必须证明 Step 0-3 不写 `S(t+dt)`。
- ScenarioConfig expected matcher:
  - 能被 P01 matcher 消费 relation / geometry event 和 sanity records。
- `source / reason / is_engineering_patch`:
  - 若记录 same-step overlay basis 或 x_plot 防护事件，应标为第一版实现约束；不得写成论文新算法。

## 8. 完成标准

- Step 0-3 可作为独立 targeted slice 执行。
- `MVS-APS-FAIL-EMPTY` 和一个 CUC 邻接关系测试场景可执行到 Step 3，并产生可匹配的 relations / geometry 证据。
- required sanity baseline 至少记录：collision=false、near_collision=false、state_machine_inconsistency=false、unexpected_ordinary_lane_change_attempt=false、multiple_commit_for_one_vehicle=false、x_plot_used_in_algorithm_path=false。
- probe 场景只要求 relations 可观测，不要求 CUC utility 或 lane-change command。
- deferred 场景不进入本阶段强验收。
- 必须生成 cleanup、boundary_generation disabled/skipped、freeze、relation_refresh 事件。
- APS candidate window 必须使用 `[x_MV_global - L_cr, x_MV_global + L_cr]`。
- lane ordering 必须使用 `x_global`。
- freeze 后不得插入新车影响本步算法。
- active lane-change / merge relation 不得按 physical y 最近 lane 连续切换。

## 9. 回归保护

- 所有算法内部使用 `x_global`，`x_plot` 只用于绘图。
- Step 1 是 pre-freeze population update；Step 2 后不得新增本步可见车辆。
- `SimulationState` 不包含 command、next-state、relations 或 history。
- `RelationsSnapshot` 是本步派生快照，不跨步持久化。
- command / next-state buffer 可以在 Step 0 清空，但 APS assignment cache 和 active maneuver trajectory state 不得被误清。
- fixed cooperative zone 不得替代 APS candidate window。
- 正在换道或合流车辆的 longitudinal role 不得仅由 physical y 决定。
- Step 3 只提供 same-step overlay basis，不创建 overlay command。
- 本阶段产生的 event / sanity 不能反向改变车辆状态。
