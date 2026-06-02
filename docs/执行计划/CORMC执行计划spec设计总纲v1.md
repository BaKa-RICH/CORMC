# CORMC 执行计划 Spec 设计总纲 v1

## 1. 文档定位

本文档用于指导 `docs/执行计划/Pxx-*.md` 的写作。它不是代码实现计划的普通任务列表，而是一套面向 CORMC 第一版复现的文档驱动开发规则。

执行计划的组织原则是：

```text
时间步流程切片 + MVS 验收门禁 + 可追溯证据链
```

每份执行计划都必须围绕一次或一段 `S(t) -> S(t+dt)` 时间步流程展开，并用一个或多个 MVS 场景证明该阶段的行为正确。执行计划不应完全按代码模块排队，也不应只写“实现 APS / CUC / CMC”这类泛任务。

第一版复现的核心目标是：

```text
先跑通 CORMC 主链路，保证算法语义、状态边界、日志证据和最小验证场景可执行。
```

第一版不以复刻论文全部数值实验为强验收目标。

## 2. 日志、Sanity、MVS Runner 与 PNG 的分层落地原则

日志、sanity check、MVS 验收和 PNG 口径不是最后阶段才开始实现的内容，而应从最小可执行场景开始逐步落地，并贯穿每个算法时间步切片。

第一版执行计划采用：

```text
早期最小可观测性 + 阶段性 targeted MVS + 最终全量回归收口
```

分层规则：

1. P01 必须提供 MVS Runner v0、ScenarioConfig 加载、expected_events / expected_sanity_checks matcher、required / probe / deferred 区分，使 MVS 场景可以作为失败测试执行。
2. P03 必须提供 EventRecord / SanityCheckRecord / TrajectoryRecord 的最小内存记录能力，以及 commit / sanity 断言能力。
3. P04-P10 每个算法切片必须同步产出本阶段所需 event、sanity check 和 targeted MVS 验收断言；不得等到 P11 再补日志。
4. PNG renderer 可以在早期只保留 expected_png_features 注册、trajectory / event 可渲染数据和 quicklook 口径；P11 再完成正式 PNG 渲染、人工复核特征和 artifact record。
5. P11 的职责是完成交付级导出、正式 PNG、artifact record、全部 required MVS smoke suite 聚合和 regression report；不是日志、sanity、MVS runner 或 PNG 口径的首次实现阶段。

任何 P04-P10 阶段如果无法用 event / sanity / targeted MVS 证明自己通过，则不得声称完成。


## 3. 执行计划文档统一模板

每份 `docs/执行计划/Pxx-*.md` 应使用统一模板，避免后续 agent 发散。

```md
# Pxx - Step 范围：标题 / MVS Gate

## 0. Slice Identity

- Algorithm-Step Coverage:
  - Primary Steps:
  - Secondary Steps:
- MVS Acceptance Gate:
  - required:
  - probe:
  - deferred:
- 本阶段解锁的能力:
- 本阶段不要求通过的后续场景:

## 1. 本阶段目标

只描述本阶段要落地的最小可验证时间步切片。

本节应回答：

- 本阶段覆盖 CORMC 时间步流程中的哪些步骤。
- 本阶段让哪些 MVS 场景从失败变为通过。
- 本阶段为后续阶段提供哪些稳定输入、command、next-state、event 或 sanity check。
- 本阶段需要哪些 targeted MVS event / sanity / PNG 证据证明完成。

## 2. 非目标 / 禁止事项

列出本阶段不做什么，尤其是：

- 第一版明确关闭项。
- 暂不进入 required suite 的 probe / deferred 场景。
- 不得临时决定的字段、参数、公式或工程补丁。
- 不得绕过 `S(t) -> command / next-state -> commit -> S(t+dt)` 的状态提交规则。

## 3. 上游 spec 引用

逐条列出必须阅读的 spec，并说明本阶段引用它的哪类权威。

建议固定写法：

- 时间步总纲：引用时间步顺序、Step 调度位置、冻结 `S(t)` 与 commit 规则。
- 公式映射：引用论文公式、第一版简化、第一版关闭、工程补丁分类。
- 状态与模块接口：引用模块读写边界、command / next-state / cache / state transition 语义。
- 代码数据结构：引用 enum、dataclass、buffer、record、ScenarioConfig 与 expected_* 字段权威。
- 道路几何：引用 `x_global`、`x_plot`、lane centerline、merging zone、APS candidate window。
- 参数规格：引用参数来源、数值、单位、来源状态，不重新定义参数。
- 车辆模型：引用纵向、横向、CUC、CMC、speed cap、Eq.10 消费等车辆运动规则。
- 输出日志：引用 event、sanity check、trajectory、PNG 与 artifact 验收语义。
- MVS：引用具体 `scenario_id`、setup、expected_events、expected_sanity_checks、expected_png_features。

## 4. 行为契约

用 Given / When / Then 写清楚本阶段行为。

每条行为契约至少说明：

- Given：输入来自冻结 `S(t)`、relations、assignment cache、command buffer、preloaded state 或 ScenarioConfig。
- When：本阶段执行哪个 Step 或 Step 子分支。
- Then：输出写入 command、next-state、cache update request、event candidate、sanity check 或 commit result。

禁止在行为契约中写“直接更新车辆真实状态”，除非该行为明确发生在 commit 阶段。

## 5. 允许实现的代码对象

只列结构、函数、模块、测试文件，不写完整代码。

如需新增核心字段，必须先修订 `CORMC代码数据结构设计_整理版.md`，再修改执行计划，不得在代码中暗增字段。

本节建议分组：

- domain / state objects
- command / next-state objects
- step runner / service functions
- event / sanity helpers
- scenario tests
- regression tests

## 6. 先写失败测试

每份执行计划必须先定义失败测试和验收断言，再允许实现。

测试分为：

- unit tests
- integration tests
- MVS scenario tests
- event log assertions
- sanity check assertions
- PNG / artifact assertions，如适用

测试断言来源必须是上游 spec 或 MVS 文档，不得由实现 agent 临时想象。

P04-P10 的 MVS scenario tests 必须是 targeted gate，绑定本阶段时间步切片；P11 只能聚合已通过的 targeted gate，不能作为前面算法切片首次被验证的阶段。

## 7. 验收证据

每个验收点必须说明证据来自哪里。

可接受证据包括：

- `EventRecord`
- `SanityCheckRecord`
- `TrajectoryRecord`
- PNG feature
- commit invariant
- ScenarioConfig expected matcher
- `source / reason / is_engineering_patch`

如果某行为属于工程补丁，必须在 event 或相关记录中保留来源和原因。

除 P00 这类纯追踪矩阵阶段外，每个 Pxx 都应定义本阶段最小 event / sanity 输出。P04-P10 还必须说明这些输出如何被 targeted MVS matcher 消费。

## 8. 完成标准

列出本阶段完成后必须满足的条件。

至少包括：

- 哪些 required MVS 必须通过。
- 哪些 probe 场景只要求可观测。
- 哪些 deferred 场景不进入本阶段强验收。
- 必须生成哪些事件。
- 必须通过哪些 sanity baseline。
- 哪些异常必须被显式记录。

## 9. 回归保护

列出后续阶段不得破坏的 invariant。

常见 invariant 包括：

- 所有算法内部使用 `x_global`，`x_plot` 只用于绘图。
- 所有模块只读冻结 `S(t)`。
- command / next-state 不反写真实状态。
- commit 是唯一生成 `S(t+dt)` 的阶段。
- 每辆车每步最多提交一次最终状态。
- `lane_change_state == executing` 与 `merge_state == executing` 不得同车同时为真。
- `CUCChoice` 不是跨步持久控制状态。
- 工程补丁不得写成论文原算法。
```

## 4. 建议执行计划拆分

P13.5 后，执行计划路线按当前代码事实重新校准：

- P11 已具备输出 / PNG / artifact manifest / regression report 的基础函数，以及 full required MVS suite aggregation 基础。
- P12 已完成 deterministic CORMC full simulation loop：固定场景、关闭随机边界生成，按 Step0-11 多步推进并能生成 demo PNG。
- P13 已完成 required MVS closure：20 个 required MVS 通过 official `MVS-*` ID、official loader、统一 deterministic runner 和 P11 matcher 进入验收，当前 suite 为 20 green。
- P14 是下一步正式 artifact bundle 阶段，把“测试中能生成输出”升级为“一次仿真自然生成正式输出包”。
- P15 是 engine core consolidation，但必须等 P14 产出可比较的 trajectory / event / sanity / PNG / manifest / regression report baseline 后再启动。
- P16 才进入 seeded random simulation：边界车辆生成、arrival headway、随机车型、随机 CHV compliance。
- P17 才进入论文实验网格、批量运行、统计指标和复现报告。

P13.5 复核命令与结果摘要：

- `python -m pytest tests\test_p11_output_export.py tests\test_p12_deterministic_simulation_loop.py` -> `30 passed`
- P11 suite summary -> `suite_status=passed`，`required_green=20`，`required_failed=[]`，`required_blocked=[]`，`runner_gaps=[]`，`probe=[MVS-CUC-1B_real_utility_probe]`，`deferred=[MVS-CUC-1C_real_utility_choice1_locked]`

### P00 - Spec 宪法、权威边界与二维追踪矩阵

**Step 覆盖**：全局，不实现代码。  
**MVS Gate**：静态追踪检查，不跑场景。  
**目标**：建立 `时间步 Step × MVS 场景 × 上游 spec × event/sanity/PNG 证据` 的二维矩阵。

本阶段应固定以下边界：

- 哪些内容属于论文原公式。
- 哪些内容属于第一版简化。
- 哪些内容属于第一版关闭。
- 哪些内容属于工程补丁。

工程补丁至少包括：

- `first_APS(MV)`
- assignment invalid
- immediate APS refresh
- 多 MV 共享 CV 仲裁
- same-step maneuver relation overlay
- 每车每步只提交一次
- boundary speed cap 不可行时的保守处理入口
- unexpected ordinary lane-change attempt

**验证点**：

- 每个后续 Pxx 至少绑定一个 Step 范围。
- 每个后续 Pxx 至少绑定一个 MVS 或 sanity gate。
- 每个后续 Pxx 至少引用一个上游 spec。
- 工程补丁必须带 `source / reason / is_engineering_patch`。
- 执行计划不得首次决定核心字段。

---

### P01 - MVS Runner、ScenarioConfig 与验收断言语言

**Step 覆盖**：测试执行层，服务所有 Step。  
**MVS Gate**：能加载 `MVS-APS-FAIL-EMPTY` 与 `MVS-COMMIT-1-lite` 的失败测试。  
**目标**：先让 MVS 成为可执行验收合同，而不是自然语言清单。

本阶段应落地：

- ScenarioConfig loader 或等价测试配置加载方式。
- expected_events matcher。
- expected_sanity_checks matcher。
- expected_png_features matcher 占位（只表达可复核特征，不要求正式 PNG 渲染）。
- targeted scenario runner。
- 失败测试输出格式。
- required / probe / deferred 状态区分。
- 数值 tolerance 机制。

**验证点**：

- 能以 failing-test 方式运行单个 MVS。
- 能表达 event 匹配。
- 能表达 sanity check 匹配。
- 能表达 required / probe / deferred 区分。
- 能在 targeted scenario report 中显示失败原因。
- `MVS-CUC-1B_real_utility_probe` 不阻塞 required suite。
- `MVS-CUC-1C_real_utility_choice1_locked` 不进入第一版强验收。

---

### P02 - Step0-3：清理、pre-freeze 生成、冻结 S(t)、relations snapshot 与几何口径

**Step 覆盖**：Step 0、Step 1、Step 2、Step 3。  
**MVS Gate**：通用 sanity baseline，并为后续 `MVS-APS-*`、`MVS-CUC-*` 提供正确 relations。  
**目标**：固定空间、状态冻结和关系刷新三件事。

本阶段应落地：

- Step 0 清理与本步 buffer reset。
- Step 1 pre-freeze boundary generation hook。
- Step 2 freeze `S(t)`。
- Step 3 relations snapshot。
- lane ordering by `x_global`。
- lane centerline resolver。
- region resolver。
- APS candidate window resolver。

**验证点**：

- `x_plot_used_in_algorithm_path = false`。
- APS candidate window 使用 `[x_MV_global - L_cr, x_MV_global + L_cr]`。
- 不使用 fixed cooperative zone 替代 APS candidate window。
- lane ordering 使用 `x_global`。
- freeze 后不得插入新车影响本步 APS/CUC/CMC/纵向/横向。
- 正在换道车辆的 longitudinal role 不得仅按 physical `y` 最近车道切换。

---

### P03 - Step9-10：Command / NextState / Commit / Event / Sanity / Trajectory 最小闭环

**Step 覆盖**：Step 9、Step 10。  
**MVS Gate**：`MVS-COMMIT-1-lite`。  
**目标**：在算法复杂化前，先证明每车每步只提交一次、日志不反写运动、CUCDecision 不持久化。

本阶段应落地：

- `CommandBuffer`
- `NextStateBuffer`
- candidate selection rule
- commit_step
- duplicate commit guard
- immutable snapshot guard
- `TrajectoryRecord`
- `EventRecord`
- `SanityCheckRecord`
- baseline sanity check runner
- OutputHistory in-memory v0
- commit event
- event / sanity assertion helper v0

本阶段不要求：

- 稳定文件导出格式。
- 完整 PNG renderer。
- 全量 smoke suite runner。
- regression report。

**验证点**：

- 每车每步最多一个 final candidate。
- duplicate candidate 触发 `multiple_commit_for_one_vehicle`。
- next-state 不反写 `S(t)`。
- commit 是唯一真实状态写入点。
- SimulationState 不包含 command、next-state 或 history。
- `CUCDecision` 只能进入 command / event / history，不进入下一步控制状态。

---

### P04 - Step4A：MV 未入 merging zone 时的 APS / cache / effective assignment

**Step 覆盖**：Step 4 的 APS 分支。  
**MVS Gate**：`MVS-APS-FAIL-EMPTY`、`MVS-APS-FAIL-CACHE`、`MVS-APS-1`、`MVS-APS-2`、`MVS-APS-3`、`MVS-APS-4`。  
**目标**：实现 APS 触发、候选搜索、case 1-4、CLV/CFV、col、Eq.10 消费对象和 assignment cache 语义。

本阶段应落地：

- APS trigger resolver：`first_APS` / `APS_due` / reuse cache。
- APS candidate collector。
- `T*_MV` 与 candidate prediction。
- CLV / CFV selector。
- APS case classifier。
- Eq.10 desired spacing command source。
- APS failure reason。
- assignment cache update / retain / invalid / cleanup request。
- `EffectiveAssignmentThisStep`。
- APS trigger / assignment / failure / cache action event。
- `MVS-APS-*` targeted event / sanity matcher。

**验证点**：

- 候选不足且无 cache 时不伪造 assignment。
- APS failure 不静默覆盖旧 cache。
- `MVS-APS-1/2/3/4` 的 case、CLV、CFV、`col_CLV`、`col_CFV` 正确。
- case 2 / 4 只给 CFV 产生 Eq.10 desired spacing。
- case 3 不给 CLV 套 Eq.10。
- `first_APS(MV)` 必须标记为工程补丁。
- 非 APS 周期通过 effective assignment 复用 cache，不修改真实状态。

---

### P05 - Step4B：MV 在 merging zone 的 CMC decision、assignment validation、Eq.53、boundary cap command

**Step 覆盖**：Step 4 的 CMC waiting / executing 分支。  
**MVS Gate**：`MVS-CMC-1`、`MVS-CMC-2`、`MVS-ASSIGN-1`，并为 `MVS-SAFE-1A` 准备 speed cap。  
**目标**：实现 MV 进入 merging zone 后的 assignment 有效性检查、Eq.53 gap 判断、merge command、waiting command、boundary speed cap command。

本阶段应落地：

- CMC trigger resolver。
- assignment validity checker。
- dynamic gap calculator。
- Eq.53 gap checker。
- boundary speed cap calculator。
- waiting command。
- merge command。
- speed cap command。
- assignment invalid event。
- CMC decision / boundary cap / merge transition event。
- assignment validity 与 speed cap targeted sanity matcher。

**验证点**：

- `MVS-CMC-1`：Eq.53 pass，开始合流。
- `MVS-CMC-2`：Eq.53 fail，继续 waiting。
- `MVS-ASSIGN-1`：assignment invalid 不偷换 actual leader/follower。
- boundary speed cap 只作为 command / speed constraint 输出，不直接提交 MV 真实速度。
- `merge_state == executing` 后不重新判断是否开始合流。

---

### P06 - Step5：Cooperative Request 汇总与多 MV 冲突仲裁

**Step 覆盖**：Step 5。  
**MVS Gate**：`MVS-CONFLICT-1A`、`MVS-CONFLICT-1B`。  
**目标**：从有效 APS assignment 中抽取 `col = 1` 的 CLV/CFV 请求，并处理多个 MV 请求同一 CV 的冲突。

本阶段应落地：

- `CooperativeRequest`
- request collector
- conflict grouping by `cv_id`
- priority resolver
- conflict result event
- loser waiting / blocked reason event
- cooperative request event matcher
- one-active-request-per-CV sanity check

默认仲裁优先级：

1. 已在 merging zone 的 MV 优先。
2. `T*_MV` 更小者优先。
3. 距离 `x0^m` 更近者优先。

该仲裁属于第一版工程补丁，不能写成论文原算法。

**验证点**：

- 只有 `col = 1` 的 CLV / CFV 生成 cooperative request。
- 同一 CV 同一步最多接收一个 active cooperative request。
- `MVS-CONFLICT-1A`：merging zone MV 优先。
- `MVS-CONFLICT-1B`：`T*_MV` 更小者优先。
- conflict event 必须记录 winner、loser、priority basis、`is_engineering_patch = true`。

---

### P07 - Step6：CUC choice、compliance、lane-change command、same-step overlay

**Step 覆盖**：Step 6。  
**MVS Gate**：`MVS-CUC-1A_override_choice1`、`MVS-CUC-1B_real_utility_probe`、`MVS-CUC-2`、`MVS-CUC-3`；`MVS-CUC-1C_real_utility_choice1_locked` deferred。  
**目标**：实现 CUC 是 maneuver choice，不是控制器；它只能生成 lane-change command、spacing override command、event，不直接更新车辆位置。

本阶段应落地：

- active cooperative request resolver。
- CUC utility calculator。
- target lane TT safety checker。
- compliance resolver。
- `CUCDecision` event。
- lane-change command。
- desired spacing override command。
- same-step maneuver relation overlay。
- CUC choice / compliance / safety fallback event。
- active lane-change skip-CUC sanity check。
- `MVS-CUC-*` targeted event / sanity matcher。

**验证点**：

- `MVS-CUC-1A`：override choice 1 时产生 lane-change command 和 same-step overlay。
- `MVS-CUC-1B`：真实 utility 输入、`U1/U2`、final choice 可观测。
- `MVS-CUC-2`：目标车道 unsafe 必须回退 stay lane 2，并让 CFV 消费 Eq.10。
- `MVS-CUC-3`：non-compliant CHV 不执行协同建议、不消费 Eq.10、不换道。
- active lane-change 车辆跳过 CUC，不重新选择目标。
- `CUCChoice` 不得成为跨步持久控制状态。

---

### P08 - Step7：纵向模型、Eq.10 spacing override、speed cap 合成

**Step 覆盖**：Step 7。  
**MVS Gate**：`MVS-CUC-2`、`MVS-CUC-3`、`MVS-SAFE-1A_waiting_cap`。  
**目标**：围绕一次冻结 `S(t)` 的 Step 7 纵向候选生成，消费 P04 / P07 产生的 Eq.10 desired spacing handoff 与 P05 产生的 boundary speed cap command，实现 CAV cruising / gap-regulating、CHV / IDM、CFV spacing override 和 MV planning speed 合成。

本阶段应落地：

- CAV cruising。
- CAV gap-regulating / CPID。
- longitudinal controller memory。
- CHV / IDM。
- MV on-ramp longitudinal behavior。
- desired spacing override consumer。
- speed cap composer。
- front-collision / conservative speed fallback composer。
- longitudinal candidate writer。
- longitudinal_model event。
- desired spacing override consumption event。
- speed cap consumption sanity check。

**验证点**：

- CAV 无 leader 或 spacing 足够大时使用 cruising。
- CAV 跟驰时使用 gap-regulating / CPID。
- CHV 使用 IDM；compliance 只影响是否接受 CUC 建议，不改变 IDM 本质。
- `MVS-CUC-2` 在 P08 验完整纵向消费：P07 unsafe fallback 留在 lane 2 的 CFV 才消费 Eq.10 desired spacing override，并生成 longitudinal candidate / event。
- case 2 / 4 中留在 lane 2 的 CFV 才消费 Eq.10。
- case 3 不给 CLV 套 Eq.10。
- non-compliant CHV 不消费 Eq.10。
- `MVS-SAFE-1A_waiting_cap` 在 P08 验完整 planning speed 合成：P05 boundary speed cap、front fallback、candidate speed 合成取最保守速度。
- P08 只写 longitudinal candidate / planning speed / event / sanity，不提交真实 `x / y / v / a / lane / state`。
- speed cap 先进入纵向 planning speed，再由 P09 横向轨迹消费。

---

### P09 - Step8：正弦横向轨迹、front-collision fallback、active maneuver progress

**Step 覆盖**：Step 8。  
**MVS Gate**：`MVS-SAFE-1B_executing_cap_lateral_consumption`、`MVS-SAFE-2`。  
**目标**：围绕一次冻结 `S(t)` 与 P08 planning speed 的 Step 8 横向候选生成，消费 P07 lane-change command / same-step overlay 与 P05 merge command，实现 CUC lane 2 -> lane 1 与 MV on-ramp -> lane 2 的正弦轨迹更新、active maneuver progress 和 completion candidate。

本阶段应落地：

- `ManeuverTrajectoryState` consumer。
- sine trajectory update。
- candidate lateral kinematics。
- candidate maneuver progress。
- completion detector。
- front-collision fallback consumption / event hook。
- boundary cap consumption in lateral trajectory。
- lateral_trajectory event。
- maneuver progress / completion sanity check。
- PNG feature 所需的 lane-change / merge marker 数据。

**验证点**：

- CUC lane change：lane 2 -> lane 1。
- CMC merge：on-ramp -> lane 2。
- active maneuver 不因每步 relations 变化重置 `start_x/start_y/target_y`。
- `MVS-SAFE-1B`：executing 状态横向轨迹消费 capped speed。
- P09 消费 P08 planning speed 推进正弦轨迹，不重新计算 P08 纵向控制、不重跑 P07 CUC、不重判 P05 Eq.53。
- 普通主线主动换道保持关闭。
- completion detector 只生成 candidate / state transition request；完成换道 / 合流只在 commit 阶段正式更新 lane / role / state。

---

### P10 - Step4-9 集成切片：APS / CMC / CUC / 纵向 / 横向 / commit 同步闭环

**Step 覆盖**：Step 4、Step 5、Step 6、Step 7、Step 8、Step 9。  
**MVS Gate**：`MVS-E2E-1`、`MVS-COMMIT-1-full`。  
**目标**：验证多个时间步切片组合后仍保持冻结输入、command / next-state 分离、唯一 commit、cache 生命周期和 active trajectory 生命周期。

本阶段不应新增核心算法，而应做组合验收和回归保护。

本阶段应落地：

- E2E event chain matcher。
- cross-step sanity aggregation。
- `MVS-E2E-1` targeted regression runner。
- `MVS-COMMIT-1-full` targeted regression runner。
- engineering patch trace checker。

**验证点**：

- `MVS-E2E-1`：APS case 1 -> no CUC -> CMC Eq.53 pass -> merge start -> commit。
- `MVS-COMMIT-1-full`：非 APS 周期复用 cache。
- active lane-change 不重跑 CUC。
- merge executing 不重判开始。
- active trajectory 可加载并延续。
- 每车每步只提交一次。
- 所有工程补丁 event 可追溯。

当前状态：P10 的集成责任已由后续 P12 deterministic loop 和 P13 required MVS closure 证明闭合。后续文档和实现不得再声称 `MVS-E2E-1` 或 `MVS-COMMIT-1-full` 仍缺 full runner route。

---

### P11 - Step10：交付级日志导出、PNG 渲染与全量 MVS 回归收口

**Step 覆盖**：Step 10 为主，回收 Step 0-9 已产生的 trajectory、event、sanity、artifact candidate。  
**MVS Gate**：全部 required MVS；probe / deferred 不阻塞。  
**目标**：把 P01-P10 已经逐步落地的 ScenarioConfig、event、sanity check、trajectory record、targeted MVS runner 和 PNG feature expectation，统一收口为可交付的 smoke suite、正式 PNG 输出、artifact record 和 regression report。

#### 前置假设

本阶段不是第一次实现日志和验收能力。进入 P11 前，以下能力必须已经存在：

1. P01 已能加载单个 MVS ScenarioConfig，并执行 targeted scenario test。
2. P03 已有最小 EventRecord、SanityCheckRecord、TrajectoryRecord 或等价内存记录结构。
3. P04-P10 的每个算法切片已经能产生本阶段 required MVS 所需的 expected_events 与 expected_sanity_checks。
4. 每个算法切片已经能通过对应的 targeted MVS gate。
5. expected_png_features 已经能在场景配置中表达；正式渲染和人工复核在本阶段收口。

本阶段应落地：

- trajectory history export。
- event history export。
- sanity check export。
- PNG renderer。
- artifact record。
- full required MVS smoke suite runner。
- regression report。
- probe / deferred 场景的非阻塞报告机制。

**验证点**：

- 所有 required MVS 均有可加载 ScenarioConfig。
- 所有 required MVS 均可由 smoke suite runner 一次性执行。
- 每个 required 场景均产生 expected_events。
- 每个 required 场景均产生 expected_sanity_checks。
- 需要人工复核的场景在 PNG 中能看到 expected_png_features。
- PNG 中使用的 `x_plot` 只作为绘图派生值，不进入算法状态。
- trajectory / event / sanity / PNG / regression report 不反向改变车辆运动。
- probe 场景可以执行并报告，但不阻塞 required suite。
- deferred 场景不进入第一版强验收。

当前状态：P11 的 exporter / renderer / manifest / regression report 基础能力和 full required MVS aggregation 基础已实现；P13 后当前 suite summary 为 `suite_status="passed"`、20 required green、`required_failed=[]`、`required_blocked=[]`、`runner_gaps=[]`。P11 不等同于 P14：P14 才负责让一次仿真自然生成正式输出包。

---

### P12 - Deterministic Full Simulation Loop

**Step 覆盖**：Step 0-11 deterministic 主循环。
**MVS Gate**：`MVS-E2E-1` deterministic loop，多步推进；P12 branch scenarios 观察 CUC / SAFE / active continuation。
**目标**：在关闭随机边界生成的固定场景中，让车辆从 `S(t)` 连续进入 `S(t+dt)`、`S(t+2dt)`，并能生成 demo PNG。

当前状态：P12 已完成。`run_deterministic_simulation()` / `run_one_deterministic_step()` 已按 Step0-11 串联 APS、CMC、cooperative request、CUC、纵向、横向、commit、history 和 time advance；随机 boundary generation、random attributes、paper grid 仍未进入本阶段。

---

### P13 - Required MVS Closure

**Step 覆盖**：MVS loader / runner / matcher closure。
**MVS Gate**：全部 20 个 required MVS；probe / deferred 非阻塞。
**目标**：把原先 blocked 的 required CUC / SAFE / COMMIT-full 等路线接入 official `MVS-*` scenario id、official loader、deterministic runner 和 P11 matcher。

当前状态：P13 已完成。当前 required suite 为 20 green，`suite_status="passed"`，`required_failed=[]`，`required_blocked=[]`，`runner_gaps=[]`。

---

### P14 - Formal Artifact Bundle Baseline

**Step 覆盖**：Step 10 交付输出路径，以 P11 基础能力和 P12/P13 deterministic loop 为输入。
**MVS Gate**：deterministic scenario run 能自然生成正式输出包；required suite regression report 能引用 artifact paths。
**目标**：把 trajectory CSV、event JSONL、sanity JSONL、PNG、manifest、scenario report、regression report 串成一次仿真的正式 artifact bundle。

P14 是 P15 的硬前置。没有 P14 输出基线，不得启动 engine consolidation。

---

### P15 - Engine Core Consolidation

**Step 覆盖**：engine / workspace / recorder / output 边界整理。
**MVS Gate**：P14 artifact baseline 前后可比较，20 required MVS 仍 green。
**目标**：在 P12/P13/P14 保护下逐步让 `simulation_loop.py` 变薄，不推倒重来，不引入随机性。

---

### P16 - Seeded Random Simulation

**Step 覆盖**：Step 1 boundary generation 扩展与 seeded random simulation。
**MVS Gate**：随机入口关闭时不得破坏全部 required MVS；seeded random run 可复现。
**目标**：实现边界车辆生成、arrival headway、随机车型、随机 CHV compliance 和 seed 管理。

---

### P17 - Paper Experiment Grid

**Step 覆盖**：论文实验网格、批量运行、统计指标、复现报告。
**MVS Gate**：不替代 deterministic required MVS；基于 P16 seeded random simulation 批量运行。
**目标**：实现论文级 grid、重复种子、统计指标和复现报告，不与 P16 混在一起。

## 5. 最终验收标准

整套执行计划写完后，应能回答以下问题。

### 5.1 任何一行核心代码为什么存在

每一行核心代码都应能追溯到至少一个来源：

- 时间步 Step。
- 上游 spec。
- 论文公式或第一版处理状态。
- MVS 场景。
- event / sanity / PNG 验收证据。
- commit invariant。

如果无法追溯，应回到执行计划或上游 spec 修订，而不是直接保留代码。

### 5.2 任何一个 bug 如何定位

APS、CMC、cooperative request、CUC、纵向、横向、commit 都必须有结构化证据。

可定位证据包括：

- APS event：触发原因、候选集合、case、CLV、CFV、col、failure reason、cache action。
- CMC event：assignment validity、Eq.53 gap、boundary cap、merge transition。
- cooperative request event：request source、CV、MV、conflict group。
- CUC event：utility 输入、TT safety、compliance、choice、fallback reason。
- longitudinal event：mode、leader、desired spacing、speed cap、planning speed。
- lateral event：maneuver type、progress、target lane、completion、fallback。
- commit event：candidate source、final state、state transition、cache cleanup。
- sanity check：collision、near_collision、boundary_violation、assignment_invalid、state_machine_inconsistency、geometry_inconsistency、unexpected_ordinary_lane_change_attempt、multiple_commit_for_one_vehicle。

不得依赖“肉眼看轨迹猜测 bug”。

### 5.3 任何一个简化或补丁是不是论文原算法

所有非论文原生内容必须被分类为：

- 第一版简化。
- 第一版关闭。
- 工程补丁。

工程补丁必须保留：

```text
source
reason
is_engineering_patch
```

不得把以下内容写成论文原算法：

- `first_APS(MV)`
- assignment invalid
- immediate APS refresh
- 多 MV 共享 CV 仲裁
- same-step maneuver relation overlay
- 每车每步只提交一次
- boundary speed cap 不可行时的保守处理入口

### 5.4 任何一个场景是否真的通过

场景通过不能只看程序是否报错，而必须检查：

- required MVS 是否全部通过。
- expected_events 是否匹配。
- expected_sanity_checks 是否匹配。
- required PNG features 是否可见或可人工复核。
- commit invariant 是否满足。
- `x_plot_used_in_algorithm_path = false`。
- 工程补丁事件是否带 source / reason / is_engineering_patch。

### 5.5 最终 required MVS 通过范围

第一版 required suite 至少覆盖：

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

Probe / deferred 场景不阻塞第一版 required suite，但必须在 ScenarioConfig 和测试报告中被明确标记。


### 5.6 日志与回归是否分层落地

整套执行计划必须体现以下分层完成标准：

- P01 已提供 MVS Runner v0 与 targeted scenario test 能力。
- P03 已提供 Event / Sanity / Trajectory 的最小记录与断言能力。
- P04-P10 每个算法切片都能通过对应 targeted MVS gate。
- P04-P10 每个算法切片都能产生本阶段 required MVS 所需的 event 与 sanity check。
- PNG 所需的 trajectory / event / marker 数据在算法切片阶段逐步产生。
- P11 只做交付级导出、正式 PNG、全量 required smoke suite 聚合和 regression report。
- 任一算法切片不得把“等待 P11 统一补日志”作为完成理由。
