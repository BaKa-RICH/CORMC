# P00 - Step 范围：全局 Spec 宪法、权威边界与二维追踪矩阵 / Static Traceability Gate

## 0. Slice Identity

- Algorithm-Step Coverage:
  - Primary Steps: 全局；不实现任何运行时代码；覆盖 Step 0-11 的权威边界、状态写入规则、MVS 追踪矩阵和证据链口径。
  - Secondary Steps: 为 P01-P12 分配 Step 范围、MVS gate、上游 spec、event / sanity / PNG 证据责任；建立工程补丁分类表。
- MVS Acceptance Gate:
  - required:
    - 静态追踪检查：每个后续 Pxx 至少绑定一个 Step 范围。
    - 静态追踪检查：每个后续 Pxx 至少绑定一个 MVS、targeted sanity gate 或 static gate。
    - 静态追踪检查：每个后续 Pxx 至少引用一个上游 spec。
    - 静态追踪检查：工程补丁必须标注 `source`、`reason`、`is_engineering_patch`。
  - probe:
    - P04-P10 的 targeted MVS 覆盖粒度是否需要继续细分。
    - PNG feature 是否需要在算法切片阶段增加更细 marker。
  - deferred:
    - 全量 required smoke suite 聚合。
    - 正式 PNG renderer、artifact record 与 regression report。
    - 论文级数值实验与宏观指标。
- 本阶段解锁的能力:
  - 固定“论文原公式 / 第一版简化 / 第一版关闭 / 工程补丁”的分类边界。
  - 固定执行计划不得首次决定核心字段的规则。
  - 固定后续每个 Pxx 的 `Step × MVS × 上游 spec × event/sanity/PNG 证据` 追踪要求。
  - 固定 P01、P03、P04-P10、P11 在 MVS runner、日志、targeted gate、正式 artifact 上的分层责任。
- 本阶段不要求通过的后续场景:
  - 不运行 `MVS-APS-*`、`MVS-CUC-*`、`MVS-CMC-*`、`MVS-SAFE-*`、`MVS-ASSIGN-*`、`MVS-CONFLICT-*`、`MVS-COMMIT-*`。
  - 不生成真实 `EventRecord`、`SanityCheckRecord`、`TrajectoryRecord` 或 PNG artifact。
  - 不要求任何仿真时间步可执行。

## 1. 本阶段目标

本阶段只产出执行计划体系的追踪宪法，不产出仿真代码。目标是把 CORMC 第一版复现的主链路拆成可审阅、可验收、可回归保护的文档切片，确保后续实现不会把模块清单误写成任务列表，也不会把工程兜底写成论文原算法。

本阶段覆盖 CORMC 时间步流程中的全局调度边界：Step 0 清理、Step 1 pre-freeze boundary generation、Step 2 freeze、Step 3 relations、Step 4 APS / CMC 分流、Step 5 cooperative request、Step 6 CUC、Step 7 longitudinal、Step 8 lateral、Step 9 commit、Step 10 information integration、Step 11 time advance。

本阶段让后续 MVS 场景具备可追踪的验收入口，但不让任何运行场景从失败变为通过。它只规定：每个 MVS 必须能追溯到一个或多个执行计划；每个执行计划必须声明 required / probe / deferred；每个算法切片必须产生本阶段所需 event、sanity 和 PNG 证据；P11 只能聚合和导出，不能首次补日志或首次验证算法。

本阶段为后续阶段提供以下稳定输入：

- 执行计划文件命名和模板结构。
- 权威 spec 引用边界。
- 二维追踪矩阵字段。
- 工程补丁清单和标注规则。
- event / sanity / PNG 证据分层落地规则。
- required / probe / deferred 的验收语义。

本阶段的 targeted 验证是静态文档验证，不是仿真验证。验证点包括：追踪矩阵完整、每个后续 Pxx 有 Step 绑定、有 gate、有上游 spec、有证据类型、有完成标准、有回归保护。

### 1.1 Pxx 成熟度状态与当前执行边界

P00 的追踪矩阵必须覆盖 P01-P12，但不同 Pxx 在不同阶段具有不同成熟度状态。P00 必须区分以下三类状态：

```text
trace_registered:
    表示该 Pxx 已在 P00 追踪矩阵中登记。
    至少包含 Step 覆盖范围、required / probe / deferred gate、上游 spec 引用类别、预计 event / sanity / PNG / artifact evidence，以及是否属于当前执行阶段。

spec_ready:
    表示该 Pxx 已有完整执行计划文档，可交给实现 agent 按 spec 执行。
    该文档必须包含 Algorithm-Step Coverage、MVS Acceptance Gate、上游 spec 引用、Given / When / Then 行为契约、允许实现的代码对象、失败测试、event / sanity / PNG 或 artifact 验证点、完成标准和回归保护。

implementation_ready:
    表示该 Pxx 已允许进入代码实现。
    进入 implementation_ready 前，必须确认前置 Pxx 已完成，所需 schema / enum / record / buffer 已由上游规格定义，所需 MVS runner / matcher / event / sanity 能表达本阶段验收，并且不需要临时发明核心字段、参数、公式或工程补丁语义。
```

当前阶段执行边界：

```text
P00:
    static traceability gate。
    只做静态追踪和权威边界检查，不实现运行时代码。

P01-P03:
    必须达到 spec_ready 与 implementation_ready。
    允许按各自 spec 完整执行；不得以“薄切片”替代 spec 中要求的 runner、matcher、geometry、freeze、relations、commit、event、sanity、trajectory 和 Step 11 time advance 最小闭环。

P04-P12:
    当前只要求 trace_registered。
    不要求已有完整执行计划，不允许在本轮补写完整 P04-P12 计划，也不允许实现 APS、CMC、cooperative request、CUC、纵向模型、横向轨迹、正式 PNG、全量 smoke suite 或论文级实验入口。
```

P00 的追踪矩阵不得因为 P04-P12 仅处于 trace_registered 状态而判定失败；但如果 P04-P12 未进入矩阵，或缺少总纲级 Step / MVS / evidence 占位，则 P00 静态追踪失败。


## 2. 非目标 / 禁止事项

- 不实现 CORMC 运行时代码。
- 不新增 enum、dataclass、record、buffer、ScenarioConfig 字段。
- 不修改任何上游 spec 的字段权威。
- 不运行 MVS 场景，不生成 PNG，不导出 artifact。
- 不把 `first_APS(MV)`、assignment invalid、immediate APS refresh、多 MV 共享 CV 仲裁、same-step maneuver relation overlay、每车每步只提交一次、boundary speed cap 不可行时的保守处理入口、unexpected ordinary lane-change attempt 写成论文原算法。
- 不把第一版关闭项改成 required：SUMO 对比、严格 MPC 横向 tracking、CMC platoon、普通主线主动换道、全局多 MV gap 优化或全局合流顺序优化均不进入第一版 required gate。
- 不允许任何后续执行计划绕过 `S(t) -> command / next-state -> commit -> S(t+dt)`。
- 不允许 P11 作为 P04-P10 日志、sanity、targeted MVS 或 PNG 口径的首次实现阶段。
- 不允许把 P04-P12 的 trace_registered 占位误写成 spec_ready 或 implementation_ready。
- 不允许以“先做薄切片”为理由削减 P01-P03 已经写入 spec 的完成标准。P01-P03 一旦进入实现，必须按各自 spec 完整执行。

## 3. 上游 spec 引用

- `CORMC执行计划spec设计总纲v1`：引用执行计划统一模板、P00-P12 拆分、日志 / sanity / MVS / PNG 分层落地原则、最终验收标准。
- `CORMC时间步执行顺序梳理.md`：引用 Step 0-11 调度顺序、冻结 `S(t)`、commit 唯一写入、第一版保留 / 关闭边界。
- `CORMC论文公式与实现映射.md`：引用“论文原公式 / 第一版简化 / 第一版关闭 / 工程补丁”分类，以及 Step 到公式 / 论文依据的映射。
- `CORMC状态与模块接口规格.md`：引用模块读写边界、command / next-state / cache / state transition 语义、工程补丁边界。
- `CORMC代码数据结构设计_整理版.md`：引用 enum、dataclass、buffer、record、ScenarioConfig 与 expected_* 字段权威；执行计划不得首次决定核心字段。
- `CORMC道路几何与区域规格.md`：引用 `x_global`、`x_plot`、lane centerline、merging zone、APS candidate window 与区域判定口径。
- `CORMC参数规格.md`：引用参数来源、数值、单位和来源状态；执行计划不得重新定义参数。
- `CORMC车辆模型规格.md`：引用纵向、横向、CUC、CMC、speed cap、Eq.10 消费、正弦轨迹与第一版简化边界。
- `CORMC输出指标与日志验证规格_整理版.md`：引用 event、sanity check、trajectory、PNG、artifact 与 smoke 验收语义。
- `CORMC最小验证场景执行规格.md`：引用 `MVS-*` 场景、setup、expected_events、expected_sanity_checks、expected_png_features、required / probe / deferred 状态。
- `CORMC复现讨论对齐记录.md`：引用第一版目标、保留范围、暂不做内容和论文语义与工程兜底边界。

## 4. 行为契约

- Given：已有 10 份原始 CORMC spec 与执行计划设计总纲。When：编写 P00 静态追踪矩阵。Then：每个后续 Pxx 必须绑定 Step 范围、MVS 或 sanity gate、上游 spec、证据类型和回归保护。
- Given：上游 spec 已将 `ScenarioConfig`、`EventRecord`、`SanityCheckRecord`、`TrajectoryRecord` 设为字段权威。When：后续执行计划发现字段缺口。Then：必须先修订代码数据结构 spec，再修改执行计划，不得在执行计划或实现中暗增字段。
- Given：公式映射与状态接口均声明工程补丁边界。When：执行计划引用工程补丁。Then：必须保留 `source`、`reason`、`is_engineering_patch` 证据要求，不得把补丁写成论文原公式。
- Given：时间步总纲规定所有模块只读冻结 `S(t)`。When：后续执行计划描述算法行为。Then：输出只能写入 command、next-state、cache update request、event candidate、sanity check 或 commit result；不得描述为中途直接更新真实车辆状态。
- Given：MVS 文档区分 required、probe、deferred。When：执行计划绑定场景。Then：required 必须阻塞本阶段验收，probe 只要求可观测，deferred 不进入第一版强验收。
- Given：PNG renderer 分层落地规则已固定。When：P04-P10 产生算法事件。Then：必须同步声明本阶段 expected_png_features 或可渲染 marker 数据；不得等待 P11 首次补齐。
- Given：当前阶段只允许 P00-P03 进入 spec_ready / implementation_ready。When：P00 为 P04-P12 建立追踪矩阵。Then：只能登记总纲级 Step / MVS / evidence 占位，不得补写完整执行计划或实现运行时代码。

## 5. 允许实现的代码对象

本阶段不允许实现运行时代码。只允许产出或维护以下文档对象：

- `docs/SPEC/P00-*.md`
- `docs/SPEC/P01-*.md`
- `docs/SPEC/P02-*.md`
- `docs/SPEC/P03-*.md`
- 后续 P04-P12 的追踪矩阵占位条目（仅 trace_registered，不补写完整执行计划）
- 静态审阅 checklist
- 工程补丁分类表
- Step × MVS × spec × evidence × maturity 二维追踪矩阵

如后续实现需要新增核心字段，必须先修订 `CORMC代码数据结构设计_整理版.md`，再修改执行计划；P00 不得替代码结构 spec 决定字段。

## 6. 先写失败测试

本阶段的失败测试是静态文档测试，不运行仿真。

- unit tests:
  - 缺少任一模板章节时失败。
  - `Algorithm-Step Coverage` 为空时失败。
  - `MVS Acceptance Gate` 未区分 required / probe / deferred 时失败。
  - `上游 spec 引用` 未列出权威来源时失败。
- integration tests:
  - 任一后续 Pxx 没有 Step 范围绑定时失败。
  - 任一后续 Pxx 没有 MVS 或 sanity gate 时失败。
  - 任一算法切片没有 event / sanity / PNG 验证点时失败。
  - P01-P03 没有达到 spec_ready / implementation_ready 要求时失败。
  - P04-P12 未进入 trace_registered 矩阵占位时失败。
  - P00 因 P04-P12 尚未 spec_ready / implementation_ready 而判定失败时失败。
- MVS scenario tests:
  - 本阶段不运行 MVS；仅检查 MVS 场景与执行计划的静态绑定是否存在。
- event log assertions:
  - 工程补丁条目没有 `source`、`reason`、`is_engineering_patch` 要求时失败。
  - P04-P10 未声明本阶段 event 被 targeted MVS matcher 消费时失败。
- sanity check assertions:
  - 任一执行计划没有声明 baseline sanity 或 targeted sanity 时失败。
  - `x_plot_used_in_algorithm_path = false` 没有作为全局 sanity baseline 传播时失败。
- PNG / artifact assertions:
  - P04-P10 未声明 expected_png_features 或可渲染数据时失败。
  - P11 被写成首次实现日志、sanity、MVS runner 或 PNG 口径时失败。

## 7. 验收证据

本阶段可接受证据为静态文档证据，不要求运行时记录。

- 追踪矩阵：每行包含 `Pxx`、Step 范围、MVS gate、上游 spec、event 证据、sanity 证据、PNG / artifact 证据、成熟度状态、当前阶段是否可执行、阻塞条件 / 后续补全点。
- 工程补丁分类表：至少包含 `first_APS(MV)`、assignment invalid、immediate APS refresh、多 MV 共享 CV 仲裁、same-step maneuver relation overlay、每车每步只提交一次、boundary speed cap 不可行时的保守处理入口、unexpected ordinary lane-change attempt。
- 静态 gate 报告：证明 required / probe / deferred 在执行计划中可区分。
- 字段权威检查：执行计划没有首次定义 ScenarioConfig、record、buffer 或 state 核心字段。
- P01-P03 文档本身：均使用统一模板，并包含 Algorithm-Step Coverage、MVS Acceptance Gate、上游 spec 引用、Given / When / Then、失败测试、event / sanity / PNG 验证点、完成标准和回归保护。


### 7.1 Step × MVS × Pxx × evidence × maturity 追踪矩阵

本矩阵是 P00 的静态追踪入口。P01-P03 必须达到 spec_ready / implementation_ready；P04-P12 在当前阶段只要求 trace_registered。后续 P04-P12 的完整执行计划必须在对应阶段另行编写，不能由 P00 代替。

| Pxx | Step 覆盖 | Required Gate | Probe | Deferred | 主要 event evidence | 主要 sanity evidence | PNG / artifact evidence | 主要上游 spec | 成熟度状态 | 当前阶段是否可执行 | 阻塞条件 / 后续补全点 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| P01 | MVS Runner / ScenarioConfig / matcher 执行层 | `MVS-APS-FAIL-EMPTY` failing contract；`MVS-COMMIT-1-lite` failing contract；required / probe / deferred 分类可表达 | probe 场景非阻塞报告 | deferred 场景不进入 required suite | scenario load / matcher report / forbidden event report | expected_sanity matcher / baseline sanity matcher | expected_png_features 口径注册，不要求真实 PNG | 最小验证场景、代码数据结构、日志验证 | spec_ready + implementation_ready | 是 | 必须按 P01 spec 完整实现，不得仅做薄 runner |
| P02 | Step 0-3：cleanup、pre-freeze hook、freeze `S(t)`、relations snapshot、geometry 口径 | geometry / freeze / relations static+targeted sanity；为 `MVS-APS-*`、`MVS-CUC-*` 提供正确空间和关系底座 | active maneuver relations 细粒度检查可后续增强 | 随机边界生成完整机制交给 P12 | cleanup / freeze / relation_refresh event candidate | `x_plot_used_in_algorithm_path=false`、geometry consistency、relations consistency | 可渲染 lane / zone / candidate window 数据，不要求真实 PNG | 时间步总纲、道路几何、参数、状态接口、代码数据结构 | spec_ready + implementation_ready | 是 | P02 可产出 matcher-consumable event/sanity candidate；OutputHistory v0 由 P03 收口 |
| P03 | Step 9-10 + Step 11 最小闭环：Command / NextState / Commit / Event / Sanity / Trajectory / time advance | `MVS-COMMIT-1-lite`；每车每步一次 commit；commit 后 Step10 information integration；Step11 time advance | `MVS-COMMIT-1-full` 的部分前置能力 | 正式导出、正式 PNG、全量 regression report 交给 P11 | commit event、state transition event、test harness candidate event | multiple_commit、state machine consistency、no write-before-commit、time advance consistency | TrajectoryRecord v0；OutputHistory in-memory v0；不要求正式 artifact | 时间步总纲、状态接口、代码数据结构、日志验证、最小验证场景 | spec_ready + implementation_ready | 是 | `MVS-COMMIT-1-lite` 的 candidate 只能来自 test harness 或 identity infrastructure，不得隐藏实现车辆模型 |
| P04 | Step 4A：MV 未入 merging zone 时的 APS / cache / effective assignment | `MVS-APS-FAIL-EMPTY`、`MVS-APS-FAIL-CACHE`、`MVS-APS-1/2/3/4` | APS 覆盖粒度可继续细分 | 无 | APS trigger / failure / assignment / cache event | no fake assignment、cache retain、Eq.10 target sanity | APS candidate window、assignment marker 数据 | 公式映射、道路几何、参数、状态接口、代码数据结构、最小验证场景 | trace_registered | 否 | 等 P04 完整 spec；依赖 P01-P03 完成 |
| P05 | Step 4B：MV 在 merging zone 的 CMC / assignment validation / Eq.53 / boundary cap command | `MVS-CMC-1`、`MVS-CMC-2`、`MVS-ASSIGN-1` | CMC 数值诊断可观测 | CMC platoon 关闭 | CMC decision、assignment_invalid、merge command、speed cap event | Eq.53 pass/fail、no actual leader substitution、boundary cap sanity | merge start / waiting / boundary marker 数据 | 公式映射、车辆模型、道路几何、参数、状态接口、日志验证、最小验证场景 | trace_registered | 否 | 等 P05 完整 spec；不得在 P00 定义 CMC 字段 |
| P06 | Step 5：cooperative request 汇总与多 MV / CV conflict resolution | `MVS-CONFLICT-1A`、`MVS-CONFLICT-1B` | conflict priority basis 诊断 | 全局多 MV gap 优化关闭 | cooperative_request、conflict_resolution event | one active request per CV、loser waiting sanity | conflict marker 数据 | 时间步总纲、公式映射、状态接口、代码数据结构、日志验证、复现讨论 | trace_registered | 否 | 等 P06 完整 spec；冲突仲裁必须标记工程补丁 |
| P07 | Step 6：CUC choice / compliance / lane-change command / same-step overlay | `MVS-CUC-1A_override_choice1`、`MVS-CUC-2`、`MVS-CUC-3` | `MVS-CUC-1B_real_utility_probe` | `MVS-CUC-1C_real_utility_choice1_locked` | CUC choice、lane-change command、spacing override、overlay event | non-compliant no action、unsafe fallback、CUCChoice not persistent | lane-change intent / overlay marker 数据 | 公式映射、车辆模型、状态接口、代码数据结构、日志验证、最小验证场景 | trace_registered | 否 | 等 P07 完整 spec；same-step overlay 必须标记工程实现约束 |
| P08 | Step 7：纵向模型 / Eq.10 spacing override / speed cap 合成 | `MVS-CUC-2`、`MVS-CUC-3`、`MVS-SAFE-1A_waiting_cap` | CPID memory 与裁剪诊断 | 论文级全量数值实验 | longitudinal_model、speed_cap consumption event | Eq.10 only CFV、non-compliant no Eq.10、planning_speed=min caps | longitudinal trace 可渲染数据 | 车辆模型、参数、公式映射、状态接口、代码数据结构、日志验证 | trace_registered | 否 | 等 P08 完整 spec；不得提前实现隐藏纵向模型 |
| P09 | Step 8：正弦横向轨迹 / active maneuver progress / safety correction | `MVS-SAFE-1B_executing_cap_lateral_consumption`、`MVS-SAFE-2` | front-collision fallback 诊断 | 严格 MPC tracking 关闭 | lateral_trajectory、maneuver progress、completion candidate event | no reset active trajectory、no ordinary lane-change、boundary risk sanity | lane-change / merge trajectory marker 数据 | 车辆模型、道路几何、时间步总纲、日志验证、最小验证场景 | trace_registered | 否 | 等 P09 完整 spec；不得把 completion 提前写入真实状态 |
| P10 | Step 4-9 集成：APS / CMC / CUC / longitudinal / lateral / commit 同步闭环 | `MVS-E2E-1`、`MVS-COMMIT-1-full` | 跨步 cache / trajectory lifecycle 诊断 | 无 | end-to-end event chain、cache lifecycle、active maneuver event | one commit per vehicle、no rerun CUC、executing merge no rejudge | full chain quicklook 数据 | 时间步总纲、状态接口、代码数据结构、日志验证、最小验证场景 | trace_registered | 否 | 等 P10 完整 spec；依赖 P04-P09 |
| P11 | Step 10 交付级收口：正式导出、PNG、artifact、required smoke suite、regression report | 全部 required MVS 聚合执行与报告 | probe 场景非阻塞报告 | deferred 场景不进入第一版强验收 | exported event history、regression event summary | exported sanity summary、suite result | 正式 PNG、artifact record、regression report | 输出日志、代码数据结构、最小验证场景、道路几何 | trace_registered | 否 | P11 不是首次实现日志 / sanity / MVS runner / PNG 口径；只做交付级聚合 |
| P12 | Step 1 扩展：边界车辆生成、随机属性、论文级实验入口 | 随机入口关闭时不得破坏全部 required MVS | 论文级实验入口可观测 | 论文级数值复刻不作为第一版强验收 | boundary_generation、random attribute、experiment config event | pre-freeze only、random disabled in smoke、entry safety sanity | 宏观指标 artifact 入口 | 时间步总纲、公式映射、参数、代码数据结构、输出指标 | trace_registered | 否 | 等 P12 完整 spec；只在主链路稳定后实现 |

## 8. 完成标准

- P00 文档完成，并可作为后续 Pxx 的审阅 checklist。
- P01-P03 的追踪条目存在，且必须达到 spec_ready / implementation_ready。
- P04-P12 的追踪条目存在，且至少达到 trace_registered。
- 每个后续 Pxx 至少有一个 Step 范围、一个 gate、一个上游 spec、一个 event / sanity / PNG 证据入口。
- P00 不得因为 P04-P12 尚未完整 spec_ready / implementation_ready 而失败；P04-P12 的完整执行计划在对应阶段另行编写。
- 所有工程补丁均被分类为工程补丁或第一版实现约束，并要求保留 `source`、`reason`、`is_engineering_patch`。
- required MVS 不在本阶段运行，但全部能在追踪矩阵中找到后续承接阶段。
- probe 场景只要求可配置、可观测，不阻塞 required suite。
- deferred 场景不进入第一版强验收。
- P11 只被定义为交付级导出、正式 PNG、artifact record、全部 required smoke suite 聚合和 regression report。

## 9. 回归保护

- 所有算法内部使用 `x_global`，`x_plot` 只用于绘图。
- 所有模块只读冻结 `S(t)`。
- command / next-state 不反写真实状态。
- commit 是唯一生成 `S(t+dt)` 的阶段。
- 每辆车每步最多提交一次最终状态。
- `lane_change_state == executing` 与 `merge_state == executing` 不得同车同时为真。
- `CUCChoice` 不是跨步持久控制状态。
- `relations snapshot` 每步生成、每步消费、下一步重建。
- 工程补丁不得写成论文原算法。
- 执行计划不得首次决定核心字段、参数、公式或 schema。
- P04-P10 不得以“等待 P11 统一补日志 / sanity / PNG”为完成理由。
- P01-P03 不得以“薄切片”方式削减已经写入 spec 的完成标准。
- P04-P12 在 P00 中只允许 trace_registered，不得由 P00 代替后续完整执行计划。
