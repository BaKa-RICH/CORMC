# P04-P07 总切片蓝图 / Dependency Sketch

## 0. 文档定位

本文档是 P04-P07 的总切片蓝图，只登记 Step 边界、MVS gate、输入输出依赖和跨阶段不变量。它不是 P05-P07 的完整执行计划，也不把 P05-P07 标记为 `spec_ready` 或 `implementation_ready`。

当前成熟度边界：

```text
P04:
    已进入并完成 Step4A APS / cache / effective assignment 的 red-before-green implementation。
    P04 required MVS targeted gate 已通过。
    当前 implementation_ready 仅限 P04 Step4A，不扩展到 P05-P12。

P05-P07:
    仅保留骨架依赖与 trace_registered 语义。
    暂不 spec_ready。
    暂不 implementation_ready。
```

本蓝图沿一次或一段 `S(t) -> command / next-state -> commit -> S(t+dt)` 时间步流程切片组织。P04-P07 都必须复用 P01-P03 已提供的 ScenarioConfig / matcher / runner、冻结 `S(t)`、relations、geometry resolver、CommandBuffer / NextStateBuffer、OutputHistory、EventRecord、SanityCheckRecord 与 commit 边界。

## 1. 四阶段 Step 边界

| 阶段 | Step 边界 | 当前成熟度 | 说明 |
| --- | --- | --- | --- |
| P04 | Step 4A：MV 未入 merging zone 时的 APS / cache / effective assignment | implementation_ready for Step4A only；P04 required targeted gate passed | 处理 `x_MV_global < x0_m_global` 的 on-ramp MV，已以 failing tests 验证 matcher 层红灯，并实现 `first_APS` / `APS_due` / `reuse_cache`、APS assignment / failure、cache action 与 `EffectiveAssignmentThisStep`，P04 required MVS targeted gate 已通过。 |
| P05 | Step 4B：MV 在 merging zone 的 CMC / assignment validation / Eq.53 / boundary cap command | trace_registered only | 处理 `x0_m_global <= x_MV_global <= x_ramp_end_global` 且 `merge_state != executing` 的 MV；消费 P04 assignment / cache，不重做 APS。 |
| P06 | Step 5：cooperative request 汇总与多 MV / CV conflict resolution | trace_registered only | 汇总 P04/P05 产生的 `col = 1` 协同需求，处理多 MV 请求同一 CV 的工程仲裁。 |
| P07 | Step 6：CUC choice / compliance / lane-change command / same-step overlay | trace_registered only | 对 P06 产出的 active cooperative request 运行 CUC choice 或测试覆盖口径，输出本步 lane-change / cooperation command 与 overlay，不提交真实状态。 |

## 2. 四阶段 MVS Gate

| 阶段 | required | probe | deferred |
| --- | --- | --- | --- |
| P04 | `MVS-APS-FAIL-EMPTY`、`MVS-APS-FAIL-CACHE`、`MVS-APS-1`、`MVS-APS-2`、`MVS-APS-3`、`MVS-APS-4` | APS 覆盖粒度可继续细分；非 required 边界诊断可观测 | 无 |
| P05 | `MVS-CMC-1`、`MVS-CMC-2`、`MVS-ASSIGN-1`；为 `MVS-SAFE-1A_waiting_cap` 提供 speed cap command 前置能力 | CMC 数值诊断可观测；boundary cap non-binding / binding 诊断 | CMC platoon 关闭 |
| P06 | `MVS-CONFLICT-1A`、`MVS-CONFLICT-1B` | conflict priority basis 诊断 | 全局多 MV gap 优化关闭 |
| P07 | `MVS-CUC-1A_override_choice1`、`MVS-CUC-2`、`MVS-CUC-3` | `MVS-CUC-1B_real_utility_probe` | `MVS-CUC-1C_real_utility_choice1_locked` |

当前已进入 implementation_ready 的 P04 必须同步产出本阶段 event、sanity、targeted MVS 验收断言和 expected_png_features / marker 数据，不得等待 P11 才补日志、sanity 或 PNG 证据。P05-P07 当前仅登记依赖骨架；它们在未来进入 spec_ready / implementation_ready 时，也必须同步产出各自阶段的 event、sanity、targeted MVS 验收断言和 expected_png_features / marker 数据。P11 只做交付级导出、正式 PNG、artifact record、全量 smoke suite 聚合和 regression report。

## 3. 输入输出依赖

| 阶段 | 读取输入 | 写入输出 | 供后续消费 | 不允许做 |
| --- | --- | --- | --- | --- |
| P04 | 冻结 `S(t)`；Step 3 `RelationsSnapshot`；P02 APS candidate window resolver；assignment cache；road / parameter config；P01 ScenarioConfig expected_*；P03 event / sanity / history helpers | APS trigger event；APS assignment / failure event；cache update / retain / invalid / cleanup request；Eq.10 desired spacing command source；`EffectiveAssignmentThisStep` 或等价本步有效 assignment；APS sanity；APS PNG marker registration | P05 消费 effective assignment / cache；P06 从 effective assignment 抽取 cooperative request；P07 / P08 后续识别 Eq.10 source；P11 聚合已有证据 | 不直接提交车辆真实 `x / y / v / a`；不写 CMC Eq.53；不写 boundary speed cap；不写 cooperative request conflict；不写 CUC utility；不实现 longitudinal / lateral candidate；不使用 `x_plot`；不重新定义 P02 几何口径 |
| P05 | 冻结 `S(t)`；relations；P04 effective assignment / APS cache；MV region / merge_state；road / parameter config；CommandBuffer 写入口 | CMC decision event；assignment validation event / sanity；Eq.53 pass/fail event；boundary speed cap command；merge command / state transition request；waiting marker | P08 消费 speed cap；P09 消费 merge command；P10 / P11 聚合 chain evidence；P06 不从 P05 计算 CUC utility | 不自行重做 APS；不偷换 actual leader / follower 替代 assignment；不直接提交 lane / y / x / v；不撤销 executing merge；不实现横向轨迹 |
| P06 | P04/P05 产生的 effective assignment；`col_CLV / col_CFV`；`T*_MV`；MV region；active vehicle / CV 状态；P01 matcher 语义 | cooperative_request event；conflict_resolution event；active cooperative request；conflict loser result；conflict sanity；conflict PNG marker | P07 消费 active cooperative request；P10 / P11 聚合 conflict chain evidence | 不计算 CUC utility；不生成 lane-change command；不提交车辆状态；不把多 MV 仲裁写成论文原算法 |
| P07 | P06 active cooperative request；relations；VehicleSpec / compliance state；road / parameter config；test harness utility override 或 real utility probe 输入；P03 command boundary | CUC event；compliance event；lane-change command；cooperation command / desired spacing override；same-step overlay；CUC sanity；CUC PNG marker | P08 消费 cooperation / Eq.10 spacing command；P09 消费 lane-change command / overlay；P10 / P11 聚合 CUC chain evidence | 不直接提交 lane / y / x / v；不把 CUCDecision 写成跨步持久状态；不处理 active lane-change 车辆的重新 CUC；不实现纵向 / 横向 candidate；不实现 commit |

## 4. 关键跨阶段不变量

- 所有算法内部使用 `x_global`；`x_plot` 只在 PNG / renderer 派生层出现。
- 所有模块只读冻结 `S(t)`。
- command / next-state 不反写真状态。
- commit 是唯一生成 `S(t+dt)` 的阶段。
- P04-P07 必须各自产生本阶段 event / sanity / PNG feature 证据，不得等待 P11 才补齐。
- `first_APS(MV)`、assignment invalid、多 MV 共享 CV 仲裁、same-step overlay 等工程补丁必须保留 `source` / `reason` / `is_engineering_patch` 或等价字段。
- P05-P07 暂不 `spec_ready`；它们只保留骨架依赖，不进入实现。
- P04 的 APS candidate collector 必须复用 P02 的 `[x_MV_global - L_cr, x_MV_global + L_cr]` candidate window。
- P04 failure tests 必须失败在 expected event / sanity / matcher 层，不能失败在 loader 崩溃、字段缺失、enum 不一致或自然语言断言。
- P05 消费 P04 的 effective assignment / cache，不得自行重做 APS 或偷换 leader / follower。
- P06 消费 P04/P05 产生的 `col = 1` 协同需求，不得计算 CUC utility。
- P07 消费 P06 的 active cooperative request，不得直接提交 lane / y / x / v，也不得把 `CUCDecision` 写成跨步持久状态。

## 5. 与 P00 追踪矩阵的关系

本蓝图承接 P00 的追踪矩阵，但更新当前执行成熟度口径如下：

```text
P04:
    已进入并完成 Step4A APS / cache / effective assignment 的 red-before-green implementation。
    P04 required MVS targeted gate 已通过。
    当前 implementation_ready 仅限 P04 Step4A，不扩展到 P05-P12。

P05-P07:
    当前仍只登记 dependency sketch / trace_registered。
    不进入 spec_ready。
    不进入 implementation_ready。

P08-P12:
    仍保持 trace_registered / 后续阶段口径。
```

若 P04 后续实现发现缺少 `EffectiveAssignmentThisStep` 或等价结构所需字段，必须先提出并修订 `docs/复现讨论/CORMC代码数据结构设计_整理版.md`，不得在代码中暗增字段。
