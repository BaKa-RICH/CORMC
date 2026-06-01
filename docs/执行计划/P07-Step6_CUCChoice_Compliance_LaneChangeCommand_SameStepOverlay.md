# P07 - Step 6 CUC Choice / Compliance / Lane-change Command / Same-step Overlay

> 本文档是 P07 的完整执行计划 spec。它只定义后续 P07 red-before-green implementation 的行为合同、测试计划和验收证据；本轮不实现 P07 CUC / compliance / lane-change command / same-step overlay 代码，不新增 P07 实际测试文件，不修改业务算法。
>
> P07 只覆盖一次 `S(t) + P06 active cooperative request -> Step 6 CUC decision / command / overlay / event / sanity -> P08 / P09 / P10 / P11` 的时间步切片。P07 不是泛化换道控制器，不是纵向模型，不是横向轨迹，也不是 commit 入口。

## 1. Slice Identity

- Algorithm-Step Coverage:
  - Primary Step: Step 6，对 P06 产出的 active cooperative request 执行 CUC choice、target lane TT safety、CHV compliance、lane-change command / spacing override command、same-step overlay、event / sanity / PNG marker registration。
  - Secondary Steps:
    - P06 handoff reader：只读取 active cooperative request，不读取 loser / suppressed request 作为 active CUC 输入。
    - active lane-change guard：`lane_change_state == executing` 的 CV 跳过 CUC，不重新选择目标，不重复发起 lane-change command。
    - CUC utility / override chooser：支持真实 utility probe，也支持 test harness override choice 1 required 场景；override 必须标记为测试钩子，不得写成论文公式。
    - target lane TT safety checker：目标 lane 1 不安全时，优先级高于 utility / override，必须 fallback 到 choice 2。
    - compliance resolver：CAV 与 compliant CHV 可接受 CUC 建议；non-compliant CHV 忽略协同建议，不换道，不消费 Eq.10 spacing override。
    - command / overlay writer：choice 1 只写 command / transition request / overlay，不直接写真实车辆状态。
    - cooperation / spacing writer：choice 2 写 P08 可消费的 desired spacing override / cooperation command，不计算 longitudinal candidate。
    - CUC event / compliance event / safety event / sanity / marker registration。
- MVS Acceptance Gate:
  - required:
    - `MVS-CUC-1A_override_choice1`
    - `MVS-CUC-2` 的 P07 Step6 targeted gate：target lane unsafe fallback + spacing handoff only；完整 Eq.10 longitudinal consumption 属于 P08。
    - `MVS-CUC-3`
  - probe:
    - `MVS-CUC-1B_real_utility_probe`
  - deferred:
    - `MVS-CUC-1C_real_utility_choice1_locked`
    - `MVS-E2E-1`
    - `MVS-SAFE-1A_waiting_cap`
    - `MVS-SAFE-1B_executing_cap_lateral_consumption`
    - `MVS-COMMIT-1-full`
    - P08-P12 的任何 full pass。
- 本阶段解锁的能力:
  - 证明 Step 6 只消费 P06 active request，并稳定产出 CUC decision / command / overlay / event / sanity。
  - 证明 choice 1 可以启动 lane 2 -> lane 1 的本步命令链，但真实 lane / y / lane_change_state 只能由后续 command / next-state / commit 边界处理。
  - 证明 unsafe target lane 与 non-compliant CHV 均能阻断 CUC 执行。
  - 证明 choice 2 能把 Eq.10 desired spacing override 交给 P08，而不是在 P07 计算纵向 candidate。
  - 证明 same-step overlay 是 first-version engineering patch，并携带 `source / reason / is_engineering_patch`。
- 本阶段不要求通过的后续场景:
  - P07 不要求 `MVS-E2E-1` 通过；E2E 需要 P08 / P09 / P10 联动。
  - P07 不要求 `MVS-SAFE-1A_waiting_cap` 或 `MVS-SAFE-1B_executing_cap_lateral_consumption` full pass；这些需要 P08 / P09 消费 P07 command。
  - P07 不要求 `MVS-COMMIT-1-full` 通过；commit 由 P10 / P11 聚合。
  - P07 不要求正式 PNG renderer 存在，只要求 P07 marker / expected_png_features 注册并可被 P01 matcher 报告。

## 2. 本阶段目标

P07 聚焦一次冻结状态下的 Step 6：

```text
S(t)
  + Step 3 relations / lane_change_neighborhood
  + P06 active cooperative request
  + VehicleSpec / compliance state
  + road / parameter config
  + optional test harness utility override
    -> Step 6 CUC choice
    -> target lane safety / compliance resolution
    -> lane-change command or spacing override command
    -> same-step overlay if lane-change starts this step
    -> event / sanity / PNG marker
    -> P08 / P09 / P10 / P11 consumers
```

P07 的核心目标不是“实现一个控制器”，而是把 P06 交付的 active cooperative request 转换为本步可消费的 maneuver choice 和 command evidence：

- choice 1：`change_to_lane_1`，表示被请求的 lane 2 CV 本步选择 lane 2 -> lane 1 的协同换道。
- choice 2：`stay_lane_2`，表示被请求的 CV 留在 lane 2，并通过 P08 后续纵向模型配合 MV 合流。

当前 P06 实现给 P07 的可用输入包括：

- `Step5CooperativeRequestRunResult.active_requests`：按 `cv_id` 索引的 active request payload。
- `Step5CooperativeRequestRunResult.command_buffer.cooperation_commands`：同一份 active request payload，可作为 Step 6 handoff。
- `Step5CooperativeRequestRunResult.suppressed_requests`：loser / suppressed request trace；P07 只能审计，不得作为 active CUC 输入。
- `Step5CooperativeRequestRunResult.conflict_results`：conflict winner / loser / priority basis；P07 只能用于追溯，不得重新仲裁。
- `Step5CooperativeRequestRunResult.actual_events`：`cooperative_request` 与 `conflict_resolution`。
- `Step5CooperativeRequestRunResult.expected_png_features`：P06 request / conflict marker。

当前 active request payload 至少包含：

```text
request_id
source_mv_id
cv_id
cv_role
col
aps_case
assignment_source
t_mv_star
mv_in_merging_zone
mv_distance_to_x0_m
desired_spacing_override
active
source_conflict_id
```

这些字段足以支持 P07 第一版的 request identity、source MV、CV role、APS case、Eq.10 desired spacing handoff 和 conflict trace。P07 不应要求 P06 提供 CUC utility、compliance、target lane safety 或 lane-change command；这些属于 Step 6 本阶段输出。

P07 需要产出：

- `CUCDecision` 或等价本步派生 decision payload。
- `CUC` / `cuc` event：utility source、U1 / U2 或 override、target lane safety、recommended choice、effective choice、fallback reason。
- compliance event：vehicle type、compliance state、accepted / ignored。
- target lane safety event：TLV / TFV、TT values、`TT_min`、safe / unsafe。
- lane-change command：choice 1 且安全且 compliant 时生成。
- lane_change_state transition request 或等价 `state_transition_commands`：只请求初始化，不直接写真实 `lane_change_state`。
- same-step maneuver relation overlay：choice 1 新启动换道时生成，供 P08 / P09 本步消费。
- cooperation / desired spacing override command：choice 2 且可执行时生成，供 P08 消费。
- sanity：active request consumption、non-compliant no action、unsafe fallback、CUCDecision not persistent、active lane-change skip CUC、no-write-before-commit。
- expected_png_features：CUC decision marker、lane-change intent marker、fallback marker、non-compliant ignored marker、same-step overlay marker。

## 3. 非目标 / 禁止事项

- 不重做 P04 APS。
- 不重跑 APS candidate collector、APS case classifier、`T*_MV` prediction、CLV / CFV selector 或 assignment cache refresh。
- 不重做 P05 CMC / Eq.53 / boundary cap。
- 不重新执行 assignment validation、CMC branch selection、merge command 或 speed cap command。
- 不重做 P06 cooperative request collection / conflict resolution。
- 不消费 P06 loser / suppressed request 作为 active CUC 输入。
- 不重新选择 CLV / CFV。
- 不重新仲裁多个 MV / CV conflict。
- 不为 invalid / failed / empty assignment 伪造 active request。
- 不实现 P08 longitudinal model、planning speed、IDM、CPID、front-collision-avoidance 合成。
- 不在 P07 直接执行 Eq.10 纵向消费或生成 longitudinal candidate；P07 只能把 desired spacing override 传给 P08。
- 不实现 P09 正弦横向轨迹、lateral candidate、lane-change progress。
- 不实现 P10 E2E integration。
- 不实现 P11 full smoke / artifact export / formal PNG renderer。
- 不实现 P12 随机边界生成或论文级实验。
- 不直接写真实车辆 `x / y / v / a / physical_lane / road_role / lane_change_state / merge_state`。
- 不写 Step 9 commit result。
- 不把 `CUCDecision` 写成跨步持久控制状态。
- 不对 `lane_change_state == executing` 的车辆重新跑 CUC 或重新发起 lane change。
- 不用 `x_plot` 做任何算法判断。
- 不把 test harness utility override 写成论文公式；必须保留 `utility_source = test_harness_override` 或等价标记。
- 不把 same-step overlay 写成论文原语义；它是第一版工程补丁，必须保留 `source / reason / is_engineering_patch`。
- 不新增字段、enum、ScenarioConfig 字段、EventRecord 字段、SanityCheckRecord 字段或 expected_* 口径作为“暗实现”。若 schema 不足，先在本 spec 标记 gap，再由后续上游修订处理。

## 4. 上游 spec 引用

- `docs/执行计划/CORMC执行计划spec设计总纲v1.md`
  - 引用 P07 Step 6 定位：CUC 是 maneuver choice，不是控制器；只能生成 lane-change command、spacing override command、event，不直接更新车辆位置。
  - 引用 P07 MVS gate：`MVS-CUC-1A_override_choice1`、`MVS-CUC-1B_real_utility_probe`、`MVS-CUC-2`、`MVS-CUC-3`，`MVS-CUC-1C_real_utility_choice1_locked` deferred。
  - 引用 `CUCChoice` 不得成为跨步持久控制状态。
- `docs/执行计划/P00-Spec宪法_权威边界与二维追踪矩阵.md`
  - 引用 P07 row：Step 6 产出 CUC choice、lane-change command、spacing override、overlay event；sanity 包括 non-compliant no action、unsafe fallback、CUCChoice not persistent。
  - 引用工程补丁必须携带 `source`、`reason`、`is_engineering_patch`。
  - P00 当前仍要求 P04-P12 只保持 `trace_registered`，不要求 P07 已 spec_ready / implementation_ready。新增本文档不自动改变 P00 静态矩阵状态。
- `docs/执行计划/P01-MVS_Runner_ScenarioConfig与验收断言语言.md`
  - 引用 required / probe / deferred 语义、expected_events、forbidden_events、expected_event_counts、expected_sanity_checks、expected_png_features。
  - 当前 runner 已登记 `MVS-CUC-1B_real_utility_probe` 为 probe、`MVS-CUC-1C_real_utility_choice1_locked` 为 deferred；required 的 `MVS-CUC-1A_override_choice1`、`MVS-CUC-2`、`MVS-CUC-3` 尚未登记。P07 首轮 required red tests 不调用这些 built-in，也不依赖未授权 inline ScenarioConfig 字段；先用 Python helper 直接构造 P06 handoff 和 Step6 fixtures。
  - 当前 built-in CUC 场景使用 event_type `"CUC"`，代码数据结构文档语义上登记 canonical lower-case `cuc`。P07 red tests 不得引入第三套 casing；若收紧 enum，应先统一 loader、matcher、历史场景和文档。
- `docs/执行计划/P02-Step0-3_清理冻结关系与几何口径.md`
  - 引用冻结 `S(t)`、relations snapshot、lane_change_neighborhood、target lane TLV / TFV / LV / FV basis。
  - Step 3 不创建 same-step overlay；P07 在本步新启动 CUC lane-change 时创建 overlay。
  - 所有算法判断使用 `x_global`，不得用 `x_plot`。
- `docs/执行计划/P03-Step9-10_Command_NextState_Commit_Event_Sanity_Trajectory闭环.md`
  - 引用 `CommandBuffer`、no-write-before-commit、`CUCDecision` non-persistent。
  - 当前代码中的 `CommandBuffer` 有 `cuc_decisions` 字段，但 `docs/复现讨论/CORMC代码数据结构设计_整理版.md` 的 `CommandBuffer` 权威字段表未列出该字段。P07 不得把 `CommandBuffer.cuc_decisions` 作为正式承载点，除非先修订数据结构规格；第一版默认把 `CUCDecision` 放在 Step6 derived result / CUC event payload，并在 lane-change / cooperation command 中只引用 `cuc_decision_id`。
- `docs/执行计划/P04-Step4A_APS_Cache_EffectiveAssignment.md`
  - 引用 APS assignment / Eq.10 desired spacing source。P07 不重算 APS，只消费经 P06 传递的 `desired_spacing_override`。
- `docs/执行计划/P05-Step4B_CMC_AssignmentValidation_Eq53_BoundaryCap.md`
  - 引用 P05 assignment validation / CMC boundary。P07 不重做 CMC，也不吸收 P05 speed cap / merge command。
- `docs/执行计划/P06-Step5_CooperativeRequest_ConflictResolution.md`
  - 引用 P06 输出 active request / suppressed request / conflict result / `CommandBuffer.cooperation_commands`。
  - P07 只消费 active request；loser / suppressed request 只作为 audit evidence。
  - P07 的输入输出边界以 P00 追踪矩阵、总纲、当前 P04-P07 完整 spec 为准：输入为 P06 active cooperative request、relations、VehicleSpec / compliance state、road / parameter config、test harness override 或 real utility probe 输入、P03 command boundary；输出为 CUC event、compliance event、lane-change command、cooperation command / desired spacing override、same-step overlay、CUC sanity、CUC PNG marker。
- `docs/复现讨论/CORMC时间步执行顺序梳理.md`
  - 引用 Step 6：active lane-change 车辆跳过 CUC；尚未换道的 active CV 执行 CUC；choice 1 时检查目标车道 TT safety；non-compliant CHV 不执行 CUC 建议。
- `docs/复现讨论/CORMC论文公式与实现映射.md`
  - 引用 CUC choice 1 / 2、CHV compliance、Eq.11-Eq.12 utility、Eq.14-Eq.15 target lane TT safety、Eq.16 final choice。
  - 引用 active lane-change 不重新 CUC 是论文 Fig. 9 时间步语义与第一版状态机约束。
- `docs/复现讨论/CORMC状态与模块接口规格.md`
  - 引用 active cooperative request、CUC choice、lane-change state、same-step maneuver relation overlay 的接口语义。
  - 引用 physical lane 与 y 不在 CUC 模块内直接修改。
- `docs/复现讨论/CORMC车辆模型规格.md`
  - 引用 CUC 是 maneuver choice；choice 1 进入 lane-changing model；choice 2 将 Eq.10 desired spacing 交给后续纵向模型；non-compliant CHV 不接受 CUC 建议。
- `docs/复现讨论/CORMC代码数据结构设计_整理版.md`
  - 引用 `CUCChoice`、`CUCFallbackReason`、`CUCDecision`、`CooperationCommand`、`LaneChangeCommand`、`SameStepManeuverRelationOverlay`、`CommandBuffer`。
  - §10.2 旧阶段拆分表曾把 `CUCDecision` / `LaneChangeCommand` 放在 P06，把 `CMCDecision` / `MergeCommand` / `SpeedCapCommand` 放在 P07；该拆分与 P00 追踪矩阵、总纲、当前 P04-P07 完整 spec 固定的阶段边界冲突，视为过期口径。当前权威边界为：P05=CMC，P06=cooperative request / conflict，P07=CUC choice / command / overlay。
- `docs/复现讨论/CORMC输出指标与日志验证规格_整理版.md`
  - 引用 Step 6 日志：active lane-change skip CUC、active CV 判断、CUC utility / safety、final choice、compliance、CUC choice 不持久化。
- `docs/复现讨论/CORMC参数规格.md`
  - 引用 CUC 参数：`alpha=-1`、`beta=1.5`、`gamma=0.5`、`zeta=-0.5`、`TT_min=1.5s`；CHV compliance state / rate。
- `docs/复现讨论/CORMC道路几何与区域规格.md`
  - 引用 CUC target lane：lane 2 -> lane 1；lane 1 centerline `y = +lane_width`；算法使用 `x_global`。
- `docs/复现讨论/CORMC最小验证场景执行规格.md`
  - 引用 `MVS-CUC-1A_override_choice1`、`MVS-CUC-1B_real_utility_probe`、`MVS-CUC-1C_real_utility_choice1_locked`、`MVS-CUC-2`、`MVS-CUC-3` 的目的、setup 和验收口径。
  - `MVS-CUC-2` 原场景文本写有 `fallback_reason = target_lane_TT_unsafe` 与 longitudinal_model consumption。P07 canonical reason 统一为 `target_lane_unsafe`；P07 只验 Step6 fallback + spacing handoff，不验 longitudinal_model consumption。后者必须由 P08 targeted gate 覆盖。

## 5. 行为契约 Given / When / Then

- Given：冻结 `S(t)`、Step 3 relations、P06 `active_requests` 已存在。When：执行 P07 Step 6。Then：P07 只读取 P06 active request、relations、vehicle spec / state、road / parameter config 和测试 override；输出 CUC 派生结构、command、overlay、event、sanity、marker；不得修改冻结车辆状态。
- Given：P06 没有 active request。When：P07 执行。Then：不运行 CUC，不生成 CUC decision、lane-change command、spacing override command 或 same-step overlay；可记录 `no_active_request_no_cuc` diagnostic event。
- Given：P06 有 suppressed / loser request，但该 CV 没有 active winner。When：P07 执行。Then：不把 suppressed / loser request 作为 CUC 输入；可在 sanity / event payload 中记录 `suppressed_requests_ignored_as_active_input = true`。
- Given：P06 同一 `cv_id` 有一个 active request。When：P07 执行。Then：以该 active request 的 `cv_id/source_mv_id/cv_role/aps_case/desired_spacing_override` 作为 Step 6 唯一 CUC input。
- Given：active request 的目标 CV 不在 `S(t).active_vehicle_ids` 或状态缺失。When：P07 preflight 执行。Then：不得伪造 CUC；输出 filtered / sanity evidence，reason 为 `active_cv_missing` 或等价结构化 reason。
- Given：CV 当前 `lane_change_state == executing`。When：P07 遍历 active request。Then：跳过 CUC utility / override / target choice；不生成新的 lane-change command；记录 skip event / sanity，reason 为 `already_executing_lane_change`。
- Given：CV 已处于 active lane-change，且 P06 仍提供 active request。When：P07 执行。Then：P07 不重新选择 lane 1 / lane 2，不覆盖既有 target lane；后续继续换道由 P09 / P10 使用已有 lane_change_state / trajectory state。
- Given：CV 是 CAV 或 compliant CHV，target lane safe，test harness override 指示 `choice_1`。When：执行 `MVS-CUC-1A_override_choice1`。Then：P07 生成 CUC decision，`recommended_choice = change_to_lane_1`，`effective_choice = change_to_lane_1`，utility source 标为 `test_harness_override`；生成 lane-change command、state transition request 或等价 command、same-step overlay、CUC event、target lane safety event、sanity、PNG marker。
- Given：test harness override 指示 `choice_1`。When：P07 记录 utility evidence。Then：必须明确 `source = test_harness_override`、`reason = override_choice1_for_required_gate` 或等价字段；不得把该 choice 写成 Eq.11-Eq.16 论文真实 utility 结果。
- Given：real utility probe 启用。When：执行 `MVS-CUC-1B_real_utility_probe`。Then：P07 记录 `utility_source = real_CUC`、utility inputs、`U1`、`U2`、target lane safety、recommended choice、final choice；probe 不作为 required strong acceptance，不要求锁定 `change_to_lane_1`。
- Given：real utility locked choice 1 场景。When：执行 `MVS-CUC-1C_real_utility_choice1_locked`。Then：当前仍 deferred；P07 implementation 不得把它纳入 required green gate，除非后续完成 utility 公式复核并显式升级。
- Given：utility 或 override 倾向 choice 1，但目标 lane 1 TT safety unsafe。When：执行 `MVS-CUC-2`。Then：target lane safety 优先级最高，P07 必须 fallback 到 `stay_lane_2`；不生成 lane-change command；不生成 same-step lane-change overlay；记录 target lane unsafe event；若 active request 有 `desired_spacing_override`，输出 P08 可消费 cooperation / spacing command。
- Given：目标 lane unsafe。When：fallback 到 choice 2。Then：payload 必须能说明 `fallback_reason = target_lane_unsafe`，并保留 TLV / TFV、TT values、`TT_min` 或 schema gap。
- Given：上游场景文本或旧 fixture 使用 `target_lane_TT_unsafe`。When：P07 red tests / matcher 编写。Then：必须在测试 fixture 侧归一为 canonical `target_lane_unsafe`，不得让 matcher 期待旧字符串。
- Given：CV 是 non-compliant CHV。When：执行 `MVS-CUC-3`。Then：non-compliant CHV 忽略 CUC 建议；不生成 lane-change command；不消费 Eq.10 desired spacing override；不生成 P08 spacing override command；记录 compliance event，reason 为 `non_compliant_chv`。
- Given：CV 是 non-compliant CHV。When：记录 CUCDecision。Then：`effective_choice` 必须使用权威 `CUCChoice` 中的 `not_applicable`，或在设计明确选择 `stay_lane_2` 且同时写 `accepted_by_vehicle=false`；不得使用未授权枚举值 `not_executed`。
- Given：CV 是 compliant CHV，final choice 为 choice 2。When：P07 输出 cooperation command。Then：CHV 的纵向行为仍由 P08 IDM / desired spacing 语义处理，P07 不直接计算加速度。
- Given：CV 是 CAV，final choice 为 choice 2，active request 带 `desired_spacing_override`。When：P07 输出 cooperation command。Then：P08 后续可消费该 override 计算纵向 candidate；P07 不产生 `CandidateLongitudinalKinematics`。
- Given：final choice 为 choice 1。When：P07 生成 lane-change command。Then：command payload 至少包含 `vehicle_id`、`source_mv_id`、`source_request_id`、`target_lane = lane_1`、`source_lane = lane_2`、`target_y` 或 `target_lane_centerline_ref`、`cuc_decision_id`、`overlay_id`、`init_maneuver = true`。
- Given：final choice 为 choice 1。When：P07 生成 state transition request。Then：只写入 `CommandBuffer.state_transition_commands` 或等价 command；不得直接修改 `VehicleState.lane_change_state`。
- Given：final choice 为 choice 1。When：P07 生成 same-step overlay。Then：overlay payload 必须包含 `overlay_id`、`vehicle_id`、`source_request_id`、`source = first_version_engineering_patch`、`reason = same_step_cuc_lane_change_relation_overlay`、`is_engineering_patch = true`、target lane neighbor references 或 schema gap。
- Given：final choice 为 choice 2。When：P07 生成 outputs。Then：不生成 lane-change command，不生成 lane-change state transition request；可生成 Step6 derived CUC decision payload、CUC / safety / compliance event、cooperation / spacing command、PNG marker。
- Given：P07 完成。When：比较冻结 `S(t)` 签名。Then：真实 `x / y / v / a / physical_lane / road_role / lane_change_state / merge_state / aps_assignment_cache` 未被 P07 修改。
- Given：P08 后续执行。When：读取 P07 output。Then：P08 消费 desired spacing override / cooperation command，计算 longitudinal candidate；P07 不提前计算或缓存 candidate。
- Given：P09 后续执行。When：读取 P07 output。Then：P09 消费 lane-change command / same-step overlay，计算 lateral trajectory；P07 不提前计算正弦轨迹或 lane-change progress。
- Given：P10 / commit 后续执行。When：提交 command / candidate。Then：只有 commit 才可能把 lane-change command 转为真实状态变化；P07 不直接提交。

## 6. 允许实现的代码对象

后续执行 P07 时，必须先新增 P07 failing tests / test skeleton 并确认 red phase，再在同一轮中实现最小 Step 6 对象。允许实现范围仅限 P07 Step 6；不得越界实现 P08-P12。

- domain / state objects:
  - `CUCDecision` 或等价本步派生 payload。
  - `CUCChoice` value：`change_to_lane_1`、`stay_lane_2`、`not_applicable`。
  - `CUCFallbackReason` value：`utility_not_better`、`target_lane_unsafe`、`non_compliant_chv`、`already_executing_lane_change`、`not_active_cv`、`not_applicable`。
  - `TargetLaneSafetyResult` 或等价 payload。
  - `ComplianceResolution` 或等价 payload。
  - `SameStepManeuverRelationOverlay` 或等价 payload。
- command / next-state objects:
  - 第一版正式承载：`CUCDecision` 存在 Step6 run result 的本步 derived payload 与 CUC event payload 中；lane-change / cooperation command 只携带 `cuc_decision_id` 引用。
  - `CommandBuffer.cuc_decisions` 当前在代码中存在，但未被数据结构权威字段表授权；不得在 P07 required tests 中断言该字段。若实现坚持使用该字段，必须先修订 `CORMC代码数据结构设计_整理版.md`、P03/P07 相关 spec 和 matcher 口径。
  - 复用 `CommandBuffer.lane_change_commands` 承载 choice 1 lane-change command。
  - 复用 `CommandBuffer.state_transition_commands` 承载 lane_change_state transition request 或等价初始化请求。
  - 复用 `CommandBuffer.same_step_overlays` 承载 same-step overlay。
  - 复用 `CommandBuffer.cooperation_commands` 承载 choice 2 desired spacing override / cooperation command。
  - 不写 `CommandBuffer.longitudinal_commands`、`merge_commands`、`speed_cap_commands`、`cache_update_commands`。
  - 不写 `NextStateBuffer`，不写 commit result。
- step runner / service functions:
  - `run_step6_cuc_choice_compliance_lane_change_overlay`
  - `load_active_cooperative_requests_from_p06`
  - `filter_active_cuc_inputs`
  - `skip_active_lane_change_cv`
  - `evaluate_cuc_utility_or_override`
  - `evaluate_target_lane_tt_safety`
  - `resolve_chv_compliance`
  - `resolve_final_cuc_choice`
  - `build_cuc_decision`
  - `build_lane_change_command`
  - `build_lane_change_state_transition_request`
  - `build_same_step_maneuver_relation_overlay`
  - `build_spacing_override_cooperation_command`
  - `register_p07_png_features`
- event / sanity helpers:
  - `emit_cuc_decision_event`
  - `emit_target_lane_safety_event`
  - `emit_compliance_event`
  - `emit_lane_change_command_event`
  - `emit_spacing_override_command_event`
  - `emit_same_step_overlay_event`
  - `run_p07_no_write_before_commit_sanity`
  - `run_p07_cuc_choice_not_persistent_sanity`
  - `run_p07_non_compliant_no_action_sanity`
  - `run_p07_unsafe_fallback_sanity`
  - `run_p07_active_lane_change_skip_sanity`
  - helper 名不是 schema enum。实际 `event_type` / `check_type` 必须使用当前已授权名称，或先修订数据结构规格。
- scenario / loader objects:
  - P07 首轮 red-before-green implementation 必须优先使用 Python test helper 构造 P06 active request handoff、relations fixture、utility override、compliance state 和 target lane safety fixture，并直接调用 Step6 runner。不得在首轮 required red tests 中调用尚未登记的 `MVS-CUC-1A_override_choice1`、`MVS-CUC-2`、`MVS-CUC-3` built-in scenario。
  - 若后续使用 built-in required scenarios，需要先在 loader / MVS docs 中登记：
    - `MVS-CUC-1A_override_choice1`
    - `MVS-CUC-2`
    - `MVS-CUC-3`
  - `MVS-CUC-1B_real_utility_probe` 与 `MVS-CUC-1C_real_utility_choice1_locked` 已有 built-in 占位，但 P07 implementation 仍需接入真实 Step 6 output。
  - 若需要 `active_request_state` 或 `test_harness_overrides.utility_choice`，必须确认 ScenarioConfig 已授权；未授权时只能在 Python test helper 中构造 P06 handoff，不能暗增顶层字段。

当前 schema / handoff gap：

```text
1. 当前代码有 CommandBuffer.cuc_decisions mapping，但数据结构权威字段表未列出该字段；它不能作为 P07 正式承载点，除非先补数据结构规格。默认第一版用 Step6 derived result + CUC event payload 表达 CUCDecision，并在命令中引用 cuc_decision_id。
2. 当前 required 的 MVS-CUC-1A / 2 / 3 尚未在 loader built-in 中登记。
3. 当前 built-in CUC probe / deferred 使用 event_type "CUC"，数据结构文档语义为 "cuc"，需要后续统一 casing 或保持兼容。
4. 当前 SanityCheckType 权威集合是否包含 target_lane_unsafe_fallback、non_compliant_no_action、active_lane_change_skip_CUC、CUCChoice_not_persistent 的细粒度名称需要核对；若未授权，第一版应使用已有 check_type 或 event payload 表达。
5. 当前 ScenarioConfig 是否允许直接加载 P06 active request、CUC utility override、CHV compliance state、target lane safety fixture，需要后续 implementation 前核对；P07 首轮 required red tests 默认用 test helper 构造 handoff，不暗增 ScenarioConfig 顶层字段。
6. 当前 P06 active request 带 desired_spacing_override，但未带 target lane TLV / TFV；P07 应从 Step 3 relations / lane_change_neighborhood 读取，不要求 P06 补字段。
7. 当前 target_y / lane centerline 可由道路几何解析；若 LaneChangeCommand schema 需要 target_y 字段，必须引用几何 resolver，不得硬编码 x_plot 或渲染坐标。
```

P07 handoff preflight 冻结口径：

| 项 | 冻结口径 | 实现前必须满足的失败边界 |
| --- | --- | --- |
| active request 来源 | 只接受 P06 `active_requests` 或 `CommandBuffer.cooperation_commands` 中 active=true 的 request。 | suppressed / loser request 被 P07 当作 active 输入时，red test 必须失败。 |
| CUCDecision 承载 | 默认 Step6 derived result + CUC event payload；命令只引用 `cuc_decision_id`。 | 在数据结构规格未补齐前，required tests 不得要求 `CommandBuffer.cuc_decisions`。 |
| choice 1 utility source | required 场景可用 test harness override。 | override 必须标记 source，不得伪装成 paper formula。 |
| real utility probe | 记录 U1 / U2 / inputs / final choice。 | `MVS-CUC-1B` 是 probe，不阻塞 required suite。 |
| target lane safety | P07 从 relations / lane_change_neighborhood 读取 TLV / TFV 并检查 TT safety。 | unsafe 时即使 override choice 1 也必须 fallback choice 2。 |
| fallback reason | target lane unsafe canonical reason 统一为 `target_lane_unsafe`。 | tests / matcher 不得期待旧字符串 `target_lane_TT_unsafe`。 |
| compliance | non-compliant CHV 忽略 CUC 建议。 | non-compliant CHV 产生 lane-change command 或 Eq.10 spacing command 时失败。 |
| same-step overlay | 只在本步新启动 CUC lane-change 时生成。 | overlay 缺 `source/reason/is_engineering_patch` 时失败。 |
| no-write-before-commit | P07 只写 command / event / sanity / marker。 | 真实车辆状态变化必须失败。 |

## 7. 先写失败测试

本次不新增实际 P07 测试文件。后续执行 P07 implementation 时，必须在同一轮中完成 red-before-green：

1. 先新增 P07 failing tests / test skeleton。
2. 首轮 required targeted tests 使用 Python helper 构造 P06 handoff 和 Step6 fixtures，不依赖尚未登记的 built-in `MVS-CUC-1A_override_choice1`、`MVS-CUC-2`、`MVS-CUC-3`，也不暗增 ScenarioConfig 顶层字段。
3. 先运行 P07 targeted tests，确认红灯发生在 expected event / sanity / command / PNG matcher 层。
4. 红灯不得是 loader error、unknown built-in scenario、unknown enum、unknown field、ImportError、AttributeError 或自然语言断言。
5. 然后同轮实现最小 P07 Step 6 CUC choice / compliance / command / overlay。
6. 再运行 P07 targeted green tests 和 P00-P06 回归。
7. 返回 red-before-green 证据：初始 red report、green report、回归报告、event / command / overlay 样例。

测试治理规则：

```text
1. 若测试是“真实红灯测试”，即 pytest failure 用于证明 P07 尚未实现，则必须使用
   pytest.mark.xfail(strict=True, reason="P07 CUC choice command overlay not implemented")
   或放入单独 red-test 命令。
2. 若测试是“合同测试”，即 runner/report 当前应返回 required_failed 且 failure reason 停在 matcher 层，
   则该测试可以在主线保持绿色。
3. 主线全量测试不得因为 P07 尚未实现而变红，除非该轮目标明确是 red phase 且随后同轮转绿。
4. 若新增 event_type、check_type、ScenarioConfig 字段或 expected_* 口径未在上游规格授权，
   必须先修订规格；不得用暗字段制造伪红灯。
```

未来 P07 tests 至少包括：

- `test_mvs_cuc_1a_override_choice1_generates_command_overlay_and_events`
  - 构造 P06 active request：`CV_CUC` 被 `MV_CUC` 请求，`desired_spacing_override` 可存在但 choice 1 不消费。
  - 构造 target lane safe relations / TLV / TFV。
  - 启用 test harness override：`recommended_choice = change_to_lane_1` 或 `U1_gt_U2 = true`。
  - 断言 CUC event 有 `utility_source = test_harness_override`、`final_choice = change_to_lane_1`。
  - 断言生成 lane-change command、state transition request 或等价 command、same-step overlay。
  - 断言 overlay 有 `source=first_version_engineering_patch`、`reason`、`is_engineering_patch=true`。
  - 断言 P07 不直接修改真实 lane / y / lane_change_state。
- `test_mvs_cuc_2_target_lane_unsafe_fallback_stay_lane2_and_spacing_handoff`
  - 构造 target lane unsafe TLV / TFV TT。
  - 即使 override / utility 倾向 choice 1，也断言 final choice 为 `stay_lane_2`。
  - 断言 canonical `fallback_reason = target_lane_unsafe`；不得使用旧字符串 `target_lane_TT_unsafe`。
  - 断言不生成 lane-change command、不生成 same-step lane-change overlay。
  - 断言输出 P08 可消费 cooperation / desired spacing override command，即 P07 Step6 spacing handoff。
  - 断言没有 longitudinal candidate。
- `test_mvs_cuc_3_non_compliant_chv_ignores_cuc_without_spacing_consumption`
  - CV 为 CHV 且 `chv_compliance_state = non_compliant`。
  - 断言 compliance event 记录 ignored / not executed。
  - 断言 CUCDecision 使用 `effective_choice = not_applicable` 或 `effective_choice = stay_lane_2` + `accepted_by_vehicle=false`，不得使用 `not_executed`。
  - 断言不生成 lane-change command。
  - 断言不生成 Eq.10 spacing override / cooperation command。
  - 断言没有 P08 / P09 candidate。
- `test_mvs_cuc_1b_real_utility_probe_logs_inputs_u1_u2_and_final_choice`
  - 关闭 test harness override。
  - 断言 `utility_source = real_CUC`、utility inputs logged、`U1` / `U2` 可观测、target lane safety 可观测、final choice 可观测。
  - 该测试不把 choice 1 作为 required strong acceptance。
- `test_p07_consumes_only_p06_active_requests`
  - 给定 P06 active request、suppressed request、conflict result。
  - 断言只有 active winner 进入 CUC decision；loser 不产生 CUC event / command。
- `test_p07_no_active_request_no_cuc`
  - active request 为空时，不生成 CUC decision / command / overlay。
- `test_p07_suppressed_loser_request_does_not_trigger_cuc`
  - suppressed request 单独存在时，不触发 CUC。
- `test_p07_active_lane_change_cv_skips_cuc_and_no_duplicate_command`
  - `lane_change_state == executing` 的 CV 跳过 CUC。
  - 断言不生成新的 lane-change command，不覆盖既有 maneuver。
- `test_p07_target_lane_safety_preempts_utility_override`
  - 同时设置 override choice 1 与 unsafe target lane。
  - 断言 final choice 必为 stay lane 2，fallback reason 为 `target_lane_unsafe`。
- `test_p07_does_not_rerun_aps_cmc_or_p06_conflict_resolution`
  - 断言 P07 不产生 APS / CMC / cooperative_request collection / conflict_resolution 新事件。
  - 断言不重选 CLV / CFV，不改变 P06 winner。
- `test_p07_does_not_create_longitudinal_or_lateral_candidates`
  - 断言没有 `CandidateLongitudinalKinematics`、没有 planning speed、没有 IDM / CPID 输出。
  - 断言没有 `CandidateLateralKinematics`、没有正弦轨迹、没有 lane-change progress。
- `test_p07_cuc_decision_not_persistent_and_no_write_before_commit`
  - 断言 `CUCDecision` 只在 command / event / history 中，不进入下一步 `VehicleState`。
  - 断言 P07 前后 `S(t)` 签名一致。
- `test_p07_png_features_registered_without_formal_renderer`
  - 断言 CUC decision marker、lane-change intent marker、fallback marker、non-compliant ignored marker、same-step overlay marker 注册。

P07 targeted test 红灯应停在以下层：

| 预期红灯位置 | 合格失败例 |
| --- | --- |
| expected event matcher | 缺 `CUC` / `cuc` event、缺 target lane safety event、缺 compliance event。 |
| command buffer inspection | 缺 lane-change command、缺 spacing override command、误生成 forbidden command。 |
| overlay payload inspection | 缺 same-step overlay 或缺工程补丁标记。 |
| sanity matcher | non-compliant no action / unsafe fallback / no-write-before-commit 未记录。 |
| expected_png_features matcher | 缺 P07 marker registration。 |

不合格红灯包括：unknown scenario、unknown enum、unknown field、ImportError、AttributeError、测试 helper 自身构造错误、自然语言字符串断言。

## 8. 验收证据

P07 implementation 完成后，必须返回以下证据，而不是只给 pytest 数字。

- Targeted MVS green evidence:
  - `MVS-CUC-1A_override_choice1` targeted green evidence。
  - `MVS-CUC-2` P07 Step6 targeted green evidence：target lane unsafe fallback + spacing handoff，不包含 longitudinal_model consumption。
  - `MVS-CUC-3` targeted green evidence。
  - `MVS-CUC-1B_real_utility_probe` evidence，并明确它是 probe，不阻塞 required。
  - `MVS-CUC-1C_real_utility_choice1_locked` 仍 deferred 的 evidence。
- CUC event 样例：

```text
event_type = CUC / cuc
module = Step6CUC
vehicle_id = CV_CUC
related_vehicle_ids = [MV_CUC, CV_CUC, TLV_ID, TFV_ID]
reason = final_choice_change_to_lane_1 / fallback_target_lane_unsafe / non_compliant_chv
source = paper_formula / test_harness_override / first_version_engineering_patch
is_engineering_patch = false unless event records overlay / first-version fallback mechanics
payload:
    cuc_decision_id
    source_request_id
    source_mv_id
    cv_id
    cv_role
    utility_source
    utility_inputs_logged
    U1
    U2
    recommended_choice
    effective_choice
    fallback_reason
    target_lane_safe
    compliance_state
```

- compliance event 样例：

```text
event_type = CUC / cuc
module = Step6CUCCompliance
vehicle_id = CV_CHV
reason = compliant_accepts_cuc / non_compliant_chv_ignored
payload:
    vehicle_type = CHV
    chv_compliance_state = non_compliant
    cuc_suggestion_executed = false
    lane_change_command_created = false
    spacing_override_consumed_by_p07 = false
```

- target lane safety event 样例：

```text
event_type = CUC / cuc
module = Step6TargetLaneSafety
vehicle_id = CV_CUC
reason = target_lane_safe / target_lane_unsafe
payload:
    target_lane = lane_1
    TLV_id
    TFV_id
    TT_CV_TLV
    TT_TFV_CV
    TT_min = 1.5
    target_lane_safe = true / false
```

- lane-change command 样例：

```text
lane_change_commands[CV_CUC]:
    command_id = p07:step:CV_CUC:lane_change
    vehicle_id = CV_CUC
    source_request_id = p06:step:CV_CUC:MV_CUC:cfv
    source_mv_id = MV_CUC
    source_lane = lane_2
    target_lane = lane_1
    target_y = lane_1_centerline
    cuc_decision_id = p07:step:CV_CUC:cuc_decision
    overlay_id = p07:step:CV_CUC:same_step_overlay
    init_maneuver = true
```

- same-step overlay 样例：

```text
same_step_overlays[CV_CUC]:
    overlay_id = p07:step:CV_CUC:same_step_overlay
    vehicle_id = CV_CUC
    source_request_id = p06:step:CV_CUC:MV_CUC:cfv
    source = first_version_engineering_patch
    reason = same_step_cuc_lane_change_relation_overlay
    is_engineering_patch = true
    source_lane = lane_2
    target_lane = lane_1
    target_lane_neighbors = {TLV_id, TFV_id}
```

- desired spacing override / cooperation command 样例：

```text
cooperation_commands[CV_CUC]:
    command_id = p07:step:CV_CUC:spacing_override
    vehicle_id = CV_CUC
    source_request_id = p06:step:CV_CUC:MV_CUC:cfv
    source_mv_id = MV_CUC
    cv_role = cfv
    aps_case = case_2 / case_4
    eq10_desired_spacing = <value from P06 active request desired_spacing_override>
    consumed_by = P08
    p07_longitudinal_candidate_created = false
```

- final choice 样例：

```text
final_choice = change_to_lane_1
effective_choice = change_to_lane_1
lane_change_command_created = true
same_step_overlay_created = true
vehicle_state_written = false
```

```text
final_choice = stay_lane_2
fallback_reason = target_lane_unsafe / utility_not_better
lane_change_command_created = false
spacing_override_command_created = true
longitudinal_candidate_created = false
```

```text
non_compliant_CHV:
    effective_choice = not_applicable
    accepted_by_vehicle = false
    fallback_reason = non_compliant_chv
    lane_change_command_created = false
    spacing_override_command_created = false
    eq10_consumed = false
```

- relevant sanity check 样例：

```text
check_type = no_write_before_commit
result = pass
reason = p07_no_write_before_commit
payload:
    state_unchanged = true
    p07_outputs_are_command_event_overlay_only = true
```

```text
check_type = state_machine_inconsistency 或已授权等价 check_type
result = pass
reason = cuc_decision_not_persistent
payload:
    cuc_decision_persisted_to_vehicle_state = false
    active_lane_change_skip_cuc = true / false
```

如果实现需要新增 `target_lane_unsafe_fallback`、`non_compliant_no_action`、`active_lane_change_skip_CUC` 或 `CUCChoice_not_persistent` 作为正式 `SanityCheckType`，必须先修订代码数据结构规格；不能在实现中直接新增未授权 enum。

- P07 PNG marker / expected_png_features 样例：

```text
cuc_decision_marker visible
lane_change_intent_marker visible
spacing_override_marker visible
target_lane_unsafe_fallback_marker visible
non_compliant_ignored_marker visible
same_step_overlay_marker visible
```

- P07 没有重做 APS 的证据：
  - 不产生 APS candidate / APS assignment / APS cache refresh event。
  - 不更新 APS cache。
  - 不重新选择 CLV / CFV。
- P07 没有重做 CMC 的证据：
  - 不产生 CMC decision / Eq.53 / boundary speed cap / merge command event。
  - 不写 `merge_commands` 或 `speed_cap_commands`。
- P07 没有重做 P06 conflict resolution 的证据：
  - 不产生新的 `cooperative_request` collection event。
  - 不产生新的 `conflict_resolution` winner / loser event。
  - P07 input winner 与 P06 active request 一致。
  - suppressed / loser request 不触发 CUC。
- P07 没有执行 P08 / P09 的证据：
  - 不产生 longitudinal candidate、planning speed、IDM / CPID result。
  - 不产生 lateral candidate、sine trajectory、lane-change progress。
- no-write-before-commit 证据：
  - P07 前后冻结 `S(t)` 签名一致。
  - 真实 `x / y / v / a / physical_lane / road_role / lane_change_state / merge_state` 不变。
- Regression evidence:
  - P00 static traceability green。
  - P01-P06 targeted / regression green。
  - P07 targeted green。

## 9. 完成标准

P07 进入完成状态时必须满足：

- `MVS-CUC-1A_override_choice1` required gate 通过。
- `MVS-CUC-2` 的 P07 Step6 targeted required gate 通过：target lane unsafe fallback + spacing handoff；不要求 P07 通过完整 Eq.10 longitudinal consumption。
- `MVS-CUC-3` required gate 通过。
- `MVS-CUC-1B_real_utility_probe` 可观测但不阻塞 required。
- `MVS-CUC-1C_real_utility_choice1_locked` 仍 deferred，除非后续单独升级。
- P07 能稳定消费 P06 active cooperative request。
- P07 不消费 P06 suppressed / loser request。
- 无 active request 时不执行 CUC。
- active lane-change 中的 CV 跳过 CUC，不重复发起 lane-change。
- target lane safety fail 时必定不启动 lane-change。
- target lane unsafe 的 canonical fallback reason 为 `target_lane_unsafe`，不得使用 `target_lane_TT_unsafe`。
- target lane safety fail 优先级高于 utility / override。
- non-compliant CHV 不执行协同建议。
- non-compliant CHV 的 CUCDecision 使用 `effective_choice = not_applicable`，或 `effective_choice = stay_lane_2` + `accepted_by_vehicle=false`；不得使用未授权 `not_executed`。
- non-compliant CHV 不生成 lane-change command。
- non-compliant CHV 不消费 Eq.10 desired spacing override。
- choice 1 只生成 command / transition request / overlay，不直接改真实 lane / y / lane_change_state。
- choice 2 只输出 P08 可消费的 spacing override / cooperation result，不直接跑纵向模型。
- real utility probe 记录 inputs、U1 / U2、target lane safety 和 final choice。
- test harness override 明确标记为测试钩子，不写成论文公式。
- same-step overlay 明确是 first-version engineering patch，并携带 `source / reason / is_engineering_patch`。
- P07 不重做 APS。
- P07 不重做 CMC。
- P07 不重做 P06 conflict resolution。
- P07 不实现 P08 / P09。
- P07 不实现 P10 / P11 / P12。
- P07 不直接写真实车辆状态。
- `CUCDecision` 不成为跨步持久控制状态。
- event / sanity / PNG marker 不等 P11 才补。
- required / probe / deferred 语义仍由 P01 runner 保持。
- 若新增字段、enum、ScenarioConfig、EventRecord、SanityCheckRecord 或 expected_* 口径，已有上游规格先行修订并可追溯。
- P00-P06 回归保持绿色。

## 10. 回归保护

- 所有算法内部使用 `x_global`；`x_plot` 只用于 PNG / renderer 派生层。
- P07 只读冻结 `S(t)`。
- P07 只消费 P06 active cooperative request。
- P07 不得把 suppressed / loser request 当作 active CUC 输入。
- P07 不得重算 APS，不得重选 CLV / CFV。
- P07 不得重算 CMC，不得重判 Eq.53。
- P07 不得重算 P06 request / conflict winner。
- P07 不得为 invalid / failed / empty assignment 伪造 active request。
- P07 不得对 active lane-change 车辆重新执行 CUC。
- P07 不得在 target lane unsafe 时发起 lane-change。
- P07 不得让 non-compliant CHV 执行 CUC 建议。
- P07 不得让 non-compliant CHV 消费 Eq.10 spacing override。
- P07 不得把 test harness override 写成 paper formula。
- P07 不得把 same-step overlay 写成论文原语义；必须保持工程补丁标记。
- P07 不得生成 longitudinal candidate、planning speed、IDM / CPID result。
- P07 不得生成 lateral candidate、正弦轨迹或 lane-change progress。
- P07 不得直接修改 `VehicleState`、`SimulationState`、APS cache、lane_change_state、merge_state 或 active maneuver state。
- P07 不得写 Step 9 commit result。
- `CUCDecision` 默认只能作为本步 derived result / event / history evidence，并通过 command payload 中的 `cuc_decision_id` 被引用；不能作为下一步控制状态。若要写入 `CommandBuffer.cuc_decisions`，必须先补数据结构规格。
- command / next-state 不反写真实状态。
- commit 是唯一生成 `S(t+dt)` 的阶段。
- `MVS-CUC-1A/2/3` 是 P07 required gate，其中 `MVS-CUC-2` 在 P07 只代表 Step6 targeted fallback + spacing handoff；`MVS-CUC-1B` 是 probe；`MVS-CUC-1C` 是 deferred。
- `MVS-E2E-1`、`MVS-SAFE-1A_waiting_cap`、`MVS-SAFE-1B_executing_cap_lateral_consumption`、`MVS-COMMIT-1-full` 不得被 P07 误报为 full pass。
- P07 event / sanity / PNG feature 不得等待 P11 才补。
- P07 必须保留 P04 targeted APS gate 行为，不破坏 `MVS-APS-*`。
- P07 必须保留 P05 targeted CMC / assignment validation 行为，不破坏 `MVS-CMC-*`、`MVS-ASSIGN-1`。
- P07 必须保留 P06 conflict behavior，不破坏 `MVS-CONFLICT-*`。
- 若后续实现需要扩展 ScenarioConfig 以直接加载 active request、utility override、compliance state 或 target lane safety fixture，必须先修订 `CORMC代码数据结构设计_整理版.md` 与 MVS 文档，再实现代码。
