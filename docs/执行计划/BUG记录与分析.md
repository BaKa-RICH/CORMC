# BUG记录与分析

本文记录 BASIC-01 当前暴露出的算法与代码问题，作为后续修复 BASIC-01 的工作底稿。

记录口径：先查验、先归档，不提前把争议点判成“成功”或“失败”。BASIC 验收既看 `first effective APS`，也尽量观察整个 `control_zone` 生命周期内的 `case / active CV / Eq.10` 是否稳定；但当前阶段以记录为主。BASIC 成功的必要条件之一是 MV 合流成功。`stay_lane_2` 是 BASIC 测试期望，不直接等同于全局算法硬规则。

当前聚焦场景：`BASIC-01`。后续先基于本文修 BASIC-01，修复过程中如果暴露新问题，再追加到本文或新文档中。

## 1. 证据来源

本轮查验主要使用以下材料：

- 代码入口与调度：
  - `cormc/engine.py`
  - `cormc/step4a_aps.py`
  - `cormc/step4b_cmc.py`
  - `cormc/step5_cooperative_request.py`
  - `cormc/step6_cuc.py`
  - `cormc/step7_longitudinal.py`
  - `cormc/step8_lateral.py`
  - `cormc/step9_11.py`
  - `cormc/step0_3.py`
  - `cormc/p145_parameters.py`
- 公式与流程文档：
  - `docs/复现讨论/CORMC时间步执行顺序梳理.md`
  - `docs/复现讨论/CORMC论文公式与实现映射.md`
  - `docs/papers/P002-2023-Hou-Cooperative-On-Ramp-Merging-Control.md`
- BASIC-01 运行证据：
  - `artifacts/basic/basic_long_dt_900/scenarios/BASIC-01/events.jsonl`
  - `artifacts/basic/basic_long_dt_900/scenarios/BASIC-01/trajectory.csv`
  - `artifacts/basic/basic_long_dt_900/scenarios/BASIC-01/numeric_summary.json`

注意：`events.jsonl` 和 `numeric_summary.json` 很大，本文只记录关键字段和结论，不直接倾倒大段内容。

## 2. 先回答一个职责问题：APS 选 case，CUC 不选 case

用户问题：`aps只选择active cv，但active cv选择case几是cuc决定的对吗？`

当前代码答案：不是。当前主链路中，APS 决定 `aps_case`，也决定 `CLV / CFV` 以及 `col_clv / col_cfv`。Step5 根据 APS assignment 中的 `col_*` 生成 active cooperative request。CUC 只处理这些 active request，并在 `change_to_lane_1` 与 `stay_lane_2` 之间选择 maneuver。

代码依据：

- `step4a_aps.py:332-358`：`classify_aps_case(...)` 返回 `case_2 / case_3 / case_4`，并返回 `col_clv / col_cfv / Eq.10 spacing`。
- `step5_cooperative_request.py:208-241`：只有 assignment 里的 `col_clv` 或 `col_cfv` 为真时，才为对应 CV 生成 cooperative request。
- `step6_cuc.py:42`：CUC 入口接收的是 `active_requests`。
- `step6_cuc.py:161-171` 与 `step6_cuc.py:495`：CUC 根据 utility 和安全条件决定 `change_to_lane_1` 或 `stay_lane_2`。

所以，BASIC-01 中 step 56 从 `case_2` 漂到 `case_3`，直接来源是 APS 重新分类，不是 CUC 选择 case。CUC 后续把 `B01_CLV` 推去 lane 1，是在 APS 已经把 active request 转给 `B01_CLV` 之后发生的。

## 3. APS 与 CUC 的滚动条件

当前代码与用户口径一致：CUC 没有独立 5 秒周期。它是每个仿真步，只要 Step5 生成 active cooperative request，就跑一次。若 `dt = 0.1s`，则在 active request 连续存在时，CUC 实际上每 `0.1s` 重新算一次。

当前调度链：

- `engine.py:158-162`：生成 `aps_eligible_mv_ids`，要求 MV 所在区域允许 APS，且 MV 还不是 `merge_state == executing`。
- `step4a_aps.py:26`：`APS_DECISION_INTERVAL_S = 5.0`。Fresh APS 受 5 秒间隔约束；非 due 时可以 reuse cache。
- `engine.py:187-189`：Step5 只拿 `p04.effective_assignments` 中属于 `aps_eligible_mv_ids` 的 assignment。
- `step5_cooperative_request.py:109-170`：Step5 从有效 assignment 生成 active request。
- `engine.py:191-203`：Step6 CUC 只吃 Step5 输出的 `p06.active_requests`。
- `step6_cuc.py:61-72`：没有 active request 时，CUC 发 `no_active_request_no_cuc` 事件并退出。

因此，BASIC-01 中 step 5 到 step 55 之间，APS 大多是 `reuse_cache`，但 Step5 每步仍可根据 cache 发 active request，CUC 每步继续算。到了 step 107，fresh APS due 失败后，虽然 cache 被保留，但当步没有 effective assignment 交给 Step5，于是 active request 为空，CUC 断掉。这一点见 BUG-005。

## 4. CUC utility：公式外形来自论文，但实现代入仍需核验

用户问题：`CUC 的决策不是“case 2 CFV 必须 stay lane_2”，而是算两个 utility。把这个计算的式子去和公式映射文档、论文核对。我要知道这个算两个 utility 是论文里的公式，还是项目自己编的。`

查验结论分两层：

1. `U1 / U2` 这两个 utility 的公式外形来自论文，不是项目凭空编的。
2. 当前项目“如何计算公式里的加速度项”和“如何处理 BASIC stay_lane_2 期望”，仍是待核验/待修复问题，不能据此说实现已经正确。

公式来源：

- `docs/复现讨论/CORMC论文公式与实现映射.md:103-113`：CUC choice、utility、safety measure、TT 安全约束、final choice 都映射到论文原语义。
- `docs/复现讨论/CORMC论文公式与实现映射.md:157-170`：CUC Eq.11-Eq.16 被归档为论文原公式。
- `docs/复现讨论/CORMC论文公式与实现映射.md:241-247`：第一版口径写为 `if U1 > U2 and Eq.14 satisfied -> choice 1; otherwise -> choice 2`。
- `docs/papers/P002-2023-Hou-Cooperative-On-Ramp-Merging-Control.md:203-208`：论文给出 `U1(t)` 和 `U2(t)`。
- `docs/papers/P002-2023-Hou-Cooperative-On-Ramp-Merging-Control.md:211-213`：论文 Eq.13 是 safety measure。
- `docs/papers/P002-2023-Hou-Cooperative-On-Ramp-Merging-Control.md:219-236`：论文给出 TT 安全约束与 Eq.16 的 choice 规则。

代码实现：

- `step6_cuc.py:427-430`：通过 `_hypothetical_longitudinal_acceleration(...)` 计算四个假设加速度项。
- `step6_cuc.py:432-443`：按 `alpha / beta / gamma / zeta` 组合 `U1 / U2`。
- `step6_cuc.py:495`：`U1 > U2` 时推荐 `change_to_lane_1`，否则推荐 `stay_lane_2`。
- `step6_cuc.py:167-169`：如果推荐换到 lane 1 但 target lane 不安全，则回退为 `stay_lane_2`。
- `step6_cuc.py:662-680`：所谓假设加速度通过 `compute_p145_longitudinal_formula(...)` 代理计算。
- `p145_parameters.py:51-55`：当前参数为 `alpha=-1.0, beta=1.5, gamma=0.5, zeta=-0.5`。

待核验点：

- 公式结构接近论文 Eq.11-Eq.16，但当前项目把论文中的 `tilde a` 加速度项交给 `compute_p145_longitudinal_formula(...)` 代理计算。这个代理是否等价于论文所说两个车辆子系统的“新加速度”，还没有完全查清。
- BASIC-01 期望 `B01_CFV` 在 `case_2 + Eq.10` 下留在 lane 2 调间距；当前公式实现没有把这个测试期望作为硬约束或强惩罚。
- 用户明确指出“当前计算公式有问题，不能这么算”。因此本文把它列为待修复/待核验 BUG，而不是判定为公式正确。

## 5. BASIC-01 当前时间线

下面是当前 artifact 与代码能对上的关键链路。

### 5.1 step 5：first effective APS 是 case_2

`events.jsonl` 中 step 5：

- APS candidate：`["B01_CFV", "B01_CLV"]`
- APS：`aps_case = case_2`
- `clv_id = B01_CLV`
- `cfv_id = B01_CFV`
- `col_clv = false`
- `col_cfv = true`
- Eq.10 绑定 `B01_CFV`
- Step5 给 `B01_CFV` 发 active request
- CUC 对 `B01_CFV` 输出 `stay_lane_2`

当时 CUC 数值：

- `U1 = -5.4456894285595245`
- `U2 = 1.2399466276842512`
- `target_lane_safe = false`
- `effective_choice = stay_lane_2`

这一步符合 BASIC-01 的 first effective APS 期望：`case_2 + B01_CFV active + Eq.10 给 CFV`。

### 5.2 step 12：CUC 把 B01_CFV 推去 lane 1

`events.jsonl` 中 step 12：

- APS：`reuse_cache`
- active request 仍来自 `case_2`，对象仍是 `B01_CFV`
- CUC：
  - `U1 = -2.370317565461286`
  - `U2 = -2.440458142973931`
  - `U1 > U2`
  - `target_lane_safe = true`
  - `recommended_choice = change_to_lane_1`
  - `effective_choice = change_to_lane_1`
- CUC 生成命令：`p07:12:B01_CFV:lane_change`

`trajectory.csv` 中 step 12：

- `B01_CFV physical_lane = lane_2`
- `B01_CFV lane_change_state = executing`
- `B01_CFV y = 0.003219360529359949`

解释：这不是 APS 重新选择了别的 case，而是 CUC 在每步重算 utility 后，发现当前 `U1 > U2` 且目标车道安全，于是让 `B01_CFV` 开始从 lane 2 换到 lane 1。

与 BASIC-01 测试期望的冲突：BASIC-01 想观察 `B01_CFV` 留在 lane 2，按 Eq.10 调间距给 MV 创造合流空间；当前 CUC 在 step 12 中断了这个期望。

### 5.3 step 56：APS 重新 due，case_2 漂成 case_3

`events.jsonl` 中 step 56：

- APS candidate 仍是 `["B01_CFV", "B01_CLV"]`
- APS due 重新计算
- APS 输出：
  - `aps_case = case_3`
  - `col_clv = true`
  - `col_cfv = false`
  - active request 转给 `B01_CLV`

关键数值链路：

- step 5 时，MV 速度约 `21.846m/s`，`d_min_CLV = v_MV * 1.2 ≈ 26.2m`，而 `d*_CLV = 30m`，所以前向间距够。
- step 56 时，MV 速度约 `28.983m/s`，`d_min_CLV = v_MV * 1.2 ≈ 34.8m`，但 `d*_CLV` 仍约 `30m`，所以前向间距从“够”变成“不够”。
- 同时，`B01_CFV` 因 step 12 开始换道，在 step 56 已经几乎到 lane 1，但仍被 APS 当成 lane 2 候选。

`trajectory.csv` 中 step 56：

- `B01_MV v = 29.02397648015787`
- `B01_CFV y = 3.259018779895297`
- `B01_CFV physical_lane = lane_2`
- `B01_CFV lane_change_state = executing`

这里暴露两个问题：

- MV 的 on-ramp 纵向模型在 control zone 内继续按自由路加速，推高了 APS 最小间距阈值。
- APS 候选收集只看 `physical_lane == lane_2`，没有排除 `lane_change_state=executing` 且横向位置已经明显离开 lane 2 的车辆。

### 5.4 step 61 到 step 93：B01_CLV 也被 CUC 推去 lane 1

`events.jsonl` 中 step 61：

- APS：`reuse_cache`
- active request 来自 step 56 的 `case_3`
- CUC 对 `B01_CLV`：
  - `U1 = 1.948647632421113`
  - `U2 = 0.15888825865259265`
  - `target_lane_safe = true`
  - `effective_choice = change_to_lane_1`
- CUC 生成命令：`p07:61:B01_CLV:lane_change`

`trajectory.csv`：

- step 61：`B01_CFV physical_lane = lane_1`，`B01_CLV lane_change_state = executing`，`B01_CLV y = 0.007360258693470206`
- step 93：`B01_CLV physical_lane = lane_1`，`B01_CLV y = 3.5`，`lane_change_state = normal`

解释：`B01_CLV` 不是自己随机换道，也不是 APS 直接命令换道；它是在 step 56 APS 漂到 `case_3` 后成为 active CV，随后 CUC 在 step 61 根据 utility 选择 `change_to_lane_1`。

### 5.5 step 107：APS due 失败，保留旧 cache，但 Step5/CUC 断掉

`events.jsonl` 中 step 106：

- APS 仍是 `reuse_cache`
- active request 仍给 `B01_CLV`
- CUC 仍在运行

`events.jsonl` 中 step 107：

- APS candidate：`candidate_count = 0`
- APS due
- fresh APS 失败：`reason = insufficient_candidates`
- 事件写出：`effective_assignment_source = cache_retained_after_failed_APS`
- cache update 事件：`retain_on_failed_aps`
- 当步没有新的 effective assignment 进入 Step5，所以没有 active request，CUC 停止。

代码依据：

- `step4a_aps.py:617-659`：fresh APS 失败时，如果已有 cache，则标记 `retain_on_failed_aps`，并写 `effective_assignment_source = cache_retained_after_failed_APS`。
- `engine.py:187-203`：Step5 只拿 `p04.effective_assignments`；Step6 只拿 Step5 active request。

这导致一种混合状态：旧 cache 还在状态里，后续 CMC 可以读到；但当步 Step5/CUC 不再根据这个保留 cache 继续产生协同控制。这是 BASIC-01 后续卡死的重要机制。

### 5.6 step 112：CMC 读旧 cache，assignment validation 失败

`trajectory.csv` 中 step 112：

- `B01_MV x_global = 6955.238155477819`
- `B01_CLV physical_lane = lane_1`
- `B01_CFV physical_lane = lane_1`

`events.jsonl` 中 step 112：

- CMC assignment validation：
  - `assigned_clv_id = B01_CLV`
  - `assigned_cfv_id = B01_CFV`
  - `assignment_valid = false`
  - `invalid_reason = clv_not_lane_2`
- CMC 只生成 waiting command：`p05:112:waiting:B01_MV`

代码依据：

- `step4b_cmc.py:383-403`：CMC assignment source 优先读本步 effective assignment，否则读 `state.aps_assignment_cache[mv_id]`。
- `step4b_cmc.py:406-496`：CMC validation 检查 assignment 中的 CLV/CFV 是否仍在 lane 2。
- `step4b_cmc.py:470`：`clv_state.physical_lane != LANE_2` 时返回 `clv_not_lane_2`。

解释：CMC 问的“数据结构”就是 `aps_assignment_cache` 或本步 effective assignment 中保存的 APS assignment。step 112 读到的是 step 56 留下来的 assignment：`B01_CLV / B01_CFV / case_3`。但此时两辆车都已经在 lane 1，所以 CMC 判断这个合流间隙已经不存在，validation 失败。

### 5.7 step 113 以后：invalidate 留空 dict，诊断变成大量 clv_missing

step 112 validation 失败后，CMC 发出 cache invalidate request。

代码依据：

- `step4b_cmc.py:708-720`：构造 `operation = invalidate`。
- `step9_11.py:647-652`：提交 cache update 时，`operation in {"update", "invalidate"}` 都执行 `next_cache[owner_vehicle_id] = dict(cache_update.new_value or {})`。

因此 invalidate 后不是删除 `aps_assignment_cache["B01_MV"]`，而是留下一个空 dict。

`events.jsonl` 中 step 113：

- CMC 仍能读到 `assignment_source = aps_cache`
- 但 `assigned_clv_id = null`
- `assigned_cfv_id = null`
- `invalid_reason = clv_missing`

这不是 BASIC-01 失败的第一原因。第一原因更早：CV 被推离 lane 2，且进入 merge_zone 后没有恢复协同链路。但“空 dict 代表无效 cache”会放大诊断噪声，让后续日志持续报 `clv_missing`，不如直接删除 key 或记录结构化 invalid status 清楚。

## 6. 待查验与待修复 BUG 清单

### BUG-001：CUC utility 让 case_2 CFV 在 step 12 换到 lane 1，冲突 BASIC-01 stay_lane_2 期望

状态：已修复并验证。修复范围只覆盖 `B01_CFV` 在 step 12 因 U1/U2 代入错误而换到 lane 1 的问题；BASIC-01 后续仍有 APS rolling case 漂移、`B01_CLV` 后续换道、CMC/assignment 生命周期等问题，见后续 BUG 项。

原现象：

- BASIC-01 first effective APS 是 `case_2`，active CV 是 `B01_CFV`，Eq.10 desired spacing 绑定 `B01_CFV`。
- step 12，CUC 对 `B01_CFV` 算出：
  - `U1 = -2.370317565461286`
  - `U2 = -2.440458142973931`
  - `U1 > U2`
  - `target_lane_safe = true`
- 因为 Eq.16 当前规则是 `U1 > U2 且目标车道 TT 安全 -> choice 1`，所以 CUC 输出 `change_to_lane_1`，生成 `p07:12:B01_CFV:lane_change`。
- 这和 BASIC-01 的测试期望冲突：`B01_CFV` 应留在 lane 2，按 Eq.10 调整与 CLV 的间距，为 MV 合流创造 gap。

查明的原因：

1. CUC utility 的公式外形来自论文，不是项目自造公式。参数也和参数规格一致：
   - `alpha = -1`
   - `beta = 1.5`
   - `gamma = 0.5`
   - `zeta = -0.5`
   - `TT_min = 1.5s`
2. 旧实现的主要问题不在权重，而在代入：
   - 论文 Eq.13 的 `c_f^l(t)` 是 follower 为避免追尾 leader 所需的减速度，值越小越安全。
   - 旧代码直接使用 `(v_f - v_l)^2 / gap`。即使 follower 比 leader 慢、没有追尾趋势，也会因为平方得到正值，凭空产生安全惩罚。
   - BASIC-01 step 12 中，`B01_CFV` 比 lane 1 前车 `B01_TLV_CFV` 慢，但旧代码仍给 `c_CV_TLV` 一个正惩罚，扭曲了 U1。
3. 旧实现还没有让 choice 2 的 `tilde_a_CV^LV` 按 Eq.10 的 stay-lane 协同目标计算：
   - 论文 choice 2 表示 CV 继续留在 lane 2，CV 和 LV 构成 no lane-changing demand subsystem。
   - 对 BASIC-01 的 `case_2 + CFV` 来说，留在 lane 2 不是普通跟车，而是应消费 APS Eq.10 给出的 desired spacing。
   - 旧代码计算 U2 时调用普通纵向模型，未把 Step5 request 中的 `desired_spacing_override` 传给 `_hypothetical_longitudinal_acceleration(...)`。
   - 也就是说，旧代码实际比较的是“换到 lane 1”与“普通留在 lane 2”，而不是“换到 lane 1”与“按 Eq.10 留在 lane 2 协同调间距”。

修复后的 U1/U2 计算口径：

论文公式：

```text
U1(t) =
    alpha * (c_CV^TLV(t) + c_TFV^CV(t))
  + beta  * tilde_a_TFV^CV(t)
  + gamma * tilde_a_CV^TLV(t)
  + zeta  * abs(tilde_a_CV^TLV(t) - a_CV^LV(t))

U2(t) =
    alpha * (c_CV^LV(t) + c_FV^CV(t))
  + beta  * tilde_a_FV^CV(t)
  + gamma * tilde_a_CV^LV(t)
  + zeta  * abs(tilde_a_CV^LV(t) - a_CV^LV(t))
```

其中：

- `choice 1`：
  - `c_CV^TLV`：CV 相对 lane 1 目标前车 TLV 的安全项。
  - `c_TFV^CV`：lane 1 目标后车 TFV 相对 CV 的安全项。
  - `tilde_a_CV^TLV`：如果 CV 换到 lane 1，CV 以 TLV 为 leader 时的新加速度。
  - `tilde_a_TFV^CV`：如果 CV 换到 lane 1，TFV 以 CV 为 leader 时的新加速度。
- `choice 2`：
  - `c_CV^LV`：CV 相对原 lane 2 前车 LV 的安全项。
  - `c_FV^CV`：原 lane 2 后车 FV 相对 CV 的安全项。
  - `tilde_a_CV^LV`：如果 CV 留在 lane 2，CV 以 LV 为 leader 时的新加速度。若 request 是 `case_2 / case_4` 的 CFV 且带 Eq.10 desired spacing，则这里必须按 Eq.10 desired spacing 计算。
  - `tilde_a_FV^CV`：如果 CV 留在 lane 2，FV 以 CV 为 leader 时的新加速度。
- `a_CV^LV(t)`：CV 当前时刻在原 lane 2 跟随 LV 的当前加速度，用于舒适性项。

本次代码修改：

- `cormc/step6_cuc.py`
  - `_cuc_safety_term(...)` 中，当 `relative_speed <= 0` 时，将 Eq.13 safety term 记为 `0`，状态为 `not_closing_no_deceleration_required`。含义是 follower 没有追近 leader，不需要额外减速度避免追尾。
  - `_evaluate_utility_or_override(...)` 中，计算 choice 2 的 `tilde_a_CV^LV` 时，如果 active request 是 `case_2 / case_4` 的 `CFV` 且有 `desired_spacing_override`，则构造 `SpacingOverrideConsumption(consumed=True, desired_spacing=Eq10, desired_spacing_source="Eq10")`，传入 `_hypothetical_longitudinal_acceleration(...)`。
  - CUC event 新增诊断字段：
    - `choice2_spacing_override_applied`
    - `choice2_desired_spacing_override`
    - `hypothetical_accelerations.CV_LV.desired_spacing_target`
    - `hypothetical_accelerations.CV_LV.desired_spacing_target_source`
- `tests/test_p07_step6_cuc.py`
  - 新增 `test_p07_real_utility_choice2_uses_eq10_spacing_for_case2_cfv`，锁定 `case_2 CFV` 的 choice 2 假设加速度必须消费 Eq.10 spacing。

修复后的 BASIC-01 step 12 验证：

修复后，BASIC-01 step 12 对 `B01_CFV` 的 CUC 数值变为：

```text
U1 = -2.3335820688706947
U2 = -2.3335820688706947
recommended_choice = stay_lane_2
effective_choice = stay_lane_2
fallback_reason = utility_not_better
choice2_spacing_override_applied = true
```

也就是说，`U1` 不再大于 `U2`，Eq.16 不再触发 choice 1，`B01_CFV` 不再生成 `lane_change` command，而是保持 `stay_lane_2` 并继续按 Eq.10 调间距。

已运行验证：

```text
python -m pytest tests\test_p07_step6_cuc.py tests\test_p08_step7_longitudinal.py
27 passed

python -m pytest tests\test_basic_numeric_diagnostics.py tests\test_basic_scenarios.py
6 passed
```

BASIC-01 900 步诊断结果：

- `B01_CFV` 在 step 12 的错误换道已修复。
- first effective APS 仍为 `case_2`。
- Eq.10 consumer 仍为 `B01_CFV`。
- BASIC-01 整体仍未合流成功，原因已经后移到后续链路：
  - APS rolling 后续仍引入 `B01_CLV`。
  - `B01_CLV` 后续在 step 81 出现 `change_to_lane_1`。
  - CMC/assignment 生命周期仍会在 merge_zone 后失效。

因此，BUG-001 当前结论是：`case_2 CFV step 12 因 U1/U2 代入错误换道` 已修复；BASIC-01 未跑通不再由该 step 12 问题直接导致，后续应转入 APS rolling / MV 纵向加速 / assignment 生命周期相关 BUG。

### BUG-002：MV 在 control zone 内按自由路目标速度加速，导致 APS case 漂移

状态：已修复并验证。修复范围覆盖 control zone 内已存在有效 APS assignment 时，MV 继续按自由路目标速度加速、推高 APS 最小间距阈值并触发 case 漂移的问题；BASIC-01 后续是否最终合流仍由 assignment 生命周期、merge_zone 内 CUC/CMC 接力等后续 BUG 决定。

#### 原现象

- BASIC-01 修复前的 first effective APS 发生在 step 5：
  - `aps_case = case_2`
  - `clv_id = B01_CLV`
  - `cfv_id = B01_CFV`
  - `col_clv = false`
  - `col_cfv = true`
  - `d*_CLV = 30.0m`
  - Eq.10 desired spacing 绑定 `B01_CFV`
- step 5 时，MV 速度约 `21.846m/s`。APS 使用的前向最小安全间距阈值是 `v_MV * tau`，其中 `tau = 1.2s`，所以 `d_min_CLV ≈ 26.2m`。此时 `d*_CLV = 30.0m`，CLV 方向满足，APS 判为 `case_2`。
- 到 step 56 重新 due 时，旧实现下 MV 已按自由路目标速度继续加速到约 `28.983m/s`。此时 `d_min_CLV ≈ 34.8m`，但 APS 记录的 `d*_CLV` 仍约 `30.0m`，于是 CLV 方向从“够”变成“不够”。
- 因此 APS 从 `case_2` 漂到 `case_3`，active request 从 `B01_CFV` 转给 `B01_CLV`。后续 `B01_CLV` 又被 CUC 推向 lane 1，成为 BASIC-01 后续失败链的一环。
- 这个漂移不是 CUC 选择 case；case 漂移直接来自 APS 重新分类，而 APS 重新分类的关键数值变化来自 MV 在 control zone 内过度加速。

#### 查明的原因

- `step7_longitudinal.py` 中 on-ramp MV 的旧纵向模型只按自由路目标速度计算：`acceleration = CAV.k1 * (desired_speed - current.v)`。
- `desired_speed` 来自车辆 spec，缺省落到 `CAV.v_e = 30.0m/s`，而 `CAV.k1 = 0.4`。
- 旧逻辑没有消费 APS 已经形成的 gap reservation 信息：
  - 不读取 `aps_assignment_cache[mv_id]` 里的 `d*_CLV`。
  - 不读取 APS 判据使用的最小汇入时间间隔 `tau = APS_MIN_MERGE_TIME_GAP_S = 1.2s`。
  - 不知道当前 `case_2` 已经预约了一个可用 gap，且 `d*_CLV = 30.0m` 只允许 MV 速度保持在 `30.0 / 1.2 = 25.0m/s` 以内，才不会在下一轮 APS due 时把 `d_min_CLV = v_MV * tau` 推到 `d*_CLV` 之上。
- 所以旧实现实际比较的是“APS 已经预约的 gap”与“继续自由加速后的 MV 新速度”。只要 MV 继续朝 `30m/s` 加速，`v_MV * 1.2` 就会超过 `30m`，原本稳定的 `case_2` 会被自己后续的纵向推进破坏。

#### 改了什么

- `cormc/step4a_aps.py`
  - APS assignment cache 现在写入 gap reservation 字段：
    - `d_star_clv`
    - `d_star_cfv`
    - `aps_min_merge_time_gap_s`
  - APS event payload 同步记录这些字段，便于后续诊断复核。
- `cormc/mvs/loader.py`
  - MVS / 预加载 assignment 支持读取和构造上述 gap reservation 字段，保证回放、单测和闭环路径使用同一份 assignment 语义。
- `cormc/step7_longitudinal.py`
  - 新增 `APSGapProtectionResult`。
  - 新增 `resolve_aps_gap_protection_speed_cap(...)`：
    - 只对 on-ramp MV 生效。
    - 只在 `control_zone` 内生效。
    - 只在存在有效 APS assignment cache 时生效。
    - 支持 `case_1 / case_2 / case_3 / case_4`。
    - 从 assignment 中读取 `d_star_clv` 和 `aps_min_merge_time_gap_s`。
    - 计算速度上限 `speed_cap = d_star_clv / tau`。
  - `_on_ramp_longitudinal_formula(...)` 不再盲目使用自由路目标速度，而是：
    - 保留 `original_desired_speed`，通常仍是 `30.0m/s`。
    - 若 APS gap protection 生效，则令 `effective_desired_speed = min(original_desired_speed, d_star_clv / tau)`。
    - 再用 `effective_desired_speed` 计算加速度。
  - longitudinal event 新增诊断字段：
    - `current_speed`
    - `original_desired_speed`
    - `effective_desired_speed`
    - `aps_gap_protection_applied`
    - `aps_gap_protection_speed_cap`
    - `aps_gap_protection_source`
    - `source_aps_case`
    - `source_d_star_clv`
    - `source_tau`
    - `aps_gap_protection_rejection_reason`
  - 进入 merge zone 后不使用这套 APS gap protection，避免和 CMC boundary speed cap 混在一起；merge zone 的速度约束仍由 CMC speed cap 口径负责。
- `cormc/engine.py`
  - Step7 调用时传入 engine 当前 geometry，使 `resolve_on_ramp_control_region(...)` 使用同一套道路区域定义，而不是隐式默认几何。
- `cormc/basic_runner.py`
  - BASIC numeric summary 增加：
    - `aps_assignment_timeline`
    - `aps_gap_protection_timeline`
  - event summary 增加 `t_star_mv / d_star_clv / d_star_cfv / aps_min_merge_time_gap_s`，用于从 artifact 里直接观察 APS reservation 与 MV 速度保护的关系。
- `cormc/sumo/mvs_replay_artifacts.py`
  - 回放导出同步保留相关诊断字段，避免 SUMO/MVS artifact 丢失本次修复依赖的 assignment 语义。
- `tests/test_p04_step4a_aps.py`
  - 新增测试，锁定 APS effective assignment 与 cache update 都必须包含 gap reservation 字段。
- `tests/test_p08_step7_longitudinal.py`
  - 新增测试，锁定 control zone 内四类 APS case 都会应用 `d_star_clv / tau` 速度保护。
  - 新增测试，锁定缺少 `d_star_clv` 时只记录诊断，不误应用保护。
  - 新增测试，锁定 assignment status 无效时不应用保护。
  - 新增测试，锁定 merge zone 内 CMC boundary speed cap 与 APS gap protection 相互独立。
- `tests/test_basic_numeric_diagnostics.py`
  - 新增 BASIC-01 70 step 诊断测试，锁定：
    - first APS 仍为 `case_2`。
    - first APS 的 `d_star_clv = 30.0`。
    - first APS 的 `aps_min_merge_time_gap_s = 1.2`。
    - step 50 以后 `current_speed * tau <= d_star_clv`。
    - APS assignment timeline 不再出现 `case_3`。
    - active CV 不再多出 `B01_CLV`。

#### 验证

已运行目标回归：

```text
python -m pytest tests\test_basic_numeric_diagnostics.py::test_basic_01_bug002_mv_gap_protection_stabilizes_aps_case tests\test_p04_step4a_aps.py::test_p04_effective_assignment_cache_includes_gap_reservation_fields tests\test_p08_step7_longitudinal.py::test_p08_mv_control_zone_applies_aps_gap_protection_for_all_cases tests\test_p08_step7_longitudinal.py::test_p08_mv_control_zone_valid_assignment_missing_d_star_clv_is_diagnostic tests\test_p08_step7_longitudinal.py::test_p08_mv_control_zone_ignores_invalid_assignment_status tests\test_p08_step7_longitudinal.py::test_p08_mv_merge_zone_keeps_cmc_boundary_speed_cap_independent
6 passed
```

BASIC-01 70 step 诊断抽查结果：

```text
first_aps_step = 5
first_aps.aps_case = case_2
first_aps.clv_id = B01_CLV
first_aps.cfv_id = B01_CFV
first_aps.col_clv = false
first_aps.col_cfv = true
first_aps.d_star_clv = 30.0
first_aps.d_star_cfv = -23.87452478299292
first_aps.aps_min_merge_time_gap_s = 1.2
first_aps.desired_spacing_override = 59.03195931723492
aps_assignment_timeline = [case_2, case_1]
active_cv_ids = [B01_CFV]
```

step 56 的关键保护数值：

```text
current_speed = 24.63273780110797
source_tau = 1.2
current_speed_times_tau = 29.559285361329565
source_d_star_clv = 30.0
original_desired_speed = 30.0
effective_desired_speed = 25.0
aps_gap_protection_applied = true
aps_gap_protection_speed_cap = 25.0
aps_gap_protection_source = d_star_clv_over_tau
source_aps_case = case_2
candidate_speed_after_lane_clip = 24.647428289063654
```

验证结论：

- 修复前 step 56 的核心问题是 `v_MV * 1.2 ≈ 34.8m > d*_CLV ≈ 30.0m`，导致 APS 从 `case_2` 漂到 `case_3`。
- 修复后 step 56 中 `v_MV * 1.2 = 29.559285361329565m <= d*_CLV = 30.0m`，MV 没有再把 APS 前向安全阈值推爆。
- BASIC-01 70 step 窗口内不再出现 `case_3`，`B01_CLV` 不再进入 active CV 集合。
- 70 step 抽查里后续出现 `case_1`，不是本 BUG 原先的 `case_2 -> case_3 / B01_CLV active` 漂移；它说明 BUG-002 已把“MV 过度加速导致 CLV 方向不足”的链路切断，后续仍需结合 assignment 生命周期和 merge_zone 接力继续看完整 BASIC-01 是否最终合流。

### BUG-003：APS 候选收集把正在离开 lane 2 的车辆仍当 lane 2 候选

状态：已修复，已在 BASIC-01 和分层回归中验证。

#### 原现象

- BASIC-01 长时间滚动诊断中，APS 会在 cached gap member 已经开始换道时继续把它当作 lane 2 gap boundary。
- 以修复前的 BASIC-01 链路为例：
  - `B01_CFV` 或后续 cached boundary 车辆已经进入 `lane_change_state = executing`。
  - 车辆的 `physical_lane` 暂时仍可能是 `lane_2`。
  - APS candidate / cache reuse 仍然可能使用这个车辆作为 CLV 或 CFV。
- 这会污染 APS case 判断、active CV 选择、Step5 cooperative request，并在 MV 进入 merge zone 后让 CMC 面对已失效的 assignment。
- cache invalidate 之前还会留下 `{}`，使后续诊断持续出现 `clv_missing` 噪声。

#### 查明的原因

- `collect_aps_candidates(...)` 原本主要依赖 `resolve_lane_ordering_by_x_global(state, LANE_2)` 的结果。
- `resolve_lane_ordering_by_x_global(...)` 的过滤条件是 `physical_lane == lane_2` 和 `is_active`，不检查 `lane_change_state`。
- 换道状态机语义上，`lane_change_state == executing` 表示车辆正在离开源车道，不应再作为稳定 lane 2 merge gap boundary。
- `y` 不能作为主判断条件：主状态来源应是 `physical_lane` 和 `lane_change_state`，`y` 只作一致性诊断。
- APS cache reuse 前没有重新校验 cached `clv_id` / `cfv_id` 是否仍然是稳定 lane 2 boundary。
- `operation = "invalidate"` 在 commit 层曾被当成 update 写入空 dict，导致 active cache key 没有真正删除。

#### 改了什么

- 新增共享判断 `resolve_lane_2_gap_boundary_eligibility(...)`，供 APS 和 CMC 复用。
- 稳定 lane 2 gap boundary 的合格条件统一为：
  - 车辆存在。
  - 车辆 active。
  - `physical_lane == lane_2`。
  - `lane_change_state != executing`。
- APS candidate 收集改为在 lane 2 窗口内再做稳定 boundary 过滤：
  - executing 车辆不进入 `candidate_ids`。
  - 被排除车辆记录到 `APS_candidate` payload 的 `excluded_candidates`，包含 `vehicle_id`、`physical_lane`、`lane_change_state`、`excluded_reason`。
- APS cache reuse 前增加 cached CLV/CFV boundary 校验：
  - 只要 cached `clv_id` 或 `cfv_id` 失效，control zone 内立即 fresh APS。
  - trigger 记为 `cached_gap_boundary_invalid`，优先级高于普通 rolling due。
- invalid boundary 触发 fresh APS 失败时：
  - 不再返回旧 effective assignment。
  - 不再标记 `cache_retained_after_failed_APS`。
  - 会产生 invalidate action，并记录 `old_cache_invalidated`、`invalid_boundary_role`、`invalid_boundary_id`、`invalid_reason`。
- CMC assignment validation 复用共享 boundary 判断：
  - CLV executing 时返回 `clv_lane_change_executing`。
  - CFV executing 时返回 `cfv_lane_change_executing`。
  - 原有 `clv_missing`、`cfv_missing`、`clv_not_lane_2`、`cfv_not_lane_2` 口径保留。
- commit cache 语义修复：
  - `operation = "invalidate"` 现在与 `cleanup` 一样删除 `aps_assignment_cache[owner_vehicle_id]`。
  - 不再在 active cache 中留下 `{}`。
  - commit event 增加 `cache_invalidate_vehicle_ids` 供诊断。
- BASIC numeric summary 增加可观测字段：
  - `aps_excluded_candidate_timeline`
  - `first_cached_boundary_invalidation`
  - `cached_boundary_invalidation_timeline`

#### 验证

- 目标单测：
  - `tests/test_p02_step0_3.py::test_lane_2_gap_boundary_eligibility_rejects_executing_lane_change`
  - `tests/test_p04_step4a_aps.py::test_p04_excludes_executing_lane_change_from_aps_candidates`
  - `tests/test_p04_step4a_aps.py::test_p04_invalid_cached_boundary_triggers_immediate_fresh_assignment`
  - `tests/test_p04_step4a_aps.py::test_p04_invalid_cached_boundary_failed_fresh_aps_does_not_retain_old_assignment`
  - `tests/test_p05_step4b_cmc.py::test_p05_rejects_executing_clv_and_cfv_with_structured_reason`
  - `tests/test_p03_step9_11.py::test_commit_invalidate_deletes_aps_assignment_cache_key`
- 分层回归：
  - P02/P03/P04/P05/P06/P07/P08/P09/P10/P12/BASIC 相关回归 `144 passed`。
  - 计划指定 P04/P05/P06/P07/P08/P09/P10/P12/BASIC 回归 `112 passed`。
- BASIC-01 900 steps 实测：
  - run_id: `bug003_basic01`
  - status: `failed`，未判定 BASIC-01 通过。
  - first excluded candidate: step 82，`B01_CLV` 被 APS 排除，reason=`lane_change_executing`。
  - first cached boundary invalidation: step 82，trigger=`cached_gap_boundary_invalid`，invalid boundary role=`clv`，id=`B01_CLV`，reason=`lane_change_executing`。
  - old cache 已 invalidate，不再留 `{}`。
  - CMC 从 step 112 起看到 `assignment_source = None`、`invalid_reason = clv_missing`，说明旧污染 assignment 已清掉，但后续仍没有恢复出有效 assignment 供 CMC 合流。
  - final MV 仍未合流成功：`merged_and_past_ramp = false`。
- 结论：BUG-003 的 APS gap membership / cache 污染链路已修复；BASIC-01 仍有 active CV 多出 `B01_CLV` 和后续无有效 assignment 的新失败链需继续追踪。

### BUG-004：APS rolling 允许 case/active CV 漂移，但 BASIC 需要全生命周期记录并尽量稳定

状态：已确认行为；是否允许漂移需要保留讨论。

现象：

- first effective APS：step 5，`case_2`，active CV 是 `B01_CFV`。
- step 56 APS due：变成 `case_3`，active CV 变成 `B01_CLV`。

当前实现依据：

- `step4a_aps.py:26`：APS 每 5 秒 due。
- `step4a_aps.py:332-358`：每次 fresh APS 都按当前状态重新分类 case。
- `engine.py` 与 `step5` 会把最新 effective assignment 转成 active request。

BASIC 观察口径：

- 不应只看 first APS 就静默判成功。
- 需要记录整个 control zone 生命周期内 `case / active CV / Eq.10` 是否漂移。
- 漂移本身未必全局不允许，但 BASIC-01 当前漂移与后续失败链路强相关，需要作为 bug/待讨论项保留。

待修复方向：

- 修完 BUG-002/BUG-003 后重跑，观察 step 56 是否还会漂移。
- 若仍漂移，需要讨论是否引入 assignment freeze、case hysteresis、或 BASIC 验收专用稳定约束。

### BUG-005：APS_due 失败时保留 stale cache，但 Step5/CUC 不继续运行

状态：已确认。

现象：

- step 106：仍 reuse step 56 cache，Step5 对 `B01_CLV` 发 active request，CUC 仍运行。
- step 107：APS due，但 lane 2 candidates 为空，fresh APS 失败。
- step 107：cache 被保留为 `retain_on_failed_aps`，但当步没有 effective assignment 给 Step5，因此 active request 为空，CUC 停止。
- step 112：CMC 又从 state cache 里读到旧 assignment。

代码依据：

- `step4a_aps.py:617-659`：fresh APS 失败时，保留已有 cache，并记录 `cache_retained_after_failed_APS`。
- `engine.py:187-203`：Step5 只使用 `p04.effective_assignments`；Step6 只使用 Step5 active request。

问题解释：

这使系统出现不一致语义：

- 对 CMC 来说：旧 cache 还存在，可以作为 assignment source。
- 对 Step5/CUC 来说：这个旧 cache 没有继续产生 active request，所以协同调间距链路断了。

待修复方向：

- fresh APS 失败但 cache 仍有效时，需要明确该 cache 是否应该成为本步 effective assignment。
- 如果 cache 无效，应立刻结构化 invalid 并提供恢复策略，而不是让 CMC 后面才读到旧数据后失败。

### BUG-006：B01_CLV 在 step 61 被 CUC 推到 lane 1，导致 step 112 clv_not_lane_2

状态：已确认。

现象：

- step 56：APS 漂到 `case_3`，active request 转给 `B01_CLV`。
- step 61：CUC 对 `B01_CLV` 算出 `U1=1.948647632421113`，`U2=0.15888825865259265`，于是输出 `change_to_lane_1`。
- step 93：`B01_CLV physical_lane = lane_1`。
- step 112：CMC validation 报 `clv_not_lane_2`。

解释：

`B01_CLV` 离开 lane 2 是当前算法链条的直接结果：step 56 APS 漂移 + step 61 CUC utility 选择。它不是单独的横向模块误动作。

待修复方向：

- 优先修 BUG-002/BUG-003，避免 step 56 错误漂移。
- 同时核验 CUC utility 在 CLV 场景下是否仍存在和 BUG-001 类似的代入问题。

### BUG-007：CMC assignment validation 读的是旧 APS assignment；失败后没有恢复策略

状态：已确认；用户判定为算法设计 bug。

现象：

- step 112，MV 进入 merge_zone 附近后，CMC 开始 validation。
- CMC 读到的 assignment 是此前 APS cache 中的 `B01_CLV / B01_CFV`。
- 但此时两车都在 lane 1，validation 失败：`clv_not_lane_2`。
- CMC 只输出 waiting，并 invalidate cache；没有重新找 gap、没有恢复 APS/CUC、没有继续调间距。

代码依据：

- `step4b_cmc.py:383-403`：CMC assignment source 读 effective assignment 或 `state.aps_assignment_cache`。
- `step4b_cmc.py:406-496`：validation 要求 assigned CLV/CFV 仍在 lane 2。
- `step4b_cmc.py:249-290`：validation 失败后生成 waiting command 与 cache invalidate request。

需要回答的“CMC 问的是什么数据结构”：

- 它问的是本步 `effective_assignments` 或 `state.aps_assignment_cache[mv_id]` 中保存的 assignment。
- 这个 assignment 包含 `clv_id / cfv_id / aps_case / col_clv / col_cfv / status / source` 等字段。
- validation 用当前 `vehicle_states` 去核对 assignment 中预约的 CLV/CFV 是否仍能构成 lane 2 合流间隙。

当前问题：

- assignment 没有在 CV 换道、APS 失败、进入 merge_zone 时得到一致更新。
- validation 发现旧 assignment 不可用后，没有 replacement assignment 或恢复协同的路径。

待修复方向：

- CMC invalid 后不能只 waiting。
- 需要设计恢复策略：重新构造 assignment、允许继续 CUC 调间距、或在 merge_zone 中由 CMC 接管并更新 CLV/CFV 目标。

### BUG-008?cache invalidate ?? dict??????? clv_missing ??

???????

????

- step 112?first invalid ? `clv_not_lane_2`?
- invalidate ????`aps_assignment_cache["B01_MV"]` ?? `{}`?
- step 113 ???CMC ??? dict?? `clv_missing`?

?????

- `build_assignment_cache_invalidate_request(...)` ?? `operation = "invalidate"`?
- commit ???? `invalidate` ? `update` ??????? `dict(new_value or {})`?
- ? dict ???? assignment????? active cache key????????????

?????

- `step9_11.py` ? `operation="invalidate"` ? `operation="cleanup"` ???? `aps_assignment_cache[owner_vehicle_id]`?
- commit event ?? `cache_invalidate_vehicle_ids`??????? event / artifact????? active assignment cache?
- APS invalid-boundary fresh failure ??? invalidate action ??? engine ?? commit ???? `CandidateCacheUpdate`?

???

- `tests/test_p03_step9_11.py::test_commit_invalidate_deletes_aps_assignment_cache_key`
- P03/P04/P05/P06-P10/P12 ???????
- ?? BASIC artifact ?????? cache key ?????? `{}` ?????? dict ???

### BUG-009：进入 merge_zone 后 Step5/CUC 被关掉，协同调间距无法继续

状态：已确认当前代码行为；用户判定为算法设计 bug。

现象：

- 进入 merge_zone 后，APS 不允许新触发，这是合理边界。
- 但当前 Step5 只从 `aps_eligible_mv_ids` 对应的 effective assignment 生成 request。
- merge_zone 中 MV 不再 APS eligible，于是 Step5 不再给 CLV/CFV 发 active request。
- Step6 CUC 没有 active request，就不运行。

代码依据：

- `engine.py:158-162`：APS eligible 依赖 `region.aps_allowed`。
- `engine.py:164-167`：CMC eligible 可以在 `region.cmc_allowed` 或 `merge_state == executing` 时运行。
- `engine.py:187-203`：Step5 request assignment 却只取 `aps_eligible_mv_ids`，不是 CMC eligible，也不是 active assignment lifecycle。
- `step6_cuc.py:61-72`：无 active request 时 CUC 不跑。

与主循环文档/论文语义的冲突：

- `docs/复现讨论/CORMC时间步执行顺序梳理.md:19-21`：第一版保留 CUC 和 CMC 主链。
- `docs/复现讨论/CORMC时间步执行顺序梳理.md:193-205`：主循环 Step7 区分 CUC 留在 lane 2 的 CV、未在 merging zone 的 MV、CMC waiting/executing MV。
- 论文描述中，CUC 是为了给 MV 创造 merging gaps；CMC 在 merging zone 中协调 MV 与 lane 2 车辆运动。若进入 merge_zone 后完全停止 CUC，等价于假设进入 merge_zone 前 gap 必须已经完全调好，这与用户当前期望不一致。

用户判断：

- 这是算法设计 bug。
- 进入 merge_zone 后，如果 CLV/CFV 还没调好间距，CUC 应继续工作，或 CMC 应接管并继续驱动协同 gap 调整。

待修复方向：

- Step5 active request 的生命周期不能只依赖 APS 当前是否允许 fresh/reuse。
- 需要让已存在且仍有效的 assignment 在 merge_zone 内继续驱动 CUC，直到 CMC 合流完成、assignment invalid 且有恢复策略、或明确终止。

### BUG-010：CMC/APS/CUC 输入更新依据没有形成一致闭环

状态：从 BUG-003/005/007/009 归纳出的系统性问题。

用户关心的问题：

- `进入merge_zone后，是只剩cuc和cmc运行，aps不运行，那cuc和cmc的输入是什么？`
- `他们的输入有及时更新吗？`
- `更新的依据是什么？`
- `如果看到整个算法链，更新依据有考虑到几辆车的状态位置吗？`

当前查验答案：

- 进入 merge_zone 后，当前代码并不是“CUC + CMC 都继续运行”。实际是 CMC 可以运行，但 CUC 可能因为 Step5 没有 active request 而停止。
- CMC 输入主要是 `effective_assignments` 或 `aps_assignment_cache` 加当前 `vehicle_states`。
- CUC 输入主要是 Step5 从 effective assignment 生成的 active request，加当前 `relations`。
- APS 候选输入来自 lane 2 ordering，而 lane 2 ordering 当前主要按 `physical_lane` 判断。

闭环缺口：

- CV 的横向状态变化没有及时反馈到 APS candidate eligibility。
- fresh APS 失败时，cache 是否还能作为 active assignment 的语义不一致。
- CMC validation 发现 assignment invalid 后，没有 replacement 或恢复机制。
- merge_zone 内 CUC 是否继续运行，没有按 assignment lifecycle 设计，而是被 APS eligibility 间接关掉。

待修复方向：

- 明确 assignment lifecycle 状态机：created / active_in_control_zone / active_in_merge_zone / invalid / completed / cleaned。
- 明确 APS、Step5、CUC、CMC 分别消费哪个状态。
- 明确每一步用哪些车辆状态更新 assignment：MV、CLV、CFV、CLV/CFV 的 target-lane 邻车、MV 前后边界车辆。

## 7. 与“第一版主循环”的对齐情况

当前代码能部分对上 `docs/复现讨论/CORMC时间步执行顺序梳理.md` 的第一版主循环，但 BASIC-01 暴露出几个没有封住的缺口。

能对上的部分：

- 每步冻结状态、刷新关系、按模块生成 command、最后统一提交，这和主循环 `S(t) -> command/next-state -> S(t+dt)` 的原则一致。
- APS 5 秒周期、CUC 处理 active request、CMC 在 merge zone 处理合流，模块顺序大体符合文档。
- lane change 执行和提交后更新 `physical_lane / lane_change_state` 的方向也符合主循环。

没有对上的关键缺口：

- 主循环文档提醒正在换道的 CV 有复杂 logical role，不能简单按物理 lane 处理；但 APS candidate 仍主要依赖 `physical_lane == lane_2`，导致 step 56 把正在离开 lane 2 的 CFV 当 lane 2 候选。
- 主循环里 CUC 留在 lane 2 的 CV 应继续形成 gap；当前 CUC 受到 Step5 active request 接线限制，在 APS_due 失败或进入 merge_zone 后会停止。
- 主循环里 CMC waiting/executing MV 应在 CMC 语义内计算纵向运动并使用 boundary speed cap；当前 CMC assignment invalid 后只有 waiting，没有恢复协同 gap 的策略。
- assignment cache 在主循环里“不清空”，但这不等于 stale assignment 可以无限保留。当前 cache invalidation 与 reuse 语义需要更明确。

结论：当前项目代码不是完全背离第一版主循环，而是主流程骨架对得上；但在 lane-changing candidate、assignment lifecycle、merge_zone 内 CUC/CMC 接力这三处闭环没有封住，BASIC-01 正好把这些问题暴露出来。

## 8. 建议修复顺序

后续建议先修 BASIC-01，不急着同时修六个场景。每次修完都跑 BASIC-01 长时间 dt，观察是否出现新 bug。

建议顺序：

1. 补必要诊断，不改算法也能观察：
   - 在 APS candidate 中记录每个 candidate 的 `physical_lane / y / lane_change_state / target_lane`。
   - 在 Step5 记录 active request 来源到底是 fresh APS、reuse cache、retained cache、还是 merge_zone active assignment。
   - 在 CMC validation 中记录 assignment 的 `created_at_step / last_updated_step / source / status`。
2. 修 BUG-003：
   - APS candidate eligibility 排除正在 lane 2 -> lane 1 换道的车辆。
   - 重跑 BASIC-01，观察 step 56 是否仍漂到 `case_3`。
3. 修 BUG-001：
   - 对照论文 Eq.11-Eq.16 核验 `tilde a` 的实现口径。
   - 决定 BASIC `case_2 + CFV + Eq.10` 是否需要硬性 stay lane 2，或通过正确 utility 代入自然得到 stay。
4. 修 BUG-002：
   - MV on-ramp pre-merge longitudinal model 接入 APS/CMC gap 目标，避免自由加速破坏已预约 gap。
5. 修 BUG-005/BUG-009：
   - 重新定义 assignment lifecycle，让有效 assignment 在 fresh APS 失败或 merge_zone 内仍可继续驱动 Step5/CUC，直到明确完成或 invalid。
6. 修 BUG-007：
   - CMC assignment invalid 后加入恢复策略，而不是只 waiting。
7. 修 BUG-008：
   - cache invalidate 不再留下空 dict；删除 key 或保留结构化 invalid record。
8. 重跑 BASIC-01：
   - 检查 first effective APS。
   - 检查 control_zone 生命周期 `case / active CV / Eq.10` 是否稳定或是否有记录明确的漂移。
   - 检查 `B01_CFV` 是否按 BASIC 期望 stay lane_2 调间距。
   - 检查 MV 是否成功合流并驶过 ramp。

## 9. 本文当前结论

BASIC-01 当前失败不是单点问题，而是一条可复现的链：

1. step 5 first APS 正确进入 `case_2`，`B01_CFV` 成为 active CV，Eq.10 给 `B01_CFV`。
2. step 12 CUC utility 让 `B01_CFV` 换到 lane 1，打断 BASIC-01 的 stay lane_2 调间距期望。
3. MV pre-merge 继续自由加速，推高 APS 最小间距阈值。
4. step 56 APS_due 时，`B01_CFV` 正在离开 lane 2 但仍被当作 lane 2 candidate，同时 MV 速度变大导致 case 从 `case_2` 漂到 `case_3`。
5. step 61 CUC 又让 `B01_CLV` 换到 lane 1。
6. step 107 APS_due 失败，旧 cache 被保留但 Step5/CUC 断掉。
7. step 112 CMC 读旧 assignment，发现 CLV 已不在 lane 2，validation 失败。
8. invalidate 后留下空 dict，后续持续出现 `clv_missing`。
9. 进入 merge_zone 后没有继续协同调间距或恢复 assignment 的路径，MV 最终无法按预期合流。

因此，后续修复不应只改一个判断条件。最小可控路线是：先修 APS candidate eligibility 和 CUC utility 代入口径，再处理 MV pre-merge 纵向模型和 merge_zone 内 assignment/CUC/CMC 生命周期。
