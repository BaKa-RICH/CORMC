# BASIC 六场景长时间 dt 滚动诊断记录

记录时间：2026-06-04

本文记录本轮 BASIC 六场景长时间 `dt` 滚动求解中已经确认的事实、已修问题、暴露问题与后续待讨论事项。目标是保留上下文，避免后续上下文压缩后丢失问题链条。

## 1. 本轮目标

本轮不做 SUMO replay，先完成内部数值仿真的 BASIC 六场景长时间滚动诊断。

核心目标有两个：

- 将算法区域判断统一到 SUMO 路网口径：
  - `x < 6650`: `pre_control`
  - `6650 <= x < 6950`: `control_zone`
  - `6950 <= x <= 7250`: `merge_zone`
  - `x > 7250`: `post_merge`
- 建立 BASIC 六场景数值诊断套件，能连续按 `dt` 推进，并输出可解释的问题链条。

本轮旧 required MVS suite 不是主验收标准；主验收是 BASIC 六场景能否长时间连续滚动，并把算法问题完整记录下来。

## 2. 已完成实现

### 2.1 区域模型与调度门控

已在 `cormc/step0_3.py` 中新增：

- `RoadGeometryConfig.control_start_global = 6650.0`
- `OnRampControlRegion`
- `resolve_on_ramp_control_region(...)`

区域语义：

| region | x 范围 | APS | cooperative request | CUC | CMC |
| --- | --- | --- | --- | --- | --- |
| `pre_control` | `x < 6650` | 禁止 | 禁止 | 禁止 | 禁止 |
| `control_zone` | `6650 <= x < 6950` | 允许 | 允许 | 允许 | 禁止 |
| `merge_zone` | `6950 <= x <= 7250` | 禁止新触发 | 禁止新触发 | 禁止新触发 | 允许 |
| `post_merge` | `x > 7250` | 禁止 | 禁止 | 禁止 | 禁止 |

已在 `cormc/engine.py` 中接入：

- 每步 freeze + relations 后计算每个 `road_role=on_ramp_mv` 的控制区。
- `run_step4a_aps(...)` 仅接收 control-zone MV。
- `run_step4b_cmc(...)` 仅接收 merge-zone MV，已经 `merge_state == executing` 的 MV 仍继续 CMC continuation。
- Step5 只消费本步 APS eligible assignment。
- Step6 在没有 active request 且 engine 明确禁止时，不再发空 CUC 事件。

已在 step 模块中做兼容改动：

- `cormc/step4a_aps.py`: `run_step4a_aps(..., eligible_mv_ids=None)`
- `cormc/step4b_cmc.py`: `run_step4b_cmc(..., eligible_mv_ids=None)`
- `cormc/step5_cooperative_request.py`: 修正 `effective_assignments={}` 不应回退到 APS cache 的问题；只有 `None` 才回退。
- `cormc/step6_cuc.py`: 新增 `emit_no_active_event=True`，engine 可关闭空请求诊断事件。

### 2.2 BASIC 场景与 runner

新增：

- `cormc/basic_scenarios.py`
  - 注册 `BASIC-01` 到 `BASIC-06`
  - 固定文档第 5.2-5.7 的初始车辆表
  - 固定每个场景的 expected APS case、expected active CV、expected Eq.10 consumer
- `cormc/basic_runner.py`
  - `run_basic_numeric_scenario(...)`
  - `run_basic_numeric_suite(...)`
  - `summarize_basic_numeric_result(...)`
  - 每场景输出 `trajectory.csv`、`events.jsonl`、`sanity.jsonl`、`numeric_summary.json`、`scenario_report.md`、`artifact_manifest.json`
  - suite 输出 `suite_summary.json`、`suite_report.md`、`artifact_manifest.json`

新增测试：

- `tests/test_basic_scenarios.py`
- `tests/test_basic_numeric_diagnostics.py`
- `tests/test_p02_step0_3.py` 中新增区域 resolver / geometry event 测试
- `tests/test_p12_deterministic_simulation_loop.py` 中新增 pre-control/control-zone/merge-zone engine 调度测试

验证结果：

```text
298 passed in 226.36s
```

## 3. 修复前后证据

### 3.1 修复前 baseline

修复前基线文件：

```text
artifacts/basic/region_gating_baseline.json
```

探针场景：MV 初始 `x_global = 6640 < 6650`，运行 5 steps。

修复前观察到的提前控制事件计数：

| event_type | count |
| --- | ---: |
| APS | 6 |
| assignment_cache | 5 |
| cooperative_request | 5 |
| CUC | 20 |
| CMC | 5 |
| assignment_validation | 0 |

baseline 结论：

```text
legacy scheduler runs control modules before x_global reaches 6650
```

这说明旧调度确实在 pre-control 阶段提前 APS / CUC / CMC。

### 3.2 修复后证据

正式 900 步 suite 中：

- `BASIC-01/02/03` 均记录了 resolved issue：
  - `region:pre_control_suppression_fixed`
- first APS 均发生在 `control_zone` 后。
- `x < 6650` 阶段未再观察到 APS / assignment_cache / cooperative_request / CUC / CMC。

具体 first control / first APS：

| scenario | first control step | first control x | first APS step | observed case |
| --- | ---: | ---: | ---: | --- |
| BASIC-01 | 5 | 6650.5689447424 | 5 | case_2 |
| BASIC-02 | 5 | 6650.5689447424 | 5 | case_3 |
| BASIC-03 | 5 | 6650.5689447424 | 5 | case_3 |

## 4. 正式 900 步 BASIC suite

正式输出目录：

```text
artifacts/basic/basic_long_dt_900/
```

根文件：

```text
artifacts/basic/basic_long_dt_900/suite_summary.json
artifacts/basic/basic_long_dt_900/suite_report.md
artifacts/basic/basic_long_dt_900/artifact_manifest.json
```

每个场景目录：

```text
artifacts/basic/basic_long_dt_900/scenarios/BASIC-01/
artifacts/basic/basic_long_dt_900/scenarios/BASIC-02/
artifacts/basic/basic_long_dt_900/scenarios/BASIC-03/
artifacts/basic/basic_long_dt_900/scenarios/BASIC-04/
artifacts/basic/basic_long_dt_900/scenarios/BASIC-05/
artifacts/basic/basic_long_dt_900/scenarios/BASIC-06/
```

每个场景目录都有：

```text
trajectory.csv
events.jsonl
sanity.jsonl
numeric_summary.json
scenario_report.md
artifact_manifest.json
```

Suite 状态统计：

| status | count |
| --- | ---: |
| failed | 3 |
| diagnosed_unresolved | 3 |

## 5. 六场景总览

| 场景 | 状态 | APS case | active CV | Eq.10 | merge | 主要诊断 |
| --- | --- | --- | --- | --- | --- | --- |
| BASIC-01 | failed | case_2 | B01_CFV, B01_CLV | B01_CFV | false | active CV 多出 B01_CLV |
| BASIC-02 | failed | case_3 | B02_CLV, B02_CFV | B02_CFV | false | active CV 多出 B02_CFV，且 Eq.10 被 CFV 消费 |
| BASIC-03 | failed | case_3 | B03_CLV, B03_CFV | B03_CFV | false | 预期 case_4，实际 first APS 为 case_3 |
| BASIC-04 | diagnosed_unresolved | case_2 | B04_CFV | B04_CFV | false | CUC 后续选择过 `change_to_lane_1`，未保持 stay lane_2 |
| BASIC-05 | diagnosed_unresolved | case_3 | B05_CLV | none | false | 主链正确，但 900 步内未完成 merged past ramp |
| BASIC-06 | diagnosed_unresolved | case_4 | B06_CLV, B06_CFV | B06_CFV | false | CUC 后续选择过 `change_to_lane_1`，未保持 stay lane_2 |

## 6. 分场景问题记录

### 6.1 BASIC-01

文档期望：

- pre-control case 2
- expected active CV: `B01_CFV`
- expected Eq.10 consumer: `B01_CFV`
- expected CUC: `B01_CFV stay_lane_2`

实际观察：

- `first_control_zone_step = 5`
- `first_control_zone_x_global = 6650.5689447424`
- `first_aps_step = 5`
- `observed_aps_case = case_2`
- `active_cv_ids = [B01_CFV, B01_CLV]`
- `eq10_consumers = [B01_CFV]`
- final MV:
  - `x_global = 7245.999997437435`
  - `v` 接近 0
  - `merge_state = not_started`
  - `physical_lane = on_ramp`
  - `y = -3.5`

已解决：

- pre-control 阶段未再触发控制模块；first APS 在进入 control zone 后发生。

未解决问题：

- `cuc_issue`: active CV 多出 `B01_CLV`。
- `cuc_issue`: 后续 CUC 出现非 stay lane 2 的选择，首次记录约 step 12。
- `recording_issue`: CMC 后续出现 assignment invalid warning：
  - 先出现 `clv_not_lane_2`
  - 随后大量 `clv_missing`
- `cmc_issue`: 900 steps 内 MV 未 merged 且未超过 `x_ramp_end_global`，卡在 ramp end 前。

判断：

- 区域门控已修。
- BASIC-01 的后续失败主要来自 CUC / assignment 生命周期 / CMC 等链路，不再是 pre-control 提前控制。

#### BUG-003/BUG-008 ?????

- ?? `run_id=bug003_basic01` ? BASIC-01 ?? 900 steps?
- ???? `failed`??? BASIC-01 ???`actual_steps = 900`?`merged_and_past_ramp = false`?final MV ?? `merge_state = not_started`?`x_global = 7245.999997437435`?
- BUG-003 ????????step 82?`B01_CLV` ? APS ? candidate ????reason ? `lane_change_executing`????????? `B01_CFV`?`candidate_count = 1`?
- cache ??????????step 82 ?? `cached_gap_boundary_invalid`?invalid boundary ? role=`clv`?id=`B01_CLV`?reason=`lane_change_executing`?old cache ? invalidate?????? `{}`?
- ?????CMC ? step 112 ??? `assignment_source = None`?`invalid_reason = clv_missing`??????? assignment ??????????????? assignment ? CMC ???
- active CV ??? `B01_CLV`?? CUC ??? stay lane 2 ????????????
- ?? artifacts?`artifacts/basic/BASIC-01/numeric_summary.json`?`artifacts/basic/BASIC-01/scenario_report.md`?

### 6.2 BASIC-02

文档期望：

- pre-control case 3
- expected active CV: `B02_CLV`
- expected Eq.10 consumer: none
- expected CUC: `B02_CLV stay_lane_2`

实际观察：

- `first_control_zone_step = 5`
- `first_control_zone_x_global = 6650.5689447424`
- `first_aps_step = 5`
- `observed_aps_case = case_3`
- `active_cv_ids = [B02_CLV, B02_CFV]`
- `eq10_consumers = [B02_CFV]`
- final MV:
  - `x_global = 7245.999997437435`
  - `v` 接近 0
  - `merge_state = not_started`
  - `physical_lane = on_ramp`
  - `y = -3.5`

已解决：

- pre-control 阶段未再触发控制模块；first APS 在进入 control zone 后发生。

未解决问题：

- `cuc_issue`: active CV 多出 `B02_CFV`。
- `cuc_issue`: case 3 场景中 `B02_CFV` 消费了 Eq.10，违反“case 3 无 Eq.10”的期望。
- `cuc_issue`: 后续 CUC 出现非 stay lane 2 的选择，首次记录约 step 58。
- `recording_issue`: assignment invalid warning：
  - `clv_not_lane_2`
  - 后续 `clv_missing`
- `cmc_issue`: 900 steps 内 MV 未 merged 且未超过 ramp end。

判断：

- 该场景 first APS case 正确，但后续 APS/cache/CUC 生命周期允许生成额外 active CV 与 Eq.10 消费，这属于 CUC/assignment 生效期策略问题。

### 6.3 BASIC-03

文档期望：

- pre-control case 4
- expected active CV: `B03_CLV`, `B03_CFV`
- expected Eq.10 consumer: `B03_CFV`
- expected CUC: both stay lane 2

实际观察：

- `first_control_zone_step = 5`
- `first_control_zone_x_global = 6650.5689447424`
- `first_aps_step = 5`
- `observed_aps_case = case_3`
- `active_cv_ids = [B03_CLV, B03_CFV]`
- `eq10_consumers = [B03_CFV]`
- final MV:
  - `x_global = 7245.999997437435`
  - `v` 接近 0
  - `merge_state = not_started`
  - `physical_lane = on_ramp`
  - `y = -3.5`

已解决：

- pre-control 阶段未再触发控制模块；first APS 在进入 control zone 后发生。

未解决问题：

- `aps_issue`: expected first APS `case_4`，实际 first APS 为 `case_3`。
- `cuc_issue`: 后续 CUC 出现非 stay lane 2 的选择，首次记录约 step 110。
- `recording_issue`: assignment invalid warning：
  - `clv_not_lane_2`
  - 后续 `clv_missing`
- `cmc_issue`: 900 steps 内 MV 未 merged 且未超过 ramp end。

判断：

- 该问题可能不是区域门控 bug，而是“pre-control 场景在 `x=6640` 起步，等到 `x>=6650` 才 first APS 后，相对几何已发生变化”的场景/算法耦合问题。
- 文档反推表按初始 `x=6640` 算 case 4，但算法 first APS 被正确延迟到 `x>=6650` 后才算，此时预测/相对位置可能已偏移，导致 first APS case 变为 case 3。
- 后续需要讨论：pre-control BASIC 的期望 case 是按初始几何，还是按进入 control zone 时的 first effective APS 几何。

### 6.4 BASIC-04

文档期望：

- ramp_pre case 2
- expected active CV: `B04_CFV`
- expected Eq.10 consumer: `B04_CFV`
- expected first APS step: 0
- expected CUC: `B04_CFV stay_lane_2`

实际观察：

- `first_control_zone_step = 0`
- `first_aps_step = 0`
- `observed_aps_case = case_2`
- `active_cv_ids = [B04_CFV]`
- `eq10_consumers = [B04_CFV]`
- CMC 从 step 40 左右开始记录 assignment validation / Eq.53 / boundary cap。
- final MV:
  - `x_global = 7245.999999530401`
  - `v` 接近 0
  - `merge_state = not_started`
  - `physical_lane = on_ramp`
  - `y = -3.5`

未解决问题：

- `cuc_issue`: CUC 后续选择过 `change_to_lane_1`，首次约 step 7，不符合 BASIC “CFV stay lane_2”期望。
- `recording_issue`: 后续出现 assignment invalid warning：
  - `cfv_not_lane_2`
  - 后续 `clv_missing`
- `cmc_issue`: 900 steps 内 MV 未 merged 且未超过 ramp end。

判断：

- first APS / active CV / Eq.10 主链正确。
- 后续未保持 stay lane 2 是策略问题：当前 CUC 每步可重新计算，未锁定 BASIC 期望中的 stay lane 2。
- 该问题在执行计划中属于“可以记录、留到下一次讨论”的策略类问题，但它会直接导致 assignment invalid 与 CMC 失败链条。

### 6.5 BASIC-05

文档期望：

- ramp_pre case 3
- expected active CV: `B05_CLV`
- expected Eq.10 consumer: none
- expected first APS step: 0
- expected CUC: `B05_CLV stay_lane_2`

实际观察：

- `first_control_zone_step = 0`
- `first_aps_step = 0`
- `observed_aps_case = case_3`
- `active_cv_ids = [B05_CLV]`
- `eq10_consumers = []`
- final MV:
  - `x_global = 7245.999999530401`
  - `v` 接近 0
  - `merge_state = executing`
  - `physical_lane = on_ramp`
  - `y = -0.05930196995621806`

未解决问题：

- `cmc_issue`: 900 steps 内未完成 `merged and x > 7250`。

判断：

- BASIC-05 主链最干净：
  - first APS 正确
  - active CV 正确
  - Eq.10 无消费正确
  - 未出现 active CV mismatch
- 但合流未完成，最终处于 `merge_state=executing` 且 y 接近 lane 2 centerline。
- 需要后续检查 merge completion / ramp end 附近速度收敛 / lateral progress completion 条件。

### 6.6 BASIC-06

文档期望：

- ramp_pre case 4
- expected active CV: `B06_CLV`, `B06_CFV`
- expected Eq.10 consumer: `B06_CFV`
- expected first APS step: 0
- expected CUC: both stay lane 2

实际观察：

- `first_control_zone_step = 0`
- `first_aps_step = 0`
- `observed_aps_case = case_4`
- `active_cv_ids = [B06_CLV, B06_CFV]`
- `eq10_consumers = [B06_CFV]`
- final MV:
  - `x_global = 7245.999999530401`
  - `v` 接近 0
  - `merge_state = not_started`
  - `physical_lane = on_ramp`
  - `y = -3.5`

未解决问题：

- `cuc_issue`: CUC 后续选择过 `change_to_lane_1`，首次约 step 10，不符合 BASIC “both stay lane_2”期望。
- `recording_issue`: assignment invalid warning：
  - `cfv_not_lane_2`
  - 后续 `clv_missing`
- `cmc_issue`: 900 steps 内 MV 未 merged 且未超过 ramp end。

判断：

- first APS / active CV / Eq.10 主链正确。
- 后续 failure 与 BASIC-04 类似，主要是 CUC 未锁定 stay lane 2，导致 assignment invalid 与 CMC 卡死链条。

## 7. 问题分类

### 7.1 已修问题

#### RG-001: pre-control 阶段提前 APS/CUC/CMC

分类：`region_gating_issue`

修复前：

- `x=6640` 运行 5 steps 即出现 APS / assignment_cache / cooperative_request / CUC / CMC。
- 证据：`artifacts/basic/region_gating_baseline.json`

修复后：

- `BASIC-01/02/03` first APS 都发生在进入 `control_zone` 后。
- pre-control 阶段非法控制事件为 0。

状态：已修。

### 7.2 未解决问题

#### APS-001: pre-control BASIC 的 expected case 与 first effective APS 几何口径冲突

影响场景：

- `BASIC-03`: expected `case_4`，first APS observed `case_3`

关联现象：

- 文档按初始 `x=6640` 反推 case。
- 修复后算法在 `x>=6650` 后才 first APS。
- 进入 control zone 时车辆相对位置 / 预测位置已经变化，可能导致 case 变化。

待讨论：

- BASIC-01/02/03 的 expected case 是否应按初始几何固定，还是按 first effective APS 时刻计算。
- 如果必须按初始几何，则场景表可能要重新倒推，使 `x>=6650` 后 first APS 仍得到目标 case。

#### CUC-001: CUC 没有锁定 BASIC 期望的 stay lane 2

影响场景：

- `BASIC-01`: 后续出现非 stay lane 2，约 step 12
- `BASIC-02`: 后续出现非 stay lane 2，约 step 58
- `BASIC-03`: 后续出现非 stay lane 2，约 step 110
- `BASIC-04`: 后续出现 `change_to_lane_1`，约 step 7
- `BASIC-06`: 后续出现 `change_to_lane_1`，约 step 10

影响：

- active CV 离开 lane 2 后，CMC assignment validation 后续可能出现 `clv_not_lane_2` / `cfv_not_lane_2`。
- 后续可能转化为 `clv_missing`。
- CMC 无法使用原 APS assignment 完成 Eq.53 合流。

待讨论：

- BASIC 场景是否需要锁定 active CV 在 assignment 生效期内 stay lane 2。
- 如果锁定，是场景测试约束、CUC 策略约束，还是 assignment 生命周期约束。

#### CUC-002: active CV 集合后续扩张 / Eq.10 后续误消费

影响场景：

- `BASIC-01`: active CV 多出 `B01_CLV`
- `BASIC-02`: active CV 多出 `B02_CFV`
- `BASIC-02`: case 3 场景中 `B02_CFV` 后续消费 Eq.10

可能原因：

- APS 周期更新 / cache 更新后产生新的 assignment，与 first APS 期望不同。
- BASIC runner 当前统计的是整个 900 步内所有 active request / Eq.10 consumer，而不是只统计 first APS 对应 assignment。

待讨论：

- BASIC gate 应该只约束 first effective APS，还是约束整个 control-zone assignment 生命周期。
- 若约束整个生命周期，需要定义 APS_due 后是否允许 case 变化、active CV 变化、Eq.10 consumer 变化。

#### CMC-001: 多场景未完成 merged past ramp

影响场景：

- 全部 BASIC 场景最终 `merged_and_past_ramp = false`

最终状态摘要：

| scenario | final x | final merge_state | final y | 备注 |
| --- | ---: | --- | ---: | --- |
| BASIC-01 | 7245.999997437435 | not_started | -3.5 | 卡在 ramp end 前，未合流 |
| BASIC-02 | 7245.999997437435 | not_started | -3.5 | 卡在 ramp end 前，未合流 |
| BASIC-03 | 7245.999997437435 | not_started | -3.5 | 卡在 ramp end 前，未合流 |
| BASIC-04 | 7245.999999530401 | not_started | -3.5 | 卡在 ramp end 前，未合流 |
| BASIC-05 | 7245.999999530401 | executing | -0.05930196995621806 | 已接近 lane 2，但未 completed |
| BASIC-06 | 7245.999999530401 | not_started | -3.5 | 卡在 ramp end 前，未合流 |

待讨论：

- `7246` 附近速度趋近 0 是否来自 boundary speed cap / longitudinal controller / CMC waiting 策略。
- BASIC-05 已接近 lane 2 centerline 但未完成 merge，需检查 lateral progress completion / merge completion 条件。
- 未合流是否需要立刻修，还是归档为下一轮 CMC 策略任务。

#### CMC-002: assignment invalid 后处理策略不明确

影响场景：

- `BASIC-01/02/03/04/06`

观察：

- active CV 换道或离开 lane 2 后出现 assignment invalid。
- 后续大量 `clv_missing` / `cfv_not_lane_2` / `clv_not_lane_2` warning。

待讨论：

- assignment invalid 后应等待下一次 APS、强制重新 APS、保守失败，还是进入其他 fallback。
- 当前实现记录了问题，但没有策略性恢复。

## 8. 当前结论

本轮已经完成：

- 区域门控 bug 已修，并有修复前后证据。
- BASIC 六场景可以连续运行 900 steps，无 runner crash。
- 每个场景都有完整 artifacts。
- 每个场景都能解释 APS / CUC / Eq.10 / CMC / merge outcome。
- 暴露出的算法问题已结构化进入 `numeric_summary.json` 和 `scenario_report.md`。

本轮没有完成：

- 六场景全部 merge。
- CUC stay lane 2 策略锁定。
- assignment invalid 后恢复策略。
- pre-control 场景 expected case 与 first effective APS 几何口径的最终判定。

## 9. 后续建议

建议下一轮按优先级处理：

1. 明确 BASIC gate 口径：
   - first APS gate 还是 full lifecycle gate。
   - pre-control 场景 expected case 按初始几何还是 control-zone first APS 几何。
2. 处理 CUC stay lane 2 锁定策略：
   - 若 BASIC 场景期望 stay lane 2 是强约束，需要防止后续 CUC 改为 lane 1。
3. 处理 assignment invalid 恢复策略：
   - 不应只产生大量 warning 而没有恢复路线。
4. 处理 ramp end 附近未 merge / 速度趋零问题：
   - 尤其是 BASIC-05，已经 `merge_state=executing` 且 y 接近 0，但未 completed。
5. 重新跑：
   - `run_basic_numeric_suite(run_id=..., max_steps=900)`
   - 全量 pytest

## 10. 关键验证命令记录

本轮最终验证：

```text
python -m pytest -q
298 passed in 226.36s
```

BASIC 900 步 suite：

```text
run_basic_numeric_suite(run_id='basic_long_dt_900', max_steps=900, render_png=False)
```

输出：

```text
artifacts/basic/basic_long_dt_900/
```
