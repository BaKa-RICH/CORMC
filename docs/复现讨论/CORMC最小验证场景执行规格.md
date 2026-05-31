# CORMC 最小验证场景执行规格

## 1. 文档定位

本文档是可执行场景层，面向代码实现、YAML / JSON / Python config 落地和 smoke suite 验收。

本文档只维护每个 `MVS-*` 场景需要加载什么、预加载什么、检查什么、按什么顺序执行；不重新定义 `TrajectoryRecord / EventRecord / SanityCheckRecord / ScenarioConfig` schema。字段、枚举、expected_* 结构和 loader 可接受字段以 `CORMC代码数据结构设计_整理版.md` 为准。日志、sanity check、PNG 的通用验收语义以 `CORMC输出指标与日志验证规格_整理版.md` 为准。

## 2. 通用执行规则

默认 module overrides：

```text
boundary_generation_enabled = false
random_arrival_enabled = false
random_vehicle_attributes_enabled = false
ordinary_mainline_lane_change_enabled = false
platoon_cmc_enabled = false
mpc_lateral_tracking_enabled = false
```

默认数值快照：

```text
dt = 0.1 s
x0_m_global = 6950 m
x_ramp_end_global = 7250 m
L_merging = 300 m
L_cr = 300 m
L = 4 m
d0 = 2 m
g_min_CM = 1.2 s
h_upper_CM = 1.2 s
xi = 2/3
a_p = 0.1 m/s^2
L_w = 3.5 m
TT_min = 1.5 s
lane_1_y = +3.5 m
lane_2_y = 0.0 m
on_ramp_y = -3.5 m
sqrt(2 * a_p / L_w) ≈ 0.2390 1/s
```

除场景另有说明：

```text
initial_time: t = 0.0 s, step = 0, dt = 0.1 s
vehicle type = CAV
compliance = not_applicable / none
a = 0
vehicle length = 4 m
x_global 是算法坐标；x_plot 只用于 PNG。
```

通用 sanity baseline：

```text
collision = false
near_collision = false
state_machine_inconsistency = false
unexpected_ordinary_lane_change_attempt = false
multiple_commit_for_one_vehicle = false
x_plot_used_in_algorithm_path = false
```

## 3. ScenarioConfig 使用规则

每个场景应落成一个 `ScenarioConfig` 或等价测试配置。本文的每个场景只列执行必需内容：

```text
scenario_id
purpose
test_level / status
module_overrides
setup / initial_vehicles / preloaded state
key numeric derivation
expected_events
expected_sanity_checks
expected_png_features
```

不得在本文复制 `ScenarioConfig`、`TrajectoryRecord`、`EventRecord` 或 `SanityCheckRecord` 的完整字段全集。实现中字段缺口应先修订 `CORMC代码数据结构设计_整理版.md`。

## 4. 场景索引表

| 顺序 | scenario_id | test_level | status | 核心验收 |
| ---: | --- | --- | --- | --- |
| 1 | `MVS-APS-FAIL-EMPTY` | unit | required | APS 候选不足且无 cache 时不伪造 assignment |
| 2 | `MVS-APS-FAIL-CACHE` | unit | required | APS failure 不用无效新结果静默覆盖旧 cache |
| 3 | `MVS-APS-1` | unit | required | APS case 1，`col_CLV=0`，`col_CFV=0` |
| 4 | `MVS-APS-2` | unit | required | APS case 2，Eq.10 给 CFV，`S_CFV=58 m` |
| 5 | `MVS-APS-3` | unit | required | APS case 3，不给 CLV 套 Eq.10 |
| 6 | `MVS-APS-4` | unit | required | APS case 4，Eq.10 给 CFV，`S_CFV=52 m` |
| 7 | `MVS-E2E-1` | smoke | required | APS case 1 -> CMC Eq.53 pass -> merge start -> commit |
| 8 | `MVS-COMMIT-1-lite` | unit | required | 每车每步一次 commit |
| 9 | `MVS-CMC-1` | integration | required | Eq.53 pass，开始合流 |
| 10 | `MVS-CMC-2` | integration | required | Eq.53 fail，继续等待 |
| 11 | `MVS-CUC-1A_override_choice1` | unit | required | override choice 1、lane-change command、same-step overlay |
| 12 | `MVS-CUC-1B_real_utility_probe` | probe | probe | 真实 utility 输入、U1/U2、final choice 可观测 |
| 13 | `MVS-CUC-1C_real_utility_choice1_locked` | integration | deferred | 真实 utility 数值锁定 choice 1 的预留强验收 |
| 14 | `MVS-CUC-2` | integration | required | 目标车道 unsafe，回退 choice 2，消费 Eq.10 |
| 15 | `MVS-CUC-3` | integration | required | non-compliant CHV 不执行协同建议 |
| 16 | `MVS-SAFE-1A_waiting_cap` | unit | required | waiting 状态 boundary cap 进入 planning speed |
| 17 | `MVS-SAFE-1B_executing_cap_lateral_consumption` | integration | required | executing 状态横向轨迹消费 capped speed |
| 18 | `MVS-SAFE-2` | unit | required | cap 不可行 / boundary risk 可记录 |
| 19 | `MVS-ASSIGN-1` | integration | required | assignment invalid 不偷换 actual leader/follower |
| 20 | `MVS-CONFLICT-1A` | unit | required | merging zone MV 优先仲裁 |
| 21 | `MVS-CONFLICT-1B` | unit | required | `T*_MV` 更小者优先仲裁 |
| 22 | `MVS-COMMIT-1-full` | integration | required | 非 APS 周期 cache、active trajectory、每车一次 commit |

## 5. 场景族 A：MVS-APS

### 5.1 APS 公共设置

`MVS-APS-1/2/3/4` 共用 MV：

```text
MV_A:
    role = on_ramp_mv
    lane = on_ramp
    x_global = 6850
    y = -3.5
    v = 20
    merge_state = not_started
```

公共候选：

```text
lane 2 candidates:
    y = 0
    v = 20
    L = 4
```

关键推导：

```text
T*_MV = (6950 - 6850) / 20 = 5 s
candidate window = [6550, 7150]
D*_j = x_j + 20 * 5 - 6950 - 4 = x_j - 6854
D_min_CLV = D_min_CFV = 20 * 1.2 = 24 m
```

### 5.2 `MVS-APS-FAIL-EMPTY`

purpose：候选车辆不足时，APS 不创建新 assignment；无旧 cache 时不得伪造 assignment。

setup：

```text
MV_FAIL_EMPTY:
    role = on_ramp_mv
    lane = on_ramp
    x_global = 6830
    y = -3.5
    v = 20
    merge_state = not_started
    assignment_cache = none

ONLY_LANE2_FAIL:
    role = mainline
    lane = lane_2
    x_global = 6860
    y = 0
    v = 20
```

key numeric derivation：

```text
candidate window = [6530, 7130]
Gap_car_MV = [ONLY_LANE2_FAIL]
n = 1 < 2
=> APS failure
=> reason = insufficient_candidates / candidate_insufficient
```

expected_events：

```text
APS:
    failure = true
    failure_reason = insufficient_candidates
    no_CLV_CFV_assignment_created = true
assignment_cache:
    previous_cache = none
    new_assignment_created = false
cooperative_request:
    none
```

expected_sanity_checks：

```text
assignment_invalid = not_applicable
multiple_commit_for_one_vehicle = false
```

expected_png_features：

```text
aps_failure_marker visible near MV_FAIL_EMPTY
no assignment arrow visible
```

### 5.3 `MVS-APS-FAIL-CACHE`

purpose：已有 cache 时，本次 APS failure 不得用失败结果静默覆盖旧 cache。

setup：沿用 `MVS-APS-FAIL-EMPTY` 的候选不足结构，但 MV id 使用 `MV_FAIL_CACHE`，并预加载：

```text
preloaded_assignments:
    mv_id = MV_FAIL_CACHE
    CLV = OLD_CLV
    CFV = OLD_CFV
    case = case_1
    created_at = -4.0 s
    status_before_this_APS = valid

OLD_CLV:
    lane = lane_2
    x_global = 6900
    y = 0
    v = 20

OLD_CFV:
    lane = lane_2
    x_global = 6780
    y = 0
    v = 20
```

expected_events：

```text
APS:
    failure = true
    failure_reason = insufficient_candidates
assignment_cache:
    previous_cache_exists = true
    invalid_new_assignment_overwrites_existing_cache = false
    post_failure_cache_policy = retained_or_marked_stale_or_marked_invalid
    policy_source = state_interface_spec_or_first_version_engineering_patch
cooperative_request:
    no_new_cooperative_request_from_failed_APS_result
```

expected_sanity_checks：

```text
assignment_cache_overwrite_by_failed_APS = false
```

expected_png_features：

```text
aps_failure_marker visible
old cache may be displayed as retained/stale/invalid, but no new assignment arrow from failed result
```

### 5.4 `MVS-APS-1/2/3/4`

purpose：覆盖 APS case 1-4、`col_CLV / col_CFV`、Eq.10 只作用于 case 2 / case 4 的 CFV。

setup and expected events：

| scenario_id | CLV id / x | CFV id / x | `D*_CLV` | `D*_CFV` | expected APS case | expected `col` | Eq.10 |
| --- | --- | --- | ---: | ---: | --- | --- | --- |
| `MVS-APS-1` | `CLV_APS_1` / 6884 | `CFV_APS_1` / 6824 | +30 | -30 | `case_1` | `col_CLV=0`, `col_CFV=0` | none |
| `MVS-APS-2` | `CLV_APS_2` / 6884 | `CFV_APS_2` / 6844 | +30 | -10 | `case_2` | `col_CLV=0`, `col_CFV=1` | `S_CFV = 58 m` |
| `MVS-APS-3` | `CLV_APS_3` / 6864 | `CFV_APS_3` / 6824 | +10 | -30 | `case_3` | `col_CLV=1`, `col_CFV=0` | none for CLV |
| `MVS-APS-4` | `CLV_APS_4` / 6864 | `CFV_APS_4` / 6844 | +10 | -10 | `case_4` | `col_CLV=1`, `col_CFV=1` | `S_CFV = 52 m` |

key numeric derivation：

```text
MVS-APS-2: S_CFV = 24 + 4 + max(24, 30) = 58 m
MVS-APS-4: S_CFV = 24 + 4 + max(24, 10) = 52 m
```

expected_sanity_checks：

```text
Eq10_applied_to_wrong_vehicle = false
assignment_invalid = false
```

expected_png_features：

```text
APS assignment marker visible: MV_A, CLV, CFV, case id
Eq.10 spacing marker visible only for MVS-APS-2 and MVS-APS-4
```

## 6. 场景族 B：MVS-E2E

### 6.1 `MVS-E2E-1`

purpose：跑通最短主链路：APS case 1 -> no CUC -> CMC Eq.53 pass -> merge start -> commit。

module_overrides：

```text
quasi_static_longitudinal_override = true
```

setup：

```text
MV_E2E:
    role = on_ramp_mv
    lane = on_ramp
    x_global = 6830
    y = -3.5
    v = 20
    merge_state = not_started

CLV_E2E:
    role = mainline
    lane = lane_2
    x_global = 6870
    y = 0
    v = 20

CFV_E2E:
    role = mainline
    lane = lane_2
    x_global = 6800
    y = 0
    v = 20
```

key numeric derivation：

```text
T*_MV = (6950 - 6830) / 20 = 6.0 s
D*_CLV = (6870 + 20*6) - 6950 - 4 = 36 m
D*_CFV = (6800 + 20*6) - 6950 - 4 = -34 m
D_min = 24 m
=> APS case 1
=> col_CLV = 0, col_CFV = 0

At x_MV = 6950 under quasi-static override:
x_CLV = 6990, x_CFV = 6920
h_tilde = 1.2 s
d_MV_CLV = 6990 - 6950 - 4 = 36 m
d_CFV_MV = 6950 - 6920 - 4 = 26 m
threshold = 20 * 1.2 = 24 m
=> Eq.53 pass
```

expected_events：

```text
APS:
    trigger = first_APS
    case = case_1
    CLV = CLV_E2E
    CFV = CFV_E2E
    col_CLV = 0
    col_CFV = 0
cooperative_request:
    none
CMC:
    assignment_valid = true
    eq53_pass = true
    merge_state_transition = not_started_or_waiting_to_executing
commit:
    each_active_vehicle_has_one_final_next_state = true
```

expected_sanity_checks：

```text
collision = false
boundary_violation = false
assignment_invalid = false
multiple_commit_for_one_vehicle = false
```

expected_png_features：

```text
APS assignment marker visible
merge start marker visible at merging zone start
MV trajectory continues from on_ramp toward lane_2
```

## 7. 场景族 C：MVS-COMMIT-lite

### 7.1 `MVS-COMMIT-1-lite`

purpose：在 E2E 后尽早验证同步提交底座，避免后续场景被一车多次提交或中途改写状态污染。

setup：可复用 `MVS-E2E-1` 中 MV 刚进入 CMC 的时间步，或加载最小三车静态状态：

```text
MV_COMMIT_LITE:
    role = on_ramp_mv
    lane = on_ramp
    x_global = 6950
    y = -3.5
    v = 20
    merge_state = waiting_or_not_started

CLV_COMMIT_LITE:
    lane = lane_2
    x_global = 6990
    y = 0
    v = 20

CFV_COMMIT_LITE:
    lane = lane_2
    x_global = 6920
    y = 0
    v = 20
```

expected_events：

```text
commit:
    each_active_vehicle_has_exactly_one_final_next_state = true
    no_module_writes_committed_state_before_commit = true
    command_buffer_and_next_state_buffer_are_separated = true
```

expected_sanity_checks：

```text
multiple_commit_for_one_vehicle = false
state_machine_inconsistency = false
```

expected_png_features：

```text
optional commit marker / trajectory point; no strong PNG requirement beyond base trajectory visibility
```

## 8. 场景族 D：MVS-CMC

### 8.1 `MVS-CMC-1`

purpose：验证 CMC Eq.53 满足时开始合流，并记录 boundary speed cap 非绑定。

setup：

```text
MV_CMC:
    role = on_ramp_mv
    lane = on_ramp
    x_global = 7000
    y = -3.5
    v = 20
    merge_state = not_started
    preloaded_assignment = {CLV_Y, CFV_X, valid}

CLV_Y:
    lane = lane_2
    x_global = 7032
    y = 0
    v = 20

CFV_X:
    lane = lane_2
    x_global = 6972
    y = 0
    v = 20
```

key numeric derivation：

```text
h_tilde = 1.2 * (1 - (2/3) * (7000 - 6950) / 300) = 1.0667 s
d_MV_CLV = 7032 - 7000 - 4 = 28 m
d_CFV_MV = 7000 - 6972 - 4 = 24 m
threshold = 20 * 1.0667 = 21.33 m
=> Eq.53 pass

v_cap = (7250 - 7000 - 2 - 2) * 0.2390 ≈ 58.8 m/s
planning_speed = min(20, 58.8) = 20 m/s
```

expected_events：

```text
CMC:
    branch = CMC
    assignment_valid = true
    h_tilde ≈ 1.0667
    eq53_pass = true
    boundary_speed_cap ≈ 58.8
    cap_binding = false
merge_command:
    transition = not_started_or_waiting_to_executing
    start_y = -3.5
    target_y = 0.0
```

expected_sanity_checks：

```text
assignment_invalid = false
boundary_violation = false
```

expected_png_features：

```text
merge start marker visible
assigned CLV/CFV markers visible
```

### 8.2 `MVS-CMC-2`

purpose：验证 CMC Eq.53 不满足时继续 waiting，不创建横向合流 command。

setup：

```text
MV_CMC:
    role = on_ramp_mv
    lane = on_ramp
    x_global = 7000
    y = -3.5
    v = 20
    merge_state = waiting_or_not_started
    preloaded_assignment = {CLV_Y, CFV_X, valid}

CLV_Y:
    lane = lane_2
    x_global = 7018
    y = 0
    v = 20

CFV_X:
    lane = lane_2
    x_global = 6972
    y = 0
    v = 20
```

key numeric derivation：

```text
h_tilde = 1.0667 s
d_MV_CLV = 7018 - 7000 - 4 = 14 m
d_CFV_MV = 7000 - 6972 - 4 = 24 m
threshold = 21.33 m
14 < 21.33, 24 >= 21.33
=> Eq.53 fail, fail_side = CLV_gap
```

expected_events：

```text
CMC:
    assignment_valid = true
    eq53_pass = false
    fail_side = CLV_gap
    merge_state_remains = waiting_or_not_started
merge_command:
    none
longitudinal_model:
    MV continues on_ramp longitudinal command under CMC waiting semantics
```

expected_sanity_checks：

```text
assignment_invalid = false
boundary_violation = false
```

expected_png_features：

```text
waiting marker visible
no merge start marker visible
```

## 9. 场景族 E：MVS-CUC

### 9.1 `MVS-CUC-1A_override_choice1`

purpose：验证 CUC choice 1、lane 2 -> lane 1 换道状态机和 same-step relation overlay。该场景是状态机单元测试，不用于证明真实 utility 一定满足 `U1 > U2`。

setup：基于 `MVS-APS-2` assignment。

```text
MV_CUC:
    role = on_ramp_mv
    lane = on_ramp
    x_global = 6850
    y = -3.5
    v = 20

CFV_X:
    role = mainline
    lane = lane_2
    x_global = 6844
    y = 0
    v = 20
    APS role = CFV
    col = 1
    Eq10_spacing = 58

CLV_Y:
    lane = lane_2
    x_global = 6884
    y = 0
    v = 20

TLV:
    lane = lane_1
    x_global = 6920
    y = +3.5
    v = 22

TFV:
    lane = lane_1
    x_global = 6750
    y = +3.5
    v = 20
```

module_overrides：

```text
test_harness_overrides:
    U1_gt_U2 = true
```

key numeric derivation：

```text
gap(CFV_X, TLV) = 6920 - 6844 - 4 = 72 m
gap(TFV, CFV_X) = 6844 - 6750 - 4 = 90 m
=> target lane safety = pass
```

expected_events：

```text
CUC:
    active_CV = CFV_X
    source_MV = MV_CUC
    target_lane_safety = pass
    utility_source = test_harness_override
    final_choice = change_to_lane_1
lane_change_command:
    vehicle = CFV_X
    target_lane = lane_1
    lane_change_state_transition = normal_to_executing
same_step_overlay:
    created = true
```

expected_sanity_checks：

```text
state_machine_inconsistency = false
unexpected_ordinary_lane_change_attempt = false
```

expected_png_features：

```text
lane-change start marker visible for CFV_X
same-step overlay / active maneuver marker visible
```

### 9.2 `MVS-CUC-1B_real_utility_probe`

purpose：验证真实 CUC utility 计算链路可观测；不把 choice 1 作为强验收条件。

setup：沿用 `MVS-CUC-1A_override_choice1` 的车辆几何，但关闭 `U1_gt_U2` override。

expected_events：

```text
CUC:
    active_CV = CFV_X
    utility_source = real_CUC
    utility_inputs_logged = true
    U1 = logged_numeric_value
    U2 = logged_numeric_value
    U1_gt_U2 = logged_boolean
    target_lane_safety = logged_pass_or_fail
    final_choice = logged_choice
```

probe pass criteria：

```text
if target_lane_safety = pass:
    final_choice must match real utility comparison
if target_lane_safety = fail:
    final_choice must not start choice 1
implementation failure if:
    U1/U2 or utility inputs missing
    U1 > U2 and safety pass but final_choice != choice_1
    U1 <= U2 but choice_1 starts without override
    safety fail but choice_1 starts
```

expected_sanity_checks：

```text
state_machine_inconsistency = false
```

expected_png_features：

```text
optional: show actual final choice; probe does not require choice_1 marker
```

### 9.3 `MVS-CUC-1C_real_utility_choice1_locked`

purpose：预留 deterministic real utility choice 1 强验收。第一版当前不作为 required smoke；等 Eq.14 / utility 权重和输入完全复核后启用。

setup：

```text
status = deferred
initial_vehicles = to_be_locked_after_real_utility_formula_review
must_use_real_CUC = true
no_test_harness_override = true
```

expected_events：

```text
CUC:
    utility_source = real_CUC
    U1 and U2 reproducible from scenario inputs
    U1_gt_U2 = true by numeric derivation
    target_lane_safety = pass
    final_choice = change_to_lane_1
lane_change_command:
    selected_CV normal_to_executing
```

expected_sanity_checks：

```text
not_executed_in_required_suite_until_locked = true
```

expected_png_features：

```text
when enabled: lane-change start marker visible for selected CV
```

### 9.4 `MVS-CUC-2`

purpose：目标车道 TT 安全不满足时必须回退 choice 2，并让留在 lane 2 的 CFV 消费 Eq.10 spacing。

setup：基于 `MVS-APS-2` assignment。

```text
MV_CUC:
    role = on_ramp_mv
    lane = on_ramp
    x_global = 6850
    y = -3.5
    v = 20

CFV_X:
    role = mainline
    lane = lane_2
    x_global = 6844
    y = 0
    v = 25
    col = 1
    Eq10_spacing = 58

CLV_Y:
    lane = lane_2
    x_global = 6884
    y = 0
    v = 20

TLV:
    lane = lane_1
    x_global = 6853
    y = +3.5
    v = 20

TFV:
    lane = lane_1
    x_global = 6780
    y = +3.5
    v = 20
```

key numeric derivation：

```text
gap(CFV_X, TLV) = 6853 - 6844 - 4 = 5 m
relative speed = 25 - 20 = 5 m/s
TT ≈ 5 / 5 = 1.0 s < TT_min = 1.5 s
=> target lane unsafe
=> final choice = stay_lane_2 / choice_2
```

expected_events：

```text
CUC:
    target_lane_safety = fail
    fallback_reason = target_lane_TT_unsafe
    final_choice = stay_lane_2
lane_change_command:
    none
longitudinal_model:
    vehicle = CFV_X
    desired_spacing_source = Eq10
    desired_spacing_override = 58 m
```

expected_sanity_checks：

```text
unexpected_lane_change_when_target_lane_unsafe = false
Eq10_applied_to_wrong_vehicle = false
```

expected_png_features：

```text
unsafe target-lane marker visible
no lane-change start marker for CFV_X
Eq.10 spacing marker visible on lane_2
```

### 9.5 `MVS-CUC-3`

purpose：non-compliant CHV 不执行协同建议。

setup：沿用 `MVS-CUC-2`，但：

```text
CFV_X:
    type = CHV
    compliance = non_compliant
```

expected_events：

```text
compliance:
    active_CV = CFV_X
    vehicle_type = CHV
    compliance = non_compliant
    execute_suggestion = false
CUC:
    suggestion_ignored_or_not_executed = true
longitudinal_model:
    desired_spacing_override = none
    model = ordinary_IDM
```

expected_sanity_checks：

```text
unexpected_CUC_execution_by_non_compliant_CHV = false
```

expected_png_features：

```text
optional non-compliant marker visible
no cooperative lane-change or Eq.10 execution marker for CFV_X
```

## 10. 场景族 F：MVS-SAFE

### 10.1 `MVS-SAFE-1A_waiting_cap`

purpose：waiting 状态下，boundary speed cap 进入纵向 planning speed 合成；不检查横向轨迹消费。

setup：

```text
MV_SAFE_WAIT:
    role = on_ramp_mv
    lane = on_ramp
    x_global = 7235
    y = -3.5
    v = 16
    merge_state = waiting
```

key numeric derivation：

```text
h_tilde = 1.2 * (1 - (2/3) * (7235 - 6950) / 300) = 0.44 s
v_cap = (7250 - 7235 - 2 - 2) * 0.2390 = 2.63 m/s
cap_feasible = true
planning_speed = min(16, 2.63) = 2.63 m/s
```

expected_events：

```text
speed_cap:
    boundary_speed_cap ≈ 2.63
    cap_feasible = true
    candidate_speed = 16
    planning_speed ≈ 2.63
    most_conservative_source = boundary_speed_cap
lateral_trajectory:
    none_or_not_applicable
    reason = merge_state_waiting
```

expected_sanity_checks：

```text
boundary_violation = false for this step
```

expected_png_features：

```text
MV_SAFE_WAIT near on-ramp downstream boundary
boundary cap marker visible
no merge executing progress marker
```

### 10.2 `MVS-SAFE-1B_executing_cap_lateral_consumption`

purpose：executing 状态下，boundary speed cap 不仅约束纵向 planning speed，也被横向正弦轨迹更新消费。

setup：同 `MVS-SAFE-1A_waiting_cap`，但：

```text
MV_SAFE_EXEC:
    role = on_ramp_mv
    lane = on_ramp
    x_global = 7235
    y = -3.5
    v = 16
    merge_state = executing

preloaded_maneuver_trajectory_states:
    vehicle = MV_SAFE_EXEC
    maneuver = merge
    start_time = t - 0.8 s
    start_y = -3.5
    target_y = 0.0
    progress = 0.30
```

key numeric derivation：

```text
boundary_speed_cap ≈ 2.63 m/s
planning_speed ≈ 2.63 m/s
```

expected_events：

```text
speed_cap:
    boundary_speed_cap ≈ 2.63
    cap_feasible = true
    candidate_speed = 16
    planning_speed ≈ 2.63
lateral_trajectory:
    merge_state = executing
    trajectory_consumed_speed ≈ 2.63
    trajectory_consumed_speed_source = planning_speed_after_boundary_cap
```

expected_sanity_checks：

```text
boundary_violation = false for this step
state_machine_inconsistency = false
```

expected_png_features：

```text
MV_SAFE_EXEC near downstream boundary
y between -3.5 and 0
boundary cap marker and merge progress marker both visible
```

### 10.3 `MVS-SAFE-2`

purpose：boundary cap 不可行或越界风险必须可定位；不在本文拍板保守策略。

setup：

```text
MV_SAFE_FAIL:
    role = on_ramp_mv
    lane = on_ramp
    x_global = 7248
    y = -3.5
    v = 16
    merge_state = waiting_or_executing
```

key numeric derivation：

```text
margin = 7250 - 7248 - 2 - 2 = -2 m
v_cap = -2 * 0.2390 = -0.478 m/s
=> cap_feasible = false
```

expected_events：

```text
speed_cap:
    cap_feasible = false
    cap_value < 0
    reason = boundary_speed_cap_infeasible
engineering_patch:
    boundary_speed_cap_infeasible recorded
```

expected_sanity_checks：

```text
if MV remains unmerged and x_MV > x_ramp_end after commit:
    boundary_violation = true
else:
    boundary_risk_warning = true
```

expected_png_features：

```text
boundary warning / infeasible cap marker visible
if post-commit beyond x_ramp_end and unmerged: boundary violation marker visible
vehicle must not be drawn as successfully merged unless merge_state actually reaches merged
```

## 11. 场景族 G：MVS-ASSIGN

### 11.1 `MVS-ASSIGN-1`

purpose：CMC 必须使用 APS assignment 中的 CLV / CFV 并做有效性检查，不得用本步 actual lane 2 leader/follower 偷换 assignment。

setup：

```text
MV_ASSIGN:
    role = on_ramp_mv
    lane = on_ramp
    x_global = 6960
    y = -3.5
    v = 16
    merge_state = waiting
    preloaded_assignment = {CLV_ASSIGN, CFV_ASSIGN}

CLV_ASSIGN:
    role = mainline
    lane = lane_2
    x_global = 7010
    y = 0
    v = 18

CFV_ASSIGN:
    role = mainline
    lane = lane_1
    x_global = 6930
    y = +3.5
    v = 18
    note = assigned CFV has moved out of lane_2
```

key numeric derivation：

```text
assigned CFV lane = lane_1
expected assigned CFV lane = lane_2
=> assignment invalid
```

expected_events：

```text
assignment_invalid:
    MV = MV_ASSIGN
    invalid_vehicle = CFV_ASSIGN
    reason = cfv_not_lane_2
    source = first_version_engineering_patch
CMC:
    assignment_valid = false
    Eq53_evaluated = false_or_skipped
    merge_state_remains = waiting
    merge_command_created = false
```

expected_sanity_checks：

```text
assignment_invalid = warning_or_fail_as_configured
no_same_step_actual_leader_follower_replacement = true
```

expected_png_features：

```text
MV_ASSIGN remains waiting
CFV_ASSIGN marked as invalid assignment in lane_1
no replacement assignment arrow to actual lane_2 follower
```

## 12. 场景族 H：MVS-CONFLICT

统一仲裁优先级：

```text
1. 已在 merging zone 的 MV 优先。
2. 若都未在 merging zone，则 T*_MV 更小者优先。
3. 若仍相同，则距离 x0_m_global 更近者优先。
```

### 12.1 `MVS-CONFLICT-1A`

purpose：Step 5 请求汇总 / 冲突仲裁单元测试，验证 merging zone MV 优先。该场景可直接加载 effective assignment；不验证 APS 端到端计算。

setup：

```text
MV_A:
    x_global = 6960
    zone = merging_zone
    T*_MV = 0
    effective_assignment:
        requested_CV = CV_X
        role = CFV
        col = 1

MV_B:
    x_global = 6890
    zone = upstream_of_merging_zone
    v = 20
    T*_MV = (6950 - 6890) / 20 = 3 s
    effective_assignment:
        requested_CV = CV_X
        role = CLV_or_CFV
        col = 1

CV_X:
    lane = lane_2
    active_vehicle = true
```

expected_events：

```text
conflict_resolution:
    requested_CV = CV_X
    requesting_MVs = [MV_A, MV_B]
    winner = MV_A
    loser = MV_B
    priority_reason = MV_in_merging_zone
    source = first_version_engineering_patch
cooperative_request:
    CV_X_receives_only_one_active_request_from = MV_A
```

expected_sanity_checks：

```text
conflicting_commands_to_same_CV = false
```

expected_png_features：

```text
conflict marker groups MV_A and MV_B requests
only winner command from MV_A visible as effective
loser MV_B request visible as suppressed or not effective
```

### 12.2 `MVS-CONFLICT-1B`

purpose：两个 MV 都未进 merging zone 时，验证 `T*_MV` 更小者优先。

setup：

```text
MV_G1:
    role = on_ramp_mv
    lane = on_ramp
    x_global = 6840
    y = -3.5
    v = 20
    merge_state = not_started

MV_G2:
    role = on_ramp_mv
    lane = on_ramp
    x_global = 6830
    y = -3.5
    v = 20
    merge_state = not_started

SHARED_CLV_G:
    role = mainline
    lane = lane_2
    x_global = 6855
    y = 0
    v = 20

CFV_G:
    role = mainline
    lane = lane_2
    x_global = 6790
    y = 0
    v = 20
```

key numeric derivation：

```text
MV_G1:
    T*_MV = 5.5 s
    SHARED_CLV_G D* = 6855 + 20*5.5 - 6950 - 4 = 11 m
    CFV_G D* = 6790 + 20*5.5 - 6950 - 4 = -54 m
    => case_3, requested_CV = SHARED_CLV_G

MV_G2:
    T*_MV = 6.0 s
    SHARED_CLV_G D* = 6855 + 20*6.0 - 6950 - 4 = 21 m
    CFV_G D* = 6790 + 20*6.0 - 6950 - 4 = -44 m
    => case_3, requested_CV = SHARED_CLV_G

Both upstream:
    5.5 < 6.0
    => winner = MV_G1
```

expected_events：

```text
conflict_resolution:
    requested_CV = SHARED_CLV_G
    requesting_MVs = [MV_G1, MV_G2]
    winner = MV_G1
    loser = MV_G2
    priority_basis = smaller_T_star_MV
    source = first_version_engineering_patch
loser_state:
    MV_G2 result = waiting_or_conflict
    no_conflicting_command_issued_to_SHARED_CLV_G = true
```

expected_sanity_checks：

```text
conflicting_commands_to_same_CV = false
```

expected_png_features：

```text
both MVs upstream of x0_m_global
SHARED_CLV_G shows only one effective cooperative command from MV_G1
MV_G2 request shown as loser / suppressed
```

## 13. 场景族 I：MVS-COMMIT-full

### 13.1 `MVS-COMMIT-1-full`

purpose：验证非 APS 周期沿用 assignment cache、active lane-change 不重新执行 CUC、merge executing 不重新判断开始合流、active trajectory 最小字段可加载、每车每步只提交一次。

setup：

```text
initial_time:
    t = 2.0 s
    step = 20
    dt = 0.1 s

preloaded_state_machine_states:
    last_APS_time(MV_CACHE) = 0.0 s
    T_APS = 5.0 s

MV_CACHE:
    role = on_ramp_mv
    lane = on_ramp
    x_global = 6890
    y = -3.5
    v = 20
    merge_state = not_started
    x_global < x0_m_global

preloaded_assignments:
    mv_id = MV_CACHE
    CLV = CLV_CACHE
    CFV = CFV_CACHE
    case = case_1_or_case_2
    created_at = 0.0 s
    status = valid
    valid_until_next_APS = true

CLV_CACHE / CFV_CACHE:
    lane = lane_2
    active_vehicle = true
    positions must be self-consistent with cached assignment
    exact positions are not strong checks for this scenario

CV_ACTIVE_LC:
    role = mainline
    lane = lane_2
    x_global = 6900
    y = 1.2
    v = 20
    lane_change_state = executing

preloaded_maneuver_trajectory_states:
    vehicle = CV_ACTIVE_LC
    maneuver = lane_change
    start_time = 1.0 s
    start_x_global = 6880
    start_y = 0.0
    target_y = +3.5
    target_lane = lane_1
    progress = 0.35

MV_ACTIVE_MERGE:
    role = on_ramp_mv
    lane = on_ramp
    x_global = 7010
    y = -2.1
    v = 18
    merge_state = executing

preloaded_maneuver_trajectory_states:
    vehicle = MV_ACTIVE_MERGE
    maneuver = merge
    start_time = 1.2 s
    start_x_global = 6990
    start_y = -3.5
    target_y = 0.0
    target_lane = lane_2
    progress = 0.40
    assigned_CLV = CLV_MERGE
    assigned_CFV = CFV_MERGE

CLV_MERGE / CFV_MERGE:
    lane = lane_2
    active_vehicle = true
    positions must be self-consistent with active merge relation
    exact positions are not strong checks for this scenario
```

expected_events：

```text
APS:
    trigger = reuse_cache
    no_new_APS_calculation = true
    effective_assignment_this_step = cache
CUC:
    CV_ACTIVE_LC skip_CUC = true
    reason = lane_change_state_executing
CMC:
    MV_ACTIVE_MERGE continues_existing_merge_trajectory = true
    no_new_Eq53_start_decision = true
commit:
    each_vehicle_has_exactly_one_final_next_state = true
    multiple_commit_sanity_check = pass
```

expected_sanity_checks：

```text
multiple_commit_for_one_vehicle = false
state_machine_inconsistency = false
lane_change_and_merge_both_executing_same_vehicle = false
```

expected_png_features：

```text
active lane-change progress marker visible for CV_ACTIVE_LC
active merge progress marker visible for MV_ACTIVE_MERGE
cache reuse marker visible for MV_CACHE
no duplicate commit marker for any vehicle
```

## 14. 推荐执行顺序

```text
1. MVS-APS-FAIL-EMPTY / MVS-APS-FAIL-CACHE
2. MVS-APS-1 / MVS-APS-2 / MVS-APS-3 / MVS-APS-4
3. MVS-E2E-1
4. MVS-COMMIT-1-lite
5. MVS-CMC-1 / MVS-CMC-2
6. MVS-CUC-1A_override_choice1 / MVS-CUC-1B_real_utility_probe / MVS-CUC-1C_real_utility_choice1_locked / MVS-CUC-2 / MVS-CUC-3
7. MVS-SAFE-1A_waiting_cap / MVS-SAFE-1B_executing_cap_lateral_consumption / MVS-SAFE-2
8. MVS-ASSIGN-1
9. MVS-CONFLICT-1A / MVS-CONFLICT-1B
10. MVS-COMMIT-1-full
```

执行注意：

```text
MVS-CUC-1B_real_utility_probe 是 probe，不应作为 strong acceptance。
MVS-CUC-1C_real_utility_choice1_locked 是 deferred，等 utility 公式完全复核后再纳入强验收。
MVS-CONFLICT-1A 是 conflict arbitration 单元测试，不是 APS 端到端测试。
```

## 15. 实现完成判据

```text
1. 上表 required 场景均有可加载 ScenarioConfig。
2. 每个 required 场景均能产生 expected_events 与 expected_sanity_checks。
3. 需要人工复核的场景均能在 PNG 中看到 expected_png_features。
4. probe / deferred 场景不会阻塞第一版 required smoke suite。
5. 所有场景使用 x_global 作为算法坐标；x_plot 只在 PNG renderer 派生。
6. 本文没有重新定义通用 record schema 或 ScenarioConfig schema。
7. 工程补丁相关场景均在 event 中记录 source / reason。
