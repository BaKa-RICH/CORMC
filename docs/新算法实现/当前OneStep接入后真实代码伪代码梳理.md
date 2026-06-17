# 当前 OneStep 接入后真实代码伪代码梳理

本文只依赖当前真实代码梳理，不把旧文档当作当前事实来源。

本次梳理覆盖的真实代码主链主要是：

- `cormc/ramp_merge_algorithm/engine.py`
- `cormc/ramp_merge_algorithm/planner.py`
- `cormc/ramp_merge_algorithm/motion.py`
- `cormc/ramp_merge_algorithm/state.py`
- `cormc/ramp_merge_algorithm/gaps.py`
- `cormc/ramp_merge_algorithm/safety.py`
- `cormc/ramp_merge_algorithm/onestep_stage2_planner.py`
- `cormc/ramp_merge_algorithm/onestep_stage2_adapter.py`
- `cormc/ramp_merge_algorithm/onestep_stage1_runner.py`
- `cormc/ramp_merge_algorithm/onestep_stage2_runner.py`
- `cormc/ramp_merge_algorithm/onestep_stage1_analysis.py`
- `cormc/ramp_merge_algorithm/onestep_stage2_analysis.py`
- `cormc/ramp_merge_algorithm/validation.py`
- `cormc/ramp_merge_onestep_scenarios.py`

---

## 一、先说结论：当前代码里实际上有两套执行壳

当前 `RampMergeEngine` 并不是“旧算法完全删掉、OneStep 完全替换”，而是保留了两条分支：

```text
algorithm_variant == "legacy_batch_c"
algorithm_variant == "onestep_stage2"
```

其中：

- `legacy_batch_c` 仍然保留控制区选 gap、合流区锁 gap、旧 planned_trajectory 推进这一整套壳。
- `onestep_stage2` 只替换了“触发规划后的 gap 评估与纵向控制执行”这一段。
- Step0-3 预处理、runtime 刷新、安全检测、触发判断、事件导出、commit/time advance 这些外层骨架，两条分支共用。

所以，当前真实代码更准确的描述是：

```text
一个共用的 ramp_merge 执行框架
    + 一个 legacy 控制区/合流区规划分支
    + 一个 OneStep stage2 触发重规划 + bundle 执行分支
```

---

## 二、当前真实运行时状态

### 1. 规划器公共状态

```text
PlannerState:
    T_plan = 2.0
    next_plan_time
    last_trigger_reason
```

### 2. 每个 MV 的状态

```text
MVPlanState:
    mv_id
    zone_state ∈ {
        outside_control_zone,
        control_zone,
        merge_zone,
        out_of_scene
    }
    merge_state
    current_plan_gap
    active_bundle_id
    locked_gap
    planned_trajectory_id
    last_plan_step
    last_plan_t
```

说明：

- `merge_state`、`locked_gap`、`planned_trajectory_id` 主要仍服务于 legacy 分支。
- `active_bundle_id`、`current_plan_gap` 是当前 OneStep stage2 真实在用的关键字段。

### 3. OneStep stage2 的三车 bundle

```text
OneStepPlanBundle:
    bundle_id
    mv_id
    start_step
    start_t
    trigger_reason
    origin_x_global

    selected_gap
    selected_rear_vehicle_id
    selected_front_vehicle_id
    selected_vehicle_ids = (mv, rear, front)
    lane_2_vehicle_order

    local_scenario
    best_gap
    best_score
    gap_rows

    boundary_state_by_vehicle_id:
        记录 trigger 当下 mv / rear / front 三车真实 x/v/a

    required_longitudinal_gap_m
    merge_point_x_global
```

### 4. 当前受 bundle 控制的车辆表

```text
OneStepControlledVehicleState:
    vehicle_id
    owner_mv_id
    bundle_id
    role ∈ {mv, rear, front}
    controlled_since_step
```

### 5. 总 runtime

```text
RampMergeRuntimeState:
    planner_state
    mv_plan_states
    onestep_plan_bundles
    controlled_vehicle_states
    planned_trajectories
    danger_vehicle_ids
    last_gap_snapshot
    version
```

注意：

- 这个 runtime 结构本身已经“形式上支持多个 MV”，因为 `mv_plan_states` 是一个映射。
- 但 OneStep stage2 规划器当前明确限制为“只支持 1 个活动 MV”，后面会单独说明。

---

## 三、场景桥接与验证入口的真实结构

### 1. 场景桥接层

当前 `cormc/ramp_merge_onestep_scenarios.py` 已经不是只支持 S07，而是统一注册了：

```text
RM-ONESTEP-S05-PLAN-STEP0
RM-ONESTEP-S05-ROLLING-ENTRY
RM-ONESTEP-S07-PLAN-STEP0
RM-ONESTEP-S07-ROLLING-ENTRY
```

每个 case spec 当前真实包含：

```text
one_step_case_id
mv_id
lane_2_vehicle_ids
mainline_x_local
plan_step0_mv_x_global
rolling_entry_mv_x_global
plan_step0_mainline_origin_x_global
rolling_entry_mainline_origin_x_global
stage1_default_max_steps
stage2_default_max_steps
stage1_expectations
```

### 2. stage1 runner 的真实作用

`onestep_stage1_runner.py` 不是运行 OneStep stage2 纵向接管，而是做“桥接场景首帧/首次触发检查”。

```text
build_initial_onestep_stage1_state(scenario_id):
    config = load_ramp_merge_onestep_scenario(scenario_id)
    workspace = build_prefreeze_workspace_from_scenario(config)
    frozen = freeze_simulation_state(workspace)
    runtime = initialize_runtime_state(frozen)
    return replace(frozen, ramp_merge_runtime=runtime), config

run_onestep_stage1_history(...):
    state = 初始 frozen state
    engine = RampMergeEngine(config, algorithm_variant="legacy_batch_c")
    连续调用 engine.advance_one_step()
    收集 events / history / sanity checks
    构造 stage1 summary
```

也就是说：

- stage1 入口仍走默认 `legacy_batch_c`
- 作用是验证 `plan-step0` / `rolling-entry` 的桥接几何、首个 trigger 时机、local frame 是否对齐原始 OneStep 场景
- 它不是当前 OneStep stage2 纵向滚动控制的执行入口

### 3. stage2 runner 的真实作用

`onestep_stage2_runner.py` 才是当前 OneStep 滚动重规划验证入口：

```text
build_initial_onestep_stage2_state(scenario_id):
    与 stage1 相同，先桥接 scenario，再 freeze，再 initialize_runtime_state

run_onestep_stage2_history(...):
    engine = RampMergeEngine(config, algorithm_variant="onestep_stage2")
    连续调用 engine.advance_one_step()
    收集完整 history / actual_events
    从 events 中提取：
        first_trigger_plan_summary
        first_trigger_gap_rows
        trigger_plan_summaries
        longitudinal_completion_event
        final_vehicle_states
```

---

## 四、单步主流程伪代码：真实的 `RampMergeEngine.advance_one_step`

下面这段最接近当前真实代码主入口。

```text
advance_one_step(state):

    scenario_id = scenario_config["scenario_id"] or state.scenario_config_ref or "unknown"

    --------------------------------------------------
    0. 运行 step0-3 外层预处理
    --------------------------------------------------
    step0_3 = _run_step0_to_3_from_state(state, scenario_config, geometry)
    frozen = step0_3.state

    --------------------------------------------------
    1. 刷新 runtime
    --------------------------------------------------
    previous_runtime = frozen.ramp_merge_runtime
    refreshed_runtime = refresh_runtime_state(previous_runtime, frozen, geometry)

    --------------------------------------------------
    2. 安全检测
    --------------------------------------------------
    safety_result = run_safety_check(frozen)

    --------------------------------------------------
    3. 检测是否有 MV 进入控制区
    --------------------------------------------------
    entry_vehicle_ids = detect_entry_vehicle_ids(previous_runtime, refreshed_runtime)
    entry_plan_trigger = detect_entry_plan_trigger(previous_runtime, refreshed_runtime)

    --------------------------------------------------
    4. 触发判断
    --------------------------------------------------
    trigger_decision = decide_trigger_plan(
        frozen,
        refreshed_runtime.planner_state,
        entry_plan_trigger,
        safety_result.safety_alert,
        entry_vehicle_ids,
    )

    runtime = replace(
        refreshed_runtime,
        planner_state = trigger_decision.planner_state,
        danger_vehicle_ids = safety_result.danger_vehicle_ids,
    )

    --------------------------------------------------
    5. 如果本步触发规划，先识别 lane_2 gaps
    --------------------------------------------------
    gap_snapshot = None

    if trigger_decision.trigger_plan:
        gap_snapshot = identify_and_number_gaps(
            frozen,
            safety_result.danger_vehicle_ids,
        )
        runtime.last_gap_snapshot = gap_snapshot

        if algorithm_variant == "onestep_stage2":
            control_plan = plan_stage2_for_trigger(
                frozen,
                runtime,
                gap_snapshot,
                trigger_decision,
            )
        else:
            control_plan = plan_control_zone_gaps(
                frozen,
                runtime,
                gap_snapshot,
            )

        runtime = control_plan.runtime
    else:
        control_plan = None

    --------------------------------------------------
    6. merge-zone 分支
    --------------------------------------------------
    if algorithm_variant == "onestep_stage2":
        merge_plan = None
    else:
        merge_plan = lock_merge_zone_gaps(
            frozen,
            runtime,
            geometry,
        )
        runtime = merge_plan.runtime

    --------------------------------------------------
    7. 生成本步运动候选
    --------------------------------------------------
    motion_outputs = build_motion_outputs(
        frozen,
        runtime,
        geometry,
        algorithm_variant,
    )
    runtime = motion_outputs.runtime

    --------------------------------------------------
    8. commit 与时间推进
    --------------------------------------------------
    command_buffer = CommandBuffer(step=frozen.step, t=frozen.t)
    next_state_buffer = NextStateBuffer(
        step=frozen.step,
        t=frozen.t,
        candidate_kinematics=motion_outputs.candidate_kinematics,
        candidate_lane_state=motion_outputs.candidate_lane_state,
        candidate_state_transitions=motion_outputs.candidate_state_transitions,
        ramp_merge_runtime=runtime,
    )

    commit = commit_step(...)
    time_advance = advance_time_after_commit_and_integration(...)

    --------------------------------------------------
    9. 导出事件
    --------------------------------------------------
    actual_events = [
        step0_3 events,
        runtime_state_event,
        zone_state_event,
        safety_event,
        trigger_event,
        optional gap_snapshot_event,

        if control_plan exists:
            if algorithm_variant == "onestep_stage2":
                emit onestep_stage2_plan_event
                emit onestep_stage2_gap_eval_event
            else:
                emit legacy gap_selection_event
                emit legacy trajectory_event

        if merge_plan exists:
            emit gap_lock_event
            emit merge_check_event
            emit legacy trajectory_event

        emit default_motion_event

        for motion_outputs.motion_events:
            if algorithm_variant == "onestep_stage2"
               and event.longitudinal_completed == True:
                emit onestep_stage2_longitudinal_completion_event
            elif event.merge_completed == True:
                emit merge_completion_event
            else:
                emit trajectory_event
    ]

    return RampMergeStepResult(...)
```

### 这里有几个必须明确的真实点

#### 1. OneStep 分支并没有替换整个主循环

它只是替换了：

```text
触发后如何规划
以及规划后如何生成运动候选
```

外层：

- freeze
- runtime 刷新
- safety
- trigger
- commit
- time advance
- events

都还是统一壳。

#### 2. OneStep 分支里明确跳过了 legacy merge-zone lock gap 流程

```text
if algorithm_variant == "onestep_stage2":
    merge_plan = None
```

所以当前 OneStep stage2 路径根本不会调用：

- `lock_merge_zone_gaps`
- `run_simplified_merge_check`
- `build_merge_execution_trajectory`

#### 3. 当前 trigger 后仍然会先做 `identify_and_number_gaps`

但这个 `gap_snapshot` 在 OneStep stage2 路径里的真实作用很有限：

- 用于记录本步 lane 2 gap 事件
- 用于给 `selected_gap` 填 `snapshot_step` / `snapshot_t`

它不是 OneStep 打分时真正使用的 gap 评估输入。

真正喂给 OneStep 的，是后面按真实三车状态重建的 local frame。

---

## 五、共享触发逻辑：真实的 `decide_trigger_plan`

当前触发判断非常具体，优先级也已经写死。

```text
decide_trigger_plan(state, planner_state, entry_plan_trigger, safety_alert, entry_vehicle_ids):

    periodic_due = (state.t + 1e-9) >= planner_state.next_plan_time

    active_reasons = []
    if periodic_due:
        active_reasons.append("periodic")
    if safety_alert:
        active_reasons.append("safety_alert")
    if entry_plan_trigger:
        active_reasons.append("MV_enter_control_zone")

    if periodic_due:
        trigger_reason = "periodic"
    elif safety_alert:
        trigger_reason = "safety_alert"
    elif entry_plan_trigger:
        trigger_reason = "MV_enter_control_zone"
    else:
        trigger_reason = "none"

    trigger_plan = (trigger_reason != "none")

    if trigger_plan and trigger_reason == "periodic":
        next_planner_state.next_plan_time = state.t + planner_state.T_plan
        next_planner_state.last_trigger_reason = "periodic"
    elif trigger_plan:
        next_planner_state.last_trigger_reason = trigger_reason
        next_plan_time 保持不变
    else:
        planner_state 保持不变

    return TriggerDecision(...)
```

### 当前真实语义

#### 1. 触发优先级不是并列，而是：

```text
periodic > safety_alert > MV_enter_control_zone
```

也就是说：

- 如果某一帧同时满足周期触发和 entry trigger，最终 `trigger_reason` 会记成 `periodic`
- `active_trigger_reasons` 会保留所有满足的原因

#### 2. 只有周期触发会推进 `next_plan_time`

这意味着：

- `safety_alert` 触发
- `MV_enter_control_zone` 触发

都会触发一次规划，但不会顺手把周期时钟往后推。

这是当前代码的真实行为，不是推测。

---

## 六、安全检测和 gap 快照：当前真实实现

### 1. 安全检测

当前 `run_safety_check` 还是一个比较临时、简单的 lane-wise 检测：

```text
run_safety_check(state):

    按 physical_lane 对车辆分组

    for each lane:
        按 x_global 排序
        对相邻 rear/front:
            bumper_gap = front.x - rear.x - rear.length
            closing_speed = rear.v - front.v

            if bumper_gap <= min_gap_m:
                标为 danger
            elif closing_speed > 0 and bumper_gap / closing_speed <= ttc_threshold_s:
                标为 danger

    return SafetyCheckResult(
        safety_alert = danger_ids 非空,
        danger_vehicle_ids,
        danger_pairs,
    )
```

当前真实含义是：

- 它会产生 `danger_vehicle_ids`
- 这些 danger 车辆在本轮 gap 快照里会被视为“有效不可控”
- 但当前代码并不会把 `safety_state` 真写回 vehicle state 对象中，真实落点是 runtime event / result

### 2. 有效可控性

```text
effective_controllable(vehicle_id):
    return (
        is_base_controllable(vehicle_spec)
        and vehicle_id not in danger_vehicle_ids
    )
```

### 3. gap 快照

当前 `identify_and_number_gaps` 只看 `lane_2`：

```text
identify_and_number_gaps(state, danger_vehicle_ids):

    lane_2_vehicle_ids = 所有 active 且 physical_lane == lane_2 的车辆
    按 x_global 排序

    for 每一对相邻 rear / front:
        根据 effective_controllable(front/rear) 决定：
            effective_control_type ∈ {
                both_controllable,
                front_controllable,
                rear_controllable,
                none_controllable
            }

        生成 GapCandidate:
            gap_id = f"gap:{step}:{index}"
            front_vehicle_id
            rear_vehicle_id
            front_x_global
            rear_x_global
            bumper_gap_m
            effective_control_type
```

注意：

- 这里生成的是 runtime/legacy 用的 `gap:{step}:{index}` 风格 id
- OneStep 内核自己的 `best_gap.gap_id` 是 `gap1/gap2/...`
- 当前代码里同时存在这两套 gap 表示法

---

## 七、OneStep stage2 触发规划：当前真实伪代码

这部分对应 `onestep_stage2_planner.py` 和 `onestep_stage2_adapter.py`。

### 1. 顶层触发规划

```text
plan_stage2_for_trigger(state, runtime, gap_snapshot, trigger_decision):

    mv_id = _resolve_single_active_mv_id(state, runtime)
    mv_state = runtime.mv_plan_states[mv_id]

    if mv_state.zone_state not in {control_zone, merge_zone}:
        return PlanningResult(runtime=runtime)

    evaluation_result = evaluate_stage2_one_step(state, mv_id)
    evaluation = evaluation_result.evaluation

    if evaluation.best_gap is None or evaluation.best_score is None:
        raise ValueError("stage2 trigger planning requires a solved OneStep evaluation")

    selected_rear_vehicle_id, selected_front_vehicle_id =
        resolve_best_gap_vehicle_ids(evaluation_result.local_frame, evaluation)

    selected_gap = GapRef(
        gap_id = evaluation.best_gap.gap_id,
        index = evaluation.best_gap.index,
        front_vehicle_id = selected_front_vehicle_id,
        rear_vehicle_id = selected_rear_vehicle_id,
        snapshot_step = gap_snapshot.step,
        snapshot_t = gap_snapshot.t,
    )

    bundle_id = f"onestep_stage2:{state.step}:{mv_id}"
    merge_point_x_global = origin_x_global + evaluation.best_score.p_m

    algorithm = build_stage2_algorithm_config()
    required_longitudinal_gap_m = (2 * algorithm.D_h + algorithm.l_m) / 2

    boundary_state_by_vehicle_id = {
        mv_id:    当前真实 x/v/a,
        rear_id:  当前真实 x/v/a,
        front_id: 当前真实 x/v/a,
    }

    bundle = OneStepPlanBundle(
        mv_id = mv_id,
        trigger_reason = trigger_decision.trigger_reason,
        origin_x_global = 当前 mv 真值 x_global,
        selected_gap = selected_gap,
        selected_rear_vehicle_id = rear_id,
        selected_front_vehicle_id = front_id,
        selected_vehicle_ids = (mv_id, rear_id, front_id),
        lane_2_vehicle_order = local_frame 中的 lane 2 顺序,
        local_scenario = 当前真实状态重建得到的 OneStep 场景,
        best_gap = evaluation.best_gap,
        best_score = evaluation.best_score,
        boundary_state_by_vehicle_id = 三车 trigger 真值边界状态,
        required_longitudinal_gap_m = required_longitudinal_gap_m,
        gap_rows = evaluation.gap_rows,
        merge_point_x_global = merge_point_x_global,
    )

    next_runtime = _replace_owner_bundle(runtime, mv_id, bundle)

    生成 plan_record
    生成 gap_eval_records

    return PlanningResult(
        runtime = next_runtime,
        gap_selection_records = (plan_record,),
        trajectory_records = gap_eval_records,
    )
```

### 2. 当前 local frame 的真实构造方式

```text
build_stage2_local_frame(state, mv_id):

    origin_x_global = state.vehicle_states[mv_id].x_global

    lane_2_vehicle_order =
        所有 active 且 physical_lane == lane_2 的车辆
        按 x_global 升序排序

    lane_2_vehicle_x_local_by_id[vehicle_id] =
        vehicle.x_global - origin_x_global

    gap_intervals_local =
        相邻 lane_2 车辆 local x 的区间

    gap_centers_local =
        每个区间中心

    return {
        mv_id,
        origin_x_global,
        lane_2_vehicle_order,
        lane_2_vehicle_x_local_by_id,
        gap_intervals_local,
        gap_centers_local,
    }
```

### 3. 当前 local scenario 的真实构造方式

```text
build_stage2_local_scenario(state, mv_id):

    local_frame = build_stage2_local_frame(...)

    x_targets =
        按 lane_2_vehicle_order 取当前真实 x_local

    return ScenarioConfig(
        x_targets = 当前真实 lane_2 相对位置,
        x_m0 = 0.0,
        v_ref = 20.0,
        v_max = 30.0,
        v_min = 0.0,
        a_max = 3.0,
        a_min = -4.0,
        T = 20.0,
    )
```

### 4. 当前 OneStep stage2 的核心语义

```text
每次 trigger 都重算
且重算时拿的是 trigger 当下真实状态
```

这里的“真实状态”至少包括：

- MV 当前真实 `x/v/a`
- 当前 lane 2 所有车真实 `x_global`
- 当前 gap 区间与 gap center
- 由此重建出来的当前 local frame 与当前 local scenario

### 5. 当前明确的单 MV 限制

```text
_resolve_single_active_mv_id(state, runtime):
    active_mv_ids = 所有 runtime.mv_plan_states 中且当前仍 active 的 mv
    if len(active_mv_ids) != 1:
        raise ValueError("stage2 currently supports exactly one MV, got ...")
```

这不是“暂时没测多 MV”，而是代码里显式报错。

### 6. 当前 bundle 替换语义

```text
_replace_owner_bundle(runtime, mv_id, bundle):

    previous_bundle_id = mv_states[mv_id].active_bundle_id

    if previous_bundle_id is not None:
        删除旧 bundle
        删除所有 controlled_vehicle_states 中属于旧 bundle 的记录

    写入新 bundle

    controlled[mv_id]    = role "mv"
    controlled[rear_id]  = role "rear"
    controlled[front_id] = role "front"

    mv_states[mv_id].current_plan_gap = bundle.selected_gap
    mv_states[mv_id].active_bundle_id = bundle.bundle_id
    mv_states[mv_id].last_plan_step = bundle.start_step
    mv_states[mv_id].last_plan_t = bundle.start_t

    return updated_runtime
```

这里当前真实行为是：

- 同一个 MV 每次新 trigger，会直接用新 bundle 覆盖自己旧 bundle
- 当前没有“多个 MV 同时抢同一辆 lane 2 车控制权”的仲裁逻辑
- 这也是多 MV 扩展前必须补的点之一

---

## 八、OneStep stage2 运动执行：当前真实伪代码

对应 `motion.py` 中的 `_build_onestep_stage2_motion_outputs`。

### 1. 顶层运动候选生成

```text
_build_onestep_stage2_motion_outputs(state, runtime):

    candidates = {}
    mv_states = dict(runtime.mv_plan_states)
    bundles = dict(runtime.onestep_plan_bundles)
    controlled = dict(runtime.controlled_vehicle_states)
    motion_records = []

    for each active vehicle_id:
        controlled_state = controlled.get(vehicle_id)
        bundle = bundles.get(controlled_state.bundle_id) if controlled_state exists else None

        if controlled_state is None or bundle is None:
            candidate = _constant_20_candidate(state, vehicle_id)
            candidates[vehicle_id] = (candidate,)
            continue

        sample_time = (state.t + state.dt) - bundle.start_t
        sample = _sample_bundle_vehicle_state(sample_time, bundle, controlled_state.role)

        candidate = CandidateKinematics(
            x_global = sample.x,
            y = 当前车辆 y,
            v = sample.v,
            a = sample.a,
        )
        candidates[vehicle_id] = (candidate,)

    completion_record = _complete_longitudinal_if_needed(
        state,
        mv_states,
        bundles,
        controlled,
        candidates,
    )
    if completion_record exists:
        motion_records.append(completion_record)

    return MotionBuildResult(
        candidate_kinematics = candidates,
        candidate_lane_state = {},
        candidate_state_transitions = {},
        runtime = updated_runtime,
        motion_events = motion_records,
    )
```

### 2. 非受控车辆的默认运动

```text
_constant_20_candidate(state, vehicle_id):
    x_next = current.x_global + current.v * dt
    y_next = current.y
    v_next = current.v
    a_next = current.a
```

这里真实代码行为是：

- 位置按当前速度积分一步
- 不主动把速度强行改成 `20`
- 只是当前桥接场景里大多数车本来就是 `20`

### 3. 当前滚动重规划的真实边界承接公式

当前每次新 bundle 都不是从“固定 `x_m0=0, v=20, t=0` 的原始 quintic 语义”直接拿来推进，而是：

```text
每个受控角色 i ∈ {mv, rear, front}

起点边界:
    x_i(0) = trigger 时刻真实 x_i
    v_i(0) = trigger 时刻真实 v_i
    a_i(0) = trigger 时刻真实 a_i

终点边界:
    x_i(T) = 本次 OneStep 结果对应的目标全局位置
    v_i(T) = v_ref = 20
    a_i(T) = 0

其中:
    T = best_score.t_m
```

然后用五次边界插值采样：

```text
sample_rolling_quintic_boundary(
    t,
    duration = T,
    x0, v0, a0,
    x1, v1, a1,
)
```

如果已经超过 bundle 规划时长：

```text
if sample_time > T:
    x = x_target + v_target * (sample_time - T)
    v = v_target
    a = 0
```

也就是：

- 先按新的滚动 quintic 从真实边界承接
- 超过该 bundle 的设计时长后，先沿终点速度匀速滑行

### 4. 三个角色的目标终点如何算

```text
MV:
    x_target = bundle.origin_x_global + sample_merge_vehicle_state(t_m, ...).x

rear:
    x_target = bundle.origin_x_global + sample_selected_rear_vehicle_state(t_m, ...).x

front:
    x_target = bundle.origin_x_global + sample_selected_front_vehicle_state(t_m, ...).x
```

特殊分支：

```text
if t_m <= 1e-12:
    mv target x = merge_point_x_global
    rear/front target x 直接按 best_gap 与 delta 星值回退/前推得到
```

终点目标速度当前都取：

```text
target_v = local_scenario.v_ref = 20.0
```

### 5. 当前纵向完成判定

```text
_evaluate_stage2_longitudinal_ready(bundle, candidates):

    mv_x_next = candidates[mv][0].x_global
    rear_x_next = candidates[rear][0].x_global
    front_x_next = candidates[front][0].x_global

    front_gap_m = front_x_next - mv_x_next
    rear_gap_m = mv_x_next - rear_x_next
    required_gap_m = bundle.required_longitudinal_gap_m

    merge_point_reached = mv_x_next + 1e-9 >= bundle.merge_point_x_global

    longitudinal_ready =
        merge_point_reached
        and front_gap_m + 1e-9 >= required_gap_m
        and rear_gap_m + 1e-9 >= required_gap_m

    rule =
        "merge_point_reached_and_front_rear_gap_ge_Greq_over_2"
```

这就是当前真实代码里的“纵向满足合流条件”。

### 6. 当前纵向完成后的真实 handoff

```text
_complete_longitudinal_if_needed(...):

    for each mv_state with active_bundle_id:
        completion_eval = _evaluate_stage2_longitudinal_ready(...)
        if not completion_eval.longitudinal_ready:
            continue

        删除 bundle
        删除属于该 bundle 的 controlled_vehicle_states
        mv_states[mv_id].active_bundle_id = None
        mv_states[mv_id].current_plan_gap = None

        return longitudinal_completion_record
```

关键真实语义：

- 清的是纵向控制 bundle
- 当前不会在这一刻做 lane/road_role/merge_state 的正式迁移
- 下一步开始，这三辆车都会回到默认运动
- 结合当前桥接场景，默认就是继续按已有速度往前走；你的当前实验设定里就是“完成后继续 20m/s 匀速”

### 7. 当前 OneStep stage2 路径尚未做的事

```text
没有横向轨迹执行
没有 merge_state = merging 的 stage2 专属状态机
没有正式把 MV 从 on-ramp 迁移成 mainline vehicle
没有 lane_state / road_role state transition 输出
```

也就是说，当前真正实现的是：

```text
单 MV 的纵向接管与纵向完成判定
```

而不是“完整合流动作全部做完”。

---

## 九、当前 runtime 刷新时保留的分支与清理逻辑

`refresh_runtime_state` 当前有几段很重要的真实清理逻辑：

```text
refresh_runtime_state(previous_runtime, state):

    if previous_runtime is None:
        return initialize_runtime_state(state)

    active_mv_ids =
        所有仍 active 且仍是 on-ramp merge vehicle 的车辆

    refreshed_mv_states[vehicle_id] =
        保留旧 mv_state 其他字段
        只重新按当前位置刷新 zone_state

    onestep_plan_bundles =
        只保留 owner mv 仍 active 的 bundle

    controlled_vehicle_states =
        只保留：
            vehicle_id 仍 active
            bundle_id 仍存在
            owner_mv_id 仍 active

    planned_trajectories =
        只保留 trajectory.mv_id 仍 active 的 legacy trajectory

    danger_vehicle_ids =
        只保留仍 active 的 danger ids

    for each mv_state:
        if mv_state.active_bundle_id 不再有效:
            清空 active_bundle_id
            清空 current_plan_gap
```

这意味着：

- runtime 并不是每一步完全重建
- 它会继承旧 runtime 中大量字段
- 但会做“存活性过滤”和“失效 bundle 清理”

---

## 十、当前代码中保留但 OneStep 路径暂未使用的 legacy 分支

这部分很重要，因为它们还在代码里，但不能误认为已经被 OneStep stage2 接管。

### 1. control-zone legacy 规划

`plan_control_zone_gaps` 真实逻辑是：

```text
按 x_global 从前到后遍历 MV
对每个 control_zone 内 MV:
    取第一个未被占用的 gap
    不做 OneStep 打分
    不做当前 stage2 的滚动真实状态重建
    直接 build_approaching_trajectory
```

它的选 gap 原因字符串甚至就是：

```text
"simplified_first_available_gap"
```

### 2. merge-zone legacy 锁 gap

`lock_merge_zone_gaps` 真实逻辑是：

```text
若 MV 在 merge_zone:
    若还没有 locked_gap 且 current_plan_gap 存在:
        进入 merge_zone 时锁定 current_plan_gap

    check = run_simplified_merge_check(...)

    if check.result == True and merge_state != merging:
        build_merge_execution_trajectory(...)
        merge_state = merging
```

### 3. legacy 轨迹推进

`advance_planned_trajectory` 真实逻辑仍然在：

```text
approaching:
    只做纵向 x += v * dt, y 不变

merge_execution:
    按 duration_steps 线性推进 y 到 lane_2 centerline
    到终点后 merge_completed = True
```

### 4. 为什么说这些在 OneStep stage2 路径里“暂未使用”

因为当前 `algorithm_variant == "onestep_stage2"` 时：

```text
不会调用 plan_control_zone_gaps
不会调用 lock_merge_zone_gaps
不会调用 build_merge_execution_trajectory
不会走 planned_trajectories 驱动的 legacy merge_execution
```

---

## 十一、当前明确未完成或带限制的点

这里列的都不是“概念上以后可以优化”，而是当前真实代码还没完成的部分。

### 1. OneStep stage2 明确只支持单 MV

```text
_resolve_single_active_mv_id(...)
    len(active_mv_ids) != 1 就直接报错
```

### 2. 缺少多 MV 的共享 gap 占用与冲突仲裁

当前 OneStep stage2 没有这些逻辑：

```text
selected_gaps
last_selected_gap
多 MV 的顺序约束
同一步多个 MV 抢同一个 gap 的冲突消解
同一辆 lane_2 车被两个 MV 同时纳入 bundle 的冲突消解
```

### 3. `_replace_owner_bundle` 只覆盖“同一个 MV 的旧 bundle”

它不会检查：

```text
新 bundle 的 rear/front 是否已经被别的 MV 控制
```

所以当前不具备多 MV bundle 级冲突管理能力。

### 4. 纵向完成后没有接后续横向执行状态机

当前只做了：

```text
longitudinal_completed -> 清 bundle -> 回默认运动
```

没做：

```text
lane 变化
road_role 变化
merge_state 专属切换
正式主线接管
```

### 5. `resolve_best_gap_vehicle_ids` 依赖 local x 精确匹配

```text
_resolve_vehicle_id_by_local_x(...):
    用 isclose(abs_tol=1e-9) 在 x_local 表里找 vehicle_id
```

当前它默认：

- lane_2 局部坐标足够分离
- 不会有多辆车 local x 完全相同

多 MV、replay、复杂数据下，这种按浮点局部位置精确反查 vehicle id 的方法不够稳。

### 6. `_complete_longitudinal_if_needed` 每步只返回 1 个完成事件

当前函数结构是：

```text
找到第一个 longitudinal_ready 的 bundle
清理它
return 该记录
```

这意味着如果未来多 MV 同一步多个 bundle 同时完成，当前代码不会在一个 step 内完整处理。

### 7. stage2 分析/报告结构当前是单 MV 视角

例如 `onestep_stage2_runner.py` / `onestep_stage2_analysis.py` 当前主要围绕：

```text
mv_id
first_trigger_plan_summary
first_trigger_gap_rows
trigger_plan_summaries
longitudinal_completion_event
```

组织产物，天然是单 MV 模式。

### 8. 当前安全检测仍然比较简化

它目前是：

```text
按 lane 分组
看相邻车 TTC / short gap
```

还不是一个面向多 MV 协同与 replay 复杂交互的最终安全层。

---

## 十二、如果下一阶段先往多 MV 扩，优先应该完善哪些函数或能力

下面这部分不是泛泛而谈，而是直接对着当前代码应该先补什么。

### 优先级 1：去掉 OneStep stage2 的单 MV 硬限制

首先必须改的是：

```text
_resolve_single_active_mv_id(...)
```

要改成：

```text
resolve_active_mv_ids_in_order(...)
```

至少需要明确：

- 多个 MV 的遍历顺序按什么来定
- 是按 on-ramp 上游到下游，还是下游到上游
- 顺序是否与 legacy 中 `_ordered_mv_ids_by_x_desc` 对齐

如果这一步不改，后面所有多 MV 讨论都落不了地。

### 优先级 2：补“多 MV 本轮规划上下文”

当前 `plan_stage2_for_trigger(...)` 默认“一次 trigger 只规划 1 辆 MV”。

多 MV 前要补成类似：

```text
plan_stage2_for_trigger(...):
    active_mv_ids = resolve_active_mv_ids_in_order(...)
    reservation_state = 本轮已占用 gap / 已占用车辆 / 已锁 bundle

    for mv_id in active_mv_ids:
        依据当前 reservation_state 规划该 mv
        更新 reservation_state
```

这里必须先定义三类约束：

```text
gap 是否允许重复被多个 MV 选中
同一辆 rear/front 车是否允许同时被多个 bundle 控制
若前一辆 MV 已选某 gap，后一辆 MV 的可选集合如何收缩
```

### 优先级 3：补 bundle 级冲突管理

当前必须补的不是简单“多加几个 if”，而是明确 ownership 规则。

建议至少补出一个专门模块或函数，类似：

```text
check_bundle_conflict(candidate_bundle, existing_bundles)
reserve_bundle_resources(bundle, reservation_state)
release_bundle_resources(bundle_id, runtime)
```

至少要能判断：

- `selected_gap` 是否冲突
- `selected_rear_vehicle_id` 是否冲突
- `selected_front_vehicle_id` 是否冲突
- 某 lane_2 车辆是否会被两个 MV 在同一时窗内同时施加不同控制

### 优先级 4：把“当前真实状态重建 local frame”扩成多 MV 可用接口

当前 `build_stage2_local_frame(state, mv_id)` / `build_stage2_local_scenario(state, mv_id)` 是单车接口。

下一步最好补出：

```text
build_stage2_local_frame_for_mv(state, mv_id, context)
build_stage2_local_scenario_for_mv(state, mv_id, context)
```

这里的 `context` 至少应容纳：

- 前面 MV 已经保留/占用的 gap 信息
- 本轮不允许再选的 lane_2 车辆
- 是否要把别的 MV 视作额外障碍物或约束项

### 优先级 5：补 trigger 后“多 MV 连续规划”的 runtime 写回协议

当前 `_replace_owner_bundle(...)` 只适合单 MV。

多 MV 前应该改成至少能支持：

```text
apply_stage2_plan_update(runtime, plan_updates):
    先统一检查冲突
    再批量写入 bundles / controlled_vehicle_states / mv_plan_states
```

否则一个 MV 写进去以后，后一个 MV 再写，容易把同一步里前面的结果部分覆盖或留下半成品状态。

### 优先级 6：补多 MV 的纵向完成处理

当前 `_complete_longitudinal_if_needed(...)` 只处理一个完成事件。

多 MV 前需要改成：

```text
collect_all_longitudinal_completions(...)
for each completed bundle:
    清 bundle
    清 controlled states
    清 mv_state.active_bundle_id/current_plan_gap
emit 所有 completion events
```

### 优先级 7：补多 MV 的分析与验证结构

当前项目其实已经有一个很好的起点：

```text
validation.detect_multi_mv_gap_conflicts(summary)
```

但它现在主要还是 summary 后验检查。

多 MV 阶段前最好补：

- 多 MV trigger summary 结构
- 每个 MV 的 bundle 生命周期表
- 同步 step 上的 gap 冲突事件
- 同步 step 上的 controlled_vehicle 冲突事件
- 多 MV 过程图导出规则

### 优先级 8：明确“纵向完成后 handoff 到什么”

你目前已经把实验语义定成：

```text
纵向完成后先保持 20m/s 匀速
```

这对单 MV 验证已经够用。

但多 MV 阶段之前，最好把这个语义正式固定在一个清晰接口里，而不是只依赖默认候选逻辑间接得到。例如：

```text
handoff_after_longitudinal_completion(vehicle_state) -> default mainline cruising candidate
```

这样后面切到 HDV / replay / 真横向执行时，替换点会更清楚。

---

## 十三、与旧版《匝道合流算法流程.md》的主要差异

下面只对比“旧伪代码描述的逻辑”和“当前真实代码”，不评价哪种一定更好。

### 差异 1：旧文档描述的是“完整算法流程”，当前真实代码是“共用外壳 + 两条算法分支”

旧文档更像：

```text
一个完整统一的匝道合流算法循环
```

当前真实代码更像：

```text
step0-3 / safety / trigger / commit 共用
    + legacy_batch_c 分支
    + onestep_stage2 分支
```

也就是说，当前 OneStep 接入还不是“旧流程被整个替换”，而是阶段性嵌入。

### 差异 2：旧文档把边界车生成、车辆移除也写在算法循环内；当前真实代码把这些外包给 step0-3 / step9-11

旧文档前半部分会写：

```text
移除驶出车辆
边界车辆生成
构造 S(t)
推进时间
```

当前真实代码里，这些已经被放到更外层框架：

- `step0_3`
- `commit_step`
- `advance_time_after_commit_and_integration`

所以 `RampMergeEngine.advance_one_step` 看到的是一个已经 freeze 完的 state。

### 差异 3：旧文档的主逻辑是“控制区筛 gap + 合流区锁 gap”；当前 OneStep stage2 路径不是这个语义

旧文档核心是：

```text
control_zone:
    ReachabilityFilter
    OrderConstraintFilter
    GapScore
    SelectBestGap
    GenerateTrajectory(approaching)

merge_zone:
    锁定 current_plan_gap
    MergeCheck
    GenerateTrajectory(merge_execution)
```

当前 OneStep stage2 真实逻辑是：

```text
每次 trigger:
    用当前真实状态重建 local frame
    调 OneStep 内核完整评估
    得到 best_gap / best_score
    生成三车 bundle
    用 bundle 驱动滚动纵向轨迹
    用纵向完成条件清 bundle
```

也就是说，OneStep stage2 当前不是“控制区规划 + 合流区执行”的二段式壳，而是“trigger 重算 + bundle 接管”的壳。

### 差异 4：旧文档强调多 MV 顺序和 `selected_gaps`；当前 OneStep stage2 真实代码还没有多 MV

旧文档里明确有：

```text
selected_gaps
last_selected_gap
按 on-ramp 顺序遍历所有 MV
order constraint
```

当前真实 OneStep stage2：

```text
明确只支持 exactly one MV
```

所以旧文档里的多 MV 协调语义，现在不能视为已经被真实代码实现。

### 差异 5：旧文档里的 gap 打分是运行时显式逐 gap 循环；当前 OneStep stage2 把打分下沉到 OneStep 内核

旧文档风格是：

```text
for each gap_j:
    score_j = GapScore(...)
SelectBestGap(...)
```

当前真实代码是：

```text
evaluation = evaluate_one_step_scenario(local_scenario, algorithm)
best_gap = evaluation.best_gap
best_score = evaluation.best_score
gap_rows = evaluation.gap_rows
```

也就是 gap 评分过程已经封装到 OneStep kernel 里了。

### 差异 6：旧文档里的“当前可控性判断”会被直接写成流程概念；当前真实代码是 runtime 级 `danger_vehicle_ids`

旧文档表述更像：

```text
vehicle.safety_state = normal / danger
EffectiveControllable(vehicle)
```

当前真实代码实际落点是：

```text
run_safety_check(...) -> danger_vehicle_ids
effective_controllable(...) = base controllable and not in danger ids
```

它没有把 `danger` 长期写回一个持久 vehicle state 字段。

### 差异 7：旧文档里的“完成合流”包含 lane/road_role 迁移；当前 OneStep stage2 只有纵向完成

旧文档会写：

```text
MV 完成合流后：
    从匝道集合移除
    加入主线集合
    更新 lane / road_section
    清空 gap 状态
```

当前 OneStep stage2 真实代码只做：

```text
满足纵向完成条件后：
    删除 bundle
    删除 controlled states
    清 active_bundle_id / current_plan_gap
    下一步回默认运动
```

所以当前真实代码实现的是：

```text
纵向接管完成
```

不是旧文档意义上的“完整合流状态迁移完成”。

### 差异 8：当前真实代码比旧文档多了完整的验证、出图、报告链

旧文档更像算法说明。

当前真实代码已经实际补齐了：

```text
S05 / S07 场景桥接
stage1 precheck runner
stage2 rolling runner
summary.json
report.md
trajectory.csv
process_x_t_local.png
process_v_t.png
以及 first-trigger 专属图和表
```

这意味着当前项目的一个真实特点是：

```text
算法实现 + 可复盘验证链
```

是一起落地的，而不是只有算法主函数。

---

## 十四、给下一阶段多 MV 扩展的简短建议

如果下一阶段只往多 MV 扩，而先不碰 HDV / replay，那么最合适的切入顺序是：

```text
1. 把单 MV planner 入口改成多 MV 顺序入口
2. 定义本轮 reservation / ownership 规则
3. 批量写 runtime，而不是逐 MV 即写即改
4. 改纵向完成处理为多 bundle 可并行收口
5. 扩 summary / validation / plots 为多 MV 版本
```

这样做的好处是：

- 不会把当前单 MV 已跑通的滚动 bundle 语义推翻重来
- 也能最大程度复用你现在已经打通的 S05 / S07 场景桥接与验证链

---

## 十五、最短版结论

当前真实代码不是“旧版多 MV 匝道合流算法已经被 OneStep 全量替换”，而是：

```text
共用 ramp_merge 外壳
    + legacy_batch_c 壳仍在
    + OneStep stage2 已接入单 MV 滚动触发重规划与纵向 bundle 接管
```

当前已经真实完成的是：

```text
S05 / S07 的场景桥接
单 MV 的每次 trigger 重算
新 bundle 用当前真实三车 x/v/a 承接
纵向完成条件判定
验证、报告、出图链
```

当前还没有完成、也是多 MV 前必须先补的，是：

```text
多 MV 规划顺序
多 MV gap / 车辆控制权冲突管理
多 bundle runtime 写回
多 bundle 完成收口
多 MV 分析验证结构
```

