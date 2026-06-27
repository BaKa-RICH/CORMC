from __future__ import annotations

import time
from dataclasses import dataclass, replace
from typing import Any, Iterable, Mapping

from cormc.traffic_flow.generation import BoundaryQueueItem, compute_spawn_decisions
from cormc.simulation_core.pre_freeze import (
    DEFAULT_ROAD_GEOMETRY,
    PreFreezeWorkspace,
    RelationsSnapshot,
    RoadGeometryConfig,
    SimulationState,
    Step0To3RunResult,
    build_prefreeze_workspace_from_state,
    emit_freeze_event_candidate,
    emit_geometry_event_candidate,
    emit_relation_refresh_event_candidate,
    freeze_simulation_state,
    refresh_relations_snapshot,
    run_geometry_sanity_baseline,
    step0_cleanup_and_prepare,
    step1_prefreeze_boundary_generation_hook,
)
from cormc.simulation_core.commit import (
    CommandBuffer,
    CommitResult,
    NextStateBuffer,
    TimeAdvanceResult,
    advance_time_after_commit_and_integration,
    commit_step,
)

from cormc.onestep.rolling.events import (
    build_default_motion_event,
    build_gap_snapshot_event,
    build_gap_lock_event,
    build_gap_selection_event,
    build_merge_check_event,
    build_merge_completion_event,
    build_onestep_stage2_bundle_lifecycle_event,
    build_onestep_stage2_gap_eval_event,
    build_onestep_stage2_longitudinal_completion_event,
    build_onestep_stage2_plan_event,
    build_runtime_state_event,
    build_safety_event,
    build_trajectory_event,
    build_trigger_event,
    build_zone_state_event,
)
from cormc.onestep.rolling.gaps import identify_and_number_gaps
from cormc.onestep.rolling.motion import build_motion_outputs
from cormc.onestep.rolling.stage2_planner import plan_stage2_for_trigger
from cormc.onestep.rolling.planner import (
    PlanningResult,
    decide_trigger_plan,
    detect_entry_vehicle_ids,
    detect_entry_plan_trigger,
    lock_merge_zone_gaps,
    plan_control_zone_gaps,
)
from cormc.onestep.rolling.safety import run_safety_check
from cormc.onestep.rolling.state import (
    GapSnapshot,
    PlanningTimingRecord,
    RampMergeRuntimeState,
    SafetyCheckResult,
    TriggerDecision,
    ZONE_MERGE,
    refresh_runtime_state,
)


@dataclass(frozen=True)
class RampMergeStepResult:
    input_state: SimulationState
    frozen_state: SimulationState
    relations: RelationsSnapshot
    command_buffer: CommandBuffer
    next_state_buffer: NextStateBuffer
    commit_result: CommitResult
    time_advance_result: TimeAdvanceResult
    advanced_state: SimulationState
    ramp_merge_runtime: RampMergeRuntimeState
    safety_result: SafetyCheckResult
    trigger_decision: TriggerDecision
    gap_snapshot: GapSnapshot | None
    planning_timing: PlanningTimingRecord | None
    actual_events: list[dict[str, Any]]
    actual_sanity_checks: list[dict[str, Any]]


@dataclass(frozen=True)
class RampMergeEngine:
    scenario_config: Mapping[str, Any]
    run_id: str = "ramp-merge-batch-c"
    geometry: RoadGeometryConfig = DEFAULT_ROAD_GEOMETRY
    algorithm_variant: str = "legacy_batch_c"
    random_queue: tuple[BoundaryQueueItem, ...] = ()
    safe_spawn_gap_m: float = 20.0

    def advance_one_step(self, state: SimulationState) -> RampMergeStepResult:
        scenario_id = str(
            self.scenario_config.get("scenario_id")
            or state.scenario_config_ref
            or "unknown_scenario"
        )
        step0_3 = _run_step0_to_3_from_state(
            state,
            self.scenario_config,
            geometry=self.geometry,
            random_queue=self.random_queue,
            safe_spawn_gap_m=self.safe_spawn_gap_m,
        )
        frozen = step0_3.state
        previous_runtime = frozen.ramp_merge_runtime
        refreshed_runtime = refresh_runtime_state(
            previous_runtime,
            frozen,
            geometry=self.geometry,
        )
        safety_result = run_safety_check(frozen)
        entry_vehicle_ids = detect_entry_vehicle_ids(previous_runtime, refreshed_runtime)
        if self.algorithm_variant == "onestep_stage2":
            entry_vehicle_ids = tuple(
                dict.fromkeys(
                    (
                        *entry_vehicle_ids,
                        *_detect_stage2_merge_zone_entry_ids(
                            previous_runtime,
                            refreshed_runtime,
                        ),
                    )
                )
            )
        entry_plan_trigger = bool(entry_vehicle_ids)
        trigger_decision = decide_trigger_plan(
            frozen,
            refreshed_runtime.planner_state,
            entry_plan_trigger=entry_plan_trigger,
            safety_alert=safety_result.safety_alert,
            entry_vehicle_ids=entry_vehicle_ids,
        )
        runtime = replace(
            refreshed_runtime,
            planner_state=trigger_decision.planner_state,
            danger_vehicle_ids=safety_result.danger_vehicle_ids,
        )
        gap_snapshot: GapSnapshot | None = None
        planning_start_ns: int | None = None
        planning_timing: PlanningTimingRecord | None = None
        if trigger_decision.trigger_plan:
            planning_start_ns = time.perf_counter_ns()
            gap_snapshot = identify_and_number_gaps(
                frozen,
                safety_result.danger_vehicle_ids,
            )
            runtime = replace(runtime, last_gap_snapshot=gap_snapshot)
            if self.algorithm_variant == "onestep_stage2":
                control_plan = plan_stage2_for_trigger(
                    frozen,
                    runtime,
                    gap_snapshot,
                    trigger_decision,
                )
            else:
                control_plan = plan_control_zone_gaps(frozen, runtime, gap_snapshot)
            runtime = control_plan.runtime
        else:
            control_plan = None
        if self.algorithm_variant == "onestep_stage2":
            merge_plan = None
        else:
            merge_plan = lock_merge_zone_gaps(
                frozen,
                runtime,
                geometry=self.geometry,
            )
            runtime = merge_plan.runtime
        motion_outputs = build_motion_outputs(
            frozen,
            runtime,
            geometry=self.geometry,
            algorithm_variant=self.algorithm_variant,
        )
        if planning_start_ns is not None:
            planning_timing = _build_planning_timing_record(
                frozen,
                trigger_decision,
                gap_snapshot,
                control_plan,
                duration_ns=time.perf_counter_ns() - planning_start_ns,
            )
        runtime = motion_outputs.runtime
        command_buffer = CommandBuffer(step=frozen.step, t=frozen.t)
        next_state_buffer = NextStateBuffer(
            step=frozen.step,
            t=frozen.t,
            candidate_kinematics=motion_outputs.candidate_kinematics,
            candidate_lane_state=motion_outputs.candidate_lane_state,
            candidate_state_transitions=motion_outputs.candidate_state_transitions,
            ramp_merge_runtime=runtime,
        )
        commit = commit_step(
            frozen,
            command_buffer,
            next_state_buffer,
            run_id=self.run_id,
            scenario_id=scenario_id,
        )
        time_advance = advance_time_after_commit_and_integration(
            commit,
            run_id=self.run_id,
            scenario_id=scenario_id,
        )
        actual_events = [
            *step0_3.actual_events,
            build_runtime_state_event(frozen, previous_runtime, runtime),
            build_zone_state_event(frozen, runtime),
            build_safety_event(frozen, safety_result),
            build_trigger_event(
                frozen,
                trigger_decision,
                gap_identification_executed=gap_snapshot is not None,
            ),
            *(
                [build_gap_snapshot_event(frozen, gap_snapshot)]
                if gap_snapshot is not None
                else []
            ),
            *(
                (
                    [
                        build_onestep_stage2_plan_event(frozen, record)
                        for record in control_plan.gap_selection_records
                    ]
                    if self.algorithm_variant == "onestep_stage2"
                    else [
                        build_gap_selection_event(frozen, record)
                        for record in control_plan.gap_selection_records
                    ]
                )
                if control_plan is not None
                else []
            ),
            *[
                (
                    build_onestep_stage2_gap_eval_event(frozen, record)
                    if self.algorithm_variant == "onestep_stage2"
                    else build_trajectory_event(frozen, record)
                )
                for record in (
                    control_plan.trajectory_records if control_plan is not None else ()
                )
            ],
            *(
                [
                    build_onestep_stage2_bundle_lifecycle_event(frozen, record)
                    for record in control_plan.bundle_lifecycle_records
                ]
                if control_plan is not None
                and self.algorithm_variant == "onestep_stage2"
                else []
            ),
            *(
                [
                    build_gap_lock_event(frozen, record)
                    for record in control_plan.gap_lock_records
                ]
                if control_plan is not None
                and self.algorithm_variant == "onestep_stage2"
                else []
            ),
            *(
                []
                if merge_plan is None
                else [
                    build_gap_lock_event(frozen, record)
                    for record in merge_plan.gap_lock_records
                ]
            ),
            *(
                []
                if merge_plan is None
                else [
                    build_merge_check_event(frozen, check_result)
                    for check_result in merge_plan.merge_check_results
                ]
            ),
            *(
                []
                if merge_plan is None
                else [
                    build_trajectory_event(frozen, record)
                    for record in merge_plan.trajectory_records
                ]
            ),
            build_default_motion_event(frozen, motion_outputs.candidate_kinematics),
            *(
                [
                    build_onestep_stage2_bundle_lifecycle_event(frozen, record)
                    for record in motion_outputs.bundle_lifecycle_records
                ]
                if self.algorithm_variant == "onestep_stage2"
                else []
            ),
            *[
                (
                    build_onestep_stage2_longitudinal_completion_event(frozen, record)
                    if self.algorithm_variant == "onestep_stage2"
                    and record.get("longitudinal_completed") is True
                    else build_gap_lock_event(frozen, record)
                    if self.algorithm_variant == "onestep_stage2"
                    and record.get("reason") == "onestep_stage2_merge_zone_gap_locked"
                    else build_merge_completion_event(frozen, record)
                    if record.get("merge_completed") is True
                    else build_trajectory_event(frozen, record)
                )
                for record in motion_outputs.motion_events
            ],
            *commit.history.event_dicts(),
        ]
        actual_sanity_checks = [
            *step0_3.actual_sanity_checks,
            *commit.history.sanity_dicts(),
        ]
        return RampMergeStepResult(
            input_state=state,
            frozen_state=frozen,
            relations=step0_3.relations,
            command_buffer=command_buffer,
            next_state_buffer=next_state_buffer,
            commit_result=commit,
            time_advance_result=time_advance,
            advanced_state=time_advance.advanced_state,
            ramp_merge_runtime=runtime,
            safety_result=safety_result,
            trigger_decision=trigger_decision,
            gap_snapshot=gap_snapshot,
            planning_timing=planning_timing,
            actual_events=actual_events,
            actual_sanity_checks=actual_sanity_checks,
        )


def _build_planning_timing_record(
    state: SimulationState,
    trigger_decision: TriggerDecision,
    gap_snapshot: GapSnapshot,
    control_plan: PlanningResult,
    *,
    duration_ns: int,
) -> PlanningTimingRecord:
    plan_records = tuple(control_plan.gap_selection_records)
    round_id = (
        str(plan_records[0]["round_id"])
        if plan_records
        else f"trigger_round:{state.step}"
    )
    return PlanningTimingRecord(
        step=state.step,
        t=state.t,
        round_id=round_id,
        trigger_reason=trigger_decision.trigger_reason,
        active_trigger_reasons=trigger_decision.active_trigger_reasons,
        entry_vehicle_ids=trigger_decision.entry_vehicle_ids,
        duration_ns=duration_ns,
        planned_mv_ids=_ordered_unique(
            str(record["mv_id"])
            for record in plan_records
        ),
        controlled_vehicle_ids=_ordered_unique(
            str(vehicle_id)
            for record in plan_records
            for vehicle_id in record["controlled_vehicle_ids"]
        ),
        gap_count=len(gap_snapshot.gaps),
        plan_count=len(plan_records),
    )


def _ordered_unique(values: Iterable[str]) -> tuple[str, ...]:
    return tuple(dict.fromkeys(values))


def _run_step0_to_3_from_state(
    state: SimulationState,
    config: Mapping[str, Any],
    *,
    geometry: RoadGeometryConfig,
    random_queue: tuple[BoundaryQueueItem, ...] = (),
    safe_spawn_gap_m: float = 20.0,
) -> Step0To3RunResult:
    workspace = _workspace_from_state(state)
    cleanup = step0_cleanup_and_prepare(workspace, geometry=geometry)
    spawn_decisions = compute_spawn_decisions(
        random_queue,
        state,
        safe_spawn_gap_m=safe_spawn_gap_m,
    )
    boundary = step1_prefreeze_boundary_generation_hook(
        workspace,
        dict(config),
        spawn_decisions=spawn_decisions,
    )
    frozen = freeze_simulation_state(workspace)
    relations = refresh_relations_snapshot(frozen, geometry=geometry)
    events = [
        cleanup,
        boundary,
        emit_freeze_event_candidate(frozen),
        emit_relation_refresh_event_candidate(frozen, relations),
        emit_geometry_event_candidate(frozen, geometry=geometry),
    ]
    sanity = run_geometry_sanity_baseline(frozen, relations, geometry=geometry)
    return Step0To3RunResult(
        state=frozen,
        relations=relations,
        actual_events=events,
        actual_sanity_checks=sanity,
        expected_png_features=[],
    )


def _detect_stage2_merge_zone_entry_ids(
    previous_runtime: RampMergeRuntimeState | None,
    refreshed_runtime: RampMergeRuntimeState,
) -> tuple[str, ...]:
    entered: list[str] = []
    for vehicle_id, refreshed_mv_state in refreshed_runtime.mv_plan_states.items():
        if refreshed_mv_state.zone_state != ZONE_MERGE:
            continue
        if refreshed_mv_state.locked_gap is not None:
            continue
        if refreshed_mv_state.current_plan_gap is None:
            continue
        previous_mv_state = (
            previous_runtime.mv_plan_states.get(vehicle_id)
            if previous_runtime is not None
            else None
        )
        if previous_mv_state is None or previous_mv_state.zone_state != ZONE_MERGE:
            entered.append(vehicle_id)
    return tuple(entered)


def _workspace_from_state(state: SimulationState) -> PreFreezeWorkspace:
    return build_prefreeze_workspace_from_state(state)
