from __future__ import annotations

from dataclasses import dataclass
from dataclasses import replace
from types import MappingProxyType
from typing import Any, Mapping

from cormc.simulation_core.pre_freeze import (
    DEFAULT_ROAD_GEOMETRY,
    RoadGeometryConfig,
    SimulationState,
    VehicleState,
    resolve_on_ramp_control_region,
)

from cormc.onestep.rolling.state import (
    EFFECTIVE_CONTROL_NONE,
    GapCandidate,
    GapRef,
    GapSnapshot,
    MVPlanState,
    PlannedTrajectory,
    PlannerState,
    RampMergeRuntimeState,
    TRIGGER_MV_ENTER_CONTROL_ZONE,
    TRIGGER_NONE,
    TRIGGER_PERIODIC,
    TRIGGER_SAFETY_ALERT,
    TriggerDecision,
    ZONE_CONTROL,
    ZONE_MERGE,
    ZONE_OUTSIDE_CONTROL,
    ZONE_OUT_OF_SCENE,
)


MERGE_STATE_MERGING = "merging"
MERGE_STATE_MERGE_COMPLETED = "merge_completed"
MERGE_STATE_NORMAL = "normal"
TRAJECTORY_APPROACHING = "approaching"
TRAJECTORY_MERGE_EXECUTION = "merge_execution"


@dataclass(frozen=True)
class MergeCheckResult:
    mv_id: str
    result: bool
    rule: str
    reason: str
    zone_state: str
    locked_gap: GapRef | None


@dataclass(frozen=True)
class PlanningResult:
    runtime: RampMergeRuntimeState
    gap_selection_records: tuple[Mapping[str, Any], ...] = ()
    gap_lock_records: tuple[Mapping[str, Any], ...] = ()
    merge_check_results: tuple[MergeCheckResult, ...] = ()
    trajectory_records: tuple[Mapping[str, Any], ...] = ()
    bundle_lifecycle_records: tuple[Mapping[str, Any], ...] = ()


def derive_zone_state(
    vehicle_state: VehicleState,
    geometry: RoadGeometryConfig = DEFAULT_ROAD_GEOMETRY,
) -> str:
    region = resolve_on_ramp_control_region(
        vehicle_state.x_global,
        vehicle_state.road_role,
        geometry=geometry,
    )
    return {
        "pre_control": ZONE_OUTSIDE_CONTROL,
        "control_zone": ZONE_CONTROL,
        "merge_zone": ZONE_MERGE,
        "post_merge": ZONE_OUT_OF_SCENE,
    }[region.region]


def detect_entry_vehicle_ids(
    previous_runtime: RampMergeRuntimeState | None,
    refreshed_runtime: RampMergeRuntimeState,
) -> tuple[str, ...]:
    entered: list[str] = []
    for vehicle_id, refreshed_mv_state in refreshed_runtime.mv_plan_states.items():
        if refreshed_mv_state.zone_state != ZONE_CONTROL:
            continue
        previous_mv_state = (
            previous_runtime.mv_plan_states.get(vehicle_id)
            if previous_runtime is not None
            else None
        )
        if previous_mv_state is None or previous_mv_state.zone_state != ZONE_CONTROL:
            entered.append(vehicle_id)
    return tuple(entered)


def detect_entry_plan_trigger(
    previous_runtime: RampMergeRuntimeState | None,
    refreshed_runtime: RampMergeRuntimeState,
) -> bool:
    return bool(detect_entry_vehicle_ids(previous_runtime, refreshed_runtime))


def decide_trigger_plan(
    state: SimulationState,
    planner_state: PlannerState,
    entry_plan_trigger: bool,
    safety_alert: bool,
    entry_vehicle_ids: tuple[str, ...] = (),
) -> TriggerDecision:
    periodic_due = state.t + 1e-9 >= planner_state.next_plan_time
    active_reasons: list[str] = []
    if periodic_due:
        active_reasons.append(TRIGGER_PERIODIC)
    if safety_alert:
        active_reasons.append(TRIGGER_SAFETY_ALERT)
    if entry_plan_trigger:
        active_reasons.append(TRIGGER_MV_ENTER_CONTROL_ZONE)

    if periodic_due:
        trigger_reason = TRIGGER_PERIODIC
    elif safety_alert:
        trigger_reason = TRIGGER_SAFETY_ALERT
    elif entry_plan_trigger:
        trigger_reason = TRIGGER_MV_ENTER_CONTROL_ZONE
    else:
        trigger_reason = TRIGGER_NONE

    trigger_plan = trigger_reason != TRIGGER_NONE
    if trigger_plan and trigger_reason == TRIGGER_PERIODIC:
        next_planner_state = replace(
            planner_state,
            next_plan_time=state.t + planner_state.T_plan,
            last_trigger_reason=trigger_reason,
        )
    elif trigger_plan:
        next_planner_state = replace(planner_state, last_trigger_reason=trigger_reason)
    else:
        next_planner_state = planner_state

    return TriggerDecision(
        trigger_plan=trigger_plan,
        trigger_reason=trigger_reason,
        active_trigger_reasons=tuple(active_reasons),
        periodic_due=periodic_due,
        safety_alert=safety_alert,
        entry_plan_trigger=entry_plan_trigger,
        entry_vehicle_ids=entry_vehicle_ids,
        planner_state=next_planner_state,
    )


def gap_ref_from_candidate(gap: GapCandidate, snapshot: GapSnapshot) -> GapRef:
    return GapRef(
        gap_id=gap.gap_id,
        index=gap.index,
        front_vehicle_id=gap.front_vehicle_id,
        rear_vehicle_id=gap.rear_vehicle_id,
        snapshot_step=snapshot.step,
        snapshot_t=snapshot.t,
    )


def plan_control_zone_gaps(
    state: SimulationState,
    runtime: RampMergeRuntimeState,
    gap_snapshot: GapSnapshot,
) -> PlanningResult:
    mv_states = dict(runtime.mv_plan_states)
    planned_trajectories = dict(runtime.planned_trajectories)
    selected_gap_indices: set[int] = set()
    records: list[Mapping[str, Any]] = []
    trajectory_records: list[Mapping[str, Any]] = []

    for mv_id in _ordered_mv_ids_by_x_desc(state, runtime):
        mv_state = mv_states[mv_id]
        if mv_state.zone_state != ZONE_CONTROL:
            continue
        selected_gap = _first_available_gap(gap_snapshot, selected_gap_indices)
        old_trajectory_id = mv_state.planned_trajectory_id
        if selected_gap is None:
            if old_trajectory_id is not None:
                planned_trajectories.pop(old_trajectory_id, None)
            mv_states[mv_id] = replace(
                mv_state,
                current_plan_gap=None,
                planned_trajectory_id=None,
                last_plan_step=state.step,
                last_plan_t=state.t,
            )
            records.append(
                _gap_selection_record(
                    state,
                    mv_id,
                    mv_states[mv_id],
                    selected_gap=None,
                    reason="no_available_gap",
                    score=None,
                )
            )
            continue

        selected_gap_indices.add(selected_gap.index)
        gap_ref = gap_ref_from_candidate(selected_gap, gap_snapshot)
        trajectory = build_approaching_trajectory(state, mv_id, gap_ref)
        if old_trajectory_id is not None and old_trajectory_id != trajectory.trajectory_id:
            planned_trajectories.pop(old_trajectory_id, None)
        planned_trajectories[trajectory.trajectory_id] = trajectory
        mv_states[mv_id] = replace(
            mv_state,
            current_plan_gap=gap_ref,
            planned_trajectory_id=trajectory.trajectory_id,
            last_plan_step=state.step,
            last_plan_t=state.t,
        )
        records.append(
            _gap_selection_record(
                state,
                mv_id,
                mv_states[mv_id],
                selected_gap=selected_gap,
                reason="simplified_first_available_gap",
                score=1.0,
            )
        )
        trajectory_records.append(
            _trajectory_record(
                state,
                mv_id,
                mv_states[mv_id],
                trajectory,
                reason="approaching_trajectory_created",
            )
        )

    return PlanningResult(
        runtime=replace(
            runtime,
            mv_plan_states=MappingProxyType(mv_states),
            planned_trajectories=MappingProxyType(planned_trajectories),
            version="batch_c_v1",
        ),
        gap_selection_records=tuple(records),
        trajectory_records=tuple(trajectory_records),
    )


def lock_merge_zone_gaps(
    state: SimulationState,
    runtime: RampMergeRuntimeState,
    geometry: RoadGeometryConfig = DEFAULT_ROAD_GEOMETRY,
) -> PlanningResult:
    mv_states = dict(runtime.mv_plan_states)
    planned_trajectories = dict(runtime.planned_trajectories)
    lock_records: list[Mapping[str, Any]] = []
    check_results: list[MergeCheckResult] = []
    trajectory_records: list[Mapping[str, Any]] = []

    for mv_id in _ordered_mv_ids_by_x_desc(state, runtime):
        mv_state = mv_states[mv_id]
        current_zone_state = derive_zone_state(state.vehicle_states[mv_id], geometry)
        if mv_state.zone_state != current_zone_state:
            mv_state = replace(mv_state, zone_state=current_zone_state)
            mv_states[mv_id] = mv_state
        if mv_state.zone_state != ZONE_MERGE:
            continue

        locked_gap = mv_state.locked_gap
        if locked_gap is None and mv_state.current_plan_gap is not None:
            locked_gap = mv_state.current_plan_gap
            mv_state = replace(mv_state, locked_gap=locked_gap)
            mv_states[mv_id] = mv_state
            lock_records.append(
                _gap_lock_record(
                    state,
                    mv_id,
                    mv_state,
                    reason="lock_current_plan_gap_on_merge_zone_entry",
                )
            )

        check = run_simplified_merge_check(state, mv_id, locked_gap)
        if locked_gap is not None:
            check_results.append(check)
        if not check.result:
            continue
        if mv_state.merge_state == MERGE_STATE_MERGING:
            continue

        old_trajectory_id = mv_state.planned_trajectory_id
        trajectory = build_merge_execution_trajectory(
            state,
            mv_id,
            locked_gap,
            geometry=geometry,
        )
        if old_trajectory_id is not None and old_trajectory_id != trajectory.trajectory_id:
            planned_trajectories.pop(old_trajectory_id, None)
        planned_trajectories[trajectory.trajectory_id] = trajectory
        mv_state = replace(
            mv_state,
            merge_state=MERGE_STATE_MERGING,
            planned_trajectory_id=trajectory.trajectory_id,
        )
        mv_states[mv_id] = mv_state
        trajectory_records.append(
            _trajectory_record(
                state,
                mv_id,
                mv_state,
                trajectory,
                reason="merge_execution_trajectory_created",
            )
        )

    return PlanningResult(
        runtime=replace(
            runtime,
            mv_plan_states=MappingProxyType(mv_states),
            planned_trajectories=MappingProxyType(planned_trajectories),
            version="batch_c_v1",
        ),
        gap_lock_records=tuple(lock_records),
        merge_check_results=tuple(check_results),
        trajectory_records=tuple(trajectory_records),
    )


def run_simplified_merge_check(
    state: SimulationState,
    mv_id: str,
    locked_gap: GapRef | None,
) -> MergeCheckResult:
    zone_state = derive_zone_state(state.vehicle_states[mv_id])
    if zone_state != ZONE_MERGE:
        return MergeCheckResult(
            mv_id=mv_id,
            result=False,
            rule="batch_c_minimal_merge_check",
            reason="mv_not_in_merge_zone",
            zone_state=zone_state,
            locked_gap=locked_gap,
        )
    if locked_gap is None:
        return MergeCheckResult(
            mv_id=mv_id,
            result=False,
            rule="batch_c_minimal_merge_check",
            reason="missing_locked_gap",
            zone_state=zone_state,
            locked_gap=locked_gap,
        )
    front_id = locked_gap.front_vehicle_id
    rear_id = locked_gap.rear_vehicle_id
    if front_id not in state.vehicle_states or rear_id not in state.vehicle_states:
        return MergeCheckResult(
            mv_id=mv_id,
            result=False,
            rule="batch_c_minimal_merge_check",
            reason="locked_gap_boundary_vehicle_missing",
            zone_state=zone_state,
            locked_gap=locked_gap,
        )
    return MergeCheckResult(
        mv_id=mv_id,
        result=True,
        rule="batch_c_minimal_merge_check",
        reason="batch_c_minimal_merge_check_passed",
        zone_state=zone_state,
        locked_gap=locked_gap,
    )


def build_approaching_trajectory(
    state: SimulationState,
    mv_id: str,
    target_gap: GapRef,
) -> PlannedTrajectory:
    current = state.vehicle_states[mv_id]
    return PlannedTrajectory(
        trajectory_id=f"ramp_merge_approaching:{state.step}:{mv_id}",
        mv_id=mv_id,
        kind=TRAJECTORY_APPROACHING,
        target_gap=target_gap,
        start_step=state.step,
        start_t=state.t,
        start_x_global=current.x_global,
        start_y=current.y,
        target_y=current.y,
        duration_steps=0,
        progress_step=0,
    )


def build_merge_execution_trajectory(
    state: SimulationState,
    mv_id: str,
    locked_gap: GapRef,
    *,
    geometry: RoadGeometryConfig = DEFAULT_ROAD_GEOMETRY,
    duration_steps: int = 10,
) -> PlannedTrajectory:
    current = state.vehicle_states[mv_id]
    return PlannedTrajectory(
        trajectory_id=f"ramp_merge_execution:{state.step}:{mv_id}",
        mv_id=mv_id,
        kind=TRAJECTORY_MERGE_EXECUTION,
        target_gap=locked_gap,
        start_step=state.step,
        start_t=state.t,
        start_x_global=current.x_global,
        start_y=current.y,
        target_y=float(geometry.lane_centerlines["lane_2"]),
        duration_steps=duration_steps,
        progress_step=0,
    )


def _ordered_mv_ids_by_x_desc(
    state: SimulationState,
    runtime: RampMergeRuntimeState,
) -> tuple[str, ...]:
    return tuple(
        sorted(
            runtime.mv_plan_states,
            key=lambda mv_id: (
                -state.vehicle_states[mv_id].x_global,
                mv_id,
            ),
        )
    )


def _first_available_gap(
    snapshot: GapSnapshot,
    selected_gap_indices: set[int],
) -> GapCandidate | None:
    for gap in snapshot.gaps:
        if gap.index in selected_gap_indices:
            continue
        if gap.effective_control_type == EFFECTIVE_CONTROL_NONE:
            continue
        return gap
    return None


def _gap_selection_record(
    state: SimulationState,
    mv_id: str,
    mv_state: MVPlanState,
    *,
    selected_gap: GapCandidate | None,
    reason: str,
    score: float | None,
) -> Mapping[str, Any]:
    return {
        "mv_id": mv_id,
        "zone_state": mv_state.zone_state,
        "current_plan_gap": mv_state.current_plan_gap,
        "locked_gap": mv_state.locked_gap,
        "selected_gap": selected_gap,
        "score": score,
        "reason": reason,
        "step": state.step,
        "t": state.t,
    }


def _gap_lock_record(
    state: SimulationState,
    mv_id: str,
    mv_state: MVPlanState,
    *,
    reason: str,
) -> Mapping[str, Any]:
    return {
        "mv_id": mv_id,
        "zone_state": mv_state.zone_state,
        "current_plan_gap": mv_state.current_plan_gap,
        "locked_gap": mv_state.locked_gap,
        "selected_gap": None,
        "reason": reason,
        "step": state.step,
        "t": state.t,
    }


def _trajectory_record(
    state: SimulationState,
    mv_id: str,
    mv_state: MVPlanState,
    trajectory: PlannedTrajectory,
    *,
    reason: str,
) -> Mapping[str, Any]:
    return {
        "mv_id": mv_id,
        "zone_state": mv_state.zone_state,
        "current_plan_gap": mv_state.current_plan_gap,
        "locked_gap": mv_state.locked_gap,
        "selected_gap": trajectory.target_gap,
        "trajectory_id": trajectory.trajectory_id,
        "trajectory_kind": trajectory.kind,
        "progress_step": trajectory.progress_step,
        "duration_steps": trajectory.duration_steps,
        "reason": reason,
        "step": state.step,
        "t": state.t,
    }
