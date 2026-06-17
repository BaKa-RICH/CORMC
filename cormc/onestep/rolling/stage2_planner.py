from __future__ import annotations

from dataclasses import dataclass, replace
from types import MappingProxyType
from typing import Any, Mapping

from cormc.onestep.rolling.stage2_adapter import (
    Stage2OneStepEvaluation,
    build_stage2_algorithm_config,
    evaluate_stage2_one_step_from_snapshot,
    resolve_best_gap_vehicle_ids,
    to_global_merge_point,
)
from cormc.onestep.kernel.models import GapEvaluationRow
from cormc.onestep.kernel.models import (
    CONTROLLABILITY_BRANCH_A,
    CONTROLLABILITY_BRANCH_B,
    CONTROLLABILITY_BRANCH_C,
    CONTROLLABILITY_BRANCH_D,
)
from cormc.onestep.rolling.planner import PlanningResult
from cormc.onestep.rolling.state import (
    BUNDLE_SHAPE_MV_FRONT,
    BUNDLE_SHAPE_MV_FRONT_REAR,
    BUNDLE_SHAPE_MV_ONLY,
    BUNDLE_SHAPE_MV_REAR,
    GapCandidate,
    GapPlan,
    GapRef,
    GapSnapshot,
    MVPlanState,
    OneStepBoundaryState,
    OneStepControlledVehicleState,
    OneStepPlanBundle,
    RampMergeRuntimeState,
    TriggerDecision,
    ZONE_CONTROL,
    ZONE_MERGE,
)
from cormc.simulation_core.pre_freeze import SimulationState


@dataclass(frozen=True)
class _RoundContext:
    selected_gap_indices: tuple[int, ...] = ()
    uncontrollable_vehicle_ids: tuple[str, ...] = ()
    tail_frontier_gap_index: int | None = None


@dataclass(frozen=True)
class _PlannedMVResult:
    mv_id: str
    bundle: OneStepPlanBundle | None
    gap_plan: GapPlan | None
    plan_record: Mapping[str, Any]
    gap_eval_records: tuple[Mapping[str, Any], ...]
    selected_gap_index: int | None
    selected_boundary_vehicle_ids: tuple[str, ...]
    reason: str


@dataclass(frozen=True)
class _RoundUpdate:
    mv_plan_states: dict[str, MVPlanState]
    onestep_plan_bundles: dict[str, OneStepPlanBundle]
    controlled_vehicle_states: dict[str, OneStepControlledVehicleState]
    gap_plans: dict[str, GapPlan]
    gap_selection_records: list[Mapping[str, Any]]
    gap_lock_records: list[Mapping[str, Any]]
    gap_eval_records: list[Mapping[str, Any]]
    bundle_lifecycle_records: list[Mapping[str, Any]]


def plan_stage2_for_trigger(
    state: SimulationState,
    runtime: RampMergeRuntimeState,
    gap_snapshot: GapSnapshot,
    trigger_decision: TriggerDecision,
) -> PlanningResult:
    update = _RoundUpdate(
        mv_plan_states=dict(runtime.mv_plan_states),
        onestep_plan_bundles=dict(runtime.onestep_plan_bundles),
        controlled_vehicle_states=dict(runtime.controlled_vehicle_states),
        gap_plans=dict(runtime.gap_plans),
        gap_selection_records=[],
        gap_lock_records=[],
        gap_eval_records=[],
        bundle_lifecycle_records=[],
    )
    context = _initial_round_context(state, runtime)
    round_id = f"trigger_round:{state.step}"

    for round_order, mv_id in enumerate(_ordered_active_mv_ids_by_x_desc(state, runtime)):
        mv_state = update.mv_plan_states[mv_id]
        if mv_state.zone_state == ZONE_MERGE:
            if mv_state.locked_gap is None and mv_state.current_plan_gap is not None:
                locked_plan_id = mv_state.current_plan_id
                if locked_plan_id is not None and locked_plan_id not in update.gap_plans:
                    locked_plan_id = None
                mv_state = replace(
                    mv_state,
                    locked_gap=mv_state.current_plan_gap,
                    locked_plan_id=locked_plan_id,
                )
                update.mv_plan_states[mv_id] = mv_state
                selected_gap = _gap_candidate_for_ref(gap_snapshot, mv_state.locked_gap)
                context_after = (
                    _advance_round_context(context, selected_gap)
                    if selected_gap is not None
                    else context
                )
                update.gap_lock_records.append(
                    _gap_lock_record(
                        state,
                        mv_id,
                        mv_state,
                        round_id=round_id,
                        round_order=round_order,
                        context_before=context,
                        context_after=context_after,
                        reason="onestep_stage2_merge_zone_gap_locked",
                    )
                )
                context = context_after
                continue
            if mv_state.locked_gap is not None:
                selected_gap = _gap_candidate_for_ref(gap_snapshot, mv_state.locked_gap)
                if selected_gap is not None:
                    context = _advance_round_context(context, selected_gap)
            continue

        if mv_state.zone_state != ZONE_CONTROL:
            continue

        planned = _plan_control_zone_mv(
            state,
            trigger_decision,
            gap_snapshot,
            update,
            context,
            mv_id=mv_id,
            round_id=round_id,
            round_order=round_order,
        )
        update.gap_selection_records.append(planned.plan_record)
        update.gap_eval_records.extend(planned.gap_eval_records)
        if planned.bundle is not None and planned.gap_plan is not None:
            _replace_owner_bundle_in_round_update(
                update,
                mv_id=mv_id,
                bundle=planned.bundle,
                gap_plan=planned.gap_plan,
                round_id=round_id,
                round_order=round_order,
            )
            selected_gap = _gap_candidate_by_index(gap_snapshot, planned.selected_gap_index)
            if selected_gap is not None:
                context = _advance_round_context(context, selected_gap)
        else:
            _clear_owner_bundle_in_round_update(
                update,
                mv_id=mv_id,
                reason="owner_replanned_without_available_gap",
                step=state.step,
                t=state.t,
                round_id=round_id,
                round_order=round_order,
            )

    next_runtime = replace(
        runtime,
        mv_plan_states=MappingProxyType(update.mv_plan_states),
        onestep_plan_bundles=MappingProxyType(update.onestep_plan_bundles),
        controlled_vehicle_states=MappingProxyType(update.controlled_vehicle_states),
        gap_plans=MappingProxyType(update.gap_plans),
        version="onestep_stage2_v1",
    )
    return PlanningResult(
        runtime=next_runtime,
        gap_selection_records=tuple(update.gap_selection_records),
        gap_lock_records=tuple(update.gap_lock_records),
        trajectory_records=tuple(update.gap_eval_records),
        bundle_lifecycle_records=tuple(update.bundle_lifecycle_records),
    )


def _ordered_active_mv_ids_by_x_desc(
    state: SimulationState,
    runtime: RampMergeRuntimeState,
) -> tuple[str, ...]:
    return tuple(
        sorted(
            (
                mv_id
                for mv_id in runtime.mv_plan_states
                if mv_id in state.active_vehicle_ids
                and state.vehicle_states[mv_id].is_active
            ),
            key=lambda mv_id: (
                -state.vehicle_states[mv_id].x_global,
                mv_id,
            ),
        )
    )


def _candidate_gap_indices_after_frontier(
    gap_snapshot: GapSnapshot,
    tail_frontier_gap_index: int | None,
) -> tuple[int, ...]:
    if tail_frontier_gap_index is None:
        return tuple(gap.index for gap in gap_snapshot.gaps)
    return tuple(
        gap.index
        for gap in gap_snapshot.gaps
        if gap.index > tail_frontier_gap_index
    )


def _advance_round_context(
    context: _RoundContext,
    selected_gap: GapCandidate,
) -> _RoundContext:
    boundary_ids = {
        *context.uncontrollable_vehicle_ids,
        selected_gap.front_vehicle_id,
        selected_gap.rear_vehicle_id,
    }
    frontier = (
        selected_gap.index
        if context.tail_frontier_gap_index is None
        else max(context.tail_frontier_gap_index, selected_gap.index)
    )
    return _RoundContext(
        selected_gap_indices=tuple(
            sorted({*context.selected_gap_indices, selected_gap.index})
        ),
        uncontrollable_vehicle_ids=tuple(sorted(boundary_ids)),
        tail_frontier_gap_index=frontier,
    )


def _initial_round_context(
    state: SimulationState,
    runtime: RampMergeRuntimeState,
) -> _RoundContext:
    uncontrollable = set(runtime.danger_vehicle_ids)
    for vehicle_id in state.active_vehicle_ids:
        spec = state.vehicle_specs[vehicle_id]
        vehicle_type = spec.vehicle_type.lower()
        compliance_state = spec.compliance_state.lower()
        if vehicle_type == "cav":
            continue
        if vehicle_type == "chv" and compliance_state == "compliant":
            continue
        uncontrollable.add(vehicle_id)
    return _RoundContext(uncontrollable_vehicle_ids=tuple(sorted(uncontrollable)))


def _plan_control_zone_mv(
    state: SimulationState,
    trigger_decision: TriggerDecision,
    gap_snapshot: GapSnapshot,
    update: _RoundUpdate,
    context: _RoundContext,
    *,
    mv_id: str,
    round_id: str,
    round_order: int,
) -> _PlannedMVResult:
    mv_state = update.mv_plan_states[mv_id]
    allowed_gap_indices = _candidate_gap_indices_after_frontier(
        gap_snapshot,
        context.tail_frontier_gap_index,
    )
    filtered_by_frontier_gap_indices = tuple(
        gap.index for gap in gap_snapshot.gaps if gap.index not in allowed_gap_indices
    )
    if not allowed_gap_indices:
        plan_record = _no_gap_plan_record(
            state,
            mv_id,
            mv_state,
            round_id=round_id,
            round_order=round_order,
            context_before=context,
            allowed_gap_indices=allowed_gap_indices,
            filtered_by_frontier_gap_indices=filtered_by_frontier_gap_indices,
            reason="no_available_gap_after_frontier",
        )
        return _PlannedMVResult(
            mv_id=mv_id,
            bundle=None,
            gap_plan=None,
            plan_record=plan_record,
            gap_eval_records=(),
            selected_gap_index=None,
            selected_boundary_vehicle_ids=(),
            reason="no_available_gap_after_frontier",
        )

    blocked_vehicle_ids = tuple(
        sorted(
            {
                *context.uncontrollable_vehicle_ids,
                *_ownership_blocked_vehicle_ids_for_mv(update, mv_id),
            }
        )
    )
    evaluation_result = evaluate_stage2_one_step_from_snapshot(
        state,
        mv_id,
        gap_snapshot,
        uncontrollable_vehicle_ids=blocked_vehicle_ids,
        allowed_gap_indices=allowed_gap_indices,
    )
    evaluation = evaluation_result.evaluation
    reason = (
        "onestep_stage2_trigger_plan"
        if evaluation.best_gap is not None and evaluation.best_score is not None
        else str(evaluation.no_solution_reason or "no_coop_feasible_gap")
    )
    gap_eval_records = _gap_eval_records(
        state,
        mv_id,
        bundle_id=f"onestep_stage2:{state.step}:{mv_id}",
        evaluation_rows=evaluation.gap_rows,
        evaluation_result=evaluation_result,
        round_id=round_id,
        round_order=round_order,
        context_before=context,
        filtered_by_frontier_gap_indices=filtered_by_frontier_gap_indices,
    )
    if evaluation.best_gap is None or evaluation.best_score is None:
        plan_record = _no_gap_plan_record(
            state,
            mv_id,
            mv_state,
            round_id=round_id,
            round_order=round_order,
            context_before=context,
            allowed_gap_indices=allowed_gap_indices,
            filtered_by_frontier_gap_indices=filtered_by_frontier_gap_indices,
            reason=reason,
        )
        return _PlannedMVResult(
            mv_id=mv_id,
            bundle=None,
            gap_plan=None,
            plan_record=plan_record,
            gap_eval_records=gap_eval_records,
            selected_gap_index=None,
            selected_boundary_vehicle_ids=(),
            reason=reason,
        )

    selected_rear_vehicle_id, selected_front_vehicle_id = resolve_best_gap_vehicle_ids(
        evaluation_result.local_frame,
        evaluation,
    )
    runtime_gap_index = _runtime_gap_index_for_kernel_index(
        evaluation_result.local_frame,
        evaluation.best_gap.index,
    )
    selected_snapshot_gap = _gap_candidate_by_index(gap_snapshot, runtime_gap_index)
    selected_gap = GapRef(
        gap_id=(
            selected_snapshot_gap.gap_id
            if selected_snapshot_gap is not None
            else evaluation.best_gap.gap_id
        ),
        index=runtime_gap_index,
        front_vehicle_id=selected_front_vehicle_id,
        rear_vehicle_id=selected_rear_vehicle_id,
        snapshot_step=gap_snapshot.step,
        snapshot_t=gap_snapshot.t,
    )
    bundle_id = f"onestep_stage2:{state.step}:{mv_id}"
    merge_point_x_global = to_global_merge_point(
        evaluation_result.origin_x_global,
        evaluation.best_score.p_m,
    )
    algorithm = build_stage2_algorithm_config()
    required_longitudinal_gap_m = float(algorithm.D_h * 2.0 + algorithm.l_m) / 2.0
    selected_row = _selected_row(evaluation.gap_rows)
    bundle_shape = _bundle_shape_for_selected_row(selected_row)
    controlled_roles = _controlled_roles_for_bundle_shape(
        mv_id,
        selected_front_vehicle_id,
        selected_rear_vehicle_id,
        bundle_shape,
    )
    controlled_vehicle_ids = tuple(vehicle_id for vehicle_id, _ in controlled_roles)
    controlled_roles_by_vehicle_id = MappingProxyType(dict(controlled_roles))
    boundary_state_by_vehicle_id = _boundary_state_by_controlled_roles(
        state,
        controlled_roles,
    )
    bundle = OneStepPlanBundle(
        bundle_id=bundle_id,
        mv_id=mv_id,
        start_step=state.step,
        start_t=state.t,
        trigger_reason=trigger_decision.trigger_reason,
        origin_x_global=evaluation_result.origin_x_global,
        selected_gap=selected_gap,
        selected_rear_vehicle_id=selected_rear_vehicle_id,
        selected_front_vehicle_id=selected_front_vehicle_id,
        selected_vehicle_ids=controlled_vehicle_ids,
        bundle_shape=bundle_shape,
        controlled_vehicle_ids=controlled_vehicle_ids,
        controlled_roles_by_vehicle_id=controlled_roles_by_vehicle_id,
        lane_2_vehicle_order=tuple(
            evaluation_result.local_frame["snapshot_lane_2_vehicle_order"]
        ),
        local_scenario=evaluation_result.local_scenario,
        best_gap=evaluation.best_gap,
        best_score=evaluation.best_score,
        boundary_state_by_vehicle_id=boundary_state_by_vehicle_id,
        required_longitudinal_gap_m=required_longitudinal_gap_m,
        gap_rows=evaluation.gap_rows,
        merge_point_x_global=merge_point_x_global,
    )
    score_summary = MappingProxyType(
        {
            "J": evaluation.best_score.J,
            "d_i": evaluation.best_score.d_i,
            "t_m": evaluation.best_score.t_m,
            "p_m_local": evaluation.best_score.p_m,
            "p_m_global": merge_point_x_global,
            "controllability_branch": selected_row.controllability_branch,
            "bundle_shape": bundle_shape,
            "controlled_vehicle_ids": controlled_vehicle_ids,
            "controlled_roles_by_vehicle_id": controlled_roles_by_vehicle_id,
            "gap_boundary_vehicle_ids": (
                selected_rear_vehicle_id,
                selected_front_vehicle_id,
            ),
        }
    )
    gap_plan = GapPlan(
        plan_id=f"gap_plan:{state.step}:{mv_id}",
        mv_id=mv_id,
        gap_id=selected_gap.gap_id,
        gap_index=selected_gap.index,
        front_vehicle_id=selected_gap.front_vehicle_id,
        rear_vehicle_id=selected_gap.rear_vehicle_id,
        snapshot_step=gap_snapshot.step,
        snapshot_t=gap_snapshot.t,
        merge_point_x_global=merge_point_x_global,
        score_summary=score_summary,
        bundle_id=bundle_id,
    )
    context_after = _advance_round_context(
        context,
        selected_snapshot_gap
        if selected_snapshot_gap is not None
        else GapCandidate(
            gap_id=selected_gap.gap_id,
            index=selected_gap.index,
            front_vehicle_id=selected_front_vehicle_id,
            rear_vehicle_id=selected_rear_vehicle_id,
            front_x_global=0.0,
            rear_x_global=0.0,
            bumper_gap_m=0.0,
            effective_control_type="unknown",
        ),
    )
    plan_record = _plan_record(
        state,
        mv_id,
        mv_state,
        bundle,
        gap_plan,
        evaluation_result,
        round_id=round_id,
        round_order=round_order,
        context_before=context,
        context_after=context_after,
        allowed_gap_indices=allowed_gap_indices,
        filtered_by_frontier_gap_indices=filtered_by_frontier_gap_indices,
        required_longitudinal_gap_m=required_longitudinal_gap_m,
        reason=reason,
    )
    return _PlannedMVResult(
        mv_id=mv_id,
        bundle=bundle,
        gap_plan=gap_plan,
        plan_record=plan_record,
        gap_eval_records=gap_eval_records,
        selected_gap_index=selected_gap.index,
        selected_boundary_vehicle_ids=(selected_rear_vehicle_id, selected_front_vehicle_id),
        reason=reason,
    )


def _replace_owner_bundle_in_round_update(
    update: _RoundUpdate,
    *,
    mv_id: str,
    bundle: OneStepPlanBundle,
    gap_plan: GapPlan,
    round_id: str,
    round_order: int,
) -> None:
    previous_mv_state = update.mv_plan_states[mv_id]
    previous_bundle_id = previous_mv_state.active_bundle_id
    release_record = _release_existing_bundle_for_owner_in_round_update(
        update,
        mv_id=mv_id,
        reason="bundle_replaced_by_new_trigger_plan",
        step=bundle.start_step,
        t=bundle.start_t,
        replaced_bundle_id=bundle.bundle_id,
        round_id=round_id,
        round_order=round_order,
    )
    if release_record is not None:
        update.bundle_lifecycle_records.append(release_record)
    update.onestep_plan_bundles[bundle.bundle_id] = bundle
    update.gap_plans[gap_plan.plan_id] = gap_plan
    for vehicle_id in bundle.controlled_vehicle_ids:
        role = str(bundle.controlled_roles_by_vehicle_id[vehicle_id])
        existing = update.controlled_vehicle_states.get(vehicle_id)
        if existing is not None and existing.owner_mv_id != mv_id:
            raise ValueError(
                "stage2 controlled vehicle ownership conflict: "
                f"{vehicle_id} already owned by {existing.owner_mv_id}"
            )
        update.controlled_vehicle_states[vehicle_id] = OneStepControlledVehicleState(
            vehicle_id=vehicle_id,
            owner_mv_id=mv_id,
            bundle_id=bundle.bundle_id,
            role=role,
            controlled_since_step=bundle.start_step,
        )
    update.mv_plan_states[mv_id] = replace(
        previous_mv_state,
        current_plan_gap=bundle.selected_gap,
        current_plan_id=gap_plan.plan_id,
        active_bundle_id=bundle.bundle_id,
        last_plan_step=bundle.start_step,
        last_plan_t=bundle.start_t,
    )
    update.bundle_lifecycle_records.append(
        _bundle_lifecycle_record(
            action="bundle_created",
            bundle=bundle,
            gap_plan_id=gap_plan.plan_id,
            reason="onestep_stage2_trigger_plan",
            step=bundle.start_step,
            t=bundle.start_t,
            replaced_bundle_id=previous_bundle_id,
            round_id=round_id,
            round_order=round_order,
        )
    )


def _clear_owner_bundle_in_round_update(
    update: _RoundUpdate,
    *,
    mv_id: str,
    reason: str,
    step: int,
    t: float,
    round_id: str,
    round_order: int,
) -> None:
    previous_mv_state = update.mv_plan_states[mv_id]
    release_record = _release_existing_bundle_for_owner_in_round_update(
        update,
        mv_id=mv_id,
        reason=reason,
        step=step,
        t=t,
        round_id=round_id,
        round_order=round_order,
    )
    if release_record is not None:
        update.bundle_lifecycle_records.append(release_record)
    for plan_id, plan in tuple(update.gap_plans.items()):
        if plan.mv_id == mv_id:
            update.gap_plans.pop(plan_id, None)
    update.mv_plan_states[mv_id] = replace(
        previous_mv_state,
        current_plan_gap=None,
        current_plan_id=None,
        active_bundle_id=None,
        last_plan_step=step,
        last_plan_t=t,
    )


def _release_existing_bundle_for_owner_in_round_update(
    update: _RoundUpdate,
    *,
    mv_id: str,
    reason: str,
    step: int,
    t: float,
    replaced_bundle_id: str | None = None,
    round_id: str | None = None,
    round_order: int | None = None,
) -> Mapping[str, Any] | None:
    previous_mv_state = update.mv_plan_states[mv_id]
    previous_bundle_id = previous_mv_state.active_bundle_id
    if previous_bundle_id is None:
        return None
    bundle = update.onestep_plan_bundles.pop(previous_bundle_id, None)
    if bundle is None:
        return None
    for vehicle_id, controlled_state in tuple(update.controlled_vehicle_states.items()):
        if controlled_state.bundle_id == previous_bundle_id:
            update.controlled_vehicle_states.pop(vehicle_id, None)
    for plan_id, plan in tuple(update.gap_plans.items()):
        if plan.mv_id == mv_id:
            update.gap_plans.pop(plan_id, None)
    return _bundle_lifecycle_record(
        action="bundle_released",
        bundle=bundle,
        gap_plan_id=previous_mv_state.current_plan_id,
        reason=reason,
        step=step,
        t=t,
        replaced_bundle_id=replaced_bundle_id,
        round_id=round_id,
        round_order=round_order,
    )


def _ownership_blocked_vehicle_ids_for_mv(
    update: _RoundUpdate,
    mv_id: str,
) -> tuple[str, ...]:
    blocked: set[str] = set()
    current_bundle_id = update.mv_plan_states[mv_id].active_bundle_id
    for vehicle_id, controlled_state in update.controlled_vehicle_states.items():
        if controlled_state.bundle_id == current_bundle_id:
            continue
        if controlled_state.owner_mv_id == mv_id:
            continue
        blocked.add(vehicle_id)
    return tuple(sorted(blocked))


def _bundle_shape_for_selected_row(row: GapEvaluationRow) -> str:
    branch = row.controllability_branch
    if branch == CONTROLLABILITY_BRANCH_A:
        return BUNDLE_SHAPE_MV_FRONT_REAR
    if branch == CONTROLLABILITY_BRANCH_B:
        return BUNDLE_SHAPE_MV_FRONT
    if branch == CONTROLLABILITY_BRANCH_C:
        return BUNDLE_SHAPE_MV_REAR
    if branch == CONTROLLABILITY_BRANCH_D:
        return BUNDLE_SHAPE_MV_ONLY
    raise ValueError(f"unsupported controllability branch: {branch}")


def _controlled_roles_for_bundle_shape(
    mv_id: str,
    selected_front_vehicle_id: str,
    selected_rear_vehicle_id: str,
    bundle_shape: str,
) -> tuple[tuple[str, str], ...]:
    roles: list[tuple[str, str]] = [(mv_id, "mv")]
    if bundle_shape in (BUNDLE_SHAPE_MV_FRONT_REAR, BUNDLE_SHAPE_MV_REAR):
        roles.append((selected_rear_vehicle_id, "rear"))
    if bundle_shape in (BUNDLE_SHAPE_MV_FRONT_REAR, BUNDLE_SHAPE_MV_FRONT):
        roles.append((selected_front_vehicle_id, "front"))
    if bundle_shape not in {
        BUNDLE_SHAPE_MV_FRONT_REAR,
        BUNDLE_SHAPE_MV_FRONT,
        BUNDLE_SHAPE_MV_REAR,
        BUNDLE_SHAPE_MV_ONLY,
    }:
        raise ValueError(f"unsupported bundle shape: {bundle_shape}")
    return tuple(roles)


def _boundary_state_by_controlled_roles(
    state: SimulationState,
    controlled_roles: tuple[tuple[str, str], ...],
) -> MappingProxyType[str, OneStepBoundaryState]:
    return MappingProxyType(
        {
            vehicle_id: OneStepBoundaryState(
                vehicle_id=vehicle_id,
                x_global=float(state.vehicle_states[vehicle_id].x_global),
                v=float(state.vehicle_states[vehicle_id].v),
                a=float(state.vehicle_states[vehicle_id].a),
            )
            for vehicle_id, _ in controlled_roles
        }
    )


def _bundle_lifecycle_record(
    *,
    action: str,
    bundle: OneStepPlanBundle,
    gap_plan_id: str | None,
    reason: str,
    step: int,
    t: float,
    replaced_bundle_id: str | None = None,
    round_id: str | None = None,
    round_order: int | None = None,
) -> Mapping[str, Any]:
    return {
        "bundle_action": action,
        "bundle_id": bundle.bundle_id,
        "replaced_bundle_id": replaced_bundle_id,
        "mv_id": bundle.mv_id,
        "gap_plan_id": gap_plan_id,
        "bundle_shape": bundle.bundle_shape,
        "controlled_vehicle_ids": list(bundle.controlled_vehicle_ids),
        "controlled_roles_by_vehicle_id": dict(bundle.controlled_roles_by_vehicle_id),
        "gap_boundary_vehicle_ids": [
            bundle.selected_rear_vehicle_id,
            bundle.selected_front_vehicle_id,
        ],
        "selected_front_vehicle_id": bundle.selected_front_vehicle_id,
        "selected_rear_vehicle_id": bundle.selected_rear_vehicle_id,
        "selected_gap": bundle.selected_gap,
        "reason": reason,
        "round_id": round_id,
        "round_order": round_order,
        "step": step,
        "t": t,
    }


def _gap_lock_record(
    state: SimulationState,
    mv_id: str,
    mv_state: MVPlanState,
    *,
    round_id: str,
    round_order: int,
    context_before: _RoundContext,
    context_after: _RoundContext,
    reason: str,
) -> Mapping[str, Any]:
    return {
        "mv_id": mv_id,
        "zone_state": mv_state.zone_state,
        "current_plan_gap": mv_state.current_plan_gap,
        "locked_gap": mv_state.locked_gap,
        "selected_gap": mv_state.locked_gap,
        "locked_plan_id": mv_state.locked_plan_id,
        "active_bundle_id": mv_state.active_bundle_id,
        "round_id": round_id,
        "round_order": round_order,
        "tail_frontier_gap_index_before": context_before.tail_frontier_gap_index,
        "tail_frontier_gap_index_after": context_after.tail_frontier_gap_index,
        "selected_gaps_round_before": list(context_before.selected_gap_indices),
        "selected_gaps_round_after": list(context_after.selected_gap_indices),
        "uncontrollable_vehicles_round_before": list(
            context_before.uncontrollable_vehicle_ids
        ),
        "uncontrollable_vehicles_round_after": list(
            context_after.uncontrollable_vehicle_ids
        ),
        "reason": reason,
        "step": state.step,
        "t": state.t,
    }


def _plan_record(
    state: SimulationState,
    mv_id: str,
    mv_state: MVPlanState,
    bundle: OneStepPlanBundle,
    gap_plan: GapPlan,
    evaluation_result: Stage2OneStepEvaluation,
    *,
    round_id: str,
    round_order: int,
    context_before: _RoundContext,
    context_after: _RoundContext,
    allowed_gap_indices: tuple[int, ...],
    filtered_by_frontier_gap_indices: tuple[int, ...],
    required_longitudinal_gap_m: float,
    reason: str,
) -> Mapping[str, Any]:
    best_row = _selected_row(evaluation_result.evaluation.gap_rows)
    return {
        "mv_id": mv_id,
        "mv_x_global": float(state.vehicle_states[mv_id].x_global),
        "bundle_id": bundle.bundle_id,
        "gap_plan_id": gap_plan.plan_id,
        "origin_x_global": bundle.origin_x_global,
        "zone_state": mv_state.zone_state,
        "current_plan_gap": bundle.selected_gap,
        "locked_gap": mv_state.locked_gap,
        "selected_gap": bundle.selected_gap,
        "selected_vehicle_ids": list(bundle.selected_vehicle_ids),
        "bundle_shape": bundle.bundle_shape,
        "controlled_vehicle_ids": list(bundle.controlled_vehicle_ids),
        "controlled_roles_by_vehicle_id": dict(bundle.controlled_roles_by_vehicle_id),
        "gap_boundary_vehicle_ids": [
            bundle.selected_rear_vehicle_id,
            bundle.selected_front_vehicle_id,
        ],
        "best_gap_id": bundle.best_gap.gap_id,
        "kernel_gap_index": bundle.best_gap.index,
        "gap_index": bundle.selected_gap.index,
        "best_gap_interval_local": [
            float(bundle.best_gap.x_rear),
            float(bundle.best_gap.x_front),
        ],
        "selected_front_vehicle_id": bundle.selected_front_vehicle_id,
        "selected_rear_vehicle_id": bundle.selected_rear_vehicle_id,
        "boundary_state_by_vehicle_id": {
            vehicle_id: {
                "x_global": boundary_state.x_global,
                "v": boundary_state.v,
                "a": boundary_state.a,
            }
            for vehicle_id, boundary_state in bundle.boundary_state_by_vehicle_id.items()
        },
        "required_longitudinal_gap_m": required_longitudinal_gap_m,
        "delta_f_star": bundle.best_score.delta_f_star,
        "delta_r_star": bundle.best_score.delta_r_star,
        "d_i": bundle.best_score.d_i,
        "t_m": bundle.best_score.t_m,
        "p_m_local": bundle.best_score.p_m,
        "p_m_global": bundle.merge_point_x_global,
        "J": bundle.best_score.J,
        "score": bundle.best_score.J,
        "controllability_branch": best_row.controllability_branch,
        "round_id": round_id,
        "round_order": round_order,
        "tail_frontier_gap_index_before": context_before.tail_frontier_gap_index,
        "tail_frontier_gap_index_after": context_after.tail_frontier_gap_index,
        "selected_gaps_round_before": list(context_before.selected_gap_indices),
        "selected_gaps_round_after": list(context_after.selected_gap_indices),
        "uncontrollable_vehicles_round_before": list(
            context_before.uncontrollable_vehicle_ids
        ),
        "uncontrollable_vehicles_round_after": list(
            context_after.uncontrollable_vehicle_ids
        ),
        "allowed_gap_indices": list(allowed_gap_indices),
        "filtered_by_frontier_gap_indices": list(filtered_by_frontier_gap_indices),
        "reason": reason,
        "step": state.step,
        "t": state.t,
    }


def _no_gap_plan_record(
    state: SimulationState,
    mv_id: str,
    mv_state: MVPlanState,
    *,
    round_id: str,
    round_order: int,
    context_before: _RoundContext,
    allowed_gap_indices: tuple[int, ...],
    filtered_by_frontier_gap_indices: tuple[int, ...],
    reason: str,
) -> Mapping[str, Any]:
    return {
        "mv_id": mv_id,
        "mv_x_global": float(state.vehicle_states[mv_id].x_global),
        "bundle_id": None,
        "origin_x_global": float(state.vehicle_states[mv_id].x_global),
        "zone_state": mv_state.zone_state,
        "current_plan_gap": None,
        "locked_gap": mv_state.locked_gap,
        "selected_gap": None,
        "selected_vehicle_ids": [],
        "bundle_shape": None,
        "controlled_vehicle_ids": [],
        "controlled_roles_by_vehicle_id": {},
        "gap_boundary_vehicle_ids": [],
        "best_gap_id": None,
        "kernel_gap_index": None,
        "gap_index": None,
        "best_gap_interval_local": [],
        "selected_front_vehicle_id": None,
        "selected_rear_vehicle_id": None,
        "boundary_state_by_vehicle_id": {},
        "round_id": round_id,
        "round_order": round_order,
        "tail_frontier_gap_index_before": context_before.tail_frontier_gap_index,
        "tail_frontier_gap_index_after": context_before.tail_frontier_gap_index,
        "selected_gaps_round_before": list(context_before.selected_gap_indices),
        "selected_gaps_round_after": list(context_before.selected_gap_indices),
        "uncontrollable_vehicles_round_before": list(
            context_before.uncontrollable_vehicle_ids
        ),
        "uncontrollable_vehicles_round_after": list(
            context_before.uncontrollable_vehicle_ids
        ),
        "allowed_gap_indices": list(allowed_gap_indices),
        "filtered_by_frontier_gap_indices": list(filtered_by_frontier_gap_indices),
        "score": None,
        "reason": reason,
        "step": state.step,
        "t": state.t,
    }


def _gap_eval_records(
    state: SimulationState,
    mv_id: str,
    *,
    bundle_id: str,
    evaluation_rows: tuple[GapEvaluationRow, ...],
    evaluation_result: Stage2OneStepEvaluation,
    round_id: str,
    round_order: int,
    context_before: _RoundContext,
    filtered_by_frontier_gap_indices: tuple[int, ...],
) -> tuple[Mapping[str, Any], ...]:
    runtime_by_kernel = dict(
        evaluation_result.local_frame["runtime_gap_index_by_kernel_index"]
    )
    return tuple(
        {
            "mv_id": mv_id,
            "bundle_id": bundle_id,
            "gap_id": row.gap_id,
            "kernel_gap_index": row.gap_index,
            "gap_index": runtime_by_kernel[row.gap_index],
            "front_vehicle_id": row.front_vehicle_id,
            "rear_vehicle_id": row.rear_vehicle_id,
            "front_controllable": row.front_controllable,
            "rear_controllable": row.rear_controllable,
            "controllability_branch": row.controllability_branch,
            "reachable": row.reachable,
            "coop_feasible": row.coop_feasible,
            "included_in_scoring": row.included_in_scoring,
            "delta_f_star": row.delta_f_star,
            "delta_r_star": row.delta_r_star,
            "d_i": row.d_i,
            "t_m": row.t_m,
            "p_m": row.p_m,
            "J": row.J,
            "failure_reason": row.failure_reason,
            "is_selected": row.is_selected,
            "round_id": round_id,
            "round_order": round_order,
            "filtered_by_frontier": False,
            "filtered_by_frontier_gap_indices": list(filtered_by_frontier_gap_indices),
            "tail_frontier_gap_index_before": context_before.tail_frontier_gap_index,
            "reason": "onestep_stage2_gap_evaluation",
            "step": state.step,
            "t": state.t,
        }
        for row in evaluation_rows
    )


def _selected_row(rows: tuple[GapEvaluationRow, ...]) -> GapEvaluationRow:
    for row in rows:
        if row.is_selected:
            return row
    raise ValueError("selected OneStep gap row is required")


def _runtime_gap_index_for_kernel_index(
    local_frame: Mapping[str, Any],
    kernel_gap_index: int,
) -> int:
    return int(dict(local_frame["runtime_gap_index_by_kernel_index"])[kernel_gap_index])


def _gap_candidate_by_index(
    gap_snapshot: GapSnapshot,
    gap_index: int | None,
) -> GapCandidate | None:
    if gap_index is None:
        return None
    for gap in gap_snapshot.gaps:
        if gap.index == gap_index:
            return gap
    return None


def _gap_candidate_for_ref(
    gap_snapshot: GapSnapshot,
    gap_ref: GapRef,
) -> GapCandidate | None:
    return _gap_candidate_by_index(gap_snapshot, gap_ref.index)
