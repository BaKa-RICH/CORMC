from __future__ import annotations

from typing import Any, Mapping

from cormc.simulation_core.pre_freeze import SimulationState
from cormc.simulation_core.commit import CandidateKinematics

from cormc.onestep.rolling.state import (
    GapRef,
    GapCandidate,
    GapSnapshot,
    RampMergeRuntimeState,
    SafetyCheckResult,
    TriggerDecision,
)


def build_runtime_state_event(
    state: SimulationState,
    previous_runtime: RampMergeRuntimeState | None,
    runtime: RampMergeRuntimeState,
) -> dict[str, Any]:
    previous_mv_ids = (
        set(previous_runtime.mv_plan_states)
        if previous_runtime is not None
        else set()
    )
    current_mv_ids = set(runtime.mv_plan_states)
    return _event(
        state,
        module="ramp_merge_runtime",
        event_type="ramp_merge_runtime",
        reason="runtime_state_initialized_or_refreshed",
        vehicle_ids=tuple(runtime.mv_plan_states),
        payload={
            "runtime_version": runtime.version,
            "runtime_mv_ids": list(runtime.mv_plan_states),
            "new_runtime_mv_ids": sorted(current_mv_ids - previous_mv_ids),
            "removed_runtime_mv_ids": sorted(previous_mv_ids - current_mv_ids),
            "zone_state_by_mv": {
                vehicle_id: mv_state.zone_state
                for vehicle_id, mv_state in runtime.mv_plan_states.items()
            },
            "danger_vehicle_ids": list(runtime.danger_vehicle_ids),
            "planned_trajectory_count": len(runtime.planned_trajectories),
            "planned_trajectory_ids": list(runtime.planned_trajectories),
            "last_gap_snapshot_step": (
                runtime.last_gap_snapshot.step
                if runtime.last_gap_snapshot is not None
                else None
            ),
            "gap_logic_executed": (
                runtime.last_gap_snapshot is not None
                and runtime.last_gap_snapshot.step == state.step
            ),
        },
    )


def build_default_motion_event(
    state: SimulationState,
    candidates: Mapping[str, tuple[CandidateKinematics, ...]],
) -> dict[str, Any]:
    candidate_ids = {
        vehicle_id: [candidate.candidate_id for candidate in vehicle_candidates]
        for vehicle_id, vehicle_candidates in candidates.items()
    }
    return _event(
        state,
        module="ramp_merge_motion",
        event_type="ramp_merge_default_motion",
        reason="batch_a_constant_speed_straight_motion",
        vehicle_ids=tuple(candidates),
        payload={
            "candidate_ids": candidate_ids,
            "candidate_count": sum(len(value) for value in candidates.values()),
            "lane_change_executed": False,
            "gap_logic_executed": False,
        },
    )


def build_zone_state_event(
    state: SimulationState,
    runtime: RampMergeRuntimeState,
) -> dict[str, Any]:
    return _event(
        state,
        module="ramp_merge_planner",
        event_type="ramp_merge_zone_state",
        reason="zone_state_refreshed_from_geometry",
        vehicle_ids=tuple(runtime.mv_plan_states),
        payload={
            "zone_state_by_mv": {
                vehicle_id: mv_state.zone_state
                for vehicle_id, mv_state in runtime.mv_plan_states.items()
            },
            "merge_state_by_mv": {
                vehicle_id: mv_state.merge_state
                for vehicle_id, mv_state in runtime.mv_plan_states.items()
            },
            "current_plan_gap_by_mv": {
                vehicle_id: _gap_ref_payload(mv_state.current_plan_gap)
                for vehicle_id, mv_state in runtime.mv_plan_states.items()
            },
            "locked_gap_by_mv": {
                vehicle_id: _gap_ref_payload(mv_state.locked_gap)
                for vehicle_id, mv_state in runtime.mv_plan_states.items()
            },
            "planned_trajectory_id_by_mv": {
                vehicle_id: mv_state.planned_trajectory_id
                for vehicle_id, mv_state in runtime.mv_plan_states.items()
            },
        },
    )


def build_safety_event(
    state: SimulationState,
    safety_result: SafetyCheckResult,
) -> dict[str, Any]:
    return _event(
        state,
        module="ramp_merge_safety",
        event_type="ramp_merge_safety",
        reason="temporary_safety_check",
        vehicle_ids=safety_result.danger_vehicle_ids,
        payload={
            "safety_alert": safety_result.safety_alert,
            "danger_vehicle_ids": list(safety_result.danger_vehicle_ids),
            "danger_pairs": [dict(pair) for pair in safety_result.danger_pairs],
            "ttc_threshold_s": safety_result.ttc_threshold_s,
            "min_gap_m": safety_result.min_gap_m,
        },
    )


def build_trigger_event(
    state: SimulationState,
    trigger_decision: TriggerDecision,
    *,
    gap_identification_executed: bool,
) -> dict[str, Any]:
    return _event(
        state,
        module="ramp_merge_planner",
        event_type="ramp_merge_trigger",
        reason=trigger_decision.trigger_reason,
        vehicle_ids=trigger_decision.entry_vehicle_ids,
        payload={
            "trigger_plan": trigger_decision.trigger_plan,
            "trigger_reason": trigger_decision.trigger_reason,
            "active_trigger_reasons": list(trigger_decision.active_trigger_reasons),
            "periodic_due": trigger_decision.periodic_due,
            "safety_alert": trigger_decision.safety_alert,
            "entry_plan_trigger": trigger_decision.entry_plan_trigger,
            "entry_vehicle_ids": list(trigger_decision.entry_vehicle_ids),
            "gap_identification_executed": gap_identification_executed,
            "next_plan_time": trigger_decision.planner_state.next_plan_time,
            "last_trigger_reason": trigger_decision.planner_state.last_trigger_reason,
        },
    )


def build_gap_snapshot_event(
    state: SimulationState,
    gap_snapshot: GapSnapshot,
) -> dict[str, Any]:
    vehicle_ids: list[str] = []
    for gap in gap_snapshot.gaps:
        for vehicle_id in (gap.rear_vehicle_id, gap.front_vehicle_id):
            if vehicle_id not in vehicle_ids:
                vehicle_ids.append(vehicle_id)
    return _event(
        state,
        module="ramp_merge_gaps",
        event_type="ramp_merge_gap_snapshot",
        reason="triggered_lane_2_gap_identification",
        vehicle_ids=tuple(vehicle_ids),
        payload={
            "step": gap_snapshot.step,
            "t": gap_snapshot.t,
            "lane_id": gap_snapshot.lane_id,
            "gap_count": len(gap_snapshot.gaps),
            "danger_vehicle_ids": list(gap_snapshot.danger_vehicle_ids),
            "gaps": [_gap_candidate_payload(gap) for gap in gap_snapshot.gaps],
        },
    )


def build_gap_selection_event(
    state: SimulationState,
    record: Mapping[str, Any],
) -> dict[str, Any]:
    return _event(
        state,
        module="ramp_merge_planner",
        event_type="ramp_merge_gap_selection",
        reason=str(record.get("reason") or "gap_selection"),
        vehicle_ids=(str(record.get("mv_id")),),
        payload=_planner_record_payload(record),
    )


def build_gap_lock_event(
    state: SimulationState,
    record: Mapping[str, Any],
) -> dict[str, Any]:
    return _event(
        state,
        module="ramp_merge_planner",
        event_type="ramp_merge_gap_lock",
        reason=str(record.get("reason") or "gap_lock"),
        vehicle_ids=(str(record.get("mv_id")),),
        payload=_planner_record_payload(record),
    )


def build_merge_check_event(
    state: SimulationState,
    merge_check_result: Any,
) -> dict[str, Any]:
    locked_gap = getattr(merge_check_result, "locked_gap", None)
    return _event(
        state,
        module="ramp_merge_planner",
        event_type="ramp_merge_merge_check",
        reason=str(getattr(merge_check_result, "reason", "merge_check")),
        vehicle_ids=(str(getattr(merge_check_result, "mv_id")),),
        payload={
            "mv_id": getattr(merge_check_result, "mv_id"),
            "zone_state": getattr(merge_check_result, "zone_state"),
            "current_plan_gap": None,
            "locked_gap": _gap_ref_payload(locked_gap),
            "selected_gap": _gap_ref_payload(locked_gap),
            "trajectory_id": None,
            "trajectory_kind": None,
            "progress_step": None,
            "duration_steps": None,
            "merge_check_result": bool(getattr(merge_check_result, "result")),
            "rule": getattr(merge_check_result, "rule"),
            "reason": getattr(merge_check_result, "reason"),
        },
    )


def build_trajectory_event(
    state: SimulationState,
    record: Mapping[str, Any],
) -> dict[str, Any]:
    return _event(
        state,
        module="ramp_merge_motion",
        event_type="ramp_merge_trajectory",
        reason=str(record.get("reason") or "trajectory"),
        vehicle_ids=(str(record.get("mv_id")),),
        payload=_planner_record_payload(record),
    )


def build_onestep_stage2_plan_event(
    state: SimulationState,
    record: Mapping[str, Any],
) -> dict[str, Any]:
    return _event(
        state,
        module="ramp_merge_planner",
        event_type="ramp_merge_onestep_stage2_plan",
        reason=str(record.get("reason") or "onestep_stage2_plan"),
        vehicle_ids=(str(record.get("mv_id")),),
        payload={
            **_planner_record_payload(record),
            "mv_x_global": record.get("mv_x_global"),
            "origin_x_global": record.get("origin_x_global"),
            "bundle_id": record.get("bundle_id"),
            "best_gap_id": record.get("best_gap_id"),
            "best_gap_interval_local": list(record.get("best_gap_interval_local") or []),
            "selected_vehicle_ids": list(record.get("selected_vehicle_ids") or []),
            "bundle_shape": record.get("bundle_shape"),
            "controlled_vehicle_ids": list(record.get("controlled_vehicle_ids") or []),
            "controlled_roles_by_vehicle_id": dict(
                record.get("controlled_roles_by_vehicle_id") or {}
            ),
            "gap_boundary_vehicle_ids": list(record.get("gap_boundary_vehicle_ids") or []),
            "selected_front_vehicle_id": record.get("selected_front_vehicle_id"),
            "selected_rear_vehicle_id": record.get("selected_rear_vehicle_id"),
            "boundary_state_by_vehicle_id": dict(record.get("boundary_state_by_vehicle_id") or {}),
            "required_longitudinal_gap_m": record.get("required_longitudinal_gap_m"),
            "delta_f_star": record.get("delta_f_star"),
            "delta_r_star": record.get("delta_r_star"),
            "d_i": record.get("d_i"),
            "t_m": record.get("t_m"),
            "p_m_local": record.get("p_m_local"),
            "p_m_global": record.get("p_m_global"),
            "J": record.get("J"),
            "gap_index": record.get("gap_index"),
            "kernel_gap_index": record.get("kernel_gap_index"),
            "gap_plan_id": record.get("gap_plan_id"),
            "controllability_branch": record.get("controllability_branch"),
            "round_id": record.get("round_id"),
            "round_order": record.get("round_order"),
            "tail_frontier_gap_index_before": record.get("tail_frontier_gap_index_before"),
            "tail_frontier_gap_index_after": record.get("tail_frontier_gap_index_after"),
            "selected_gaps_round_before": list(record.get("selected_gaps_round_before") or []),
            "selected_gaps_round_after": list(record.get("selected_gaps_round_after") or []),
            "uncontrollable_vehicles_round_before": list(
                record.get("uncontrollable_vehicles_round_before") or []
            ),
            "uncontrollable_vehicles_round_after": list(
                record.get("uncontrollable_vehicles_round_after") or []
            ),
            "allowed_gap_indices": list(record.get("allowed_gap_indices") or []),
            "filtered_by_frontier_gap_indices": list(
                record.get("filtered_by_frontier_gap_indices") or []
            ),
        },
    )


def build_onestep_stage2_gap_eval_event(
    state: SimulationState,
    record: Mapping[str, Any],
) -> dict[str, Any]:
    return _event(
        state,
        module="ramp_merge_planner",
        event_type="ramp_merge_onestep_stage2_gap_eval",
        reason=str(record.get("reason") or "onestep_stage2_gap_evaluation"),
        vehicle_ids=(str(record.get("mv_id")),),
        payload={
            "mv_id": record.get("mv_id"),
            "bundle_id": record.get("bundle_id"),
            "gap_id": record.get("gap_id"),
            "gap_index": record.get("gap_index"),
            "kernel_gap_index": record.get("kernel_gap_index"),
            "front_vehicle_id": record.get("front_vehicle_id"),
            "rear_vehicle_id": record.get("rear_vehicle_id"),
            "front_controllable": record.get("front_controllable"),
            "rear_controllable": record.get("rear_controllable"),
            "controllability_branch": record.get("controllability_branch"),
            "reachable": record.get("reachable"),
            "coop_feasible": record.get("coop_feasible"),
            "included_in_scoring": record.get("included_in_scoring"),
            "delta_f_star": record.get("delta_f_star"),
            "delta_r_star": record.get("delta_r_star"),
            "d_i": record.get("d_i"),
            "t_m": record.get("t_m"),
            "p_m": record.get("p_m"),
            "J": record.get("J"),
            "failure_reason": record.get("failure_reason"),
            "is_selected": record.get("is_selected"),
            "round_id": record.get("round_id"),
            "round_order": record.get("round_order"),
            "filtered_by_frontier": record.get("filtered_by_frontier"),
            "filtered_by_frontier_gap_indices": list(
                record.get("filtered_by_frontier_gap_indices") or []
            ),
            "tail_frontier_gap_index_before": record.get("tail_frontier_gap_index_before"),
        },
    )


def build_onestep_stage2_longitudinal_completion_event(
    state: SimulationState,
    record: Mapping[str, Any],
) -> dict[str, Any]:
    return _event(
        state,
        module="ramp_merge_motion",
        event_type="ramp_merge_onestep_longitudinal_completion",
        reason=str(record.get("reason") or "onestep_stage2_longitudinal_completion"),
        vehicle_ids=(str(record.get("mv_id")),),
        payload={
            "mv_id": record.get("mv_id"),
            "bundle_id": record.get("bundle_id"),
            "merge_point_x_global": record.get("merge_point_x_global"),
            "x_global_next": record.get("x_global_next"),
            "front_gap_m": record.get("front_gap_m"),
            "rear_gap_m": record.get("rear_gap_m"),
            "gap_center_x_global": record.get("gap_center_x_global"),
            "required_longitudinal_gap_m": record.get("required_longitudinal_gap_m"),
            "merge_point_reached": record.get("merge_point_reached"),
            "longitudinal_ready_rule": record.get("longitudinal_ready_rule"),
            "selected_vehicle_ids": list(record.get("selected_vehicle_ids") or []),
            "longitudinal_completed": record.get("longitudinal_completed"),
            "lateral_started": record.get("lateral_started"),
            "trajectory_id": record.get("trajectory_id"),
            "active_lateral_trajectory_id": record.get("active_lateral_trajectory_id"),
            "trajectory_kind": record.get("trajectory_kind"),
            "current_plan_gap": _gap_like_payload(record.get("current_plan_gap")),
            "locked_gap": _gap_like_payload(record.get("locked_gap")),
            "selected_gap": _gap_like_payload(record.get("selected_gap")),
            "locked_plan_id": record.get("locked_plan_id"),
            "source_plan_id": record.get("source_plan_id"),
            "start_y": record.get("start_y"),
            "target_y": record.get("target_y"),
            "duration_steps": record.get("duration_steps"),
            "merge_state": record.get("merge_state"),
            "reason": record.get("reason"),
        },
    )


def build_onestep_stage2_bundle_lifecycle_event(
    state: SimulationState,
    record: Mapping[str, Any],
) -> dict[str, Any]:
    reason = str(record.get("reason") or "")
    module = (
        "ramp_merge_motion"
        if reason in {
            "onestep_stage2_longitudinal_ready_release",
            "onestep_stage2_lateral_start_release",
        }
        else "ramp_merge_planner"
    )
    return _event(
        state,
        module=module,
        event_type="ramp_merge_onestep_stage2_bundle_lifecycle",
        reason=str(record.get("reason") or "onestep_stage2_bundle_lifecycle"),
        vehicle_ids=(str(record.get("mv_id")),),
        payload={
            "bundle_action": record.get("bundle_action"),
            "bundle_id": record.get("bundle_id"),
            "replaced_bundle_id": record.get("replaced_bundle_id"),
            "mv_id": record.get("mv_id"),
            "gap_plan_id": record.get("gap_plan_id"),
            "bundle_shape": record.get("bundle_shape"),
            "controlled_vehicle_ids": list(record.get("controlled_vehicle_ids") or []),
            "controlled_roles_by_vehicle_id": dict(
                record.get("controlled_roles_by_vehicle_id") or {}
            ),
            "gap_boundary_vehicle_ids": list(
                record.get("gap_boundary_vehicle_ids") or []
            ),
            "selected_front_vehicle_id": record.get("selected_front_vehicle_id"),
            "selected_rear_vehicle_id": record.get("selected_rear_vehicle_id"),
            "selected_gap": _gap_like_payload(record.get("selected_gap")),
            "reason": record.get("reason"),
            "round_id": record.get("round_id"),
            "round_order": record.get("round_order"),
        },
    )


def build_merge_completion_event(
    state: SimulationState,
    record: Mapping[str, Any],
) -> dict[str, Any]:
    return _event(
        state,
        module="ramp_merge_motion",
        event_type="ramp_merge_merge_completion",
        reason=str(record.get("reason") or "merge_completion"),
        vehicle_ids=(str(record.get("mv_id")),),
        payload=_planner_record_payload(record),
    )


def _gap_candidate_payload(gap: GapCandidate) -> dict[str, Any]:
    return {
        "gap_id": gap.gap_id,
        "index": gap.index,
        "front_vehicle_id": gap.front_vehicle_id,
        "rear_vehicle_id": gap.rear_vehicle_id,
        "front_x_global": gap.front_x_global,
        "rear_x_global": gap.rear_x_global,
        "bumper_gap_m": gap.bumper_gap_m,
        "effective_control_type": gap.effective_control_type,
    }


def _planner_record_payload(record: Mapping[str, Any]) -> dict[str, Any]:
    return {
        "mv_id": record.get("mv_id"),
        "zone_state": record.get("zone_state"),
        "current_plan_gap": _gap_like_payload(record.get("current_plan_gap")),
        "locked_gap": _gap_like_payload(record.get("locked_gap")),
        "selected_gap": _gap_like_payload(record.get("selected_gap")),
        "trajectory_id": record.get("trajectory_id"),
        "trajectory_kind": record.get("trajectory_kind"),
        "progress_step": record.get("progress_step"),
        "duration_steps": record.get("duration_steps"),
        "active_lateral_trajectory_id": record.get("active_lateral_trajectory_id"),
        "source_plan_id": record.get("source_plan_id"),
        "start_y": record.get("start_y"),
        "target_y": record.get("target_y"),
        "y_next": record.get("y_next"),
        "x_global_next": record.get("x_global_next"),
        "merge_check_result": record.get("merge_check_result"),
        "merge_completed": record.get("merge_completed"),
        "score": record.get("score"),
        "bundle_id": record.get("bundle_id"),
        "locked_plan_id": record.get("locked_plan_id"),
        "active_bundle_id": record.get("active_bundle_id"),
        "round_id": record.get("round_id"),
        "round_order": record.get("round_order"),
        "tail_frontier_gap_index_before": record.get("tail_frontier_gap_index_before"),
        "tail_frontier_gap_index_after": record.get("tail_frontier_gap_index_after"),
        "selected_gaps_round_before": list(record.get("selected_gaps_round_before") or []),
        "selected_gaps_round_after": list(record.get("selected_gaps_round_after") or []),
        "uncontrollable_vehicles_round_before": list(
            record.get("uncontrollable_vehicles_round_before") or []
        ),
        "uncontrollable_vehicles_round_after": list(
            record.get("uncontrollable_vehicles_round_after") or []
        ),
        "reason": record.get("reason"),
    }


def _gap_like_payload(value: Any) -> dict[str, Any] | None:
    if value is None:
        return None
    if isinstance(value, GapCandidate):
        return _gap_candidate_payload(value)
    if isinstance(value, GapRef):
        return _gap_ref_payload(value)
    return dict(value) if isinstance(value, Mapping) else None


def _gap_ref_payload(gap_ref: GapRef | None) -> dict[str, Any] | None:
    if gap_ref is None:
        return None
    return {
        "gap_id": gap_ref.gap_id,
        "index": gap_ref.index,
        "front_vehicle_id": gap_ref.front_vehicle_id,
        "rear_vehicle_id": gap_ref.rear_vehicle_id,
        "snapshot_step": gap_ref.snapshot_step,
        "snapshot_t": gap_ref.snapshot_t,
    }


def _event(
    state: SimulationState,
    *,
    module: str,
    event_type: str,
    reason: str,
    vehicle_ids: tuple[str, ...],
    payload: Mapping[str, Any],
) -> dict[str, Any]:
    return {
        "event_id": f"ramp-merge-batch-c:{state.step}:{event_type}",
        "step": state.step,
        "t": state.t,
        "module": module,
        "event_type": event_type,
        "vehicle_ids": list(vehicle_ids),
        "reason": reason,
        "result": "ok",
        "source": "ramp_merge_batch_c",
        "payload": dict(payload),
    }
