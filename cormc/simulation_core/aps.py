from __future__ import annotations

from dataclasses import dataclass, fields
from types import MappingProxyType
from typing import Any, Iterable, Mapping

from cormc.simulation_core.assignment_lifecycle import AssignmentStepView, assignment_lifecycle_manager
from cormc.simulation_core.pre_freeze import (
    DEFAULT_ROAD_GEOMETRY,
    LANE_2,
    ON_RAMP,
    ON_RAMP_MV_ROLE,
    RelationsSnapshot,
    RoadGeometryConfig,
    SimulationState,
    VehicleState,
    assert_x_plot_not_used_in_algorithm_path,
    build_prefreeze_workspace_from_scenario,
    freeze_simulation_state,
    refresh_relations_snapshot,
    resolve_aps_candidate_window,
    resolve_lane_2_gap_boundary_eligibility,
    resolve_lane_ordering_by_x_global,
    resolve_region,
)


APS_DECISION_INTERVAL_S = 3.0
APS_MIN_MERGE_TIME_GAP_S = 1.2
PAPER_VEHICLE_LENGTH_M = 4.0
EQ10_CFV_MIN_GAP_WEIGHT = 0.4
MIN_APS_SPEED_MPS = 1.0e-6


@dataclass(frozen=True)
class APSCandidatePrediction:
    vehicle_id: str
    x_global: float
    v: float
    predicted_x_global: float
    d_star: float


@dataclass(frozen=True)
class APSAssignment:
    mv_id: str
    clv_id: str
    cfv_id: str
    aps_case: str
    col_clv: bool
    col_cfv: bool
    desired_spacing_override: float | None
    t_star_mv: float
    d_star_clv: float
    d_star_cfv: float
    source: str = "aps_updated_this_step"
    status: str = "valid"
    valid_until_next_aps: bool = True

    def to_cache_value(self, *, t: float, step: int) -> dict[str, Any]:
        return {
            "mv_id": self.mv_id,
            "clv_id": self.clv_id,
            "cfv_id": self.cfv_id,
            "aps_case": self.aps_case,
            "col_clv": self.col_clv,
            "col_cfv": self.col_cfv,
            "desired_spacing_override": self.desired_spacing_override,
            "t_star_mv": self.t_star_mv,
            "t_mv_star": self.t_star_mv,
            "d_star_clv": self.d_star_clv,
            "d_star_cfv": self.d_star_cfv,
            "aps_min_merge_time_gap_s": APS_MIN_MERGE_TIME_GAP_S,
            "status": self.status,
            "created_at_t": t,
            "created_at_step": step,
            "source": self.source,
            "valid_until_next_aps": self.valid_until_next_aps,
            "staleness_policy": "valid_until_next_aps",
        }


@dataclass(frozen=True)
class APSCacheAction:
    mv_id: str
    action: str
    previous_cache_exists: bool
    cache_modified_in_p04: bool
    update_request: Mapping[str, Any] | None = None
    reason: str = "not_applicable"
    invalid_boundary_role: str | None = None
    invalid_boundary_id: str | None = None
    invalid_reason: str | None = None


@dataclass(frozen=True)
class APSRunResult:
    state: SimulationState
    relations: RelationsSnapshot
    actual_events: list[dict[str, Any]]
    actual_sanity_checks: list[dict[str, Any]]
    assignment_views: Mapping[str, AssignmentStepView]
    assignment_record_updates: Mapping[str, Mapping[str, Any]]
    cache_actions: tuple[APSCacheAction, ...]


def run_step4a_aps_for_scenario(
    scenario: str | dict[str, Any],
    *,
    geometry: RoadGeometryConfig = DEFAULT_ROAD_GEOMETRY,
) -> APSRunResult:
    workspace, config = build_prefreeze_workspace_from_scenario(scenario, geometry=geometry)
    state = freeze_simulation_state(workspace)
    relations = refresh_relations_snapshot(state, geometry=geometry)
    return run_step4a_aps(state, relations, config=config, geometry=geometry)


def run_step4a_aps(
    state: SimulationState,
    relations: RelationsSnapshot,
    *,
    config: dict[str, Any] | None = None,
    geometry: RoadGeometryConfig = DEFAULT_ROAD_GEOMETRY,
    eligible_mv_ids: Iterable[str] | None = None,
) -> APSRunResult:
    scenario_id = str((config or {}).get("scenario_id") or state.scenario_config_ref or "unknown")
    before_signature = _state_signature(state)
    events: list[dict[str, Any]] = []
    sanity_checks: list[dict[str, Any]] = []
    assignment_views: dict[str, AssignmentStepView] = {}
    assignment_record_updates: dict[str, Mapping[str, Any]] = {}
    cache_actions: list[APSCacheAction] = []
    last_aps_times = _last_aps_times(config or {})
    freeze_first_assignment = _freeze_first_aps_assignment_until_cmc(config or {})

    for mv_id in _step4a_mv_ids(state, eligible_mv_ids=eligible_mv_ids):
        mv_state = state.vehicle_states[mv_id]
        region = resolve_region(mv_state.x_global, mv_state.road_role, geometry=geometry)
        record_value = state.assignment_records_by_mv.get(mv_id)
        merge_zone_handoff = _is_merge_zone_handoff_refresh_candidate(
            state,
            mv_id,
            existing_cache=record_value,
            geometry=geometry,
        )
        if (
            (region.in_merging_zone and not merge_zone_handoff)
            or region.past_ramp_end
            or mv_state.merge_state == "executing"
        ):
            events.append(_handoff_event(state, mv_id, scenario_id=scenario_id))
            sanity_checks.append(
                _sanity(
                    state,
                    "aps_not_applicable",
                    "not_applicable",
                    vehicle_ids=(mv_id,),
                    scenario_id=scenario_id,
                    reason="handed_off_to_cmc",
                    payload={
                        "branch": "handed_off_to_cmc",
                        "cache_modified": False,
                        "new_assignment_created": False,
                    },
                )
            )
            continue

        if merge_zone_handoff:
            trigger = "merge_zone_handoff"
        else:
            trigger = resolve_aps_trigger(
                state,
                mv_id,
                last_aps_time=last_aps_times.get(mv_id),
                existing_cache=record_value,
                freeze_existing_assignment=freeze_first_assignment,
            )
        invalid_boundary = _cached_assignment_invalid_boundary(state, record_value)
        if record_value is not None and invalid_boundary is not None:
            trigger = "cached_gap_boundary_invalid"
        if trigger == "reuse_cache" and record_value is not None:
            record = assignment_lifecycle_manager.from_state_dict(record_value)
            view = assignment_lifecycle_manager.derive_step5_view(state, record)
            if view is not None:
                assignment_views[mv_id] = view
            cache_action = APSCacheAction(
                mv_id=mv_id,
                action="retain",
                previous_cache_exists=True,
                cache_modified_in_p04=False,
                reason="reuse_cache_until_next_APS",
            )
            cache_actions.append(cache_action)
            events.append(
                _aps_event(
                    state,
                    mv_id=mv_id,
                    related_vehicle_ids=(mv_id,),
                    scenario_id=scenario_id,
                    reason="reuse_cache",
                    source="first_version_engineering_patch",
                    payload={
                        "trigger": "reuse_cache",
                        "effective_assignment_source": "cache_reused",
                        "aps_executed": False,
                        "cache_modified": False,
                        "new_assignment_created": False,
                    },
                    is_engineering_patch=True,
                )
            )
            events.append(_cache_event(state, cache_action, scenario_id=scenario_id))
            continue

        aps_outcome = _run_fresh_aps(
            state,
            mv_id,
            trigger=trigger,
            existing_cache=record_value,
            invalid_boundary=invalid_boundary,
            geometry=geometry,
            scenario_id=scenario_id,
        )
        events.extend(aps_outcome["events"])
        cache_actions.extend(aps_outcome["cache_actions"])
        assignment_record_updates.update(aps_outcome.get("assignment_record_updates", {}))
        if aps_outcome.get("failure_sanity") is not None:
            sanity_checks.append(aps_outcome["failure_sanity"])
        if aps_outcome["assignment_view"] is not None:
            assignment_views[mv_id] = aps_outcome["assignment_view"]

    sanity_checks.extend(
        run_aps_assignment_sanity(
            state,
            scenario_id=scenario_id,
            state_unchanged=before_signature == _state_signature(state),
            has_eq10_wrong_vehicle=False,
            has_assignment_invalid=False,
            assignment_vehicle_ids=tuple(assignment_views),
        )
    )
    return APSRunResult(
        state=state,
        relations=relations,
        actual_events=events,
        actual_sanity_checks=sanity_checks,
        assignment_views=MappingProxyType(assignment_views),
        assignment_record_updates=MappingProxyType(assignment_record_updates),
        cache_actions=tuple(cache_actions),
    )


def resolve_aps_trigger(
    state: SimulationState,
    mv_id: str,
    *,
    last_aps_time: float | None,
    existing_cache: Mapping[str, Any] | None,
    aps_decision_interval_s: float = APS_DECISION_INTERVAL_S,
    freeze_existing_assignment: bool = False,
) -> str:
    if existing_cache is None:
        return "first_APS"
    if freeze_existing_assignment:
        return "reuse_cache"
    last_update_t = last_aps_time
    if last_update_t is None and existing_cache.get("created_at_t") is not None:
        last_update_t = float(existing_cache["created_at_t"])
    if last_update_t is None:
        return "APS_due"
    if state.t - float(last_update_t) >= aps_decision_interval_s:
        return "APS_due"
    return "reuse_cache"


def collect_aps_candidates(
    state: SimulationState,
    mv_id: str,
    *,
    geometry: RoadGeometryConfig = DEFAULT_ROAD_GEOMETRY,
) -> tuple[list[str], dict[str, Any]]:
    mv_state = state.vehicle_states[mv_id]
    window = resolve_aps_candidate_window(
        mv_state.x_global,
        mv_id=mv_id,
        geometry=geometry,
    )
    candidate_ids: list[str] = []
    excluded_candidates: list[dict[str, Any]] = []
    for vehicle_id in resolve_lane_ordering_by_x_global(state, LANE_2):
        vehicle_state = state.vehicle_states[vehicle_id]
        if not window.start_x_global <= vehicle_state.x_global <= window.end_x_global:
            continue
        eligibility = resolve_lane_2_gap_boundary_eligibility(state, vehicle_id)
        if eligibility.eligible:
            candidate_ids.append(vehicle_id)
            continue
        excluded_candidates.append(
            {
                "vehicle_id": vehicle_id,
                "physical_lane": eligibility.physical_lane,
                "lane_change_state": eligibility.lane_change_state,
                "excluded_reason": eligibility.reason,
            }
        )
    return candidate_ids, {
        "mv_id": mv_id,
        "x_mv_global": window.x_mv_global,
        "start_x_global": window.start_x_global,
        "end_x_global": window.end_x_global,
        "l_cr": window.l_cr,
        "parameter_name": window.parameter_name,
        "uses_fixed_cooperative_zone": window.uses_fixed_cooperative_zone,
        "uses_dynamic_coop_window": window.uses_dynamic_coop_window,
        "uses_x_global": window.uses_x_global,
        "uses_x_plot": window.uses_x_plot,
        "excluded_candidates": excluded_candidates,
    }


def compute_t_star_mv(
    state: SimulationState,
    mv_id: str,
    *,
    geometry: RoadGeometryConfig = DEFAULT_ROAD_GEOMETRY,
) -> float | None:
    mv_state = state.vehicle_states[mv_id]
    if mv_state.v <= MIN_APS_SPEED_MPS:
        return None
    return (geometry.x0_m_global - mv_state.x_global) / mv_state.v


def predict_aps_candidate_positions(
    state: SimulationState,
    candidate_ids: list[str],
    *,
    t_star_mv: float,
    geometry: RoadGeometryConfig = DEFAULT_ROAD_GEOMETRY,
    vehicle_length_m: float = PAPER_VEHICLE_LENGTH_M,
) -> list[APSCandidatePrediction]:
    predictions = []
    for vehicle_id in candidate_ids:
        vehicle = state.vehicle_states[vehicle_id]
        predicted_x = vehicle.x_global + vehicle.v * t_star_mv
        predictions.append(
            APSCandidatePrediction(
                vehicle_id=vehicle_id,
                x_global=vehicle.x_global,
                v=vehicle.v,
                predicted_x_global=predicted_x,
                d_star=predicted_x - geometry.x0_m_global - vehicle_length_m,
            )
        )
    return sorted(predictions, key=lambda item: (item.predicted_x_global, item.vehicle_id), reverse=True)


def select_clv_cfv(
    predictions: list[APSCandidatePrediction],
) -> tuple[APSCandidatePrediction, APSCandidatePrediction] | None:
    for index in range(len(predictions) - 1):
        clv = predictions[index]
        cfv = predictions[index + 1]
        if clv.d_star > 0 and cfv.d_star < 0:
            return clv, cfv
    return None


def select_local_clv_cfv(
    state: SimulationState,
    mv_id: str,
    candidate_ids: list[str],
    predictions: list[APSCandidatePrediction],
    *,
    existing_cache: Mapping[str, Any] | None = None,
) -> tuple[APSCandidatePrediction, APSCandidatePrediction] | None:
    by_vehicle_id = {prediction.vehicle_id: prediction for prediction in predictions}
    retained = _retained_pair_from_cache(existing_cache, by_vehicle_id)
    if retained is not None:
        return retained

    grouped = _same_prefix_pair(state, mv_id, by_vehicle_id)
    if grouped is not None:
        return grouped

    mv_x = float(state.vehicle_states[mv_id].x_global)
    ordered = sorted(
        (
            vehicle_id
            for vehicle_id in candidate_ids
            if vehicle_id in by_vehicle_id
        ),
        key=lambda vehicle_id: (
            float(state.vehicle_states[vehicle_id].x_global),
            vehicle_id,
        ),
    )
    leader_id: str | None = None
    follower_id: str | None = None
    for vehicle_id in ordered:
        x_global = float(state.vehicle_states[vehicle_id].x_global)
        if x_global > mv_x and leader_id is None:
            leader_id = vehicle_id
        if x_global < mv_x:
            follower_id = vehicle_id
    if leader_id is None or follower_id is None:
        return None
    return by_vehicle_id[leader_id], by_vehicle_id[follower_id]


def classify_aps_case(
    state: SimulationState,
    mv_id: str,
    clv: APSCandidatePrediction,
    cfv: APSCandidatePrediction,
    *,
    min_merge_time_gap_s: float = APS_MIN_MERGE_TIME_GAP_S,
) -> tuple[str, bool, bool, float | None]:
    mv_state = state.vehicle_states[mv_id]
    cfv_state = state.vehicle_states[cfv.vehicle_id]
    d_min_clv = mv_state.v * min_merge_time_gap_s
    d_min_cfv = cfv_state.v * min_merge_time_gap_s
    clv_ok = clv.d_star >= d_min_clv
    cfv_ok = abs(cfv.d_star) >= d_min_cfv
    if clv_ok and cfv_ok:
        return "case_1", False, False, None
    if clv_ok and not cfv_ok:
        return "case_2", False, True, _eq10_desired_spacing(
            d_min_cfv=d_min_cfv,
            d_min_clv=d_min_clv,
            d_star_clv=clv.d_star,
        )
    if not clv_ok and cfv_ok:
        return "case_3", True, False, None
    return "case_4", True, True, _eq10_desired_spacing(
        d_min_cfv=d_min_cfv,
        d_min_clv=d_min_clv,
        d_star_clv=clv.d_star,
    )


def build_aps_assignment(
    state: SimulationState,
    mv_id: str,
    clv: APSCandidatePrediction,
    cfv: APSCandidatePrediction,
    *,
    t_star_mv: float,
) -> APSAssignment:
    aps_case, col_clv, col_cfv, desired_spacing_override = classify_aps_case(
        state,
        mv_id,
        clv,
        cfv,
    )
    return APSAssignment(
        mv_id=mv_id,
        clv_id=clv.vehicle_id,
        cfv_id=cfv.vehicle_id,
        aps_case=aps_case,
        col_clv=col_clv,
        col_cfv=col_cfv,
        desired_spacing_override=desired_spacing_override,
        t_star_mv=t_star_mv,
        d_star_clv=clv.d_star,
        d_star_cfv=cfv.d_star,
    )


def run_aps_assignment_sanity(
    state: SimulationState,
    *,
    scenario_id: str,
    state_unchanged: bool,
    has_eq10_wrong_vehicle: bool,
    has_assignment_invalid: bool,
    assignment_vehicle_ids: tuple[str, ...] = (),
) -> list[dict[str, Any]]:
    vehicle_ids = tuple(state.active_vehicle_ids)
    checks: list[dict[str, Any]] = []
    if assignment_vehicle_ids:
        checks.append(
            _sanity(
                state,
                "assignment_invalid",
                "fail" if has_assignment_invalid else "pass",
                vehicle_ids=assignment_vehicle_ids,
                scenario_id=scenario_id,
                reason="p04_assignment_payload_valid",
            )
        )
    checks.extend(
        [
        _sanity(
            state,
            "assignment_cache_overwrite_by_failed_APS",
            "pass",
            vehicle_ids=vehicle_ids,
            scenario_id=scenario_id,
            reason="failed_APS_does_not_overwrite_cache",
            payload={"assignment_cache_overwrite_by_failed_APS": False},
        ),
        _sanity(
            state,
            "Eq10_applied_to_wrong_vehicle",
            "fail" if has_eq10_wrong_vehicle else "pass",
            vehicle_ids=vehicle_ids,
            scenario_id=scenario_id,
            reason="eq10_only_bound_to_case_2_or_4_cfv",
            payload={"Eq10_applied_to_wrong_vehicle": has_eq10_wrong_vehicle},
        ),
        _sanity(
            state,
            "x_plot_used_in_algorithm_path",
            "pass" if assert_x_plot_not_used_in_algorithm_path(state) else "fail",
            vehicle_ids=vehicle_ids,
            scenario_id=scenario_id,
            reason="x_global_only_algorithm_path",
            payload={"x_plot_used_in_algorithm_path": False},
        ),
        _sanity(
            state,
            "p04_no_write_before_commit",
            "pass" if state_unchanged else "fail",
            vehicle_ids=vehicle_ids,
            scenario_id=scenario_id,
            reason="p04_outputs_are_commands_events_and_cache_requests_only",
            payload={"p04_no_write_before_commit": state_unchanged},
        ),
        _sanity(
            state,
            "multiple_commit_for_one_vehicle",
            "pass",
            vehicle_ids=vehicle_ids,
            scenario_id=scenario_id,
            reason="p04_does_not_commit_vehicle_state",
        ),
        ]
    )
    return checks


def _run_fresh_aps(
    state: SimulationState,
    mv_id: str,
    *,
    trigger: str,
    existing_cache: Mapping[str, Any] | None,
    invalid_boundary: dict[str, Any] | None,
    geometry: RoadGeometryConfig,
    scenario_id: str,
) -> dict[str, Any]:
    candidate_ids, window_payload = collect_aps_candidates(state, mv_id, geometry=geometry)
    excluded_candidates = list(window_payload.get("excluded_candidates") or [])
    events = [
        _event(
            state,
            module="APS_candidate",
            event_type="APS_candidate",
            vehicle_id=mv_id,
            related_vehicle_ids=tuple([mv_id, *candidate_ids]),
            scenario_id=scenario_id,
            reason="candidate_window_collected",
            source="paper_formula",
            is_engineering_patch=False,
            payload={
                "candidate_window": window_payload,
                "candidate_ids": list(candidate_ids),
                "candidate_count": len(candidate_ids),
                "excluded_candidates": excluded_candidates,
                "uses_x_global": True,
                "uses_x_plot": False,
                "uses_fixed_cooperative_zone": False,
            },
        )
    ]
    events.append(_aps_trigger_event(state, mv_id, trigger=trigger, scenario_id=scenario_id))

    if len(candidate_ids) < 2:
        failure = _fresh_aps_failure(
            state,
            mv_id,
            trigger=trigger,
            existing_cache=existing_cache,
            invalid_boundary=invalid_boundary,
            failure_reason="insufficient_candidates",
            candidate_ids=candidate_ids,
            scenario_id=scenario_id,
        )
        failure["events"] = events + failure["events"]
        return failure

    t_star_mv = compute_t_star_mv(state, mv_id, geometry=geometry)
    if t_star_mv is None:
        failure = _fresh_aps_failure(
            state,
            mv_id,
            trigger=trigger,
            existing_cache=existing_cache,
            invalid_boundary=invalid_boundary,
            failure_reason="mv_speed_too_low",
            candidate_ids=candidate_ids,
            scenario_id=scenario_id,
        )
        failure["events"] = events + failure["events"]
        return failure

    predictions = predict_aps_candidate_positions(
        state,
        candidate_ids,
        t_star_mv=t_star_mv,
        geometry=geometry,
    )
    pair = select_local_clv_cfv(
        state,
        mv_id,
        candidate_ids,
        predictions,
        existing_cache=existing_cache,
    )
    if pair is None:
        failure = _fresh_aps_failure(
            state,
            mv_id,
            trigger=trigger,
            existing_cache=existing_cache,
            invalid_boundary=invalid_boundary,
            failure_reason="no_insert_pair",
            candidate_ids=candidate_ids,
            scenario_id=scenario_id,
        )
        failure["events"] = events + failure["events"]
        return failure

    clv, cfv = pair
    assignment = build_aps_assignment(state, mv_id, clv, cfv, t_star_mv=t_star_mv)
    record = assignment_lifecycle_manager.create_from_aps_success(state, assignment)
    if trigger == "merge_zone_handoff":
        record_value_for_merge_zone = assignment_lifecycle_manager.to_state_dict(record)
        record_value_for_merge_zone.update(
            {
                "lifecycle_state": "active_merge_zone",
                "last_updated_step": state.step,
                "last_updated_t": state.t,
                "source": "aps_updated_this_step",
            }
        )
        record = assignment_lifecycle_manager.from_state_dict(record_value_for_merge_zone)
    record_value = assignment_lifecycle_manager.to_state_dict(record)
    assignment_view = _derive_p04_assignment_view(state, record)
    cache_action = APSCacheAction(
        mv_id=mv_id,
        action="update_request",
        previous_cache_exists=existing_cache is not None,
        cache_modified_in_p04=False,
        update_request=MappingProxyType(record_value),
        reason="APS_assignment_updated_this_step",
        invalid_boundary_role=(
            _optional_str(invalid_boundary.get("role")) if invalid_boundary is not None else None
        ),
        invalid_boundary_id=(
            _optional_str(invalid_boundary.get("vehicle_id")) if invalid_boundary is not None else None
        ),
        invalid_reason=(
            _optional_str(invalid_boundary.get("reason")) if invalid_boundary is not None else None
        ),
    )
    events.append(
        _aps_event(
            state,
            mv_id=mv_id,
            related_vehicle_ids=(mv_id, assignment.clv_id, assignment.cfv_id),
            scenario_id=scenario_id,
            reason=_reason_for_trigger(trigger),
            source="paper_formula",
            payload={
                "trigger": trigger,
                "failure": False,
                "aps_case": assignment.aps_case,
                "clv_id": assignment.clv_id,
                "cfv_id": assignment.cfv_id,
                "col_clv": assignment.col_clv,
                "col_cfv": assignment.col_cfv,
                "t_star_mv": assignment.t_star_mv,
                "d_star_clv": assignment.d_star_clv,
                "d_star_cfv": assignment.d_star_cfv,
                "aps_min_merge_time_gap_s": APS_MIN_MERGE_TIME_GAP_S,
                "desired_spacing_override": assignment.desired_spacing_override,
                "eq10_vehicle_role": "cfv" if assignment.desired_spacing_override is not None else None,
                "effective_assignment_source": "aps_updated_this_step",
                "cache_update_request_created": True,
                "invalid_boundary_role": cache_action.invalid_boundary_role,
                "invalid_boundary_id": cache_action.invalid_boundary_id,
                "invalid_reason": cache_action.invalid_reason,
                "selection_order": "lane2_physical_order",
                "predictions": [_dataclass_to_plain(prediction) for prediction in predictions],
            },
            is_engineering_patch=trigger == "first_APS",
        )
    )
    events.append(_cache_event(state, cache_action, scenario_id=scenario_id))
    if assignment.desired_spacing_override is not None:
        events.append(
            _event(
                state,
                module="APS",
                event_type="eq10_desired_spacing_source",
                vehicle_id=assignment.cfv_id,
                related_vehicle_ids=(mv_id, assignment.cfv_id),
                scenario_id=scenario_id,
                reason="case_2_or_4_cfv_only",
                source="paper_formula",
                is_engineering_patch=False,
                payload={
                    "mv_id": mv_id,
                    "cfv_id": assignment.cfv_id,
                    "aps_case": assignment.aps_case,
                    "eq10_vehicle_role": "cfv",
                    "desired_spacing_override": assignment.desired_spacing_override,
                    "eq10_applied_to_clv": False,
                },
            )
        )
    return {
        "events": events,
        "cache_actions": [cache_action],
        "assignment_record_updates": {mv_id: MappingProxyType(record_value)},
        "assignment_view": assignment_view,
    }


def _fresh_aps_failure(
    state: SimulationState,
    mv_id: str,
    *,
    trigger: str,
    existing_cache: Mapping[str, Any] | None,
    invalid_boundary: dict[str, Any] | None,
    failure_reason: str,
    candidate_ids: list[str],
    scenario_id: str,
) -> dict[str, Any]:
    old_boundary_invalid = trigger == "cached_gap_boundary_invalid" and existing_cache is not None
    retained_record_value: Mapping[str, Any] | None = None
    assignment_view: AssignmentStepView | None = None
    if existing_cache is not None:
        existing_record = assignment_lifecycle_manager.from_state_dict(existing_cache)
        if old_boundary_invalid:
            retained_record = assignment_lifecycle_manager.mark_recovery_required(
                state,
                existing_record,
                _optional_str((invalid_boundary or {}).get("reason")) or failure_reason,
            )
        else:
            retained_record = assignment_lifecycle_manager.retain_after_aps_failure(
                state,
                existing_record,
                failure_reason,
            )
            assignment_view = _derive_p04_assignment_view(state, retained_record)
        retained_record_value = MappingProxyType(
            assignment_lifecycle_manager.to_state_dict(retained_record)
        )
    cache_action = APSCacheAction(
        mv_id=mv_id,
        action=(
            "update_request"
            if old_boundary_invalid
            else ("retain" if existing_cache is not None else "no_cache")
        ),
        previous_cache_exists=existing_cache is not None,
        cache_modified_in_p04=False,
        update_request=retained_record_value,
        reason=(
            "cached_gap_boundary_invalid"
            if old_boundary_invalid
            else ("retain_on_failed_aps" if existing_cache is not None else "no_cache_to_update")
        ),
        invalid_boundary_role=(
            _optional_str(invalid_boundary.get("role")) if invalid_boundary is not None else None
        ),
        invalid_boundary_id=(
            _optional_str(invalid_boundary.get("vehicle_id")) if invalid_boundary is not None else None
        ),
        invalid_reason=(
            _optional_str(invalid_boundary.get("reason")) if invalid_boundary is not None else None
        ),
    )
    return {
        "events": [
            _aps_event(
                state,
                mv_id=mv_id,
                related_vehicle_ids=(mv_id,),
                scenario_id=scenario_id,
                reason=failure_reason,
                source="paper_formula",
                payload={
                    "trigger": trigger,
                    "failure": True,
                    "failure_reason": failure_reason,
                    "candidate_ids": list(candidate_ids),
                    "candidate_count": len(candidate_ids),
                    "new_assignment_created": False,
                    "old_cache_invalidated": False,
                    "old_assignment_marked_recovery_required": old_boundary_invalid,
                    "invalid_boundary_role": cache_action.invalid_boundary_role,
                    "invalid_boundary_id": cache_action.invalid_boundary_id,
                    "invalid_reason": cache_action.invalid_reason,
                    "effective_assignment_source": (
                        None
                        if old_boundary_invalid
                        else ("cache_retained_after_failed_APS" if existing_cache is not None else None)
                    ),
                    "lifecycle_state": (
                        dict(retained_record_value).get("lifecycle_state")
                        if retained_record_value is not None
                        else None
                    ),
                },
                is_engineering_patch=trigger == "first_APS",
            ),
            _cache_event(state, cache_action, scenario_id=scenario_id),
        ],
        "cache_actions": [cache_action],
        "assignment_record_updates": (
            {mv_id: retained_record_value} if retained_record_value is not None else {}
        ),
        "assignment_view": assignment_view,
        "failure_sanity": _sanity(
            state,
            "assignment_invalid",
            "not_applicable",
            vehicle_ids=(mv_id,),
            scenario_id=scenario_id,
            reason="no_assignment_created_after_failed_APS",
            payload={"new_assignment_created": False},
        ),
    }


def _derive_p04_assignment_view(
    state: SimulationState,
    record: Any,
) -> AssignmentStepView | None:
    view = assignment_lifecycle_manager.derive_step5_view(state, record)
    if view is None:
        return None
    record_value = dict(view.record)
    lifecycle_state = str(record_value.get("lifecycle_state") or "").lower()
    return AssignmentStepView(
        mv_id=view.mv_id,
        record=MappingProxyType(record_value),
        source=view.source,
        consumable_by_step5=view.consumable_by_step5,
        consumable_by_cmc=(
            view.consumable_by_cmc or lifecycle_state == "active_merge_zone"
        ),
    )


def _step4a_mv_ids(
    state: SimulationState,
    *,
    eligible_mv_ids: Iterable[str] | None = None,
) -> list[str]:
    eligible = None if eligible_mv_ids is None else set(eligible_mv_ids)
    return [
        vehicle_id
        for vehicle_id in state.active_vehicle_ids
        if _is_mv_candidate(state.vehicle_states[vehicle_id])
        and (eligible is None or vehicle_id in eligible)
    ]


def _is_mv_candidate(vehicle: VehicleState) -> bool:
    return vehicle.physical_lane == ON_RAMP or vehicle.road_role == ON_RAMP_MV_ROLE


def _is_merge_zone_handoff_refresh_candidate(
    state: SimulationState,
    mv_id: str,
    *,
    existing_cache: Mapping[str, Any] | None,
    geometry: RoadGeometryConfig,
) -> bool:
    mv_state = state.vehicle_states[mv_id]
    region = resolve_region(mv_state.x_global, mv_state.road_role, geometry=geometry)
    if not region.in_merging_zone or mv_state.merge_state == "executing":
        return False
    if not _is_first_merge_zone_entry_step(state, mv_id, geometry=geometry):
        return False
    if existing_cache is None:
        return False
    try:
        record = assignment_lifecycle_manager.from_state_dict(existing_cache)
    except (TypeError, ValueError):
        return False
    return (
        record.status in {"valid", "available", "ok"}
        and record.gap_type == "bounded"
        and record.lifecycle_state in {"active_control_zone", "refresh_failed_retained"}
    )


def _is_first_merge_zone_entry_step(
    state: SimulationState,
    mv_id: str,
    *,
    geometry: RoadGeometryConfig,
) -> bool:
    if state.step <= 0:
        return False
    mv_state = state.vehicle_states[mv_id]
    previous_x_estimate = float(mv_state.x_global) - float(mv_state.v) * float(state.dt)
    return previous_x_estimate < float(geometry.x0_m_global)


def _last_aps_times(config: dict[str, Any]) -> dict[str, float]:
    values: dict[str, float] = {}
    for item in config.get("preloaded_state_machine_states", []):
        if item.get("last_aps_time") is not None:
            values[str(item["vehicle_id"])] = float(item["last_aps_time"])
    return values


def _freeze_first_aps_assignment_until_cmc(config: dict[str, Any]) -> bool:
    module_overrides = config.get("module_overrides") or {}
    harness = module_overrides.get("test_harness_overrides") or {}
    return bool(harness.get("freeze_first_aps_assignment_until_cmc"))


def _retained_pair_from_cache(
    existing_cache: Mapping[str, Any] | None,
    predictions_by_vehicle_id: Mapping[str, APSCandidatePrediction],
) -> tuple[APSCandidatePrediction, APSCandidatePrediction] | None:
    if existing_cache is None:
        return None
    clv_id = _optional_str(existing_cache.get("clv_id"))
    cfv_id = _optional_str(existing_cache.get("cfv_id"))
    if clv_id is None or cfv_id is None:
        return None
    clv = predictions_by_vehicle_id.get(clv_id)
    cfv = predictions_by_vehicle_id.get(cfv_id)
    if clv is None or cfv is None:
        return None
    return clv, cfv


def _same_prefix_pair(
    state: SimulationState,
    mv_id: str,
    predictions_by_vehicle_id: Mapping[str, APSCandidatePrediction],
) -> tuple[APSCandidatePrediction, APSCandidatePrediction] | None:
    prefix = _basic_unit_prefix(mv_id)
    if prefix is None:
        return None
    clv_id = f"{prefix}_CLV"
    cfv_id = f"{prefix}_CFV"
    clv = predictions_by_vehicle_id.get(clv_id)
    cfv = predictions_by_vehicle_id.get(cfv_id)
    if clv is None or cfv is None:
        return None
    clv_state = state.vehicle_states.get(clv_id)
    cfv_state = state.vehicle_states.get(cfv_id)
    if clv_state is None or cfv_state is None:
        return None
    if float(clv_state.x_global) <= float(cfv_state.x_global):
        return None
    return clv, cfv


def _basic_unit_prefix(mv_id: str) -> str | None:
    if mv_id.endswith("_MV"):
        return mv_id.removesuffix("_MV")
    return None


def _eq10_desired_spacing(*, d_min_cfv: float, d_min_clv: float, d_star_clv: float) -> float:
    return (
        EQ10_CFV_MIN_GAP_WEIGHT * d_min_cfv
        + PAPER_VEHICLE_LENGTH_M
        + max(d_min_clv, d_star_clv)
    )


def _reason_for_trigger(trigger: str) -> str:
    if trigger == "first_APS":
        return "first_aps"
    if trigger == "APS_due":
        return "APS_due"
    if trigger == "merge_zone_handoff":
        return "merge_zone_handoff"
    return trigger


def _aps_trigger_event(
    state: SimulationState,
    mv_id: str,
    *,
    trigger: str,
    scenario_id: str,
) -> dict[str, Any]:
    is_first_aps = trigger == "first_APS"
    return _aps_event(
        state,
        mv_id=mv_id,
        related_vehicle_ids=(mv_id,),
        scenario_id=scenario_id,
        reason="first_aps" if is_first_aps else trigger,
        source="first_version_engineering_patch" if is_first_aps else "paper_formula",
        payload={
            "trigger": trigger,
            "aps_trigger_resolved": True,
            "aps_executed": True,
            "is_engineering_patch": is_first_aps,
        },
        is_engineering_patch=is_first_aps,
    )


def _handoff_event(
    state: SimulationState,
    mv_id: str,
    *,
    scenario_id: str,
) -> dict[str, Any]:
    return _aps_event(
        state,
        mv_id=mv_id,
        related_vehicle_ids=(mv_id,),
        scenario_id=scenario_id,
        reason="mv_in_merging_zone",
        source="first_version_engineering_patch",
        payload={
            "branch": "handed_off_to_cmc",
            "aps_executed": False,
            "cache_modified": False,
            "new_assignment_created": False,
        },
        is_engineering_patch=True,
    )


def _aps_event(
    state: SimulationState,
    *,
    mv_id: str,
    related_vehicle_ids: tuple[str, ...],
    scenario_id: str,
    reason: str,
    source: str,
    payload: dict[str, Any],
    is_engineering_patch: bool,
) -> dict[str, Any]:
    return _event(
        state,
        module="APS",
        event_type="APS",
        vehicle_id=mv_id,
        related_vehicle_ids=related_vehicle_ids,
        scenario_id=scenario_id,
        reason=reason,
        source=source,
        is_engineering_patch=is_engineering_patch,
        payload=payload,
    )


def _cache_event(
    state: SimulationState,
    action: APSCacheAction,
    *,
    scenario_id: str,
) -> dict[str, Any]:
    return _event(
        state,
        module="assignment_cache",
        event_type="assignment_cache",
        vehicle_id=action.mv_id,
        related_vehicle_ids=(action.mv_id,),
        scenario_id=scenario_id,
        reason=action.reason,
        source="first_version_engineering_patch",
        is_engineering_patch=True,
        payload={
            "action": action.action,
            "previous_cache": None if not action.previous_cache_exists else "present",
            "previous_cache_exists": action.previous_cache_exists,
            "new_assignment_created": action.update_request is not None,
            "cache_modified": action.cache_modified_in_p04,
            "invalid_new_assignment_overwrites_existing_cache": False,
            "old_cache_invalidated": action.action == "invalidate",
            "invalid_boundary_role": action.invalid_boundary_role,
            "invalid_boundary_id": action.invalid_boundary_id,
            "invalid_reason": action.invalid_reason,
            "update_request": dict(action.update_request or {}),
        },
    )


def _event(
    state: SimulationState,
    *,
    module: str,
    event_type: str,
    vehicle_id: str | None,
    related_vehicle_ids: tuple[str, ...],
    scenario_id: str,
    reason: str,
    source: str,
    is_engineering_patch: bool,
    payload: dict[str, Any],
) -> dict[str, Any]:
    vehicle_ids = list(related_vehicle_ids)
    if vehicle_id is not None and vehicle_id not in vehicle_ids:
        vehicle_ids.append(vehicle_id)
    return {
        "step": state.step,
        "t": state.t,
        "module": module,
        "event_type": event_type,
        "vehicle_id": vehicle_id,
        "vehicle_ids": vehicle_ids,
        "related_vehicle_ids": vehicle_ids,
        "scenario_id": scenario_id,
        "reason": reason,
        "source": source,
        "is_engineering_patch": is_engineering_patch,
        "payload": payload,
    }


def _sanity(
    state: SimulationState,
    check_type: str,
    result: str,
    *,
    vehicle_ids: tuple[str, ...],
    scenario_id: str,
    reason: str,
    payload: dict[str, Any] | None = None,
) -> dict[str, Any]:
    return {
        "step": state.step,
        "t": state.t,
        "check_type": check_type,
        "result": result,
        "vehicle_ids": list(vehicle_ids),
        "scenario_id": scenario_id,
        "reason": reason,
        "payload": payload or {},
    }


def _state_signature(state: SimulationState) -> tuple[Any, ...]:
    return (
        state.t,
        state.step,
        state.dt,
        state.active_vehicle_ids,
        tuple(
            (
                vehicle_id,
                state.vehicle_states[vehicle_id].x_global,
                state.vehicle_states[vehicle_id].y,
                state.vehicle_states[vehicle_id].v,
                state.vehicle_states[vehicle_id].a,
                state.vehicle_states[vehicle_id].physical_lane,
                state.vehicle_states[vehicle_id].road_role,
                state.vehicle_states[vehicle_id].lane_change_state,
                state.vehicle_states[vehicle_id].merge_state,
            )
            for vehicle_id in state.active_vehicle_ids
        ),
        tuple((key, tuple(sorted(value.items()))) for key, value in state.assignment_records_by_mv.items()),
    )


def _cached_assignment_invalid_boundary(
    state: SimulationState,
    cache: Mapping[str, Any] | None,
) -> dict[str, Any] | None:
    if cache is None:
        return None
    for role, key in (("clv", "clv_id"), ("cfv", "cfv_id")):
        vehicle_id = _optional_str(cache.get(key))
        eligibility = resolve_lane_2_gap_boundary_eligibility(state, vehicle_id)
        if eligibility.eligible:
            continue
        return {
            "role": role,
            "vehicle_id": vehicle_id,
            "reason": eligibility.reason,
            "physical_lane": eligibility.physical_lane,
            "lane_change_state": eligibility.lane_change_state,
        }
    return None


def _optional_str(value: Any) -> str | None:
    if value in (None, ""):
        return None
    return str(value)


def _dataclass_to_plain(value: Any) -> dict[str, Any]:
    return {field.name: getattr(value, field.name) for field in fields(value)}
