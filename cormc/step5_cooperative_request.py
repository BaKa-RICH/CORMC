from __future__ import annotations

from dataclasses import dataclass, fields
from types import MappingProxyType
from typing import Any, Mapping

from cormc.step0_3 import (
    DEFAULT_ROAD_GEOMETRY,
    RelationsSnapshot,
    RoadGeometryConfig,
    SimulationState,
    build_prefreeze_workspace_from_scenario,
    freeze_simulation_state,
    refresh_relations_snapshot,
    resolve_region,
)
from cormc.step4a_aps import EffectiveAssignmentThisStep
from cormc.step9_11 import CommandBuffer


ENGINEERING_PATCH_SOURCE = "first_version_engineering_patch"
PAPER_FORMULA_SOURCE = "paper_formula"


@dataclass(frozen=True)
class CooperativeRequest:
    request_id: str
    source_mv_id: str
    cv_id: str
    cv_role: str
    col: bool
    aps_case: str | None
    assignment_source: str
    t_mv_star: float
    mv_in_merging_zone: bool
    mv_distance_to_x0_m: float
    desired_spacing_override: float | None = None

    def to_payload(self) -> dict[str, Any]:
        return _dataclass_to_plain(self)


@dataclass(frozen=True)
class ConflictResolutionResult:
    conflict_id: str
    cv_id: str
    request_ids: tuple[str, ...]
    winner_request_id: str
    loser_request_ids: tuple[str, ...]
    winner_mv_id: str
    loser_mv_ids: tuple[str, ...]
    priority_basis: str
    priority_values_by_request: Mapping[str, Mapping[str, Any]]
    active_request_count_for_cv: int = 1
    one_active_request_per_cv: bool = True
    conflicting_commands_to_same_CV: bool = False
    source: str = ENGINEERING_PATCH_SOURCE
    reason: str = "multi_mv_shared_cv_conflict_resolution"
    is_engineering_patch: bool = True

    def to_payload(self) -> dict[str, Any]:
        payload = _dataclass_to_plain(self)
        payload["request_ids"] = list(self.request_ids)
        payload["loser_request_ids"] = list(self.loser_request_ids)
        payload["loser_mv_ids"] = list(self.loser_mv_ids)
        payload["priority_values_by_request"] = {
            request_id: dict(values)
            for request_id, values in self.priority_values_by_request.items()
        }
        return payload


@dataclass(frozen=True)
class Step5CooperativeRequestRunResult:
    state: SimulationState
    relations: RelationsSnapshot
    command_buffer: CommandBuffer
    actual_events: list[dict[str, Any]]
    actual_sanity_checks: list[dict[str, Any]]
    expected_png_features: list[dict[str, Any]]
    cooperative_requests: tuple[CooperativeRequest, ...]
    active_requests: Mapping[str, Mapping[str, Any]]
    suppressed_requests: tuple[Mapping[str, Any], ...]
    conflict_results: tuple[ConflictResolutionResult, ...]


def run_step5_cooperative_request_conflict_resolution_for_scenario(
    scenario: str | dict[str, Any],
    *,
    geometry: RoadGeometryConfig = DEFAULT_ROAD_GEOMETRY,
    effective_assignments: Mapping[str, EffectiveAssignmentThisStep] | None = None,
    p05_validation_results: Mapping[str, Any] | None = None,
) -> Step5CooperativeRequestRunResult:
    workspace, config = build_prefreeze_workspace_from_scenario(scenario, geometry=geometry)
    state = freeze_simulation_state(workspace)
    relations = refresh_relations_snapshot(state, geometry=geometry)
    if effective_assignments is None:
        effective_assignments = _effective_assignments_from_state_cache(state)
    return run_step5_cooperative_request_conflict_resolution(
        state,
        relations,
        config=config,
        geometry=geometry,
        effective_assignments=effective_assignments,
        p05_validation_results=p05_validation_results,
    )


def run_step5_cooperative_request_conflict_resolution(
    state: SimulationState,
    relations: RelationsSnapshot,
    *,
    config: dict[str, Any] | None = None,
    geometry: RoadGeometryConfig = DEFAULT_ROAD_GEOMETRY,
    effective_assignments: Mapping[str, EffectiveAssignmentThisStep] | None = None,
    p05_validation_results: Mapping[str, Any] | None = None,
) -> Step5CooperativeRequestRunResult:
    scenario_id = str((config or {}).get("scenario_id") or state.scenario_config_ref or "unknown")
    before_signature = _state_signature(state)
    assignments = effective_assignments or _effective_assignments_from_state_cache(state)
    events: list[dict[str, Any]] = []

    _, filtered_assignments = filter_valid_request_assignments(
        assignments,
        p05_validation_results=p05_validation_results,
    )
    requests = collect_cooperative_requests(
        state,
        assignments,
        geometry=geometry,
        p05_validation_results=p05_validation_results,
    )
    events.extend(
        emit_cooperative_request_event(state, request, scenario_id=scenario_id)
        for request in requests
    )
    active_requests, suppressed_requests, conflict_results = build_active_cooperative_requests(
        requests
    )
    events.extend(
        emit_conflict_resolution_event(state, result, scenario_id=scenario_id)
        for result in conflict_results
    )
    command_buffer = CommandBuffer(
        step=state.step,
        t=state.t,
        cooperation_commands=MappingProxyType(dict(active_requests)),
    )
    sanity_checks = [
        run_p06_invalid_assignment_suppressed_sanity(
            state,
            scenario_id=scenario_id,
            filtered_assignments=filtered_assignments,
        ),
        run_p06_no_write_before_commit_sanity(
            state,
            scenario_id=scenario_id,
            state_unchanged=before_signature == _state_signature(state),
        )
    ]
    return Step5CooperativeRequestRunResult(
        state=state,
        relations=relations,
        command_buffer=command_buffer,
        actual_events=events,
        actual_sanity_checks=sanity_checks,
        expected_png_features=register_p06_png_features(
            requests,
            conflict_results=conflict_results,
            suppressed_requests=suppressed_requests,
        ),
        cooperative_requests=tuple(requests),
        active_requests=MappingProxyType(dict(active_requests)),
        suppressed_requests=tuple(suppressed_requests),
        conflict_results=tuple(conflict_results),
    )


def filter_valid_request_assignments(
    assignments: Mapping[str, EffectiveAssignmentThisStep],
    *,
    p05_validation_results: Mapping[str, Any] | None = None,
) -> tuple[Mapping[str, EffectiveAssignmentThisStep], tuple[dict[str, Any], ...]]:
    valid: dict[str, EffectiveAssignmentThisStep] = {}
    filtered: list[dict[str, Any]] = []
    for mv_id, effective_assignment in assignments.items():
        reason = _request_filter_reason(
            effective_assignment,
            validation=(p05_validation_results or {}).get(mv_id),
        )
        if reason is None:
            valid[mv_id] = effective_assignment
            continue
        filtered.append(
            {
                "mv_id": mv_id,
                "reason": reason,
                "assignment_source": effective_assignment.source,
            }
        )
    return MappingProxyType(valid), tuple(filtered)


def collect_cooperative_requests(
    state: SimulationState,
    assignments: Mapping[str, EffectiveAssignmentThisStep],
    *,
    geometry: RoadGeometryConfig = DEFAULT_ROAD_GEOMETRY,
    p05_validation_results: Mapping[str, Any] | None = None,
) -> list[CooperativeRequest]:
    valid_assignments, _ = filter_valid_request_assignments(
        assignments,
        p05_validation_results=p05_validation_results,
    )
    requests: list[CooperativeRequest] = []
    for mv_id in sorted(valid_assignments):
        effective_assignment = valid_assignments[mv_id]
        assignment = effective_assignment.assignment
        if _truthy_bool(assignment.get("col_clv")):
            request = build_cooperative_request_from_assignment(
                state,
                effective_assignment,
                cv_role="clv",
                geometry=geometry,
            )
            if request is not None:
                requests.append(request)
        if _truthy_bool(assignment.get("col_cfv")):
            request = build_cooperative_request_from_assignment(
                state,
                effective_assignment,
                cv_role="cfv",
                geometry=geometry,
            )
            if request is not None:
                requests.append(request)
    return requests


def build_cooperative_request_from_assignment(
    state: SimulationState,
    effective_assignment: EffectiveAssignmentThisStep,
    *,
    cv_role: str,
    geometry: RoadGeometryConfig = DEFAULT_ROAD_GEOMETRY,
) -> CooperativeRequest | None:
    assignment = effective_assignment.assignment
    source_mv_id = str(assignment.get("mv_id") or effective_assignment.mv_id)
    cv_id = _optional_str(assignment.get(f"{cv_role}_id"))
    if cv_id is None or cv_id not in state.vehicle_states:
        return None
    cv_state = state.vehicle_states[cv_id]
    if not cv_state.is_active:
        return None
    mv_state = state.vehicle_states[source_mv_id]
    region = resolve_region(mv_state.x_global, mv_state.road_role, geometry=geometry)
    t_mv_star = _required_t_mv_star(assignment, mv_id=source_mv_id)
    return CooperativeRequest(
        request_id=f"p06:{state.step}:{cv_id}:{source_mv_id}:{cv_role}",
        source_mv_id=source_mv_id,
        cv_id=cv_id,
        cv_role=cv_role,
        col=True,
        aps_case=_optional_str(assignment.get("aps_case")),
        assignment_source=effective_assignment.source,
        t_mv_star=t_mv_star,
        mv_in_merging_zone=region.in_merging_zone,
        mv_distance_to_x0_m=abs(float(mv_state.x_global) - float(geometry.x0_m_global)),
        desired_spacing_override=_optional_float(assignment.get("desired_spacing_override")),
    )


def group_requests_by_cv(
    requests: list[CooperativeRequest] | tuple[CooperativeRequest, ...],
) -> Mapping[str, tuple[CooperativeRequest, ...]]:
    grouped: dict[str, list[CooperativeRequest]] = {}
    for request in requests:
        grouped.setdefault(request.cv_id, []).append(request)
    return MappingProxyType(
        {
            cv_id: tuple(sorted(group, key=lambda request: request.request_id))
            for cv_id, group in grouped.items()
        }
    )


def resolve_request_conflicts(
    requests: list[CooperativeRequest] | tuple[CooperativeRequest, ...],
) -> tuple[dict[str, dict[str, Any]], tuple[dict[str, Any], ...], tuple[ConflictResolutionResult, ...]]:
    active_requests: dict[str, dict[str, Any]] = {}
    suppressed_requests: list[dict[str, Any]] = []
    conflict_results: list[ConflictResolutionResult] = []
    for cv_id, group in group_requests_by_cv(requests).items():
        if len(group) == 1:
            request = group[0]
            active_requests[cv_id] = _active_request_payload(request, source_conflict_id=None)
            continue

        winner, priority_basis = select_conflict_winner(group)
        losers = tuple(request for request in group if request.request_id != winner.request_id)
        conflict_id = f"p06:{_step_from_request(winner)}:conflict:{cv_id}"
        result = ConflictResolutionResult(
            conflict_id=conflict_id,
            cv_id=cv_id,
            request_ids=tuple(request.request_id for request in group),
            winner_request_id=winner.request_id,
            loser_request_ids=tuple(request.request_id for request in losers),
            winner_mv_id=winner.source_mv_id,
            loser_mv_ids=tuple(request.source_mv_id for request in losers),
            priority_basis=priority_basis,
            priority_values_by_request=MappingProxyType(
                {
                    request.request_id: MappingProxyType(_priority_payload(request))
                    for request in group
                }
            ),
            reason=(
                "deterministic_tie_breaker_after_equal_priority"
                if priority_basis == "deterministic_tie_breaker"
                else "multi_mv_shared_cv_conflict_resolution"
            ),
        )
        conflict_results.append(result)
        active_requests[cv_id] = _active_request_payload(
            winner,
            source_conflict_id=conflict_id,
        )
        suppressed_requests.extend(
            _suppressed_request_payload(
                loser,
                suppressed_by_request_id=winner.request_id,
                suppressed_reason=priority_basis,
                conflict_id=conflict_id,
            )
            for loser in losers
        )
    return active_requests, tuple(suppressed_requests), tuple(conflict_results)


def build_active_cooperative_requests(
    requests: list[CooperativeRequest] | tuple[CooperativeRequest, ...],
) -> tuple[dict[str, dict[str, Any]], tuple[dict[str, Any], ...], tuple[ConflictResolutionResult, ...]]:
    return resolve_request_conflicts(requests)


def select_conflict_winner(
    requests: tuple[CooperativeRequest, ...] | list[CooperativeRequest],
) -> tuple[CooperativeRequest, str]:
    candidates = tuple(sorted(requests, key=_deterministic_key))
    in_zone_candidates = tuple(request for request in candidates if request.mv_in_merging_zone)
    if in_zone_candidates and len(in_zone_candidates) != len(candidates):
        if len(in_zone_candidates) == 1:
            return in_zone_candidates[0], "MV_in_merging_zone"
        candidates = in_zone_candidates

    min_t_mv_star = min(request.t_mv_star for request in candidates)
    t_candidates = tuple(
        request for request in candidates if request.t_mv_star == min_t_mv_star
    )
    if len(t_candidates) == 1:
        return t_candidates[0], "smaller_T_star_MV"

    min_distance = min(request.mv_distance_to_x0_m for request in t_candidates)
    distance_candidates = tuple(
        request for request in t_candidates if request.mv_distance_to_x0_m == min_distance
    )
    if len(distance_candidates) == 1:
        return distance_candidates[0], "closer_to_x0_m"
    return sorted(distance_candidates, key=_deterministic_key)[0], "deterministic_tie_breaker"


def emit_cooperative_request_event(
    state: SimulationState,
    request: CooperativeRequest,
    *,
    scenario_id: str,
) -> dict[str, Any]:
    return _event(
        state,
        module="Step5CooperativeRequest",
        event_type="cooperative_request",
        vehicle_id=request.cv_id,
        related_vehicle_ids=(request.source_mv_id, request.cv_id),
        scenario_id=scenario_id,
        reason=f"col_{request.cv_role}_request",
        source=PAPER_FORMULA_SOURCE,
        is_engineering_patch=False,
        payload={
            **request.to_payload(),
            "t_star_mv": request.t_mv_star,
            "request_candidate": True,
        },
    )


def emit_conflict_resolution_event(
    state: SimulationState,
    result: ConflictResolutionResult,
    *,
    scenario_id: str,
) -> dict[str, Any]:
    related_vehicle_ids = (result.cv_id, result.winner_mv_id, *result.loser_mv_ids)
    return _event(
        state,
        module="Step5ConflictResolver",
        event_type="conflict_resolution",
        vehicle_id=result.cv_id,
        related_vehicle_ids=related_vehicle_ids,
        scenario_id=scenario_id,
        reason=result.priority_basis,
        source=ENGINEERING_PATCH_SOURCE,
        is_engineering_patch=True,
        payload=result.to_payload(),
    )


def emit_suppressed_request_event(
    state: SimulationState,
    suppressed_request: Mapping[str, Any],
    *,
    scenario_id: str,
) -> dict[str, Any]:
    return _event(
        state,
        module="Step5ConflictResolver",
        event_type="conflict_resolution",
        vehicle_id=str(suppressed_request["cv_id"]),
        related_vehicle_ids=(
            str(suppressed_request["source_mv_id"]),
            str(suppressed_request["cv_id"]),
        ),
        scenario_id=scenario_id,
        reason=str(suppressed_request["suppressed_reason"]),
        source=ENGINEERING_PATCH_SOURCE,
        is_engineering_patch=True,
        payload=dict(suppressed_request),
    )


def run_p06_invalid_assignment_suppressed_sanity(
    state: SimulationState,
    *,
    scenario_id: str,
    filtered_assignments: tuple[dict[str, Any], ...],
) -> dict[str, Any]:
    return _sanity(
        state,
        "assignment_invalid",
        "warning" if filtered_assignments else "pass",
        vehicle_ids=tuple(item["mv_id"] for item in filtered_assignments) or tuple(state.active_vehicle_ids),
        scenario_id=scenario_id,
        reason="invalid_assignment_filtered" if filtered_assignments else "no_invalid_assignment",
        payload={
            "filtered_assignments": list(filtered_assignments),
            "invalid_assignment_restored_as_active_request": False,
        },
    )


def run_p06_no_write_before_commit_sanity(
    state: SimulationState,
    *,
    scenario_id: str,
    state_unchanged: bool,
) -> dict[str, Any]:
    return _sanity(
        state,
        "no_write_before_commit",
        "pass" if state_unchanged else "fail",
        vehicle_ids=tuple(state.active_vehicle_ids),
        scenario_id=scenario_id,
        reason="p06_no_write_before_commit",
        payload={
            "state_unchanged": state_unchanged,
            "aps_assignment_cache_unchanged": state_unchanged,
            "p06_outputs_are_derived_only": True,
        },
    )


def register_p06_png_features(
    requests: list[CooperativeRequest] | tuple[CooperativeRequest, ...],
    *,
    conflict_results: tuple[ConflictResolutionResult, ...],
    suppressed_requests: tuple[Mapping[str, Any], ...],
) -> list[dict[str, Any]]:
    vehicle_ids = sorted(
        {vehicle_id for request in requests for vehicle_id in (request.source_mv_id, request.cv_id)}
    )
    features = [
        {
            "feature_type": "cooperative_request_marker",
            "required": True,
            "vehicle_ids": vehicle_ids,
            "expected_visibility": "visible",
            "notes": "registered only; renderer deferred",
        }
    ]
    if conflict_results:
        conflict_vehicle_ids = sorted(
            {
                vehicle_id
                for result in conflict_results
                for vehicle_id in (result.cv_id, result.winner_mv_id, *result.loser_mv_ids)
            }
        )
        features.append(
            {
                "feature_type": "conflict_group_marker",
                "required": True,
                "vehicle_ids": conflict_vehicle_ids,
                "expected_visibility": "visible",
                "notes": "registered only; renderer deferred",
            }
        )
        features.append(
            {
                "feature_type": "active_request_marker",
                "required": True,
                "vehicle_ids": sorted(
                    {
                        result.winner_mv_id
                        for result in conflict_results
                    }
                    | {result.cv_id for result in conflict_results}
                ),
                "expected_visibility": "visible",
                "notes": "registered only; renderer deferred",
            }
        )
    if suppressed_requests:
        features.append(
            {
                "feature_type": "suppressed_request_marker",
                "required": True,
                "vehicle_ids": sorted(
                    {
                        str(item["source_mv_id"])
                        for item in suppressed_requests
                    }
                    | {str(item["cv_id"]) for item in suppressed_requests}
                ),
                "expected_visibility": "visible",
                "notes": "registered only; renderer deferred",
            }
        )
    return features


def _effective_assignments_from_state_cache(
    state: SimulationState,
) -> Mapping[str, EffectiveAssignmentThisStep]:
    assignments: dict[str, EffectiveAssignmentThisStep] = {}
    for mv_id, assignment in state.aps_assignment_cache.items():
        status = str(assignment.get("status", "")).lower()
        assignments[mv_id] = EffectiveAssignmentThisStep(
            mv_id=mv_id,
            assignment=MappingProxyType(dict(assignment)),
            source=str(assignment.get("source") or "aps_cache"),
            available_for_cooperative_request=status in {"valid", "available", "ok"},
        )
    return MappingProxyType(assignments)


def _request_filter_reason(
    effective_assignment: EffectiveAssignmentThisStep,
    *,
    validation: Any = None,
) -> str | None:
    if not effective_assignment.available_for_cooperative_request:
        return "not_available_for_cooperative_request"
    status = str(effective_assignment.assignment.get("status", "valid")).lower()
    if status in {"failed", "failure"}:
        return "failed_assignment_filtered"
    if status in {"invalid", "empty", "not_available", "stale"}:
        return "invalid_assignment_filtered"
    if status not in {"valid", "available", "ok"}:
        return "invalid_assignment_filtered"
    if validation is not None and not _validation_is_valid(validation):
        return _validation_invalid_reason(validation) or "invalid_assignment_filtered"
    return None


def _validation_is_valid(validation: Any) -> bool:
    if isinstance(validation, Mapping):
        if "validation_status" in validation:
            return str(validation["validation_status"]).lower() == "valid"
        if "assignment_valid" in validation:
            return bool(validation["assignment_valid"])
    return bool(getattr(validation, "assignment_valid", False))


def _validation_invalid_reason(validation: Any) -> str | None:
    if isinstance(validation, Mapping):
        value = validation.get("invalid_reason")
        return str(value) if value not in (None, "") else None
    value = getattr(validation, "invalid_reason", None)
    return str(value) if value not in (None, "") else None


def _required_t_mv_star(assignment: Mapping[str, Any], *, mv_id: str) -> float:
    value = assignment.get("t_mv_star", assignment.get("t_star_mv"))
    if value is None:
        raise ValueError(f"P06 handoff preflight failed: missing t_mv_star for {mv_id}")
    return float(value)


def _active_request_payload(
    request: CooperativeRequest,
    *,
    source_conflict_id: str | None,
) -> dict[str, Any]:
    return {
        **request.to_payload(),
        "active": True,
        "source_conflict_id": source_conflict_id,
    }


def _suppressed_request_payload(
    request: CooperativeRequest,
    *,
    suppressed_by_request_id: str,
    suppressed_reason: str,
    conflict_id: str,
) -> dict[str, Any]:
    return {
        **request.to_payload(),
        "active": False,
        "suppressed_by_request_id": suppressed_by_request_id,
        "suppressed_reason": suppressed_reason,
        "conflict_id": conflict_id,
    }


def _priority_payload(request: CooperativeRequest) -> dict[str, Any]:
    return {
        "source_mv_id": request.source_mv_id,
        "mv_in_merging_zone": request.mv_in_merging_zone,
        "t_mv_star": request.t_mv_star,
        "mv_distance_to_x0_m": request.mv_distance_to_x0_m,
        "deterministic_tie_breaker_key": list(_deterministic_key(request)),
    }


def _step_from_request(request: CooperativeRequest) -> str:
    parts = request.request_id.split(":", 3)
    if len(parts) >= 3:
        return parts[1]
    return "unknown"


def _deterministic_key(request: CooperativeRequest) -> tuple[str, str, str]:
    return (request.cv_id, request.source_mv_id, request.request_id)


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
        "related_vehicle_ids": list(related_vehicle_ids),
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
        tuple((key, tuple(sorted(value.items()))) for key, value in state.aps_assignment_cache.items()),
    )


def _optional_str(value: Any) -> str | None:
    if value in (None, ""):
        return None
    return str(value)


def _optional_float(value: Any) -> float | None:
    if value is None:
        return None
    return float(value)


def _truthy_bool(value: Any) -> bool:
    if isinstance(value, str):
        return value.lower() in {"true", "1", "yes"}
    return bool(value)


def _dataclass_to_plain(value: Any) -> dict[str, Any]:
    return {field.name: getattr(value, field.name) for field in fields(value)}
