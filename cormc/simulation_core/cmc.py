from __future__ import annotations

from dataclasses import dataclass, fields
from types import MappingProxyType
from typing import Any, Iterable, Mapping

from cormc.simulation_core.assignment_lifecycle import (
    AssignmentRecord,
    AssignmentStepView,
    assignment_lifecycle_manager,
)
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
    resolve_lane_2_gap_boundary_eligibility,
    resolve_region,
)
from cormc.simulation_core.commit import CommandBuffer


CMC_UPPER_MERGE_TIME_GAP_S = 1.2
CMC_DYNAMIC_GAP_XI = 2.0 / 3.0
PAPER_VEHICLE_LENGTH_M = 4.0
STANDSTILL_SPACING_M = 2.0
LANE_CHANGE_PLANNED_ACCELERATION_MPS2 = 0.1


@dataclass(frozen=True)
class CMCAssignmentSource:
    mv_id: str
    assignment: Mapping[str, Any] | None
    source: str | None
    record: AssignmentRecord | None = None


@dataclass(frozen=True)
class CMCAssignmentValidationResult:
    mv_id: str
    assignment_source: str | None
    assignment_valid: bool
    assigned_clv_id: str | None
    assigned_cfv_id: str | None
    invalid_reason: str | None = None
    replacement_assignment_created: bool = False


@dataclass(frozen=True)
class CMCDynamicGapResult:
    mv_id: str
    h_tilde: float
    h_upper_cm: float
    xi: float
    x0_m_global: float
    x_ramp_end_global: float
    x_mv_global: float


@dataclass(frozen=True)
class Eq53GapResult:
    mv_id: str
    clv_id: str | None
    cfv_id: str | None
    h_tilde: float
    d_MV_to_CLV: float
    d_CFV_to_MV: float
    threshold: float
    eq53_pass: bool
    fail_side: str | None


@dataclass(frozen=True)
class CMCRecoveryResult:
    mv_id: str
    attempted: bool
    recovery_record: Mapping[str, Any] | None
    safety_pass: bool
    reason: str
    leader_id: str | None = None
    follower_id: str | None = None
    gap_type: str | None = None
    eq53: Eq53GapResult | None = None


@dataclass(frozen=True)
class BoundarySpeedCapResult:
    mv_id: str
    boundary_speed_cap: float | None
    cap_source: str
    cap_reason: str
    cap_feasible: bool
    cap_binding: bool
    current_speed: float


@dataclass(frozen=True)
class Step4BCMCRunResult:
    state: SimulationState
    relations: RelationsSnapshot
    command_buffer: CommandBuffer
    actual_events: list[dict[str, Any]]
    actual_sanity_checks: list[dict[str, Any]]
    expected_png_features: list[dict[str, Any]]


def run_step4b_cmc_for_scenario(
    scenario: str | dict[str, Any],
    *,
    geometry: RoadGeometryConfig = DEFAULT_ROAD_GEOMETRY,
    assignment_views: Mapping[str, AssignmentStepView] | None = None,
) -> Step4BCMCRunResult:
    workspace, config = build_prefreeze_workspace_from_scenario(scenario, geometry=geometry)
    state = freeze_simulation_state(workspace)
    relations = refresh_relations_snapshot(state, geometry=geometry)
    return run_step4b_cmc(
        state,
        relations,
        config=config,
        geometry=geometry,
        assignment_views=assignment_views,
    )


def run_step4b_cmc(
    state: SimulationState,
    relations: RelationsSnapshot,
    *,
    config: dict[str, Any] | None = None,
    geometry: RoadGeometryConfig = DEFAULT_ROAD_GEOMETRY,
    assignment_views: Mapping[str, AssignmentStepView] | None = None,
    eligible_mv_ids: Iterable[str] | None = None,
) -> Step4BCMCRunResult:
    scenario_id = str((config or {}).get("scenario_id") or state.scenario_config_ref or "unknown")
    before_signature = _state_signature(state)
    events: list[dict[str, Any]] = []
    sanity_checks: list[dict[str, Any]] = []
    png_features: list[dict[str, Any]] = []
    longitudinal_commands: dict[str, Any] = {}
    merge_commands: dict[str, Any] = {}
    speed_cap_commands: dict[str, tuple[Any, ...]] = {}
    state_transition_commands: dict[str, tuple[Any, ...]] = {}
    cache_update_commands: list[Any] = []

    for mv_id in _step4b_mv_ids(state, eligible_mv_ids=eligible_mv_ids):
        mv_state = state.vehicle_states[mv_id]
        region = resolve_region(mv_state.x_global, mv_state.road_role, geometry=geometry)
        if mv_state.merge_state == "executing":
            branch_payload = {
                "branch": "cmc_executing_continuation",
                "mv_id": mv_id,
                "x_global": mv_state.x_global,
                "merge_state": mv_state.merge_state,
                "zone_state": region.region_name,
                "assignment_validation_evaluated": False,
                "no_new_eq53_start_decision": True,
            }
            events.append(
                _cmc_event(
                    state,
                    mv_id=mv_id,
                    related_vehicle_ids=(mv_id,),
                    scenario_id=scenario_id,
                    reason="cmc_executing_continuation",
                    source="first_version_engineering_patch",
                    is_engineering_patch=True,
                    payload=branch_payload,
                )
            )
            cap = compute_boundary_speed_cap(state, mv_id, geometry=geometry)
            speed_cap_command = build_boundary_speed_cap_command(state, cap)
            speed_cap_commands[mv_id] = (speed_cap_command,)
            events.append(emit_boundary_cap_event(state, cap, scenario_id=scenario_id))
            merge_command = build_executing_merge_continuation_command(
                state,
                mv_id,
                cap_command_id=speed_cap_command["command_id"],
                geometry=geometry,
            )
            merge_commands[mv_id] = merge_command
            events.append(
                emit_executing_continuation_event(
                    state,
                    mv_id,
                    merge_command,
                    scenario_id=scenario_id,
                )
            )
            sanity_checks.extend(
                _standard_sanity_checks(
                    state,
                    scenario_id=scenario_id,
                    vehicle_ids=(mv_id,),
                    state_unchanged=before_signature == _state_signature(state),
                    boundary_cap=cap,
                    include_assignment_validity=False,
                )
            )
            png_features.extend(
                _png_features(
                    mv_id,
                    "executing_continuation_marker",
                    "boundary_cap_marker",
                )
            )
            continue

        if not region.in_merging_zone:
            events.append(
                _cmc_event(
                    state,
                    mv_id=mv_id,
                    related_vehicle_ids=(mv_id,),
                    scenario_id=scenario_id,
                    reason="handed_off_to_p04",
                    source="first_version_engineering_patch",
                    is_engineering_patch=True,
                    payload={
                        "branch": "handed_off_to_p04",
                        "mv_id": mv_id,
                        "x_global": mv_state.x_global,
                        "merge_state": mv_state.merge_state,
                        "zone_state": region.region_name,
                    },
                )
            )
            continue

        events.append(
            _cmc_event(
                state,
                mv_id=mv_id,
                related_vehicle_ids=(mv_id,),
                scenario_id=scenario_id,
                reason="cmc_waiting_decision",
                source="first_version_engineering_patch",
                is_engineering_patch=True,
                payload={
                    "branch": "cmc_waiting_decision",
                    "mv_id": mv_id,
                    "x_global": mv_state.x_global,
                    "merge_state": mv_state.merge_state,
                    "zone_state": region.region_name,
                },
            )
        )
        source = resolve_cmc_assignment_source(
            state,
            mv_id,
            assignment_views=assignment_views,
        )
        validation = validate_cmc_assignment(state, mv_id, source)
        events.append(emit_assignment_validation_event(state, validation, scenario_id=scenario_id))
        cap = compute_boundary_speed_cap(state, mv_id, geometry=geometry)
        speed_cap_command = build_boundary_speed_cap_command(state, cap)
        speed_cap_commands[mv_id] = (speed_cap_command,)
        events.append(emit_boundary_cap_event(state, cap, scenario_id=scenario_id))
        png_features.extend(_png_features(mv_id, "cmc_decision_marker", "boundary_cap_marker"))

        if not validation.assignment_valid:
            events.append(emit_assignment_invalid_event(state, validation, scenario_id=scenario_id))
            recovery_required = build_assignment_recovery_required_update(
                state,
                source,
                validation,
            )
            if recovery_required is not None:
                cache_update_commands.append(recovery_required)
            recovery = attempt_cmc_recovery_current_gap(
                state,
                mv_id,
                source.record,
                validation,
                geometry=geometry,
            )
            events.append(emit_cmc_recovery_event(state, recovery, scenario_id=scenario_id))
            if recovery.recovery_record is not None:
                cache_update_commands.append(
                    build_assignment_record_update_request(
                        state,
                        mv_id,
                        recovery.recovery_record,
                        reason="cmc_recovery_current_gap",
                    )
                )
            if recovery.safety_pass and recovery.eq53 is not None:
                recovery_validation = CMCAssignmentValidationResult(
                    mv_id=mv_id,
                    assignment_source="cmc_recovery_current_gap",
                    assignment_valid=True,
                    assigned_clv_id=recovery.leader_id,
                    assigned_cfv_id=recovery.follower_id,
                    replacement_assignment_created=True,
                )
                events.append(emit_eq53_gap_event(state, recovery.eq53, scenario_id=scenario_id))
                merge_command = build_merge_start_command(
                    state,
                    recovery_validation,
                    cap_command_id=speed_cap_command["command_id"],
                    geometry=geometry,
                )
                merge_commands[mv_id] = merge_command
                transition = build_cmc_state_transition_request(state, mv_id)
                state_transition_commands[mv_id] = (transition,)
                events.append(
                    emit_merge_start_command_event(
                        state,
                        mv_id,
                        merge_command,
                        transition,
                        scenario_id=scenario_id,
                    )
                )
                png_features.extend(_png_features(mv_id, "merge_start_marker"))
            else:
                waiting_command = build_waiting_command(
                    state,
                    mv_id,
                    reason="assignment_invalid",
                    cap_command_id=speed_cap_command["command_id"],
                )
                longitudinal_commands[mv_id] = waiting_command
                events.append(
                    emit_waiting_command_event(state, mv_id, waiting_command, scenario_id=scenario_id)
                )
            sanity_checks.extend(
                _standard_sanity_checks(
                    state,
                    scenario_id=scenario_id,
                    vehicle_ids=(mv_id,),
                    state_unchanged=before_signature == _state_signature(state),
                    boundary_cap=cap,
                    assignment_invalid_reason=validation.invalid_reason,
                )
            )
            png_features.extend(
                _png_features(
                    mv_id,
                    "assigned_clv_cfv_marker",
                    "assignment_invalid_marker",
                    "waiting_marker",
                    related_vehicle_ids=tuple(
                        item
                        for item in (
                            validation.assigned_clv_id,
                            validation.assigned_cfv_id,
                        )
                        if item is not None
                    ),
                )
            )
            continue

        if source.record is not None and source.record.lifecycle_state in {
            "active_control_zone",
            "refresh_failed_retained",
        }:
            promoted = assignment_lifecycle_manager.promote_to_merge_zone(state, source.record)
            cache_update_commands.append(
                build_assignment_record_update_request(
                    state,
                    mv_id,
                    assignment_lifecycle_manager.to_state_dict(promoted),
                    reason="cmc_promote_to_merge_zone",
                )
            )

        dynamic_gap = compute_cmc_dynamic_acceptable_gap(state, mv_id, geometry=geometry)
        events.append(emit_eq52_dynamic_gap_event(state, dynamic_gap, scenario_id=scenario_id))
        eq53 = check_eq53_gap(state, validation, dynamic_gap)
        events.append(emit_eq53_gap_event(state, eq53, scenario_id=scenario_id))
        related_ids = (eq53.clv_id, eq53.cfv_id)
        png_features.extend(
            _png_features(
                mv_id,
                "assigned_clv_cfv_marker",
                related_vehicle_ids=related_ids,
            )
        )

        if eq53.eq53_pass:
            merge_command = build_merge_start_command(
                state,
                validation,
                cap_command_id=speed_cap_command["command_id"],
                geometry=geometry,
            )
            merge_commands[mv_id] = merge_command
            transition = build_cmc_state_transition_request(state, mv_id)
            state_transition_commands[mv_id] = (transition,)
            events.append(
                emit_merge_start_command_event(
                    state,
                    mv_id,
                    merge_command,
                    transition,
                    scenario_id=scenario_id,
                )
            )
            png_features.extend(_png_features(mv_id, "merge_start_marker"))
        else:
            waiting_command = build_waiting_command(
                state,
                mv_id,
                reason="eq53_gap_failed",
                cap_command_id=speed_cap_command["command_id"],
            )
            longitudinal_commands[mv_id] = waiting_command
            events.append(
                emit_waiting_command_event(state, mv_id, waiting_command, scenario_id=scenario_id)
            )
            png_features.extend(_png_features(mv_id, "waiting_marker"))

        sanity_checks.extend(
            _standard_sanity_checks(
                state,
                scenario_id=scenario_id,
                vehicle_ids=(mv_id,),
                state_unchanged=before_signature == _state_signature(state),
                boundary_cap=cap,
                assignment_invalid_reason=None,
            )
        )

    command_buffer = CommandBuffer(
        step=state.step,
        t=state.t,
        longitudinal_commands=MappingProxyType(longitudinal_commands),
        merge_commands=MappingProxyType(merge_commands),
        speed_cap_commands=MappingProxyType(speed_cap_commands),
        state_transition_commands=MappingProxyType(state_transition_commands),
        cache_update_commands=tuple(cache_update_commands),
    )
    return Step4BCMCRunResult(
        state=state,
        relations=relations,
        command_buffer=command_buffer,
        actual_events=events,
        actual_sanity_checks=sanity_checks,
        expected_png_features=png_features,
    )


def resolve_step4b_cmc_branch(
    state: SimulationState,
    mv_id: str,
    *,
    geometry: RoadGeometryConfig = DEFAULT_ROAD_GEOMETRY,
) -> str:
    mv_state = state.vehicle_states[mv_id]
    if mv_state.merge_state == "executing":
        return "cmc_executing_continuation"
    region = resolve_region(mv_state.x_global, mv_state.road_role, geometry=geometry)
    if region.in_merging_zone:
        return "cmc_waiting_decision"
    return "handed_off_to_p04"


def resolve_cmc_assignment_source(
    state: SimulationState,
    mv_id: str,
    *,
    assignment_views: Mapping[str, AssignmentStepView] | None = None,
) -> CMCAssignmentSource:
    view = (assignment_views or {}).get(mv_id)
    if view is not None and view.consumable_by_cmc:
        record = assignment_lifecycle_manager.from_state_dict(view.record)
        return CMCAssignmentSource(
            mv_id=mv_id,
            assignment=view.record,
            source=view.source,
            record=record,
        )
    record_value = state.assignment_records_by_mv.get(mv_id)
    if record_value is None:
        return CMCAssignmentSource(mv_id=mv_id, assignment=None, source=None)
    record = assignment_lifecycle_manager.from_state_dict(record_value)
    view = assignment_lifecycle_manager.derive_cmc_view(state, record)
    if view is None:
        return CMCAssignmentSource(
            mv_id=mv_id,
            assignment=record_value,
            source=record.source,
            record=record,
        )
    return CMCAssignmentSource(
        mv_id=mv_id,
        assignment=view.record,
        source=view.source,
        record=record,
    )


def validate_cmc_assignment(
    state: SimulationState,
    mv_id: str,
    source: CMCAssignmentSource,
) -> CMCAssignmentValidationResult:
    assignment = source.assignment
    if assignment is None:
        return CMCAssignmentValidationResult(
            mv_id=mv_id,
            assignment_source=source.source,
            assignment_valid=False,
            assigned_clv_id=None,
            assigned_cfv_id=None,
            invalid_reason="clv_missing",
        )
    clv_id = _optional_str(assignment.get("clv_id"))
    cfv_id = _optional_str(assignment.get("cfv_id"))
    status = str(assignment.get("status", "valid")).lower()
    lifecycle_state = str(assignment.get("lifecycle_state") or "").lower()
    if lifecycle_state == "recovery_required":
        return CMCAssignmentValidationResult(
            mv_id=mv_id,
            assignment_source=source.source,
            assignment_valid=False,
            assigned_clv_id=clv_id,
            assigned_cfv_id=cfv_id,
            invalid_reason=(
                _optional_str(assignment.get("invalid_reason"))
                or _optional_str(assignment.get("recovery_reason"))
                or "wrong_order"
            ),
        )
    if status not in {"valid", "available", "ok"}:
        return CMCAssignmentValidationResult(
            mv_id=mv_id,
            assignment_source=source.source,
            assignment_valid=False,
            assigned_clv_id=clv_id,
            assigned_cfv_id=cfv_id,
            invalid_reason="stale_assignment",
        )
    if not clv_id or clv_id not in state.vehicle_states:
        return CMCAssignmentValidationResult(
            mv_id=mv_id,
            assignment_source=source.source,
            assignment_valid=False,
            assigned_clv_id=clv_id,
            assigned_cfv_id=cfv_id,
            invalid_reason="clv_missing",
        )
    if not cfv_id or cfv_id not in state.vehicle_states:
        return CMCAssignmentValidationResult(
            mv_id=mv_id,
            assignment_source=source.source,
            assignment_valid=False,
            assigned_clv_id=clv_id,
            assigned_cfv_id=cfv_id,
            invalid_reason="cfv_missing",
        )
    mv_state = state.vehicle_states[mv_id]
    clv_state = state.vehicle_states[clv_id]
    cfv_state = state.vehicle_states[cfv_id]
    if not clv_state.is_active or not cfv_state.is_active:
        return CMCAssignmentValidationResult(
            mv_id=mv_id,
            assignment_source=source.source,
            assignment_valid=False,
            assigned_clv_id=clv_id,
            assigned_cfv_id=cfv_id,
            invalid_reason="vehicle_exited",
        )
    clv_eligibility = resolve_lane_2_gap_boundary_eligibility(state, clv_id)
    if not clv_eligibility.eligible:
        return CMCAssignmentValidationResult(
            mv_id=mv_id,
            assignment_source=source.source,
            assignment_valid=False,
            assigned_clv_id=clv_id,
            assigned_cfv_id=cfv_id,
            invalid_reason=_cmc_boundary_invalid_reason("clv", clv_eligibility.reason),
        )
    cfv_eligibility = resolve_lane_2_gap_boundary_eligibility(state, cfv_id)
    if not cfv_eligibility.eligible:
        return CMCAssignmentValidationResult(
            mv_id=mv_id,
            assignment_source=source.source,
            assignment_valid=False,
            assigned_clv_id=clv_id,
            assigned_cfv_id=cfv_id,
            invalid_reason=_cmc_boundary_invalid_reason("cfv", cfv_eligibility.reason),
        )
    if not (clv_state.x_global > mv_state.x_global > cfv_state.x_global):
        return CMCAssignmentValidationResult(
            mv_id=mv_id,
            assignment_source=source.source,
            assignment_valid=False,
            assigned_clv_id=clv_id,
            assigned_cfv_id=cfv_id,
            invalid_reason="wrong_order",
        )
    return CMCAssignmentValidationResult(
        mv_id=mv_id,
        assignment_source=source.source,
        assignment_valid=True,
        assigned_clv_id=clv_id,
        assigned_cfv_id=cfv_id,
    )


def compute_cmc_dynamic_acceptable_gap(
    state: SimulationState,
    mv_id: str,
    *,
    geometry: RoadGeometryConfig = DEFAULT_ROAD_GEOMETRY,
    h_upper_cm: float = CMC_UPPER_MERGE_TIME_GAP_S,
    xi: float = CMC_DYNAMIC_GAP_XI,
) -> CMCDynamicGapResult:
    x_mv = state.vehicle_states[mv_id].x_global
    denominator = geometry.x_ramp_end_global - geometry.x0_m_global
    h_tilde = h_upper_cm * (1.0 - xi * ((x_mv - geometry.x0_m_global) / denominator))
    return CMCDynamicGapResult(
        mv_id=mv_id,
        h_tilde=h_tilde,
        h_upper_cm=h_upper_cm,
        xi=xi,
        x0_m_global=geometry.x0_m_global,
        x_ramp_end_global=geometry.x_ramp_end_global,
        x_mv_global=x_mv,
    )


def compute_eq53_gap_inputs(
    state: SimulationState,
    validation: CMCAssignmentValidationResult,
    dynamic_gap: CMCDynamicGapResult,
) -> Eq53GapResult:
    if not validation.assigned_clv_id or not validation.assigned_cfv_id:
        raise ValueError("Eq.53 requires assigned CLV and CFV ids")
    mv_state = state.vehicle_states[validation.mv_id]
    clv_state = state.vehicle_states[validation.assigned_clv_id]
    cfv_state = state.vehicle_states[validation.assigned_cfv_id]
    clv_length = _vehicle_length(state, validation.assigned_clv_id)
    mv_length = _vehicle_length(state, validation.mv_id)
    d_mv_to_clv = clv_state.x_global - mv_state.x_global - clv_length
    d_cfv_to_mv = mv_state.x_global - cfv_state.x_global - mv_length
    threshold = mv_state.v * dynamic_gap.h_tilde
    clv_ok = d_mv_to_clv >= threshold
    cfv_ok = d_cfv_to_mv >= threshold
    if clv_ok and cfv_ok:
        fail_side = None
    elif not clv_ok and not cfv_ok:
        fail_side = "both"
    elif not clv_ok:
        fail_side = "CLV_gap"
    else:
        fail_side = "CFV_gap"
    return Eq53GapResult(
        mv_id=validation.mv_id,
        clv_id=validation.assigned_clv_id,
        cfv_id=validation.assigned_cfv_id,
        h_tilde=dynamic_gap.h_tilde,
        d_MV_to_CLV=d_mv_to_clv,
        d_CFV_to_MV=d_cfv_to_mv,
        threshold=threshold,
        eq53_pass=clv_ok and cfv_ok,
        fail_side=fail_side,
    )


def check_eq53_gap(
    state: SimulationState,
    validation: CMCAssignmentValidationResult,
    dynamic_gap: CMCDynamicGapResult,
) -> Eq53GapResult:
    return compute_eq53_gap_inputs(state, validation, dynamic_gap)


def attempt_cmc_recovery_current_gap(
    state: SimulationState,
    mv_id: str,
    invalid_record: AssignmentRecord | None,
    validation: CMCAssignmentValidationResult,
    *,
    geometry: RoadGeometryConfig = DEFAULT_ROAD_GEOMETRY,
) -> CMCRecoveryResult:
    if validation.invalid_reason == "wrong_order":
        return CMCRecoveryResult(
            mv_id=mv_id,
            attempted=True,
            recovery_record=None,
            safety_pass=False,
            reason="wrong_order_recovery_deferred_to_original_assignment",
        )
    if validation.invalid_reason not in {
        "clv_missing",
        "cfv_missing",
        "clv_not_lane_2",
        "cfv_not_lane_2",
        "vehicle_exited",
    }:
        return CMCRecoveryResult(
            mv_id=mv_id,
            attempted=False,
            recovery_record=None,
            safety_pass=False,
            reason=f"unsupported_recovery_reason:{validation.invalid_reason}",
        )
    leader_id, follower_id = _nearest_lane_2_gap_boundaries(state, mv_id)
    if leader_id is not None and follower_id is not None:
        gap_type = "bounded"
    elif leader_id is not None:
        gap_type = "front_only"
    elif follower_id is not None:
        gap_type = "rear_only"
    else:
        gap_type = "unbounded_clear"
    record = _build_cmc_recovery_record(
        state,
        mv_id,
        invalid_record,
        leader_id=leader_id,
        follower_id=follower_id,
        gap_type=gap_type,
        reason=validation.invalid_reason or "assignment_invalid",
    )
    dynamic_gap = compute_cmc_dynamic_acceptable_gap(state, mv_id, geometry=geometry)
    eq53 = validate_recovery_gap(state, record, dynamic_gap)
    return CMCRecoveryResult(
        mv_id=mv_id,
        attempted=True,
        recovery_record=MappingProxyType(assignment_lifecycle_manager.to_state_dict(record)),
        safety_pass=eq53.eq53_pass,
        reason="recovery_safety_pass" if eq53.eq53_pass else "recovery_safety_failed",
        leader_id=leader_id,
        follower_id=follower_id,
        gap_type=gap_type,
        eq53=eq53,
    )


def validate_recovery_gap(
    state: SimulationState,
    recovery_record: AssignmentRecord,
    dynamic_gap: CMCDynamicGapResult,
) -> Eq53GapResult:
    mv_state = state.vehicle_states[recovery_record.mv_id]
    mv_length = _vehicle_length(state, recovery_record.mv_id)
    threshold = mv_state.v * dynamic_gap.h_tilde
    if recovery_record.clv_id is None:
        d_mv_to_clv = float("inf")
        clv_ok = True
    else:
        clv_state = state.vehicle_states[recovery_record.clv_id]
        clv_length = _vehicle_length(state, recovery_record.clv_id)
        d_mv_to_clv = clv_state.x_global - mv_state.x_global - clv_length
        clv_ok = d_mv_to_clv >= threshold
    if recovery_record.cfv_id is None:
        d_cfv_to_mv = float("inf")
        cfv_ok = True
    else:
        cfv_state = state.vehicle_states[recovery_record.cfv_id]
        d_cfv_to_mv = mv_state.x_global - cfv_state.x_global - mv_length
        cfv_ok = d_cfv_to_mv >= threshold
    if clv_ok and cfv_ok:
        fail_side = None
    elif not clv_ok and not cfv_ok:
        fail_side = "both"
    elif not clv_ok:
        fail_side = "CLV_gap"
    else:
        fail_side = "CFV_gap"
    return Eq53GapResult(
        mv_id=recovery_record.mv_id,
        clv_id=recovery_record.clv_id,
        cfv_id=recovery_record.cfv_id,
        h_tilde=dynamic_gap.h_tilde,
        d_MV_to_CLV=d_mv_to_clv,
        d_CFV_to_MV=d_cfv_to_mv,
        threshold=threshold,
        eq53_pass=clv_ok and cfv_ok,
        fail_side=fail_side,
    )


def compute_boundary_speed_cap(
    state: SimulationState,
    mv_id: str,
    *,
    geometry: RoadGeometryConfig = DEFAULT_ROAD_GEOMETRY,
    standstill_spacing_m: float = STANDSTILL_SPACING_M,
    planned_acceleration_mps2: float = LANE_CHANGE_PLANNED_ACCELERATION_MPS2,
) -> BoundarySpeedCapResult:
    mv_state = state.vehicle_states[mv_id]
    mv_length = _vehicle_length(state, mv_id)
    multiplier = (2.0 * planned_acceleration_mps2 / geometry.lane_width) ** 0.5
    raw_cap = (
        geometry.x_ramp_end_global
        - mv_state.x_global
        - mv_length / 2.0
        - standstill_spacing_m
    ) * multiplier
    if raw_cap < 0:
        return BoundarySpeedCapResult(
            mv_id=mv_id,
            boundary_speed_cap=raw_cap,
            cap_source="boundary_collision_avoidance",
            cap_reason="cap_negative",
            cap_feasible=False,
            cap_binding=True,
            current_speed=mv_state.v,
        )
    return BoundarySpeedCapResult(
        mv_id=mv_id,
        boundary_speed_cap=raw_cap,
        cap_source="boundary_collision_avoidance",
        cap_reason="normal_cap",
        cap_feasible=True,
        cap_binding=raw_cap < mv_state.v,
        current_speed=mv_state.v,
    )


def build_boundary_speed_cap_command(
    state: SimulationState,
    cap: BoundarySpeedCapResult,
) -> dict[str, Any]:
    return {
        "command_id": f"p05:{state.step}:speed_cap:{cap.mv_id}",
        "vehicle_id": cap.mv_id,
        "command_type": "speed_cap",
        "speed_cap": cap.boundary_speed_cap,
        "cap_source": cap.cap_source,
        "cap_reason": cap.cap_reason,
        "cap_feasible": cap.cap_feasible,
        "cap_binding": cap.cap_binding,
        "source": "paper_formula",
    }


def build_waiting_command(
    state: SimulationState,
    mv_id: str,
    *,
    reason: str,
    cap_command_id: str,
) -> dict[str, Any]:
    return {
        "command_id": f"p05:{state.step}:waiting:{mv_id}",
        "vehicle_id": mv_id,
        "command_type": "longitudinal",
        "longitudinal_mode": "cmc_waiting",
        "merge_command_created": False,
        "reason": reason,
        "source_speed_cap_command_id": cap_command_id,
        "source": "first_version_engineering_patch",
        "is_engineering_patch": True,
    }


def build_merge_start_command(
    state: SimulationState,
    validation: CMCAssignmentValidationResult,
    *,
    cap_command_id: str,
    geometry: RoadGeometryConfig = DEFAULT_ROAD_GEOMETRY,
) -> dict[str, Any]:
    target_y = float(geometry.lane_centerlines[LANE_2])
    return {
        "command_id": f"p05:{state.step}:merge_start:{validation.mv_id}",
        "vehicle_id": validation.mv_id,
        "command_type": "merge",
        "init_or_continue_maneuver": "init",
        "target_lane": LANE_2,
        "target_y": target_y,
        "assigned_clv_id": validation.assigned_clv_id,
        "assigned_cfv_id": validation.assigned_cfv_id,
        "source_speed_cap_command_id": cap_command_id,
        "source": "first_version_engineering_patch",
        "is_engineering_patch": True,
    }


def build_executing_merge_continuation_command(
    state: SimulationState,
    mv_id: str,
    *,
    cap_command_id: str,
    geometry: RoadGeometryConfig = DEFAULT_ROAD_GEOMETRY,
) -> dict[str, Any]:
    maneuver = state.active_maneuvers.get(mv_id)
    return {
        "command_id": f"p05:{state.step}:merge_continue:{mv_id}",
        "vehicle_id": mv_id,
        "command_type": "merge",
        "init_or_continue_maneuver": "continue",
        "target_lane": maneuver.target_lane if maneuver is not None else LANE_2,
        "target_y": (
            maneuver.target_y if maneuver is not None else float(geometry.lane_centerlines[LANE_2])
        ),
        "active_maneuver_present": maneuver is not None,
        "no_new_eq53_start_decision": True,
        "does_not_rejudge_merge_start": True,
        "source_speed_cap_command_id": cap_command_id,
        "source": "first_version_engineering_patch",
        "is_engineering_patch": True,
    }


def build_cmc_state_transition_request(
    state: SimulationState,
    mv_id: str,
) -> dict[str, Any]:
    old_state = state.vehicle_states[mv_id].merge_state
    return {
        "command_id": f"p05:{state.step}:merge_state:{mv_id}",
        "vehicle_id": mv_id,
        "state_name": "merge_state",
        "old_state": old_state,
        "new_state": "executing",
        "reason": "eq53_gap_pass",
        "source": "first_version_engineering_patch",
        "is_engineering_patch": True,
    }


def build_assignment_recovery_required_update(
    state: SimulationState,
    source: CMCAssignmentSource,
    validation: CMCAssignmentValidationResult,
) -> dict[str, Any] | None:
    if source.record is None:
        return None
    record = assignment_lifecycle_manager.mark_recovery_required(
        state,
        source.record,
        validation.invalid_reason or "assignment_invalid",
    )
    return build_assignment_record_update_request(
        state,
        validation.mv_id,
        assignment_lifecycle_manager.to_state_dict(record),
        reason=validation.invalid_reason or "assignment_invalid",
    )


def build_assignment_record_update_request(
    state: SimulationState,
    mv_id: str,
    record: Mapping[str, Any],
    *,
    reason: str,
) -> dict[str, Any]:
    return {
        "command_id": f"p05:{state.step}:assignment_record:{mv_id}:{reason}",
        "cache_name": "assignment_records_by_mv",
        "owner_vehicle_id": mv_id,
        "operation": "update",
        "new_value": MappingProxyType(dict(record)),
        "reason": reason,
        "source": "first_version_engineering_patch",
        "is_engineering_patch": True,
    }


def emit_assignment_validation_event(
    state: SimulationState,
    validation: CMCAssignmentValidationResult,
    *,
    scenario_id: str,
) -> dict[str, Any]:
    related = tuple(
        item
        for item in (
            validation.mv_id,
            validation.assigned_clv_id,
            validation.assigned_cfv_id,
        )
        if item is not None
    )
    return _cmc_event(
        state,
        mv_id=validation.mv_id,
        related_vehicle_ids=related,
        scenario_id=scenario_id,
        reason="assignment_validation",
        source="first_version_engineering_patch",
        is_engineering_patch=True,
        payload={
            **_dataclass_to_plain(validation),
            "assignment_validation_evaluated": True,
        },
    )


def emit_assignment_invalid_event(
    state: SimulationState,
    validation: CMCAssignmentValidationResult,
    *,
    scenario_id: str,
) -> dict[str, Any]:
    related = tuple(
        item
        for item in (
            validation.mv_id,
            validation.assigned_clv_id,
            validation.assigned_cfv_id,
        )
        if item is not None
    )
    return _event(
        state,
        module="CMC",
        event_type="assignment_invalid",
        vehicle_id=validation.mv_id,
        related_vehicle_ids=related,
        scenario_id=scenario_id,
        reason=validation.invalid_reason or "unknown",
        source="first_version_engineering_patch",
        is_engineering_patch=True,
        payload={
            "mv_id": validation.mv_id,
            "reason": validation.invalid_reason or "unknown",
            "assigned_clv_id": validation.assigned_clv_id,
            "assigned_cfv_id": validation.assigned_cfv_id,
            "Eq53_evaluated": False,
            "merge_command_created": False,
            "replacement_assignment_created": False,
            "source": "first_version_engineering_patch",
            "is_engineering_patch": True,
        },
    )


def emit_cmc_recovery_event(
    state: SimulationState,
    recovery: CMCRecoveryResult,
    *,
    scenario_id: str,
) -> dict[str, Any]:
    related = tuple(
        item
        for item in (recovery.mv_id, recovery.leader_id, recovery.follower_id)
        if item is not None
    )
    return _cmc_event(
        state,
        mv_id=recovery.mv_id,
        related_vehicle_ids=related,
        scenario_id=scenario_id,
        reason="cmc_recovery_current_gap",
        source="first_version_engineering_patch",
        is_engineering_patch=True,
        payload={
            "mv_id": recovery.mv_id,
            "attempted": recovery.attempted,
            "reason": recovery.reason,
            "leader_id": recovery.leader_id,
            "follower_id": recovery.follower_id,
            "gap_type": recovery.gap_type,
            "safety_pass": recovery.safety_pass,
            "replacement_assignment_created": recovery.recovery_record is not None,
            "recovery_record": dict(recovery.recovery_record or {}),
            "eq53": _dataclass_to_plain(recovery.eq53) if recovery.eq53 is not None else None,
        },
    )


def emit_eq52_dynamic_gap_event(
    state: SimulationState,
    dynamic_gap: CMCDynamicGapResult,
    *,
    scenario_id: str,
) -> dict[str, Any]:
    return _cmc_event(
        state,
        mv_id=dynamic_gap.mv_id,
        related_vehicle_ids=(dynamic_gap.mv_id,),
        scenario_id=scenario_id,
        reason="eq52_dynamic_gap",
        source="paper_formula",
        is_engineering_patch=False,
        payload=_dataclass_to_plain(dynamic_gap),
    )


def emit_eq53_gap_event(
    state: SimulationState,
    eq53: Eq53GapResult,
    *,
    scenario_id: str,
) -> dict[str, Any]:
    return _cmc_event(
        state,
        mv_id=eq53.mv_id,
        related_vehicle_ids=(eq53.mv_id, eq53.clv_id, eq53.cfv_id),
        scenario_id=scenario_id,
        reason="eq53_gap",
        source="paper_formula",
        is_engineering_patch=False,
        payload=_dataclass_to_plain(eq53),
    )


def emit_boundary_cap_event(
    state: SimulationState,
    cap: BoundarySpeedCapResult,
    *,
    scenario_id: str,
) -> dict[str, Any]:
    return _cmc_event(
        state,
        mv_id=cap.mv_id,
        related_vehicle_ids=(cap.mv_id,),
        scenario_id=scenario_id,
        reason="boundary_speed_cap",
        source="paper_formula",
        is_engineering_patch=False,
        payload=_dataclass_to_plain(cap),
    )


def emit_waiting_command_event(
    state: SimulationState,
    mv_id: str,
    waiting_command: Mapping[str, Any],
    *,
    scenario_id: str,
) -> dict[str, Any]:
    return _cmc_event(
        state,
        mv_id=mv_id,
        related_vehicle_ids=(mv_id,),
        scenario_id=scenario_id,
        reason="waiting_command",
        source="first_version_engineering_patch",
        is_engineering_patch=True,
        payload=dict(waiting_command),
    )


def emit_merge_start_command_event(
    state: SimulationState,
    mv_id: str,
    merge_command: Mapping[str, Any],
    transition: Mapping[str, Any],
    *,
    scenario_id: str,
) -> dict[str, Any]:
    return _cmc_event(
        state,
        mv_id=mv_id,
        related_vehicle_ids=(mv_id,),
        scenario_id=scenario_id,
        reason="merge_start",
        source="first_version_engineering_patch",
        is_engineering_patch=True,
        payload={
            **dict(merge_command),
            "state_transition_request": transition["new_state"],
            "state_transition_command_id": transition["command_id"],
        },
    )


def emit_executing_continuation_event(
    state: SimulationState,
    mv_id: str,
    merge_command: Mapping[str, Any],
    *,
    scenario_id: str,
) -> dict[str, Any]:
    return _cmc_event(
        state,
        mv_id=mv_id,
        related_vehicle_ids=(mv_id,),
        scenario_id=scenario_id,
        reason="executing_continuation",
        source="first_version_engineering_patch",
        is_engineering_patch=True,
        payload=dict(merge_command),
    )


def run_cmc_assignment_sanity(
    state: SimulationState,
    *,
    scenario_id: str,
    vehicle_ids: tuple[str, ...],
    invalid_reason: str | None,
) -> dict[str, Any]:
    return _sanity(
        state,
        "assignment_invalid",
        "warning" if invalid_reason is not None else "pass",
        scenario_id=scenario_id,
        vehicle_ids=vehicle_ids,
        reason=invalid_reason or "assignment_valid",
        payload={
            "assignment_invalid": invalid_reason is not None,
            "invalid_reason": invalid_reason,
            "source": "first_version_engineering_patch",
            "is_engineering_patch": invalid_reason is not None,
        },
    )


def run_cmc_boundary_cap_sanity(
    state: SimulationState,
    *,
    scenario_id: str,
    vehicle_ids: tuple[str, ...],
    boundary_cap: BoundarySpeedCapResult,
) -> dict[str, Any]:
    return _sanity(
        state,
        "boundary_violation",
        "pass" if boundary_cap.cap_feasible else "warning",
        scenario_id=scenario_id,
        vehicle_ids=vehicle_ids,
        reason=boundary_cap.cap_reason,
        payload=_dataclass_to_plain(boundary_cap),
    )


def run_p05_no_write_before_commit_sanity(
    state: SimulationState,
    *,
    scenario_id: str,
    vehicle_ids: tuple[str, ...],
    state_unchanged: bool,
) -> dict[str, Any]:
    return _sanity(
        state,
        "no_write_before_commit",
        "pass" if state_unchanged else "fail",
        scenario_id=scenario_id,
        vehicle_ids=vehicle_ids,
        reason="p05_outputs_are_commands_events_and_sanity_only",
        payload={"p05_no_write_before_commit": state_unchanged},
    )


def register_p05_png_features(
    mv_id: str,
    *feature_types: str,
    related_vehicle_ids: tuple[str, ...] = (),
) -> list[dict[str, Any]]:
    vehicle_ids = [mv_id, *related_vehicle_ids]
    return [
        {
            "feature_type": feature_type,
            "required": True,
            "vehicle_ids": vehicle_ids,
            "expected_visibility": "visible",
            "notes": "registered only; renderer deferred",
        }
        for feature_type in feature_types
    ]


def _standard_sanity_checks(
    state: SimulationState,
    *,
    scenario_id: str,
    vehicle_ids: tuple[str, ...],
    state_unchanged: bool,
    boundary_cap: BoundarySpeedCapResult,
    assignment_invalid_reason: str | None = None,
    include_assignment_validity: bool = True,
) -> list[dict[str, Any]]:
    checks: list[dict[str, Any]] = []
    if include_assignment_validity:
        checks.append(
            run_cmc_assignment_sanity(
                state,
                scenario_id=scenario_id,
                vehicle_ids=vehicle_ids,
                invalid_reason=assignment_invalid_reason,
            )
        )
    checks.extend(
        [
            run_cmc_boundary_cap_sanity(
                state,
                scenario_id=scenario_id,
                vehicle_ids=vehicle_ids,
                boundary_cap=boundary_cap,
            ),
            run_p05_no_write_before_commit_sanity(
                state,
                scenario_id=scenario_id,
                vehicle_ids=vehicle_ids,
                state_unchanged=state_unchanged,
            ),
            _sanity(
                state,
                "x_plot_used_in_algorithm_path",
                "pass" if assert_x_plot_not_used_in_algorithm_path(state) else "fail",
                scenario_id=scenario_id,
                vehicle_ids=vehicle_ids,
                reason="x_global_only_algorithm_path",
                payload={"x_plot_used_in_algorithm_path": False},
            ),
            _sanity(
                state,
                "state_machine_inconsistency",
                _state_machine_status(state, vehicle_ids),
                scenario_id=scenario_id,
                vehicle_ids=vehicle_ids,
                reason="lane_change_and_merge_executing_exclusive",
            ),
        ]
    )
    return checks


def _step4b_mv_ids(
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


def _nearest_lane_2_gap_boundaries(
    state: SimulationState,
    mv_id: str,
) -> tuple[str | None, str | None]:
    mv_x = state.vehicle_states[mv_id].x_global
    leader_id: str | None = None
    follower_id: str | None = None
    leader_dx: float | None = None
    follower_dx: float | None = None
    for vehicle_id, vehicle_state in state.vehicle_states.items():
        if vehicle_id == mv_id:
            continue
        if vehicle_state.physical_lane != LANE_2 or not vehicle_state.is_active:
            continue
        if vehicle_state.lane_change_state == "executing":
            continue
        dx = vehicle_state.x_global - mv_x
        if dx >= 0.0:
            if leader_dx is None or dx < leader_dx:
                leader_id = vehicle_id
                leader_dx = dx
        else:
            behind_dx = -dx
            if follower_dx is None or behind_dx < follower_dx:
                follower_id = vehicle_id
                follower_dx = behind_dx
    return leader_id, follower_id


def _build_cmc_recovery_record(
    state: SimulationState,
    mv_id: str,
    invalid_record: AssignmentRecord | None,
    *,
    leader_id: str | None,
    follower_id: str | None,
    gap_type: str,
    reason: str,
) -> AssignmentRecord:
    base = (
        assignment_lifecycle_manager.to_state_dict(invalid_record)
        if invalid_record is not None
        else {
            "record_version": 1,
            "mv_id": mv_id,
            "aps_case": None,
            "col_clv": False,
            "col_cfv": False,
            "desired_spacing_override": None,
            "created_at_step": state.step,
            "created_at_t": state.t,
        }
    )
    base.update(
        {
            "mv_id": mv_id,
            "clv_id": leader_id,
            "cfv_id": follower_id,
            "gap_type": gap_type,
            "status": "valid",
            "lifecycle_state": "active_merge_zone",
            "last_updated_step": state.step,
            "last_updated_t": state.t,
            "last_validated_step": state.step,
            "last_validated_t": state.t,
            "invalid_reason": None,
            "recovery_reason": reason,
            "source": "cmc_recovery_current_gap",
            "col_clv": False,
            "col_cfv": False,
        }
    )
    return assignment_lifecycle_manager.from_state_dict(base)


def _vehicle_length(state: SimulationState, vehicle_id: str) -> float:
    spec = state.vehicle_specs.get(vehicle_id)
    if spec is None:
        return PAPER_VEHICLE_LENGTH_M
    return float(spec.length)


def _optional_str(value: Any) -> str | None:
    if value in (None, ""):
        return None
    return str(value)


def _cmc_boundary_invalid_reason(role: str, reason: str) -> str:
    if reason == "lane_change_executing":
        return f"{role}_lane_change_executing"
    if reason == "inactive_vehicle":
        return "vehicle_exited"
    if reason == "missing":
        return f"{role}_missing"
    if reason.startswith("not_"):
        return f"{role}_not_lane_2"
    return f"{role}_{reason}"


def _png_features(
    mv_id: str,
    *feature_types: str,
    related_vehicle_ids: tuple[str, ...] = (),
) -> list[dict[str, Any]]:
    return register_p05_png_features(
        mv_id,
        *feature_types,
        related_vehicle_ids=related_vehicle_ids,
    )


def _cmc_event(
    state: SimulationState,
    *,
    mv_id: str,
    related_vehicle_ids: tuple[str, ...],
    scenario_id: str,
    reason: str,
    source: str,
    is_engineering_patch: bool,
    payload: dict[str, Any],
) -> dict[str, Any]:
    return _event(
        state,
        module="CMC",
        event_type="CMC",
        vehicle_id=mv_id,
        related_vehicle_ids=related_vehicle_ids,
        scenario_id=scenario_id,
        reason=reason,
        source=source,
        is_engineering_patch=is_engineering_patch,
        payload=payload,
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
    scenario_id: str,
    vehicle_ids: tuple[str, ...],
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


def _state_machine_status(state: SimulationState, vehicle_ids: tuple[str, ...]) -> str:
    for vehicle_id in vehicle_ids:
        vehicle_state = state.vehicle_states[vehicle_id]
        if (
            vehicle_state.lane_change_state == "executing"
            and vehicle_state.merge_state == "executing"
        ):
            return "fail"
    return "pass"


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


def _dataclass_to_plain(value: Any) -> dict[str, Any]:
    return {field.name: getattr(value, field.name) for field in fields(value)}
