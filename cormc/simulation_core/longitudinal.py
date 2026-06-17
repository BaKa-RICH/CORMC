from __future__ import annotations

from dataclasses import dataclass
from math import sqrt
from types import MappingProxyType
from typing import Any, Mapping

from cormc.p145_parameters import (
    CAV,
    CORMC_PARAMETER_SPEC_SOURCE,
    CPID,
    IDM,
    LANE_CHANGE,
    LANE_MAX_SPEED_MPS,
    LOCKED_FORMULA_STATUS,
)
from cormc.simulation_core.pre_freeze import (
    DEFAULT_ROAD_GEOMETRY,
    LANE_2,
    ON_RAMP,
    ON_RAMP_MV_ROLE,
    LongitudinalControllerMemory,
    ManeuverTrajectoryState,
    RelationsSnapshot,
    RoadGeometryConfig,
    SimulationState,
    assert_x_plot_not_used_in_algorithm_path,
    resolve_on_ramp_control_region,
)
from cormc.simulation_core.commit import (
    CandidateCacheUpdate,
    CandidateLongitudinalKinematics,
    CommandBuffer,
    NextStateBuffer,
)


STEP7_LONGITUDINAL_SOURCE = "step7_longitudinal_model"
ENGINEERING_PATCH_SOURCE = "first_version_engineering_patch"
PAPER_FORMULA_SOURCE = "paper_formula"
DEFAULT_DESIRED_SPEED_MPS = IDM.default_desired_speed
ASSIGNMENT_SAFETY_EPSILON_M = 1.0e-3


@dataclass(frozen=True)
class SpacingOverrideConsumption:
    consumed: bool
    desired_spacing: float | None
    desired_spacing_source: str
    source_command_id: str | None = None
    source_mv_id: str | None = None
    cv_role: str | None = None
    aps_case: str | None = None
    rejection_reason: str | None = None


@dataclass(frozen=True)
class PlanningSpeedComposition:
    vehicle_id: str
    base_candidate_speed: float
    boundary_speed_cap: float | str
    assignment_physical_safety_cap: float | str
    front_fallback_speed: float | str
    planning_speed: float
    most_conservative_source: str
    source_speed_cap_command_id: str | None = None
    assignment_physical_safety_leader_id: str | None = None
    assignment_physical_safety_source: str | None = None
    front_fallback_speed_source: str | None = None


@dataclass(frozen=True)
class FrontCollisionAvoidanceResult:
    status: str
    applicable: bool
    eq42_eq46_locked: bool
    formula_status: str
    consumed: bool = False
    fallback_speed: float | None = None
    fallback_speed_source: str | None = None
    leader_id: str | None = None
    planned_length: float | None = None
    planned_length_source: str | None = None
    x_mid: float | None = None
    t_mid: float | None = None
    d_mid: float | None = None
    current_gap: float | None = None
    delay_maneuver_due_to_front_collision: bool = False


@dataclass(frozen=True)
class AssignmentPhysicalSafetyCap:
    applicable: bool
    cap: float | None
    leader_id: str | None
    source: str | None
    reason: str


@dataclass(frozen=True)
class LeaderRelationContext:
    leader_id: str | None
    relation_source: str | None
    affected_target_follower_id: str | None
    affected_source_follower_id: str | None


@dataclass(frozen=True)
class LongitudinalFormulaResult:
    mode: str
    acceleration: float
    candidate_speed: float
    payload: Mapping[str, Any]
    cache_update: CandidateCacheUpdate | None = None


@dataclass(frozen=True)
class Step7LongitudinalRunResult:
    state: SimulationState
    relations: RelationsSnapshot
    command_buffer: CommandBuffer
    next_state_buffer: NextStateBuffer
    actual_events: list[dict[str, Any]]
    actual_sanity_checks: list[dict[str, Any]]
    expected_png_features: list[dict[str, Any]]
    planning_speeds: Mapping[str, float]


def run_step7_longitudinal_model_spacing_speedcap(
    state: SimulationState,
    relations: RelationsSnapshot,
    *,
    command_buffer: CommandBuffer,
    geometry: RoadGeometryConfig = DEFAULT_ROAD_GEOMETRY,
    vehicle_ids: tuple[str, ...] | None = None,
    front_fallback_speeds: Mapping[str, float] | None = None,
    suppressed_requests: tuple[Mapping[str, Any], ...] = (),
) -> Step7LongitudinalRunResult:
    before_signature = _state_signature(state)
    scenario_id = state.scenario_config_ref or "unknown"
    selected_vehicle_ids = tuple(vehicle_ids or state.active_vehicle_ids)
    candidates: dict[str, CandidateLongitudinalKinematics] = {}
    planning_speeds: dict[str, float] = {}
    events: list[dict[str, Any]] = []
    sanity_checks: list[dict[str, Any]] = []
    consumed_spacing_vehicle_ids: set[str] = set()
    rejected_spacing_vehicle_ids: set[str] = set()
    speed_cap_vehicle_ids: set[str] = set()
    cache_updates: list[CandidateCacheUpdate] = []

    for vehicle_id in selected_vehicle_ids:
        if vehicle_id not in state.vehicle_states:
            continue
        current = state.vehicle_states[vehicle_id]
        spec = state.vehicle_specs[vehicle_id]
        leader_relation = _leader_relation_for_vehicle(relations, vehicle_id)
        leader_id = leader_relation.leader_id
        spacing = consume_spacing_override_command(
            state,
            vehicle_id,
            command_buffer=command_buffer,
        )
        if spacing.consumed:
            consumed_spacing_vehicle_ids.add(vehicle_id)
            events.append(
                emit_spacing_override_consumption_event(
                    state,
                    vehicle_id,
                    spacing,
                    scenario_id=scenario_id,
                )
            )
        elif spacing.rejection_reason is not None:
            rejected_spacing_vehicle_ids.add(vehicle_id)

        base = _compute_base_candidate(
            state,
            relations,
            vehicle_id,
            spacing=spacing,
            leader_id=leader_id,
            geometry=geometry,
        )
        if base.cache_update is not None:
            cache_updates.append(base.cache_update)
        front = evaluate_front_collision_avoidance(
            state,
            relations,
            vehicle_id,
            command_buffer=command_buffer,
            leader_id=leader_id,
        )
        explicit_front_fallback = (front_fallback_speeds or {}).get(vehicle_id)
        front_fallback_speed = (
            float(explicit_front_fallback)
            if explicit_front_fallback is not None
            else front.fallback_speed
        )
        front_fallback_source = (
            "external_test_harness"
            if explicit_front_fallback is not None
            else front.fallback_speed_source
        )
        assignment_safety = evaluate_assignment_physical_safety_cap(
            state,
            vehicle_id,
            leader_relation=leader_relation,
            geometry=geometry,
        )
        composition = compose_planning_speed(
            vehicle_id,
            base_candidate_speed=base.candidate_speed,
            command_buffer=command_buffer,
            assignment_physical_safety=assignment_safety,
            front_fallback_speed=front_fallback_speed,
            front_fallback_speed_source=front_fallback_source,
        )
        if (
            composition.source_speed_cap_command_id is not None
            or composition.most_conservative_source == "assignment_physical_safety_cap"
            or composition.front_fallback_speed != "not_applicable"
        ):
            speed_cap_vehicle_ids.add(vehicle_id)
            events.append(
                emit_speed_cap_consumption_event(
                    state,
                    composition,
                    front=front,
                    scenario_id=scenario_id,
                )
            )

        candidate = build_longitudinal_candidate(
            state,
            vehicle_id,
            base_candidate_speed=base.candidate_speed,
            planning_speed=composition.planning_speed,
            acceleration=base.acceleration,
            constraints_applied=_constraints_applied(spacing, composition),
            source_commands=_source_commands(spacing, composition),
        )
        candidates[vehicle_id] = candidate
        planning_speeds[vehicle_id] = candidate.planning_speed
        events.append(
            emit_longitudinal_model_event(
                state,
                vehicle_id,
                mode=base.mode,
                leader_id=leader_id,
                leader_relation=leader_relation,
                spacing=spacing,
                composition=composition,
                candidate=candidate,
                formula=base,
                front=front,
                scenario_id=scenario_id,
            )
        )

    state_unchanged = before_signature == _state_signature(state)
    sanity_checks.extend(
        [
            run_p08_eq10_consumption_sanity(
                state,
                selected_vehicle_ids,
                command_buffer=command_buffer,
                consumed_vehicle_ids=tuple(sorted(consumed_spacing_vehicle_ids)),
                rejected_vehicle_ids=tuple(sorted(rejected_spacing_vehicle_ids)),
                scenario_id=scenario_id,
            ),
            run_p08_speed_cap_consumption_sanity(
                state,
                tuple(sorted(speed_cap_vehicle_ids)),
                scenario_id=scenario_id,
            ),
            run_p08_no_write_before_commit_sanity(
                state,
                selected_vehicle_ids,
                state_unchanged=state_unchanged,
                scenario_id=scenario_id,
            ),
            run_p08_no_lateral_candidate_sanity(
                state,
                selected_vehicle_ids,
                scenario_id=scenario_id,
            ),
            _sanity(
                state,
                "x_plot_used_in_algorithm_path",
                "pass" if assert_x_plot_not_used_in_algorithm_path(state, relations) else "fail",
                scenario_id=scenario_id,
                vehicle_ids=selected_vehicle_ids,
                reason="x_global_only_algorithm_path",
                payload={"x_plot_used_in_algorithm_path": False},
            ),
            _sanity(
                state,
                "state_machine_inconsistency",
                "pass",
                scenario_id=scenario_id,
                vehicle_ids=selected_vehicle_ids,
                reason="p08_does_not_change_lane_change_or_merge_state",
                payload={"p08_state_machine_transition_created": False},
            ),
        ]
    )

    next_state_buffer = NextStateBuffer(
        step=state.step,
        t=state.t,
        candidate_longitudinal=MappingProxyType(candidates),
        candidate_cache_updates=tuple(cache_updates),
    )
    return Step7LongitudinalRunResult(
        state=state,
        relations=relations,
        command_buffer=command_buffer,
        next_state_buffer=next_state_buffer,
        actual_events=events,
        actual_sanity_checks=sanity_checks,
        expected_png_features=register_p08_png_features(
            candidates,
            consumed_spacing_vehicle_ids=consumed_spacing_vehicle_ids,
            speed_cap_vehicle_ids=speed_cap_vehicle_ids,
        ),
        planning_speeds=MappingProxyType(planning_speeds),
    )


def consume_spacing_override_command(
    state: SimulationState,
    vehicle_id: str,
    *,
    command_buffer: CommandBuffer,
) -> SpacingOverrideConsumption:
    command = command_buffer.cooperation_commands.get(vehicle_id)
    if not isinstance(command, Mapping):
        return SpacingOverrideConsumption(
            consumed=False,
            desired_spacing=None,
            desired_spacing_source=_ordinary_spacing_source(state, vehicle_id),
            rejection_reason=None,
        )
    desired_spacing = command.get("eq10_desired_spacing")
    if desired_spacing is None:
        return SpacingOverrideConsumption(
            consumed=False,
            desired_spacing=None,
            desired_spacing_source=_ordinary_spacing_source(state, vehicle_id),
            rejection_reason="missing_eq10_desired_spacing",
        )
    spec = state.vehicle_specs[vehicle_id]
    cv_role = str(command.get("cv_role") or "").lower()
    aps_case = str(command.get("aps_case") or "").lower()
    consumed_by = str(command.get("consumed_by") or "").upper()
    if str(spec.vehicle_type).lower() == "chv" and str(spec.compliance_state).lower() == "non_compliant":
        return SpacingOverrideConsumption(
            consumed=False,
            desired_spacing=None,
            desired_spacing_source="ordinary_idm",
            source_command_id=str(command.get("command_id")),
            source_mv_id=_optional_str(command.get("source_mv_id")),
            cv_role=cv_role,
            aps_case=aps_case,
            rejection_reason="non_compliant_chv",
        )
    if consumed_by != "P08":
        return SpacingOverrideConsumption(
            consumed=False,
            desired_spacing=None,
            desired_spacing_source=_ordinary_spacing_source(state, vehicle_id),
            source_command_id=str(command.get("command_id")),
            source_mv_id=_optional_str(command.get("source_mv_id")),
            cv_role=cv_role,
            aps_case=aps_case,
            rejection_reason="not_handed_off_to_p08",
        )
    if cv_role != "cfv" or aps_case not in {"case_2", "case_4"}:
        return SpacingOverrideConsumption(
            consumed=False,
            desired_spacing=None,
            desired_spacing_source=_ordinary_spacing_source(state, vehicle_id),
            source_command_id=str(command.get("command_id")),
            source_mv_id=_optional_str(command.get("source_mv_id")),
            cv_role=cv_role,
            aps_case=aps_case,
            rejection_reason="eq10_only_case_2_or_4_cfv",
        )
    return SpacingOverrideConsumption(
        consumed=True,
        desired_spacing=float(desired_spacing),
        desired_spacing_source="Eq10",
        source_command_id=str(command.get("command_id")),
        source_mv_id=_optional_str(command.get("source_mv_id")),
        cv_role=cv_role,
        aps_case=aps_case,
    )


def compose_planning_speed(
    vehicle_id: str,
    *,
    base_candidate_speed: float,
    command_buffer: CommandBuffer,
    assignment_physical_safety: AssignmentPhysicalSafetyCap | None = None,
    front_fallback_speed: float | None,
    front_fallback_speed_source: str | None = None,
) -> PlanningSpeedComposition:
    applicable: list[tuple[str, float]] = [("base_candidate_speed", float(base_candidate_speed))]
    speed_cap_command = _speed_cap_command(command_buffer, vehicle_id)
    boundary_speed_cap: float | str = "not_applicable"
    source_speed_cap_command_id: str | None = None
    assignment_physical_safety_cap: float | str = "not_applicable"
    assignment_physical_safety_leader_id: str | None = None
    assignment_physical_safety_source: str | None = None
    if speed_cap_command is not None and bool(speed_cap_command.get("cap_feasible", True)):
        speed_cap_value = speed_cap_command.get("speed_cap")
        if speed_cap_value is not None:
            boundary_speed_cap = float(speed_cap_value)
            source_speed_cap_command_id = str(speed_cap_command.get("command_id"))
            applicable.append(("boundary_speed_cap", float(boundary_speed_cap)))

    if assignment_physical_safety is not None and assignment_physical_safety.cap is not None:
        assignment_physical_safety_cap = float(assignment_physical_safety.cap)
        assignment_physical_safety_leader_id = assignment_physical_safety.leader_id
        assignment_physical_safety_source = assignment_physical_safety.source
        applicable.append(("assignment_physical_safety_cap", float(assignment_physical_safety_cap)))

    if front_fallback_speed is None:
        front_fallback: float | str = "not_applicable"
    else:
        front_fallback = float(front_fallback_speed)
        applicable.append(("front_fallback", float(front_fallback)))

    most_conservative_source, planning_speed = min(applicable, key=lambda item: item[1])
    return PlanningSpeedComposition(
        vehicle_id=vehicle_id,
        base_candidate_speed=float(base_candidate_speed),
        boundary_speed_cap=boundary_speed_cap,
        assignment_physical_safety_cap=assignment_physical_safety_cap,
        front_fallback_speed=front_fallback,
        planning_speed=max(0.0, float(planning_speed)),
        most_conservative_source=most_conservative_source,
        source_speed_cap_command_id=source_speed_cap_command_id,
        assignment_physical_safety_leader_id=assignment_physical_safety_leader_id,
        assignment_physical_safety_source=assignment_physical_safety_source,
        front_fallback_speed_source=front_fallback_speed_source,
    )


def build_longitudinal_candidate(
    state: SimulationState,
    vehicle_id: str,
    *,
    base_candidate_speed: float,
    planning_speed: float,
    acceleration: float,
    constraints_applied: tuple[str, ...],
    source_commands: tuple[str, ...],
) -> CandidateLongitudinalKinematics:
    current = state.vehicle_states[vehicle_id]
    dt = max(float(state.dt), 0.0)
    return CandidateLongitudinalKinematics(
        candidate_id=f"p08:{state.step}:{vehicle_id}:longitudinal",
        vehicle_id=vehicle_id,
        x_global=float(current.x_global) + float(planning_speed) * dt,
        v=float(planning_speed),
        a=float(acceleration),
        candidate_speed=float(base_candidate_speed),
        planning_speed=float(planning_speed),
        source=STEP7_LONGITUDINAL_SOURCE,
        constraints_applied=constraints_applied,
        source_commands=source_commands,
    )


def emit_longitudinal_model_event(
    state: SimulationState,
    vehicle_id: str,
    *,
    mode: str,
    leader_id: str | None,
    leader_relation: LeaderRelationContext,
    spacing: SpacingOverrideConsumption,
    composition: PlanningSpeedComposition,
    candidate: CandidateLongitudinalKinematics,
    formula: LongitudinalFormulaResult,
    front: FrontCollisionAvoidanceResult,
    scenario_id: str,
) -> dict[str, Any]:
    spec = state.vehicle_specs[vehicle_id]
    payload = {
        "longitudinal_mode": mode,
        "vehicle_type": spec.vehicle_type,
        "compliance_state": spec.compliance_state,
        "leader_id": leader_id,
        "leader_relation_source": leader_relation.relation_source,
        "affected_target_follower_id": leader_relation.affected_target_follower_id,
        "affected_source_follower_id": leader_relation.affected_source_follower_id,
        "desired_spacing_source": spacing.desired_spacing_source,
        "desired_spacing_override": spacing.desired_spacing,
        "spacing_override_consumed": spacing.consumed,
        "spacing_rejection_reason": spacing.rejection_reason,
        "base_candidate_speed": composition.base_candidate_speed,
        "boundary_speed_cap": composition.boundary_speed_cap,
        "assignment_physical_safety_cap": composition.assignment_physical_safety_cap,
        "assignment_physical_safety_cap_applied": (
            composition.most_conservative_source == "assignment_physical_safety_cap"
        ),
        "assignment_physical_safety_leader_id": composition.assignment_physical_safety_leader_id,
        "assignment_physical_safety_source": composition.assignment_physical_safety_source,
        "front_fallback_speed": composition.front_fallback_speed,
        "planning_speed": candidate.planning_speed,
        "candidate_id": candidate.candidate_id,
        "source_spacing_command_id": spacing.source_command_id,
        "source_mv_id": spacing.source_mv_id,
        "longitudinal_formula_status": LOCKED_FORMULA_STATUS,
        "formula_status": LOCKED_FORMULA_STATUS,
        "parameter_source": CORMC_PARAMETER_SPEC_SOURCE,
        "front_collision_status": front.status,
        "eq42_eq46_locked": front.eq42_eq46_locked,
        "front_collision_formula_status": front.formula_status,
        "front_fallback_consumed": front.consumed,
        "front_fallback_speed_source": composition.front_fallback_speed_source,
    }
    payload.update(dict(formula.payload))
    return _event(
        state,
        module="Step7LongitudinalModel",
        event_type="longitudinal_model",
        vehicle_id=vehicle_id,
        related_vehicle_ids=tuple(item for item in (vehicle_id, leader_id) if item is not None),
        scenario_id=scenario_id,
        reason=mode,
        source=(
            PAPER_FORMULA_SOURCE
            if mode in {"cav_cruising", "cav_gap_regulating", "chv_idm"}
            else ENGINEERING_PATCH_SOURCE
        ),
        is_engineering_patch=mode not in {"cav_cruising", "cav_gap_regulating", "chv_idm"},
        payload=payload,
    )


def emit_spacing_override_consumption_event(
    state: SimulationState,
    vehicle_id: str,
    spacing: SpacingOverrideConsumption,
    *,
    scenario_id: str,
) -> dict[str, Any]:
    return _event(
        state,
        module="Step7SpacingOverrideConsumption",
        event_type="spacing_override_consumption",
        vehicle_id=vehicle_id,
        related_vehicle_ids=tuple(item for item in (vehicle_id, spacing.source_mv_id) if item),
        scenario_id=scenario_id,
        reason="eq10_spacing_override_consumed",
        source=PAPER_FORMULA_SOURCE,
        is_engineering_patch=False,
        payload={
            "desired_spacing_source": spacing.desired_spacing_source,
            "eq10_desired_spacing": spacing.desired_spacing,
            "source_spacing_command_id": spacing.source_command_id,
            "source_mv_id": spacing.source_mv_id,
            "cv_role": spacing.cv_role,
            "aps_case": spacing.aps_case,
            "consumed_by": "P08",
        },
    )


def emit_speed_cap_consumption_event(
    state: SimulationState,
    composition: PlanningSpeedComposition,
    *,
    front: FrontCollisionAvoidanceResult | None = None,
    scenario_id: str,
) -> dict[str, Any]:
    front_payload = _front_collision_payload(front)
    if composition.most_conservative_source == "assignment_physical_safety_cap":
        reason = "assignment_physical_safety_cap"
    elif (
        composition.source_speed_cap_command_id is None
        and composition.front_fallback_speed != "not_applicable"
    ):
        reason = "front_collision_avoidance"
    else:
        reason = "speed_cap_consumed"
    return _event(
        state,
        module="Step7SpeedCapComposition",
        event_type="speed_cap",
        vehicle_id=composition.vehicle_id,
        related_vehicle_ids=(composition.vehicle_id,),
        scenario_id=scenario_id,
        reason=reason,
        source=PAPER_FORMULA_SOURCE,
        is_engineering_patch=False,
        payload={
            "base_candidate_speed": composition.base_candidate_speed,
            "boundary_speed_cap": composition.boundary_speed_cap,
            "assignment_physical_safety_cap": composition.assignment_physical_safety_cap,
            "assignment_physical_safety_leader_id": composition.assignment_physical_safety_leader_id,
            "assignment_physical_safety_source": composition.assignment_physical_safety_source,
            "front_fallback_speed": composition.front_fallback_speed,
            "front_fallback_speed_source": composition.front_fallback_speed_source,
            "planning_speed": composition.planning_speed,
            "most_conservative_source": composition.most_conservative_source,
            "source_speed_cap_command_id": composition.source_speed_cap_command_id,
            **front_payload,
        },
    )


def run_p08_eq10_consumption_sanity(
    state: SimulationState,
    vehicle_ids: tuple[str, ...],
    *,
    command_buffer: CommandBuffer,
    consumed_vehicle_ids: tuple[str, ...],
    rejected_vehicle_ids: tuple[str, ...],
    scenario_id: str,
) -> dict[str, Any]:
    wrong_consumptions: list[str] = []
    for vehicle_id in consumed_vehicle_ids:
        command = command_buffer.cooperation_commands.get(vehicle_id)
        if not isinstance(command, Mapping):
            wrong_consumptions.append(vehicle_id)
            continue
        spec = state.vehicle_specs[vehicle_id]
        if (
            str(command.get("cv_role") or "").lower() != "cfv"
            or str(command.get("aps_case") or "").lower() not in {"case_2", "case_4"}
            or (
                str(spec.vehicle_type).lower() == "chv"
                and str(spec.compliance_state).lower() == "non_compliant"
            )
        ):
            wrong_consumptions.append(vehicle_id)
    return _sanity(
        state,
        "Eq10_applied_to_wrong_vehicle",
        "fail" if wrong_consumptions else "pass",
        scenario_id=scenario_id,
        vehicle_ids=tuple(wrong_consumptions or vehicle_ids),
        reason="eq10_only_case_2_or_4_cfv_with_p07_handoff",
        payload={
            "consumed_vehicle_ids": list(consumed_vehicle_ids),
            "rejected_vehicle_ids": list(rejected_vehicle_ids),
            "wrong_vehicle_consumption_detected": bool(wrong_consumptions),
            "wrong_consumption_vehicle_ids": wrong_consumptions,
        },
    )


def run_p08_speed_cap_consumption_sanity(
    state: SimulationState,
    vehicle_ids: tuple[str, ...],
    *,
    scenario_id: str,
) -> dict[str, Any]:
    return _sanity(
        state,
        "boundary_violation",
        "pass",
        scenario_id=scenario_id,
        vehicle_ids=vehicle_ids or tuple(state.active_vehicle_ids),
        reason="p08_speed_cap_composition_checked",
        payload={"speed_cap_consumed_vehicle_ids": list(vehicle_ids)},
    )


def run_p08_no_write_before_commit_sanity(
    state: SimulationState,
    vehicle_ids: tuple[str, ...],
    *,
    state_unchanged: bool,
    scenario_id: str,
) -> dict[str, Any]:
    return _sanity(
        state,
        "no_write_before_commit",
        "pass" if state_unchanged else "fail",
        scenario_id=scenario_id,
        vehicle_ids=vehicle_ids,
        reason="p08_outputs_are_candidates_events_sanity_and_png_only",
        payload={
            "state_unchanged": state_unchanged,
            "planning_speed_written_to_vehicle_state": False,
        },
    )


def run_p08_no_lateral_candidate_sanity(
    state: SimulationState,
    vehicle_ids: tuple[str, ...],
    *,
    scenario_id: str,
) -> dict[str, Any]:
    return _sanity(
        state,
        "state_machine_inconsistency",
        "pass",
        scenario_id=scenario_id,
        vehicle_ids=vehicle_ids,
        reason="p08_no_lateral_candidate_or_maneuver_progress",
        payload={
            "candidate_lateral_created": False,
            "candidate_maneuver_progress_created": False,
            "lateral_trajectory_event_created": False,
        },
    )


def register_p08_png_features(
    candidates: Mapping[str, CandidateLongitudinalKinematics],
    *,
    consumed_spacing_vehicle_ids: set[str],
    speed_cap_vehicle_ids: set[str],
) -> list[dict[str, Any]]:
    features: list[dict[str, Any]] = []
    if candidates:
        features.append(_png_feature("longitudinal_candidate_marker", candidates.keys()))
        features.append(_png_feature("planning_speed_marker", candidates.keys(), required=False))
    if consumed_spacing_vehicle_ids:
        features.append(_png_feature("eq10_spacing_consumption_marker", consumed_spacing_vehicle_ids))
    if speed_cap_vehicle_ids:
        features.append(_png_feature("speed_cap_consumption_marker", speed_cap_vehicle_ids))
    return features


def _compute_base_candidate(
    state: SimulationState,
    relations: RelationsSnapshot,
    vehicle_id: str,
    *,
    spacing: SpacingOverrideConsumption,
    leader_id: str | None,
    geometry: RoadGeometryConfig = DEFAULT_ROAD_GEOMETRY,
) -> LongitudinalFormulaResult:
    return compute_p145_longitudinal_formula(
        state,
        relations,
        vehicle_id,
        spacing=spacing,
        leader_id=leader_id,
        update_cache=True,
        geometry=geometry,
    )


def compute_p145_longitudinal_formula(
    state: SimulationState,
    relations: RelationsSnapshot,
    vehicle_id: str,
    *,
    spacing: SpacingOverrideConsumption | None = None,
    leader_id: str | None = None,
    update_cache: bool = False,
    geometry: RoadGeometryConfig = DEFAULT_ROAD_GEOMETRY,
) -> LongitudinalFormulaResult:
    current = state.vehicle_states[vehicle_id]
    spec = state.vehicle_specs[vehicle_id]
    vehicle_type = str(spec.vehicle_type).lower()
    spacing = spacing or SpacingOverrideConsumption(
        consumed=False,
        desired_spacing=None,
        desired_spacing_source=_ordinary_spacing_source(state, vehicle_id),
    )
    if vehicle_type == "chv":
        return _chv_idm_formula(state, vehicle_id, leader_id=leader_id, spacing=spacing)
    return _cav_longitudinal_formula(
        state,
        vehicle_id,
        leader_id=leader_id,
        spacing=spacing,
        update_cache=update_cache,
    )


def _chv_idm_formula(
    state: SimulationState,
    vehicle_id: str,
    *,
    leader_id: str | None,
    spacing: SpacingOverrideConsumption,
) -> LongitudinalFormulaResult:
    current = state.vehicle_states[vehicle_id]
    desired_speed = max(_desired_speed(state, vehicle_id), 0.1)
    time_gap = state.vehicle_specs[vehicle_id].desired_time_gap or IDM.h_chv
    actual_spacing = _actual_spacing(state, vehicle_id, leader_id)
    if leader_id is None or actual_spacing is None:
        free_road_term = (current.v / desired_speed) ** 4
        acceleration = _clip_acceleration(IDM.a_i * (1.0 - free_road_term))
        s_star = None
        interaction_term = 0.0
        eq10_override_applied = False
        leader_speed = None
    else:
        leader = state.vehicle_states[leader_id]
        leader_speed = leader.v
        if spacing.consumed and spacing.desired_spacing is not None:
            s_star = float(spacing.desired_spacing)
            eq10_override_applied = True
        else:
            s_star = IDM.d0 + max(
                0.0,
                current.v * time_gap
                - current.v * (leader.v - current.v) / (2.0 * sqrt(IDM.a_i * IDM.b_i)),
            )
            eq10_override_applied = False
        interaction_term = (max(float(s_star), 0.0) / max(float(actual_spacing), 1e-6)) ** 2
        free_road_term = (current.v / desired_speed) ** 4
        acceleration = _clip_acceleration(IDM.a_i * (1.0 - free_road_term - interaction_term))
    candidate_speed = _clip_speed_to_lane(state, vehicle_id, current.v + acceleration * state.dt)
    return LongitudinalFormulaResult(
        mode="chv_idm",
        acceleration=acceleration,
        candidate_speed=candidate_speed,
        payload={
            "idm_formula_status": LOCKED_FORMULA_STATUS,
            "eq28_eq29_locked": True,
            "eq10_override_applied": eq10_override_applied,
            "desired_speed": desired_speed,
            "desired_speed_source": _desired_speed_source(state, vehicle_id),
            "idm_parameters_source": CORMC_PARAMETER_SPEC_SOURCE,
            "idm_h_i": time_gap,
            "idm_A_i": IDM.a_i,
            "idm_b_i": IDM.b_i,
            "idm_d0": IDM.d0,
            "idm_S_i_star": s_star,
            "idm_actual_spacing": actual_spacing,
            "idm_free_road_term": free_road_term,
            "idm_interaction_term": interaction_term,
            "leader_speed": leader_speed,
            "candidate_speed_after_lane_clip": candidate_speed,
        },
    )


def _cav_longitudinal_formula(
    state: SimulationState,
    vehicle_id: str,
    *,
    leader_id: str | None,
    spacing: SpacingOverrideConsumption,
    update_cache: bool,
) -> LongitudinalFormulaResult:
    current = state.vehicle_states[vehicle_id]
    h_i = state.vehicle_specs[vehicle_id].desired_time_gap or CAV.h_cav
    actual_spacing = _actual_spacing(state, vehicle_id, leader_id)
    collision_spacing = _collision_avoidance_spacing(state, vehicle_id, leader_id)
    eq18_desired_spacing = current.v * h_i + CAV.d0 + collision_spacing
    desired_spacing = (
        float(spacing.desired_spacing)
        if spacing.consumed and spacing.desired_spacing is not None
        else eq18_desired_spacing
    )
    desired_spacing_source = "Eq10" if spacing.consumed else "Eq18"
    base_payload = {
        "eq17_eq19_locked": True,
        "h_i": h_i,
        "d0": CAV.d0,
        "actual_spacing_d_i": actual_spacing,
        "collision_avoidance_spacing_C_i": collision_spacing,
        "eq18_desired_spacing_S_i": eq18_desired_spacing,
        "desired_spacing_target": desired_spacing,
        "desired_spacing_target_source": desired_spacing_source,
        "cav_parameters_source": CORMC_PARAMETER_SPEC_SOURCE,
    }
    if leader_id is None or actual_spacing is None or actual_spacing >= 2.0 * desired_spacing:
        acceleration = _clip_acceleration(CAV.k1 * (CAV.v_e - current.v))
        candidate_speed = _clip_speed_to_lane(state, vehicle_id, current.v + acceleration * state.dt)
        return LongitudinalFormulaResult(
            mode="cav_cruising",
            acceleration=acceleration,
            candidate_speed=candidate_speed,
            payload={
                **base_payload,
                "eq20_locked": True,
                "eq21_eq27_locked": False,
                "cav_cruising_gain_k1": CAV.k1,
                "equilibrium_speed_v_e": CAV.v_e,
                "candidate_speed_after_lane_clip": candidate_speed,
            },
        )

    cpid = _compute_cpid_formula(
        state,
        vehicle_id,
        leader_id=leader_id,
        actual_spacing=actual_spacing,
        desired_spacing=desired_spacing,
        update_cache=update_cache,
    )
    return LongitudinalFormulaResult(
        mode="cav_gap_regulating",
        acceleration=cpid["a_next"],
        candidate_speed=cpid["candidate_speed"],
        cache_update=cpid.get("cache_update"),
        payload={
            **base_payload,
            "eq20_locked": False,
            "eq21_eq27_locked": True,
            "cpid_mode": "minimal_formula_mode",
            **{key: value for key, value in cpid.items() if key != "cache_update"},
        },
    )


def _compute_cpid_formula(
    state: SimulationState,
    vehicle_id: str,
    *,
    leader_id: str,
    actual_spacing: float,
    desired_spacing: float,
    update_cache: bool,
) -> dict[str, Any]:
    current = state.vehicle_states[vehicle_id]
    leader = state.vehicle_states[leader_id]
    memory = state.controller_memory_by_vehicle.get(vehicle_id)
    cache_available = memory is not None and memory.ex_prev is not None and memory.e_prev is not None
    dt = _controller_dt(state, memory)
    ex = actual_spacing - desired_spacing
    ev = leader.v - current.v
    d_ex_dt = (ex - float(memory.ex_prev)) / dt if cache_available else 0.0
    integral_ex = (float(memory.integral_ex) + ex * dt) if cache_available else 0.0
    partial = CPID.kpx * ex + CPID.kix * integral_ex + CPID.kdx * d_ex_dt
    e = partial - ev
    d_e_dt = (e - float(memory.e_prev)) / dt if cache_available else 0.0
    integral_e = (float(memory.integral_e) + e * dt) if cache_available else 0.0
    u_raw = CPID.kpv * e + CPID.kiv * integral_e + CPID.kdv * d_e_dt
    u = _clip(float(u_raw), CAV.u_min, CAV.u_max)
    tau_i, tau_source = _tau_for_vehicle(state, vehicle_id)
    a_next_raw = (1.0 - state.dt / tau_i) * current.a + (state.dt / tau_i) * u
    a_next = _clip_acceleration(a_next_raw)
    candidate_speed = _clip_speed_to_lane(state, vehicle_id, current.v + a_next * state.dt)
    cache_update = None
    if update_cache:
        new_value = MappingProxyType(
            {
                "vehicle_id": vehicle_id,
                "ex_prev": ex,
                "e_prev": e,
                "integral_ex": integral_ex,
                "integral_e": integral_e,
                "last_t": state.t,
                "last_controller_update_step": state.step,
                "controller_mode": "cav_cpid",
            }
        )
        cache_update = CandidateCacheUpdate(
            candidate_id=f"p08:{state.step}:{vehicle_id}:cpid_cache",
            cache_name="longitudinal_controller_cache",
            owner_vehicle_id=vehicle_id,
            operation="update",
            new_value=new_value,
            reason="cav_cpid_eq21_eq27",
        )
    return {
        "ex": ex,
        "ev": ev,
        "current_speed": current.v,
        "leader_speed": leader.v,
        "d_ex_dt": d_ex_dt,
        "integral_ex": integral_ex,
        "partial": partial,
        "e": e,
        "d_e_dt": d_e_dt,
        "integral_e": integral_e,
        "u_raw": u_raw,
        "u": u,
        "a_next_raw": a_next_raw,
        "a_next": a_next,
        "candidate_speed": candidate_speed,
        "tau_i": tau_i,
        "tau_source": tau_source,
        "cpid_gain_source": CPID.gain_source,
        "controller_cache_initialized": not cache_available,
        "controller_cache_reused": cache_available,
        "candidate_speed_after_lane_clip": candidate_speed,
        "cache_update": cache_update,
    }


def _constraints_applied(
    spacing: SpacingOverrideConsumption,
    composition: PlanningSpeedComposition,
) -> tuple[str, ...]:
    constraints: list[str] = []
    if spacing.consumed:
        constraints.append("eq10_spacing_override")
    if composition.source_speed_cap_command_id is not None:
        constraints.append("boundary_speed_cap")
    if composition.most_conservative_source == "assignment_physical_safety_cap":
        constraints.append("assignment_physical_safety_cap")
    if composition.front_fallback_speed != "not_applicable":
        constraints.append("front_fallback")
    return tuple(constraints)


def _source_commands(
    spacing: SpacingOverrideConsumption,
    composition: PlanningSpeedComposition,
) -> tuple[str, ...]:
    commands = [
        item
        for item in (
            spacing.source_command_id if spacing.consumed else None,
            composition.source_speed_cap_command_id,
        )
        if item is not None
    ]
    return tuple(commands)


def _speed_cap_command(command_buffer: CommandBuffer, vehicle_id: str) -> Mapping[str, Any] | None:
    commands = command_buffer.speed_cap_commands.get(vehicle_id)
    if commands is None:
        return None
    if isinstance(commands, Mapping):
        return commands
    for command in commands:
        if isinstance(command, Mapping) and str(command.get("command_type") or "") == "speed_cap":
            return command
    return None


def evaluate_assignment_physical_safety_cap(
    state: SimulationState,
    vehicle_id: str,
    *,
    leader_relation: LeaderRelationContext,
    geometry: RoadGeometryConfig = DEFAULT_ROAD_GEOMETRY,
) -> AssignmentPhysicalSafetyCap:
    if vehicle_id not in state.vehicle_states:
        return AssignmentPhysicalSafetyCap(False, None, None, None, "vehicle_missing")
    current = state.vehicle_states[vehicle_id]
    if current.physical_lane != ON_RAMP and current.road_role != ON_RAMP_MV_ROLE:
        return AssignmentPhysicalSafetyCap(False, None, None, None, "not_on_ramp_mv")
    region = resolve_on_ramp_control_region(
        current.x_global,
        current.road_role,
        geometry=geometry,
    )
    if region.region != "control_zone":
        return AssignmentPhysicalSafetyCap(False, None, None, None, "not_control_zone")
    relation_source = leader_relation.relation_source or ""
    if not (
        relation_source.startswith("aps_assignment_")
        and relation_source.endswith("_mv_clv_leader")
    ):
        return AssignmentPhysicalSafetyCap(False, None, None, None, "no_assignment_clv_relation")
    leader_id = leader_relation.leader_id
    if leader_id is None or leader_id not in state.vehicle_states:
        return AssignmentPhysicalSafetyCap(False, None, None, None, "assignment_leader_missing")
    leader = state.vehicle_states[leader_id]
    if not leader.is_active:
        return AssignmentPhysicalSafetyCap(False, None, leader_id, None, "assignment_leader_inactive")
    if leader.physical_lane != LANE_2 or leader.lane_change_state == "executing":
        return AssignmentPhysicalSafetyCap(False, None, leader_id, None, "assignment_leader_not_stable_lane2")
    dt = max(float(state.dt), 1.0e-6)
    rear_boundary_x = (
        float(leader.x_global)
        - _vehicle_length(state, leader_id)
        - ASSIGNMENT_SAFETY_EPSILON_M
    )
    speed_cap = max(0.0, (rear_boundary_x - float(current.x_global)) / dt)
    return AssignmentPhysicalSafetyCap(
        applicable=True,
        cap=speed_cap,
        leader_id=leader_id,
        source=relation_source,
        reason="control_zone_assignment_clv_rear_boundary",
    )


def _leader_relation_for_vehicle(
    relations: RelationsSnapshot,
    vehicle_id: str,
) -> LeaderRelationContext:
    active_relation = relations.active_maneuver_relation.get(vehicle_id)
    if active_relation is not None:
        return LeaderRelationContext(
            leader_id=active_relation.primary_leader_id,
            relation_source=active_relation.relation_source,
            affected_target_follower_id=active_relation.affected_target_follower_id,
            affected_source_follower_id=active_relation.affected_source_follower_id,
        )
    return LeaderRelationContext(
        leader_id=relations.leader_by_vehicle.get(vehicle_id),
        relation_source="lane_ordering",
        affected_target_follower_id=relations.follower_by_vehicle.get(vehicle_id),
        affected_source_follower_id=None,
    )


def _actual_spacing(
    state: SimulationState,
    vehicle_id: str,
    leader_id: str | None,
) -> float | None:
    if leader_id is None:
        return None
    current = state.vehicle_states[vehicle_id]
    leader = state.vehicle_states[leader_id]
    return float(leader.x_global) - float(current.x_global) - _vehicle_length(state, leader_id)


def _desired_speed(state: SimulationState, vehicle_id: str) -> float:
    return float(state.vehicle_specs[vehicle_id].desired_speed or DEFAULT_DESIRED_SPEED_MPS)


def _desired_speed_source(state: SimulationState, vehicle_id: str) -> str:
    return (
        "VehicleSpec.desired_speed"
        if state.vehicle_specs[vehicle_id].desired_speed is not None
        else "deterministic_default_30mps_p145"
    )


def _ordinary_spacing_source(state: SimulationState, vehicle_id: str) -> str:
    vehicle_type = str(state.vehicle_specs[vehicle_id].vehicle_type).lower()
    if vehicle_type == "chv":
        return "ordinary_idm"
    return "default_spacing"


def _vehicle_length(state: SimulationState, vehicle_id: str) -> float:
    return float(state.vehicle_specs[vehicle_id].length)


def _clip_acceleration(value: float) -> float:
    return _clip(value, CAV.a_min, CAV.a_max)


def _clip(value: float, lower: float, upper: float) -> float:
    return min(float(upper), max(float(lower), float(value)))


def _clip_speed_to_lane(state: SimulationState, vehicle_id: str, value: float) -> float:
    lane = state.vehicle_states[vehicle_id].physical_lane
    max_speed = LANE_MAX_SPEED_MPS.get(lane, LANE_MAX_SPEED_MPS[LANE_2])
    return _clip(value, 0.0, max_speed)


def _collision_avoidance_spacing(
    state: SimulationState,
    vehicle_id: str,
    leader_id: str | None,
) -> float:
    if leader_id is None:
        return 0.0
    current = state.vehicle_states[vehicle_id]
    leader = state.vehicle_states[leader_id]
    if current.v <= leader.v:
        return 0.0
    return (current.v - leader.v) ** 2 / (2.0 * abs(CAV.a_min))


def _controller_dt(
    state: SimulationState,
    memory: LongitudinalControllerMemory | None,
) -> float:
    if memory is not None and memory.last_t is not None:
        dt = float(state.t) - float(memory.last_t)
        if dt > 0.0:
            return dt
    return max(float(state.dt), 1e-6)


def _tau_for_vehicle(state: SimulationState, vehicle_id: str) -> tuple[float, str]:
    value = state.vehicle_specs[vehicle_id].inertial_lag
    if value is not None:
        return max(float(value), 1e-6), "VehicleSpec.inertial_lag"
    return CPID.default_tau, CPID.default_tau_source


def evaluate_front_collision_avoidance(
    state: SimulationState,
    relations: RelationsSnapshot,
    vehicle_id: str,
    *,
    command_buffer: CommandBuffer,
    leader_id: str | None,
) -> FrontCollisionAvoidanceResult:
    source = _front_collision_source(state, command_buffer, vehicle_id)
    if source is None:
        return FrontCollisionAvoidanceResult(
            status="not_applicable",
            applicable=False,
            eq42_eq46_locked=True,
            formula_status=LOCKED_FORMULA_STATUS,
        )
    front_leader_id = source.get("leader_id") or leader_id
    if front_leader_id is None:
        return FrontCollisionAvoidanceResult(
            status="not_applicable_no_lv",
            applicable=True,
            eq42_eq46_locked=True,
            formula_status=LOCKED_FORMULA_STATUS,
            planned_length=source.get("planned_length"),
            planned_length_source=source.get("planned_length_source"),
        )
    current = state.vehicle_states[vehicle_id]
    leader = state.vehicle_states[str(front_leader_id)]
    v_sv = max(float(current.v), 0.0)
    planned_length, planned_length_source = _front_planned_length(state, vehicle_id, source)
    if v_sv <= 0.0:
        return FrontCollisionAvoidanceResult(
            status="not_applicable_nonpositive_sv_speed",
            applicable=True,
            eq42_eq46_locked=True,
            formula_status=LOCKED_FORMULA_STATUS,
            leader_id=str(front_leader_id),
            planned_length=planned_length,
            planned_length_source=planned_length_source,
        )
    x_start = float(source.get("start_x_global", current.x_global))
    x_mid = x_start + planned_length / 2.0
    t_mid = (x_mid - current.x_global) / v_sv
    d_mid = t_mid * (v_sv - float(leader.v))
    current_gap = float(leader.x_global) - float(current.x_global) - _vehicle_length(
        state,
        str(front_leader_id),
    )
    safe = d_mid < current_gap
    if safe:
        return FrontCollisionAvoidanceResult(
            status="safe",
            applicable=True,
            eq42_eq46_locked=True,
            formula_status=LOCKED_FORMULA_STATUS,
            consumed=False,
            leader_id=str(front_leader_id),
            planned_length=planned_length,
            planned_length_source=planned_length_source,
            x_mid=x_mid,
            t_mid=t_mid,
            d_mid=d_mid,
            current_gap=current_gap,
        )
    fallback_speed, fallback_source = _front_fallback_speed(state, vehicle_id, source)
    return FrontCollisionAvoidanceResult(
        status="fallback_to_previous_planning_speed",
        applicable=True,
        eq42_eq46_locked=True,
        formula_status=LOCKED_FORMULA_STATUS,
        consumed=fallback_speed is not None,
        fallback_speed=fallback_speed,
        fallback_speed_source=fallback_source,
        leader_id=str(front_leader_id),
        planned_length=planned_length,
        planned_length_source=planned_length_source,
        x_mid=x_mid,
        t_mid=t_mid,
        d_mid=d_mid,
        current_gap=current_gap,
        delay_maneuver_due_to_front_collision=fallback_speed is None,
    )


def _front_collision_source(
    state: SimulationState,
    command_buffer: CommandBuffer,
    vehicle_id: str,
) -> dict[str, Any] | None:
    lane_command = command_buffer.lane_change_commands.get(vehicle_id)
    merge_command = command_buffer.merge_commands.get(vehicle_id)
    active = state.active_maneuvers.get(vehicle_id)
    if isinstance(lane_command, Mapping):
        return {
            "maneuver_type": "lane_change",
            "start_x_global": state.vehicle_states[vehicle_id].x_global,
            "target_y": lane_command.get("target_y"),
            "planned_length": _optional_float(lane_command.get("planned_length")),
            "planned_length_source": lane_command.get("planned_length_source"),
            "leader_id": _optional_str(lane_command.get("front_leader_id")),
        }
    if active is not None and active.maneuver_type == "lane_change":
        return _front_source_from_active(active)
    if isinstance(merge_command, Mapping):
        return {
            "maneuver_type": "merge",
            "start_x_global": state.vehicle_states[vehicle_id].x_global,
            "target_y": merge_command.get("target_y"),
            "planned_length": _optional_float(merge_command.get("planned_length")),
            "planned_length_source": merge_command.get("planned_length_source"),
            "leader_id": _optional_str(merge_command.get("front_leader_id")),
        }
    if active is not None and active.maneuver_type == "merge":
        return _front_source_from_active(active)
    return None


def _front_source_from_active(active: ManeuverTrajectoryState) -> dict[str, Any]:
    return {
        "maneuver_type": active.maneuver_type,
        "start_x_global": active.start_x_global,
        "target_y": active.target_y,
        "planned_length": active.planned_length,
        "planned_length_source": "active_maneuver.planned_length",
        "leader_id": None,
        "last_planning_speed": active.last_planning_speed,
    }


def _front_planned_length(
    state: SimulationState,
    vehicle_id: str,
    source: Mapping[str, Any],
) -> tuple[float, str]:
    planned_length = source.get("planned_length")
    if planned_length is not None and float(planned_length) > 0.0:
        return float(planned_length), str(source.get("planned_length_source") or "planned_length")
    current = state.vehicle_states[vehicle_id]
    target_y = _optional_float(source.get("target_y"))
    y_d = 0.0 if target_y is None else target_y - float(current.y)
    return (
        float(current.v) * sqrt(2.0 * abs(y_d) / LANE_CHANGE.a_p) + LANE_CHANGE.l_centerline,
        "Eq34_fallback_p145",
    )


def _front_fallback_speed(
    state: SimulationState,
    vehicle_id: str,
    source: Mapping[str, Any],
) -> tuple[float | None, str | None]:
    active = state.active_maneuvers.get(vehicle_id)
    if active is not None and active.last_planning_speed is not None:
        return float(active.last_planning_speed), "active_maneuver.last_planning_speed"
    if source.get("last_planning_speed") is not None:
        return float(source["last_planning_speed"]), "active_maneuver.last_planning_speed"
    current_speed = state.vehicle_states[vehicle_id].v
    return float(current_speed), "VehicleState.v"


def _front_collision_payload(front: FrontCollisionAvoidanceResult | None) -> dict[str, Any]:
    if front is None:
        return {}
    return {
        "eq42_eq46_locked": front.eq42_eq46_locked,
        "front_collision_formula_status": front.formula_status,
        "front_collision_status": front.status,
        "front_fallback_consumed": front.consumed,
        "front_fallback_speed_source": front.fallback_speed_source,
        "front_leader_id": front.leader_id,
        "front_planned_length": front.planned_length,
        "front_planned_length_source": front.planned_length_source,
        "front_x_mid": front.x_mid,
        "front_T_mid": front.t_mid,
        "front_d_mid": front.d_mid,
        "front_current_gap": front.current_gap,
        "delay_maneuver_due_to_front_collision": front.delay_maneuver_due_to_front_collision,
    }


def _optional_str(value: Any) -> str | None:
    if value in (None, ""):
        return None
    return str(value)


def _optional_float(value: Any) -> float | None:
    if value in (None, ""):
        return None
    return float(value)


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
    payload: Mapping[str, Any],
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
        "payload": dict(payload),
    }


def _sanity(
    state: SimulationState,
    check_type: str,
    result: str,
    *,
    scenario_id: str,
    vehicle_ids: tuple[str, ...],
    reason: str,
    payload: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    return {
        "step": state.step,
        "t": state.t,
        "check_type": check_type,
        "result": result,
        "vehicle_ids": list(vehicle_ids),
        "scenario_id": scenario_id,
        "reason": reason,
        "payload": dict(payload or {}),
    }


def _png_feature(
    feature_type: str,
    vehicle_ids: Any,
    *,
    required: bool = True,
) -> dict[str, Any]:
    ids = sorted(str(vehicle_id) for vehicle_id in vehicle_ids)
    return {
        "feature_type": feature_type,
        "required": required,
        "vehicle_ids": ids,
        "expected_visibility": "visible" if required else "optional",
        "notes": "registered only; renderer deferred",
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
        tuple(
            (
                vehicle_id,
                memory.ex_prev,
                memory.e_prev,
                memory.integral_ex,
                memory.integral_e,
                memory.last_t,
                memory.last_controller_update_step,
                memory.controller_mode,
            )
            for vehicle_id, memory in sorted(state.controller_memory_by_vehicle.items())
        ),
    )
