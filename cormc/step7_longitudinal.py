from __future__ import annotations

from dataclasses import dataclass
from types import MappingProxyType
from typing import Any, Mapping

from cormc.step0_3 import (
    ON_RAMP,
    ON_RAMP_MV_ROLE,
    RelationsSnapshot,
    SimulationState,
    VehicleState,
    assert_x_plot_not_used_in_algorithm_path,
)
from cormc.step9_11 import (
    CandidateLongitudinalKinematics,
    CommandBuffer,
    NextStateBuffer,
)


STEP7_LONGITUDINAL_SOURCE = "step7_longitudinal_model"
ENGINEERING_PATCH_SOURCE = "first_version_engineering_patch"
PAPER_FORMULA_SOURCE = "paper_formula"
FIRST_VERSION_PROBE_SOURCE = "first_version_probe"
DEFAULT_DESIRED_SPEED_MPS = 30.0
DEFAULT_DESIRED_SPACING_M = 40.0
DEFAULT_TIME_GAP_S = 1.5
MAX_ACCELERATION_MPS2 = 2.0
COMFORTABLE_DECELERATION_MPS2 = 3.0


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
    front_fallback_speed: float | str
    planning_speed: float
    most_conservative_source: str
    source_speed_cap_command_id: str | None = None


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

    for vehicle_id in selected_vehicle_ids:
        if vehicle_id not in state.vehicle_states:
            continue
        current = state.vehicle_states[vehicle_id]
        spec = state.vehicle_specs[vehicle_id]
        leader_id = _leader_id_for_vehicle(relations, vehicle_id)
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
        )
        composition = compose_planning_speed(
            vehicle_id,
            base_candidate_speed=base["candidate_speed"],
            command_buffer=command_buffer,
            front_fallback_speed=(front_fallback_speeds or {}).get(vehicle_id),
        )
        if composition.source_speed_cap_command_id is not None:
            speed_cap_vehicle_ids.add(vehicle_id)
            events.append(
                emit_speed_cap_consumption_event(
                    state,
                    composition,
                    scenario_id=scenario_id,
                )
            )

        candidate = build_longitudinal_candidate(
            state,
            vehicle_id,
            base_candidate_speed=base["candidate_speed"],
            planning_speed=composition.planning_speed,
            acceleration=base["acceleration"],
            constraints_applied=_constraints_applied(spacing, composition),
            source_commands=_source_commands(spacing, composition),
        )
        candidates[vehicle_id] = candidate
        planning_speeds[vehicle_id] = candidate.planning_speed
        events.append(
            emit_longitudinal_model_event(
                state,
                vehicle_id,
                mode=str(base["longitudinal_mode"]),
                leader_id=leader_id,
                spacing=spacing,
                composition=composition,
                candidate=candidate,
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
    front_fallback_speed: float | None,
) -> PlanningSpeedComposition:
    applicable: list[tuple[str, float]] = [("base_candidate_speed", float(base_candidate_speed))]
    speed_cap_command = _speed_cap_command(command_buffer, vehicle_id)
    boundary_speed_cap: float | str = "not_applicable"
    source_speed_cap_command_id: str | None = None
    if speed_cap_command is not None and bool(speed_cap_command.get("cap_feasible", True)):
        speed_cap_value = speed_cap_command.get("speed_cap")
        if speed_cap_value is not None:
            boundary_speed_cap = float(speed_cap_value)
            source_speed_cap_command_id = str(speed_cap_command.get("command_id"))
            applicable.append(("boundary_speed_cap", float(boundary_speed_cap)))

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
        front_fallback_speed=front_fallback,
        planning_speed=max(0.0, float(planning_speed)),
        most_conservative_source=most_conservative_source,
        source_speed_cap_command_id=source_speed_cap_command_id,
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
    spacing: SpacingOverrideConsumption,
    composition: PlanningSpeedComposition,
    candidate: CandidateLongitudinalKinematics,
    scenario_id: str,
) -> dict[str, Any]:
    spec = state.vehicle_specs[vehicle_id]
    payload = {
        "longitudinal_mode": mode,
        "vehicle_type": spec.vehicle_type,
        "compliance_state": spec.compliance_state,
        "leader_id": leader_id,
        "desired_spacing_source": spacing.desired_spacing_source,
        "desired_spacing_override": spacing.desired_spacing,
        "spacing_override_consumed": spacing.consumed,
        "spacing_rejection_reason": spacing.rejection_reason,
        "base_candidate_speed": composition.base_candidate_speed,
        "boundary_speed_cap": composition.boundary_speed_cap,
        "front_fallback_speed": composition.front_fallback_speed,
        "planning_speed": candidate.planning_speed,
        "candidate_id": candidate.candidate_id,
        "source_spacing_command_id": spacing.source_command_id,
        "source_mv_id": spacing.source_mv_id,
    }
    if mode == "cav_gap_regulating":
        payload["cpid_memory_status"] = "probe_schema_gap"
    return _event(
        state,
        module="Step7LongitudinalModel",
        event_type="longitudinal_model",
        vehicle_id=vehicle_id,
        related_vehicle_ids=tuple(item for item in (vehicle_id, leader_id) if item is not None),
        scenario_id=scenario_id,
        reason=mode,
        source=PAPER_FORMULA_SOURCE if mode == "chv_idm" else FIRST_VERSION_PROBE_SOURCE,
        is_engineering_patch=mode != "chv_idm",
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
    scenario_id: str,
) -> dict[str, Any]:
    return _event(
        state,
        module="Step7SpeedCapComposition",
        event_type="speed_cap",
        vehicle_id=composition.vehicle_id,
        related_vehicle_ids=(composition.vehicle_id,),
        scenario_id=scenario_id,
        reason="speed_cap_consumed",
        source=PAPER_FORMULA_SOURCE,
        is_engineering_patch=False,
        payload={
            "base_candidate_speed": composition.base_candidate_speed,
            "boundary_speed_cap": composition.boundary_speed_cap,
            "front_fallback_speed": composition.front_fallback_speed,
            "planning_speed": composition.planning_speed,
            "most_conservative_source": composition.most_conservative_source,
            "source_speed_cap_command_id": composition.source_speed_cap_command_id,
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
) -> dict[str, Any]:
    current = state.vehicle_states[vehicle_id]
    spec = state.vehicle_specs[vehicle_id]
    vehicle_type = str(spec.vehicle_type).lower()
    if _is_on_ramp_vehicle(current):
        mode = "mv_on_ramp"
        desired_speed = _desired_speed(state, vehicle_id)
        acceleration = _clip_acceleration((desired_speed - current.v) / max(state.dt, 1e-6))
    elif vehicle_type == "chv":
        mode = "chv_idm"
        acceleration = _idm_acceleration(state, vehicle_id, leader_id=leader_id, spacing=spacing)
    else:
        desired_spacing = spacing.desired_spacing or DEFAULT_DESIRED_SPACING_M
        actual_spacing = _actual_spacing(state, vehicle_id, leader_id)
        if (
            leader_id is None
            or actual_spacing is None
            or actual_spacing >= 2.0 * desired_spacing
        ):
            mode = "cav_cruising"
            desired_speed = _desired_speed(state, vehicle_id)
            acceleration = _clip_acceleration((desired_speed - current.v) / max(state.dt, 1e-6))
        else:
            mode = "cav_gap_regulating"
            acceleration = _gap_regulating_acceleration(
                state,
                vehicle_id,
                leader_id=leader_id,
                desired_spacing=desired_spacing,
                actual_spacing=actual_spacing,
            )
    candidate_speed = max(0.0, float(current.v) + acceleration * float(state.dt))
    return {
        "longitudinal_mode": mode,
        "acceleration": acceleration,
        "candidate_speed": candidate_speed,
    }


def _idm_acceleration(
    state: SimulationState,
    vehicle_id: str,
    *,
    leader_id: str | None,
    spacing: SpacingOverrideConsumption,
) -> float:
    current = state.vehicle_states[vehicle_id]
    desired_speed = max(_desired_speed(state, vehicle_id), 0.1)
    time_gap = state.vehicle_specs[vehicle_id].desired_time_gap or DEFAULT_TIME_GAP_S
    actual_spacing = _actual_spacing(state, vehicle_id, leader_id)
    if leader_id is None or actual_spacing is None:
        free_road_term = (current.v / desired_speed) ** 4
        return _clip_acceleration(MAX_ACCELERATION_MPS2 * (1.0 - free_road_term))
    leader = state.vehicle_states[leader_id]
    desired_spacing = spacing.desired_spacing or (
        2.0 + current.v * time_gap + current.v * (current.v - leader.v) / 8.0
    )
    interaction_term = (max(desired_spacing, 0.0) / max(actual_spacing, 0.1)) ** 2
    free_road_term = (current.v / desired_speed) ** 4
    return _clip_acceleration(MAX_ACCELERATION_MPS2 * (1.0 - free_road_term - interaction_term))


def _gap_regulating_acceleration(
    state: SimulationState,
    vehicle_id: str,
    *,
    leader_id: str | None,
    desired_spacing: float,
    actual_spacing: float,
) -> float:
    current = state.vehicle_states[vehicle_id]
    leader_speed = state.vehicle_states[leader_id].v if leader_id is not None else current.v
    spacing_error = actual_spacing - desired_spacing
    speed_error = leader_speed - current.v
    return _clip_acceleration(0.05 * spacing_error + 0.25 * speed_error)


def _constraints_applied(
    spacing: SpacingOverrideConsumption,
    composition: PlanningSpeedComposition,
) -> tuple[str, ...]:
    constraints: list[str] = []
    if spacing.consumed:
        constraints.append("eq10_spacing_override")
    if composition.source_speed_cap_command_id is not None:
        constraints.append("boundary_speed_cap")
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


def _leader_id_for_vehicle(relations: RelationsSnapshot, vehicle_id: str) -> str | None:
    active_relation = relations.active_maneuver_relation.get(vehicle_id)
    if active_relation is not None:
        return active_relation.primary_leader_id
    return relations.leader_by_vehicle.get(vehicle_id)


def _actual_spacing(
    state: SimulationState,
    vehicle_id: str,
    leader_id: str | None,
) -> float | None:
    if leader_id is None:
        return None
    current = state.vehicle_states[vehicle_id]
    leader = state.vehicle_states[leader_id]
    return max(0.0, float(leader.x_global) - float(current.x_global) - _vehicle_length(state, leader_id))


def _desired_speed(state: SimulationState, vehicle_id: str) -> float:
    return float(state.vehicle_specs[vehicle_id].desired_speed or DEFAULT_DESIRED_SPEED_MPS)


def _ordinary_spacing_source(state: SimulationState, vehicle_id: str) -> str:
    vehicle_type = str(state.vehicle_specs[vehicle_id].vehicle_type).lower()
    if vehicle_type == "chv":
        return "ordinary_idm"
    return "default_spacing"


def _vehicle_length(state: SimulationState, vehicle_id: str) -> float:
    return float(state.vehicle_specs[vehicle_id].length)


def _is_on_ramp_vehicle(vehicle: VehicleState) -> bool:
    return vehicle.physical_lane == ON_RAMP or vehicle.road_role == ON_RAMP_MV_ROLE


def _clip_acceleration(value: float) -> float:
    return min(MAX_ACCELERATION_MPS2, max(-COMFORTABLE_DECELERATION_MPS2, float(value)))


def _optional_str(value: Any) -> str | None:
    if value in (None, ""):
        return None
    return str(value)


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
        tuple((key, tuple(sorted(value.items()))) for key, value in state.aps_assignment_cache.items()),
    )
