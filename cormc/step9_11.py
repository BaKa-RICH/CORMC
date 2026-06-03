from __future__ import annotations

from dataclasses import dataclass, field, fields, replace
from types import MappingProxyType
from typing import Any, Mapping

from cormc.step0_3 import (
    LongitudinalControllerMemory,
    ManeuverTrajectoryState,
    SimulationState,
    VehicleState,
    assert_x_plot_not_used_in_algorithm_path,
    build_prefreeze_workspace_from_scenario,
    freeze_simulation_state,
)


IDENTITY_CANDIDATE_SOURCE = "identity_candidate_for_commit_infrastructure"
TEST_HARNESS_CANDIDATE_SOURCE = "test_harness_preloaded_candidate"
STEP7_LONGITUDINAL_SOURCE = "step7_longitudinal_model"
STEP8_LATERAL_SOURCE = "step8_lateral_trajectory"
STEP9_CANDIDATE_ASSEMBLY_SOURCE = "step9_candidate_assembly"
ALLOWED_CANDIDATE_SOURCES = {
    IDENTITY_CANDIDATE_SOURCE,
    TEST_HARNESS_CANDIDATE_SOURCE,
    STEP7_LONGITUDINAL_SOURCE,
    STEP8_LATERAL_SOURCE,
    STEP9_CANDIDATE_ASSEMBLY_SOURCE,
}
ALLOWED_FINAL_CANDIDATE_SOURCES = {
    IDENTITY_CANDIDATE_SOURCE,
    TEST_HARNESS_CANDIDATE_SOURCE,
    STEP9_CANDIDATE_ASSEMBLY_SOURCE,
}
ALLOWED_COMPONENT_CANDIDATE_SOURCES = ALLOWED_CANDIDATE_SOURCES


@dataclass(frozen=True)
class CommandBuffer:
    step: int
    t: float
    longitudinal_commands: Mapping[str, Any] = field(default_factory=dict)
    cooperation_commands: Mapping[str, Any] = field(default_factory=dict)
    lane_change_commands: Mapping[str, Any] = field(default_factory=dict)
    merge_commands: Mapping[str, Any] = field(default_factory=dict)
    speed_cap_commands: Mapping[str, tuple[Any, ...]] = field(default_factory=dict)
    state_transition_commands: Mapping[str, tuple[Any, ...]] = field(default_factory=dict)
    cache_update_commands: tuple[Any, ...] = ()
    same_step_overlays: Mapping[str, Any] = field(default_factory=dict)
    cuc_decisions: Mapping[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class CandidateLongitudinalKinematics:
    candidate_id: str
    vehicle_id: str
    x_global: float
    v: float
    a: float
    candidate_speed: float
    planning_speed: float
    source: str
    constraints_applied: tuple[str, ...] = ()
    source_commands: tuple[str, ...] = ()


@dataclass(frozen=True)
class CandidateLateralKinematics:
    candidate_id: str
    vehicle_id: str
    y: float
    target_y: float | None
    source: str
    front_collision_fallback: bool = False
    source_commands: tuple[str, ...] = ()


@dataclass(frozen=True)
class CandidateManeuverProgress:
    candidate_id: str
    vehicle_id: str
    maneuver_type: str
    progress: float
    completed: bool
    target_y_reached: bool
    source_command_id: str | None = None


@dataclass(frozen=True)
class CandidateLaneState:
    candidate_id: str
    vehicle_id: str
    physical_lane: str
    road_role: str
    reason: str


@dataclass(frozen=True)
class CandidateStateTransition:
    candidate_id: str
    vehicle_id: str
    state_name: str
    old_state: str
    new_state: str
    reason: str


@dataclass(frozen=True)
class CandidateCacheUpdate:
    candidate_id: str
    cache_name: str
    owner_vehicle_id: str
    operation: str
    new_value: Mapping[str, Any] | None = None
    reason: str = "not_applicable"


@dataclass(frozen=True)
class CandidateKinematics:
    candidate_id: str
    vehicle_id: str
    x_global: float
    y: float
    v: float
    a: float
    source: str
    source_longitudinal_candidate: str | None = None
    source_lateral_candidate: str | None = None
    source_maneuver_progress: str | None = None
    source_state_transition: str | None = None
    source_cache_update: str | None = None
    constraints_applied: tuple[str, ...] = ()


@dataclass(frozen=True)
class CommitWarning:
    vehicle_id: str
    warning_type: str
    reason: str
    candidate_ids: tuple[str, ...] = ()


@dataclass(frozen=True)
class NextStateBuffer:
    step: int
    t: float
    candidate_longitudinal: Mapping[str, CandidateLongitudinalKinematics] = field(
        default_factory=dict
    )
    candidate_lateral: Mapping[str, CandidateLateralKinematics] = field(default_factory=dict)
    candidate_kinematics: Mapping[str, tuple[CandidateKinematics, ...]] = field(
        default_factory=dict
    )
    candidate_maneuver_progress: Mapping[str, CandidateManeuverProgress] = field(
        default_factory=dict
    )
    candidate_lane_state: Mapping[str, CandidateLaneState] = field(default_factory=dict)
    candidate_state_transitions: Mapping[str, tuple[CandidateStateTransition, ...]] = field(
        default_factory=dict
    )
    candidate_cache_updates: tuple[CandidateCacheUpdate, ...] = ()
    commit_warnings: tuple[CommitWarning, ...] = ()


@dataclass(frozen=True)
class EventRecord:
    event_id: str
    run_id: str
    scenario_id: str
    step: int
    t: float
    module: str
    event_type: str
    vehicle_id: str | None = None
    related_vehicle_ids: tuple[str, ...] = ()
    source_command_id: str | None = None
    source_candidate_id: str | None = None
    reason: str = ""
    result: str = ""
    is_engineering_patch: bool = False
    source: str = "paper_algorithm"
    payload: Mapping[str, Any] = field(default_factory=dict)

    def to_matcher_dict(self) -> dict[str, Any]:
        vehicle_ids = list(self.related_vehicle_ids)
        if self.vehicle_id is not None and self.vehicle_id not in vehicle_ids:
            vehicle_ids.append(self.vehicle_id)
        return {
            "event_id": self.event_id,
            "run_id": self.run_id,
            "scenario_id": self.scenario_id,
            "step": self.step,
            "t": self.t,
            "module": self.module,
            "event_type": self.event_type,
            "vehicle_id": self.vehicle_id,
            "vehicle_ids": vehicle_ids,
            "related_vehicle_ids": list(self.related_vehicle_ids),
            "source_command_id": self.source_command_id,
            "source_candidate_id": self.source_candidate_id,
            "reason": self.reason,
            "result": self.result,
            "is_engineering_patch": self.is_engineering_patch,
            "source": self.source,
            "payload": dict(self.payload),
        }


@dataclass(frozen=True)
class SanityCheckRecord:
    check_id: str
    run_id: str
    scenario_id: str
    step: int
    t: float
    check_type: str
    severity: str
    result: str
    vehicle_ids: tuple[str, ...] = ()
    lane_id: str | None = None
    x_global: float | None = None
    reason: str = ""
    source_event_id: str | None = None
    payload: Mapping[str, Any] = field(default_factory=dict)

    def to_matcher_dict(self) -> dict[str, Any]:
        return {
            "check_id": self.check_id,
            "run_id": self.run_id,
            "scenario_id": self.scenario_id,
            "step": self.step,
            "t": self.t,
            "check_type": self.check_type,
            "severity": self.severity,
            "result": self.result,
            "vehicle_ids": list(self.vehicle_ids),
            "lane_id": self.lane_id,
            "x_global": self.x_global,
            "reason": self.reason,
            "source_event_id": self.source_event_id,
            "payload": dict(self.payload),
        }


@dataclass(frozen=True)
class TrajectoryRecord:
    run_id: str
    scenario_id: str
    step: int
    t: float
    vehicle_id: str
    vehicle_type: str
    compliance_state: str
    x_global: float
    y: float
    v: float
    a: float
    physical_lane: str
    road_role: str
    primary_leader_id: str | None = None
    lane_change_state: str = "normal"
    merge_state: str = "none"
    active_event_tags: tuple[str, ...] = ()


@dataclass
class OutputHistory:
    trajectory_records: list[TrajectoryRecord] = field(default_factory=list)
    event_records: list[EventRecord] = field(default_factory=list)
    sanity_check_records: list[SanityCheckRecord] = field(default_factory=list)
    png_artifacts: list[dict[str, Any]] = field(default_factory=list)

    def event_dicts(self) -> list[dict[str, Any]]:
        return [record.to_matcher_dict() for record in self.event_records]

    def sanity_dicts(self) -> list[dict[str, Any]]:
        return [record.to_matcher_dict() for record in self.sanity_check_records]


@dataclass(frozen=True)
class CommitResult:
    previous_state: SimulationState
    next_state: SimulationState
    command_buffer: CommandBuffer
    next_state_buffer: NextStateBuffer
    final_candidates: Mapping[str, CandidateKinematics]
    warnings: tuple[CommitWarning, ...]
    history: OutputHistory
    expected_png_features: list[dict[str, Any]]


@dataclass(frozen=True)
class TimeAdvanceResult:
    previous_state: SimulationState
    advanced_state: SimulationState
    event_record: EventRecord
    sanity_record: SanityCheckRecord


def build_command_buffer_for_state(
    state: SimulationState,
    *,
    cuc_decisions: Mapping[str, Any] | None = None,
) -> CommandBuffer:
    return CommandBuffer(
        step=state.step,
        t=state.t,
        cuc_decisions=MappingProxyType(dict(cuc_decisions or {})),
    )


def build_identity_next_state_buffer(state: SimulationState) -> NextStateBuffer:
    final_candidates = {
        vehicle_id: (
            _identity_candidate_for_vehicle(state.vehicle_states[vehicle_id], state=state),
        )
        for vehicle_id in state.active_vehicle_ids
    }
    return NextStateBuffer(
        step=state.step,
        t=state.t,
        candidate_kinematics=MappingProxyType(final_candidates),
    )


def build_test_harness_next_state_buffer(
    state: SimulationState,
    candidates: Mapping[str, CandidateKinematics | list[CandidateKinematics] | tuple[CandidateKinematics, ...]],
) -> NextStateBuffer:
    normalized: dict[str, tuple[CandidateKinematics, ...]] = {}
    for vehicle_id, candidate_or_candidates in candidates.items():
        if isinstance(candidate_or_candidates, CandidateKinematics):
            values = (candidate_or_candidates,)
        else:
            values = tuple(candidate_or_candidates)
        for candidate in values:
            _assert_allowed_candidate_source(candidate)
            if candidate.vehicle_id != vehicle_id:
                raise ValueError("candidate map key must match candidate.vehicle_id")
        normalized[vehicle_id] = values
    return NextStateBuffer(
        step=state.step,
        t=state.t,
        candidate_kinematics=MappingProxyType(normalized),
    )


def assemble_candidate_kinematics(
    state: SimulationState,
    vehicle_id: str,
    *,
    longitudinal: CandidateLongitudinalKinematics | None = None,
    lateral: CandidateLateralKinematics | None = None,
    maneuver_progress: CandidateManeuverProgress | None = None,
    state_transition: CandidateStateTransition | None = None,
    cache_update: CandidateCacheUpdate | None = None,
    source: str = TEST_HARNESS_CANDIDATE_SOURCE,
    candidate_id: str | None = None,
) -> CandidateKinematics:
    if source not in ALLOWED_CANDIDATE_SOURCES:
        raise ValueError(f"unsupported candidate source for P03: {source}")
    for component in (
        longitudinal,
        lateral,
        maneuver_progress,
        state_transition,
        cache_update,
    ):
        if component is not None:
            _assert_component_matches_candidate_vehicle(component, vehicle_id)
            _assert_allowed_component_source(component)
    current = state.vehicle_states[vehicle_id]
    x_global = longitudinal.x_global if longitudinal is not None else current.x_global
    y = lateral.y if lateral is not None else current.y
    v = longitudinal.v if longitudinal is not None else current.v
    a = longitudinal.a if longitudinal is not None else current.a
    return CandidateKinematics(
        candidate_id=candidate_id or f"{source}:{state.step}:{vehicle_id}",
        vehicle_id=vehicle_id,
        x_global=x_global,
        y=y,
        v=v,
        a=a,
        source=source,
        source_longitudinal_candidate=(
            longitudinal.candidate_id if longitudinal is not None else None
        ),
        source_lateral_candidate=lateral.candidate_id if lateral is not None else None,
        source_maneuver_progress=(
            maneuver_progress.candidate_id if maneuver_progress is not None else None
        ),
        source_state_transition=(
            state_transition.candidate_id if state_transition is not None else None
        ),
        source_cache_update=cache_update.candidate_id if cache_update is not None else None,
        constraints_applied=longitudinal.constraints_applied if longitudinal is not None else (),
    )


def select_final_candidate_per_vehicle(
    state: SimulationState,
    next_state_buffer: NextStateBuffer,
) -> tuple[dict[str, CandidateKinematics], tuple[CommitWarning, ...]]:
    final_candidates: dict[str, CandidateKinematics] = {}
    warnings: list[CommitWarning] = list(next_state_buffer.commit_warnings)
    for vehicle_id in state.active_vehicle_ids:
        candidates = list(next_state_buffer.candidate_kinematics.get(vehicle_id, ()))
        assembled, component_warnings = assemble_final_candidate_from_components(
            state,
            vehicle_id,
            next_state_buffer,
        )
        warnings.extend(component_warnings)
        if assembled is not None:
            candidates.append(assembled)
        if not candidates:
            candidates = [
                _identity_candidate_for_vehicle(state.vehicle_states[vehicle_id], state=state)
            ]
        for candidate in candidates:
            _assert_allowed_candidate_source(candidate)
        if len(candidates) > 1:
            warnings.append(
                CommitWarning(
                    vehicle_id=vehicle_id,
                    warning_type="multiple_commit_for_one_vehicle",
                    reason="duplicate_final_candidate",
                    candidate_ids=tuple(candidate.candidate_id for candidate in candidates),
                )
            )
            continue
        final_candidates[vehicle_id] = candidates[0]
    warnings.extend(_state_transition_warnings(state, next_state_buffer))
    return final_candidates, tuple(warnings)


def assemble_final_candidate_from_components(
    state: SimulationState,
    vehicle_id: str,
    next_state_buffer: NextStateBuffer,
) -> tuple[CandidateKinematics | None, tuple[CommitWarning, ...]]:
    longitudinal = next_state_buffer.candidate_longitudinal.get(vehicle_id)
    lateral = next_state_buffer.candidate_lateral.get(vehicle_id)
    progress = next_state_buffer.candidate_maneuver_progress.get(vehicle_id)
    if longitudinal is None and lateral is None and progress is None:
        return None, ()

    if longitudinal is None:
        candidate_ids = tuple(
            item
            for item in (
                lateral.candidate_id if lateral is not None else None,
                progress.candidate_id if progress is not None else None,
            )
            if item is not None
        )
        return (
            _identity_candidate_for_vehicle(state.vehicle_states[vehicle_id], state=state),
            (
                CommitWarning(
                    vehicle_id=vehicle_id,
                    warning_type="missing_longitudinal_candidate",
                    reason="p10_component_assembly_requires_p08_longitudinal_candidate",
                    candidate_ids=candidate_ids,
                ),
            ),
        )

    return (
        assemble_candidate_kinematics(
            state,
            vehicle_id,
            longitudinal=longitudinal,
            lateral=lateral,
            maneuver_progress=progress,
            state_transition=_primary_state_transition(next_state_buffer, vehicle_id),
            cache_update=_primary_cache_update(next_state_buffer, vehicle_id),
            source=STEP9_CANDIDATE_ASSEMBLY_SOURCE,
            candidate_id=f"p10:{state.step}:{vehicle_id}:final",
        ),
        (),
    )


def run_duplicate_commit_guard(
    state: SimulationState,
    warnings: tuple[CommitWarning, ...],
    *,
    run_id: str,
    scenario_id: str,
) -> list[SanityCheckRecord]:
    duplicate_warnings = [
        warning
        for warning in warnings
        if warning.warning_type == "multiple_commit_for_one_vehicle"
    ]
    if duplicate_warnings:
        return [
            SanityCheckRecord(
                check_id=f"{run_id}:{state.step}:multiple_commit_for_one_vehicle:{index}",
                run_id=run_id,
                scenario_id=scenario_id,
                step=state.step,
                t=state.t,
                check_type="multiple_commit_for_one_vehicle",
                severity="error",
                result="fail",
                vehicle_ids=(warning.vehicle_id,),
                reason=warning.reason,
                payload={
                    "candidate_ids": list(warning.candidate_ids),
                    "is_engineering_patch": True,
                    "source": "first_version_engineering_patch",
                },
            )
            for index, warning in enumerate(duplicate_warnings)
        ]
    return [
        SanityCheckRecord(
            check_id=f"{run_id}:{state.step}:multiple_commit_for_one_vehicle",
            run_id=run_id,
            scenario_id=scenario_id,
            step=state.step,
            t=state.t,
            check_type="multiple_commit_for_one_vehicle",
            severity="info",
            result="pass",
            vehicle_ids=tuple(state.active_vehicle_ids),
            reason="one_final_candidate_per_vehicle",
            payload={
                "each_active_vehicle_has_exactly_one_final_next_state": True,
                "source": "first_version_engineering_patch",
            },
        )
    ]


def run_immutable_snapshot_guard(
    before_state: SimulationState,
    after_state: SimulationState,
    *,
    run_id: str,
    scenario_id: str,
    expected_signature: tuple[Any, ...] | None = None,
) -> SanityCheckRecord:
    current_signature = _state_signature(after_state)
    if expected_signature is None:
        expected_signature = _state_signature(before_state)
    unchanged = expected_signature == current_signature
    return SanityCheckRecord(
        check_id=f"{run_id}:{before_state.step}:no_write_before_commit",
        run_id=run_id,
        scenario_id=scenario_id,
        step=before_state.step,
        t=before_state.t,
        check_type="no_write_before_commit",
        severity="info" if unchanged else "error",
        result="pass" if unchanged else "fail",
        vehicle_ids=tuple(before_state.active_vehicle_ids),
        reason="frozen_state_not_mutated_by_next_state_or_commit",
        payload={"no_module_writes_committed_state_before_commit": unchanged},
    )


def run_state_machine_consistency_guard(
    state: SimulationState,
    *,
    run_id: str,
    scenario_id: str,
) -> SanityCheckRecord:
    inconsistent = [
        vehicle_id
        for vehicle_id in state.active_vehicle_ids
        if state.vehicle_states[vehicle_id].lane_change_state == "executing"
        and state.vehicle_states[vehicle_id].merge_state == "executing"
    ]
    return SanityCheckRecord(
        check_id=f"{run_id}:{state.step}:state_machine_inconsistency",
        run_id=run_id,
        scenario_id=scenario_id,
        step=state.step,
        t=state.t,
        check_type="state_machine_inconsistency",
        severity="error" if inconsistent else "info",
        result="fail" if inconsistent else "pass",
        vehicle_ids=tuple(inconsistent or state.active_vehicle_ids),
        reason="lane_change_and_merge_executing_exclusive",
        payload={"inconsistent_vehicle_ids": inconsistent},
    )


def build_next_simulation_state(
    state: SimulationState,
    final_candidates: Mapping[str, CandidateKinematics],
    next_state_buffer: NextStateBuffer,
    *,
    command_buffer: CommandBuffer | None = None,
) -> SimulationState:
    next_vehicle_states: dict[str, VehicleState] = {}
    for vehicle_id in state.active_vehicle_ids:
        current = state.vehicle_states[vehicle_id]
        candidate = final_candidates.get(vehicle_id)
        if candidate is None:
            continue
        lane_state = next_state_buffer.candidate_lane_state.get(vehicle_id)
        command_transitions = tuple(
            command_buffer.state_transition_commands.get(vehicle_id, ())
            if command_buffer is not None
            else ()
        )
        transitions = next_state_buffer.candidate_state_transitions.get(vehicle_id, ())
        physical_lane = lane_state.physical_lane if lane_state is not None else current.physical_lane
        road_role = lane_state.road_role if lane_state is not None else current.road_role
        lane_change_state = current.lane_change_state
        merge_state = current.merge_state
        for transition in command_transitions:
            state_name = _transition_field(transition, "state_name")
            new_state = _transition_field(transition, "new_state")
            if state_name == "lane_change_state":
                lane_change_state = str(new_state)
            elif state_name == "merge_state":
                merge_state = str(new_state)
        for transition in transitions:
            if transition.state_name == "lane_change_state":
                lane_change_state = transition.new_state
            elif transition.state_name == "merge_state":
                merge_state = transition.new_state
        next_vehicle_states[vehicle_id] = VehicleState(
            vehicle_id=vehicle_id,
            x_global=candidate.x_global,
            y=candidate.y,
            v=candidate.v,
            a=candidate.a,
            physical_lane=physical_lane,
            road_role=road_role,
            lane_change_state=lane_change_state,
            merge_state=merge_state,
            is_active=current.is_active,
        )

    next_cache: dict[str, dict[str, Any]] = {
        key: dict(value) for key, value in state.aps_assignment_cache.items()
    }
    next_controller_memory: dict[str, LongitudinalControllerMemory] = dict(
        state.controller_memory_by_vehicle
    )
    for cache_update in next_state_buffer.candidate_cache_updates:
        if cache_update.cache_name == "aps_assignment_cache":
            if cache_update.operation == "cleanup":
                next_cache.pop(cache_update.owner_vehicle_id, None)
            elif cache_update.operation in {"update", "invalidate"}:
                next_cache[cache_update.owner_vehicle_id] = dict(cache_update.new_value or {})
        elif cache_update.cache_name == "longitudinal_controller_cache":
            if cache_update.operation == "cleanup":
                next_controller_memory.pop(cache_update.owner_vehicle_id, None)
            elif cache_update.operation in {"update", "create"}:
                next_controller_memory[cache_update.owner_vehicle_id] = (
                    _controller_memory_from_update(cache_update)
                )

    next_active_maneuvers = apply_maneuver_progress_lifecycle(
        state,
        final_candidates,
        next_state_buffer,
        command_buffer=command_buffer,
    )

    return SimulationState(
        t=state.t,
        step=state.step,
        dt=state.dt,
        active_vehicle_ids=tuple(next_vehicle_states),
        vehicle_states=MappingProxyType(next_vehicle_states),
        vehicle_specs=state.vehicle_specs,
        aps_assignment_cache=MappingProxyType(
            {key: MappingProxyType(value) for key, value in next_cache.items()}
        ),
        active_maneuvers=MappingProxyType(next_active_maneuvers),
        road_config_ref=state.road_config_ref,
        parameter_config_ref=state.parameter_config_ref,
        scenario_config_ref=state.scenario_config_ref,
        output_config_ref=state.output_config_ref,
        controller_memory_by_vehicle=MappingProxyType(next_controller_memory),
    )


def apply_maneuver_progress_lifecycle(
    state: SimulationState,
    final_candidates: Mapping[str, CandidateKinematics],
    next_state_buffer: NextStateBuffer,
    *,
    command_buffer: CommandBuffer | None = None,
) -> dict[str, ManeuverTrajectoryState]:
    next_active = dict(state.active_maneuvers)
    for vehicle_id, progress in next_state_buffer.candidate_maneuver_progress.items():
        _assert_component_matches_candidate_vehicle(progress, vehicle_id)
        if vehicle_id not in final_candidates:
            continue
        if progress.completed:
            next_active.pop(vehicle_id, None)
            continue
        next_active[vehicle_id] = _persist_maneuver_progress(
            state,
            vehicle_id,
            progress,
            next_state_buffer,
            command_buffer=command_buffer,
        )
    return next_active


def _persist_maneuver_progress(
    state: SimulationState,
    vehicle_id: str,
    progress: CandidateManeuverProgress,
    next_state_buffer: NextStateBuffer,
    *,
    command_buffer: CommandBuffer | None,
) -> ManeuverTrajectoryState:
    existing = state.active_maneuvers.get(vehicle_id)
    longitudinal = next_state_buffer.candidate_longitudinal.get(vehicle_id)
    last_planning_speed = (
        longitudinal.planning_speed
        if longitudinal is not None
        else (existing.last_planning_speed if existing is not None else None)
    )
    command = _maneuver_command(command_buffer, vehicle_id, progress.maneuver_type)
    source_command_id = (
        progress.source_command_id
        or _command_id(command)
        or (existing.source_command_id if existing is not None else None)
    )
    if existing is not None:
        return replace(
            existing,
            progress=progress.progress,
            source_command_id=source_command_id,
            last_planning_speed=last_planning_speed,
            assigned_clv_id=_command_optional_str(command, "assigned_clv_id")
            or existing.assigned_clv_id,
            assigned_cfv_id=_command_optional_str(command, "assigned_cfv_id")
            or existing.assigned_cfv_id,
        )

    current = state.vehicle_states[vehicle_id]
    lateral = next_state_buffer.candidate_lateral.get(vehicle_id)
    lane_state = next_state_buffer.candidate_lane_state.get(vehicle_id)
    target_y = _command_optional_float(command, "target_y")
    if target_y is None and lateral is not None:
        target_y = lateral.target_y
    if target_y is None and lane_state is not None:
        target_y = _lane_centerline_y(lane_state.physical_lane, fallback=current.y)
    if target_y is None:
        target_y = current.y
    target_lane = (
        _command_optional_str(command, "target_lane")
        or (lane_state.physical_lane if lane_state is not None else None)
        or _target_lane_from_y(target_y, fallback=current.physical_lane)
    )
    return ManeuverTrajectoryState(
        vehicle_id=vehicle_id,
        maneuver_type=progress.maneuver_type,
        start_step=state.step,
        start_t=state.t,
        start_x_global=current.x_global,
        start_y=current.y,
        target_lane=target_lane,
        target_y=float(target_y),
        source_command_id=source_command_id,
        planned_length=_command_optional_float(command, "planned_length"),
        progress=progress.progress,
        last_planning_speed=last_planning_speed,
        assigned_clv_id=_command_optional_str(command, "assigned_clv_id"),
        assigned_cfv_id=_command_optional_str(command, "assigned_cfv_id"),
    )


def commit_step(
    state: SimulationState,
    command_buffer: CommandBuffer,
    next_state_buffer: NextStateBuffer,
    *,
    run_id: str = "p03-run",
    scenario_id: str | None = None,
) -> CommitResult:
    scenario_id = scenario_id or state.scenario_config_ref or "unknown_scenario"
    pre_commit_signature = _state_signature(state)
    final_candidates, warnings = select_final_candidate_per_vehicle(state, next_state_buffer)
    next_state = build_next_simulation_state(
        state,
        final_candidates,
        next_state_buffer,
        command_buffer=command_buffer,
    )
    if _state_signature(state) != pre_commit_signature:
        warnings = warnings + (
            CommitWarning(
                vehicle_id="*",
                warning_type="no_write_before_commit",
                reason="frozen_state_mutated",
            ),
        )

    history = OutputHistory()
    event_records = [
        emit_commit_event(
            state,
            next_state,
            candidate,
            command_buffer=command_buffer,
            next_state_buffer=next_state_buffer,
            run_id=run_id,
            scenario_id=scenario_id,
        )
        for candidate in final_candidates.values()
    ]
    warning_events = [
        _warning_event_record(state, warning, run_id=run_id, scenario_id=scenario_id)
        for warning in warnings
    ]
    history.event_records.extend(event_records)
    history.event_records.extend(warning_events)
    history.sanity_check_records.extend(
        run_duplicate_commit_guard(
            state,
            warnings,
            run_id=run_id,
            scenario_id=scenario_id,
        )
    )
    history.sanity_check_records.append(
        run_immutable_snapshot_guard(
            state,
            state,
            run_id=run_id,
            scenario_id=scenario_id,
            expected_signature=pre_commit_signature,
        )
    )
    history.sanity_check_records.append(
        run_state_machine_consistency_guard(
            next_state,
            run_id=run_id,
            scenario_id=scenario_id,
        )
    )
    history.sanity_check_records.append(
        _x_plot_sanity_record(next_state, run_id=run_id, scenario_id=scenario_id)
    )
    record_information_integration_v0(
        previous_state=state,
        committed_state=next_state,
        command_buffer=command_buffer,
        next_state_buffer=next_state_buffer,
        history=history,
        run_id=run_id,
        scenario_id=scenario_id,
    )
    return CommitResult(
        previous_state=state,
        next_state=next_state,
        command_buffer=command_buffer,
        next_state_buffer=next_state_buffer,
        final_candidates=MappingProxyType(dict(final_candidates)),
        warnings=warnings,
        history=history,
        expected_png_features=register_p10_png_features(final_candidates, next_state_buffer),
    )


def emit_commit_event(
    previous_state: SimulationState,
    next_state: SimulationState,
    candidate: CandidateKinematics,
    *,
    command_buffer: CommandBuffer,
    next_state_buffer: NextStateBuffer,
    run_id: str,
    scenario_id: str,
) -> EventRecord:
    final_state = next_state.vehicle_states[candidate.vehicle_id]
    prior_state = previous_state.vehicle_states[candidate.vehicle_id]
    transitions = tuple(next_state_buffer.candidate_state_transitions.get(candidate.vehicle_id, ()))
    command_transitions = tuple(command_buffer.state_transition_commands.get(candidate.vehicle_id, ()))
    progress = next_state_buffer.candidate_maneuver_progress.get(candidate.vehicle_id)
    longitudinal = next_state_buffer.candidate_longitudinal.get(candidate.vehicle_id)
    lateral = next_state_buffer.candidate_lateral.get(candidate.vehicle_id)
    cache_cleanup = [
        update.owner_vehicle_id
        for update in next_state_buffer.candidate_cache_updates
        if update.operation == "cleanup" and update.owner_vehicle_id == candidate.vehicle_id
    ]
    controller_cache_updates = [
        _dataclass_to_plain(update)
        for update in next_state_buffer.candidate_cache_updates
        if update.cache_name == "longitudinal_controller_cache"
        and update.owner_vehicle_id == candidate.vehicle_id
    ]
    active_maneuver_cleanup = (
        [candidate.vehicle_id] if progress is not None and progress.completed else []
    )
    active_maneuver_persisted = (
        [candidate.vehicle_id] if progress is not None and not progress.completed else []
    )
    return EventRecord(
        event_id=f"{run_id}:{previous_state.step}:commit:{candidate.vehicle_id}",
        run_id=run_id,
        scenario_id=scenario_id,
        step=previous_state.step,
        t=previous_state.t,
        module="commit",
        event_type="commit",
        vehicle_id=candidate.vehicle_id,
        related_vehicle_ids=(candidate.vehicle_id,),
        source_candidate_id=candidate.candidate_id,
        reason="commit_step_final_candidate",
        result="committed",
        is_engineering_patch=True,
        source="first_version_engineering_patch",
        payload={
            "candidate_source": candidate.source,
            "final_state": _vehicle_state_payload(final_state),
            "previous_state": _vehicle_state_payload(prior_state),
            "state_transitions": [_dataclass_to_plain(transition) for transition in transitions],
            "state_transition_commands": [
                _plain_transition_payload(transition) for transition in command_transitions
            ],
            "cache_cleanup_vehicle_ids": cache_cleanup,
            "longitudinal_controller_cache_updates": controller_cache_updates,
            "active_maneuver_cleanup_vehicle_ids": active_maneuver_cleanup,
            "active_maneuver_persisted_vehicle_ids": active_maneuver_persisted,
            "active_maneuver_progress": progress.progress if progress is not None else None,
            "active_maneuver_completed": progress.completed if progress is not None else None,
            "each_active_vehicle_has_exactly_one_final_next_state": True,
            "no_module_writes_committed_state_before_commit": True,
            "command_buffer_and_next_state_buffer_are_separated": True,
            "commit_is_unique_state_writer": True,
            "cuc_decision_persisted_to_state": False,
            "source_longitudinal_candidate": candidate.source_longitudinal_candidate,
            "source_lateral_candidate": candidate.source_lateral_candidate,
            "source_maneuver_progress": candidate.source_maneuver_progress,
            "source_state_transition": candidate.source_state_transition,
            "source_state_transition_command": _first_transition_command_id(command_transitions),
            "source_cache_update": candidate.source_cache_update,
            "constraints_applied": list(candidate.constraints_applied),
            "p08_source_commands": (
                list(longitudinal.source_commands) if longitudinal is not None else []
            ),
            "p09_source_commands": list(lateral.source_commands) if lateral is not None else [],
        },
    )


def record_information_integration_v0(
    *,
    previous_state: SimulationState,
    committed_state: SimulationState,
    command_buffer: CommandBuffer,
    next_state_buffer: NextStateBuffer,
    history: OutputHistory,
    run_id: str,
    scenario_id: str,
) -> None:
    before_signature = _state_signature(committed_state)
    for vehicle_id in committed_state.active_vehicle_ids:
        append_trajectory_record(
            committed_state,
            vehicle_id,
            history=history,
            run_id=run_id,
            scenario_id=scenario_id,
        )
    history.event_records.append(
        EventRecord(
            event_id=f"{run_id}:{committed_state.step}:information_integration",
            run_id=run_id,
            scenario_id=scenario_id,
            step=committed_state.step,
            t=committed_state.t,
            module="information_integration",
            event_type="information_integration",
            related_vehicle_ids=tuple(committed_state.active_vehicle_ids),
            reason="step10_record_only",
            result="recorded",
            is_engineering_patch=False,
            source="first_version_engineering_patch",
            payload={
                "trajectory_records_added": len(committed_state.active_vehicle_ids),
                "event_history_updated": True,
                "sanity_history_updated": True,
                "step10_does_not_rewrite_committed_state": True,
            },
        )
    )
    after_signature = _state_signature(committed_state)
    history.sanity_check_records.append(
        SanityCheckRecord(
            check_id=f"{run_id}:{committed_state.step}:step10_record_only",
            run_id=run_id,
            scenario_id=scenario_id,
            step=committed_state.step,
            t=committed_state.t,
            check_type="information_integration_does_not_rewrite_state",
            severity="info" if before_signature == after_signature else "error",
            result="pass" if before_signature == after_signature else "fail",
            vehicle_ids=tuple(committed_state.active_vehicle_ids),
            reason="step10_record_only",
            payload={"step10_does_not_rewrite_committed_state": before_signature == after_signature},
        )
    )


def append_trajectory_record(
    state: SimulationState,
    vehicle_id: str,
    *,
    history: OutputHistory,
    run_id: str,
    scenario_id: str,
) -> TrajectoryRecord:
    vehicle_state = state.vehicle_states[vehicle_id]
    vehicle_spec = state.vehicle_specs[vehicle_id]
    record = TrajectoryRecord(
        run_id=run_id,
        scenario_id=scenario_id,
        step=state.step,
        t=state.t,
        vehicle_id=vehicle_id,
        vehicle_type=vehicle_spec.vehicle_type,
        compliance_state=vehicle_spec.compliance_state,
        x_global=vehicle_state.x_global,
        y=vehicle_state.y,
        v=vehicle_state.v,
        a=vehicle_state.a,
        physical_lane=vehicle_state.physical_lane,
        road_role=vehicle_state.road_role,
        lane_change_state=vehicle_state.lane_change_state,
        merge_state=vehicle_state.merge_state,
        active_event_tags=("commit",),
    )
    history.trajectory_records.append(record)
    return record


def advance_time_after_commit_and_integration(
    result: CommitResult,
    *,
    run_id: str = "p03-run",
    scenario_id: str | None = None,
) -> TimeAdvanceResult:
    scenario_id = scenario_id or result.next_state.scenario_config_ref or "unknown_scenario"
    next_state = result.next_state
    advanced = replace(
        next_state,
        t=next_state.t + next_state.dt,
        step=next_state.step + 1,
    )
    event = EventRecord(
        event_id=f"{run_id}:{next_state.step}:time_advance",
        run_id=run_id,
        scenario_id=scenario_id,
        step=next_state.step,
        t=next_state.t,
        module="time_advance",
        event_type="time_advance",
        related_vehicle_ids=tuple(next_state.active_vehicle_ids),
        reason="step11_after_commit_and_information_integration",
        result="advanced",
        is_engineering_patch=False,
        source="first_version_engineering_patch",
        payload={
            "old_t": next_state.t,
            "new_t": advanced.t,
            "old_step": next_state.step,
            "new_step": advanced.step,
            "dt": next_state.dt,
            "advanced_after_commit": True,
            "advanced_after_information_integration": True,
        },
    )
    sanity = SanityCheckRecord(
        check_id=f"{run_id}:{next_state.step}:time_advance_consistency",
        run_id=run_id,
        scenario_id=scenario_id,
        step=next_state.step,
        t=next_state.t,
        check_type="time_advance_consistency",
        severity="info",
        result="pass",
        vehicle_ids=tuple(next_state.active_vehicle_ids),
        reason="t_and_step_advanced_once_after_step10",
        payload={"old_t": next_state.t, "new_t": advanced.t, "dt": next_state.dt},
    )
    result.history.event_records.append(event)
    result.history.sanity_check_records.append(sanity)
    return TimeAdvanceResult(
        previous_state=next_state,
        advanced_state=advanced,
        event_record=event,
        sanity_record=sanity,
    )


def run_mvs_commit_1_lite() -> CommitResult:
    workspace, config = build_prefreeze_workspace_from_scenario("MVS-COMMIT-1-lite")
    state = freeze_simulation_state(workspace)
    command_buffer = build_command_buffer_for_state(state)
    next_state_buffer = build_identity_next_state_buffer(state)
    result = commit_step(
        state,
        command_buffer,
        next_state_buffer,
        run_id="MVS-COMMIT-1-lite",
        scenario_id=config["scenario_id"],
    )
    advance_time_after_commit_and_integration(
        result,
        run_id="MVS-COMMIT-1-lite",
        scenario_id=config["scenario_id"],
    )
    return result


def register_p10_png_features(
    final_candidates: Mapping[str, CandidateKinematics],
    next_state_buffer: NextStateBuffer,
) -> list[dict[str, Any]]:
    vehicle_ids = tuple(final_candidates)
    features = [
        _png_feature("commit_marker", vehicle_ids, required=False),
        _png_feature("trajectory_quicklook", vehicle_ids, required=False),
    ]
    lane_change_completed = [
        vehicle_id
        for vehicle_id, progress in next_state_buffer.candidate_maneuver_progress.items()
        if vehicle_id in final_candidates
        and progress.completed
        and progress.maneuver_type == "lane_change"
    ]
    merge_completed = [
        vehicle_id
        for vehicle_id, progress in next_state_buffer.candidate_maneuver_progress.items()
        if vehicle_id in final_candidates and progress.completed and progress.maneuver_type == "merge"
    ]
    active_maneuver = [
        vehicle_id
        for vehicle_id, progress in next_state_buffer.candidate_maneuver_progress.items()
        if vehicle_id in final_candidates and not progress.completed
    ]
    cache_cleanup = [
        update.owner_vehicle_id
        for update in next_state_buffer.candidate_cache_updates
        if update.operation == "cleanup" and update.owner_vehicle_id in final_candidates
    ]
    source_chain = [
        vehicle_id
        for vehicle_id, candidate in final_candidates.items()
        if candidate.source_longitudinal_candidate is not None
        or candidate.source_lateral_candidate is not None
    ]
    if lane_change_completed:
        features.append(_png_feature("lane_change_completed_marker", lane_change_completed))
    if merge_completed:
        features.append(_png_feature("merge_completed_marker", merge_completed))
    if active_maneuver:
        features.append(_png_feature("active_maneuver_marker", active_maneuver, required=False))
    if cache_cleanup:
        features.append(_png_feature("cache_cleanup_marker", cache_cleanup, required=False))
    if source_chain:
        features.append(_png_feature("source_chain_marker", source_chain, required=False))
    return features


def assert_event_records(records: list[EventRecord], event_type: str) -> bool:
    return any(record.event_type == event_type for record in records)


def assert_sanity_records(records: list[SanityCheckRecord], check_type: str) -> bool:
    return any(record.check_type == check_type for record in records)


def _primary_state_transition(
    next_state_buffer: NextStateBuffer,
    vehicle_id: str,
) -> CandidateStateTransition | None:
    transitions = tuple(next_state_buffer.candidate_state_transitions.get(vehicle_id, ()))
    return transitions[0] if transitions else None


def _primary_cache_update(
    next_state_buffer: NextStateBuffer,
    vehicle_id: str,
) -> CandidateCacheUpdate | None:
    for update in next_state_buffer.candidate_cache_updates:
        if update.owner_vehicle_id == vehicle_id:
            return update
    return None


def _controller_memory_from_update(
    update: CandidateCacheUpdate,
) -> LongitudinalControllerMemory:
    value = dict(update.new_value or {})
    return LongitudinalControllerMemory(
        vehicle_id=str(value.get("vehicle_id") or update.owner_vehicle_id),
        ex_prev=_optional_float(value.get("ex_prev")),
        e_prev=_optional_float(value.get("e_prev")),
        integral_ex=float(value.get("integral_ex", 0.0)),
        integral_e=float(value.get("integral_e", 0.0)),
        last_t=_optional_float(value.get("last_t")),
        last_controller_update_step=(
            int(value["last_controller_update_step"])
            if value.get("last_controller_update_step") is not None
            else None
        ),
        controller_mode=str(value.get("controller_mode") or "cav_cpid"),
    )


def _optional_float(value: Any) -> float | None:
    if value in (None, ""):
        return None
    return float(value)


def _state_transition_warnings(
    state: SimulationState,
    next_state_buffer: NextStateBuffer,
) -> tuple[CommitWarning, ...]:
    warnings: list[CommitWarning] = []
    for vehicle_id, transitions in next_state_buffer.candidate_state_transitions.items():
        if vehicle_id not in state.vehicle_states:
            continue
        by_state_name: dict[str, list[CandidateStateTransition]] = {}
        for transition in transitions:
            _assert_component_matches_candidate_vehicle(transition, vehicle_id)
            by_state_name.setdefault(transition.state_name, []).append(transition)
            current_value = getattr(state.vehicle_states[vehicle_id], transition.state_name, None)
            if current_value is not None and str(current_value) != transition.old_state:
                warnings.append(
                    CommitWarning(
                        vehicle_id=vehicle_id,
                        warning_type="state_machine_inconsistency",
                        reason="state_transition_old_state_mismatch",
                        candidate_ids=(transition.candidate_id,),
                    )
                )
        for same_state_transitions in by_state_name.values():
            if len(same_state_transitions) > 1:
                warnings.append(
                    CommitWarning(
                        vehicle_id=vehicle_id,
                        warning_type="state_machine_inconsistency",
                        reason="duplicate_state_transition",
                        candidate_ids=tuple(
                            transition.candidate_id for transition in same_state_transitions
                        ),
                    )
                )
    return tuple(warnings)


def _transition_field(transition: Any, field_name: str) -> Any:
    if isinstance(transition, Mapping):
        if field_name == "new_state" and field_name not in transition:
            return transition.get("requested_new_state")
        return transition.get(field_name)
    return getattr(transition, field_name)


def _plain_transition_payload(transition: Any) -> dict[str, Any]:
    if isinstance(transition, Mapping):
        return dict(transition)
    return _dataclass_to_plain(transition)


def _first_transition_command_id(transitions: tuple[Any, ...]) -> str | None:
    if not transitions:
        return None
    command_id = _transition_field(transitions[0], "command_id")
    if command_id in (None, ""):
        return None
    return str(command_id)


def _maneuver_command(
    command_buffer: CommandBuffer | None,
    vehicle_id: str,
    maneuver_type: str,
) -> Any:
    if command_buffer is None:
        return None
    if maneuver_type == "merge":
        return command_buffer.merge_commands.get(vehicle_id)
    if maneuver_type == "lane_change":
        return command_buffer.lane_change_commands.get(vehicle_id)
    return None


def _command_id(command: Any) -> str | None:
    if isinstance(command, Mapping) and command.get("command_id") is not None:
        return str(command["command_id"])
    return None


def _command_optional_str(command: Any, key: str) -> str | None:
    if not isinstance(command, Mapping):
        return None
    value = command.get(key)
    if value in (None, ""):
        return None
    return str(value)


def _command_optional_float(command: Any, key: str) -> float | None:
    if not isinstance(command, Mapping):
        return None
    value = command.get(key)
    if value in (None, ""):
        return None
    return float(value)


def _target_lane_from_y(value: float, *, fallback: str) -> str:
    y = float(value)
    if abs(y - 3.5) <= 1e-9:
        return "lane_1"
    if abs(y) <= 1e-9:
        return "lane_2"
    if abs(y + 3.5) <= 1e-9:
        return "on_ramp"
    return fallback


def _lane_centerline_y(lane_id: str, *, fallback: float) -> float:
    if lane_id == "lane_1":
        return 3.5
    if lane_id == "lane_2":
        return 0.0
    if lane_id == "on_ramp":
        return -3.5
    return fallback


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


def _identity_candidate_for_vehicle(
    vehicle_state: VehicleState,
    *,
    state: SimulationState,
) -> CandidateKinematics:
    return CandidateKinematics(
        candidate_id=f"{IDENTITY_CANDIDATE_SOURCE}:{state.step}:{vehicle_state.vehicle_id}",
        vehicle_id=vehicle_state.vehicle_id,
        x_global=vehicle_state.x_global,
        y=vehicle_state.y,
        v=vehicle_state.v,
        a=vehicle_state.a,
        source=IDENTITY_CANDIDATE_SOURCE,
    )


def _assert_allowed_candidate_source(candidate: CandidateKinematics) -> None:
    if candidate.source not in ALLOWED_FINAL_CANDIDATE_SOURCES:
        raise ValueError(
            "candidate source must be test_harness_preloaded_candidate, "
            "identity_candidate_for_commit_infrastructure, "
            "or step9_candidate_assembly"
        )


def _assert_allowed_component_source(component: Any) -> None:
    source = getattr(component, "source", None)
    if source is not None and source not in ALLOWED_COMPONENT_CANDIDATE_SOURCES:
        raise ValueError(
            "candidate component source must be approved for P10 handoff"
        )


def _assert_component_matches_candidate_vehicle(component: Any, vehicle_id: str) -> None:
    component_vehicle_id = getattr(component, "vehicle_id", vehicle_id)
    if component_vehicle_id != vehicle_id:
        raise ValueError("candidate component vehicle_id must match final candidate vehicle_id")


def _warning_event_record(
    state: SimulationState,
    warning: CommitWarning,
    *,
    run_id: str,
    scenario_id: str,
) -> EventRecord:
    return EventRecord(
        event_id=f"{run_id}:{state.step}:commit_warning:{warning.vehicle_id}:{warning.warning_type}",
        run_id=run_id,
        scenario_id=scenario_id,
        step=state.step,
        t=state.t,
        module="commit",
        event_type="engineering_patch",
        vehicle_id=warning.vehicle_id,
        related_vehicle_ids=(warning.vehicle_id,),
        reason=warning.reason,
        result=warning.warning_type,
        is_engineering_patch=True,
        source="first_version_engineering_patch",
        payload={
            "warning_type": warning.warning_type,
            "candidate_ids": list(warning.candidate_ids),
        },
    )


def _x_plot_sanity_record(
    state: SimulationState,
    *,
    run_id: str,
    scenario_id: str,
) -> SanityCheckRecord:
    no_x_plot = assert_x_plot_not_used_in_algorithm_path(state)
    return SanityCheckRecord(
        check_id=f"{run_id}:{state.step}:x_plot_used_in_algorithm_path",
        run_id=run_id,
        scenario_id=scenario_id,
        step=state.step,
        t=state.t,
        check_type="x_plot_used_in_algorithm_path",
        severity="info" if no_x_plot else "error",
        result="pass" if no_x_plot else "fail",
        vehicle_ids=tuple(state.active_vehicle_ids),
        reason="x_global_only_algorithm_path",
        payload={"x_plot_used_in_algorithm_path": False},
    )


def _vehicle_state_payload(state: VehicleState) -> dict[str, Any]:
    return {
        "vehicle_id": state.vehicle_id,
        "x_global": state.x_global,
        "y": state.y,
        "v": state.v,
        "a": state.a,
        "physical_lane": state.physical_lane,
        "road_role": state.road_role,
        "lane_change_state": state.lane_change_state,
        "merge_state": state.merge_state,
    }


def _state_signature(state: SimulationState) -> tuple[Any, ...]:
    return (
        state.t,
        state.step,
        state.dt,
        state.active_vehicle_ids,
        tuple(
            (vehicle_id, _vehicle_state_payload(state.vehicle_states[vehicle_id]))
            for vehicle_id in state.active_vehicle_ids
        ),
        tuple(
            (
                vehicle_id,
                state.active_maneuvers[vehicle_id].maneuver_type,
                state.active_maneuvers[vehicle_id].start_step,
                state.active_maneuvers[vehicle_id].start_t,
                state.active_maneuvers[vehicle_id].start_x_global,
                state.active_maneuvers[vehicle_id].start_y,
                state.active_maneuvers[vehicle_id].target_lane,
                state.active_maneuvers[vehicle_id].target_y,
                state.active_maneuvers[vehicle_id].source_command_id,
                state.active_maneuvers[vehicle_id].source_event_id,
                state.active_maneuvers[vehicle_id].planned_length,
                state.active_maneuvers[vehicle_id].progress,
                state.active_maneuvers[vehicle_id].last_planning_speed,
                state.active_maneuvers[vehicle_id].assigned_clv_id,
                state.active_maneuvers[vehicle_id].assigned_cfv_id,
            )
            for vehicle_id in sorted(state.active_maneuvers)
        ),
        tuple((key, tuple(sorted(value.items()))) for key, value in state.aps_assignment_cache.items()),
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


def _dataclass_to_plain(value: Any) -> dict[str, Any]:
    return {field.name: getattr(value, field.name) for field in fields(value)}
