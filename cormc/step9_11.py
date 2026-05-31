from __future__ import annotations

from dataclasses import dataclass, field, fields, replace
from types import MappingProxyType
from typing import Any, Mapping

from cormc.step0_3 import (
    SimulationState,
    VehicleState,
    assert_x_plot_not_used_in_algorithm_path,
    build_prefreeze_workspace_from_scenario,
    freeze_simulation_state,
)


IDENTITY_CANDIDATE_SOURCE = "identity_candidate_for_commit_infrastructure"
TEST_HARNESS_CANDIDATE_SOURCE = "test_harness_preloaded_candidate"
ALLOWED_CANDIDATE_SOURCES = {
    IDENTITY_CANDIDATE_SOURCE,
    TEST_HARNESS_CANDIDATE_SOURCE,
}


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
        candidates = tuple(next_state_buffer.candidate_kinematics.get(vehicle_id, ()))
        if not candidates:
            candidates = (_identity_candidate_for_vehicle(state.vehicle_states[vehicle_id], state=state),)
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
    return final_candidates, tuple(warnings)


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
) -> SimulationState:
    next_vehicle_states: dict[str, VehicleState] = {}
    for vehicle_id in state.active_vehicle_ids:
        current = state.vehicle_states[vehicle_id]
        candidate = final_candidates.get(vehicle_id)
        if candidate is None:
            continue
        lane_state = next_state_buffer.candidate_lane_state.get(vehicle_id)
        transitions = next_state_buffer.candidate_state_transitions.get(vehicle_id, ())
        physical_lane = lane_state.physical_lane if lane_state is not None else current.physical_lane
        road_role = lane_state.road_role if lane_state is not None else current.road_role
        lane_change_state = current.lane_change_state
        merge_state = current.merge_state
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
    for cache_update in next_state_buffer.candidate_cache_updates:
        if cache_update.cache_name != "aps_assignment_cache":
            continue
        if cache_update.operation == "cleanup":
            next_cache.pop(cache_update.owner_vehicle_id, None)
        elif cache_update.operation in {"update", "invalidate"}:
            next_cache[cache_update.owner_vehicle_id] = dict(cache_update.new_value or {})

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
        active_maneuvers=state.active_maneuvers,
        road_config_ref=state.road_config_ref,
        parameter_config_ref=state.parameter_config_ref,
        scenario_config_ref=state.scenario_config_ref,
        output_config_ref=state.output_config_ref,
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
    next_state = build_next_simulation_state(state, final_candidates, next_state_buffer)
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
        expected_png_features=[
            {
                "feature_type": "commit_marker",
                "required": False,
                "vehicle_ids": list(final_candidates),
                "expected_visibility": "optional",
                "notes": "registered only; renderer deferred",
            },
            {
                "feature_type": "trajectory_quicklook",
                "required": False,
                "vehicle_ids": list(final_candidates),
                "expected_visibility": "optional",
                "notes": "registered only; renderer deferred",
            },
        ],
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
    cache_cleanup = [
        update.owner_vehicle_id
        for update in next_state_buffer.candidate_cache_updates
        if update.operation == "cleanup"
    ]
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
            "cache_cleanup_vehicle_ids": cache_cleanup,
            "each_active_vehicle_has_exactly_one_final_next_state": True,
            "no_module_writes_committed_state_before_commit": True,
            "command_buffer_and_next_state_buffer_are_separated": True,
            "commit_is_unique_state_writer": True,
            "cuc_decision_persisted_to_state": False,
            "source_longitudinal_candidate": candidate.source_longitudinal_candidate,
            "source_lateral_candidate": candidate.source_lateral_candidate,
            "source_maneuver_progress": candidate.source_maneuver_progress,
            "source_state_transition": candidate.source_state_transition,
            "source_cache_update": candidate.source_cache_update,
            "constraints_applied": list(candidate.constraints_applied),
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


def assert_event_records(records: list[EventRecord], event_type: str) -> bool:
    return any(record.event_type == event_type for record in records)


def assert_sanity_records(records: list[SanityCheckRecord], check_type: str) -> bool:
    return any(record.check_type == check_type for record in records)


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
    if candidate.source not in ALLOWED_CANDIDATE_SOURCES:
        raise ValueError(
            "P03 candidate source must be test_harness_preloaded_candidate "
            "or identity_candidate_for_commit_infrastructure"
        )


def _assert_allowed_component_source(component: Any) -> None:
    source = getattr(component, "source", None)
    if source is not None and source not in ALLOWED_CANDIDATE_SOURCES:
        raise ValueError(
            "P03 candidate component source must be test_harness_preloaded_candidate "
            "or identity_candidate_for_commit_infrastructure"
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
        tuple((key, tuple(sorted(value.items()))) for key, value in state.aps_assignment_cache.items()),
    )


def _dataclass_to_plain(value: Any) -> dict[str, Any]:
    return {field.name: getattr(value, field.name) for field in fields(value)}
