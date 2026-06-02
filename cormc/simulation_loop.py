from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from types import MappingProxyType
from typing import Any, Callable, Iterable, Mapping

from cormc.mvs.loader import load_builtin_scenario, load_scenario_config
from cormc.step0_3 import (
    DEFAULT_ROAD_GEOMETRY,
    ManeuverTrajectoryState,
    PreFreezeWorkspace,
    RelationsSnapshot,
    RoadGeometryConfig,
    SimulationState,
    VehicleSpec,
    VehicleState,
    emit_freeze_event_candidate,
    emit_geometry_event_candidate,
    emit_relation_refresh_event_candidate,
    freeze_simulation_state,
    refresh_relations_snapshot,
    run_geometry_sanity_baseline,
    step0_cleanup_and_prepare,
    step1_prefreeze_boundary_generation_hook,
)
from cormc.step4a_aps import APSCacheAction, APSRunResult, run_step4a_aps
from cormc.step4b_cmc import Step4BCMCRunResult, run_step4b_cmc
from cormc.step5_cooperative_request import (
    Step5CooperativeRequestRunResult,
    run_step5_cooperative_request_conflict_resolution,
)
from cormc.step6_cuc import Step6CUCRunResult, run_step6_cuc_choice_compliance_lane_change_overlay
from cormc.step7_longitudinal import Step7LongitudinalRunResult, run_step7_longitudinal_model_spacing_speedcap
from cormc.step8_lateral import Step8LateralRunResult, run_step8_lateral_trajectory_planning_speed_progress
from cormc.step9_11 import (
    CandidateCacheUpdate,
    CommandBuffer,
    CommitResult,
    CommitWarning,
    EventRecord,
    NextStateBuffer,
    OutputHistory,
    SanityCheckRecord,
    TimeAdvanceResult,
    advance_time_after_commit_and_integration,
    commit_step,
)


LANE_CHANGE_CENTERLINE_LENGTH_M = 100.0
LANE_CHANGE_CENTERLINE_LENGTH_SOURCE = "lane_change_centerline_length"


@dataclass(frozen=True)
class SimulationLoopConfig:
    scenario: str | Mapping[str, Any] | None = None
    scenario_id: str | None = None
    run_id: str = "p12-run"
    max_steps: int = 1
    stop_conditions: tuple[str | Callable[[SimulationState], bool], ...] = ()
    output_dir: str | Path = "artifacts"
    render_png: bool = True
    deterministic_profile_enabled: bool = True


@dataclass(frozen=True)
class Step0To3LoopResult:
    state: SimulationState
    relations: RelationsSnapshot
    actual_events: list[dict[str, Any]]
    actual_sanity_checks: list[dict[str, Any]]
    expected_png_features: list[dict[str, Any]]


@dataclass(frozen=True)
class StepLoopTrace:
    step: int
    t: float
    step0_3_result: Step0To3LoopResult
    aps_result: APSRunResult
    cmc_result: Step4BCMCRunResult
    p06_result: Step5CooperativeRequestRunResult
    cuc_result: Step6CUCRunResult
    p08_result: Step7LongitudinalRunResult
    p09_result: Step8LateralRunResult
    canonical_command_buffer: CommandBuffer
    canonical_next_state_buffer: NextStateBuffer
    commit_result: CommitResult
    time_advance_result: TimeAdvanceResult
    actual_events: list[dict[str, Any]] = field(default_factory=list)
    actual_sanity_checks: list[dict[str, Any]] = field(default_factory=list)
    expected_png_features: list[dict[str, Any]] = field(default_factory=list)


@dataclass(frozen=True)
class SimulationLoopResult:
    initial_state: SimulationState
    final_state: SimulationState
    history: OutputHistory
    step_traces: tuple[StepLoopTrace, ...]
    expected_png_features: tuple[dict[str, Any], ...]
    png_path: str | None
    status: str
    scenario_id: str
    run_id: str


def run_deterministic_simulation(config: SimulationLoopConfig) -> SimulationLoopResult:
    scenario_config = _load_loop_scenario_config(config)
    scenario_id = str(scenario_config["scenario_id"])
    workspace = _workspace_from_scenario_config(scenario_config)
    initial_state = freeze_simulation_state(workspace)
    state = initial_state
    traces: list[StepLoopTrace] = []
    history = OutputHistory()
    expected_png_features: list[dict[str, Any]] = []
    status = "max_steps_reached"

    for _ in range(max(0, int(config.max_steps))):
        trace = run_one_deterministic_step(
            state,
            scenario_config,
            run_id=config.run_id,
        )
        traces.append(trace)
        expected_png_features.extend(trace.expected_png_features)
        _extend_history(history, _pre_commit_history(trace, config.run_id, scenario_id))
        _extend_history(history, trace.commit_result.history)
        state = trace.time_advance_result.advanced_state
        if _stop_conditions_met(state, config.stop_conditions):
            status = "stopped_by_condition"
            break

    png_path: str | None = None
    if config.render_png:
        from cormc.p11_output import render_time_space_png

        png_target = Path(config.output_dir) / scenario_id / config.run_id / "time_space.png"
        render = render_time_space_png(
            history.trajectory_records,
            expected_png_features,
            png_target,
            events=history.event_records,
        )
        png_path = render.png_path
        history.png_artifacts.append(render.to_dict())

    return SimulationLoopResult(
        initial_state=initial_state,
        final_state=state,
        history=history,
        step_traces=tuple(traces),
        expected_png_features=tuple(_unique_features(expected_png_features)),
        png_path=png_path,
        status=status,
        scenario_id=scenario_id,
        run_id=config.run_id,
    )


def run_one_deterministic_step(
    state: SimulationState,
    scenario_config: Mapping[str, Any],
    run_id: str,
) -> StepLoopTrace:
    config = load_scenario_config(dict(scenario_config))
    scenario_id = str(config["scenario_id"])
    step0_3 = _run_step0_to_3_from_state(state, config)
    frozen = step0_3.state
    relations = step0_3.relations

    p04 = run_step4a_aps(frozen, relations, config=config)
    p05 = run_step4b_cmc(
        frozen,
        relations,
        config=config,
        effective_assignments=p04.effective_assignments,
    )
    p06 = run_step5_cooperative_request_conflict_resolution(
        frozen,
        relations,
        config=config,
        effective_assignments=p04.effective_assignments,
    )
    p07 = run_step6_cuc_choice_compliance_lane_change_overlay(
        frozen,
        relations,
        active_requests=p06.active_requests,
        suppressed_requests=p06.suppressed_requests,
        utility_overrides=_utility_overrides(config),
    )
    command_buffer = normalize_maneuver_commands(
        build_step_command_buffer(p05, p06, p07)
    )
    command_sanity = _command_buffer_sanity(
        frozen,
        command_buffer,
        run_id=run_id,
        scenario_id=scenario_id,
    )
    p08 = run_step7_longitudinal_model_spacing_speedcap(
        frozen,
        relations,
        command_buffer=command_buffer,
    )
    p09 = run_step8_lateral_trajectory_planning_speed_progress(
        frozen,
        relations,
        command_buffer=command_buffer,
        p08_next_state_buffer=p08.next_state_buffer,
        planning_speeds=p08.planning_speeds,
        boundary_risk_diagnostics=_boundary_risk_diagnostics(command_buffer),
    )
    next_state_buffer = build_step_next_state_buffer(
        p04,
        p05,
        p08,
        p09,
        command_buffer,
    )
    commit = commit_step(
        frozen,
        command_buffer,
        next_state_buffer,
        run_id=run_id,
        scenario_id=scenario_id,
    )
    time_advance = advance_time_after_commit_and_integration(
        commit,
        run_id=run_id,
        scenario_id=scenario_id,
    )
    events = _collect_dict_events(step0_3, p04, p05, p06, p07, p08, p09)
    sanity = [
        *_collect_dict_sanity(step0_3, p04, p05, p06, p07, p08, p09),
        *[record.to_matcher_dict() for record in command_sanity],
    ]
    features = _collect_expected_png_features(step0_3, p05, p06, p07, p08, p09, commit)
    return StepLoopTrace(
        step=frozen.step,
        t=frozen.t,
        step0_3_result=step0_3,
        aps_result=p04,
        cmc_result=p05,
        p06_result=p06,
        cuc_result=p07,
        p08_result=p08,
        p09_result=p09,
        canonical_command_buffer=command_buffer,
        canonical_next_state_buffer=next_state_buffer,
        commit_result=commit,
        time_advance_result=time_advance,
        actual_events=events,
        actual_sanity_checks=sanity,
        expected_png_features=features,
    )


def build_step_command_buffer(
    p05_result: Step4BCMCRunResult,
    p06_result: Step5CooperativeRequestRunResult,
    p07_result: Step6CUCRunResult,
) -> CommandBuffer:
    state = p05_result.state
    state_transition_commands = _merge_tuple_command_maps(
        p05_result.command_buffer.state_transition_commands,
        p07_result.command_buffer.state_transition_commands,
    )
    return CommandBuffer(
        step=state.step,
        t=state.t,
        longitudinal_commands=MappingProxyType(dict(p05_result.command_buffer.longitudinal_commands)),
        cooperation_commands=MappingProxyType(dict(p07_result.command_buffer.cooperation_commands)),
        lane_change_commands=MappingProxyType(dict(p07_result.command_buffer.lane_change_commands)),
        merge_commands=MappingProxyType(dict(p05_result.command_buffer.merge_commands)),
        speed_cap_commands=MappingProxyType(dict(p05_result.command_buffer.speed_cap_commands)),
        state_transition_commands=MappingProxyType(state_transition_commands),
        cache_update_commands=tuple(p05_result.command_buffer.cache_update_commands),
        same_step_overlays=MappingProxyType(dict(p07_result.command_buffer.same_step_overlays)),
        cuc_decisions=MappingProxyType(dict(p07_result.cuc_decisions)),
    )


def build_step_next_state_buffer(
    p04_result: APSRunResult,
    p05_result: Step4BCMCRunResult,
    p08_result: Step7LongitudinalRunResult,
    p09_result: Step8LateralRunResult,
    command_buffer: CommandBuffer,
) -> NextStateBuffer:
    state = p04_result.state
    cache_updates = (
        *aps_cache_actions_to_candidate_updates(p04_result),
        *command_cache_updates_to_candidate_updates(command_buffer),
    )
    warnings = _candidate_boundary_warnings(state, p05_result, p08_result, p09_result)
    return NextStateBuffer(
        step=state.step,
        t=state.t,
        candidate_longitudinal=MappingProxyType(
            dict(p08_result.next_state_buffer.candidate_longitudinal)
        ),
        candidate_lateral=MappingProxyType(dict(p09_result.next_state_buffer.candidate_lateral)),
        candidate_maneuver_progress=MappingProxyType(
            dict(p09_result.next_state_buffer.candidate_maneuver_progress)
        ),
        candidate_lane_state=MappingProxyType(
            dict(p09_result.next_state_buffer.candidate_lane_state)
        ),
        candidate_state_transitions=MappingProxyType(
            dict(p09_result.next_state_buffer.candidate_state_transitions)
        ),
        candidate_cache_updates=tuple(cache_updates),
        commit_warnings=warnings,
    )


def aps_cache_actions_to_candidate_updates(
    p04_result: APSRunResult,
) -> tuple[CandidateCacheUpdate, ...]:
    updates: list[CandidateCacheUpdate] = []
    for action in p04_result.cache_actions:
        if action.action != "update_request" or action.update_request is None:
            continue
        updates.append(
            CandidateCacheUpdate(
                candidate_id=f"p12:{p04_result.state.step}:aps_cache:{action.mv_id}",
                cache_name="aps_assignment_cache",
                owner_vehicle_id=action.mv_id,
                operation="update",
                new_value=MappingProxyType(dict(action.update_request)),
                reason=action.reason,
            )
        )
    return tuple(updates)


def command_cache_updates_to_candidate_updates(
    command_buffer: CommandBuffer,
) -> tuple[CandidateCacheUpdate, ...]:
    updates: list[CandidateCacheUpdate] = []
    for index, command in enumerate(command_buffer.cache_update_commands):
        if not isinstance(command, Mapping):
            continue
        cache_name = str(command.get("cache_name") or "")
        owner_vehicle_id = str(command.get("owner_vehicle_id") or "")
        operation = str(command.get("operation") or "")
        if not cache_name or not owner_vehicle_id or not operation:
            continue
        updates.append(
            CandidateCacheUpdate(
                candidate_id=(
                    str(command.get("command_id"))
                    if command.get("command_id") is not None
                    else f"p12:{command_buffer.step}:cache_update:{index}"
                ),
                cache_name=cache_name,
                owner_vehicle_id=owner_vehicle_id,
                operation=operation,
                new_value=MappingProxyType(dict(command.get("new_value") or {})),
                reason=str(command.get("reason") or "command_cache_update"),
            )
        )
    return tuple(updates)


def normalize_maneuver_commands(command_buffer: CommandBuffer) -> CommandBuffer:
    lane_change_commands = {
        vehicle_id: _with_planned_length(command)
        for vehicle_id, command in command_buffer.lane_change_commands.items()
    }
    merge_commands = {
        vehicle_id: _with_planned_length(command)
        for vehicle_id, command in command_buffer.merge_commands.items()
    }
    return CommandBuffer(
        step=command_buffer.step,
        t=command_buffer.t,
        longitudinal_commands=command_buffer.longitudinal_commands,
        cooperation_commands=command_buffer.cooperation_commands,
        lane_change_commands=MappingProxyType(lane_change_commands),
        merge_commands=MappingProxyType(merge_commands),
        speed_cap_commands=command_buffer.speed_cap_commands,
        state_transition_commands=command_buffer.state_transition_commands,
        cache_update_commands=command_buffer.cache_update_commands,
        same_step_overlays=command_buffer.same_step_overlays,
        cuc_decisions=command_buffer.cuc_decisions,
    )


def collect_step_history(
    trace: StepLoopTrace,
    *,
    run_id: str,
    scenario_id: str,
) -> OutputHistory:
    history = _pre_commit_history(trace, run_id, scenario_id)
    _extend_history(history, trace.commit_result.history)
    return history


def _run_step0_to_3_from_state(
    state: SimulationState,
    config: Mapping[str, Any],
    *,
    geometry: RoadGeometryConfig = DEFAULT_ROAD_GEOMETRY,
) -> Step0To3LoopResult:
    workspace = _workspace_from_state(state)
    cleanup = step0_cleanup_and_prepare(workspace, geometry=geometry)
    boundary = step1_prefreeze_boundary_generation_hook(workspace, dict(config))
    frozen = freeze_simulation_state(workspace)
    relations = refresh_relations_snapshot(frozen, geometry=geometry)
    events = [
        cleanup,
        boundary,
        emit_freeze_event_candidate(frozen),
        emit_relation_refresh_event_candidate(frozen, relations),
        emit_geometry_event_candidate(frozen, geometry=geometry),
    ]
    sanity = run_geometry_sanity_baseline(frozen, relations, geometry=geometry)
    features = [
        {
            "feature_type": "lane_centerline_quicklook",
            "required": False,
            "vehicle_ids": [],
            "expected_visibility": "optional",
            "notes": "registered by deterministic loop",
        },
        {
            "feature_type": "merging_zone_boundary_quicklook",
            "required": True,
            "vehicle_ids": [],
            "expected_visibility": "visible",
            "notes": "registered by deterministic loop",
        },
    ]
    return Step0To3LoopResult(
        state=frozen,
        relations=relations,
        actual_events=events,
        actual_sanity_checks=sanity,
        expected_png_features=features,
    )


def _workspace_from_state(state: SimulationState) -> PreFreezeWorkspace:
    return PreFreezeWorkspace(
        t=state.t,
        step=state.step,
        dt=state.dt,
        active_vehicle_ids=list(state.active_vehicle_ids),
        vehicle_states={
            vehicle_id: state.vehicle_states[vehicle_id]
            for vehicle_id in state.active_vehicle_ids
        },
        vehicle_specs={
            vehicle_id: state.vehicle_specs[vehicle_id]
            for vehicle_id in state.active_vehicle_ids
            if vehicle_id in state.vehicle_specs
        },
        aps_assignment_cache={
            vehicle_id: dict(value)
            for vehicle_id, value in state.aps_assignment_cache.items()
        },
        active_maneuvers=dict(state.active_maneuvers),
        command_buffer={},
        next_state_buffer={},
        road_config_ref=state.road_config_ref,
        parameter_config_ref=state.parameter_config_ref,
        scenario_config_ref=state.scenario_config_ref,
        output_config_ref=state.output_config_ref,
    )


def _workspace_from_scenario_config(config: Mapping[str, Any]) -> PreFreezeWorkspace:
    loaded = load_scenario_config(dict(config))
    initial_time = loaded.get("initial_time") or {}
    t = float(initial_time.get("t", 0.0))
    step = int(initial_time.get("step", 0))
    dt = float(initial_time.get("dt", 0.1))
    vehicle_states: dict[str, VehicleState] = {}
    vehicle_specs: dict[str, VehicleSpec] = {}
    active_vehicle_ids: list[str] = []
    for item in loaded.get("initial_vehicles", []):
        state = VehicleState(
            vehicle_id=str(item["vehicle_id"]),
            x_global=float(item["initial_x_global"]),
            y=float(item["initial_y"]),
            v=float(item["initial_v"]),
            a=float(item.get("initial_a", 0.0)),
            physical_lane=str(item["physical_lane"]),
            road_role=str(item["road_role"]),
            lane_change_state=str(item.get("lane_change_state", "normal")).lower(),
            merge_state=str(item.get("merge_state", "none")).lower(),
            is_active=True,
        )
        spec_overrides = item.get("spec_overrides") or {}
        spec = VehicleSpec(
            vehicle_id=state.vehicle_id,
            vehicle_type=str(item["vehicle_type"]).lower(),
            compliance_state=_normalize_compliance_state(item.get("compliance_state")),
            desired_speed=_optional_float(spec_overrides.get("desired_speed")),
            desired_time_gap=_optional_float(spec_overrides.get("desired_time_gap")),
            desired_time_gap_class=spec_overrides.get("desired_time_gap_class"),
            assigned_arrival_headway=_optional_float(spec_overrides.get("assigned_arrival_headway")),
            inertial_lag=_optional_float(spec_overrides.get("inertial_lag")),
            length=float(spec_overrides.get("length", 4.0)),
            source_lane_at_generation=state.physical_lane,
            generation_step=step,
            generation_t=t,
        )
        active_vehicle_ids.append(state.vehicle_id)
        vehicle_states[state.vehicle_id] = state
        vehicle_specs[state.vehicle_id] = spec

    return PreFreezeWorkspace(
        t=t,
        step=step,
        dt=dt,
        active_vehicle_ids=active_vehicle_ids,
        vehicle_states=vehicle_states,
        vehicle_specs=vehicle_specs,
        aps_assignment_cache={
            str(item["mv_id"]): dict(item)
            for item in loaded.get("preloaded_assignments", [])
        },
        active_maneuvers=_active_maneuvers_from_config(loaded, step=step, t=t),
        command_buffer={},
        next_state_buffer={},
        road_config_ref=str(loaded.get("road_config_ref") or DEFAULT_ROAD_GEOMETRY.config_id),
        parameter_config_ref=str(loaded.get("parameter_config_ref") or "paper_table_i_first_version"),
        scenario_config_ref=str(loaded["scenario_id"]),
        output_config_ref=(
            str(loaded["output_config_ref"]) if loaded.get("output_config_ref") is not None else None
        ),
    )


def _active_maneuvers_from_config(
    config: Mapping[str, Any],
    *,
    step: int,
    t: float,
) -> dict[str, ManeuverTrajectoryState]:
    active: dict[str, ManeuverTrajectoryState] = {}
    for item in config.get("preloaded_maneuver_trajectory_states", []):
        target_lane = str(item.get("target_lane") or "lane_2")
        target_y = _optional_float(item.get("target_y"))
        if target_y is None:
            target_y = 0.0 if target_lane == "lane_2" else 3.5
        active[str(item["vehicle_id"])] = ManeuverTrajectoryState(
            vehicle_id=str(item["vehicle_id"]),
            maneuver_type=str(item["maneuver_type"]),
            start_step=int(item.get("start_step", step)),
            start_t=float(item.get("start_t", t)),
            start_x_global=float(item["start_x_global"]),
            start_y=float(item["start_y"]),
            target_lane=target_lane,
            target_y=float(target_y),
            planned_length=_optional_float(item.get("planned_length")),
            progress=float(item.get("progress", 0.0)),
            assigned_clv_id=item.get("assigned_clv_id"),
            assigned_cfv_id=item.get("assigned_cfv_id"),
        )
    return active


def _with_planned_length(command: Any) -> Any:
    if not isinstance(command, Mapping):
        return command
    normalized = dict(command)
    if normalized.get("planned_length") in (None, ""):
        normalized["planned_length"] = LANE_CHANGE_CENTERLINE_LENGTH_M
        normalized["planned_length_source"] = LANE_CHANGE_CENTERLINE_LENGTH_SOURCE
        normalized["planned_length_source_status"] = "parameter_spec_first_version"
    return normalized


def _merge_tuple_command_maps(
    left: Mapping[str, Any],
    right: Mapping[str, Any],
) -> dict[str, tuple[Any, ...]]:
    merged: dict[str, list[Any]] = {}
    for source in (left, right):
        for vehicle_id, commands in source.items():
            if isinstance(commands, tuple):
                values = list(commands)
            elif isinstance(commands, list):
                values = commands
            else:
                values = [commands]
            merged.setdefault(vehicle_id, []).extend(values)
    return {vehicle_id: tuple(values) for vehicle_id, values in merged.items()}


def _command_buffer_sanity(
    state: SimulationState,
    command_buffer: CommandBuffer,
    *,
    run_id: str,
    scenario_id: str,
) -> tuple[SanityCheckRecord, ...]:
    records: list[SanityCheckRecord] = []
    maneuver_conflicts = sorted(
        set(command_buffer.merge_commands).intersection(command_buffer.lane_change_commands)
    )
    records.append(
        SanityCheckRecord(
            check_id=f"{run_id}:{state.step}:p12_command_namespace_conflict",
            run_id=run_id,
            scenario_id=scenario_id,
            step=state.step,
            t=state.t,
            check_type="p12_command_namespace_conflict",
            severity="error" if maneuver_conflicts else "info",
            result="fail" if maneuver_conflicts else "pass",
            vehicle_ids=tuple(maneuver_conflicts or state.active_vehicle_ids),
            reason="merge_and_lane_change_commands_are_exclusive",
            payload={
                "conflicting_vehicle_ids": maneuver_conflicts,
                "p06_active_requests_are_trace_only": True,
                "p07_cooperation_commands_are_canonical_for_p08": True,
            },
        )
    )
    records.extend(
        _duplicate_state_transition_sanity(
            state,
            command_buffer,
            run_id=run_id,
            scenario_id=scenario_id,
        )
    )
    return tuple(records)


def _duplicate_state_transition_sanity(
    state: SimulationState,
    command_buffer: CommandBuffer,
    *,
    run_id: str,
    scenario_id: str,
) -> tuple[SanityCheckRecord, ...]:
    conflicts: list[dict[str, Any]] = []
    for vehicle_id, commands in command_buffer.state_transition_commands.items():
        by_name: dict[str, set[str]] = {}
        for command in commands:
            if not isinstance(command, Mapping):
                continue
            state_name = str(command.get("state_name") or "")
            new_state = str(command.get("new_state", command.get("requested_new_state", "")))
            if not state_name:
                continue
            by_name.setdefault(state_name, set()).add(new_state)
        for state_name, new_states in by_name.items():
            if len(new_states) > 1:
                conflicts.append(
                    {
                        "vehicle_id": vehicle_id,
                        "state_name": state_name,
                        "new_states": sorted(new_states),
                    }
                )
    return (
        SanityCheckRecord(
            check_id=f"{run_id}:{state.step}:p12_state_transition_conflict",
            run_id=run_id,
            scenario_id=scenario_id,
            step=state.step,
            t=state.t,
            check_type="p12_state_transition_conflict",
            severity="error" if conflicts else "info",
            result="fail" if conflicts else "pass",
            vehicle_ids=tuple(item["vehicle_id"] for item in conflicts) or tuple(state.active_vehicle_ids),
            reason="same_vehicle_same_state_transition_conflict",
            payload={"conflicts": conflicts},
        ),
    )


def _candidate_boundary_warnings(
    state: SimulationState,
    p05_result: Step4BCMCRunResult,
    p08_result: Step7LongitudinalRunResult,
    p09_result: Step8LateralRunResult,
) -> tuple[CommitWarning, ...]:
    warnings: list[CommitWarning] = []
    p08_vehicle_ids = set(p08_result.next_state_buffer.candidate_longitudinal)
    for vehicle_id in p09_result.next_state_buffer.candidate_lateral:
        if vehicle_id not in p08_vehicle_ids:
            warnings.append(
                CommitWarning(
                    vehicle_id=vehicle_id,
                    warning_type="missing_longitudinal_candidate",
                    reason="p09_lateral_without_p08_longitudinal_candidate",
                    candidate_ids=(
                        p09_result.next_state_buffer.candidate_lateral[vehicle_id].candidate_id,
                    ),
                )
            )
    return tuple(warnings)


def _boundary_risk_diagnostics(
    command_buffer: CommandBuffer,
) -> dict[str, dict[str, Any]]:
    diagnostics: dict[str, dict[str, Any]] = {}
    for vehicle_id, commands in command_buffer.speed_cap_commands.items():
        for command in commands:
            if not isinstance(command, Mapping):
                continue
            if bool(command.get("cap_feasible", True)):
                continue
            diagnostics[vehicle_id] = {
                "risk_status": "cap_infeasible",
                "risk_source_id": command.get("command_id"),
                "cap_reason": command.get("cap_reason"),
                "boundary_speed_cap": command.get("speed_cap"),
            }
    return diagnostics


def _collect_dict_events(*results: Any) -> list[dict[str, Any]]:
    events: list[dict[str, Any]] = []
    for result in results:
        events.extend(dict(event) for event in getattr(result, "actual_events", []))
    return events


def _collect_dict_sanity(*results: Any) -> list[dict[str, Any]]:
    sanity: list[dict[str, Any]] = []
    for result in results:
        sanity.extend(dict(check) for check in getattr(result, "actual_sanity_checks", []))
    return sanity


def _collect_expected_png_features(*results: Any) -> list[dict[str, Any]]:
    features: list[dict[str, Any]] = []
    for result in results:
        features.extend(dict(feature) for feature in getattr(result, "expected_png_features", []))
    return _unique_features(features)


def _pre_commit_history(
    trace: StepLoopTrace,
    run_id: str,
    scenario_id: str,
) -> OutputHistory:
    history = OutputHistory()
    for index, event in enumerate(trace.actual_events):
        history.event_records.append(_event_record_from_dict(event, index, run_id, scenario_id))
    for index, check in enumerate(trace.actual_sanity_checks):
        history.sanity_check_records.append(_sanity_record_from_dict(check, index, run_id, scenario_id))
    return history


def _event_record_from_dict(
    event: Mapping[str, Any],
    index: int,
    run_id: str,
    scenario_id: str,
) -> EventRecord:
    payload = dict(event.get("payload") or {})
    vehicle_id = _optional_str(event.get("vehicle_id"))
    related = tuple(str(item) for item in event.get("related_vehicle_ids") or event.get("vehicle_ids") or ())
    return EventRecord(
        event_id=f"{run_id}:{event.get('step', 'na')}:precommit_event:{index}",
        run_id=run_id,
        scenario_id=str(event.get("scenario_id") or scenario_id),
        step=int(event.get("step", 0)),
        t=float(event.get("t", 0.0)),
        module=str(event.get("module") or event.get("event_type") or "unknown"),
        event_type=str(event.get("event_type") or event.get("module") or "unknown"),
        vehicle_id=vehicle_id,
        related_vehicle_ids=related,
        reason=str(event.get("reason") or ""),
        result=str(event.get("result") or ""),
        is_engineering_patch=bool(event.get("is_engineering_patch", False)),
        source=str(event.get("source") or "paper_algorithm"),
        payload=MappingProxyType(payload),
    )


def _sanity_record_from_dict(
    check: Mapping[str, Any],
    index: int,
    run_id: str,
    scenario_id: str,
) -> SanityCheckRecord:
    result = str(check.get("result") or "not_applicable")
    severity = "error" if result == "fail" else ("warning" if result == "warning" else "info")
    return SanityCheckRecord(
        check_id=f"{run_id}:{check.get('step', 'na')}:precommit_sanity:{index}",
        run_id=run_id,
        scenario_id=str(check.get("scenario_id") or scenario_id),
        step=int(check.get("step", 0)),
        t=float(check.get("t", 0.0)),
        check_type=str(check.get("check_type") or "unknown"),
        severity=severity,
        result=result,
        vehicle_ids=tuple(str(item) for item in check.get("vehicle_ids") or ()),
        reason=str(check.get("reason") or ""),
        payload=MappingProxyType(dict(check.get("payload") or {})),
    )


def _extend_history(target: OutputHistory, source: OutputHistory) -> None:
    target.trajectory_records.extend(source.trajectory_records)
    target.event_records.extend(source.event_records)
    target.sanity_check_records.extend(source.sanity_check_records)
    target.png_artifacts.extend(source.png_artifacts)


def _unique_features(features: Iterable[Mapping[str, Any]]) -> list[dict[str, Any]]:
    unique: dict[tuple[Any, ...], dict[str, Any]] = {}
    for feature in features:
        key = (
            feature.get("feature_type"),
            tuple(feature.get("vehicle_ids") or ()),
            feature.get("expected_visibility"),
        )
        unique.setdefault(key, dict(feature))
    return list(unique.values())


def _stop_conditions_met(
    state: SimulationState,
    stop_conditions: tuple[str | Callable[[SimulationState], bool], ...],
) -> bool:
    for condition in stop_conditions:
        if callable(condition) and condition(state):
            return True
        if isinstance(condition, str) and _string_stop_condition_met(state, condition):
            return True
    return False


def _string_stop_condition_met(state: SimulationState, condition: str) -> bool:
    parts = condition.split(":")
    if len(parts) == 4 and parts[0] == "vehicle_state":
        _, vehicle_id, state_name, expected_value = parts
        vehicle_state = state.vehicle_states.get(vehicle_id)
        return vehicle_state is not None and str(getattr(vehicle_state, state_name)) == expected_value
    if len(parts) == 2 and parts[0] == "merge_completed":
        vehicle_state = state.vehicle_states.get(parts[1])
        return vehicle_state is not None and vehicle_state.merge_state == "merged"
    if len(parts) == 2 and parts[0] == "lane_change_completed":
        vehicle_state = state.vehicle_states.get(parts[1])
        return vehicle_state is not None and vehicle_state.lane_change_state == "normal"
    return False


def _load_loop_scenario_config(config: SimulationLoopConfig) -> dict[str, Any]:
    scenario = config.scenario
    if scenario is None:
        scenario = config.scenario_id
    if scenario is None:
        raise ValueError("SimulationLoopConfig requires scenario or scenario_id")
    if isinstance(scenario, str):
        return load_builtin_scenario(scenario)
    return load_scenario_config(dict(scenario))


def _utility_overrides(config: Mapping[str, Any]) -> Mapping[str, Mapping[str, Any]]:
    module_overrides = config.get("module_overrides") or {}
    harness = module_overrides.get("test_harness_overrides") or {}
    return MappingProxyType(
        {
            str(vehicle_id): MappingProxyType(dict(value))
            for vehicle_id, value in (harness.get("cuc_utility_overrides") or {}).items()
        }
    )


def _normalize_compliance_state(value: Any) -> str:
    lowered = str(value or "not_applicable").lower()
    if lowered == "none":
        return "not_applicable"
    return lowered


def _optional_float(value: Any) -> float | None:
    if value in (None, ""):
        return None
    return float(value)


def _optional_str(value: Any) -> str | None:
    if value in (None, ""):
        return None
    return str(value)
