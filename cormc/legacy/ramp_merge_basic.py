from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Any, Mapping

from cormc.scenes import load_scene_config
from cormc.simulation_core.pre_freeze import SimulationState, build_prefreeze_workspace_from_scenario, freeze_simulation_state
from cormc.simulation_core.commit import OutputHistory

from cormc.onestep.rolling.engine import RampMergeEngine, RampMergeStepResult
from cormc.onestep.rolling.state import (
    RampMergeRuntimeState,
    initialize_runtime_state,
)
from cormc.onestep.rolling.validation import AcceptanceReport, build_acceptance_report


@dataclass(frozen=True)
class RampMergeBasicHistoryRun:
    scenario_id: str
    run_id: str
    max_steps: int
    summary: Mapping[str, Any]
    acceptance_report: AcceptanceReport
    history: OutputHistory
    actual_events: tuple[Mapping[str, Any], ...]
    actual_sanity_checks: tuple[Mapping[str, Any], ...]


@dataclass(frozen=True)
class _RampMergeStepRun:
    config: Mapping[str, Any]
    initial_state: SimulationState
    final_state: SimulationState
    step_results: tuple[RampMergeStepResult, ...]
    history: OutputHistory
    actual_events: tuple[Mapping[str, Any], ...]
    actual_sanity_checks: tuple[Mapping[str, Any], ...]


def build_initial_ramp_merge_state(
    scenario: str | Mapping[str, Any] = "BASIC-01",
) -> tuple[SimulationState, dict[str, Any]]:
    config = load_scene_config(scenario) if isinstance(scenario, str) else dict(scenario)
    workspace, loaded_config = build_prefreeze_workspace_from_scenario(config)
    frozen = freeze_simulation_state(workspace)
    runtime = initialize_runtime_state(frozen)
    return replace(frozen, ramp_merge_runtime=runtime), loaded_config


def run_ramp_merge_basic_smoke(
    scenario_id: str = "BASIC-01",
    max_steps: int = 5,
    run_id: str = "ramp-merge-batch-c-smoke",
) -> dict[str, Any]:
    run = _run_ramp_merge_basic_steps(
        scenario_id=scenario_id,
        max_steps=max_steps,
        run_id=run_id,
    )
    return _build_smoke_summary(run, run_id=run_id)


def run_ramp_merge_basic_acceptance(
    scenario_id: str = "BASIC-04",
    max_steps: int = 65,
    run_id: str = "ramp-merge-basic-acceptance",
) -> dict[str, Any]:
    summary = run_ramp_merge_basic_smoke(
        scenario_id=scenario_id,
        max_steps=max_steps,
        run_id=run_id,
    )
    return {
        "summary": summary,
        "acceptance_report": build_acceptance_report(summary),
    }


def run_ramp_merge_basic_history(
    scenario_id: str = "BASIC-04",
    max_steps: int = 65,
    run_id: str = "ramp-merge-basic-history",
) -> RampMergeBasicHistoryRun:
    run = _run_ramp_merge_basic_steps(
        scenario_id=scenario_id,
        max_steps=max_steps,
        run_id=run_id,
    )
    summary = _build_smoke_summary(run, run_id=run_id)
    return RampMergeBasicHistoryRun(
        scenario_id=str(run.config["scenario_id"]),
        run_id=run_id,
        max_steps=max_steps,
        summary=summary,
        acceptance_report=build_acceptance_report(summary),
        history=run.history,
        actual_events=run.actual_events,
        actual_sanity_checks=run.actual_sanity_checks,
    )


def _run_ramp_merge_basic_steps(
    *,
    scenario_id: str,
    max_steps: int,
    run_id: str,
) -> _RampMergeStepRun:
    state, config = build_initial_ramp_merge_state(scenario_id)
    initial_state = state
    engine = RampMergeEngine(config, run_id=run_id)
    step_results: list[RampMergeStepResult] = []
    history = OutputHistory()
    events: list[Mapping[str, Any]] = []
    sanity_checks: list[Mapping[str, Any]] = []

    for _ in range(max_steps):
        result = engine.advance_one_step(state)
        step_results.append(result)
        events.extend(result.actual_events)
        sanity_checks.extend(result.actual_sanity_checks)
        _extend_history(history, result.commit_result.history)
        state = result.advanced_state

    return _RampMergeStepRun(
        config=config,
        initial_state=initial_state,
        final_state=state,
        step_results=tuple(step_results),
        history=history,
        actual_events=tuple(events),
        actual_sanity_checks=tuple(sanity_checks),
    )


def _build_smoke_summary(
    run: _RampMergeStepRun,
    *,
    run_id: str,
) -> dict[str, Any]:
    runtime = run.final_state.ramp_merge_runtime
    runtime_mv_ids: tuple[str, ...] = ()
    runtime_mv_states: dict[str, dict[str, Any]] = {}
    if isinstance(runtime, RampMergeRuntimeState):
        runtime_mv_ids = tuple(runtime.mv_plan_states)
        runtime_mv_states = _runtime_mv_state_summary(runtime)

    return {
        "scenario_id": str(run.config["scenario_id"]),
        "run_id": run_id,
        "steps_run": len(run.step_results),
        "initial_t": run.initial_state.t,
        "final_t": run.final_state.t,
        "initial_positions": _positions(run.initial_state),
        "final_positions": _positions(run.final_state),
        "final_vehicle_states": _final_vehicle_states(run.final_state),
        "event_types": _unique_event_types(run.actual_events),
        "sanity_results": _sanity_results(run.actual_sanity_checks),
        "runtime_mv_ids": runtime_mv_ids,
        "runtime_mv_states": runtime_mv_states,
        "zone_state_timeline": _zone_state_timeline(run.actual_events),
        "current_plan_gap_timeline": _gap_timeline(run.actual_events, "current_plan_gap_by_mv"),
        "current_plan_gap_state_timeline": _gap_timeline(
            run.actual_events,
            "current_plan_gap_by_mv",
        ),
        "locked_gap_timeline": _gap_timeline(run.actual_events, "locked_gap_by_mv"),
        "locked_gap_state_timeline": _gap_timeline(run.actual_events, "locked_gap_by_mv"),
        "merge_state_timeline": _merge_state_timeline(run.actual_events),
        "trigger_events": _trigger_events(run.actual_events),
        "gap_snapshots": _gap_snapshots(run.actual_events),
        "gap_selection_events": _typed_payload_events(run.actual_events, "ramp_merge_gap_selection"),
        "gap_lock_events": _typed_payload_events(run.actual_events, "ramp_merge_gap_lock"),
        "trajectory_events": _typed_payload_events(run.actual_events, "ramp_merge_trajectory"),
        "merge_check_events": _typed_payload_events(run.actual_events, "ramp_merge_merge_check"),
        "merge_completion_events": _typed_payload_events(
            run.actual_events,
            "ramp_merge_merge_completion",
        ),
        "danger_vehicle_ids_by_step": _danger_vehicle_ids_by_step(run.actual_events),
        "gap_identification_steps": _gap_identification_steps(run.actual_events),
        "non_trigger_gap_event_count": _non_trigger_gap_event_count(run.actual_events),
        "old_assignment_record_count": len(run.final_state.assignment_records_by_mv),
        "old_active_maneuver_count": len(run.final_state.active_maneuvers),
    }


def _extend_history(target: OutputHistory, source: OutputHistory) -> None:
    target.trajectory_records.extend(source.trajectory_records)
    target.event_records.extend(source.event_records)
    target.sanity_check_records.extend(source.sanity_check_records)
    target.png_artifacts.extend(source.png_artifacts)


def _positions(state: SimulationState) -> dict[str, dict[str, float]]:
    return {
        vehicle_id: {
            "x_global": state.vehicle_states[vehicle_id].x_global,
            "y": state.vehicle_states[vehicle_id].y,
            "v": state.vehicle_states[vehicle_id].v,
        }
        for vehicle_id in state.active_vehicle_ids
    }


def _final_vehicle_states(state: SimulationState) -> dict[str, dict[str, Any]]:
    return {
        vehicle_id: {
            "x_global": state.vehicle_states[vehicle_id].x_global,
            "y": state.vehicle_states[vehicle_id].y,
            "v": state.vehicle_states[vehicle_id].v,
            "physical_lane": state.vehicle_states[vehicle_id].physical_lane,
            "road_role": state.vehicle_states[vehicle_id].road_role,
            "merge_state": state.vehicle_states[vehicle_id].merge_state,
        }
        for vehicle_id in state.active_vehicle_ids
    }


def _unique_event_types(events: tuple[Mapping[str, Any], ...]) -> tuple[str, ...]:
    event_types: list[str] = []
    seen: set[str] = set()
    for event in events:
        event_type = str(event.get("event_type"))
        if event_type not in seen:
            seen.add(event_type)
            event_types.append(event_type)
    return tuple(event_types)


def _sanity_results(sanity_checks: tuple[Mapping[str, Any], ...]) -> dict[str, str]:
    results: dict[str, str] = {}
    for sanity in sanity_checks:
        results[str(sanity.get("check_type"))] = str(sanity.get("result"))
    return results


def _runtime_mv_state_summary(runtime: RampMergeRuntimeState) -> dict[str, dict[str, Any]]:
    summary: dict[str, dict[str, Any]] = {}
    trajectories_by_mv: dict[str, Any] = {
        trajectory.mv_id: trajectory
        for trajectory in runtime.planned_trajectories.values()
    }
    for vehicle_id, state in runtime.mv_plan_states.items():
        summary[vehicle_id] = {
            "zone_state": state.zone_state,
            "merge_state": state.merge_state,
            "current_plan_gap": _gap_ref_payload(state.current_plan_gap),
            "locked_gap": _gap_ref_payload(state.locked_gap),
            "planned_trajectory_id": state.planned_trajectory_id,
            "last_plan_step": state.last_plan_step,
            "last_plan_t": state.last_plan_t,
            "planned_trajectory": _trajectory_payload(trajectories_by_mv.get(vehicle_id)),
        }
    return summary


def _zone_state_timeline(events: tuple[Mapping[str, Any], ...]) -> list[dict[str, Any]]:
    return [
        {
            "step": event["step"],
            "t": event["t"],
            "zone_state_by_mv": dict(event.get("payload", {}).get("zone_state_by_mv", {})),
        }
        for event in events
        if event.get("event_type") == "ramp_merge_zone_state"
    ]


def _gap_timeline(events: tuple[Mapping[str, Any], ...], payload_key: str) -> list[dict[str, Any]]:
    return [
        {
            "step": event["step"],
            "t": event["t"],
            payload_key: dict(event.get("payload", {}).get(payload_key, {})),
        }
        for event in events
        if event.get("event_type") == "ramp_merge_zone_state"
    ]


def _merge_state_timeline(events: tuple[Mapping[str, Any], ...]) -> list[dict[str, Any]]:
    return [
        {
            "step": event["step"],
            "t": event["t"],
            "merge_state_by_mv": dict(
                event.get("payload", {}).get("merge_state_by_mv", {})
            ),
        }
        for event in events
        if event.get("event_type") == "ramp_merge_zone_state"
    ]


def _trigger_events(events: tuple[Mapping[str, Any], ...]) -> list[dict[str, Any]]:
    return [
        {
            "step": event["step"],
            "t": event["t"],
            **dict(event.get("payload", {})),
        }
        for event in events
        if event.get("event_type") == "ramp_merge_trigger"
    ]


def _typed_payload_events(
    events: tuple[Mapping[str, Any], ...],
    event_type: str,
) -> list[dict[str, Any]]:
    return [
        {
            "step": event["step"],
            "t": event["t"],
            **dict(event.get("payload", {})),
        }
        for event in events
        if event.get("event_type") == event_type
    ]


def _gap_snapshots(events: tuple[Mapping[str, Any], ...]) -> list[dict[str, Any]]:
    return [
        {
            "step": event["step"],
            "t": event["t"],
            **dict(event.get("payload", {})),
        }
        for event in events
        if event.get("event_type") == "ramp_merge_gap_snapshot"
    ]


def _danger_vehicle_ids_by_step(events: tuple[Mapping[str, Any], ...]) -> dict[int, tuple[str, ...]]:
    result: dict[int, tuple[str, ...]] = {}
    for event in events:
        if event.get("event_type") != "ramp_merge_safety":
            continue
        result[int(event["step"])] = tuple(
            event.get("payload", {}).get("danger_vehicle_ids", ())
        )
    return result


def _gap_identification_steps(events: tuple[Mapping[str, Any], ...]) -> tuple[int, ...]:
    return tuple(
        int(event["step"])
        for event in events
        if event.get("event_type") == "ramp_merge_trigger"
        and event.get("payload", {}).get("gap_identification_executed") is True
    )


def _non_trigger_gap_event_count(events: tuple[Mapping[str, Any], ...]) -> int:
    trigger_steps = {
        int(event["step"])
        for event in events
        if event.get("event_type") == "ramp_merge_trigger"
        and event.get("payload", {}).get("trigger_plan") is True
    }
    return sum(
        1
        for event in events
        if event.get("event_type") == "ramp_merge_gap_snapshot"
        and int(event["step"]) not in trigger_steps
    )


def _gap_ref_payload(gap_ref: Any) -> dict[str, Any] | None:
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


def _trajectory_payload(trajectory: Any) -> dict[str, Any] | None:
    if trajectory is None:
        return None
    return {
        "trajectory_id": trajectory.trajectory_id,
        "mv_id": trajectory.mv_id,
        "kind": trajectory.kind,
        "target_gap": _gap_ref_payload(trajectory.target_gap),
        "start_step": trajectory.start_step,
        "start_t": trajectory.start_t,
        "start_x_global": trajectory.start_x_global,
        "start_y": trajectory.start_y,
        "target_y": trajectory.target_y,
        "duration_steps": trajectory.duration_steps,
        "progress_step": trajectory.progress_step,
    }
