from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Any, Mapping

from cormc.scenes import (
    RM_ONESTEP_STAGE1_DEFAULT_MAX_STEPS,
    get_ramp_merge_onestep_case_spec,
    get_ramp_merge_onestep_stage1_expectation,
    load_scene_config,
)
from cormc.simulation_core.pre_freeze import LANE_2, SimulationState, build_prefreeze_workspace_from_scenario, freeze_simulation_state
from cormc.simulation_core.commit import OutputHistory

from cormc.onestep.rolling.engine import RampMergeEngine, RampMergeStepResult
from cormc.onestep.rolling.state import RampMergeRuntimeState, initialize_runtime_state


@dataclass(frozen=True)
class OneStepStage1HistoryRun:
    scenario_id: str
    run_id: str
    max_steps: int
    summary: Mapping[str, Any]
    history: OutputHistory
    actual_events: tuple[Mapping[str, Any], ...]
    actual_sanity_checks: tuple[Mapping[str, Any], ...]


@dataclass(frozen=True)
class _OneStepStage1StepRun:
    config: Mapping[str, Any]
    initial_state: SimulationState
    final_state: SimulationState
    step_results: tuple[RampMergeStepResult, ...]
    history: OutputHistory
    actual_events: tuple[Mapping[str, Any], ...]
    actual_sanity_checks: tuple[Mapping[str, Any], ...]


def build_initial_onestep_stage1_state(
    scenario_id: str,
) -> tuple[SimulationState, dict[str, Any]]:
    config = load_scene_config(scenario_id)
    workspace, loaded_config = build_prefreeze_workspace_from_scenario(config)
    frozen = freeze_simulation_state(workspace)
    runtime = initialize_runtime_state(frozen)
    return replace(frozen, ramp_merge_runtime=runtime), loaded_config


def run_onestep_stage1_history(
    scenario_id: str,
    max_steps: int | None = None,
    run_id: str = "onestep-stage1-history",
) -> OneStepStage1HistoryRun:
    resolved_max_steps = _resolve_max_steps(scenario_id, max_steps)
    run = _run_onestep_stage1_steps(
        scenario_id=scenario_id,
        max_steps=resolved_max_steps,
        run_id=run_id,
    )
    summary = _build_stage1_summary(
        run,
        run_id=run_id,
        max_steps=resolved_max_steps,
    )
    return OneStepStage1HistoryRun(
        scenario_id=str(run.config["scenario_id"]),
        run_id=run_id,
        max_steps=resolved_max_steps,
        summary=summary,
        history=run.history,
        actual_events=run.actual_events,
        actual_sanity_checks=run.actual_sanity_checks,
    )


def run_onestep_stage1_summary(
    scenario_id: str,
    max_steps: int | None = None,
    run_id: str = "onestep-stage1-summary",
) -> dict[str, Any]:
    result = run_onestep_stage1_history(
        scenario_id=scenario_id,
        max_steps=max_steps,
        run_id=run_id,
    )
    return dict(result.summary)


def _resolve_max_steps(scenario_id: str, max_steps: int | None) -> int:
    if max_steps is not None:
        return int(max_steps)
    try:
        return int(RM_ONESTEP_STAGE1_DEFAULT_MAX_STEPS[scenario_id])
    except KeyError as exc:
        raise ValueError(f"unknown RM-ONESTEP scenario_id: {scenario_id}") from exc


def _run_onestep_stage1_steps(
    *,
    scenario_id: str,
    max_steps: int,
    run_id: str,
) -> _OneStepStage1StepRun:
    state, config = build_initial_onestep_stage1_state(scenario_id)
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

    return _OneStepStage1StepRun(
        config=config,
        initial_state=initial_state,
        final_state=state,
        step_results=tuple(step_results),
        history=history,
        actual_events=tuple(events),
        actual_sanity_checks=tuple(sanity_checks),
    )


def _build_stage1_summary(
    run: _OneStepStage1StepRun,
    *,
    run_id: str,
    max_steps: int,
) -> dict[str, Any]:
    scenario_id = str(run.config["scenario_id"])
    spec = get_ramp_merge_onestep_case_spec(scenario_id)
    mv_id = spec.mv_id
    expectation = get_ramp_merge_onestep_stage1_expectation(scenario_id)
    if expectation.expected_first_check_step >= len(run.step_results):
        raise ValueError(
            "stage-1 run did not advance far enough to reach the expected first-check step"
        )

    first_check_result = run.step_results[expectation.expected_first_check_step]
    first_check_state = first_check_result.frozen_state
    first_check_trigger_event = _event_at_step(
        run.actual_events,
        event_type="ramp_merge_trigger",
        step=expectation.expected_first_check_step,
    )
    first_check_gap_snapshot = _event_at_step(
        run.actual_events,
        event_type="ramp_merge_gap_snapshot",
        step=expectation.expected_first_check_step,
    )
    first_check_mv_state = _first_check_mv_state(
        first_check_state,
        first_check_result.ramp_merge_runtime,
        mv_id,
    )
    first_check_mv_local_frame = _first_check_mv_local_frame(first_check_state, mv_id)
    initial_runtime = run.initial_state.ramp_merge_runtime
    initial_zone_state = None
    if isinstance(initial_runtime, RampMergeRuntimeState):
        mv_state = initial_runtime.mv_plan_states.get(mv_id)
        initial_zone_state = mv_state.zone_state if mv_state is not None else None

    return {
        "scenario_id": scenario_id,
        "case_spec": spec.to_dict(),
        "mv_id": mv_id,
        "mode": expectation.mode,
        "run_id": run_id,
        "max_steps": max_steps,
        "actual_steps": len(run.step_results),
        "dt": run.initial_state.dt,
        "stage1_expectation": expectation.to_dict(),
        "initial_zone_state": initial_zone_state,
        "initial_vehicle_table": _initial_vehicle_table(run.initial_state),
        "zone_state_timeline": _zone_state_timeline(run.actual_events),
        "trigger_events": _typed_payload_events(run.actual_events, "ramp_merge_trigger"),
        "gap_snapshots": _typed_payload_events(run.actual_events, "ramp_merge_gap_snapshot"),
        "first_check_step": expectation.expected_first_check_step,
        "first_check_t": first_check_state.t,
        "first_check_trigger_event": first_check_trigger_event,
        "first_check_gap_snapshot": first_check_gap_snapshot,
        "first_check_mv_state": first_check_mv_state,
        "first_check_mv_local_frame": first_check_mv_local_frame,
        "final_vehicle_states": _final_vehicle_states(run.final_state),
    }


def _extend_history(target: OutputHistory, source: OutputHistory) -> None:
    target.trajectory_records.extend(source.trajectory_records)
    target.event_records.extend(source.event_records)
    target.sanity_check_records.extend(source.sanity_check_records)
    target.png_artifacts.extend(source.png_artifacts)


def _typed_payload_events(
    events: tuple[Mapping[str, Any], ...],
    event_type: str,
) -> list[dict[str, Any]]:
    return [
        {
            "step": int(event["step"]),
            "t": float(event["t"]),
            **dict(event.get("payload", {})),
        }
        for event in events
        if event.get("event_type") == event_type
    ]


def _zone_state_timeline(events: tuple[Mapping[str, Any], ...]) -> list[dict[str, Any]]:
    return [
        {
            "step": int(event["step"]),
            "t": float(event["t"]),
            "zone_state_by_mv": dict(event.get("payload", {}).get("zone_state_by_mv", {})),
        }
        for event in events
        if event.get("event_type") == "ramp_merge_zone_state"
    ]


def _event_at_step(
    events: tuple[Mapping[str, Any], ...],
    *,
    event_type: str,
    step: int,
) -> dict[str, Any] | None:
    for event in events:
        if event.get("event_type") != event_type or int(event["step"]) != step:
            continue
        return {
            "step": int(event["step"]),
            "t": float(event["t"]),
            **dict(event.get("payload", {})),
        }
    return None


def _initial_vehicle_table(state: SimulationState) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for vehicle_id in sorted(
        state.active_vehicle_ids,
        key=lambda item: (state.vehicle_states[item].x_global, item),
    ):
        vehicle = state.vehicle_states[vehicle_id]
        rows.append(
            {
                "vehicle_id": vehicle_id,
                "x_global": vehicle.x_global,
                "y": vehicle.y,
                "v": vehicle.v,
                "physical_lane": vehicle.physical_lane,
                "road_role": vehicle.road_role,
            }
        )
    return rows


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


def _first_check_mv_state(
    state: SimulationState,
    runtime: RampMergeRuntimeState,
    mv_id: str,
) -> dict[str, Any]:
    vehicle_state = state.vehicle_states[mv_id]
    mv_runtime = runtime.mv_plan_states[mv_id]
    return {
        "x_global": vehicle_state.x_global,
        "y": vehicle_state.y,
        "v": vehicle_state.v,
        "physical_lane": vehicle_state.physical_lane,
        "road_role": vehicle_state.road_role,
        "zone_state": mv_runtime.zone_state,
        "merge_state": mv_runtime.merge_state,
    }


def _first_check_mv_local_frame(state: SimulationState, mv_id: str) -> dict[str, Any]:
    mv_state = state.vehicle_states[mv_id]
    origin_x_global = mv_state.x_global
    lane_2_vehicle_order = tuple(
        sorted(
            (
                vehicle_id
                for vehicle_id in state.active_vehicle_ids
                if state.vehicle_states[vehicle_id].is_active
                and state.vehicle_states[vehicle_id].physical_lane == LANE_2
            ),
            key=lambda vehicle_id: (
                state.vehicle_states[vehicle_id].x_global,
                vehicle_id,
            ),
        )
    )
    lane_2_vehicle_x_local_by_id = {
        vehicle_id: state.vehicle_states[vehicle_id].x_global - origin_x_global
        for vehicle_id in lane_2_vehicle_order
    }
    gap_intervals_local = [
        [
            lane_2_vehicle_x_local_by_id[rear_id],
            lane_2_vehicle_x_local_by_id[front_id],
        ]
        for rear_id, front_id in zip(lane_2_vehicle_order, lane_2_vehicle_order[1:])
    ]
    gap_centers_local = [
        (interval[0] + interval[1]) / 2.0
        for interval in gap_intervals_local
    ]
    return {
        "mv_id": mv_id,
        "origin_x_global": origin_x_global,
        "x_m0_local": 0.0,
        "lane_2_vehicle_order": list(lane_2_vehicle_order),
        "lane_2_vehicle_x_local_by_id": lane_2_vehicle_x_local_by_id,
        "gap_intervals_local": gap_intervals_local,
        "gap_centers_local": gap_centers_local,
    }
