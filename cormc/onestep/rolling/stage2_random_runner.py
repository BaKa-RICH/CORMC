from __future__ import annotations

from collections import Counter
from dataclasses import dataclass, replace
from typing import Any, Mapping

from cormc.traffic_flow.generation import BoundaryQueueItem, queue_fingerprint
from cormc.scenes import (
    DEFAULT_ONESTEP_RANDOM_MAX_STEPS,
    DEFAULT_ONESTEP_RANDOM_SEED,
    RM_ONESTEP_RANDOM_S07_LANE2_RAMP_100S_SCENARIO_ID,
    TrafficFlowSceneSpec,
    compile_traffic_flow_scene,
    get_traffic_flow_scene_spec,
)
from cormc.scenes.onestep import RM_ONESTEP_CASE_SPECS
from cormc.simulation_core.pre_freeze import (
    ON_RAMP,
    ON_RAMP_MV_ROLE,
    SimulationState,
    build_prefreeze_workspace_from_scenario,
    freeze_simulation_state,
)
from cormc.simulation_core.commit import OutputHistory

from cormc.onestep.rolling.engine import RampMergeEngine, RampMergeStepResult
from cormc.onestep.rolling.stage2_runner import (
    _OneStepStage2StepRun,
    _build_stage2_summary_from_run,
    _extend_history,
)
from cormc.onestep.rolling.state import RampMergeRuntimeState, initialize_runtime_state


@dataclass(frozen=True)
class OneStepStage2RandomHistoryRun:
    scenario_id: str
    run_id: str
    max_steps: int
    horizon_s: float
    seed: int
    profile_id: str
    boundary_queue: tuple[BoundaryQueueItem, ...]
    boundary_flow_summary: Mapping[str, Any]
    summary: Mapping[str, Any]
    history: OutputHistory
    actual_events: tuple[Mapping[str, Any], ...]
    actual_sanity_checks: tuple[Mapping[str, Any], ...]


def build_initial_onestep_stage2_random_state(
    scenario_id: str = RM_ONESTEP_RANDOM_S07_LANE2_RAMP_100S_SCENARIO_ID,
) -> tuple[SimulationState, dict[str, Any]]:
    spec = get_traffic_flow_scene_spec(scenario_id)
    return _build_initial_state_from_spec(spec)


def run_onestep_stage2_random_history(
    spec_or_scenario_id: TrafficFlowSceneSpec | str = RM_ONESTEP_RANDOM_S07_LANE2_RAMP_100S_SCENARIO_ID,
    *,
    max_steps: int | None = None,
    run_id: str = "onestep-stage2-random-history",
) -> OneStepStage2RandomHistoryRun:
    spec = _resolve_spec(spec_or_scenario_id)
    resolved_max_steps = int(
        max_steps if max_steps is not None else spec.stop_condition.max_steps
    )
    horizon_s = float(spec.stop_condition.horizon_s)
    queue = spec.boundary_flow_source.build_queue(horizon_s)
    run = _run_onestep_stage2_random_steps(
        spec=spec,
        boundary_queue=queue,
        max_steps=resolved_max_steps,
        run_id=run_id,
    )
    boundary_flow_summary = spec.boundary_flow_source.to_summary()
    metadata = _build_random_scenario_metadata(
        spec,
        run,
        queue,
        boundary_flow_summary,
        horizon_s=horizon_s,
    )
    summary = _build_stage2_summary_from_run(
        run,
        run_id=run_id,
        max_steps=resolved_max_steps,
        case_spec=RM_ONESTEP_CASE_SPECS["S07"],
        scenario_metadata=metadata,
    )
    return OneStepStage2RandomHistoryRun(
        scenario_id=str(run.config["scenario_id"]),
        run_id=run_id,
        max_steps=resolved_max_steps,
        horizon_s=horizon_s,
        seed=int(boundary_flow_summary.get("seed", DEFAULT_ONESTEP_RANDOM_SEED)),
        profile_id=str(boundary_flow_summary.get("profile_id") or "unknown_profile"),
        boundary_queue=queue,
        boundary_flow_summary=boundary_flow_summary,
        summary=summary,
        history=run.history,
        actual_events=run.actual_events,
        actual_sanity_checks=run.actual_sanity_checks,
    )


def run_onestep_stage2_random_summary(
    spec_or_scenario_id: TrafficFlowSceneSpec | str = RM_ONESTEP_RANDOM_S07_LANE2_RAMP_100S_SCENARIO_ID,
    *,
    max_steps: int | None = None,
    run_id: str = "onestep-stage2-random-summary",
) -> dict[str, Any]:
    result = run_onestep_stage2_random_history(
        spec_or_scenario_id,
        max_steps=max_steps,
        run_id=run_id,
    )
    return dict(result.summary)


def _resolve_spec(
    spec_or_scenario_id: TrafficFlowSceneSpec | str,
) -> TrafficFlowSceneSpec:
    if isinstance(spec_or_scenario_id, str):
        return get_traffic_flow_scene_spec(spec_or_scenario_id)
    return spec_or_scenario_id


def _build_initial_state_from_spec(
    spec: TrafficFlowSceneSpec,
) -> tuple[SimulationState, dict[str, Any]]:
    config = compile_traffic_flow_scene(spec)
    workspace, loaded_config = build_prefreeze_workspace_from_scenario(config)
    frozen = freeze_simulation_state(workspace)
    runtime = initialize_runtime_state(frozen)
    return replace(frozen, ramp_merge_runtime=runtime), loaded_config


def _run_onestep_stage2_random_steps(
    *,
    spec: TrafficFlowSceneSpec,
    boundary_queue: tuple[BoundaryQueueItem, ...],
    max_steps: int,
    run_id: str,
) -> _OneStepStage2StepRun:
    state, config = _build_initial_state_from_spec(spec)
    initial_state = state
    engine = RampMergeEngine(
        config,
        run_id=run_id,
        algorithm_variant="onestep_stage2",
        random_queue=boundary_queue,
        safe_spawn_gap_m=spec.safe_spawn_gap_m,
    )
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

    return _OneStepStage2StepRun(
        config=config,
        initial_state=initial_state,
        final_state=state,
        step_results=tuple(step_results),
        history=history,
        actual_events=tuple(events),
        actual_sanity_checks=tuple(sanity_checks),
    )


def _build_random_scenario_metadata(
    spec: TrafficFlowSceneSpec,
    run: _OneStepStage2StepRun,
    queue: tuple[BoundaryQueueItem, ...],
    boundary_flow_summary: Mapping[str, Any],
    *,
    horizon_s: float,
) -> dict[str, Any]:
    generated, blocked = _boundary_generation_stats(run.actual_events)
    generated_on_ramp_mv_ids = tuple(
        vehicle_id
        for vehicle_id in generated
        if _is_on_ramp_mv(vehicle_id, run.final_state)
        or _generated_lane(run.actual_events, vehicle_id) == ON_RAMP
    )
    completed_mv_ids, open_mv_ids = _classify_random_mv_lifecycle(run)
    validation = spec.validation
    return {
        "traffic_mode": "boundary_flow",
        "boundary_flow_source": dict(boundary_flow_summary),
        "seed": boundary_flow_summary.get("seed"),
        "profile_id": boundary_flow_summary.get("profile_id"),
        "horizon_s": float(horizon_s),
        "safe_spawn_gap_m": float(spec.safe_spawn_gap_m),
        "boundary_queue_size": len(queue),
        "boundary_queue_fingerprint": [list(item) for item in queue_fingerprint(queue)],
        "generated_vehicle_count": len(generated),
        "blocked_spawn_count": len(blocked),
        "generated_by_lane": dict(sorted(Counter(generated.values()).items())),
        "generated_on_ramp_mv_count": len(generated_on_ramp_mv_ids),
        "generated_on_ramp_mv_ids": list(generated_on_ramp_mv_ids),
        "blocked_spawn_vehicle_ids": list(blocked),
        "completed_mv_count": len(completed_mv_ids),
        "completed_mv_ids": list(completed_mv_ids),
        "open_mv_count_at_horizon": len(open_mv_ids),
        "open_mv_ids_at_horizon": list(open_mv_ids),
        "flow_stop_condition": {
            "mode": spec.stop_condition.mode,
            "max_steps": int(spec.stop_condition.max_steps),
            "horizon_s": float(spec.stop_condition.horizon_s),
        },
        "flow_validation": {
            "min_generated_lane2_count": int(validation.min_generated_lane2_count),
            "min_generated_on_ramp_mv_count": int(
                validation.min_generated_on_ramp_mv_count
            ),
            "min_completed_mv_count": int(validation.min_completed_mv_count),
            "allow_open_mvs_at_horizon": bool(validation.allow_open_mvs_at_horizon),
        },
    }


def _boundary_generation_stats(
    events: tuple[Mapping[str, Any], ...],
) -> tuple[dict[str, str], dict[str, str]]:
    generated: dict[str, str] = {}
    blocked: dict[str, str] = {}
    for event in events:
        if event.get("event_type") != "boundary_generation":
            continue
        payload = dict(event.get("payload") or {})
        lane_by_id = dict(payload.get("lane_id") or payload.get("lane_ids") or {})
        for vehicle_id in payload.get("generated_vehicle_ids") or []:
            generated[str(vehicle_id)] = str(lane_by_id.get(str(vehicle_id)) or "")
        for vehicle_id in payload.get("blocked_spawn_vehicle_ids") or []:
            blocked[str(vehicle_id)] = str(lane_by_id.get(str(vehicle_id)) or "")
    return generated, blocked


def _generated_lane(events: tuple[Mapping[str, Any], ...], vehicle_id: str) -> str | None:
    for event in events:
        if event.get("event_type") != "boundary_generation":
            continue
        payload = dict(event.get("payload") or {})
        lane_by_id = dict(payload.get("lane_id") or payload.get("lane_ids") or {})
        if vehicle_id in lane_by_id:
            return str(lane_by_id[vehicle_id])
    return None


def _classify_random_mv_lifecycle(
    run: _OneStepStage2StepRun,
) -> tuple[tuple[str, ...], tuple[str, ...]]:
    completed: set[str] = set()
    open_mvs: set[str] = set()
    final_runtime = run.final_state.ramp_merge_runtime
    runtime_mv_ids = (
        set(str(mv_id) for mv_id in final_runtime.mv_plan_states)
        if isinstance(final_runtime, RampMergeRuntimeState)
        else set()
    )
    final_states = {
        vehicle_id: run.final_state.vehicle_states[vehicle_id]
        for vehicle_id in run.final_state.active_vehicle_ids
    }
    all_mv_ids: set[str] = set(runtime_mv_ids)
    for vehicle_id, state in final_states.items():
        if state.road_role == ON_RAMP_MV_ROLE or state.physical_lane == ON_RAMP:
            all_mv_ids.add(str(vehicle_id))
    for event in run.actual_events:
        if event.get("event_type") == "ramp_merge_merge_completion":
            payload = dict(event.get("payload") or {})
            if payload.get("mv_id") is not None:
                all_mv_ids.add(str(payload["mv_id"]))
        if event.get("event_type") == "cleanup":
            payload = dict(event.get("payload") or {})
            for vehicle_id in payload.get("mainline_converted_vehicle_ids") or []:
                all_mv_ids.add(str(vehicle_id))
                completed.add(str(vehicle_id))
        if event.get("event_type") == "boundary_generation":
            payload = dict(event.get("payload") or {})
            lane_by_id = dict(payload.get("lane_id") or payload.get("lane_ids") or {})
            for vehicle_id in payload.get("generated_vehicle_ids") or []:
                if lane_by_id.get(str(vehicle_id)) == ON_RAMP:
                    all_mv_ids.add(str(vehicle_id))
    for vehicle_id in all_mv_ids:
        state = final_states.get(vehicle_id)
        if (
            vehicle_id in completed
            or state is not None
            and state.physical_lane == "lane_2"
            and state.road_role == "mainline"
            and state.merge_state == "normal"
            and vehicle_id not in runtime_mv_ids
        ):
            completed.add(vehicle_id)
        else:
            open_mvs.add(vehicle_id)
    return tuple(sorted(completed)), tuple(sorted(open_mvs - completed))


def _is_on_ramp_mv(vehicle_id: str, state: SimulationState) -> bool:
    vehicle_state = state.vehicle_states.get(vehicle_id)
    if vehicle_state is None:
        return False
    return (
        vehicle_state.road_role == ON_RAMP_MV_ROLE
        or vehicle_state.physical_lane == ON_RAMP
    )
