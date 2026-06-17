from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Any, Mapping

from cormc.scenes import (
    RM_ONESTEP_STAGE2_DEFAULT_MAX_STEPS,
    get_ramp_merge_onestep_case_spec,
    load_scene_config,
)
from cormc.scenes.multimv import (
    RM_MULTIMV_ORIGIN_X_GLOBAL,
    RM_MULTIMV_SCENARIO_IDS,
    get_multimv_case_spec,
    multimv_mv_vehicle_ids,
)
from cormc.scenes.onestep import OneStepRampMergeCaseSpec
from cormc.simulation_core.pre_freeze import SimulationState, build_prefreeze_workspace_from_scenario, freeze_simulation_state
from cormc.simulation_core.commit import OutputHistory

from cormc.onestep.rolling.engine import RampMergeEngine, RampMergeStepResult
from cormc.onestep.rolling.gaps import identify_and_number_gaps
from cormc.onestep.rolling.stage2_adapter import build_stage2_local_frame_from_snapshot
from cormc.onestep.rolling.state import RampMergeRuntimeState, initialize_runtime_state


@dataclass(frozen=True)
class OneStepStage2HistoryRun:
    scenario_id: str
    run_id: str
    max_steps: int
    summary: Mapping[str, Any]
    history: OutputHistory
    actual_events: tuple[Mapping[str, Any], ...]
    actual_sanity_checks: tuple[Mapping[str, Any], ...]


@dataclass(frozen=True)
class _OneStepStage2StepRun:
    config: Mapping[str, Any]
    initial_state: SimulationState
    final_state: SimulationState
    step_results: tuple[RampMergeStepResult, ...]
    history: OutputHistory
    actual_events: tuple[Mapping[str, Any], ...]
    actual_sanity_checks: tuple[Mapping[str, Any], ...]


def build_initial_onestep_stage2_state(
    scenario_id: str,
) -> tuple[SimulationState, dict[str, Any]]:
    config = load_scene_config(scenario_id)
    workspace, loaded_config = build_prefreeze_workspace_from_scenario(config)
    frozen = freeze_simulation_state(workspace)
    runtime = initialize_runtime_state(frozen)
    return replace(frozen, ramp_merge_runtime=runtime), loaded_config


def run_onestep_stage2_history(
    scenario_id: str,
    max_steps: int | None = None,
    run_id: str = "onestep-stage2-history",
) -> OneStepStage2HistoryRun:
    resolved_max_steps = _resolve_max_steps(scenario_id, max_steps)
    run = _run_onestep_stage2_steps(
        scenario_id=scenario_id,
        max_steps=resolved_max_steps,
        run_id=run_id,
    )
    summary = _build_stage2_summary(
        run,
        run_id=run_id,
        max_steps=resolved_max_steps,
    )
    return OneStepStage2HistoryRun(
        scenario_id=str(run.config["scenario_id"]),
        run_id=run_id,
        max_steps=resolved_max_steps,
        summary=summary,
        history=run.history,
        actual_events=run.actual_events,
        actual_sanity_checks=run.actual_sanity_checks,
    )


def run_onestep_stage2_summary(
    scenario_id: str,
    max_steps: int | None = None,
    run_id: str = "onestep-stage2-summary",
) -> dict[str, Any]:
    result = run_onestep_stage2_history(
        scenario_id=scenario_id,
        max_steps=max_steps,
        run_id=run_id,
    )
    return dict(result.summary)


def _resolve_max_steps(scenario_id: str, max_steps: int | None) -> int:
    if max_steps is not None:
        return int(max_steps)
    if scenario_id in RM_MULTIMV_SCENARIO_IDS:
        case = get_multimv_case_spec(scenario_id)
        return 420 + 180 * (case.mv_count - 1)
    try:
        return int(RM_ONESTEP_STAGE2_DEFAULT_MAX_STEPS[scenario_id])
    except KeyError as exc:
        raise ValueError(f"unknown RM-ONESTEP scenario_id: {scenario_id}") from exc


def _run_onestep_stage2_steps(
    *,
    scenario_id: str,
    max_steps: int,
    run_id: str,
) -> _OneStepStage2StepRun:
    state, config = build_initial_onestep_stage2_state(scenario_id)
    initial_state = state
    engine = RampMergeEngine(
        config,
        run_id=run_id,
        algorithm_variant="onestep_stage2",
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


def _build_stage2_summary(
    run: _OneStepStage2StepRun,
    *,
    run_id: str,
    max_steps: int,
    case_spec: OneStepRampMergeCaseSpec | None = None,
    scenario_metadata: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    scenario_id = str(run.config["scenario_id"])
    spec = case_spec
    if spec is None and scenario_id not in RM_MULTIMV_SCENARIO_IDS:
        spec = get_ramp_merge_onestep_case_spec(scenario_id)
    mv_id = _primary_mv_id(scenario_id, spec)
    trigger_plan_summaries = _typed_payload_events(
        run.actual_events,
        "ramp_merge_onestep_stage2_plan",
    )
    first_trigger = trigger_plan_summaries[0] if trigger_plan_summaries else None
    first_trigger_gap_rows = _typed_payload_events(
        run.actual_events,
        "ramp_merge_onestep_stage2_gap_eval",
    )
    if first_trigger is not None:
        bundle_id = first_trigger["bundle_id"]
        first_trigger_gap_rows = [
            row for row in first_trigger_gap_rows if row.get("bundle_id") == bundle_id
        ]
    trigger_round_plan_summaries = trigger_plan_summaries
    trigger_round_gap_rows = _typed_payload_events(
        run.actual_events,
        "ramp_merge_onestep_stage2_gap_eval",
    )
    longitudinal_completion_event = _first_event(
        run.actual_events,
        "ramp_merge_onestep_longitudinal_completion",
    )
    gap_lock_summaries = _typed_payload_events(
        run.actual_events,
        "ramp_merge_gap_lock",
    )
    lateral_start_summaries = [
        record
        for record in _typed_payload_events(
            run.actual_events,
            "ramp_merge_onestep_longitudinal_completion",
        )
        if record.get("lateral_started") is True
    ]
    lateral_progress_summaries = [
        record
        for record in _typed_payload_events(
            run.actual_events,
            "ramp_merge_trajectory",
        )
        if record.get("reason") == "onestep_stage2_lateral_progress"
    ]
    merge_completion_summaries = _typed_payload_events(
        run.actual_events,
        "ramp_merge_merge_completion",
    )
    step0_mainline_conversion_summaries = [
        record
        for record in _typed_payload_events(run.actual_events, "cleanup")
        if record.get("mainline_converted_vehicle_ids")
    ]
    bundle_lifecycle_summaries = _typed_payload_events(
        run.actual_events,
        "ramp_merge_onestep_stage2_bundle_lifecycle",
    )
    bundle_created_summaries = [
        record
        for record in bundle_lifecycle_summaries
        if record.get("bundle_action") == "bundle_created"
    ]
    bundle_released_summaries = [
        record
        for record in bundle_lifecycle_summaries
        if record.get("bundle_action") == "bundle_released"
    ]
    mv_lifecycle_summaries = _mv_lifecycle_summaries(
        run,
        gap_lock_summaries=gap_lock_summaries,
        lateral_start_summaries=lateral_start_summaries,
        merge_completion_summaries=merge_completion_summaries,
        step0_mainline_conversion_summaries=step0_mainline_conversion_summaries,
        bundle_released_summaries=bundle_released_summaries,
    )
    runtime = run.final_state.ramp_merge_runtime
    runtime_version = runtime.version if isinstance(runtime, RampMergeRuntimeState) else None
    legacy_summary = {
        "scenario_id": scenario_id,
        "case_spec": spec.to_dict() if spec is not None else {},
        "mv_id": mv_id,
        "run_id": run_id,
        "max_steps": max_steps,
        "actual_steps": len(run.step_results),
        "dt": run.initial_state.dt,
        "first_trigger_step": first_trigger["step"] if first_trigger is not None else None,
        "first_trigger_t": first_trigger["t"] if first_trigger is not None else None,
        "first_trigger_mv_x_global": (
            first_trigger["mv_x_global"] if first_trigger is not None else None
        ),
        "first_trigger_mv_local_frame": _first_trigger_local_frame(
            run.step_results,
            first_trigger["step"] if first_trigger is not None else None,
            mv_id,
        ),
        "first_trigger_plan_summary": first_trigger,
        "first_trigger_gap_rows": first_trigger_gap_rows,
        "trigger_plan_summaries": trigger_plan_summaries,
        "trigger_round_plan_summaries": trigger_round_plan_summaries,
        "trigger_round_gap_rows": trigger_round_gap_rows,
        "longitudinal_completion_event": longitudinal_completion_event,
        "gap_lock_summaries": gap_lock_summaries,
        "lateral_start_summaries": lateral_start_summaries,
        "lateral_progress_summaries": lateral_progress_summaries,
        "merge_completion_summaries": merge_completion_summaries,
        "step0_mainline_conversion_summaries": step0_mainline_conversion_summaries,
        "mv_lifecycle_summaries": mv_lifecycle_summaries,
        "bundle_lifecycle_summaries": bundle_lifecycle_summaries,
        "bundle_created_summaries": bundle_created_summaries,
        "bundle_released_summaries": bundle_released_summaries,
        "trigger_events": _typed_payload_events(
            run.actual_events,
            "ramp_merge_trigger",
        ),
        "final_vehicle_states": _final_vehicle_states(run.final_state),
        "final_active_bundle_ids": (
            list(runtime.onestep_plan_bundles)
            if isinstance(runtime, RampMergeRuntimeState)
            else []
        ),
        "final_gap_plan_ids": (
            list(runtime.gap_plans)
            if isinstance(runtime, RampMergeRuntimeState)
            else []
        ),
        "final_lateral_trajectory_ids": (
            list(runtime.lateral_trajectories)
            if isinstance(runtime, RampMergeRuntimeState)
            else []
        ),
        "final_controlled_vehicle_states": (
            {
                vehicle_id: {
                    "owner_mv_id": controlled.owner_mv_id,
                    "bundle_id": controlled.bundle_id,
                    "role": controlled.role,
                }
                for vehicle_id, controlled in runtime.controlled_vehicle_states.items()
            }
            if isinstance(runtime, RampMergeRuntimeState)
            else {}
        ),
        "final_mv_plan_states": (
            {
                vehicle_id: {
                    "current_plan_id": mv_state.current_plan_id,
                    "current_plan_gap": _gap_ref_payload(mv_state.current_plan_gap),
                    "locked_plan_id": mv_state.locked_plan_id,
                    "locked_gap": _gap_ref_payload(mv_state.locked_gap),
                    "active_bundle_id": mv_state.active_bundle_id,
                    "active_lateral_trajectory_id": mv_state.active_lateral_trajectory_id,
                    "merge_state": mv_state.merge_state,
                }
                for vehicle_id, mv_state in runtime.mv_plan_states.items()
            }
            if isinstance(runtime, RampMergeRuntimeState)
            else {}
        ),
        "runtime_version": runtime_version,
    }
    return _formalize_stage2_summary(
        run,
        legacy_summary,
        scenario_metadata={
            **_build_multimv_scenario_metadata(scenario_id),
            **dict(scenario_metadata or {}),
        },
    )


def _build_stage2_summary_from_run(
    run: _OneStepStage2StepRun,
    *,
    run_id: str,
    max_steps: int,
    case_spec: OneStepRampMergeCaseSpec | None = None,
    scenario_metadata: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    return _build_stage2_summary(
        run,
        run_id=run_id,
        max_steps=max_steps,
        case_spec=case_spec,
        scenario_metadata=scenario_metadata,
    )


def _primary_mv_id(
    scenario_id: str,
    case_spec: OneStepRampMergeCaseSpec | None,
) -> str:
    if scenario_id in RM_MULTIMV_SCENARIO_IDS:
        case = get_multimv_case_spec(scenario_id)
        return multimv_mv_vehicle_ids(case)[0]
    if case_spec is None:
        raise ValueError(f"missing case_spec for scenario_id: {scenario_id}")
    return case_spec.mv_id


def _build_multimv_scenario_metadata(scenario_id: str) -> dict[str, object]:
    if scenario_id not in RM_MULTIMV_SCENARIO_IDS:
        return {}
    case = get_multimv_case_spec(scenario_id)
    return {
        "case_family": "rm_multimv",
        "multimv_case": {
            "id": case.id,
            "scenario_id": case.scenario_id,
            "mv_count": case.mv_count,
            "category": case.category,
            "title": case.title,
            "x_targets": list(case.x_targets),
            "x_m_list": list(case.x_m_list),
            "planning_order": list(case.planning_order),
            "purpose": case.purpose,
            "modules": list(case.modules),
            "expected": case.expected,
            "final_x_targets_after_static_rolling": list(
                case.final_x_targets_after_static_rolling
            ),
            "hdv_lane2_indices": list(case.hdv_lane2_indices),
        },
        "coordinate_policy": {
            "origin_x_global": RM_MULTIMV_ORIGIN_X_GLOBAL,
            "lane2_global_x": "origin_x_global + x_targets[i]",
            "mv_global_x": "origin_x_global + x_m_list[i]",
        },
    }


def _mv_lifecycle_summaries(
    run: _OneStepStage2StepRun,
    *,
    gap_lock_summaries: list[dict[str, Any]],
    lateral_start_summaries: list[dict[str, Any]],
    merge_completion_summaries: list[dict[str, Any]],
    step0_mainline_conversion_summaries: list[dict[str, Any]],
    bundle_released_summaries: list[dict[str, Any]],
) -> dict[str, dict[str, Any]]:
    mv_ids = {
        str(record.get("mv_id"))
        for records in (
            gap_lock_summaries,
            lateral_start_summaries,
            merge_completion_summaries,
            bundle_released_summaries,
        )
        for record in records
        if record.get("mv_id") is not None
    }
    initial_runtime = run.initial_state.ramp_merge_runtime
    if isinstance(initial_runtime, RampMergeRuntimeState):
        mv_ids.update(str(mv_id) for mv_id in initial_runtime.mv_plan_states)
    final_runtime = run.final_state.ramp_merge_runtime
    if isinstance(final_runtime, RampMergeRuntimeState):
        mv_ids.update(str(mv_id) for mv_id in final_runtime.mv_plan_states)

    result: dict[str, dict[str, Any]] = {}
    for mv_id in sorted(mv_ids):
        lock = _first_record_for_mv(gap_lock_summaries, mv_id)
        lateral = _first_record_for_mv(lateral_start_summaries, mv_id)
        merge_completed = _first_record_for_mv(merge_completion_summaries, mv_id)
        mainline_conversion = _first_conversion_for_mv(
            step0_mainline_conversion_summaries,
            mv_id,
        )
        bundle_release = _first_record_for_mv(
            [
                record
                for record in bundle_released_summaries
                if record.get("reason") == "onestep_stage2_lateral_start_release"
            ],
            mv_id,
        )
        final_state = _final_vehicle_states(run.final_state).get(mv_id)
        result[mv_id] = {
            "mv_id": mv_id,
            "locked_gap_step": _record_step(lock),
            "locked_gap": lock.get("locked_gap") if lock is not None else None,
            "locked_plan_id": lock.get("locked_plan_id") if lock is not None else None,
            "lateral_start_step": _record_step(lateral),
            "active_lateral_trajectory_id": (
                lateral.get("active_lateral_trajectory_id")
                if lateral is not None
                else None
            ),
            "bundle_release_step": _record_step(bundle_release),
            "merge_completed_step": _record_step(merge_completed),
            "mainline_conversion_step": _record_step(mainline_conversion),
            "final_vehicle_state": final_state,
        }
    return result


FORMAL_EVENT_KINDS = frozenset(
    {
        "trigger_round",
        "gap_evaluation",
        "bundle_created",
        "bundle_released",
        "current_plan_updated",
        "locked_gap_created",
        "lateral_started",
        "lateral_completed",
        "mainline_converted",
    }
)


def _formalize_stage2_summary(
    run: _OneStepStage2StepRun,
    legacy: Mapping[str, Any],
    *,
    scenario_metadata: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    final_runtime_leftovers = _final_runtime_leftovers(legacy)
    mv_ids = _collect_mv_ids(run, legacy)
    formal_events = _build_formal_events(legacy)
    round_summaries = _build_round_summaries(run, legacy)
    mv_summaries = _build_mv_summaries(run, legacy, mv_ids)
    cross_mv_summary = _build_cross_mv_summary(
        round_summaries,
        mv_summaries,
        formal_events,
        final_runtime_leftovers,
    )
    scenario_summary = {
        "scenario_id": legacy["scenario_id"],
        "run_id": legacy["run_id"],
        "algorithm_variant": "onestep_stage2",
        "max_steps": legacy["max_steps"],
        "actual_steps": legacy["actual_steps"],
        "dt": legacy["dt"],
        "case_spec": dict(legacy["case_spec"]),
        "mv_ids": list(mv_ids),
        "primary_mv_id": legacy.get("mv_id"),
        "initial_vehicle_states": _vehicle_states_payload(run.initial_state),
        "final_vehicle_states": dict(legacy["final_vehicle_states"]),
        "runtime_version": legacy["runtime_version"],
        "final_runtime_leftovers": final_runtime_leftovers,
        "formal_events": formal_events,
        "event_counts": _event_counts(formal_events),
    }
    if scenario_metadata:
        scenario_summary.update(dict(scenario_metadata))
    return {
        "scenario_summary": scenario_summary,
        "round_summaries": round_summaries,
        "mv_summaries": mv_summaries,
        "cross_mv_summary": cross_mv_summary,
        "artifact_paths": {},
    }


def _collect_mv_ids(
    run: _OneStepStage2StepRun,
    legacy: Mapping[str, Any],
) -> tuple[str, ...]:
    mv_ids: set[str] = {str(legacy.get("mv_id"))}
    for key in (
        "trigger_round_plan_summaries",
        "trigger_round_gap_rows",
        "gap_lock_summaries",
        "lateral_start_summaries",
        "merge_completion_summaries",
        "bundle_lifecycle_summaries",
    ):
        for record in legacy.get(key) or []:
            if record.get("mv_id") is not None:
                mv_ids.add(str(record["mv_id"]))
    initial_runtime = run.initial_state.ramp_merge_runtime
    if isinstance(initial_runtime, RampMergeRuntimeState):
        mv_ids.update(str(mv_id) for mv_id in initial_runtime.mv_plan_states)
    final_runtime = run.final_state.ramp_merge_runtime
    if isinstance(final_runtime, RampMergeRuntimeState):
        mv_ids.update(str(mv_id) for mv_id in final_runtime.mv_plan_states)
    for step_result in run.step_results:
        for vehicle_id in step_result.frozen_state.active_vehicle_ids:
            state = step_result.frozen_state.vehicle_states.get(vehicle_id)
            if state is None:
                continue
            if state.road_role == "on_ramp_mv" or state.physical_lane == "on_ramp":
                mv_ids.add(str(vehicle_id))
        for vehicle_id in step_result.advanced_state.active_vehicle_ids:
            state = step_result.advanced_state.vehicle_states.get(vehicle_id)
            if state is None:
                continue
            if state.road_role == "on_ramp_mv" or state.physical_lane == "on_ramp":
                mv_ids.add(str(vehicle_id))
        for event in step_result.actual_events:
            if event.get("event_type") != "boundary_generation":
                continue
            payload = dict(event.get("payload") or {})
            lane_ids = dict(payload.get("lane_id") or payload.get("lane_ids") or {})
            for vehicle_id in payload.get("generated_vehicle_ids") or []:
                state = step_result.frozen_state.vehicle_states.get(str(vehicle_id))
                if lane_ids.get(str(vehicle_id)) == "on_ramp" or (
                    state is not None and state.road_role == "on_ramp_mv"
                ):
                    mv_ids.add(str(vehicle_id))
    return tuple(sorted(mv_id for mv_id in mv_ids if mv_id and mv_id != "None"))


def _build_formal_events(legacy: Mapping[str, Any]) -> list[dict[str, Any]]:
    events: list[dict[str, Any]] = []
    emitted_rounds: set[str] = set()
    for record in legacy.get("trigger_round_plan_summaries") or []:
        round_id = str(record.get("round_id") or "")
        if round_id in emitted_rounds:
            continue
        emitted_rounds.add(round_id)
        events.append(
            _formal_event(
                "trigger_round",
                record,
                payload={
                    "trigger_reason": _raw_trigger_reason_for_step(
                        legacy,
                        int(record.get("step") or -1),
                    ),
                    "active_trigger_reasons": _round_active_trigger_reasons(
                        legacy,
                        int(record.get("step") or -1),
                        [record],
                    ),
                },
            )
        )
    for record in legacy.get("gap_lock_summaries") or []:
        round_id = str(record.get("round_id") or "")
        if round_id and round_id not in emitted_rounds:
            emitted_rounds.add(round_id)
            events.append(
                _formal_event(
                    "trigger_round",
                    record,
                    payload={
                        "trigger_reason": _raw_trigger_reason_for_step(
                            legacy,
                            int(record.get("step") or -1),
                        ),
                        "active_trigger_reasons": _round_active_trigger_reasons(
                            legacy,
                            int(record.get("step") or -1),
                            [],
                        ),
                    },
                )
            )

    for record in legacy.get("trigger_round_gap_rows") or []:
        events.append(
            _formal_event(
                "gap_evaluation",
                record,
                payload=_formal_gap_row(record),
            )
        )
    for record in legacy.get("bundle_created_summaries") or []:
        events.append(
            _formal_event(
                "bundle_created",
                record,
                payload=_without_common_event_fields(record),
            )
        )
    for record in legacy.get("bundle_released_summaries") or []:
        events.append(
            _formal_event(
                "bundle_released",
                record,
                payload=_without_common_event_fields(record),
            )
        )
    for record in legacy.get("trigger_round_plan_summaries") or []:
        if record.get("gap_plan_id") is None and record.get("current_plan_gap") is None:
            continue
        events.append(
            _formal_event(
                "current_plan_updated",
                record,
                payload=_without_common_event_fields(record),
            )
        )
    for record in legacy.get("gap_lock_summaries") or []:
        events.append(
            _formal_event(
                "locked_gap_created",
                record,
                payload=_without_common_event_fields(record),
            )
        )
    for record in legacy.get("lateral_start_summaries") or []:
        events.append(
            _formal_event(
                "lateral_started",
                record,
                payload=_without_common_event_fields(record),
            )
        )
    for record in legacy.get("merge_completion_summaries") or []:
        events.append(
            _formal_event(
                "lateral_completed",
                record,
                payload=_without_common_event_fields(record),
            )
        )
    for record in legacy.get("step0_mainline_conversion_summaries") or []:
        for mv_id in record.get("mainline_converted_vehicle_ids") or []:
            payload = _without_common_event_fields(record)
            payload["mainline_converted_vehicle_id"] = mv_id
            events.append(
                _formal_event(
                    "mainline_converted",
                    {**record, "mv_id": mv_id},
                    payload=payload,
                )
            )
    return sorted(
        events,
        key=lambda event: (
            int(event.get("step") or 0),
            _formal_event_order(str(event.get("event_kind") or "")),
            int(event.get("round_order") or 0),
            str(event.get("mv_id") or ""),
        ),
    )


def _formal_event(
    event_kind: str,
    record: Mapping[str, Any],
    *,
    payload: Mapping[str, Any],
) -> dict[str, Any]:
    return {
        "event_kind": event_kind,
        "step": _int_or_none(record.get("step")),
        "t": record.get("t"),
        "mv_id": record.get("mv_id"),
        "round_id": record.get("round_id"),
        "round_order": _int_or_none(record.get("round_order")),
        "payload": dict(payload),
    }


def _build_round_summaries(
    run: _OneStepStage2StepRun,
    legacy: Mapping[str, Any],
) -> list[dict[str, Any]]:
    plan_records = list(legacy.get("trigger_round_plan_summaries") or [])
    lock_records = list(legacy.get("gap_lock_summaries") or [])
    gap_rows = [_formal_gap_row(row) for row in legacy.get("trigger_round_gap_rows") or []]
    round_ids = tuple(
        dict.fromkeys(
            str(record.get("round_id"))
            for record in [*plan_records, *lock_records, *gap_rows]
            if record.get("round_id") is not None
        )
    )
    result: list[dict[str, Any]] = []
    for round_id in round_ids:
        round_plans = [
            _with_plan_local_frame(run, record)
            for record in _records_for_round(plan_records, round_id)
        ]
        round_locks = _records_for_round(lock_records, round_id)
        round_gap_rows = _records_for_round(gap_rows, round_id)
        anchor = (round_plans or round_locks or round_gap_rows)[0]
        step = int(anchor["step"])
        gap_snapshot = _gap_snapshot_payload_for_step(run, step)
        ordered_records = sorted(
            [*round_plans, *round_locks],
            key=lambda item: (
                int(item.get("round_order") or 0),
                str(item.get("mv_id") or ""),
            ),
        )
        mv_order = [
            str(record.get("mv_id"))
            for record in ordered_records
            if record.get("mv_id") is not None
        ]
        mv_order = list(dict.fromkeys(mv_order))
        result.append(
            {
                "round_id": round_id,
                "step": step,
                "t": anchor.get("t"),
                "trigger_reason": _round_trigger_reason(
                    legacy,
                    step,
                    round_plans,
                    round_locks,
                ),
                "active_trigger_reasons": _round_active_trigger_reasons(
                    legacy,
                    step,
                    round_plans,
                ),
                "mv_order": mv_order,
                "gap_snapshot": gap_snapshot,
                "gap_count": len(gap_snapshot.get("gaps", [])),
                "danger_vehicle_ids": list(gap_snapshot.get("danger_vehicle_ids", [])),
                "plan_summaries": round_plans,
                "locked_gap_events": round_locks,
                "selected_gap_indices": [
                    int(record["gap_index"])
                    for record in round_plans
                    if record.get("gap_index") is not None
                ],
                "locked_gap_indices": [
                    int(record["locked_gap"]["index"])
                    for record in round_locks
                    if isinstance(record.get("locked_gap"), Mapping)
                    and record["locked_gap"].get("index") is not None
                ],
                "tail_frontier_updates": _tail_frontier_updates(ordered_records),
                "uncontrollable_vehicle_ids_after": _last_uncontrollable_after(
                    ordered_records
                ),
                "gap_rows": round_gap_rows,
            }
        )
    return result


def _build_mv_summaries(
    run: _OneStepStage2StepRun,
    legacy: Mapping[str, Any],
    mv_ids: tuple[str, ...],
) -> dict[str, dict[str, Any]]:
    lifecycle_legacy = dict(legacy.get("mv_lifecycle_summaries") or {})
    result: dict[str, dict[str, Any]] = {}
    final_mv_plan_states = dict(legacy.get("final_mv_plan_states") or {})
    initial_vehicle_states = _vehicle_states_payload(run.initial_state)
    final_vehicle_states = dict(legacy.get("final_vehicle_states") or {})
    for mv_id in mv_ids:
        plans = _records_for_mv(legacy.get("trigger_round_plan_summaries") or [], mv_id)
        gap_rows = [
            _formal_gap_row(row)
            for row in _records_for_mv(legacy.get("trigger_round_gap_rows") or [], mv_id)
        ]
        locks = _records_for_mv(legacy.get("gap_lock_summaries") or [], mv_id)
        lateral_starts = _records_for_mv(
            legacy.get("lateral_start_summaries") or [],
            mv_id,
        )
        lateral_completions = _records_for_mv(
            legacy.get("merge_completion_summaries") or [],
            mv_id,
        )
        conversions = [
            record
            for record in legacy.get("step0_mainline_conversion_summaries") or []
            if mv_id in set(record.get("mainline_converted_vehicle_ids") or [])
        ]
        bundles = _records_for_mv(legacy.get("bundle_lifecycle_summaries") or [], mv_id)
        legacy_lifecycle = dict(lifecycle_legacy.get(mv_id) or {})
        final_state = dict(final_vehicle_states.get(mv_id) or {})
        lifecycle = {
            "first_control_zone_step": _first_plan_step(plans),
            "first_trigger_step": _first_plan_step(plans),
            "first_current_plan_step": _first_plan_step(plans),
            "locked_gap_step": legacy_lifecycle.get("locked_gap_step"),
            "lateral_start_step": legacy_lifecycle.get("lateral_start_step"),
            "lateral_completed_step": legacy_lifecycle.get("merge_completed_step"),
            "mainline_conversion_step": legacy_lifecycle.get("mainline_conversion_step"),
            "final_status": {
                "physical_lane": final_state.get("physical_lane"),
                "road_role": final_state.get("road_role"),
                "merge_state": final_state.get("merge_state"),
                "runtime_present": mv_id in final_mv_plan_states,
            },
        }
        result[mv_id] = {
            "mv_id": mv_id,
            "initial_vehicle_state": dict(initial_vehicle_states.get(mv_id) or {}),
            "final_vehicle_state": final_state,
            "lifecycle": lifecycle,
            "plans": plans,
            "bundles": bundles,
            "gap_rows": gap_rows,
            "locked_gap": legacy_lifecycle.get("locked_gap"),
            "locked_plan_id": legacy_lifecycle.get("locked_plan_id"),
            "lateral": {
                "active_lateral_trajectory_id": legacy_lifecycle.get(
                    "active_lateral_trajectory_id"
                ),
                "start": lateral_starts[0] if lateral_starts else None,
                "completed": lateral_completions[0] if lateral_completions else None,
                "start_step": legacy_lifecycle.get("lateral_start_step"),
                "completed_step": legacy_lifecycle.get("merge_completed_step"),
                "duration_steps": (
                    lateral_starts[0].get("duration_steps") if lateral_starts else None
                ),
                "target_y": lateral_starts[0].get("target_y") if lateral_starts else None,
            },
            "step0_conversion": conversions[0] if conversions else None,
        }
    return result


def _build_cross_mv_summary(
    round_summaries: list[dict[str, Any]],
    mv_summaries: Mapping[str, Mapping[str, Any]],
    formal_events: list[dict[str, Any]],
    final_runtime_leftovers: Mapping[str, Any],
) -> dict[str, Any]:
    gap_conflicts = _detect_formal_gap_conflicts(round_summaries)
    frontier_violations = _detect_frontier_violations(round_summaries)
    ownership_timeline, ownership_conflicts = _bundle_ownership_timeline(formal_events)
    return {
        "mv_order_by_round": {
            item["round_id"]: list(item["mv_order"]) for item in round_summaries
        },
        "selected_gap_indices_by_round": {
            item["round_id"]: list(item["selected_gap_indices"]) for item in round_summaries
        },
        "locked_gap_indices_by_mv": {
            mv_id: summary.get("locked_gap", {}).get("index")
            for mv_id, summary in mv_summaries.items()
            if isinstance(summary.get("locked_gap"), Mapping)
        },
        "frontier_timeline": [
            {
                "round_id": item["round_id"],
                "updates": list(item["tail_frontier_updates"]),
            }
            for item in round_summaries
        ],
        "uncontrollable_vehicle_timeline": [
            {
                "round_id": item["round_id"],
                "uncontrollable_vehicle_ids_after": list(
                    item["uncontrollable_vehicle_ids_after"]
                ),
            }
            for item in round_summaries
        ],
        "bundle_ownership_timeline": ownership_timeline,
        "lifecycle_order": {
            mv_id: dict(summary["lifecycle"]) for mv_id, summary in mv_summaries.items()
        },
        "gap_conflicts": gap_conflicts,
        "frontier_violations": frontier_violations,
        "ownership_conflicts": ownership_conflicts,
        "final_runtime_leftovers": dict(final_runtime_leftovers),
    }


def _detect_formal_gap_conflicts(
    round_summaries: list[dict[str, Any]],
) -> list[dict[str, Any]]:
    conflicts: list[dict[str, Any]] = []
    for round_summary in round_summaries:
        by_key: dict[tuple[Any, ...], list[str]] = {}
        for record in [
            *round_summary.get("plan_summaries", []),
            *round_summary.get("locked_gap_events", []),
        ]:
            gap = record.get("selected_gap") or record.get("locked_gap")
            key = _structured_gap_key(gap)
            if key is None:
                continue
            by_key.setdefault(key, []).append(str(record.get("mv_id")))
        for key, mv_ids in by_key.items():
            unique_mv_ids = list(dict.fromkeys(mv_ids))
            if len(unique_mv_ids) > 1:
                conflicts.append(
                    {
                        "round_id": round_summary["round_id"],
                        "gap_key": list(key),
                        "mv_ids": unique_mv_ids,
                    }
                )
    return conflicts


def _detect_frontier_violations(
    round_summaries: list[dict[str, Any]],
) -> list[dict[str, Any]]:
    violations: list[dict[str, Any]] = []
    for round_summary in round_summaries:
        for record in [
            *round_summary.get("plan_summaries", []),
            *round_summary.get("locked_gap_events", []),
        ]:
            before = record.get("tail_frontier_gap_index_before")
            gap_index = record.get("gap_index")
            if gap_index is None and isinstance(record.get("locked_gap"), Mapping):
                gap_index = record["locked_gap"].get("index")
            if before is None or gap_index is None:
                continue
            if int(gap_index) <= int(before):
                violations.append(
                    {
                        "round_id": round_summary["round_id"],
                        "mv_id": record.get("mv_id"),
                        "round_order": record.get("round_order"),
                        "gap_index": int(gap_index),
                        "tail_frontier_gap_index_before": int(before),
                    }
                )
    return violations


def _bundle_ownership_timeline(
    formal_events: list[dict[str, Any]],
) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    active_by_bundle: dict[str, dict[str, Any]] = {}
    timeline: list[dict[str, Any]] = []
    conflicts: list[dict[str, Any]] = []
    for event in formal_events:
        if event["event_kind"] not in {"bundle_created", "bundle_released"}:
            continue
        payload = dict(event.get("payload") or {})
        bundle_id = payload.get("bundle_id")
        if not bundle_id:
            continue
        if event["event_kind"] == "bundle_created":
            active_by_bundle[str(bundle_id)] = {
                "bundle_id": bundle_id,
                "mv_id": event.get("mv_id"),
                "controlled_vehicle_ids": list(payload.get("controlled_vehicle_ids") or []),
                "start_step": event.get("step"),
            }
        else:
            active_by_bundle.pop(str(bundle_id), None)
        owner_by_vehicle: dict[str, list[str]] = {}
        for bundle in active_by_bundle.values():
            for vehicle_id in bundle.get("controlled_vehicle_ids", []):
                owner_by_vehicle.setdefault(str(vehicle_id), []).append(
                    str(bundle["bundle_id"])
                )
        step_conflicts = {
            vehicle_id: bundle_ids
            for vehicle_id, bundle_ids in owner_by_vehicle.items()
            if len(bundle_ids) > 1
        }
        if step_conflicts:
            conflicts.append(
                {
                    "step": event.get("step"),
                    "t": event.get("t"),
                    "vehicle_owners": step_conflicts,
                }
            )
        timeline.append(
            {
                "step": event.get("step"),
                "t": event.get("t"),
                "event_kind": event["event_kind"],
                "bundle_id": bundle_id,
                "mv_id": event.get("mv_id"),
                "active_bundle_ids": sorted(active_by_bundle),
                "active_owners_by_vehicle": owner_by_vehicle,
            }
        )
    return timeline, conflicts


def _final_runtime_leftovers(legacy: Mapping[str, Any]) -> dict[str, Any]:
    return {
        "mv_plan_state_ids": sorted((legacy.get("final_mv_plan_states") or {}).keys()),
        "active_bundle_ids": list(legacy.get("final_active_bundle_ids") or []),
        "gap_plan_ids": list(legacy.get("final_gap_plan_ids") or []),
        "lateral_trajectory_ids": list(legacy.get("final_lateral_trajectory_ids") or []),
        "controlled_vehicle_states": dict(
            legacy.get("final_controlled_vehicle_states") or {}
        ),
    }


def _vehicle_states_payload(state: SimulationState) -> dict[str, dict[str, Any]]:
    return {
        vehicle_id: {
            "x_global": vehicle_state.x_global,
            "y": vehicle_state.y,
            "v": vehicle_state.v,
            "physical_lane": vehicle_state.physical_lane,
            "road_role": vehicle_state.road_role,
            "merge_state": vehicle_state.merge_state,
        }
        for vehicle_id, vehicle_state in state.vehicle_states.items()
        if vehicle_id in state.active_vehicle_ids
    }


def _formal_gap_row(record: Mapping[str, Any]) -> dict[str, Any]:
    return {
        "step": record.get("step"),
        "t": record.get("t"),
        "mv_id": record.get("mv_id"),
        "round_id": record.get("round_id"),
        "round_order": record.get("round_order"),
        "bundle_id": record.get("bundle_id"),
        "gap_id": record.get("gap_id"),
        "gap_index": record.get("gap_index"),
        "kernel_gap_index": record.get("kernel_gap_index"),
        "front_vehicle_id": record.get("front_vehicle_id"),
        "rear_vehicle_id": record.get("rear_vehicle_id"),
        "front_controllable": record.get("front_controllable"),
        "rear_controllable": record.get("rear_controllable"),
        "controllability_branch": record.get("controllability_branch"),
        "branch": record.get("controllability_branch"),
        "reachable": record.get("reachable"),
        "coop_feasible": record.get("coop_feasible"),
        "included_in_scoring": record.get("included_in_scoring"),
        "failure_reason": record.get("failure_reason"),
        "is_selected": record.get("is_selected"),
        "J": record.get("J"),
        "tail_frontier_gap_index_before": record.get("tail_frontier_gap_index_before"),
        "filtered_by_frontier": record.get("filtered_by_frontier"),
        "filtered_by_frontier_gap_indices": list(
            record.get("filtered_by_frontier_gap_indices") or []
        ),
    }


def _gap_snapshot_payload_for_step(
    run: _OneStepStage2StepRun,
    step: int,
) -> dict[str, Any]:
    if step < len(run.step_results):
        result = run.step_results[step]
        snapshot = result.gap_snapshot
        if snapshot is None:
            snapshot = identify_and_number_gaps(
                result.frozen_state,
                tuple(result.safety_result.danger_vehicle_ids),
            )
        return {
            "step": snapshot.step,
            "t": snapshot.t,
            "lane_id": snapshot.lane_id,
            "danger_vehicle_ids": list(snapshot.danger_vehicle_ids),
            "gaps": [
                {
                    "gap_id": gap.gap_id,
                    "index": gap.index,
                    "front_vehicle_id": gap.front_vehicle_id,
                    "rear_vehicle_id": gap.rear_vehicle_id,
                    "front_x_global": gap.front_x_global,
                    "rear_x_global": gap.rear_x_global,
                    "bumper_gap_m": gap.bumper_gap_m,
                    "effective_control_type": gap.effective_control_type,
                }
                for gap in snapshot.gaps
            ],
        }
    return {
        "step": step,
        "t": None,
        "lane_id": None,
        "danger_vehicle_ids": [],
        "gaps": [],
    }


def _with_plan_local_frame(
    run: _OneStepStage2StepRun,
    record: Mapping[str, Any],
) -> dict[str, Any]:
    result = dict(record)
    step = _int_or_none(record.get("step"))
    mv_id = record.get("mv_id")
    if step is None or mv_id is None or step >= len(run.step_results):
        result["mv_local_frame"] = None
        return result
    result["mv_local_frame"] = _local_frame_for_step_mv(
        run.step_results,
        step,
        str(mv_id),
    )
    return result


def _local_frame_for_step_mv(
    step_results: tuple[RampMergeStepResult, ...],
    step: int,
    mv_id: str,
) -> dict[str, Any] | None:
    result = step_results[step]
    state = result.frozen_state
    danger_vehicle_ids = tuple(result.safety_result.danger_vehicle_ids)
    gap_snapshot = result.gap_snapshot or identify_and_number_gaps(state, danger_vehicle_ids)
    local_frame = build_stage2_local_frame_from_snapshot(state, mv_id, gap_snapshot)
    return {
        "mv_id": mv_id,
        "origin_x_global": float(local_frame["origin_x_global"]),
        "x_m0_local": 0.0,
        "lane_2_vehicle_order": list(local_frame["lane_2_vehicle_order"]),
        "lane_2_vehicle_x_local_by_id": dict(local_frame["lane_2_vehicle_x_local_by_id"]),
        "gap_intervals_local": [
            list(interval)
            for interval in tuple(local_frame["gap_intervals_local"])
        ],
        "gap_centers_local": list(local_frame["gap_centers_local"]),
        "gap_vehicle_ids_by_index": dict(local_frame["gap_vehicle_ids_by_index"]),
        "runtime_gap_index_by_kernel_index": dict(
            local_frame["runtime_gap_index_by_kernel_index"]
        ),
    }


def _records_for_round(records: list[Mapping[str, Any]], round_id: str) -> list[dict[str, Any]]:
    return [dict(record) for record in records if record.get("round_id") == round_id]


def _records_for_mv(records: list[Mapping[str, Any]], mv_id: str) -> list[dict[str, Any]]:
    return [dict(record) for record in records if record.get("mv_id") == mv_id]


def _round_trigger_reason(
    legacy: Mapping[str, Any],
    step: int,
    plan_records: list[Mapping[str, Any]],
    lock_records: list[Mapping[str, Any]],
) -> str | None:
    raw_reason = _raw_trigger_reason_for_step(legacy, step)
    if raw_reason is not None:
        return raw_reason
    for record in [*plan_records, *lock_records]:
        if record.get("reason") is not None:
            return str(record["reason"])
    return None


def _raw_trigger_reason_for_step(
    legacy: Mapping[str, Any],
    step: int,
) -> str | None:
    trigger = next(
        (
            record
            for record in legacy.get("trigger_events") or []
            if int(record.get("step") or -1) == step
        ),
        None,
    )
    if trigger is None:
        return None
    return trigger.get("trigger_reason")


def _round_active_trigger_reasons(
    legacy: Mapping[str, Any],
    step: int,
    plan_records: list[Mapping[str, Any]],
) -> list[str]:
    trigger = next(
        (
            record
            for record in legacy.get("trigger_events") or []
            if int(record.get("step") or -1) == step
        ),
        None,
    )
    if trigger is not None:
        return list(trigger.get("active_trigger_reasons") or [])
    return [
        str(record.get("reason"))
        for record in plan_records
        if record.get("reason") is not None
    ]


def _tail_frontier_updates(records: list[Mapping[str, Any]]) -> list[dict[str, Any]]:
    return [
        {
            "mv_id": record.get("mv_id"),
            "round_order": record.get("round_order"),
            "tail_frontier_gap_index_before": record.get(
                "tail_frontier_gap_index_before"
            ),
            "tail_frontier_gap_index_after": record.get("tail_frontier_gap_index_after"),
        }
        for record in records
        if record.get("tail_frontier_gap_index_before") is not None
        or record.get("tail_frontier_gap_index_after") is not None
    ]


def _last_uncontrollable_after(records: list[Mapping[str, Any]]) -> list[str]:
    for record in reversed(records):
        value = record.get("uncontrollable_vehicles_round_after")
        if value is not None:
            return list(value)
    return []


def _first_plan_step(records: list[Mapping[str, Any]]) -> int | None:
    if not records:
        return None
    return _int_or_none(records[0].get("step"))


def _event_counts(events: list[Mapping[str, Any]]) -> dict[str, int]:
    counts = {event_kind: 0 for event_kind in sorted(FORMAL_EVENT_KINDS)}
    for event in events:
        kind = str(event.get("event_kind"))
        counts[kind] = counts.get(kind, 0) + 1
    return counts


def _formal_event_order(event_kind: str) -> int:
    return {
        "trigger_round": 0,
        "gap_evaluation": 1,
        "bundle_released": 2,
        "bundle_created": 3,
        "current_plan_updated": 4,
        "locked_gap_created": 5,
        "lateral_started": 6,
        "lateral_completed": 7,
        "mainline_converted": 8,
    }.get(event_kind, 99)


def _without_common_event_fields(record: Mapping[str, Any]) -> dict[str, Any]:
    return {
        key: value
        for key, value in record.items()
        if key not in {"step", "t", "mv_id", "round_id", "round_order"}
    }


def _structured_gap_key(value: Any) -> tuple[Any, ...] | None:
    if not isinstance(value, Mapping):
        return None
    front_vehicle_id = value.get("front_vehicle_id")
    rear_vehicle_id = value.get("rear_vehicle_id")
    if front_vehicle_id is None or rear_vehicle_id is None:
        return None
    return (
        value.get("snapshot_step"),
        value.get("index"),
        front_vehicle_id,
        rear_vehicle_id,
    )


def _int_or_none(value: Any) -> int | None:
    return int(value) if value is not None else None


def _first_record_for_mv(
    records: list[dict[str, Any]],
    mv_id: str,
) -> dict[str, Any] | None:
    for record in records:
        if record.get("mv_id") == mv_id:
            return record
    return None


def _first_conversion_for_mv(
    records: list[dict[str, Any]],
    mv_id: str,
) -> dict[str, Any] | None:
    for record in records:
        if mv_id in set(record.get("mainline_converted_vehicle_ids") or []):
            return record
    return None


def _record_step(record: dict[str, Any] | None) -> int | None:
    return int(record["step"]) if record is not None else None


def _first_trigger_local_frame(
    step_results: tuple[RampMergeStepResult, ...],
    trigger_step: int | None,
    mv_id: str,
) -> dict[str, Any] | None:
    if trigger_step is None:
        return None
    return _local_frame_for_step_mv(step_results, int(trigger_step), mv_id)


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


def _first_event(
    events: tuple[Mapping[str, Any], ...],
    event_type: str,
) -> dict[str, Any] | None:
    for event in events:
        if event.get("event_type") != event_type:
            continue
        return {
            "step": int(event["step"]),
            "t": float(event["t"]),
            **dict(event.get("payload", {})),
        }
    return None


def _gap_ref_payload(value: Any) -> dict[str, Any] | None:
    if value is None:
        return None
    return {
        "gap_id": value.gap_id,
        "index": value.index,
        "front_vehicle_id": value.front_vehicle_id,
        "rear_vehicle_id": value.rear_vehicle_id,
        "snapshot_step": value.snapshot_step,
        "snapshot_t": value.snapshot_t,
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


def _extend_history(target: OutputHistory, source: OutputHistory) -> None:
    target.trajectory_records.extend(source.trajectory_records)
    target.event_records.extend(source.event_records)
    target.sanity_check_records.extend(source.sanity_check_records)
    target.png_artifacts.extend(source.png_artifacts)
