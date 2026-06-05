from __future__ import annotations

import json
from dataclasses import dataclass
from datetime import UTC, datetime
from pathlib import Path
from typing import Any, Iterable, Mapping

from cormc.basic_scenarios import (
    BASIC_PRE_CONTROL_IDS,
    BASIC_RAMP_PRE_IDS,
    BASIC_SCENARIO_IDS,
    BasicScenarioExpectation,
    get_basic_expectation,
    load_basic_scenario,
)
from cormc.p11_output import (
    ArtifactManifest,
    ArtifactManifestEntry,
    export_event_history,
    export_sanity_history,
    export_trajectory_history,
    render_time_space_png,
    write_artifact_manifest,
)
from cormc.simulation_loop import SimulationLoopConfig, SimulationLoopResult, run_deterministic_simulation
from cormc.step0_3 import DEFAULT_ROAD_GEOMETRY, SimulationState


BASIC_DEFAULT_MAX_STEPS = 900
BASIC_OUTPUT_ROOT = Path("artifacts/basic")


@dataclass(frozen=True)
class BasicScenarioRunResult:
    scenario_id: str
    run_id: str
    status: str
    scenario_dir: str
    simulation_result: SimulationLoopResult
    numeric_summary: Mapping[str, Any]
    numeric_summary_path: str
    scenario_report_path: str
    artifact_manifest_path: str

    def to_dict(self) -> dict[str, Any]:
        return {
            "scenario_id": self.scenario_id,
            "run_id": self.run_id,
            "status": self.status,
            "scenario_dir": self.scenario_dir,
            "numeric_summary": _to_plain(self.numeric_summary),
            "numeric_summary_path": self.numeric_summary_path,
            "scenario_report_path": self.scenario_report_path,
            "artifact_manifest_path": self.artifact_manifest_path,
        }


@dataclass(frozen=True)
class BasicSuiteRunResult:
    run_id: str
    output_dir: str
    scenario_results: tuple[BasicScenarioRunResult, ...]
    suite_summary: Mapping[str, Any]
    suite_summary_path: str
    suite_report_path: str
    artifact_manifest_path: str

    def to_dict(self) -> dict[str, Any]:
        return {
            "run_id": self.run_id,
            "output_dir": self.output_dir,
            "scenario_results": [result.to_dict() for result in self.scenario_results],
            "suite_summary": _to_plain(self.suite_summary),
            "suite_summary_path": self.suite_summary_path,
            "suite_report_path": self.suite_report_path,
            "artifact_manifest_path": self.artifact_manifest_path,
        }


def run_basic_numeric_suite(
    output_dir: str | Path = BASIC_OUTPUT_ROOT,
    *,
    run_id: str | None = None,
    max_steps: int = BASIC_DEFAULT_MAX_STEPS,
    render_png: bool = False,
    scenario_ids: Iterable[str] = BASIC_SCENARIO_IDS,
) -> BasicSuiteRunResult:
    run_id = run_id or _default_run_id()
    suite_dir = Path(output_dir) / run_id
    scenario_results = tuple(
        run_basic_numeric_scenario(
            scenario_id,
            output_dir=suite_dir / "scenarios",
            run_id=run_id,
            max_steps=max_steps,
            render_png=render_png,
        )
        for scenario_id in scenario_ids
    )
    suite_summary = _build_suite_summary(
        run_id=run_id,
        output_dir=suite_dir,
        scenario_results=scenario_results,
        max_steps=max_steps,
    )
    suite_summary_path = _write_json(suite_summary, suite_dir / "suite_summary.json")
    suite_report_path = _write_suite_report(
        suite_dir / "suite_report.md",
        run_id=run_id,
        suite_summary=suite_summary,
        scenario_results=scenario_results,
    )
    manifest_path = write_artifact_manifest(
        _build_suite_manifest(run_id=run_id, scenario_results=scenario_results),
        suite_dir / "artifact_manifest.json",
    )
    return BasicSuiteRunResult(
        run_id=run_id,
        output_dir=str(suite_dir),
        scenario_results=scenario_results,
        suite_summary=suite_summary,
        suite_summary_path=str(suite_summary_path),
        suite_report_path=str(suite_report_path),
        artifact_manifest_path=str(manifest_path),
    )


def run_basic_numeric_scenario(
    scenario_id: str,
    output_dir: str | Path,
    *,
    run_id: str,
    max_steps: int = BASIC_DEFAULT_MAX_STEPS,
    render_png: bool = False,
) -> BasicScenarioRunResult:
    config = load_basic_scenario(scenario_id)
    expectation = get_basic_expectation(scenario_id)
    simulation = run_deterministic_simulation(
        SimulationLoopConfig(
            scenario=config,
            run_id=run_id,
            max_steps=max_steps,
            stop_conditions=(_basic_merge_complete_condition(expectation.mv_id),),
            output_dir=Path(output_dir),
            render_png=False,
        )
    )
    scenario_dir = Path(output_dir) / scenario_id
    scenario_dir.mkdir(parents=True, exist_ok=True)
    artifact_paths = _write_scenario_artifacts(
        scenario_dir=scenario_dir,
        simulation=simulation,
        render_png=render_png,
    )
    summary = summarize_basic_numeric_result(
        simulation,
        expectation=expectation,
        max_steps=max_steps,
        artifact_paths=artifact_paths,
    )
    numeric_summary_path = _write_json(summary, scenario_dir / "numeric_summary.json")
    scenario_report_path = _write_scenario_report(
        scenario_dir / "scenario_report.md",
        summary=summary,
        artifact_paths=artifact_paths,
    )
    manifest_path = write_artifact_manifest(
        _build_scenario_manifest(
            run_id=run_id,
            scenario_id=scenario_id,
            status=str(summary["status"]),
            artifact_paths=artifact_paths,
            scenario_report_path=scenario_report_path,
        ),
        scenario_dir / "artifact_manifest.json",
    )
    return BasicScenarioRunResult(
        scenario_id=scenario_id,
        run_id=run_id,
        status=str(summary["status"]),
        scenario_dir=str(scenario_dir),
        simulation_result=simulation,
        numeric_summary=summary,
        numeric_summary_path=str(numeric_summary_path),
        scenario_report_path=str(scenario_report_path),
        artifact_manifest_path=str(manifest_path),
    )


def summarize_basic_numeric_result(
    simulation: SimulationLoopResult,
    *,
    expectation: BasicScenarioExpectation,
    max_steps: int,
    artifact_paths: Mapping[str, str] | None = None,
) -> dict[str, Any]:
    mv_id = expectation.mv_id
    events = simulation.history.event_dicts()
    sanity = simulation.history.sanity_dicts()
    first_control = _first_region_trace(simulation, mv_id, "control_zone")
    first_merge = _first_region_trace(simulation, mv_id, "merge_zone")
    first_aps = _first_aps(events, mv_id)
    observed_case = _payload_value(first_aps, "aps_case")
    active_cv_ids = _active_cv_ids(events, source_mv_id=mv_id)
    cuc_choice_timeline = _cuc_choice_timeline(events, active_cv_ids)
    illegal_cuc_choices = tuple(
        item
        for item in cuc_choice_timeline
        if item.get("final_choice") not in {"stay_lane_2", None}
    )
    eq10_consumers = _eq10_consumers(events, source_mv_id=mv_id)
    illegal_eq10_consumers = tuple(
        vehicle_id
        for vehicle_id in eq10_consumers
        if vehicle_id not in expectation.expected_eq10_consumer_ids
    )
    missing_eq10_consumers = tuple(
        vehicle_id
        for vehicle_id in expectation.expected_eq10_consumer_ids
        if vehicle_id not in eq10_consumers
    )
    assignment_validity = _event_timeline(events, "CMC", mv_id, "assignment_validation")
    aps_excluded_candidates = _aps_excluded_candidate_timeline(events, mv_id)
    aps_assignment_timeline = _aps_assignment_timeline(events, mv_id)
    aps_gap_protection_timeline = _aps_gap_protection_timeline(events, mv_id)
    cached_boundary_invalidations = _cached_boundary_invalidation_timeline(events, mv_id)
    eq53_timeline = _event_timeline(events, "CMC", mv_id, "eq53_gap")
    boundary_cap_timeline = _event_timeline(events, "CMC", mv_id, "boundary_speed_cap")
    merge_state_timeline = _merge_state_timeline(simulation, mv_id)
    merge_transition_timeline = _event_timeline(events, "CMC", mv_id, "merge_start")
    illegal_pre_control = _illegal_pre_control_events(simulation, mv_id)
    pre_control_suppressed_counts = {
        event_type: 0
        for event_type in ("APS", "assignment_cache", "cooperative_request", "CUC", "CMC")
    }
    failed_or_warning_sanity = _sanity_issues(sanity, scenario_id=expectation.scenario_id)
    diagnostic_issues = [
        *_gate_issues(
            simulation,
            expectation=expectation,
            first_control=first_control,
            first_aps=first_aps,
            observed_case=observed_case,
            active_cv_ids=active_cv_ids,
            illegal_cuc_choices=illegal_cuc_choices,
            eq10_consumers=eq10_consumers,
            illegal_eq10_consumers=illegal_eq10_consumers,
            missing_eq10_consumers=missing_eq10_consumers,
            illegal_pre_control=illegal_pre_control,
        ),
        *failed_or_warning_sanity,
    ]
    final_mv = simulation.final_state.vehicle_states.get(mv_id)
    merged_and_past_ramp = bool(
        final_mv is not None
        and final_mv.x_global > DEFAULT_ROAD_GEOMETRY.x_ramp_end_global
        and final_mv.merge_state in {"merged", "none"}
    )
    if not merged_and_past_ramp:
        diagnostic_issues.append(
            _issue(
                issue_id=f"{expectation.scenario_id}:outcome:not_merged_past_ramp",
                category="cmc_issue",
                severity="warning",
                step=simulation.final_state.step,
                t=simulation.final_state.t,
                vehicle_ids=[mv_id],
                message="MV did not finish as merged past x_ramp_end_global during this run.",
                evidence={
                    "final_x_global": final_mv.x_global if final_mv is not None else None,
                    "final_merge_state": final_mv.merge_state if final_mv is not None else None,
                    "simulation_status": simulation.status,
                },
                status="unresolved",
            )
        )
    resolved_issues = [issue for issue in diagnostic_issues if issue["status"] == "resolved"]
    unresolved_issues = [issue for issue in diagnostic_issues if issue["status"] != "resolved"]
    status = _scenario_status(
        unresolved_issues=unresolved_issues,
        merged_and_past_ramp=merged_and_past_ramp,
        simulation=simulation,
    )
    return {
        "scenario_id": expectation.scenario_id,
        "status": status,
        "max_steps": max_steps,
        "actual_steps": len(simulation.step_traces),
        "simulation_loop_status": simulation.status,
        "initial_vehicle_table": _initial_vehicle_table(simulation.initial_state),
        "first_control_zone_step": _optional_field(first_control, "step"),
        "first_control_zone_t": _optional_field(first_control, "t"),
        "first_control_zone_x_global": _optional_field(first_control, "x_global"),
        "first_merge_zone_step": _optional_field(first_merge, "step"),
        "first_merge_zone_t": _optional_field(first_merge, "t"),
        "first_merge_zone_x_global": _optional_field(first_merge, "x_global"),
        "pre_control_suppressed_module_counts": pre_control_suppressed_counts,
        "illegal_pre_control_module_events": illegal_pre_control,
        "first_aps": first_aps,
        "expected_aps_case": expectation.expected_aps_case,
        "observed_aps_case": observed_case,
        "clv_id": _payload_value(first_aps, "clv_id"),
        "cfv_id": _payload_value(first_aps, "cfv_id"),
        "active_cv_ids": list(active_cv_ids),
        "expected_active_cv_ids": list(expectation.expected_active_cv_ids),
        "cuc_choice_timeline": cuc_choice_timeline,
        "illegal_cuc_choices": list(illegal_cuc_choices),
        "eq10_consumers": list(eq10_consumers),
        "expected_eq10_consumer_ids": list(expectation.expected_eq10_consumer_ids),
        "illegal_eq10_consumers": list(illegal_eq10_consumers),
        "assignment_validity_timeline": assignment_validity,
        "aps_assignment_timeline": aps_assignment_timeline,
        "aps_gap_protection_timeline": aps_gap_protection_timeline,
        "aps_excluded_candidate_timeline": aps_excluded_candidates,
        "first_cached_boundary_invalidation": (
            cached_boundary_invalidations[0] if cached_boundary_invalidations else None
        ),
        "cached_boundary_invalidation_timeline": cached_boundary_invalidations,
        "eq53_timeline": eq53_timeline,
        "boundary_cap_timeline": boundary_cap_timeline,
        "merge_state_timeline": merge_state_timeline,
        "merge_transition_timeline": merge_transition_timeline,
        "final_mv_state": _vehicle_state_payload(final_mv),
        "merged_and_past_ramp": merged_and_past_ramp,
        "diagnostic_issues": diagnostic_issues,
        "resolved_issues": resolved_issues,
        "unresolved_issues": unresolved_issues,
        "artifact_paths": dict(artifact_paths or {}),
    }


def _basic_merge_complete_condition(mv_id: str):
    def stop(state: SimulationState) -> bool:
        vehicle = state.vehicle_states.get(mv_id)
        return bool(
            vehicle is not None
            and vehicle.x_global > DEFAULT_ROAD_GEOMETRY.x_ramp_end_global
            and vehicle.merge_state in {"merged", "none"}
        )

    return stop


def _write_scenario_artifacts(
    *,
    scenario_dir: Path,
    simulation: SimulationLoopResult,
    render_png: bool,
) -> dict[str, str]:
    trajectory_path = export_trajectory_history(simulation.history, scenario_dir / "trajectory.csv")
    events_path = export_event_history(simulation.history, scenario_dir / "events.jsonl")
    sanity_path = export_sanity_history(simulation.history, scenario_dir / "sanity.jsonl")
    artifact_paths = {
        "trajectory": str(trajectory_path),
        "events": str(events_path),
        "sanity": str(sanity_path),
    }
    if render_png:
        render = render_time_space_png(
            simulation.history.trajectory_records,
            simulation.expected_png_features,
            scenario_dir / "time_space.png",
            events=simulation.history.event_records,
        )
        artifact_paths["time_space_png"] = render.png_path
    return artifact_paths


def _first_region_trace(
    simulation: SimulationLoopResult,
    mv_id: str,
    region_name: str,
) -> dict[str, Any] | None:
    for trace in simulation.step_traces:
        region = trace.on_ramp_control_regions.get(mv_id)
        if region is None or region.region != region_name:
            continue
        state = trace.step0_3_result.state.vehicle_states[mv_id]
        return {
            "step": trace.step,
            "t": trace.t,
            "x_global": state.x_global,
            "region": region.region,
        }
    return None


def _first_aps(events: list[dict[str, Any]], mv_id: str) -> dict[str, Any] | None:
    for event in events:
        if event.get("event_type") != "APS":
            continue
        if event.get("vehicle_id") != mv_id:
            continue
        payload = event.get("payload") or {}
        if payload.get("failure") is True:
            return _event_summary(event)
        if payload.get("aps_case") is not None:
            return _event_summary(event)
    return None


def _active_cv_ids(events: list[dict[str, Any]], *, source_mv_id: str) -> tuple[str, ...]:
    ids: list[str] = []
    for event in events:
        if event.get("event_type") != "cooperative_request":
            continue
        payload = event.get("payload") or {}
        if payload.get("source_mv_id") != source_mv_id:
            continue
        cv_id = str(payload.get("cv_id") or event.get("vehicle_id"))
        if cv_id not in ids:
            ids.append(cv_id)
    return tuple(ids)


def _cuc_choice_timeline(
    events: list[dict[str, Any]],
    active_cv_ids: Iterable[str],
) -> list[dict[str, Any]]:
    active = set(active_cv_ids)
    timeline: list[dict[str, Any]] = []
    for event in events:
        if event.get("event_type") != "CUC":
            continue
        vehicle_id = event.get("vehicle_id")
        if vehicle_id not in active:
            continue
        payload = event.get("payload") or {}
        if payload.get("final_choice") is None:
            continue
        timeline.append(
            {
                "step": event.get("step"),
                "t": event.get("t"),
                "vehicle_id": vehicle_id,
                "source_mv_id": payload.get("source_mv_id"),
                "recommended_choice": payload.get("recommended_choice"),
                "effective_choice": payload.get("effective_choice"),
                "final_choice": payload.get("final_choice"),
                "target_lane_safe": payload.get("target_lane_safe"),
                "fallback_reason": payload.get("fallback_reason"),
            }
        )
    return timeline


def _eq10_consumers(events: list[dict[str, Any]], *, source_mv_id: str) -> tuple[str, ...]:
    consumers: list[str] = []
    for event in events:
        if event.get("event_type") != "spacing_override_consumption":
            continue
        payload = event.get("payload") or {}
        if payload.get("source_mv_id") != source_mv_id:
            continue
        vehicle_id = str(event.get("vehicle_id"))
        if vehicle_id not in consumers:
            consumers.append(vehicle_id)
    return tuple(consumers)


def _event_timeline(
    events: list[dict[str, Any]],
    event_type: str,
    vehicle_id: str,
    reason: str,
) -> list[dict[str, Any]]:
    return [
        _event_summary(event)
        for event in events
        if event.get("event_type") == event_type
        and event.get("vehicle_id") == vehicle_id
        and event.get("reason") == reason
    ]


def _aps_excluded_candidate_timeline(
    events: list[dict[str, Any]],
    mv_id: str,
) -> list[dict[str, Any]]:
    timeline: list[dict[str, Any]] = []
    for event in events:
        if event.get("event_type") != "APS_candidate" or event.get("vehicle_id") != mv_id:
            continue
        payload = event.get("payload") or {}
        excluded = payload.get("excluded_candidates") or []
        if not excluded:
            continue
        timeline.append(
            {
                "step": event.get("step"),
                "t": event.get("t"),
                "event_type": event.get("event_type"),
                "vehicle_id": event.get("vehicle_id"),
                "reason": event.get("reason"),
                "payload": {
                    "candidate_count": payload.get("candidate_count"),
                    "candidate_ids": list(payload.get("candidate_ids") or ()),
                    "excluded_candidates": [dict(item) for item in excluded],
                },
            }
        )
    return timeline


def _aps_assignment_timeline(
    events: list[dict[str, Any]],
    mv_id: str,
) -> list[dict[str, Any]]:
    timeline: list[dict[str, Any]] = []
    for event in events:
        if event.get("event_type") != "APS" or event.get("vehicle_id") != mv_id:
            continue
        payload = event.get("payload") or {}
        if payload.get("aps_case") is None:
            continue
        timeline.append(
            {
                "step": event.get("step"),
                "t": event.get("t"),
                "aps_case": payload.get("aps_case"),
                "d_star_clv": payload.get("d_star_clv"),
                "d_star_cfv": payload.get("d_star_cfv"),
                "t_star_mv": payload.get("t_star_mv"),
                "clv_id": payload.get("clv_id"),
                "cfv_id": payload.get("cfv_id"),
            }
        )
    return timeline


def _aps_gap_protection_timeline(
    events: list[dict[str, Any]],
    mv_id: str,
) -> list[dict[str, Any]]:
    timeline: list[dict[str, Any]] = []
    for event in events:
        if event.get("event_type") != "longitudinal_model" or event.get("vehicle_id") != mv_id:
            continue
        payload = event.get("payload") or {}
        if (
            payload.get("aps_gap_protection_applied") is not True
            and payload.get("aps_gap_protection_rejection_reason") is None
        ):
            continue
        current_speed = payload.get("current_speed")
        source_tau = payload.get("source_tau")
        timeline.append(
            {
                "step": event.get("step"),
                "t": event.get("t"),
                "current_speed": current_speed,
                "current_speed_times_tau": (
                    float(current_speed) * float(source_tau)
                    if current_speed is not None and source_tau is not None
                    else None
                ),
                "original_desired_speed": payload.get("original_desired_speed"),
                "effective_desired_speed": payload.get("effective_desired_speed"),
                "aps_gap_protection_applied": payload.get("aps_gap_protection_applied"),
                "aps_gap_protection_speed_cap": payload.get("aps_gap_protection_speed_cap"),
                "aps_gap_protection_source": payload.get("aps_gap_protection_source"),
                "source_aps_case": payload.get("source_aps_case"),
                "source_d_star_clv": payload.get("source_d_star_clv"),
                "source_tau": source_tau,
                "rejection_reason": payload.get("aps_gap_protection_rejection_reason"),
                "candidate_speed_after_lane_clip": payload.get("candidate_speed_after_lane_clip"),
            }
        )
    return timeline


def _cached_boundary_invalidation_timeline(
    events: list[dict[str, Any]],
    mv_id: str,
) -> list[dict[str, Any]]:
    timeline: list[dict[str, Any]] = []
    for event in events:
        if event.get("vehicle_id") != mv_id:
            continue
        if event.get("event_type") not in {"APS", "assignment_cache"}:
            continue
        payload = event.get("payload") or {}
        if (
            payload.get("old_cache_invalidated") is not True
            and payload.get("old_assignment_marked_recovery_required") is not True
        ):
            continue
        timeline.append(_event_summary(event))
    return timeline


def _merge_state_timeline(simulation: SimulationLoopResult, mv_id: str) -> list[dict[str, Any]]:
    timeline: list[dict[str, Any]] = []
    last_state: str | None = None
    for trace in simulation.step_traces:
        vehicle = trace.step0_3_result.state.vehicle_states.get(mv_id)
        if vehicle is None:
            continue
        if vehicle.merge_state == last_state:
            continue
        timeline.append(
            {
                "step": trace.step,
                "t": trace.t,
                "x_global": vehicle.x_global,
                "merge_state": vehicle.merge_state,
            }
        )
        last_state = vehicle.merge_state
    final = simulation.final_state.vehicle_states.get(mv_id)
    if final is not None and final.merge_state != last_state:
        timeline.append(
            {
                "step": simulation.final_state.step,
                "t": simulation.final_state.t,
                "x_global": final.x_global,
                "merge_state": final.merge_state,
            }
        )
    return timeline


def _illegal_pre_control_events(
    simulation: SimulationLoopResult,
    mv_id: str,
) -> list[dict[str, Any]]:
    illegal: list[dict[str, Any]] = []
    by_step_region = {
        trace.step: trace.on_ramp_control_regions.get(mv_id)
        for trace in simulation.step_traces
    }
    for event in simulation.history.event_dicts():
        region = by_step_region.get(int(event.get("step", -1)))
        if region is None or region.region != "pre_control":
            continue
        event_type = str(event.get("event_type"))
        if event_type == "APS" and event.get("vehicle_id") == mv_id:
            illegal.append(_event_summary(event))
        elif event_type in {"assignment_cache", "CMC"} and event.get("vehicle_id") == mv_id:
            illegal.append(_event_summary(event))
        elif event_type in {"cooperative_request", "CUC"}:
            related = set(event.get("vehicle_ids") or event.get("related_vehicle_ids") or ())
            if mv_id in related:
                illegal.append(_event_summary(event))
    return illegal


def _sanity_issues(
    sanity: list[dict[str, Any]],
    *,
    scenario_id: str,
) -> list[dict[str, Any]]:
    grouped: dict[tuple[Any, ...], dict[str, Any]] = {}
    for check in sanity:
        if check.get("result") not in {"fail", "warning"}:
            continue
        vehicle_ids = tuple(check.get("vehicle_ids") or ())
        key = (
            check.get("check_type"),
            check.get("result"),
            check.get("reason"),
            vehicle_ids,
        )
        if key not in grouped:
            grouped[key] = {
                "check": check,
                "count": 0,
                "first_step": check.get("step"),
                "first_t": check.get("t"),
                "last_step": check.get("step"),
                "last_t": check.get("t"),
            }
        grouped[key]["count"] += 1
        grouped[key]["last_step"] = check.get("step")
        grouped[key]["last_t"] = check.get("t")
    issues: list[dict[str, Any]] = []
    for index, item in enumerate(grouped.values()):
        check = item["check"]
        issues.append(
            _issue(
                issue_id=f"{scenario_id}:sanity:{index}",
                category="recording_issue",
                severity="warning" if check.get("result") == "warning" else "error",
                step=item["first_step"],
                t=item["first_t"],
                vehicle_ids=check.get("vehicle_ids") or [],
                message=f"{check.get('check_type')}={check.get('result')}: {check.get('reason')}",
                evidence={
                    "occurrence_count": item["count"],
                    "first_step": item["first_step"],
                    "first_t": item["first_t"],
                    "last_step": item["last_step"],
                    "last_t": item["last_t"],
                    "sample_payload": check.get("payload") or {},
                },
                status="unresolved",
            )
        )
    return issues


def _gate_issues(
    simulation: SimulationLoopResult,
    *,
    expectation: BasicScenarioExpectation,
    first_control: dict[str, Any] | None,
    first_aps: dict[str, Any] | None,
    observed_case: Any,
    active_cv_ids: tuple[str, ...],
    illegal_cuc_choices: tuple[Mapping[str, Any], ...],
    eq10_consumers: tuple[str, ...],
    illegal_eq10_consumers: tuple[str, ...],
    missing_eq10_consumers: tuple[str, ...],
    illegal_pre_control: list[dict[str, Any]],
) -> list[dict[str, Any]]:
    issues: list[dict[str, Any]] = []
    scenario_id = expectation.scenario_id
    mv_id = expectation.mv_id
    if illegal_pre_control:
        first = illegal_pre_control[0]
        issues.append(
            _issue(
                issue_id=f"{scenario_id}:region:illegal_pre_control",
                category="region_gating_issue",
                severity="error",
                step=first.get("step"),
                t=first.get("t"),
                vehicle_ids=[mv_id],
                message="Control module event appeared while MV was still in pre_control.",
                evidence={"events": illegal_pre_control},
                status="unresolved",
            )
        )
    elif (
        scenario_id in BASIC_PRE_CONTROL_IDS
        and first_aps is not None
        and first_control is not None
        and int(first_aps["step"]) >= int(first_control["step"])
    ):
        issues.append(
            _issue(
                issue_id=f"{scenario_id}:region:pre_control_suppression_fixed",
                category="region_gating_issue",
                severity="info",
                step=first_control.get("step"),
                t=first_control.get("t"),
                vehicle_ids=[mv_id],
                message="Pre-control module suppression held until MV entered control_zone.",
                evidence={
                    "first_control_zone": first_control,
                    "first_aps": first_aps,
                    "baseline_artifact": "artifacts/basic/region_gating_baseline.json",
                },
                status="resolved",
            )
        )
    if scenario_id in BASIC_PRE_CONTROL_IDS and first_aps is not None:
        first_control_step = _optional_field(first_control, "step")
        if first_control_step is None or int(first_aps["step"]) < int(first_control_step):
            issues.append(
                _issue(
                    issue_id=f"{scenario_id}:aps:before_control_zone",
                    category="region_gating_issue",
                    severity="error",
                    step=first_aps.get("step"),
                    t=first_aps.get("t"),
                    vehicle_ids=[mv_id],
                    message="First APS occurred before MV entered control_zone.",
                    evidence={"first_aps": first_aps, "first_control_zone": first_control},
                    status="unresolved",
                )
            )
    if scenario_id in BASIC_RAMP_PRE_IDS and (first_aps is None or int(first_aps["step"]) != 0):
        issues.append(
            _issue(
                issue_id=f"{scenario_id}:aps:not_step0",
                category="aps_issue",
                severity="error",
                step=_optional_field(first_aps, "step"),
                t=_optional_field(first_aps, "t"),
                vehicle_ids=[mv_id],
                message="ramp_pre BASIC scenario should run first APS at step 0.",
                evidence={"first_aps": first_aps},
                status="unresolved",
            )
        )
    if first_aps is None:
        issues.append(
            _issue(
                issue_id=f"{scenario_id}:aps:missing",
                category="aps_issue",
                severity="error",
                step=simulation.final_state.step,
                t=simulation.final_state.t,
                vehicle_ids=[mv_id],
                message="No effective APS event was observed.",
                evidence={},
                status="unresolved",
            )
        )
    elif observed_case != expectation.expected_aps_case:
        issues.append(
            _issue(
                issue_id=f"{scenario_id}:aps:case_mismatch",
                category="aps_issue",
                severity="error",
                step=first_aps.get("step"),
                t=first_aps.get("t"),
                vehicle_ids=[mv_id],
                message=f"Expected {expectation.expected_aps_case}, observed {observed_case}.",
                evidence={"first_aps": first_aps},
                status="unresolved",
            )
        )
    if set(active_cv_ids) != set(expectation.expected_active_cv_ids):
        issues.append(
            _issue(
                issue_id=f"{scenario_id}:cuc:active_cv_mismatch",
                category="cuc_issue",
                severity="error",
                step=_optional_field(first_aps, "step"),
                t=_optional_field(first_aps, "t"),
                vehicle_ids=[mv_id, *active_cv_ids],
                message="Active CV set does not match BASIC expectation.",
                evidence={
                    "expected_active_cv_ids": list(expectation.expected_active_cv_ids),
                    "observed_active_cv_ids": list(active_cv_ids),
                },
                status="unresolved",
            )
        )
    if illegal_cuc_choices:
        first = illegal_cuc_choices[0]
        issues.append(
            _issue(
                issue_id=f"{scenario_id}:cuc:choice_not_stay_lane_2",
                category="cuc_issue",
                severity="warning",
                step=first.get("step"),
                t=first.get("t"),
                vehicle_ids=[str(item.get("vehicle_id")) for item in illegal_cuc_choices],
                message="BASIC expected active CVs to stay in lane_2, but CUC selected another final choice.",
                evidence={"illegal_cuc_choices": list(illegal_cuc_choices)},
                status="unresolved",
            )
        )
    if illegal_eq10_consumers or missing_eq10_consumers:
        issues.append(
            _issue(
                issue_id=f"{scenario_id}:eq10:consumer_mismatch",
                category="cuc_issue",
                severity="error",
                step=_optional_field(first_aps, "step"),
                t=_optional_field(first_aps, "t"),
                vehicle_ids=[*eq10_consumers, *missing_eq10_consumers],
                message="Eq.10 consumer set does not match BASIC case semantics.",
                evidence={
                    "expected_eq10_consumer_ids": list(expectation.expected_eq10_consumer_ids),
                    "observed_eq10_consumers": list(eq10_consumers),
                    "illegal_eq10_consumers": list(illegal_eq10_consumers),
                    "missing_eq10_consumers": list(missing_eq10_consumers),
                },
                status="unresolved",
            )
        )
    return issues


def _scenario_status(
    *,
    unresolved_issues: list[dict[str, Any]],
    merged_and_past_ramp: bool,
    simulation: SimulationLoopResult,
) -> str:
    errors = [issue for issue in unresolved_issues if issue.get("severity") == "error"]
    if errors:
        return "failed"
    if merged_and_past_ramp:
        return "passed"
    if simulation.status == "stopped_by_condition":
        return "passed_with_diagnostics"
    return "diagnosed_unresolved"


def _build_suite_summary(
    *,
    run_id: str,
    output_dir: Path,
    scenario_results: tuple[BasicScenarioRunResult, ...],
    max_steps: int,
) -> dict[str, Any]:
    summaries = [dict(result.numeric_summary) for result in scenario_results]
    status_counts: dict[str, int] = {}
    for summary in summaries:
        status = str(summary["status"])
        status_counts[status] = status_counts.get(status, 0) + 1
    return {
        "run_id": run_id,
        "output_dir": str(output_dir),
        "max_steps": max_steps,
        "scenario_count": len(scenario_results),
        "status_counts": status_counts,
        "all_scenarios_have_artifacts": all(
            Path(result.numeric_summary_path).exists()
            and Path(result.scenario_report_path).exists()
            and Path(result.artifact_manifest_path).exists()
            for result in scenario_results
        ),
        "scenario_summaries": [
            {
                "scenario_id": summary["scenario_id"],
                "status": summary["status"],
                "actual_steps": summary["actual_steps"],
                "expected_aps_case": summary["expected_aps_case"],
                "observed_aps_case": summary["observed_aps_case"],
                "active_cv_ids": summary["active_cv_ids"],
                "eq10_consumers": summary["eq10_consumers"],
                "merged_and_past_ramp": summary["merged_and_past_ramp"],
                "unresolved_issue_count": len(summary["unresolved_issues"]),
                "primary_unresolved_issue": (
                    summary["unresolved_issues"][0]["message"]
                    if summary["unresolved_issues"]
                    else None
                ),
            }
            for summary in summaries
        ],
    }


def _write_scenario_report(
    path: Path,
    *,
    summary: Mapping[str, Any],
    artifact_paths: Mapping[str, str],
) -> Path:
    lines = [
        f"# {summary['scenario_id']} BASIC Numeric Diagnostic",
        "",
        f"- status: `{summary['status']}`",
        f"- actual_steps: `{summary['actual_steps']}` / max `{summary['max_steps']}`",
        f"- expected APS case: `{summary['expected_aps_case']}`",
        f"- observed APS case: `{summary.get('observed_aps_case')}`",
        f"- active CVs: `{', '.join(summary.get('active_cv_ids') or []) or 'none'}`",
        f"- Eq.10 consumers: `{', '.join(summary.get('eq10_consumers') or []) or 'none'}`",
        f"- merged past ramp: `{summary['merged_and_past_ramp']}`",
        "",
        "## Initial Vehicles",
        "",
        "| vehicle_id | lane | road_role | x_global | y | v | type |",
        "| --- | --- | --- | ---: | ---: | ---: | --- |",
    ]
    for vehicle in summary["initial_vehicle_table"]:
        lines.append(
            "| {vehicle_id} | {physical_lane} | {road_role} | {x_global} | {y} | {v} | {vehicle_type} |".format(
                **vehicle
            )
        )
    lines.extend(
        [
            "",
            "## Region",
            "",
            f"- first control-zone step: `{summary.get('first_control_zone_step')}` at x `{summary.get('first_control_zone_x_global')}`",
            f"- first merge-zone step: `{summary.get('first_merge_zone_step')}` at x `{summary.get('first_merge_zone_x_global')}`",
            f"- illegal pre-control events: `{len(summary.get('illegal_pre_control_module_events') or [])}`",
            "",
            "## Timelines",
            "",
            f"- first APS: `{_compact_event(summary.get('first_aps'))}`",
            f"- APS excluded candidates: `{_compact_excluded_candidates(summary.get('aps_excluded_candidate_timeline') or [])}`",
            f"- first cached boundary invalidation: `{_compact_event(summary.get('first_cached_boundary_invalidation'))}`",
            f"- CUC choices: `{_compact_timeline(summary.get('cuc_choice_timeline') or [], 'final_choice')}`",
            f"- assignment validity: `{_compact_timeline(summary.get('assignment_validity_timeline') or [], 'assignment_valid')}`",
            f"- assignment invalid reasons: `{_compact_timeline(summary.get('assignment_validity_timeline') or [], 'invalid_reason')}`",
            f"- Eq.53: `{_compact_timeline(summary.get('eq53_timeline') or [], 'eq53_pass')}`",
            f"- merge states: `{_compact_timeline(summary.get('merge_state_timeline') or [], 'merge_state')}`",
            "",
            "## Issues",
            "",
            "### Resolved",
            "",
        ]
    )
    resolved = summary.get("resolved_issues") or []
    if not resolved:
        lines.append("- none")
    else:
        lines.extend(f"- `{issue['issue_id']}`: {issue['message']}" for issue in resolved)
    lines.extend(["", "### Unresolved", ""])
    unresolved = summary.get("unresolved_issues") or []
    if not unresolved:
        lines.append("- none")
    else:
        lines.extend(
            f"- `{issue['issue_id']}` [{issue['category']}]: {issue['message']}"
            for issue in unresolved
        )
    lines.extend(["", "## Artifacts", ""])
    for name, artifact_path in artifact_paths.items():
        lines.append(f"- {name}: `{artifact_path}`")
    lines.append(f"- numeric_summary: `{path.with_name('numeric_summary.json')}`")
    _ensure_parent(path).write_text("\n".join(lines) + "\n", encoding="utf-8")
    return path


def _write_suite_report(
    path: Path,
    *,
    run_id: str,
    suite_summary: Mapping[str, Any],
    scenario_results: tuple[BasicScenarioRunResult, ...],
) -> Path:
    lines = [
        "# BASIC Numeric Suite",
        "",
        f"- run_id: `{run_id}`",
        f"- scenario_count: `{suite_summary['scenario_count']}`",
        f"- status_counts: `{suite_summary['status_counts']}`",
        "",
        "| scenario | status | case | active CVs | Eq.10 | merged | primary issue |",
        "| --- | --- | --- | --- | --- | --- | --- |",
    ]
    for result in scenario_results:
        summary = result.numeric_summary
        issue = (
            summary["unresolved_issues"][0]["message"]
            if summary["unresolved_issues"]
            else ""
        )
        lines.append(
            "| {scenario_id} | {status} | {observed}/{expected} | {active} | {eq10} | {merged} | {issue} |".format(
                scenario_id=summary["scenario_id"],
                status=summary["status"],
                observed=summary.get("observed_aps_case"),
                expected=summary["expected_aps_case"],
                active=", ".join(summary.get("active_cv_ids") or []) or "none",
                eq10=", ".join(summary.get("eq10_consumers") or []) or "none",
                merged=summary["merged_and_past_ramp"],
                issue=issue.replace("|", "/"),
            )
        )
    lines.extend(["", "## Artifacts", ""])
    for result in scenario_results:
        lines.append(f"- {result.scenario_id}: `{result.scenario_report_path}`")
    _ensure_parent(path).write_text("\n".join(lines) + "\n", encoding="utf-8")
    return path


def _build_scenario_manifest(
    *,
    run_id: str,
    scenario_id: str,
    status: str,
    artifact_paths: Mapping[str, str],
    scenario_report_path: Path,
) -> ArtifactManifest:
    return ArtifactManifest(
        run_id=run_id,
        entries=(
            ArtifactManifestEntry(
                scenario_id=scenario_id,
                run_id=run_id,
                status=status,
                input_config_ref=f"basic_scenario:{scenario_id}",
                exports=artifact_paths,
                scenario_report_path=str(scenario_report_path),
                human_summary_path=str(scenario_report_path),
            ),
        ),
    )


def _build_suite_manifest(
    *,
    run_id: str,
    scenario_results: tuple[BasicScenarioRunResult, ...],
) -> ArtifactManifest:
    return ArtifactManifest(
        run_id=run_id,
        entries=tuple(
            ArtifactManifestEntry(
                scenario_id=result.scenario_id,
                run_id=run_id,
                status=result.status,
                input_config_ref=f"basic_scenario:{result.scenario_id}",
                exports=dict(result.numeric_summary.get("artifact_paths") or {}),
                scenario_report_path=result.scenario_report_path,
                human_summary_path=result.scenario_report_path,
            )
            for result in scenario_results
        ),
    )


def _initial_vehicle_table(state: SimulationState) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for vehicle_id in state.active_vehicle_ids:
        vehicle = state.vehicle_states[vehicle_id]
        spec = state.vehicle_specs[vehicle_id]
        rows.append(
            {
                "vehicle_id": vehicle_id,
                "physical_lane": vehicle.physical_lane,
                "road_role": vehicle.road_role,
                "x_global": vehicle.x_global,
                "y": vehicle.y,
                "v": vehicle.v,
                "a": vehicle.a,
                "vehicle_type": spec.vehicle_type,
                "compliance_state": spec.compliance_state,
                "merge_state": vehicle.merge_state,
                "lane_change_state": vehicle.lane_change_state,
            }
        )
    return rows


def _event_summary(event: Mapping[str, Any]) -> dict[str, Any]:
    payload = dict(event.get("payload") or {})
    compact_keys = (
        "trigger",
        "failure",
        "aps_case",
        "clv_id",
        "cfv_id",
        "col_clv",
        "col_cfv",
        "desired_spacing_override",
        "t_star_mv",
        "d_star_clv",
        "d_star_cfv",
        "aps_min_merge_time_gap_s",
        "source_mv_id",
        "cv_id",
        "cv_role",
        "assignment_source",
        "assignment_valid",
        "assigned_clv_id",
        "assigned_cfv_id",
        "invalid_reason",
        "eq53_pass",
        "fail_side",
        "boundary_speed_cap",
        "cap_feasible",
        "cap_binding",
        "final_choice",
        "fallback_reason",
        "candidate_count",
        "excluded_candidates",
        "old_cache_invalidated",
        "old_assignment_marked_recovery_required",
        "lifecycle_state",
        "invalid_boundary_role",
        "invalid_boundary_id",
    )
    return {
        "step": event.get("step"),
        "t": event.get("t"),
        "event_type": event.get("event_type"),
        "module": event.get("module"),
        "vehicle_id": event.get("vehicle_id"),
        "reason": event.get("reason"),
        "payload": {
            key: payload[key]
            for key in compact_keys
            if key in payload
        },
    }


def _issue(
    *,
    issue_id: str,
    category: str,
    severity: str,
    step: Any,
    t: Any,
    vehicle_ids: Iterable[str],
    message: str,
    evidence: Mapping[str, Any],
    status: str,
) -> dict[str, Any]:
    return {
        "issue_id": issue_id,
        "category": category,
        "severity": severity,
        "step": step,
        "t": t,
        "vehicle_ids": list(vehicle_ids),
        "message": message,
        "evidence": dict(evidence),
        "status": status,
    }


def _payload_value(event: Mapping[str, Any] | None, key: str) -> Any:
    if event is None:
        return None
    return (event.get("payload") or {}).get(key)


def _optional_field(value: Mapping[str, Any] | None, key: str) -> Any:
    if value is None:
        return None
    return value.get(key)


def _vehicle_state_payload(vehicle: Any) -> dict[str, Any] | None:
    if vehicle is None:
        return None
    return {
        "vehicle_id": vehicle.vehicle_id,
        "x_global": vehicle.x_global,
        "y": vehicle.y,
        "v": vehicle.v,
        "a": vehicle.a,
        "physical_lane": vehicle.physical_lane,
        "road_role": vehicle.road_role,
        "lane_change_state": vehicle.lane_change_state,
        "merge_state": vehicle.merge_state,
    }


def _compact_event(event: Mapping[str, Any] | None) -> str:
    if event is None:
        return "none"
    bits = [f"step={event.get('step')}", f"reason={event.get('reason')}"]
    payload = event.get("payload") or {}
    for key in ("aps_case", "clv_id", "cfv_id", "assignment_valid", "eq53_pass"):
        if key in payload:
            bits.append(f"{key}={payload[key]}")
    return ", ".join(bits)


def _compact_timeline(timeline: Iterable[Mapping[str, Any]], key: str) -> str:
    items = []
    for item in timeline:
        value = item.get(key)
        if value is None:
            value = (item.get("payload") or {}).get(key)
        items.append(f"{item.get('step')}:{value}")
    return "; ".join(items) or "none"


def _compact_excluded_candidates(timeline: Iterable[Mapping[str, Any]]) -> str:
    items: list[str] = []
    for item in timeline:
        excluded = (item.get("payload") or {}).get("excluded_candidates") or []
        labels = [
            f"{candidate.get('vehicle_id')}({candidate.get('excluded_reason')})"
            for candidate in excluded
        ]
        if labels:
            items.append(f"{item.get('step')}:{','.join(labels)}")
    return "; ".join(items) or "none"


def _write_json(payload: Mapping[str, Any], path: Path) -> Path:
    output_path = _ensure_parent(path)
    output_path.write_text(
        json.dumps(_to_plain(payload), ensure_ascii=False, indent=2, sort_keys=True),
        encoding="utf-8",
    )
    return output_path


def _ensure_parent(path: str | Path) -> Path:
    output_path = Path(path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    return output_path


def _default_run_id() -> str:
    return datetime.now(UTC).strftime("basic_%Y%m%d_%H%M%S")


def _to_plain(value: Any) -> Any:
    if hasattr(value, "to_dict"):
        return value.to_dict()
    if isinstance(value, Mapping):
        return {str(key): _to_plain(item) for key, item in value.items()}
    if isinstance(value, tuple):
        return [_to_plain(item) for item in value]
    if isinstance(value, list):
        return [_to_plain(item) for item in value]
    return value
