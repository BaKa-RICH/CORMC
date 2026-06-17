from __future__ import annotations

import json
from collections import defaultdict
from dataclasses import dataclass
from datetime import UTC, datetime
from pathlib import Path
from typing import Any, Iterable, Mapping

from cormc.legacy.basic_numeric import (
    _active_cv_ids,
    _aps_assignment_timeline,
    _bounded_assignment_merge_outcome,
    _cached_boundary_invalidation_timeline,
    _compact_event,
    _compact_excluded_candidates,
    _compact_timeline,
    _cuc_choice_timeline,
    _eq10_consumers,
    _event_timeline,
    _first_aps,
    _first_region_trace,
    _initial_vehicle_table,
    _merge_state_timeline,
    _mv_longitudinal_relation_timeline,
    _sanity_issues,
    _to_plain,
    _vehicle_state_payload,
)
from cormc.legacy.artifact_reports import (
    ArtifactManifest,
    ArtifactManifestEntry,
    export_event_history,
    export_sanity_history,
    export_trajectory_history,
    render_time_space_png,
    write_artifact_manifest,
)
from cormc.scenes import (
    ROLLING_BASIC_EXPECTATIONS,
    ROLLING_BASIC_MV_IDS,
    ROLLING_BASIC_SCENARIO_ID,
    RollingBasicMVExpectation,
    get_rolling_basic_expectations,
    load_scene_config,
)
from cormc.simulation_core.loop import SimulationLoopConfig, SimulationLoopResult, run_deterministic_simulation
from cormc.simulation_core.pre_freeze import DEFAULT_ROAD_GEOMETRY, SimulationState


ROLLING_BASIC_DEFAULT_MAX_STEPS = 1200
ROLLING_BASIC_OUTPUT_ROOT = Path("artifacts/rolling_basic")
ROLLING_BASIC_EQ10_SPACING_MAX_M = 120.0


@dataclass(frozen=True)
class RollingBasicRunResult:
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


def run_rolling_basic_numeric_scenario(
    output_dir: str | Path = ROLLING_BASIC_OUTPUT_ROOT,
    *,
    run_id: str | None = None,
    max_steps: int = ROLLING_BASIC_DEFAULT_MAX_STEPS,
    render_png: bool = False,
) -> RollingBasicRunResult:
    run_id = run_id or _default_run_id()
    config = load_scene_config(ROLLING_BASIC_SCENARIO_ID)
    simulation = run_deterministic_simulation(
        SimulationLoopConfig(
            scenario=config,
            run_id=run_id,
            max_steps=max_steps,
            stop_conditions=(_rolling_basic_complete_condition(),),
            output_dir=Path(output_dir),
            render_png=False,
        )
    )
    scenario_dir = Path(output_dir) / run_id / ROLLING_BASIC_SCENARIO_ID
    scenario_dir.mkdir(parents=True, exist_ok=True)
    artifact_paths = _write_scenario_artifacts(
        scenario_dir=scenario_dir,
        simulation=simulation,
        render_png=render_png,
    )
    summary = summarize_rolling_basic_numeric_result(
        simulation,
        expectations=get_rolling_basic_expectations(),
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
        _build_manifest(
            run_id=run_id,
            status=str(summary["status"]),
            artifact_paths={
                **artifact_paths,
                "numeric_summary": str(numeric_summary_path),
                "scenario_report": str(scenario_report_path),
            },
            scenario_report_path=scenario_report_path,
        ),
        scenario_dir / "artifact_manifest.json",
    )
    return RollingBasicRunResult(
        scenario_id=ROLLING_BASIC_SCENARIO_ID,
        run_id=run_id,
        status=str(summary["status"]),
        scenario_dir=str(scenario_dir),
        simulation_result=simulation,
        numeric_summary=summary,
        numeric_summary_path=str(numeric_summary_path),
        scenario_report_path=str(scenario_report_path),
        artifact_manifest_path=str(manifest_path),
    )


def summarize_rolling_basic_numeric_result(
    simulation: SimulationLoopResult,
    *,
    expectations: Mapping[str, RollingBasicMVExpectation] = ROLLING_BASIC_EXPECTATIONS,
    max_steps: int,
    artifact_paths: Mapping[str, str] | None = None,
) -> dict[str, Any]:
    events = simulation.history.event_dicts()
    sanity = simulation.history.sanity_dicts()
    mv_summaries = {
        mv_id: _summarize_mv(simulation, events, expectation=expectations[mv_id])
        for mv_id in ROLLING_BASIC_MV_IDS
    }
    cross_mv_summary = _build_cross_mv_summary(simulation, events)
    bug_findings = _detect_bug_findings(
        simulation,
        events=events,
        mv_summaries=mv_summaries,
        cross_mv_summary=cross_mv_summary,
        expectations=expectations,
    )
    sanity_findings = [
        _finding_from_sanity_issue(issue, index=index)
        for index, issue in enumerate(_sanity_issues(sanity, scenario_id=ROLLING_BASIC_SCENARIO_ID))
    ]
    bug_findings.extend(sanity_findings)
    merged_all = all(summary["merged_and_past_ramp"] for summary in mv_summaries.values())
    status = _rolling_status(
        bug_findings=bug_findings,
        merged_all=merged_all,
        simulation=simulation,
    )
    artifacts = dict(artifact_paths or {})
    return {
        "scenario_id": ROLLING_BASIC_SCENARIO_ID,
        "status": status,
        "run_id": simulation.run_id,
        "max_steps": max_steps,
        "actual_steps": len(simulation.step_traces),
        "simulation_loop_status": simulation.status,
        "dt": simulation.initial_state.dt,
        "initial_vehicle_table": _initial_vehicle_table(simulation.initial_state),
        "mv_summaries": mv_summaries,
        "cross_mv_summary": cross_mv_summary,
        "bug_findings": bug_findings,
        "bug_finding_count": len(bug_findings),
        "artifact_paths": artifacts,
    }


def _summarize_mv(
    simulation: SimulationLoopResult,
    events: list[dict[str, Any]],
    *,
    expectation: RollingBasicMVExpectation,
) -> dict[str, Any]:
    mv_id = expectation.mv_id
    first_control = _first_region_trace(simulation, mv_id, "control_zone")
    first_merge = _first_region_trace(simulation, mv_id, "merge_zone")
    first_aps = _first_aps(events, mv_id)
    expected_clv_id = expectation.expected_clv_id
    expected_cfv_id = expectation.expected_cfv_id
    active_cv_ids = _active_cv_ids(events, source_mv_id=mv_id)
    cuc_choice_timeline = _cuc_choice_timeline(events, active_cv_ids)
    eq10_consumers = _eq10_consumers(events, source_mv_id=mv_id)
    assignment_validity = _event_timeline(events, "CMC", mv_id, "assignment_validation")
    aps_candidate_timeline = _aps_candidate_timeline(events, mv_id)
    aps_assignment_timeline = _aps_assignment_timeline(events, mv_id)
    cached_boundary_invalidations = _cached_boundary_invalidation_timeline(events, mv_id)
    eq53_timeline = _event_timeline(events, "CMC", mv_id, "eq53_gap")
    boundary_cap_timeline = _event_timeline(events, "CMC", mv_id, "boundary_speed_cap")
    merge_state_timeline = _merge_state_timeline(simulation, mv_id)
    merge_transition_timeline = _event_timeline(events, "CMC", mv_id, "merge_start")
    final_mv = simulation.final_state.vehicle_states.get(mv_id)
    merged_and_past_ramp = bool(
        final_mv is not None
        and final_mv.x_global > DEFAULT_ROAD_GEOMETRY.x_ramp_end_global
        and final_mv.merge_state in {"merged", "none"}
    )
    observed_case = _payload_value(first_aps, "aps_case")
    observed_clv = _payload_value(first_aps, "clv_id")
    observed_cfv = _payload_value(first_aps, "cfv_id")
    bounded_merge = _bounded_assignment_merge_outcome(
        events,
        mv_id,
        expected_clv_id=expected_clv_id,
        expected_cfv_id=expected_cfv_id,
    )
    return {
        "mv_id": mv_id,
        "expectation": expectation.to_dict(),
        "first_control_zone": first_control,
        "first_control_zone_step": _optional_field(first_control, "step"),
        "first_control_zone_t": _optional_field(first_control, "t"),
        "first_control_zone_x_global": _optional_field(first_control, "x_global"),
        "first_merge_zone": first_merge,
        "first_merge_zone_step": _optional_field(first_merge, "step"),
        "first_merge_zone_t": _optional_field(first_merge, "t"),
        "first_merge_zone_x_global": _optional_field(first_merge, "x_global"),
        "first_aps": first_aps,
        "expected_aps_case": expectation.expected_aps_case,
        "observed_aps_case": observed_case,
        "expected_clv_id": expected_clv_id,
        "expected_cfv_id": expected_cfv_id,
        "observed_clv_id": observed_clv,
        "observed_cfv_id": observed_cfv,
        "aps_candidate_timeline": aps_candidate_timeline,
        "aps_assignment_timeline": aps_assignment_timeline,
        "active_cv_ids": list(active_cv_ids),
        "expected_active_cv_ids": list(expectation.expected_active_cv_ids),
        "eq10_consumers": list(eq10_consumers),
        "expected_eq10_consumer_ids": list(expectation.expected_eq10_consumer_ids),
        "cuc_choice_timeline": cuc_choice_timeline,
        "assignment_record_timeline": _assignment_record_timeline(simulation, mv_id),
        "assignment_cache_update_timeline": _assignment_cache_update_timeline(events, mv_id),
        "assignment_validation_timeline": assignment_validity,
        "cached_boundary_invalidation_timeline": cached_boundary_invalidations,
        "eq53_timeline": eq53_timeline,
        "boundary_cap_timeline": boundary_cap_timeline,
        "mv_longitudinal_relation_timeline": _mv_longitudinal_relation_timeline(events, mv_id),
        "merge_state_timeline": merge_state_timeline,
        "merge_transition_timeline": merge_transition_timeline,
        **bounded_merge,
        "final_mv_state": _vehicle_state_payload(final_mv),
        "merged_and_past_ramp": merged_and_past_ramp,
        "per_mv_issues": [],
    }


def _build_cross_mv_summary(
    simulation: SimulationLoopResult,
    events: list[dict[str, Any]],
) -> dict[str, Any]:
    active_assignments = _per_step_active_assignments_by_mv(simulation)
    shared_cv_timeline = _shared_cv_timeline(active_assignments)
    conflict_resolution = _conflict_resolution_timeline(events)
    suppressed = _suppressed_request_timeline(events)
    multi_aps = _same_step_multi_mv_events(events, "APS")
    multi_cmc = _same_step_multi_mv_events(events, "CMC")
    multi_cuc = _same_step_multi_source_cuc(events)
    return {
        "per_step_active_assignments_by_mv": active_assignments,
        "shared_cv_timeline": shared_cv_timeline,
        "step5_conflict_resolution_timeline": conflict_resolution,
        "suppressed_cooperative_request_timeline": suppressed,
        "same_step_multi_mv_aps_timeline": multi_aps,
        "same_step_multi_mv_cmc_timeline": multi_cmc,
        "same_step_multi_mv_cuc_timeline": multi_cuc,
    }


def _detect_bug_findings(
    simulation: SimulationLoopResult,
    *,
    events: list[dict[str, Any]],
    mv_summaries: Mapping[str, Mapping[str, Any]],
    cross_mv_summary: Mapping[str, Any],
    expectations: Mapping[str, RollingBasicMVExpectation],
) -> list[dict[str, Any]]:
    findings: list[dict[str, Any]] = []
    for mv_id in ROLLING_BASIC_MV_IDS:
        summary = mv_summaries[mv_id]
        expectation = expectations[mv_id]
        first_control = summary.get("first_control_zone")
        first_aps = summary.get("first_aps")
        first_aps_step = _optional_field(first_aps, "step")
        first_control_step = _optional_field(first_control, "step")
        if first_control is None:
            final = summary.get("final_mv_state") or {}
            findings.append(
                _bug_finding(
                    bug_id="RB-APS-001",
                    category="aps_issue",
                    severity="error",
                    title="MV never reached control zone, so first APS could not be evaluated.",
                    affected_mv_ids=[mv_id],
                    affected_vehicle_ids=[mv_id],
                    first_step=simulation.final_state.step,
                    first_t=simulation.final_state.t,
                    symptom="No control-zone trace was recorded for the MV.",
                    expected="Every rolling MV should reach control_zone and trigger its first APS immediately.",
                    observed=f"final_x_global={final.get('x_global')} merge_state={final.get('merge_state')}",
                    evidence_refs=[_trajectory_ref(simulation.final_state.step, mv_id)],
                    evidence_payload={"final_mv_state": final},
                    status="observed",
                )
            )
        elif first_aps is None or first_aps_step != first_control_step:
            evidence_refs = [_trajectory_ref(first_control_step, mv_id)]
            if first_aps is not None:
                evidence_refs.append(_event_ref(first_aps))
            findings.append(
                _bug_finding(
                    bug_id="RB-APS-001",
                    category="aps_issue",
                    severity="error",
                    title="First control-zone step did not have an immediate effective APS.",
                    affected_mv_ids=[mv_id],
                    affected_vehicle_ids=[mv_id],
                    first_step=first_control_step,
                    first_t=_optional_field(first_control, "t"),
                    symptom="The first APS event is missing or delayed relative to first control-zone entry.",
                    expected=f"first APS step == first control-zone step ({first_control_step}).",
                    observed=f"first APS step == {first_aps_step}.",
                    evidence_refs=evidence_refs,
                    evidence_payload={"first_control_zone": first_control, "first_aps": first_aps},
                    status="observed",
                )
            )

        observed_clv = summary.get("observed_clv_id")
        observed_cfv = summary.get("observed_cfv_id")
        if first_aps is not None and (
            observed_clv != expectation.expected_clv_id
            or observed_cfv != expectation.expected_cfv_id
        ):
            findings.append(
                _bug_finding(
                    bug_id="RB-APS-002",
                    category="aps_issue",
                    severity="error",
                    title="First APS chose a CLV/CFV pair outside this rolling unit's expectation.",
                    affected_mv_ids=[mv_id],
                    affected_vehicle_ids=_non_empty(
                        [mv_id, expectation.expected_clv_id, expectation.expected_cfv_id, observed_clv, observed_cfv]
                    ),
                    first_step=first_aps.get("step"),
                    first_t=first_aps.get("t"),
                    symptom="Observed APS pair differs from the documented RBxx pair.",
                    expected=f"CLV={expectation.expected_clv_id}, CFV={expectation.expected_cfv_id}.",
                    observed=f"CLV={observed_clv}, CFV={observed_cfv}.",
                    evidence_refs=[_event_ref(first_aps)],
                    evidence_payload={"first_aps": first_aps},
                    status="observed",
                )
            )

        observed_case = summary.get("observed_aps_case")
        if first_aps is not None and observed_case != expectation.expected_aps_case:
            if not _case_mismatch_tolerated(summary, expectation, events, mv_id):
                findings.append(
                    _bug_finding(
                        bug_id="RB-APS-003",
                        category="aps_issue",
                        severity="error",
                        title="Observed first APS case does not match the rolling expectation.",
                        affected_mv_ids=[mv_id],
                        affected_vehicle_ids=[mv_id],
                        first_step=first_aps.get("step"),
                        first_t=first_aps.get("t"),
                        symptom="APS case mismatch at first effective APS.",
                        expected=expectation.expected_aps_case,
                        observed=str(observed_case),
                        evidence_refs=[_event_ref(first_aps)],
                        evidence_payload={"first_aps": first_aps},
                        status="observed",
                    )
                )

        candidate_issue = _cross_unit_candidate_issue(summary, expectation)
        if candidate_issue is not None:
            findings.append(candidate_issue)

        illegal_cuc = [
            item
            for item in summary.get("cuc_choice_timeline") or []
            if item.get("final_choice") not in {"stay_lane_2", None}
        ]
        if illegal_cuc:
            first = illegal_cuc[0]
            findings.append(
                _bug_finding(
                    bug_id="RB-CUC-001",
                    category="cuc_issue",
                    severity="warning",
                    title="Active CV final CUC choice was not stay_lane_2.",
                    affected_mv_ids=[mv_id],
                    affected_vehicle_ids=_unique([str(item.get("vehicle_id")) for item in illegal_cuc]),
                    first_step=first.get("step"),
                    first_t=first.get("t"),
                    symptom="A rolling active CV changed away from lane 2 during the diagnostic window.",
                    expected="All active CVs stay_lane_2 for this BASIC diagnostic probe.",
                    observed=f"{first.get('vehicle_id')} final_choice={first.get('final_choice')}",
                    evidence_refs=[
                        _event_ref_from_values(first.get("step"), "CUC", first.get("vehicle_id"))
                    ],
                    evidence_payload={"illegal_cuc_choices": illegal_cuc},
                    status="observed",
                )
            )

        eq10_consumers = set(summary.get("eq10_consumers") or [])
        expected_eq10 = set(expectation.expected_eq10_consumer_ids)
        eq10_value_issue = _eq10_value_issue(summary, expectation, events, mv_id)
        if eq10_value_issue is not None:
            findings.append(eq10_value_issue)
        if (
            eq10_consumers != expected_eq10
            and not _eq10_mismatch_tolerated(summary, expectation, events, mv_id)
        ):
            refs = [
                _event_ref_from_values(item.get("step"), "spacing_override_consumption", item.get("vehicle_id"))
                for item in _eq10_consumption_events(events, mv_id)
            ]
            if not refs and first_aps is not None:
                refs = [_event_ref(first_aps)]
            if not refs:
                ref_step = summary.get("first_control_zone_step")
                if ref_step is None:
                    ref_step = simulation.final_state.step
                refs = [_trajectory_ref(ref_step, mv_id)]
            findings.append(
                _bug_finding(
                    bug_id="RB-EQ10-001",
                    category="eq10_issue",
                    severity="error",
                    title="Eq.10 consumer set does not match the rolling case expectation.",
                    affected_mv_ids=[mv_id],
                    affected_vehicle_ids=_unique([*eq10_consumers, *expected_eq10]),
                    first_step=_optional_field(first_aps, "step"),
                    first_t=_optional_field(first_aps, "t"),
                    symptom="Eq.10 spacing override was missing, consumed by the wrong vehicle, or crossed rolling units.",
                    expected=f"Eq.10 consumers={sorted(expected_eq10)}",
                    observed=f"Eq.10 consumers={sorted(eq10_consumers)}",
                    evidence_refs=refs,
                    evidence_payload={
                        "expected_eq10_consumer_ids": sorted(expected_eq10),
                        "observed_eq10_consumers": sorted(eq10_consumers),
                    },
                    status="observed",
                )
            )

        owner_mismatch = _assignment_owner_mismatch(summary)
        if owner_mismatch is not None:
            findings.append(owner_mismatch)

        invalid_entered_merge = _invalid_assignment_entered_merge(summary)
        if invalid_entered_merge is not None:
            findings.append(invalid_entered_merge)

        cmc_no_request = _merge_zone_no_active_request(summary, events)
        if cmc_no_request is not None:
            findings.append(cmc_no_request)

        if not bool(summary.get("merged_and_past_ramp")):
            final_state = summary.get("final_mv_state") or {}
            findings.append(
                _bug_finding(
                    bug_id="RB-CMC-002",
                    category="cmc_issue",
                    severity="error",
                    title="MV did not merge and pass the ramp end within the numeric run.",
                    affected_mv_ids=[mv_id],
                    affected_vehicle_ids=[mv_id],
                    first_step=simulation.final_state.step,
                    first_t=simulation.final_state.t,
                    symptom="Run ended before this MV reached x_global > 7250 with merged/none state.",
                    expected="x_global > 7250 and merge_state in {merged, none}.",
                    observed=f"x_global={final_state.get('x_global')} merge_state={final_state.get('merge_state')}",
                    evidence_refs=[_trajectory_ref(simulation.final_state.step, mv_id)],
                    evidence_payload={"final_mv_state": final_state},
                    status="observed",
                )
            )

        cache_issue = _cache_lifecycle_issue(summary)
        if cache_issue is not None:
            findings.append(cache_issue)

    shared = cross_mv_summary.get("shared_cv_timeline") or []
    conflicts = cross_mv_summary.get("step5_conflict_resolution_timeline") or []
    if shared:
        first = shared[0]
        status = "observed" if conflicts else "inferred_from_data"
        findings.append(
            _bug_finding(
                bug_id="RB-ASG-002",
                category="cross_mv_assignment_issue",
                severity="error" if conflicts else "warning",
                title="The same CV was referenced by multiple rolling MV assignments.",
                affected_mv_ids=_unique(first.get("mv_ids") or []),
                affected_vehicle_ids=[str(first.get("cv_id"))],
                first_step=first.get("step"),
                first_t=first.get("t"),
                symptom="A CV appeared in more than one active assignment in the same step.",
                expected="Each rolling MV should keep its own CLV/CFV pair without shared active CV ownership.",
                observed=f"cv_id={first.get('cv_id')} mv_ids={first.get('mv_ids')}",
                evidence_refs=[
                    _trajectory_ref(first.get("step"), str(first.get("cv_id"))),
                ],
                evidence_payload={"first_shared_cv": first, "conflict_resolution_events": conflicts[:5]},
                status=status,
            )
        )
    elif conflicts:
        first = conflicts[0]
        findings.append(
            _bug_finding(
                bug_id="RB-ASG-002",
                category="cross_mv_assignment_issue",
                severity="error",
                title="Step5 conflict resolution was triggered in the rolling scenario.",
                affected_mv_ids=_unique([first.get("winner_mv_id"), *(first.get("loser_mv_ids") or [])]),
                affected_vehicle_ids=[str(first.get("cv_id"))],
                first_step=first.get("step"),
                first_t=first.get("t"),
                symptom="Step5 had to suppress at least one cooperative request for a shared CV.",
                expected="No rolling unit should require shared-CV conflict resolution.",
                observed=f"winner={first.get('winner_mv_id')} losers={first.get('loser_mv_ids')} cv={first.get('cv_id')}",
                evidence_refs=[_event_ref_from_values(first.get("step"), "conflict_resolution", first.get("cv_id"))],
                evidence_payload={"first_conflict_resolution": first},
                status="observed",
            )
        )
    return _dedupe_findings(findings)


def _rolling_basic_complete_condition():
    def stop(state: SimulationState) -> bool:
        for mv_id in ROLLING_BASIC_MV_IDS:
            vehicle = state.vehicle_states.get(mv_id)
            if not (
                vehicle is not None
                and vehicle.x_global > DEFAULT_ROAD_GEOMETRY.x_ramp_end_global
                and vehicle.merge_state in {"merged", "none"}
            ):
                return False
        return True

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


def _aps_candidate_timeline(events: list[dict[str, Any]], mv_id: str) -> list[dict[str, Any]]:
    timeline: list[dict[str, Any]] = []
    for event in events:
        if event.get("event_type") != "APS_candidate" or event.get("vehicle_id") != mv_id:
            continue
        payload = event.get("payload") or {}
        window = payload.get("candidate_window") or {}
        timeline.append(
            {
                "step": event.get("step"),
                "t": event.get("t"),
                "event_type": event.get("event_type"),
                "vehicle_id": event.get("vehicle_id"),
                "reason": event.get("reason"),
                "candidate_ids": list(payload.get("candidate_ids") or ()),
                "candidate_count": payload.get("candidate_count"),
                "excluded_candidates": [dict(item) for item in payload.get("excluded_candidates") or ()],
                "window_start_x_global": window.get("start_x_global"),
                "window_end_x_global": window.get("end_x_global"),
                "x_mv_global": window.get("x_mv_global"),
                "l_cr": window.get("l_cr"),
            }
        )
    return timeline


def _assignment_record_timeline(
    simulation: SimulationLoopResult,
    mv_id: str,
) -> list[dict[str, Any]]:
    timeline: list[dict[str, Any]] = []
    last_signature: str | None = None
    for trace in simulation.step_traces:
        record = trace.step0_3_result.state.assignment_records_by_mv.get(mv_id)
        signature = json.dumps(_to_plain(record), sort_keys=True) if record is not None else None
        if signature == last_signature:
            continue
        state = trace.step0_3_result.state.vehicle_states.get(mv_id)
        timeline.append(
            {
                "step": trace.step,
                "t": trace.t,
                "mv_x_global": state.x_global if state is not None else None,
                "record": dict(record) if record is not None else None,
            }
        )
        last_signature = signature
    return timeline


def _assignment_cache_update_timeline(
    events: list[dict[str, Any]],
    mv_id: str,
) -> list[dict[str, Any]]:
    return [
        _compact_assignment_cache_event(event)
        for event in events
        if event.get("event_type") == "assignment_cache" and event.get("vehicle_id") == mv_id
    ]


def _compact_assignment_cache_event(event: Mapping[str, Any]) -> dict[str, Any]:
    payload = event.get("payload") or {}
    update = payload.get("update_request") or {}
    return {
        "step": event.get("step"),
        "t": event.get("t"),
        "event_type": event.get("event_type"),
        "vehicle_id": event.get("vehicle_id"),
        "reason": event.get("reason"),
        "payload": {
            "action": payload.get("action"),
            "previous_cache_exists": payload.get("previous_cache_exists"),
            "new_assignment_created": payload.get("new_assignment_created"),
            "old_cache_invalidated": payload.get("old_cache_invalidated"),
            "invalid_boundary_role": payload.get("invalid_boundary_role"),
            "invalid_boundary_id": payload.get("invalid_boundary_id"),
            "invalid_reason": payload.get("invalid_reason"),
            "update_request": {
                key: update.get(key)
                for key in (
                    "mv_id",
                    "clv_id",
                    "cfv_id",
                    "aps_case",
                    "status",
                    "lifecycle_state",
                    "source",
                    "invalid_reason",
                    "recovery_reason",
                )
                if key in update
            },
        },
    }


def _per_step_active_assignments_by_mv(simulation: SimulationLoopResult) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for trace in simulation.step_traces:
        assignments: dict[str, dict[str, Any]] = {}
        for mv_id in ROLLING_BASIC_MV_IDS:
            record = trace.step0_3_result.state.assignment_records_by_mv.get(mv_id)
            if record is None:
                continue
            if str(record.get("status", "valid")).lower() not in {"valid", "available", "ok"}:
                continue
            assignments[mv_id] = {
                "mv_id": record.get("mv_id"),
                "clv_id": record.get("clv_id"),
                "cfv_id": record.get("cfv_id"),
                "aps_case": record.get("aps_case"),
                "lifecycle_state": record.get("lifecycle_state"),
                "status": record.get("status"),
                "source": record.get("source"),
            }
        if assignments:
            rows.append(
                {
                    "step": trace.step,
                    "t": trace.t,
                    "assignments": assignments,
                }
            )
    return rows


def _shared_cv_timeline(active_assignments: Iterable[Mapping[str, Any]]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for item in active_assignments:
        by_cv: dict[str, list[dict[str, Any]]] = defaultdict(list)
        assignments = item.get("assignments") or {}
        for owner_mv_id, record in assignments.items():
            for role in ("clv_id", "cfv_id"):
                cv_id = record.get(role)
                if cv_id is None:
                    continue
                by_cv[str(cv_id)].append(
                    {
                        "owner_mv_id": owner_mv_id,
                        "payload_mv_id": record.get("mv_id"),
                        "role": role.removesuffix("_id"),
                        "aps_case": record.get("aps_case"),
                        "lifecycle_state": record.get("lifecycle_state"),
                    }
                )
        for cv_id, refs in by_cv.items():
            mv_ids = _unique(ref.get("owner_mv_id") for ref in refs)
            if len(mv_ids) <= 1:
                continue
            rows.append(
                {
                    "step": item.get("step"),
                    "t": item.get("t"),
                    "cv_id": cv_id,
                    "mv_ids": mv_ids,
                    "references": refs,
                }
            )
    return rows


def _conflict_resolution_timeline(events: list[dict[str, Any]]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for event in events:
        if event.get("event_type") != "conflict_resolution":
            continue
        payload = event.get("payload") or {}
        if payload.get("winner_mv_id") is None:
            continue
        rows.append(
            {
                "step": event.get("step"),
                "t": event.get("t"),
                "event_type": event.get("event_type"),
                "cv_id": payload.get("cv_id") or event.get("vehicle_id"),
                "winner_mv_id": payload.get("winner_mv_id"),
                "loser_mv_ids": list(payload.get("loser_mv_ids") or ()),
                "winner_request_id": payload.get("winner_request_id"),
                "loser_request_ids": list(payload.get("loser_request_ids") or ()),
                "priority_basis": payload.get("priority_basis") or event.get("reason"),
                "request_ids": list(payload.get("request_ids") or ()),
            }
        )
    return rows


def _suppressed_request_timeline(events: list[dict[str, Any]]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for event in events:
        if event.get("event_type") != "conflict_resolution":
            continue
        payload = event.get("payload") or {}
        if payload.get("suppressed_by_request_id") is None:
            continue
        rows.append(
            {
                "step": event.get("step"),
                "t": event.get("t"),
                "event_type": event.get("event_type"),
                "cv_id": payload.get("cv_id") or event.get("vehicle_id"),
                "source_mv_id": payload.get("source_mv_id"),
                "suppressed_by_request_id": payload.get("suppressed_by_request_id"),
                "suppressed_reason": payload.get("suppressed_reason") or event.get("reason"),
                "conflict_id": payload.get("conflict_id"),
            }
        )
    return rows


def _same_step_multi_mv_events(
    events: list[dict[str, Any]],
    event_type: str,
) -> list[dict[str, Any]]:
    grouped: dict[Any, list[dict[str, Any]]] = defaultdict(list)
    for event in events:
        if event.get("event_type") != event_type:
            continue
        vehicle_id = event.get("vehicle_id")
        if vehicle_id not in ROLLING_BASIC_MV_IDS:
            continue
        grouped[event.get("step")].append(event)
    rows: list[dict[str, Any]] = []
    for step, group in sorted(grouped.items()):
        mv_ids = _unique(event.get("vehicle_id") for event in group)
        if len(mv_ids) <= 1:
            continue
        rows.append(
            {
                "step": step,
                "t": group[0].get("t"),
                "event_type": event_type,
                "mv_ids": mv_ids,
                "events": [_event_minimal(event) for event in group],
            }
        )
    return rows


def _same_step_multi_source_cuc(events: list[dict[str, Any]]) -> list[dict[str, Any]]:
    grouped: dict[Any, list[dict[str, Any]]] = defaultdict(list)
    for event in events:
        if event.get("event_type") != "CUC":
            continue
        payload = event.get("payload") or {}
        source_mv_id = payload.get("source_mv_id")
        if source_mv_id in ROLLING_BASIC_MV_IDS:
            grouped[event.get("step")].append(event)
    rows: list[dict[str, Any]] = []
    for step, group in sorted(grouped.items()):
        mv_ids = _unique((event.get("payload") or {}).get("source_mv_id") for event in group)
        if len(mv_ids) <= 1:
            continue
        rows.append(
            {
                "step": step,
                "t": group[0].get("t"),
                "mv_ids": mv_ids,
                "events": [_event_minimal(event) for event in group],
            }
        )
    return rows


def _event_minimal(event: Mapping[str, Any]) -> dict[str, Any]:
    payload = event.get("payload") or {}
    return {
        "event_type": event.get("event_type"),
        "vehicle_id": event.get("vehicle_id"),
        "reason": event.get("reason"),
        "source_mv_id": payload.get("source_mv_id"),
        "final_choice": payload.get("final_choice"),
        "aps_case": payload.get("aps_case"),
        "assignment_valid": payload.get("assignment_valid"),
        "invalid_reason": payload.get("invalid_reason"),
    }


def _cross_unit_candidate_issue(
    summary: Mapping[str, Any],
    expectation: RollingBasicMVExpectation,
) -> dict[str, Any] | None:
    observed_clv = summary.get("observed_clv_id")
    observed_cfv = summary.get("observed_cfv_id")
    expected_prefix = expectation.mv_id.removesuffix("_MV")
    selected = _non_empty([observed_clv, observed_cfv])
    selected_cross = [
        vehicle_id
        for vehicle_id in selected
        if not vehicle_id.startswith(expected_prefix)
    ]
    if not selected_cross:
        return None
    first_aps = summary.get("first_aps")
    candidate = (summary.get("aps_candidate_timeline") or [{}])[0]
    return _bug_finding(
        bug_id="RB-APS-004",
        category="aps_issue",
        severity="error",
        title="APS selected a vehicle from another rolling unit.",
        affected_mv_ids=[expectation.mv_id],
        affected_vehicle_ids=_unique([expectation.mv_id, *selected_cross]),
        first_step=_optional_field(first_aps, "step"),
        first_t=_optional_field(first_aps, "t"),
        symptom="APS candidate window overlap led to a cross-unit CLV/CFV pair.",
        expected=f"First APS pair should stay within prefix {expected_prefix}.",
        observed=f"selected_cross_unit_ids={selected_cross}",
        evidence_refs=[_event_ref(first_aps)] if first_aps is not None else [],
        evidence_payload={"first_aps": first_aps, "first_candidate_window": candidate},
        status="observed",
    )


def _case_mismatch_tolerated(
    summary: Mapping[str, Any],
    expectation: RollingBasicMVExpectation,
    events: list[dict[str, Any]],
    mv_id: str,
) -> bool:
    return (
        _original_pair_selected(summary, expectation)
        and _bounded_merge_succeeded(summary, expectation)
        and _eq10_consumption_is_local_cfv_only(events, mv_id, expectation)
    )


def _eq10_mismatch_tolerated(
    summary: Mapping[str, Any],
    expectation: RollingBasicMVExpectation,
    events: list[dict[str, Any]],
    mv_id: str,
) -> bool:
    eq10_consumers = set(summary.get("eq10_consumers") or [])
    expected_eq10 = set(expectation.expected_eq10_consumer_ids)
    later_local_cfv_only = not expected_eq10 and eq10_consumers == {expectation.expected_cfv_id}
    return (
        later_local_cfv_only
        and _original_pair_selected(summary, expectation)
        and _bounded_merge_succeeded(summary, expectation)
        and _eq10_consumption_is_local_cfv_only(events, mv_id, expectation)
    )


def _original_pair_selected(
    summary: Mapping[str, Any],
    expectation: RollingBasicMVExpectation,
) -> bool:
    return (
        summary.get("observed_clv_id") == expectation.expected_clv_id
        and summary.get("observed_cfv_id") == expectation.expected_cfv_id
    )


def _bounded_merge_succeeded(
    summary: Mapping[str, Any],
    expectation: RollingBasicMVExpectation,
) -> bool:
    return (
        bool(summary.get("merged_and_past_ramp"))
        and bool(summary.get("bounded_assignment_merge_success"))
        and summary.get("merge_success_clv_id") == expectation.expected_clv_id
        and summary.get("merge_success_cfv_id") == expectation.expected_cfv_id
        and not bool(summary.get("used_front_only_recovery_for_success"))
    )


def _eq10_consumption_is_local_cfv_only(
    events: list[dict[str, Any]],
    mv_id: str,
    expectation: RollingBasicMVExpectation,
) -> bool:
    for event in _eq10_consumption_events(events, mv_id):
        payload = event.get("payload") or {}
        if event.get("vehicle_id") != expectation.expected_cfv_id:
            return False
        if payload.get("cv_role") not in {None, "cfv"}:
            return False
        if payload.get("aps_case") not in {None, "case_2", "case_4"}:
            return False
        spacing = _eq10_spacing_value(event)
        if spacing is None or spacing <= 0.0 or spacing > ROLLING_BASIC_EQ10_SPACING_MAX_M:
            return False
    return True


def _eq10_value_issue(
    summary: Mapping[str, Any],
    expectation: RollingBasicMVExpectation,
    events: list[dict[str, Any]],
    mv_id: str,
) -> dict[str, Any] | None:
    for event in _eq10_consumption_events(events, mv_id):
        spacing = _eq10_spacing_value(event)
        if spacing is not None and 0.0 < spacing <= ROLLING_BASIC_EQ10_SPACING_MAX_M:
            continue
        payload = event.get("payload") or {}
        return _bug_finding(
            bug_id="RB-EQ10-002",
            category="eq10_issue",
            severity="error",
            title="Eq.10 spacing override was outside the rolling diagnostic safety range.",
            affected_mv_ids=[mv_id],
            affected_vehicle_ids=_non_empty([mv_id, event.get("vehicle_id"), expectation.expected_cfv_id]),
            first_step=event.get("step"),
            first_t=event.get("t"),
            symptom="A consumed Eq.10 spacing override was missing, non-positive, or above the rolling cap.",
            expected=f"0 < Eq.10 spacing <= {ROLLING_BASIC_EQ10_SPACING_MAX_M:g} m.",
            observed=f"eq10_desired_spacing={payload.get('eq10_desired_spacing')}",
            evidence_refs=[_event_ref(event)],
            evidence_payload={"spacing_override_consumption": event},
            status="observed",
        )
    return None


def _eq10_spacing_value(event: Mapping[str, Any]) -> float | None:
    payload = event.get("payload") or {}
    value = payload.get("eq10_desired_spacing")
    if value is None:
        value = payload.get("desired_spacing_override")
    if value is None:
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _assignment_owner_mismatch(summary: Mapping[str, Any]) -> dict[str, Any] | None:
    for item in summary.get("assignment_record_timeline") or []:
        record = item.get("record") or {}
        payload_mv = record.get("mv_id")
        owner_mv = summary.get("mv_id")
        if payload_mv and payload_mv != owner_mv:
            return _bug_finding(
                bug_id="RB-ASG-001",
                category="assignment_lifecycle_issue",
                severity="error",
                title="Assignment owner MV does not match assignment payload mv_id.",
                affected_mv_ids=_unique([owner_mv, payload_mv]),
                affected_vehicle_ids=_non_empty([owner_mv, payload_mv, record.get("clv_id"), record.get("cfv_id")]),
                first_step=item.get("step"),
                first_t=item.get("t"),
                symptom="State owner key and payload mv_id disagree.",
                expected=f"owner_mv_id == payload mv_id == {owner_mv}.",
                observed=f"owner_mv_id={owner_mv}, payload mv_id={payload_mv}.",
                evidence_refs=[_trajectory_ref(item.get("step"), str(owner_mv))],
                evidence_payload={"assignment_record": record},
                status="observed",
            )
    return None


def _invalid_assignment_entered_merge(summary: Mapping[str, Any]) -> dict[str, Any] | None:
    invalids = [
        item
        for item in summary.get("assignment_validation_timeline") or []
        if (item.get("payload") or {}).get("assignment_valid") is False
    ]
    first_merge_step = summary.get("first_merge_zone_step")
    if not invalids or first_merge_step is None:
        return None
    first_invalid = invalids[0]
    if int(first_invalid.get("step", 10**9)) > int(first_merge_step):
        return None
    mv_id = str(summary.get("mv_id"))
    return _bug_finding(
        bug_id="RB-ASG-003",
        category="assignment_lifecycle_issue",
        severity="error",
        title="Assignment became invalid and was not restored before merge-zone handling.",
        affected_mv_ids=[mv_id],
        affected_vehicle_ids=_non_empty([mv_id, *(first_invalid.get("vehicle_ids") or [])]),
        first_step=first_invalid.get("step"),
        first_t=first_invalid.get("t"),
        symptom="CMC validation reported an invalid assignment at or after merge-zone entry.",
        expected="Invalid assignment should recover before CMC merge-zone decision proceeds.",
        observed=f"invalid_reason={(first_invalid.get('payload') or {}).get('invalid_reason')}",
        evidence_refs=[_event_ref(first_invalid), _trajectory_ref(first_merge_step, mv_id)],
        evidence_payload={"first_invalid_assignment": first_invalid, "first_merge_zone_step": first_merge_step},
        status="observed",
    )


def _merge_zone_no_active_request(
    summary: Mapping[str, Any],
    events: list[dict[str, Any]],
) -> dict[str, Any] | None:
    first_merge_step = summary.get("first_merge_zone_step")
    if first_merge_step is None:
        return None
    if bool(summary.get("bounded_assignment_merge_success")):
        return None
    mv_id = str(summary.get("mv_id"))
    first_merge_step = int(first_merge_step)
    relevant_requests = [
        event
        for event in events
        if event.get("event_type") == "cooperative_request"
        and (event.get("payload") or {}).get("source_mv_id") == mv_id
        and int(event.get("step", -1)) >= first_merge_step
    ]
    validations = [
        event
        for event in summary.get("assignment_validation_timeline") or []
        if int(event.get("step", -1)) >= first_merge_step
    ]
    valid_validation = [
        event
        for event in validations
        if (event.get("payload") or {}).get("assignment_valid") is True
    ]
    if relevant_requests or not validations:
        return None
    first_validation = validations[0]
    status = "observed" if valid_validation else "inferred_from_data"
    return _bug_finding(
        bug_id="RB-CMC-001",
        category="cmc_issue",
        severity="warning",
        title="Merge-zone CMC phase had assignment validation but no active cooperative request.",
        affected_mv_ids=[mv_id],
        affected_vehicle_ids=[mv_id],
        first_step=first_merge_step,
        first_t=summary.get("first_merge_zone_t"),
        symptom="After merge-zone entry, Step5/CUC did not emit cooperative_request events for this MV.",
        expected="Valid assignment lifecycle should keep Step5/CUC active until CMC can merge.",
        observed=f"post_merge_request_count={len(relevant_requests)}, validation_count={len(validations)}",
        evidence_refs=[_event_ref(first_validation), _trajectory_ref(first_merge_step, mv_id)],
        evidence_payload={
            "first_merge_zone": summary.get("first_merge_zone"),
            "post_merge_assignment_validations": validations[:5],
        },
        status=status,
    )


def _cache_lifecycle_issue(summary: Mapping[str, Any]) -> dict[str, Any] | None:
    invalidations = summary.get("cached_boundary_invalidation_timeline") or []
    if not invalidations:
        return None
    first = invalidations[0]
    mv_id = str(summary.get("mv_id"))
    payload = first.get("payload") or {}
    affected = _non_empty([mv_id, payload.get("invalid_boundary_id")])
    return _bug_finding(
        bug_id="RB-LIFE-001",
        category="assignment_lifecycle_issue",
        severity="warning",
        title="Assignment cache invalidation appeared during rolling lifecycle.",
        affected_mv_ids=[mv_id],
        affected_vehicle_ids=affected,
        first_step=first.get("step"),
        first_t=first.get("t"),
        symptom="Cached boundary invalidation or recovery-required marking changed assignment lifecycle.",
        expected="Cache invalidation should affect only the owning MV and should not strand the assignment.",
        observed=f"invalid_boundary_id={payload.get('invalid_boundary_id')} invalid_reason={payload.get('invalid_reason')}",
        evidence_refs=[_event_ref(first)],
        evidence_payload={"first_cached_boundary_invalidation": first},
        status="inferred_from_data",
    )


def _eq10_consumption_events(events: list[dict[str, Any]], source_mv_id: str) -> list[dict[str, Any]]:
    return [
        event
        for event in events
        if event.get("event_type") == "spacing_override_consumption"
        and (event.get("payload") or {}).get("source_mv_id") == source_mv_id
    ]


def _finding_from_sanity_issue(issue: Mapping[str, Any], *, index: int) -> dict[str, Any]:
    return _bug_finding(
        bug_id=f"RB-SANITY-{index + 1:03d}",
        category=str(issue.get("category") or "recording_issue"),
        severity="warning" if issue.get("severity") == "warning" else "error",
        title=str(issue.get("message") or "Sanity check issue observed."),
        affected_mv_ids=[vehicle_id for vehicle_id in issue.get("vehicle_ids", []) if vehicle_id in ROLLING_BASIC_MV_IDS],
        affected_vehicle_ids=issue.get("vehicle_ids") or [],
        first_step=issue.get("step"),
        first_t=issue.get("t"),
        symptom=str(issue.get("message") or "Sanity check emitted a warning/failure."),
        expected="Sanity checks should pass or be not_applicable.",
        observed=str(issue.get("message") or "sanity issue"),
        evidence_refs=[f"sanity.jsonl:step={issue.get('step')}:check={issue.get('issue_id')}"],
        evidence_payload={"sanity_issue": dict(issue)},
        status="observed",
    )


def _bug_finding(
    *,
    bug_id: str,
    category: str,
    severity: str,
    title: str,
    affected_mv_ids: Iterable[str],
    affected_vehicle_ids: Iterable[str],
    first_step: Any,
    first_t: Any,
    symptom: str,
    expected: str,
    observed: str,
    evidence_refs: Iterable[str],
    evidence_payload: Mapping[str, Any],
    status: str,
) -> dict[str, Any]:
    refs = [ref for ref in evidence_refs if ref]
    return {
        "bug_id": bug_id,
        "category": category,
        "severity": severity,
        "title": title,
        "affected_mv_ids": _unique(affected_mv_ids),
        "affected_vehicle_ids": _unique(affected_vehicle_ids),
        "first_step": first_step,
        "first_t": first_t,
        "symptom": symptom,
        "expected": expected,
        "observed": observed,
        "evidence_refs": refs,
        "evidence_payload": dict(evidence_payload),
        "status": status,
    }


def _dedupe_findings(findings: Iterable[dict[str, Any]]) -> list[dict[str, Any]]:
    deduped: list[dict[str, Any]] = []
    seen: set[tuple[Any, ...]] = set()
    for finding in findings:
        key = (
            finding.get("bug_id"),
            tuple(finding.get("affected_mv_ids") or ()),
            finding.get("first_step"),
            finding.get("title"),
        )
        if key in seen:
            continue
        seen.add(key)
        deduped.append(finding)
    return deduped


def _rolling_status(
    *,
    bug_findings: Iterable[Mapping[str, Any]],
    merged_all: bool,
    simulation: SimulationLoopResult,
) -> str:
    findings = list(bug_findings)
    if any(finding.get("severity") == "error" for finding in findings):
        return "failed"
    if merged_all and not findings:
        return "passed"
    if merged_all:
        return "passed_with_diagnostics"
    if simulation.status == "stopped_by_condition":
        return "passed_with_diagnostics"
    return "diagnosed_unresolved"


def _write_scenario_report(
    path: Path,
    *,
    summary: Mapping[str, Any],
    artifact_paths: Mapping[str, str],
) -> Path:
    lines = [
        f"# {summary['scenario_id']} Rolling BASIC Numeric Diagnostic",
        "",
        f"- run_id: `{summary['run_id']}`",
        f"- status: `{summary['status']}`",
        f"- actual_steps: `{summary['actual_steps']}` / max `{summary['max_steps']}`",
        f"- dt: `{summary['dt']}`",
        f"- bug_findings: `{summary['bug_finding_count']}`",
        "",
        "## MV Summary",
        "",
        "| MV | first control | first APS | expected/observed case | expected pair | observed pair | active CVs | Eq.10 | merged past ramp |",
        "| --- | ---: | ---: | --- | --- | --- | --- | --- | --- |",
    ]
    for mv_id in ROLLING_BASIC_MV_IDS:
        mv = (summary.get("mv_summaries") or {})[mv_id]
        lines.append(
            "| {mv} | {control} | {aps} | {expected}/{observed} | {eclv},{ecfv} | {oclv},{ocfv} | {active} | {eq10} | {merged} |".format(
                mv=mv_id,
                control=mv.get("first_control_zone_step"),
                aps=_optional_field(mv.get("first_aps"), "step"),
                expected=mv.get("expected_aps_case"),
                observed=mv.get("observed_aps_case"),
                eclv=mv.get("expected_clv_id"),
                ecfv=mv.get("expected_cfv_id"),
                oclv=mv.get("observed_clv_id"),
                ocfv=mv.get("observed_cfv_id"),
                active=", ".join(mv.get("active_cv_ids") or []) or "none",
                eq10=", ".join(mv.get("eq10_consumers") or []) or "none",
                merged=mv.get("merged_and_past_ramp"),
            )
        )
    lines.extend(["", "## Per-MV Timelines", ""])
    for mv_id in ROLLING_BASIC_MV_IDS:
        mv = (summary.get("mv_summaries") or {})[mv_id]
        lines.extend(
            [
                f"### {mv_id}",
                "",
                f"- first APS: `{_compact_event(mv.get('first_aps'))}`",
                f"- APS candidates: `{_compact_aps_candidates(mv.get('aps_candidate_timeline') or [])}`",
                f"- excluded candidates: `{_compact_excluded_candidates(mv.get('aps_candidate_timeline') or [])}`",
                f"- assignment validity: `{_compact_timeline(mv.get('assignment_validation_timeline') or [], 'assignment_valid')}`",
                f"- assignment invalid reasons: `{_compact_timeline(mv.get('assignment_validation_timeline') or [], 'invalid_reason')}`",
                f"- CUC choices: `{_compact_timeline(mv.get('cuc_choice_timeline') or [], 'final_choice')}`",
                f"- Eq.53: `{_compact_timeline(mv.get('eq53_timeline') or [], 'eq53_pass')}`",
                f"- merge states: `{_compact_timeline(mv.get('merge_state_timeline') or [], 'merge_state')}`",
                "",
            ]
        )
    cross = summary.get("cross_mv_summary") or {}
    lines.extend(
        [
            "## Cross-MV",
            "",
            f"- shared CV timeline count: `{len(cross.get('shared_cv_timeline') or [])}`",
            f"- conflict resolution count: `{len(cross.get('step5_conflict_resolution_timeline') or [])}`",
            f"- suppressed request count: `{len(cross.get('suppressed_cooperative_request_timeline') or [])}`",
            f"- same-step multi-MV APS count: `{len(cross.get('same_step_multi_mv_aps_timeline') or [])}`",
            f"- same-step multi-MV CMC count: `{len(cross.get('same_step_multi_mv_cmc_timeline') or [])}`",
            "",
            "## Bug Findings",
            "",
        ]
    )
    findings = summary.get("bug_findings") or []
    if not findings:
        lines.append("- none")
    else:
        for finding in findings:
            lines.append(
                "- `{bug_id}` [{category}/{severity}/{status}] step `{step}`: {title}".format(
                    bug_id=finding.get("bug_id"),
                    category=finding.get("category"),
                    severity=finding.get("severity"),
                    status=finding.get("status"),
                    step=finding.get("first_step"),
                    title=str(finding.get("title", "")).replace("|", "/"),
                )
            )
    lines.extend(["", "## Artifacts", ""])
    for name, artifact_path in artifact_paths.items():
        lines.append(f"- {name}: `{artifact_path}`")
    lines.append(f"- numeric_summary: `{path.with_name('numeric_summary.json')}`")
    _ensure_parent(path).write_text("\n".join(lines) + "\n", encoding="utf-8")
    return path


def _compact_aps_candidates(timeline: Iterable[Mapping[str, Any]]) -> str:
    items: list[str] = []
    for item in timeline:
        ids = ",".join(str(value) for value in item.get("candidate_ids") or [])
        items.append(f"{item.get('step')}:[{ids}]")
    return "; ".join(items) or "none"


def _build_manifest(
    *,
    run_id: str,
    status: str,
    artifact_paths: Mapping[str, str],
    scenario_report_path: Path,
) -> ArtifactManifest:
    return ArtifactManifest(
        run_id=run_id,
        entries=(
            ArtifactManifestEntry(
                scenario_id=ROLLING_BASIC_SCENARIO_ID,
                run_id=run_id,
                status=status,
                input_config_ref=f"rolling_basic_scenario:{ROLLING_BASIC_SCENARIO_ID}",
                exports=artifact_paths,
                scenario_report_path=str(scenario_report_path),
                human_summary_path=str(scenario_report_path),
            ),
        ),
    )


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
    return datetime.now(UTC).strftime("rolling_basic_%Y%m%d_%H%M%S")


def _payload_value(event: Mapping[str, Any] | None, key: str) -> Any:
    if event is None:
        return None
    return (event.get("payload") or {}).get(key)


def _optional_field(value: Mapping[str, Any] | None, key: str) -> Any:
    if value is None:
        return None
    return value.get(key)


def _event_ref(event: Mapping[str, Any] | None) -> str:
    if event is None:
        return ""
    return _event_ref_from_values(event.get("step"), event.get("event_type"), event.get("vehicle_id"))


def _event_ref_from_values(step: Any, event_type: Any, vehicle_id: Any) -> str:
    return f"events.jsonl:step={step}:event_type={event_type}:vehicle_id={vehicle_id}"


def _trajectory_ref(step: Any, vehicle_id: str) -> str:
    return f"trajectory.csv:step={step}:vehicle_id={vehicle_id}"


def _unique(values: Iterable[Any]) -> list[str]:
    result: list[str] = []
    for value in values:
        if value in (None, ""):
            continue
        label = str(value)
        if label not in result:
            result.append(label)
    return result


def _non_empty(values: Iterable[Any]) -> list[str]:
    return _unique(values)
