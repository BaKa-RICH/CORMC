from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Mapping


ERROR = "error"
WARNING = "warning"


@dataclass(frozen=True)
class AcceptanceIssue:
    check_id: str
    severity: str
    message: str
    step: int | None = None
    vehicle_id: str | None = None
    payload: Mapping[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class VehicleLifecycleSummary:
    vehicle_id: str
    entered_control_zone_step: int | None
    trigger_steps: tuple[int, ...]
    selected_gap_steps: tuple[int, ...]
    locked_gap_step: int | None
    entered_merging_step: int | None
    merge_completed_step: int | None
    final_physical_lane: str | None
    final_road_role: str | None
    final_merge_state: str | None


@dataclass(frozen=True)
class AcceptanceReport:
    scenario_id: str
    passed: bool
    issues: tuple[AcceptanceIssue, ...]
    vehicle_lifecycles: Mapping[str, VehicleLifecycleSummary]
    integrity: Mapping[str, Any]
    human_report: str


STAGE2_SUMMARY_TOP_LEVEL_KEYS = frozenset(
    {
        "scenario_summary",
        "round_summaries",
        "mv_summaries",
        "cross_mv_summary",
        "artifact_paths",
    }
)

STAGE2_FORMAL_EVENT_KINDS = frozenset(
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


def build_onestep_stage2_acceptance_report(
    summary: Mapping[str, Any],
) -> AcceptanceReport:
    scenario_summary = dict(summary.get("scenario_summary") or {})
    scenario_id = str(scenario_summary.get("scenario_id") or "unknown_scenario")
    issues: list[AcceptanceIssue] = []
    issues.extend(_check_stage2_summary_schema(summary))
    issues.extend(_check_stage2_formal_events(summary))
    issues.extend(_check_stage2_rounds(summary))
    issues.extend(_check_stage2_gap_rows(summary))
    issues.extend(_check_stage2_lifecycle(summary))
    issues.extend(_check_stage2_cross_mv(summary))
    issues.extend(_check_stage2_runtime_leftovers(summary))

    lifecycles = _build_stage2_lifecycles(summary)
    cross = dict(summary.get("cross_mv_summary") or {})
    leftovers = dict(cross.get("final_runtime_leftovers") or {})
    integrity = {
        "summary_schema_valid": set(summary) == STAGE2_SUMMARY_TOP_LEVEL_KEYS,
        "minimal_event_set_valid": _stage2_event_set_valid(summary),
        "multi_mv_gap_conflict": bool(cross.get("gap_conflicts")),
        "frontier_violation": bool(cross.get("frontier_violations")),
        "ownership_conflict": bool(cross.get("ownership_conflicts")),
        "runtime_leftover_count": _stage2_leftover_count(leftovers),
    }
    passed = not any(issue.severity == ERROR for issue in issues)
    return AcceptanceReport(
        scenario_id=scenario_id,
        passed=passed,
        issues=tuple(issues),
        vehicle_lifecycles=lifecycles,
        integrity=integrity,
        human_report=_build_stage2_human_report(
            scenario_id,
            passed,
            lifecycles,
            integrity,
            issues,
        ),
    )


def build_onestep_stage2_random_acceptance_report(
    summary: Mapping[str, Any],
) -> AcceptanceReport:
    scenario_summary = dict(summary.get("scenario_summary") or {})
    scenario_id = str(scenario_summary.get("scenario_id") or "unknown_scenario")
    issues: list[AcceptanceIssue] = []
    issues.extend(_check_stage2_summary_schema(summary))
    issues.extend(_check_stage2_random_formal_events(summary))
    issues.extend(_check_stage2_rounds(summary))
    issues.extend(_check_stage2_gap_rows(summary))
    issues.extend(_check_stage2_random_lifecycle(summary))
    issues.extend(_check_stage2_cross_mv(summary))
    issues.extend(_check_stage2_random_generation(summary))

    lifecycles = _build_stage2_lifecycles(summary)
    cross = dict(summary.get("cross_mv_summary") or {})
    generated_by_lane = dict(scenario_summary.get("generated_by_lane") or {})
    integrity = {
        "summary_schema_valid": set(summary) == STAGE2_SUMMARY_TOP_LEVEL_KEYS,
        "minimal_event_set_valid": _stage2_event_set_valid(summary),
        "generated_vehicle_count": int(
            scenario_summary.get("generated_vehicle_count") or 0
        ),
        "generated_by_lane": generated_by_lane,
        "blocked_spawn_count": int(scenario_summary.get("blocked_spawn_count") or 0),
        "completed_mv_count": int(scenario_summary.get("completed_mv_count") or 0),
        "open_mv_count_at_horizon": int(
            scenario_summary.get("open_mv_count_at_horizon") or 0
        ),
        "round_count": len(_stage2_round_summaries(summary)),
        "gap_row_count": len(_stage2_gap_rows(summary)),
        "cross_mv_conflict_count": len(cross.get("gap_conflicts") or []),
        "multi_mv_gap_conflict": bool(cross.get("gap_conflicts")),
        "frontier_violation": bool(cross.get("frontier_violations")),
        "ownership_conflict": bool(cross.get("ownership_conflicts")),
    }
    passed = not any(issue.severity == ERROR for issue in issues)
    return AcceptanceReport(
        scenario_id=scenario_id,
        passed=passed,
        issues=tuple(issues),
        vehicle_lifecycles=lifecycles,
        integrity=integrity,
        human_report=_build_stage2_random_human_report(
            scenario_id,
            passed,
            lifecycles,
            integrity,
            issues,
        ),
    )


def build_acceptance_report(summary: Mapping[str, Any]) -> AcceptanceReport:
    scenario_id = str(summary.get("scenario_id") or "unknown_scenario")
    issues: list[AcceptanceIssue] = []
    issues.extend(_check_required_fields(summary))
    issues.extend(_check_trigger_timeline(summary))
    issues.extend(_check_non_trigger_gap_events(summary))
    issues.extend(_check_gap_selection(summary))
    issues.extend(_check_gap_lock(summary))
    issues.extend(_check_merge_check(summary))
    issues.extend(_check_trajectory_events(summary))
    issues.extend(_check_merge_completion(summary))
    issues.extend(_check_final_merge_state(summary))
    issues.extend(_check_old_state_pollution(summary))
    issues.extend(detect_multi_mv_gap_conflicts(summary))

    lifecycles = _build_lifecycles(summary)
    integrity = {
        "old_assignment_record_count": summary.get("old_assignment_record_count"),
        "old_active_maneuver_count": summary.get("old_active_maneuver_count"),
        "non_trigger_gap_event_count": summary.get("non_trigger_gap_event_count"),
        "multi_mv_gap_conflict": any(
            issue.check_id == "ACCEPT-012" and issue.severity == ERROR
            for issue in issues
        ),
        "required_field_count": len(REQUIRED_SUMMARY_FIELDS),
        "present_required_field_count": sum(
            1 for field_name in REQUIRED_SUMMARY_FIELDS if field_name in summary
        ),
    }
    passed = not any(issue.severity == ERROR for issue in issues)
    return AcceptanceReport(
        scenario_id=scenario_id,
        passed=passed,
        issues=tuple(issues),
        vehicle_lifecycles=lifecycles,
        integrity=integrity,
        human_report=_build_human_report(
            scenario_id,
            passed,
            lifecycles,
            integrity,
            issues,
            summary,
        ),
    )


def build_stability_acceptance_report(summary: Mapping[str, Any]) -> AcceptanceReport:
    return build_acceptance_report(summary)


def _check_stage2_summary_schema(
    summary: Mapping[str, Any],
) -> tuple[AcceptanceIssue, ...]:
    actual = set(summary)
    if actual == STAGE2_SUMMARY_TOP_LEVEL_KEYS:
        return ()
    return (
        AcceptanceIssue(
            check_id="S2-SUMMARY-001",
            severity=ERROR,
            message="Stage2 summary top-level keys must be the formal five-layer schema.",
            payload={
                "expected": sorted(STAGE2_SUMMARY_TOP_LEVEL_KEYS),
                "actual": sorted(actual),
            },
        ),
    )


def _check_stage2_formal_events(
    summary: Mapping[str, Any],
) -> tuple[AcceptanceIssue, ...]:
    events = _stage2_formal_events(summary)
    kinds = {str(event.get("event_kind")) for event in events}
    unknown = tuple(sorted(kinds - STAGE2_FORMAL_EVENT_KINDS))
    missing = tuple(sorted(STAGE2_FORMAL_EVENT_KINDS - kinds))
    issues: list[AcceptanceIssue] = []
    if unknown:
        issues.append(
            AcceptanceIssue(
                check_id="S2-EVENT-001",
                severity=ERROR,
                message="Stage2 summary has formal event kinds outside the minimal set.",
                payload={"unknown_event_kinds": unknown},
            )
        )
    if missing:
        issues.append(
            AcceptanceIssue(
                check_id="S2-EVENT-001",
                severity=ERROR,
                message="Stage2 summary is missing required formal event kinds.",
                payload={"missing_event_kinds": missing},
            )
        )
    required_fields = {
        "event_kind",
        "step",
        "t",
        "mv_id",
        "round_id",
        "round_order",
        "payload",
    }
    for event in events:
        missing_fields = tuple(sorted(required_fields - set(event)))
        if missing_fields:
            issues.append(
                AcceptanceIssue(
                    check_id="S2-EVENT-001",
                    severity=ERROR,
                    message="Formal event is missing required fields.",
                    step=_int_or_none(event.get("step")),
                    vehicle_id=_vehicle_id(event),
                    payload={"missing_fields": missing_fields},
                )
            )
    return tuple(issues)


def _check_stage2_random_formal_events(
    summary: Mapping[str, Any],
) -> tuple[AcceptanceIssue, ...]:
    events = _stage2_formal_events(summary)
    kinds = {str(event.get("event_kind")) for event in events}
    unknown = tuple(sorted(kinds - STAGE2_FORMAL_EVENT_KINDS))
    issues: list[AcceptanceIssue] = []
    if unknown:
        issues.append(
            AcceptanceIssue(
                check_id="S2-EVENT-001",
                severity=ERROR,
                message="Stage2 summary has formal event kinds outside the minimal set.",
                payload={"unknown_event_kinds": unknown},
            )
        )
    required_fields = {
        "event_kind",
        "step",
        "t",
        "mv_id",
        "round_id",
        "round_order",
        "payload",
    }
    for event in events:
        missing_fields = tuple(sorted(required_fields - set(event)))
        if missing_fields:
            issues.append(
                AcceptanceIssue(
                    check_id="S2-EVENT-001",
                    severity=ERROR,
                    message="Formal event is missing required fields.",
                    step=_int_or_none(event.get("step")),
                    vehicle_id=_vehicle_id(event),
                    payload={"missing_fields": missing_fields},
                )
            )
    return tuple(issues)


def _check_stage2_rounds(summary: Mapping[str, Any]) -> tuple[AcceptanceIssue, ...]:
    issues: list[AcceptanceIssue] = []
    for round_summary in _stage2_round_summaries(summary):
        records = sorted(
            [
                *list(round_summary.get("plan_summaries") or []),
                *list(round_summary.get("locked_gap_events") or []),
            ],
            key=lambda item: (
                int(item.get("round_order") or 0),
                str(item.get("mv_id") or ""),
            ),
        )
        expected_order = list(
            dict.fromkeys(
                str(record.get("mv_id"))
                for record in records
                if record.get("mv_id") is not None
            )
        )
        if list(round_summary.get("mv_order") or []) != expected_order:
            issues.append(
                AcceptanceIssue(
                    check_id="S2-ROUND-001",
                    severity=ERROR,
                    message="Round MV order does not match actual round_order records.",
                    step=_int_or_none(round_summary.get("step")),
                    payload={
                        "round_id": round_summary.get("round_id"),
                        "expected_order": expected_order,
                        "actual_order": list(round_summary.get("mv_order") or []),
                    },
                )
            )
        for record in records:
            before = record.get("tail_frontier_gap_index_before")
            gap_index = record.get("gap_index")
            if gap_index is None and isinstance(record.get("locked_gap"), Mapping):
                gap_index = record["locked_gap"].get("index")
            if before is None or gap_index is None:
                continue
            if int(gap_index) <= int(before):
                issues.append(
                    AcceptanceIssue(
                        check_id="S2-ROUND-002",
                        severity=ERROR,
                        message="Stage2 round selected or locked a gap at/before frontier.",
                        step=_int_or_none(record.get("step")),
                        vehicle_id=_vehicle_id(record),
                        payload={
                            "round_id": round_summary.get("round_id"),
                            "gap_index": gap_index,
                            "tail_frontier_gap_index_before": before,
                        },
                    )
                )
    return tuple(issues)


def _check_stage2_gap_rows(summary: Mapping[str, Any]) -> tuple[AcceptanceIssue, ...]:
    issues: list[AcceptanceIssue] = []
    required = {
        "branch",
        "front_controllable",
        "rear_controllable",
        "J",
        "failure_reason",
        "is_selected",
    }
    for row in _stage2_gap_rows(summary):
        missing = tuple(sorted(required - set(row)))
        if missing:
            issues.append(
                AcceptanceIssue(
                    check_id="S2-GAPROW-001",
                    severity=ERROR,
                    message="Stage2 gap row is missing formal evidence fields.",
                    step=_int_or_none(row.get("step")),
                    vehicle_id=_vehicle_id(row),
                    payload={"missing_fields": missing, "row": dict(row)},
                )
            )
    return tuple(issues)


def _check_stage2_lifecycle(summary: Mapping[str, Any]) -> tuple[AcceptanceIssue, ...]:
    issues: list[AcceptanceIssue] = []
    for mv_id, mv_summary in _stage2_mv_summaries(summary).items():
        lifecycle = dict(mv_summary.get("lifecycle") or {})
        ordered_steps = (
            lifecycle.get("locked_gap_step"),
            lifecycle.get("lateral_start_step"),
            lifecycle.get("lateral_completed_step"),
            lifecycle.get("mainline_conversion_step"),
        )
        if any(step is None for step in ordered_steps) or not (
            int(ordered_steps[0])
            <= int(ordered_steps[1])
            < int(ordered_steps[2])
            < int(ordered_steps[3])
        ):
            issues.append(
                AcceptanceIssue(
                    check_id="S2-LIFE-001",
                    severity=ERROR,
                    message="Stage2 MV lifecycle is incomplete or out of order.",
                    vehicle_id=mv_id,
                    payload={"lifecycle": lifecycle},
                )
            )
        final_status = dict(lifecycle.get("final_status") or {})
        expected = {
            "physical_lane": "lane_2",
            "road_role": "mainline",
            "merge_state": "normal",
            "runtime_present": False,
        }
        mismatches = {
            key: final_status.get(key)
            for key, expected_value in expected.items()
            if final_status.get(key) != expected_value
        }
        if mismatches:
            issues.append(
                AcceptanceIssue(
                    check_id="S2-LIFE-002",
                    severity=ERROR,
                    message="Stage2 MV final status is not lane_2/mainline/normal with no runtime owner.",
                    vehicle_id=mv_id,
                    payload={"mismatches": mismatches, "final_status": final_status},
                )
            )
    return tuple(issues)


def _check_stage2_random_lifecycle(
    summary: Mapping[str, Any],
) -> tuple[AcceptanceIssue, ...]:
    scenario_summary = dict(summary.get("scenario_summary") or {})
    completed = set(str(item) for item in scenario_summary.get("completed_mv_ids") or [])
    open_at_horizon = set(
        str(item) for item in scenario_summary.get("open_mv_ids_at_horizon") or []
    )
    allow_open = bool(
        dict(scenario_summary.get("flow_validation") or {}).get(
            "allow_open_mvs_at_horizon",
            True,
        )
    )
    issues: list[AcceptanceIssue] = []
    for mv_id, mv_summary in _stage2_mv_summaries(summary).items():
        lifecycle = dict(mv_summary.get("lifecycle") or {})
        final_status = dict(lifecycle.get("final_status") or {})
        lock = _int_or_none(lifecycle.get("locked_gap_step"))
        lateral = _int_or_none(lifecycle.get("lateral_start_step"))
        completed_step = _int_or_none(lifecycle.get("lateral_completed_step"))
        conversion = _int_or_none(lifecycle.get("mainline_conversion_step"))

        if mv_id in completed:
            if any(step is None for step in (lock, lateral, completed_step, conversion)) or not (
                lock <= lateral < completed_step < conversion
            ):
                issues.append(
                    AcceptanceIssue(
                        check_id="S2R-LIFE-001",
                        severity=ERROR,
                        message="Completed random MV lifecycle is incomplete or out of order.",
                        vehicle_id=mv_id,
                        payload={"lifecycle": lifecycle},
                    )
                )
            expected = {
                "physical_lane": "lane_2",
                "road_role": "mainline",
                "merge_state": "normal",
            }
            mismatches = {
                key: final_status.get(key)
                for key, expected_value in expected.items()
                if final_status.get(key) != expected_value
            }
            if mismatches:
                issues.append(
                    AcceptanceIssue(
                        check_id="S2R-LIFE-002",
                        severity=ERROR,
                        message="Completed random MV final status is not lane_2/mainline/normal.",
                        vehicle_id=mv_id,
                        payload={"mismatches": mismatches, "final_status": final_status},
                    )
                )
            continue

        if mv_id not in open_at_horizon:
            issues.append(
                AcceptanceIssue(
                    check_id="S2R-LIFE-003",
                    severity=ERROR,
                    message="Random MV is neither completed nor recorded open at horizon.",
                    vehicle_id=mv_id,
                    payload={"lifecycle": lifecycle},
                )
            )
            continue
        if not allow_open:
            issues.append(
                AcceptanceIssue(
                    check_id="S2R-LIFE-004",
                    severity=ERROR,
                    message="Random MV is open at horizon but scenario validation disallows open MVs.",
                    vehicle_id=mv_id,
                    payload={"lifecycle": lifecycle},
                )
            )
        if not final_status:
            issues.append(
                AcceptanceIssue(
                    check_id="S2R-LIFE-005",
                    severity=ERROR,
                    message="Open random MV is missing final status.",
                    vehicle_id=mv_id,
                    payload={"lifecycle": lifecycle},
                )
            )
        if lateral is not None and completed_step is None and mv_id not in open_at_horizon:
            issues.append(
                AcceptanceIssue(
                    check_id="S2R-LIFE-006",
                    severity=ERROR,
                    message="Random MV started lateral motion but is not classified as open at horizon.",
                    vehicle_id=mv_id,
                    payload={"lifecycle": lifecycle},
                )
            )
        if conversion is not None:
            expected = {
                "physical_lane": "lane_2",
                "road_role": "mainline",
                "merge_state": "normal",
            }
            mismatches = {
                key: final_status.get(key)
                for key, expected_value in expected.items()
                if final_status.get(key) != expected_value
            }
            if mismatches:
                issues.append(
                    AcceptanceIssue(
                        check_id="S2R-LIFE-007",
                        severity=ERROR,
                        message="Converted random MV final status is not lane_2/mainline/normal.",
                        vehicle_id=mv_id,
                        payload={"mismatches": mismatches, "final_status": final_status},
                    )
                )
    return tuple(issues)


def _check_stage2_random_generation(
    summary: Mapping[str, Any],
) -> tuple[AcceptanceIssue, ...]:
    scenario = dict(summary.get("scenario_summary") or {})
    validation = dict(scenario.get("flow_validation") or {})
    generated_by_lane = dict(scenario.get("generated_by_lane") or {})
    issues: list[AcceptanceIssue] = []
    checks = (
        (
            "S2R-GEN-001",
            "lane_2",
            int(validation.get("min_generated_lane2_count") or 0),
            int(generated_by_lane.get("lane_2") or 0),
            "Random flow generated fewer lane_2 vehicles than required.",
        ),
        (
            "S2R-GEN-002",
            "on_ramp",
            int(validation.get("min_generated_on_ramp_mv_count") or 0),
            int(scenario.get("generated_on_ramp_mv_count") or 0),
            "Random flow generated fewer on-ramp MVs than required.",
        ),
        (
            "S2R-GEN-003",
            "completed_mv",
            int(validation.get("min_completed_mv_count") or 0),
            int(scenario.get("completed_mv_count") or 0),
            "Random flow completed fewer MVs than required.",
        ),
    )
    for check_id, lane, minimum, actual, message in checks:
        if actual < minimum:
            issues.append(
                AcceptanceIssue(
                    check_id=check_id,
                    severity=ERROR,
                    message=message,
                    payload={"minimum": minimum, "actual": actual, "name": lane},
                )
            )
    if len(_stage2_round_summaries(summary)) <= 0:
        issues.append(
            AcceptanceIssue(
                check_id="S2R-GEN-004",
                severity=ERROR,
                message="Random Stage2 run did not produce any trigger round.",
            )
        )
    if len(_stage2_gap_rows(summary)) <= 0:
        issues.append(
            AcceptanceIssue(
                check_id="S2R-GEN-005",
                severity=ERROR,
                message="Random Stage2 run did not produce any gap row evidence.",
            )
        )
    return tuple(issues)


def _check_stage2_cross_mv(summary: Mapping[str, Any]) -> tuple[AcceptanceIssue, ...]:
    cross = dict(summary.get("cross_mv_summary") or {})
    issues: list[AcceptanceIssue] = []
    for conflict in cross.get("gap_conflicts") or []:
        issues.append(
            AcceptanceIssue(
                check_id="S2-CROSS-001",
                severity=ERROR,
                message="Stage2 cross-MV same-round structured gap conflict.",
                payload={"conflict": dict(conflict)},
            )
        )
    for violation in cross.get("frontier_violations") or []:
        issues.append(
            AcceptanceIssue(
                check_id="S2-ROUND-002",
                severity=ERROR,
                message="Stage2 cross-MV frontier violation.",
                step=_int_or_none(violation.get("step")),
                vehicle_id=_vehicle_id(violation),
                payload={"violation": dict(violation)},
            )
        )
    for conflict in cross.get("ownership_conflicts") or []:
        issues.append(
            AcceptanceIssue(
                check_id="S2-CROSS-002",
                severity=ERROR,
                message="Stage2 active bundle ownership conflict.",
                step=_int_or_none(conflict.get("step")),
                payload={"conflict": dict(conflict)},
            )
        )
    return tuple(issues)


def _check_stage2_runtime_leftovers(
    summary: Mapping[str, Any],
) -> tuple[AcceptanceIssue, ...]:
    cross = dict(summary.get("cross_mv_summary") or {})
    leftovers = dict(cross.get("final_runtime_leftovers") or {})
    if _stage2_leftover_count(leftovers) == 0:
        return ()
    return (
        AcceptanceIssue(
            check_id="S2-RUNTIME-001",
            severity=ERROR,
            message="Stage2 final runtime still has plan/bundle/gap/lateral/controlled leftovers.",
            payload={"final_runtime_leftovers": leftovers},
        ),
    )


def _build_stage2_lifecycles(
    summary: Mapping[str, Any],
) -> dict[str, VehicleLifecycleSummary]:
    result: dict[str, VehicleLifecycleSummary] = {}
    for mv_id, mv_summary in _stage2_mv_summaries(summary).items():
        lifecycle = dict(mv_summary.get("lifecycle") or {})
        final_status = dict(lifecycle.get("final_status") or {})
        result[mv_id] = VehicleLifecycleSummary(
            vehicle_id=mv_id,
            entered_control_zone_step=_int_or_none(lifecycle.get("first_control_zone_step")),
            trigger_steps=_tuple_optional_step(lifecycle.get("first_trigger_step")),
            selected_gap_steps=_tuple_optional_step(lifecycle.get("first_current_plan_step")),
            locked_gap_step=_int_or_none(lifecycle.get("locked_gap_step")),
            entered_merging_step=_int_or_none(lifecycle.get("lateral_start_step")),
            merge_completed_step=_int_or_none(lifecycle.get("lateral_completed_step")),
            final_physical_lane=final_status.get("physical_lane"),
            final_road_role=final_status.get("road_role"),
            final_merge_state=final_status.get("merge_state"),
        )
    return result


def _build_stage2_human_report(
    scenario_id: str,
    passed: bool,
    lifecycles: Mapping[str, VehicleLifecycleSummary],
    integrity: Mapping[str, Any],
    issues: list[AcceptanceIssue],
) -> str:
    lines = [
        f"Scenario {scenario_id}: {'validation passed' if passed else 'validation failed'}",
        "",
        "Cross-MV Validation:",
        f"  multi_mv_gap_conflict={integrity.get('multi_mv_gap_conflict')}",
        f"  frontier_violation={integrity.get('frontier_violation')}",
        f"  ownership_conflict={integrity.get('ownership_conflict')}",
        f"  runtime_leftover_count={integrity.get('runtime_leftover_count')}",
        "",
        "MV Lifecycle:",
    ]
    for mv_id, lifecycle in lifecycles.items():
        lines.append(
            "  "
            f"{mv_id}: lock={_fmt_step(lifecycle.locked_gap_step)}, "
            f"lateral={_fmt_step(lifecycle.entered_merging_step)}, "
            f"complete={_fmt_step(lifecycle.merge_completed_step)}, "
            f"final={lifecycle.final_physical_lane}/{lifecycle.final_road_role}/{lifecycle.final_merge_state}"
        )
    if issues:
        lines.append("")
        lines.append("Issues:")
        for issue in issues:
            lines.append(f"  {issue.check_id} [{issue.severity}]: {issue.message}")
    return "\n".join(lines)


def _build_stage2_random_human_report(
    scenario_id: str,
    passed: bool,
    lifecycles: Mapping[str, VehicleLifecycleSummary],
    integrity: Mapping[str, Any],
    issues: list[AcceptanceIssue],
) -> str:
    lines = [
        f"Scenario {scenario_id}: {'validation passed' if passed else 'validation failed'}",
        "",
        "Random Flow Integrity:",
        f"  generated_vehicle_count={integrity.get('generated_vehicle_count')}",
        f"  generated_by_lane={integrity.get('generated_by_lane')}",
        f"  blocked_spawn_count={integrity.get('blocked_spawn_count')}",
        f"  completed_mv_count={integrity.get('completed_mv_count')}",
        f"  open_mv_count_at_horizon={integrity.get('open_mv_count_at_horizon')}",
        f"  round_count={integrity.get('round_count')}",
        f"  gap_row_count={integrity.get('gap_row_count')}",
        f"  cross_mv_conflict_count={integrity.get('cross_mv_conflict_count')}",
        "",
        "Cross-MV Validation:",
        f"  multi_mv_gap_conflict={integrity.get('multi_mv_gap_conflict')}",
        f"  frontier_violation={integrity.get('frontier_violation')}",
        f"  ownership_conflict={integrity.get('ownership_conflict')}",
        "",
        "MV Lifecycle:",
    ]
    for mv_id, lifecycle in lifecycles.items():
        lines.append(
            "  "
            f"{mv_id}: lock={_fmt_step(lifecycle.locked_gap_step)}, "
            f"lateral={_fmt_step(lifecycle.entered_merging_step)}, "
            f"complete={_fmt_step(lifecycle.merge_completed_step)}, "
            f"final={lifecycle.final_physical_lane}/{lifecycle.final_road_role}/{lifecycle.final_merge_state}"
        )
    if issues:
        lines.append("")
        lines.append("Issues:")
        for issue in issues:
            lines.append(f"  {issue.check_id} [{issue.severity}]: {issue.message}")
    return "\n".join(lines)


def _stage2_formal_events(summary: Mapping[str, Any]) -> tuple[Mapping[str, Any], ...]:
    scenario_summary = dict(summary.get("scenario_summary") or {})
    value = scenario_summary.get("formal_events") or ()
    return tuple(value) if isinstance(value, (list, tuple)) else ()


def _stage2_round_summaries(summary: Mapping[str, Any]) -> tuple[Mapping[str, Any], ...]:
    value = summary.get("round_summaries") or ()
    return tuple(value) if isinstance(value, (list, tuple)) else ()


def _stage2_mv_summaries(summary: Mapping[str, Any]) -> Mapping[str, Mapping[str, Any]]:
    value = summary.get("mv_summaries") or {}
    return value if isinstance(value, Mapping) else {}


def _stage2_gap_rows(summary: Mapping[str, Any]) -> tuple[Mapping[str, Any], ...]:
    rows: list[Mapping[str, Any]] = []
    for mv_summary in _stage2_mv_summaries(summary).values():
        rows.extend(list(mv_summary.get("gap_rows") or []))
    return tuple(rows)


def _stage2_event_set_valid(summary: Mapping[str, Any]) -> bool:
    kinds = {str(event.get("event_kind")) for event in _stage2_formal_events(summary)}
    return kinds == STAGE2_FORMAL_EVENT_KINDS


def _stage2_leftover_count(leftovers: Mapping[str, Any]) -> int:
    count = 0
    for value in leftovers.values():
        if isinstance(value, Mapping):
            count += len(value)
        elif isinstance(value, (list, tuple, set)):
            count += len(value)
        elif value:
            count += 1
    return count


def _tuple_optional_step(value: Any) -> tuple[int, ...]:
    step = _int_or_none(value)
    return () if step is None else (step,)


def detect_multi_mv_gap_conflicts(
    summary: Mapping[str, Any],
) -> tuple[AcceptanceIssue, ...]:
    issues: list[AcceptanceIssue] = []
    selections_by_step: dict[int, dict[tuple[Any, ...], list[str]]] = {}
    for event in _events(summary, "gap_selection_events"):
        selected_gap = event.get("selected_gap")
        if not isinstance(selected_gap, Mapping):
            continue
        key = _structured_gap_key(selected_gap)
        if key is None:
            continue
        step = _int_or_none(event.get("step"))
        if step is None:
            continue
        mv_id = str(event.get("mv_id") or "")
        selections_by_step.setdefault(step, {}).setdefault(key, []).append(mv_id)

    for step, selections in selections_by_step.items():
        for key, mv_ids in selections.items():
            unique_mv_ids = tuple(dict.fromkeys(mv_ids))
            if len(unique_mv_ids) <= 1:
                continue
            issues.append(
                AcceptanceIssue(
                    check_id="ACCEPT-012",
                    severity=ERROR,
                    message="Multiple MVs selected the same structured gap in one frame.",
                    step=step,
                    payload={
                        "gap_key": key,
                        "mv_ids": unique_mv_ids,
                    },
                )
            )
    return tuple(issues)


REQUIRED_SUMMARY_FIELDS = (
    "scenario_id",
    "zone_state_timeline",
    "current_plan_gap_state_timeline",
    "locked_gap_state_timeline",
    "merge_state_timeline",
    "trigger_events",
    "gap_snapshots",
    "gap_selection_events",
    "gap_lock_events",
    "merge_check_events",
    "trajectory_events",
    "merge_completion_events",
    "final_vehicle_states",
    "runtime_mv_ids",
    "runtime_mv_states",
    "old_assignment_record_count",
    "old_active_maneuver_count",
    "non_trigger_gap_event_count",
)


def _check_required_fields(summary: Mapping[str, Any]) -> tuple[AcceptanceIssue, ...]:
    issues: list[AcceptanceIssue] = []
    missing = tuple(field_name for field_name in REQUIRED_SUMMARY_FIELDS if field_name not in summary)
    if missing:
        issues.append(
            AcceptanceIssue(
                check_id="ACCEPT-001",
                severity=ERROR,
                message="Summary is missing required acceptance fields.",
                payload={"missing_fields": missing},
            )
        )
    return tuple(issues)


def _check_trigger_timeline(summary: Mapping[str, Any]) -> tuple[AcceptanceIssue, ...]:
    events = _events(summary, "trigger_events")
    if not events:
        return (
            AcceptanceIssue(
                check_id="ACCEPT-002",
                severity=ERROR,
                message="Summary has no trigger timeline.",
            ),
        )
    for event in events:
        if "trigger_plan" not in event or "trigger_reason" not in event:
            return (
                AcceptanceIssue(
                    check_id="ACCEPT-002",
                    severity=ERROR,
                    message="Trigger event is missing trigger_plan or trigger_reason.",
                    step=_int_or_none(event.get("step")),
                    payload={"event": dict(event)},
                ),
            )
    return ()


def _check_non_trigger_gap_events(summary: Mapping[str, Any]) -> tuple[AcceptanceIssue, ...]:
    count = summary.get("non_trigger_gap_event_count")
    if count == 0:
        return ()
    return (
        AcceptanceIssue(
            check_id="ACCEPT-003",
            severity=ERROR,
            message="Gap snapshots were emitted on non-trigger frames.",
            payload={"non_trigger_gap_event_count": count},
        ),
    )


def _check_gap_selection(summary: Mapping[str, Any]) -> tuple[AcceptanceIssue, ...]:
    events = _events(summary, "gap_selection_events")
    if not events:
        return (
            AcceptanceIssue(
                check_id="ACCEPT-004",
                severity=ERROR,
                message="No current_plan_gap selection event was recorded.",
            ),
        )
    for event in events:
        if "current_plan_gap" not in event:
            return (
                AcceptanceIssue(
                    check_id="ACCEPT-004",
                    severity=ERROR,
                    message="Gap selection event is missing current_plan_gap.",
                    step=_int_or_none(event.get("step")),
                    vehicle_id=_vehicle_id(event),
                ),
            )
        gap = event.get("current_plan_gap") or event.get("selected_gap")
        if gap is None:
            continue
        if not _is_structured_gap_ref(gap):
            return (
                AcceptanceIssue(
                    check_id="ACCEPT-004",
                    severity=ERROR,
                    message="Selected gap is not represented with structured fields.",
                    step=_int_or_none(event.get("step")),
                    vehicle_id=_vehicle_id(event),
                    payload={"gap": gap},
                ),
            )
    return ()


def _check_gap_lock(summary: Mapping[str, Any]) -> tuple[AcceptanceIssue, ...]:
    events = _events(summary, "gap_lock_events")
    if not events:
        return (
            AcceptanceIssue(
                check_id="ACCEPT-005",
                severity=ERROR,
                message="No locked_gap event was recorded.",
            ),
        )
    first_key_by_mv: dict[str, tuple[Any, ...]] = {}
    for event in events:
        mv_id = _vehicle_id(event) or ""
        if "locked_gap" not in event:
            return (
                AcceptanceIssue(
                    check_id="ACCEPT-005",
                    severity=ERROR,
                    message="Gap lock event is missing locked_gap.",
                    step=_int_or_none(event.get("step")),
                    vehicle_id=_vehicle_id(event),
                ),
            )
        locked_gap = event.get("locked_gap")
        if not _is_structured_gap_ref(locked_gap):
            return (
                AcceptanceIssue(
                    check_id="ACCEPT-005",
                    severity=ERROR,
                    message="Locked gap is not represented with structured fields.",
                    step=_int_or_none(event.get("step")),
                    vehicle_id=_vehicle_id(event),
                    payload={"locked_gap": locked_gap},
                ),
            )
        key = _structured_gap_key(locked_gap)
        if mv_id not in first_key_by_mv:
            first_key_by_mv[mv_id] = key
        elif key != first_key_by_mv[mv_id]:
            return (
                AcceptanceIssue(
                    check_id="ACCEPT-005",
                    severity=ERROR,
                    message="Locked gap changed after being established for one MV.",
                    step=_int_or_none(event.get("step")),
                    vehicle_id=_vehicle_id(event),
                    payload={
                        "first_key": first_key_by_mv[mv_id],
                        "current_key": key,
                    },
                ),
            )
    return ()


def _check_merge_check(summary: Mapping[str, Any]) -> tuple[AcceptanceIssue, ...]:
    events = _events(summary, "merge_check_events")
    if not events:
        return (
            AcceptanceIssue(
                check_id="ACCEPT-006",
                severity=ERROR,
                message="No merge_check event was recorded.",
            ),
        )
    for event in events:
        if "merge_check_result" not in event:
            return (
                AcceptanceIssue(
                    check_id="ACCEPT-006",
                    severity=ERROR,
                    message="Merge check event is missing merge_check_result.",
                    step=_int_or_none(event.get("step")),
                    vehicle_id=_vehicle_id(event),
                ),
            )
    return ()


def _check_trajectory_events(summary: Mapping[str, Any]) -> tuple[AcceptanceIssue, ...]:
    events = _events(summary, "trajectory_events")
    if not events:
        return (
            AcceptanceIssue(
                check_id="ACCEPT-007",
                severity=ERROR,
                message="No planned trajectory event was recorded.",
            ),
        )
    for event in events:
        if event.get("trajectory_kind") is None:
            return (
                AcceptanceIssue(
                    check_id="ACCEPT-007",
                    severity=ERROR,
                    message="Trajectory event is missing trajectory_kind.",
                    step=_int_or_none(event.get("step")),
                    vehicle_id=_vehicle_id(event),
                ),
            )
        if event.get("progress_step") is None or event.get("duration_steps") is None:
            return (
                AcceptanceIssue(
                    check_id="ACCEPT-007",
                    severity=ERROR,
                    message="Trajectory event progress is not readable.",
                    step=_int_or_none(event.get("step")),
                    vehicle_id=_vehicle_id(event),
                ),
            )
    return ()


def _check_merge_completion(summary: Mapping[str, Any]) -> tuple[AcceptanceIssue, ...]:
    if _events(summary, "merge_completion_events"):
        return ()
    return (
        AcceptanceIssue(
            check_id="ACCEPT-008",
            severity=ERROR,
            message="No merge completion event was recorded.",
        ),
    )


def _check_final_merge_state(summary: Mapping[str, Any]) -> tuple[AcceptanceIssue, ...]:
    issues: list[AcceptanceIssue] = []
    for event in _events(summary, "merge_completion_events"):
        mv_id = _vehicle_id(event)
        if mv_id is None:
            continue
        final_state = _final_vehicle_state(summary, mv_id)
        if final_state is None:
            issues.append(
                AcceptanceIssue(
                    check_id="ACCEPT-009",
                    severity=ERROR,
                    message="Completed MV is missing from final_vehicle_states.",
                    step=_int_or_none(event.get("step")),
                    vehicle_id=mv_id,
                )
            )
            continue
        expected = {
            "physical_lane": "lane_2",
            "road_role": "mainline",
            "merge_state": "normal",
        }
        mismatches = {
            key: final_state.get(key)
            for key, expected_value in expected.items()
            if final_state.get(key) != expected_value
        }
        if mismatches:
            issues.append(
                AcceptanceIssue(
                    check_id="ACCEPT-009",
                    severity=ERROR,
                    message="Completed MV final state is not mainline/lane_2/normal.",
                    step=_int_or_none(event.get("step")),
                    vehicle_id=mv_id,
                    payload={"actual": dict(final_state), "mismatches": mismatches},
                )
            )
    return tuple(issues)


def _check_old_state_pollution(summary: Mapping[str, Any]) -> tuple[AcceptanceIssue, ...]:
    issues: list[AcceptanceIssue] = []
    if summary.get("old_assignment_record_count") != 0:
        issues.append(
            AcceptanceIssue(
                check_id="ACCEPT-010",
                severity=ERROR,
                message="Old assignment records polluted the new algorithm path.",
                payload={"old_assignment_record_count": summary.get("old_assignment_record_count")},
            )
        )
    if summary.get("old_active_maneuver_count") != 0:
        issues.append(
            AcceptanceIssue(
                check_id="ACCEPT-011",
                severity=ERROR,
                message="Old active maneuvers polluted the new algorithm path.",
                payload={"old_active_maneuver_count": summary.get("old_active_maneuver_count")},
            )
        )
    return tuple(issues)


def _build_lifecycles(
    summary: Mapping[str, Any],
) -> dict[str, VehicleLifecycleSummary]:
    vehicle_ids = _vehicle_ids_seen_in_acceptance(summary)
    lifecycles: dict[str, VehicleLifecycleSummary] = {}
    for vehicle_id in vehicle_ids:
        final_state = _final_vehicle_state(summary, vehicle_id) or {}
        lifecycles[vehicle_id] = VehicleLifecycleSummary(
            vehicle_id=vehicle_id,
            entered_control_zone_step=_first_zone_step(summary, vehicle_id, "control_zone"),
            trigger_steps=_trigger_steps(summary, vehicle_id),
            selected_gap_steps=_event_steps(summary, "gap_selection_events", vehicle_id),
            locked_gap_step=_first_event_step(summary, "gap_lock_events", vehicle_id),
            entered_merging_step=_first_merging_step(summary, vehicle_id),
            merge_completed_step=_first_event_step(summary, "merge_completion_events", vehicle_id),
            final_physical_lane=final_state.get("physical_lane"),
            final_road_role=final_state.get("road_role"),
            final_merge_state=final_state.get("merge_state"),
        )
    return lifecycles


def _build_human_report(
    scenario_id: str,
    passed: bool,
    lifecycles: Mapping[str, VehicleLifecycleSummary],
    integrity: Mapping[str, Any],
    issues: list[AcceptanceIssue],
    summary: Mapping[str, Any],
) -> str:
    lines: list[str] = [f"Scenario {scenario_id}: {'PASS' if passed else 'FAIL'}", ""]
    for vehicle_id, lifecycle in lifecycles.items():
        selected_event = _first_event(summary, "gap_selection_events", vehicle_id)
        locked_event = _first_event(summary, "gap_lock_events", vehicle_id)
        progress = _trajectory_progress_range(summary, vehicle_id)
        lines.extend(
            [
                f"{vehicle_id}:",
                f"  entered control_zone: step {_fmt_step(lifecycle.entered_control_zone_step)}",
                (
                    "  selected current_plan_gap: "
                    f"step {_fmt_step(lifecycle.selected_gap_steps[0] if lifecycle.selected_gap_steps else None)}, "
                    f"{_gap_brief(selected_event.get('selected_gap') if selected_event else None)}"
                ),
                (
                    "  locked_gap: "
                    f"step {_fmt_step(lifecycle.locked_gap_step)}, "
                    f"{_gap_brief(locked_event.get('locked_gap') if locked_event else None)}"
                ),
                f"  entered merging: step {_fmt_step(lifecycle.entered_merging_step)}",
                f"  trajectory: {progress}",
                f"  merge completed: step {_fmt_step(lifecycle.merge_completed_step)}",
                (
                    "  final: "
                    f"road_role={lifecycle.final_road_role}, "
                    f"physical_lane={lifecycle.final_physical_lane}, "
                    f"merge_state={lifecycle.final_merge_state}"
                ),
                "",
            ]
        )
    lines.extend(
        [
            "Integrity:",
            f"  old_assignment_record_count={integrity.get('old_assignment_record_count')}",
            f"  old_active_maneuver_count={integrity.get('old_active_maneuver_count')}",
            f"  non_trigger_gap_event_count={integrity.get('non_trigger_gap_event_count')}",
            f"  multi_mv_gap_conflict={integrity.get('multi_mv_gap_conflict')}",
        ]
    )
    if issues:
        lines.append("")
        lines.append("Issues:")
        for issue in issues:
            lines.append(
                f"  {issue.check_id} [{issue.severity}]: {issue.message}"
            )
    return "\n".join(lines)


def _events(summary: Mapping[str, Any], key: str) -> tuple[Mapping[str, Any], ...]:
    value = summary.get(key, ())
    if not isinstance(value, (list, tuple)):
        return ()
    return tuple(event for event in value if isinstance(event, Mapping))


def _is_structured_gap_ref(value: Any) -> bool:
    if not isinstance(value, Mapping):
        return False
    return all(
        field_name in value
        for field_name in (
            "index",
            "front_vehicle_id",
            "rear_vehicle_id",
        )
    )


def _structured_gap_key(value: Mapping[str, Any]) -> tuple[Any, ...] | None:
    if not _is_structured_gap_ref(value):
        return None
    return (
        value.get("snapshot_step"),
        value.get("index"),
        value.get("front_vehicle_id"),
        value.get("rear_vehicle_id"),
    )


def _vehicle_id(event: Mapping[str, Any]) -> str | None:
    value = event.get("mv_id")
    return str(value) if value is not None else None


def _int_or_none(value: Any) -> int | None:
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def _final_vehicle_state(
    summary: Mapping[str, Any],
    vehicle_id: str,
) -> Mapping[str, Any] | None:
    final_states = summary.get("final_vehicle_states")
    if not isinstance(final_states, Mapping):
        return None
    value = final_states.get(vehicle_id)
    return value if isinstance(value, Mapping) else None


def _vehicle_ids_seen_in_acceptance(summary: Mapping[str, Any]) -> tuple[str, ...]:
    seen: dict[str, None] = {}
    runtime_mv_ids = summary.get("runtime_mv_ids")
    if isinstance(runtime_mv_ids, (list, tuple)):
        for vehicle_id in runtime_mv_ids:
            seen[str(vehicle_id)] = None
    runtime_mv_states = summary.get("runtime_mv_states")
    if isinstance(runtime_mv_states, Mapping):
        for vehicle_id in runtime_mv_states:
            seen[str(vehicle_id)] = None
    for event in _events(summary, "zone_state_timeline"):
        zones = event.get("zone_state_by_mv")
        if isinstance(zones, Mapping):
            for vehicle_id in zones:
                seen[str(vehicle_id)] = None
    for event in _events(summary, "trigger_events"):
        entry_ids = event.get("entry_vehicle_ids")
        if isinstance(entry_ids, (list, tuple)):
            for vehicle_id in entry_ids:
                seen[str(vehicle_id)] = None
    for key in (
        "gap_selection_events",
        "gap_lock_events",
        "merge_check_events",
        "trajectory_events",
        "merge_completion_events",
    ):
        for event in _events(summary, key):
            vehicle_id = _vehicle_id(event)
            if vehicle_id is not None:
                seen[vehicle_id] = None
    return tuple(seen)


def _first_zone_step(
    summary: Mapping[str, Any],
    vehicle_id: str,
    zone_state: str,
) -> int | None:
    for event in _events(summary, "zone_state_timeline"):
        zones = event.get("zone_state_by_mv")
        if isinstance(zones, Mapping) and zones.get(vehicle_id) == zone_state:
            return _int_or_none(event.get("step"))
    return None


def _trigger_steps(summary: Mapping[str, Any], vehicle_id: str) -> tuple[int, ...]:
    steps: list[int] = []
    for event in _events(summary, "trigger_events"):
        step = _int_or_none(event.get("step"))
        if step is None or not event.get("trigger_plan"):
            continue
        entry_ids = event.get("entry_vehicle_ids")
        if isinstance(entry_ids, (list, tuple)) and vehicle_id in entry_ids:
            steps.append(step)
        elif event.get("trigger_reason") == "periodic":
            steps.append(step)
    return tuple(dict.fromkeys(steps))


def _event_steps(
    summary: Mapping[str, Any],
    key: str,
    vehicle_id: str,
) -> tuple[int, ...]:
    steps: list[int] = []
    for event in _events(summary, key):
        if _vehicle_id(event) == vehicle_id:
            step = _int_or_none(event.get("step"))
            if step is not None:
                steps.append(step)
    return tuple(steps)


def _first_event_step(
    summary: Mapping[str, Any],
    key: str,
    vehicle_id: str,
) -> int | None:
    steps = _event_steps(summary, key, vehicle_id)
    return steps[0] if steps else None


def _first_merging_step(summary: Mapping[str, Any], vehicle_id: str) -> int | None:
    for event in _events(summary, "merge_state_timeline"):
        states = event.get("merge_state_by_mv")
        if isinstance(states, Mapping) and states.get(vehicle_id) == "merging":
            return _int_or_none(event.get("step"))
    merge_check_step = _first_event_step(summary, "merge_check_events", vehicle_id)
    if merge_check_step is None:
        return None
    for event in _events(summary, "trajectory_events"):
        if (
            _vehicle_id(event) == vehicle_id
            and event.get("trajectory_kind") == "merge_execution"
        ):
            return _int_or_none(event.get("step"))
    return merge_check_step


def _first_event(
    summary: Mapping[str, Any],
    key: str,
    vehicle_id: str,
) -> Mapping[str, Any] | None:
    for event in _events(summary, key):
        if _vehicle_id(event) == vehicle_id:
            return event
    return None


def _trajectory_progress_range(summary: Mapping[str, Any], vehicle_id: str) -> str:
    progress_values = [
        event.get("progress_step")
        for event in _events(summary, "trajectory_events")
        if _vehicle_id(event) == vehicle_id
        and event.get("trajectory_kind") == "merge_execution"
        and event.get("progress_step") is not None
    ]
    completion = _first_event(summary, "merge_completion_events", vehicle_id)
    if completion is not None and completion.get("progress_step") is not None:
        progress_values.append(completion.get("progress_step"))
    if not progress_values:
        return "not recorded"
    duration = (
        completion.get("duration_steps")
        if completion is not None
        else None
    )
    if duration is None:
        duration = "?"
    return f"merge_execution, progress {min(progress_values)}/{duration} -> {max(progress_values)}/{duration}"


def _gap_brief(gap: Any) -> str:
    if not isinstance(gap, Mapping):
        return "gap not recorded"
    return (
        f"gap index={gap.get('index')}, "
        f"front={gap.get('front_vehicle_id')}, "
        f"rear={gap.get('rear_vehicle_id')}"
    )


def _fmt_step(step: int | None) -> str:
    return str(step) if step is not None else "not recorded"
