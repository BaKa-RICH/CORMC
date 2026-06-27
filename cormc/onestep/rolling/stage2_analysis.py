from __future__ import annotations

import csv
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping

from cormc.simulation_core.commit import OutputHistory

from cormc.onestep.rolling.stage2_plots import build_onestep_stage2_plot_artifacts
from cormc.onestep.rolling.stage2_random_runner import (
    run_onestep_stage2_random_history,
)
from cormc.onestep.rolling.stage2_runner import run_onestep_stage2_history
from cormc.onestep.rolling.validation import (
    build_onestep_stage2_acceptance_report,
    build_onestep_stage2_random_acceptance_report,
)


@dataclass(frozen=True)
class OneStepStage2ArtifactResult:
    scenario_id: str
    run_id: str
    summary: Mapping[str, Any]
    summary_json_path: str
    report_path: str
    trajectory_csv_path: str
    process_x_t_local_plot_path: str
    process_v_t_plot_path: str
    process_y_t_plot_path: str
    lifecycle_timeline_plot_path: str
    gap_rows_json_path: str
    planning_timing_csv_path: str


def export_onestep_stage2_analysis(
    summary: Mapping[str, Any],
    output_dir: str | Path,
    *,
    history: OutputHistory,
) -> OneStepStage2ArtifactResult:
    scenario_dir = Path(output_dir)
    scenario_dir.mkdir(parents=True, exist_ok=True)
    plot_artifacts = build_onestep_stage2_plot_artifacts(history, summary, scenario_dir)
    summary_json_path = scenario_dir / "stage2_summary.json"
    report_path = scenario_dir / "stage2_report.md"
    gap_rows_json_path = scenario_dir / "gap_rows.json"
    planning_timing_csv_path = scenario_dir / "planning_timing_rows.csv"
    artifact_paths = {
        "summary_json": str(summary_json_path),
        "report_markdown": str(report_path),
        "trajectory_csv": plot_artifacts.trajectory_csv_path,
        "process_x_t_local_plot": plot_artifacts.process_x_t_local_plot_path,
        "process_v_t_plot": plot_artifacts.process_v_t_plot_path,
        "process_y_t_plot": plot_artifacts.process_y_t_plot_path,
        "lifecycle_timeline_plot": plot_artifacts.lifecycle_timeline_plot_path,
        "gap_rows_json": str(gap_rows_json_path),
        "planning_timing_csv": str(planning_timing_csv_path),
    }
    exported_summary = {
        **dict(summary),
        "artifact_paths": artifact_paths,
    }
    gap_rows_json_path.write_text(
        json.dumps(
            _flat_gap_rows(exported_summary),
            ensure_ascii=False,
            indent=2,
            sort_keys=True,
        ),
        encoding="utf-8",
    )
    _write_planning_timing_csv(planning_timing_csv_path, exported_summary)
    summary_json_path.write_text(
        json.dumps(exported_summary, ensure_ascii=False, indent=2, sort_keys=True),
        encoding="utf-8",
    )
    report_path.write_text(build_onestep_stage2_report(exported_summary), encoding="utf-8")
    return OneStepStage2ArtifactResult(
        scenario_id=str(summary["scenario_summary"]["scenario_id"]),
        run_id=str(summary["scenario_summary"]["run_id"]),
        summary=exported_summary,
        summary_json_path=str(summary_json_path),
        report_path=str(report_path),
        trajectory_csv_path=plot_artifacts.trajectory_csv_path,
        process_x_t_local_plot_path=plot_artifacts.process_x_t_local_plot_path,
        process_v_t_plot_path=plot_artifacts.process_v_t_plot_path,
        process_y_t_plot_path=plot_artifacts.process_y_t_plot_path,
        lifecycle_timeline_plot_path=plot_artifacts.lifecycle_timeline_plot_path,
        gap_rows_json_path=str(gap_rows_json_path),
        planning_timing_csv_path=str(planning_timing_csv_path),
    )


def run_onestep_stage2_analysis(
    scenario_id: str,
    output_root: str | Path,
    *,
    max_steps: int | None = None,
    run_id: str = "onestep-stage2-analysis",
) -> OneStepStage2ArtifactResult:
    history_run = run_onestep_stage2_history(
        scenario_id=scenario_id,
        max_steps=max_steps,
        run_id=run_id,
    )
    return export_onestep_stage2_analysis(
        history_run.summary,
        Path(output_root) / scenario_id / run_id,
        history=history_run.history,
    )


def run_onestep_stage2_random_analysis(
    scenario_id: str,
    output_root: str | Path,
    *,
    max_steps: int | None = None,
    run_id: str = "onestep-stage2-random-analysis",
) -> OneStepStage2ArtifactResult:
    history_run = run_onestep_stage2_random_history(
        scenario_id,
        max_steps=max_steps,
        run_id=run_id,
    )
    return export_onestep_stage2_analysis(
        history_run.summary,
        Path(output_root) / scenario_id / run_id,
        history=history_run.history,
    )


def build_onestep_stage2_report(summary: Mapping[str, Any]) -> str:
    scenario = dict(summary["scenario_summary"])
    cross = dict(summary["cross_mv_summary"])
    if scenario.get("traffic_mode") == "boundary_flow":
        validation = build_onestep_stage2_random_acceptance_report(summary)
    else:
        validation = build_onestep_stage2_acceptance_report(summary)
    status = "validation passed" if validation.passed else "validation failed"

    lines = [
        f"# {scenario['scenario_id']} OneStep Stage 2 Report",
        "",
        "## Scenario Summary",
        f"- run_id: `{scenario['run_id']}`",
        f"- algorithm_variant: `{scenario['algorithm_variant']}`",
        f"- actual_steps: `{scenario['actual_steps']}` / max `{scenario['max_steps']}`",
        f"- mv_ids: `{', '.join(str(mv_id) for mv_id in scenario['mv_ids'])}`",
        f"- formal_event_kinds: `{', '.join(sorted(scenario['event_counts']))}`",
        "",
        "## Round Summary",
        f"- round_count: `{len(summary['round_summaries'])}`",
    ]
    for round_summary in summary["round_summaries"]:
        lines.append(
            "- {round_id}: step `{step}`, mv_order `{mv_order}`, selected `{selected}`, locked `{locked}`".format(
                round_id=round_summary["round_id"],
                step=round_summary["step"],
                mv_order=round_summary["mv_order"],
                selected=round_summary["selected_gap_indices"],
                locked=round_summary["locked_gap_indices"],
            )
        )

    lines.extend(_planning_timing_report_lines(scenario))

    lines.extend(["", "## MV Lifecycle"])
    for mv_id, mv_summary in summary["mv_summaries"].items():
        lifecycle = dict(mv_summary["lifecycle"])
        final_status = dict(lifecycle["final_status"])
        lines.append(
            "- {mv_id}: locked `{locked}`, lateral `{lateral}`, completed `{completed}`, mainline `{mainline}`, final `{lane}/{role}/{state}`, gap_rows `{rows}`".format(
                mv_id=mv_id,
                locked=lifecycle.get("locked_gap_step"),
                lateral=lifecycle.get("lateral_start_step"),
                completed=lifecycle.get("lateral_completed_step"),
                mainline=lifecycle.get("mainline_conversion_step"),
                lane=final_status.get("physical_lane"),
                role=final_status.get("road_role"),
                state=final_status.get("merge_state"),
                rows=len(mv_summary.get("gap_rows") or []),
            )
        )

    lines.extend(
        [
            "",
            "## Cross-MV Validation",
            f"- status: `{status}`",
            f"- gap_conflicts: `{cross.get('gap_conflicts')}`",
            f"- frontier_violations: `{cross.get('frontier_violations')}`",
            f"- ownership_conflicts: `{cross.get('ownership_conflicts')}`",
            f"- final_runtime_leftovers: `{cross.get('final_runtime_leftovers')}`",
            f"- issue_ids: `{[issue.check_id for issue in validation.issues]}`",
            "",
            "## Artifacts",
        ]
    )
    for name, artifact_path in summary["artifact_paths"].items():
        lines.append(f"- {name}: `{artifact_path}`")
    return "\n".join(lines) + "\n"


def _planning_timing_report_lines(scenario: Mapping[str, Any]) -> list[str]:
    timing = dict(scenario.get("planning_timing_summary") or {})
    lines = [
        "",
        "## Planning Timing",
        f"- clock: `{timing.get('clock')}`",
        f"- timed_round_count: `{timing.get('timed_round_count')}`",
        "",
        "### By planned MV count",
        "| count | samples | mean_ms | median_ms | min_ms | max_ms | round_ids |",
        "| ---: | ---: | ---: | ---: | ---: | ---: | --- |",
    ]
    lines.extend(_timing_table_rows(timing.get("by_planned_mv_count") or {}))
    lines.extend(
        [
            "",
            "### By controlled vehicle count",
            "| count | samples | mean_ms | median_ms | min_ms | max_ms | round_ids |",
            "| ---: | ---: | ---: | ---: | ---: | ---: | --- |",
        ]
    )
    lines.extend(_timing_table_rows(timing.get("by_controlled_vehicle_count") or {}))
    return lines


def _timing_table_rows(grouped: Mapping[str, Any]) -> list[str]:
    rows: list[str] = []
    for count, payload in sorted(grouped.items(), key=lambda item: int(item[0])):
        item = dict(payload)
        rows.append(
            "| {count} | {samples} | {mean} | {median} | {min_value} | {max_value} | {round_ids} |".format(
                count=count,
                samples=item.get("sample_count"),
                mean=_format_ms(item.get("mean_ms")),
                median=_format_ms(item.get("median_ms")),
                min_value=_format_ms(item.get("min_ms")),
                max_value=_format_ms(item.get("max_ms")),
                round_ids=", ".join(str(round_id) for round_id in item.get("round_ids") or []),
            )
        )
    return rows


def _format_ms(value: Any) -> str:
    if value is None:
        return ""
    return f"{float(value):.6f}"


def _write_planning_timing_csv(path: Path, summary: Mapping[str, Any]) -> None:
    scenario = dict(summary["scenario_summary"])
    fieldnames = (
        "scenario_id",
        "run_id",
        "round_id",
        "step",
        "t",
        "trigger_reason",
        "active_trigger_reasons",
        "entry_vehicle_ids",
        "planned_mv_count",
        "planned_mv_ids",
        "controlled_vehicle_count",
        "controlled_vehicle_ids",
        "duration_ms",
        "duration_ns",
        "gap_count",
        "plan_count",
    )
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for round_summary in summary["round_summaries"]:
            timing = round_summary.get("planning_timing")
            if timing is None:
                continue
            writer.writerow(
                {
                    "scenario_id": scenario["scenario_id"],
                    "run_id": scenario["run_id"],
                    "round_id": timing["round_id"],
                    "step": timing["step"],
                    "t": timing["t"],
                    "trigger_reason": timing["trigger_reason"],
                    "active_trigger_reasons": ";".join(
                        str(value) for value in timing["active_trigger_reasons"]
                    ),
                    "entry_vehicle_ids": ";".join(
                        str(value) for value in timing["entry_vehicle_ids"]
                    ),
                    "planned_mv_count": timing["planned_mv_count"],
                    "planned_mv_ids": ";".join(
                        str(value) for value in timing["planned_mv_ids"]
                    ),
                    "controlled_vehicle_count": timing["controlled_vehicle_count"],
                    "controlled_vehicle_ids": ";".join(
                        str(value) for value in timing["controlled_vehicle_ids"]
                    ),
                    "duration_ms": timing["duration_ms"],
                    "duration_ns": timing["duration_ns"],
                    "gap_count": timing["gap_count"],
                    "plan_count": timing["plan_count"],
                }
            )


def _flat_gap_rows(summary: Mapping[str, Any]) -> list[dict[str, Any]]:
    scenario = dict(summary["scenario_summary"])
    rows: list[dict[str, Any]] = []
    for mv_id, mv_summary in summary["mv_summaries"].items():
        for row in mv_summary.get("gap_rows") or []:
            rows.append(
                {
                    "scenario_id": scenario["scenario_id"],
                    "run_id": scenario["run_id"],
                    "mv_id": mv_id,
                    "round_id": row.get("round_id"),
                    "step": row.get("step"),
                    "gap_index": row.get("gap_index"),
                    "branch": row.get("branch"),
                    "J": row.get("J"),
                    "is_selected": row.get("is_selected"),
                    "failure_reason": row.get("failure_reason"),
                }
            )
    return rows
