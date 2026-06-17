from __future__ import annotations

import csv
import json
import traceback
from collections.abc import Sequence
from datetime import UTC, datetime
from pathlib import Path
from typing import Any, Mapping

from cormc.observation.artifacts import build_observation_artifact_bundle
from cormc.onestep.rolling.stage2_analysis import export_onestep_stage2_analysis
from cormc.onestep.rolling.stage2_runner import run_onestep_stage2_history
from cormc.scenes.multimv import (
    RM_MULTIMV_SCENARIO_IDS,
    get_multimv_case_spec,
    multimv_mv_vehicle_ids,
)


def multimv_hard_step_limit(mv_count: int) -> int:
    return 420 + 180 * (int(mv_count) - 1)


def run_multimv_rolling_archive(
    output_root: Path,
    scenario_ids: Sequence[str] | None = None,
    run_id: str | None = None,
) -> dict[str, object]:
    output = Path(output_root)
    output.mkdir(parents=True, exist_ok=True)
    resolved_run_id = run_id or _default_run_id()
    resolved_scenario_ids = (
        tuple(str(scenario_id) for scenario_id in scenario_ids)
        if scenario_ids is not None
        else RM_MULTIMV_SCENARIO_IDS
    )

    results = [
        run_one_multimv_archive(scenario_id, output, resolved_run_id)
        for scenario_id in resolved_scenario_ids
    ]
    manifest_path = output / "multimv_run_manifest.json"
    summary_csv_path = output / "multimv_summary.csv"
    report_path = output / "multimv_report.md"
    manifest = {
        "artifact_schema": "multimv_rolling_archive.v1",
        "generated_at": datetime.now(UTC).replace(microsecond=0).isoformat(),
        "run_id": resolved_run_id,
        "output_root": str(output),
        "scenario_count": len(results),
        "results": results,
    }
    _write_json(manifest_path, manifest)
    _write_summary_csv(summary_csv_path, results)
    report_path.write_text(
        _build_batch_report(
            run_id=resolved_run_id,
            output_root=output,
            results=results,
        ),
        encoding="utf-8",
    )
    return {
        "run_id": resolved_run_id,
        "output_root": str(output),
        "scenario_count": len(results),
        "results": results,
        "manifest_path": str(manifest_path),
        "summary_csv_path": str(summary_csv_path),
        "report_path": str(report_path),
    }


def run_one_multimv_archive(
    scenario_id: str,
    output_root: Path,
    run_id: str,
) -> dict[str, object]:
    output_dir = Path(output_root) / scenario_id / run_id
    try:
        output_dir.mkdir(parents=True, exist_ok=True)
        case = get_multimv_case_spec(scenario_id)
        max_steps = multimv_hard_step_limit(case.mv_count)
        mv_ids = list(multimv_mv_vehicle_ids(case))
        history_run = run_onestep_stage2_history(
            scenario_id,
            max_steps=max_steps,
            run_id=run_id,
        )
        artifact_result = export_onestep_stage2_analysis(
            history_run.summary,
            output_dir,
            history=history_run.history,
        )
        summary = artifact_result.summary
        mv_lifecycle = _mv_lifecycle(summary)
        completed = _completed_mv_count(mv_lifecycle)
        completed_status = completed == case.mv_count
        status = "completed" if completed_status else "incomplete"
        observation_manifest = None
        sumo_replay_dir = None
        try:
            observation = build_observation_artifact_bundle(
                output_dir,
                play=False,
                smoke=False,
            )
            observation_manifest = observation.manifest_path
            sumo_replay_dir = observation.replay_dir
            if observation.status != "passed":
                status = (
                    "observation_failed"
                    if completed_status
                    else "incomplete_observation_failed"
                )
        except Exception as exc:
            diagnostics = _diagnostics(summary, exc)
            return _result_row(
                scenario_id=scenario_id,
                case_id=case.id,
                status=(
                    "observation_failed"
                    if completed_status
                    else "incomplete_observation_failed"
                ),
                mv_count=case.mv_count,
                steps_run=_actual_steps(summary),
                max_steps=max_steps,
                mv_ids=mv_ids,
                mv_lifecycle=mv_lifecycle,
                selected_gap_records=_selected_gap_records(summary),
                diagnostics=diagnostics,
                stage2_output_dir=str(output_dir),
                observation_manifest=observation_manifest,
                sumo_replay_dir=sumo_replay_dir,
            )
        return _result_row(
            scenario_id=scenario_id,
            case_id=case.id,
            status=status,
            mv_count=case.mv_count,
            steps_run=_actual_steps(summary),
            max_steps=max_steps,
            mv_ids=mv_ids,
            mv_lifecycle=mv_lifecycle,
            selected_gap_records=_selected_gap_records(summary),
            diagnostics=_diagnostics(summary, None),
            stage2_output_dir=str(output_dir),
            observation_manifest=observation_manifest,
            sumo_replay_dir=sumo_replay_dir,
        )
    except Exception as exc:
        return _result_row(
            scenario_id=scenario_id,
            case_id=_case_id_or_none(scenario_id),
            status="exception",
            mv_count=_mv_count_or_zero(scenario_id),
            steps_run=0,
            max_steps=_max_steps_or_zero(scenario_id),
            mv_ids=_mv_ids_or_empty(scenario_id),
            mv_lifecycle={},
            selected_gap_records=[],
            diagnostics=_exception_diagnostics(exc),
            stage2_output_dir=str(output_dir),
            observation_manifest=None,
            sumo_replay_dir=None,
        )


def _result_row(
    *,
    scenario_id: str,
    case_id: str,
    status: str,
    mv_count: int,
    steps_run: int,
    max_steps: int,
    mv_ids: list[str],
    mv_lifecycle: Mapping[str, Mapping[str, Any]],
    selected_gap_records: list[dict[str, Any]],
    diagnostics: Mapping[str, Any],
    stage2_output_dir: str,
    observation_manifest: str | None,
    sumo_replay_dir: str | None,
) -> dict[str, object]:
    return {
        "scenario_id": scenario_id,
        "case_id": case_id,
        "status": status,
        "mv_count": mv_count,
        "steps_run": steps_run,
        "max_steps": max_steps,
        "mv_ids": mv_ids,
        "mv_lifecycle": {key: dict(value) for key, value in mv_lifecycle.items()},
        "final_statuses": _final_statuses(mv_lifecycle),
        "selected_gap_records": selected_gap_records,
        "diagnostics": dict(diagnostics),
        "stage2_output_dir": stage2_output_dir,
        "observation_manifest": observation_manifest,
        "sumo_replay_dir": sumo_replay_dir,
    }


def _case_id_or_none(scenario_id: str) -> str | None:
    try:
        return get_multimv_case_spec(scenario_id).id
    except ValueError:
        return None


def _mv_count_or_zero(scenario_id: str) -> int:
    try:
        return get_multimv_case_spec(scenario_id).mv_count
    except ValueError:
        return 0


def _max_steps_or_zero(scenario_id: str) -> int:
    try:
        return multimv_hard_step_limit(get_multimv_case_spec(scenario_id).mv_count)
    except ValueError:
        return 0


def _mv_ids_or_empty(scenario_id: str) -> list[str]:
    try:
        return list(multimv_mv_vehicle_ids(get_multimv_case_spec(scenario_id)))
    except ValueError:
        return []


def _mv_lifecycle(summary: Mapping[str, Any]) -> dict[str, dict[str, Any]]:
    mv_summaries = dict(summary.get("mv_summaries") or {})
    result: dict[str, dict[str, Any]] = {}
    for mv_id, mv_summary in mv_summaries.items():
        lifecycle = dict(dict(mv_summary).get("lifecycle") or {})
        final_status = dict(lifecycle.get("final_status") or {})
        result[str(mv_id)] = {
            "locked_gap_step": lifecycle.get("locked_gap_step"),
            "lateral_start_step": lifecycle.get("lateral_start_step"),
            "lateral_completed_step": lifecycle.get("lateral_completed_step"),
            "mainline_conversion_step": lifecycle.get("mainline_conversion_step"),
            "final_physical_lane": final_status.get("physical_lane"),
            "final_road_role": final_status.get("road_role"),
            "final_merge_state": final_status.get("merge_state"),
            "runtime_present": final_status.get("runtime_present"),
        }
    return result


def _final_statuses(
    mv_lifecycle: Mapping[str, Mapping[str, Any]],
) -> dict[str, dict[str, Any]]:
    return {
        mv_id: {
            "physical_lane": lifecycle.get("final_physical_lane"),
            "road_role": lifecycle.get("final_road_role"),
            "merge_state": lifecycle.get("final_merge_state"),
            "runtime_present": lifecycle.get("runtime_present"),
        }
        for mv_id, lifecycle in mv_lifecycle.items()
    }


def _completed_mv_count(mv_lifecycle: Mapping[str, Mapping[str, Any]]) -> int:
    return sum(
        1
        for lifecycle in mv_lifecycle.values()
        if lifecycle.get("mainline_conversion_step") is not None
    )


def _selected_gap_records(summary: Mapping[str, Any]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for mv_id, mv_summary in dict(summary.get("mv_summaries") or {}).items():
        for row in dict(mv_summary).get("gap_rows") or []:
            if row.get("is_selected") is not True:
                continue
            rows.append(
                {
                    "mv_id": str(mv_id),
                    "round_id": row.get("round_id"),
                    "round_order": row.get("round_order"),
                    "step": row.get("step"),
                    "selected_gap_id": row.get("gap_id"),
                    "selected_gap_index": row.get("gap_index"),
                    "branch": row.get("branch"),
                    "front_controllable": row.get("front_controllable"),
                    "rear_controllable": row.get("rear_controllable"),
                    "reachable": row.get("reachable"),
                    "coop_feasible": row.get("coop_feasible"),
                    "failure_reason": row.get("failure_reason"),
                }
            )
    return rows


def _diagnostics(
    summary: Mapping[str, Any],
    exc: BaseException | None,
) -> dict[str, Any]:
    cross = dict(summary.get("cross_mv_summary") or {})
    diagnostics = {
        "gap_conflicts": len(cross.get("gap_conflicts") or []),
        "frontier_violations": len(cross.get("frontier_violations") or []),
        "ownership_conflicts": len(cross.get("ownership_conflicts") or []),
        "exception_type": None,
        "exception_message": None,
    }
    if exc is not None:
        diagnostics.update(_exception_diagnostics(exc))
    return diagnostics


def _exception_diagnostics(exc: BaseException) -> dict[str, Any]:
    return {
        "gap_conflicts": 0,
        "frontier_violations": 0,
        "ownership_conflicts": 0,
        "exception_type": type(exc).__name__,
        "exception_message": str(exc),
        "traceback": traceback.format_exception(type(exc), exc, exc.__traceback__),
    }


def _actual_steps(summary: Mapping[str, Any]) -> int:
    scenario = dict(summary.get("scenario_summary") or {})
    return int(scenario.get("actual_steps") or 0)


def _default_run_id() -> str:
    return datetime.now(UTC).strftime("multimv_rolling_%Y%m%d_%H%M%S")


def _write_json(path: Path, payload: Mapping[str, Any]) -> None:
    path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True),
        encoding="utf-8",
    )


def _write_summary_csv(path: Path, results: Sequence[Mapping[str, Any]]) -> None:
    fieldnames = (
        "scenario_id",
        "case_id",
        "status",
        "mv_count",
        "steps_run",
        "max_steps",
        "mv_completion_count",
        "stage2_output_dir",
        "observation_manifest",
        "sumo_replay_dir",
        "exception_type",
        "exception_message",
    )
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for result in results:
            diagnostics = dict(result.get("diagnostics") or {})
            writer.writerow(
                {
                    "scenario_id": result.get("scenario_id"),
                    "case_id": result.get("case_id"),
                    "status": result.get("status"),
                    "mv_count": result.get("mv_count"),
                    "steps_run": result.get("steps_run"),
                    "max_steps": result.get("max_steps"),
                    "mv_completion_count": _completed_mv_count(
                        dict(result.get("mv_lifecycle") or {})
                    ),
                    "stage2_output_dir": result.get("stage2_output_dir"),
                    "observation_manifest": result.get("observation_manifest"),
                    "sumo_replay_dir": result.get("sumo_replay_dir"),
                    "exception_type": diagnostics.get("exception_type"),
                    "exception_message": diagnostics.get("exception_message"),
                }
            )


def _build_batch_report(
    *,
    run_id: str,
    output_root: Path,
    results: Sequence[Mapping[str, Any]],
) -> str:
    lines = [
        "# Multi-MV Rolling Archive",
        "",
        "## Run Info",
        "",
        f"- run_id: `{run_id}`",
        f"- output_root: `{output_root}`",
        f"- scenario_count: `{len(results)}`",
        "",
        "## Scenario Summary",
        "",
        "| scenario_id | case_id | status | steps_run | max_steps | MV completion count | artifact path |",
        "| --- | --- | --- | ---: | ---: | ---: | --- |",
    ]
    for result in results:
        completion_count = _completed_mv_count(dict(result.get("mv_lifecycle") or {}))
        lines.append(
            "| `{scenario_id}` | `{case_id}` | `{status}` | `{steps}` | `{max_steps}` | `{complete}` | `{path}` |".format(
                scenario_id=result.get("scenario_id"),
                case_id=result.get("case_id"),
                status=result.get("status"),
                steps=result.get("steps_run"),
                max_steps=result.get("max_steps"),
                complete=completion_count,
                path=result.get("stage2_output_dir"),
            )
        )

    lines.extend(["", "## Incomplete/Failure Diagnostics", ""])
    incomplete = [result for result in results if result.get("status") != "completed"]
    if not incomplete:
        lines.append("- none")
    for result in incomplete:
        diagnostics = dict(result.get("diagnostics") or {})
        incomplete_mv_ids = [
            mv_id
            for mv_id, lifecycle in dict(result.get("mv_lifecycle") or {}).items()
            if dict(lifecycle).get("mainline_conversion_step") is None
        ]
        lines.append(
            "- `{scenario_id}` status `{status}`: incomplete_mvs `{incomplete}`, gap_conflicts `{gap}`, frontier_violations `{frontier}`, ownership_conflicts `{owner}`, exception `{etype}: {emsg}`".format(
                scenario_id=result.get("scenario_id"),
                status=result.get("status"),
                incomplete=incomplete_mv_ids,
                gap=diagnostics.get("gap_conflicts"),
                frontier=diagnostics.get("frontier_violations"),
                owner=diagnostics.get("ownership_conflicts"),
                etype=diagnostics.get("exception_type"),
                emsg=diagnostics.get("exception_message"),
            )
        )

    lines.extend(["", "## MV Lifecycle Summary", ""])
    for result in results:
        lines.append(f"### {result.get('scenario_id')}")
        lifecycle = dict(result.get("mv_lifecycle") or {})
        if not lifecycle:
            lines.append("- no lifecycle records")
            continue
        for mv_id, item in lifecycle.items():
            lines.append(
                "- `{mv_id}`: conversion `{conversion}`, final `{lane}/{role}/{state}`".format(
                    mv_id=mv_id,
                    conversion=dict(item).get("mainline_conversion_step"),
                    lane=dict(item).get("final_physical_lane"),
                    role=dict(item).get("final_road_role"),
                    state=dict(item).get("final_merge_state"),
                )
            )

    lines.extend(["", "## Replay Outputs", ""])
    for result in results:
        lines.append(
            "- `{scenario_id}`: observation `{manifest}`, replay `{replay}`".format(
                scenario_id=result.get("scenario_id"),
                manifest=result.get("observation_manifest"),
                replay=result.get("sumo_replay_dir"),
            )
        )
    return "\n".join(lines) + "\n"
