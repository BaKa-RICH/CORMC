from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping

from cormc.simulation_core.commit import OutputHistory

from cormc.legacy.onestep_stage1_plots import build_onestep_stage1_plot_artifacts
from cormc.legacy.onestep_stage1_runner import run_onestep_stage1_history


@dataclass(frozen=True)
class OneStepStage1ArtifactResult:
    scenario_id: str
    run_id: str
    summary: Mapping[str, Any]
    summary_json_path: str
    report_path: str
    trajectory_csv_path: str | None = None
    x_t_local_plot_path: str | None = None
    v_t_plot_path: str | None = None


def export_onestep_stage1_analysis(
    summary: Mapping[str, Any],
    output_dir: str | Path,
    *,
    history: OutputHistory | None = None,
) -> OneStepStage1ArtifactResult:
    scenario_id = str(summary["scenario_id"])
    run_id = str(summary["run_id"])
    scenario_dir = Path(output_dir)
    scenario_dir.mkdir(parents=True, exist_ok=True)

    summary_json_path = scenario_dir / "stage1_summary.json"
    report_path = scenario_dir / "stage1_report.md"
    trajectory_csv_path = None
    x_t_local_plot_path = None
    v_t_plot_path = None
    artifact_paths = {
        "summary_json": str(summary_json_path),
        "report_markdown": str(report_path),
    }
    if history is not None:
        plot_artifacts = build_onestep_stage1_plot_artifacts(history, summary, scenario_dir)
        trajectory_csv_path = plot_artifacts.trajectory_csv_path
        x_t_local_plot_path = plot_artifacts.x_t_local_plot_path
        v_t_plot_path = plot_artifacts.v_t_plot_path
        artifact_paths.update(
            {
                "trajectory_csv": trajectory_csv_path,
                "x_t_local_plot": x_t_local_plot_path,
                "v_t_plot": v_t_plot_path,
            }
        )
    exported_summary = {
        **dict(summary),
        "artifact_paths": artifact_paths,
    }
    _write_json(summary_json_path, exported_summary)
    report_path.write_text(build_onestep_stage1_report(exported_summary), encoding="utf-8")

    return OneStepStage1ArtifactResult(
        scenario_id=scenario_id,
        run_id=run_id,
        summary=exported_summary,
        summary_json_path=str(summary_json_path),
        report_path=str(report_path),
        trajectory_csv_path=trajectory_csv_path,
        x_t_local_plot_path=x_t_local_plot_path,
        v_t_plot_path=v_t_plot_path,
    )


def run_onestep_stage1_analysis(
    scenario_id: str,
    output_root: str | Path,
    *,
    max_steps: int | None = None,
    run_id: str = "onestep-stage1-analysis",
) -> OneStepStage1ArtifactResult:
    history_run = run_onestep_stage1_history(
        scenario_id=scenario_id,
        max_steps=max_steps,
        run_id=run_id,
    )
    return export_onestep_stage1_analysis(
        history_run.summary,
        Path(output_root) / scenario_id / run_id,
        history=history_run.history,
    )


def build_onestep_stage1_report(summary: Mapping[str, Any]) -> str:
    expectation = dict(summary["stage1_expectation"])
    first_check_mv_state = dict(summary["first_check_mv_state"] or {})
    first_check_mv_local_frame = dict(summary["first_check_mv_local_frame"] or {})
    lane_2_vehicle_order = list(first_check_mv_local_frame.get("lane_2_vehicle_order") or [])
    lane_2_vehicle_x_local_by_id = dict(
        first_check_mv_local_frame.get("lane_2_vehicle_x_local_by_id") or {}
    )
    gap_intervals_local = list(first_check_mv_local_frame.get("gap_intervals_local") or [])
    case_spec = dict(summary.get("case_spec") or {})
    expected_gap_intervals_local = list(case_spec.get("gap_intervals_local") or [])
    mv_id = str(summary.get("mv_id") or "")

    scenario_loaded = (
        summary.get("initial_zone_state") == expectation.get("expected_initial_zone_state")
    )
    mv_check_frame_ok = (
        int(summary.get("first_check_step") or -1) == int(expectation["expected_first_check_step"])
        and _float_close(
            float(summary.get("first_check_t") or 0.0),
            float(expectation["expected_first_check_t"]),
        )
        and _float_close(
            float(first_check_mv_state.get("x_global") or 0.0),
            float(expectation["expected_mv_x_global_at_check"]),
        )
        and _float_close(
            float(first_check_mv_local_frame.get("x_m0_local") or 0.0),
            float(expectation["expected_mv_x_local_at_check"]),
        )
        and first_check_mv_state.get("zone_state") == "control_zone"
    )
    lane_2_gap_geometry_ok = _gap_intervals_match_expected(
        gap_intervals_local,
        expected_gap_intervals_local,
    )
    trigger_reason = None
    if isinstance(summary.get("first_check_trigger_event"), Mapping):
        trigger_reason = summary["first_check_trigger_event"].get("trigger_reason")
    zone_change_summary = _zone_change_summary(summary.get("zone_state_timeline") or [], mv_id)

    lines = [
        f"# {summary['scenario_id']} OneStep Stage 1 Precheck",
        "",
        "## Verdict",
        f"- scenario loaded correctly: `{scenario_loaded}`",
        f"- MV reached the expected check frame: `{mv_check_frame_ok}`",
        f"- lane 2 gap geometry aligns with {case_spec.get('one_step_case_id')}: `{lane_2_gap_geometry_ok}`",
        "",
        "## Run",
        f"- mode: `{summary['mode']}`",
        f"- run_id: `{summary['run_id']}`",
        f"- actual_steps: `{summary['actual_steps']}` / max `{summary['max_steps']}`",
        f"- initial_zone_state: `{summary.get('initial_zone_state')}`",
        f"- expected_initial_zone_state: `{expectation['expected_initial_zone_state']}`",
        "",
        "## Initial Global Layout",
    ]
    for row in summary.get("initial_vehicle_table") or []:
        lines.append(
            "- {vehicle_id}: lane `{physical_lane}`, role `{road_role}`, x `{x_global}`, y `{y}`, v `{v}`".format(
                **row
            )
        )

    lines.extend(
        [
            "",
            "## First Check Frame",
            f"- step: `{summary['first_check_step']}`",
            f"- t: `{summary['first_check_t']}`",
            f"- trigger_reason: `{trigger_reason}`",
            f"- expected_trigger_reason: `{expectation['expected_first_trigger_reason']}`",
            f"- MV origin_x_global: `{first_check_mv_local_frame.get('origin_x_global')}`",
            f"- MV x_m0_local: `{first_check_mv_local_frame.get('x_m0_local')}`",
            f"- MV zone_state: `{first_check_mv_state.get('zone_state')}`",
            f"- MV physical_lane: `{first_check_mv_state.get('physical_lane')}`",
            f"- MV road_role: `{first_check_mv_state.get('road_role')}`",
            "",
            "## Lane 2 Local Frame",
            f"- lane_2_vehicle_order: `{lane_2_vehicle_order}`",
            f"- lane_2_vehicle_x_local_by_id: `{lane_2_vehicle_x_local_by_id}`",
            f"- gap_intervals_local: `{gap_intervals_local}`",
            f"- expected_gap_intervals_local: `{expected_gap_intervals_local}`",
            f"- gap_centers_local: `{first_check_mv_local_frame.get('gap_centers_local')}`",
            "",
            "## Zone State Changes",
            f"- {zone_change_summary}",
            "",
            "## Files",
        ]
    )
    for name, artifact_path in summary["artifact_paths"].items():
        lines.append(f"- {name}: `{artifact_path}`")
    return "\n".join(lines) + "\n"


def _gap_intervals_match_expected(
    gap_intervals_local: list[Any],
    expected_gap_intervals_local: list[Any],
) -> bool:
    if len(gap_intervals_local) != len(expected_gap_intervals_local):
        return False
    for observed, expected in zip(gap_intervals_local, expected_gap_intervals_local):
        if not isinstance(observed, (list, tuple)) or len(observed) != 2:
            return False
        if not isinstance(expected, (list, tuple)) or len(expected) != 2:
            return False
        if not _float_close(float(observed[0]), float(expected[0])):
            return False
        if not _float_close(float(observed[1]), float(expected[1])):
            return False
    return True


def _zone_change_summary(timeline: list[Any], mv_id: str) -> str:
    changes: list[str] = []
    last_zone = None
    for item in timeline:
        if not isinstance(item, Mapping):
            continue
        zone_state = (item.get("zone_state_by_mv") or {}).get(mv_id)
        if zone_state == last_zone:
            continue
        changes.append(f"step {item.get('step')}: {zone_state}")
        last_zone = zone_state
    return " -> ".join(changes) if changes else "no zone-state events"


def _float_close(left: float, right: float, tol: float = 1e-6) -> bool:
    return abs(left - right) <= tol


def _write_json(path: Path, payload: Mapping[str, Any]) -> None:
    path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True),
        encoding="utf-8",
    )
