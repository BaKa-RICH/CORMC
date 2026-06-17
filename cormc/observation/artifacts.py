from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from datetime import UTC, datetime
from pathlib import Path
from typing import Any, Mapping

from cormc.observation.plotting import ObservationPlotArtifacts, build_observation_plot_artifacts
from cormc.observation.stage2_artifacts import load_stage2_observation_dataset
from cormc.observation.sumo_replay import (
    ObservationSumoReplayArtifacts,
    build_observation_sumo_replay_artifacts,
)
from cormc.sumo.trajectory_gui_replay import run_trajectory_gui_replay


@dataclass(frozen=True)
class ObservationArtifactBundle:
    scenario_id: str
    run_id: str
    source_dir: str
    status: str
    plots_dir: str
    replay_dir: str
    manifest_path: str
    report_path: str
    plot_artifacts: ObservationPlotArtifacts
    sumo_replay_artifacts: ObservationSumoReplayArtifacts
    validation: Mapping[str, Any]

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def build_observation_artifact_bundle(
    source_dir: str | Path,
    *,
    play: bool = False,
    smoke: bool = False,
) -> ObservationArtifactBundle:
    source_path = Path(source_dir)
    dataset = load_stage2_observation_dataset(source_path)
    plots_dir = source_path / "stage8_plots"
    replay_dir = source_path / "stage8_sumo_replay"
    manifest_path = source_path / "stage8_artifact_manifest.json"
    report_path = source_path / "stage8_report.md"

    plot_artifacts = build_observation_plot_artifacts(dataset, plots_dir)
    replay_artifacts = build_observation_sumo_replay_artifacts(
        dataset,
        replay_dir,
        validate_gui_smoke=smoke,
    )
    validation = _validation_payload(dataset, plot_artifacts, replay_artifacts)
    status = "passed" if validation["passed"] else "failed"
    manifest = _manifest_payload(
        dataset=dataset,
        plot_artifacts=plot_artifacts,
        replay_artifacts=replay_artifacts,
        validation=validation,
        status=status,
    )
    _write_json(manifest_path, manifest)
    report_path.write_text(_report_markdown(manifest), encoding="utf-8")

    if play:
        run_trajectory_gui_replay(
            replay_artifacts.sumo_config_path,
            replay_artifacts.replay_trajectory_path,
            track_vehicle_id=replay_artifacts.track_vehicle_id,
            delay_ms=150,
            hold_seconds=0.0,
            post_roll_steps=5,
            keep_open_after_replay=True,
        )

    return ObservationArtifactBundle(
        scenario_id=dataset.scenario_id,
        run_id=dataset.run_id,
        source_dir=str(source_path),
        status=status,
        plots_dir=str(plots_dir),
        replay_dir=str(replay_dir),
        manifest_path=str(manifest_path),
        report_path=str(report_path),
        plot_artifacts=plot_artifacts,
        sumo_replay_artifacts=replay_artifacts,
        validation=validation,
    )


def _validation_payload(
    dataset: Any,
    plot_artifacts: ObservationPlotArtifacts,
    replay_artifacts: ObservationSumoReplayArtifacts,
) -> dict[str, Any]:
    plot_paths = asdict(plot_artifacts)
    replay_paths = {
        "replay_trajectory_jsonl": replay_artifacts.replay_trajectory_path,
        "sumo_config": replay_artifacts.sumo_config_path,
        "play_gui_replay_ps1": replay_artifacts.gui_replay_script_path,
        "gui_smoke_status_json": replay_artifacts.gui_smoke_status_path,
    }
    source_files = _source_files(dataset.source_path)
    missing_paths = [
        path
        for path in [*plot_paths.values(), *replay_paths.values(), *source_files.values()]
        if not Path(path).exists()
    ]
    return {
        "passed": not missing_paths and replay_artifacts.replay_fidelity["status"] == "passed",
        "status": "validation passed"
        if not missing_paths and replay_artifacts.replay_fidelity["status"] == "passed"
        else "validation failed",
        "source_summary_schema": "five-level summary: scenario_summary / round_summaries / mv_summaries / cross_mv_summary / artifact_paths",
        "trajectory_record_count": len(dataset.trajectory_records),
        "vehicle_count": len(dataset.vehicle_ids()),
        "mv_ids": list(dataset.mv_ids),
        "step_range": list(dataset.step_range()),
        "t_range": list(dataset.t_range()),
        "gap_row_count": len(dataset.gap_rows),
        "plot_paths_checked": plot_paths,
        "replay_paths_checked": replay_paths,
        "missing_paths": missing_paths,
        "replay_fidelity": dict(replay_artifacts.replay_fidelity),
        "visual_replay_hint_count": replay_artifacts.visual_replay_hint_count,
    }


def _manifest_payload(
    *,
    dataset: Any,
    plot_artifacts: ObservationPlotArtifacts,
    replay_artifacts: ObservationSumoReplayArtifacts,
    validation: Mapping[str, Any],
    status: str,
) -> dict[str, Any]:
    source_files = _source_files(dataset.source_path)
    plot_paths = asdict(plot_artifacts)
    sumo_paths = {
        "replay_trajectory_jsonl": replay_artifacts.replay_trajectory_path,
        "sumo_config": replay_artifacts.sumo_config_path,
        "play_gui_replay_ps1": replay_artifacts.gui_replay_script_path,
        "gui_smoke_status_json": replay_artifacts.gui_smoke_status_path,
        "network_files": dict(replay_artifacts.network_files),
    }
    return {
        "artifact_schema": "observation_artifact.v1",
        "generated_at": datetime.now(UTC).replace(microsecond=0).isoformat(),
        "scenario_id": dataset.scenario_id,
        "run_id": dataset.run_id,
        "source_dir": str(dataset.source_path),
        "status": status,
        "source_files": source_files,
        "plot_paths": plot_paths,
        "sumo_replay_paths": sumo_paths,
        "manual_replay_command": _cli_command(dataset.source_path, "--play"),
        "smoke_replay_command": _cli_command(dataset.source_path, "--smoke"),
        "generated_script_manual_replay_command": replay_artifacts.manual_replay_command,
        "generated_script_smoke_replay_command": replay_artifacts.smoke_replay_command,
        "track_vehicle_id": replay_artifacts.track_vehicle_id,
        "role_map": dict(replay_artifacts.role_map),
        "validation": dict(validation),
        "boundary_statement": {
            "kind": "SUMO-GUI offline replay",
            "trajectory_source": "stage2 trajectory.csv",
            "not_closed_loop_authority": True,
            "scope": "Replay consumes committed observation records only; it does not repair trajectories or participate in decisions.",
        },
    }


def _report_markdown(manifest: Mapping[str, Any]) -> str:
    validation = manifest["validation"]
    lines = [
        f"# Observation Artifact: {manifest['scenario_id']}",
        "",
        "## Scenario",
        "",
        f"- scenario_id: `{manifest['scenario_id']}`",
        f"- run_id: `{manifest['run_id']}`",
        f"- status: `{manifest['status']}`",
        f"- mv_ids: `{', '.join(validation['mv_ids'])}`",
        f"- step_range: `{validation['step_range']}`",
        f"- trajectory_records: `{validation['trajectory_record_count']}`",
        "",
        "## Input Data",
        "",
        "- source: five-level summary / trajectory.csv / gap_rows.json",
        f"- source_dir: `{manifest['source_dir']}`",
        f"- summary_schema: `{validation['source_summary_schema']}`",
        f"- gap_rows: `{validation['gap_row_count']}`",
    ]
    for name, value in manifest["source_files"].items():
        lines.append(f"- {name}: `{value}`")
    lines.extend(["", "## Plots", ""])
    for name, value in manifest["plot_paths"].items():
        lines.append(f"- {name}: `{value}`")
    lines.extend(["", "## SUMO Replay", ""])
    lines.append("- mode: `offline SUMO-GUI replay`")
    lines.append("- not_closed_loop_authority: `true`")
    lines.append(f"- replay_fidelity: `{validation['replay_fidelity']['status']}`")
    lines.append(f"- visual_replay_hint_count: `{validation['visual_replay_hint_count']}`")
    for name, value in manifest["sumo_replay_paths"].items():
        if isinstance(value, dict):
            continue
        lines.append(f"- {name}: `{value}`")
    lines.extend(["", "## Lifecycle", ""])
    lines.append("| mv_id | locked | lateral_start | lateral_completed | mainline_conversion | final |")
    lines.append("| --- | ---: | ---: | ---: | ---: | --- |")
    for mv_id, lifecycle in _lifecycles_for_report(manifest).items():
        lines.append(
            f"| `{mv_id}` | `{lifecycle['locked_gap_step']}` | `{lifecycle['lateral_start_step']}` | "
            f"`{lifecycle['lateral_completed_step']}` | `{lifecycle['mainline_conversion_step']}` | "
            f"`{lifecycle['final_physical_lane']}/{lifecycle['final_road_role']}/{lifecycle['final_merge_state']}` |"
        )
    lines.extend(
        [
            "",
            "## Validation",
            "",
            f"- status: `{validation['status']}`",
            f"- missing_paths: `{validation['missing_paths']}`",
            f"- checked_records: `{validation['replay_fidelity']['checked_records']}`",
            "",
            "## Commands",
            "",
            "```powershell",
            manifest["manual_replay_command"],
            "```",
            "",
            "```powershell",
            manifest["smoke_replay_command"],
            "```",
            "",
        ]
    )
    return "\n".join(lines)


def _lifecycles_for_report(manifest: Mapping[str, Any]) -> dict[str, Any]:
    summary_path = Path(manifest["source_files"]["stage2_summary_json"])
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    lifecycles: dict[str, Any] = {}
    for mv_id, mv_summary in summary["mv_summaries"].items():
        lifecycle = dict(mv_summary["lifecycle"])
        final = dict(lifecycle["final_status"])
        lifecycles[mv_id] = {
            "locked_gap_step": lifecycle.get("locked_gap_step"),
            "lateral_start_step": lifecycle.get("lateral_start_step"),
            "lateral_completed_step": lifecycle.get("lateral_completed_step"),
            "mainline_conversion_step": lifecycle.get("mainline_conversion_step"),
            "final_physical_lane": final.get("physical_lane"),
            "final_road_role": final.get("road_role"),
            "final_merge_state": final.get("merge_state"),
        }
    return lifecycles


def _source_files(source_path: Path) -> dict[str, str]:
    return {
        "stage2_summary_json": str(source_path / "stage2_summary.json"),
        "trajectory_csv": str(source_path / "trajectory.csv"),
        "gap_rows_json": str(source_path / "gap_rows.json"),
    }


def _cli_command(source_path: Path, flag: str) -> str:
    return (
        "$env:PYTHONIOENCODING='utf-8'; "
        f'python -m cormc.observation.cli --source-dir "{source_path.resolve()}" {flag}'
    )


def _write_json(path: Path, payload: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True),
        encoding="utf-8",
    )
