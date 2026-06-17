from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping

from cormc.observation.dataset import ObservationDataset, ObservationTrajectoryRecord, ordered_records
from cormc.observation.plotting import build_observation_plot_artifacts
from cormc.simulation_core.commit import OutputHistory


@dataclass(frozen=True)
class OneStepStage2PlotArtifacts:
    trajectory_csv_path: str
    process_x_t_local_plot_path: str
    process_v_t_plot_path: str
    process_y_t_plot_path: str
    lifecycle_timeline_plot_path: str
    first_trigger_trajectory_csv_path: str | None = None
    first_trigger_x_t_local_plot_path: str | None = None
    first_trigger_v_t_plot_path: str | None = None


def build_onestep_stage2_plot_artifacts(
    history: OutputHistory,
    summary: Mapping[str, Any],
    output_dir: str | Path,
) -> OneStepStage2PlotArtifacts:
    dataset = _dataset_from_history(summary, history, Path(output_dir))
    artifacts = build_observation_plot_artifacts(dataset, output_dir)
    return OneStepStage2PlotArtifacts(
        trajectory_csv_path=artifacts.trajectory_csv_path,
        process_x_t_local_plot_path=artifacts.process_x_t_local_plot_path,
        process_v_t_plot_path=artifacts.process_v_t_plot_path,
        process_y_t_plot_path=artifacts.process_y_t_plot_path,
        lifecycle_timeline_plot_path=artifacts.lifecycle_timeline_plot_path,
    )


def _dataset_from_history(
    summary: Mapping[str, Any],
    history: OutputHistory,
    source_dir: Path,
) -> ObservationDataset:
    scenario = dict(summary["scenario_summary"])
    mv_ids = tuple(str(mv_id) for mv_id in scenario["mv_ids"])
    lifecycles = {}
    for mv_id, mv_summary in summary["mv_summaries"].items():
        lifecycle = dict(mv_summary["lifecycle"])
        final = dict(lifecycle["final_status"])
        from cormc.observation.dataset import ObservationLifecycle

        lifecycles[str(mv_id)] = ObservationLifecycle(
            mv_id=str(mv_id),
            first_trigger_step=_optional_int(lifecycle.get("first_trigger_step")),
            locked_gap_step=_optional_int(lifecycle.get("locked_gap_step")),
            lateral_start_step=_optional_int(lifecycle.get("lateral_start_step")),
            lateral_completed_step=_optional_int(lifecycle.get("lateral_completed_step")),
            mainline_conversion_step=_optional_int(lifecycle.get("mainline_conversion_step")),
            final_physical_lane=str(final["physical_lane"]),
            final_road_role=str(final["road_role"]),
            final_merge_state=str(final["merge_state"]),
        )

    gap_rows = tuple(
        row
        for mv_summary in summary["mv_summaries"].values()
        for row in mv_summary.get("gap_rows") or []
    )
    records = tuple(
        ObservationTrajectoryRecord(
            scenario_id=record.scenario_id,
            run_id=record.run_id,
            step=record.step,
            t=record.t,
            vehicle_id=record.vehicle_id,
            vehicle_type=record.vehicle_type,
            compliance_state=record.compliance_state,
            x_global=record.x_global,
            y=record.y,
            v=record.v,
            a=record.a,
            physical_lane=record.physical_lane,
            road_role=record.road_role,
            primary_leader_id=record.primary_leader_id,
            lane_change_state=record.lane_change_state,
            merge_state=record.merge_state,
            active_event_tags=tuple(record.active_event_tags),
        )
        for record in history.trajectory_records
    )
    return ObservationDataset(
        scenario_id=str(scenario["scenario_id"]),
        run_id=str(scenario["run_id"]),
        source_dir=str(source_dir),
        summary=summary,
        trajectory_records=ordered_records(records),
        gap_rows=gap_rows,
        mv_ids=mv_ids,
        lifecycles=lifecycles,
        artifact_paths={str(key): str(value) for key, value in dict(summary.get("artifact_paths") or {}).items()},
    )


def _optional_int(value: Any) -> int | None:
    if value is None:
        return None
    return int(value)
