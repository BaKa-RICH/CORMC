from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping

from cormc.legacy.ramp_merge_plotting import (
    RampMergeTrajectoryPlotRequest,
    build_ramp_merge_local_xt_plot_series,
    build_ramp_merge_plot_artifacts,
    build_ramp_merge_vt_plot_series,
)
from cormc.simulation_core.commit import OutputHistory


@dataclass(frozen=True)
class OneStepStage1PlotArtifacts:
    trajectory_csv_path: str
    x_t_local_plot_path: str
    v_t_plot_path: str


def build_onestep_stage1_plot_request(
    summary: Mapping[str, Any],
) -> RampMergeTrajectoryPlotRequest:
    mv_id = str(summary["mv_id"])
    first_check_mv_local_frame = dict(summary["first_check_mv_local_frame"])
    lane_2_vehicle_order = tuple(first_check_mv_local_frame.get("lane_2_vehicle_order") or ())
    selected_vehicle_ids = (mv_id, *lane_2_vehicle_order)
    return RampMergeTrajectoryPlotRequest(
        selected_vehicle_ids=selected_vehicle_ids,
        origin_x_global=float(first_check_mv_local_frame["origin_x_global"]),
        anchor_step=int(summary["first_check_step"]),
        anchor_t=float(summary["first_check_t"]),
        anchor_x_local_by_id=_first_check_x_local_by_id(summary),
        color_by_vehicle_id={
            vehicle_id: _series_color(vehicle_id)
            for vehicle_id in selected_vehicle_ids
        },
        label_by_vehicle_id={
            vehicle_id: vehicle_id
            for vehicle_id in selected_vehicle_ids
        },
        trajectory_csv_filename="trajectory.csv",
        x_t_plot_filename="x_t_local.png",
        v_t_plot_filename="v_t.png",
        x_t_title="OneStep Stage 1 Local X/T Trajectories",
        v_t_title="OneStep Stage 1 V/T Trajectories",
    )


def build_onestep_stage1_xt_plot_series(
    history: OutputHistory,
    summary: Mapping[str, Any],
) -> list[dict[str, object]]:
    return build_ramp_merge_local_xt_plot_series(
        history,
        build_onestep_stage1_plot_request(summary),
    )


def build_onestep_stage1_vt_plot_series(
    history: OutputHistory,
    summary: Mapping[str, Any],
) -> list[dict[str, object]]:
    return build_ramp_merge_vt_plot_series(
        history,
        build_onestep_stage1_plot_request(summary),
    )


def build_onestep_stage1_plot_artifacts(
    history: OutputHistory,
    summary: Mapping[str, Any],
    output_dir: str | Path,
) -> OneStepStage1PlotArtifacts:
    artifacts = build_ramp_merge_plot_artifacts(
        history,
        build_onestep_stage1_plot_request(summary),
        output_dir,
    )
    return OneStepStage1PlotArtifacts(
        trajectory_csv_path=artifacts.trajectory_csv_path,
        x_t_local_plot_path=artifacts.x_t_plot_path,
        v_t_plot_path=artifacts.v_t_plot_path,
    )


def _first_check_x_local_by_id(summary: Mapping[str, Any]) -> dict[str, float]:
    mv_id = str(summary["mv_id"])
    first_check_mv_local_frame = dict(summary["first_check_mv_local_frame"])
    lane_2_vehicle_x_local_by_id = dict(
        first_check_mv_local_frame.get("lane_2_vehicle_x_local_by_id") or {}
    )
    return {
        mv_id: float(first_check_mv_local_frame.get("x_m0_local") or 0.0),
        **{
            vehicle_id: float(value)
            for vehicle_id, value in lane_2_vehicle_x_local_by_id.items()
        },
    }

def _series_color(vehicle_id: str) -> str:
    return "red" if vehicle_id[-3:] == "_MV" else "blue"
