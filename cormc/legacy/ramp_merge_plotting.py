from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass, field
from pathlib import Path
from typing import Mapping

import matplotlib

matplotlib.use("Agg")

from matplotlib import pyplot as plt

from cormc.legacy.artifact_reports import export_trajectory_history
from cormc.simulation_core.commit import OutputHistory, TrajectoryRecord


@dataclass(frozen=True)
class RampMergeTrajectoryPlotRequest:
    selected_vehicle_ids: tuple[str, ...]
    origin_x_global: float
    v_t_vehicle_ids: tuple[str, ...] | None = None
    anchor_step: int | None = None
    anchor_t: float | None = None
    anchor_x_local_by_id: Mapping[str, float] = field(default_factory=dict)
    color_by_vehicle_id: Mapping[str, str] = field(default_factory=dict)
    label_by_vehicle_id: Mapping[str, str] = field(default_factory=dict)
    trajectory_csv_filename: str = "trajectory.csv"
    x_t_plot_filename: str = "x_t_local.png"
    v_t_plot_filename: str = "v_t.png"
    x_t_title: str = "Ramp-Merge Local X/T Trajectories"
    v_t_title: str = "Ramp-Merge V/T Trajectories"


@dataclass(frozen=True)
class RampMergePlotArtifacts:
    trajectory_csv_path: str
    x_t_plot_path: str
    v_t_plot_path: str


def build_ramp_merge_local_xt_plot_series(
    history: OutputHistory,
    request: RampMergeTrajectoryPlotRequest,
) -> list[dict[str, object]]:
    grouped = _group_records_by_vehicle(history.trajectory_records, request.selected_vehicle_ids)
    anchor_step = int(request.anchor_step) if request.anchor_step is not None else None
    series: list[dict[str, object]] = []
    for vehicle_id in request.selected_vehicle_ids:
        points: list[tuple[float, float]] = []
        for record in grouped.get(vehicle_id, []):
            if anchor_step is not None and int(record.step) == anchor_step:
                continue
            points.append(
                (
                    float(record.t),
                    float(record.x_global) - float(request.origin_x_global),
                )
            )
        if anchor_step is not None and vehicle_id in request.anchor_x_local_by_id:
            points.append(
                (
                    float(request.anchor_t),
                    float(request.anchor_x_local_by_id[vehicle_id]),
                )
            )
        points.sort(key=lambda item: item[0])
        series.append(
            {
                "vehicle_id": vehicle_id,
                "color": request.color_by_vehicle_id.get(vehicle_id, "blue"),
                "label": request.label_by_vehicle_id.get(vehicle_id, vehicle_id),
                "t": [t_value for t_value, _ in points],
                "value": [value for _, value in points],
            }
        )
    return series


def build_ramp_merge_vt_plot_series(
    history: OutputHistory,
    request: RampMergeTrajectoryPlotRequest,
) -> list[dict[str, object]]:
    vt_vehicle_ids = request.v_t_vehicle_ids or request.selected_vehicle_ids
    grouped = _group_records_by_vehicle(history.trajectory_records, vt_vehicle_ids)
    series: list[dict[str, object]] = []
    for vehicle_id in vt_vehicle_ids:
        records = grouped.get(vehicle_id, [])
        series.append(
            {
                "vehicle_id": vehicle_id,
                "color": request.color_by_vehicle_id.get(vehicle_id, "blue"),
                "label": request.label_by_vehicle_id.get(vehicle_id, vehicle_id),
                "t": [float(record.t) for record in records],
                "value": [float(record.v) for record in records],
            }
        )
    return series


def build_ramp_merge_plot_artifacts(
    history: OutputHistory,
    request: RampMergeTrajectoryPlotRequest,
    output_dir: str | Path,
) -> RampMergePlotArtifacts:
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)
    trajectory_csv_path = export_trajectory_history(
        history,
        output_path / request.trajectory_csv_filename,
    )
    x_t_plot_path = render_ramp_merge_local_xt_plot(history, request, output_path)
    v_t_plot_path = render_ramp_merge_vt_plot(history, request, output_path)
    return RampMergePlotArtifacts(
        trajectory_csv_path=str(trajectory_csv_path),
        x_t_plot_path=str(x_t_plot_path),
        v_t_plot_path=str(v_t_plot_path),
    )


def render_ramp_merge_local_xt_plot(
    history: OutputHistory,
    request: RampMergeTrajectoryPlotRequest,
    output_dir: str | Path,
) -> Path:
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)
    png_path = output_path / request.x_t_plot_filename
    series = build_ramp_merge_local_xt_plot_series(history, request)
    _render_time_series_plot(
        series,
        png_path,
        xlabel="t",
        ylabel="x_local",
        title=request.x_t_title,
    )
    return png_path


def render_ramp_merge_vt_plot(
    history: OutputHistory,
    request: RampMergeTrajectoryPlotRequest,
    output_dir: str | Path,
) -> Path:
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)
    png_path = output_path / request.v_t_plot_filename
    series = build_ramp_merge_vt_plot_series(history, request)
    _render_time_series_plot(
        series,
        png_path,
        xlabel="t",
        ylabel="v",
        title=request.v_t_title,
    )
    return png_path


def _render_time_series_plot(
    series: list[dict[str, object]],
    output_path: Path,
    *,
    xlabel: str,
    ylabel: str,
    title: str,
) -> None:
    fig, ax = plt.subplots(figsize=(10, 6))
    for item in series:
        ax.plot(item["t"], item["value"], color=item["color"], label=item["label"])
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.legend()
    fig.tight_layout()
    fig.savefig(output_path, dpi=150)
    plt.close(fig)


def _group_records_by_vehicle(
    records: list[TrajectoryRecord],
    selected_vehicle_ids: tuple[str, ...],
) -> dict[str, list[TrajectoryRecord]]:
    grouped: dict[str, list[TrajectoryRecord]] = defaultdict(list)
    selected_set = set(selected_vehicle_ids)
    for record in records:
        if record.vehicle_id not in selected_set:
            continue
        grouped[record.vehicle_id].append(record)
    for vehicle_id in grouped:
        grouped[vehicle_id].sort(key=lambda record: (float(record.t), int(record.step)))
    return dict(grouped)
