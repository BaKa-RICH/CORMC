from __future__ import annotations

import csv
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

import matplotlib

matplotlib.use("Agg")

from matplotlib import pyplot as plt

from cormc.observation.dataset import (
    TRAJECTORY_FIELDNAMES,
    ObservationDataset,
    ObservationTrajectoryRecord,
)


@dataclass(frozen=True)
class ObservationPlotArtifacts:
    trajectory_csv_path: str
    process_x_t_local_plot_path: str
    process_v_t_plot_path: str
    process_y_t_plot_path: str
    lifecycle_timeline_plot_path: str


def build_observation_plot_artifacts(
    dataset: ObservationDataset,
    output_dir: str | Path,
) -> ObservationPlotArtifacts:
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)

    trajectory_csv_path = write_observation_trajectory_csv(dataset, output_path / "trajectory.csv")
    x_t_path = output_path / "process_x_t_local.png"
    v_t_path = output_path / "process_v_t.png"
    y_t_path = output_path / "process_y_t.png"
    lifecycle_path = output_path / "lifecycle_timeline.png"

    render_observation_x_t_local_plot(dataset, x_t_path)
    render_observation_v_t_plot(dataset, v_t_path)
    render_observation_y_t_plot(dataset, y_t_path)
    render_observation_lifecycle_timeline(dataset, lifecycle_path)

    return ObservationPlotArtifacts(
        trajectory_csv_path=str(trajectory_csv_path),
        process_x_t_local_plot_path=str(x_t_path),
        process_v_t_plot_path=str(v_t_path),
        process_y_t_plot_path=str(y_t_path),
        lifecycle_timeline_plot_path=str(lifecycle_path),
    )


def write_observation_trajectory_csv(dataset: ObservationDataset, path: str | Path) -> Path:
    output_path = Path(path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=TRAJECTORY_FIELDNAMES)
        writer.writeheader()
        for record in dataset.trajectory_records:
            writer.writerow(record.to_csv_row())
    return output_path


def build_observation_x_t_local_plot_series(dataset: ObservationDataset) -> list[dict[str, Any]]:
    origin = _origin_x_global(dataset)
    vehicle_ids = _x_t_vehicle_ids(dataset)
    return _series_for_records(
        dataset,
        vehicle_ids,
        value=lambda record: record.x_global - origin,
    )


def build_observation_v_t_plot_series(dataset: ObservationDataset) -> list[dict[str, Any]]:
    return _series_for_records(
        dataset,
        _v_t_vehicle_ids(dataset),
        value=lambda record: record.v,
    )


def build_observation_y_t_plot_series(dataset: ObservationDataset) -> list[dict[str, Any]]:
    return _series_for_records(
        dataset,
        dataset.mv_ids,
        value=lambda record: record.y,
    )


def build_observation_lifecycle_timeline_series(dataset: ObservationDataset) -> list[dict[str, Any]]:
    event_fields = (
        ("locked_gap_step", "locked"),
        ("lateral_start_step", "lateral start"),
        ("lateral_completed_step", "lateral completed"),
        ("mainline_conversion_step", "mainline"),
    )
    series: list[dict[str, Any]] = []
    for row_index, mv_id in enumerate(dataset.mv_ids):
        lifecycle = dataset.lifecycles[mv_id]
        for field_name, label in event_fields:
            step = getattr(lifecycle, field_name)
            if step is None:
                continue
            series.append(
                {
                    "mv_id": mv_id,
                    "row": row_index,
                    "step": int(step),
                    "field": field_name,
                    "label": label,
                }
            )
    return series


def render_observation_x_t_local_plot(dataset: ObservationDataset, path: str | Path) -> Path:
    return _render_time_series_plot(
        build_observation_x_t_local_plot_series(dataset),
        Path(path),
        xlabel="t",
        ylabel="x_global - origin_x_global",
        title="Observation Process Local X/T",
    )


def render_observation_v_t_plot(dataset: ObservationDataset, path: str | Path) -> Path:
    return _render_time_series_plot(
        build_observation_v_t_plot_series(dataset),
        Path(path),
        xlabel="t",
        ylabel="v",
        title="Observation Process V/T",
    )


def render_observation_y_t_plot(dataset: ObservationDataset, path: str | Path) -> Path:
    output_path = Path(path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(10, 6))
    for item in build_observation_y_t_plot_series(dataset):
        ax.plot(item["t"], item["value"], color=item["color"], label=item["label"])
    t0, t1 = dataset.t_range()
    ax.plot([t0, t1], [0.0, 0.0], color="#6f7378", linestyle="--", linewidth=1.0, label="lane_2 baseline")
    ax.plot([t0, t1], [-3.5, -3.5], color="#9a6b28", linestyle="--", linewidth=1.0, label="on-ramp baseline")
    ax.set_xlabel("t")
    ax.set_ylabel("y")
    ax.set_title("Observation Process Y/T")
    ax.legend()
    fig.tight_layout()
    fig.savefig(output_path, dpi=150)
    plt.close(fig)
    return output_path


def render_observation_lifecycle_timeline(dataset: ObservationDataset, path: str | Path) -> Path:
    output_path = Path(path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    events = build_observation_lifecycle_timeline_series(dataset)
    colors = {
        "locked_gap_step": "#0068a8",
        "lateral_start_step": "#b85c00",
        "lateral_completed_step": "#2f7d32",
        "mainline_conversion_step": "#7b4bb3",
    }

    fig, ax = plt.subplots(figsize=(10, max(3.0, 1.2 + len(dataset.mv_ids) * 0.8)))
    for row_index, mv_id in enumerate(dataset.mv_ids):
        ax.hlines(row_index, *dataset.step_range(), color="#d2d6dc", linewidth=1.5)
        ax.text(dataset.step_range()[0], row_index + 0.16, mv_id, fontsize=9, va="bottom")

    for event in events:
        color = colors.get(str(event["field"]), "#333333")
        ax.scatter([event["step"]], [event["row"]], color=color, s=45, zorder=3, label=event["label"])
        ax.text(event["step"], event["row"] + 0.08, str(event["step"]), fontsize=8, ha="center")

    handles, labels = ax.get_legend_handles_labels()
    dedup: dict[str, Any] = {}
    for handle, label in zip(handles, labels, strict=False):
        dedup.setdefault(label, handle)
    ax.legend(dedup.values(), dedup.keys(), loc="upper right")
    ax.set_xlabel("step")
    ax.set_yticks(range(len(dataset.mv_ids)), dataset.mv_ids)
    ax.set_title("Observation Lifecycle Timeline")
    ax.set_ylim(-0.5, max(0.5, len(dataset.mv_ids) - 0.5))
    fig.tight_layout()
    fig.savefig(output_path, dpi=150)
    plt.close(fig)
    return output_path


def _series_for_records(
    dataset: ObservationDataset,
    vehicle_ids: Iterable[str],
    *,
    value: Any,
) -> list[dict[str, Any]]:
    palette = _vehicle_palette(tuple(vehicle_ids), dataset.mv_ids)
    series: list[dict[str, Any]] = []
    for vehicle_id in vehicle_ids:
        records = dataset.records_for(vehicle_id)
        if not records:
            continue
        series.append(
            {
                "vehicle_id": vehicle_id,
                "label": vehicle_id,
                "color": palette[vehicle_id],
                "t": [record.t for record in records],
                "value": [value(record) for record in records],
            }
        )
    return series


def _render_time_series_plot(
    series: list[dict[str, Any]],
    output_path: Path,
    *,
    xlabel: str,
    ylabel: str,
    title: str,
) -> Path:
    output_path.parent.mkdir(parents=True, exist_ok=True)
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
    return output_path


def _origin_x_global(dataset: ObservationDataset) -> float:
    initial_values: list[float] = []
    for mv_id in dataset.mv_ids:
        records = dataset.records_for(mv_id)
        if not records:
            raise ValueError(f"MV {mv_id!r} has no trajectory records")
        initial_values.append(records[0].x_global)
    return max(initial_values)


def _x_t_vehicle_ids(dataset: ObservationDataset) -> tuple[str, ...]:
    selected = list(dataset.mv_ids)
    seen = set(selected)
    for record in dataset.trajectory_records:
        if record.vehicle_id in seen:
            continue
        if record.physical_lane == "lane_2" and record.road_role == "mainline":
            selected.append(record.vehicle_id)
            seen.add(record.vehicle_id)
    return tuple(selected)


def _v_t_vehicle_ids(dataset: ObservationDataset) -> tuple[str, ...]:
    selected = list(dataset.mv_ids)
    seen = set(selected)
    for vehicle_id in _selected_plan_vehicle_ids(dataset.summary):
        if vehicle_id in seen:
            continue
        selected.append(vehicle_id)
        seen.add(vehicle_id)
    return tuple(selected)


def _selected_plan_vehicle_ids(summary: Any) -> tuple[str, ...]:
    ids: list[str] = []
    seen: set[str] = set()
    for round_summary in summary.get("round_summaries") or []:
        for plan in round_summary.get("plan_summaries") or []:
            for key in ("selected_vehicle_ids", "controlled_vehicle_ids"):
                for vehicle_id in plan.get(key) or []:
                    vehicle_id = str(vehicle_id)
                    if vehicle_id not in seen:
                        ids.append(vehicle_id)
                        seen.add(vehicle_id)
            for key in ("selected_front_vehicle_id", "selected_rear_vehicle_id"):
                vehicle_id = plan.get(key)
                if vehicle_id is not None and str(vehicle_id) not in seen:
                    ids.append(str(vehicle_id))
                    seen.add(str(vehicle_id))
    return tuple(ids)


def _vehicle_palette(vehicle_ids: tuple[str, ...], mv_ids: tuple[str, ...]) -> dict[str, str]:
    mv_colors = ("#c23b22", "#8e44ad", "#d35400")
    context_colors = (
        "#2f6f9f",
        "#2e8b57",
        "#6c757d",
        "#b58900",
        "#008b8b",
        "#4f6d7a",
        "#7f8c8d",
        "#4a7c59",
    )
    colors: dict[str, str] = {}
    for index, mv_id in enumerate(mv_ids):
        colors[mv_id] = mv_colors[index % len(mv_colors)]
    context_index = 0
    for vehicle_id in vehicle_ids:
        if vehicle_id in colors:
            continue
        colors[vehicle_id] = context_colors[context_index % len(context_colors)]
        context_index += 1
    return colors
