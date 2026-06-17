from __future__ import annotations

import csv
from collections import defaultdict
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

from matplotlib import pyplot as plt

from cormc.onestep.kernel.evaluation import evaluate_one_step_scenario
from cormc.onestep.kernel.models import (
    OneStepEvaluationResult,
    OneStepScenarioArtifacts,
    TrajectoryBundle,
    TrajectoryContract,
    TrajectorySample,
)
from cormc.onestep.lab.reference_case import (
    get_reference_algorithm_config,
    get_reference_expected,
    get_reference_scenario,
)
from cormc.onestep.kernel.trajectory import build_best_gap_trajectory_bundle

CSV_COLUMNS = (
    "t",
    "vehicle_id",
    "role",
    "x",
    "v",
    "is_selected_gap_vehicle",
    "is_merge_vehicle",
)


def trajectory_samples_to_rows(bundle: TrajectoryBundle) -> list[dict[str, object]]:
    ordered = sorted(bundle.samples, key=lambda sample: (sample.t, sample.vehicle_id))
    return [
        {
            "t": sample.t,
            "vehicle_id": sample.vehicle_id,
            "role": sample.role,
            "x": sample.x,
            "v": sample.v,
            "is_selected_gap_vehicle": sample.is_selected_gap_vehicle,
            "is_merge_vehicle": sample.is_merge_vehicle,
        }
        for sample in ordered
    ]


def write_trajectory_csv(bundle: TrajectoryBundle, output_dir: str | Path) -> Path:
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)
    csv_path = output_path / "trajectory.csv"
    rows = trajectory_samples_to_rows(bundle)
    with csv_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(CSV_COLUMNS))
        writer.writeheader()
        writer.writerows(rows)
    return csv_path


def _group_samples_by_vehicle(samples: tuple[TrajectorySample, ...]) -> dict[str, list[TrajectorySample]]:
    grouped: dict[str, list[TrajectorySample]] = defaultdict(list)
    for sample in samples:
        grouped[sample.vehicle_id].append(sample)
    for vehicle_id in grouped:
        grouped[vehicle_id].sort(key=lambda sample: sample.t)
    return dict(grouped)


def _build_xt_plot_series(bundle: TrajectoryBundle, contract: TrajectoryContract) -> list[dict[str, object]]:
    color_map = {
        "merge_vehicle": "red",
        "selected_gap_vehicles": "blue",
        "non_selected_vehicles": "green",
    }
    grouped = _group_samples_by_vehicle(bundle.samples)
    series = []
    for vehicle_id, samples in sorted(grouped.items()):
        first = samples[0]
        if first.is_merge_vehicle:
            category = "merge_vehicle"
        elif first.is_selected_gap_vehicle:
            category = "selected_gap_vehicles"
        else:
            category = "non_selected_vehicles"
        series.append(
            {
                "vehicle_id": vehicle_id,
                "category": category,
                "color": color_map[category],
                "label": f"{vehicle_id} [{category[:-1] if category.endswith('s') else category}]",
                "t": [sample.t for sample in samples],
                "value": [sample.x for sample in samples],
            }
        )
    return series


def _build_vt_plot_series(bundle: TrajectoryBundle, contract: TrajectoryContract) -> list[dict[str, object]]:
    grouped = _group_samples_by_vehicle(bundle.samples)
    series = []
    for vehicle_id in (
        "merge_vehicle",
        *contract.selected_gap_vehicle_ids,
    ):
        samples = grouped[vehicle_id]
        color = "red" if vehicle_id == "merge_vehicle" else "blue"
        series.append(
            {
                "vehicle_id": vehicle_id,
                "color": color,
                "label": vehicle_id,
                "t": [sample.t for sample in samples],
                "value": [sample.v for sample in samples],
            }
        )
    return series


def render_xt_plot(
    bundle: TrajectoryBundle,
    contract: TrajectoryContract,
    output_dir: str | Path,
) -> Path:
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)
    png_path = output_path / "x_t.png"
    series = _build_xt_plot_series(bundle, contract)
    fig, ax = plt.subplots(figsize=(10, 6))
    for item in series:
        ax.plot(item["t"], item["value"], color=item["color"], label=item["label"])
    ax.set_xlabel("t")
    ax.set_ylabel("x")
    ax.set_title("One-Step X/T Trajectories")
    ax.legend()
    fig.tight_layout()
    fig.savefig(png_path, dpi=150)
    plt.close(fig)
    return png_path


def render_vt_plot(
    bundle: TrajectoryBundle,
    contract: TrajectoryContract,
    output_dir: str | Path,
) -> Path:
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)
    png_path = output_path / "v_t.png"
    series = _build_vt_plot_series(bundle, contract)
    fig, ax = plt.subplots(figsize=(10, 6))
    for item in series:
        ax.plot(item["t"], item["value"], color=item["color"], label=item["label"])
    ax.set_xlabel("t")
    ax.set_ylabel("v")
    ax.set_title("One-Step V/T Trajectories")
    ax.legend()
    fig.tight_layout()
    fig.savefig(png_path, dpi=150)
    plt.close(fig)
    return png_path


def build_one_step_artifacts(
    evaluation: OneStepEvaluationResult,
    contract: TrajectoryContract,
    output_dir: str | Path,
) -> OneStepScenarioArtifacts:
    if evaluation.status == "no_solution":
        return OneStepScenarioArtifacts(
            evaluation=evaluation,
            bundle=None,
            trajectory_csv_path=None,
            xt_plot_path=None,
            vt_plot_path=None,
        )

    best_score = evaluation.best_score
    if best_score is None:
        raise ValueError("solved evaluation must include best_score")

    bundle = build_best_gap_trajectory_bundle(
        evaluation.scenario,
        evaluation.gaps,
        best_score,
        contract,
    )
    csv_path = write_trajectory_csv(bundle, output_dir)
    xt_path = render_xt_plot(bundle, contract, output_dir)
    vt_path = render_vt_plot(bundle, contract, output_dir)

    return OneStepScenarioArtifacts(
        evaluation=evaluation,
        bundle=bundle,
        trajectory_csv_path=str(csv_path),
        xt_plot_path=str(xt_path),
        vt_plot_path=str(vt_path),
    )


def build_one_step_artifacts_for_reference(output_dir: str | Path) -> OneStepScenarioArtifacts:
    scenario = get_reference_scenario()
    algorithm = get_reference_algorithm_config()
    contract = get_reference_expected().trajectory_contract
    evaluation = evaluate_one_step_scenario(scenario, algorithm)
    return build_one_step_artifacts(
        evaluation,
        contract,
        output_dir,
    )
