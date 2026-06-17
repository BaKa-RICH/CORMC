from __future__ import annotations

import csv
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def _png_header(path: Path) -> bytes:
    with path.open("rb") as handle:
        return handle.read(8)


def test_trajectory_csv_matches_contract_columns(tmp_path: Path) -> None:
    from cormc.onestep.lab import build_one_step_artifacts_for_reference, get_reference_expected

    result = build_one_step_artifacts_for_reference(tmp_path)
    csv_path = Path(result.trajectory_csv_path)
    contract = get_reference_expected().trajectory_contract

    assert csv_path.exists()
    with csv_path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.reader(handle)
        header = next(reader)
        rows = list(reader)

    assert tuple(header) == contract.required_csv_columns
    vehicle_count = len({sample.vehicle_id for sample in result.bundle.samples})
    time_count = len({sample.t for sample in result.bundle.samples})
    assert len(rows) == vehicle_count * time_count


def test_reference_csv_contains_all_required_vehicle_roles(tmp_path: Path) -> None:
    from cormc.onestep.lab import build_one_step_artifacts_for_reference

    result = build_one_step_artifacts_for_reference(tmp_path)
    csv_path = Path(result.trajectory_csv_path)

    with csv_path.open("r", encoding="utf-8", newline="") as handle:
        rows = list(csv.DictReader(handle))

    vehicle_ids = {row["vehicle_id"] for row in rows}
    roles = {row["role"] for row in rows}
    merge_rows = [row for row in rows if row["vehicle_id"] == "merge_vehicle"]
    selected_rows = [row for row in rows if row["is_selected_gap_vehicle"] == "True"]
    non_selected_rows = [row for row in rows if row["role"] == "non_selected_vehicle"]

    assert "merge_vehicle" in vehicle_ids
    assert "target_lane_rear_30m" in vehicle_ids
    assert "target_lane_front_110m" in vehicle_ids
    assert "target_lane_-180m" in vehicle_ids
    assert "merge_vehicle" in roles
    assert "selected_gap_rear_vehicle" in roles
    assert "selected_gap_front_vehicle" in roles
    assert "non_selected_vehicle" in roles
    assert merge_rows and all(row["is_merge_vehicle"] == "True" for row in merge_rows)
    assert selected_rows
    assert all(row["is_selected_gap_vehicle"] == "True" for row in selected_rows)
    assert non_selected_rows
    assert all(row["is_selected_gap_vehicle"] == "False" for row in non_selected_rows)
    assert all(row["is_merge_vehicle"] == "False" for row in non_selected_rows)


def test_xt_plot_and_vt_plot_are_written(tmp_path: Path) -> None:
    from cormc.onestep.lab import build_one_step_artifacts_for_reference

    result = build_one_step_artifacts_for_reference(tmp_path)
    xt_path = Path(result.xt_plot_path)
    vt_path = Path(result.vt_plot_path)

    assert xt_path.exists()
    assert vt_path.exists()
    assert xt_path.stat().st_size > 0
    assert vt_path.stat().st_size > 0
    assert _png_header(xt_path) == b"\x89PNG\r\n\x1a\n"
    assert _png_header(vt_path) == b"\x89PNG\r\n\x1a\n"


def test_xt_plot_uses_all_vehicles_and_vt_plot_uses_only_three_vehicles(tmp_path: Path) -> None:
    from cormc.onestep.lab import build_one_step_artifacts_for_reference
    from cormc.onestep.lab.artifacts import _build_vt_plot_series, _build_xt_plot_series

    result = build_one_step_artifacts_for_reference(tmp_path)
    bundle = result.bundle
    contract = __import__("cormc.onestep.lab.reference_case", fromlist=["get_reference_expected"]).get_reference_expected().trajectory_contract

    xt_series = _build_xt_plot_series(bundle, contract)
    vt_series = _build_vt_plot_series(bundle, contract)

    assert len(xt_series) == len({sample.vehicle_id for sample in bundle.samples})
    assert len(vt_series) == 3
    assert {item["vehicle_id"] for item in vt_series} == {
        "merge_vehicle",
        "target_lane_rear_30m",
        "target_lane_front_110m",
    }
    assert {item["color"] for item in xt_series} == {"red", "blue", "green"}
    assert {item["color"] for item in vt_series} == {"red", "blue"}


def test_reference_artifact_builder_writes_three_outputs(tmp_path: Path) -> None:
    from cormc.onestep.lab import build_one_step_artifacts_for_reference

    result = build_one_step_artifacts_for_reference(tmp_path)

    assert Path(result.trajectory_csv_path).name == "trajectory.csv"
    assert Path(result.xt_plot_path).name == "x_t.png"
    assert Path(result.vt_plot_path).name == "v_t.png"
    assert Path(result.trajectory_csv_path).exists()
    assert Path(result.xt_plot_path).exists()
    assert Path(result.vt_plot_path).exists()
    assert result.bundle.selected_gap_id == "gap4"
