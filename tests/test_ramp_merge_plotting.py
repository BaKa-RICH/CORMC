from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from cormc.legacy import (
    RampMergeTrajectoryPlotRequest,
    build_ramp_merge_local_xt_plot_series,
    build_ramp_merge_plot_artifacts,
    build_ramp_merge_vt_plot_series,
)
from cormc.simulation_core.commit import OutputHistory, TrajectoryRecord


def test_shared_plotting_core_supports_explicit_plot_request(tmp_path: Path) -> None:
    history = OutputHistory(
        trajectory_records=[
            _record(step=0, t=0.0, vehicle_id="MV_A", x_global=98.0, v=20.0, physical_lane="on_ramp"),
            _record(step=1, t=0.1, vehicle_id="MV_A", x_global=102.0, v=21.0, physical_lane="on_ramp"),
            _record(step=2, t=0.2, vehicle_id="MV_A", x_global=104.0, v=22.0, physical_lane="on_ramp"),
            _record(step=0, t=0.0, vehicle_id="ML_1", x_global=110.0, v=20.0, physical_lane="lane_2"),
            _record(step=1, t=0.1, vehicle_id="ML_1", x_global=112.0, v=19.5, physical_lane="lane_2"),
            _record(step=2, t=0.2, vehicle_id="ML_1", x_global=114.0, v=19.0, physical_lane="lane_2"),
            _record(step=0, t=0.0, vehicle_id="IGNORED", x_global=140.0, v=30.0, physical_lane="lane_1"),
        ]
    )
    request = RampMergeTrajectoryPlotRequest(
        selected_vehicle_ids=("MV_A", "ML_1"),
        origin_x_global=100.0,
        anchor_step=1,
        anchor_t=0.1,
        anchor_x_local_by_id={"MV_A": 0.0, "ML_1": 15.0},
        color_by_vehicle_id={"MV_A": "red", "ML_1": "blue"},
        label_by_vehicle_id={"MV_A": "MV_A", "ML_1": "ML_1"},
        trajectory_csv_filename="generic_trajectory.csv",
        x_t_plot_filename="generic_x_t.png",
        v_t_plot_filename="generic_v_t.png",
        x_t_title="Generic Local X/T",
        v_t_title="Generic V/T",
    )

    xt_series = build_ramp_merge_local_xt_plot_series(history, request)
    vt_series = build_ramp_merge_vt_plot_series(history, request)
    artifacts = build_ramp_merge_plot_artifacts(history, request, tmp_path / "generic")

    assert [item["vehicle_id"] for item in xt_series] == ["MV_A", "ML_1"]
    assert [item["vehicle_id"] for item in vt_series] == ["MV_A", "ML_1"]
    assert _value_at_t(xt_series[0], 0.1) == pytest.approx(0.0)
    assert _value_at_t(xt_series[1], 0.1) == pytest.approx(15.0)
    assert list(xt_series[0]["t"]) == [0.0, 0.1, 0.2]
    assert list(xt_series[0]["value"]) == pytest.approx([-2.0, 0.0, 4.0])
    assert list(xt_series[1]["value"]) == pytest.approx([10.0, 15.0, 14.0])
    assert list(vt_series[0]["value"]) == pytest.approx([20.0, 21.0, 22.0])
    assert list(vt_series[1]["value"]) == pytest.approx([20.0, 19.5, 19.0])
    assert Path(artifacts.trajectory_csv_path).name == "generic_trajectory.csv"
    assert Path(artifacts.x_t_plot_path).name == "generic_x_t.png"
    assert Path(artifacts.v_t_plot_path).name == "generic_v_t.png"
    assert Path(artifacts.trajectory_csv_path).exists()
    assert Path(artifacts.x_t_plot_path).read_bytes().startswith(b"\x89PNG\r\n\x1a\n")
    assert Path(artifacts.v_t_plot_path).read_bytes().startswith(b"\x89PNG\r\n\x1a\n")


def _record(
    *,
    step: int,
    t: float,
    vehicle_id: str,
    x_global: float,
    v: float,
    physical_lane: str,
) -> TrajectoryRecord:
    return TrajectoryRecord(
        run_id="generic-plot-run",
        scenario_id="generic-plot-scenario",
        step=step,
        t=t,
        vehicle_id=vehicle_id,
        vehicle_type="car",
        compliance_state="nominal",
        x_global=x_global,
        y=0.0,
        v=v,
        a=0.0,
        physical_lane=physical_lane,
        road_role="mainline_vehicle",
    )


def _value_at_t(series: dict[str, object], t_value: float) -> float:
    t_values = list(series["t"])
    values = list(series["value"])
    for observed_t, value in zip(t_values, values):
        if float(observed_t) == pytest.approx(t_value):
            return float(value)
    raise AssertionError(f"did not find t={t_value} in series")
