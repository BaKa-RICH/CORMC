from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from cormc.scenes import (
    RM_ONESTEP_S05_PLAN_STEP0_SCENARIO_ID,
    RM_ONESTEP_S05_ROLLING_ENTRY_SCENARIO_ID,
    RM_ONESTEP_S07_PLAN_STEP0_SCENARIO_ID,
    RM_ONESTEP_S07_ROLLING_ENTRY_SCENARIO_ID,
)
from cormc.legacy import (
    RampMergeTrajectoryPlotRequest,
    build_onestep_stage1_plot_artifacts,
    build_onestep_stage1_plot_request,
    build_onestep_stage1_vt_plot_series,
    build_onestep_stage1_xt_plot_series,
    build_ramp_merge_local_xt_plot_series,
    build_ramp_merge_vt_plot_series,
    run_onestep_stage1_history,
)


@pytest.mark.parametrize(
    ("scenario_id", "max_steps"),
    [
        (RM_ONESTEP_S05_PLAN_STEP0_SCENARIO_ID, 5),
        (RM_ONESTEP_S05_ROLLING_ENTRY_SCENARIO_ID, 35),
        (RM_ONESTEP_S07_PLAN_STEP0_SCENARIO_ID, 5),
        (RM_ONESTEP_S07_ROLLING_ENTRY_SCENARIO_ID, 35),
    ],
)
def test_xt_plot_series_uses_first_check_local_origin_and_vehicle_subset(
    scenario_id: str,
    max_steps: int,
) -> None:
    result = run_onestep_stage1_history(
        scenario_id,
        max_steps=max_steps,
        run_id="stage1-plot-series",
    )
    summary = dict(result.summary)
    mv_id = str(summary["mv_id"])
    series = build_onestep_stage1_xt_plot_series(result.history, summary)
    series_by_id = {str(item["vehicle_id"]): item for item in series}
    expected_ids = [
        mv_id,
        *summary["first_check_mv_local_frame"]["lane_2_vehicle_order"],
    ]
    observed_ids = sorted({record.vehicle_id for record in result.history.trajectory_records})

    assert [str(item["vehicle_id"]) for item in series] == expected_ids
    assert sorted(series_by_id) == sorted(expected_ids)
    assert observed_ids == sorted(expected_ids)
    assert _value_at_t(
        series_by_id[mv_id],
        float(summary["first_check_t"]),
    ) == pytest.approx(0.0)
    for item in series:
        assert list(item["t"]) == sorted(item["t"])


@pytest.mark.parametrize(
    ("scenario_id", "max_steps"),
    [
        (RM_ONESTEP_S05_PLAN_STEP0_SCENARIO_ID, 5),
        (RM_ONESTEP_S05_ROLLING_ENTRY_SCENARIO_ID, 35),
        (RM_ONESTEP_S07_PLAN_STEP0_SCENARIO_ID, 5),
        (RM_ONESTEP_S07_ROLLING_ENTRY_SCENARIO_ID, 35),
    ],
)
def test_vt_plot_series_stays_on_mv_plus_lane2_subset_with_constant_speed(
    scenario_id: str,
    max_steps: int,
) -> None:
    result = run_onestep_stage1_history(
        scenario_id,
        max_steps=max_steps,
        run_id="stage1-vt-series",
    )
    summary = dict(result.summary)
    mv_id = str(summary["mv_id"])
    series = build_onestep_stage1_vt_plot_series(result.history, summary)
    expected_ids = [
        mv_id,
        *summary["first_check_mv_local_frame"]["lane_2_vehicle_order"],
    ]

    assert [str(item["vehicle_id"]) for item in series] == expected_ids
    for item in series:
        assert item["value"]
        assert min(item["value"]) == pytest.approx(20.0)
        assert max(item["value"]) == pytest.approx(20.0)


@pytest.mark.parametrize(
    ("scenario_id", "max_steps"),
    [
        (RM_ONESTEP_S05_PLAN_STEP0_SCENARIO_ID, 5),
        (RM_ONESTEP_S05_ROLLING_ENTRY_SCENARIO_ID, 35),
        (RM_ONESTEP_S07_PLAN_STEP0_SCENARIO_ID, 5),
        (RM_ONESTEP_S07_ROLLING_ENTRY_SCENARIO_ID, 35),
    ],
)
def test_stage1_plot_request_adapts_summary_into_shared_plot_core(
    scenario_id: str,
    max_steps: int,
) -> None:
    result = run_onestep_stage1_history(
        scenario_id,
        max_steps=max_steps,
        run_id="stage1-plot-request",
    )
    summary = dict(result.summary)
    mv_id = str(summary["mv_id"])
    request = build_onestep_stage1_plot_request(summary)
    expected_ids = (
        mv_id,
        *summary["first_check_mv_local_frame"]["lane_2_vehicle_order"],
    )

    assert isinstance(request, RampMergeTrajectoryPlotRequest)
    assert request.selected_vehicle_ids == expected_ids
    assert request.origin_x_global == pytest.approx(
        float(summary["first_check_mv_local_frame"]["origin_x_global"])
    )
    assert request.anchor_step == int(summary["first_check_step"])
    assert request.anchor_t == pytest.approx(float(summary["first_check_t"]))
    assert build_ramp_merge_local_xt_plot_series(result.history, request) == (
        build_onestep_stage1_xt_plot_series(result.history, summary)
    )
    assert build_ramp_merge_vt_plot_series(result.history, request) == (
        build_onestep_stage1_vt_plot_series(result.history, summary)
    )


@pytest.mark.parametrize(
    ("scenario_id", "max_steps"),
    [
        (RM_ONESTEP_S05_PLAN_STEP0_SCENARIO_ID, 5),
        (RM_ONESTEP_S05_ROLLING_ENTRY_SCENARIO_ID, 35),
        (RM_ONESTEP_S07_PLAN_STEP0_SCENARIO_ID, 5),
        (RM_ONESTEP_S07_ROLLING_ENTRY_SCENARIO_ID, 35),
    ],
)
def test_stage1_plot_artifacts_export_csv_and_pngs(
    tmp_path: Path,
    scenario_id: str,
    max_steps: int,
) -> None:
    result = run_onestep_stage1_history(
        scenario_id,
        max_steps=max_steps,
        run_id="stage1-plot-artifacts",
    )
    artifacts = build_onestep_stage1_plot_artifacts(
        result.history,
        result.summary,
        tmp_path / scenario_id,
    )

    assert Path(artifacts.trajectory_csv_path).exists()
    assert Path(artifacts.x_t_local_plot_path).exists()
    assert Path(artifacts.v_t_plot_path).exists()
    assert Path(artifacts.x_t_local_plot_path).read_bytes().startswith(b"\x89PNG\r\n\x1a\n")
    assert Path(artifacts.v_t_plot_path).read_bytes().startswith(b"\x89PNG\r\n\x1a\n")


def _value_at_t(series: dict[str, object], t_value: float) -> float:
    t_values = list(series["t"])
    values = list(series["value"])
    for observed_t, value in zip(t_values, values):
        if float(observed_t) == pytest.approx(t_value):
            return float(value)
    raise AssertionError(f"did not find t={t_value} in series")
