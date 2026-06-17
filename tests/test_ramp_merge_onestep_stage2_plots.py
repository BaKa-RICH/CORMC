from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from cormc.scenes import (
    RM_ONESTEP_S05_PLAN_STEP0_SCENARIO_ID,
    RM_ONESTEP_S07_PLAN_STEP0_SCENARIO_ID,
)
from cormc.onestep.rolling import (
    build_onestep_stage2_plot_artifacts,
    run_onestep_stage2_history,
)
from cormc.observation.plotting import build_observation_v_t_plot_series
from cormc.onestep.rolling.stage2_plots import _dataset_from_history


@pytest.mark.parametrize(
    "scenario_id",
    [
        RM_ONESTEP_S05_PLAN_STEP0_SCENARIO_ID,
        RM_ONESTEP_S07_PLAN_STEP0_SCENARIO_ID,
    ],
)
def test_stage2_plot_wrapper_uses_observation_vt_vehicle_subset(scenario_id: str) -> None:
    history_run = run_onestep_stage2_history(
        scenario_id,
        max_steps=5,
        run_id="stage2-plot-request-test",
    )
    dataset = _dataset_from_history(history_run.summary, history_run.history, Path("."))

    process_series = build_observation_v_t_plot_series(dataset)
    assert process_series


@pytest.mark.parametrize(
    "scenario_id",
    [
        RM_ONESTEP_S05_PLAN_STEP0_SCENARIO_ID,
        RM_ONESTEP_S07_PLAN_STEP0_SCENARIO_ID,
    ],
)
def test_stage2_plot_artifacts_export_fixed_files(tmp_path: Path, scenario_id: str) -> None:
    history_run = run_onestep_stage2_history(
        scenario_id,
        max_steps=5,
        run_id="stage2-plot-artifacts-test",
    )
    artifacts = build_onestep_stage2_plot_artifacts(
        history_run.history,
        history_run.summary,
        tmp_path / "stage2",
    )

    paths = [
        artifacts.trajectory_csv_path,
        artifacts.process_x_t_local_plot_path,
        artifacts.process_v_t_plot_path,
        artifacts.process_y_t_plot_path,
        artifacts.lifecycle_timeline_plot_path,
    ]
    for path in paths:
        assert Path(path).exists()
    assert Path(artifacts.process_x_t_local_plot_path).read_bytes().startswith(b"\x89PNG\r\n\x1a\n")
    assert Path(artifacts.process_v_t_plot_path).read_bytes().startswith(b"\x89PNG\r\n\x1a\n")
    assert Path(artifacts.process_y_t_plot_path).read_bytes().startswith(b"\x89PNG\r\n\x1a\n")
    assert Path(artifacts.lifecycle_timeline_plot_path).read_bytes().startswith(b"\x89PNG\r\n\x1a\n")
