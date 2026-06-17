from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from cormc.observation.plotting import (
    build_observation_lifecycle_timeline_series,
    build_observation_plot_artifacts,
    build_observation_y_t_plot_series,
)
from cormc.observation.stage2_artifacts import load_stage2_observation_dataset

PHASE7_ROOT = Path("artifacts/scene_interface_phase7_validation")
PHASE7_SCENARIO_DIRS = (
    PHASE7_ROOT / "RM-ONESTEP-S05-PLAN-STEP0" / "scene-interface-phase7-validation",
    PHASE7_ROOT / "RM-ONESTEP-S05-ROLLING-ENTRY" / "scene-interface-phase7-validation",
    PHASE7_ROOT / "RM-ONESTEP-S07-PLAN-STEP0" / "scene-interface-phase7-validation",
    PHASE7_ROOT / "RM-ONESTEP-S07-ROLLING-ENTRY" / "scene-interface-phase7-validation",
    PHASE7_ROOT,
)


@pytest.mark.parametrize("source_dir", PHASE7_SCENARIO_DIRS)
def test_observation_plotting_exports_fixed_files(tmp_path: Path, source_dir: Path) -> None:
    dataset = load_stage2_observation_dataset(source_dir)
    artifacts = build_observation_plot_artifacts(dataset, tmp_path / dataset.scenario_id)

    for path in (
        artifacts.trajectory_csv_path,
        artifacts.process_x_t_local_plot_path,
        artifacts.process_v_t_plot_path,
        artifacts.process_y_t_plot_path,
        artifacts.lifecycle_timeline_plot_path,
    ):
        assert Path(path).exists()
    for path in (
        artifacts.process_x_t_local_plot_path,
        artifacts.process_v_t_plot_path,
        artifacts.process_y_t_plot_path,
        artifacts.lifecycle_timeline_plot_path,
    ):
        assert Path(path).read_bytes().startswith(b"\x89PNG\r\n\x1a\n")


def test_observation_plotting_2mv_y_and_lifecycle_series() -> None:
    dataset = load_stage2_observation_dataset(PHASE7_ROOT)

    y_series = {item["vehicle_id"]: item for item in build_observation_y_t_plot_series(dataset)}
    assert set(y_series) == {"S07_MV", "S07_MV_REAR"}
    for mv_id in dataset.mv_ids:
        assert min(y_series[mv_id]["value"]) == pytest.approx(-3.5)
        assert max(y_series[mv_id]["value"]) == pytest.approx(0.0)

    lifecycle = {
        (item["mv_id"], item["field"]): item["step"]
        for item in build_observation_lifecycle_timeline_series(dataset)
    }
    assert lifecycle[("S07_MV", "locked_gap_step")] == 143
    assert lifecycle[("S07_MV", "lateral_start_step")] == 183
    assert lifecycle[("S07_MV", "lateral_completed_step")] == 192
    assert lifecycle[("S07_MV", "mainline_conversion_step")] == 193
    assert lifecycle[("S07_MV_REAR", "locked_gap_step")] == 209
    assert lifecycle[("S07_MV_REAR", "lateral_start_step")] == 209
    assert lifecycle[("S07_MV_REAR", "lateral_completed_step")] == 218
    assert lifecycle[("S07_MV_REAR", "mainline_conversion_step")] == 219
