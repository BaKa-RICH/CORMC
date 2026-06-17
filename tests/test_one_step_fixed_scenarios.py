from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from cormc.onestep.lab.experiments import ONE_STEP_FIXED_SCENARIO_IDS
from cormc.onestep.lab.runner import run_one_step_fixed_scenario


@pytest.mark.parametrize("scenario_id", ONE_STEP_FIXED_SCENARIO_IDS)
def test_fixed_scenarios_match_catalog_expectations_and_artifact_rules(
    tmp_path: Path,
    scenario_id: str,
) -> None:
    result = run_one_step_fixed_scenario(
        scenario_id,
        tmp_path,
        run_id="fixed-test",
    )
    summary = result.evaluation_summary

    assert summary["scenario_id"] == scenario_id
    assert summary["validation_passed"] is True
    assert summary["validation_issues"] == []
    assert Path(result.evaluation_summary_path).exists()
    assert Path(result.scenario_report_path).exists()
    assert Path(result.artifact_manifest_path).exists()

    if summary["status"] == "solved":
        assert summary["best_gap_id"] is not None
        assert summary["best_t_m"] is not None
        assert "trajectory_csv" in result.artifact_paths
        assert "x_t_plot" in result.artifact_paths
        assert "v_t_plot" in result.artifact_paths
        assert Path(result.artifact_paths["trajectory_csv"]).exists()
        assert Path(result.artifact_paths["x_t_plot"]).exists()
        assert Path(result.artifact_paths["v_t_plot"]).exists()
    else:
        assert scenario_id == "S04"
        assert summary["status"] == "no_solution"
        assert summary["no_solution_reason"] == "no_reachable_gap"
        assert summary["plots_skipped"] is True
        assert "trajectory_csv" not in result.artifact_paths
        assert "x_t_plot" not in result.artifact_paths
        assert "v_t_plot" not in result.artifact_paths


def test_fixed_s06_has_equal_split_and_s12_uses_more_forward_gap_for_tie_breaking(
    tmp_path: Path,
) -> None:
    s06 = run_one_step_fixed_scenario("S06", tmp_path / "s06", run_id="fixed-test").evaluation_summary
    s12 = run_one_step_fixed_scenario("S12", tmp_path / "s12", run_id="fixed-test").evaluation_summary

    assert s06["best_gap_id"] == "gap2"
    assert s06["best_delta_f_star"] == pytest.approx(10.0, abs=1e-9)
    assert s06["best_delta_r_star"] == pytest.approx(10.0, abs=1e-9)

    assert s12["best_gap_id"] == "gap2"
    assert s12["best_gap_interval"] == [0.0, 85.0]


def test_fixed_s05_marks_dense_middle_gaps_infeasible(tmp_path: Path) -> None:
    summary = run_one_step_fixed_scenario("S05", tmp_path, run_id="fixed-test").evaluation_summary
    rows = {row["gap_id"]: row for row in summary["gap_rows"]}

    assert rows["gap2"]["coop_feasible"] is False
    assert rows["gap3"]["coop_feasible"] is False
    assert rows["gap4"]["coop_feasible"] is False
    assert rows["gap2"]["included_in_scoring"] is False
    assert rows["gap3"]["included_in_scoring"] is False
    assert rows["gap4"]["included_in_scoring"] is False
