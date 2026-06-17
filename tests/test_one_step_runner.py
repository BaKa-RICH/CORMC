from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from cormc.onestep.lab.runner import (
    run_one_step_fixed_scenario,
    run_one_step_fixed_suite,
    run_one_step_full_suite,
    run_one_step_sweep_scenario,
)


def test_fixed_runner_writes_summary_report_manifest_and_solved_artifacts(tmp_path: Path) -> None:
    result = run_one_step_fixed_scenario("S01", tmp_path, run_id="runner-test")

    assert result.status == "solved"
    assert result.evaluation_summary["best_gap_id"] == "gap4"
    assert Path(result.evaluation_summary_path).exists()
    assert Path(result.scenario_report_path).exists()
    assert Path(result.artifact_manifest_path).exists()
    assert Path(result.artifact_paths["trajectory_csv"]).exists()
    assert Path(result.artifact_paths["x_t_plot"]).exists()
    assert Path(result.artifact_paths["v_t_plot"]).exists()


def test_fixed_suite_runs_s01_to_s12_and_writes_root_outputs(tmp_path: Path) -> None:
    suite = run_one_step_fixed_suite(tmp_path, run_id="runner-suite")

    assert len(suite.scenario_results) == 12
    assert suite.suite_summary["fixed_scenario_count"] == 12
    assert suite.suite_summary["sweep_scenario_count"] == 0
    assert suite.suite_summary["all_scenarios_have_artifacts"] is True
    assert Path(suite.suite_summary_path).exists()
    assert Path(suite.suite_report_path).exists()
    assert Path(suite.artifact_manifest_path).exists()


def test_sweep_runner_writes_summary_report_manifest_and_keeps_point_summaries(tmp_path: Path) -> None:
    result = run_one_step_sweep_scenario("S11", tmp_path, run_id="runner-sweep")

    assert result.parameter_name == "w_c"
    assert len(result.point_results) == 3
    assert Path(result.sweep_summary_path).exists()
    assert Path(result.sweep_report_path).exists()
    assert Path(result.artifact_manifest_path).exists()
    assert all("evaluation_summary" in point.artifact_paths for point in result.point_results)


def test_full_suite_returns_fixed_and_sweeps_and_keeps_non_representative_points_plotless(
    tmp_path: Path,
) -> None:
    suite = run_one_step_full_suite(tmp_path, run_id="runner-full")
    sweep_map = {result.scenario_id: result for result in suite.sweep_results}
    s11_points = {point.parameter_value: point for point in sweep_map["S11"].point_results}

    assert len(suite.scenario_results) == 12
    assert len(suite.sweep_results) == 3
    assert Path(suite.suite_summary_path).exists()
    assert Path(suite.suite_report_path).exists()
    assert Path(suite.artifact_manifest_path).exists()
    assert "x_t_plot" not in s11_points[0.2].artifact_paths
    assert "v_t_plot" not in s11_points[0.2].artifact_paths
