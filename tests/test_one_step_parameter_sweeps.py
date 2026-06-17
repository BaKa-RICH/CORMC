from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from cormc.onestep.lab.runner import run_one_step_sweep_scenario


def test_s11_wc_sweep_shifts_toward_lower_cooperation_cost(tmp_path: Path) -> None:
    result = run_one_step_sweep_scenario("S11", tmp_path, run_id="sweep-test")
    points = {point.parameter_value: point.evaluation_summary for point in result.point_results}

    assert points[0.0]["best_gap_id"] == "gap2"
    assert points[0.2]["best_gap_id"] == "gap3"
    assert points[1.0]["best_gap_id"] == "gap1"
    assert points[1.0]["best_C_coop"] <= points[0.2]["best_C_coop"] <= points[0.0]["best_C_coop"]
    assert "gap2" in result.sweep_summary["trend_observation"]
    assert "gap1" in result.sweep_summary["trend_observation"]


def test_s13_q_sweep_keeps_q6_baseline_and_strengthens_gap4_front_bias(tmp_path: Path) -> None:
    result = run_one_step_sweep_scenario("S13", tmp_path, run_id="sweep-test")
    points = {point.parameter_value: point.evaluation_summary for point in result.point_results}

    gap4_bias_low = next(
        row["delta_f_star"] - row["delta_r_star"]
        for row in points[0.5]["gap_rows"]
        if row["gap_id"] == "gap4"
    )
    gap4_bias_q6 = next(
        row["delta_f_star"] - row["delta_r_star"]
        for row in points[6.0]["gap_rows"]
        if row["gap_id"] == "gap4"
    )

    assert points[6.0]["best_gap_id"] == "gap4"
    assert gap4_bias_q6 >= gap4_bias_low
    assert "q=6" in result.sweep_summary["trend_observation"]


def test_s14_wt_sweep_exposes_shorter_time_preference_and_switches_best_gap(tmp_path: Path) -> None:
    result = run_one_step_sweep_scenario("S14", tmp_path, run_id="sweep-test")
    points = {point.parameter_value: point.evaluation_summary for point in result.point_results}

    assert points[1.0]["best_gap_id"] == "gap1"
    assert points[5.0]["best_gap_id"] == "gap2"
    assert points[100.0]["best_gap_id"] == "gap2"
    assert points[100.0]["best_t_m"] <= points[1.0]["best_t_m"]
    assert "best_t_m" in result.sweep_summary["trend_observation"]


def test_only_representative_sweep_values_generate_plots(tmp_path: Path) -> None:
    result = run_one_step_sweep_scenario("S14", tmp_path, run_id="sweep-test")
    points = {point.parameter_value: point for point in result.point_results}

    assert "x_t_plot" in points[1.0].artifact_paths
    assert "v_t_plot" in points[1.0].artifact_paths
    assert "x_t_plot" in points[10.0].artifact_paths
    assert "v_t_plot" in points[10.0].artifact_paths
    assert "x_t_plot" in points[100.0].artifact_paths
    assert "v_t_plot" in points[100.0].artifact_paths
    assert "x_t_plot" not in points[5.0].artifact_paths
    assert "v_t_plot" not in points[5.0].artifact_paths
    assert "x_t_plot" not in points[30.0].artifact_paths
    assert "v_t_plot" not in points[30.0].artifact_paths
