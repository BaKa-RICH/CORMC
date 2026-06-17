from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from cormc.onestep.lab.reference_case import (
    get_reference_algorithm_config,
    get_reference_expected,
    get_reference_scenario,
)
from cormc.onestep.kernel.evaluation import evaluate_one_step_scenario
from cormc.onestep.kernel.models import TrajectoryContract
from cormc.onestep.kernel.trajectory import (
    build_best_gap_trajectory_bundle,
)


def test_copied_onestep_kernel_matches_reference_case_baseline() -> None:
    scenario = get_reference_scenario()
    algorithm = get_reference_algorithm_config()
    expected = get_reference_expected()

    evaluation = evaluate_one_step_scenario(scenario, algorithm)

    assert evaluation.best_gap is not None
    assert evaluation.best_score is not None
    assert evaluation.best_gap.gap_id == expected.best_gap_id
    assert evaluation.best_score.delta_f_star == pytest.approx(5.0)
    assert evaluation.best_score.delta_r_star == pytest.approx(0.0)
    assert evaluation.best_score.d_i == pytest.approx(72.5)
    assert evaluation.best_score.t_m == pytest.approx(13.594, rel=1e-4)
    assert evaluation.best_score.p_m == pytest.approx(344.375, rel=1e-4)

    contract = TrajectoryContract(
        selected_gap_id=expected.trajectory_contract.selected_gap_id,
        selected_gap_interval=expected.trajectory_contract.selected_gap_interval,
        merge_time_s=expected.trajectory_contract.merge_time_s,
        merge_point_x=expected.trajectory_contract.merge_point_x,
        selected_gap_vehicle_ids=expected.trajectory_contract.selected_gap_vehicle_ids,
        non_selected_motion_rule=expected.trajectory_contract.non_selected_motion_rule,
        sampling_dt=expected.trajectory_contract.sampling_dt,
        required_csv_columns=expected.trajectory_contract.required_csv_columns,
        xt_plot_vehicle_groups=expected.trajectory_contract.xt_plot_vehicle_groups,
        xt_plot_color_rules=expected.trajectory_contract.xt_plot_color_rules,
        vt_plot_vehicle_groups=expected.trajectory_contract.vt_plot_vehicle_groups,
        vt_plot_color_rules=expected.trajectory_contract.vt_plot_color_rules,
        required_check_times=expected.trajectory_contract.required_check_times,
    )
    bundle = build_best_gap_trajectory_bundle(
        scenario,
        evaluation.gaps,
        evaluation.best_score,
        contract,
    )
    assert bundle.merge_point_x == pytest.approx(344.375, rel=1e-4)
