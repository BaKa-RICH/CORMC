from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def test_one_step_public_api_exports() -> None:
    from cormc.onestep.kernel import (
        AlgorithmConfig,
        ReferenceExpected,
        ScenarioConfig,
        TrajectoryContract,
    )
    from cormc.onestep.lab import (
        get_reference_algorithm_config,
        get_reference_expected,
        get_reference_scenario,
    )

    assert ScenarioConfig.__name__ == "ScenarioConfig"
    assert AlgorithmConfig.__name__ == "AlgorithmConfig"
    assert ReferenceExpected.__name__ == "ReferenceExpected"
    assert TrajectoryContract.__name__ == "TrajectoryContract"
    assert callable(get_reference_scenario)
    assert callable(get_reference_algorithm_config)
    assert callable(get_reference_expected)


def test_reference_inputs_are_frozen() -> None:
    from cormc.onestep.lab import (
        get_reference_algorithm_config,
        get_reference_scenario,
    )

    scenario = get_reference_scenario()
    algorithm = get_reference_algorithm_config()

    assert scenario.x_targets == (-180.0, -90.0, -25.0, 30.0, 110.0, 190.0, 250.0)
    assert scenario.x_m0 == 0.0
    assert scenario.v_ref == 20.0
    assert scenario.v_max == 30.0
    assert scenario.v_min == 0.0
    assert scenario.a_max == 3.0
    assert scenario.a_min == -4.0
    assert scenario.T == 20.0

    assert algorithm.D_h == 40.0
    assert algorithm.l_m == 5.0
    assert algorithm.w_c == 0.2
    assert algorithm.w_e == 1.0
    assert algorithm.w_t == 10.0
    assert algorithm.delta_ref == 35.0
    assert algorithm.q == 6.0
    assert algorithm.epsilon_delta == 0.05
    assert algorithm.K == 120.0 / 7.0
    assert algorithm.boundary_adjustment == 100.0


def test_reference_expected_snapshot_counts_and_key_rows() -> None:
    from cormc.onestep.lab import (
        get_reference_algorithm_config,
        get_reference_expected,
        get_reference_scenario,
    )

    expected = get_reference_expected()
    gap_rows = {row.gap_id: row for row in expected.gap_rows}
    reachability_rows = {row.gap_id: row for row in expected.reachability_rows}

    assert expected.scenario == get_reference_scenario()
    assert expected.algorithm == get_reference_algorithm_config()
    assert len(expected.gap_rows) == 6
    assert len(expected.reachability_rows) == 6
    assert len(expected.cooperation_rows) == 5
    assert len(expected.strict_score_rows) == 5

    assert gap_rows["gap1"].G_i == 90.0
    assert gap_rows["gap4"].c_i == 70.0
    assert gap_rows["gap6"].D_i == 220.0

    assert reachability_rows["gap1"].t_reach == 12.583
    assert reachability_rows["gap5"].p_pre == 508.333
    assert reachability_rows["gap6"].reachable is False


def test_reference_cooperation_and_strict_score_baselines_are_frozen() -> None:
    from cormc.onestep.lab import get_reference_expected

    expected = get_reference_expected()
    cooperation_rows = {row.gap_id: row for row in expected.cooperation_rows}
    strict_score_rows = {row.gap_id: row for row in expected.strict_score_rows}

    assert cooperation_rows["gap2"].Delta == 20.0
    assert cooperation_rows["gap3"].delta_f_star == 28.991
    assert cooperation_rows["gap3"].delta_r_star == 1.009
    assert cooperation_rows["gap4"].delta_f_star == 5.0
    assert cooperation_rows["gap4"].delta_r_star == 0.0
    assert cooperation_rows["gap5"].d_i == 147.5

    assert expected.best_gap_id == "gap4"
    assert expected.best_gap_interval == (30.0, 110.0)
    assert expected.best_delta_f_star == 5.0
    assert expected.best_delta_r_star == 0.0
    assert expected.best_d_i == 72.5
    assert expected.best_t_m == 13.594
    assert expected.best_p_m == 344.375
    assert strict_score_rows["gap4"].J == 176.808


def test_trajectory_contract_is_frozen() -> None:
    from cormc.onestep.lab import get_reference_expected

    contract = get_reference_expected().trajectory_contract

    assert contract.selected_gap_id == "gap4"
    assert contract.selected_gap_interval == (30.0, 110.0)
    assert contract.merge_time_s == 13.594
    assert contract.merge_point_x == 344.375
    assert contract.selected_gap_vehicle_ids == (
        "target_lane_rear_30m",
        "target_lane_front_110m",
    )
    assert contract.non_selected_motion_rule == "x=x0+20t; v=20"
    assert contract.sampling_dt == 0.1
    assert contract.required_check_times == (0.0, 3.3985, 6.797, 10.1955, 13.594)
    assert contract.required_csv_columns == (
        "t",
        "vehicle_id",
        "role",
        "x",
        "v",
        "is_selected_gap_vehicle",
        "is_merge_vehicle",
    )
    assert contract.xt_plot_vehicle_groups == ("all_vehicles",)
    assert contract.xt_plot_color_rules == (
        "merge_vehicle:red",
        "selected_gap_vehicles:blue",
        "non_selected_vehicles:green",
    )
    assert contract.vt_plot_vehicle_groups == (
        "merge_vehicle",
        "selected_gap_rear_vehicle",
        "selected_gap_front_vehicle",
    )
    assert contract.vt_plot_color_rules == (
        "merge_vehicle:red",
        "selected_gap_vehicles:blue",
    )
