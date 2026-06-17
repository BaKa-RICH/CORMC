from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def test_cooperation_public_api_exports() -> None:
    from cormc.onestep.kernel import (
        CooperationResult,
        apply_deadzone,
        check_coop_feasibility,
        compute_adjusted_displacement,
        compute_adjustment_capacity,
        compute_boundary_gap,
        compute_coop_cost,
        compute_cooperation_for_gap,
        compute_gamma,
        compute_gap_deficit,
        compute_adjacent_gap_lengths,
        compute_projection_bounds,
        solve_projected_adjustment,
    )

    assert CooperationResult.__name__ == "CooperationResult"
    assert callable(compute_gap_deficit)
    assert callable(compute_boundary_gap)
    assert callable(compute_adjacent_gap_lengths)
    assert callable(compute_adjustment_capacity)
    assert callable(check_coop_feasibility)
    assert callable(compute_gamma)
    assert callable(compute_projection_bounds)
    assert callable(solve_projected_adjustment)
    assert callable(apply_deadzone)
    assert callable(compute_coop_cost)
    assert callable(compute_adjusted_displacement)
    assert callable(compute_cooperation_for_gap)


def test_gap_deficit_and_boundary_gap_match_reference_contract() -> None:
    from cormc.onestep.kernel import (
        build_gaps,
        compute_adjacent_gap_lengths,
        compute_boundary_gap,
        compute_derived_params,
        compute_gap_deficit,
    )
    from cormc.onestep.lab import (
        get_reference_algorithm_config,
        get_reference_scenario,
    )

    scenario = get_reference_scenario()
    algorithm = get_reference_algorithm_config()
    derived = compute_derived_params(scenario, algorithm)
    gaps = build_gaps(scenario)

    boundary_gap = compute_boundary_gap(derived, algorithm)
    assert boundary_gap == 145.0

    assert compute_gap_deficit(gaps[0], derived) == 0.0
    assert compute_gap_deficit(gaps[1], derived) == 20.0
    assert compute_gap_deficit(gaps[2], derived) == 30.0
    assert compute_gap_deficit(gaps[3], derived) == 5.0
    assert compute_gap_deficit(gaps[4], derived) == 5.0

    assert compute_adjacent_gap_lengths(gaps, 0, boundary_gap) == (145.0, 65.0)
    assert compute_adjacent_gap_lengths(gaps, 2, boundary_gap) == (65.0, 80.0)
    assert compute_adjacent_gap_lengths(gaps, 4, boundary_gap) == (80.0, 60.0)


def test_adjustment_capacity_matches_reference_contract() -> None:
    from cormc.onestep.kernel import (
        build_gaps,
        compute_adjacent_gap_lengths,
        compute_adjustment_capacity,
        compute_boundary_gap,
        compute_derived_params,
    )
    from cormc.onestep.lab import (
        get_reference_algorithm_config,
        get_reference_scenario,
    )

    scenario = get_reference_scenario()
    algorithm = get_reference_algorithm_config()
    derived = compute_derived_params(scenario, algorithm)
    gaps = build_gaps(scenario)
    boundary_gap = compute_boundary_gap(derived, algorithm)

    expected = {
        "gap1": (20.0, 100.0),
        "gap2": (10.0, 45.0),
        "gap3": (35.0, 20.0),
        "gap4": (35.0, 10.0),
        "gap5": (15.0, 35.0),
    }

    for gap in gaps[:5]:
        G_prev, G_next = compute_adjacent_gap_lengths(gaps, gap.index, boundary_gap)
        assert compute_adjustment_capacity(G_prev, G_next, derived) == expected[gap.gap_id]


def test_coop_feasibility_has_true_and_false_cases() -> None:
    from cormc.onestep.kernel import check_coop_feasibility

    assert check_coop_feasibility(20.0, 10.0, 45.0) is True
    assert check_coop_feasibility(30.0, 35.0, 20.0) is True
    assert check_coop_feasibility(100.0, 10.0, 20.0) is False


def test_gamma_and_projection_bounds_match_formula() -> None:
    from cormc.onestep.kernel import (
        compute_gamma,
        compute_projection_bounds,
    )
    from cormc.onestep.lab import get_reference_algorithm_config

    algorithm = get_reference_algorithm_config()

    assert compute_gamma(20.0, algorithm) == pytest.approx(28.723, abs=1e-3)
    assert compute_gamma(100.0, algorithm) == pytest.approx(0.001838, abs=1e-6)
    assert compute_gamma(10.0, algorithm) == pytest.approx(1838.266, abs=1e-3)
    assert compute_gamma(45.0, algorithm) == pytest.approx(0.221, abs=1e-3)
    assert compute_gamma(35.0, algorithm) == pytest.approx(1.0, abs=1e-9)
    assert compute_gamma(15.0, algorithm) == pytest.approx(161.384, abs=1e-3)
    assert compute_gamma(0.0, algorithm) == float("inf")

    assert compute_projection_bounds(0.0, 20.0, 100.0) == (0.0, 0.0)
    assert compute_projection_bounds(20.0, 10.0, 45.0) == (0.0, 10.0)
    assert compute_projection_bounds(30.0, 35.0, 20.0) == (10.0, 30.0)
    assert compute_projection_bounds(5.0, 35.0, 10.0) == (0.0, 5.0)
    assert compute_projection_bounds(5.0, 15.0, 35.0) == (0.0, 5.0)


def test_projected_solution_matches_analytic_solution() -> None:
    from cormc.onestep.kernel import solve_projected_adjustment

    L, U, delta_f_raw, delta_f_proj = solve_projected_adjustment(
        20.0,
        10.0,
        45.0,
        1838.265625,
        0.22137734950822388,
    )
    assert (L, U) == (0.0, 10.0)
    assert delta_f_raw == pytest.approx(0.002408, abs=1e-6)
    assert delta_f_proj == pytest.approx(0.002408, abs=1e-6)

    L, U, delta_f_raw, delta_f_proj = solve_projected_adjustment(
        30.0,
        35.0,
        20.0,
        1.0,
        28.722900390625,
    )
    assert (L, U) == (10.0, 30.0)
    assert delta_f_raw == pytest.approx(28.990677, abs=1e-6)
    assert delta_f_proj == pytest.approx(28.990677, abs=1e-6)

    L, U, delta_f_raw, delta_f_proj = solve_projected_adjustment(
        10.0,
        1.0,
        100.0,
        1.0,
        1.0,
    )
    assert (L, U) == (0.0, 1.0)
    assert delta_f_raw == pytest.approx(5.0, abs=1e-9)
    assert delta_f_proj == pytest.approx(1.0, abs=1e-9)


def test_deadzone_can_zero_front_or_rear_side_without_special_casing() -> None:
    from cormc.onestep.kernel import apply_deadzone
    from cormc.onestep.lab import get_reference_algorithm_config

    algorithm = get_reference_algorithm_config()

    delta_f_star, delta_r_star = apply_deadzone(30.0, 35.0, 20.0, 28.990677, algorithm)
    assert delta_f_star == pytest.approx(28.990677, abs=1e-6)
    assert delta_r_star == pytest.approx(1.009323, abs=1e-6)

    delta_f_star, delta_r_star = apply_deadzone(20.0, 10.0, 45.0, 0.002408, algorithm)
    assert delta_f_star == 0.0
    assert delta_r_star == 20.0

    delta_f_star, delta_r_star = apply_deadzone(5.0, 35.0, 10.0, 4.997282, algorithm)
    assert delta_f_star == 5.0
    assert delta_r_star == 0.0

    delta_f_star, delta_r_star = apply_deadzone(4.0, 10.0, 10.0, 0.003996, algorithm)
    assert delta_f_star == 0.0
    assert delta_r_star == 4.0

    delta_f_star, delta_r_star = apply_deadzone(4.0, 10.0, 10.0, 3.996004, algorithm)
    assert delta_f_star == 4.0
    assert delta_r_star == 0.0


def test_coop_cost_and_adjusted_displacement_match_reference_contract() -> None:
    from cormc.onestep.kernel import (
        build_gaps,
        compute_adjusted_displacement,
        compute_coop_cost,
    )
    from cormc.onestep.lab import get_reference_scenario

    scenario = get_reference_scenario()
    gaps = {gap.gap_id: gap for gap in build_gaps(scenario)}

    assert compute_coop_cost(28.722900390625, 0.001838265625, 0.0, 0.0) == pytest.approx(0.0, abs=1e-9)
    assert compute_coop_cost(1838.265625, 0.22137734950822388, 0.0, 20.0) == pytest.approx(88.551, abs=1e-3)
    assert compute_coop_cost(1.0, 28.722900390625, 28.990677, 1.009323) == pytest.approx(869.720, abs=1e-3)
    assert compute_coop_cost(1.0, 1838.265625, 5.0, 0.0) == pytest.approx(25.0, abs=1e-9)
    assert compute_coop_cost(161.38408779149526, 1.0, 0.0, 5.0) == pytest.approx(25.0, abs=1e-9)

    assert compute_adjusted_displacement(gaps["gap1"], 0.0, 0.0, scenario) == pytest.approx(-135.0, abs=1e-9)
    assert compute_adjusted_displacement(gaps["gap2"], 0.0, 20.0, scenario) == pytest.approx(-67.5, abs=1e-9)
    assert compute_adjusted_displacement(gaps["gap3"], 28.990677, 1.009323, scenario) == pytest.approx(16.491, abs=1e-3)
    assert compute_adjusted_displacement(gaps["gap4"], 5.0, 0.0, scenario) == pytest.approx(72.5, abs=1e-9)
    assert compute_adjusted_displacement(gaps["gap5"], 0.0, 5.0, scenario) == pytest.approx(147.5, abs=1e-9)


def test_reference_cooperation_matches_stage1_baseline() -> None:
    from cormc.onestep.kernel import (
        build_gaps,
        compute_cooperation_for_gap,
        compute_derived_params,
        compute_reachability_for_gap,
        filter_reachable_gaps,
    )
    from cormc.onestep.lab import (
        get_reference_algorithm_config,
        get_reference_expected,
        get_reference_scenario,
    )

    scenario = get_reference_scenario()
    algorithm = get_reference_algorithm_config()
    derived = compute_derived_params(scenario, algorithm)
    gaps = build_gaps(scenario)
    reachability = tuple(compute_reachability_for_gap(gap, scenario, derived) for gap in gaps)
    reachable_gaps = filter_reachable_gaps(gaps, reachability)

    assert tuple(gap.gap_id for gap in reachable_gaps) == (
        "gap1",
        "gap2",
        "gap3",
        "gap4",
        "gap5",
    )

    results = tuple(
        compute_cooperation_for_gap(gap, gaps, scenario, algorithm, derived)
        for gap in reachable_gaps
    )
    expected_rows = get_reference_expected().cooperation_rows

    assert len(results) == len(expected_rows)

    for result, expected in zip(results, expected_rows):
        assert result.gap_id == expected.gap_id
        assert result.Delta == pytest.approx(expected.Delta, abs=1e-9)
        assert result.delta_f_bar == pytest.approx(expected.delta_f_bar, abs=1e-9)
        assert result.delta_r_bar == pytest.approx(expected.delta_r_bar, abs=1e-9)
        assert result.gamma_f == pytest.approx(expected.gamma_f, abs=1e-3)
        assert result.gamma_r == pytest.approx(expected.gamma_r, abs=1e-3)
        assert result.L == pytest.approx(expected.L, abs=1e-9)
        assert result.U == pytest.approx(expected.U, abs=1e-9)
        assert result.delta_f_star == pytest.approx(expected.delta_f_star, abs=1e-3)
        assert result.delta_r_star == pytest.approx(expected.delta_r_star, abs=1e-3)
        assert result.C_coop == pytest.approx(expected.C_coop, abs=1e-3)
        assert result.d_i == pytest.approx(expected.d_i, abs=1e-3)
        assert result.coop_feasible is True

    by_id = {result.gap_id: result for result in results}
    assert by_id["gap2"].delta_f_star == pytest.approx(0.0, abs=1e-9)
    assert by_id["gap2"].delta_r_star == pytest.approx(20.0, abs=1e-9)
    assert by_id["gap3"].delta_f_star == pytest.approx(28.991, abs=1e-3)
    assert by_id["gap3"].delta_r_star == pytest.approx(1.009, abs=1e-3)
    assert by_id["gap4"].delta_f_star == pytest.approx(5.0, abs=1e-9)
    assert by_id["gap4"].delta_r_star == pytest.approx(0.0, abs=1e-9)
    assert by_id["gap5"].delta_f_star == pytest.approx(0.0, abs=1e-9)
    assert by_id["gap5"].delta_r_star == pytest.approx(5.0, abs=1e-9)


def test_compute_cooperation_for_gap_handles_infeasible_case_without_special_data() -> None:
    from cormc.onestep.kernel import (
        AlgorithmConfig,
        Gap,
        ScenarioConfig,
        compute_cooperation_for_gap,
        compute_derived_params,
    )

    scenario = ScenarioConfig(
        x_targets=(0.0, 45.0, 90.0, 135.0),
        x_m0=0.0,
        v_ref=20.0,
        v_max=30.0,
        v_min=0.0,
        a_max=3.0,
        a_min=-4.0,
        T=20.0,
    )
    algorithm = AlgorithmConfig(
        D_h=40.0,
        l_m=5.0,
        w_c=0.2,
        w_e=1.0,
        w_t=10.0,
        delta_ref=35.0,
        q=6.0,
        epsilon_delta=0.05,
        K=120.0 / 7.0,
        boundary_adjustment=0.0,
    )
    derived = compute_derived_params(scenario, algorithm)
    gaps = (
        Gap("gap1", 0, 0.0, 45.0, 45.0, 22.5, 22.5),
        Gap("gap2", 1, 45.0, 90.0, 45.0, 67.5, 67.5),
        Gap("gap3", 2, 90.0, 135.0, 45.0, 112.5, 112.5),
    )

    result = compute_cooperation_for_gap(gaps[1], gaps, scenario, algorithm, derived)

    assert result.Delta == 40.0
    assert result.G_prev == 45.0
    assert result.G_next == 45.0
    assert result.delta_f_bar == 0.0
    assert result.delta_r_bar == 0.0
    assert result.coop_feasible is False
    assert result.gamma_f == float("inf")
    assert result.gamma_r == float("inf")
    assert result.L == pytest.approx(40.0, abs=1e-9)
    assert result.U == pytest.approx(0.0, abs=1e-9)
    assert result.delta_f_raw == pytest.approx(0.0, abs=1e-9)
    assert result.delta_f_star == pytest.approx(0.0, abs=1e-9)
    assert result.delta_r_star == pytest.approx(0.0, abs=1e-9)
    assert result.C_coop == pytest.approx(0.0, abs=1e-9)
    assert result.d_i == pytest.approx(67.5, abs=1e-9)
