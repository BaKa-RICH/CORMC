from __future__ import annotations

import sys
from math import sqrt
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def test_timing_scoring_public_api_exports() -> None:
    from cormc.onestep.kernel import (
        ScoreResult,
        TimingResult,
        S,
        S_double_prime,
        S_prime,
        compute_acc_limit,
        compute_acc_time_lower_bound,
        compute_ego_cost,
        compute_speed_time_lower_bound,
        compute_timing_for_gap,
        compute_total_score,
        compute_unconstrained_time,
        score_gap,
        select_best_gap,
    )

    assert TimingResult.__name__ == "TimingResult"
    assert ScoreResult.__name__ == "ScoreResult"
    assert callable(S)
    assert callable(S_prime)
    assert callable(S_double_prime)
    assert callable(compute_acc_limit)
    assert callable(compute_unconstrained_time)
    assert callable(compute_acc_time_lower_bound)
    assert callable(compute_speed_time_lower_bound)
    assert callable(compute_timing_for_gap)
    assert callable(compute_ego_cost)
    assert callable(compute_total_score)
    assert callable(score_gap)
    assert callable(select_best_gap)


def test_quintic_basis_functions_match_boundary_conditions_and_peak_coefficients() -> None:
    from cormc.onestep.kernel import S, S_double_prime, S_prime

    tau_peak = (3.0 - sqrt(3.0)) / 6.0

    assert S(0.0) == pytest.approx(0.0, abs=1e-12)
    assert S(1.0) == pytest.approx(1.0, abs=1e-12)
    assert S_prime(0.0) == pytest.approx(0.0, abs=1e-12)
    assert S_prime(1.0) == pytest.approx(0.0, abs=1e-12)
    assert S_double_prime(0.0) == pytest.approx(0.0, abs=1e-12)
    assert S_double_prime(1.0) == pytest.approx(0.0, abs=1e-12)
    assert S_prime(0.5) == pytest.approx(15.0 / 8.0, abs=1e-12)
    assert S_double_prime(tau_peak) == pytest.approx(10.0 / sqrt(3.0), abs=1e-12)


def test_time_lower_bound_functions_match_formula_for_positive_and_negative_d() -> None:
    from cormc.onestep.kernel import (
        compute_acc_limit,
        compute_acc_time_lower_bound,
        compute_speed_time_lower_bound,
        compute_unconstrained_time,
    )
    from cormc.onestep.lab import (
        get_reference_algorithm_config,
        get_reference_scenario,
    )

    scenario = get_reference_scenario()
    algorithm = get_reference_algorithm_config()
    a_lim = compute_acc_limit(scenario)

    assert compute_unconstrained_time(72.5, algorithm) == pytest.approx(12.8224, abs=1e-4)
    assert a_lim == pytest.approx(3.0, abs=1e-12)
    assert compute_acc_time_lower_bound(72.5, a_lim) == pytest.approx(11.8121, abs=1e-4)
    assert compute_speed_time_lower_bound(72.5, scenario) == pytest.approx(13.59375, abs=1e-6)

    assert compute_speed_time_lower_bound(-135.0, scenario) == pytest.approx(12.65625, abs=1e-6)
    assert compute_unconstrained_time(10.0, algorithm) == pytest.approx(4.7621, abs=1e-4)
    assert compute_acc_time_lower_bound(10.0, a_lim) == pytest.approx(4.3869, abs=1e-4)
    assert compute_speed_time_lower_bound(10.0, scenario) == pytest.approx(1.875, abs=1e-9)


def test_merge_time_can_be_dominated_by_t0_tv_or_ta() -> None:
    from cormc.onestep.kernel import (
        AlgorithmConfig,
        Gap,
        ScenarioConfig,
        compute_acc_limit,
        compute_acc_time_lower_bound,
        compute_speed_time_lower_bound,
        compute_timing_for_gap,
        compute_unconstrained_time,
    )
    from cormc.onestep.lab import (
        get_reference_algorithm_config,
        get_reference_scenario,
    )
    from cormc.onestep.kernel.models import CooperationResult

    scenario = get_reference_scenario()
    algorithm = get_reference_algorithm_config()
    a_lim = compute_acc_limit(scenario)

    t0 = compute_unconstrained_time(10.0, algorithm)
    ta = compute_acc_time_lower_bound(10.0, a_lim)
    tv = compute_speed_time_lower_bound(10.0, scenario)
    assert t0 > ta
    assert t0 > tv

    low_acc_scenario = ScenarioConfig(
        x_targets=scenario.x_targets,
        x_m0=scenario.x_m0,
        v_ref=scenario.v_ref,
        v_max=scenario.v_max,
        v_min=scenario.v_min,
        a_max=0.5,
        a_min=scenario.a_min,
        T=scenario.T,
    )
    low_acc_limit = compute_acc_limit(low_acc_scenario)
    assert low_acc_limit == pytest.approx(0.5, abs=1e-12)
    low_acc_ta = compute_acc_time_lower_bound(10.0, low_acc_limit)
    assert low_acc_ta > t0
    assert low_acc_ta > tv

    gap = Gap("gap_test", 0, 0.0, 10.0, 10.0, 5.0, 5.0)
    coop = CooperationResult(
        gap_id="gap_test",
        controllability_branch="A_both_controllable",
        front_controllable=True,
        rear_controllable=True,
        Delta=0.0,
        G_prev=0.0,
        G_next=0.0,
        delta_f_bar=0.0,
        delta_r_bar=0.0,
        coop_feasible=True,
        gamma_f=0.0,
        gamma_r=0.0,
        L=0.0,
        U=0.0,
        delta_f_raw=0.0,
        delta_f_star=0.0,
        delta_r_star=0.0,
        C_coop=0.0,
        d_i=10.0,
    )
    timing = compute_timing_for_gap(gap, coop, low_acc_scenario, algorithm)
    assert timing.t_m == pytest.approx(low_acc_ta, abs=1e-9)


def test_reference_timing_matches_document_strict_values() -> None:
    from cormc.onestep.kernel import (
        build_gaps,
        compute_cooperation_for_gap,
        compute_derived_params,
        compute_reachability_for_gap,
        compute_timing_for_gap,
        filter_reachable_gaps,
    )
    from cormc.onestep.lab import (
        get_reference_algorithm_config,
        get_reference_scenario,
    )

    scenario = get_reference_scenario()
    algorithm = get_reference_algorithm_config()
    derived = compute_derived_params(scenario, algorithm)
    gaps = build_gaps(scenario)
    reachability = tuple(compute_reachability_for_gap(gap, scenario, derived) for gap in gaps)
    reachable_gaps = filter_reachable_gaps(gaps, reachability)
    cooperation = {
        gap.gap_id: compute_cooperation_for_gap(gap, gaps, scenario, algorithm, derived)
        for gap in reachable_gaps
    }
    timing = {
        gap.gap_id: compute_timing_for_gap(gap, cooperation[gap.gap_id], scenario, algorithm)
        for gap in reachable_gaps
    }

    assert timing["gap1"].t_0 == pytest.approx(17.497, abs=1e-3)
    assert timing["gap1"].t_a == pytest.approx(16.119, abs=1e-3)
    assert timing["gap1"].t_v == pytest.approx(12.656, abs=1e-3)
    assert timing["gap1"].t_m == pytest.approx(17.497, abs=1e-3)
    assert timing["gap1"].p_m == pytest.approx(214.944, abs=1e-3)

    assert timing["gap4"].t_0 == pytest.approx(12.822, abs=1e-3)
    assert timing["gap4"].t_a == pytest.approx(11.812, abs=1e-3)
    assert timing["gap4"].t_v == pytest.approx(13.594, abs=1e-3)
    assert timing["gap4"].t_m == pytest.approx(13.594, abs=1e-3)
    assert timing["gap4"].p_m == pytest.approx(344.375, abs=1e-3)

    assert timing["gap5"].t_v == pytest.approx(27.656, abs=1e-3)
    assert timing["gap5"].t_m == pytest.approx(27.656, abs=1e-3)
    assert timing["gap5"].p_m == pytest.approx(700.625, abs=1e-3)


def test_reference_strict_scores_match_stage1_baseline() -> None:
    from cormc.onestep.kernel import (
        build_gaps,
        compute_cooperation_for_gap,
        compute_derived_params,
        compute_reachability_for_gap,
        compute_timing_for_gap,
        filter_reachable_gaps,
        score_gap,
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
    expected_rows = get_reference_expected().strict_score_rows

    results = []
    for gap in reachable_gaps:
        coop = compute_cooperation_for_gap(gap, gaps, scenario, algorithm, derived)
        timing = compute_timing_for_gap(gap, coop, scenario, algorithm)
        results.append(score_gap(gap, coop, timing, algorithm))
    score_results = tuple(results)

    assert len(score_results) == len(expected_rows)

    for result, expected in zip(score_results, expected_rows):
        assert result.gap_id == expected.gap_id
        assert result.d_i == pytest.approx(expected.d_i, abs=1e-3)
        assert result.t_v == pytest.approx(expected.t_v, abs=1e-3)
        assert result.t_m == pytest.approx(expected.t_m, abs=1e-3)
        assert result.p_m == pytest.approx(expected.p_m, abs=1e-3)
        assert result.C_coop == pytest.approx(expected.C_coop, abs=1e-3)
        assert result.C_ego == pytest.approx(expected.C_ego, abs=1e-3)
        assert result.J == pytest.approx(expected.J, abs=1e-3)

    by_id = {result.gap_id: result for result in score_results}
    assert by_id["gap1"].C_ego == pytest.approx(233.296, abs=1e-3)
    assert by_id["gap1"].J == pytest.approx(233.296, abs=1e-3)
    assert by_id["gap4"].C_coop == pytest.approx(25.0, abs=1e-9)
    assert by_id["gap4"].C_ego == pytest.approx(171.808, abs=1e-3)
    assert by_id["gap4"].J == pytest.approx(176.808, abs=1e-3)
    assert by_id["gap5"].C_ego == pytest.approx(294.194, abs=1e-3)
    assert by_id["gap5"].J == pytest.approx(299.194, abs=1e-3)


def test_select_best_gap_uses_lowest_J_then_more_forward_gap() -> None:
    from cormc.onestep.kernel import ScoreResult, select_best_gap

    scores = (
        ScoreResult("gap4", 3, 30.0, 110.0, 72.5, 5.0, 0.0, 25.0, 12.8, 3.0, 11.8, 13.59, 13.59, 344.375, 171.808, 176.808),
        ScoreResult("gap3", 2, -25.0, 30.0, 16.491, 28.991, 1.009, 869.720, 6.1, 3.0, 5.63, 3.09, 6.115, 138.797, 81.539, 176.808),
    )

    best = select_best_gap(scores)

    assert best.gap_id == "gap4"
    assert best.gap_index == 3


def test_reference_decision_selects_gap4_with_expected_key_values() -> None:
    from cormc.onestep.kernel import (
        build_gaps,
        compute_cooperation_for_gap,
        compute_derived_params,
        compute_reachability_for_gap,
        compute_timing_for_gap,
        filter_reachable_gaps,
        score_gap,
        select_best_gap,
    )
    from cormc.onestep.lab import (
        get_reference_algorithm_config,
        get_reference_scenario,
    )

    scenario = get_reference_scenario()
    algorithm = get_reference_algorithm_config()
    derived = compute_derived_params(scenario, algorithm)
    gaps = build_gaps(scenario)
    reachability = tuple(compute_reachability_for_gap(gap, scenario, derived) for gap in gaps)
    reachable_gaps = filter_reachable_gaps(gaps, reachability)

    score_results = []
    for gap in reachable_gaps:
        coop = compute_cooperation_for_gap(gap, gaps, scenario, algorithm, derived)
        if not coop.coop_feasible:
            continue
        timing = compute_timing_for_gap(gap, coop, scenario, algorithm)
        score_results.append(score_gap(gap, coop, timing, algorithm))

    score_results_tuple = tuple(score_results)
    best = select_best_gap(score_results_tuple)

    assert len(score_results_tuple) == 5
    assert best.gap_id == "gap4"
    assert best.x_rear == pytest.approx(30.0, abs=1e-9)
    assert best.x_front == pytest.approx(110.0, abs=1e-9)
    assert best.delta_f_star == pytest.approx(5.0, abs=1e-9)
    assert best.delta_r_star == pytest.approx(0.0, abs=1e-9)
    assert best.d_i == pytest.approx(72.5, abs=1e-9)
    assert best.t_m == pytest.approx(13.594, abs=1e-3)
    assert best.p_m == pytest.approx(344.375, abs=1e-3)
    assert best.J == pytest.approx(176.808, abs=1e-3)
