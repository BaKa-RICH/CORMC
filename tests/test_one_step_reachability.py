from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def test_reachability_public_api_exports() -> None:
    from cormc.onestep.kernel import (
        DirectionalReachKinematics,
        ReachabilityResult,
        compute_front_reach_kinematics,
        compute_reachability_for_gap,
        compute_rear_reach_kinematics,
        filter_reachable_gaps,
    )

    assert DirectionalReachKinematics.__name__ == "DirectionalReachKinematics"
    assert ReachabilityResult.__name__ == "ReachabilityResult"
    assert callable(compute_front_reach_kinematics)
    assert callable(compute_rear_reach_kinematics)
    assert callable(compute_reachability_for_gap)
    assert callable(filter_reachable_gaps)


def test_front_reach_kinematics_triangular_case_uses_zero_cruise() -> None:
    from cormc.onestep.kernel import (
        compute_derived_params,
        compute_front_reach_kinematics,
    )
    from cormc.onestep.lab import (
        get_reference_algorithm_config,
        get_reference_scenario,
    )

    scenario = get_reference_scenario()
    derived = compute_derived_params(scenario, get_reference_algorithm_config())

    result = compute_front_reach_kinematics(2.5, scenario, derived)

    assert result.direction == "front"
    assert result.v_peak == pytest.approx(2.9277, abs=1e-4)
    assert result.t_acc == pytest.approx(0.9759, abs=1e-4)
    assert result.t_dec == pytest.approx(0.7319, abs=1e-4)
    assert result.s_acc == pytest.approx(1.428571, abs=1e-6)
    assert result.s_dec == pytest.approx(1.071429, abs=1e-6)
    assert result.s_cruise == pytest.approx(0.0, abs=1e-9)
    assert result.t_reach == pytest.approx(1.7078, abs=1e-4)


def test_front_reach_kinematics_trapezoidal_case_hits_speed_cap() -> None:
    from cormc.onestep.kernel import (
        compute_derived_params,
        compute_front_reach_kinematics,
    )
    from cormc.onestep.lab import (
        get_reference_algorithm_config,
        get_reference_scenario,
    )

    scenario = get_reference_scenario()
    derived = compute_derived_params(scenario, get_reference_algorithm_config())

    result = compute_front_reach_kinematics(70.0, scenario, derived)

    assert result.direction == "front"
    assert result.v_peak == pytest.approx(10.0, abs=1e-9)
    assert result.t_acc == pytest.approx(3.333333, abs=1e-6)
    assert result.t_dec == pytest.approx(2.5, abs=1e-9)
    assert result.s_acc == pytest.approx(16.666667, abs=1e-6)
    assert result.s_dec == pytest.approx(12.5, abs=1e-9)
    assert result.s_cruise == pytest.approx(40.833333, abs=1e-6)
    assert result.t_reach == pytest.approx(9.916667, abs=1e-6)


def test_rear_reach_kinematics_triangular_case_uses_zero_cruise() -> None:
    from cormc.onestep.kernel import (
        compute_derived_params,
        compute_rear_reach_kinematics,
    )
    from cormc.onestep.lab import (
        get_reference_algorithm_config,
        get_reference_scenario,
    )

    scenario = get_reference_scenario()
    derived = compute_derived_params(scenario, get_reference_algorithm_config())

    result = compute_rear_reach_kinematics(-57.5, scenario, derived)

    assert result.direction == "rear"
    assert result.v_peak == pytest.approx(14.040757, abs=1e-6)
    assert result.t_acc == pytest.approx(3.510189, abs=1e-6)
    assert result.t_dec == pytest.approx(4.680252, abs=1e-6)
    assert result.s_acc == pytest.approx(24.642857, abs=1e-6)
    assert result.s_dec == pytest.approx(32.857143, abs=1e-6)
    assert result.s_cruise == pytest.approx(0.0, abs=1e-9)
    assert result.t_reach == pytest.approx(8.190442, abs=1e-6)


def test_rear_reach_kinematics_trapezoidal_case_hits_speed_cap() -> None:
    from cormc.onestep.kernel import (
        compute_derived_params,
        compute_rear_reach_kinematics,
    )
    from cormc.onestep.lab import (
        get_reference_algorithm_config,
        get_reference_scenario,
    )

    scenario = get_reference_scenario()
    derived = compute_derived_params(scenario, get_reference_algorithm_config())

    result = compute_rear_reach_kinematics(-135.0, scenario, derived)

    assert result.direction == "rear"
    assert result.v_peak == pytest.approx(20.0, abs=1e-9)
    assert result.t_acc == pytest.approx(5.0, abs=1e-9)
    assert result.t_dec == pytest.approx(6.666667, abs=1e-6)
    assert result.s_acc == pytest.approx(50.0, abs=1e-9)
    assert result.s_dec == pytest.approx(66.666667, abs=1e-6)
    assert result.s_cruise == pytest.approx(18.333333, abs=1e-6)
    assert result.t_reach == pytest.approx(12.583333, abs=1e-6)


def test_reference_reachability_matches_stage1_baseline() -> None:
    from cormc.onestep.kernel import (
        build_gaps,
        compute_derived_params,
        compute_reachability_for_gap,
    )
    from cormc.onestep.lab import (
        get_reference_algorithm_config,
        get_reference_expected,
        get_reference_scenario,
    )

    scenario = get_reference_scenario()
    derived = compute_derived_params(scenario, get_reference_algorithm_config())
    gaps = build_gaps(scenario)
    expected_rows = get_reference_expected().reachability_rows

    results = tuple(compute_reachability_for_gap(gap, scenario, derived) for gap in gaps)

    assert len(results) == len(expected_rows)

    for result, expected in zip(results, expected_rows):
        assert result.gap_id == expected.gap_id
        assert result.D_i == expected.D_i
        assert result.t_reach == pytest.approx(expected.t_reach, abs=1e-3)
        assert result.p_pre == pytest.approx(expected.p_pre, abs=1e-3)
        assert result.reachable is expected.reachable

    by_id = {result.gap_id: result for result in results}
    assert by_id["gap1"].reachable is True
    assert by_id["gap5"].reachable is True
    assert by_id["gap6"].reachable is False
    assert by_id["gap6"].t_reach == pytest.approx(24.917, abs=1e-3)
    assert by_id["gap6"].p_pre == pytest.approx(718.333, abs=1e-3)


def test_filter_reachable_gaps_returns_gap1_to_gap5_only() -> None:
    from cormc.onestep.kernel import (
        build_gaps,
        compute_derived_params,
        compute_reachability_for_gap,
        filter_reachable_gaps,
    )
    from cormc.onestep.lab import (
        get_reference_algorithm_config,
        get_reference_scenario,
    )

    scenario = get_reference_scenario()
    derived = compute_derived_params(scenario, get_reference_algorithm_config())
    gaps = build_gaps(scenario)
    reachability = tuple(
        compute_reachability_for_gap(gap, scenario, derived)
        for gap in gaps
    )

    filtered = filter_reachable_gaps(gaps, reachability)

    assert tuple(gap.gap_id for gap in filtered) == (
        "gap1",
        "gap2",
        "gap3",
        "gap4",
        "gap5",
    )


def test_filter_reachable_gaps_uses_precomputed_boolean_only() -> None:
    from cormc.onestep.kernel import (
        ReachabilityResult,
        build_gaps,
        filter_reachable_gaps,
    )
    from cormc.onestep.lab import get_reference_scenario

    gaps = build_gaps(get_reference_scenario())
    synthetic = (
        ReachabilityResult("gap1", -135.0, "rear", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 999.0, 0.0, False),
        ReachabilityResult("gap2", -57.5, "rear", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 999.0, 0.0, True),
        ReachabilityResult("gap3", 2.5, "front", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 999.0, 0.0, False),
        ReachabilityResult("gap4", 70.0, "front", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 999.0, 0.0, True),
        ReachabilityResult("gap5", 150.0, "front", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 999.0, 0.0, False),
        ReachabilityResult("gap6", 220.0, "front", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 999.0, 0.0, True),
    )

    filtered = filter_reachable_gaps(gaps, synthetic)

    assert tuple(gap.gap_id for gap in filtered) == ("gap2", "gap4", "gap6")
