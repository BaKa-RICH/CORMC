from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def _build_reference_best_score():
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
        get_reference_expected,
        get_reference_scenario,
    )

    scenario = get_reference_scenario()
    algorithm = get_reference_algorithm_config()
    derived = compute_derived_params(scenario, algorithm)
    gaps = build_gaps(scenario)
    reachability = tuple(compute_reachability_for_gap(gap, scenario, derived) for gap in gaps)
    reachable_gaps = filter_reachable_gaps(gaps, reachability)

    scores = []
    for gap in reachable_gaps:
        coop = compute_cooperation_for_gap(gap, gaps, scenario, algorithm, derived)
        if not coop.coop_feasible:
            continue
        timing = compute_timing_for_gap(gap, coop, scenario, algorithm)
        scores.append(score_gap(gap, coop, timing, algorithm))

    best_score = select_best_gap(tuple(scores))
    contract = get_reference_expected().trajectory_contract
    return scenario, gaps, best_score, contract


def test_trajectory_public_api_exports() -> None:
    from cormc.onestep.kernel import (
        TrajectoryArtifacts,
        TrajectoryBundle,
        TrajectorySample,
        build_best_gap_trajectory_bundle,
        build_time_grid,
        compute_dynamic_gap_center,
        compute_dynamic_gap_length,
        compute_tau,
        sample_constant_speed_vehicle_state,
        sample_merge_vehicle_state,
        sample_selected_front_vehicle_state,
        sample_selected_rear_vehicle_state,
    )

    assert TrajectorySample.__name__ == "TrajectorySample"
    assert TrajectoryBundle.__name__ == "TrajectoryBundle"
    assert TrajectoryArtifacts.__name__ == "TrajectoryArtifacts"
    assert callable(build_time_grid)
    assert callable(compute_tau)
    assert callable(sample_merge_vehicle_state)
    assert callable(sample_selected_rear_vehicle_state)
    assert callable(sample_selected_front_vehicle_state)
    assert callable(sample_constant_speed_vehicle_state)
    assert callable(compute_dynamic_gap_length)
    assert callable(compute_dynamic_gap_center)
    assert callable(build_best_gap_trajectory_bundle)


def test_time_grid_includes_zero_and_merge_time() -> None:
    from cormc.onestep.kernel import build_time_grid

    grid = build_time_grid(13.59375, 0.1)

    assert grid[0] == 0.0
    assert grid[-1] == pytest.approx(13.59375, abs=1e-12)
    assert 0.0 in grid
    assert 13.59375 in grid
    assert all(left < right for left, right in zip(grid, grid[1:]))


def test_merge_vehicle_quintic_trajectory_matches_formula_endpoints() -> None:
    from cormc.onestep.kernel import sample_merge_vehicle_state
    from cormc.onestep.lab import get_reference_scenario

    scenario, _, best_score, _ = _build_reference_best_score()

    start = sample_merge_vehicle_state(0.0, scenario, best_score)
    end = sample_merge_vehicle_state(best_score.t_m, scenario, best_score)
    after = sample_merge_vehicle_state(best_score.t_m + 1.0, scenario, best_score)

    assert start.x == pytest.approx(0.0, abs=1e-12)
    assert start.v == pytest.approx(20.0, abs=1e-12)
    assert start.a == pytest.approx(0.0, abs=1e-12)
    assert end.x == pytest.approx(344.375, abs=1e-6)
    assert end.v == pytest.approx(20.0, abs=1e-9)
    assert end.a == pytest.approx(0.0, abs=1e-9)
    assert after.x == pytest.approx(344.375 + 20.0, abs=1e-6)
    assert after.v == pytest.approx(20.0, abs=1e-12)
    assert after.a == pytest.approx(0.0, abs=1e-12)


def test_selected_gap_vehicle_trajectories_match_formula_endpoints() -> None:
    from cormc.onestep.kernel import (
        sample_selected_front_vehicle_state,
        sample_selected_rear_vehicle_state,
    )

    scenario, gaps, best_score, _ = _build_reference_best_score()
    best_gap = next(gap for gap in gaps if gap.gap_id == best_score.gap_id)

    rear_end = sample_selected_rear_vehicle_state(best_score.t_m, scenario, best_gap, best_score)
    front_end = sample_selected_front_vehicle_state(best_score.t_m, scenario, best_gap, best_score)
    rear_after = sample_selected_rear_vehicle_state(best_score.t_m + 1.0, scenario, best_gap, best_score)
    front_after = sample_selected_front_vehicle_state(best_score.t_m + 1.0, scenario, best_gap, best_score)

    assert rear_end.x == pytest.approx(301.875, abs=1e-6)
    assert rear_end.v == pytest.approx(20.0, abs=1e-9)
    assert rear_end.a == pytest.approx(0.0, abs=1e-9)
    assert front_end.x == pytest.approx(386.875, abs=1e-6)
    assert front_end.v == pytest.approx(20.0, abs=1e-9)
    assert front_end.a == pytest.approx(0.0, abs=1e-9)
    assert rear_after.x == pytest.approx(301.875 + 20.0, abs=1e-6)
    assert front_after.x == pytest.approx(386.875 + 20.0, abs=1e-6)


def test_reference_dynamic_gap_length_and_center_match_document() -> None:
    from cormc.onestep.kernel import (
        compute_dynamic_gap_center,
        compute_dynamic_gap_length,
        sample_selected_front_vehicle_state,
        sample_selected_rear_vehicle_state,
    )

    scenario, gaps, best_score, _ = _build_reference_best_score()
    best_gap = next(gap for gap in gaps if gap.gap_id == best_score.gap_id)

    rear = sample_selected_rear_vehicle_state(best_score.t_m, scenario, best_gap, best_score)
    front = sample_selected_front_vehicle_state(best_score.t_m, scenario, best_gap, best_score)

    assert compute_dynamic_gap_length(front.x, rear.x) == pytest.approx(85.0, abs=1e-6)
    assert compute_dynamic_gap_center(front.x, rear.x) == pytest.approx(344.375, abs=1e-6)


def test_non_selected_vehicles_remain_constant_speed_20() -> None:
    from cormc.onestep.kernel import NON_SELECTED_ROLE, build_best_gap_trajectory_bundle

    scenario, gaps, best_score, contract = _build_reference_best_score()
    bundle = build_best_gap_trajectory_bundle(scenario, gaps, best_score, contract)

    first_positions = {
        -180.0: "target_lane_-180m",
        -90.0: "target_lane_-90m",
        -25.0: "target_lane_-25m",
        190.0: "target_lane_190m",
        250.0: "target_lane_250m",
    }

    for x0, vehicle_id in first_positions.items():
        vehicle_samples = [sample for sample in bundle.samples if sample.vehicle_id == vehicle_id]
        assert vehicle_samples
        for sample in vehicle_samples:
            assert sample.role == NON_SELECTED_ROLE
            assert sample.v == pytest.approx(20.0, abs=1e-12)
            assert sample.x == pytest.approx(x0 + 20.0 * sample.t, abs=1e-9)
            assert sample.a == pytest.approx(0.0, abs=1e-12)
            assert sample.is_selected_gap_vehicle is False
            assert sample.is_merge_vehicle is False


def test_reference_best_gap_trajectory_bundle_uses_real_stage5_decision() -> None:
    from cormc.onestep.kernel import build_best_gap_trajectory_bundle

    scenario, gaps, best_score, contract = _build_reference_best_score()
    bundle = build_best_gap_trajectory_bundle(scenario, gaps, best_score, contract)

    vehicle_ids = {sample.vehicle_id for sample in bundle.samples}

    assert bundle.selected_gap_id == "gap4"
    assert bundle.selected_gap_interval == (30.0, 110.0)
    assert bundle.merge_point_x == pytest.approx(344.375, abs=1e-3)
    assert contract.selected_gap_vehicle_ids[0] in vehicle_ids
    assert contract.selected_gap_vehicle_ids[1] in vehicle_ids
    assert "merge_vehicle" in vehicle_ids
    assert bundle.check_times == contract.required_check_times
