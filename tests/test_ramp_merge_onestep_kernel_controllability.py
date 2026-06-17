from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from cormc.onestep.lab.reference_case import (
    get_reference_algorithm_config,
    get_reference_scenario,
)
from cormc.onestep.kernel.config import (
    GapBoundaryControllability,
    ScenarioConfig,
)
from cormc.onestep.kernel.evaluation import evaluate_one_step_scenario
from cormc.onestep.kernel.gaps import build_gaps
from cormc.onestep.kernel.models import (
    CONTROLLABILITY_BRANCH_A,
    CONTROLLABILITY_BRANCH_B,
    CONTROLLABILITY_BRANCH_C,
    CONTROLLABILITY_BRANCH_D,
)
from cormc.onestep.kernel.timing_scoring import FAILED_GAP_SCORE


def _reference_scenario(
    gap_boundary_controllability: tuple[GapBoundaryControllability, ...] = (),
) -> ScenarioConfig:
    reference = get_reference_scenario()
    return ScenarioConfig(
        x_targets=reference.x_targets,
        x_m0=reference.x_m0,
        v_ref=reference.v_ref,
        v_max=reference.v_max,
        v_min=reference.v_min,
        a_max=reference.a_max,
        a_min=reference.a_min,
        T=reference.T,
        gap_boundary_controllability=gap_boundary_controllability,
    )


def _evaluate(
    gap_boundary_controllability: tuple[GapBoundaryControllability, ...] = (),
):
    return evaluate_one_step_scenario(
        _reference_scenario(gap_boundary_controllability),
        get_reference_algorithm_config(),
    )


def _rows_by_id(evaluation):
    return {row.gap_id: row for row in evaluation.gap_rows}


def test_build_gaps_defaults_to_both_controllable_and_preserves_numbering() -> None:
    scenario = ScenarioConfig(
        x_targets=(0.0, 50.0, 120.0, 200.0),
        x_m0=0.0,
        v_ref=20.0,
        v_max=30.0,
        v_min=0.0,
        a_max=3.0,
        a_min=-4.0,
        T=20.0,
    )

    gaps = build_gaps(scenario)

    assert tuple(gap.gap_id for gap in gaps) == ("gap1", "gap2", "gap3")
    assert tuple(gap.index for gap in gaps) == (0, 1, 2)
    assert all(gap.front_controllable for gap in gaps)
    assert all(gap.rear_controllable for gap in gaps)


def test_build_gaps_propagates_explicit_boundary_controllability_by_gap_index() -> None:
    scenario = ScenarioConfig(
        x_targets=(0.0, 50.0, 120.0, 200.0),
        x_m0=0.0,
        v_ref=20.0,
        v_max=30.0,
        v_min=0.0,
        a_max=3.0,
        a_min=-4.0,
        T=20.0,
        gap_boundary_controllability=(
            GapBoundaryControllability(
                gap_index=0,
                front_controllable=True,
                rear_controllable=False,
                front_vehicle_id="front-0",
                rear_vehicle_id="rear-0",
            ),
            GapBoundaryControllability(
                gap_index=1,
                front_controllable=False,
                rear_controllable=True,
                front_vehicle_id="front-1",
                rear_vehicle_id="rear-1",
            ),
            GapBoundaryControllability(
                gap_index=2,
                front_controllable=False,
                rear_controllable=False,
                front_vehicle_id="front-2",
                rear_vehicle_id="rear-2",
            ),
        ),
    )

    gaps = build_gaps(scenario)

    assert [(gap.front_controllable, gap.rear_controllable) for gap in gaps] == [
        (True, False),
        (False, True),
        (False, False),
    ]
    assert tuple(gap.front_vehicle_id for gap in gaps) == (
        "front-0",
        "front-1",
        "front-2",
    )
    assert tuple(gap.rear_vehicle_id for gap in gaps) == (
        "rear-0",
        "rear-1",
        "rear-2",
    )


def test_reference_case_default_rows_are_branch_a_and_keep_best_gap_values() -> None:
    evaluation = _evaluate()
    scored_rows = [row for row in evaluation.gap_rows if row.included_in_scoring]

    assert evaluation.best_gap is not None
    assert evaluation.best_score is not None
    assert evaluation.best_gap.gap_id == "gap4"
    assert evaluation.best_score.delta_f_star == pytest.approx(5.0)
    assert evaluation.best_score.delta_r_star == pytest.approx(0.0)
    assert evaluation.best_score.J == pytest.approx(176.80832649151614)
    assert all(row.controllability_branch == CONTROLLABILITY_BRANCH_A for row in scored_rows)


def test_branch_b_front_only_success_zeroes_rear_capacity_and_scores_normally() -> None:
    evaluation = _evaluate(
        (
            GapBoundaryControllability(
                gap_index=2,
                front_controllable=True,
                rear_controllable=False,
                front_vehicle_id="front-gap3",
                rear_vehicle_id="rear-gap3",
            ),
        )
    )
    row = _rows_by_id(evaluation)["gap3"]

    assert row.controllability_branch == CONTROLLABILITY_BRANCH_B
    assert row.front_vehicle_id == "front-gap3"
    assert row.rear_vehicle_id == "rear-gap3"
    assert row.front_controllable is True
    assert row.rear_controllable is False
    assert row.delta_f_bar == pytest.approx(35.0)
    assert row.delta_r_bar == pytest.approx(0.0)
    assert row.delta_f_star == pytest.approx(30.0)
    assert row.delta_r_star == pytest.approx(0.0)
    assert row.included_in_scoring is True
    assert row.J != FAILED_GAP_SCORE


def test_branch_c_rear_only_success_zeroes_front_capacity_and_scores_normally() -> None:
    evaluation = _evaluate(
        (
            GapBoundaryControllability(
                gap_index=1,
                front_controllable=False,
                rear_controllable=True,
                front_vehicle_id="front-gap2",
                rear_vehicle_id="rear-gap2",
            ),
        )
    )
    row = _rows_by_id(evaluation)["gap2"]

    assert row.controllability_branch == CONTROLLABILITY_BRANCH_C
    assert row.front_controllable is False
    assert row.rear_controllable is True
    assert row.delta_f_bar == pytest.approx(0.0)
    assert row.delta_r_bar == pytest.approx(45.0)
    assert row.delta_f_star == pytest.approx(0.0)
    assert row.delta_r_star == pytest.approx(20.0)
    assert row.included_in_scoring is True
    assert row.J != FAILED_GAP_SCORE


@pytest.mark.parametrize(
    ("gap_index", "gap_id", "front_controllable", "rear_controllable", "branch"),
    [
        (1, "gap2", True, False, CONTROLLABILITY_BRANCH_B),
        (2, "gap3", False, True, CONTROLLABILITY_BRANCH_C),
    ],
)
def test_branch_b_or_c_single_side_insufficient_capacity_keeps_failed_row_out_of_best_selection(
    gap_index: int,
    gap_id: str,
    front_controllable: bool,
    rear_controllable: bool,
    branch: str,
) -> None:
    evaluation = _evaluate(
        (
            GapBoundaryControllability(
                gap_index=gap_index,
                front_controllable=front_controllable,
                rear_controllable=rear_controllable,
            ),
        )
    )
    row = _rows_by_id(evaluation)[gap_id]

    assert row.controllability_branch == branch
    assert row.coop_feasible is False
    assert row.J == FAILED_GAP_SCORE
    assert row.included_in_scoring is False
    assert row.is_selected is False
    assert row.failure_reason == "insufficient_single_side_capacity"
    assert evaluation.best_gap is not None
    assert evaluation.best_gap.gap_id == "gap4"


def test_branch_d_gap_not_over_95m_keeps_failed_row_with_large_score() -> None:
    evaluation = _evaluate(
        (
            GapBoundaryControllability(
                gap_index=2,
                front_controllable=False,
                rear_controllable=False,
            ),
        )
    )
    row = _rows_by_id(evaluation)["gap3"]

    assert row.controllability_branch == CONTROLLABILITY_BRANCH_D
    assert row.G_i == pytest.approx(55.0)
    assert row.delta_f_bar == pytest.approx(0.0)
    assert row.delta_r_bar == pytest.approx(0.0)
    assert row.coop_feasible is False
    assert row.J == FAILED_GAP_SCORE
    assert row.included_in_scoring is False
    assert row.is_selected is False
    assert row.failure_reason == "none_controllable_gap_not_over_95m"
    assert evaluation.best_gap is not None
    assert evaluation.best_gap.gap_id != row.gap_id


def test_branch_d_gap_over_95m_scores_as_ego_only_fallback() -> None:
    scenario = ScenarioConfig(
        x_targets=(10.0, -100.0),
        x_m0=0.0,
        v_ref=20.0,
        v_max=30.0,
        v_min=0.0,
        a_max=3.0,
        a_min=-4.0,
        T=20.0,
        gap_boundary_controllability=(
            GapBoundaryControllability(
                gap_index=0,
                front_controllable=False,
                rear_controllable=False,
                front_vehicle_id="front",
                rear_vehicle_id="rear",
            ),
        ),
    )

    evaluation = evaluate_one_step_scenario(scenario, get_reference_algorithm_config())
    row = evaluation.gap_rows[0]

    assert evaluation.status == "solved"
    assert evaluation.best_gap is not None
    assert evaluation.best_gap.gap_id == row.gap_id
    assert row.controllability_branch == CONTROLLABILITY_BRANCH_D
    assert row.G_i == pytest.approx(110.0)
    assert row.delta_f_bar == pytest.approx(0.0)
    assert row.delta_r_bar == pytest.approx(0.0)
    assert row.delta_f_star == pytest.approx(0.0)
    assert row.delta_r_star == pytest.approx(0.0)
    assert row.C_coop == pytest.approx(0.0)
    assert row.d_i == pytest.approx(row.c_i - scenario.x_m0)
    assert row.included_in_scoring is True
    assert row.is_selected is True
    assert row.failure_reason is None
    assert row.J != FAILED_GAP_SCORE


def test_all_reachable_failed_rows_leave_evaluation_without_best_gap() -> None:
    evaluation = _evaluate(
        tuple(
            GapBoundaryControllability(
                gap_index=index,
                front_controllable=False,
                rear_controllable=False,
            )
            for index in range(6)
        )
    )
    reachable_rows = [row for row in evaluation.gap_rows if row.reachable]

    assert evaluation.status == "no_solution"
    assert evaluation.no_solution_reason == "no_coop_feasible_gap"
    assert evaluation.best_gap is None
    assert evaluation.best_score is None
    assert all(row.J == FAILED_GAP_SCORE for row in reachable_rows)
    assert all(row.included_in_scoring is False for row in reachable_rows)
    assert all(row.is_selected is False for row in reachable_rows)
