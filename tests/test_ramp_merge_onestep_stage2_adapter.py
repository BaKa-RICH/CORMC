from __future__ import annotations

from dataclasses import dataclass, replace
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from cormc.onestep.lab.reference_case import get_reference_expected
from cormc.scenes import (
    RM_ONESTEP_S05_GAP_INTERVALS_LOCAL,
    RM_ONESTEP_S05_MAINLINE_X_LOCAL,
    RM_ONESTEP_S05_MV_ID,
    RM_ONESTEP_S05_PLAN_STEP0_SCENARIO_ID,
    RM_ONESTEP_S05_ROLLING_ENTRY_SCENARIO_ID,
    RM_ONESTEP_S07_GAP_INTERVALS_LOCAL,
    RM_ONESTEP_S07_MAINLINE_X_LOCAL,
    RM_ONESTEP_S07_MV_ID,
    RM_ONESTEP_S07_PLAN_STEP0_SCENARIO_ID,
    RM_ONESTEP_S07_ROLLING_ENTRY_SCENARIO_ID,
)
from cormc.onestep.rolling import (
    build_initial_onestep_stage2_state,
    build_stage2_local_frame,
    evaluate_stage2_one_step_from_snapshot,
    identify_and_number_gaps,
    evaluate_stage2_one_step,
    resolve_best_gap_vehicle_ids,
)
from cormc.onestep.kernel.models import (
    CONTROLLABILITY_BRANCH_A,
    CONTROLLABILITY_BRANCH_B,
    CONTROLLABILITY_BRANCH_C,
    CONTROLLABILITY_BRANCH_D,
)
from cormc.onestep.kernel.timing_scoring import FAILED_GAP_SCORE


@dataclass(frozen=True)
class _BestGap:
    index: int
    x_rear: float = 0.0
    x_front: float = 0.0


@dataclass(frozen=True)
class _EvaluationLike:
    best_gap: _BestGap | None


@pytest.mark.parametrize(
    ("scenario_id", "mv_id", "expected_x_targets", "expected_gap_intervals", "expected_gap_interval", "expected_ids"),
    [
        (
            RM_ONESTEP_S05_PLAN_STEP0_SCENARIO_ID,
            RM_ONESTEP_S05_MV_ID,
            RM_ONESTEP_S05_MAINLINE_X_LOCAL,
            RM_ONESTEP_S05_GAP_INTERVALS_LOCAL,
            (-100.0, -50.0),
            ("S05_L2_01", "S05_L2_02"),
        ),
        (
            RM_ONESTEP_S07_PLAN_STEP0_SCENARIO_ID,
            RM_ONESTEP_S07_MV_ID,
            RM_ONESTEP_S07_MAINLINE_X_LOCAL,
            RM_ONESTEP_S07_GAP_INTERVALS_LOCAL,
            (30.0, 110.0),
            ("S07_L2_04", "S07_L2_05"),
        ),
    ],
)
def test_stage2_adapter_matches_plan_step0_local_frame(
    scenario_id: str,
    mv_id: str,
    expected_x_targets: tuple[float, ...],
    expected_gap_intervals: tuple[tuple[float, float], ...],
    expected_gap_interval: tuple[float, float],
    expected_ids: tuple[str, str],
) -> None:
    state, _ = build_initial_onestep_stage2_state(scenario_id)
    local_frame = build_stage2_local_frame(state, mv_id)
    evaluation = evaluate_stage2_one_step(state, mv_id)
    rear_id, front_id = resolve_best_gap_vehicle_ids(local_frame, evaluation.evaluation)

    assert local_frame["origin_x_global"] == pytest.approx(6650.0)
    assert evaluation.local_scenario.x_targets == expected_x_targets
    assert local_frame["gap_intervals_local"] == expected_gap_intervals
    assert local_frame["gap_vehicle_ids_by_index"][evaluation.evaluation.best_gap.index] == expected_ids
    assert evaluation.evaluation.best_gap is not None
    assert (
        evaluation.evaluation.best_gap.x_rear,
        evaluation.evaluation.best_gap.x_front,
    ) == pytest.approx(expected_gap_interval)
    assert rear_id == expected_ids[0]
    assert front_id == expected_ids[1]


def test_stage2_adapter_local_frame_maps_gap_index_to_lane2_vehicle_ids() -> None:
    state, _ = build_initial_onestep_stage2_state(RM_ONESTEP_S07_PLAN_STEP0_SCENARIO_ID)
    local_frame = build_stage2_local_frame(state, RM_ONESTEP_S07_MV_ID)

    assert local_frame["gap_vehicle_ids_by_index"] == {
        0: ("S07_L2_01", "S07_L2_02"),
        1: ("S07_L2_02", "S07_L2_03"),
        2: ("S07_L2_03", "S07_L2_04"),
        3: ("S07_L2_04", "S07_L2_05"),
        4: ("S07_L2_05", "S07_L2_06"),
        5: ("S07_L2_06", "S07_L2_07"),
    }
    assert local_frame["runtime_gap_index_by_kernel_index"] == {
        0: 6,
        1: 5,
        2: 4,
        3: 3,
        4: 2,
        5: 1,
    }


def test_resolve_best_gap_vehicle_ids_uses_gap_index_not_local_x_lookup() -> None:
    local_frame = {
        "lane_2_vehicle_x_local_by_id": {
            "wrong_rear_by_x": 10.0,
            "wrong_front_by_x": 20.0,
        },
        "gap_vehicle_ids_by_index": {
            2: ("rear_from_index", "front_from_index"),
        },
    }
    evaluation = _EvaluationLike(best_gap=_BestGap(index=2, x_rear=10.0, x_front=20.0))

    assert resolve_best_gap_vehicle_ids(local_frame, evaluation) == (
        "rear_from_index",
        "front_from_index",
    )


def test_stage2_adapter_first_effective_trigger_matches_reference_case_in_s07_rolling_entry() -> None:
    state, _ = build_initial_onestep_stage2_state(RM_ONESTEP_S07_ROLLING_ENTRY_SCENARIO_ID)
    for _ in range(25):
        next_vehicle_states = {
            vehicle_id: replace(
                vehicle,
                x_global=vehicle.x_global + 20.0 * state.dt,
            )
            for vehicle_id, vehicle in state.vehicle_states.items()
        }
        state = replace(
            state,
            t=state.t + state.dt,
            step=state.step + 1,
            vehicle_states=next_vehicle_states,
        )
    evaluation = evaluate_stage2_one_step(state, RM_ONESTEP_S07_MV_ID)
    expected = get_reference_expected()

    assert evaluation.origin_x_global == pytest.approx(6650.0)
    assert evaluation.local_scenario.x_targets == expected.scenario.x_targets
    assert evaluation.evaluation.best_gap is not None
    assert evaluation.evaluation.best_score is not None
    assert evaluation.evaluation.best_gap.gap_id == expected.best_gap_id
    assert evaluation.evaluation.best_score.delta_f_star == pytest.approx(expected.best_delta_f_star)
    assert evaluation.evaluation.best_score.delta_r_star == pytest.approx(expected.best_delta_r_star)
    assert evaluation.evaluation.best_score.d_i == pytest.approx(expected.best_d_i)
    assert evaluation.evaluation.best_score.t_m == pytest.approx(expected.best_t_m, rel=1e-4)
    assert evaluation.evaluation.best_score.p_m == pytest.approx(expected.best_p_m, rel=1e-4)
    assert all(
        row.controllability_branch == CONTROLLABILITY_BRANCH_A
        for row in evaluation.evaluation.gap_rows
        if row.included_in_scoring
    )


def test_stage2_adapter_first_effective_trigger_matches_s05_expected_case() -> None:
    state, _ = build_initial_onestep_stage2_state(RM_ONESTEP_S05_ROLLING_ENTRY_SCENARIO_ID)
    for _ in range(25):
        next_vehicle_states = {
            vehicle_id: replace(
                vehicle,
                x_global=vehicle.x_global + 20.0 * state.dt,
            )
            for vehicle_id, vehicle in state.vehicle_states.items()
        }
        state = replace(
            state,
            t=state.t + state.dt,
            step=state.step + 1,
            vehicle_states=next_vehicle_states,
        )
    evaluation = evaluate_stage2_one_step(state, RM_ONESTEP_S05_MV_ID)

    assert evaluation.origin_x_global == pytest.approx(6650.0)
    assert evaluation.local_scenario.x_targets == RM_ONESTEP_S05_MAINLINE_X_LOCAL
    assert evaluation.evaluation.best_gap is not None
    assert evaluation.evaluation.best_score is not None
    assert evaluation.evaluation.best_gap.gap_id == "gap1"
    assert evaluation.evaluation.best_score.delta_f_star == pytest.approx(0.0)
    assert evaluation.evaluation.best_score.delta_r_star == pytest.approx(35.0)
    assert evaluation.evaluation.best_score.d_i == pytest.approx(-92.5)


def test_stage2_adapter_uses_real_boundary_vehicle_effective_controllability() -> None:
    state, _ = build_initial_onestep_stage2_state(RM_ONESTEP_S07_PLAN_STEP0_SCENARIO_ID)
    vehicle_specs = dict(state.vehicle_specs)
    for vehicle_id in ("S07_L2_04", "S07_L2_05"):
        vehicle_specs[vehicle_id] = replace(
            vehicle_specs[vehicle_id],
            vehicle_type="HDV",
        )
    state = replace(state, vehicle_specs=vehicle_specs)

    evaluation = evaluate_stage2_one_step(state, RM_ONESTEP_S07_MV_ID).evaluation
    rows = {row.gap_id: row for row in evaluation.gap_rows}

    assert rows["gap3"].rear_vehicle_id == "S07_L2_03"
    assert rows["gap3"].front_vehicle_id == "S07_L2_04"
    assert rows["gap3"].controllability_branch == CONTROLLABILITY_BRANCH_C
    assert rows["gap3"].front_controllable is False
    assert rows["gap3"].rear_controllable is True
    assert rows["gap3"].J == FAILED_GAP_SCORE
    assert rows["gap3"].failure_reason == "insufficient_single_side_capacity"

    assert rows["gap4"].rear_vehicle_id == "S07_L2_04"
    assert rows["gap4"].front_vehicle_id == "S07_L2_05"
    assert rows["gap4"].controllability_branch == CONTROLLABILITY_BRANCH_D
    assert rows["gap4"].front_controllable is False
    assert rows["gap4"].rear_controllable is False
    assert rows["gap4"].J == FAILED_GAP_SCORE
    assert rows["gap4"].failure_reason == "none_controllable_gap_not_over_95m"

    assert rows["gap5"].rear_vehicle_id == "S07_L2_05"
    assert rows["gap5"].front_vehicle_id == "S07_L2_06"
    assert rows["gap5"].controllability_branch == CONTROLLABILITY_BRANCH_B
    assert rows["gap5"].front_controllable is True
    assert rows["gap5"].rear_controllable is False
    assert rows["gap5"].delta_f_bar == pytest.approx(15.0)


def test_stage2_adapter_from_snapshot_shares_gap_identity_across_two_mvs_and_filters_allowed_gaps() -> None:
    from cormc.scenes import (
        RM_ONESTEP_S07_2MV_REAR_MV_ID,
        RM_ONESTEP_S07_2MV_ROLLING_ENTRY_SCENARIO_ID,
    )

    state, _ = build_initial_onestep_stage2_state(
        RM_ONESTEP_S07_2MV_ROLLING_ENTRY_SCENARIO_ID
    )
    snapshot = identify_and_number_gaps(state, ())
    front_eval = evaluate_stage2_one_step_from_snapshot(
        state,
        RM_ONESTEP_S07_MV_ID,
        snapshot,
    )
    rear_eval = evaluate_stage2_one_step_from_snapshot(
        state,
        RM_ONESTEP_S07_2MV_REAR_MV_ID,
        snapshot,
        uncontrollable_vehicle_ids=("S07_L2_04", "S07_L2_05"),
        allowed_gap_indices=(4, 5, 6),
    )

    assert front_eval.local_frame["snapshot_gap_vehicle_ids_by_index"] == rear_eval.local_frame["snapshot_gap_vehicle_ids_by_index"]
    assert front_eval.local_frame["snapshot_lane_2_vehicle_order"] == rear_eval.local_frame["snapshot_lane_2_vehicle_order"]
    assert front_eval.local_frame["snapshot_lane_2_vehicle_x_local_by_id"] != rear_eval.local_frame["snapshot_lane_2_vehicle_x_local_by_id"]
    assert rear_eval.local_frame["runtime_gap_index_by_kernel_index"] == {0: 6, 1: 5, 2: 4}
    assert tuple(row.gap_index for row in rear_eval.evaluation.gap_rows) == (0, 1, 2)
    rows_by_runtime_index = {
        rear_eval.local_frame["runtime_gap_index_by_kernel_index"][row.gap_index]: row
        for row in rear_eval.evaluation.gap_rows
    }
    assert rows_by_runtime_index[4].front_vehicle_id == "S07_L2_04"
    assert rows_by_runtime_index[4].front_controllable is False
    assert rows_by_runtime_index[4].controllability_branch == CONTROLLABILITY_BRANCH_C
