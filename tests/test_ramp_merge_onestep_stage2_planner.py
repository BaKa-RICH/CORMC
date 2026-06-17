from __future__ import annotations

from dataclasses import replace
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from cormc.onestep.rolling import (
    BUNDLE_SHAPE_MV_FRONT,
    BUNDLE_SHAPE_MV_FRONT_REAR,
    BUNDLE_SHAPE_MV_ONLY,
    BUNDLE_SHAPE_MV_REAR,
    build_initial_onestep_stage2_state,
    decide_trigger_plan,
    identify_and_number_gaps,
    plan_stage2_for_trigger,
)
from cormc.scenes import (
    RM_ONESTEP_S05_MV_ID,
    RM_ONESTEP_S05_PLAN_STEP0_SCENARIO_ID,
    RM_ONESTEP_S07_MV_ID,
    RM_ONESTEP_S07_2MV_REAR_MV_ID,
    RM_ONESTEP_S07_2MV_ROLLING_ENTRY_SCENARIO_ID,
    RM_ONESTEP_S07_PLAN_STEP0_SCENARIO_ID,
)
from cormc.onestep.kernel.models import CONTROLLABILITY_BRANCH_A
from cormc.onestep.rolling.stage2_planner import (
    _RoundContext,
    _advance_round_context,
    _candidate_gap_indices_after_frontier,
)


@pytest.mark.parametrize(
    ("scenario_id", "mv_id", "expected_bundle_members"),
    [
        (
            RM_ONESTEP_S05_PLAN_STEP0_SCENARIO_ID,
            RM_ONESTEP_S05_MV_ID,
            {"S05_MV", "S05_L2_01", "S05_L2_02"},
        ),
        (
            RM_ONESTEP_S07_PLAN_STEP0_SCENARIO_ID,
            RM_ONESTEP_S07_MV_ID,
            {"S07_MV", "S07_L2_04", "S07_L2_05"},
        ),
    ],
)
def test_stage2_planner_creates_single_bundle_and_three_controlled_vehicle_states(
    scenario_id: str,
    mv_id: str,
    expected_bundle_members: set[str],
) -> None:
    state, _ = build_initial_onestep_stage2_state(scenario_id)
    runtime = state.ramp_merge_runtime
    trigger = decide_trigger_plan(
        state,
        runtime.planner_state,
        entry_plan_trigger=False,
        safety_alert=False,
        entry_vehicle_ids=(),
    )
    snapshot = identify_and_number_gaps(state, ())

    result = plan_stage2_for_trigger(state, runtime, snapshot, trigger)
    runtime = result.runtime
    mv_state = runtime.mv_plan_states[mv_id]

    assert len(runtime.onestep_plan_bundles) == 1
    assert len(runtime.controlled_vehicle_states) == 3
    assert mv_state.current_plan_gap is not None
    assert mv_state.active_bundle_id is not None
    bundle = runtime.onestep_plan_bundles[mv_state.active_bundle_id]
    assert bundle.bundle_shape == BUNDLE_SHAPE_MV_FRONT_REAR
    assert set(bundle.controlled_vehicle_ids) == expected_bundle_members
    assert set(bundle.controlled_roles_by_vehicle_id) == expected_bundle_members
    assert set(bundle.boundary_state_by_vehicle_id) == expected_bundle_members
    assert bundle.boundary_state_by_vehicle_id[mv_id].x_global == state.vehicle_states[mv_id].x_global
    assert bundle.boundary_state_by_vehicle_id[mv_id].v == state.vehicle_states[mv_id].v
    assert bundle.required_longitudinal_gap_m == pytest.approx(42.5)
    assert result.gap_selection_records
    assert result.trajectory_records
    selected_rows = [
        record for record in result.trajectory_records if record["is_selected"]
    ]
    assert len(selected_rows) == 1
    assert selected_rows[0]["kernel_gap_index"] == bundle.best_gap.index
    assert selected_rows[0]["gap_index"] == bundle.selected_gap.index
    for record in result.trajectory_records:
        assert "front_vehicle_id" in record
        assert "rear_vehicle_id" in record
        assert "front_controllable" in record
        assert "rear_controllable" in record
        assert "controllability_branch" in record
        assert "failure_reason" in record
        assert "is_selected" in record
        if record["included_in_scoring"]:
            assert record["controllability_branch"] == CONTROLLABILITY_BRANCH_A


@pytest.mark.parametrize(
    (
        "scenario_id",
        "mv_id",
        "hdv_vehicle_ids",
        "position_updates",
        "expected_shape",
        "expected_controlled_ids",
        "expected_boundary_ids",
    ),
    [
        (
            RM_ONESTEP_S07_PLAN_STEP0_SCENARIO_ID,
            RM_ONESTEP_S07_MV_ID,
            (),
            {},
            BUNDLE_SHAPE_MV_FRONT_REAR,
            ("S07_MV", "S07_L2_04", "S07_L2_05"),
            ("S07_L2_04", "S07_L2_05"),
        ),
        (
            RM_ONESTEP_S07_PLAN_STEP0_SCENARIO_ID,
            RM_ONESTEP_S07_MV_ID,
            ("S07_L2_04",),
            {},
            BUNDLE_SHAPE_MV_FRONT,
            ("S07_MV", "S07_L2_05"),
            ("S07_L2_04", "S07_L2_05"),
        ),
        (
            RM_ONESTEP_S05_PLAN_STEP0_SCENARIO_ID,
            RM_ONESTEP_S05_MV_ID,
            ("S05_L2_02",),
            {},
            BUNDLE_SHAPE_MV_REAR,
            ("S05_MV", "S05_L2_01"),
            ("S05_L2_01", "S05_L2_02"),
        ),
        (
            RM_ONESTEP_S07_PLAN_STEP0_SCENARIO_ID,
            RM_ONESTEP_S07_MV_ID,
            ("S07_L2_04", "S07_L2_05"),
            {"S07_L2_04": 6660.0, "S07_L2_05": 6790.0},
            BUNDLE_SHAPE_MV_ONLY,
            ("S07_MV",),
            ("S07_L2_04", "S07_L2_05"),
        ),
    ],
)
def test_stage2_planner_creates_bundle_shape_from_selected_controllability_branch(
    scenario_id: str,
    mv_id: str,
    hdv_vehicle_ids: tuple[str, ...],
    position_updates: dict[str, float],
    expected_shape: str,
    expected_controlled_ids: tuple[str, ...],
    expected_boundary_ids: tuple[str, str],
) -> None:
    state, _ = build_initial_onestep_stage2_state(scenario_id)
    state = _with_vehicle_x_updates(state, position_updates)
    state = _with_hdv_specs(state, hdv_vehicle_ids)
    result = _plan_once(state)
    runtime = result.runtime
    mv_state = runtime.mv_plan_states[mv_id]
    assert mv_state.active_bundle_id is not None

    bundle = runtime.onestep_plan_bundles[mv_state.active_bundle_id]
    rear_id, front_id = expected_boundary_ids

    assert bundle.bundle_shape == expected_shape
    assert bundle.controlled_vehicle_ids == expected_controlled_ids
    assert bundle.selected_vehicle_ids == expected_controlled_ids
    assert bundle.selected_rear_vehicle_id == rear_id
    assert bundle.selected_front_vehicle_id == front_id
    assert bundle.selected_gap.rear_vehicle_id == rear_id
    assert bundle.selected_gap.front_vehicle_id == front_id
    assert set(bundle.boundary_state_by_vehicle_id) == set(expected_controlled_ids)
    assert set(runtime.controlled_vehicle_states) == set(expected_controlled_ids)
    for vehicle_id in expected_boundary_ids:
        controlled = runtime.controlled_vehicle_states.get(vehicle_id)
        if vehicle_id in expected_controlled_ids:
            assert controlled is not None
            assert controlled.owner_mv_id == mv_id
            assert controlled.bundle_id == bundle.bundle_id
        else:
            assert controlled is None
    created = [
        record
        for record in result.bundle_lifecycle_records
        if record["bundle_action"] == "bundle_created"
    ]
    assert len(created) == 1
    assert created[0]["bundle_shape"] == expected_shape
    assert created[0]["controlled_vehicle_ids"] == list(expected_controlled_ids)
    assert created[0]["gap_boundary_vehicle_ids"] == list(expected_boundary_ids)


def test_stage2_planner_replaces_active_bundle_and_records_lifecycle() -> None:
    state, _ = build_initial_onestep_stage2_state(RM_ONESTEP_S07_PLAN_STEP0_SCENARIO_ID)
    first = _plan_once(state)
    first_runtime = first.runtime
    first_mv_state = first_runtime.mv_plan_states[RM_ONESTEP_S07_MV_ID]
    assert first_mv_state.active_bundle_id is not None
    old_bundle_id = first_mv_state.active_bundle_id
    old_plan_id = first_mv_state.current_plan_id

    replan_state = replace(
        state,
        step=state.step + 1,
        t=state.t + state.dt,
        ramp_merge_runtime=first_runtime,
    )
    second = _plan_once(replan_state)
    second_runtime = second.runtime
    second_mv_state = second_runtime.mv_plan_states[RM_ONESTEP_S07_MV_ID]
    assert second_mv_state.active_bundle_id is not None
    new_bundle_id = second_mv_state.active_bundle_id

    assert new_bundle_id != old_bundle_id
    assert second_mv_state.current_plan_id != old_plan_id
    assert old_bundle_id not in second_runtime.onestep_plan_bundles
    assert new_bundle_id in second_runtime.onestep_plan_bundles
    assert all(
        controlled.bundle_id != old_bundle_id
        for controlled in second_runtime.controlled_vehicle_states.values()
    )
    assert all(
        plan.bundle_id != old_bundle_id
        for plan in second_runtime.gap_plans.values()
    )
    actions = [record["bundle_action"] for record in second.bundle_lifecycle_records]
    assert actions == ["bundle_released", "bundle_created"]
    release, create = second.bundle_lifecycle_records
    assert release["bundle_id"] == old_bundle_id
    assert release["replaced_bundle_id"] == new_bundle_id
    assert create["bundle_id"] == new_bundle_id
    assert create["replaced_bundle_id"] == old_bundle_id


def test_stage2_planner_releases_active_bundle_when_replan_has_no_gap() -> None:
    state, _ = build_initial_onestep_stage2_state(RM_ONESTEP_S07_PLAN_STEP0_SCENARIO_ID)
    first = _plan_once(state)
    first_runtime = first.runtime
    first_mv_state = first_runtime.mv_plan_states[RM_ONESTEP_S07_MV_ID]
    assert first_mv_state.active_bundle_id is not None
    old_bundle_id = first_mv_state.active_bundle_id

    no_gap_state = replace(
        state,
        step=state.step + 1,
        t=state.t + state.dt,
        ramp_merge_runtime=first_runtime,
    )
    trigger = decide_trigger_plan(
        no_gap_state,
        first_runtime.planner_state,
        entry_plan_trigger=False,
        safety_alert=False,
        entry_vehicle_ids=(),
    )
    empty_snapshot = replace(
        identify_and_number_gaps(no_gap_state, ()),
        gaps=(),
    )

    result = plan_stage2_for_trigger(
        no_gap_state,
        first_runtime,
        empty_snapshot,
        trigger,
    )
    runtime = result.runtime
    mv_state = runtime.mv_plan_states[RM_ONESTEP_S07_MV_ID]

    assert old_bundle_id not in runtime.onestep_plan_bundles
    assert all(
        controlled.bundle_id != old_bundle_id
        for controlled in runtime.controlled_vehicle_states.values()
    )
    assert mv_state.active_bundle_id is None
    assert mv_state.current_plan_id is None
    assert mv_state.current_plan_gap is None
    assert len(result.bundle_lifecycle_records) == 1
    release = result.bundle_lifecycle_records[0]
    assert release["bundle_action"] == "bundle_released"
    assert release["bundle_id"] == old_bundle_id
    assert release["reason"] == "owner_replanned_without_available_gap"


def test_stage2_planner_round_context_filters_after_tail_frontier() -> None:
    state, _ = build_initial_onestep_stage2_state(RM_ONESTEP_S07_PLAN_STEP0_SCENARIO_ID)
    snapshot = identify_and_number_gaps(state, ())

    assert _candidate_gap_indices_after_frontier(snapshot, None) == (1, 2, 3, 4, 5, 6)
    assert _candidate_gap_indices_after_frontier(snapshot, 3) == (4, 5, 6)

    context = _advance_round_context(_RoundContext(), snapshot.gaps[1])
    assert context.selected_gap_indices == (2,)
    assert context.tail_frontier_gap_index == 2
    assert set(context.uncontrollable_vehicle_ids) == {
        snapshot.gaps[1].front_vehicle_id,
        snapshot.gaps[1].rear_vehicle_id,
    }
    context = _advance_round_context(context, snapshot.gaps[3])
    assert context.selected_gap_indices == (2, 4)
    assert context.tail_frontier_gap_index == 4


def test_stage2_planner_processes_two_mvs_in_one_round_with_frontier_propagation() -> None:
    from cormc.onestep.rolling import run_onestep_stage2_summary
    from cormc.onestep.kernel.models import CONTROLLABILITY_BRANCH_C

    summary = run_onestep_stage2_summary(
        RM_ONESTEP_S07_2MV_ROLLING_ENTRY_SCENARIO_ID,
        max_steps=56,
        run_id="stage2-2mv-planner-test",
    )
    round_summary = next(
        item for item in summary["round_summaries"] if item["round_id"] == "trigger_round:55"
    )
    plans = round_summary["plan_summaries"]

    assert [record["mv_id"] for record in plans] == [
        RM_ONESTEP_S07_MV_ID,
        RM_ONESTEP_S07_2MV_REAR_MV_ID,
    ]
    assert [record["round_order"] for record in plans] == [0, 1]
    front_plan, rear_plan = plans
    assert front_plan["gap_index"] == 3
    assert rear_plan["gap_index"] == 5
    assert rear_plan["gap_index"] > front_plan["gap_index"]
    assert rear_plan["tail_frontier_gap_index_before"] == front_plan["gap_index"]
    assert set(rear_plan["filtered_by_frontier_gap_indices"]) == {1, 2, 3}

    rear_rows = [
        row
        for row in round_summary["gap_rows"]
        if row["mv_id"] == RM_ONESTEP_S07_2MV_REAR_MV_ID
    ]
    assert {row["gap_index"] for row in rear_rows} == {4, 5, 6}
    shared_boundary_row = next(row for row in rear_rows if row["gap_index"] == 4)
    assert shared_boundary_row["front_vehicle_id"] == "S07_L2_04"
    assert shared_boundary_row["front_controllable"] is False
    assert shared_boundary_row["controllability_branch"] == CONTROLLABILITY_BRANCH_C


def _plan_once(state):
    runtime = state.ramp_merge_runtime
    trigger = decide_trigger_plan(
        state,
        runtime.planner_state,
        entry_plan_trigger=False,
        safety_alert=False,
        entry_vehicle_ids=(),
    )
    snapshot = identify_and_number_gaps(state, ())
    return plan_stage2_for_trigger(state, runtime, snapshot, trigger)


def _with_hdv_specs(state, vehicle_ids: tuple[str, ...]):
    vehicle_specs = dict(state.vehicle_specs)
    for vehicle_id in vehicle_ids:
        vehicle_specs[vehicle_id] = replace(
            vehicle_specs[vehicle_id],
            vehicle_type="HDV",
            compliance_state="not_applicable",
        )
    return replace(state, vehicle_specs=vehicle_specs)


def _with_vehicle_x_updates(state, updates: dict[str, float]):
    if not updates:
        return state
    vehicle_states = dict(state.vehicle_states)
    for vehicle_id, x_global in updates.items():
        vehicle_states[vehicle_id] = replace(
            vehicle_states[vehicle_id],
            x_global=x_global,
        )
    return replace(state, vehicle_states=vehicle_states)
