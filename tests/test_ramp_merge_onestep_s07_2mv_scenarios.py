from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from cormc.scenes import RM_ONESTEP_S07_2MV_ROLLING_ENTRY_SCENARIO_ID as PUBLIC_2MV_ID
from cormc.scenes import (
    RM_ONESTEP_S07_2MV_MV_IDS,
    RM_ONESTEP_S07_2MV_REAR_MV_ID,
    RM_ONESTEP_S07_2MV_ROLLING_ENTRY_SCENARIO_ID,
    RM_ONESTEP_S07_2MV_SCENARIO_IDS,
    RM_ONESTEP_S07_MV_ID,
    RM_ONESTEP_S07_ROLLING_ENTRY_SCENARIO_ID,
    RM_ONESTEP_S07_SCENARIO_IDS,
    RM_ONESTEP_SCENARIO_IDS,
    RM_ONESTEP_SCENARIO_TO_CASE_ID,
    RM_ONESTEP_STAGE2_DEFAULT_MAX_STEPS,
    load_scene_config,
)
from cormc.onestep.rolling import (
    build_onestep_stage2_acceptance_report,
    build_initial_onestep_stage2_state,
    evaluate_stage2_one_step,
    identify_and_number_gaps,
    run_onestep_stage2_history,
    run_onestep_stage2_summary,
)
from cormc.onestep.kernel.models import CONTROLLABILITY_BRANCH_A
from cormc.onestep.kernel.models import CONTROLLABILITY_BRANCH_C
from cormc.simulation_core.pre_freeze import LANE_2


def test_s07_2mv_catalog_exposes_formal_scenario_id() -> None:
    scenarios = {scenario_id: load_scene_config(scenario_id) for scenario_id in RM_ONESTEP_SCENARIO_IDS}

    assert PUBLIC_2MV_ID == RM_ONESTEP_S07_2MV_ROLLING_ENTRY_SCENARIO_ID
    assert RM_ONESTEP_S07_2MV_SCENARIO_IDS == (
        RM_ONESTEP_S07_2MV_ROLLING_ENTRY_SCENARIO_ID,
    )
    assert RM_ONESTEP_S07_2MV_ROLLING_ENTRY_SCENARIO_ID in RM_ONESTEP_S07_SCENARIO_IDS
    assert RM_ONESTEP_S07_2MV_ROLLING_ENTRY_SCENARIO_ID in RM_ONESTEP_SCENARIO_IDS
    assert RM_ONESTEP_S07_2MV_ROLLING_ENTRY_SCENARIO_ID in scenarios
    assert (
        RM_ONESTEP_SCENARIO_TO_CASE_ID[RM_ONESTEP_S07_2MV_ROLLING_ENTRY_SCENARIO_ID]
        == "S07"
    )
    assert (
        RM_ONESTEP_STAGE2_DEFAULT_MAX_STEPS[
            RM_ONESTEP_S07_2MV_ROLLING_ENTRY_SCENARIO_ID
        ]
        == 420
    )


def test_s07_2mv_rolling_config_places_two_plain_on_ramp_mvs() -> None:
    config = load_scene_config(RM_ONESTEP_S07_2MV_ROLLING_ENTRY_SCENARIO_ID)
    single_mv_config = load_scene_config(RM_ONESTEP_S07_ROLLING_ENTRY_SCENARIO_ID)

    vehicles = {vehicle["vehicle_id"]: vehicle for vehicle in config["initial_vehicles"]}
    single_mv_vehicles = {
        vehicle["vehicle_id"]: vehicle for vehicle in single_mv_config["initial_vehicles"]
    }
    lane2_ids = tuple(
        vehicle["vehicle_id"]
        for vehicle in config["initial_vehicles"]
        if vehicle["physical_lane"] == "lane_2"
    )
    single_mv_lane2_ids = tuple(
        vehicle["vehicle_id"]
        for vehicle in single_mv_config["initial_vehicles"]
        if vehicle["physical_lane"] == "lane_2"
    )

    front_mv = vehicles[RM_ONESTEP_S07_MV_ID]
    rear_mv = vehicles[RM_ONESTEP_S07_2MV_REAR_MV_ID]
    assert RM_ONESTEP_S07_2MV_MV_IDS == (
        RM_ONESTEP_S07_MV_ID,
        RM_ONESTEP_S07_2MV_REAR_MV_ID,
    )
    assert front_mv["initial_x_global"] == pytest.approx(6600.0)
    assert rear_mv["initial_x_global"] == pytest.approx(6540.0)
    assert front_mv["initial_x_global"] - rear_mv["initial_x_global"] == pytest.approx(60.0)
    for mv in (front_mv, rear_mv):
        assert mv["physical_lane"] == "on_ramp"
        assert mv["road_role"] == "on_ramp_mv"

    assert lane2_ids == single_mv_lane2_ids
    for vehicle_id in lane2_ids:
        assert vehicles[vehicle_id]["initial_x_global"] == pytest.approx(
            single_mv_vehicles[vehicle_id]["initial_x_global"]
        )
    vehicle_ids = tuple(vehicle["vehicle_id"] for vehicle in config["initial_vehicles"])
    assert len(vehicle_ids) == len(set(vehicle_ids))


def test_s07_2mv_build_initial_stage2_state_registers_two_mvs() -> None:
    state, _ = build_initial_onestep_stage2_state(RM_ONESTEP_S07_2MV_ROLLING_ENTRY_SCENARIO_ID)
    runtime = state.ramp_merge_runtime
    assert runtime is not None

    assert tuple(runtime.mv_plan_states) == (
        RM_ONESTEP_S07_MV_ID,
        RM_ONESTEP_S07_2MV_REAR_MV_ID,
    )
    for mv_id in (RM_ONESTEP_S07_MV_ID, RM_ONESTEP_S07_2MV_REAR_MV_ID):
        assert runtime.mv_plan_states[mv_id].zone_state == "outside_control_zone"
    assert state.vehicle_states[RM_ONESTEP_S07_MV_ID].x_global == pytest.approx(6600.0)
    assert state.vehicle_states[RM_ONESTEP_S07_2MV_REAR_MV_ID].x_global == pytest.approx(
        6540.0
    )


def test_s07_2mv_initial_gap_snapshot_observes_lane2_ids_without_on_ramp_mvs() -> None:
    state, _ = build_initial_onestep_stage2_state(RM_ONESTEP_S07_2MV_ROLLING_ENTRY_SCENARIO_ID)

    snapshot = identify_and_number_gaps(state, danger_vehicle_ids=())

    assert snapshot.gaps
    assert tuple(gap.index for gap in snapshot.gaps) == tuple(
        range(1, len(snapshot.gaps) + 1)
    )
    assert all(gap.front_vehicle_id is not None for gap in snapshot.gaps)
    assert all(gap.rear_vehicle_id is not None for gap in snapshot.gaps)
    assert all(gap.rear_x_global < gap.front_x_global for gap in snapshot.gaps)
    assert tuple(gap.rear_x_global for gap in snapshot.gaps) == tuple(
        sorted((gap.rear_x_global for gap in snapshot.gaps), reverse=True)
    )
    gap_boundary_ids = {
        vehicle_id
        for gap in snapshot.gaps
        for vehicle_id in (gap.front_vehicle_id, gap.rear_vehicle_id)
    }
    assert not set(RM_ONESTEP_S07_2MV_MV_IDS) & gap_boundary_ids
    assert all(
        state.vehicle_states[vehicle_id].physical_lane == LANE_2
        for vehicle_id in gap_boundary_ids
    )


def test_s07_2mv_each_mv_can_be_observed_through_stage2_adapter_kernel_rows() -> None:
    state, _ = build_initial_onestep_stage2_state(RM_ONESTEP_S07_2MV_ROLLING_ENTRY_SCENARIO_ID)

    for mv_id in (RM_ONESTEP_S07_MV_ID, RM_ONESTEP_S07_2MV_REAR_MV_ID):
        result = evaluate_stage2_one_step(state, mv_id)
        gap_vehicle_ids_by_index = dict(result.local_frame["gap_vehicle_ids_by_index"])

        assert result.evaluation.gap_rows
        for row in result.evaluation.gap_rows:
            expected_rear_id, expected_front_id = gap_vehicle_ids_by_index[row.gap_index]
            assert row.rear_vehicle_id == expected_rear_id
            assert row.front_vehicle_id == expected_front_id
            assert row.controllability_branch
            if row.reachable or row.included_in_scoring:
                assert row.controllability_branch == CONTROLLABILITY_BRANCH_A


def test_s07_2mv_stage2_runner_short_entry_run_no_longer_raises_single_mv_limit() -> None:
    summary = run_onestep_stage2_summary(
        RM_ONESTEP_S07_2MV_ROLLING_ENTRY_SCENARIO_ID,
        max_steps=5,
        run_id="stage2-2mv-short-entry",
    )

    assert set(summary) == {
        "scenario_summary",
        "round_summaries",
        "mv_summaries",
        "cross_mv_summary",
        "artifact_paths",
    }
    assert summary["scenario_summary"]["scenario_id"] == (
        RM_ONESTEP_S07_2MV_ROLLING_ENTRY_SCENARIO_ID
    )
    assert summary["scenario_summary"]["runtime_version"] == "onestep_stage2_v1"
    assert summary["round_summaries"] == []


def test_s07_2mv_stage2_runner_observes_formal_multi_mv_trigger_round() -> None:
    summary = run_onestep_stage2_summary(
        RM_ONESTEP_S07_2MV_ROLLING_ENTRY_SCENARIO_ID,
        max_steps=56,
        run_id="stage2-2mv-round",
    )
    round_summary = next(
        item for item in summary["round_summaries"] if len(item["plan_summaries"]) == 2
    )
    round_records = round_summary["plan_summaries"]

    assert [record["mv_id"] for record in round_records] == [
        RM_ONESTEP_S07_MV_ID,
        RM_ONESTEP_S07_2MV_REAR_MV_ID,
    ]
    assert [record["round_order"] for record in round_records] == [0, 1]
    first, second = round_records
    assert second["gap_index"] > first["gap_index"]
    assert len({record["gap_index"] for record in round_records}) == 2
    assert second["tail_frontier_gap_index_before"] == first["gap_index"]
    assert set(first["controlled_vehicle_ids"]).isdisjoint(
        set(second["controlled_vehicle_ids"])
    )
    assert set(second["filtered_by_frontier_gap_indices"]) >= {
        index for index in range(1, int(first["gap_index"]) + 1)
    }

    rear_rows = [
        row
        for row in round_summary["gap_rows"]
        if row["round_id"] == second["round_id"]
        and row["mv_id"] == RM_ONESTEP_S07_2MV_REAR_MV_ID
    ]
    assert {row["gap_index"] for row in rear_rows}.isdisjoint(
        set(second["filtered_by_frontier_gap_indices"])
    )
    shared_boundary_rows = [
        row
        for row in rear_rows
        if row["front_vehicle_id"] == first["selected_rear_vehicle_id"]
        or row["rear_vehicle_id"] == first["selected_front_vehicle_id"]
    ]
    assert any(
        row["controllability_branch"] == CONTROLLABILITY_BRANCH_C
        for row in shared_boundary_rows
    )
    round_creates = [
        event
        for event in summary["scenario_summary"]["formal_events"]
        if event["event_kind"] == "bundle_created"
        and event["round_id"] == first["round_id"]
    ]
    assert [record["mv_id"] for record in round_creates] == [
        RM_ONESTEP_S07_MV_ID,
        RM_ONESTEP_S07_2MV_REAR_MV_ID,
    ]
    controlled_sets = [
        set(record["payload"]["controlled_vehicle_ids"])
        for record in round_creates
    ]
    assert controlled_sets[0].isdisjoint(controlled_sets[1])


def test_2mv_s07_stage2_lifecycle_closes_for_both_mvs() -> None:
    result = run_onestep_stage2_history(
        RM_ONESTEP_S07_2MV_ROLLING_ENTRY_SCENARIO_ID,
        max_steps=700,
        run_id="stage6-2mv-lifecycle",
    )
    summary = result.summary
    assert set(summary) == {
        "scenario_summary",
        "round_summaries",
        "mv_summaries",
        "cross_mv_summary",
        "artifact_paths",
    }
    assert summary["scenario_summary"]["scenario_id"] == (
        RM_ONESTEP_S07_2MV_ROLLING_ENTRY_SCENARIO_ID
    )
    assert tuple(summary["scenario_summary"]["mv_ids"]) == RM_ONESTEP_S07_2MV_MV_IDS
    lifecycle = {
        mv_id: mv_summary["lifecycle"]
        for mv_id, mv_summary in summary["mv_summaries"].items()
    }

    assert set(lifecycle) >= set(RM_ONESTEP_S07_2MV_MV_IDS)
    locked_gap_indices = []
    lateral_start_steps = []
    for mv_id in RM_ONESTEP_S07_2MV_MV_IDS:
        item = lifecycle[mv_id]
        assert item["locked_gap_step"] is not None
        assert item["lateral_start_step"] is not None
        assert summary["mv_summaries"][mv_id]["bundles"]
        assert item["lateral_completed_step"] is not None
        assert item["mainline_conversion_step"] is not None
        assert (
            item["locked_gap_step"]
            <= item["lateral_start_step"]
            < item["lateral_completed_step"]
            < item["mainline_conversion_step"]
        )
        final_status = item["final_status"]
        assert final_status["physical_lane"] == "lane_2"
        assert final_status["road_role"] == "mainline"
        assert final_status["merge_state"] == "normal"
        assert final_status["runtime_present"] is False
        assert any(row["is_selected"] is True for row in summary["mv_summaries"][mv_id]["gap_rows"])
        locked_gap_indices.append(summary["mv_summaries"][mv_id]["locked_gap"]["index"])
        lateral_start_steps.append(item["lateral_start_step"])

    assert locked_gap_indices[0] < locked_gap_indices[1]
    assert len(set(locked_gap_indices)) == len(locked_gap_indices)
    assert lateral_start_steps[0] <= lateral_start_steps[1]
    cross = summary["cross_mv_summary"]
    front_lock_round = next(
        item
        for item in summary["round_summaries"]
        if RM_ONESTEP_S07_MV_ID in item["mv_order"]
        and item["locked_gap_indices"]
    )
    same_round_rear_plan = next(
        record
        for record in front_lock_round["plan_summaries"]
        if record["mv_id"] == RM_ONESTEP_S07_2MV_REAR_MV_ID
    )
    assert front_lock_round["locked_gap_indices"] == [locked_gap_indices[0]]
    assert same_round_rear_plan["tail_frontier_gap_index_before"] == locked_gap_indices[0]
    assert same_round_rear_plan["gap_index"] > locked_gap_indices[0]
    assert cross["gap_conflicts"] == []
    assert cross["frontier_violations"] == []
    assert cross["ownership_conflicts"] == []
    assert cross["final_runtime_leftovers"] == {
        "mv_plan_state_ids": [],
        "active_bundle_ids": [],
        "gap_plan_ids": [],
        "lateral_trajectory_ids": [],
        "controlled_vehicle_states": {},
    }
    assert not any(
        record["mv_id"] in RM_ONESTEP_S07_2MV_MV_IDS
        for round_summary in summary["round_summaries"]
        for record in round_summary["plan_summaries"]
        if record["step"] >= lifecycle[RM_ONESTEP_S07_MV_ID]["lateral_start_step"]
        and record["mv_id"] == RM_ONESTEP_S07_MV_ID
    )
    report = build_onestep_stage2_acceptance_report(summary)
    assert report.passed is True
    assert report.issues == ()
