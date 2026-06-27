from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from cormc.onestep.lab.reference_case import get_reference_expected
from cormc.scenes import (
    RM_MULTIMV_M4_S03_SCENARIO_ID,
    RM_ONESTEP_S05_PLAN_STEP0_SCENARIO_ID,
    RM_ONESTEP_S05_ROLLING_ENTRY_SCENARIO_ID,
    RM_ONESTEP_S07_PLAN_STEP0_SCENARIO_ID,
    RM_ONESTEP_S07_ROLLING_ENTRY_SCENARIO_ID,
)
from cormc.onestep.rolling import run_onestep_stage2_summary
from cormc.onestep.kernel.models import CONTROLLABILITY_BRANCH_A


def test_stage2_runner_s07_plan_step0_first_trigger_matches_reference_case() -> None:
    summary = run_onestep_stage2_summary(
        RM_ONESTEP_S07_PLAN_STEP0_SCENARIO_ID,
        max_steps=5,
        run_id="stage2-plan-step0-test",
    )
    expected = get_reference_expected()
    first_trigger = _first_plan(summary)
    first_gap_rows = _first_plan_gap_rows(summary)

    assert first_trigger["step"] == 0
    assert first_trigger["mv_x_global"] == pytest.approx(6650.0)
    assert first_trigger["best_gap_id"] == expected.best_gap_id
    assert first_trigger["gap_index"] == 3
    assert first_trigger["kernel_gap_index"] == 3
    assert tuple(first_trigger["best_gap_interval_local"]) == pytest.approx(expected.best_gap_interval)
    assert first_trigger["delta_f_star"] == pytest.approx(expected.best_delta_f_star)
    assert first_trigger["delta_r_star"] == pytest.approx(expected.best_delta_r_star)
    assert first_trigger["d_i"] == pytest.approx(expected.best_d_i)
    assert first_trigger["t_m"] == pytest.approx(expected.best_t_m, rel=1e-4)
    assert first_trigger["p_m_local"] == pytest.approx(expected.best_p_m, rel=1e-4)
    assert all(
        row["controllability_branch"] == CONTROLLABILITY_BRANCH_A
        for row in first_gap_rows
        if row["included_in_scoring"]
    )
    assert _bundle_created_events(summary)
    first_created = _bundle_created_events(summary)[0]["payload"]
    assert first_created["bundle_action"] == "bundle_created"
    assert first_created["bundle_id"] == first_trigger["bundle_id"]
    assert first_created["bundle_shape"] == first_trigger["bundle_shape"]
    assert first_created["controlled_vehicle_ids"] == first_trigger["controlled_vehicle_ids"]
    assert first_created["gap_boundary_vehicle_ids"] == [
        first_trigger["selected_rear_vehicle_id"],
        first_trigger["selected_front_vehicle_id"],
    ]


def test_stage2_runner_s05_plan_step0_first_trigger_matches_expected_case() -> None:
    summary = run_onestep_stage2_summary(
        RM_ONESTEP_S05_PLAN_STEP0_SCENARIO_ID,
        max_steps=5,
        run_id="stage2-s05-plan-step0-test",
    )
    first_trigger = _first_plan(summary)

    assert first_trigger["step"] == 0
    assert first_trigger["mv_x_global"] == pytest.approx(6650.0)
    assert first_trigger["best_gap_id"] == "gap1"
    assert first_trigger["gap_index"] == 6
    assert first_trigger["kernel_gap_index"] == 0
    assert tuple(first_trigger["best_gap_interval_local"]) == pytest.approx((-100.0, -50.0))
    assert first_trigger["delta_f_star"] == pytest.approx(0.0)
    assert first_trigger["delta_r_star"] == pytest.approx(35.0)
    assert first_trigger["d_i"] == pytest.approx(-92.5)


def test_stage2_runner_s07_rolling_entry_first_trigger_is_at_6650_and_completes_longitudinally() -> None:
    summary = run_onestep_stage2_summary(
        RM_ONESTEP_S07_ROLLING_ENTRY_SCENARIO_ID,
        max_steps=420,
        run_id="stage2-rolling-entry-test",
    )
    expected = get_reference_expected()
    first_trigger = _first_plan(summary)

    assert first_trigger["mv_x_global"] == pytest.approx(6650.0)
    assert first_trigger["best_gap_id"] == expected.best_gap_id
    assert first_trigger["delta_f_star"] == pytest.approx(expected.best_delta_f_star)
    assert _all_plans(summary)
    completion = _lateral_start(summary)
    assert completion is not None
    assert _bundle_released_events(summary)
    release = dict(_bundle_released_events(summary)[-1]["payload"])
    assert float(completion["front_gap_m"]) >= 42.5 - 1e-6
    assert float(completion["rear_gap_m"]) >= 42.5 - 1e-6
    assert completion["required_longitudinal_gap_m"] == pytest.approx(42.5)
    assert completion["merge_point_reached"] is True
    assert release["bundle_id"] == completion["bundle_id"]
    assert release["bundle_action"] == "bundle_released"
    assert release["bundle_id"] not in summary["cross_mv_summary"]["final_runtime_leftovers"]["active_bundle_ids"]


def test_stage2_runner_s05_rolling_entry_first_trigger_is_at_6650_and_filters_dense_gaps() -> None:
    summary = run_onestep_stage2_summary(
        RM_ONESTEP_S05_ROLLING_ENTRY_SCENARIO_ID,
        max_steps=160,
        run_id="stage2-s05-rolling-entry-test",
    )
    first_trigger = _first_plan(summary)
    first_gap_rows = _first_plan_gap_rows(summary)
    gap_rows = {row["gap_id"]: row for row in first_gap_rows}

    assert first_trigger["step"] == 25
    assert first_trigger["t"] == pytest.approx(2.5)
    assert first_trigger["mv_x_global"] == pytest.approx(6650.0)
    assert first_trigger["best_gap_id"] == "gap1"
    assert first_trigger["delta_f_star"] == pytest.approx(0.0)
    assert first_trigger["delta_r_star"] == pytest.approx(35.0)
    assert first_trigger["d_i"] == pytest.approx(-92.5)
    assert _all_plans(summary)
    assert gap_rows["gap2"]["coop_feasible"] is False
    assert gap_rows["gap2"]["included_in_scoring"] is False
    assert gap_rows["gap3"]["coop_feasible"] is False
    assert gap_rows["gap3"]["included_in_scoring"] is False
    assert gap_rows["gap4"]["coop_feasible"] is False
    assert gap_rows["gap4"]["included_in_scoring"] is False
    for row in first_gap_rows:
        assert "front_vehicle_id" in row
        assert "rear_vehicle_id" in row
        assert "front_controllable" in row
        assert "rear_controllable" in row
        assert "controllability_branch" in row
        assert "failure_reason" in row
        assert "is_selected" in row
        if row["included_in_scoring"]:
            assert row["controllability_branch"] == CONTROLLABILITY_BRANCH_A


def test_stage2_runner_rm_m4_s03_reports_round_level_planning_timing() -> None:
    summary = run_onestep_stage2_summary(
        RM_MULTIMV_M4_S03_SCENARIO_ID,
        max_steps=320,
        run_id="timing-summary-test",
    )
    timing_summary = summary["scenario_summary"]["planning_timing_summary"]

    assert timing_summary["clock"] == "time.perf_counter_ns"
    for planned_mv_count in ("1", "2", "3", "4"):
        assert (
            timing_summary["by_planned_mv_count"][planned_mv_count]["sample_count"]
            > 0
        )
    assert (
        timing_summary["by_controlled_vehicle_count"]["3"]["sample_count"]
        > 0
    )
    assert (
        timing_summary["by_controlled_vehicle_count"]["4"]["sample_count"]
        > 0
    )

    rounds = {round_summary["round_id"]: round_summary for round_summary in summary["round_summaries"]}
    assert rounds["trigger_round:207"]["planning_timing"]["controlled_vehicle_count"] == 4

    timed_round_count = 0
    for round_summary in summary["round_summaries"]:
        timing = round_summary["planning_timing"]
        if timing is None:
            continue
        timed_round_count += 1
        assert timing["duration_ns"] > 0
        assert timing["duration_ms"] > 0
        assert timing["planned_mv_count"] == len(round_summary["plan_summaries"])
        controlled_vehicle_ids = _ordered_unique(
            vehicle_id
            for plan in round_summary["plan_summaries"]
            for vehicle_id in plan["controlled_vehicle_ids"]
        )
        assert timing["controlled_vehicle_ids"] == controlled_vehicle_ids
        assert timing["controlled_vehicle_count"] == len(controlled_vehicle_ids)

    assert timed_round_count == timing_summary["timed_round_count"]


def _all_plans(summary):
    return [
        plan
        for round_summary in summary["round_summaries"]
        for plan in round_summary["plan_summaries"]
    ]


def _first_plan(summary):
    return _all_plans(summary)[0]


def _first_plan_gap_rows(summary):
    first = _first_plan(summary)
    return [
        row
        for row in summary["mv_summaries"][first["mv_id"]]["gap_rows"]
        if row["bundle_id"] == first["bundle_id"]
    ]


def _formal_events(summary, kind):
    return [
        event
        for event in summary["scenario_summary"]["formal_events"]
        if event["event_kind"] == kind
    ]


def _bundle_created_events(summary):
    return _formal_events(summary, "bundle_created")


def _bundle_released_events(summary):
    return _formal_events(summary, "bundle_released")


def _lateral_start(summary):
    events = _formal_events(summary, "lateral_started")
    return events[0]["payload"] if events else None


def _ordered_unique(values):
    return list(dict.fromkeys(values))
