from __future__ import annotations

from dataclasses import replace
from pathlib import Path

import pytest

from cormc.legacy import (
    EFFECTIVE_CONTROL_BOTH,
    EFFECTIVE_CONTROL_FRONT,
    EFFECTIVE_CONTROL_NONE,
    EFFECTIVE_CONTROL_REAR,
    TRIGGER_MV_ENTER_CONTROL_ZONE,
    TRIGGER_NONE,
    TRIGGER_PERIODIC,
    TRIGGER_SAFETY_ALERT,
    ZONE_CONTROL,
    ZONE_MERGE,
    ZONE_OUTSIDE_CONTROL,
    ZONE_OUT_OF_SCENE,
    RampMergeEngine,
    build_initial_ramp_merge_state,
    decide_trigger_plan,
    derive_zone_state,
    detect_entry_plan_trigger,
    detect_entry_vehicle_ids,
    effective_controllable,
    identify_and_number_gaps,
    initialize_runtime_state,
    is_base_controllable,
    refresh_runtime_state,
    run_ramp_merge_basic_smoke,
    run_safety_check,
)


def test_zone_state_derivation_and_runtime_initialization() -> None:
    basic_01, _ = build_initial_ramp_merge_state("BASIC-01")
    basic_04, _ = build_initial_ramp_merge_state("BASIC-04")

    assert basic_01.ramp_merge_runtime.mv_plan_states["B01_MV"].zone_state == ZONE_OUTSIDE_CONTROL
    assert basic_04.ramp_merge_runtime.mv_plan_states["B04_MV"].zone_state == ZONE_CONTROL

    mv_state = basic_01.vehicle_states["B01_MV"]
    assert derive_zone_state(replace(mv_state, x_global=6649.9)) == ZONE_OUTSIDE_CONTROL
    assert derive_zone_state(replace(mv_state, x_global=6650.0)) == ZONE_CONTROL
    assert derive_zone_state(replace(mv_state, x_global=6949.9)) == ZONE_CONTROL
    assert derive_zone_state(replace(mv_state, x_global=6950.0)) == ZONE_MERGE
    assert derive_zone_state(replace(mv_state, x_global=7250.0)) == ZONE_MERGE
    assert derive_zone_state(replace(mv_state, x_global=7250.1)) == ZONE_OUT_OF_SCENE


def test_entry_trigger_and_trigger_priority_without_moving_periodic_clock() -> None:
    state, _ = build_initial_ramp_merge_state("BASIC-01")
    previous_runtime = initialize_runtime_state(state)
    entered_state = _with_vehicle_state(
        state,
        "B01_MV",
        x_global=6650.0,
    )
    refreshed_runtime = refresh_runtime_state(previous_runtime, entered_state)

    assert detect_entry_vehicle_ids(previous_runtime, refreshed_runtime) == ("B01_MV",)
    assert detect_entry_plan_trigger(previous_runtime, refreshed_runtime) is True

    periodic = decide_trigger_plan(
        state,
        previous_runtime.planner_state,
        entry_plan_trigger=False,
        safety_alert=False,
    )
    assert periodic.trigger_plan is True
    assert periodic.trigger_reason == TRIGGER_PERIODIC
    assert periodic.planner_state.next_plan_time == pytest.approx(2.0)
    assert periodic.planner_state.last_trigger_reason == TRIGGER_PERIODIC

    no_trigger_state = replace(state, t=0.1)
    no_trigger = decide_trigger_plan(
        no_trigger_state,
        periodic.planner_state,
        entry_plan_trigger=False,
        safety_alert=False,
    )
    assert no_trigger.trigger_plan is False
    assert no_trigger.trigger_reason == TRIGGER_NONE
    assert no_trigger.planner_state.next_plan_time == pytest.approx(2.0)
    assert no_trigger.planner_state.last_trigger_reason == TRIGGER_PERIODIC

    safety_over_entry = decide_trigger_plan(
        no_trigger_state,
        no_trigger.planner_state,
        entry_plan_trigger=True,
        safety_alert=True,
        entry_vehicle_ids=("B01_MV",),
    )
    assert safety_over_entry.trigger_reason == TRIGGER_SAFETY_ALERT
    assert safety_over_entry.planner_state.next_plan_time == pytest.approx(2.0)
    assert safety_over_entry.planner_state.last_trigger_reason == TRIGGER_SAFETY_ALERT
    assert safety_over_entry.active_trigger_reasons == (
        TRIGGER_SAFETY_ALERT,
        TRIGGER_MV_ENTER_CONTROL_ZONE,
    )


def test_safety_check_is_temporary_and_effective_controllable_uses_danger() -> None:
    state, _ = build_initial_ramp_merge_state("BASIC-01")
    risky_state = _with_vehicle_state(
        _with_vehicle_state(state, "B01_CFV", x_global=100.0, v=30.0),
        "B01_CLV",
        x_global=105.0,
        v=20.0,
    )
    before_vehicle_states = dict(risky_state.vehicle_states)

    safety = run_safety_check(risky_state)

    assert set(safety.danger_vehicle_ids) == {"B01_CFV", "B01_CLV"}
    assert safety.safety_alert is True
    assert dict(risky_state.vehicle_states) == before_vehicle_states
    assert not hasattr(risky_state.vehicle_states["B01_CFV"], "safety_state")

    resolved_state = _with_vehicle_state(risky_state, "B01_CLV", x_global=140.0)
    resolved_safety = run_safety_check(resolved_state)
    assert resolved_safety.danger_vehicle_ids == ()
    assert resolved_safety.safety_alert is False

    assert is_base_controllable(risky_state.vehicle_specs["B01_CFV"]) is True
    assert effective_controllable("B01_CFV", risky_state, ()) is True
    assert effective_controllable("B01_CFV", risky_state, safety.danger_vehicle_ids) is False

    compliant_chv_state = _with_vehicle_spec(
        risky_state,
        "B01_CLV",
        vehicle_type="CHV",
        compliance_state="compliant",
    )
    non_compliant_chv_state = _with_vehicle_spec(
        risky_state,
        "B01_CLV",
        vehicle_type="CHV",
        compliance_state="non_compliant",
    )
    assert is_base_controllable(compliant_chv_state.vehicle_specs["B01_CLV"]) is True
    assert effective_controllable("B01_CLV", compliant_chv_state, ()) is True
    assert is_base_controllable(non_compliant_chv_state.vehicle_specs["B01_CLV"]) is False
    assert effective_controllable("B01_CLV", non_compliant_chv_state, ()) is False


def test_gap_identification_numbers_lane_2_pairs_by_position_and_danger_control_type() -> None:
    state, _ = build_initial_ramp_merge_state("BASIC-01")
    snapshot = identify_and_number_gaps(state, ())

    assert snapshot.step == state.step
    assert len(snapshot.gaps) == 1
    gap = snapshot.gaps[0]
    assert gap.index == 1
    assert gap.gap_id == "gap:0:1"
    assert gap.front_vehicle_id == "B01_CLV"
    assert gap.rear_vehicle_id == "B01_CFV"
    assert gap.effective_control_type == EFFECTIVE_CONTROL_BOTH

    front_danger = identify_and_number_gaps(state, ("B01_CLV",)).gaps[0]
    rear_danger = identify_and_number_gaps(state, ("B01_CFV",)).gaps[0]
    both_danger = identify_and_number_gaps(state, ("B01_CLV", "B01_CFV")).gaps[0]
    assert front_danger.effective_control_type == EFFECTIVE_CONTROL_REAR
    assert rear_danger.effective_control_type == EFFECTIVE_CONTROL_FRONT
    assert both_danger.effective_control_type == EFFECTIVE_CONTROL_NONE

    swapped_positions = _with_vehicle_state(
        _with_vehicle_state(state, "B01_CLV", x_global=100.0),
        "B01_CFV",
        x_global=150.0,
    )
    sorted_gap = identify_and_number_gaps(swapped_positions, ()).gaps[0]
    assert sorted_gap.index == 1
    assert sorted_gap.front_vehicle_id == "B01_CFV"
    assert sorted_gap.rear_vehicle_id == "B01_CLV"


def test_engine_runs_periodic_no_trigger_and_entry_trigger_without_old_commands() -> None:
    state, config = build_initial_ramp_merge_state("BASIC-01")
    engine = RampMergeEngine(config, run_id="ramp-merge-batch-b-test")

    step_0 = engine.advance_one_step(state)
    assert step_0.trigger_decision.trigger_reason == TRIGGER_PERIODIC
    assert step_0.trigger_decision.planner_state.next_plan_time == pytest.approx(2.0)
    assert step_0.gap_snapshot is not None
    assert len(step_0.gap_snapshot.gaps) == 1
    assert step_0.gap_snapshot.gaps[0].front_vehicle_id == "B01_CLV"
    assert step_0.gap_snapshot.gaps[0].rear_vehicle_id == "B01_CFV"

    step_1 = engine.advance_one_step(step_0.advanced_state)
    assert step_1.trigger_decision.trigger_plan is False
    assert step_1.trigger_decision.trigger_reason == TRIGGER_NONE
    assert step_1.gap_snapshot is None
    assert step_1.ramp_merge_runtime.last_gap_snapshot is not None
    assert step_1.ramp_merge_runtime.last_gap_snapshot.step == 0
    assert not any(
        event["event_type"] == "ramp_merge_gap_snapshot"
        for event in step_1.actual_events
    )

    state_at_step_2 = step_1.advanced_state
    step_2 = engine.advance_one_step(state_at_step_2)
    step_3 = engine.advance_one_step(step_2.advanced_state)
    step_4 = engine.advance_one_step(step_3.advanced_state)
    step_5 = engine.advance_one_step(step_4.advanced_state)
    assert step_5.frozen_state.vehicle_states["B01_MV"].x_global == pytest.approx(6650.0)
    assert step_5.ramp_merge_runtime.mv_plan_states["B01_MV"].zone_state == ZONE_CONTROL
    assert step_5.trigger_decision.trigger_reason == TRIGGER_MV_ENTER_CONTROL_ZONE
    assert step_5.trigger_decision.entry_vehicle_ids == ("B01_MV",)
    assert step_5.gap_snapshot is not None

    assert step_5.command_buffer.longitudinal_commands == {}
    assert step_5.command_buffer.cooperation_commands == {}
    assert step_5.command_buffer.lane_change_commands == {}
    assert step_5.command_buffer.merge_commands == {}
    assert all(
        len(step_5.next_state_buffer.candidate_kinematics[vehicle_id]) == 1
        for vehicle_id in step_5.frozen_state.active_vehicle_ids
    )
    mv_plan_state = step_5.ramp_merge_runtime.mv_plan_states["B01_MV"]
    assert mv_plan_state.current_plan_gap is not None
    assert mv_plan_state.current_plan_gap.index == 1
    assert mv_plan_state.current_plan_gap.front_vehicle_id == "B01_CLV"
    assert mv_plan_state.current_plan_gap.rear_vehicle_id == "B01_CFV"
    assert mv_plan_state.locked_gap is None
    assert mv_plan_state.planned_trajectory_id == "ramp_merge_approaching:5:B01_MV"


def test_runner_summary_separates_trigger_and_non_trigger_gap_events() -> None:
    summary = run_ramp_merge_basic_smoke(
        "BASIC-01",
        max_steps=7,
        run_id="ramp-merge-batch-b-summary-test",
    )

    assert summary["zone_state_timeline"][0]["zone_state_by_mv"]["B01_MV"] == ZONE_OUTSIDE_CONTROL
    assert any(
        item["step"] == 5
        and item["zone_state_by_mv"]["B01_MV"] == ZONE_CONTROL
        for item in summary["zone_state_timeline"]
    )
    trigger_by_step = {
        item["step"]: item
        for item in summary["trigger_events"]
    }
    assert trigger_by_step[0]["trigger_reason"] == TRIGGER_PERIODIC
    assert trigger_by_step[1]["trigger_plan"] is False
    assert trigger_by_step[1]["gap_identification_executed"] is False
    assert trigger_by_step[5]["trigger_reason"] == TRIGGER_MV_ENTER_CONTROL_ZONE
    assert trigger_by_step[5]["entry_vehicle_ids"] == ["B01_MV"]

    assert summary["gap_identification_steps"] == (0, 5)
    assert summary["non_trigger_gap_event_count"] == 0
    assert [item["step"] for item in summary["gap_snapshots"]] == [0, 5]
    assert summary["gap_snapshots"][1]["gaps"][0]["index"] == 1
    assert summary["runtime_mv_states"]["B01_MV"]["current_plan_gap"]["index"] == 1
    assert summary["runtime_mv_states"]["B01_MV"]["planned_trajectory_id"] == (
        "ramp_merge_approaching:5:B01_MV"
    )
    assert summary["runtime_mv_states"]["B01_MV"]["locked_gap"] is None


def test_batch_b_source_stays_isolated_from_old_algorithm_path() -> None:
    package_dir = Path(__file__).parents[1] / "cormc" / "onestep" / "rolling"
    source = "\n".join(path.read_text(encoding="utf-8") for path in package_dir.glob("*.py"))

    for fragment in (
        "from cormc.simulation_core.aps",
        "from cormc.simulation_core.cmc",
        "from cormc.simulation_core.cooperative_request",
        "from cormc.simulation_core.cuc",
        "assignment_lifecycle",
        "from cormc.simulation_core.engine",
        "import cormc.simulation_core.engine",
    ):
        assert fragment not in source

    for fragment in (
        ".startswith(",
        ".endswith(",
    ):
        assert fragment not in (package_dir / "gaps.py").read_text(encoding="utf-8")
        assert fragment not in (package_dir / "planner.py").read_text(encoding="utf-8")


def _with_vehicle_state(state, vehicle_id: str, **updates):
    vehicle_states = dict(state.vehicle_states)
    vehicle_states[vehicle_id] = replace(vehicle_states[vehicle_id], **updates)
    return replace(state, vehicle_states=vehicle_states)


def _with_vehicle_spec(state, vehicle_id: str, **updates):
    vehicle_specs = dict(state.vehicle_specs)
    vehicle_specs[vehicle_id] = replace(vehicle_specs[vehicle_id], **updates)
    return replace(state, vehicle_specs=vehicle_specs)
