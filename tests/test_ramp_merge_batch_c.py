from __future__ import annotations

from dataclasses import replace
from pathlib import Path

import pytest

from cormc.legacy import (
    EFFECTIVE_CONTROL_NONE,
    MERGE_STATE_MERGING,
    MERGE_STATE_NORMAL,
    TRAJECTORY_APPROACHING,
    TRAJECTORY_MERGE_EXECUTION,
    ZONE_CONTROL,
    ZONE_MERGE,
    GapCandidate,
    GapSnapshot,
    RampMergeEngine,
    build_initial_ramp_merge_state,
    identify_and_number_gaps,
    lock_merge_zone_gaps,
    plan_control_zone_gaps,
    run_ramp_merge_basic_smoke,
    run_simplified_merge_check,
)


def test_batch_c_state_schema_fields_exist_on_initial_runtime() -> None:
    state, _ = build_initial_ramp_merge_state("BASIC-04")
    runtime = state.ramp_merge_runtime
    mv_state = runtime.mv_plan_states["B04_MV"]

    assert runtime.version == "batch_c_v1"
    assert mv_state.zone_state == ZONE_CONTROL
    assert mv_state.current_plan_gap is None
    assert mv_state.locked_gap is None
    assert mv_state.planned_trajectory_id is None
    assert mv_state.last_plan_step is None
    assert mv_state.last_plan_t is None
    assert dict(runtime.planned_trajectories) == {}


def test_control_zone_selects_first_available_gap_and_creates_approaching_plan() -> None:
    state, _ = build_initial_ramp_merge_state("BASIC-04")
    runtime = state.ramp_merge_runtime
    snapshot = identify_and_number_gaps(state, ())

    result = plan_control_zone_gaps(state, runtime, snapshot)
    planned = result.runtime.mv_plan_states["B04_MV"]

    assert planned.current_plan_gap is not None
    assert planned.current_plan_gap.index == snapshot.gaps[0].index
    assert planned.current_plan_gap.front_vehicle_id == "B04_CLV"
    assert planned.current_plan_gap.rear_vehicle_id == "B04_CFV"
    assert planned.locked_gap is None
    assert planned.last_plan_step == state.step
    assert planned.last_plan_t == state.t
    assert planned.planned_trajectory_id == "ramp_merge_approaching:0:B04_MV"

    trajectory = result.runtime.planned_trajectories[planned.planned_trajectory_id]
    assert trajectory.kind == TRAJECTORY_APPROACHING
    assert trajectory.target_gap == planned.current_plan_gap
    assert trajectory.start_y == pytest.approx(-3.5)
    assert trajectory.target_y == pytest.approx(-3.5)
    assert result.gap_selection_records[0]["score"] == 1.0
    assert result.gap_selection_records[0]["reason"] == "simplified_first_available_gap"


def test_control_zone_does_not_assign_same_gap_to_two_mvs_in_one_frame() -> None:
    state, _ = build_initial_ramp_merge_state("BASIC-04")
    second_mv_state = replace(
        state.vehicle_states["B04_MV"],
        vehicle_id="B04_MV_2",
        x_global=6840.0,
    )
    second_mv_spec = replace(
        state.vehicle_specs["B04_MV"],
        vehicle_id="B04_MV_2",
    )
    multi_mv_state = replace(
        state,
        active_vehicle_ids=state.active_vehicle_ids + ("B04_MV_2",),
        vehicle_states={**dict(state.vehicle_states), "B04_MV_2": second_mv_state},
        vehicle_specs={**dict(state.vehicle_specs), "B04_MV_2": second_mv_spec},
    )
    runtime = multi_mv_state.ramp_merge_runtime
    runtime = replace(
        runtime,
        mv_plan_states={
            **dict(runtime.mv_plan_states),
            "B04_MV_2": replace(
                runtime.mv_plan_states["B04_MV"],
                mv_id="B04_MV_2",
            ),
        },
    )
    multi_mv_state = replace(multi_mv_state, ramp_merge_runtime=runtime)
    snapshot = identify_and_number_gaps(multi_mv_state, ())

    result = plan_control_zone_gaps(multi_mv_state, runtime, snapshot)
    selected = [
        record["current_plan_gap"].index
        for record in result.gap_selection_records
        if record["current_plan_gap"] is not None
    ]

    assert selected == [1]
    assert result.runtime.mv_plan_states["B04_MV"].current_plan_gap is not None
    assert result.runtime.mv_plan_states["B04_MV_2"].current_plan_gap is None
    assert result.gap_selection_records[1]["reason"] == "no_available_gap"


def test_unavailable_gap_clears_current_plan_and_does_not_create_trajectory() -> None:
    state, _ = build_initial_ramp_merge_state("BASIC-04")
    runtime = state.ramp_merge_runtime
    gap = identify_and_number_gaps(state, ()).gaps[0]
    unavailable = GapSnapshot(
        step=state.step,
        t=state.t,
        lane_id="lane_2",
        gaps=(
            GapCandidate(
                gap_id=gap.gap_id,
                index=gap.index,
                front_vehicle_id=gap.front_vehicle_id,
                rear_vehicle_id=gap.rear_vehicle_id,
                front_x_global=gap.front_x_global,
                rear_x_global=gap.rear_x_global,
                bumper_gap_m=gap.bumper_gap_m,
                effective_control_type=EFFECTIVE_CONTROL_NONE,
            ),
        ),
    )

    result = plan_control_zone_gaps(state, runtime, unavailable)
    planned = result.runtime.mv_plan_states["B04_MV"]

    assert planned.current_plan_gap is None
    assert planned.planned_trajectory_id is None
    assert dict(result.runtime.planned_trajectories) == {}
    assert result.gap_selection_records[0]["reason"] == "no_available_gap"


def test_merge_zone_locks_gap_and_enters_merging_without_replacing_locked_gap() -> None:
    state, _ = build_initial_ramp_merge_state("BASIC-04")
    snapshot = identify_and_number_gaps(state, ())
    planned = plan_control_zone_gaps(state, state.ramp_merge_runtime, snapshot).runtime
    merge_state = _with_vehicle_state(state, "B04_MV", x_global=6950.0)
    merge_state = replace(merge_state, ramp_merge_runtime=planned)

    locked_result = lock_merge_zone_gaps(merge_state, planned)
    mv_plan = locked_result.runtime.mv_plan_states["B04_MV"]

    assert mv_plan.locked_gap == mv_plan.current_plan_gap
    assert mv_plan.merge_state == MERGE_STATE_MERGING
    assert mv_plan.planned_trajectory_id == "ramp_merge_execution:0:B04_MV"
    assert locked_result.gap_lock_records[0]["reason"] == "lock_current_plan_gap_on_merge_zone_entry"
    assert locked_result.merge_check_results[0].result is True
    assert locked_result.trajectory_records[0]["trajectory_kind"] == TRAJECTORY_MERGE_EXECUTION

    changed_current = replace(
        locked_result.runtime.mv_plan_states["B04_MV"],
        current_plan_gap=None,
    )
    runtime_with_existing_lock = replace(
        locked_result.runtime,
        mv_plan_states={"B04_MV": changed_current},
    )
    next_result = lock_merge_zone_gaps(merge_state, runtime_with_existing_lock)
    assert next_result.runtime.mv_plan_states["B04_MV"].locked_gap == mv_plan.locked_gap
    assert next_result.gap_lock_records == ()


def test_minimal_merge_check_fails_without_locked_gap_or_boundary_vehicles() -> None:
    state, _ = build_initial_ramp_merge_state("BASIC-04")
    merge_state = _with_vehicle_state(state, "B04_MV", x_global=6950.0)

    missing_gap = run_simplified_merge_check(merge_state, "B04_MV", None)
    assert missing_gap.result is False
    assert missing_gap.reason == "missing_locked_gap"

    snapshot = identify_and_number_gaps(state, ())
    planned = plan_control_zone_gaps(state, state.ramp_merge_runtime, snapshot).runtime
    locked_gap = planned.mv_plan_states["B04_MV"].current_plan_gap
    missing_vehicle_state = replace(
        merge_state,
        vehicle_states={
            key: value
            for key, value in merge_state.vehicle_states.items()
            if key != locked_gap.front_vehicle_id
        },
    )
    missing_vehicle = run_simplified_merge_check(
        missing_vehicle_state,
        "B04_MV",
        locked_gap,
    )
    assert missing_vehicle.result is False
    assert missing_vehicle.reason == "locked_gap_boundary_vehicle_missing"


def test_engine_simplified_merge_trajectory_completes_and_cleans_runtime() -> None:
    summary = run_ramp_merge_basic_smoke(
        "BASIC-04",
        max_steps=62,
        run_id="ramp-merge-batch-c-completion-test",
    )

    assert summary["gap_selection_events"]
    assert summary["gap_selection_events"][0]["selected_gap"]["index"] == 1
    assert summary["gap_lock_events"][0]["locked_gap"]["index"] == 1
    assert summary["merge_check_events"][0]["merge_check_result"] is True
    assert any(
        event["trajectory_kind"] == TRAJECTORY_MERGE_EXECUTION
        and event["progress_step"] == 1
        for event in summary["trajectory_events"]
    )
    assert [
        event["progress_step"]
        for event in summary["trajectory_events"]
        if event["trajectory_kind"] == TRAJECTORY_MERGE_EXECUTION
        and event["reason"] == "advance_planned_trajectory"
    ] == list(range(1, 10))
    assert summary["merge_completion_events"][0]["progress_step"] == 10
    assert summary["merge_completion_events"][0]["duration_steps"] == 10

    final_mv = summary["final_vehicle_states"]["B04_MV"]
    assert final_mv["y"] == pytest.approx(0.0)
    assert final_mv["physical_lane"] == "lane_2"
    assert final_mv["road_role"] == "mainline"
    assert final_mv["merge_state"] == MERGE_STATE_NORMAL
    assert summary["runtime_mv_ids"] == ()
    assert summary["runtime_mv_states"] == {}
    assert summary["old_assignment_record_count"] == 0
    assert summary["old_active_maneuver_count"] == 0


def test_non_trigger_frames_do_not_identify_gaps_but_continue_trajectory() -> None:
    state, config = build_initial_ramp_merge_state("BASIC-04")
    engine = RampMergeEngine(config, run_id="ramp-merge-batch-c-non-trigger-test")
    result = None
    states = [state]
    for _ in range(52):
        result = engine.advance_one_step(states[-1])
        states.append(result.advanced_state)

    assert result is not None
    assert result.frozen_state.step == 51
    assert result.trigger_decision.trigger_plan is False
    assert result.gap_snapshot is None
    assert not any(
        event["event_type"] == "ramp_merge_gap_snapshot"
        for event in result.actual_events
    )
    assert any(
        event["event_type"] == "ramp_merge_trajectory"
        and event["payload"]["trajectory_kind"] == TRAJECTORY_MERGE_EXECUTION
        for event in result.actual_events
    )


def test_batch_c_source_stays_isolated_from_old_algorithm_path() -> None:
    package_dir = Path(__file__).parents[1] / "cormc" / "onestep" / "rolling"
    legacy_runner = (
        Path(__file__).parents[1] / "cormc" / "legacy" / "ramp_merge_basic.py"
    )
    source_by_file = {
        path.name: path.read_text(encoding="utf-8")
        for path in package_dir.glob("*.py")
    }
    source = "\n".join(source_by_file.values())

    for fragment in (
        "from cormc.simulation_core.aps",
        "from cormc.simulation_core.cmc",
        "from cormc.simulation_core.cooperative_request",
        "from cormc.simulation_core.cuc",
        "from cormc.simulation_core.engine",
        "import cormc.simulation_core.engine",
        "gap_id.split",
        ".startswith(",
        ".endswith(",
    ):
        assert fragment not in source

    algorithm_source = "\n".join(
        text
        for filename, text in source_by_file.items()
        if filename != "stage2_runner.py"
    )
    assert "assignment_records_by_mv" not in algorithm_source
    assert "active_maneuvers" not in algorithm_source
    legacy_source = legacy_runner.read_text(encoding="utf-8")
    assert "assignment_records_by_mv" in legacy_source
    assert "active_maneuvers" in legacy_source


def _with_vehicle_state(state, vehicle_id: str, **updates):
    vehicle_states = dict(state.vehicle_states)
    vehicle_states[vehicle_id] = replace(vehicle_states[vehicle_id], **updates)
    return replace(state, vehicle_states=vehicle_states)
