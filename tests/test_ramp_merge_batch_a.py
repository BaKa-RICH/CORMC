from __future__ import annotations

from pathlib import Path

import pytest

from cormc.legacy import (
    GapRef,
    RampMergeEngine,
    RampMergeRuntimeState,
    ZONE_OUTSIDE_CONTROL,
    build_initial_ramp_merge_state,
    initialize_runtime_state,
    run_ramp_merge_basic_smoke,
)


def test_ramp_merge_package_is_isolated_from_old_algorithm_path() -> None:
    package_dir = Path(__file__).parents[1] / "cormc" / "ramp_merge_algorithm"
    source = "\n".join(path.read_text(encoding="utf-8") for path in package_dir.glob("*.py"))

    forbidden_fragments = (
        "from cormc.simulation_core.aps",
        "from cormc.simulation_core.cmc",
        "from cormc.simulation_core.cooperative_request",
        "from cormc.simulation_core.cuc",
        "assignment_lifecycle",
        "from cormc.simulation_core.engine",
        "import cormc.simulation_core.engine",
    )
    for fragment in forbidden_fragments:
        assert fragment not in source

    forbidden_vehicle_id_parsing = (
        'vehicle_id.startswith("MV',
        "vehicle_id.startswith('MV",
        'vehicle_id.endswith("',
        "vehicle_id.endswith('",
        'mv_id.startswith("MV',
        "mv_id.startswith('MV",
        'mv_id.endswith("',
        "mv_id.endswith('",
    )
    for fragment in forbidden_vehicle_id_parsing:
        assert fragment not in source


def test_runtime_initialization_uses_structured_on_ramp_attributes() -> None:
    state, _ = build_initial_ramp_merge_state("BASIC-01")
    runtime = state.ramp_merge_runtime

    assert isinstance(runtime, RampMergeRuntimeState)
    assert "B01_MV" in runtime.mv_plan_states
    assert set(runtime.mv_plan_states) == {"B01_MV"}
    mv_state = runtime.mv_plan_states["B01_MV"]
    assert mv_state.merge_state == "normal"
    assert mv_state.zone_state == ZONE_OUTSIDE_CONTROL
    assert mv_state.current_plan_gap is None
    assert mv_state.locked_gap is None
    assert dict(runtime.planned_trajectories) == {}
    assert mv_state.__dataclass_fields__["current_plan_gap"].type in {
        GapRef | None,
        "GapRef | None",
    }
    assert dict(state.assignment_records_by_mv) == {}


def test_runtime_refresh_adds_and_removes_on_ramp_mvs_without_gap_logic() -> None:
    state, _ = build_initial_ramp_merge_state("BASIC-01")
    runtime = initialize_runtime_state(state)

    without_mv = state.__class__(
        t=state.t,
        step=state.step,
        dt=state.dt,
        active_vehicle_ids=tuple(
            vehicle_id for vehicle_id in state.active_vehicle_ids if vehicle_id != "B01_MV"
        ),
        vehicle_states=state.vehicle_states,
        vehicle_specs=state.vehicle_specs,
        assignment_records_by_mv=state.assignment_records_by_mv,
        active_maneuvers=state.active_maneuvers,
        road_config_ref=state.road_config_ref,
        parameter_config_ref=state.parameter_config_ref,
        scenario_config_ref=state.scenario_config_ref,
        output_config_ref=state.output_config_ref,
        controller_memory_by_vehicle=state.controller_memory_by_vehicle,
        ramp_merge_runtime=runtime,
    )

    from cormc.onestep.rolling.state import refresh_runtime_state

    refreshed = refresh_runtime_state(runtime, without_mv)

    assert dict(refreshed.mv_plan_states) == {}
    assert dict(refreshed.planned_trajectories) == {}


def test_engine_single_step_empty_commands_and_commit_health() -> None:
    state, config = build_initial_ramp_merge_state("BASIC-01")
    result = RampMergeEngine(config, run_id="ramp-merge-batch-a-test").advance_one_step(state)

    assert result.advanced_state.step == state.step + 1
    assert result.advanced_state.t == pytest.approx(state.t + state.dt)
    assert result.command_buffer.longitudinal_commands == {}
    assert result.command_buffer.cooperation_commands == {}
    assert result.command_buffer.lane_change_commands == {}
    assert result.command_buffer.merge_commands == {}
    assert set(result.commit_result.final_candidates) == set(state.active_vehicle_ids)
    assert all(
        len(result.next_state_buffer.candidate_kinematics[vehicle_id]) == 1
        for vehicle_id in state.active_vehicle_ids
    )

    sanity_results = {
        record.check_type: record.result
        for record in result.commit_result.history.sanity_check_records
    }
    assert sanity_results["multiple_commit_for_one_vehicle"] == "pass"
    assert sanity_results["no_write_before_commit"] == "pass"
    assert sanity_results["state_machine_inconsistency"] == "pass"


def test_basic_01_batch_a_smoke_runs_straight_without_old_state_or_events() -> None:
    summary = run_ramp_merge_basic_smoke(
        scenario_id="BASIC-01",
        max_steps=5,
        run_id="ramp-merge-batch-a-test",
    )

    assert summary["scenario_id"] == "BASIC-01"
    assert summary["steps_run"] == 5
    assert summary["initial_t"] == 0.0
    assert summary["final_t"] == pytest.approx(0.5)
    assert summary["runtime_mv_ids"] == ("B01_MV",)
    assert summary["runtime_mv_states"]["B01_MV"] == {
        "zone_state": ZONE_OUTSIDE_CONTROL,
        "merge_state": "normal",
        "current_plan_gap": None,
        "locked_gap": None,
        "planned_trajectory_id": None,
        "last_plan_step": None,
        "last_plan_t": None,
        "planned_trajectory": None,
    }

    initial_positions = summary["initial_positions"]
    final_positions = summary["final_positions"]
    assert initial_positions["B01_MV"]["x_global"] == pytest.approx(6640.0)
    assert final_positions["B01_MV"]["x_global"] == pytest.approx(6650.0)
    assert final_positions["B01_MV"]["y"] == pytest.approx(-3.5)
    assert final_positions["B01_CLV"]["x_global"] == pytest.approx(
        initial_positions["B01_CLV"]["x_global"] + 10.0
    )
    assert final_positions["B01_TLV_CFV"]["x_global"] == pytest.approx(
        initial_positions["B01_TLV_CFV"]["x_global"] + 7.5
    )
    assert summary["old_assignment_record_count"] == 0
    assert summary["old_active_maneuver_count"] == 0

    event_types = set(summary["event_types"])
    assert {
        "cleanup",
        "boundary_generation",
        "freeze",
        "relation_refresh",
        "geometry",
        "ramp_merge_runtime",
        "ramp_merge_zone_state",
        "ramp_merge_safety",
        "ramp_merge_trigger",
        "ramp_merge_default_motion",
        "commit",
        "information_integration",
        "time_advance",
    }.issubset(event_types)
    forbidden_event_markers = (
        "aps",
        "cmc",
        "cooperative_request",
        "cuc",
        "assignment_cache",
        "lane_change_command",
    )
    assert not any(
        marker in event_type.lower().replace("snapshot", "")
        for marker in forbidden_event_markers
        for event_type in event_types
    )
    assert summary["sanity_results"]["multiple_commit_for_one_vehicle"] == "pass"
    assert summary["sanity_results"]["no_write_before_commit"] == "pass"
    assert summary["sanity_results"]["state_machine_inconsistency"] == "pass"
