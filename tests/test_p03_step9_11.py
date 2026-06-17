from __future__ import annotations

from dataclasses import fields

import pytest

from cormc.simulation_core.commit import (
    IDENTITY_CANDIDATE_SOURCE,
    TEST_HARNESS_CANDIDATE_SOURCE,
    CandidateCacheUpdate,
    CandidateKinematics,
    CandidateLaneState,
    CandidateLateralKinematics,
    CandidateLongitudinalKinematics,
    CandidateManeuverProgress,
    CandidateStateTransition,
    CommandBuffer,
    EventRecord,
    NextStateBuffer,
    OutputHistory,
    SanityCheckRecord,
    TrajectoryRecord,
    advance_time_after_commit_and_integration,
    assemble_candidate_kinematics,
    build_command_buffer_for_state,
    build_identity_next_state_buffer,
    build_test_harness_next_state_buffer,
    commit_step,
)
from cormc.simulation_core.pre_freeze import (
    build_prefreeze_workspace_from_scenario,
    freeze_simulation_state,
)
from cormc.scenario_schema import (
    match_expected_events,
    match_expected_png_features_v0,
    match_expected_sanity_checks,
)


def test_commit_step_records_each_vehicle_once_and_identity_candidate_source() -> None:
    result = _run_commit_lite()

    assert set(result.final_candidates) == set(result.previous_state.active_vehicle_ids)
    assert all(
        candidate.source == IDENTITY_CANDIDATE_SOURCE
        for candidate in result.final_candidates.values()
    )
    commit_events = [
        record for record in result.history.event_records if record.event_type == "commit"
    ]
    assert len(commit_events) == len(result.previous_state.active_vehicle_ids)
    assert {record.vehicle_id for record in commit_events} == set(
        result.previous_state.active_vehicle_ids
    )


def test_duplicate_candidate_triggers_multiple_commit_sanity() -> None:
    state = _frozen_commit_lite_state()
    command_buffer = build_command_buffer_for_state(state)
    first = _candidate(state, "MV_COMMIT_LITE", "candidate-a", x_delta=1.0)
    second = _candidate(state, "MV_COMMIT_LITE", "candidate-b", x_delta=2.0)
    buffer = build_test_harness_next_state_buffer(
        state,
        {"MV_COMMIT_LITE": [first, second]},
    )

    result = commit_step(
        state,
        command_buffer,
        buffer,
        run_id="duplicate-test",
        scenario_id="P03-DUPLICATE",
    )

    duplicate_checks = [
        record
        for record in result.history.sanity_check_records
        if record.check_type == "multiple_commit_for_one_vehicle"
    ]
    assert duplicate_checks
    assert duplicate_checks[0].result == "fail"
    assert duplicate_checks[0].vehicle_ids == ("MV_COMMIT_LITE",)
    assert "MV_COMMIT_LITE" not in result.next_state.active_vehicle_ids


def test_duplicate_guard_allows_single_final_candidate_with_multiple_components() -> None:
    state = _frozen_commit_lite_state()
    longitudinal = CandidateLongitudinalKinematics(
        candidate_id="long-1",
        vehicle_id="MV_COMMIT_LITE",
        x_global=6951.0,
        v=20.0,
        a=0.0,
        candidate_speed=20.0,
        planning_speed=20.0,
        source=TEST_HARNESS_CANDIDATE_SOURCE,
        constraints_applied=("test_constraint",),
    )
    lateral = CandidateLateralKinematics(
        candidate_id="lat-1",
        vehicle_id="MV_COMMIT_LITE",
        y=-3.4,
        target_y=0.0,
        source=TEST_HARNESS_CANDIDATE_SOURCE,
    )
    progress = CandidateManeuverProgress(
        candidate_id="progress-1",
        vehicle_id="MV_COMMIT_LITE",
        maneuver_type="merge",
        progress=0.1,
        completed=False,
        target_y_reached=False,
    )
    candidate = assemble_candidate_kinematics(
        state,
        "MV_COMMIT_LITE",
        longitudinal=longitudinal,
        lateral=lateral,
        maneuver_progress=progress,
        source=TEST_HARNESS_CANDIDATE_SOURCE,
        candidate_id="assembled-single",
    )
    buffer = build_test_harness_next_state_buffer(state, {"MV_COMMIT_LITE": candidate})

    result = commit_step(
        state,
        build_command_buffer_for_state(state),
        buffer,
        run_id="component-test",
        scenario_id="P03-COMPONENTS",
    )

    sanity = _sanity_by_type(result.history.sanity_check_records, "multiple_commit_for_one_vehicle")
    assert sanity.result == "pass"
    assert result.next_state.vehicle_states["MV_COMMIT_LITE"].x_global == 6951.0
    assert result.next_state.vehicle_states["MV_COMMIT_LITE"].y == -3.4


def test_next_state_does_not_mutate_frozen_state() -> None:
    state = _frozen_commit_lite_state()
    before = state.vehicle_states["MV_COMMIT_LITE"]
    buffer = build_test_harness_next_state_buffer(
        state,
        {"MV_COMMIT_LITE": _candidate(state, "MV_COMMIT_LITE", "move", x_delta=3.0)},
    )

    result = commit_step(
        state,
        build_command_buffer_for_state(state),
        buffer,
        run_id="immutability-test",
        scenario_id="P03-IMMUTABLE",
    )

    assert state.vehicle_states["MV_COMMIT_LITE"] == before
    assert result.next_state.vehicle_states["MV_COMMIT_LITE"].x_global == before.x_global + 3.0
    no_write = _sanity_by_type(result.history.sanity_check_records, "no_write_before_commit")
    assert no_write.result == "pass"


def test_commit_applies_lane_state_transition_and_cache_cleanup_only_to_next_state() -> None:
    state = _frozen_commit_lite_state()
    original_mv = state.vehicle_states["MV_COMMIT_LITE"]
    cache_with_assignment = {"MV_COMMIT_LITE": {"status": "valid", "clv_id": "CLV_COMMIT_LITE"}}
    state = _replace_state_cache(state, cache_with_assignment)
    candidate = _candidate(state, "MV_COMMIT_LITE", "merge-complete", x_delta=1.0)
    buffer = NextStateBuffer(
        step=state.step,
        t=state.t,
        candidate_kinematics={"MV_COMMIT_LITE": (candidate,)},
        candidate_lane_state={
            "MV_COMMIT_LITE": CandidateLaneState(
                candidate_id="lane-mainline",
                vehicle_id="MV_COMMIT_LITE",
                physical_lane="lane_2",
                road_role="mainline",
                reason="merge_centerline_reached",
            )
        },
        candidate_state_transitions={
            "MV_COMMIT_LITE": (
                CandidateStateTransition(
                    candidate_id="merge-state",
                    vehicle_id="MV_COMMIT_LITE",
                    state_name="merge_state",
                    old_state=original_mv.merge_state,
                    new_state="merged",
                    reason="merge_centerline_reached",
                ),
            )
        },
        candidate_cache_updates=(
            CandidateCacheUpdate(
                candidate_id="cache-cleanup",
                cache_name="assignment_records_by_mv",
                owner_vehicle_id="MV_COMMIT_LITE",
                operation="cleanup",
                reason="merge_completed",
            ),
        ),
    )

    result = commit_step(
        state,
        build_command_buffer_for_state(state),
        buffer,
        run_id="transition-test",
        scenario_id="P03-TRANSITION",
    )

    assert state.vehicle_states["MV_COMMIT_LITE"] == original_mv
    committed_mv = result.next_state.vehicle_states["MV_COMMIT_LITE"]
    assert committed_mv.physical_lane == "lane_2"
    assert committed_mv.road_role == "mainline"
    assert committed_mv.merge_state == "merged"
    assert "MV_COMMIT_LITE" in state.assignment_records_by_mv
    assert "MV_COMMIT_LITE" not in result.next_state.assignment_records_by_mv
    event = _event_by_vehicle(result.history.event_records, "MV_COMMIT_LITE")
    assert event.payload["state_transitions"][0]["new_state"] == "merged"
    assert event.payload["cache_cleanup_vehicle_ids"] == ["MV_COMMIT_LITE"]


def test_commit_invalidate_deletes_assignment_records_by_mv_key() -> None:
    state = _frozen_commit_lite_state()
    state = _replace_state_cache(state, {"MV_COMMIT_LITE": {"status": "valid", "clv_id": "CLV"}})
    candidate = _candidate(state, "MV_COMMIT_LITE", "wait", x_delta=0.0)
    buffer = NextStateBuffer(
        step=state.step,
        t=state.t,
        candidate_kinematics={"MV_COMMIT_LITE": (candidate,)},
        candidate_cache_updates=(
            CandidateCacheUpdate(
                candidate_id="cache-invalidate",
                cache_name="assignment_records_by_mv",
                owner_vehicle_id="MV_COMMIT_LITE",
                operation="invalidate",
                reason="cached_gap_boundary_invalid",
            ),
        ),
    )

    result = commit_step(
        state,
        build_command_buffer_for_state(state),
        buffer,
        run_id="invalidate-test",
        scenario_id="P03-INVALIDATE",
    )

    assert "MV_COMMIT_LITE" in state.assignment_records_by_mv
    assert "MV_COMMIT_LITE" not in result.next_state.assignment_records_by_mv
    event = _event_by_vehicle(result.history.event_records, "MV_COMMIT_LITE")
    assert event.payload["cache_invalidate_vehicle_ids"] == ["MV_COMMIT_LITE"]


def test_candidate_source_rejects_hidden_model_candidate() -> None:
    state = _frozen_commit_lite_state()
    bad_candidate = CandidateKinematics(
        candidate_id="bad",
        vehicle_id="MV_COMMIT_LITE",
        x_global=6951.0,
        y=-3.5,
        v=20.0,
        a=0.0,
        source="longitudinal_model",
    )

    with pytest.raises(ValueError, match="candidate source"):
        build_test_harness_next_state_buffer(state, {"MV_COMMIT_LITE": bad_candidate})


def test_candidate_assembly_rejects_hidden_component_source() -> None:
    state = _frozen_commit_lite_state()
    longitudinal = CandidateLongitudinalKinematics(
        candidate_id="hidden-longitudinal",
        vehicle_id="MV_COMMIT_LITE",
        x_global=6951.0,
        v=20.0,
        a=0.0,
        candidate_speed=20.0,
        planning_speed=20.0,
        source="longitudinal_model",
    )

    with pytest.raises(ValueError, match="candidate component source"):
        assemble_candidate_kinematics(
            state,
            "MV_COMMIT_LITE",
            longitudinal=longitudinal,
            source=TEST_HARNESS_CANDIDATE_SOURCE,
        )


def test_candidate_assembly_rejects_component_vehicle_mismatch() -> None:
    state = _frozen_commit_lite_state()
    lateral = CandidateLateralKinematics(
        candidate_id="wrong-lateral",
        vehicle_id="CLV_COMMIT_LITE",
        y=0.0,
        target_y=0.0,
        source=TEST_HARNESS_CANDIDATE_SOURCE,
    )

    with pytest.raises(ValueError, match="component vehicle_id"):
        assemble_candidate_kinematics(
            state,
            "MV_COMMIT_LITE",
            lateral=lateral,
            source=TEST_HARNESS_CANDIDATE_SOURCE,
        )


def test_cuc_decision_not_persisted_after_commit() -> None:
    state = _frozen_commit_lite_state()
    command_buffer = build_command_buffer_for_state(
        state,
        cuc_decisions={"CLV_COMMIT_LITE": {"choice": "change_to_lane_1"}},
    )

    result = commit_step(
        state,
        command_buffer,
        build_identity_next_state_buffer(state),
        run_id="cuc-history-test",
        scenario_id="P03-CUC-NON-PERSISTENT",
    )

    next_state_fields = set(vars(result.next_state.vehicle_states["CLV_COMMIT_LITE"]))
    assert "cuc_choice" not in next_state_fields
    assert "cuc_decision" not in next_state_fields
    commit_event = _event_by_vehicle(result.history.event_records, "CLV_COMMIT_LITE")
    assert commit_event.payload["cuc_decision_persisted_to_state"] is False


def test_step10_does_not_rewrite_committed_state() -> None:
    result = _run_commit_lite()

    step10 = _sanity_by_type(
        result.history.sanity_check_records,
        "information_integration_does_not_rewrite_state",
    )
    assert step10.result == "pass"
    assert step10.payload["step10_does_not_rewrite_committed_state"] is True


def test_event_sanity_trajectory_output_history_v0_lands() -> None:
    result = _run_commit_lite()

    assert isinstance(result.history, OutputHistory)
    assert result.history.event_records
    assert result.history.sanity_check_records
    assert result.history.trajectory_records
    assert all(isinstance(record, EventRecord) for record in result.history.event_records)
    assert all(
        isinstance(record, SanityCheckRecord)
        for record in result.history.sanity_check_records
    )
    assert all(
        isinstance(record, TrajectoryRecord)
        for record in result.history.trajectory_records
    )
    assert len(result.history.trajectory_records) == len(result.previous_state.active_vehicle_ids)
    assert "x_plot" not in {field.name for field in fields(result.history.trajectory_records[0])}


def test_p03_event_and_sanity_records_are_consumable_by_p01_matcher() -> None:
    result = _run_commit_lite()

    events = match_expected_events(
        [
            {
                "event_type": "commit",
                "required": True,
                "vehicle_ids": ["MV_COMMIT_LITE"],
                "match": {
                    "each_active_vehicle_has_exactly_one_final_next_state": True,
                    "no_module_writes_committed_state_before_commit": True,
                    "command_buffer_and_next_state_buffer_are_separated": True,
                    "candidate_source": IDENTITY_CANDIDATE_SOURCE,
                },
                "source": "first_version_engineering_patch",
            },
            {
                "event_type": "information_integration",
                "required": True,
                "match": {"step10_does_not_rewrite_committed_state": True},
            },
            {
                "event_type": "time_advance",
                "required": True,
                "match": {
                    "advanced_after_commit": True,
                    "advanced_after_information_integration": True,
                },
            },
        ],
        result.history.event_dicts(),
        {"derived_formula_abs": 0.01},
    )
    sanity = match_expected_sanity_checks(
        [
            {
                "check_type": "multiple_commit_for_one_vehicle",
                "required": True,
                "expected_status": "pass",
                "vehicle_ids": ["MV_COMMIT_LITE"],
            },
            {
                "check_type": "state_machine_inconsistency",
                "required": True,
                "expected_status": "pass",
                "vehicle_ids": ["MV_COMMIT_LITE"],
            },
            {
                "check_type": "time_advance_consistency",
                "required": True,
                "expected_status": "pass",
            },
        ],
        result.history.sanity_dicts(),
    )
    png = match_expected_png_features_v0(result.expected_png_features)

    assert events.passed is True
    assert sanity.passed is True
    assert png.passed is True


def test_step11_time_advance_lands_after_commit_and_step10() -> None:
    result = _run_commit_lite()
    time_advance_event = [
        record for record in result.history.event_records if record.event_type == "time_advance"
    ][0]

    assert time_advance_event.payload["old_step"] == 0
    assert time_advance_event.payload["new_step"] == 1
    assert time_advance_event.payload["old_t"] == 0.0
    assert time_advance_event.payload["new_t"] == 0.1
    assert time_advance_event.payload["advanced_after_commit"] is True
    assert time_advance_event.payload["advanced_after_information_integration"] is True


def test_explicit_time_advance_returns_advanced_state() -> None:
    state = _frozen_commit_lite_state()
    result = commit_step(
        state,
        build_command_buffer_for_state(state),
        build_identity_next_state_buffer(state),
        run_id="explicit-advance",
        scenario_id="P03-TIME",
    )

    advance = advance_time_after_commit_and_integration(
        result,
        run_id="explicit-advance",
        scenario_id="P03-TIME",
    )

    assert advance.advanced_state.step == result.next_state.step + 1
    assert advance.advanced_state.t == result.next_state.t + result.next_state.dt
    assert advance.event_record.event_type == "time_advance"
    assert advance.sanity_record.check_type == "time_advance_consistency"


def _frozen_commit_lite_state():
    workspace, _ = build_prefreeze_workspace_from_scenario(_commit_lite_config())
    return freeze_simulation_state(workspace)


def _run_commit_lite():
    workspace, config = build_prefreeze_workspace_from_scenario(_commit_lite_config())
    state = freeze_simulation_state(workspace)
    result = commit_step(
        state,
        build_command_buffer_for_state(state),
        build_identity_next_state_buffer(state),
        run_id="P03-COMMIT-LITE",
        scenario_id=config["scenario_id"],
    )
    advance_time_after_commit_and_integration(
        result,
        run_id="P03-COMMIT-LITE",
        scenario_id=config["scenario_id"],
    )
    return result


def _commit_lite_config() -> dict:
    return {
        "scenario_id": "P03-COMMIT-LITE",
        "scenario_name": "P03 commit lite",
        "purpose": "Inline commit infrastructure scenario.",
        "test_level": "unit",
        "status": "required",
        "initial_time": {"t": 0.0, "step": 0, "dt": 0.1},
        "initial_vehicles": [
            _vehicle("MV_COMMIT_LITE", "on_ramp", 6950.0, -3.5, road_role="on_ramp_mv", merge_state="executing"),
            _vehicle("CLV_COMMIT_LITE", "lane_2", 7000.0, 0.0),
        ],
        "module_overrides": {
            "boundary_generation_enabled": False,
            "random_arrival_enabled": False,
            "random_vehicle_attributes_enabled": False,
            "ordinary_mainline_lane_change_enabled": False,
            "platoon_cmc_enabled": False,
            "mpc_lateral_tracking_enabled": False,
        },
        "preloaded_assignments": [],
        "preloaded_state_machine_states": [],
        "preloaded_maneuver_trajectory_states": [],
        "expected_events": [],
        "forbidden_events": [],
        "expected_event_counts": [],
        "expected_sanity_checks": [],
        "expected_png_features": [],
    }


def _vehicle(
    vehicle_id: str,
    lane: str,
    x_global: float,
    y: float,
    *,
    road_role: str = "mainline",
    merge_state: str = "none",
) -> dict:
    return {
        "vehicle_id": vehicle_id,
        "vehicle_type": "CAV",
        "compliance_state": "not_applicable",
        "initial_x_global": x_global,
        "initial_y": y,
        "initial_v": 20.0,
        "initial_a": 0.0,
        "physical_lane": lane,
        "road_role": road_role,
        "lane_change_state": "normal",
        "merge_state": merge_state,
        "spec_overrides": {},
    }


def _replace_state_cache(state, cache):
    from dataclasses import replace
    from types import MappingProxyType

    return replace(
        state,
        assignment_records_by_mv=MappingProxyType(
            {
                vehicle_id: MappingProxyType(dict(value))
                for vehicle_id, value in cache.items()
            }
        ),
    )


def _candidate(
    state,
    vehicle_id: str,
    candidate_id: str,
    *,
    x_delta: float = 0.0,
) -> CandidateKinematics:
    current = state.vehicle_states[vehicle_id]
    return CandidateKinematics(
        candidate_id=candidate_id,
        vehicle_id=vehicle_id,
        x_global=current.x_global + x_delta,
        y=current.y,
        v=current.v,
        a=current.a,
        source=TEST_HARNESS_CANDIDATE_SOURCE,
    )


def _event_by_vehicle(records: list[EventRecord], vehicle_id: str) -> EventRecord:
    for record in records:
        if record.event_type == "commit" and record.vehicle_id == vehicle_id:
            return record
    raise AssertionError(f"missing commit event for {vehicle_id}")


def _sanity_by_type(records: list[SanityCheckRecord], check_type: str) -> SanityCheckRecord:
    for record in records:
        if record.check_type == check_type:
            return record
    raise AssertionError(f"missing sanity check {check_type}")
