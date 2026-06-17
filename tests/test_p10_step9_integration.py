from __future__ import annotations

from types import MappingProxyType
from typing import Any

from cormc.simulation_core.pre_freeze import (
    build_prefreeze_workspace_from_scenario,
    freeze_simulation_state,
    refresh_relations_snapshot,
)
from cormc.simulation_core.commit import (
    CandidateCacheUpdate,
    CandidateKinematics,
    CandidateLaneState,
    CandidateLateralKinematics,
    CandidateLongitudinalKinematics,
    CandidateManeuverProgress,
    CandidateStateTransition,
    CommandBuffer,
    NextStateBuffer,
    commit_step,
)


def test_p10_assembles_p08_longitudinal_and_p09_lateral_candidate() -> None:
    state = _state(_p10_config(vehicle_ids=("MV_E2E",)))
    buffer = _buffer(
        state,
        "MV_E2E",
        longitudinal=_longitudinal(state, "MV_E2E", x_global=6841.2, v=12.0, a=-1.0),
        lateral=_lateral(state, "MV_E2E", y=-3.42, target_y=0.0),
        progress=_progress(state, "MV_E2E", "merge", progress=0.1, completed=False),
    )

    result = commit_step(
        state,
        _command_buffer(state),
        buffer,
        run_id="P10-ASSEMBLY",
        scenario_id="P10-ASSEMBLY",
    )

    candidate = result.final_candidates["MV_E2E"]
    assert candidate.source == "step9_candidate_assembly"
    assert candidate.x_global == 6841.2
    assert candidate.v == 12.0
    assert candidate.a == -1.0
    assert candidate.y == -3.42
    assert candidate.source_longitudinal_candidate == "p08:0:MV_E2E:longitudinal"
    assert candidate.source_lateral_candidate == "p09:0:MV_E2E:lateral"
    assert candidate.source_maneuver_progress == "p09:0:MV_E2E:maneuver_progress"
    assert candidate.constraints_applied == ("boundary_speed_cap",)


def test_p10_normal_vehicle_without_lateral_candidate_keeps_current_y_and_lane() -> None:
    state = _state(_p10_config(vehicle_ids=("CFV_X",)))
    current = state.vehicle_states["CFV_X"]
    buffer = _buffer(
        state,
        "CFV_X",
        longitudinal=_longitudinal(state, "CFV_X", x_global=6851.0, v=14.0, a=0.2),
    )

    result = commit_step(
        state,
        _command_buffer(state),
        buffer,
        run_id="P10-NO-LATERAL",
        scenario_id="P10-NO-LATERAL",
    )

    candidate = result.final_candidates["CFV_X"]
    committed = result.next_state.vehicle_states["CFV_X"]
    assert candidate.x_global == 6851.0
    assert candidate.y == current.y
    assert candidate.source_lateral_candidate is None
    assert committed.physical_lane == current.physical_lane
    assert committed.road_role == current.road_role
    assert committed.lane_change_state == current.lane_change_state
    assert committed.merge_state == current.merge_state


def test_p10_missing_longitudinal_candidate_requires_identity_or_diagnostic_fallback() -> None:
    state = _state(_p10_config(vehicle_ids=("CFV_X",)))
    buffer = _buffer(
        state,
        "CFV_X",
        lateral=_lateral(state, "CFV_X", y=3.5, target_y=3.5),
        progress=_progress(state, "CFV_X", "lane_change", progress=0.5, completed=False),
    )

    result = commit_step(
        state,
        _command_buffer(state),
        buffer,
        run_id="P10-MISSING-LONG",
        scenario_id="P10-MISSING-LONG",
    )

    candidate = result.final_candidates["CFV_X"]
    assert candidate.source == "identity_candidate_for_commit_infrastructure"
    assert candidate.x_global == state.vehicle_states["CFV_X"].x_global
    assert candidate.y == state.vehicle_states["CFV_X"].y
    assert _warning(result, "missing_longitudinal_candidate").vehicle_id == "CFV_X"
    warning_event = _event(
        result.history.event_records,
        event_type="engineering_patch",
        vehicle_id="CFV_X",
    )
    assert warning_event.payload["warning_type"] == "missing_longitudinal_candidate"


def test_p10_mvs_commit_1_full_one_commit_per_vehicle_and_duplicate_guard() -> None:
    state = _state(_p10_config(vehicle_ids=("MV_E2E",)))
    explicit = CandidateKinematics(
        candidate_id="explicit-final",
        vehicle_id="MV_E2E",
        x_global=6842.0,
        y=-3.4,
        v=12.0,
        a=0.0,
        source="step9_candidate_assembly",
    )
    buffer = _buffer(
        state,
        "MV_E2E",
        longitudinal=_longitudinal(state, "MV_E2E", x_global=6841.2, v=12.0),
        lateral=_lateral(state, "MV_E2E", y=-3.42, target_y=0.0),
        explicit_candidates=(explicit,),
    )

    result = commit_step(
        state,
        _command_buffer(state),
        buffer,
        run_id="P10-DUPLICATE",
        scenario_id="P10-DUPLICATE",
    )

    duplicate = _sanity(result, "multiple_commit_for_one_vehicle")
    assert duplicate.result == "fail"
    assert duplicate.vehicle_ids == ("MV_E2E",)
    assert "MV_E2E" not in result.next_state.active_vehicle_ids
    assert not [
        record
        for record in result.history.event_records
        if record.event_type == "commit" and record.vehicle_id == "MV_E2E"
    ]


def test_p10_lane_change_in_progress_commit_persists_active_maneuver() -> None:
    state = _state(
        _p10_config(
            vehicle_ids=("CFV_X",),
            lane_change_state="executing",
            active_maneuvers=[
                _maneuver(
                    "CFV_X",
                    "lane_change",
                    start_x_global=6800.0,
                    start_y=0.0,
                    target_lane="lane_1",
                    target_y=3.5,
                    planned_length=100.0,
                    progress=0.4,
                )
            ],
        )
    )
    buffer = _buffer(
        state,
        "CFV_X",
        longitudinal=_longitudinal(state, "CFV_X", x_global=6851.0, v=10.0),
        lateral=_lateral(state, "CFV_X", y=1.2, target_y=3.5),
        progress=_progress(
            state,
            "CFV_X",
            "lane_change",
            progress=0.55,
            completed=False,
            source_command_id="p07:0:CFV_X:lane_change",
        ),
    )

    result = commit_step(
        state,
        _command_buffer(state),
        buffer,
        run_id="P10-LC-PROGRESS",
        scenario_id="P10-LC-PROGRESS",
    )

    committed = result.next_state.vehicle_states["CFV_X"]
    active = result.next_state.active_maneuvers["CFV_X"]
    assert committed.x_global == 6851.0
    assert committed.y == 1.2
    assert committed.physical_lane == "lane_2"
    assert committed.lane_change_state == "executing"
    assert active.progress == 0.55
    assert active.last_planning_speed == 10.0
    assert active.source_command_id == "p07:0:CFV_X:lane_change"


def test_p10_lane_change_completion_commit_applies_lane_state_and_cleans_active_maneuver() -> None:
    state = _state(
        _p10_config(
            vehicle_ids=("CFV_X",),
            lane_change_state="executing",
            active_maneuvers=[
                _maneuver(
                    "CFV_X",
                    "lane_change",
                    start_x_global=6800.0,
                    start_y=0.0,
                    target_lane="lane_1",
                    target_y=3.5,
                    planned_length=100.0,
                    progress=0.99,
                )
            ],
        )
    )
    buffer = _buffer(
        state,
        "CFV_X",
        longitudinal=_longitudinal(state, "CFV_X", x_global=6852.0, v=10.0),
        lateral=_lateral(state, "CFV_X", y=3.5, target_y=3.5),
        progress=_progress(state, "CFV_X", "lane_change", progress=1.0, completed=True),
        lane_state=CandidateLaneState(
            candidate_id="p09:0:CFV_X:lane_state",
            vehicle_id="CFV_X",
            physical_lane="lane_1",
            road_role="mainline",
            reason="lane_change_target_y_reached",
        ),
        transitions=(
            CandidateStateTransition(
                candidate_id="p09:0:CFV_X:lane_change_state",
                vehicle_id="CFV_X",
                state_name="lane_change_state",
                old_state="executing",
                new_state="normal",
                reason="lane_change_target_y_reached",
            ),
        ),
    )

    result = commit_step(
        state,
        _command_buffer(state),
        buffer,
        run_id="P10-LC-COMPLETE",
        scenario_id="P10-LC-COMPLETE",
    )

    committed = result.next_state.vehicle_states["CFV_X"]
    event = _event(result.history.event_records, event_type="commit", vehicle_id="CFV_X")
    assert committed.y == 3.5
    assert committed.physical_lane == "lane_1"
    assert committed.lane_change_state == "normal"
    assert "CFV_X" not in result.next_state.active_maneuvers
    assert event.payload["source_state_transition"] == "p09:0:CFV_X:lane_change_state"
    assert event.payload["active_maneuver_cleanup_vehicle_ids"] == ["CFV_X"]


def test_p10_merge_in_progress_commit_persists_active_maneuver_without_rejudging_eq53() -> None:
    state = _state(
        _p10_config(
            vehicle_ids=("MV_E2E",),
            merge_state="executing",
            active_maneuvers=[
                _maneuver(
                    "MV_E2E",
                    "merge",
                    start_x_global=6840.0,
                    start_y=-3.5,
                    target_lane="lane_2",
                    target_y=0.0,
                    planned_length=120.0,
                    progress=0.2,
                )
            ],
        )
    )
    buffer = _buffer(
        state,
        "MV_E2E",
        longitudinal=_longitudinal(state, "MV_E2E", x_global=6841.0, v=8.0),
        lateral=_lateral(state, "MV_E2E", y=-3.0, target_y=0.0),
        progress=_progress(state, "MV_E2E", "merge", progress=0.3, completed=False),
    )

    result = commit_step(
        state,
        _command_buffer(state),
        buffer,
        run_id="P10-MERGE-PROGRESS",
        scenario_id="P10-MERGE-PROGRESS",
    )

    committed = result.next_state.vehicle_states["MV_E2E"]
    assert committed.y == -3.0
    assert committed.physical_lane == "on_ramp"
    assert committed.road_role == "on_ramp"
    assert committed.merge_state == "executing"
    assert result.next_state.active_maneuvers["MV_E2E"].progress == 0.3
    assert _forbidden_events_absent(
        result,
        {"CMC", "assignment_validation", "speed_cap", "longitudinal_model", "lateral_trajectory"},
    )


def test_p10_merge_completion_commit_applies_mainline_state_and_cache_cleanup() -> None:
    state = _state(
        _p10_config(
            vehicle_ids=("MV_E2E",),
            merge_state="executing",
            aps_cache={"MV_E2E": {"status": "valid", "case": "case_1"}},
            active_maneuvers=[
                _maneuver(
                    "MV_E2E",
                    "merge",
                    start_x_global=6840.0,
                    start_y=-3.5,
                    target_lane="lane_2",
                    target_y=0.0,
                    planned_length=120.0,
                    progress=0.99,
                )
            ],
        )
    )
    buffer = _buffer(
        state,
        "MV_E2E",
        longitudinal=_longitudinal(state, "MV_E2E", x_global=6850.0, v=10.0),
        lateral=_lateral(state, "MV_E2E", y=0.0, target_y=0.0),
        progress=_progress(state, "MV_E2E", "merge", progress=1.0, completed=True),
        lane_state=CandidateLaneState(
            candidate_id="p09:0:MV_E2E:lane_state",
            vehicle_id="MV_E2E",
            physical_lane="lane_2",
            road_role="mainline",
            reason="merge_target_y_reached",
        ),
        transitions=(
            CandidateStateTransition(
                candidate_id="p09:0:MV_E2E:merge_state",
                vehicle_id="MV_E2E",
                state_name="merge_state",
                old_state="executing",
                new_state="merged",
                reason="merge_target_y_reached",
            ),
        ),
        cache_updates=(
            CandidateCacheUpdate(
                candidate_id="p10:0:MV_E2E:cache_cleanup",
                cache_name="assignment_records_by_mv",
                owner_vehicle_id="MV_E2E",
                operation="cleanup",
                reason="merge_completed",
            ),
        ),
    )

    result = commit_step(
        state,
        _command_buffer(state),
        buffer,
        run_id="P10-MERGE-COMPLETE",
        scenario_id="P10-MERGE-COMPLETE",
    )

    committed = result.next_state.vehicle_states["MV_E2E"]
    event = _event(result.history.event_records, event_type="commit", vehicle_id="MV_E2E")
    assert committed.physical_lane == "lane_2"
    assert committed.road_role == "mainline"
    assert committed.merge_state == "merged"
    assert "MV_E2E" not in result.next_state.active_maneuvers
    assert "MV_E2E" not in result.next_state.assignment_records_by_mv
    assert event.payload["cache_cleanup_vehicle_ids"] == ["MV_E2E"]
    assert event.payload["active_maneuver_cleanup_vehicle_ids"] == ["MV_E2E"]


def test_p10_mvs_safe_1b_commits_p09_candidate_without_speed_cap_recomposition() -> None:
    state = _state(
        _p10_config(
            vehicle_ids=("MV_E2E",),
            merge_state="executing",
            active_maneuvers=[
                _maneuver(
                    "MV_E2E",
                    "merge",
                    start_x_global=6840.0,
                    start_y=-3.5,
                    target_lane="lane_2",
                    target_y=0.0,
                    planned_length=120.0,
                    progress=0.2,
                )
            ],
        )
    )
    buffer = _buffer(
        state,
        "MV_E2E",
        longitudinal=_longitudinal(
            state,
            "MV_E2E",
            x_global=6840.263,
            v=2.63,
            source_commands=("p05:0:speed_cap:MV_E2E",),
        ),
        lateral=_lateral(
            state,
            "MV_E2E",
            y=-3.45,
            target_y=0.0,
            source_commands=("p08:0:MV_E2E:longitudinal",),
        ),
        progress=_progress(state, "MV_E2E", "merge", progress=0.22, completed=False),
    )

    result = commit_step(
        state,
        _command_buffer(state),
        buffer,
        run_id="P10-SAFE-1B",
        scenario_id="P10-SAFE-1B",
    )

    candidate = result.final_candidates["MV_E2E"]
    event = _event(result.history.event_records, event_type="commit", vehicle_id="MV_E2E")
    assert candidate.constraints_applied == ("boundary_speed_cap",)
    assert event.payload["source_longitudinal_candidate"] == "p08:0:MV_E2E:longitudinal"
    assert event.payload["source_lateral_candidate"] == "p09:0:MV_E2E:lateral"
    assert event.payload["p08_source_commands"] == ["p05:0:speed_cap:MV_E2E"]
    assert _forbidden_events_absent(
        result,
        {"speed_cap", "longitudinal_model", "lateral_trajectory"},
    )


def test_p10_step10_output_history_records_commit_trajectory_event_sanity() -> None:
    state = _state(_p10_config(vehicle_ids=("MV_E2E",)))
    buffer = _buffer(
        state,
        "MV_E2E",
        longitudinal=_longitudinal(state, "MV_E2E", x_global=6841.2, v=12.0),
        lateral=_lateral(state, "MV_E2E", y=-3.42, target_y=0.0),
        progress=_progress(state, "MV_E2E", "merge", progress=0.1, completed=False),
    )

    result = commit_step(
        state,
        _command_buffer(state),
        buffer,
        run_id="P10-STEP10",
        scenario_id="P10-STEP10",
    )

    trajectory = result.history.trajectory_records[0]
    assert trajectory.vehicle_id == "MV_E2E"
    assert trajectory.x_global == result.next_state.vehicle_states["MV_E2E"].x_global
    assert trajectory.y == result.next_state.vehicle_states["MV_E2E"].y
    assert _event(result.history.event_records, event_type="commit", vehicle_id="MV_E2E")
    assert _event(result.history.event_records, event_type="information_integration")
    assert _sanity(result, "multiple_commit_for_one_vehicle").result == "pass"
    assert _sanity(result, "information_integration_does_not_rewrite_state").result == "pass"


def test_p10_does_not_rerun_aps_cmc_p06_p07_p08_or_p09() -> None:
    state = _state(_p10_config(vehicle_ids=("MV_E2E",)))
    buffer = _buffer(
        state,
        "MV_E2E",
        longitudinal=_longitudinal(state, "MV_E2E", x_global=6841.2, v=12.0),
        lateral=_lateral(state, "MV_E2E", y=-3.42, target_y=0.0),
        progress=_progress(state, "MV_E2E", "merge", progress=0.1, completed=False),
    )

    result = commit_step(
        state,
        _command_buffer(state),
        buffer,
        run_id="P10-NO-RERUN",
        scenario_id="P10-NO-RERUN",
    )

    assert _forbidden_events_absent(
        result,
        {
            "APS",
            "APS_candidate",
            "CMC",
            "assignment_validation",
            "cooperative_request",
            "conflict_resolution",
            "CUC",
            "longitudinal_model",
            "spacing_override_consumption",
            "speed_cap",
            "lateral_trajectory",
        },
    )


def test_p10_expected_png_features_register_commit_trajectory_and_completion_markers() -> None:
    state = _state(
        _p10_config(
            vehicle_ids=("CFV_X", "MV_E2E"),
            lane_change_state="executing",
            merge_state="executing",
            active_maneuvers=[
                _maneuver(
                    "CFV_X",
                    "lane_change",
                    start_x_global=6800.0,
                    start_y=0.0,
                    target_lane="lane_1",
                    target_y=3.5,
                    planned_length=100.0,
                    progress=0.99,
                ),
                _maneuver(
                    "MV_E2E",
                    "merge",
                    start_x_global=6840.0,
                    start_y=-3.5,
                    target_lane="lane_2",
                    target_y=0.0,
                    planned_length=120.0,
                    progress=0.2,
                ),
            ],
        )
    )
    buffer = NextStateBuffer(
        step=state.step,
        t=state.t,
        candidate_longitudinal=MappingProxyType(
            {
                "CFV_X": _longitudinal(state, "CFV_X", x_global=6852.0, v=10.0),
                "MV_E2E": _longitudinal(state, "MV_E2E", x_global=6841.0, v=8.0),
            }
        ),
        candidate_lateral=MappingProxyType(
            {
                "CFV_X": _lateral(state, "CFV_X", y=3.5, target_y=3.5),
                "MV_E2E": _lateral(state, "MV_E2E", y=-3.0, target_y=0.0),
            }
        ),
        candidate_maneuver_progress=MappingProxyType(
            {
                "CFV_X": _progress(state, "CFV_X", "lane_change", progress=1.0, completed=True),
                "MV_E2E": _progress(state, "MV_E2E", "merge", progress=0.3, completed=False),
            }
        ),
        candidate_lane_state=MappingProxyType(
            {
                "CFV_X": CandidateLaneState(
                    candidate_id="p09:0:CFV_X:lane_state",
                    vehicle_id="CFV_X",
                    physical_lane="lane_1",
                    road_role="mainline",
                    reason="lane_change_target_y_reached",
                )
            }
        ),
        candidate_state_transitions=MappingProxyType(
            {
                "CFV_X": (
                    CandidateStateTransition(
                        candidate_id="p09:0:CFV_X:lane_change_state",
                        vehicle_id="CFV_X",
                        state_name="lane_change_state",
                        old_state="executing",
                        new_state="normal",
                        reason="lane_change_target_y_reached",
                    ),
                )
            }
        ),
    )

    result = commit_step(
        state,
        _command_buffer(state),
        buffer,
        run_id="P10-PNG",
        scenario_id="P10-PNG",
    )

    feature_types = {feature["feature_type"] for feature in result.expected_png_features}
    assert {
        "commit_marker",
        "trajectory_quicklook",
        "lane_change_completed_marker",
        "active_maneuver_marker",
    }.issubset(feature_types)


def _state(config: dict[str, Any]) -> Any:
    workspace, _ = build_prefreeze_workspace_from_scenario(config)
    return freeze_simulation_state(workspace)


def _p10_config(
    *,
    vehicle_ids: tuple[str, ...],
    lane_change_state: str = "normal",
    merge_state: str = "not_started",
    active_maneuvers: list[dict[str, Any]] | None = None,
    aps_cache: dict[str, dict[str, Any]] | None = None,
) -> dict[str, Any]:
    vehicles = {
        "MV_E2E": _vehicle(
            "MV_E2E",
            "on_ramp",
            6840.0,
            -3.5,
            road_role="on_ramp",
            merge_state=merge_state,
            initial_v=16.0,
        ),
        "CFV_X": _vehicle(
            "CFV_X",
            "lane_2",
            6844.0,
            0.0,
            lane_change_state=lane_change_state,
            initial_v=20.0,
        ),
    }
    return {
        "scenario_id": "P10-STEP9-UNIT",
        "scenario_name": "P10 Step9 unit",
        "purpose": "Inline P10 Step9 integration tests",
        "test_level": "unit",
        "status": "required",
        "initial_time": {"t": 0.0, "step": 0, "dt": 0.1},
        "initial_vehicles": [vehicles[vehicle_id] for vehicle_id in vehicle_ids],
        "module_overrides": {
            "boundary_generation_enabled": False,
            "random_arrival_enabled": False,
            "random_vehicle_attributes_enabled": False,
        },
        "preloaded_assignments": [
            {
                "mv_id": vehicle_id,
                "aps_case": value.get("aps_case") or value.get("case") or "case_1",
                "clv_id": value.get("clv_id") or "CLV_X",
                "cfv_id": value.get("cfv_id") or "CFV_X",
                "created_at_step": 0,
            }
            for vehicle_id, value in (aps_cache or {}).items()
        ],
        "preloaded_state_machine_states": [],
        "preloaded_maneuver_trajectory_states": active_maneuvers or [],
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
    lane_change_state: str = "normal",
    merge_state: str = "none",
    initial_v: float = 20.0,
) -> dict[str, Any]:
    return {
        "vehicle_id": vehicle_id,
        "vehicle_type": "CAV",
        "compliance_state": "not_applicable",
        "initial_x_global": x_global,
        "initial_y": y,
        "initial_v": initial_v,
        "initial_a": 0.0,
        "physical_lane": lane,
        "road_role": road_role,
        "lane_change_state": lane_change_state,
        "merge_state": merge_state,
        "spec_overrides": {},
    }


def _maneuver(
    vehicle_id: str,
    maneuver_type: str,
    *,
    start_x_global: float,
    start_y: float,
    target_lane: str,
    target_y: float,
    planned_length: float,
    progress: float,
) -> dict[str, Any]:
    return {
        "vehicle_id": vehicle_id,
        "maneuver_type": maneuver_type,
        "start_step": 0,
        "start_t": 0.0,
        "start_x_global": start_x_global,
        "start_y": start_y,
        "target_lane": target_lane,
        "target_y": target_y,
        "planned_length": planned_length,
        "progress": progress,
    }


def _buffer(
    state: Any,
    vehicle_id: str,
    *,
    longitudinal: CandidateLongitudinalKinematics | None = None,
    lateral: CandidateLateralKinematics | None = None,
    progress: CandidateManeuverProgress | None = None,
    lane_state: CandidateLaneState | None = None,
    transitions: tuple[CandidateStateTransition, ...] = (),
    cache_updates: tuple[CandidateCacheUpdate, ...] = (),
    explicit_candidates: tuple[CandidateKinematics, ...] = (),
) -> NextStateBuffer:
    return NextStateBuffer(
        step=state.step,
        t=state.t,
        candidate_longitudinal=MappingProxyType(
            {vehicle_id: longitudinal} if longitudinal is not None else {}
        ),
        candidate_lateral=MappingProxyType({vehicle_id: lateral} if lateral is not None else {}),
        candidate_kinematics=MappingProxyType(
            {vehicle_id: explicit_candidates} if explicit_candidates else {}
        ),
        candidate_maneuver_progress=MappingProxyType(
            {vehicle_id: progress} if progress is not None else {}
        ),
        candidate_lane_state=MappingProxyType({vehicle_id: lane_state} if lane_state else {}),
        candidate_state_transitions=MappingProxyType({vehicle_id: transitions} if transitions else {}),
        candidate_cache_updates=cache_updates,
    )


def _longitudinal(
    state: Any,
    vehicle_id: str,
    *,
    x_global: float,
    v: float,
    a: float = 0.0,
    source_commands: tuple[str, ...] = ("p05:0:speed_cap:MV_E2E",),
) -> CandidateLongitudinalKinematics:
    return CandidateLongitudinalKinematics(
        candidate_id=f"p08:{state.step}:{vehicle_id}:longitudinal",
        vehicle_id=vehicle_id,
        x_global=x_global,
        v=v,
        a=a,
        candidate_speed=max(v, state.vehicle_states[vehicle_id].v),
        planning_speed=v,
        source="step7_longitudinal_model",
        constraints_applied=("boundary_speed_cap",),
        source_commands=source_commands,
    )


def _lateral(
    state: Any,
    vehicle_id: str,
    *,
    y: float,
    target_y: float,
    source_commands: tuple[str, ...] = (),
) -> CandidateLateralKinematics:
    return CandidateLateralKinematics(
        candidate_id=f"p09:{state.step}:{vehicle_id}:lateral",
        vehicle_id=vehicle_id,
        y=y,
        target_y=target_y,
        source="step8_lateral_trajectory",
        source_commands=source_commands,
    )


def _progress(
    state: Any,
    vehicle_id: str,
    maneuver_type: str,
    *,
    progress: float,
    completed: bool,
    source_command_id: str | None = None,
) -> CandidateManeuverProgress:
    return CandidateManeuverProgress(
        candidate_id=f"p09:{state.step}:{vehicle_id}:maneuver_progress",
        vehicle_id=vehicle_id,
        maneuver_type=maneuver_type,
        progress=progress,
        completed=completed,
        target_y_reached=completed,
        source_command_id=source_command_id,
    )


def _command_buffer(state: Any) -> CommandBuffer:
    return CommandBuffer(
        step=state.step,
        t=state.t,
        merge_commands=MappingProxyType(
            {
                "MV_E2E": {
                    "command_id": f"p05:{state.step}:merge_start:MV_E2E",
                    "command_type": "merge",
                    "vehicle_id": "MV_E2E",
                    "target_lane": "lane_2",
                    "target_y": 0.0,
                    "assigned_clv_id": "CLV_X",
                    "assigned_cfv_id": "CFV_X",
                    "does_not_rejudge_merge_start": True,
                    "source_speed_cap_command_id": f"p05:{state.step}:speed_cap:MV_E2E",
                }
            }
        ),
        lane_change_commands=MappingProxyType(
            {
                "CFV_X": {
                    "command_id": f"p07:{state.step}:CFV_X:lane_change",
                    "command_type": "lane_change",
                    "vehicle_id": "CFV_X",
                    "target_lane": "lane_1",
                    "target_y": 3.5,
                    "source_mv_id": "MV_E2E",
                }
            }
        ),
        speed_cap_commands=MappingProxyType(
            {
                "MV_E2E": (
                    {
                        "command_id": f"p05:{state.step}:speed_cap:MV_E2E",
                        "command_type": "speed_cap",
                        "vehicle_id": "MV_E2E",
                        "speed_cap": 2.63,
                    },
                )
            }
        ),
    )


def _event(records: list[Any], *, event_type: str, vehicle_id: str | None = None) -> Any:
    for record in records:
        if record.event_type != event_type:
            continue
        if vehicle_id is not None and record.vehicle_id != vehicle_id:
            continue
        return record
    raise AssertionError(f"missing event {event_type} {vehicle_id or ''}")


def _dict_event(records: list[dict[str, Any]], *, event_type: str, reason: str) -> dict[str, Any]:
    for record in records:
        if record.get("event_type") == event_type and record.get("reason") == reason:
            return record
    raise AssertionError(f"missing event {event_type} {reason}")


def _dict_has_event_type(records: list[dict[str, Any]], event_type: str) -> bool:
    return any(record.get("event_type") == event_type for record in records)


def _warning(result: Any, warning_type: str) -> Any:
    for warning in result.warnings:
        if warning.warning_type == warning_type:
            return warning
    raise AssertionError(f"missing warning {warning_type}")


def _sanity(result: Any, check_type: str) -> Any:
    for record in result.history.sanity_check_records:
        if record.check_type == check_type:
            return record
    raise AssertionError(f"missing sanity {check_type}")


def _forbidden_events_absent(result: Any, forbidden_event_types: set[str]) -> bool:
    return forbidden_event_types.isdisjoint(
        {record.event_type for record in result.history.event_records}
    )
