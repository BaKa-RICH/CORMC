from __future__ import annotations

from types import MappingProxyType
from typing import Any

from cormc import build_prefreeze_workspace_from_scenario, freeze_simulation_state, refresh_relations_snapshot
from cormc.step8_lateral import run_step8_lateral_trajectory_planning_speed_progress
from cormc.step9_11 import CandidateLongitudinalKinematics, CommandBuffer, NextStateBuffer


def test_p09_mvs_safe_1b_executing_cap_consumes_p08_planning_speed_for_lateral_progress() -> None:
    state, relations = _state_and_relations(
        _p09_config(
            mv_x=7060.0,
            merge_state="executing",
            preloaded_maneuvers=[
                _maneuver(
                    "MV_CUC",
                    "merge",
                    start_x_global=7040.0,
                    start_y=-3.5,
                    target_lane="lane_2",
                    target_y=0.0,
                    source_command_id="p05:0:merge_continue:MV_CUC",
                    planned_length=120.0,
                    progress=0.20,
                )
            ],
        )
    )
    command_buffer = CommandBuffer(
        step=state.step,
        t=state.t,
        merge_commands=MappingProxyType(
            {"MV_CUC": _merge_continue_command(state, "MV_CUC", cap_command_id="p05:0:speed_cap:MV_CUC")}
        ),
    )
    p08_buffer = _p08_buffer(state, "MV_CUC", planning_speed=2.63, constrained=True)

    result = run_step8_lateral_trajectory_planning_speed_progress(
        state,
        relations,
        command_buffer=command_buffer,
        p08_next_state_buffer=p08_buffer,
    )

    assert "MV_CUC" in result.next_state_buffer.candidate_lateral
    assert "MV_CUC" in result.next_state_buffer.candidate_maneuver_progress
    event = _event(result.actual_events, event_type="lateral_trajectory", vehicle_id="MV_CUC")
    assert event["payload"]["trajectory_consumed_speed"] == 2.63
    assert event["payload"]["trajectory_consumed_speed_source"] == "p08_planning_speed"
    assert event["payload"]["speed_cap_recomputed_by_p09"] is False
    assert event["payload"]["active_maneuver_was_reset"] is False
    assert not _has_event_type(result.actual_events, "speed_cap")
    assert not _has_event_type(result.actual_events, "longitudinal_model")
    assert _state_signature(state) == _state_signature(result.state)


def test_p09_mvs_safe_2_boundary_risk_is_observable_without_inventing_commit_strategy() -> None:
    state, relations = _state_and_relations(
        _p09_config(
            mv_x=7060.0,
            merge_state="executing",
            preloaded_maneuvers=[
                _maneuver(
                    "MV_CUC",
                    "merge",
                    start_x_global=7040.0,
                    start_y=-3.5,
                    target_lane="lane_2",
                    target_y=0.0,
                    planned_length=120.0,
                    progress=0.20,
                )
            ],
        )
    )
    command_buffer = CommandBuffer(
        step=state.step,
        t=state.t,
        merge_commands=MappingProxyType({"MV_CUC": _merge_continue_command(state, "MV_CUC")}),
    )

    result = run_step8_lateral_trajectory_planning_speed_progress(
        state,
        relations,
        command_buffer=command_buffer,
        p08_next_state_buffer=_p08_buffer(state, "MV_CUC", planning_speed=1.0),
        boundary_risk_diagnostics={
            "MV_CUC": {
                "risk_source_id": "p08:0:MV_CUC:boundary_risk",
                "risk_status": "cap_infeasible",
            }
        },
    )

    boundary = _sanity(result.actual_sanity_checks, "boundary_violation")
    assert boundary["result"] in {"warning", "fail"}
    assert boundary["payload"]["full_conservative_policy_implemented"] is False
    assert boundary["payload"]["true_state_written_by_p09"] is False
    event = _event(result.actual_events, event_type="lateral_trajectory", vehicle_id="MV_CUC")
    assert event["payload"]["boundary_risk_status"] == "cap_infeasible"
    assert _state_signature(state) == _state_signature(result.state)


def test_p09_mvs_cuc_1a_lateral_consumption_creates_lane2_to_lane1_lateral_candidate() -> None:
    state, relations = _state_and_relations(_p09_config())
    command_buffer = CommandBuffer(
        step=state.step,
        t=state.t,
        lane_change_commands=MappingProxyType({"CFV_X": _lane_change_command(state, "CFV_X")}),
        same_step_overlays=MappingProxyType({"CFV_X": _same_step_overlay(state, "CFV_X")}),
    )

    result = run_step8_lateral_trajectory_planning_speed_progress(
        state,
        relations,
        command_buffer=command_buffer,
        p08_next_state_buffer=_p08_buffer(state, "CFV_X", planning_speed=12.0),
    )

    candidate = result.next_state_buffer.candidate_lateral["CFV_X"]
    assert candidate.target_y == 3.5
    assert "p07:0:CFV_X:lane_change" in candidate.source_commands
    assert "p07:0:CFV_X:same_step_overlay" in candidate.source_commands
    event = _event(result.actual_events, event_type="lateral_trajectory", vehicle_id="CFV_X")
    assert event["payload"]["source_scenario_id"] == "MVS-CUC-1A_override_choice1"
    assert event["payload"]["target_lane"] == "lane_1"
    assert event["payload"]["cuc_rerun_by_p09"] is False


def test_p09_lane_change_overlay_id_is_not_consumed_when_same_step_overlay_missing() -> None:
    state, relations = _state_and_relations(_p09_config())
    command_buffer = CommandBuffer(
        step=state.step,
        t=state.t,
        lane_change_commands=MappingProxyType({"CFV_X": _lane_change_command(state, "CFV_X")}),
        same_step_overlays=MappingProxyType({}),
    )

    result = run_step8_lateral_trajectory_planning_speed_progress(
        state,
        relations,
        command_buffer=command_buffer,
        p08_next_state_buffer=_p08_buffer(state, "CFV_X", planning_speed=12.0),
    )

    candidate = result.next_state_buffer.candidate_lateral["CFV_X"]
    assert "p07:0:CFV_X:lane_change" in candidate.source_commands
    assert "p07:0:CFV_X:same_step_overlay" not in candidate.source_commands
    event = _event(result.actual_events, event_type="lateral_trajectory", vehicle_id="CFV_X")
    assert event["payload"]["source_overlay_id"] is None
    assert event["payload"]["declared_overlay_id"] == "p07:0:CFV_X:same_step_overlay"
    assert event["payload"]["same_step_overlay_consumed"] is False
    assert event["payload"]["same_step_overlay_missing"] is True


def test_p09_source_scenario_id_is_not_defaulted_without_explicit_trace() -> None:
    state, relations = _state_and_relations(_p09_config())
    command_buffer = CommandBuffer(
        step=state.step,
        t=state.t,
        lane_change_commands=MappingProxyType(
            {"CFV_X": _lane_change_command(state, "CFV_X", source_scenario_id=None)}
        ),
        same_step_overlays=MappingProxyType(
            {"CFV_X": _same_step_overlay(state, "CFV_X", source_scenario_id=None)}
        ),
    )

    result = run_step8_lateral_trajectory_planning_speed_progress(
        state,
        relations,
        command_buffer=command_buffer,
        p08_next_state_buffer=_p08_buffer(state, "CFV_X", planning_speed=12.0),
    )

    event = _event(result.actual_events, event_type="lateral_trajectory", vehicle_id="CFV_X")
    assert "source_scenario_id" not in event["payload"]
    assert event["payload"]["same_step_overlay_consumed"] is True


def test_p09_p05_merge_start_command_creates_on_ramp_to_lane2_lateral_candidate() -> None:
    state, relations = _state_and_relations(_p09_config())
    command_buffer = CommandBuffer(
        step=state.step,
        t=state.t,
        merge_commands=MappingProxyType({"MV_CUC": _merge_start_command(state, "MV_CUC")}),
    )

    result = run_step8_lateral_trajectory_planning_speed_progress(
        state,
        relations,
        command_buffer=command_buffer,
        p08_next_state_buffer=_p08_buffer(state, "MV_CUC", planning_speed=8.0),
    )

    candidate = result.next_state_buffer.candidate_lateral["MV_CUC"]
    assert candidate.target_y == 0.0
    assert "p05:0:merge_start:MV_CUC" in candidate.source_commands
    progress = result.next_state_buffer.candidate_maneuver_progress["MV_CUC"]
    assert progress.maneuver_type == "merge"
    event = _event(result.actual_events, event_type="lateral_trajectory", vehicle_id="MV_CUC")
    assert event["payload"]["target_lane"] == "lane_2"
    assert event["payload"]["merge_progress_payload_created"] is True
    assert event["payload"]["eq53_rejudged_by_p09"] is False


def test_p09_active_lane_change_continuation_uses_existing_maneuver_state_without_reset() -> None:
    state, relations = _state_and_relations(
        _p09_config(
            lane_change_state="executing",
            preloaded_maneuvers=[
                _maneuver(
                    "CFV_X",
                    "lane_change",
                    start_x_global=6800.0,
                    start_y=0.0,
                    target_lane="lane_1",
                    target_y=3.5,
                    source_command_id="p07:0:CFV_X:lane_change",
                    planned_length=100.0,
                    progress=0.40,
                )
            ],
        )
    )

    result = run_step8_lateral_trajectory_planning_speed_progress(
        state,
        relations,
        command_buffer=CommandBuffer(step=state.step, t=state.t),
        p08_next_state_buffer=_p08_buffer(state, "CFV_X", planning_speed=10.0),
        vehicle_ids=("CFV_X",),
    )

    event = _event(result.actual_events, event_type="lateral_trajectory", vehicle_id="CFV_X")
    assert event["reason"] == "lane_change_continuation"
    assert event["payload"]["start_x_global"] == 6800.0
    assert event["payload"]["start_y"] == 0.0
    assert event["payload"]["target_y"] == 3.5
    assert event["payload"]["active_maneuver_was_reset"] is False
    assert not _has_event_type(result.actual_events, "CUC")


def test_p09_active_merge_continuation_does_not_rejudge_eq53_or_boundary_cap() -> None:
    state, relations = _state_and_relations(
        _p09_config(
            merge_state="executing",
            preloaded_maneuvers=[
                _maneuver(
                    "MV_CUC",
                    "merge",
                    start_x_global=7040.0,
                    start_y=-3.5,
                    target_lane="lane_2",
                    target_y=0.0,
                    planned_length=120.0,
                    progress=0.20,
                )
            ],
        )
    )
    command_buffer = CommandBuffer(
        step=state.step,
        t=state.t,
        merge_commands=MappingProxyType({"MV_CUC": _merge_continue_command(state, "MV_CUC")}),
    )

    result = run_step8_lateral_trajectory_planning_speed_progress(
        state,
        relations,
        command_buffer=command_buffer,
        p08_next_state_buffer=_p08_buffer(state, "MV_CUC", planning_speed=6.0),
    )

    event = _event(result.actual_events, event_type="lateral_trajectory", vehicle_id="MV_CUC")
    assert event["reason"] == "merge_continuation"
    assert event["payload"]["does_not_rejudge_merge_start"] is True
    assert event["payload"]["eq53_rejudged_by_p09"] is False
    assert event["payload"]["boundary_cap_recomputed_by_p09"] is False
    assert not _has_event_type(result.actual_events, "CMC")


def test_p09_planning_speed_handoff_uses_p08_planning_speed_not_vehicle_v() -> None:
    state, relations = _state_and_relations(_p09_config(cfv_v=25.0))
    command_buffer = CommandBuffer(
        step=state.step,
        t=state.t,
        lane_change_commands=MappingProxyType({"CFV_X": _lane_change_command(state, "CFV_X")}),
        same_step_overlays=MappingProxyType({"CFV_X": _same_step_overlay(state, "CFV_X")}),
    )

    result = run_step8_lateral_trajectory_planning_speed_progress(
        state,
        relations,
        command_buffer=command_buffer,
        p08_next_state_buffer=_p08_buffer(state, "CFV_X", planning_speed=4.0),
    )

    event = _event(result.actual_events, event_type="lateral_trajectory", vehicle_id="CFV_X")
    assert event["payload"]["trajectory_consumed_speed"] == 4.0
    assert event["payload"]["vehicle_state_v"] == 25.0
    assert event["payload"]["used_vehicle_state_v_for_progress"] is False


def test_p09_missing_p08_planning_speed_does_not_fallback_to_vehicle_v() -> None:
    state, relations = _state_and_relations(_p09_config(cfv_v=25.0))
    command_buffer = CommandBuffer(
        step=state.step,
        t=state.t,
        lane_change_commands=MappingProxyType({"CFV_X": _lane_change_command(state, "CFV_X")}),
        same_step_overlays=MappingProxyType({"CFV_X": _same_step_overlay(state, "CFV_X")}),
    )

    result = run_step8_lateral_trajectory_planning_speed_progress(
        state,
        relations,
        command_buffer=command_buffer,
        p08_next_state_buffer=NextStateBuffer(step=state.step, t=state.t),
    )

    assert result.next_state_buffer.candidate_lateral == {}
    assert not _has_event_type(result.actual_events, "lateral_trajectory")


def test_p09_completion_detector_writes_candidates_only() -> None:
    state, relations = _state_and_relations(
        _p09_config(
            mv_x=7160.0,
            mv_y=-0.01,
            merge_state="executing",
            preloaded_maneuvers=[
                _maneuver(
                    "MV_CUC",
                    "merge",
                    start_x_global=7040.0,
                    start_y=-3.5,
                    target_lane="lane_2",
                    target_y=0.0,
                    planned_length=100.0,
                    progress=0.99,
                )
            ],
        )
    )
    before_signature = _state_signature(state)

    result = run_step8_lateral_trajectory_planning_speed_progress(
        state,
        relations,
        command_buffer=CommandBuffer(step=state.step, t=state.t),
        p08_next_state_buffer=_p08_buffer(state, "MV_CUC", planning_speed=20.0),
        vehicle_ids=("MV_CUC",),
    )

    progress = result.next_state_buffer.candidate_maneuver_progress["MV_CUC"]
    assert progress.completed is True
    assert progress.target_y_reached is True
    assert result.next_state_buffer.candidate_lane_state["MV_CUC"].physical_lane == "lane_2"
    transitions = result.next_state_buffer.candidate_state_transitions["MV_CUC"]
    assert any(item.state_name == "merge_state" and item.new_state == "merged" for item in transitions)
    event = _event(result.actual_events, event_type="lateral_trajectory", vehicle_id="MV_CUC")
    assert event["payload"]["true_state_written_by_p09"] is False
    assert _state_signature(state) == before_signature


def test_p09_no_ordinary_mainline_lane_change_without_p07_command() -> None:
    state, relations = _state_and_relations(_p09_config())

    result = run_step8_lateral_trajectory_planning_speed_progress(
        state,
        relations,
        command_buffer=CommandBuffer(step=state.step, t=state.t),
        p08_next_state_buffer=_p08_buffer(state, "CFV_X", planning_speed=10.0),
        vehicle_ids=("CFV_X",),
    )

    assert result.next_state_buffer.candidate_lateral == {}
    assert _sanity(result.actual_sanity_checks, "unexpected_ordinary_lane_change_attempt")["result"] == "pass"


def test_p09_does_not_rerun_aps_cmc_p06_p07_or_p08() -> None:
    state, relations = _state_and_relations(_p09_config())

    result = run_step8_lateral_trajectory_planning_speed_progress(
        state,
        relations,
        command_buffer=CommandBuffer(
            step=state.step,
            t=state.t,
            merge_commands=MappingProxyType({"MV_CUC": _merge_start_command(state, "MV_CUC")}),
        ),
        p08_next_state_buffer=_p08_buffer(state, "MV_CUC", planning_speed=8.0),
    )

    forbidden_event_types = {
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
    }
    assert forbidden_event_types.isdisjoint({event["event_type"] for event in result.actual_events})


def test_p09_does_not_create_longitudinal_candidates_or_execute_commit() -> None:
    state, relations = _state_and_relations(_p09_config())

    result = run_step8_lateral_trajectory_planning_speed_progress(
        state,
        relations,
        command_buffer=CommandBuffer(
            step=state.step,
            t=state.t,
            merge_commands=MappingProxyType({"MV_CUC": _merge_start_command(state, "MV_CUC")}),
        ),
        p08_next_state_buffer=_p08_buffer(state, "MV_CUC", planning_speed=8.0),
    )

    assert result.next_state_buffer.candidate_longitudinal == {}
    assert result.next_state_buffer.candidate_kinematics == {}
    assert not _has_event_type(result.actual_events, "commit")


def test_p09_no_write_before_commit_and_expected_png_features() -> None:
    state, relations = _state_and_relations(_p09_config())
    before_signature = _state_signature(state)

    result = run_step8_lateral_trajectory_planning_speed_progress(
        state,
        relations,
        command_buffer=CommandBuffer(
            step=state.step,
            t=state.t,
            merge_commands=MappingProxyType({"MV_CUC": _merge_start_command(state, "MV_CUC")}),
        ),
        p08_next_state_buffer=_p08_buffer(state, "MV_CUC", planning_speed=8.0),
    )

    assert _state_signature(state) == before_signature
    assert _sanity(result.actual_sanity_checks, "no_write_before_commit")["result"] == "pass"
    for feature_type in (
        "merge_trajectory_marker",
        "maneuver_progress_marker",
        "planning_speed_consumption_marker",
    ):
        assert _png_feature(result.expected_png_features, feature_type)["expected_visibility"] in {
            "visible",
            "optional",
        }


def _p09_config(
    *,
    cv_type: str = "CAV",
    compliance_state: str = "not_applicable",
    lane_change_state: str = "normal",
    merge_state: str = "not_started",
    cfv_x: float = 6844.0,
    cfv_y: float = 0.0,
    cfv_v: float = 20.0,
    mv_x: float = 6840.0,
    mv_y: float = -3.5,
    mv_v: float = 16.0,
    preloaded_maneuvers: list[dict[str, Any]] | None = None,
) -> dict[str, Any]:
    return {
        "scenario_id": "P09-STEP8-UNIT",
        "scenario_name": "P09 Step8 unit",
        "purpose": "Inline P09 Step8 tests",
        "test_level": "unit",
        "status": "required",
        "initial_time": {"t": 0.0, "step": 0, "dt": 0.1},
        "initial_vehicles": [
            _vehicle(
                "MV_CUC",
                "on_ramp",
                mv_x,
                mv_y,
                road_role="on_ramp_mv",
                merge_state=merge_state,
                initial_v=mv_v,
            ),
            _vehicle(
                "CFV_X",
                "lane_2",
                cfv_x,
                cfv_y,
                vehicle_type=cv_type,
                compliance_state=compliance_state,
                lane_change_state=lane_change_state,
                initial_v=cfv_v,
            ),
            _vehicle("LV_X", "lane_2", 6890.0, 0.0),
            _vehicle("FV_X", "lane_2", 6780.0, 0.0),
            _vehicle("TLV_X", "lane_1", 6900.0, 3.5),
            _vehicle("TFV_X", "lane_1", 6700.0, 3.5),
        ],
        "module_overrides": {
            "boundary_generation_enabled": False,
            "random_arrival_enabled": False,
            "random_vehicle_attributes_enabled": False,
            "ordinary_mainline_lane_change_enabled": False,
            "mpc_lateral_tracking_enabled": False,
        },
        "preloaded_assignments": [],
        "preloaded_state_machine_states": [],
        "preloaded_maneuver_trajectory_states": preloaded_maneuvers or [],
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
    vehicle_type: str = "CAV",
    compliance_state: str = "not_applicable",
    road_role: str = "mainline",
    lane_change_state: str = "normal",
    merge_state: str = "none",
    initial_v: float = 20.0,
) -> dict[str, Any]:
    return {
        "vehicle_id": vehicle_id,
        "vehicle_type": vehicle_type,
        "compliance_state": compliance_state,
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
    source_command_id: str | None = None,
    planned_length: float | None = None,
    progress: float = 0.0,
) -> dict[str, Any]:
    del source_command_id
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


def _state_and_relations(config: dict[str, Any]) -> tuple[Any, Any]:
    workspace, _ = build_prefreeze_workspace_from_scenario(config)
    state = freeze_simulation_state(workspace)
    relations = refresh_relations_snapshot(state)
    return state, relations


def _p08_buffer(
    state: Any,
    vehicle_id: str,
    *,
    planning_speed: float,
    constrained: bool = False,
) -> NextStateBuffer:
    current = state.vehicle_states[vehicle_id]
    candidate = CandidateLongitudinalKinematics(
        candidate_id=f"p08:{state.step}:{vehicle_id}:longitudinal",
        vehicle_id=vehicle_id,
        x_global=current.x_global + planning_speed * state.dt,
        v=planning_speed,
        a=0.0,
        candidate_speed=max(planning_speed, current.v),
        planning_speed=planning_speed,
        source="step7_longitudinal_model",
        constraints_applied=("boundary_speed_cap",) if constrained else (),
        source_commands=("p05:0:speed_cap:MV_CUC",) if constrained else (),
    )
    return NextStateBuffer(
        step=state.step,
        t=state.t,
        candidate_longitudinal=MappingProxyType({vehicle_id: candidate}),
    )


def _lane_change_command(
    state: Any,
    vehicle_id: str,
    *,
    source_scenario_id: str | None = "MVS-CUC-1A_override_choice1",
) -> dict[str, Any]:
    command = {
        "command_id": f"p07:{state.step}:{vehicle_id}:lane_change",
        "command_type": "lane_change",
        "module": "Step6CUC",
        "vehicle_id": vehicle_id,
        "source_request_id": f"p06:{state.step}:{vehicle_id}:MV_CUC:cfv",
        "source_mv_id": "MV_CUC",
        "source_lane": "lane_2",
        "target_lane": "lane_1",
        "target_y": 3.5,
        "cuc_decision_id": f"p07:{state.step}:{vehicle_id}:cuc_decision",
        "overlay_id": f"p07:{state.step}:{vehicle_id}:same_step_overlay",
        "init_maneuver": True,
    }
    if source_scenario_id is not None:
        command["source_scenario_id"] = source_scenario_id
    return command


def _same_step_overlay(
    state: Any,
    vehicle_id: str,
    *,
    source_scenario_id: str | None = "MVS-CUC-1A_override_choice1",
) -> dict[str, Any]:
    overlay = {
        "overlay_id": f"p07:{state.step}:{vehicle_id}:same_step_overlay",
        "vehicle_id": vehicle_id,
        "source_request_id": f"p06:{state.step}:{vehicle_id}:MV_CUC:cfv",
        "source_mv_id": "MV_CUC",
        "source": "first_version_engineering_patch",
        "reason": "same_step_cuc_lane_change_relation_overlay",
        "is_engineering_patch": True,
        "source_lane": "lane_2",
        "target_lane": "lane_1",
        "target_lane_neighbors": {"TLV_id": "TLV_X", "TFV_id": "TFV_X"},
        "cuc_decision_id": f"p07:{state.step}:{vehicle_id}:cuc_decision",
    }
    if source_scenario_id is not None:
        overlay["source_scenario_id"] = source_scenario_id
    return overlay


def _merge_start_command(state: Any, vehicle_id: str) -> dict[str, Any]:
    return {
        "command_id": f"p05:{state.step}:merge_start:{vehicle_id}",
        "vehicle_id": vehicle_id,
        "command_type": "merge",
        "init_or_continue_maneuver": "init",
        "target_lane": "lane_2",
        "target_y": 0.0,
        "assigned_clv_id": "LV_X",
        "assigned_cfv_id": "CFV_X",
        "source_speed_cap_command_id": f"p05:{state.step}:speed_cap:{vehicle_id}",
        "source": "first_version_engineering_patch",
        "is_engineering_patch": True,
    }


def _merge_continue_command(
    state: Any,
    vehicle_id: str,
    *,
    cap_command_id: str | None = None,
) -> dict[str, Any]:
    maneuver = state.active_maneuvers.get(vehicle_id)
    return {
        "command_id": f"p05:{state.step}:merge_continue:{vehicle_id}",
        "vehicle_id": vehicle_id,
        "command_type": "merge",
        "init_or_continue_maneuver": "continue",
        "target_lane": maneuver.target_lane if maneuver is not None else "lane_2",
        "target_y": maneuver.target_y if maneuver is not None else 0.0,
        "active_maneuver_present": maneuver is not None,
        "no_new_eq53_start_decision": True,
        "does_not_rejudge_merge_start": True,
        "source_speed_cap_command_id": cap_command_id or f"p05:{state.step}:speed_cap:{vehicle_id}",
        "source": "first_version_engineering_patch",
        "is_engineering_patch": True,
    }


def _event(events: list[dict[str, Any]], *, event_type: str, vehicle_id: str) -> dict[str, Any]:
    for event in events:
        if event.get("event_type") == event_type and event.get("vehicle_id") == vehicle_id:
            return event
    raise AssertionError(f"missing event: {event_type} {vehicle_id}")


def _sanity(checks: list[dict[str, Any]], check_type: str) -> dict[str, Any]:
    for check in checks:
        if check.get("check_type") == check_type:
            return check
    raise AssertionError(f"missing sanity: {check_type}")


def _png_feature(features: list[dict[str, Any]], feature_type: str) -> dict[str, Any]:
    for feature in features:
        if feature.get("feature_type") == feature_type:
            return feature
    raise AssertionError(f"missing png feature: {feature_type}")


def _has_event_type(events: list[dict[str, Any]], event_type: str) -> bool:
    return any(event.get("event_type") == event_type for event in events)


def _state_signature(state: Any) -> tuple[Any, ...]:
    return (
        state.t,
        state.step,
        state.dt,
        state.active_vehicle_ids,
        tuple(
            (
                vehicle_id,
                state.vehicle_states[vehicle_id].x_global,
                state.vehicle_states[vehicle_id].y,
                state.vehicle_states[vehicle_id].v,
                state.vehicle_states[vehicle_id].a,
                state.vehicle_states[vehicle_id].physical_lane,
                state.vehicle_states[vehicle_id].road_role,
                state.vehicle_states[vehicle_id].lane_change_state,
                state.vehicle_states[vehicle_id].merge_state,
            )
            for vehicle_id in state.active_vehicle_ids
        ),
        tuple(
            (
                vehicle_id,
                state.active_maneuvers[vehicle_id].maneuver_type,
                state.active_maneuvers[vehicle_id].start_step,
                state.active_maneuvers[vehicle_id].start_t,
                state.active_maneuvers[vehicle_id].start_x_global,
                state.active_maneuvers[vehicle_id].start_y,
                state.active_maneuvers[vehicle_id].target_lane,
                state.active_maneuvers[vehicle_id].target_y,
                state.active_maneuvers[vehicle_id].source_command_id,
                state.active_maneuvers[vehicle_id].source_event_id,
                state.active_maneuvers[vehicle_id].planned_length,
                state.active_maneuvers[vehicle_id].progress,
                state.active_maneuvers[vehicle_id].last_planning_speed,
                state.active_maneuvers[vehicle_id].assigned_clv_id,
                state.active_maneuvers[vehicle_id].assigned_cfv_id,
            )
            for vehicle_id in sorted(state.active_maneuvers)
        ),
        tuple((key, tuple(sorted(value.items()))) for key, value in state.aps_assignment_cache.items()),
    )
