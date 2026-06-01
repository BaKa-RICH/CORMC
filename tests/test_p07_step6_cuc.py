from __future__ import annotations

from typing import Any

from cormc import build_prefreeze_workspace_from_scenario, freeze_simulation_state, refresh_relations_snapshot
from cormc.step6_cuc import run_step6_cuc_choice_compliance_lane_change_overlay


def test_mvs_cuc_1a_override_choice1_generates_command_overlay_and_events() -> None:
    state, relations = _state_and_relations(_cuc_config())
    before_signature = _state_signature(state)

    result = run_step6_cuc_choice_compliance_lane_change_overlay(
        state,
        relations,
        active_requests={"CFV_X": _active_request("CFV_X")},
        utility_overrides={"CFV_X": {"recommended_choice": "change_to_lane_1"}},
    )

    decision = result.cuc_decisions["CFV_X"]
    assert decision["utility_source"] == "test_harness_override"
    assert decision["recommended_choice"] == "change_to_lane_1"
    assert decision["effective_choice"] == "change_to_lane_1"
    assert "CFV_X" in result.command_buffer.lane_change_commands
    assert "CFV_X" in result.command_buffer.state_transition_commands
    assert "CFV_X" in result.command_buffer.same_step_overlays
    overlay = result.command_buffer.same_step_overlays["CFV_X"]
    assert overlay["source"] == "first_version_engineering_patch"
    assert overlay["reason"] == "same_step_cuc_lane_change_relation_overlay"
    assert overlay["is_engineering_patch"] is True
    assert _actual_event(result.actual_events, module="Step6CUC")["payload"]["final_choice"] == "change_to_lane_1"
    assert _registered_feature(result.expected_png_features, "same_step_overlay_marker")["expected_visibility"] == "visible"
    assert _state_signature(state) == before_signature


def test_mvs_cuc_2_target_lane_unsafe_fallback_stay_lane2_and_spacing_handoff() -> None:
    state, relations = _state_and_relations(_cuc_config(tlv_x=6854.0, tfv_x=6837.0))

    result = run_step6_cuc_choice_compliance_lane_change_overlay(
        state,
        relations,
        active_requests={"CFV_X": _active_request("CFV_X", desired_spacing_override=58.0)},
        utility_overrides={"CFV_X": {"recommended_choice": "change_to_lane_1"}},
    )

    decision = result.cuc_decisions["CFV_X"]
    assert decision["effective_choice"] == "stay_lane_2"
    assert decision["fallback_reason"] == "target_lane_unsafe"
    assert decision["fallback_reason"] != "target_lane_TT_unsafe"
    assert result.command_buffer.lane_change_commands == {}
    assert result.command_buffer.same_step_overlays == {}
    spacing_command = result.command_buffer.cooperation_commands["CFV_X"]
    assert spacing_command["eq10_desired_spacing"] == 58.0
    assert spacing_command["consumed_by"] == "P08"
    assert spacing_command["p07_longitudinal_candidate_created"] is False
    assert result.command_buffer.longitudinal_commands == {}
    safety_event = _actual_event(result.actual_events, module="Step6TargetLaneSafety")
    assert safety_event["payload"]["target_lane_safe"] is False
    assert safety_event["payload"]["fallback_reason"] == "target_lane_unsafe"
    assert safety_event["payload"]["target_lane_safety_method"] == "first_version_gap_over_cv_speed_proxy"
    assert safety_event["payload"]["eq14_eq15_locked"] is False


def test_mvs_cuc_3_non_compliant_chv_ignores_cuc_without_spacing_consumption() -> None:
    state, relations = _state_and_relations(_cuc_config(cv_type="CHV", compliance_state="non_compliant"))

    result = run_step6_cuc_choice_compliance_lane_change_overlay(
        state,
        relations,
        active_requests={"CFV_X": _active_request("CFV_X", desired_spacing_override=58.0)},
        utility_overrides={"CFV_X": {"recommended_choice": "change_to_lane_1"}},
    )

    decision = result.cuc_decisions["CFV_X"]
    assert decision["effective_choice"] == "not_applicable"
    assert decision["accepted_by_vehicle"] is False
    assert decision["fallback_reason"] == "non_compliant_chv"
    assert "not_executed" not in decision.values()
    assert result.command_buffer.lane_change_commands == {}
    assert result.command_buffer.cooperation_commands == {}
    compliance_event = _actual_event(result.actual_events, module="Step6CUCCompliance")
    assert compliance_event["payload"]["cuc_suggestion_executed"] is False
    assert compliance_event["payload"]["spacing_override_consumed_by_p07"] is False


def test_mvs_cuc_1b_real_utility_probe_logs_inputs_u1_u2_and_final_choice() -> None:
    state, relations = _state_and_relations(_cuc_config())

    result = run_step6_cuc_choice_compliance_lane_change_overlay(
        state,
        relations,
        active_requests={"CFV_X": _active_request("CFV_X")},
    )

    decision = result.cuc_decisions["CFV_X"]
    assert decision["utility_source"] == "real_CUC"
    assert decision["utility_formula_status"] == "first_version_probe_not_eq11_eq12_locked"
    assert decision["eq11_eq12_locked"] is False
    assert decision["utility_inputs_logged"] is True
    assert decision["U1"] is not None
    assert decision["U2"] is not None
    assert decision["final_choice"] in {"change_to_lane_1", "stay_lane_2", "not_applicable"}


def test_p07_consumes_only_p06_active_requests_and_ignores_suppressed_loser() -> None:
    state, relations = _state_and_relations(_cuc_config())

    result = run_step6_cuc_choice_compliance_lane_change_overlay(
        state,
        relations,
        active_requests={"CFV_X": _active_request("CFV_X")},
        suppressed_requests=(
            {
                **_active_request("CFV_X", source_mv_id="MV_LOSER"),
                "request_id": "p06:0:CFV_X:MV_LOSER:cfv",
                "active": False,
            },
        ),
        utility_overrides={"CFV_X": {"recommended_choice": "change_to_lane_1"}},
    )

    assert set(result.cuc_decisions) == {"CFV_X"}
    assert all(event["payload"].get("source_mv_id") != "MV_LOSER" for event in result.actual_events)
    assert result.suppressed_requests_ignored_as_active_input is True


def test_p07_no_active_request_no_cuc() -> None:
    state, relations = _state_and_relations(_cuc_config())

    result = run_step6_cuc_choice_compliance_lane_change_overlay(
        state,
        relations,
        active_requests={},
    )

    assert result.cuc_decisions == {}
    assert result.command_buffer.lane_change_commands == {}
    assert result.command_buffer.cooperation_commands == {}
    assert result.command_buffer.same_step_overlays == {}
    assert _actual_event(result.actual_events, reason="no_active_request_no_cuc")


def test_p07_active_lane_change_cv_skips_cuc_and_no_duplicate_command() -> None:
    state, relations = _state_and_relations(_cuc_config(lane_change_state="executing"))

    result = run_step6_cuc_choice_compliance_lane_change_overlay(
        state,
        relations,
        active_requests={"CFV_X": _active_request("CFV_X")},
        utility_overrides={"CFV_X": {"recommended_choice": "change_to_lane_1"}},
    )

    decision = result.cuc_decisions["CFV_X"]
    assert decision["effective_choice"] == "not_applicable"
    assert decision["fallback_reason"] == "already_executing_lane_change"
    assert result.command_buffer.lane_change_commands == {}
    assert result.command_buffer.state_transition_commands == {}
    assert _actual_sanity(result.actual_sanity_checks, reason="active_lane_change_skip_cuc")["result"] == "pass"


def test_p07_does_not_rerun_aps_cmc_or_p06_conflict_resolution() -> None:
    state, relations = _state_and_relations(_cuc_config())

    result = run_step6_cuc_choice_compliance_lane_change_overlay(
        state,
        relations,
        active_requests={"CFV_X": _active_request("CFV_X")},
    )

    forbidden_event_types = {"APS", "APS_candidate", "CMC", "assignment_validation", "cooperative_request", "conflict_resolution"}
    assert forbidden_event_types.isdisjoint({event["event_type"] for event in result.actual_events})
    assert result.command_buffer.merge_commands == {}
    assert result.command_buffer.speed_cap_commands == {}


def test_p07_does_not_create_longitudinal_or_lateral_candidates() -> None:
    state, relations = _state_and_relations(_cuc_config())

    result = run_step6_cuc_choice_compliance_lane_change_overlay(
        state,
        relations,
        active_requests={"CFV_X": _active_request("CFV_X")},
        utility_overrides={"CFV_X": {"recommended_choice": "change_to_lane_1"}},
    )

    assert result.longitudinal_candidates == {}
    assert result.lateral_candidates == {}
    assert result.command_buffer.longitudinal_commands == {}


def test_p07_cuc_decision_not_persistent_and_no_write_before_commit() -> None:
    state, relations = _state_and_relations(_cuc_config())
    before_signature = _state_signature(state)

    result = run_step6_cuc_choice_compliance_lane_change_overlay(
        state,
        relations,
        active_requests={"CFV_X": _active_request("CFV_X")},
    )

    assert _state_signature(state) == before_signature
    assert all("cuc_choice" not in vars(vehicle) for vehicle in state.vehicle_states.values())
    assert "CFV_X" not in result.command_buffer.cuc_decisions
    assert _actual_sanity(result.actual_sanity_checks, reason="p07_no_write_before_commit")["result"] == "pass"
    assert _actual_sanity(result.actual_sanity_checks, reason="cuc_decision_not_persistent")["result"] == "pass"


def _cuc_config(
    *,
    cv_type: str = "CAV",
    compliance_state: str = "not_applicable",
    lane_change_state: str = "normal",
    tlv_x: float = 6900.0,
    tfv_x: float = 6700.0,
) -> dict[str, Any]:
    return {
        "scenario_id": "P07-CUC-UNIT",
        "scenario_name": "P07 CUC unit",
        "purpose": "Inline P07 Step6 tests",
        "test_level": "unit",
        "status": "required",
        "initial_time": {"t": 0.0, "step": 0, "dt": 0.1},
        "initial_vehicles": [
            _vehicle("MV_CUC", "on_ramp", 6840.0, -3.5, road_role="on_ramp_mv", merge_state="not_started"),
            _vehicle("CFV_X", "lane_2", 6844.0, 0.0, vehicle_type=cv_type, compliance_state=compliance_state, lane_change_state=lane_change_state),
            _vehicle("LV_X", "lane_2", 6890.0, 0.0),
            _vehicle("FV_X", "lane_2", 6780.0, 0.0),
            _vehicle("TLV_X", "lane_1", tlv_x, 3.5),
            _vehicle("TFV_X", "lane_1", tfv_x, 3.5),
        ],
        "module_overrides": {
            "boundary_generation_enabled": False,
            "random_arrival_enabled": False,
            "random_vehicle_attributes_enabled": False,
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
    vehicle_type: str = "CAV",
    compliance_state: str = "not_applicable",
    road_role: str = "mainline",
    lane_change_state: str = "normal",
    merge_state: str = "none",
) -> dict[str, Any]:
    return {
        "vehicle_id": vehicle_id,
        "vehicle_type": vehicle_type,
        "compliance_state": compliance_state,
        "initial_x_global": x_global,
        "initial_y": y,
        "initial_v": 20.0,
        "initial_a": 0.0,
        "physical_lane": lane,
        "road_role": road_role,
        "lane_change_state": lane_change_state,
        "merge_state": merge_state,
        "spec_overrides": {},
    }


def _active_request(
    cv_id: str,
    *,
    source_mv_id: str = "MV_CUC",
    desired_spacing_override: float | None = None,
) -> dict[str, Any]:
    return {
        "request_id": f"p06:0:{cv_id}:{source_mv_id}:cfv",
        "source_mv_id": source_mv_id,
        "cv_id": cv_id,
        "cv_role": "cfv",
        "col": True,
        "aps_case": "case_2",
        "assignment_source": "aps_updated_this_step",
        "t_mv_star": 5.5,
        "mv_in_merging_zone": False,
        "mv_distance_to_x0_m": 110.0,
        "desired_spacing_override": desired_spacing_override,
        "active": True,
        "source_conflict_id": None,
    }


def _state_and_relations(config: dict[str, Any]) -> tuple[Any, Any]:
    workspace, _ = build_prefreeze_workspace_from_scenario(config)
    state = freeze_simulation_state(workspace)
    relations = refresh_relations_snapshot(state)
    return state, relations


def _actual_event(
    events: list[dict[str, Any]],
    *,
    module: str | None = None,
    reason: str | None = None,
) -> dict[str, Any]:
    for event in events:
        if module is not None and event.get("module") != module:
            continue
        if reason is not None and event.get("reason") != reason:
            continue
        return event
    raise AssertionError(f"missing actual event: module={module} reason={reason}")


def _actual_sanity(checks: list[dict[str, Any]], *, reason: str) -> dict[str, Any]:
    for check in checks:
        if check.get("reason") == reason:
            return check
    raise AssertionError(f"missing sanity check: {reason}")


def _registered_feature(features: list[dict[str, Any]], feature_type: str) -> dict[str, Any]:
    for feature in features:
        if feature["feature_type"] == feature_type:
            return feature
    raise AssertionError(f"missing registered feature: {feature_type}")


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
        tuple((key, tuple(sorted(value.items()))) for key, value in state.aps_assignment_cache.items()),
    )
