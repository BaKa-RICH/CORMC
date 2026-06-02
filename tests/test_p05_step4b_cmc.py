from __future__ import annotations

from types import MappingProxyType
from typing import Any

from cormc import build_prefreeze_workspace_from_scenario, freeze_simulation_state, refresh_relations_snapshot
from cormc.mvs import build_scenario_report, load_builtin_scenario, load_scenario_config, run_targeted_scenario
from cormc.mvs.runner import ScenarioRunResult, ScenarioRuntimeContext
from cormc.step4a_aps import EffectiveAssignmentThisStep
from cormc.step4b_cmc import run_step4b_cmc


def test_mvs_cmc_1_eq53_pass_starts_merge_contract() -> None:
    report = run_targeted_scenario("MVS-CMC-1")

    _assert_required_p05_pass(report)
    assert _matcher_result(report, "expected_event_counts").passed is True
    assert _registered_feature(report, "merge_start_marker")["renderer_status"] == "renderer_deferred"


def test_mvs_cmc_2_eq53_fail_waiting_contract() -> None:
    report = run_targeted_scenario("MVS-CMC-2")

    _assert_required_p05_pass(report)
    assert _matcher_result(report, "expected_event_counts").passed is True
    assert _registered_feature(report, "waiting_marker")["expected_visibility"] == "visible"


def test_mvs_assign_1_invalid_assignment_does_not_swap_actual_leader_follower_contract() -> None:
    report = run_targeted_scenario("MVS-ASSIGN-1")

    _assert_required_p05_pass(report)
    assert _matcher_result(report, "expected_event_counts").passed is True
    assert _registered_feature(report, "no_replacement_assignment_arrow")["expected_visibility"] == "not_visible"


def test_waiting_boundary_speed_cap_command_for_safe_1a_prereq_contract() -> None:
    report = run_targeted_scenario("MVS-SAFE-1A_waiting_cap")

    assert report.scenario_id == "MVS-SAFE-1A_waiting_cap"
    assert report.status == "required"
    assert report.classification == "required_passed"
    assert report.passed is True
    assert report.blocks_required_suite is False
    assert report.failure_reasons == []
    assert all(result.passed for result in report.matcher_results)


def test_executing_merge_continuation_does_not_rejudge_merge_start_contract() -> None:
    report = run_targeted_scenario("P05-EXECUTING-CONTINUATION")

    _assert_required_p05_pass(report)
    assert _matcher_result(report, "expected_event_counts").passed is True
    assert _registered_feature(report, "executing_continuation_marker")["expected_visibility"] == "visible"


def test_p05_consumes_p04_effective_assignment_without_rerunning_aps() -> None:
    config = load_scenario_config(_effective_assignment_precedence_config())
    workspace, _ = build_prefreeze_workspace_from_scenario(config)
    frozen = freeze_simulation_state(workspace)
    relations = refresh_relations_snapshot(frozen)
    effective_assignment = EffectiveAssignmentThisStep(
        mv_id="MV_EFFECTIVE",
        assignment=MappingProxyType(
            {
                "mv_id": "MV_EFFECTIVE",
                "clv_id": "CLV_EFFECTIVE_NEW",
                "cfv_id": "CFV_EFFECTIVE_NEW",
                "aps_case": "case_1",
                "col_clv": False,
                "col_cfv": False,
                "desired_spacing_override": None,
                "status": "valid",
                "source": "aps_updated_this_step",
                "valid_until_next_aps": True,
            }
        ),
        source="aps_updated_this_step",
        available_for_cooperative_request=True,
    )

    result = run_step4b_cmc(
        frozen,
        relations,
        config=config,
        effective_assignments={"MV_EFFECTIVE": effective_assignment},
    )
    report = build_scenario_report(
        ScenarioRuntimeContext(config=config),
        ScenarioRunResult(
            actual_events=result.actual_events,
            actual_sanity_checks=result.actual_sanity_checks,
            actual_png_artifacts=result.expected_png_features,
        ),
    )

    _assert_required_p05_pass(report)
    assert _matcher_result(report, "expected_event_counts").passed is True
    validation_event = _actual_event(
        result.actual_events,
        reason="assignment_validation",
        vehicle_id="MV_EFFECTIVE",
    )
    assert validation_event["payload"]["assignment_source"] == "effective_assignment_this_step"
    assert validation_event["payload"]["assigned_clv_id"] == "CLV_EFFECTIVE_NEW"
    assert validation_event["payload"]["assigned_cfv_id"] == "CFV_EFFECTIVE_NEW"


def test_p05_does_not_write_vehicle_state_before_commit() -> None:
    config = load_builtin_scenario("MVS-CMC-1")
    workspace, _ = build_prefreeze_workspace_from_scenario(config)
    frozen = freeze_simulation_state(workspace)
    relations = refresh_relations_snapshot(frozen)
    before_signature = _state_signature(frozen)

    result = run_step4b_cmc(frozen, relations, config=config)

    assert any(
        check["check_type"] == "no_write_before_commit" and check["result"] == "pass"
        for check in result.actual_sanity_checks
    )
    assert _state_signature(frozen) == before_signature


def test_assignment_invalid_marked_engineering_patch() -> None:
    report = run_targeted_scenario("MVS-ASSIGN-1")
    assignment_invalid_expected = _expected_event(report, "assignment_invalid")

    _assert_required_p05_pass(report)
    assert assignment_invalid_expected["source"] == "first_version_engineering_patch"
    assert assignment_invalid_expected["reason_code"] == "cfv_not_lane_2"
    assert assignment_invalid_expected["match"]["reason"] == "cfv_not_lane_2"


def _assert_required_p05_pass(report: Any) -> None:
    assert report.status == "required"
    assert report.classification == "required_passed"
    assert report.blocks_required_suite is False
    assert report.failure_reasons == []
    assert all(result.passed for result in report.matcher_results)


def _matcher_result(report: Any, name: str) -> Any:
    for matcher_result in report.matcher_results:
        if matcher_result.name == name:
            return matcher_result
    raise AssertionError(f"missing matcher result: {name}")


def _registered_feature(report: Any, feature_type: str) -> dict[str, Any]:
    for feature in report.registered_png_features:
        if feature["feature_type"] == feature_type:
            return feature
    raise AssertionError(f"missing registered feature: {feature_type}")


def _expected_event(report: Any, event_type: str) -> dict[str, Any]:
    for matcher_result in report.matcher_results:
        for issue in matcher_result.issues:
            expected = issue.expected
            if expected.get("event_type") == event_type:
                return expected
    config = load_builtin_scenario(report.scenario_id)
    for expected in config["expected_events"]:
        if expected.get("event_type") == event_type:
            return expected
    raise AssertionError(f"missing expected event: {event_type}")


def _actual_event(events: list[dict[str, Any]], *, reason: str, vehicle_id: str) -> dict[str, Any]:
    for event in events:
        if event.get("reason") == reason and event.get("vehicle_id") == vehicle_id:
            return event
    raise AssertionError(f"missing actual event: {reason} {vehicle_id}")


def _effective_assignment_precedence_config() -> dict[str, Any]:
    return {
        "scenario_id": "P05-EFFECTIVE-ASSIGNMENT-PRECEDENCE",
        "scenario_name": "P05 consumes P04 effective assignment before APS cache",
        "purpose": "P05 must prefer EffectiveAssignmentThisStep and must not rerun APS.",
        "test_level": "unit",
        "status": "required",
        "derivation_ref": ["P05-Step4B_CMC_AssignmentValidation_Eq53_BoundaryCap.md#6"],
        "initial_time": {"t": 0.0, "step": 0, "dt": 0.1},
        "initial_vehicles": [
            _vehicle("MV_EFFECTIVE", "on_ramp", 7000.0, -3.5, road_role="on_ramp_mv", merge_state="waiting"),
            _vehicle("CLV_EFFECTIVE_OLD", "lane_2", 7015.0, 0.0),
            _vehicle("CFV_EFFECTIVE_OLD", "lane_2", 6990.0, 0.0),
            _vehicle("CLV_EFFECTIVE_NEW", "lane_2", 7030.0, 0.0),
            _vehicle("CFV_EFFECTIVE_NEW", "lane_2", 6970.0, 0.0),
        ],
        "module_overrides": {
            "boundary_generation_enabled": False,
            "random_arrival_enabled": False,
            "random_vehicle_attributes_enabled": False,
            "ordinary_mainline_lane_change_enabled": False,
            "platoon_cmc_enabled": False,
            "mpc_lateral_tracking_enabled": False,
            "test_harness_overrides": {"source": "test_harness_override"},
        },
        "preloaded_assignments": [
            {
                "mv_id": "MV_EFFECTIVE",
                "clv_id": "CLV_EFFECTIVE_OLD",
                "cfv_id": "CFV_EFFECTIVE_OLD",
                "aps_case": "case_1",
                "col_clv": False,
                "col_cfv": False,
                "desired_spacing_override": None,
                "status": "valid",
                "created_at_t": -1.0,
                "created_at_step": -10,
                "source": "aps_cache",
                "valid_until_next_aps": True,
                "staleness_policy": "valid_until_next_aps",
            }
        ],
        "preloaded_state_machine_states": [],
        "preloaded_maneuver_trajectory_states": [],
        "expected_events": [
            {
                "event_type": "CMC",
                "required": True,
                "vehicle_ids": ["MV_EFFECTIVE", "CLV_EFFECTIVE_NEW", "CFV_EFFECTIVE_NEW"],
                "match": {
                    "assignment_source": "effective_assignment_this_step",
                    "assignment_valid": True,
                    "assigned_clv_id": "CLV_EFFECTIVE_NEW",
                    "assigned_cfv_id": "CFV_EFFECTIVE_NEW",
                },
                "reason_code": "assignment_validation",
                "source": "first_version_engineering_patch",
            },
            {
                "event_type": "CMC",
                "required": True,
                "vehicle_ids": ["MV_EFFECTIVE", "CLV_EFFECTIVE_NEW", "CFV_EFFECTIVE_NEW"],
                "match": {"eq53_pass": True, "fail_side": None},
                "reason_code": "eq53_gap",
                "source": "paper_formula",
            },
        ],
        "forbidden_events": [
            {
                "event_type": "CMC",
                "vehicle_ids": ["MV_EFFECTIVE", "CLV_EFFECTIVE_OLD", "CFV_EFFECTIVE_OLD"],
                "match": {
                    "assignment_source": "aps_cache",
                    "assigned_clv_id": "CLV_EFFECTIVE_OLD",
                    "assigned_cfv_id": "CFV_EFFECTIVE_OLD",
                },
                "source": "first_version_engineering_patch",
            }
        ],
        "expected_event_counts": [
            {
                "event_type": "APS",
                "vehicle_ids": ["MV_EFFECTIVE"],
                "expected_count": 0,
                "comparison": "exactly",
            },
            {
                "event_type": "assignment_cache",
                "vehicle_ids": ["MV_EFFECTIVE"],
                "expected_count": 0,
                "comparison": "exactly",
            },
        ],
        "expected_sanity_checks": [
            {
                "check_type": "assignment_invalid",
                "required": True,
                "expected_status": "pass",
                "vehicle_ids": ["MV_EFFECTIVE"],
            },
            {
                "check_type": "no_write_before_commit",
                "required": True,
                "expected_status": "pass",
                "vehicle_ids": ["MV_EFFECTIVE"],
            },
        ],
        "expected_png_features": [
            {
                "feature_type": "assigned_clv_cfv_marker",
                "required": True,
                "vehicle_ids": ["MV_EFFECTIVE", "CLV_EFFECTIVE_NEW", "CFV_EFFECTIVE_NEW"],
                "expected_visibility": "visible",
            }
        ],
    }


def _vehicle(
    vehicle_id: str,
    lane: str,
    x_global: float,
    y: float,
    *,
    road_role: str = "mainline",
    merge_state: str = "none",
) -> dict[str, Any]:
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
    )
