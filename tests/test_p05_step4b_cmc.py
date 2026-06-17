from __future__ import annotations

from types import MappingProxyType
from typing import Any

from cormc.simulation_core.assignment_lifecycle import AssignmentStepView
from cormc.simulation_core.pre_freeze import build_prefreeze_workspace_from_scenario, freeze_simulation_state, refresh_relations_snapshot
from cormc.scenario_schema import build_scenario_report, load_scenario_config
from cormc.scenario_schema.reporting import ScenarioRunResult, ScenarioRuntimeContext
from cormc.simulation_core.cmc import run_step4b_cmc


def test_p05_consumes_lifecycle_view_without_rerunning_aps() -> None:
    config = load_scenario_config(_effective_assignment_precedence_config())
    workspace, _ = build_prefreeze_workspace_from_scenario(config)
    frozen = freeze_simulation_state(workspace)
    relations = refresh_relations_snapshot(frozen)
    assignment_view = AssignmentStepView(
        mv_id="MV_EFFECTIVE",
        record=MappingProxyType(
            {
                "record_version": 1,
                "mv_id": "MV_EFFECTIVE",
                "clv_id": "CLV_EFFECTIVE_NEW",
                "cfv_id": "CFV_EFFECTIVE_NEW",
                "gap_type": "bounded",
                "aps_case": "case_1",
                "col_clv": False,
                "col_cfv": False,
                "desired_spacing_override": None,
                "status": "valid",
                "lifecycle_state": "active_control_zone",
                "source": "aps_updated_this_step",
                "valid_until_next_aps": True,
            }
        ),
        source="aps_updated_this_step",
        consumable_by_step5=True,
        consumable_by_cmc=True,
    )

    result = run_step4b_cmc(
        frozen,
        relations,
        config=config,
        assignment_views={"MV_EFFECTIVE": assignment_view},
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
    assert validation_event["payload"]["assignment_source"] == "aps_updated_this_step"
    assert validation_event["payload"]["assigned_clv_id"] == "CLV_EFFECTIVE_NEW"
    assert validation_event["payload"]["assigned_cfv_id"] == "CFV_EFFECTIVE_NEW"


def test_p05_does_not_write_vehicle_state_before_commit() -> None:
    config = load_scenario_config(_effective_assignment_precedence_config())
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


def test_p05_rejects_executing_clv_and_cfv_with_structured_reason() -> None:
    for role, expected_reason in (
        ("clv", "clv_lane_change_executing"),
        ("cfv", "cfv_lane_change_executing"),
    ):
        config = _executing_boundary_invalid_config(role)
        workspace, _ = build_prefreeze_workspace_from_scenario(config)
        frozen = freeze_simulation_state(workspace)
        relations = refresh_relations_snapshot(frozen)

        result = run_step4b_cmc(frozen, relations, config=config)
        validation_event = _actual_event(
            result.actual_events,
            reason="assignment_validation",
            vehicle_id="MV_EXEC_BOUNDARY",
        )
        invalid_event = _actual_event(
            result.actual_events,
            reason=expected_reason,
            vehicle_id="MV_EXEC_BOUNDARY",
        )

        assert validation_event["payload"]["assignment_valid"] is False
        assert validation_event["payload"]["invalid_reason"] == expected_reason
        assert invalid_event["event_type"] == "assignment_invalid"
        assert result.command_buffer.cache_update_commands[0]["operation"] == "update"
        assert (
            result.command_buffer.cache_update_commands[0]["new_value"]["lifecycle_state"]
            == "recovery_required"
        )


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


def _actual_event(events: list[dict[str, Any]], *, reason: str, vehicle_id: str) -> dict[str, Any]:
    for event in events:
        if event.get("reason") == reason and event.get("vehicle_id") == vehicle_id:
            return event
    raise AssertionError(f"missing actual event: {reason} {vehicle_id}")


def _effective_assignment_precedence_config() -> dict[str, Any]:
    return {
        "scenario_id": "P05-EFFECTIVE-ASSIGNMENT-PRECEDENCE",
        "scenario_name": "P05 consumes P04 effective assignment before APS cache",
        "purpose": "P05 must prefer lifecycle view and must not rerun APS.",
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
                    "assignment_source": "aps_updated_this_step",
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


def _executing_boundary_invalid_config(role: str) -> dict[str, Any]:
    clv_state = "executing" if role == "clv" else "normal"
    cfv_state = "executing" if role == "cfv" else "normal"
    return {
        "scenario_id": f"P05-{role.upper()}-LANE-CHANGE-EXECUTING",
        "scenario_name": "P05 rejects executing lane-change gap boundary",
        "purpose": "CMC must reject cached lane 2 gap boundaries that are actively changing lanes.",
        "test_level": "unit",
        "status": "required",
        "initial_time": {"t": 0.0, "step": 0, "dt": 0.1},
        "initial_vehicles": [
            _vehicle("MV_EXEC_BOUNDARY", "on_ramp", 7000.0, -3.5, road_role="on_ramp_mv", merge_state="waiting"),
            _vehicle("CLV_EXEC_BOUNDARY", "lane_2", 7030.0, 0.0, lane_change_state=clv_state),
            _vehicle("CFV_EXEC_BOUNDARY", "lane_2", 6970.0, 0.0, lane_change_state=cfv_state),
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
                "mv_id": "MV_EXEC_BOUNDARY",
                "clv_id": "CLV_EXEC_BOUNDARY",
                "cfv_id": "CFV_EXEC_BOUNDARY",
                "aps_case": "case_1",
                "col_clv": False,
                "col_cfv": False,
                "desired_spacing_override": None,
                "status": "valid",
                "created_at_t": -1.0,
                "created_at_step": -10,
                "source": "aps_cache",
                "valid_until_next_aps": True,
            }
        ],
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
    lane_change_state: str = "normal",
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
        "lane_change_state": lane_change_state,
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
