from __future__ import annotations

from types import MappingProxyType
from typing import Any

from cormc.assignment_lifecycle import AssignmentStepView
from cormc import build_prefreeze_workspace_from_scenario, freeze_simulation_state, refresh_relations_snapshot
from cormc.mvs import load_builtin_scenario, run_targeted_scenario
from cormc.step5_cooperative_request import (
    collect_cooperative_requests,
    run_step5_cooperative_request_conflict_resolution,
)


def test_mvs_conflict_1a_merging_zone_mv_priority_contract() -> None:
    config = load_builtin_scenario("MVS-CONFLICT-1A")

    report = run_targeted_scenario(config)

    _assert_required_p06_pass(report)
    assert _matcher_result(report, "expected_event_counts").passed is True
    assert _expected_event(config, "conflict_resolution")["match"]["winner_mv_id"] == "MV_A"
    assert _registered_feature(report, "suppressed_request_marker")["renderer_status"] == "renderer_deferred"


def test_mvs_conflict_1b_smaller_t_star_priority_contract() -> None:
    config = load_builtin_scenario("MVS-CONFLICT-1B")

    report = run_targeted_scenario(config)

    _assert_required_p06_pass(report)
    assert _matcher_result(report, "expected_event_counts").passed is True
    assert _expected_event(config, "conflict_resolution")["match"]["priority_basis"] == "smaller_T_star_MV"
    assert _expected_event(config, "cooperative_request", source_mv_id="MV_G1")["match"]["t_mv_star"] == 5.5


def test_p06_collects_col_true_requests_from_lifecycle_view() -> None:
    state, _ = _state_and_relations(_collector_config())

    case_1 = _assignment_view("MV_COLLECT", col_clv=False, col_cfv=False)
    case_2 = _assignment_view("MV_COLLECT", col_clv=False, col_cfv=True)
    case_3 = _assignment_view("MV_COLLECT", col_clv=True, col_cfv=False)
    case_4 = _assignment_view("MV_COLLECT", col_clv=True, col_cfv=True)

    assert collect_cooperative_requests(state, {"MV_COLLECT": case_1}) == []
    assert [request.cv_role for request in collect_cooperative_requests(state, {"MV_COLLECT": case_2})] == ["cfv"]
    assert [request.cv_role for request in collect_cooperative_requests(state, {"MV_COLLECT": case_3})] == ["clv"]
    assert [request.cv_role for request in collect_cooperative_requests(state, {"MV_COLLECT": case_4})] == ["clv", "cfv"]


def test_p06_keeps_or_filters_request_based_on_p05_assignment_validation() -> None:
    state, relations = _state_and_relations(_collector_config())
    effective_assignment = _assignment_view("MV_COLLECT", col_clv=True, col_cfv=False)

    kept = run_step5_cooperative_request_conflict_resolution(
        state,
        relations,
        assignment_views={"MV_COLLECT": effective_assignment},
        p05_validation_results={"MV_COLLECT": {"validation_status": "valid"}},
    )
    filtered = run_step5_cooperative_request_conflict_resolution(
        state,
        relations,
        assignment_views={"MV_COLLECT": effective_assignment},
        p05_validation_results={
            "MV_COLLECT": {
                "validation_status": "invalid",
                "invalid_reason": "cfv_not_lane_2",
            }
        },
    )

    assert len(kept.cooperative_requests) == 1
    assert kept.cooperative_requests[0].cv_id == "CLV_COLLECT"
    assert filtered.cooperative_requests == ()
    assert filtered.active_requests == {}


def test_failed_invalid_empty_assignment_does_not_create_active_request() -> None:
    state, relations = _state_and_relations(_collector_config())

    for status in ("failed", "invalid", "empty"):
        result = run_step5_cooperative_request_conflict_resolution(
            state,
            relations,
            assignment_views={
                "MV_COLLECT": _assignment_view(
                    "MV_COLLECT",
                    col_clv=True,
                    col_cfv=True,
                    status=status,
                )
            },
        )

        assert result.cooperative_requests == ()
        assert result.active_requests == {}


def test_loser_request_has_suppressed_trace() -> None:
    result = _run_conflict_1a()

    assert len(result.suppressed_requests) == 1
    suppressed = result.suppressed_requests[0]
    assert suppressed["source_mv_id"] == "MV_B"
    assert suppressed["active"] is False
    assert suppressed["suppressed_by_request_id"] == result.conflict_results[0].winner_request_id
    assert suppressed["suppressed_reason"] == "MV_in_merging_zone"
    assert suppressed["conflict_id"] == result.conflict_results[0].conflict_id


def test_conflict_resolution_marked_engineering_patch() -> None:
    result = _run_conflict_1a()
    event = _actual_event(result.actual_events, event_type="conflict_resolution")

    assert event["source"] == "first_version_engineering_patch"
    assert event["is_engineering_patch"] is True
    assert event["payload"]["source"] == "first_version_engineering_patch"
    assert event["payload"]["is_engineering_patch"] is True
    assert event["payload"]["priority_basis"] == "MV_in_merging_zone"


def test_same_cv_has_at_most_one_active_request() -> None:
    result = _run_conflict_1a()

    assert set(result.active_requests) == {"CV_X"}
    assert result.conflict_results[0].active_request_count_for_cv == 1
    assert result.conflict_results[0].one_active_request_per_cv is True
    assert result.conflict_results[0].conflicting_commands_to_same_CV is False


def test_p06_does_not_execute_cuc_or_lane_change_command() -> None:
    result = _run_conflict_1a()

    event_types = {event["event_type"] for event in result.actual_events}
    assert "CUC" not in event_types
    assert "lane_change_command" not in event_types
    assert result.command_buffer.lane_change_commands == {}
    assert result.command_buffer.same_step_overlays == {}
    assert result.command_buffer.cuc_decisions == {}


def test_p06_does_not_write_vehicle_state_before_commit() -> None:
    config = load_builtin_scenario("MVS-CONFLICT-1A")
    state, relations = _state_and_relations(config)
    before_signature = _state_signature(state)

    result = run_step5_cooperative_request_conflict_resolution(state, relations, config=config)

    assert _state_signature(state) == before_signature
    assert any(
        check["check_type"] == "no_write_before_commit" and check["result"] == "pass"
        for check in result.actual_sanity_checks
    )


def test_p06_does_not_rerun_aps_or_cmc() -> None:
    result = _run_conflict_1a()

    forbidden_event_types = {"APS_candidate", "APS", "CMC", "assignment_validation"}
    assert forbidden_event_types.isdisjoint({event["event_type"] for event in result.actual_events})
    assert result.command_buffer.merge_commands == {}
    assert result.command_buffer.speed_cap_commands == {}
    assert result.command_buffer.state_transition_commands == {}
    assert result.command_buffer.cache_update_commands == ()


def _assert_required_p06_pass(report: Any) -> None:
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


def _expected_event(config: dict[str, Any], event_type: str, **match_subset: Any) -> dict[str, Any]:
    for expected in config["expected_events"]:
        if expected["event_type"] != event_type:
            continue
        match = expected.get("match") or {}
        if all(match.get(key) == value for key, value in match_subset.items()):
            return expected
    raise AssertionError(f"missing expected event: {event_type} {match_subset}")


def _registered_feature(report: Any, feature_type: str) -> dict[str, Any]:
    for feature in report.registered_png_features:
        if feature["feature_type"] == feature_type:
            return feature
    raise AssertionError(f"missing registered feature: {feature_type}")


def _run_conflict_1a() -> Any:
    config = load_builtin_scenario("MVS-CONFLICT-1A")
    state, relations = _state_and_relations(config)
    return run_step5_cooperative_request_conflict_resolution(state, relations, config=config)


def _state_and_relations(config: dict[str, Any]) -> tuple[Any, Any]:
    workspace, _ = build_prefreeze_workspace_from_scenario(config)
    state = freeze_simulation_state(workspace)
    relations = refresh_relations_snapshot(state)
    return state, relations


def _collector_config() -> dict[str, Any]:
    return {
        "scenario_id": "P06-COLLECTOR-UNIT",
        "scenario_name": "P06 collector unit",
        "purpose": "Inline P06 collector tests",
        "test_level": "unit",
        "status": "required",
        "initial_time": {"t": 0.0, "step": 0, "dt": 0.1},
        "initial_vehicles": [
            _vehicle("MV_COLLECT", "on_ramp", 6840.0, -3.5, road_role="on_ramp_mv", merge_state="not_started"),
            _vehicle("CLV_COLLECT", "lane_2", 6920.0, 0.0),
            _vehicle("CFV_COLLECT", "lane_2", 6800.0, 0.0),
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


def _assignment_view(
    mv_id: str,
    *,
    col_clv: bool,
    col_cfv: bool,
    status: str = "valid",
) -> AssignmentStepView:
    return AssignmentStepView(
        mv_id=mv_id,
        record=MappingProxyType(
            {
                "record_version": 1,
                "mv_id": mv_id,
                "clv_id": "CLV_COLLECT",
                "cfv_id": "CFV_COLLECT",
                "gap_type": "bounded",
                "aps_case": "case_4",
                "col_clv": col_clv,
                "col_cfv": col_cfv,
                "desired_spacing_override": None,
                "t_mv_star": 5.5,
                "t_star_mv": 5.5,
                "status": status,
                "lifecycle_state": "active_control_zone",
                "source": "aps_updated_this_step",
            }
        ),
        source="aps_updated_this_step",
        consumable_by_step5=status == "valid",
        consumable_by_cmc=status == "valid",
    )


def _actual_event(events: list[dict[str, Any]], *, event_type: str) -> dict[str, Any]:
    for event in events:
        if event["event_type"] == event_type:
            return event
    raise AssertionError(f"missing actual event: {event_type}")


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
        tuple((key, tuple(sorted(value.items()))) for key, value in state.assignment_records_by_mv.items()),
    )
