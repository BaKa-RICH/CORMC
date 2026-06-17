from __future__ import annotations

from dataclasses import fields
from types import MappingProxyType

import pytest

from cormc.simulation_core.pre_freeze import (
    RoadGeometryConfig,
    VehicleSpec,
    VehicleState,
    assert_x_plot_not_used_in_algorithm_path,
    build_prefreeze_workspace_from_scenario,
    freeze_simulation_state,
    overlay_assignment_logical_relations,
    refresh_relations_snapshot,
    resolve_aps_candidate_ids,
    resolve_aps_candidate_window,
    resolve_lane_2_gap_boundary_eligibility,
    resolve_lane_centerline,
    resolve_on_ramp_control_region,
    resolve_region,
    run_step0_to_step3,
    step0_cleanup_and_prepare,
    step1_prefreeze_boundary_generation_hook,
)
from cormc.scenario_schema import (
    match_expected_events,
    match_expected_sanity_checks,
    match_expected_png_features_v0,
)


def test_step0_clears_buffers_retains_cache_and_maneuver_state() -> None:
    workspace, _ = build_prefreeze_workspace_from_scenario(
        _cuc_relation_scenario(),
        command_buffer={"stale": object()},
        next_state_buffer={"stale": object()},
    )
    workspace.assignment_records_by_mv["MV_CUC"] = {"status": "valid"}
    workspace.active_maneuvers["CFV_X"] = workspace.active_maneuvers["CFV_X"]

    event = step0_cleanup_and_prepare(workspace)

    assert workspace.command_buffer == {}
    assert workspace.next_state_buffer == {}
    assert workspace.assignment_records_by_mv == {"MV_CUC": {"status": "valid"}}
    assert "CFV_X" in workspace.active_maneuvers
    assert event["event_type"] == "cleanup"
    assert event["payload"]["retained_assignment_record_vehicle_ids"] == ["MV_CUC"]


def test_lane_2_gap_boundary_eligibility_rejects_executing_lane_change() -> None:
    workspace, _ = build_prefreeze_workspace_from_scenario(_cuc_relation_scenario())
    state = freeze_simulation_state(workspace)

    stable = resolve_lane_2_gap_boundary_eligibility(state, "CLV_Y")
    executing = resolve_lane_2_gap_boundary_eligibility(state, "CFV_X")
    wrong_lane = resolve_lane_2_gap_boundary_eligibility(state, "TLV")

    assert stable.eligible is True
    assert stable.reason == "stable_lane_2"
    assert executing.eligible is False
    assert executing.reason == "lane_change_executing"
    assert wrong_lane.eligible is False
    assert wrong_lane.reason == "not_lane_2"


def test_step1_disabled_boundary_generation_does_not_insert_vehicle() -> None:
    workspace, config = build_prefreeze_workspace_from_scenario(_aps_fail_empty_scenario())
    before = list(workspace.active_vehicle_ids)
    candidate = (
        VehicleState(
            vehicle_id="LATE_BOUNDARY",
            x_global=10.0,
            y=0.0,
            v=30.0,
            a=0.0,
            physical_lane="lane_2",
            road_role="mainline",
        ),
        VehicleSpec(
            vehicle_id="LATE_BOUNDARY",
            vehicle_type="cav",
            compliance_state="not_applicable",
            source_lane_at_generation="lane_2",
        ),
    )

    event = step1_prefreeze_boundary_generation_hook(
        workspace,
        config,
        new_vehicle_candidates=[candidate],
    )

    assert workspace.active_vehicle_ids == before
    assert "LATE_BOUNDARY" not in workspace.vehicle_states
    assert event["event_type"] == "boundary_generation"
    assert event["reason"] == "disabled"
    assert event["payload"]["freeze_phase"] == "pre_freeze"


def test_step1_enabled_boundary_generation_only_affects_prefreeze_active_set() -> None:
    scenario = _base_scenario(
        "P02-BOUNDARY-ENABLED",
        vehicles=[_vehicle("MV", "on_ramp", 6830.0, -3.5, road_role="on_ramp_mv")],
        module_overrides={"boundary_generation_enabled": True},
    )
    workspace, config = build_prefreeze_workspace_from_scenario(scenario)
    candidate = (
        VehicleState(
            vehicle_id="PRE_FREEZE_NEW",
            x_global=5.0,
            y=0.0,
            v=30.0,
            a=0.0,
            physical_lane="lane_2",
            road_role="mainline",
        ),
        VehicleSpec(
            vehicle_id="PRE_FREEZE_NEW",
            vehicle_type="cav",
            compliance_state="not_applicable",
            source_lane_at_generation="lane_2",
        ),
    )

    event = step1_prefreeze_boundary_generation_hook(
        workspace,
        config,
        new_vehicle_candidates=[candidate],
    )
    frozen = freeze_simulation_state(workspace)
    workspace.active_vehicle_ids.append("POST_FREEZE_NEW")

    assert event["reason"] == "generated_pre_freeze"
    assert "PRE_FREEZE_NEW" in frozen.active_vehicle_ids
    assert "POST_FREEZE_NEW" not in frozen.active_vehicle_ids


def test_step2_freeze_is_immutable_to_late_vehicle_insert() -> None:
    workspace, _ = build_prefreeze_workspace_from_scenario(_aps_fail_empty_scenario())
    frozen = freeze_simulation_state(workspace)
    workspace.active_vehicle_ids.append("LATE")
    workspace.vehicle_states["LATE"] = VehicleState(
        vehicle_id="LATE",
        x_global=1.0,
        y=0.0,
        v=1.0,
        a=0.0,
        physical_lane="lane_2",
        road_role="mainline",
    )

    relations = refresh_relations_snapshot(frozen)

    assert "LATE" not in frozen.active_vehicle_ids
    assert "LATE" not in relations.lane_ordering["lane_2"]
    with pytest.raises(TypeError):
        frozen.vehicle_states["LATE"] = workspace.vehicle_states["LATE"]  # type: ignore[index]


def test_simulation_state_excludes_command_next_state_history_relations() -> None:
    workspace, _ = build_prefreeze_workspace_from_scenario(
        _aps_fail_empty_scenario(),
        command_buffer={"stale": "command"},
        next_state_buffer={"stale": "next"},
    )
    frozen = freeze_simulation_state(workspace)
    field_names = {field.name for field in fields(frozen)}

    assert "command_buffer" not in field_names
    assert "next_state_buffer" not in field_names
    assert "relations" not in field_names
    assert "event_records" not in field_names
    assert "sanity_check_records" not in field_names
    assert "trajectory_records" not in field_names
    assert "history" not in field_names


def test_step3_lane_ordering_uses_x_global() -> None:
    scenario = _base_scenario(
        "P02-X-GLOBAL-ORDER",
        vehicles=[
            _vehicle("A", "lane_2", 700.0, 0.0),
            _vehicle("B", "lane_2", 100.0, 0.0),
            _vehicle("C", "lane_2", 400.0, 0.0),
        ],
    )
    result = run_step0_to_step3(scenario)

    assert result.relations.lane_ordering["lane_2"] == ("B", "C", "A")
    event = _event_by_type(result.actual_events, "relation_refresh")
    lane_2_payload = _lane_payload(event, "lane_2")
    assert lane_2_payload["ordered_vehicle_ids"] == ["B", "C", "A"]
    assert lane_2_payload["ordered_x_global"] == [100.0, 400.0, 700.0]
    assert lane_2_payload["ordering_coordinate"] == "x_global"
    assert lane_2_payload["x_plot_used"] is False


def test_aps_candidate_window_uses_lcr_not_fixed_cooperative_zone() -> None:
    window = resolve_aps_candidate_window(6830.0, mv_id="MV_FAIL_EMPTY")

    assert window.start_x_global == 6530.0
    assert window.end_x_global == 7130.0
    assert window.l_cr == 300.0
    assert window.parameter_name == "L_cr"
    assert window.uses_fixed_cooperative_zone is False
    assert window.uses_dynamic_coop_window is False


def test_mvs_aps_fail_empty_candidates_supported_by_geometry_and_relations() -> None:
    result = run_step0_to_step3(_aps_fail_empty_scenario())

    assert resolve_aps_candidate_ids(result.state, "MV_FAIL_EMPTY") == ["ONLY_LANE2_FAIL"]
    geometry_event = _event_by_type(result.actual_events, "geometry")
    window = geometry_event["payload"]["aps_candidate_windows"]["MV_FAIL_EMPTY"]
    assert window["start_x_global"] == 6530.0
    assert window["end_x_global"] == 7130.0
    assert window["candidate_vehicle_ids"] == ["ONLY_LANE2_FAIL"]
    assert geometry_event["payload"]["uses_fixed_cooperative_zone_for_aps_window"] is False


def test_lane_centerline_and_region_resolvers() -> None:
    assert resolve_lane_centerline("lane_1").y == 3.5
    assert resolve_lane_centerline("lane_2").y == 0.0
    assert resolve_lane_centerline("on_ramp").y == -3.5

    before = resolve_region(6949.9, "on_ramp_mv")
    merging = resolve_region(6950.0, "on_ramp_mv")
    ramp_end = resolve_region(7250.0, "on_ramp_mv")
    past = resolve_region(7250.1, "on_ramp_mv")

    assert before.before_merging_zone is True
    assert merging.in_merging_zone is True
    assert ramp_end.in_merging_zone is True
    assert past.past_ramp_end is True
    assert all(not result.uses_x_plot for result in [before, merging, ramp_end, past])


def test_on_ramp_control_region_resolver_matches_sumo_boundaries() -> None:
    pre_control = resolve_on_ramp_control_region(6649.9, "on_ramp_mv")
    control_start = resolve_on_ramp_control_region(6650.0, "on_ramp_mv")
    merge_start = resolve_on_ramp_control_region(6950.0, "on_ramp_mv")
    ramp_end = resolve_on_ramp_control_region(7250.0, "on_ramp_mv")
    post_merge = resolve_on_ramp_control_region(7250.1, "on_ramp_mv")

    assert pre_control.region == "pre_control"
    assert pre_control.aps_allowed is False
    assert pre_control.cooperative_request_allowed is False
    assert pre_control.cuc_allowed is False
    assert pre_control.cmc_allowed is False
    assert control_start.region == "control_zone"
    assert control_start.aps_allowed is True
    assert control_start.cooperative_request_allowed is True
    assert control_start.cuc_allowed is True
    assert control_start.cmc_allowed is False
    assert merge_start.region == "merge_zone"
    assert merge_start.aps_allowed is False
    assert merge_start.cmc_allowed is True
    assert ramp_end.region == "merge_zone"
    assert post_merge.region == "post_merge"
    assert post_merge.cmc_allowed is False
    assert all(
        region.uses_x_global and not region.uses_x_plot
        for region in [pre_control, control_start, merge_start, ramp_end, post_merge]
    )


def test_geometry_event_records_on_ramp_control_region() -> None:
    result = run_step0_to_step3(
        _base_scenario(
            "P02-ON-RAMP-CONTROL-REGION",
            vehicles=[
                _vehicle("MV_PRE", "on_ramp", 6640.0, -3.5, road_role="on_ramp_mv"),
                _vehicle("MV_CONTROL", "on_ramp", 6850.0, -3.5, road_role="on_ramp_mv"),
                _vehicle("MV_MERGE", "on_ramp", 6950.0, -3.5, road_role="on_ramp_mv"),
            ],
        )
    )

    geometry_event = _event_by_type(result.actual_events, "geometry")
    regions = geometry_event["payload"]["on_ramp_control_regions"]

    assert geometry_event["payload"]["control_zone_global"] == [6650.0, 6950.0]
    assert regions["MV_PRE"]["region"] == "pre_control"
    assert regions["MV_CONTROL"]["region"] == "control_zone"
    assert regions["MV_MERGE"]["region"] == "merge_zone"


def test_active_lane_change_relation_not_switched_by_physical_y() -> None:
    result = run_step0_to_step3(_cuc_relation_scenario())

    neighborhood = result.relations.lane_change_neighborhood["CFV_X"]
    active_relation = result.relations.active_maneuver_relation["CFV_X"]

    assert neighborhood.source_lane == "lane_2"
    assert neighborhood.target_lane == "lane_1"
    assert neighborhood.tlv_id == "TLV"
    assert neighborhood.tfv_id == "TFV"
    assert neighborhood.lv_id == "CLV_Y"
    assert neighborhood.fv_id == "FV_Z"
    assert active_relation.primary_leader_id == "TLV"
    assert active_relation.affected_target_follower_id == "TFV"
    assert active_relation.affected_source_follower_id == "FV_Z"
    assert active_relation.relation_source == "active_lane_change"
    assert result.state.vehicle_states["CFV_X"].physical_lane == "lane_2"


def test_assignment_logical_relation_overlays_mv_clv_leader() -> None:
    workspace, _ = build_prefreeze_workspace_from_scenario(_assignment_relation_scenario())
    state = freeze_simulation_state(workspace)
    base_relations = refresh_relations_snapshot(state)

    relations = overlay_assignment_logical_relations(state, base_relations)

    relation = relations.active_maneuver_relation["B02_MV"]
    assert relation.primary_leader_id == "B02_CLV"
    assert relation.affected_target_follower_id == "B02_CFV"
    assert relation.affected_source_follower_id is None
    assert relation.relation_source == "aps_assignment_case_3_mv_clv_leader"

    assert "B02_CFV" not in relations.active_maneuver_relation


def test_assignment_logical_relation_rear_boundary_waits_for_merge_zone_gap() -> None:
    workspace, _ = build_prefreeze_workspace_from_scenario(
        _assignment_relation_scenario(
            mv_x=6960.0,
            clv_x=7000.0,
            cfv_x=6920.0,
            aps_case="case_4",
            col_cfv=True,
        )
    )
    state = freeze_simulation_state(workspace)
    record = {
        **dict(state.assignment_records_by_mv["B02_MV"]),
        "lifecycle_state": "active_merge_zone",
    }
    state = type(state)(
        t=state.t,
        step=state.step,
        dt=state.dt,
        active_vehicle_ids=state.active_vehicle_ids,
        vehicle_states=state.vehicle_states,
        vehicle_specs=state.vehicle_specs,
        assignment_records_by_mv=MappingProxyType({"B02_MV": MappingProxyType(record)}),
        active_maneuvers=state.active_maneuvers,
        road_config_ref=state.road_config_ref,
        parameter_config_ref=state.parameter_config_ref,
        scenario_config_ref=state.scenario_config_ref,
        output_config_ref=state.output_config_ref,
        controller_memory_by_vehicle=state.controller_memory_by_vehicle,
    )
    base_relations = refresh_relations_snapshot(state)

    relations = overlay_assignment_logical_relations(state, base_relations)

    rear_boundary = relations.active_maneuver_relation["B02_CFV"]
    assert rear_boundary.primary_leader_id == "B02_MV"
    assert rear_boundary.affected_target_follower_id is None
    assert rear_boundary.affected_source_follower_id is None
    assert rear_boundary.relation_source == "aps_assignment_case_4_cfv_mv_rear_boundary"


def test_assignment_logical_relation_skips_executing_cfv_rear_boundary() -> None:
    workspace, _ = build_prefreeze_workspace_from_scenario(
        _assignment_relation_scenario(cfv_lane_change_state="executing")
    )
    state = freeze_simulation_state(workspace)
    base_relations = refresh_relations_snapshot(state)

    relations = overlay_assignment_logical_relations(state, base_relations)

    assert relations.active_maneuver_relation["B02_MV"].primary_leader_id == "B02_CLV"
    relation = relations.active_maneuver_relation["B02_CFV"]
    assert relation.relation_source == "active_lane_change"
    assert relation.primary_leader_id != "B02_MV"


def test_assignment_logical_relation_skips_invalid_clv_position_or_lane() -> None:
    behind_workspace, _ = build_prefreeze_workspace_from_scenario(
        _assignment_relation_scenario(clv_x=6840.0)
    )
    behind_state = freeze_simulation_state(behind_workspace)
    behind_relations = overlay_assignment_logical_relations(
        behind_state,
        refresh_relations_snapshot(behind_state),
    )
    assert "B02_MV" not in behind_relations.active_maneuver_relation

    wrong_lane_workspace, _ = build_prefreeze_workspace_from_scenario(
        _assignment_relation_scenario(clv_lane="lane_1", clv_y=3.5)
    )
    wrong_lane_state = freeze_simulation_state(wrong_lane_workspace)
    wrong_lane_relations = overlay_assignment_logical_relations(
        wrong_lane_state,
        refresh_relations_snapshot(wrong_lane_state),
    )
    assert "B02_MV" not in wrong_lane_relations.active_maneuver_relation


def test_p02_event_and_sanity_candidates_are_consumable_by_p01_matcher() -> None:
    result = run_step0_to_step3(_cuc_relation_scenario())

    events = match_expected_events(
        [
            {"event_type": "cleanup", "required": True},
            {"event_type": "boundary_generation", "required": True, "reason_code": "disabled"},
            {
                "event_type": "freeze",
                "required": True,
                "match": {"snapshot_is_read_only": True},
            },
            {
                "event_type": "relation_refresh",
                "required": True,
                "match": {"relations_based_on_frozen_s_t": True},
            },
            {
                "event_type": "geometry",
                "required": True,
                "match": {
                    "aps_candidate_window_parameter": "L_cr",
                    "uses_fixed_cooperative_zone_for_aps_window": False,
                },
            },
        ],
        result.actual_events,
        {"derived_formula_abs": 0.01},
    )
    sanity = match_expected_sanity_checks(
        [
            {"check_type": "collision", "required": True, "expected_status": "pass"},
            {"check_type": "near_collision", "required": True, "expected_status": "pass"},
            {
                "check_type": "state_machine_inconsistency",
                "required": True,
                "expected_status": "pass",
            },
            {
                "check_type": "unexpected_ordinary_lane_change_attempt",
                "required": True,
                "expected_status": "pass",
            },
            {
                "check_type": "multiple_commit_for_one_vehicle",
                "required": True,
                "expected_status": "not_applicable",
            },
            {
                "check_type": "x_plot_used_in_algorithm_path",
                "required": True,
                "expected_status": "pass",
            },
            {
                "check_type": "geometry_inconsistency",
                "required": True,
                "expected_status": "pass",
            },
            {
                "check_type": "relations_consistency",
                "required": True,
                "expected_status": "pass",
            },
        ],
        result.actual_sanity_checks,
    )
    png = match_expected_png_features_v0(result.expected_png_features)

    assert events.passed is True
    assert sanity.passed is True
    assert png.passed is True
    assert {item["feature_type"] for item in png.registered} == {
        "lane_centerline_quicklook",
        "merging_zone_boundary_quicklook",
        "aps_candidate_window_quicklook",
    }


def test_x_plot_is_absent_from_algorithm_state_and_relations() -> None:
    result = run_step0_to_step3(_cuc_relation_scenario())

    assert assert_x_plot_not_used_in_algorithm_path(result.state, result.relations) is True


def test_custom_geometry_sanity_detects_centerline_mismatch() -> None:
    geometry = RoadGeometryConfig(
        lane_centerlines={"lane_1": 4.0, "lane_2": 0.0, "on_ramp": -3.5}
    )
    result = run_step0_to_step3(_aps_fail_empty_scenario(), geometry=geometry)

    geometry_sanity = _sanity_by_type(result.actual_sanity_checks, "geometry_inconsistency")
    assert geometry_sanity["result"] == "fail"


def _base_scenario(
    scenario_id: str,
    *,
    vehicles: list[dict],
    module_overrides: dict | None = None,
    preloaded_assignments: list[dict] | None = None,
    preloaded_maneuver_trajectory_states: list[dict] | None = None,
) -> dict:
    return {
        "scenario_id": scenario_id,
        "scenario_name": scenario_id,
        "purpose": "P02 targeted scenario",
        "test_level": "unit",
        "status": "required",
        "initial_time": {"t": 0.0, "step": 0, "dt": 0.1},
        "initial_vehicles": vehicles,
        "module_overrides": {
            "boundary_generation_enabled": False,
            "random_arrival_enabled": False,
            "random_vehicle_attributes_enabled": False,
            "ordinary_mainline_lane_change_enabled": False,
            "platoon_cmc_enabled": False,
            "mpc_lateral_tracking_enabled": False,
            **(module_overrides or {}),
        },
        "preloaded_assignments": preloaded_assignments or [],
        "preloaded_maneuver_trajectory_states": preloaded_maneuver_trajectory_states or [],
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
        "lane_change_state": lane_change_state,
        "merge_state": merge_state,
        "spec_overrides": {},
    }


def _cuc_relation_scenario() -> dict:
    return _base_scenario(
        "P02-CUC-RELATIONS",
        vehicles=[
            _vehicle("MV_CUC", "on_ramp", 6850.0, -3.5, road_role="on_ramp_mv"),
            _vehicle("CFV_X", "lane_2", 6844.0, 3.1, lane_change_state="executing"),
            _vehicle("CLV_Y", "lane_2", 6884.0, 0.0),
            _vehicle("FV_Z", "lane_2", 6800.0, 0.0),
            _vehicle("TLV", "lane_1", 6920.0, 3.5),
            _vehicle("TFV", "lane_1", 6750.0, 3.5),
        ],
        preloaded_maneuver_trajectory_states=[
            {
                "vehicle_id": "CFV_X",
                "maneuver_type": "lane_change",
                "start_step": 0,
                "start_t": 0.0,
                "start_x_global": 6840.0,
                "start_y": 0.0,
                "target_lane": "lane_1",
                "target_y": 3.5,
                "planned_length": 100.0,
                "progress": 0.8,
            }
        ],
    )


def _assignment_relation_scenario(
    *,
    mv_x: float = 6850.0,
    clv_x: float = 6865.0,
    cfv_x: float = 6810.0,
    clv_lane: str = "lane_2",
    clv_y: float = 0.0,
    cfv_lane_change_state: str = "normal",
    aps_case: str = "case_3",
    col_cfv: bool = False,
) -> dict:
    assignment = {
        "mv_id": "B02_MV",
        "clv_id": "B02_CLV",
        "cfv_id": "B02_CFV",
        "aps_case": aps_case,
        "col_clv": True,
        "col_cfv": col_cfv,
        "status": "valid",
        "source": "aps_cache",
    }
    return _base_scenario(
        "P02-ASSIGNMENT-RELATION",
        vehicles=[
            _vehicle("B02_MV", "on_ramp", mv_x, -3.5, road_role="on_ramp_mv"),
            _vehicle("B02_CLV", clv_lane, clv_x, clv_y),
            _vehicle("B02_CFV", "lane_2", cfv_x, 0.0, lane_change_state=cfv_lane_change_state),
        ],
        preloaded_assignments=[assignment],
    )


def _aps_fail_empty_scenario() -> dict:
    return _base_scenario(
        "P02-APS-FAIL-EMPTY",
        vehicles=[
            _vehicle("MV_FAIL_EMPTY", "on_ramp", 6830.0, -3.5, road_role="on_ramp_mv"),
            _vehicle("ONLY_LANE2_FAIL", "lane_2", 6840.0, 0.0),
        ],
    )


def _event_by_type(events: list[dict], event_type: str) -> dict:
    for event in events:
        if event["event_type"] == event_type:
            return event
    raise AssertionError(f"missing event: {event_type}")


def _lane_payload(event: dict, lane_id: str) -> dict:
    for payload in event["payload"]["lane_ordering"]:
        if payload["lane_id"] == lane_id:
            return payload
    raise AssertionError(f"missing lane payload: {lane_id}")


def _sanity_by_type(sanity_checks: list[dict], check_type: str) -> dict:
    for check in sanity_checks:
        if check["check_type"] == check_type:
            return check
    raise AssertionError(f"missing sanity check: {check_type}")
