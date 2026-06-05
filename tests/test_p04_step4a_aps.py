from __future__ import annotations

from typing import Any

from cormc import build_prefreeze_workspace_from_scenario, freeze_simulation_state
from cormc.mvs import run_targeted_scenario
from cormc.step4a_aps import run_step4a_aps_for_scenario


def test_mvs_aps_fail_empty_p04_required_fails_until_aps_implemented() -> None:
    report = run_targeted_scenario("MVS-APS-FAIL-EMPTY")

    _assert_required_p04_pass(report)


def test_mvs_aps_fail_cache_p04_required_fails_until_aps_implemented() -> None:
    report = run_targeted_scenario(_aps_fail_cache_config())

    _assert_required_p04_pass(report)
    assert _matcher_result(report, "expected_event_counts").passed is True


def test_mvs_aps_case_1_contract() -> None:
    report = run_targeted_scenario(
        _aps_case_config(
            scenario_id="MVS-APS-1",
            clv_id="CLV_APS_1",
            clv_x=6884.0,
            cfv_id="CFV_APS_1",
            cfv_x=6824.0,
            aps_case="case_1",
            col_clv=False,
            col_cfv=False,
        )
    )

    _assert_required_p04_pass(report)


def test_mvs_aps_case_2_eq10_to_cfv_only_contract() -> None:
    report = run_targeted_scenario(
        _aps_case_config(
            scenario_id="MVS-APS-2",
            clv_id="CLV_APS_2",
            clv_x=6884.0,
            cfv_id="CFV_APS_2",
            cfv_x=6844.0,
            aps_case="case_2",
            col_clv=False,
            col_cfv=True,
            eq10_spacing=58.0,
        )
    )

    _assert_required_p04_pass(report)


def test_mvs_aps_case_3_no_eq10_to_clv_contract() -> None:
    report = run_targeted_scenario(
        _aps_case_config(
            scenario_id="MVS-APS-3",
            clv_id="CLV_APS_3",
            clv_x=6864.0,
            cfv_id="CFV_APS_3",
            cfv_x=6824.0,
            aps_case="case_3",
            col_clv=True,
            col_cfv=False,
            forbid_eq10_to_clv=True,
        )
    )

    _assert_required_p04_pass(report)
    assert _matcher_result(report, "forbidden_events").passed is True


def test_p04_effective_assignment_preserves_t_star_handoff() -> None:
    config = _aps_case_config(
        scenario_id="P04-T-STAR-HANDOFF",
        clv_id="CLV_T_STAR",
        clv_x=6864.0,
        cfv_id="CFV_T_STAR",
        cfv_x=6824.0,
        aps_case="case_3",
        col_clv=True,
        col_cfv=False,
    )

    result = run_step4a_aps_for_scenario(config)

    assignment = result.effective_assignments["MV_A"].assignment
    aps_event = next(
        event for event in result.actual_events if "t_star_mv" in event.get("payload", {})
    )
    assert assignment["t_star_mv"] == aps_event["payload"]["t_star_mv"]
    assert assignment["t_mv_star"] == aps_event["payload"]["t_star_mv"]


def test_p04_excludes_executing_lane_change_from_aps_candidates() -> None:
    result = run_step4a_aps_for_scenario(_aps_executing_candidate_config())

    candidate_event = _actual_event(result.actual_events, "APS_candidate", "MV_EXEC_FILTER")

    assert "CFV_EXECUTING" not in candidate_event["payload"]["candidate_ids"]
    assert candidate_event["payload"]["candidate_ids"] == ["CFV_STABLE", "CLV_STABLE"]
    assert candidate_event["payload"]["excluded_candidates"] == [
        {
            "vehicle_id": "CFV_EXECUTING",
            "physical_lane": "lane_2",
            "lane_change_state": "executing",
            "excluded_reason": "lane_change_executing",
        }
    ]


def test_p04_invalid_cached_boundary_triggers_immediate_fresh_assignment() -> None:
    result = run_step4a_aps_for_scenario(_invalid_cache_with_replacement_config())

    aps_event = _actual_event(result.actual_events, "APS", "MV_INVALID_CACHE", reason="cached_gap_boundary_invalid")
    assignment = result.effective_assignments["MV_INVALID_CACHE"].assignment

    assert aps_event["payload"]["trigger"] == "cached_gap_boundary_invalid"
    assert assignment["clv_id"] in {"CLV_OLD", "CLV_REPLACEMENT"}
    assert assignment["cfv_id"] == "CFV_REPLACEMENT"
    assert assignment["cfv_id"] != "CFV_EXECUTING_OLD"
    assert result.cache_actions[0].action == "update_request"


def test_p04_invalid_cached_boundary_failed_fresh_aps_does_not_retain_old_assignment() -> None:
    result = run_step4a_aps_for_scenario(_invalid_cache_without_replacement_config())

    aps_event = _actual_event(result.actual_events, "APS", "MV_INVALID_FAIL", reason="insufficient_candidates")
    cache_event = _actual_event(result.actual_events, "assignment_cache", "MV_INVALID_FAIL")

    assert "MV_INVALID_FAIL" not in result.effective_assignments
    assert result.cache_actions[0].action == "invalidate"
    assert aps_event["payload"]["trigger"] == "cached_gap_boundary_invalid"
    assert aps_event["payload"]["old_cache_invalidated"] is True
    assert aps_event["payload"]["invalid_boundary_role"] == "cfv"
    assert aps_event["payload"]["invalid_boundary_id"] == "CFV_EXECUTING_OLD"
    assert aps_event["payload"]["invalid_reason"] == "lane_change_executing"
    assert aps_event["payload"]["effective_assignment_source"] is None
    assert cache_event["payload"]["action"] == "invalidate"


def test_mvs_aps_case_4_eq10_to_cfv_only_contract() -> None:
    report = run_targeted_scenario(
        _aps_case_config(
            scenario_id="MVS-APS-4",
            clv_id="CLV_APS_4",
            clv_x=6864.0,
            cfv_id="CFV_APS_4",
            cfv_x=6844.0,
            aps_case="case_4",
            col_clv=True,
            col_cfv=True,
            eq10_spacing=52.0,
        )
    )

    _assert_required_p04_pass(report)


def test_first_aps_event_marked_engineering_patch() -> None:
    report = run_targeted_scenario(_first_aps_trigger_config())

    _assert_required_p04_pass(report)


def test_non_aps_period_reuses_cache_without_mutating_state() -> None:
    config = _reuse_cache_config()
    workspace, _ = build_prefreeze_workspace_from_scenario(config)
    frozen = freeze_simulation_state(workspace)
    before_signature = _state_signature(frozen)

    report = run_targeted_scenario(config)

    _assert_required_p04_pass(report)
    assert _state_signature(frozen) == before_signature


def test_p04_does_not_write_vehicle_state_before_commit() -> None:
    config = _p04_no_write_contract_config()
    workspace, _ = build_prefreeze_workspace_from_scenario(config)
    frozen = freeze_simulation_state(workspace)
    before_signature = _state_signature(frozen)

    report = run_targeted_scenario(config)

    _assert_required_p04_pass(report)
    assert _state_signature(frozen) == before_signature


def test_p04_hands_off_merging_zone_or_executing_mv_to_p05() -> None:
    config = _p04_handoff_to_cmc_config()
    workspace, _ = build_prefreeze_workspace_from_scenario(config)
    frozen = freeze_simulation_state(workspace)
    before_signature = _state_signature(frozen)

    report = run_targeted_scenario(config)

    _assert_required_p04_pass(report)
    assert _state_signature(frozen) == before_signature
    assert _matcher_result(report, "expected_event_counts").passed is True


def _assert_required_p04_pass(report: Any) -> None:
    assert report.status == "required"
    assert report.classification == "required_passed"
    assert report.blocks_required_suite is False
    assert report.failure_reasons == []
    assert all(result.passed for result in report.matcher_results)


def _base_config(
    scenario_id: str,
    *,
    vehicles: list[dict[str, Any]],
    expected_events: list[dict[str, Any]],
    expected_sanity_checks: list[dict[str, Any]],
    expected_png_features: list[dict[str, Any]] | None = None,
    forbidden_events: list[dict[str, Any]] | None = None,
    expected_event_counts: list[dict[str, Any]] | None = None,
    preloaded_assignments: list[dict[str, Any]] | None = None,
    preloaded_state_machine_states: list[dict[str, Any]] | None = None,
) -> dict[str, Any]:
    return {
        "scenario_id": scenario_id,
        "scenario_name": scenario_id,
        "purpose": "P04 APS targeted green test",
        "test_level": "unit",
        "status": "required",
        "derivation_ref": ["CORMC minimal validation scenario spec #5"],
        "initial_time": {"t": 0.0, "step": 0, "dt": 0.1},
        "initial_vehicles": vehicles,
        "module_overrides": {
            "boundary_generation_enabled": False,
            "random_arrival_enabled": False,
            "random_vehicle_attributes_enabled": False,
            "ordinary_mainline_lane_change_enabled": False,
            "platoon_cmc_enabled": False,
            "mpc_lateral_tracking_enabled": False,
            "test_harness_overrides": {"source": "test_harness_override"},
        },
        "preloaded_assignments": preloaded_assignments or [],
        "preloaded_state_machine_states": preloaded_state_machine_states or [],
        "preloaded_maneuver_trajectory_states": [],
        "expected_events": expected_events,
        "forbidden_events": forbidden_events or [],
        "expected_event_counts": expected_event_counts or [],
        "expected_sanity_checks": expected_sanity_checks,
        "expected_png_features": expected_png_features or [],
    }


def _vehicle(
    vehicle_id: str,
    lane: str,
    x_global: float,
    y: float,
    *,
    road_role: str = "mainline",
    vehicle_type: str = "CAV",
    compliance_state: str = "not_applicable",
    merge_state: str = "none",
    lane_change_state: str = "normal",
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


def _aps_fail_cache_config() -> dict[str, Any]:
    return _base_config(
        "MVS-APS-FAIL-CACHE",
        vehicles=[
            _vehicle(
                "MV_FAIL_CACHE",
                "on_ramp",
                6830.0,
                -3.5,
                road_role="on_ramp_mv",
                merge_state="not_started",
            ),
            _vehicle("ONLY_LANE2_FAIL", "lane_2", 6860.0, 0.0),
            _vehicle("OLD_CLV", "lane_2", 7200.0, 0.0),
            _vehicle("OLD_CFV", "lane_2", 6450.0, 0.0),
        ],
        preloaded_assignments=[
            {
                "mv_id": "MV_FAIL_CACHE",
                "clv_id": "OLD_CLV",
                "cfv_id": "OLD_CFV",
                "aps_case": "case_1",
                "col_clv": False,
                "col_cfv": False,
                "desired_spacing_override": None,
                "status": "valid",
                "created_at_t": -5.0,
                "created_at_step": -50,
                "source": "aps_cache",
                "valid_until_next_aps": True,
                "staleness_policy": "retain_on_failed_aps",
            }
        ],
        expected_events=[
            {
                "event_type": "APS",
                "required": True,
                "vehicle_ids": ["MV_FAIL_CACHE"],
                "match": {"failure": True},
                "reason_code": "insufficient_candidates",
                "source": "paper_formula",
            },
            {
                "event_type": "assignment_cache",
                "required": True,
                "vehicle_ids": ["MV_FAIL_CACHE"],
                "match": {
                    "previous_cache_exists": True,
                    "invalid_new_assignment_overwrites_existing_cache": False,
                },
                "source": "first_version_engineering_patch",
            },
        ],
        expected_sanity_checks=[
            {
                "check_type": "assignment_cache_overwrite_by_failed_APS",
                "required": True,
                "expected_status": "pass",
                "vehicle_ids": ["MV_FAIL_CACHE"],
            }
        ],
        expected_png_features=[
            {
                "feature_type": "aps_failure_marker",
                "required": True,
                "vehicle_ids": ["MV_FAIL_CACHE"],
                "expected_visibility": "visible",
            },
            {
                "feature_type": "cache_reuse_marker",
                "required": True,
                "vehicle_ids": ["MV_FAIL_CACHE"],
                "expected_visibility": "visible",
            },
        ],
        expected_event_counts=[
            {
                "event_type": "cooperative_request",
                "vehicle_ids": ["MV_FAIL_CACHE"],
                "expected_count": 0,
                "comparison": "exactly",
            }
        ],
    )


def _aps_case_config(
    *,
    scenario_id: str,
    clv_id: str,
    clv_x: float,
    cfv_id: str,
    cfv_x: float,
    aps_case: str,
    col_clv: bool,
    col_cfv: bool,
    eq10_spacing: float | None = None,
    forbid_eq10_to_clv: bool = False,
) -> dict[str, Any]:
    expected_match: dict[str, Any] = {
        "trigger": "first_APS",
        "aps_case": aps_case,
        "clv_id": clv_id,
        "cfv_id": cfv_id,
        "col_clv": col_clv,
        "col_cfv": col_cfv,
    }
    numeric_expectations: dict[str, Any] = {}
    expected_events = [
        {
            "event_type": "APS",
            "required": True,
            "vehicle_ids": ["MV_A", clv_id, cfv_id],
            "match": expected_match,
            "numeric_expectations": numeric_expectations,
            "source": "paper_formula",
        }
    ]
    expected_png_features = [
        {
            "feature_type": "aps_assignment_marker",
            "required": True,
            "vehicle_ids": ["MV_A", clv_id, cfv_id],
            "expected_visibility": "visible",
        }
    ]
    if eq10_spacing is not None:
        expected_match["eq10_vehicle_role"] = "cfv"
        numeric_expectations["desired_spacing_override"] = {
            "value": eq10_spacing,
            "tolerance": "derived_formula_abs",
        }
        expected_events.append(
            {
                "event_type": "eq10_desired_spacing_source",
                "required": True,
                "vehicle_ids": ["MV_A", cfv_id],
                "match": {"eq10_vehicle_role": "cfv"},
                "numeric_expectations": {
                    "desired_spacing_override": {
                        "value": eq10_spacing,
                        "tolerance": "derived_formula_abs",
                    }
                },
                "source": "paper_formula",
            }
        )
        expected_png_features.append(
            {
                "feature_type": "eq10_spacing_marker",
                "required": True,
                "vehicle_ids": ["MV_A", cfv_id],
                "expected_visibility": "visible",
            }
        )
    return _base_config(
        scenario_id,
        vehicles=[
            _vehicle(
                "MV_A",
                "on_ramp",
                6850.0,
                -3.5,
                road_role="on_ramp_mv",
                merge_state="not_started",
            ),
            _vehicle(clv_id, "lane_2", clv_x, 0.0),
            _vehicle(cfv_id, "lane_2", cfv_x, 0.0),
        ],
        expected_events=expected_events,
        forbidden_events=(
            [
                {
                    "event_type": "APS",
                    "vehicle_ids": ["MV_A", clv_id],
                    "match": {"eq10_vehicle_role": "clv"},
                    "source": "paper_formula",
                }
            ]
            if forbid_eq10_to_clv
            else []
        ),
        expected_sanity_checks=[
            {
                "check_type": "assignment_invalid",
                "required": True,
                "expected_status": "pass",
                "vehicle_ids": ["MV_A"],
            },
            {
                "check_type": "Eq10_applied_to_wrong_vehicle",
                "required": True,
                "expected_status": "pass",
                "vehicle_ids": ["MV_A"],
            },
            {
                "check_type": "x_plot_used_in_algorithm_path",
                "required": True,
                "expected_status": "pass",
                "vehicle_ids": ["MV_A"],
            },
        ],
        expected_png_features=expected_png_features,
    )


def _aps_executing_candidate_config() -> dict[str, Any]:
    return _base_config(
        "P04-APS-EXECUTING-CANDIDATE-FILTER",
        vehicles=[
            _vehicle(
                "MV_EXEC_FILTER",
                "on_ramp",
                6830.0,
                -3.5,
                road_role="on_ramp_mv",
                merge_state="not_started",
            ),
            _vehicle("CFV_EXECUTING", "lane_2", 6820.0, 0.0, lane_change_state="executing"),
            _vehicle("CFV_STABLE", "lane_2", 6824.0, 0.0),
            _vehicle("CLV_STABLE", "lane_2", 6884.0, 0.0),
        ],
        expected_events=[],
        expected_sanity_checks=[],
    )


def _invalid_cache_with_replacement_config() -> dict[str, Any]:
    return _base_config(
        "P04-CACHE-INVALID-REPLACEMENT",
        vehicles=[
            _vehicle(
                "MV_INVALID_CACHE",
                "on_ramp",
                6830.0,
                -3.5,
                road_role="on_ramp_mv",
                merge_state="not_started",
            ),
            _vehicle("CLV_OLD", "lane_2", 6884.0, 0.0),
            _vehicle("CFV_EXECUTING_OLD", "lane_2", 6824.0, 0.0, lane_change_state="executing"),
            _vehicle("CFV_REPLACEMENT", "lane_2", 6826.0, 0.0),
            _vehicle("CLV_REPLACEMENT", "lane_2", 6890.0, 0.0),
        ],
        preloaded_assignments=[
            {
                "mv_id": "MV_INVALID_CACHE",
                "clv_id": "CLV_OLD",
                "cfv_id": "CFV_EXECUTING_OLD",
                "aps_case": "case_1",
                "col_clv": False,
                "col_cfv": False,
                "desired_spacing_override": None,
                "status": "valid",
                "created_at_t": 0.0,
                "created_at_step": 0,
                "source": "aps_cache",
                "valid_until_next_aps": True,
            }
        ],
        preloaded_state_machine_states=[
            {"vehicle_id": "MV_INVALID_CACHE", "last_aps_time": 0.0}
        ],
        expected_events=[],
        expected_sanity_checks=[],
    )


def _invalid_cache_without_replacement_config() -> dict[str, Any]:
    return _base_config(
        "P04-CACHE-INVALID-FAILED-FRESH",
        vehicles=[
            _vehicle(
                "MV_INVALID_FAIL",
                "on_ramp",
                6830.0,
                -3.5,
                road_role="on_ramp_mv",
                merge_state="not_started",
            ),
            _vehicle("CLV_OLD_FAIL", "lane_2", 6884.0, 0.0),
            _vehicle("CFV_EXECUTING_OLD", "lane_2", 6824.0, 0.0, lane_change_state="executing"),
        ],
        preloaded_assignments=[
            {
                "mv_id": "MV_INVALID_FAIL",
                "clv_id": "CLV_OLD_FAIL",
                "cfv_id": "CFV_EXECUTING_OLD",
                "aps_case": "case_1",
                "col_clv": False,
                "col_cfv": True,
                "desired_spacing_override": None,
                "status": "valid",
                "created_at_t": 0.0,
                "created_at_step": 0,
                "source": "aps_cache",
                "valid_until_next_aps": True,
            }
        ],
        preloaded_state_machine_states=[
            {"vehicle_id": "MV_INVALID_FAIL", "last_aps_time": 0.0}
        ],
        expected_events=[],
        expected_sanity_checks=[],
    )


def _first_aps_trigger_config() -> dict[str, Any]:
    return _base_config(
        "P04-FIRST-APS-ENGINEERING-PATCH",
        vehicles=[
            _vehicle(
                "MV_FIRST",
                "on_ramp",
                6850.0,
                -3.5,
                road_role="on_ramp_mv",
                merge_state="not_started",
            ),
            _vehicle("CLV_FIRST", "lane_2", 6884.0, 0.0),
            _vehicle("CFV_FIRST", "lane_2", 6824.0, 0.0),
        ],
        expected_events=[
            {
                "event_type": "APS",
                "required": True,
                "vehicle_ids": ["MV_FIRST"],
                "match": {"trigger": "first_APS"},
                "reason_code": "first_aps",
                "source": "first_version_engineering_patch",
            }
        ],
        expected_sanity_checks=[
            {
                "check_type": "x_plot_used_in_algorithm_path",
                "required": True,
                "expected_status": "pass",
                "vehicle_ids": ["MV_FIRST"],
            }
        ],
    )


def _reuse_cache_config() -> dict[str, Any]:
    return _base_config(
        "P04-REUSE-CACHE",
        vehicles=[
            _vehicle(
                "MV_CACHE",
                "on_ramp",
                6890.0,
                -3.5,
                road_role="on_ramp_mv",
                merge_state="not_started",
            ),
            _vehicle("CLV_CACHE", "lane_2", 6920.0, 0.0),
            _vehicle("CFV_CACHE", "lane_2", 6840.0, 0.0),
        ],
        preloaded_assignments=[
            {
                "mv_id": "MV_CACHE",
                "clv_id": "CLV_CACHE",
                "cfv_id": "CFV_CACHE",
                "aps_case": "case_1",
                "col_clv": False,
                "col_cfv": False,
                "desired_spacing_override": None,
                "status": "valid",
                "created_at_t": 0.0,
                "created_at_step": 0,
                "source": "aps_cache",
                "valid_until_next_aps": True,
                "staleness_policy": "reuse_until_next_aps",
            }
        ],
        preloaded_state_machine_states=[
            {"vehicle_id": "MV_CACHE", "last_aps_time": 0.0, "notes": "same-step reuse cache"}
        ],
        expected_events=[
            {
                "event_type": "APS",
                "required": True,
                "vehicle_ids": ["MV_CACHE"],
                "match": {
                    "trigger": "reuse_cache",
                    "effective_assignment_source": "cache_reused",
                },
                "source": "first_version_engineering_patch",
            }
        ],
        expected_sanity_checks=[
            {
                "check_type": "p04_no_write_before_commit",
                "required": True,
                "expected_status": "pass",
                "vehicle_ids": ["MV_CACHE"],
            }
        ],
        expected_png_features=[
            {
                "feature_type": "cache_reuse_marker",
                "required": True,
                "vehicle_ids": ["MV_CACHE"],
                "expected_visibility": "visible",
            }
        ],
    )


def _p04_no_write_contract_config() -> dict[str, Any]:
    return _base_config(
        "P04-NO-WRITE-BEFORE-COMMIT",
        vehicles=[
            _vehicle(
                "MV_NOWRITE",
                "on_ramp",
                6850.0,
                -3.5,
                road_role="on_ramp_mv",
                merge_state="not_started",
            ),
            _vehicle("CLV_NOWRITE", "lane_2", 6884.0, 0.0),
            _vehicle("CFV_NOWRITE", "lane_2", 6824.0, 0.0),
        ],
        expected_events=[
            {
                "event_type": "APS",
                "required": True,
                "vehicle_ids": ["MV_NOWRITE"],
                "match": {"trigger": "first_APS"},
                "source": "first_version_engineering_patch",
            }
        ],
        expected_sanity_checks=[
            {
                "check_type": "p04_no_write_before_commit",
                "required": True,
                "expected_status": "pass",
                "vehicle_ids": ["MV_NOWRITE"],
            }
        ],
    )


def _p04_handoff_to_cmc_config() -> dict[str, Any]:
    return _base_config(
        "P04-HANDOFF-TO-CMC",
        vehicles=[
            _vehicle(
                "MV_HANDOFF",
                "on_ramp",
                6960.0,
                -3.5,
                road_role="on_ramp_mv",
                merge_state="executing",
            ),
            _vehicle("CLV_HANDOFF", "lane_2", 7000.0, 0.0),
            _vehicle("CFV_HANDOFF", "lane_2", 6920.0, 0.0),
        ],
        preloaded_assignments=[
            {
                "mv_id": "MV_HANDOFF",
                "clv_id": "CLV_HANDOFF",
                "cfv_id": "CFV_HANDOFF",
                "aps_case": "case_1",
                "col_clv": False,
                "col_cfv": False,
                "desired_spacing_override": None,
                "status": "valid",
                "created_at_t": -1.0,
                "created_at_step": -10,
                "source": "aps_cache",
                "valid_until_next_aps": True,
                "staleness_policy": "handoff_to_cmc",
            }
        ],
        expected_events=[
            {
                "event_type": "APS",
                "required": True,
                "vehicle_ids": ["MV_HANDOFF"],
                "match": {
                    "branch": "handed_off_to_cmc",
                    "aps_executed": False,
                    "cache_modified": False,
                    "new_assignment_created": False,
                },
                "reason_code": "mv_in_merging_zone",
                "source": "first_version_engineering_patch",
            }
        ],
        expected_sanity_checks=[
            {
                "check_type": "aps_not_applicable",
                "required": True,
                "expected_status": "not_applicable",
                "vehicle_ids": ["MV_HANDOFF"],
                "reason_code": "handed_off_to_cmc",
            }
        ],
        expected_event_counts=[
            {
                "event_type": "assignment_created",
                "vehicle_ids": ["MV_HANDOFF"],
                "expected_count": 0,
                "comparison": "exactly",
            }
        ],
        expected_png_features=[
            {
                "feature_type": "handoff_to_cmc_marker",
                "required": True,
                "vehicle_ids": ["MV_HANDOFF"],
                "expected_visibility": "visible",
            }
        ],
    )


def _matcher_result(report: Any, name: str) -> Any:
    for matcher_result in report.matcher_results:
        if matcher_result.name == name:
            return matcher_result
    raise AssertionError(f"missing matcher result: {name}")


def _actual_event(
    events: list[dict[str, Any]],
    event_type: str,
    vehicle_id: str,
    *,
    reason: str | None = None,
) -> dict[str, Any]:
    for event in events:
        if event.get("event_type") != event_type or event.get("vehicle_id") != vehicle_id:
            continue
        if reason is not None and event.get("reason") != reason:
            continue
        return event
    raise AssertionError(f"missing event: {event_type} {vehicle_id} {reason}")


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
