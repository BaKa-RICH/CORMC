from __future__ import annotations

import pytest

from cormc.scenario_schema import (
    ScenarioConfigError,
    compare_with_tolerance,
    load_scenario_config,
    match_expected_events,
    match_expected_sanity_checks,
    match_event_counts,
    match_forbidden_events,
    register_expected_png_features,
    run_targeted_scenario,
)


def test_scenario_config_loader_rejects_missing_scenario_id() -> None:
    with pytest.raises(ScenarioConfigError, match="scenario_id"):
        load_scenario_config({"test_level": "unit", "status": "required"})


def test_scenario_config_loader_rejects_unknown_core_fields() -> None:
    config = _minimal_config()
    config["unknown_core_field"] = []

    with pytest.raises(ScenarioConfigError, match="unknown core fields"):
        load_scenario_config(config)


def test_scenario_config_loader_accepts_forbidden_events_and_event_counts() -> None:
    loaded = load_scenario_config(_minimal_config())

    assert loaded["forbidden_events"][0]["event_type"] == "assignment_created"
    assert loaded["expected_event_counts"][0]["event_type"] == "cooperative_request"
    assert loaded["expected_event_counts"][0]["expected_count"] == 0


def test_scenario_config_loader_accepts_preloaded_assignment_t_star_handoff() -> None:
    config = _minimal_config()
    config["preloaded_assignments"] = [
        {
            "mv_id": "MV_A",
            "clv_id": "CLV_A",
            "cfv_id": "CFV_A",
            "aps_case": "case_3",
            "col_clv": True,
            "col_cfv": False,
            "desired_spacing_override": None,
            "t_mv_star": 5.5,
            "t_star_mv": 5.5,
            "status": "valid",
            "created_at_t": 0.0,
            "created_at_step": 0,
            "source": "test_preload",
            "valid_until_next_aps": True,
            "staleness_policy": "valid_until_next_aps",
        }
    ]

    loaded = load_scenario_config(config)

    assignment = loaded["preloaded_assignments"][0]
    assert assignment["t_mv_star"] == 5.5
    assert assignment["t_star_mv"] == 5.5


def test_scenario_config_loader_rejects_private_event_count_in_expected_events() -> None:
    config = _minimal_config()
    config["expected_events"].append(
        {
            "event_type": "cooperative_request",
            "required": True,
            "match": {"event_count": 0},
        }
    )

    with pytest.raises(ScenarioConfigError, match="expected_events must not encode"):
        load_scenario_config(config)


def test_scenario_config_loader_rejects_private_forbidden_in_expected_events() -> None:
    config = _minimal_config()
    config["expected_events"].append(
        {
            "event_type": "assignment_created",
            "required": True,
            "match": {"forbidden": True},
        }
    )

    with pytest.raises(ScenarioConfigError, match="expected_events must not encode"):
        load_scenario_config(config)


def test_scenario_config_loader_rejects_event_count_without_expected_count() -> None:
    config = _minimal_config()
    config["expected_event_counts"] = [{"event_type": "cooperative_request"}]

    with pytest.raises(ScenarioConfigError, match="expected_count"):
        load_scenario_config(config)


def test_scenario_config_loader_rejects_unknown_lane_change_state() -> None:
    config = _minimal_config()
    config["initial_vehicles"][0]["lane_change_state"] = "not_started"

    with pytest.raises(ScenarioConfigError, match="lane_change_state"):
        load_scenario_config(config)


def test_scenario_config_loader_rejects_unknown_merge_state() -> None:
    config = _minimal_config()
    config["initial_vehicles"][0]["merge_state"] = "waiting_or_not_started"

    with pytest.raises(ScenarioConfigError, match="merge_state"):
        load_scenario_config(config)


def test_scenario_config_loader_rejects_cav_compliance_state_compliant() -> None:
    config = _minimal_config()
    config["initial_vehicles"][0]["compliance_state"] = "compliant"

    with pytest.raises(ScenarioConfigError, match="CAV"):
        load_scenario_config(config)


def test_runner_consumes_explicit_config_fields() -> None:
    report = run_targeted_scenario(
        _minimal_config(),
        actual_events=[
            {
                "event_type": "assignment_created",
                "vehicle_id": "MV_SCHEMA",
                "source": "test",
            },
            {
                "event_type": "cooperative_request",
                "vehicle_id": "MV_SCHEMA",
                "source": "test",
            },
        ],
    )
    issue_codes = [
        issue.code
        for matcher_result in report.matcher_results
        for issue in matcher_result.issues
    ]

    assert report.scenario_id == "SCHEMA-MATCHER-UNIT"
    assert "forbidden_event_present" in issue_codes
    assert "event_count_mismatch" in issue_codes


def test_matcher_reports_required_failure_reason() -> None:
    result = match_expected_events(
        [
            {
                "event_type": "APS",
                "required": True,
                "vehicle_ids": ["MV_SCHEMA"],
                "reason_code": "insufficient_candidates",
            }
        ],
        [],
        {"derived_formula_abs": 0.01},
    )
    assert result.passed is False
    assert result.issues[0].code == "missing_event"
    assert "APS" in result.issues[0].message


def test_matcher_reports_reason_mismatch() -> None:
    result = match_expected_events(
        [
            {
                "event_type": "APS",
                "required": True,
                "vehicle_ids": ["MV_SCHEMA"],
                "reason_code": "insufficient_candidates",
            }
        ],
        [
            {
                "event_type": "APS",
                "vehicle_id": "MV_SCHEMA",
                "reason": "other_reason",
                "payload": {},
            }
        ],
        {"derived_formula_abs": 0.01},
    )
    assert result.passed is False
    assert result.issues[0].code == "event_mismatch"


def test_matcher_reports_source_mismatch() -> None:
    result = match_expected_events(
        [
            {
                "event_type": "engineering_patch",
                "required": True,
                "source": "first_version_engineering_patch",
            }
        ],
        [
            {
                "event_type": "engineering_patch",
                "source": "paper_formula",
                "payload": {},
            }
        ],
        {"derived_formula_abs": 0.01},
    )
    assert result.passed is False
    assert result.issues[0].expected["source"] == "first_version_engineering_patch"


def test_engineering_patch_source_can_use_record_flag() -> None:
    result = match_expected_events(
        [
            {
                "event_type": "engineering_patch",
                "required": True,
                "source": "first_version_engineering_patch",
            }
        ],
        [{"event_type": "engineering_patch", "is_engineering_patch": True, "payload": {}}],
        {"derived_formula_abs": 0.01},
    )
    assert result.passed is True


def test_matcher_reports_vehicle_mismatch() -> None:
    result = match_expected_events(
        [
            {
                "event_type": "APS",
                "required": True,
                "vehicle_ids": ["MV_SCHEMA"],
            }
        ],
        [{"event_type": "APS", "vehicle_id": "OTHER", "payload": {}}],
        {"derived_formula_abs": 0.01},
    )
    assert result.passed is False
    assert result.issues[0].code == "event_mismatch"


def test_matcher_reports_time_window_mismatch() -> None:
    result = match_expected_events(
        [
            {
                "event_type": "APS",
                "required": True,
                "time_window": {"step_min": 2, "step_max": 3},
            }
        ],
        [{"event_type": "APS", "step": 1, "payload": {}}],
        {"derived_formula_abs": 0.01},
    )
    assert result.passed is False
    assert result.issues[0].code == "event_mismatch"


def test_tolerance_comes_from_scenario_config() -> None:
    expected = [
        {
            "event_type": "CMC",
            "required": True,
            "numeric_expectations": {
                "gap": {"value": 10.0, "tolerance": "derived_formula_abs"}
            },
        }
    ]
    actual = [{"event_type": "CMC", "payload": {"gap": 10.2}}]
    failed = match_expected_events(expected, actual, {"derived_formula_abs": 0.01})
    passed = match_expected_events(expected, actual, {"derived_formula_abs": 0.25})
    assert failed.passed is False
    assert passed.passed is True
    assert compare_with_tolerance(10.0, 10.2, 0.25) is True


def test_explicit_forbidden_events_and_event_count_matchers() -> None:
    forbidden = match_forbidden_events(
        [{"event_type": "assignment_created", "vehicle_ids": ["MV_SCHEMA"]}],
        [{"event_type": "assignment_created", "vehicle_id": "MV_SCHEMA"}],
    )
    event_count = match_event_counts(
        [{"event_type": "cooperative_request", "expected_count": 0}],
        [],
    )
    assert forbidden.passed is False
    assert forbidden.name == "forbidden_events"
    assert event_count.passed is True
    assert event_count.name == "expected_event_counts"


def test_event_count_comparison_modes() -> None:
    actual_events = [
        {"event_type": "cooperative_request", "vehicle_id": "MV_SCHEMA"},
        {"event_type": "cooperative_request", "vehicle_id": "MV_SCHEMA"},
    ]
    at_least = match_event_counts(
        [
            {
                "event_type": "cooperative_request",
                "vehicle_ids": ["MV_SCHEMA"],
                "expected_count": 1,
                "comparison": "at_least",
            }
        ],
        actual_events,
    )
    at_most = match_event_counts(
        [
            {
                "event_type": "cooperative_request",
                "vehicle_ids": ["MV_SCHEMA"],
                "expected_count": 1,
                "comparison": "at_most",
            }
        ],
        actual_events,
    )
    assert at_least.passed is True
    assert at_most.passed is False


def test_event_count_and_forbidden_matchers_respect_vehicle_scope() -> None:
    allowed_other_vehicle = match_forbidden_events(
        [{"event_type": "assignment_created", "vehicle_ids": ["MV_SCHEMA"]}],
        [{"event_type": "assignment_created", "vehicle_id": "OTHER"}],
    )
    scoped_count = match_event_counts(
        [
            {
                "event_type": "cooperative_request",
                "vehicle_ids": ["MV_SCHEMA"],
                "expected_count": 0,
            }
        ],
        [{"event_type": "cooperative_request", "vehicle_id": "OTHER"}],
    )
    assert allowed_other_vehicle.passed is True
    assert scoped_count.passed is True


def test_expected_sanity_check_status_mismatch() -> None:
    result = match_expected_sanity_checks(
        [
            {
                "check_type": "multiple_commit_for_one_vehicle",
                "required": True,
                "expected_status": "pass",
                "vehicle_ids": ["MV_COMMIT"],
            }
        ],
        [
            {
                "check_type": "multiple_commit_for_one_vehicle",
                "result": "fail",
                "vehicle_ids": ["MV_COMMIT"],
            }
        ],
    )
    assert result.passed is False
    assert result.issues[0].code == "sanity_check_mismatch"


def test_expected_sanity_check_matches_warning_status_vehicle_and_reason() -> None:
    result = match_expected_sanity_checks(
        [
            {
                "check_type": "boundary_speed_cap",
                "required": True,
                "expected_status": "warning",
                "vehicle_ids": ["MV_WARN"],
                "reason_code": "cap_infeasible",
            }
        ],
        [
            {
                "check_type": "boundary_speed_cap",
                "result": "warning",
                "vehicle_ids": ["MV_WARN"],
                "reason": "cap_infeasible",
            }
        ],
    )

    assert result.passed is True
    assert result.issues == []


def test_expected_png_features_register_without_renderer() -> None:
    registered = register_expected_png_features(
        [
            {
                "feature_type": "aps_failure_marker",
                "required": True,
                "vehicle_ids": ["MV_SCHEMA"],
                "expected_visibility": "visible",
            }
        ]
    )
    assert registered == [
        {
            "feature_type": "aps_failure_marker",
            "required": True,
            "vehicle_ids": ["MV_SCHEMA"],
            "expected_visibility": "visible",
            "registration_status": "feature_registered",
            "renderer_status": "renderer_deferred",
        }
    ]


def _minimal_config() -> dict[str, object]:
    return {
        "scenario_id": "SCHEMA-MATCHER-UNIT",
        "scenario_name": "Schema matcher unit config",
        "purpose": "Exercise ScenarioConfig schema and matcher fields.",
        "test_level": "unit",
        "status": "required",
        "initial_time": {"t": 0.0, "step": 0, "dt": 0.1},
        "initial_vehicles": [
            {
                "vehicle_id": "MV_SCHEMA",
                "vehicle_type": "CAV",
                "compliance_state": "not_applicable",
                "initial_x_global": 6640.0,
                "initial_y": -3.5,
                "initial_v": 20.0,
                "initial_a": 0.0,
                "physical_lane": "on_ramp",
                "road_role": "on_ramp_mv",
                "lane_change_state": "normal",
                "merge_state": "not_started",
                "spec_overrides": {},
            }
        ],
        "module_overrides": {
            "boundary_generation_enabled": False,
            "random_arrival_enabled": False,
            "random_vehicle_attributes_enabled": False,
            "ordinary_mainline_lane_change_enabled": False,
            "platoon_cmc_enabled": False,
            "mpc_lateral_tracking_enabled": False,
        },
        "expected_events": [],
        "forbidden_events": [
            {
                "event_type": "assignment_created",
                "vehicle_ids": ["MV_SCHEMA"],
            }
        ],
        "expected_event_counts": [
            {
                "event_type": "cooperative_request",
                "vehicle_ids": ["MV_SCHEMA"],
                "expected_count": 0,
            }
        ],
        "expected_sanity_checks": [],
        "expected_png_features": [],
        "tolerances": {"derived_formula_abs": 0.01},
    }
