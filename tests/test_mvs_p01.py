from __future__ import annotations

import pytest

from cormc.mvs import (
    ScenarioConfigError,
    compare_with_tolerance,
    load_builtin_scenario,
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
    config = load_builtin_scenario("MVS-APS-FAIL-EMPTY")
    config["unknown_core_field"] = []
    with pytest.raises(ScenarioConfigError, match="unknown core fields"):
        load_scenario_config(config)


def test_scenario_config_loader_accepts_forbidden_events_and_event_counts() -> None:
    config = load_builtin_scenario("MVS-APS-FAIL-EMPTY")
    assert config["forbidden_events"][0]["event_type"] == "assignment_created"
    assert config["expected_event_counts"][0]["event_type"] == "cooperative_request"
    assert config["expected_event_counts"][0]["expected_count"] == 0


def test_scenario_config_loader_rejects_private_event_count_in_expected_events() -> None:
    config = load_builtin_scenario("MVS-APS-FAIL-EMPTY")
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
    config = load_builtin_scenario("MVS-APS-FAIL-EMPTY")
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
    config = load_builtin_scenario("MVS-APS-FAIL-EMPTY")
    config["expected_event_counts"] = [{"event_type": "cooperative_request"}]
    with pytest.raises(ScenarioConfigError, match="expected_count"):
        load_scenario_config(config)


def test_required_probe_deferred_status_are_distinct() -> None:
    assert load_builtin_scenario("MVS-APS-FAIL-EMPTY")["status"] == "required"
    assert load_builtin_scenario("MVS-CUC-1B_real_utility_probe")["status"] == "probe"
    assert (
        load_builtin_scenario("MVS-CUC-1C_real_utility_choice1_locked")["status"]
        == "deferred"
    )


def test_builtin_initial_vehicle_enums_are_spec_aligned() -> None:
    for scenario_id in ("MVS-APS-FAIL-EMPTY", "MVS-COMMIT-1-lite"):
        config = load_builtin_scenario(scenario_id)
        for vehicle in config["initial_vehicles"]:
            assert vehicle["lane_change_state"] in {"normal", "executing"}
            assert vehicle["merge_state"] in {
                "none",
                "not_started",
                "waiting",
                "executing",
                "merged",
            }
            if vehicle["vehicle_type"].lower() == "cav":
                assert vehicle["compliance_state"] == "not_applicable"


def test_scenario_config_loader_rejects_unknown_lane_change_state() -> None:
    config = load_builtin_scenario("MVS-APS-FAIL-EMPTY")
    config["initial_vehicles"][0]["lane_change_state"] = "not_started"

    with pytest.raises(ScenarioConfigError, match="lane_change_state"):
        load_scenario_config(config)


def test_scenario_config_loader_rejects_unknown_merge_state() -> None:
    config = load_builtin_scenario("MVS-COMMIT-1-lite")
    config["initial_vehicles"][0]["merge_state"] = "waiting_or_not_started"

    with pytest.raises(ScenarioConfigError, match="merge_state"):
        load_scenario_config(config)


def test_scenario_config_loader_rejects_cav_compliance_state_compliant() -> None:
    config = load_builtin_scenario("MVS-COMMIT-1-lite")
    config["initial_vehicles"][0]["compliance_state"] = "compliant"

    with pytest.raises(ScenarioConfigError, match="CAV"):
        load_scenario_config(config)


def test_mvs_aps_fail_empty_failing_contract() -> None:
    report = run_targeted_scenario("MVS-APS-FAIL-EMPTY")
    as_dict = report.to_dict()
    assert as_dict["scenario_id"] == "MVS-APS-FAIL-EMPTY"
    assert as_dict["test_level"] == "unit"
    assert as_dict["status"] == "required"
    assert as_dict["classification"] == "required_failed"
    assert as_dict["blocks_required_suite"] is True
    assert any("missing expected event: APS" in reason for reason in report.failure_reasons)
    assert report.registered_png_features
    assert report.registered_png_features[0]["renderer_status"] == "renderer_deferred"
    matcher_names = [result["name"] for result in as_dict["matcher_results"]]
    assert "forbidden_events" in matcher_names
    assert "expected_event_counts" in matcher_names


def test_runner_consumes_explicit_forbidden_and_event_count_config_fields() -> None:
    report = run_targeted_scenario(
        "MVS-APS-FAIL-EMPTY",
        actual_events=[
            {
                "event_type": "assignment_created",
                "vehicle_id": "MV_FAIL_EMPTY",
                "source": "first_version_engineering_patch",
            },
            {
                "event_type": "cooperative_request",
                "vehicle_id": "MV_FAIL_EMPTY",
                "source": "first_version_engineering_patch",
            },
        ],
    )
    issue_codes = [
        issue.code
        for matcher_result in report.matcher_results
        for issue in matcher_result.issues
    ]
    assert "forbidden_event_present" in issue_codes
    assert "event_count_mismatch" in issue_codes


def test_mvs_commit_1_lite_failing_contract() -> None:
    report = run_targeted_scenario("MVS-COMMIT-1-lite")
    assert report.scenario_id == "MVS-COMMIT-1-lite"
    assert report.status == "required"
    assert report.classification == "required_passed"
    assert report.blocks_required_suite is False
    assert report.failure_reasons == []


def test_probe_does_not_block_required_suite() -> None:
    report = run_targeted_scenario("MVS-CUC-1B_real_utility_probe")
    assert report.status == "probe"
    assert report.passed is True
    assert report.blocks_required_suite is False
    assert report.classification == "probe_failed"
    assert report.failure_reasons


def test_deferred_does_not_enter_required_suite() -> None:
    report = run_targeted_scenario("MVS-CUC-1C_real_utility_choice1_locked")
    assert report.status == "deferred"
    assert report.passed is True
    assert report.blocks_required_suite is False
    assert report.classification == "skipped_deferred"


def test_matcher_reports_required_failure_reason() -> None:
    result = match_expected_events(
        [
            {
                "event_type": "APS",
                "required": True,
                "vehicle_ids": ["MV_FAIL_EMPTY"],
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
                "vehicle_ids": ["MV_FAIL_EMPTY"],
                "reason_code": "insufficient_candidates",
            }
        ],
        [
            {
                "event_type": "APS",
                "vehicle_id": "MV_FAIL_EMPTY",
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
                "vehicle_ids": ["MV_FAIL_EMPTY"],
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
        [{"event_type": "assignment_created", "vehicle_ids": ["MV_FAIL_EMPTY"]}],
        [{"event_type": "assignment_created", "vehicle_id": "MV_FAIL_EMPTY"}],
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
        {"event_type": "cooperative_request", "vehicle_id": "MV_FAIL_EMPTY"},
        {"event_type": "cooperative_request", "vehicle_id": "MV_FAIL_EMPTY"},
    ]
    at_least = match_event_counts(
        [
            {
                "event_type": "cooperative_request",
                "vehicle_ids": ["MV_FAIL_EMPTY"],
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
                "vehicle_ids": ["MV_FAIL_EMPTY"],
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
        [{"event_type": "assignment_created", "vehicle_ids": ["MV_FAIL_EMPTY"]}],
        [{"event_type": "assignment_created", "vehicle_id": "OTHER"}],
    )
    scoped_count = match_event_counts(
        [
            {
                "event_type": "cooperative_request",
                "vehicle_ids": ["MV_FAIL_EMPTY"],
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
                "vehicle_ids": ["MV_COMMIT_LITE"],
            }
        ],
        [
            {
                "check_type": "multiple_commit_for_one_vehicle",
                "result": "fail",
                "vehicle_ids": ["MV_COMMIT_LITE"],
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
                "vehicle_ids": ["MV_FAIL_EMPTY"],
                "expected_visibility": "visible",
            }
        ]
    )
    assert registered == [
        {
            "feature_type": "aps_failure_marker",
            "required": True,
            "vehicle_ids": ["MV_FAIL_EMPTY"],
            "expected_visibility": "visible",
            "registration_status": "feature_registered",
            "renderer_status": "renderer_deferred",
        }
    ]
