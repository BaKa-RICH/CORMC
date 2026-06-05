from __future__ import annotations

from copy import deepcopy
from pathlib import Path
from typing import Any

import yaml


class ScenarioConfigError(ValueError):
    """Raised when ScenarioConfig v0 input violates the upstream field contract."""


SCENARIO_TOP_LEVEL_FIELDS = frozenset(
    {
        "scenario_id",
        "scenario_name",
        "description",
        "purpose",
        "test_level",
        "status",
        "derivation_ref",
        "road_config_ref",
        "parameter_config_ref",
        "control_policy_config_ref",
        "vehicle_generation_config_ref",
        "output_config_ref",
        "initial_time",
        "initial_vehicles",
        "module_overrides",
        "preloaded_assignments",
        "preloaded_state_machine_states",
        "preloaded_maneuver_trajectory_states",
        "expected_events",
        "forbidden_events",
        "expected_event_counts",
        "expected_sanity_checks",
        "expected_png_features",
        "tolerances",
        "notes",
    }
)

INITIAL_TIME_FIELDS = frozenset({"t", "step", "dt"})
INITIAL_VEHICLE_FIELDS = frozenset(
    {
        "vehicle_id",
        "vehicle_type",
        "compliance_state",
        "initial_x_global",
        "initial_y",
        "initial_v",
        "initial_a",
        "physical_lane",
        "road_role",
        "lane_change_state",
        "merge_state",
        "spec_overrides",
    }
)
MODULE_OVERRIDE_FIELDS = frozenset(
    {
        "boundary_generation_enabled",
        "random_arrival_enabled",
        "random_vehicle_attributes_enabled",
        "ordinary_mainline_lane_change_enabled",
        "platoon_cmc_enabled",
        "mpc_lateral_tracking_enabled",
        "quasi_static_longitudinal_override",
        "test_harness_overrides",
    }
)
PRELOADED_ASSIGNMENT_FIELDS = frozenset(
    {
        "mv_id",
        "clv_id",
        "cfv_id",
        "aps_case",
        "col_clv",
        "col_cfv",
        "desired_spacing_override",
        "d_star_clv",
        "d_star_cfv",
        "aps_min_merge_time_gap_s",
        "t_mv_star",
        "t_star_mv",
        "status",
        "created_at_t",
        "created_at_step",
        "source",
        "valid_until_next_aps",
        "staleness_policy",
    }
)
PRELOADED_STATE_FIELDS = frozenset(
    {
        "vehicle_id",
        "lane_change_state",
        "merge_state",
        "last_aps_time",
        "active_request_state",
        "notes",
    }
)
PRELOADED_TRAJECTORY_FIELDS = frozenset(
    {
        "vehicle_id",
        "maneuver_type",
        "start_t",
        "start_step",
        "start_x_global",
        "start_y",
        "target_lane",
        "target_y",
        "planned_length",
        "progress",
        "assigned_clv_id",
        "assigned_cfv_id",
    }
)
EXPECTED_EVENT_FIELDS = frozenset(
    {
        "event_type",
        "required",
        "time_window",
        "vehicle_ids",
        "match",
        "numeric_expectations",
        "reason_code",
        "source",
    }
)
EXPECTED_EVENT_COUNT_FIELDS = frozenset(
    {
        "event_type",
        "time_window",
        "vehicle_ids",
        "match",
        "reason_code",
        "expected_count",
        "comparison",
    }
)
EXPECTED_SANITY_FIELDS = frozenset(
    {
        "check_type",
        "required",
        "expected_status",
        "vehicle_ids",
        "time_window",
        "reason_code",
    }
)
EXPECTED_PNG_FIELDS = frozenset(
    {
        "feature_type",
        "required",
        "vehicle_ids",
        "time_window",
        "expected_visibility",
        "notes",
    }
)
TOLERANCE_FIELDS = frozenset(
    {
        "position_abs_m",
        "speed_abs_mps",
        "time_abs_s",
        "derived_formula_abs",
    }
)

COMPLIANCE_STATES = frozenset({"not_applicable", "compliant", "non_compliant"})
COMPLIANCE_ALIASES = {"none": "not_applicable"}
LANE_CHANGE_STATES = frozenset({"normal", "executing"})
MERGE_STATES = frozenset({"none", "not_started", "waiting", "executing", "merged"})
SANITY_STATUSES = frozenset({"pass", "fail", "warning", "not_applicable"})

DEFAULT_TOLERANCES: dict[str, float] = {
    "position_abs_m": 0.05,
    "speed_abs_mps": 0.05,
    "time_abs_s": 0.1,
    "derived_formula_abs": 0.01,
}


BUILTIN_SCENARIOS: dict[str, dict[str, Any]] = {
    "MVS-APS-FAIL-EMPTY": {
        "scenario_id": "MVS-APS-FAIL-EMPTY",
        "scenario_name": "APS failure when candidate set is insufficient",
        "purpose": "APS candidate shortage and no cache must not create assignment.",
        "test_level": "unit",
        "status": "required",
        "derivation_ref": ["CORMC最小验证场景执行规格.md#5.2"],
        "initial_time": {"t": 0.0, "step": 0, "dt": 0.1},
        "initial_vehicles": [
            {
                "vehicle_id": "MV_FAIL_EMPTY",
                "vehicle_type": "CAV",
                "compliance_state": "not_applicable",
                "initial_x_global": 6830.0,
                "initial_y": -3.5,
                "initial_v": 20.0,
                "initial_a": 0.0,
                "physical_lane": "on_ramp",
                "road_role": "on_ramp_mv",
                "lane_change_state": "normal",
                "merge_state": "not_started",
                "spec_overrides": {},
            },
            {
                "vehicle_id": "ONLY_LANE2_FAIL",
                "vehicle_type": "CAV",
                "compliance_state": "not_applicable",
                "initial_x_global": 6860.0,
                "initial_y": 0.0,
                "initial_v": 20.0,
                "initial_a": 0.0,
                "physical_lane": "lane_2",
                "road_role": "mainline",
                "lane_change_state": "normal",
                "merge_state": "none",
                "spec_overrides": {},
            },
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
        "expected_events": [
            {
                "event_type": "APS",
                "required": True,
                "vehicle_ids": ["MV_FAIL_EMPTY"],
                "match": {"failure": True},
                "reason_code": "insufficient_candidates",
                "source": "paper_formula",
            },
            {
                "event_type": "assignment_cache",
                "required": True,
                "vehicle_ids": ["MV_FAIL_EMPTY"],
                "match": {"previous_cache": None, "new_assignment_created": False},
                "source": "first_version_engineering_patch",
            },
        ],
        "forbidden_events": [
            {
                "event_type": "assignment_created",
                "vehicle_ids": ["MV_FAIL_EMPTY"],
                "source": "first_version_engineering_patch",
            }
        ],
        "expected_event_counts": [
            {
                "event_type": "cooperative_request",
                "vehicle_ids": ["MV_FAIL_EMPTY"],
                "expected_count": 0,
                "comparison": "exactly",
            }
        ],
        "expected_sanity_checks": [
            {
                "check_type": "assignment_invalid",
                "required": True,
                "expected_status": "not_applicable",
                "vehicle_ids": ["MV_FAIL_EMPTY"],
            },
            {
                "check_type": "multiple_commit_for_one_vehicle",
                "required": True,
                "expected_status": "pass",
                "vehicle_ids": ["MV_FAIL_EMPTY"],
            },
        ],
        "expected_png_features": [
            {
                "feature_type": "aps_failure_marker",
                "required": True,
                "vehicle_ids": ["MV_FAIL_EMPTY"],
                "expected_visibility": "visible",
                "notes": "registered only; renderer deferred",
            },
            {
                "feature_type": "assignment_arrow",
                "required": True,
                "vehicle_ids": ["MV_FAIL_EMPTY"],
                "expected_visibility": "not_visible",
                "notes": "no assignment arrow visible",
            },
        ],
        "tolerances": deepcopy(DEFAULT_TOLERANCES),
    },
    "MVS-COMMIT-1-lite": {
        "scenario_id": "MVS-COMMIT-1-lite",
        "scenario_name": "Commit duplicate guard failing contract",
        "purpose": "One active vehicle may commit at most once per step.",
        "test_level": "unit",
        "status": "required",
        "derivation_ref": ["CORMC最小验证场景执行规格.md#7.1"],
        "initial_time": {"t": 0.0, "step": 0, "dt": 0.1},
        "initial_vehicles": [
            {
                "vehicle_id": "MV_COMMIT_LITE",
                "vehicle_type": "CAV",
                "compliance_state": "not_applicable",
                "initial_x_global": 6950.0,
                "initial_y": -3.5,
                "initial_v": 20.0,
                "initial_a": 0.0,
                "physical_lane": "on_ramp",
                "road_role": "on_ramp_mv",
                "lane_change_state": "normal",
                "merge_state": "not_started",
                "spec_overrides": {},
            },
            {
                "vehicle_id": "CLV_COMMIT_LITE",
                "vehicle_type": "CAV",
                "compliance_state": "not_applicable",
                "initial_x_global": 6990.0,
                "initial_y": 0.0,
                "initial_v": 20.0,
                "initial_a": 0.0,
                "physical_lane": "lane_2",
                "road_role": "mainline",
                "lane_change_state": "normal",
                "merge_state": "none",
                "spec_overrides": {},
            },
            {
                "vehicle_id": "CFV_COMMIT_LITE",
                "vehicle_type": "CAV",
                "compliance_state": "not_applicable",
                "initial_x_global": 6920.0,
                "initial_y": 0.0,
                "initial_v": 20.0,
                "initial_a": 0.0,
                "physical_lane": "lane_2",
                "road_role": "mainline",
                "lane_change_state": "normal",
                "merge_state": "none",
                "spec_overrides": {},
            },
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
        "expected_events": [
            {
                "event_type": "commit",
                "required": True,
                "vehicle_ids": ["MV_COMMIT_LITE"],
                "match": {
                    "each_active_vehicle_has_exactly_one_final_next_state": True,
                    "no_module_writes_committed_state_before_commit": True,
                    "command_buffer_and_next_state_buffer_are_separated": True,
                },
                "source": "first_version_engineering_patch",
            }
        ],
        "forbidden_events": [],
        "expected_event_counts": [],
        "expected_sanity_checks": [
            {
                "check_type": "multiple_commit_for_one_vehicle",
                "required": True,
                "expected_status": "pass",
                "vehicle_ids": ["MV_COMMIT_LITE"],
            },
            {
                "check_type": "state_machine_inconsistency",
                "required": True,
                "expected_status": "pass",
                "vehicle_ids": ["MV_COMMIT_LITE"],
            },
        ],
        "expected_png_features": [
            {
                "feature_type": "commit_marker",
                "required": False,
                "vehicle_ids": ["MV_COMMIT_LITE"],
                "expected_visibility": "optional",
                "notes": "registered only; renderer deferred",
            }
        ],
        "tolerances": deepcopy(DEFAULT_TOLERANCES),
    },
    "MVS-CUC-1B_real_utility_probe": {
        "scenario_id": "MVS-CUC-1B_real_utility_probe",
        "scenario_name": "Real CUC utility probe",
        "purpose": "Observe real utility inputs without blocking required acceptance.",
        "test_level": "probe",
        "status": "probe",
        "derivation_ref": ["CORMC最小验证场景执行规格.md#9.2"],
        "initial_time": {"t": 0.0, "step": 0, "dt": 0.1},
        "module_overrides": {
            "boundary_generation_enabled": False,
            "random_arrival_enabled": False,
            "random_vehicle_attributes_enabled": False,
            "test_harness_overrides": {"source": "test_harness_override"},
        },
        "expected_events": [
            {
                "event_type": "CUC",
                "required": True,
                "vehicle_ids": ["CFV_X"],
                "match": {"utility_source": "real_CUC", "utility_inputs_logged": True},
                "source": "paper_formula",
            }
        ],
        "forbidden_events": [],
        "expected_event_counts": [],
        "expected_sanity_checks": [
            {
                "check_type": "state_machine_inconsistency",
                "required": True,
                "expected_status": "pass",
                "vehicle_ids": ["CFV_X"],
            }
        ],
        "expected_png_features": [
            {
                "feature_type": "actual_final_choice",
                "required": False,
                "vehicle_ids": ["CFV_X"],
                "expected_visibility": "optional",
            }
        ],
        "tolerances": deepcopy(DEFAULT_TOLERANCES),
    },
    "MVS-CUC-1C_real_utility_choice1_locked": {
        "scenario_id": "MVS-CUC-1C_real_utility_choice1_locked",
        "scenario_name": "Deferred locked CUC utility choice",
        "purpose": "Reserved strong acceptance after utility formula review.",
        "test_level": "integration",
        "status": "deferred",
        "derivation_ref": ["CORMC最小验证场景执行规格.md#9.3"],
        "initial_time": {"t": 0.0, "step": 0, "dt": 0.1},
        "module_overrides": {
            "boundary_generation_enabled": False,
            "random_arrival_enabled": False,
            "random_vehicle_attributes_enabled": False,
        },
        "expected_events": [
            {
                "event_type": "CUC",
                "required": True,
                "match": {"utility_source": "real_CUC", "final_choice": "change_to_lane_1"},
                "source": "paper_formula",
            }
        ],
        "forbidden_events": [],
        "expected_event_counts": [],
        "expected_sanity_checks": [
            {
                "check_type": "not_executed_in_required_suite_until_locked",
                "required": True,
                "expected_status": "pass",
            }
        ],
        "expected_png_features": [],
        "tolerances": deepcopy(DEFAULT_TOLERANCES),
    },
}


def load_builtin_scenario(scenario_id: str) -> dict[str, Any]:
    try:
        return load_scenario_config(deepcopy(BUILTIN_SCENARIOS[scenario_id]))
    except KeyError as exc:
        raise ScenarioConfigError(f"unknown built-in scenario_id: {scenario_id}") from exc


def load_scenario_config(source: str | Path | dict[str, Any]) -> dict[str, Any]:
    if isinstance(source, (str, Path)):
        data = yaml.safe_load(Path(source).read_text(encoding="utf-8"))
    else:
        data = deepcopy(source)
    if not isinstance(data, dict):
        raise ScenarioConfigError("ScenarioConfig must be a mapping")

    _reject_unknown("ScenarioConfig", data, SCENARIO_TOP_LEVEL_FIELDS)
    _require(data, "scenario_id")
    _require(data, "test_level")
    _require(data, "status")
    data.setdefault("initial_vehicles", [])
    data.setdefault("module_overrides", {})
    data.setdefault("preloaded_assignments", [])
    data.setdefault("preloaded_state_machine_states", [])
    data.setdefault("preloaded_maneuver_trajectory_states", [])
    data.setdefault("expected_events", [])
    data.setdefault("forbidden_events", [])
    data.setdefault("expected_event_counts", [])
    data.setdefault("expected_sanity_checks", [])
    data.setdefault("expected_png_features", [])
    data["tolerances"] = {**DEFAULT_TOLERANCES, **data.get("tolerances", {})}

    _validate_status(data["status"])
    _validate_test_level(data["test_level"])
    _validate_optional_mapping(data, "initial_time", INITIAL_TIME_FIELDS)
    _validate_sequence(data, "initial_vehicles", INITIAL_VEHICLE_FIELDS)
    _validate_optional_mapping(data, "module_overrides", MODULE_OVERRIDE_FIELDS)
    _validate_sequence(data, "preloaded_assignments", PRELOADED_ASSIGNMENT_FIELDS)
    _validate_sequence(
        data,
        "preloaded_state_machine_states",
        PRELOADED_STATE_FIELDS,
    )
    _validate_sequence(
        data,
        "preloaded_maneuver_trajectory_states",
        PRELOADED_TRAJECTORY_FIELDS,
    )
    _validate_sequence(data, "expected_events", EXPECTED_EVENT_FIELDS)
    _validate_sequence(data, "forbidden_events", EXPECTED_EVENT_FIELDS)
    _validate_sequence(data, "expected_event_counts", EXPECTED_EVENT_COUNT_FIELDS)
    _validate_sequence(data, "expected_sanity_checks", EXPECTED_SANITY_FIELDS)
    _validate_sequence(data, "expected_png_features", EXPECTED_PNG_FIELDS)
    _validate_optional_mapping(data, "tolerances", TOLERANCE_FIELDS)
    return data


def classify_scenario_status(config: dict[str, Any]) -> str:
    status = str(config.get("status", "")).lower()
    _validate_status(status)
    return status


def _require(data: dict[str, Any], field: str) -> None:
    if field not in data or data[field] in (None, ""):
        raise ScenarioConfigError(f"ScenarioConfig missing required field: {field}")


def _reject_unknown(label: str, data: dict[str, Any], allowed_fields: frozenset[str]) -> None:
    unknown = sorted(set(data) - allowed_fields)
    if unknown:
        raise ScenarioConfigError(
            f"{label} contains unknown core fields: {', '.join(unknown)}"
        )


def _validate_status(status: Any) -> None:
    if str(status).lower() not in {"required", "optional", "probe", "deferred"}:
        raise ScenarioConfigError(f"unsupported scenario status: {status}")


def _validate_test_level(test_level: Any) -> None:
    if str(test_level).lower() not in {"unit", "integration", "smoke", "probe", "deferred"}:
        raise ScenarioConfigError(f"unsupported test_level: {test_level}")


def _validate_optional_mapping(
    data: dict[str, Any],
    field: str,
    allowed_fields: frozenset[str],
) -> None:
    value = data.get(field)
    if value is None:
        return
    if not isinstance(value, dict):
        raise ScenarioConfigError(f"{field} must be a mapping")
    _reject_unknown(field, value, allowed_fields)


def _validate_sequence(
    data: dict[str, Any],
    field: str,
    allowed_fields: frozenset[str],
) -> None:
    value = data.get(field, [])
    if not isinstance(value, list):
        raise ScenarioConfigError(f"{field} must be a list")
    for index, item in enumerate(value):
        if not isinstance(item, dict):
            raise ScenarioConfigError(f"{field}[{index}] must be a mapping")
        _reject_unknown(f"{field}[{index}]", item, allowed_fields)
        if field == "initial_vehicles":
            _validate_initial_vehicle_enum_values(item, index)
        if field == "preloaded_state_machine_states":
            _validate_state_machine_enum_values(item, index)
        if field == "expected_sanity_checks":
            _validate_expected_sanity_enum_values(item, index)
        if field == "expected_events":
            match = item.get("match") or {}
            if "event_count" in match or "forbidden" in match:
                raise ScenarioConfigError(
                    "expected_events must not encode forbidden events or event counts; "
                    "use forbidden_events or expected_event_counts"
                )
        if field == "expected_event_counts":
            if "expected_count" not in item:
                raise ScenarioConfigError(
                    f"{field}[{index}] missing required field: expected_count"
                )
            comparison = item.get("comparison", "exactly")
            if comparison not in {"exactly", "at_least", "at_most"}:
                raise ScenarioConfigError(
                    f"{field}[{index}] has unsupported comparison: {comparison}"
                )


def _validate_initial_vehicle_enum_values(item: dict[str, Any], index: int) -> None:
    vehicle_type = str(item.get("vehicle_type", "")).lower()
    compliance_state = _normalized_compliance_state(
        item.get("compliance_state", "not_applicable")
    )
    if compliance_state not in COMPLIANCE_STATES:
        raise ScenarioConfigError(
            f"initial_vehicles[{index}] has unsupported compliance_state: "
            f"{item.get('compliance_state')}"
        )
    if vehicle_type == "cav" and compliance_state != "not_applicable":
        raise ScenarioConfigError(
            "CAV initial_vehicles must use compliance_state=not_applicable"
        )

    lane_change_state = str(item.get("lane_change_state", "normal")).lower()
    if lane_change_state not in LANE_CHANGE_STATES:
        raise ScenarioConfigError(
            f"initial_vehicles[{index}] has unsupported lane_change_state: "
            f"{item.get('lane_change_state')}"
        )

    merge_state = str(item.get("merge_state", "none")).lower()
    if merge_state not in MERGE_STATES:
        raise ScenarioConfigError(
            f"initial_vehicles[{index}] has unsupported merge_state: "
            f"{item.get('merge_state')}"
        )


def _validate_state_machine_enum_values(item: dict[str, Any], index: int) -> None:
    if "lane_change_state" in item:
        lane_change_state = str(item["lane_change_state"]).lower()
        if lane_change_state not in LANE_CHANGE_STATES:
            raise ScenarioConfigError(
                f"preloaded_state_machine_states[{index}] has unsupported "
                f"lane_change_state: {item['lane_change_state']}"
            )
    if "merge_state" in item:
        merge_state = str(item["merge_state"]).lower()
        if merge_state not in MERGE_STATES:
            raise ScenarioConfigError(
                f"preloaded_state_machine_states[{index}] has unsupported "
                f"merge_state: {item['merge_state']}"
            )


def _validate_expected_sanity_enum_values(item: dict[str, Any], index: int) -> None:
    expected_status = item.get("expected_status")
    if expected_status is not None and str(expected_status).lower() not in SANITY_STATUSES:
        raise ScenarioConfigError(
            f"expected_sanity_checks[{index}] has unsupported expected_status: "
            f"{expected_status}"
        )


def _normalized_compliance_state(value: Any) -> str:
    lowered = str(value).lower()
    return COMPLIANCE_ALIASES.get(lowered, lowered)


def _p04_vehicle(
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


def _p04_base_scenario(
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
        "purpose": "P04 Step4A APS targeted MVS gate",
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
        "tolerances": deepcopy(DEFAULT_TOLERANCES),
    }


def _p04_fail_cache_scenario() -> dict[str, Any]:
    return _p04_base_scenario(
        "MVS-APS-FAIL-CACHE",
        vehicles=[
            _p04_vehicle(
                "MV_FAIL_CACHE",
                "on_ramp",
                6830.0,
                -3.5,
                road_role="on_ramp_mv",
                merge_state="not_started",
            ),
            _p04_vehicle("ONLY_LANE2_FAIL", "lane_2", 6860.0, 0.0),
            _p04_vehicle("OLD_CLV", "lane_2", 7200.0, 0.0),
            _p04_vehicle("OLD_CFV", "lane_2", 6450.0, 0.0),
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


def _p04_aps_case_scenario(
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
    return _p04_base_scenario(
        scenario_id,
        vehicles=[
            _p04_vehicle(
                "MV_A",
                "on_ramp",
                6850.0,
                -3.5,
                road_role="on_ramp_mv",
                merge_state="not_started",
            ),
            _p04_vehicle(clv_id, "lane_2", clv_x, 0.0),
            _p04_vehicle(cfv_id, "lane_2", cfv_x, 0.0),
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


BUILTIN_SCENARIOS.update(
    {
        "MVS-APS-FAIL-CACHE": _p04_fail_cache_scenario(),
        "MVS-APS-1": _p04_aps_case_scenario(
            scenario_id="MVS-APS-1",
            clv_id="CLV_APS_1",
            clv_x=6884.0,
            cfv_id="CFV_APS_1",
            cfv_x=6824.0,
            aps_case="case_1",
            col_clv=False,
            col_cfv=False,
        ),
        "MVS-APS-2": _p04_aps_case_scenario(
            scenario_id="MVS-APS-2",
            clv_id="CLV_APS_2",
            clv_x=6884.0,
            cfv_id="CFV_APS_2",
            cfv_x=6844.0,
            aps_case="case_2",
            col_clv=False,
            col_cfv=True,
            eq10_spacing=58.0,
        ),
        "MVS-APS-3": _p04_aps_case_scenario(
            scenario_id="MVS-APS-3",
            clv_id="CLV_APS_3",
            clv_x=6864.0,
            cfv_id="CFV_APS_3",
            cfv_x=6824.0,
            aps_case="case_3",
            col_clv=True,
            col_cfv=False,
            forbid_eq10_to_clv=True,
        ),
        "MVS-APS-4": _p04_aps_case_scenario(
            scenario_id="MVS-APS-4",
            clv_id="CLV_APS_4",
            clv_x=6864.0,
            cfv_id="CFV_APS_4",
            cfv_x=6844.0,
            aps_case="case_4",
            col_clv=True,
            col_cfv=True,
            eq10_spacing=52.0,
        ),
    }
)


def _p05_vehicle(
    vehicle_id: str,
    lane: str,
    x_global: float,
    y: float,
    *,
    road_role: str = "mainline",
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
        "lane_change_state": "normal",
        "merge_state": merge_state,
        "spec_overrides": {},
    }


def _p05_assignment(
    *,
    mv_id: str,
    clv_id: str,
    cfv_id: str,
    source: str = "test_preload",
) -> dict[str, Any]:
    return {
        "mv_id": mv_id,
        "clv_id": clv_id,
        "cfv_id": cfv_id,
        "aps_case": "case_1",
        "col_clv": False,
        "col_cfv": False,
        "desired_spacing_override": None,
        "status": "valid",
        "created_at_t": -1.0,
        "created_at_step": -10,
        "source": source,
        "valid_until_next_aps": True,
        "staleness_policy": "valid_until_next_aps",
    }


def _p05_base_scenario(
    scenario_id: str,
    *,
    vehicles: list[dict[str, Any]],
    expected_events: list[dict[str, Any]],
    expected_sanity_checks: list[dict[str, Any]],
    expected_png_features: list[dict[str, Any]],
    status: str = "required",
    test_level: str = "unit",
    preloaded_assignments: list[dict[str, Any]] | None = None,
    preloaded_maneuver_trajectory_states: list[dict[str, Any]] | None = None,
    forbidden_events: list[dict[str, Any]] | None = None,
    expected_event_counts: list[dict[str, Any]] | None = None,
) -> dict[str, Any]:
    return {
        "scenario_id": scenario_id,
        "scenario_name": scenario_id,
        "purpose": "P05 Step4B CMC targeted MVS gate",
        "test_level": test_level,
        "status": status,
        "derivation_ref": ["CORMC最小验证场景执行规格.md#P05"],
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
        "preloaded_state_machine_states": [],
        "preloaded_maneuver_trajectory_states": preloaded_maneuver_trajectory_states or [],
        "expected_events": expected_events,
        "forbidden_events": forbidden_events or [],
        "expected_event_counts": expected_event_counts or [],
        "expected_sanity_checks": expected_sanity_checks,
        "expected_png_features": expected_png_features,
        "tolerances": deepcopy(DEFAULT_TOLERANCES),
    }


def _p05_standard_sanity(mv_id: str) -> list[dict[str, Any]]:
    return [
        {
            "check_type": "assignment_invalid",
            "required": True,
            "expected_status": "pass",
            "vehicle_ids": [mv_id],
        },
        {
            "check_type": "boundary_violation",
            "required": True,
            "expected_status": "pass",
            "vehicle_ids": [mv_id],
        },
        {
            "check_type": "no_write_before_commit",
            "required": True,
            "expected_status": "pass",
            "vehicle_ids": [mv_id],
        },
        {
            "check_type": "x_plot_used_in_algorithm_path",
            "required": True,
            "expected_status": "pass",
            "vehicle_ids": [mv_id],
        },
        {
            "check_type": "state_machine_inconsistency",
            "required": True,
            "expected_status": "pass",
            "vehicle_ids": [mv_id],
        },
    ]


def _p05_common_png(mv_id: str, *extra_vehicle_ids: str) -> list[dict[str, Any]]:
    vehicle_ids = [mv_id, *extra_vehicle_ids]
    return [
        {
            "feature_type": "cmc_decision_marker",
            "required": True,
            "vehicle_ids": vehicle_ids,
            "expected_visibility": "visible",
        },
        {
            "feature_type": "assigned_clv_cfv_marker",
            "required": True,
            "vehicle_ids": vehicle_ids,
            "expected_visibility": "visible",
        },
        {
            "feature_type": "boundary_cap_marker",
            "required": True,
            "vehicle_ids": [mv_id],
            "expected_visibility": "visible",
        },
    ]


def _p05_cmc_1_scenario() -> dict[str, Any]:
    return _p05_base_scenario(
        "MVS-CMC-1",
        vehicles=[
            _p05_vehicle(
                "MV_CMC_1",
                "on_ramp",
                7000.0,
                -3.5,
                road_role="on_ramp_mv",
                merge_state="not_started",
            ),
            _p05_vehicle("CLV_CMC_1", "lane_2", 7030.0, 0.0),
            _p05_vehicle("CFV_CMC_1", "lane_2", 6970.0, 0.0),
        ],
        preloaded_assignments=[
            _p05_assignment(mv_id="MV_CMC_1", clv_id="CLV_CMC_1", cfv_id="CFV_CMC_1")
        ],
        expected_events=[
            {
                "event_type": "CMC",
                "required": True,
                "vehicle_ids": ["MV_CMC_1"],
                "match": {
                    "branch": "cmc_waiting_decision",
                    "zone_state": "merging_zone",
                    "merge_state": "not_started",
                },
                "reason_code": "cmc_waiting_decision",
                "source": "first_version_engineering_patch",
            },
            {
                "event_type": "CMC",
                "required": True,
                "vehicle_ids": ["MV_CMC_1", "CLV_CMC_1", "CFV_CMC_1"],
                "match": {
                    "assignment_source": "test_preload",
                    "assignment_valid": True,
                    "assigned_clv_id": "CLV_CMC_1",
                    "assigned_cfv_id": "CFV_CMC_1",
                },
                "reason_code": "assignment_validation",
                "source": "first_version_engineering_patch",
            },
            {
                "event_type": "CMC",
                "required": True,
                "vehicle_ids": ["MV_CMC_1", "CLV_CMC_1", "CFV_CMC_1"],
                "match": {"eq53_pass": True, "fail_side": None},
                "numeric_expectations": {
                    "h_tilde": {"value": 1.0666666667, "tolerance": "derived_formula_abs"},
                    "threshold": {"value": 21.3333333333, "tolerance": "derived_formula_abs"},
                    "d_MV_to_CLV": {"value": 26.0, "tolerance": "derived_formula_abs"},
                    "d_CFV_to_MV": {"value": 26.0, "tolerance": "derived_formula_abs"},
                },
                "reason_code": "eq53_gap",
                "source": "paper_formula",
            },
            {
                "event_type": "CMC",
                "required": True,
                "vehicle_ids": ["MV_CMC_1"],
                "match": {
                    "cap_source": "boundary_collision_avoidance",
                    "cap_reason": "normal_cap",
                    "cap_feasible": True,
                    "cap_binding": False,
                },
                "reason_code": "boundary_speed_cap",
                "source": "paper_formula",
            },
            {
                "event_type": "CMC",
                "required": True,
                "vehicle_ids": ["MV_CMC_1"],
                "match": {
                    "init_or_continue_maneuver": "init",
                    "target_lane": "lane_2",
                    "target_y": 0.0,
                    "state_transition_request": "executing",
                },
                "reason_code": "merge_start",
                "source": "first_version_engineering_patch",
            },
        ],
        forbidden_events=[
            {
                "event_type": "assignment_invalid",
                "vehicle_ids": ["MV_CMC_1"],
                "source": "first_version_engineering_patch",
            }
        ],
        expected_event_counts=[
            {
                "event_type": "APS",
                "vehicle_ids": ["MV_CMC_1"],
                "expected_count": 0,
                "comparison": "exactly",
            }
        ],
        expected_sanity_checks=_p05_standard_sanity("MV_CMC_1"),
        expected_png_features=[
            *_p05_common_png("MV_CMC_1", "CLV_CMC_1", "CFV_CMC_1"),
            {
                "feature_type": "merge_start_marker",
                "required": True,
                "vehicle_ids": ["MV_CMC_1"],
                "expected_visibility": "visible",
            },
        ],
    )


def _p05_cmc_2_scenario() -> dict[str, Any]:
    return _p05_base_scenario(
        "MVS-CMC-2",
        vehicles=[
            _p05_vehicle(
                "MV_CMC_2",
                "on_ramp",
                7000.0,
                -3.5,
                road_role="on_ramp_mv",
                merge_state="waiting",
            ),
            _p05_vehicle("CLV_CMC_2", "lane_2", 7015.0, 0.0),
            _p05_vehicle("CFV_CMC_2", "lane_2", 6970.0, 0.0),
        ],
        preloaded_assignments=[
            _p05_assignment(mv_id="MV_CMC_2", clv_id="CLV_CMC_2", cfv_id="CFV_CMC_2")
        ],
        expected_events=[
            {
                "event_type": "CMC",
                "required": True,
                "vehicle_ids": ["MV_CMC_2", "CLV_CMC_2", "CFV_CMC_2"],
                "match": {"assignment_valid": True, "assignment_source": "test_preload"},
                "reason_code": "assignment_validation",
                "source": "first_version_engineering_patch",
            },
            {
                "event_type": "CMC",
                "required": True,
                "vehicle_ids": ["MV_CMC_2", "CLV_CMC_2", "CFV_CMC_2"],
                "match": {"eq53_pass": False, "fail_side": "CLV_gap"},
                "numeric_expectations": {
                    "d_MV_to_CLV": {"value": 11.0, "tolerance": "derived_formula_abs"},
                    "d_CFV_to_MV": {"value": 26.0, "tolerance": "derived_formula_abs"},
                    "threshold": {"value": 21.3333333333, "tolerance": "derived_formula_abs"},
                },
                "reason_code": "eq53_gap",
                "source": "paper_formula",
            },
            {
                "event_type": "CMC",
                "required": True,
                "vehicle_ids": ["MV_CMC_2"],
                "match": {"merge_command_created": False, "longitudinal_mode": "cmc_waiting"},
                "reason_code": "waiting_command",
                "source": "first_version_engineering_patch",
            },
            {
                "event_type": "CMC",
                "required": True,
                "vehicle_ids": ["MV_CMC_2"],
                "match": {"cap_feasible": True, "cap_binding": False},
                "reason_code": "boundary_speed_cap",
                "source": "paper_formula",
            },
        ],
        expected_event_counts=[
            {
                "event_type": "CMC",
                "vehicle_ids": ["MV_CMC_2"],
                "match": {"init_or_continue_maneuver": "init"},
                "expected_count": 0,
                "comparison": "exactly",
            },
            {
                "event_type": "APS",
                "vehicle_ids": ["MV_CMC_2"],
                "expected_count": 0,
                "comparison": "exactly",
            },
        ],
        expected_sanity_checks=_p05_standard_sanity("MV_CMC_2"),
        expected_png_features=[
            *_p05_common_png("MV_CMC_2", "CLV_CMC_2", "CFV_CMC_2"),
            {
                "feature_type": "waiting_marker",
                "required": True,
                "vehicle_ids": ["MV_CMC_2"],
                "expected_visibility": "visible",
            },
        ],
    )


def _p05_assign_1_scenario() -> dict[str, Any]:
    return _p05_base_scenario(
        "MVS-ASSIGN-1",
        vehicles=[
            _p05_vehicle(
                "MV_ASSIGN_1",
                "on_ramp",
                7000.0,
                -3.5,
                road_role="on_ramp_mv",
                merge_state="waiting",
            ),
            _p05_vehicle("CLV_ASSIGN_1", "lane_2", 7030.0, 0.0),
            _p05_vehicle("CFV_ASSIGN_1", "lane_1", 6970.0, 3.5),
            _p05_vehicle("ACTUAL_LANE2_FOLLOWER_ASSIGN_1", "lane_2", 6972.0, 0.0),
        ],
        preloaded_assignments=[
            _p05_assignment(
                mv_id="MV_ASSIGN_1",
                clv_id="CLV_ASSIGN_1",
                cfv_id="CFV_ASSIGN_1",
            )
        ],
        expected_events=[
            {
                "event_type": "CMC",
                "required": True,
                "vehicle_ids": ["MV_ASSIGN_1", "CLV_ASSIGN_1", "CFV_ASSIGN_1"],
                "match": {
                    "assignment_valid": False,
                    "invalid_reason": "cfv_not_lane_2",
                    "assigned_cfv_id": "CFV_ASSIGN_1",
                    "replacement_assignment_created": False,
                },
                "reason_code": "assignment_validation",
                "source": "first_version_engineering_patch",
            },
            {
                "event_type": "assignment_invalid",
                "required": True,
                "vehicle_ids": ["MV_ASSIGN_1", "CFV_ASSIGN_1"],
                "match": {
                    "reason": "cfv_not_lane_2",
                    "Eq53_evaluated": False,
                    "merge_command_created": False,
                    "replacement_assignment_created": False,
                },
                "reason_code": "cfv_not_lane_2",
                "source": "first_version_engineering_patch",
            },
            {
                "event_type": "CMC",
                "required": True,
                "vehicle_ids": ["MV_ASSIGN_1"],
                "match": {"merge_command_created": False, "longitudinal_mode": "cmc_waiting"},
                "reason_code": "waiting_command",
                "source": "first_version_engineering_patch",
            },
        ],
        expected_event_counts=[
            {
                "event_type": "CMC",
                "vehicle_ids": ["MV_ASSIGN_1"],
                "match": {"eq53_pass": True},
                "expected_count": 0,
                "comparison": "exactly",
            },
            {
                "event_type": "CMC",
                "vehicle_ids": ["MV_ASSIGN_1"],
                "match": {"init_or_continue_maneuver": "init"},
                "expected_count": 0,
                "comparison": "exactly",
            },
            {
                "event_type": "APS",
                "vehicle_ids": ["MV_ASSIGN_1"],
                "expected_count": 0,
                "comparison": "exactly",
            },
        ],
        expected_sanity_checks=[
            {
                "check_type": "assignment_invalid",
                "required": True,
                "expected_status": "warning",
                "vehicle_ids": ["MV_ASSIGN_1"],
                "reason_code": "cfv_not_lane_2",
            },
            {
                "check_type": "no_write_before_commit",
                "required": True,
                "expected_status": "pass",
                "vehicle_ids": ["MV_ASSIGN_1"],
            },
            {
                "check_type": "x_plot_used_in_algorithm_path",
                "required": True,
                "expected_status": "pass",
                "vehicle_ids": ["MV_ASSIGN_1"],
            },
        ],
        expected_png_features=[
            *_p05_common_png("MV_ASSIGN_1", "CLV_ASSIGN_1", "CFV_ASSIGN_1"),
            {
                "feature_type": "assignment_invalid_marker",
                "required": True,
                "vehicle_ids": ["MV_ASSIGN_1", "CFV_ASSIGN_1"],
                "expected_visibility": "visible",
            },
            {
                "feature_type": "no_replacement_assignment_arrow",
                "required": True,
                "vehicle_ids": ["MV_ASSIGN_1", "ACTUAL_LANE2_FOLLOWER_ASSIGN_1"],
                "expected_visibility": "not_visible",
            },
        ],
    )


def _p05_safe_1a_waiting_cap_scenario() -> dict[str, Any]:
    return _p05_base_scenario(
        "MVS-SAFE-1A_waiting_cap",
        status="probe",
        test_level="probe",
        vehicles=[
            _p05_vehicle(
                "MV_SAFE_1A",
                "on_ramp",
                7235.0,
                -3.5,
                road_role="on_ramp_mv",
                merge_state="waiting",
            ),
            _p05_vehicle("CLV_SAFE_1A", "lane_2", 7245.0, 0.0),
            _p05_vehicle("CFV_SAFE_1A", "lane_2", 7200.0, 0.0),
        ],
        preloaded_assignments=[
            _p05_assignment(mv_id="MV_SAFE_1A", clv_id="CLV_SAFE_1A", cfv_id="CFV_SAFE_1A")
        ],
        expected_events=[
            {
                "event_type": "CMC",
                "required": True,
                "vehicle_ids": ["MV_SAFE_1A"],
                "match": {
                    "cap_source": "boundary_collision_avoidance",
                    "cap_reason": "normal_cap",
                    "cap_feasible": True,
                    "cap_binding": True,
                },
                "numeric_expectations": {
                    "boundary_speed_cap": {"value": 2.6295, "tolerance": "derived_formula_abs"}
                },
                "reason_code": "boundary_speed_cap",
                "source": "paper_formula",
            },
            {
                "event_type": "CMC",
                "required": True,
                "vehicle_ids": ["MV_SAFE_1A"],
                "match": {"eq53_pass": False, "fail_side": "CLV_gap"},
                "reason_code": "eq53_gap",
                "source": "paper_formula",
            },
        ],
        expected_sanity_checks=[
            {
                "check_type": "boundary_violation",
                "required": True,
                "expected_status": "pass",
                "vehicle_ids": ["MV_SAFE_1A"],
            },
            {
                "check_type": "assignment_invalid",
                "required": True,
                "expected_status": "pass",
                "vehicle_ids": ["MV_SAFE_1A"],
            },
        ],
        expected_png_features=[
            *_p05_common_png("MV_SAFE_1A", "CLV_SAFE_1A", "CFV_SAFE_1A"),
            {
                "feature_type": "waiting_marker",
                "required": True,
                "vehicle_ids": ["MV_SAFE_1A"],
                "expected_visibility": "visible",
            },
        ],
    )


def _p05_executing_continuation_scenario() -> dict[str, Any]:
    return _p05_base_scenario(
        "P05-EXECUTING-CONTINUATION",
        vehicles=[
            _p05_vehicle(
                "MV_EXECUTING",
                "on_ramp",
                7050.0,
                -2.0,
                road_role="on_ramp_mv",
                merge_state="executing",
            ),
            _p05_vehicle("CLV_EXECUTING", "lane_2", 7085.0, 0.0),
            _p05_vehicle("CFV_EXECUTING", "lane_2", 7015.0, 0.0),
        ],
        preloaded_assignments=[
            _p05_assignment(
                mv_id="MV_EXECUTING",
                clv_id="CLV_EXECUTING",
                cfv_id="CFV_EXECUTING",
            )
        ],
        preloaded_maneuver_trajectory_states=[
            {
                "vehicle_id": "MV_EXECUTING",
                "maneuver_type": "merge",
                "start_t": -0.5,
                "start_step": -5,
                "start_x_global": 7035.0,
                "start_y": -3.5,
                "target_lane": "lane_2",
                "target_y": 0.0,
                "planned_length": 120.0,
                "progress": 0.2,
                "assigned_clv_id": "CLV_EXECUTING",
                "assigned_cfv_id": "CFV_EXECUTING",
            }
        ],
        expected_events=[
            {
                "event_type": "CMC",
                "required": True,
                "vehicle_ids": ["MV_EXECUTING"],
                "match": {
                    "branch": "cmc_executing_continuation",
                    "merge_state": "executing",
                },
                "reason_code": "cmc_executing_continuation",
                "source": "first_version_engineering_patch",
            },
            {
                "event_type": "CMC",
                "required": True,
                "vehicle_ids": ["MV_EXECUTING"],
                "match": {
                    "init_or_continue_maneuver": "continue",
                    "no_new_eq53_start_decision": True,
                    "does_not_rejudge_merge_start": True,
                },
                "reason_code": "executing_continuation",
                "source": "first_version_engineering_patch",
            },
            {
                "event_type": "CMC",
                "required": True,
                "vehicle_ids": ["MV_EXECUTING"],
                "match": {"cap_feasible": True},
                "reason_code": "boundary_speed_cap",
                "source": "paper_formula",
            },
        ],
        expected_event_counts=[
            {
                "event_type": "CMC",
                "vehicle_ids": ["MV_EXECUTING"],
                "match": {"eq53_pass": True},
                "expected_count": 0,
                "comparison": "exactly",
            },
            {
                "event_type": "CMC",
                "vehicle_ids": ["MV_EXECUTING"],
                "match": {"assignment_validation_evaluated": True},
                "expected_count": 0,
                "comparison": "exactly",
            },
        ],
        expected_sanity_checks=[
            {
                "check_type": "no_write_before_commit",
                "required": True,
                "expected_status": "pass",
                "vehicle_ids": ["MV_EXECUTING"],
            },
            {
                "check_type": "boundary_violation",
                "required": True,
                "expected_status": "pass",
                "vehicle_ids": ["MV_EXECUTING"],
            },
        ],
        expected_png_features=[
            {
                "feature_type": "executing_continuation_marker",
                "required": True,
                "vehicle_ids": ["MV_EXECUTING"],
                "expected_visibility": "visible",
            },
            {
                "feature_type": "boundary_cap_marker",
                "required": True,
                "vehicle_ids": ["MV_EXECUTING"],
                "expected_visibility": "visible",
            },
        ],
    )


BUILTIN_SCENARIOS.update(
    {
        "MVS-CMC-1": _p05_cmc_1_scenario(),
        "MVS-CMC-2": _p05_cmc_2_scenario(),
        "MVS-ASSIGN-1": _p05_assign_1_scenario(),
        "MVS-SAFE-1A_waiting_cap": _p05_safe_1a_waiting_cap_scenario(),
        "P05-EXECUTING-CONTINUATION": _p05_executing_continuation_scenario(),
    }
)


def _p06_vehicle(
    vehicle_id: str,
    lane: str,
    x_global: float,
    y: float,
    *,
    road_role: str = "mainline",
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
        "lane_change_state": "normal",
        "merge_state": merge_state,
        "spec_overrides": {},
    }


def _p06_assignment(
    *,
    mv_id: str,
    clv_id: str,
    cfv_id: str,
    aps_case: str,
    col_clv: bool,
    col_cfv: bool,
    t_mv_star: float,
    source: str = "test_preload",
) -> dict[str, Any]:
    return {
        "mv_id": mv_id,
        "clv_id": clv_id,
        "cfv_id": cfv_id,
        "aps_case": aps_case,
        "col_clv": col_clv,
        "col_cfv": col_cfv,
        "desired_spacing_override": None,
        "t_mv_star": t_mv_star,
        "t_star_mv": t_mv_star,
        "status": "valid",
        "created_at_t": -1.0,
        "created_at_step": -10,
        "source": source,
        "valid_until_next_aps": True,
        "staleness_policy": "valid_until_next_aps",
    }


def _p06_base_scenario(
    scenario_id: str,
    *,
    vehicles: list[dict[str, Any]],
    preloaded_assignments: list[dict[str, Any]],
    expected_events: list[dict[str, Any]],
    expected_event_counts: list[dict[str, Any]],
    expected_png_features: list[dict[str, Any]],
) -> dict[str, Any]:
    return {
        "scenario_id": scenario_id,
        "scenario_name": scenario_id,
        "purpose": "P06 Step5 cooperative request conflict resolution targeted MVS gate",
        "test_level": "unit",
        "status": "required",
        "derivation_ref": ["P06-Step5_CooperativeRequest_ConflictResolution.md#6"],
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
        "preloaded_assignments": preloaded_assignments,
        "preloaded_state_machine_states": [],
        "preloaded_maneuver_trajectory_states": [],
        "expected_events": expected_events,
        "forbidden_events": [
            {"event_type": "APS_candidate"},
            {"event_type": "APS"},
            {"event_type": "CMC"},
            {"event_type": "CUC"},
            {"event_type": "lane_change_command"},
        ],
        "expected_event_counts": expected_event_counts,
        "expected_sanity_checks": [
            {
                "check_type": "no_write_before_commit",
                "required": True,
                "expected_status": "pass",
            }
        ],
        "expected_png_features": expected_png_features,
        "tolerances": deepcopy(DEFAULT_TOLERANCES),
    }


def _p06_conflict_1a_scenario() -> dict[str, Any]:
    cv_id = "CV_X"
    return _p06_base_scenario(
        "MVS-CONFLICT-1A",
        vehicles=[
            _p06_vehicle(
                "MV_A",
                "on_ramp",
                6970.0,
                -3.5,
                road_role="on_ramp_mv",
                merge_state="waiting",
            ),
            _p06_vehicle(
                "MV_B",
                "on_ramp",
                6840.0,
                -3.5,
                road_role="on_ramp_mv",
                merge_state="not_started",
            ),
            _p06_vehicle(cv_id, "lane_2", 6920.0, 0.0),
            _p06_vehicle("CLV_FOR_MV_A", "lane_2", 7040.0, 0.0),
            _p06_vehicle("CFV_FOR_MV_B", "lane_2", 6800.0, 0.0),
        ],
        preloaded_assignments=[
            _p06_assignment(
                mv_id="MV_A",
                clv_id="CLV_FOR_MV_A",
                cfv_id=cv_id,
                aps_case="case_2",
                col_clv=False,
                col_cfv=True,
                t_mv_star=4.0,
            ),
            _p06_assignment(
                mv_id="MV_B",
                clv_id=cv_id,
                cfv_id="CFV_FOR_MV_B",
                aps_case="case_3",
                col_clv=True,
                col_cfv=False,
                t_mv_star=5.5,
            ),
        ],
        expected_events=[
            {
                "event_type": "cooperative_request",
                "required": True,
                "vehicle_ids": ["MV_A", cv_id],
                "match": {
                    "source_mv_id": "MV_A",
                    "cv_id": cv_id,
                    "cv_role": "cfv",
                    "col": True,
                    "mv_in_merging_zone": True,
                },
                "reason_code": "col_cfv_request",
                "source": "paper_formula",
            },
            {
                "event_type": "cooperative_request",
                "required": True,
                "vehicle_ids": ["MV_B", cv_id],
                "match": {
                    "source_mv_id": "MV_B",
                    "cv_id": cv_id,
                    "cv_role": "clv",
                    "col": True,
                    "mv_in_merging_zone": False,
                },
                "reason_code": "col_clv_request",
                "source": "paper_formula",
            },
            {
                "event_type": "conflict_resolution",
                "required": True,
                "vehicle_ids": ["MV_A", "MV_B", cv_id],
                "match": {
                    "cv_id": cv_id,
                    "winner_mv_id": "MV_A",
                    "loser_mv_ids": ["MV_B"],
                    "priority_basis": "MV_in_merging_zone",
                    "active_request_count_for_cv": 1,
                    "one_active_request_per_cv": True,
                    "conflicting_commands_to_same_CV": False,
                },
                "reason_code": "MV_in_merging_zone",
                "source": "first_version_engineering_patch",
            },
        ],
        expected_event_counts=[
            {
                "event_type": "cooperative_request",
                "vehicle_ids": [cv_id],
                "expected_count": 2,
                "comparison": "exactly",
            },
            {
                "event_type": "conflict_resolution",
                "vehicle_ids": [cv_id],
                "expected_count": 1,
                "comparison": "exactly",
            },
        ],
        expected_png_features=[
            {
                "feature_type": "cooperative_request_marker",
                "required": True,
                "vehicle_ids": ["MV_A", "MV_B", cv_id],
                "expected_visibility": "visible",
            },
            {
                "feature_type": "conflict_group_marker",
                "required": True,
                "vehicle_ids": ["MV_A", "MV_B", cv_id],
                "expected_visibility": "visible",
            },
            {
                "feature_type": "active_request_marker",
                "required": True,
                "vehicle_ids": ["MV_A", cv_id],
                "expected_visibility": "visible",
            },
            {
                "feature_type": "suppressed_request_marker",
                "required": True,
                "vehicle_ids": ["MV_B", cv_id],
                "expected_visibility": "visible",
            },
        ],
    )


def _p06_conflict_1b_scenario() -> dict[str, Any]:
    cv_id = "SHARED_CLV_G"
    return _p06_base_scenario(
        "MVS-CONFLICT-1B",
        vehicles=[
            _p06_vehicle(
                "MV_G1",
                "on_ramp",
                6840.0,
                -3.5,
                road_role="on_ramp_mv",
                merge_state="not_started",
            ),
            _p06_vehicle(
                "MV_G2",
                "on_ramp",
                6830.0,
                -3.5,
                road_role="on_ramp_mv",
                merge_state="not_started",
            ),
            _p06_vehicle(cv_id, "lane_2", 6920.0, 0.0),
            _p06_vehicle("CFV_G1", "lane_2", 6800.0, 0.0),
            _p06_vehicle("CFV_G2", "lane_2", 6790.0, 0.0),
        ],
        preloaded_assignments=[
            _p06_assignment(
                mv_id="MV_G1",
                clv_id=cv_id,
                cfv_id="CFV_G1",
                aps_case="case_3",
                col_clv=True,
                col_cfv=False,
                t_mv_star=5.5,
            ),
            _p06_assignment(
                mv_id="MV_G2",
                clv_id=cv_id,
                cfv_id="CFV_G2",
                aps_case="case_3",
                col_clv=True,
                col_cfv=False,
                t_mv_star=6.0,
            ),
        ],
        expected_events=[
            {
                "event_type": "cooperative_request",
                "required": True,
                "vehicle_ids": ["MV_G1", cv_id],
                "match": {
                    "source_mv_id": "MV_G1",
                    "cv_id": cv_id,
                    "cv_role": "clv",
                    "col": True,
                    "t_mv_star": 5.5,
                    "mv_in_merging_zone": False,
                },
                "reason_code": "col_clv_request",
                "source": "paper_formula",
            },
            {
                "event_type": "cooperative_request",
                "required": True,
                "vehicle_ids": ["MV_G2", cv_id],
                "match": {
                    "source_mv_id": "MV_G2",
                    "cv_id": cv_id,
                    "cv_role": "clv",
                    "col": True,
                    "t_mv_star": 6.0,
                    "mv_in_merging_zone": False,
                },
                "reason_code": "col_clv_request",
                "source": "paper_formula",
            },
            {
                "event_type": "conflict_resolution",
                "required": True,
                "vehicle_ids": ["MV_G1", "MV_G2", cv_id],
                "match": {
                    "cv_id": cv_id,
                    "winner_mv_id": "MV_G1",
                    "loser_mv_ids": ["MV_G2"],
                    "priority_basis": "smaller_T_star_MV",
                    "active_request_count_for_cv": 1,
                    "one_active_request_per_cv": True,
                    "conflicting_commands_to_same_CV": False,
                },
                "reason_code": "smaller_T_star_MV",
                "source": "first_version_engineering_patch",
            },
        ],
        expected_event_counts=[
            {
                "event_type": "cooperative_request",
                "vehicle_ids": [cv_id],
                "expected_count": 2,
                "comparison": "exactly",
            },
            {
                "event_type": "conflict_resolution",
                "vehicle_ids": [cv_id],
                "expected_count": 1,
                "comparison": "exactly",
            },
        ],
        expected_png_features=[
            {
                "feature_type": "cooperative_request_marker",
                "required": True,
                "vehicle_ids": ["MV_G1", "MV_G2", cv_id],
                "expected_visibility": "visible",
            },
            {
                "feature_type": "conflict_group_marker",
                "required": True,
                "vehicle_ids": ["MV_G1", "MV_G2", cv_id],
                "expected_visibility": "visible",
            },
            {
                "feature_type": "active_request_marker",
                "required": True,
                "vehicle_ids": ["MV_G1", cv_id],
                "expected_visibility": "visible",
            },
            {
                "feature_type": "suppressed_request_marker",
                "required": True,
                "vehicle_ids": ["MV_G2", cv_id],
                "expected_visibility": "visible",
            },
        ],
    )


BUILTIN_SCENARIOS.update(
    {
        "MVS-CONFLICT-1A": _p06_conflict_1a_scenario(),
        "MVS-CONFLICT-1B": _p06_conflict_1b_scenario(),
    }
)


def _p12_vehicle(
    vehicle_id: str,
    lane: str,
    x_global: float,
    y: float,
    *,
    road_role: str = "mainline",
    vehicle_type: str = "CAV",
    compliance_state: str = "not_applicable",
    initial_v: float = 20.0,
    lane_change_state: str = "normal",
    merge_state: str = "none",
    desired_speed: float | None = 20.0,
) -> dict[str, Any]:
    spec_overrides: dict[str, Any] = {}
    if desired_speed is not None:
        spec_overrides["desired_speed"] = desired_speed
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
        "spec_overrides": spec_overrides,
    }


def _p12_module_overrides(
    *,
    cuc_utility_overrides: dict[str, dict[str, Any]] | None = None,
) -> dict[str, Any]:
    return {
        "boundary_generation_enabled": False,
        "random_arrival_enabled": False,
        "random_vehicle_attributes_enabled": False,
        "ordinary_mainline_lane_change_enabled": False,
        "platoon_cmc_enabled": False,
        "mpc_lateral_tracking_enabled": False,
        "test_harness_overrides": {
            "source": "test_harness_override",
            "cuc_utility_overrides": cuc_utility_overrides or {},
        },
    }


def _p12_assignment(
    *,
    mv_id: str,
    clv_id: str,
    cfv_id: str,
    aps_case: str,
    col_clv: bool,
    col_cfv: bool,
    desired_spacing_override: float | None,
    d_star_clv: float = 30.0,
    d_star_cfv: float = -24.0,
    aps_min_merge_time_gap_s: float = 1.2,
    t_mv_star: float = 5.0,
    created_at_t: float = 0.0,
    created_at_step: int = 0,
) -> dict[str, Any]:
    return {
        "mv_id": mv_id,
        "clv_id": clv_id,
        "cfv_id": cfv_id,
        "aps_case": aps_case,
        "col_clv": col_clv,
        "col_cfv": col_cfv,
        "desired_spacing_override": desired_spacing_override,
        "d_star_clv": d_star_clv,
        "d_star_cfv": d_star_cfv,
        "aps_min_merge_time_gap_s": aps_min_merge_time_gap_s,
        "t_mv_star": t_mv_star,
        "t_star_mv": t_mv_star,
        "status": "valid",
        "created_at_t": created_at_t,
        "created_at_step": created_at_step,
        "source": "test_preload",
        "valid_until_next_aps": True,
        "staleness_policy": "valid_until_next_aps",
    }


def _p12_e2e_1_scenario() -> dict[str, Any]:
    return {
        "scenario_id": "MVS-E2E-1",
        "scenario_name": "Deterministic APS CMC merge loop",
        "purpose": "P12 deterministic runner route for APS case 1 through CMC/P08/P09/P10.",
        "test_level": "smoke",
        "status": "required",
        "derivation_ref": ["CORMC minimal validation scenario spec #6.1"],
        "initial_time": {"t": 0.0, "step": 0, "dt": 0.1},
        "initial_vehicles": [
            _p12_vehicle(
                "MV_DEMO",
                "on_ramp",
                6830.0,
                -3.5,
                road_role="on_ramp_mv",
                merge_state="not_started",
            ),
            _p12_vehicle(
                "CLV_DEMO",
                "lane_2",
                6870.0,
                0.0,
                vehicle_type="CHV",
                compliance_state="compliant",
            ),
            _p12_vehicle(
                "CFV_DEMO",
                "lane_2",
                6800.0,
                0.0,
                vehicle_type="CHV",
                compliance_state="compliant",
            ),
            _p12_vehicle("BG_LANE1_DEMO", "lane_1", 6840.0, 3.5),
        ],
        "module_overrides": _p12_module_overrides(),
        "preloaded_assignments": [],
        "preloaded_state_machine_states": [],
        "preloaded_maneuver_trajectory_states": [],
        "expected_events": [
            {
                "event_type": "APS",
                "required": True,
                "vehicle_ids": ["MV_DEMO", "CLV_DEMO", "CFV_DEMO"],
                "match": {
                    "trigger": "first_APS",
                    "aps_case": "case_1",
                    "clv_id": "CLV_DEMO",
                    "cfv_id": "CFV_DEMO",
                    "col_clv": False,
                    "col_cfv": False,
                },
                "reason_code": "first_aps",
                "source": "paper_formula",
            },
            {
                "event_type": "CMC",
                "required": True,
                "vehicle_ids": ["MV_DEMO", "CLV_DEMO", "CFV_DEMO"],
                "match": {"assignment_valid": True, "assigned_clv_id": "CLV_DEMO", "assigned_cfv_id": "CFV_DEMO"},
                "reason_code": "assignment_validation",
                "source": "first_version_engineering_patch",
            },
            {
                "event_type": "CMC",
                "required": True,
                "vehicle_ids": ["MV_DEMO", "CLV_DEMO", "CFV_DEMO"],
                "match": {"eq53_pass": True, "fail_side": None},
                "reason_code": "eq53_gap",
                "source": "paper_formula",
            },
            {
                "event_type": "CMC",
                "required": True,
                "vehicle_ids": ["MV_DEMO"],
                "match": {
                    "init_or_continue_maneuver": "init",
                    "target_lane": "lane_2",
                    "target_y": 0.0,
                    "state_transition_request": "executing",
                },
                "reason_code": "merge_start",
                "source": "first_version_engineering_patch",
            },
            {
                "event_type": "longitudinal_model",
                "required": True,
                "vehicle_ids": ["MV_DEMO"],
                "match": {"longitudinal_mode": "mv_on_ramp"},
                "source": "first_version_engineering_patch",
            },
            {
                "event_type": "lateral_trajectory",
                "required": True,
                "vehicle_ids": ["MV_DEMO"],
                "match": {"maneuver_type": "merge"},
                "source": "paper_formula",
            },
            {
                "event_type": "commit",
                "required": True,
                "vehicle_ids": ["MV_DEMO"],
                "match": {
                    "each_active_vehicle_has_exactly_one_final_next_state": True,
                    "commit_is_unique_state_writer": True,
                },
                "source": "first_version_engineering_patch",
            },
            {
                "event_type": "information_integration",
                "required": True,
                "vehicle_ids": ["MV_DEMO"],
                "match": {"step10_does_not_rewrite_committed_state": True},
                "source": "first_version_engineering_patch",
            },
            {
                "event_type": "time_advance",
                "required": True,
                "vehicle_ids": ["MV_DEMO"],
                "match": {"advanced_after_commit": True},
                "source": "first_version_engineering_patch",
            },
        ],
        "forbidden_events": [],
        "expected_event_counts": [
            {
                "event_type": "cooperative_request",
                "vehicle_ids": ["MV_DEMO"],
                "expected_count": 0,
                "comparison": "exactly",
            }
        ],
        "expected_sanity_checks": [
            {
                "check_type": "multiple_commit_for_one_vehicle",
                "required": True,
                "expected_status": "pass",
                "vehicle_ids": ["MV_DEMO"],
            },
            {
                "check_type": "state_machine_inconsistency",
                "required": True,
                "expected_status": "pass",
                "vehicle_ids": ["MV_DEMO"],
            },
            {
                "check_type": "p12_command_namespace_conflict",
                "required": True,
                "expected_status": "pass",
                "vehicle_ids": ["MV_DEMO"],
            },
            {
                "check_type": "x_plot_used_in_algorithm_path",
                "required": True,
                "expected_status": "pass",
                "vehicle_ids": ["MV_DEMO"],
            },
        ],
        "expected_png_features": [
            {
                "feature_type": "aps_assignment_marker",
                "required": True,
                "vehicle_ids": ["MV_DEMO", "CLV_DEMO", "CFV_DEMO"],
                "expected_visibility": "visible",
            },
            {
                "feature_type": "merge_start_marker",
                "required": True,
                "vehicle_ids": ["MV_DEMO"],
                "expected_visibility": "visible",
            },
            {
                "feature_type": "merge_trajectory_marker",
                "required": True,
                "vehicle_ids": ["MV_DEMO"],
                "expected_visibility": "visible",
            },
            {
                "feature_type": "commit_marker",
                "required": True,
                "vehicle_ids": ["MV_DEMO"],
                "expected_visibility": "visible",
            },
        ],
        "tolerances": deepcopy(DEFAULT_TOLERANCES),
    }


def _p12_cuc_lanechange_scenario() -> dict[str, Any]:
    return {
        "scenario_id": "P12-BRANCH-CUC-LANECHANGE",
        "scenario_name": "P12 CUC lane-change branch",
        "purpose": "Deterministic CUC override starts a lane-change through the full loop.",
        "test_level": "integration",
        "status": "probe",
        "derivation_ref": ["P12 deterministic branch scenario"],
        "initial_time": {"t": 0.0, "step": 0, "dt": 0.1},
        "initial_vehicles": [
            _p12_vehicle("MV_CUC", "on_ramp", 6850.0, -3.5, road_role="on_ramp_mv", merge_state="not_started"),
            _p12_vehicle("CFV_X", "lane_2", 6844.0, 0.0),
            _p12_vehicle("CLV_Y", "lane_2", 6884.0, 0.0),
            _p12_vehicle("TLV", "lane_1", 6920.0, 3.5),
            _p12_vehicle("TFV", "lane_1", 6750.0, 3.5),
        ],
        "module_overrides": _p12_module_overrides(
            cuc_utility_overrides={"CFV_X": {"recommended_choice": "change_to_lane_1", "U1": 1.0, "U2": 0.0}}
        ),
        "preloaded_assignments": [],
        "preloaded_state_machine_states": [],
        "preloaded_maneuver_trajectory_states": [],
        "expected_events": [],
        "forbidden_events": [],
        "expected_event_counts": [],
        "expected_sanity_checks": [],
        "expected_png_features": [
            {"feature_type": "lane_change_intent_marker", "required": True, "vehicle_ids": ["CFV_X"], "expected_visibility": "visible"},
            {"feature_type": "lane_change_trajectory_marker", "required": True, "vehicle_ids": ["CFV_X"], "expected_visibility": "visible"},
        ],
        "tolerances": deepcopy(DEFAULT_TOLERANCES),
    }


def _p12_demo_e2e_scenario() -> dict[str, Any]:
    scenario = _p12_e2e_1_scenario()
    scenario["scenario_id"] = "P12-DEMO-E2E"
    scenario["status"] = "probe"
    scenario["test_level"] = "integration"
    scenario["scenario_name"] = "P12 deterministic E2E demo"
    return scenario


def _p12_cuc_fallback_scenario(*, non_compliant: bool = False) -> dict[str, Any]:
    scenario_id = "P12-BRANCH-CUC-NONCOMPLIANT" if non_compliant else "P12-BRANCH-CUC-FALLBACK"
    vehicle_type = "CHV" if non_compliant else "CAV"
    compliance = "non_compliant" if non_compliant else "not_applicable"
    return {
        "scenario_id": scenario_id,
        "scenario_name": scenario_id,
        "purpose": "Deterministic CUC unsafe fallback and optional non-compliant branch.",
        "test_level": "integration",
        "status": "probe",
        "derivation_ref": ["P12 deterministic branch scenario"],
        "initial_time": {"t": 0.0, "step": 0, "dt": 0.1},
        "initial_vehicles": [
            _p12_vehicle("MV_CUC", "on_ramp", 6850.0, -3.5, road_role="on_ramp_mv", merge_state="not_started"),
            _p12_vehicle("CFV_X", "lane_2", 6844.0, 0.0, vehicle_type=vehicle_type, compliance_state=compliance, initial_v=25.0, desired_speed=25.0),
            _p12_vehicle("CLV_Y", "lane_2", 6884.0, 0.0),
            _p12_vehicle("TLV", "lane_1", 6847.0, 3.5),
            _p12_vehicle("TFV", "lane_1", 6750.0, 3.5),
        ],
        "module_overrides": _p12_module_overrides(
            cuc_utility_overrides={"CFV_X": {"recommended_choice": "change_to_lane_1", "U1": 1.0, "U2": 0.0}}
        ),
        "preloaded_assignments": [
            _p12_assignment(
                mv_id="MV_CUC",
                clv_id="CLV_Y",
                cfv_id="CFV_X",
                aps_case="case_2",
                col_clv=False,
                col_cfv=True,
                desired_spacing_override=58.0,
            )
        ],
        "preloaded_state_machine_states": [],
        "preloaded_maneuver_trajectory_states": [],
        "expected_events": [],
        "forbidden_events": [],
        "expected_event_counts": [],
        "expected_sanity_checks": [],
        "expected_png_features": [
            {"feature_type": "target_lane_unsafe_fallback_marker", "required": not non_compliant, "vehicle_ids": ["CFV_X"], "expected_visibility": "visible"},
            {"feature_type": "non_compliant_ignored_marker", "required": non_compliant, "vehicle_ids": ["CFV_X"], "expected_visibility": "visible"},
        ],
        "tolerances": deepcopy(DEFAULT_TOLERANCES),
    }


def _p12_safe_cap_scenario() -> dict[str, Any]:
    return {
        "scenario_id": "P12-BRANCH-SAFE-CAP",
        "scenario_name": "P12 safe cap branch",
        "purpose": "Waiting, executing, and infeasible boundary-cap branches in one deterministic route.",
        "test_level": "integration",
        "status": "probe",
        "derivation_ref": ["P12 deterministic branch scenario"],
        "initial_time": {"t": 0.0, "step": 0, "dt": 0.1},
        "initial_vehicles": [
            _p12_vehicle("MV_SAFE_WAIT", "on_ramp", 7235.0, -3.5, road_role="on_ramp_mv", merge_state="waiting", initial_v=16.0, desired_speed=16.0),
            _p12_vehicle("CLV_SAFE_WAIT", "lane_2", 7245.0, 0.0, initial_v=16.0, desired_speed=16.0),
            _p12_vehicle("CFV_SAFE_WAIT", "lane_2", 7200.0, 0.0, initial_v=16.0, desired_speed=16.0),
            _p12_vehicle("MV_SAFE_EXEC", "on_ramp", 7235.0, -3.5, road_role="on_ramp_mv", merge_state="executing", initial_v=16.0, desired_speed=16.0),
            _p12_vehicle("CLV_SAFE_EXEC", "lane_2", 7270.0, 0.0, initial_v=16.0, desired_speed=16.0),
            _p12_vehicle("CFV_SAFE_EXEC", "lane_2", 7190.0, 0.0, initial_v=16.0, desired_speed=16.0),
            _p12_vehicle("MV_SAFE_RISK", "on_ramp", 7248.0, -3.5, road_role="on_ramp_mv", merge_state="waiting", initial_v=16.0, desired_speed=16.0),
            _p12_vehicle("CLV_SAFE_RISK", "lane_2", 7290.0, 0.0, initial_v=16.0, desired_speed=16.0),
            _p12_vehicle("CFV_SAFE_RISK", "lane_2", 7220.0, 0.0, initial_v=16.0, desired_speed=16.0),
        ],
        "module_overrides": _p12_module_overrides(),
        "preloaded_assignments": [
            _p12_assignment(mv_id="MV_SAFE_WAIT", clv_id="CLV_SAFE_WAIT", cfv_id="CFV_SAFE_WAIT", aps_case="case_1", col_clv=False, col_cfv=False, desired_spacing_override=None),
            _p12_assignment(mv_id="MV_SAFE_EXEC", clv_id="CLV_SAFE_EXEC", cfv_id="CFV_SAFE_EXEC", aps_case="case_1", col_clv=False, col_cfv=False, desired_spacing_override=None),
            _p12_assignment(mv_id="MV_SAFE_RISK", clv_id="CLV_SAFE_RISK", cfv_id="CFV_SAFE_RISK", aps_case="case_1", col_clv=False, col_cfv=False, desired_spacing_override=None),
        ],
        "preloaded_state_machine_states": [],
        "preloaded_maneuver_trajectory_states": [
            {
                "vehicle_id": "MV_SAFE_EXEC",
                "maneuver_type": "merge",
                "start_t": -0.8,
                "start_step": -8,
                "start_x_global": 7220.0,
                "start_y": -3.5,
                "target_lane": "lane_2",
                "target_y": 0.0,
                "planned_length": 100.0,
                "progress": 0.30,
                "assigned_clv_id": "CLV_SAFE_EXEC",
                "assigned_cfv_id": "CFV_SAFE_EXEC",
            }
        ],
        "expected_events": [],
        "forbidden_events": [],
        "expected_event_counts": [],
        "expected_sanity_checks": [],
        "expected_png_features": [
            {"feature_type": "boundary_cap_marker", "required": True, "vehicle_ids": ["MV_SAFE_WAIT", "MV_SAFE_EXEC"], "expected_visibility": "visible"},
            {"feature_type": "boundary_risk_marker", "required": True, "vehicle_ids": ["MV_SAFE_RISK"], "expected_visibility": "visible"},
            {"feature_type": "merge_trajectory_marker", "required": True, "vehicle_ids": ["MV_SAFE_EXEC"], "expected_visibility": "visible"},
        ],
        "tolerances": deepcopy(DEFAULT_TOLERANCES),
    }


def _p12_active_continuation_scenario() -> dict[str, Any]:
    return {
        "scenario_id": "P12-BRANCH-ACTIVE-CONTINUATION",
        "scenario_name": "P12 active continuation branch",
        "purpose": "Cache reuse, active lane-change, and active merge continue without rerunning start decisions.",
        "test_level": "integration",
        "status": "probe",
        "derivation_ref": ["P12 deterministic branch scenario"],
        "initial_time": {"t": 2.0, "step": 20, "dt": 0.1},
        "initial_vehicles": [
            _p12_vehicle("MV_CACHE", "on_ramp", 6840.0, -3.5, road_role="on_ramp_mv", merge_state="not_started"),
            _p12_vehicle("CLV_CACHE", "lane_2", 6890.0, 0.0),
            _p12_vehicle("CFV_CACHE", "lane_2", 6810.0, 0.0),
            _p12_vehicle("CFV_ACTIVE", "lane_2", 6860.0, 0.3, lane_change_state="executing"),
            _p12_vehicle("TLV_ACTIVE", "lane_1", 6900.0, 3.5),
            _p12_vehicle("TFV_ACTIVE", "lane_1", 6780.0, 3.5),
            _p12_vehicle("MV_ACTIVE", "on_ramp", 7050.0, -2.0, road_role="on_ramp_mv", merge_state="executing"),
            _p12_vehicle("CLV_ACTIVE_M", "lane_2", 7100.0, 0.0),
            _p12_vehicle("CFV_ACTIVE_M", "lane_2", 7000.0, 0.0),
        ],
        "module_overrides": _p12_module_overrides(),
        "preloaded_assignments": [
            _p12_assignment(mv_id="MV_CACHE", clv_id="CLV_CACHE", cfv_id="CFV_CACHE", aps_case="case_1", col_clv=False, col_cfv=False, desired_spacing_override=None, created_at_t=1.0, created_at_step=10),
            _p12_assignment(mv_id="MV_ACTIVE", clv_id="CLV_ACTIVE_M", cfv_id="CFV_ACTIVE_M", aps_case="case_1", col_clv=False, col_cfv=False, desired_spacing_override=None, created_at_t=1.0, created_at_step=10),
        ],
        "preloaded_state_machine_states": [],
        "preloaded_maneuver_trajectory_states": [
            {
                "vehicle_id": "CFV_ACTIVE",
                "maneuver_type": "lane_change",
                "start_t": 1.5,
                "start_step": 15,
                "start_x_global": 6850.0,
                "start_y": 0.0,
                "target_lane": "lane_1",
                "target_y": 3.5,
                "planned_length": 100.0,
                "progress": 0.20,
            },
            {
                "vehicle_id": "MV_ACTIVE",
                "maneuver_type": "merge",
                "start_t": 1.0,
                "start_step": 10,
                "start_x_global": 7035.0,
                "start_y": -3.5,
                "target_lane": "lane_2",
                "target_y": 0.0,
                "planned_length": 100.0,
                "progress": 0.20,
                "assigned_clv_id": "CLV_ACTIVE_M",
                "assigned_cfv_id": "CFV_ACTIVE_M",
            },
        ],
        "expected_events": [],
        "forbidden_events": [],
        "expected_event_counts": [],
        "expected_sanity_checks": [],
        "expected_png_features": [
            {"feature_type": "active_maneuver_marker", "required": True, "vehicle_ids": ["CFV_ACTIVE", "MV_ACTIVE"], "expected_visibility": "visible"},
            {"feature_type": "lane_change_trajectory_marker", "required": True, "vehicle_ids": ["CFV_ACTIVE"], "expected_visibility": "visible"},
            {"feature_type": "merge_trajectory_marker", "required": True, "vehicle_ids": ["MV_ACTIVE"], "expected_visibility": "visible"},
        ],
        "tolerances": deepcopy(DEFAULT_TOLERANCES),
    }


def _p13_required_deterministic_scenario(
    *,
    scenario_id: str,
    scenario_name: str,
    purpose: str,
    initial_time: dict[str, Any],
    vehicles: list[dict[str, Any]],
    expected_events: list[dict[str, Any]],
    expected_sanity_checks: list[dict[str, Any]],
    expected_png_features: list[dict[str, Any]],
    preloaded_assignments: list[dict[str, Any]] | None = None,
    preloaded_maneuver_trajectory_states: list[dict[str, Any]] | None = None,
    forbidden_events: list[dict[str, Any]] | None = None,
    expected_event_counts: list[dict[str, Any]] | None = None,
) -> dict[str, Any]:
    return {
        "scenario_id": scenario_id,
        "scenario_name": scenario_name,
        "purpose": purpose,
        "test_level": "integration",
        "status": "required",
        "derivation_ref": ["CORMC minimal validation scenario spec #P13"],
        "initial_time": initial_time,
        "initial_vehicles": vehicles,
        "module_overrides": _p12_module_overrides(
            cuc_utility_overrides=_p13_cuc_overrides_for(scenario_id)
        ),
        "preloaded_assignments": preloaded_assignments or [],
        "preloaded_state_machine_states": [],
        "preloaded_maneuver_trajectory_states": preloaded_maneuver_trajectory_states or [],
        "expected_events": expected_events,
        "forbidden_events": forbidden_events or [],
        "expected_event_counts": expected_event_counts or [],
        "expected_sanity_checks": expected_sanity_checks,
        "expected_png_features": expected_png_features,
        "tolerances": deepcopy(DEFAULT_TOLERANCES),
    }


def _p13_cuc_overrides_for(scenario_id: str) -> dict[str, dict[str, Any]]:
    return {}


def _p13_loop_sanity(vehicle_ids: list[str]) -> list[dict[str, Any]]:
    return [
        {
            "check_type": "multiple_commit_for_one_vehicle",
            "required": True,
            "expected_status": "pass",
            "vehicle_ids": vehicle_ids,
        },
        {
            "check_type": "no_write_before_commit",
            "required": True,
            "expected_status": "pass",
            "vehicle_ids": vehicle_ids,
        },
        {
            "check_type": "p12_command_namespace_conflict",
            "required": True,
            "expected_status": "pass",
            "vehicle_ids": vehicle_ids,
        },
        {
            "check_type": "information_integration_does_not_rewrite_state",
            "required": True,
            "expected_status": "pass",
            "vehicle_ids": vehicle_ids,
        },
        {
            "check_type": "time_advance_consistency",
            "required": True,
            "expected_status": "pass",
            "vehicle_ids": vehicle_ids,
        },
    ]


def _p13_commit_and_time_events(vehicle_ids: list[str]) -> list[dict[str, Any]]:
    return [
        {
            "event_type": "commit",
            "required": True,
            "vehicle_ids": vehicle_ids[:1],
            "match": {
                "each_active_vehicle_has_exactly_one_final_next_state": True,
                "commit_is_unique_state_writer": True,
            },
            "source": "first_version_engineering_patch",
        },
        {
            "event_type": "time_advance",
            "required": True,
            "vehicle_ids": vehicle_ids,
            "match": {"advanced_after_commit": True},
            "reason_code": "step11_after_commit_and_information_integration",
            "source": "first_version_engineering_patch",
        },
    ]


def _mvs_cuc_1a_override_choice1_scenario() -> dict[str, Any]:
    vehicle_ids = ["MV_CUC", "CFV_X", "CLV_Y", "TLV", "TFV"]
    return _p13_required_deterministic_scenario(
        scenario_id="MVS-CUC-1A_override_choice1",
        scenario_name="CUC choice 1 override starts target-lane lane change",
        purpose="Official CUC required route for test-harness choice 1 with safe target lane.",
        initial_time={"t": 0.0, "step": 0, "dt": 0.1},
        vehicles=[
            _p12_vehicle("MV_CUC", "on_ramp", 6850.0, -3.5, road_role="on_ramp_mv", merge_state="not_started"),
            _p12_vehicle("CFV_X", "lane_2", 6844.0, 0.0),
            _p12_vehicle("CLV_Y", "lane_2", 6884.0, 0.0),
            _p12_vehicle("TLV", "lane_1", 6920.0, 3.5, initial_v=22.0, desired_speed=22.0),
            _p12_vehicle("TFV", "lane_1", 6750.0, 3.5),
        ],
        preloaded_assignments=[
            _p12_assignment(
                mv_id="MV_CUC",
                clv_id="CLV_Y",
                cfv_id="CFV_X",
                aps_case="case_2",
                col_clv=False,
                col_cfv=True,
                desired_spacing_override=58.0,
            )
        ],
        expected_events=[
            {
                "event_type": "APS",
                "required": True,
                "vehicle_ids": ["MV_CUC"],
                "match": {
                    "trigger": "reuse_cache",
                    "effective_assignment_source": "cache_reused",
                    "new_assignment_created": False,
                },
                "reason_code": "reuse_cache",
                "source": "first_version_engineering_patch",
            },
            {
                "event_type": "cooperative_request",
                "required": True,
                "vehicle_ids": ["MV_CUC", "CFV_X"],
                "match": {
                    "aps_case": "case_2",
                    "cv_id": "CFV_X",
                    "source_mv_id": "MV_CUC",
                    "desired_spacing_override": 58.0,
                },
                "reason_code": "col_cfv_request",
                "source": "paper_formula",
            },
            {
                "event_type": "CUC",
                "required": True,
                "vehicle_ids": ["MV_CUC", "CFV_X", "TLV", "TFV"],
                "match": {
                    "target_lane_safe": True,
                    "target_lane": "lane_1",
                    "TT_min": 1.5,
                },
                "reason_code": "target_lane_safe",
                "source": "paper_formula",
            },
            {
                "event_type": "CUC",
                "required": True,
                "vehicle_ids": ["MV_CUC", "CFV_X"],
                "match": {
                    "recommended_choice": "change_to_lane_1",
                    "final_choice": "change_to_lane_1",
                    "utility_formula_status": "locked_formula",
                    "formula_status": "locked_formula",
                    "cuc_suggestion_executed": True,
                },
                "reason_code": "final_choice_change_to_lane_1",
                "source": "paper_formula",
            },
            {
                "event_type": "CUC",
                "required": True,
                "vehicle_ids": ["MV_CUC", "CFV_X"],
                "match": {"command_type": "lane_change", "target_lane": "lane_1"},
                "reason_code": "lane_change_command_created",
                "source": "paper_formula",
            },
            {
                "event_type": "lateral_trajectory",
                "required": True,
                "vehicle_ids": ["CFV_X"],
                "match": {
                    "maneuver_type": "lane_change",
                    "trajectory_consumed_speed_source": "p08_planning_speed",
                    "target_lane": "lane_1",
                },
                "reason_code": "lane_change_start",
                "source": "paper_formula",
            },
            *_p13_commit_and_time_events(vehicle_ids),
        ],
        expected_event_counts=[
            {
                "event_type": "spacing_override_consumption",
                "vehicle_ids": ["CFV_X"],
                "expected_count": 0,
                "comparison": "exactly",
            }
        ],
        expected_sanity_checks=[
            *_p13_loop_sanity(vehicle_ids),
            {
                "check_type": "state_machine_inconsistency",
                "required": True,
                "expected_status": "pass",
                "vehicle_ids": ["CFV_X"],
                "reason_code": "cuc_decision_not_persistent",
            },
        ],
        expected_png_features=[
            {"feature_type": "cuc_decision_marker", "required": True, "vehicle_ids": ["CFV_X"], "expected_visibility": "visible"},
            {"feature_type": "lane_change_intent_marker", "required": True, "vehicle_ids": ["CFV_X"], "expected_visibility": "visible"},
            {"feature_type": "lane_change_trajectory_marker", "required": True, "vehicle_ids": ["CFV_X"], "expected_visibility": "visible"},
            {"feature_type": "commit_marker", "required": True, "vehicle_ids": vehicle_ids, "expected_visibility": "visible"},
        ],
    )


def _mvs_cuc_2_scenario() -> dict[str, Any]:
    vehicle_ids = ["MV_CUC", "CFV_X", "CLV_Y", "TLV", "TFV"]
    return _p13_required_deterministic_scenario(
        scenario_id="MVS-CUC-2",
        scenario_name="CUC unsafe target lane falls back to Eq10 spacing",
        purpose="Official CUC required route for unsafe target lane fallback and spacing handoff.",
        initial_time={"t": 0.0, "step": 0, "dt": 0.1},
        vehicles=[
            _p12_vehicle("MV_CUC", "on_ramp", 6850.0, -3.5, road_role="on_ramp_mv", merge_state="not_started"),
            _p12_vehicle("CFV_X", "lane_2", 6844.0, 0.0, initial_v=25.0, desired_speed=25.0),
            _p12_vehicle("CLV_Y", "lane_2", 6850.0, 0.0),
            _p12_vehicle("TLV", "lane_1", 6920.0, 3.5, initial_v=22.0, desired_speed=22.0),
            _p12_vehicle("TFV", "lane_1", 6837.0, 3.5, initial_v=30.0, desired_speed=30.0),
        ],
        preloaded_assignments=[
            _p12_assignment(
                mv_id="MV_CUC",
                clv_id="CLV_Y",
                cfv_id="CFV_X",
                aps_case="case_2",
                col_clv=False,
                col_cfv=True,
                desired_spacing_override=58.0,
            )
        ],
        expected_events=[
            {
                "event_type": "cooperative_request",
                "required": True,
                "vehicle_ids": ["MV_CUC", "CFV_X"],
                "match": {"aps_case": "case_2", "desired_spacing_override": 58.0},
                "reason_code": "col_cfv_request",
                "source": "paper_formula",
            },
            {
                "event_type": "CUC",
                "required": True,
                "vehicle_ids": ["MV_CUC", "CFV_X", "TLV", "TFV"],
                "match": {"target_lane_safe": False, "target_lane": "lane_1"},
                "reason_code": "target_lane_unsafe",
                "source": "paper_formula",
            },
            {
                "event_type": "CUC",
                "required": True,
                "vehicle_ids": ["MV_CUC", "CFV_X"],
                "match": {
                    "recommended_choice": "change_to_lane_1",
                    "final_choice": "stay_lane_2",
                    "fallback_reason": "target_lane_unsafe",
                    "target_lane_safe": False,
                    "utility_formula_status": "locked_formula",
                    "formula_status": "locked_formula",
                },
                "reason_code": "fallback_target_lane_unsafe",
                "source": "paper_formula",
            },
            {
                "event_type": "CUC",
                "required": True,
                "vehicle_ids": ["MV_CUC", "CFV_X"],
                "match": {"command_type": "cooperation", "eq10_desired_spacing": 58.0},
                "reason_code": "spacing_override_handoff_to_p08",
                "source": "paper_formula",
            },
            {
                "event_type": "spacing_override_consumption",
                "required": True,
                "vehicle_ids": ["MV_CUC", "CFV_X"],
                "match": {"eq10_desired_spacing": 58.0},
                "reason_code": "eq10_spacing_override_consumed",
                "source": "paper_formula",
            },
            {
                "event_type": "longitudinal_model",
                "required": True,
                "vehicle_ids": ["CFV_X"],
                "match": {
                    "longitudinal_mode": "cav_gap_regulating",
                    "desired_spacing_override": 58.0,
                    "spacing_override_consumed": True,
                    "longitudinal_formula_status": "locked_formula",
                    "cpid_mode": "minimal_formula_mode",
                },
                "source": "paper_formula",
            },
            *_p13_commit_and_time_events(vehicle_ids),
        ],
        expected_event_counts=[
            {
                "event_type": "lateral_trajectory",
                "vehicle_ids": ["CFV_X"],
                "expected_count": 0,
                "comparison": "exactly",
            }
        ],
        expected_sanity_checks=[
            *_p13_loop_sanity(vehicle_ids),
            {
                "check_type": "state_machine_inconsistency",
                "required": True,
                "expected_status": "pass",
                "vehicle_ids": ["CFV_X"],
                "reason_code": "target_lane_unsafe_fallback",
            },
            {
                "check_type": "Eq10_applied_to_wrong_vehicle",
                "required": True,
                "expected_status": "pass",
                "vehicle_ids": vehicle_ids,
            },
        ],
        expected_png_features=[
            {"feature_type": "target_lane_unsafe_fallback_marker", "required": True, "vehicle_ids": ["CFV_X"], "expected_visibility": "visible"},
            {"feature_type": "spacing_override_marker", "required": True, "vehicle_ids": ["CFV_X"], "expected_visibility": "visible"},
            {"feature_type": "commit_marker", "required": True, "vehicle_ids": vehicle_ids, "expected_visibility": "visible"},
        ],
    )


def _mvs_cuc_3_scenario() -> dict[str, Any]:
    vehicle_ids = ["MV_CUC", "CFV_X", "CLV_Y", "TLV", "TFV"]
    return _p13_required_deterministic_scenario(
        scenario_id="MVS-CUC-3",
        scenario_name="CUC non-compliant CHV ignores suggestion",
        purpose="Official CUC required route for non-compliant CHV no-action branch.",
        initial_time={"t": 0.0, "step": 0, "dt": 0.1},
        vehicles=[
            _p12_vehicle("MV_CUC", "on_ramp", 6850.0, -3.5, road_role="on_ramp_mv", merge_state="not_started"),
            _p12_vehicle(
                "CFV_X",
                "lane_2",
                6844.0,
                0.0,
                vehicle_type="CHV",
                compliance_state="non_compliant",
                initial_v=25.0,
                desired_speed=25.0,
            ),
            _p12_vehicle("CLV_Y", "lane_2", 6884.0, 0.0),
            _p12_vehicle("TLV", "lane_1", 6853.0, 3.5),
            _p12_vehicle("TFV", "lane_1", 6780.0, 3.5),
        ],
        preloaded_assignments=[
            _p12_assignment(
                mv_id="MV_CUC",
                clv_id="CLV_Y",
                cfv_id="CFV_X",
                aps_case="case_2",
                col_clv=False,
                col_cfv=True,
                desired_spacing_override=58.0,
            )
        ],
        expected_events=[
            {
                "event_type": "cooperative_request",
                "required": True,
                "vehicle_ids": ["MV_CUC", "CFV_X"],
                "match": {"aps_case": "case_2", "desired_spacing_override": 58.0},
                "reason_code": "col_cfv_request",
                "source": "paper_formula",
            },
            {
                "event_type": "CUC",
                "required": True,
                "vehicle_ids": ["MV_CUC", "CFV_X"],
                "match": {
                    "vehicle_type": "chv",
                    "accepted_by_vehicle": False,
                    "cuc_suggestion_executed": False,
                },
                "reason_code": "non_compliant_chv_ignored",
                "source": "paper_formula",
            },
            {
                "event_type": "CUC",
                "required": True,
                "vehicle_ids": ["MV_CUC", "CFV_X"],
                "match": {
                    "final_choice": "not_applicable",
                    "fallback_reason": "non_compliant_chv",
                    "compliance_state": "non_compliant",
                    "cuc_suggestion_executed": False,
                },
                "reason_code": "non_compliant_chv",
                "source": "paper_formula",
            },
            {
                "event_type": "longitudinal_model",
                "required": True,
                "vehicle_ids": ["CFV_X"],
                "match": {
                    "longitudinal_mode": "chv_idm",
                    "spacing_override_consumed": False,
                    "compliance_state": "non_compliant",
                    "idm_formula_status": "locked_formula",
                },
                "source": "paper_formula",
            },
            *_p13_commit_and_time_events(vehicle_ids),
        ],
        expected_event_counts=[
            {
                "event_type": "spacing_override_consumption",
                "vehicle_ids": ["CFV_X"],
                "expected_count": 0,
                "comparison": "exactly",
            },
            {
                "event_type": "lateral_trajectory",
                "vehicle_ids": ["CFV_X"],
                "expected_count": 0,
                "comparison": "exactly",
            },
            {
                "event_type": "CUC",
                "vehicle_ids": ["CFV_X"],
                "reason_code": "lane_change_command_created",
                "expected_count": 0,
                "comparison": "exactly",
            },
        ],
        expected_sanity_checks=[
            *_p13_loop_sanity(vehicle_ids),
            {
                "check_type": "state_machine_inconsistency",
                "required": True,
                "expected_status": "pass",
                "vehicle_ids": ["CFV_X"],
                "reason_code": "non_compliant_no_action",
            },
            {
                "check_type": "Eq10_applied_to_wrong_vehicle",
                "required": True,
                "expected_status": "pass",
                "vehicle_ids": vehicle_ids,
            },
        ],
        expected_png_features=[
            {"feature_type": "non_compliant_ignored_marker", "required": True, "vehicle_ids": ["CFV_X"], "expected_visibility": "visible"},
            {"feature_type": "commit_marker", "required": True, "vehicle_ids": vehicle_ids, "expected_visibility": "visible"},
        ],
    )


def _mvs_safe_1a_waiting_cap_scenario() -> dict[str, Any]:
    vehicle_ids = ["MV_SAFE_WAIT", "CLV_SAFE_WAIT", "CFV_SAFE_WAIT"]
    return _p13_required_deterministic_scenario(
        scenario_id="MVS-SAFE-1A_waiting_cap",
        scenario_name="SAFE waiting boundary speed cap consumed by P08",
        purpose="Official SAFE required route for waiting-state boundary cap and no lateral trajectory.",
        initial_time={"t": 0.0, "step": 0, "dt": 0.1},
        vehicles=[
            _p12_vehicle("MV_SAFE_WAIT", "on_ramp", 7235.0, -3.5, road_role="on_ramp_mv", merge_state="waiting", initial_v=16.0, desired_speed=16.0),
            _p12_vehicle("CLV_SAFE_WAIT", "lane_2", 7245.0, 0.0, initial_v=16.0, desired_speed=16.0),
            _p12_vehicle("CFV_SAFE_WAIT", "lane_2", 7200.0, 0.0, initial_v=16.0, desired_speed=16.0),
        ],
        preloaded_assignments=[
            _p12_assignment(mv_id="MV_SAFE_WAIT", clv_id="CLV_SAFE_WAIT", cfv_id="CFV_SAFE_WAIT", aps_case="case_1", col_clv=False, col_cfv=False, desired_spacing_override=None)
        ],
        expected_events=[
            {
                "event_type": "CMC",
                "required": True,
                "vehicle_ids": ["MV_SAFE_WAIT"],
                "match": {
                    "cap_source": "boundary_collision_avoidance",
                    "cap_reason": "normal_cap",
                    "cap_feasible": True,
                    "cap_binding": True,
                },
                "numeric_expectations": {
                    "boundary_speed_cap": {"value": 2.6295, "tolerance": "derived_formula_abs"}
                },
                "reason_code": "boundary_speed_cap",
                "source": "paper_formula",
            },
            {
                "event_type": "CMC",
                "required": True,
                "vehicle_ids": ["MV_SAFE_WAIT"],
                "match": {"command_type": "longitudinal", "longitudinal_mode": "cmc_waiting"},
                "reason_code": "waiting_command",
                "source": "first_version_engineering_patch",
            },
            {
                "event_type": "speed_cap",
                "required": True,
                "vehicle_ids": ["MV_SAFE_WAIT"],
                "match": {"most_conservative_source": "boundary_speed_cap"},
                "numeric_expectations": {
                    "planning_speed": {"value": 2.6295, "tolerance": "derived_formula_abs"}
                },
                "reason_code": "speed_cap_consumed",
                "source": "paper_formula",
            },
            {
                "event_type": "longitudinal_model",
                "required": True,
                "vehicle_ids": ["MV_SAFE_WAIT"],
                "match": {
                    "longitudinal_mode": "mv_on_ramp",
                    "boundary_speed_cap": 2.6295029405356662,
                    "planning_speed": 2.6295029405356662,
                },
                "source": "first_version_engineering_patch",
            },
            *_p13_commit_and_time_events(vehicle_ids),
        ],
        expected_event_counts=[
            {
                "event_type": "lateral_trajectory",
                "vehicle_ids": ["MV_SAFE_WAIT"],
                "expected_count": 0,
                "comparison": "exactly",
            }
        ],
        expected_sanity_checks=[
            *_p13_loop_sanity(vehicle_ids),
            {
                "check_type": "boundary_violation",
                "required": True,
                "expected_status": "pass",
                "vehicle_ids": ["MV_SAFE_WAIT"],
                "reason_code": "normal_cap",
            },
        ],
        expected_png_features=[
            {"feature_type": "boundary_cap_marker", "required": True, "vehicle_ids": ["MV_SAFE_WAIT"], "expected_visibility": "visible"},
            {"feature_type": "speed_cap_consumption_marker", "required": True, "vehicle_ids": ["MV_SAFE_WAIT"], "expected_visibility": "visible"},
            {"feature_type": "planning_speed_marker", "required": True, "vehicle_ids": ["MV_SAFE_WAIT"], "expected_visibility": "visible"},
        ],
    )


def _mvs_safe_1b_executing_cap_lateral_consumption_scenario() -> dict[str, Any]:
    vehicle_ids = ["MV_SAFE_EXEC", "CLV_SAFE_EXEC", "CFV_SAFE_EXEC"]
    return _p13_required_deterministic_scenario(
        scenario_id="MVS-SAFE-1B_executing_cap_lateral_consumption",
        scenario_name="SAFE executing cap consumed by lateral trajectory",
        purpose="Official SAFE required route for executing merge speed cap and P09 progress consumption.",
        initial_time={"t": 0.0, "step": 0, "dt": 0.1},
        vehicles=[
            _p12_vehicle("MV_SAFE_EXEC", "on_ramp", 7235.0, -3.5, road_role="on_ramp_mv", merge_state="executing", initial_v=16.0, desired_speed=16.0),
            _p12_vehicle("CLV_SAFE_EXEC", "lane_2", 7270.0, 0.0, initial_v=16.0, desired_speed=16.0),
            _p12_vehicle("CFV_SAFE_EXEC", "lane_2", 7190.0, 0.0, initial_v=16.0, desired_speed=16.0),
        ],
        preloaded_assignments=[
            _p12_assignment(mv_id="MV_SAFE_EXEC", clv_id="CLV_SAFE_EXEC", cfv_id="CFV_SAFE_EXEC", aps_case="case_1", col_clv=False, col_cfv=False, desired_spacing_override=None)
        ],
        preloaded_maneuver_trajectory_states=[
            {
                "vehicle_id": "MV_SAFE_EXEC",
                "maneuver_type": "merge",
                "start_t": -0.8,
                "start_step": -8,
                "start_x_global": 7220.0,
                "start_y": -3.5,
                "target_lane": "lane_2",
                "target_y": 0.0,
                "planned_length": 100.0,
                "progress": 0.30,
                "assigned_clv_id": "CLV_SAFE_EXEC",
                "assigned_cfv_id": "CFV_SAFE_EXEC",
            }
        ],
        expected_events=[
            {
                "event_type": "CMC",
                "required": True,
                "vehicle_ids": ["MV_SAFE_EXEC"],
                "match": {"branch": "cmc_executing_continuation", "no_new_eq53_start_decision": True},
                "reason_code": "cmc_executing_continuation",
                "source": "first_version_engineering_patch",
            },
            {
                "event_type": "speed_cap",
                "required": True,
                "vehicle_ids": ["MV_SAFE_EXEC"],
                "match": {"most_conservative_source": "boundary_speed_cap"},
                "numeric_expectations": {
                    "planning_speed": {"value": 2.6295, "tolerance": "derived_formula_abs"}
                },
                "reason_code": "speed_cap_consumed",
                "source": "paper_formula",
            },
            {
                "event_type": "lateral_trajectory",
                "required": True,
                "vehicle_ids": ["MV_SAFE_EXEC"],
                "match": {
                    "maneuver_type": "merge",
                    "trajectory_consumed_speed_source": "p08_planning_speed",
                    "previous_progress": 0.30,
                    "does_not_rejudge_merge_start": True,
                },
                "numeric_expectations": {
                    "progress": {"value": 0.30263, "tolerance": "derived_formula_abs"}
                },
                "reason_code": "merge_continuation",
                "source": "paper_formula",
            },
            *_p13_commit_and_time_events(vehicle_ids),
        ],
        expected_sanity_checks=[
            *_p13_loop_sanity(vehicle_ids),
            {
                "check_type": "boundary_violation",
                "required": True,
                "expected_status": "pass",
                "vehicle_ids": ["MV_SAFE_EXEC"],
                "reason_code": "normal_cap",
            },
        ],
        expected_png_features=[
            {"feature_type": "boundary_cap_marker", "required": True, "vehicle_ids": ["MV_SAFE_EXEC"], "expected_visibility": "visible"},
            {"feature_type": "merge_trajectory_marker", "required": True, "vehicle_ids": ["MV_SAFE_EXEC"], "expected_visibility": "visible"},
            {"feature_type": "planning_speed_consumption_marker", "required": True, "vehicle_ids": ["MV_SAFE_EXEC"], "expected_visibility": "visible"},
        ],
    )


def _mvs_safe_2_scenario() -> dict[str, Any]:
    vehicle_ids = ["MV_SAFE_FAIL", "CLV_SAFE_FAIL", "CFV_SAFE_FAIL"]
    return _p13_required_deterministic_scenario(
        scenario_id="MVS-SAFE-2",
        scenario_name="SAFE infeasible boundary cap exposes risk marker",
        purpose="Official SAFE required route for cap-infeasible boundary risk evidence.",
        initial_time={"t": 0.0, "step": 0, "dt": 0.1},
        vehicles=[
            _p12_vehicle("MV_SAFE_FAIL", "on_ramp", 7248.0, -3.5, road_role="on_ramp_mv", merge_state="waiting", initial_v=16.0, desired_speed=16.0),
            _p12_vehicle("CLV_SAFE_FAIL", "lane_2", 7290.0, 0.0, initial_v=16.0, desired_speed=16.0),
            _p12_vehicle("CFV_SAFE_FAIL", "lane_2", 7220.0, 0.0, initial_v=16.0, desired_speed=16.0),
        ],
        preloaded_assignments=[
            _p12_assignment(mv_id="MV_SAFE_FAIL", clv_id="CLV_SAFE_FAIL", cfv_id="CFV_SAFE_FAIL", aps_case="case_1", col_clv=False, col_cfv=False, desired_spacing_override=None)
        ],
        expected_events=[
            {
                "event_type": "CMC",
                "required": True,
                "vehicle_ids": ["MV_SAFE_FAIL"],
                "match": {
                    "cap_source": "boundary_collision_avoidance",
                    "cap_reason": "cap_negative",
                    "cap_feasible": False,
                    "cap_binding": True,
                },
                "numeric_expectations": {
                    "boundary_speed_cap": {"value": -0.4781, "tolerance": "derived_formula_abs"}
                },
                "reason_code": "boundary_speed_cap",
                "source": "paper_formula",
            },
            {
                "event_type": "lateral_trajectory",
                "required": True,
                "vehicle_ids": ["MV_SAFE_FAIL"],
                "match": {
                    "maneuver_type": "merge",
                    "boundary_risk_status": "cap_infeasible",
                    "trajectory_consumed_speed_source": "p08_planning_speed",
                },
                "reason_code": "merge_start",
                "source": "paper_formula",
            },
            *_p13_commit_and_time_events(vehicle_ids),
        ],
        expected_sanity_checks=[
            *_p13_loop_sanity(vehicle_ids),
            {
                "check_type": "boundary_violation",
                "required": True,
                "expected_status": "warning",
                "vehicle_ids": ["MV_SAFE_FAIL"],
                "reason_code": "cap_negative",
            },
            {
                "check_type": "boundary_violation",
                "required": True,
                "expected_status": "warning",
                "vehicle_ids": ["MV_SAFE_FAIL"],
                "reason_code": "p09_boundary_risk_observable",
            },
        ],
        expected_png_features=[
            {"feature_type": "boundary_risk_marker", "required": True, "vehicle_ids": ["MV_SAFE_FAIL"], "expected_visibility": "visible"},
            {"feature_type": "merge_trajectory_marker", "required": True, "vehicle_ids": ["MV_SAFE_FAIL"], "expected_visibility": "visible"},
        ],
    )


def _mvs_commit_1_full_scenario() -> dict[str, Any]:
    vehicle_ids = [
        "MV_CACHE",
        "CLV_CACHE",
        "CFV_CACHE",
        "CV_ACTIVE_LC",
        "TLV_ACTIVE",
        "TFV_ACTIVE",
        "MV_ACTIVE_MERGE",
        "CLV_MERGE",
        "CFV_MERGE",
    ]
    return _p13_required_deterministic_scenario(
        scenario_id="MVS-COMMIT-1-full",
        scenario_name="Full deterministic commit route with cache and active maneuvers",
        purpose="Official commit required route for cache reuse, active lane change, active merge, commit, and Step11.",
        initial_time={"t": 2.0, "step": 20, "dt": 0.1},
        vehicles=[
            _p12_vehicle("MV_CACHE", "on_ramp", 6890.0, -3.5, road_role="on_ramp_mv", merge_state="not_started"),
            _p12_vehicle("CLV_CACHE", "lane_2", 6940.0, 0.0),
            _p12_vehicle("CFV_CACHE", "lane_2", 6840.0, 0.0),
            _p12_vehicle("CV_ACTIVE_LC", "lane_2", 6900.0, 1.2, lane_change_state="executing"),
            _p12_vehicle("TLV_ACTIVE", "lane_1", 6960.0, 3.5),
            _p12_vehicle("TFV_ACTIVE", "lane_1", 6820.0, 3.5),
            _p12_vehicle("MV_ACTIVE_MERGE", "on_ramp", 7010.0, -2.1, road_role="on_ramp_mv", merge_state="executing", initial_v=18.0, desired_speed=18.0),
            _p12_vehicle("CLV_MERGE", "lane_2", 7060.0, 0.0, initial_v=18.0, desired_speed=18.0),
            _p12_vehicle("CFV_MERGE", "lane_2", 6970.0, 0.0, initial_v=18.0, desired_speed=18.0),
        ],
        preloaded_assignments=[
            _p12_assignment(
                mv_id="MV_CACHE",
                clv_id="CLV_CACHE",
                cfv_id="CFV_CACHE",
                aps_case="case_1",
                col_clv=False,
                col_cfv=False,
                desired_spacing_override=None,
                created_at_t=0.0,
                created_at_step=0,
            )
        ],
        preloaded_maneuver_trajectory_states=[
            {
                "vehicle_id": "CV_ACTIVE_LC",
                "maneuver_type": "lane_change",
                "start_t": 1.0,
                "start_step": 10,
                "start_x_global": 6880.0,
                "start_y": 0.0,
                "target_lane": "lane_1",
                "target_y": 3.5,
                "planned_length": 100.0,
                "progress": 0.35,
            },
            {
                "vehicle_id": "MV_ACTIVE_MERGE",
                "maneuver_type": "merge",
                "start_t": 1.2,
                "start_step": 12,
                "start_x_global": 6990.0,
                "start_y": -3.5,
                "target_lane": "lane_2",
                "target_y": 0.0,
                "planned_length": 100.0,
                "progress": 0.40,
                "assigned_clv_id": "CLV_MERGE",
                "assigned_cfv_id": "CFV_MERGE",
            },
        ],
        expected_events=[
            {
                "event_type": "APS",
                "required": True,
                "vehicle_ids": ["MV_CACHE"],
                "match": {
                    "trigger": "reuse_cache",
                    "effective_assignment_source": "cache_reused",
                    "aps_executed": False,
                },
                "reason_code": "reuse_cache",
                "source": "first_version_engineering_patch",
            },
            {
                "event_type": "CMC",
                "required": True,
                "vehicle_ids": ["MV_ACTIVE_MERGE"],
                "match": {"branch": "cmc_executing_continuation", "no_new_eq53_start_decision": True},
                "reason_code": "cmc_executing_continuation",
                "source": "first_version_engineering_patch",
            },
            {
                "event_type": "lateral_trajectory",
                "required": True,
                "vehicle_ids": ["CV_ACTIVE_LC"],
                "match": {
                    "maneuver_type": "lane_change",
                    "previous_progress": 0.35,
                    "trajectory_consumed_speed_source": "p08_planning_speed",
                    "cuc_rerun_by_p09": False,
                },
                "reason_code": "lane_change_continuation",
                "source": "paper_formula",
            },
            {
                "event_type": "lateral_trajectory",
                "required": True,
                "vehicle_ids": ["MV_ACTIVE_MERGE"],
                "match": {
                    "maneuver_type": "merge",
                    "previous_progress": 0.40,
                    "trajectory_consumed_speed_source": "p08_planning_speed",
                    "does_not_rejudge_merge_start": True,
                },
                "reason_code": "merge_continuation",
                "source": "paper_formula",
            },
            *_p13_commit_and_time_events(vehicle_ids),
        ],
        expected_event_counts=[
            {
                "event_type": "CUC",
                "vehicle_ids": ["CV_ACTIVE_LC"],
                "reason_code": "final_choice_change_to_lane_1",
                "expected_count": 0,
                "comparison": "exactly",
            },
            {
                "event_type": "CMC",
                "vehicle_ids": ["MV_ACTIVE_MERGE"],
                "reason_code": "eq53_gap",
                "expected_count": 0,
                "comparison": "exactly",
            },
            {
                "event_type": "APS",
                "vehicle_ids": ["MV_CACHE"],
                "match": {"trigger": "first_APS"},
                "expected_count": 0,
                "comparison": "exactly",
            },
        ],
        expected_sanity_checks=[
            *_p13_loop_sanity(vehicle_ids),
            {
                "check_type": "time_advance_consistency",
                "required": True,
                "expected_status": "pass",
                "vehicle_ids": vehicle_ids,
                "time_window": {"step": 20},
            },
        ],
        expected_png_features=[
            {"feature_type": "cache_reuse_marker", "required": True, "vehicle_ids": ["MV_CACHE"], "expected_visibility": "visible"},
            {"feature_type": "lane_change_trajectory_marker", "required": True, "vehicle_ids": ["CV_ACTIVE_LC"], "expected_visibility": "visible"},
            {"feature_type": "merge_trajectory_marker", "required": True, "vehicle_ids": ["MV_ACTIVE_MERGE"], "expected_visibility": "visible"},
            {"feature_type": "commit_marker", "required": True, "vehicle_ids": vehicle_ids, "expected_visibility": "visible"},
        ],
    )


BUILTIN_SCENARIOS.update(
    {
        "MVS-E2E-1": _p12_e2e_1_scenario(),
        "P12-DEMO-E2E": _p12_demo_e2e_scenario(),
        "P12-BRANCH-CUC-LANECHANGE": _p12_cuc_lanechange_scenario(),
        "P12-BRANCH-CUC-FALLBACK": _p12_cuc_fallback_scenario(),
        "P12-BRANCH-CUC-NONCOMPLIANT": _p12_cuc_fallback_scenario(non_compliant=True),
        "P12-BRANCH-SAFE-CAP": _p12_safe_cap_scenario(),
        "P12-BRANCH-ACTIVE-CONTINUATION": _p12_active_continuation_scenario(),
    }
)

BUILTIN_SCENARIOS.update(
    {
        "MVS-CUC-1A_override_choice1": _mvs_cuc_1a_override_choice1_scenario(),
        "MVS-CUC-2": _mvs_cuc_2_scenario(),
        "MVS-CUC-3": _mvs_cuc_3_scenario(),
        "MVS-SAFE-1A_waiting_cap": _mvs_safe_1a_waiting_cap_scenario(),
        "MVS-SAFE-1B_executing_cap_lateral_consumption": (
            _mvs_safe_1b_executing_cap_lateral_consumption_scenario()
        ),
        "MVS-SAFE-2": _mvs_safe_2_scenario(),
        "MVS-COMMIT-1-full": _mvs_commit_1_full_scenario(),
    }
)
