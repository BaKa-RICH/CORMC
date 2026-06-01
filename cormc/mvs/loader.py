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
