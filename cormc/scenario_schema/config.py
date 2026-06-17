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
        "traffic_flow_source",
        "flow_stop_condition",
        "flow_validation",
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
