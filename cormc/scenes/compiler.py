from __future__ import annotations

from copy import deepcopy
from typing import Any, Mapping

from cormc.scenario_schema.config import load_scenario_config
from cormc.scenes.model import SceneVehicle, StaticSceneSpec, TrafficFlowSceneSpec


DEFAULT_STATIC_MODULE_OVERRIDES: dict[str, Any] = {
    "boundary_generation_enabled": False,
    "random_arrival_enabled": False,
    "random_vehicle_attributes_enabled": False,
    "ordinary_mainline_lane_change_enabled": False,
    "platoon_cmc_enabled": False,
    "mpc_lateral_tracking_enabled": False,
}


def compile_static_scene(spec: StaticSceneSpec) -> dict[str, Any]:
    config: dict[str, Any] = {
        "scenario_id": spec.scenario_id,
        "scenario_name": spec.scenario_name,
        "purpose": spec.purpose,
        "test_level": spec.test_level,
        "status": spec.status,
        "initial_time": dict(spec.initial_time),
        "initial_vehicles": [_compile_vehicle(vehicle) for vehicle in spec.vehicles],
        "module_overrides": _module_overrides(spec.module_overrides),
        "preloaded_assignments": [deepcopy(dict(item)) for item in spec.preloaded_assignments],
        "preloaded_state_machine_states": [
            deepcopy(dict(item)) for item in spec.preloaded_state_machine_states
        ],
        "preloaded_maneuver_trajectory_states": [
            deepcopy(dict(item)) for item in spec.preloaded_maneuver_trajectory_states
        ],
        "expected_events": [deepcopy(dict(item)) for item in spec.expected_events],
        "forbidden_events": [deepcopy(dict(item)) for item in spec.forbidden_events],
        "expected_event_counts": [
            deepcopy(dict(item)) for item in spec.expected_event_counts
        ],
        "expected_sanity_checks": [
            deepcopy(dict(item)) for item in spec.expected_sanity_checks
        ],
        "expected_png_features": [
            deepcopy(dict(item)) for item in spec.expected_png_features
        ],
        "notes": list(spec.notes),
    }
    _set_optional(config, "description", spec.description)
    _set_optional(config, "derivation_ref", list(spec.derivation_ref) or None)
    _set_optional(config, "road_config_ref", spec.road_config_ref)
    _set_optional(config, "parameter_config_ref", spec.parameter_config_ref)
    _set_optional(config, "control_policy_config_ref", spec.control_policy_config_ref)
    _set_optional(config, "vehicle_generation_config_ref", spec.vehicle_generation_config_ref)
    _set_optional(config, "output_config_ref", spec.output_config_ref)
    if spec.tolerances:
        config["tolerances"] = dict(spec.tolerances)
    return load_scenario_config(config)


def compile_traffic_flow_scene(spec: TrafficFlowSceneSpec) -> dict[str, Any]:
    source_summary = spec.boundary_flow_source.to_summary()
    config: dict[str, Any] = {
        "scenario_id": spec.scenario_id,
        "scenario_name": spec.scenario_name,
        "purpose": spec.purpose,
        "test_level": spec.test_level,
        "status": spec.status,
        "initial_time": dict(spec.initial_time),
        "initial_vehicles": [
            _compile_vehicle(vehicle) for vehicle in spec.initial_vehicles
        ],
        "module_overrides": _traffic_flow_module_overrides(spec.module_overrides),
        "preloaded_assignments": [],
        "preloaded_state_machine_states": [],
        "preloaded_maneuver_trajectory_states": [],
        "expected_events": [],
        "forbidden_events": [],
        "expected_event_counts": [],
        "expected_sanity_checks": [],
        "expected_png_features": [],
        "derivation_ref": list(spec.derivation_ref),
        "notes": list(spec.notes),
        "traffic_flow_source": source_summary,
        "flow_stop_condition": {
            "mode": spec.stop_condition.mode,
            "max_steps": int(spec.stop_condition.max_steps),
            "horizon_s": float(spec.stop_condition.horizon_s),
        },
        "flow_validation": {
            "min_generated_lane2_count": int(spec.validation.min_generated_lane2_count),
            "min_generated_on_ramp_mv_count": int(
                spec.validation.min_generated_on_ramp_mv_count
            ),
            "min_completed_mv_count": int(spec.validation.min_completed_mv_count),
            "allow_open_mvs_at_horizon": bool(spec.validation.allow_open_mvs_at_horizon),
        },
    }
    return load_scenario_config(config)


def _compile_vehicle(vehicle: SceneVehicle) -> dict[str, Any]:
    y = vehicle.y
    if y is None:
        y = {"lane_1": 3.5, "lane_2": 0.0, "on_ramp": -3.5}[vehicle.lane_id]
    return {
        "vehicle_id": vehicle.vehicle_id,
        "vehicle_type": vehicle.vehicle_type,
        "compliance_state": vehicle.compliance_state,
        "initial_x_global": float(vehicle.x),
        "initial_y": float(y),
        "initial_v": float(vehicle.speed),
        "initial_a": float(vehicle.acceleration),
        "physical_lane": vehicle.lane_id,
        "road_role": vehicle.role,
        "lane_change_state": vehicle.lane_change_state,
        "merge_state": vehicle.state,
        "spec_overrides": deepcopy(dict(vehicle.spec_overrides)),
    }


def _module_overrides(overrides: Mapping[str, Any]) -> dict[str, Any]:
    return {**DEFAULT_STATIC_MODULE_OVERRIDES, **deepcopy(dict(overrides))}


def _traffic_flow_module_overrides(overrides: Mapping[str, Any]) -> dict[str, Any]:
    return {
        **DEFAULT_STATIC_MODULE_OVERRIDES,
        "boundary_generation_enabled": True,
        "random_arrival_enabled": True,
        "random_vehicle_attributes_enabled": True,
        "ordinary_mainline_lane_change_enabled": False,
        "platoon_cmc_enabled": False,
        "mpc_lateral_tracking_enabled": False,
        **deepcopy(dict(overrides)),
    }


def _set_optional(config: dict[str, Any], key: str, value: Any) -> None:
    if value is not None:
        config[key] = value
