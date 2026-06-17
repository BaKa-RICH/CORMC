from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any, Mapping

if TYPE_CHECKING:
    from cormc.traffic_flow.source import BoundaryFlowSource


@dataclass(frozen=True)
class BoundarySourceSpec:
    source_type: str = "reserved"
    description: str = "reserved for boundary generation v1"


@dataclass(frozen=True)
class RandomBoundarySpec:
    profile_id: str = "reserved"
    seed: int | None = None
    enabled: bool = False


@dataclass(frozen=True)
class SceneVehicle:
    vehicle_id: str
    role: str
    lane_id: str
    x: float
    speed: float
    state: str = "none"
    y: float | None = None
    vehicle_type: str = "CAV"
    compliance_state: str = "not_applicable"
    acceleration: float = 0.0
    lane_change_state: str = "normal"
    spec_overrides: Mapping[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class StaticSceneSpec:
    scenario_id: str
    scenario_name: str
    purpose: str
    vehicles: tuple[SceneVehicle, ...]
    description: str | None = None
    test_level: str = "integration"
    status: str = "probe"
    derivation_ref: tuple[str, ...] = ()
    road_config_ref: str | None = "paper_fig10_first_version"
    parameter_config_ref: str | None = "paper_table_i_first_version"
    control_policy_config_ref: str | None = None
    vehicle_generation_config_ref: str | None = None
    output_config_ref: str | None = None
    initial_time: Mapping[str, Any] = field(
        default_factory=lambda: {"t": 0.0, "step": 0, "dt": 0.1}
    )
    module_overrides: Mapping[str, Any] = field(default_factory=dict)
    preloaded_assignments: tuple[Mapping[str, Any], ...] = ()
    preloaded_state_machine_states: tuple[Mapping[str, Any], ...] = ()
    preloaded_maneuver_trajectory_states: tuple[Mapping[str, Any], ...] = ()
    expected_events: tuple[Mapping[str, Any], ...] = ()
    forbidden_events: tuple[Mapping[str, Any], ...] = ()
    expected_event_counts: tuple[Mapping[str, Any], ...] = ()
    expected_sanity_checks: tuple[Mapping[str, Any], ...] = ()
    expected_png_features: tuple[Mapping[str, Any], ...] = ()
    tolerances: Mapping[str, float] = field(default_factory=dict)
    notes: tuple[str, ...] = ()
    boundary_source: BoundarySourceSpec | None = None
    random_boundary: RandomBoundarySpec | None = None


@dataclass(frozen=True)
class FlowStopConditionSpec:
    max_steps: int
    horizon_s: float
    mode: str = "max_steps"


@dataclass(frozen=True)
class FlowValidationSpec:
    min_generated_lane2_count: int
    min_generated_on_ramp_mv_count: int
    min_completed_mv_count: int
    allow_open_mvs_at_horizon: bool = True


@dataclass(frozen=True)
class TrafficFlowSceneSpec:
    scenario_id: str
    scenario_name: str
    purpose: str
    initial_vehicles: tuple[SceneVehicle, ...]
    boundary_flow_source: BoundaryFlowSource
    safe_spawn_gap_m: float
    stop_condition: FlowStopConditionSpec
    validation: FlowValidationSpec
    module_overrides: Mapping[str, Any] = field(default_factory=dict)
    notes: tuple[str, ...] = ()
    test_level: str = "integration"
    status: str = "required"
    derivation_ref: tuple[str, ...] = ()
    initial_time: Mapping[str, Any] = field(
        default_factory=lambda: {"t": 0.0, "step": 0, "dt": 0.1}
    )


def mv(vehicle_id: str, x: float, *, speed: float = 20.0, note_key: str | None = None, note: str | None = None) -> SceneVehicle:
    return vehicle(
        vehicle_id,
        lane_id="on_ramp",
        role="on_ramp_mv",
        x=x,
        y=-3.5,
        speed=speed,
        state="not_started",
        note_key=note_key,
        note=note,
    )


def lane2(vehicle_id: str, x: float, *, speed: float = 20.0, note_key: str | None = None, note: str | None = None) -> SceneVehicle:
    return vehicle(
        vehicle_id,
        lane_id="lane_2",
        role="mainline",
        x=x,
        y=0.0,
        speed=speed,
        state="none",
        note_key=note_key,
        note=note,
    )


def lane1(vehicle_id: str, x: float, *, speed: float = 15.0, note_key: str | None = None, note: str | None = None) -> SceneVehicle:
    return vehicle(
        vehicle_id,
        lane_id="lane_1",
        role="mainline",
        x=x,
        y=3.5,
        speed=speed,
        state="none",
        note_key=note_key,
        note=note,
    )


def vehicle(
    vehicle_id: str,
    *,
    lane_id: str,
    role: str,
    x: float,
    y: float,
    speed: float,
    state: str,
    vehicle_type: str = "CAV",
    compliance_state: str = "not_applicable",
    acceleration: float = 0.0,
    lane_change_state: str = "normal",
    note_key: str | None = None,
    note: str | None = None,
    spec_overrides: Mapping[str, Any] | None = None,
) -> SceneVehicle:
    overrides = dict(spec_overrides or {})
    if note_key is not None and note is not None:
        overrides[note_key] = note
    return SceneVehicle(
        vehicle_id=vehicle_id,
        role=role,
        lane_id=lane_id,
        x=float(x),
        y=float(y),
        speed=float(speed),
        state=state,
        vehicle_type=vehicle_type,
        compliance_state=compliance_state,
        acceleration=float(acceleration),
        lane_change_state=lane_change_state,
        spec_overrides=overrides,
    )
