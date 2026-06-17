from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any


@dataclass(frozen=True)
class ControlledTrajectoryCommand:
    vehicle_id: str
    step: int
    t: float
    target_t: float
    x_global: float
    y: float
    v: float
    a: float
    yaw: float | None = None
    physical_lane: str = "lane_2"
    road_role: str = "mainline"
    authority_mode: str = "trajectory_authority"
    authority_reason: str = ""
    source_candidate_id: str | None = None
    source_command_id: str | None = None
    source_maneuver_type: str | None = None
    assigned_clv_id: str | None = None
    assigned_cfv_id: str | None = None

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class SpawnCommand:
    vehicle_id: str
    step: int
    t: float
    route_id: str
    edge_id: str
    lane_index: int
    depart_pos: float
    x_global: float
    y: float
    v: float
    vehicle_type: str
    compliance_state: str
    source_queue_seed: int
    source_profile_id: str
    source_spawn_reason: str

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class RealizationRecord:
    vehicle_id: str
    step: int
    t: float
    command_x_global: float
    command_y: float
    command_v: float
    command_a: float
    realized_x_global: float | None
    realized_y: float | None
    realized_v: float | None
    realized_edge_id: str | None
    realized_lane_id: str | None
    realized_lane_position: float | None
    dx_abs: float | None
    dy_abs: float | None
    dv_abs: float | None
    result: str

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class SumoTrajectoryAuthorityConfig:
    step_length: float = 0.1
    lateral_resolution: float = 0.25
    collision_action: str = "warn"
    post_merge_hold_steps: int = 10
    executor_mode: str = "trajectory_authority"
    mismatch_x_tolerance_m: float = 0.75
    mismatch_y_tolerance_m: float = 0.35
    mismatch_v_tolerance_mps: float = 1.0

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class SumoSimulationResult:
    run_id: str
    status: str
    sumo_version: str
    net_file: str
    route_file: str
    sumocfg_file: str
    steps: int
    realization_records: tuple[RealizationRecord, ...] = ()
    collision_events: tuple[dict[str, Any], ...] = ()
    teleport_events: tuple[dict[str, Any], ...] = ()

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class SumoArtifactResult:
    run_id: str
    output_dir: str
    network_files: dict[str, str]
    simulation_result: SumoSimulationResult | None = None
    manifest_path: str | None = None
    report_path: str | None = None
    extra_paths: dict[str, str] = field(default_factory=dict)
    scenario_id: str | None = None
    status: str | None = None
    sumo_config_path: str | None = None
    artifact_manifest_path: str | None = None
    run_report_path: str | None = None
    realization_path: str | None = None
    trajectory_path: str | None = None
    events_path: str | None = None
    sanity_path: str | None = None
    generated_count: int = 0
    blocked_spawn_count: int = 0
    collision_count: int = 0
    teleport_count: int = 0
    realization_mismatch_count: int = 0
    active_controlled_vehicle_ids: tuple[str, ...] = ()

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)
