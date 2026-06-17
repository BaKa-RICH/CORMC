from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass, field, fields, is_dataclass
from types import MappingProxyType
from typing import Any, Mapping

from cormc.simulation_core.assignment_lifecycle import assignment_lifecycle_manager
from cormc.scenario_schema.config import load_scenario_config
from cormc.scenes import load_scene_config


LANE_1 = "lane_1"
LANE_2 = "lane_2"
ON_RAMP = "on_ramp"
MAINLINE = "mainline"
ON_RAMP_ROLE = "on_ramp"
ON_RAMP_MV_ROLE = "on_ramp_mv"


@dataclass(frozen=True)
class RoadGeometryConfig:
    config_id: str = "paper_fig10_first_version"
    mainline_start_global: float = 0.0
    mainline_end_global: float = 10000.0
    warmup_length: float = 4000.0
    control_start_global: float = 6650.0
    x0_m_global: float = 6950.0
    l_merging: float = 300.0
    x_ramp_end_global: float = 7250.0
    l_coop_fixed: float = 300.0
    communication_range: float = 300.0
    lane_width: float = 3.5
    lane_centerlines: Mapping[str, float] = MappingProxyType(
        {
            LANE_1: 3.5,
            LANE_2: 0.0,
            ON_RAMP: -3.5,
        }
    )


DEFAULT_ROAD_GEOMETRY = RoadGeometryConfig()


@dataclass(frozen=True)
class VehicleSpec:
    vehicle_id: str
    vehicle_type: str
    compliance_state: str
    desired_speed: float | None = None
    desired_time_gap: float | None = None
    desired_time_gap_class: str | None = None
    assigned_arrival_headway: float | None = None
    inertial_lag: float | None = None
    length: float = 4.0
    source_lane_at_generation: str = LANE_2
    generation_step: int = 0
    generation_t: float = 0.0


@dataclass(frozen=True)
class VehicleState:
    vehicle_id: str
    x_global: float
    y: float
    v: float
    a: float
    physical_lane: str
    road_role: str
    lane_change_state: str = "normal"
    merge_state: str = "none"
    is_active: bool = True


@dataclass(frozen=True)
class ManeuverTrajectoryState:
    vehicle_id: str
    maneuver_type: str
    start_step: int
    start_t: float
    start_x_global: float
    start_y: float
    target_lane: str
    target_y: float
    source_command_id: str | None = None
    source_event_id: str | None = None
    planned_length: float | None = None
    progress: float = 0.0
    last_planning_speed: float | None = None
    assigned_clv_id: str | None = None
    assigned_cfv_id: str | None = None


@dataclass(frozen=True)
class LongitudinalControllerMemory:
    vehicle_id: str
    ex_prev: float | None = None
    e_prev: float | None = None
    integral_ex: float = 0.0
    integral_e: float = 0.0
    last_t: float | None = None
    last_controller_update_step: int | None = None
    controller_mode: str = "cav_cpid"


@dataclass(frozen=True)
class Lane2GapBoundaryEligibility:
    vehicle_id: str | None
    eligible: bool
    reason: str
    required_lane: str = LANE_2
    physical_lane: str | None = None
    lane_change_state: str | None = None
    is_active: bool | None = None


@dataclass
class PreFreezeWorkspace:
    t: float
    step: int
    dt: float
    active_vehicle_ids: list[str]
    vehicle_states: dict[str, VehicleState]
    vehicle_specs: dict[str, VehicleSpec]
    assignment_records_by_mv: dict[str, dict[str, Any]]
    active_maneuvers: dict[str, ManeuverTrajectoryState]
    command_buffer: dict[str, Any]
    next_state_buffer: dict[str, Any]
    controller_memory_by_vehicle: dict[str, LongitudinalControllerMemory] = field(default_factory=dict)
    ramp_merge_runtime: Any | None = None
    road_config_ref: str = DEFAULT_ROAD_GEOMETRY.config_id
    parameter_config_ref: str = "paper_table_i_first_version"
    scenario_config_ref: str | None = None
    output_config_ref: str | None = None


@dataclass(frozen=True)
class SimulationState:
    t: float
    step: int
    dt: float
    active_vehicle_ids: tuple[str, ...]
    vehicle_states: Mapping[str, VehicleState]
    vehicle_specs: Mapping[str, VehicleSpec]
    assignment_records_by_mv: Mapping[str, Mapping[str, Any]]
    active_maneuvers: Mapping[str, ManeuverTrajectoryState]
    road_config_ref: str
    parameter_config_ref: str
    scenario_config_ref: str | None = None
    output_config_ref: str | None = None
    controller_memory_by_vehicle: Mapping[str, LongitudinalControllerMemory] = field(default_factory=dict)
    ramp_merge_runtime: Any | None = None


@dataclass(frozen=True)
class LaneChangeNeighborhood:
    vehicle_id: str
    source_lane: str
    target_lane: str
    tlv_id: str | None
    tfv_id: str | None
    lv_id: str | None
    fv_id: str | None
    snapshot_source: str = "step3_relations_snapshot"


@dataclass(frozen=True)
class ActiveManeuverRelation:
    vehicle_id: str
    primary_leader_id: str | None
    affected_target_follower_id: str | None
    affected_source_follower_id: str | None
    relation_source: str


@dataclass(frozen=True)
class RelationsSnapshot:
    step: int
    t: float
    lane_ordering: Mapping[str, tuple[str, ...]]
    leader_by_vehicle: Mapping[str, str | None]
    follower_by_vehicle: Mapping[str, str | None]
    lane_change_neighborhood: Mapping[str, LaneChangeNeighborhood]
    active_maneuver_relation: Mapping[str, ActiveManeuverRelation]


@dataclass(frozen=True)
class LaneCenterlineResult:
    lane_id: str
    y: float
    source_status: str = "first-version-default"


@dataclass(frozen=True)
class RegionResult:
    x_global: float
    road_role: str
    region_name: str
    before_merging_zone: bool
    in_merging_zone: bool
    past_ramp_end: bool
    uses_x_global: bool = True
    uses_x_plot: bool = False


@dataclass(frozen=True)
class OnRampControlRegion:
    x_global: float
    road_role: str
    region: str
    aps_allowed: bool
    cooperative_request_allowed: bool
    cuc_allowed: bool
    cmc_allowed: bool
    uses_x_global: bool = True
    uses_x_plot: bool = False


@dataclass(frozen=True)
class APSCandidateWindowResult:
    mv_id: str | None
    x_mv_global: float
    start_x_global: float
    end_x_global: float
    l_cr: float
    parameter_name: str = "L_cr"
    uses_fixed_cooperative_zone: bool = False
    uses_dynamic_coop_window: bool = False
    uses_x_global: bool = True
    uses_x_plot: bool = False


@dataclass(frozen=True)
class Step0To3RunResult:
    state: SimulationState
    relations: RelationsSnapshot
    actual_events: list[dict[str, Any]]
    actual_sanity_checks: list[dict[str, Any]]
    expected_png_features: list[dict[str, Any]]


def build_prefreeze_workspace_from_scenario(
    scenario: str | dict[str, Any],
    *,
    geometry: RoadGeometryConfig = DEFAULT_ROAD_GEOMETRY,
    command_buffer: dict[str, Any] | None = None,
    next_state_buffer: dict[str, Any] | None = None,
) -> tuple[PreFreezeWorkspace, dict[str, Any]]:
    if isinstance(scenario, str):
        config = load_scene_config(scenario)
    else:
        config = load_scenario_config(scenario)

    initial_time = config.get("initial_time") or {}
    t = float(initial_time.get("t", 0.0))
    step = int(initial_time.get("step", 0))
    dt = float(initial_time.get("dt", 0.1))
    vehicle_states: dict[str, VehicleState] = {}
    vehicle_specs: dict[str, VehicleSpec] = {}
    active_vehicle_ids: list[str] = []

    for initial_vehicle in config.get("initial_vehicles", []):
        state = _vehicle_state_from_initial(initial_vehicle)
        spec = _vehicle_spec_from_initial(initial_vehicle, step=step, t=t)
        active_vehicle_ids.append(state.vehicle_id)
        vehicle_states[state.vehicle_id] = state
        vehicle_specs[spec.vehicle_id] = spec

    active_maneuvers = _active_maneuvers_from_config(config, step=step, t=t, geometry=geometry)
    workspace = PreFreezeWorkspace(
        t=t,
        step=step,
        dt=dt,
        active_vehicle_ids=active_vehicle_ids,
        vehicle_states=vehicle_states,
        vehicle_specs=vehicle_specs,
        assignment_records_by_mv=_assignment_records_from_config(config),
        active_maneuvers=active_maneuvers,
        command_buffer=dict(command_buffer or {}),
        next_state_buffer=dict(next_state_buffer or {}),
        road_config_ref=str(config.get("road_config_ref") or geometry.config_id),
        parameter_config_ref=str(config.get("parameter_config_ref") or "paper_table_i_first_version"),
        scenario_config_ref=str(config.get("scenario_id")),
        output_config_ref=(
            str(config["output_config_ref"]) if config.get("output_config_ref") is not None else None
        ),
    )
    return workspace, config


def step0_cleanup_and_prepare(
    workspace: PreFreezeWorkspace,
    *,
    geometry: RoadGeometryConfig = DEFAULT_ROAD_GEOMETRY,
) -> dict[str, Any]:
    removed_vehicle_ids: list[str] = []
    mainline_converted_vehicle_ids: list[str] = []
    for vehicle_id in list(workspace.active_vehicle_ids):
        state = workspace.vehicle_states[vehicle_id]
        if state.x_global > geometry.mainline_end_global:
            removed_vehicle_ids.append(vehicle_id)
            workspace.active_vehicle_ids.remove(vehicle_id)
            workspace.vehicle_states.pop(vehicle_id, None)
            workspace.controller_memory_by_vehicle.pop(vehicle_id, None)
            continue
        if (
            state.road_role == ON_RAMP_MV_ROLE
            and state.merge_state == "merge_completed"
        ):
            workspace.vehicle_states[vehicle_id] = VehicleState(
                vehicle_id=state.vehicle_id,
                x_global=state.x_global,
                y=float(geometry.lane_centerlines[LANE_2]),
                v=state.v,
                a=state.a,
                physical_lane=LANE_2,
                road_role=MAINLINE,
                lane_change_state=state.lane_change_state,
                merge_state="normal",
                is_active=state.is_active,
            )
            mainline_converted_vehicle_ids.append(vehicle_id)

    workspace.command_buffer.clear()
    workspace.next_state_buffer.clear()
    return emit_cleanup_event_candidate(
        workspace,
        removed_vehicle_ids,
        mainline_converted_vehicle_ids=mainline_converted_vehicle_ids,
    )


def step1_prefreeze_boundary_generation_hook(
    workspace: PreFreezeWorkspace,
    scenario_config: dict[str, Any],
    *,
    new_vehicle_candidates: list[tuple[VehicleState, VehicleSpec]] | None = None,
    spawn_decisions: list[Any] | tuple[Any, ...] | None = None,
) -> dict[str, Any]:
    module_overrides = scenario_config.get("module_overrides") or {}
    enabled = bool(module_overrides.get("boundary_generation_enabled", False))
    generated_vehicle_ids: list[str] = []
    blocked_spawn_vehicle_ids: list[str] = []
    blocked_reasons: dict[str, str] = {}
    lane_ids: dict[str, str] = {}
    assigned_arrival_headways: dict[str, float] = {}
    seed: int | None = None
    profile_id: str | None = None
    reason = "disabled"

    if enabled:
        reason = "no_boundary_candidates"
        for state, spec in new_vehicle_candidates or []:
            if state.vehicle_id not in workspace.vehicle_states:
                workspace.active_vehicle_ids.append(state.vehicle_id)
            workspace.vehicle_states[state.vehicle_id] = state
            workspace.vehicle_specs[spec.vehicle_id] = spec
            generated_vehicle_ids.append(state.vehicle_id)
            lane_ids[state.vehicle_id] = state.physical_lane
            if spec.assigned_arrival_headway is not None:
                assigned_arrival_headways[state.vehicle_id] = spec.assigned_arrival_headway
        for decision in spawn_decisions or ():
            item = getattr(decision, "queue_item")
            state = getattr(item, "initial_state")
            spec = getattr(item, "spec")
            vehicle_id = str(getattr(item, "vehicle_id"))
            lane_ids[vehicle_id] = str(getattr(item, "lane_id"))
            assigned_arrival_headways[vehicle_id] = float(getattr(item, "assigned_arrival_headway"))
            seed = int(getattr(item, "seed"))
            profile_id = str(getattr(item, "profile_id"))
            if bool(getattr(decision, "generated")):
                if state.vehicle_id not in workspace.vehicle_states:
                    workspace.active_vehicle_ids.append(state.vehicle_id)
                workspace.vehicle_states[state.vehicle_id] = state
                workspace.vehicle_specs[spec.vehicle_id] = spec
                generated_vehicle_ids.append(state.vehicle_id)
            else:
                blocked_spawn_vehicle_ids.append(vehicle_id)
                blocked_reasons[vehicle_id] = str(
                    getattr(decision, "blocked_reason") or getattr(decision, "reason")
                )
        if generated_vehicle_ids:
            reason = "generated_pre_freeze"
        if blocked_spawn_vehicle_ids and not generated_vehicle_ids:
            reason = "blocked_safe_spawn_gap"
        elif blocked_spawn_vehicle_ids and generated_vehicle_ids:
            reason = "generated_and_blocked_pre_freeze"

    return emit_boundary_generation_event_candidate(
        workspace,
        enabled=enabled,
        reason=reason,
        generated_vehicle_ids=generated_vehicle_ids,
        blocked_spawn_vehicle_ids=blocked_spawn_vehicle_ids,
        blocked_reasons=blocked_reasons,
        lane_ids=lane_ids,
        assigned_arrival_headways=assigned_arrival_headways,
        seed=seed,
        profile_id=profile_id,
    )


def freeze_simulation_state(workspace: PreFreezeWorkspace) -> SimulationState:
    active_vehicle_ids = tuple(workspace.active_vehicle_ids)
    return SimulationState(
        t=workspace.t,
        step=workspace.step,
        dt=workspace.dt,
        active_vehicle_ids=active_vehicle_ids,
        vehicle_states=MappingProxyType(
            {vehicle_id: workspace.vehicle_states[vehicle_id] for vehicle_id in active_vehicle_ids}
        ),
        vehicle_specs=MappingProxyType(
            {
                vehicle_id: workspace.vehicle_specs[vehicle_id]
                for vehicle_id in active_vehicle_ids
                if vehicle_id in workspace.vehicle_specs
            }
        ),
        assignment_records_by_mv=_freeze_nested_mapping(workspace.assignment_records_by_mv),
        active_maneuvers=MappingProxyType(dict(workspace.active_maneuvers)),
        road_config_ref=workspace.road_config_ref,
        parameter_config_ref=workspace.parameter_config_ref,
        scenario_config_ref=workspace.scenario_config_ref,
        output_config_ref=workspace.output_config_ref,
        controller_memory_by_vehicle=MappingProxyType(dict(workspace.controller_memory_by_vehicle)),
        ramp_merge_runtime=workspace.ramp_merge_runtime,
    )


def build_prefreeze_workspace_from_state(state: SimulationState) -> PreFreezeWorkspace:
    return PreFreezeWorkspace(
        t=state.t,
        step=state.step,
        dt=state.dt,
        active_vehicle_ids=list(state.active_vehicle_ids),
        vehicle_states={
            vehicle_id: state.vehicle_states[vehicle_id]
            for vehicle_id in state.active_vehicle_ids
        },
        vehicle_specs={
            vehicle_id: state.vehicle_specs[vehicle_id]
            for vehicle_id in state.active_vehicle_ids
            if vehicle_id in state.vehicle_specs
        },
        assignment_records_by_mv={
            vehicle_id: dict(value)
            for vehicle_id, value in state.assignment_records_by_mv.items()
        },
        active_maneuvers=dict(state.active_maneuvers),
        command_buffer={},
        next_state_buffer={},
        controller_memory_by_vehicle=dict(state.controller_memory_by_vehicle),
        ramp_merge_runtime=state.ramp_merge_runtime,
        road_config_ref=state.road_config_ref,
        parameter_config_ref=state.parameter_config_ref,
        scenario_config_ref=state.scenario_config_ref,
        output_config_ref=state.output_config_ref,
    )


def refresh_relations_snapshot(
    state: SimulationState,
    *,
    geometry: RoadGeometryConfig = DEFAULT_ROAD_GEOMETRY,
) -> RelationsSnapshot:
    lane_ordering: dict[str, tuple[str, ...]] = {
        lane_id: tuple(resolve_lane_ordering_by_x_global(state, lane_id))
        for lane_id in geometry.lane_centerlines
    }
    leader_by_vehicle: dict[str, str | None] = {}
    follower_by_vehicle: dict[str, str | None] = {}

    for ordered_vehicle_ids in lane_ordering.values():
        for index, vehicle_id in enumerate(ordered_vehicle_ids):
            follower_by_vehicle[vehicle_id] = (
                ordered_vehicle_ids[index - 1] if index > 0 else None
            )
            leader_by_vehicle[vehicle_id] = (
                ordered_vehicle_ids[index + 1]
                if index < len(ordered_vehicle_ids) - 1
                else None
            )

    neighborhoods: dict[str, LaneChangeNeighborhood] = {}
    for vehicle_id in state.active_vehicle_ids:
        vehicle_state = state.vehicle_states[vehicle_id]
        if vehicle_state.physical_lane == LANE_2 or vehicle_state.lane_change_state == "executing":
            neighborhoods[vehicle_id] = _resolve_lane_change_neighborhood(state, vehicle_id)

    active_relations: dict[str, ActiveManeuverRelation] = {}
    for vehicle_id in state.active_vehicle_ids:
        vehicle_state = state.vehicle_states[vehicle_id]
        if vehicle_state.lane_change_state == "executing":
            neighborhood = neighborhoods.get(vehicle_id) or _resolve_lane_change_neighborhood(
                state,
                vehicle_id,
            )
            active_relations[vehicle_id] = ActiveManeuverRelation(
                vehicle_id=vehicle_id,
                primary_leader_id=neighborhood.tlv_id,
                affected_target_follower_id=neighborhood.tfv_id,
                affected_source_follower_id=neighborhood.fv_id,
                relation_source="active_lane_change",
            )
        elif vehicle_state.merge_state == "executing":
            leader_id, follower_id = _nearest_neighbors_by_x(
                state,
                LANE_2,
                vehicle_state.x_global,
                exclude_vehicle_id=vehicle_id,
            )
            active_relations[vehicle_id] = ActiveManeuverRelation(
                vehicle_id=vehicle_id,
                primary_leader_id=leader_id,
                affected_target_follower_id=follower_id,
                affected_source_follower_id=None,
                relation_source="active_merge",
            )

    return RelationsSnapshot(
        step=state.step,
        t=state.t,
        lane_ordering=MappingProxyType(lane_ordering),
        leader_by_vehicle=MappingProxyType(leader_by_vehicle),
        follower_by_vehicle=MappingProxyType(follower_by_vehicle),
        lane_change_neighborhood=MappingProxyType(neighborhoods),
        active_maneuver_relation=MappingProxyType(active_relations),
    )


def overlay_assignment_logical_relations(
    state: SimulationState,
    relations: RelationsSnapshot,
    *,
    assignment_views: Mapping[str, Any] | None = None,
) -> RelationsSnapshot:
    active_relations = dict(relations.active_maneuver_relation)
    overlay_records = _iter_assignment_overlay_records(state, assignment_views=assignment_views)
    for mv_id, record in overlay_records.items():
        for relation in _assignment_logical_relations_for_record(state, mv_id, record):
            active_relations[relation.vehicle_id] = relation
    return RelationsSnapshot(
        step=relations.step,
        t=relations.t,
        lane_ordering=relations.lane_ordering,
        leader_by_vehicle=relations.leader_by_vehicle,
        follower_by_vehicle=relations.follower_by_vehicle,
        lane_change_neighborhood=relations.lane_change_neighborhood,
        active_maneuver_relation=MappingProxyType(active_relations),
    )


def resolve_lane_ordering_by_x_global(state: SimulationState, lane_id: str) -> list[str]:
    vehicle_ids = [
        vehicle_id
        for vehicle_id in state.active_vehicle_ids
        if state.vehicle_states[vehicle_id].physical_lane == lane_id
        and state.vehicle_states[vehicle_id].is_active
    ]
    return sorted(
        vehicle_ids,
        key=lambda vehicle_id: (
            state.vehicle_states[vehicle_id].x_global,
            vehicle_id,
        ),
    )


def resolve_lane_2_gap_boundary_eligibility(
    state: SimulationState,
    vehicle_id: str | None,
    *,
    required_lane: str = LANE_2,
) -> Lane2GapBoundaryEligibility:
    if not vehicle_id or vehicle_id not in state.vehicle_states:
        return Lane2GapBoundaryEligibility(
            vehicle_id=vehicle_id,
            eligible=False,
            reason="missing",
            required_lane=required_lane,
        )
    vehicle_state = state.vehicle_states[vehicle_id]
    if not vehicle_state.is_active:
        return Lane2GapBoundaryEligibility(
            vehicle_id=vehicle_id,
            eligible=False,
            reason="inactive_vehicle",
            required_lane=required_lane,
            physical_lane=vehicle_state.physical_lane,
            lane_change_state=vehicle_state.lane_change_state,
            is_active=False,
        )
    if vehicle_state.physical_lane != required_lane:
        return Lane2GapBoundaryEligibility(
            vehicle_id=vehicle_id,
            eligible=False,
            reason=f"not_{required_lane}",
            required_lane=required_lane,
            physical_lane=vehicle_state.physical_lane,
            lane_change_state=vehicle_state.lane_change_state,
            is_active=True,
        )
    if vehicle_state.lane_change_state == "executing":
        return Lane2GapBoundaryEligibility(
            vehicle_id=vehicle_id,
            eligible=False,
            reason="lane_change_executing",
            required_lane=required_lane,
            physical_lane=vehicle_state.physical_lane,
            lane_change_state=vehicle_state.lane_change_state,
            is_active=True,
        )
    return Lane2GapBoundaryEligibility(
        vehicle_id=vehicle_id,
        eligible=True,
        reason="stable_lane_2",
        required_lane=required_lane,
        physical_lane=vehicle_state.physical_lane,
        lane_change_state=vehicle_state.lane_change_state,
        is_active=True,
    )


def resolve_lane_centerline(
    lane_id: str,
    *,
    geometry: RoadGeometryConfig = DEFAULT_ROAD_GEOMETRY,
) -> LaneCenterlineResult:
    try:
        y = float(geometry.lane_centerlines[lane_id])
    except KeyError as exc:
        raise ValueError(f"unknown lane id: {lane_id}") from exc
    return LaneCenterlineResult(lane_id=lane_id, y=y)


def resolve_region(
    x_global: float,
    road_role: str,
    *,
    geometry: RoadGeometryConfig = DEFAULT_ROAD_GEOMETRY,
) -> RegionResult:
    x_global = float(x_global)
    before = x_global < geometry.x0_m_global
    in_merging = geometry.x0_m_global <= x_global <= geometry.x_ramp_end_global
    past = x_global > geometry.x_ramp_end_global
    if before:
        region_name = "before_merging_zone"
    elif in_merging:
        region_name = "merging_zone"
    else:
        region_name = "past_ramp_end"
    return RegionResult(
        x_global=x_global,
        road_role=road_role,
        region_name=region_name,
        before_merging_zone=before,
        in_merging_zone=in_merging,
        past_ramp_end=past,
    )


def resolve_on_ramp_control_region(
    x_global: float,
    road_role: str,
    *,
    geometry: RoadGeometryConfig = DEFAULT_ROAD_GEOMETRY,
) -> OnRampControlRegion:
    x_global = float(x_global)
    control_start = float(geometry.control_start_global)
    merge_start = float(geometry.x0_m_global)
    ramp_end = float(geometry.x_ramp_end_global)
    if x_global < control_start:
        region = "pre_control"
        aps_allowed = False
        cooperative_request_allowed = False
        cuc_allowed = False
        cmc_allowed = False
    elif x_global < merge_start:
        region = "control_zone"
        aps_allowed = True
        cooperative_request_allowed = True
        cuc_allowed = True
        cmc_allowed = False
    elif x_global <= ramp_end:
        region = "merge_zone"
        aps_allowed = False
        cooperative_request_allowed = False
        cuc_allowed = False
        cmc_allowed = True
    else:
        region = "post_merge"
        aps_allowed = False
        cooperative_request_allowed = False
        cuc_allowed = False
        cmc_allowed = False
    return OnRampControlRegion(
        x_global=x_global,
        road_role=road_role,
        region=region,
        aps_allowed=aps_allowed,
        cooperative_request_allowed=cooperative_request_allowed,
        cuc_allowed=cuc_allowed,
        cmc_allowed=cmc_allowed,
    )


def resolve_aps_candidate_window(
    x_mv_global: float,
    *,
    mv_id: str | None = None,
    geometry: RoadGeometryConfig = DEFAULT_ROAD_GEOMETRY,
) -> APSCandidateWindowResult:
    x_mv_global = float(x_mv_global)
    l_cr = float(geometry.communication_range)
    return APSCandidateWindowResult(
        mv_id=mv_id,
        x_mv_global=x_mv_global,
        start_x_global=x_mv_global - l_cr,
        end_x_global=x_mv_global + l_cr,
        l_cr=l_cr,
    )


def resolve_aps_candidate_ids(
    state: SimulationState,
    mv_id: str,
    *,
    geometry: RoadGeometryConfig = DEFAULT_ROAD_GEOMETRY,
) -> list[str]:
    mv_state = state.vehicle_states[mv_id]
    window = resolve_aps_candidate_window(
        mv_state.x_global,
        mv_id=mv_id,
        geometry=geometry,
    )
    candidate_ids = [
        vehicle_id
        for vehicle_id in resolve_lane_ordering_by_x_global(state, LANE_2)
        if window.start_x_global
        <= state.vehicle_states[vehicle_id].x_global
        <= window.end_x_global
    ]
    return candidate_ids


def emit_cleanup_event_candidate(
    workspace: PreFreezeWorkspace,
    removed_vehicle_ids: list[str],
    *,
    mainline_converted_vehicle_ids: list[str] | None = None,
) -> dict[str, Any]:
    mainline_converted_vehicle_ids = list(mainline_converted_vehicle_ids or [])
    return _event(
        step=workspace.step,
        t=workspace.t,
        module="step0_cleanup",
        event_type="cleanup",
        vehicle_ids=list(workspace.active_vehicle_ids),
        reason="step0_cleanup_and_prepare",
        payload={
            "removed_vehicle_ids": list(removed_vehicle_ids),
            "mainline_converted_vehicle_ids": mainline_converted_vehicle_ids,
            "mainline_conversion_reason": (
                "onestep_stage2_merge_completed_step0_mainline_conversion"
                if mainline_converted_vehicle_ids
                else None
            ),
            "cleared_command_buffer": True,
            "cleared_next_state_buffer": True,
            "retained_assignment_record_vehicle_ids": sorted(workspace.assignment_records_by_mv),
            "retained_active_maneuver_vehicle_ids": sorted(workspace.active_maneuvers),
        },
    )


def emit_boundary_generation_event_candidate(
    workspace: PreFreezeWorkspace,
    *,
    enabled: bool,
    reason: str,
    generated_vehicle_ids: list[str],
    blocked_spawn_vehicle_ids: list[str] | None = None,
    blocked_reasons: Mapping[str, str] | None = None,
    lane_ids: Mapping[str, str] | None = None,
    assigned_arrival_headways: Mapping[str, float] | None = None,
    seed: int | None = None,
    profile_id: str | None = None,
) -> dict[str, Any]:
    blocked_spawn_vehicle_ids = list(blocked_spawn_vehicle_ids or [])
    payload = {
        "enabled": enabled,
        "generated_vehicle_ids": list(generated_vehicle_ids),
        "blocked_spawn_vehicle_ids": blocked_spawn_vehicle_ids,
        "blocked_reason": dict(blocked_reasons or {}),
        "lane_id": dict(lane_ids or {}),
        "assigned_arrival_headway": dict(assigned_arrival_headways or {}),
        "active_vehicle_ids": list(workspace.active_vehicle_ids),
        "freeze_phase": "pre_freeze",
    }
    if seed is not None:
        payload["seed"] = seed
    if profile_id is not None:
        payload["profile_id"] = profile_id
    payload["random_generation_complete_mechanism"] = (
        "p16_seeded_boundary_queue_spawn_decisions"
        if enabled and (generated_vehicle_ids or blocked_spawn_vehicle_ids)
        else "disabled_or_not_applicable"
    )
    return _event(
        step=workspace.step,
        t=workspace.t,
        module="step1_prefreeze_boundary_generation_hook",
        event_type="boundary_generation",
        vehicle_ids=list(generated_vehicle_ids),
        reason=reason,
        payload=payload,
    )


def emit_freeze_event_candidate(state: SimulationState) -> dict[str, Any]:
    return _event(
        step=state.step,
        t=state.t,
        module="step2_freeze",
        event_type="freeze",
        vehicle_ids=list(state.active_vehicle_ids),
        reason="simulation_state_snapshot_created",
        payload={
            "active_vehicle_ids": list(state.active_vehicle_ids),
            "active_vehicle_count": len(state.active_vehicle_ids),
            "snapshot_is_read_only": True,
            "command_buffer_included": False,
            "next_state_buffer_included": False,
            "relations_included": False,
            "history_included": False,
        },
    )


def emit_relation_refresh_event_candidate(
    state: SimulationState,
    relations: RelationsSnapshot,
) -> dict[str, Any]:
    lane_ordering_payload: list[dict[str, Any]] = []
    for lane_id, vehicle_ids in relations.lane_ordering.items():
        lane_ordering_payload.append(
            {
                "lane_id": lane_id,
                "ordered_vehicle_ids": list(vehicle_ids),
                "ordered_x_global": [
                    state.vehicle_states[vehicle_id].x_global
                    for vehicle_id in vehicle_ids
                ],
                "ordering_coordinate": "x_global",
                "x_plot_used": False,
            }
        )

    return _event(
        step=state.step,
        t=state.t,
        module="step3_relation_refresh",
        event_type="relation_refresh",
        vehicle_ids=list(state.active_vehicle_ids),
        reason="relations_snapshot_refreshed_from_frozen_state",
        payload={
            "snapshot_step": relations.step,
            "lane_ordering": lane_ordering_payload,
            "leader_by_vehicle": dict(relations.leader_by_vehicle),
            "follower_by_vehicle": dict(relations.follower_by_vehicle),
            "lane_change_neighborhood": {
                vehicle_id: _dataclass_to_plain(neighborhood)
                for vehicle_id, neighborhood in relations.lane_change_neighborhood.items()
            },
            "active_maneuver_relation": {
                vehicle_id: _dataclass_to_plain(relation)
                for vehicle_id, relation in relations.active_maneuver_relation.items()
            },
            "relations_based_on_frozen_s_t": True,
        },
    )


def emit_geometry_event_candidate(
    state: SimulationState,
    *,
    geometry: RoadGeometryConfig = DEFAULT_ROAD_GEOMETRY,
) -> dict[str, Any]:
    aps_windows: dict[str, dict[str, Any]] = {}
    control_regions: dict[str, dict[str, Any]] = {}
    for vehicle_id in state.active_vehicle_ids:
        vehicle_state = state.vehicle_states[vehicle_id]
        if vehicle_state.physical_lane == ON_RAMP or vehicle_state.road_role in {
            ON_RAMP_ROLE,
            ON_RAMP_MV_ROLE,
        }:
            control_region = resolve_on_ramp_control_region(
                vehicle_state.x_global,
                vehicle_state.road_role,
                geometry=geometry,
            )
            control_regions[vehicle_id] = _dataclass_to_plain(control_region)
            window = resolve_aps_candidate_window(
                vehicle_state.x_global,
                mv_id=vehicle_id,
                geometry=geometry,
            )
            aps_windows[vehicle_id] = {
                **_dataclass_to_plain(window),
                "candidate_vehicle_ids": resolve_aps_candidate_ids(
                    state,
                    vehicle_id,
                    geometry=geometry,
                ),
            }

    return _event(
        step=state.step,
        t=state.t,
        module="geometry",
        event_type="geometry",
        vehicle_ids=list(state.active_vehicle_ids),
        reason="geometry_resolvers_available",
        payload={
            "lane_centerlines": dict(geometry.lane_centerlines),
            "control_zone_global": [
                geometry.control_start_global,
                geometry.x0_m_global,
            ],
            "merging_zone_global": [
                geometry.x0_m_global,
                geometry.x_ramp_end_global,
            ],
            "fixed_cooperative_zone_global": [
                geometry.x0_m_global - geometry.l_coop_fixed,
                geometry.x0_m_global,
            ],
            "on_ramp_control_regions": control_regions,
            "aps_candidate_windows": aps_windows,
            "aps_candidate_window_parameter": "L_cr",
            "uses_fixed_cooperative_zone_for_aps_window": False,
            "uses_x_global": True,
            "uses_x_plot": False,
        },
    )


def run_geometry_sanity_baseline(
    state: SimulationState,
    relations: RelationsSnapshot,
    *,
    geometry: RoadGeometryConfig = DEFAULT_ROAD_GEOMETRY,
    commit_executed: bool = False,
) -> list[dict[str, Any]]:
    checks = [
        _sanity(state, "collision", "pass", reason="not_evaluated_as_collision_in_p02_baseline"),
        _sanity(
            state,
            "near_collision",
            "pass",
            reason="not_evaluated_as_near_collision_in_p02_baseline",
        ),
        _sanity(
            state,
            "state_machine_inconsistency",
            _state_machine_status(state),
            reason="state_machine_baseline",
        ),
        _sanity(
            state,
            "unexpected_ordinary_lane_change_attempt",
            "pass",
            reason="ordinary_mainline_lane_change_not_executed_in_p02",
        ),
        _sanity(
            state,
            "multiple_commit_for_one_vehicle",
            "pass" if commit_executed else "not_applicable",
            reason="commit_not_executed_in_p02" if not commit_executed else "commit_executed",
        ),
        _sanity(
            state,
            "x_plot_used_in_algorithm_path",
            "pass" if assert_x_plot_not_used_in_algorithm_path(state, relations) else "fail",
            reason="x_global_only_algorithm_path",
            payload={"x_plot_used_in_algorithm_path": False},
        ),
        _sanity(
            state,
            "geometry_inconsistency",
            _geometry_status(geometry),
            reason="geometry_resolver_baseline",
        ),
        _sanity(
            state,
            "relations_consistency",
            _relations_status(state, relations),
            reason="relations_snapshot_baseline",
        ),
    ]
    return checks


def assert_x_plot_not_used_in_algorithm_path(*objects: Any) -> bool:
    return not any(_contains_x_plot_key(obj) for obj in objects)


def run_step0_to_step3(
    scenario: str | dict[str, Any],
    *,
    geometry: RoadGeometryConfig = DEFAULT_ROAD_GEOMETRY,
    command_buffer: dict[str, Any] | None = None,
    next_state_buffer: dict[str, Any] | None = None,
    boundary_vehicle_candidates: list[tuple[VehicleState, VehicleSpec]] | None = None,
    boundary_spawn_decisions: list[Any] | tuple[Any, ...] | None = None,
) -> Step0To3RunResult:
    workspace, config = build_prefreeze_workspace_from_scenario(
        scenario,
        geometry=geometry,
        command_buffer=command_buffer,
        next_state_buffer=next_state_buffer,
    )
    events = [
        step0_cleanup_and_prepare(workspace, geometry=geometry),
        step1_prefreeze_boundary_generation_hook(
            workspace,
            config,
            new_vehicle_candidates=boundary_vehicle_candidates,
            spawn_decisions=boundary_spawn_decisions,
        ),
    ]
    state = freeze_simulation_state(workspace)
    relations = refresh_relations_snapshot(state, geometry=geometry)
    events.extend(
        [
            emit_freeze_event_candidate(state),
            emit_relation_refresh_event_candidate(state, relations),
            emit_geometry_event_candidate(state, geometry=geometry),
        ]
    )
    sanity_checks = run_geometry_sanity_baseline(state, relations, geometry=geometry)
    expected_png_features = [
        {
            "feature_type": "lane_centerline_quicklook",
            "required": False,
            "vehicle_ids": [],
            "expected_visibility": "optional",
            "notes": "registered only; renderer deferred",
        },
        {
            "feature_type": "merging_zone_boundary_quicklook",
            "required": False,
            "vehicle_ids": [],
            "expected_visibility": "optional",
            "notes": "registered only; renderer deferred",
        },
        {
            "feature_type": "aps_candidate_window_quicklook",
            "required": False,
            "vehicle_ids": list(state.active_vehicle_ids),
            "expected_visibility": "optional",
            "notes": "registered only; renderer deferred",
        },
    ]
    return Step0To3RunResult(
        state=state,
        relations=relations,
        actual_events=events,
        actual_sanity_checks=sanity_checks,
        expected_png_features=expected_png_features,
    )


def _vehicle_state_from_initial(initial_vehicle: dict[str, Any]) -> VehicleState:
    return VehicleState(
        vehicle_id=str(initial_vehicle["vehicle_id"]),
        x_global=float(initial_vehicle["initial_x_global"]),
        y=float(initial_vehicle["initial_y"]),
        v=float(initial_vehicle["initial_v"]),
        a=float(initial_vehicle.get("initial_a", 0.0)),
        physical_lane=str(initial_vehicle["physical_lane"]),
        road_role=str(initial_vehicle["road_role"]),
        lane_change_state=str(initial_vehicle.get("lane_change_state", "normal")).lower(),
        merge_state=str(initial_vehicle.get("merge_state", "none")).lower(),
        is_active=True,
    )


def _vehicle_spec_from_initial(
    initial_vehicle: dict[str, Any],
    *,
    step: int,
    t: float,
) -> VehicleSpec:
    overrides = initial_vehicle.get("spec_overrides") or {}
    return VehicleSpec(
        vehicle_id=str(initial_vehicle["vehicle_id"]),
        vehicle_type=str(initial_vehicle["vehicle_type"]).lower(),
        compliance_state=_normalize_compliance_state(
            initial_vehicle.get("compliance_state", "not_applicable")
        ),
        desired_speed=_optional_float(overrides.get("desired_speed")),
        desired_time_gap=_optional_float(overrides.get("desired_time_gap")),
        desired_time_gap_class=overrides.get("desired_time_gap_class"),
        assigned_arrival_headway=_optional_float(overrides.get("assigned_arrival_headway")),
        inertial_lag=_optional_float(overrides.get("inertial_lag")),
        length=float(overrides.get("length", 4.0)),
        source_lane_at_generation=str(initial_vehicle["physical_lane"]),
        generation_step=step,
        generation_t=t,
    )


def _active_maneuvers_from_config(
    config: dict[str, Any],
    *,
    step: int,
    t: float,
    geometry: RoadGeometryConfig,
) -> dict[str, ManeuverTrajectoryState]:
    active_maneuvers: dict[str, ManeuverTrajectoryState] = {}
    for item in config.get("preloaded_maneuver_trajectory_states", []):
        vehicle_id = str(item["vehicle_id"])
        target_lane = str(item.get("target_lane") or LANE_1)
        target_y = float(item.get("target_y", geometry.lane_centerlines[target_lane]))
        active_maneuvers[vehicle_id] = ManeuverTrajectoryState(
            vehicle_id=vehicle_id,
            maneuver_type=str(item["maneuver_type"]),
            start_step=int(item.get("start_step", step)),
            start_t=float(item.get("start_t", t)),
            start_x_global=float(item["start_x_global"]),
            start_y=float(item["start_y"]),
            target_lane=target_lane,
            target_y=target_y,
            planned_length=_optional_float(item.get("planned_length")),
            progress=float(item.get("progress", 0.0)),
            assigned_clv_id=item.get("assigned_clv_id"),
            assigned_cfv_id=item.get("assigned_cfv_id"),
        )
    return active_maneuvers


def _assignment_records_from_config(config: dict[str, Any]) -> dict[str, dict[str, Any]]:
    initial_time = config.get("initial_time") or {}
    step = int(initial_time.get("step", 0))
    t = float(initial_time.get("t", 0.0))
    cache: dict[str, dict[str, Any]] = {}
    for item in config.get("preloaded_assignments", []):
        record = assignment_lifecycle_manager.from_legacy_assignment(item, step=step, t=t)
        cache[str(item["mv_id"])] = assignment_lifecycle_manager.to_state_dict(record)
    return cache


def _resolve_lane_change_neighborhood(
    state: SimulationState,
    vehicle_id: str,
) -> LaneChangeNeighborhood:
    vehicle_state = state.vehicle_states[vehicle_id]
    source_lane = vehicle_state.physical_lane
    active_maneuver = state.active_maneuvers.get(vehicle_id)
    target_lane = active_maneuver.target_lane if active_maneuver is not None else LANE_1
    if source_lane == target_lane:
        source_lane = LANE_2

    tlv_id, tfv_id = _nearest_neighbors_by_x(
        state,
        target_lane,
        vehicle_state.x_global,
        exclude_vehicle_id=vehicle_id,
    )
    lv_id, fv_id = _nearest_neighbors_by_x(
        state,
        source_lane,
        vehicle_state.x_global,
        exclude_vehicle_id=vehicle_id,
    )
    return LaneChangeNeighborhood(
        vehicle_id=vehicle_id,
        source_lane=source_lane,
        target_lane=target_lane,
        tlv_id=tlv_id,
        tfv_id=tfv_id,
        lv_id=lv_id,
        fv_id=fv_id,
    )


def _nearest_neighbors_by_x(
    state: SimulationState,
    lane_id: str,
    x_global: float,
    *,
    exclude_vehicle_id: str | None = None,
) -> tuple[str | None, str | None]:
    ordered = [
        vehicle_id
        for vehicle_id in resolve_lane_ordering_by_x_global(state, lane_id)
        if vehicle_id != exclude_vehicle_id
    ]
    leader_id: str | None = None
    follower_id: str | None = None
    for vehicle_id in ordered:
        candidate_x = state.vehicle_states[vehicle_id].x_global
        if candidate_x > x_global and leader_id is None:
            leader_id = vehicle_id
        if candidate_x < x_global:
            follower_id = vehicle_id
    return leader_id, follower_id


def _iter_assignment_overlay_records(
    state: SimulationState,
    *,
    assignment_views: Mapping[str, Any] | None,
) -> dict[str, Mapping[str, Any]]:
    records: dict[str, Mapping[str, Any]] = {
        str(mv_id): record for mv_id, record in state.assignment_records_by_mv.items()
    }
    for mv_id, view in (assignment_views or {}).items():
        record = getattr(view, "record", None)
        if record is None and isinstance(view, Mapping):
            record = view.get("record") or view
        if isinstance(record, Mapping):
            records[str(mv_id)] = record
    return records


def _assignment_logical_relations_for_record(
    state: SimulationState,
    mv_id: str,
    record: Mapping[str, Any],
) -> tuple[ActiveManeuverRelation, ...]:
    mv_id = str(record.get("mv_id") or mv_id)
    clv_id = _optional_assignment_str(record.get("clv_id"))
    cfv_id = _optional_assignment_str(record.get("cfv_id"))
    if mv_id not in state.vehicle_states or clv_id not in state.vehicle_states:
        return ()
    mv_state = state.vehicle_states[mv_id]
    clv_state = state.vehicle_states[clv_id]
    if mv_state.physical_lane != ON_RAMP and mv_state.road_role != ON_RAMP_MV_ROLE:
        return ()
    if mv_state.merge_state == "merged":
        return ()
    if str(record.get("gap_type") or "bounded").lower() != "bounded":
        return ()
    if str(record.get("status") or "valid").lower() not in {"valid", "available", "ok"}:
        return ()
    lifecycle_state = str(record.get("lifecycle_state") or "active_control_zone").lower()
    if lifecycle_state not in {"active_control_zone", "refresh_failed_retained", "active_merge_zone"}:
        return ()
    if not clv_state.is_active:
        return ()
    if clv_state.physical_lane != LANE_2 or clv_state.lane_change_state == "executing":
        return ()
    if float(clv_state.x_global) <= float(mv_state.x_global):
        return ()
    relations = [
        ActiveManeuverRelation(
            vehicle_id=mv_id,
            primary_leader_id=clv_id,
            affected_target_follower_id=cfv_id,
            affected_source_follower_id=None,
            relation_source=f"aps_assignment_{record.get('aps_case')}_mv_clv_leader",
        )
    ]
    rear_boundary = _assignment_rear_boundary_relation_for_record(
        state,
        mv_id,
        cfv_id=cfv_id,
        record=record,
    )
    if rear_boundary is not None:
        relations.append(rear_boundary)
    return tuple(relations)


def _assignment_rear_boundary_relation_for_record(
    state: SimulationState,
    mv_id: str,
    *,
    cfv_id: str | None,
    record: Mapping[str, Any],
) -> ActiveManeuverRelation | None:
    if cfv_id not in state.vehicle_states:
        return None
    mv_state = state.vehicle_states[mv_id]
    cfv_state = state.vehicle_states[cfv_id]
    eligibility = resolve_lane_2_gap_boundary_eligibility(state, cfv_id)
    if not eligibility.eligible:
        return None
    lifecycle_state = str(record.get("lifecycle_state") or "active_control_zone").lower()
    if lifecycle_state != "active_merge_zone":
        return None
    aps_case = str(record.get("aps_case") or "").lower()
    if aps_case not in {"case_2", "case_4"} and not _truthy_assignment_bool(record.get("col_cfv")):
        return None
    if float(cfv_state.x_global) >= float(mv_state.x_global):
        return None
    if _lane2_vehicle_between(state, cfv_id=cfv_id, mv_id=mv_id):
        return None
    return ActiveManeuverRelation(
        vehicle_id=cfv_id,
        primary_leader_id=mv_id,
        affected_target_follower_id=None,
        affected_source_follower_id=None,
        relation_source=f"aps_assignment_{record.get('aps_case')}_cfv_mv_rear_boundary",
    )


def _lane2_vehicle_between(state: SimulationState, *, cfv_id: str, mv_id: str) -> bool:
    cfv_x = float(state.vehicle_states[cfv_id].x_global)
    mv_x = float(state.vehicle_states[mv_id].x_global)
    for vehicle_id in resolve_lane_ordering_by_x_global(state, LANE_2):
        if vehicle_id in {cfv_id, mv_id}:
            continue
        vehicle_state = state.vehicle_states[vehicle_id]
        if cfv_x < float(vehicle_state.x_global) < mv_x:
            return True
    return False


def _optional_assignment_str(value: Any) -> str | None:
    if value in (None, ""):
        return None
    return str(value)


def _truthy_assignment_bool(value: Any) -> bool:
    if isinstance(value, str):
        return value.lower() in {"1", "true", "yes", "y"}
    return bool(value)


def _state_machine_status(state: SimulationState) -> str:
    for vehicle_id in state.active_vehicle_ids:
        vehicle_state = state.vehicle_states[vehicle_id]
        if (
            vehicle_state.lane_change_state == "executing"
            and vehicle_state.merge_state == "executing"
        ):
            return "fail"
    return "pass"


def _geometry_status(geometry: RoadGeometryConfig) -> str:
    expected = {
        LANE_1: geometry.lane_width,
        LANE_2: 0.0,
        ON_RAMP: -geometry.lane_width,
    }
    for lane_id, expected_y in expected.items():
        if float(geometry.lane_centerlines[lane_id]) != float(expected_y):
            return "fail"
    if geometry.x_ramp_end_global != geometry.x0_m_global + geometry.l_merging:
        return "fail"
    if not (geometry.control_start_global < geometry.x0_m_global < geometry.x_ramp_end_global):
        return "fail"
    return "pass"


def _relations_status(state: SimulationState, relations: RelationsSnapshot) -> str:
    for lane_id, vehicle_ids in relations.lane_ordering.items():
        ordered_x = [state.vehicle_states[vehicle_id].x_global for vehicle_id in vehicle_ids]
        if ordered_x != sorted(ordered_x):
            return "fail"
        for vehicle_id in vehicle_ids:
            if state.vehicle_states[vehicle_id].physical_lane != lane_id:
                return "fail"
    return "pass"


def _normalize_compliance_state(value: Any) -> str:
    lowered = str(value).lower()
    if lowered == "none":
        return "not_applicable"
    return lowered


def _optional_float(value: Any) -> float | None:
    if value is None:
        return None
    return float(value)


def _event(
    *,
    step: int,
    t: float,
    module: str,
    event_type: str,
    vehicle_ids: list[str],
    reason: str,
    payload: dict[str, Any],
) -> dict[str, Any]:
    return {
        "step": step,
        "t": t,
        "module": module,
        "event_type": event_type,
        "vehicle_ids": list(vehicle_ids),
        "reason": reason,
        "source": "first_version_engineering_patch",
        "is_engineering_patch": True,
        "payload": payload,
    }


def _sanity(
    state: SimulationState,
    check_type: str,
    result: str,
    *,
    reason: str,
    payload: dict[str, Any] | None = None,
) -> dict[str, Any]:
    return {
        "step": state.step,
        "t": state.t,
        "check_type": check_type,
        "result": result,
        "vehicle_ids": list(state.active_vehicle_ids),
        "reason": reason,
        "payload": payload or {},
    }


def _freeze_nested_mapping(source: Mapping[str, Any]) -> Mapping[str, Mapping[str, Any]]:
    return MappingProxyType(
        {key: MappingProxyType(deepcopy(value)) for key, value in source.items()}
    )


def _dataclass_to_plain(value: Any) -> dict[str, Any]:
    return {field.name: getattr(value, field.name) for field in fields(value)}


def _contains_x_plot_key(value: Any) -> bool:
    if is_dataclass(value):
        return any(
            field.name == "x_plot" or _contains_x_plot_key(getattr(value, field.name))
            for field in fields(value)
        )
    if isinstance(value, Mapping):
        return any(
            key == "x_plot" or _contains_x_plot_key(nested_value)
            for key, nested_value in value.items()
        )
    if isinstance(value, (list, tuple, set)):
        return any(_contains_x_plot_key(item) for item in value)
    return False
