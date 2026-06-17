from __future__ import annotations

from dataclasses import dataclass, field, replace
from types import MappingProxyType
from typing import Any, Mapping

from cormc.simulation_core.pre_freeze import (
    DEFAULT_ROAD_GEOMETRY,
    ON_RAMP,
    ON_RAMP_MV_ROLE,
    RoadGeometryConfig,
    SimulationState,
    VehicleState,
)
from cormc.onestep.kernel.config import ScenarioConfig
from cormc.onestep.kernel.models import (
    Gap,
    GapEvaluationRow,
    ScoreResult,
)


ZONE_OUTSIDE_CONTROL = "outside_control_zone"
ZONE_CONTROL = "control_zone"
ZONE_MERGE = "merge_zone"
ZONE_OUT_OF_SCENE = "out_of_scene"

TRIGGER_NONE = "none"
TRIGGER_PERIODIC = "periodic"
TRIGGER_SAFETY_ALERT = "safety_alert"
TRIGGER_MV_ENTER_CONTROL_ZONE = "MV_enter_control_zone"

EFFECTIVE_CONTROL_BOTH = "both_controllable"
EFFECTIVE_CONTROL_FRONT = "front_controllable"
EFFECTIVE_CONTROL_REAR = "rear_controllable"
EFFECTIVE_CONTROL_NONE = "none_controllable"

BUNDLE_SHAPE_MV_FRONT_REAR = "mv_front_rear"
BUNDLE_SHAPE_MV_FRONT = "mv_front"
BUNDLE_SHAPE_MV_REAR = "mv_rear"
BUNDLE_SHAPE_MV_ONLY = "mv_only"


@dataclass(frozen=True)
class OneStepBoundaryState:
    vehicle_id: str
    x_global: float
    v: float
    a: float


@dataclass(frozen=True)
class GapRef:
    gap_id: str
    index: int
    front_vehicle_id: str | None
    rear_vehicle_id: str | None
    snapshot_step: int
    snapshot_t: float


@dataclass(frozen=True)
class GapPlan:
    plan_id: str
    mv_id: str
    gap_id: str
    gap_index: int
    front_vehicle_id: str | None
    rear_vehicle_id: str | None
    snapshot_step: int
    snapshot_t: float
    merge_point_x_global: float | None = None
    score_summary: Mapping[str, Any] = field(
        default_factory=lambda: MappingProxyType({})
    )
    bundle_id: str | None = None


@dataclass(frozen=True)
class LateralTrajectoryRef:
    trajectory_id: str
    owner_mv_id: str
    source_plan_id: str
    start_step: int
    start_t: float
    start_y: float
    target_y: float
    duration_steps: int


@dataclass(frozen=True)
class PlannerState:
    T_plan: float = 2.0
    next_plan_time: float = 0.0
    last_trigger_reason: str | None = None


@dataclass(frozen=True)
class MVPlanState:
    mv_id: str
    zone_state: str = ZONE_OUTSIDE_CONTROL
    merge_state: str = "normal"
    current_plan_gap: GapRef | None = None
    active_bundle_id: str | None = None
    locked_gap: GapRef | None = None
    planned_trajectory_id: str | None = None
    current_plan_id: str | None = None
    locked_plan_id: str | None = None
    active_lateral_trajectory_id: str | None = None
    last_plan_step: int | None = None
    last_plan_t: float | None = None


@dataclass(frozen=True)
class OneStepControlledVehicleState:
    vehicle_id: str
    owner_mv_id: str
    bundle_id: str
    role: str
    controlled_since_step: int


@dataclass(frozen=True)
class OneStepPlanBundle:
    bundle_id: str
    mv_id: str
    start_step: int
    start_t: float
    trigger_reason: str
    origin_x_global: float
    selected_gap: GapRef
    selected_rear_vehicle_id: str
    selected_front_vehicle_id: str
    selected_vehicle_ids: tuple[str, ...]
    bundle_shape: str
    controlled_vehicle_ids: tuple[str, ...]
    controlled_roles_by_vehicle_id: Mapping[str, str]
    lane_2_vehicle_order: tuple[str, ...]
    local_scenario: ScenarioConfig
    best_gap: Gap
    best_score: ScoreResult
    boundary_state_by_vehicle_id: Mapping[str, OneStepBoundaryState] = field(
        default_factory=lambda: MappingProxyType({})
    )
    required_longitudinal_gap_m: float = 0.0
    gap_rows: tuple[GapEvaluationRow, ...] = ()
    merge_point_x_global: float = 0.0


@dataclass(frozen=True)
class PlannedTrajectory:
    trajectory_id: str
    mv_id: str
    kind: str
    start_step: int
    start_t: float
    target_gap: GapRef | None = None
    start_x_global: float = 0.0
    start_y: float = 0.0
    target_y: float = 0.0
    duration_steps: int = 0
    progress_step: int = 0
    points: tuple[Any, ...] = ()


@dataclass(frozen=True)
class SafetyCheckResult:
    step: int
    t: float
    safety_alert: bool
    danger_vehicle_ids: tuple[str, ...] = ()
    danger_pairs: tuple[Mapping[str, Any], ...] = ()
    ttc_threshold_s: float = 1.5
    min_gap_m: float = 2.0


@dataclass(frozen=True)
class TriggerDecision:
    trigger_plan: bool
    trigger_reason: str
    active_trigger_reasons: tuple[str, ...]
    periodic_due: bool
    safety_alert: bool
    entry_plan_trigger: bool
    entry_vehicle_ids: tuple[str, ...]
    planner_state: PlannerState


@dataclass(frozen=True)
class GapCandidate:
    gap_id: str
    index: int
    front_vehicle_id: str
    rear_vehicle_id: str
    front_x_global: float
    rear_x_global: float
    bumper_gap_m: float
    effective_control_type: str


@dataclass(frozen=True)
class GapSnapshot:
    step: int
    t: float
    lane_id: str
    gaps: tuple[GapCandidate, ...] = ()
    danger_vehicle_ids: tuple[str, ...] = ()


@dataclass(frozen=True)
class RampMergeRuntimeState:
    planner_state: PlannerState = field(default_factory=PlannerState)
    mv_plan_states: Mapping[str, MVPlanState] = field(
        default_factory=lambda: MappingProxyType({})
    )
    onestep_plan_bundles: Mapping[str, OneStepPlanBundle] = field(
        default_factory=lambda: MappingProxyType({})
    )
    controlled_vehicle_states: Mapping[str, OneStepControlledVehicleState] = field(
        default_factory=lambda: MappingProxyType({})
    )
    planned_trajectories: Mapping[str, PlannedTrajectory] = field(
        default_factory=lambda: MappingProxyType({})
    )
    gap_plans: Mapping[str, GapPlan] = field(
        default_factory=lambda: MappingProxyType({})
    )
    lateral_trajectories: Mapping[str, LateralTrajectoryRef] = field(
        default_factory=lambda: MappingProxyType({})
    )
    danger_vehicle_ids: tuple[str, ...] = ()
    last_gap_snapshot: GapSnapshot | None = None
    version: str = "batch_c_v1"


def initialize_runtime_state(
    state: SimulationState,
    geometry: RoadGeometryConfig = DEFAULT_ROAD_GEOMETRY,
) -> RampMergeRuntimeState:
    from cormc.onestep.rolling.planner import derive_zone_state

    mv_plan_states = {
        vehicle_id: MVPlanState(
            mv_id=vehicle_id,
            zone_state=derive_zone_state(state.vehicle_states[vehicle_id], geometry),
        )
        for vehicle_id in state.active_vehicle_ids
        if _is_on_ramp_merge_vehicle(state.vehicle_states[vehicle_id])
    }
    return RampMergeRuntimeState(
        mv_plan_states=MappingProxyType(mv_plan_states),
        onestep_plan_bundles=MappingProxyType({}),
        controlled_vehicle_states=MappingProxyType({}),
        planned_trajectories=MappingProxyType({}),
        gap_plans=MappingProxyType({}),
        lateral_trajectories=MappingProxyType({}),
    )


def refresh_runtime_state(
    previous_runtime: RampMergeRuntimeState | None,
    state: SimulationState,
    geometry: RoadGeometryConfig = DEFAULT_ROAD_GEOMETRY,
) -> RampMergeRuntimeState:
    from cormc.onestep.rolling.planner import derive_zone_state

    if previous_runtime is None:
        return initialize_runtime_state(state, geometry=geometry)

    active_mv_ids = tuple(
        vehicle_id
        for vehicle_id in state.active_vehicle_ids
        if _is_on_ramp_merge_vehicle(state.vehicle_states[vehicle_id])
    )
    refreshed_mv_states: dict[str, MVPlanState] = {}
    for vehicle_id in active_mv_ids:
        previous_mv_state = previous_runtime.mv_plan_states.get(
            vehicle_id,
            MVPlanState(mv_id=vehicle_id),
        )
        refreshed_mv_states[vehicle_id] = replace(
            previous_mv_state,
            zone_state=derive_zone_state(state.vehicle_states[vehicle_id], geometry),
        )
    active_vehicle_ids = set(state.active_vehicle_ids)
    active_mv_id_set = set(active_mv_ids)
    onestep_plan_bundles = {
        bundle_id: bundle
        for bundle_id, bundle in previous_runtime.onestep_plan_bundles.items()
        if bundle.mv_id in active_mv_id_set
    }
    controlled_vehicle_states = {
        vehicle_id: controlled_state
        for vehicle_id, controlled_state in previous_runtime.controlled_vehicle_states.items()
        if vehicle_id in active_vehicle_ids
        and controlled_state.bundle_id in onestep_plan_bundles
        and controlled_state.owner_mv_id in active_mv_id_set
    }
    planned_trajectories = {
        trajectory_id: trajectory
        for trajectory_id, trajectory in previous_runtime.planned_trajectories.items()
        if trajectory.mv_id in active_vehicle_ids
    }
    gap_plans = {
        plan_id: plan
        for plan_id, plan in previous_runtime.gap_plans.items()
        if plan.mv_id in active_mv_id_set
    }
    lateral_trajectories = {
        trajectory_id: trajectory
        for trajectory_id, trajectory in previous_runtime.lateral_trajectories.items()
        if trajectory.owner_mv_id in active_mv_id_set
    }
    danger_vehicle_ids = tuple(
        vehicle_id
        for vehicle_id in previous_runtime.danger_vehicle_ids
        if vehicle_id in active_vehicle_ids
    )
    valid_bundle_ids = set(onestep_plan_bundles)
    for vehicle_id, mv_state in tuple(refreshed_mv_states.items()):
        if mv_state.active_bundle_id is not None and mv_state.active_bundle_id not in valid_bundle_ids:
            refreshed_mv_states[vehicle_id] = replace(
                mv_state,
                active_bundle_id=None,
                current_plan_gap=None,
            )
    valid_plan_ids = set(gap_plans)
    valid_lateral_trajectory_ids = set(lateral_trajectories)
    for vehicle_id, mv_state in tuple(refreshed_mv_states.items()):
        updates: dict[str, None] = {}
        if (
            mv_state.current_plan_id is not None
            and mv_state.current_plan_id not in valid_plan_ids
        ):
            updates["current_plan_id"] = None
        if (
            mv_state.locked_plan_id is not None
            and mv_state.locked_plan_id not in valid_plan_ids
        ):
            updates["locked_plan_id"] = None
        if (
            mv_state.active_lateral_trajectory_id is not None
            and mv_state.active_lateral_trajectory_id
            not in valid_lateral_trajectory_ids
        ):
            updates["active_lateral_trajectory_id"] = None
        if updates:
            refreshed_mv_states[vehicle_id] = replace(mv_state, **updates)
    return replace(
        previous_runtime,
        mv_plan_states=MappingProxyType(refreshed_mv_states),
        onestep_plan_bundles=MappingProxyType(onestep_plan_bundles),
        controlled_vehicle_states=MappingProxyType(controlled_vehicle_states),
        planned_trajectories=MappingProxyType(planned_trajectories),
        gap_plans=MappingProxyType(gap_plans),
        lateral_trajectories=MappingProxyType(lateral_trajectories),
        danger_vehicle_ids=danger_vehicle_ids,
        version=previous_runtime.version,
    )


def _is_on_ramp_merge_vehicle(vehicle_state: VehicleState) -> bool:
    return (
        vehicle_state.road_role == ON_RAMP_MV_ROLE
        or vehicle_state.physical_lane == ON_RAMP
    )
