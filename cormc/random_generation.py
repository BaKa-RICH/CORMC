from __future__ import annotations

import random
from dataclasses import dataclass, field
from typing import Any, Iterable, Mapping

from cormc.step0_3 import (
    DEFAULT_ROAD_GEOMETRY,
    LANE_1,
    LANE_2,
    MAINLINE,
    ON_RAMP,
    ON_RAMP_ROLE,
    RoadGeometryConfig,
    SimulationState,
    VehicleSpec,
    VehicleState,
)


ALLOWED_RANDOM_LANES = frozenset({LANE_1, LANE_2, ON_RAMP})
P16_DEMO_SCENARIO_ID = "P16-RANDOM-DEMO-internal"
DEFAULT_P16_SEED = 16001
DEFAULT_P16_MAX_STEPS = 100


@dataclass(frozen=True)
class ArrivalStream:
    lane_id: str
    shifted_headway: float
    initial_speed: float
    spawn_x: float
    spawn_y: float
    mean_headway: float | None = None
    flow_policy: str = "shifted_exponential"


@dataclass(frozen=True)
class SeededRandomProfile:
    enabled: bool = True
    seed: int = DEFAULT_P16_SEED
    profile_id: str = "p16_internal_demo_v1"
    arrival_streams: tuple[ArrivalStream, ...] = field(default_factory=tuple)
    cav_penetration_rate: float = 0.60
    chv_compliance_rate: float = 0.75
    safe_spawn_gap_m: float = 20.0
    desired_speed_mean: float = 30.0
    desired_speed_std: float = 1.5
    cav_inertial_lag_min: float = 0.4
    cav_inertial_lag_max: float = 0.7
    max_queue_items_per_lane: int = 256


@dataclass(frozen=True)
class BoundaryQueueItem:
    vehicle_id: str
    lane_id: str
    scheduled_arrival_t: float
    assigned_arrival_headway: float
    vehicle_type: str
    compliance_state: str
    desired_speed: float | None
    desired_time_gap: float | None
    inertial_lag: float | None
    initial_state: VehicleState
    spec: VehicleSpec
    seed: int
    profile_id: str


@dataclass(frozen=True)
class SpawnDecision:
    queue_item: BoundaryQueueItem
    generated: bool
    reason: str
    blocked_reason: str | None = None
    nearest_vehicle_id: str | None = None
    nearest_gap_m: float | None = None


def default_p16_seeded_random_profile(
    *,
    seed: int = DEFAULT_P16_SEED,
    geometry: RoadGeometryConfig = DEFAULT_ROAD_GEOMETRY,
    enabled: bool = True,
) -> SeededRandomProfile:
    return SeededRandomProfile(
        enabled=enabled,
        seed=seed,
        profile_id="p16_internal_demo_v1",
        arrival_streams=(
            ArrivalStream(
                lane_id=LANE_1,
                shifted_headway=1.2,
                initial_speed=30.0,
                spawn_x=geometry.mainline_start_global,
                spawn_y=float(geometry.lane_centerlines[LANE_1]),
                mean_headway=1.8,
            ),
            ArrivalStream(
                lane_id=LANE_2,
                shifted_headway=1.2,
                initial_speed=30.0,
                spawn_x=geometry.mainline_start_global,
                spawn_y=float(geometry.lane_centerlines[LANE_2]),
                mean_headway=1.8,
            ),
            ArrivalStream(
                lane_id=ON_RAMP,
                shifted_headway=3.5,
                initial_speed=16.0,
                spawn_x=geometry.x0_m_global - 100.0,
                spawn_y=float(geometry.lane_centerlines[ON_RAMP]),
                mean_headway=4.5,
            ),
        ),
        cav_penetration_rate=0.60,
        chv_compliance_rate=0.75,
        safe_spawn_gap_m=20.0,
    )


def build_p16_demo_scenario_config(
    *,
    seed: int = DEFAULT_P16_SEED,
    geometry: RoadGeometryConfig = DEFAULT_ROAD_GEOMETRY,
) -> dict[str, Any]:
    return {
        "scenario_id": P16_DEMO_SCENARIO_ID,
        "scenario_name": P16_DEMO_SCENARIO_ID,
        "purpose": "P16 internal seeded random simulation demo.",
        "test_level": "smoke",
        "status": "optional",
        "derivation_ref": ["P16 seeded random internal simulation"],
        "initial_time": {"t": 0.0, "step": 0, "dt": 0.1},
        "initial_vehicles": [],
        "module_overrides": {
            "boundary_generation_enabled": True,
            "random_arrival_enabled": True,
            "random_vehicle_attributes_enabled": True,
            "ordinary_mainline_lane_change_enabled": False,
            "platoon_cmc_enabled": False,
            "mpc_lateral_tracking_enabled": False,
        },
        "preloaded_assignments": [],
        "preloaded_state_machine_states": [],
        "preloaded_maneuver_trajectory_states": [],
        "expected_events": [],
        "forbidden_events": [],
        "expected_event_counts": [],
        "expected_sanity_checks": [],
        "expected_png_features": [
            {
                "feature_type": "seeded_random_flow_marker",
                "required": False,
                "vehicle_ids": [],
                "expected_visibility": "optional",
            }
        ],
    }


def profile_from_mapping(
    value: Mapping[str, Any] | SeededRandomProfile | None,
    *,
    seed: int | None = None,
    enabled: bool | None = None,
    geometry: RoadGeometryConfig = DEFAULT_ROAD_GEOMETRY,
) -> SeededRandomProfile:
    if isinstance(value, SeededRandomProfile):
        profile = value
    elif value is None:
        profile = default_p16_seeded_random_profile(
            seed=DEFAULT_P16_SEED if seed is None else int(seed),
            enabled=True if enabled is None else bool(enabled),
            geometry=geometry,
        )
    else:
        streams = tuple(
            ArrivalStream(
                lane_id=str(item["lane_id"]),
                shifted_headway=float(item["shifted_headway"]),
                initial_speed=float(item["initial_speed"]),
                spawn_x=float(item["spawn_x"]),
                spawn_y=float(item["spawn_y"]),
                mean_headway=(
                    float(item["mean_headway"]) if item.get("mean_headway") is not None else None
                ),
                flow_policy=str(item.get("flow_policy") or "shifted_exponential"),
            )
            for item in value.get("arrival_streams", ())
        )
        profile = SeededRandomProfile(
            enabled=bool(value.get("enabled", True) if enabled is None else enabled),
            seed=int(value.get("seed", DEFAULT_P16_SEED) if seed is None else seed),
            profile_id=str(value.get("profile_id") or "p16_internal_demo_v1"),
            arrival_streams=streams,
            cav_penetration_rate=float(value.get("cav_penetration_rate", 0.60)),
            chv_compliance_rate=float(value.get("chv_compliance_rate", 0.75)),
            safe_spawn_gap_m=float(value.get("safe_spawn_gap_m", 20.0)),
            desired_speed_mean=float(value.get("desired_speed_mean", 30.0)),
            desired_speed_std=float(value.get("desired_speed_std", 1.5)),
            cav_inertial_lag_min=float(value.get("cav_inertial_lag_min", 0.4)),
            cav_inertial_lag_max=float(value.get("cav_inertial_lag_max", 0.7)),
            max_queue_items_per_lane=int(value.get("max_queue_items_per_lane", 256)),
        )
    validate_profile(profile)
    return profile


def profile_to_dict(profile: SeededRandomProfile) -> dict[str, Any]:
    return {
        "enabled": profile.enabled,
        "seed": profile.seed,
        "profile_id": profile.profile_id,
        "arrival_streams": [
            {
                "lane_id": stream.lane_id,
                "shifted_headway": stream.shifted_headway,
                "initial_speed": stream.initial_speed,
                "spawn_x": stream.spawn_x,
                "spawn_y": stream.spawn_y,
                "mean_headway": stream.mean_headway,
                "flow_policy": stream.flow_policy,
            }
            for stream in profile.arrival_streams
        ],
        "cav_penetration_rate": profile.cav_penetration_rate,
        "chv_compliance_rate": profile.chv_compliance_rate,
        "safe_spawn_gap_m": profile.safe_spawn_gap_m,
        "desired_speed_mean": profile.desired_speed_mean,
        "desired_speed_std": profile.desired_speed_std,
        "cav_inertial_lag_min": profile.cav_inertial_lag_min,
        "cav_inertial_lag_max": profile.cav_inertial_lag_max,
        "max_queue_items_per_lane": profile.max_queue_items_per_lane,
    }


def validate_profile(profile: SeededRandomProfile) -> None:
    if profile.enabled is False:
        return
    if not profile.arrival_streams:
        raise ValueError("SeededRandomProfile requires at least one arrival stream when enabled")
    _validate_probability("cav_penetration_rate", profile.cav_penetration_rate)
    _validate_probability("chv_compliance_rate", profile.chv_compliance_rate)
    if profile.safe_spawn_gap_m < 0.0:
        raise ValueError("safe_spawn_gap_m must be non-negative")
    if profile.desired_speed_std < 0.0:
        raise ValueError("desired_speed_std must be non-negative")
    if profile.cav_inertial_lag_min <= 0.0 or profile.cav_inertial_lag_max <= 0.0:
        raise ValueError("CAV inertial lag bounds must be positive")
    if profile.cav_inertial_lag_min > profile.cav_inertial_lag_max:
        raise ValueError("cav_inertial_lag_min must be <= cav_inertial_lag_max")
    if profile.max_queue_items_per_lane < 1:
        raise ValueError("max_queue_items_per_lane must be positive")
    for stream in profile.arrival_streams:
        _validate_stream(stream)


def generate_boundary_queue(
    profile: SeededRandomProfile,
    *,
    max_t: float,
    start_step: int = 0,
    start_t: float = 0.0,
) -> tuple[BoundaryQueueItem, ...]:
    validate_profile(profile)
    if not profile.enabled:
        return ()
    if max_t < start_t:
        return ()

    rng = random.Random(profile.seed)
    queue: list[BoundaryQueueItem] = []
    for stream in profile.arrival_streams:
        scheduled_t = float(start_t)
        for index in range(profile.max_queue_items_per_lane):
            headway = _sample_headway(stream, rng)
            scheduled_t += headway
            if scheduled_t > max_t + 1e-9:
                break
            item = _build_queue_item(
                profile,
                stream,
                rng,
                lane_index=index,
                scheduled_t=scheduled_t,
                headway=headway,
                start_step=start_step,
                start_t=start_t,
            )
            queue.append(item)
    return tuple(sorted(queue, key=lambda item: (item.scheduled_arrival_t, item.lane_id, item.vehicle_id)))


def compute_spawn_decisions(
    queue: Iterable[BoundaryQueueItem],
    state: SimulationState,
    *,
    safe_spawn_gap_m: float,
    eps: float = 1e-9,
) -> tuple[SpawnDecision, ...]:
    decisions: list[SpawnDecision] = []
    occupied = _spawn_occupancy_by_lane(state)
    for item in queue:
        if item.scheduled_arrival_t > state.t + eps:
            continue
        if item.vehicle_id in state.vehicle_states:
            continue
        nearest_id, nearest_gap = _nearest_spawn_gap(item, occupied)
        if nearest_gap is not None and nearest_gap < safe_spawn_gap_m:
            decisions.append(
                SpawnDecision(
                    queue_item=item,
                    generated=False,
                    reason="blocked_safe_spawn_gap",
                    blocked_reason="safe_spawn_gap_not_met",
                    nearest_vehicle_id=nearest_id,
                    nearest_gap_m=nearest_gap,
                )
            )
            continue
        decisions.append(SpawnDecision(queue_item=item, generated=True, reason="generated_pre_freeze"))
        occupied.setdefault(item.lane_id, []).append(
            (item.vehicle_id, float(item.initial_state.x_global))
        )
    return tuple(decisions)


def queue_fingerprint(queue: Iterable[BoundaryQueueItem]) -> tuple[tuple[Any, ...], ...]:
    return tuple(
        (
            item.vehicle_id,
            item.lane_id,
            round(item.scheduled_arrival_t, 6),
            round(item.assigned_arrival_headway, 6),
            item.vehicle_type,
            item.compliance_state,
            None if item.desired_speed is None else round(item.desired_speed, 6),
            None if item.inertial_lag is None else round(item.inertial_lag, 6),
            round(item.initial_state.x_global, 6),
            round(item.initial_state.y, 6),
            round(item.initial_state.v, 6),
        )
        for item in queue
    )


def _validate_stream(stream: ArrivalStream) -> None:
    if stream.lane_id not in ALLOWED_RANDOM_LANES:
        raise ValueError(f"unsupported lane_id for seeded random generation: {stream.lane_id}")
    if stream.shifted_headway <= 0.0:
        raise ValueError("shifted headway must be positive")
    if stream.initial_speed <= 0.0:
        raise ValueError("initial speed must be positive")
    if stream.mean_headway is not None and stream.mean_headway <= stream.shifted_headway:
        raise ValueError("mean_headway must be greater than shifted headway")
    if stream.flow_policy != "shifted_exponential":
        raise ValueError(f"unsupported flow/headway policy: {stream.flow_policy}")


def _validate_probability(name: str, value: float) -> None:
    if value < 0.0 or value > 1.0:
        raise ValueError(f"{name} must be in [0, 1]")


def _sample_headway(stream: ArrivalStream, rng: random.Random) -> float:
    mean_headway = stream.mean_headway
    if mean_headway is None:
        mean_headway = stream.shifted_headway + max(0.1, stream.shifted_headway * 0.5)
    return stream.shifted_headway + rng.expovariate(1.0 / (mean_headway - stream.shifted_headway))


def _build_queue_item(
    profile: SeededRandomProfile,
    stream: ArrivalStream,
    rng: random.Random,
    *,
    lane_index: int,
    scheduled_t: float,
    headway: float,
    start_step: int,
    start_t: float,
) -> BoundaryQueueItem:
    vehicle_id = f"p16_{profile.seed}_{stream.lane_id}_{lane_index:04d}"
    is_cav = rng.random() < profile.cav_penetration_rate
    vehicle_type = "cav" if is_cav else "chv"
    compliance = "not_applicable"
    desired_speed = None
    desired_time_gap = 1.2 if is_cav else 2.0
    inertial_lag = None
    if is_cav:
        inertial_lag = rng.uniform(profile.cav_inertial_lag_min, profile.cav_inertial_lag_max)
    else:
        compliance = "compliant" if rng.random() < profile.chv_compliance_rate else "non_compliant"
        desired_speed = max(0.1, rng.gauss(profile.desired_speed_mean, profile.desired_speed_std))

    road_role = ON_RAMP_ROLE if stream.lane_id == ON_RAMP else MAINLINE
    merge_state = "not_started" if stream.lane_id == ON_RAMP else "none"
    state = VehicleState(
        vehicle_id=vehicle_id,
        x_global=float(stream.spawn_x),
        y=float(stream.spawn_y),
        v=float(stream.initial_speed),
        a=0.0,
        physical_lane=stream.lane_id,
        road_role=road_role,
        lane_change_state="normal",
        merge_state=merge_state,
    )
    spec = VehicleSpec(
        vehicle_id=vehicle_id,
        vehicle_type=vehicle_type,
        compliance_state=compliance,
        desired_speed=desired_speed,
        desired_time_gap=desired_time_gap,
        assigned_arrival_headway=headway,
        inertial_lag=inertial_lag,
        source_lane_at_generation=stream.lane_id,
        generation_step=start_step,
        generation_t=start_t + scheduled_t,
    )
    return BoundaryQueueItem(
        vehicle_id=vehicle_id,
        lane_id=stream.lane_id,
        scheduled_arrival_t=scheduled_t,
        assigned_arrival_headway=headway,
        vehicle_type=vehicle_type,
        compliance_state=compliance,
        desired_speed=desired_speed,
        desired_time_gap=desired_time_gap,
        inertial_lag=inertial_lag,
        initial_state=state,
        spec=spec,
        seed=profile.seed,
        profile_id=profile.profile_id,
    )


def _spawn_occupancy_by_lane(state: SimulationState) -> dict[str, list[tuple[str, float]]]:
    occupied: dict[str, list[tuple[str, float]]] = {}
    for vehicle_id in state.active_vehicle_ids:
        vehicle_state = state.vehicle_states[vehicle_id]
        if not vehicle_state.is_active:
            continue
        occupied.setdefault(vehicle_state.physical_lane, []).append(
            (vehicle_id, float(vehicle_state.x_global))
        )
    return occupied


def _nearest_spawn_gap(
    item: BoundaryQueueItem,
    occupied: Mapping[str, list[tuple[str, float]]],
) -> tuple[str | None, float | None]:
    nearest_id: str | None = None
    nearest_gap: float | None = None
    spawn_x = item.initial_state.x_global
    for vehicle_id, x_global in occupied.get(item.lane_id, ()):
        gap = abs(float(x_global) - float(spawn_x))
        if nearest_gap is None or gap < nearest_gap:
            nearest_id = vehicle_id
            nearest_gap = gap
    return nearest_id, nearest_gap
