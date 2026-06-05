from __future__ import annotations

from copy import deepcopy
from dataclasses import asdict, dataclass, is_dataclass
from types import MappingProxyType, ModuleType
from typing import Any, Iterable, Mapping

from cormc.step0_3 import (
    LongitudinalControllerMemory,
    ManeuverTrajectoryState,
    SimulationState,
    VehicleSpec,
    VehicleState,
)
from cormc.sumo.env import import_traci
from cormc.sumo.mapping import LANE_ROLE_MAP, from_sumo_position


@dataclass(frozen=True)
class SumoRealizedAdapterMetadata:
    step: int
    t: float
    dt: float
    vehicle_count: int
    source: str

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


class SumoRealizedStateAdapter:
    """Convert realized SUMO vehicles into the frozen CORMC state contract."""

    def __init__(self, *, traci_module: ModuleType | Any | None = None) -> None:
        self.traci = traci_module

    def adapt(
        self,
        realized: Mapping[str, Any] | Iterable[Any] | ModuleType | Any | None,
        *,
        previous_state: SimulationState | None = None,
        step: int | None = None,
        t: float | None = None,
        dt: float | None = None,
        traci_module: ModuleType | Any | None = None,
    ) -> SimulationState:
        traci = traci_module or self.traci
        source = "realized_snapshot"
        if realized is None or _looks_like_traci(realized):
            traci = realized if realized is not None else traci
            if traci is None:
                traci = import_traci()
            snapshot = _snapshot_from_traci(traci)
            source = "traci"
        else:
            snapshot = _snapshot_from_realized(realized, traci)

        resolved_step = int(step if step is not None else (previous_state.step if previous_state is not None else 0))
        resolved_t = float(t if t is not None else (previous_state.t if previous_state is not None else 0.0))
        resolved_dt = float(dt if dt is not None else (previous_state.dt if previous_state is not None else 0.1))

        previous_vehicle_states = dict(previous_state.vehicle_states) if previous_state is not None else {}
        previous_specs = dict(previous_state.vehicle_specs) if previous_state is not None else {}
        previous_maneuvers = dict(previous_state.active_maneuvers) if previous_state is not None else {}
        previous_controller_memory = (
            dict(previous_state.controller_memory_by_vehicle) if previous_state is not None else {}
        )

        vehicle_states: dict[str, VehicleState] = {}
        vehicle_specs: dict[str, VehicleSpec] = {}
        active_ids: list[str] = []
        for item in snapshot:
            vehicle_id = str(item["vehicle_id"])
            previous = previous_vehicle_states.get(vehicle_id)
            lane_id = _optional_str(item.get("lane_id"))
            edge_id = _optional_str(item.get("edge_id"))
            lane_position = _optional_float(item.get("lane_position"))
            x_global = _resolve_x_global(item, edge_id, lane_position)
            physical_lane = _resolve_physical_lane(edge_id, lane_id, previous)
            road_role = _resolve_road_role(edge_id, physical_lane, previous)
            y = _resolve_float(item.get("y"), previous.y if previous is not None else _lane_y(edge_id, physical_lane), 0.0)
            v = _resolve_float(item.get("v"), previous.v if previous is not None else None, 0.0)
            a = _resolve_float(item.get("a"), previous.a if previous is not None else None, 0.0)
            lane_change_state = _optional_str(item.get("lane_change_state")) or (
                previous.lane_change_state if previous is not None else "normal"
            )
            merge_state = _optional_str(item.get("merge_state")) or (
                previous.merge_state if previous is not None else _default_merge_state(road_role)
            )
            is_active = bool(item.get("is_active", True))
            vehicle_states[vehicle_id] = VehicleState(
                vehicle_id=vehicle_id,
                x_global=x_global,
                y=y,
                v=v,
                a=a,
                physical_lane=physical_lane,
                road_role=road_role,
                lane_change_state=lane_change_state,
                merge_state=merge_state,
                is_active=is_active,
            )
            vehicle_specs[vehicle_id] = _spec_for_vehicle(
                vehicle_id,
                item,
                previous_specs.get(vehicle_id),
                physical_lane=physical_lane,
                step=resolved_step,
                t=resolved_t,
            )
            active_ids.append(vehicle_id)

        active_set = set(active_ids)
        assignment_records = _freeze_nested_mapping(
            {
                vehicle_id: value
                for vehicle_id, value in (previous_state.assignment_records_by_mv.items() if previous_state is not None else ())
                if vehicle_id in active_set
            }
        )
        active_maneuvers = MappingProxyType(
            {
                vehicle_id: maneuver
                for vehicle_id, maneuver in previous_maneuvers.items()
                if vehicle_id in active_set
            }
        )
        controller_memory = MappingProxyType(
            {
                vehicle_id: memory
                for vehicle_id, memory in previous_controller_memory.items()
                if vehicle_id in active_set
            }
        )

        return SimulationState(
            t=resolved_t,
            step=resolved_step,
            dt=resolved_dt,
            active_vehicle_ids=tuple(active_ids),
            vehicle_states=MappingProxyType(vehicle_states),
            vehicle_specs=MappingProxyType(vehicle_specs),
            assignment_records_by_mv=assignment_records,
            active_maneuvers=active_maneuvers,
            road_config_ref=previous_state.road_config_ref if previous_state is not None else "paper_fig10_first_version",
            parameter_config_ref=previous_state.parameter_config_ref if previous_state is not None else "paper_table_i_first_version",
            scenario_config_ref=previous_state.scenario_config_ref if previous_state is not None else None,
            output_config_ref=previous_state.output_config_ref if previous_state is not None else None,
            controller_memory_by_vehicle=controller_memory,
        )

    def metadata(self, state: SimulationState, *, source: str = "realized_snapshot") -> SumoRealizedAdapterMetadata:
        return SumoRealizedAdapterMetadata(
            step=state.step,
            t=state.t,
            dt=state.dt,
            vehicle_count=len(state.active_vehicle_ids),
            source=source,
        )


def adapt_sumo_realized_to_state(
    realized: Mapping[str, Any] | Iterable[Any] | ModuleType | Any | None,
    *,
    previous_state: SimulationState | None = None,
    step: int | None = None,
    t: float | None = None,
    dt: float | None = None,
    traci_module: ModuleType | Any | None = None,
) -> SimulationState:
    return SumoRealizedStateAdapter(traci_module=traci_module).adapt(
        realized,
        previous_state=previous_state,
        step=step,
        t=t,
        dt=dt,
        traci_module=traci_module,
    )


def _snapshot_from_traci(traci_module: Any) -> list[dict[str, Any]]:
    snapshot: list[dict[str, Any]] = []
    for vehicle_id in traci_module.vehicle.getIDList():
        x_sumo, y_sumo = traci_module.vehicle.getPosition(vehicle_id)
        snapshot.append(
            {
                "vehicle_id": vehicle_id,
                "x_global": x_sumo,
                "y": y_sumo,
                "v": traci_module.vehicle.getSpeed(vehicle_id),
                "edge_id": traci_module.vehicle.getRoadID(vehicle_id),
                "lane_id": traci_module.vehicle.getLaneID(vehicle_id),
                "lane_position": traci_module.vehicle.getLanePosition(vehicle_id),
            }
        )
    return snapshot


def _snapshot_from_realized(realized: Mapping[str, Any] | Iterable[Any], traci_module: Any | None) -> list[dict[str, Any]]:
    if isinstance(realized, Mapping):
        if "vehicle_id" in realized or "id" in realized:
            values = [realized]
        else:
            values = []
            for vehicle_id, value in realized.items():
                item = _plain_mapping(value)
                item.setdefault("vehicle_id", vehicle_id)
                values.append(item)
    else:
        values = [_plain_mapping(value) for value in realized]

    snapshot: list[dict[str, Any]] = []
    for value in values:
        item = dict(value)
        vehicle_id = str(item.get("vehicle_id") or item.get("id"))
        item["vehicle_id"] = vehicle_id
        if traci_module is not None and "y" not in item and vehicle_id in set(traci_module.vehicle.getIDList()):
            _, y_sumo = traci_module.vehicle.getPosition(vehicle_id)
            item["y"] = y_sumo
        snapshot.append(item)
    return snapshot


def _plain_mapping(value: Any) -> dict[str, Any]:
    if isinstance(value, Mapping):
        return dict(value)
    if is_dataclass(value):
        return asdict(value)
    result: dict[str, Any] = {}
    for name in (
        "vehicle_id",
        "id",
        "x_global",
        "x",
        "y",
        "v",
        "speed",
        "a",
        "edge_id",
        "edgeID",
        "lane_id",
        "laneID",
        "lane_position",
        "lanePosition",
    ):
        if hasattr(value, name):
            result[name] = getattr(value, name)
    return result


def _looks_like_traci(value: Any) -> bool:
    return hasattr(value, "vehicle") and hasattr(value.vehicle, "getIDList")


def _resolve_x_global(item: Mapping[str, Any], edge_id: str | None, lane_position: float | None) -> float:
    if edge_id is not None and lane_position is not None:
        try:
            return from_sumo_position(edge_id, lane_position)
        except ValueError:
            pass
    value = item.get("x_global", item.get("x"))
    if value is None:
        raise ValueError(f"realized vehicle {item.get('vehicle_id')!r} is missing x_global/x and edge lane position")
    return float(value)


def _resolve_physical_lane(edge_id: str | None, lane_id: str | None, previous: VehicleState | None) -> str:
    if edge_id is None and lane_id is not None:
        edge_id = lane_id.rsplit("_", 1)[0]
    lane_index = _lane_index_from_lane_id(lane_id)
    if edge_id is not None and lane_index is not None:
        for lane in LANE_ROLE_MAP.get(edge_id, ()):
            if lane.lane_index == lane_index:
                return lane.role
    return previous.physical_lane if previous is not None else "lane_2"


def _resolve_road_role(edge_id: str | None, physical_lane: str, previous: VehicleState | None) -> str:
    if previous is not None and previous.road_role == "on_ramp_mv":
        return "on_ramp_mv"
    if edge_id in {"ramp_pre"} or physical_lane == "on_ramp":
        return "on_ramp"
    return previous.road_role if previous is not None and previous.road_role == "on_ramp_mv" else "mainline"


def _spec_for_vehicle(
    vehicle_id: str,
    item: Mapping[str, Any],
    previous: VehicleSpec | None,
    *,
    physical_lane: str,
    step: int,
    t: float,
) -> VehicleSpec:
    if previous is not None:
        return previous
    vehicle_type = str(item.get("vehicle_type") or item.get("type") or item.get("type_id") or "sumo_background").lower()
    compliance_state = str(item.get("compliance_state") or "not_applicable").lower()
    if compliance_state == "none":
        compliance_state = "not_applicable"
    return VehicleSpec(
        vehicle_id=vehicle_id,
        vehicle_type=vehicle_type,
        compliance_state=compliance_state,
        desired_speed=_optional_float(item.get("desired_speed")),
        desired_time_gap=_optional_float(item.get("desired_time_gap")),
        desired_time_gap_class=_optional_str(item.get("desired_time_gap_class")),
        assigned_arrival_headway=_optional_float(item.get("assigned_arrival_headway")),
        inertial_lag=_optional_float(item.get("inertial_lag")),
        length=float(item.get("length", 5.0)),
        source_lane_at_generation=physical_lane,
        generation_step=step,
        generation_t=t,
    )


def _lane_index_from_lane_id(lane_id: str | None) -> int | None:
    if not lane_id:
        return None
    try:
        return int(lane_id.rsplit("_", 1)[1])
    except (IndexError, ValueError):
        return None


def _lane_y(edge_id: str | None, physical_lane: str) -> float | None:
    if edge_id is None:
        return None
    for lane in LANE_ROLE_MAP.get(edge_id, ()):
        if lane.role == physical_lane:
            return lane.y
    return None


def _default_merge_state(road_role: str) -> str:
    return "not_started" if road_role in {"on_ramp", "on_ramp_mv"} else "none"


def _optional_str(value: Any) -> str | None:
    if value in (None, ""):
        return None
    return str(value)


def _optional_float(value: Any) -> float | None:
    if value in (None, ""):
        return None
    return float(value)


def _resolve_float(value: Any, fallback: float | None, default: float) -> float:
    if value in (None, ""):
        return default if fallback is None else float(fallback)
    return float(value)


def _freeze_nested_mapping(source: Mapping[str, Any]) -> Mapping[str, Mapping[str, Any]]:
    return MappingProxyType({key: MappingProxyType(deepcopy(dict(value))) for key, value in source.items()})
