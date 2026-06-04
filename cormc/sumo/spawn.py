from __future__ import annotations

from dataclasses import asdict, dataclass, field
from types import ModuleType
from typing import Any, Iterable

from cormc.step0_3 import ON_RAMP
from cormc.sumo.commands import SpawnCommand
from cormc.sumo.env import import_traci
from cormc.sumo.mapping import to_sumo_position, to_sumo_xy


ACTIVE_TYPES = {"cav", "cv", "clv", "cfv", "mv"}


@dataclass(frozen=True)
class SpawnRealization:
    vehicle_id: str
    result: str
    reason: str
    command: SpawnCommand | None = None
    vehicle_type: str | None = None
    compliance_state: str | None = None
    error: str | None = None

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class SpawnRegistry:
    inserted_vehicle_ids: set[str] = field(default_factory=set)
    blocked_vehicle_ids: set[str] = field(default_factory=set)
    failures: dict[str, str] = field(default_factory=dict)

    def has_inserted(self, vehicle_id: str) -> bool:
        return vehicle_id in self.inserted_vehicle_ids


class SumoSpawnAdapter:
    def __init__(
        self,
        *,
        traci_module: ModuleType | Any | None = None,
        registry: SpawnRegistry | None = None,
    ) -> None:
        self.traci = traci_module
        self.registry = registry or SpawnRegistry()

    def realize_decisions(self, decisions: Iterable[Any], *, traci_module: ModuleType | Any | None = None) -> tuple[SpawnRealization, ...]:
        return tuple(self.realize_decision(decision, traci_module=traci_module) for decision in decisions)

    def realize_decision(self, decision: Any, *, traci_module: ModuleType | Any | None = None) -> SpawnRealization:
        command = spawn_command_from_decision(decision)
        vehicle_id = command.vehicle_id
        if not bool(getattr(decision, "generated", False)):
            self.registry.blocked_vehicle_ids.add(vehicle_id)
            return SpawnRealization(
                vehicle_id=vehicle_id,
                result="blocked",
                reason=str(getattr(decision, "blocked_reason", None) or getattr(decision, "reason", "blocked")),
                command=command,
                vehicle_type=command.vehicle_type,
                compliance_state=command.compliance_state,
            )
        if self.registry.has_inserted(vehicle_id):
            return SpawnRealization(
                vehicle_id=vehicle_id,
                result="duplicate",
                reason="already_inserted",
                command=command,
                vehicle_type=command.vehicle_type,
                compliance_state=command.compliance_state,
            )

        traci = traci_module or self.traci or import_traci()
        try:
            traci.vehicle.add(
                vehicle_id,
                command.route_id,
                typeID=command.vehicle_type,
                depart="now",
                departLane=str(command.lane_index),
                departPos=f"{command.depart_pos:.6f}",
                departSpeed=f"{command.v:.6f}",
            )
            x_sumo, y_sumo = to_sumo_xy(command.x_global, command.y, _road_role_from_route(command.route_id))
            traci.vehicle.moveToXY(
                vehicle_id,
                command.edge_id,
                command.lane_index,
                x_sumo,
                y_sumo,
                90.0,
                keepRoute=3,
            )
        except Exception as exc:  # TraCI exceptions vary by SUMO version.
            self.registry.failures[vehicle_id] = str(exc)
            return SpawnRealization(
                vehicle_id=vehicle_id,
                result="integration_failure",
                reason="traci_add_or_move_failed",
                command=command,
                vehicle_type=command.vehicle_type,
                compliance_state=command.compliance_state,
                error=str(exc),
            )

        self.registry.inserted_vehicle_ids.add(vehicle_id)
        return SpawnRealization(
            vehicle_id=vehicle_id,
            result="generated",
            reason=str(getattr(decision, "reason", "generated_pre_freeze")),
            command=command,
            vehicle_type=command.vehicle_type,
            compliance_state=command.compliance_state,
        )


def spawn_command_from_decision(decision: Any) -> SpawnCommand:
    item = getattr(decision, "queue_item")
    state = getattr(item, "initial_state")
    spec = getattr(item, "spec")
    lane_id = str(getattr(item, "lane_id", getattr(state, "physical_lane")))
    road_role = str(getattr(state, "road_role", "on_ramp" if lane_id == ON_RAMP else "mainline"))
    edge_id, lane_index, depart_pos = to_sumo_position(float(state.x_global), lane_id, road_role)
    vehicle_type = _sumo_vehicle_type(str(getattr(spec, "vehicle_type", getattr(item, "vehicle_type", "sumo_background"))))
    return SpawnCommand(
        vehicle_id=str(getattr(item, "vehicle_id")),
        step=int(round(float(getattr(item, "scheduled_arrival_t", 0.0)) / 0.1)),
        t=float(getattr(item, "scheduled_arrival_t", 0.0)),
        route_id="route_ramp" if lane_id == ON_RAMP else "route_main",
        edge_id=edge_id,
        lane_index=lane_index,
        depart_pos=depart_pos,
        x_global=float(state.x_global),
        y=float(state.y),
        v=float(state.v),
        vehicle_type=vehicle_type,
        compliance_state=str(getattr(spec, "compliance_state", getattr(item, "compliance_state", "not_applicable"))),
        source_queue_seed=int(getattr(item, "seed", 0)),
        source_profile_id=str(getattr(item, "profile_id", "")),
        source_spawn_reason=str(getattr(decision, "reason", "")),
    )


def realize_spawn_decisions(
    decisions: Iterable[Any],
    *,
    traci_module: ModuleType | Any | None = None,
    registry: SpawnRegistry | None = None,
) -> tuple[SpawnRealization, ...]:
    return SumoSpawnAdapter(traci_module=traci_module, registry=registry).realize_decisions(decisions)


def _sumo_vehicle_type(vehicle_type: str) -> str:
    return "cormc_active" if vehicle_type.lower() in ACTIVE_TYPES else "sumo_background"


def _road_role_from_route(route_id: str) -> str:
    return "on_ramp" if route_id == "route_ramp" else "mainline"
