from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Iterable

from cormc.sumo.commands import ControlledTrajectoryCommand, RealizationRecord, SumoTrajectoryAuthorityConfig
from cormc.sumo.executor import RealizedVehicleState


MATCHED = "matched"
MISMATCH = "mismatch"
MISSING = "missing"
COLLIDED = "collided"
TELEPORTED = "teleported"


class RealizationMonitor:
    def __init__(self, config: SumoTrajectoryAuthorityConfig | None = None) -> None:
        self.config = config or SumoTrajectoryAuthorityConfig()

    def classify_realization(
        self,
        command: ControlledTrajectoryCommand,
        realized: RealizedVehicleState,
        *,
        collision_vehicle_ids: Iterable[str] = (),
        teleported_vehicle_ids: Iterable[str] = (),
    ) -> RealizationRecord:
        collision_ids = set(collision_vehicle_ids)
        teleport_ids = set(teleported_vehicle_ids)

        if command.vehicle_id in collision_ids:
            result = COLLIDED
        elif command.vehicle_id in teleport_ids:
            result = TELEPORTED
        elif realized.x_global is None or realized.y is None or realized.v is None:
            result = MISSING
        else:
            result = self._matched_or_mismatch(command, realized)

        dx_abs = None if realized.x_global is None else abs(realized.x_global - command.x_global)
        dy_abs = None if realized.y is None else abs(realized.y - command.y)
        dv_abs = None if realized.v is None else abs(realized.v - command.v)

        return RealizationRecord(
            vehicle_id=command.vehicle_id,
            step=command.step,
            t=command.t,
            command_x_global=command.x_global,
            command_y=command.y,
            command_v=command.v,
            command_a=command.a,
            realized_x_global=realized.x_global,
            realized_y=realized.y,
            realized_v=realized.v,
            realized_edge_id=realized.edge_id,
            realized_lane_id=realized.lane_id,
            realized_lane_position=realized.lane_position,
            dx_abs=dx_abs,
            dy_abs=dy_abs,
            dv_abs=dv_abs,
            result=result,
        )

    def _matched_or_mismatch(self, command: ControlledTrajectoryCommand, realized: RealizedVehicleState) -> str:
        assert realized.x_global is not None
        assert realized.y is not None
        assert realized.v is not None
        if abs(realized.x_global - command.x_global) > self.config.mismatch_x_tolerance_m:
            return MISMATCH
        if abs(realized.y - command.y) > self.config.mismatch_y_tolerance_m:
            return MISMATCH
        if abs(realized.v - command.v) > self.config.mismatch_v_tolerance_mps:
            return MISMATCH
        return MATCHED


def classify_collision(vehicle_ids: Iterable[str], active_vehicle_ids: Iterable[str]) -> dict[str, Any]:
    vehicles = tuple(vehicle_ids)
    active = tuple(vehicle_id for vehicle_id in vehicles if vehicle_id in set(active_vehicle_ids))
    background = tuple(vehicle_id for vehicle_id in vehicles if vehicle_id not in set(active_vehicle_ids))
    if len(active) == len(vehicles) and active:
        collision_type = "active_vs_active_collision"
    elif active:
        collision_type = "active_vs_background_collision"
    else:
        collision_type = "background_vs_background_collision"
    return {
        "event_type": "collision",
        "collision_type": collision_type,
        "vehicle_ids": vehicles,
        "active_vehicle_ids": active,
        "background_vehicle_ids": background,
    }


def collect_collision_events(traci_module: Any, *, step: int, t: float, active_vehicle_ids: Iterable[str]) -> list[dict[str, Any]]:
    events: list[dict[str, Any]] = []
    for collision in traci_module.simulation.getCollisions():
        collider = collision.collider
        victim = collision.victim
        event = classify_collision((collider, victim), active_vehicle_ids)
        event.update(
            {
                "step": step,
                "t": t,
                "collider": collider,
                "victim": victim,
            }
        )
        events.append(event)
    return events


def collect_teleport_events(traci_module: Any, *, step: int, t: float, active_vehicle_ids: Iterable[str]) -> list[dict[str, Any]]:
    active = set(active_vehicle_ids)
    events: list[dict[str, Any]] = []
    for vehicle_id in traci_module.simulation.getEndingTeleportIDList():
        events.append(
            {
                "event_type": "teleport",
                "teleport_type": "active_vehicle_teleported" if vehicle_id in active else "background_vehicle_teleported",
                "vehicle_id": vehicle_id,
                "step": step,
                "t": t,
            }
        )
    return events


def write_realization_jsonl(path: str | Path, records: Iterable[RealizationRecord]) -> None:
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", encoding="utf-8") as handle:
        for record in records:
            handle.write(json.dumps(record.to_dict(), ensure_ascii=False, sort_keys=True) + "\n")


def read_realization_jsonl(path: str | Path) -> list[RealizationRecord]:
    records: list[RealizationRecord] = []
    with Path(path).open("r", encoding="utf-8") as handle:
        for line in handle:
            if not line.strip():
                continue
            records.append(RealizationRecord(**json.loads(line)))
    return records


def write_events_jsonl(path: str | Path, events: Iterable[dict[str, Any]]) -> None:
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", encoding="utf-8") as handle:
        for event in events:
            handle.write(json.dumps(event, ensure_ascii=False, sort_keys=True) + "\n")
