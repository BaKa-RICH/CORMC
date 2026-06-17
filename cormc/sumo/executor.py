from __future__ import annotations

import math
from dataclasses import asdict, dataclass
from types import ModuleType
from typing import Any

from cormc.sumo.commands import ControlledTrajectoryCommand, SumoTrajectoryAuthorityConfig
from cormc.sumo.env import import_traci
from cormc.sumo.mapping import from_sumo_position, to_sumo_position, to_sumo_xy


EXECUTOR_MODE = "move_to_xy_trajectory_authority"
DEFAULT_SPEED_MODE_BITSET = 0
DEFAULT_LANE_CHANGE_MODE_BITSET = 2560
DEFAULT_MOVE_TO_XY_KEEP_ROUTE = 3


@dataclass(frozen=True)
class RealizedVehicleState:
    vehicle_id: str
    x_global: float | None
    y: float | None
    v: float | None
    edge_id: str | None
    lane_id: str | None
    lane_position: float | None

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class MoveToXYExecutorMetadata:
    executor_mode: str
    speed_mode_bitset: int
    lane_change_mode_bitset: int
    move_to_xy_keep_route: int
    lateral_resolution: float
    collision_action: str

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


class MoveToXYTrajectoryExecutor:
    """Apply neutral CORMC trajectory commands through TraCI moveToXY."""

    def __init__(
        self,
        config: SumoTrajectoryAuthorityConfig | None = None,
        *,
        traci_module: ModuleType | Any | None = None,
        speed_mode_bitset: int = DEFAULT_SPEED_MODE_BITSET,
        lane_change_mode_bitset: int = DEFAULT_LANE_CHANGE_MODE_BITSET,
        move_to_xy_keep_route: int = DEFAULT_MOVE_TO_XY_KEEP_ROUTE,
        default_sumo_angle: float = 90.0,
    ) -> None:
        self.config = config or SumoTrajectoryAuthorityConfig(executor_mode=EXECUTOR_MODE)
        self.traci = traci_module if traci_module is not None else import_traci()
        self.speed_mode_bitset = speed_mode_bitset
        self.lane_change_mode_bitset = lane_change_mode_bitset
        self.move_to_xy_keep_route = move_to_xy_keep_route
        self.default_sumo_angle = _normalize_sumo_angle(default_sumo_angle)
        self._last_command_xy: dict[str, tuple[float, float]] = {}
        self._last_heading: dict[str, float] = {}
        self._authority_configured: set[str] = set()

    @property
    def metadata(self) -> MoveToXYExecutorMetadata:
        return MoveToXYExecutorMetadata(
            executor_mode=EXECUTOR_MODE,
            speed_mode_bitset=self.speed_mode_bitset,
            lane_change_mode_bitset=self.lane_change_mode_bitset,
            move_to_xy_keep_route=self.move_to_xy_keep_route,
            lateral_resolution=self.config.lateral_resolution,
            collision_action=self.config.collision_action,
        )

    def configure_vehicle_authority(self, vehicle_id: str) -> None:
        if vehicle_id in self._authority_configured:
            return
        self.traci.vehicle.setSpeedMode(vehicle_id, self.speed_mode_bitset)
        self.traci.vehicle.setLaneChangeMode(vehicle_id, self.lane_change_mode_bitset)
        self._authority_configured.add(vehicle_id)

    def apply_command(
        self,
        command: ControlledTrajectoryCommand,
        *,
        realized_speed_at_t: float | None = None,
    ) -> float:
        self.configure_vehicle_authority(command.vehicle_id)

        edge_hint, lane_hint, _ = to_sumo_position(command.x_global, command.physical_lane, command.road_role)
        x_cmd, y_cmd = to_sumo_xy(command.x_global, command.y, command.road_role)
        angle = self.derive_sumo_angle(command)
        previous_speed = command.v if realized_speed_at_t is None else realized_speed_at_t

        self.traci.vehicle.setPreviousSpeed(command.vehicle_id, previous_speed)
        self.traci.vehicle.setSpeed(command.vehicle_id, command.v)
        self.traci.vehicle.moveToXY(
            command.vehicle_id,
            edge_hint,
            lane_hint,
            x_cmd,
            y_cmd,
            angle,
            keepRoute=self.move_to_xy_keep_route,
        )

        self._last_command_xy[command.vehicle_id] = (x_cmd, y_cmd)
        self._last_heading[command.vehicle_id] = angle
        return angle

    def derive_sumo_angle(self, command: ControlledTrajectoryCommand) -> float:
        if command.yaw is not None:
            return _normalize_sumo_angle(90.0 - math.degrees(command.yaw))

        previous_xy = self._last_command_xy.get(command.vehicle_id)
        if previous_xy is None:
            return self._last_heading.get(command.vehicle_id, self.default_sumo_angle)

        dx = command.x_global - previous_xy[0]
        dy = command.y - previous_xy[1]
        if math.hypot(dx, dy) <= 1e-9:
            return self._last_heading.get(command.vehicle_id, self.default_sumo_angle)
        return _normalize_sumo_angle(90.0 - math.degrees(math.atan2(dy, dx)))

    def read_realized_vehicle(self, vehicle_id: str) -> RealizedVehicleState:
        return read_realized_vehicle(self.traci, vehicle_id)


def read_realized_vehicle(traci_module: ModuleType | Any, vehicle_id: str) -> RealizedVehicleState:
    if vehicle_id not in set(traci_module.vehicle.getIDList()):
        return RealizedVehicleState(vehicle_id, None, None, None, None, None, None)

    x_sumo, y_sumo = traci_module.vehicle.getPosition(vehicle_id)
    edge_id = traci_module.vehicle.getRoadID(vehicle_id)
    lane_id = traci_module.vehicle.getLaneID(vehicle_id)
    lane_position = traci_module.vehicle.getLanePosition(vehicle_id)
    speed = traci_module.vehicle.getSpeed(vehicle_id)

    try:
        x_from_lane = from_sumo_position(edge_id, lane_position)
    except ValueError:
        x_global = x_sumo
    else:
        x_global = x_from_lane if abs(x_from_lane - x_sumo) <= 1.0 else x_sumo

    return RealizedVehicleState(
        vehicle_id=vehicle_id,
        x_global=x_global,
        y=y_sumo,
        v=speed,
        edge_id=edge_id,
        lane_id=lane_id,
        lane_position=lane_position,
    )


def _normalize_sumo_angle(angle: float) -> float:
    normalized = angle % 360.0
    if math.isclose(normalized, 360.0):
        return 0.0
    return normalized
