from __future__ import annotations

from pathlib import Path

import pytest

from cormc.sumo.commands import ControlledTrajectoryCommand, SumoTrajectoryAuthorityConfig
from cormc.sumo.env import ensure_sumo_available_or_skip, import_traci
from cormc.sumo.executor import (
    DEFAULT_LANE_CHANGE_MODE_BITSET,
    DEFAULT_MOVE_TO_XY_KEEP_ROUTE,
    DEFAULT_SPEED_MODE_BITSET,
    EXECUTOR_MODE,
    MoveToXYTrajectoryExecutor,
)
from cormc.sumo.monitoring import RealizationMonitor
from cormc.sumo.network import SumoNetworkConfig, build_sumo_network


class _FakeVehicleApi:
    def __init__(self) -> None:
        self.calls: list[tuple[str, tuple[object, ...], dict[str, object]]] = []

    def setSpeedMode(self, *args: object, **kwargs: object) -> None:
        self.calls.append(("setSpeedMode", args, kwargs))

    def setLaneChangeMode(self, *args: object, **kwargs: object) -> None:
        self.calls.append(("setLaneChangeMode", args, kwargs))

    def setPreviousSpeed(self, *args: object, **kwargs: object) -> None:
        self.calls.append(("setPreviousSpeed", args, kwargs))

    def setSpeed(self, *args: object, **kwargs: object) -> None:
        self.calls.append(("setSpeed", args, kwargs))

    def moveToXY(self, *args: object, **kwargs: object) -> None:
        self.calls.append(("moveToXY", args, kwargs))


class _FakeTraci:
    def __init__(self) -> None:
        self.vehicle = _FakeVehicleApi()


def _command(step: int, x: float, y: float = 0.0, v: float = 30.0) -> ControlledTrajectoryCommand:
    return ControlledTrajectoryCommand(
        vehicle_id="MAIN_ACTIVE",
        step=step,
        t=step * 0.1,
        target_t=(step + 1) * 0.1,
        x_global=x,
        y=y,
        v=v,
        a=0.0,
        physical_lane="lane_2",
        road_role="mainline",
    )


def test_move_to_xy_executor_records_metadata_and_call_order() -> None:
    fake = _FakeTraci()
    executor = MoveToXYTrajectoryExecutor(traci_module=fake)

    angle0 = executor.apply_command(_command(0, 100.0), realized_speed_at_t=28.5)
    angle1 = executor.apply_command(_command(1, 103.0), realized_speed_at_t=30.0)

    assert executor.metadata.executor_mode == EXECUTOR_MODE
    assert executor.metadata.speed_mode_bitset == DEFAULT_SPEED_MODE_BITSET
    assert executor.metadata.lane_change_mode_bitset == DEFAULT_LANE_CHANGE_MODE_BITSET
    assert executor.metadata.move_to_xy_keep_route == DEFAULT_MOVE_TO_XY_KEEP_ROUTE
    assert angle0 == pytest.approx(90.0)
    assert angle1 == pytest.approx(90.0)
    assert [name for name, _, _ in fake.vehicle.calls] == [
        "setSpeedMode",
        "setLaneChangeMode",
        "setPreviousSpeed",
        "setSpeed",
        "moveToXY",
        "setPreviousSpeed",
        "setSpeed",
        "moveToXY",
    ]
    assert fake.vehicle.calls[2][1] == ("MAIN_ACTIVE", 28.5)
    move_call = fake.vehicle.calls[4]
    assert move_call[1][:6] == ("MAIN_ACTIVE", "main_pre", 0, 100.0, 0.0, 90.0)
    assert move_call[2] == {"keepRoute": 3}


def test_move_to_xy_executor_heading_for_lane_change() -> None:
    fake = _FakeTraci()
    executor = MoveToXYTrajectoryExecutor(traci_module=fake)

    executor.apply_command(_command(0, 100.0, 0.0))
    angle = executor.apply_command(_command(1, 103.0, 3.0))

    assert angle == pytest.approx(45.0)


def test_move_to_xy_executor_real_traci_single_vehicle_straight_replay_matched(tmp_path: Path) -> None:
    paths = ensure_sumo_available_or_skip()
    config = SumoTrajectoryAuthorityConfig(step_length=0.1)
    files = build_sumo_network(
        tmp_path,
        SumoNetworkConfig(step_length=config.step_length, end=1.0),
    )
    traci = import_traci()
    traci.start([paths.sumo, "-c", files.sumocfg_file, "--no-step-log", "true", "--no-warnings", "true"])

    try:
        traci.vehicle.add(
            "MAIN_ACTIVE",
            "route_main",
            typeID="cormc_active",
            depart="now",
            departLane="0",
            departPos="100",
            departSpeed="30",
        )
        traci.simulationStep()
        executor = MoveToXYTrajectoryExecutor(config, traci_module=traci)
        monitor = RealizationMonitor(config)

        records = []
        for step in range(6):
            command = _command(step, 100.0 + step * 3.0)
            executor.apply_command(command, realized_speed_at_t=30.0)
            traci.simulationStep()
            records.append(monitor.classify_realization(command, executor.read_realized_vehicle(command.vehicle_id)))

        assert {record.result for record in records} == {"matched"}
        assert records[-1].dx_abs is not None and records[-1].dx_abs <= config.mismatch_x_tolerance_m
        assert records[-1].dy_abs is not None and records[-1].dy_abs <= config.mismatch_y_tolerance_m
    finally:
        traci.close(False)
