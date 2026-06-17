from __future__ import annotations

from pathlib import Path

from cormc.sumo.commands import ControlledTrajectoryCommand
from cormc.sumo.executor import RealizedVehicleState
from cormc.sumo.monitoring import (
    COLLIDED,
    MATCHED,
    MISMATCH,
    MISSING,
    TELEPORTED,
    RealizationMonitor,
    classify_collision,
    read_realization_jsonl,
    write_realization_jsonl,
)


def _command() -> ControlledTrajectoryCommand:
    return ControlledTrajectoryCommand(
        vehicle_id="veh",
        step=1,
        t=0.1,
        target_t=0.2,
        x_global=100.0,
        y=0.0,
        v=30.0,
        a=0.0,
    )


def _realized(x: float | None = 100.1, y: float | None = 0.1, v: float | None = 30.2) -> RealizedVehicleState:
    return RealizedVehicleState(
        vehicle_id="veh",
        x_global=x,
        y=y,
        v=v,
        edge_id="main_pre" if x is not None else None,
        lane_id="main_pre_0" if x is not None else None,
        lane_position=x if x is not None else None,
    )


def test_realization_monitor_classifies_matched_mismatch_missing_collision_and_teleport() -> None:
    monitor = RealizationMonitor()
    command = _command()

    assert monitor.classify_realization(command, _realized()).result == MATCHED
    assert monitor.classify_realization(command, _realized(x=101.0)).result == MISMATCH
    assert monitor.classify_realization(command, _realized(y=0.5)).result == MISMATCH
    assert monitor.classify_realization(command, _realized(v=31.5)).result == MISMATCH
    assert monitor.classify_realization(command, _realized(x=None, y=None, v=None)).result == MISSING
    assert monitor.classify_realization(command, _realized(), collision_vehicle_ids=("veh",)).result == COLLIDED
    assert monitor.classify_realization(command, _realized(), teleported_vehicle_ids=("veh",)).result == TELEPORTED


def test_collision_classification_splits_active_and_background() -> None:
    assert classify_collision(("a", "b"), ("a", "b"))["collision_type"] == "active_vs_active_collision"
    assert classify_collision(("a", "bg"), ("a",))["collision_type"] == "active_vs_background_collision"
    assert classify_collision(("bg1", "bg2"), ("a",))["collision_type"] == "background_vs_background_collision"


def test_realization_jsonl_round_trips_records(tmp_path: Path) -> None:
    monitor = RealizationMonitor()
    record = monitor.classify_realization(_command(), _realized())
    path = tmp_path / "realization.jsonl"

    write_realization_jsonl(path, [record])

    assert read_realization_jsonl(path) == [record]
