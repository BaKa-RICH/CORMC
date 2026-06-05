from __future__ import annotations

import argparse
import json
import time
from collections import defaultdict
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

from cormc.sumo.env import ensure_sumo_tools_on_path, import_traci
from cormc.sumo.executor import (
    DEFAULT_LANE_CHANGE_MODE_BITSET,
    DEFAULT_MOVE_TO_XY_KEEP_ROUTE,
    DEFAULT_SPEED_MODE_BITSET,
)
from cormc.sumo.gui_replay import DEFAULT_GUI_DELAY_MS, DEFAULT_OBSERVATION_END_SECONDS
from cormc.sumo.mapping import to_sumo_position


@dataclass(frozen=True)
class MvsGuiReplaySummary:
    status: str
    sumocfg_file: str
    replay_jsonl: str
    sumo_gui_started: bool
    replayed_steps: int
    replayed_vehicle_ids: tuple[str, ...]
    closed_on_finish: bool
    track_vehicle_id: str | None
    error: str | None = None

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def run_mvs_gui_replay(
    sumocfg_file: str | Path,
    replay_jsonl: str | Path,
    *,
    track_vehicle_id: str | None = None,
    delay_ms: int = DEFAULT_GUI_DELAY_MS,
    hold_seconds: float = 0.0,
    post_roll_steps: int = 5,
    keep_open_after_replay: bool = False,
) -> MvsGuiReplaySummary:
    """Open SUMO-GUI and replay P17.1 numeric trajectory records through TraCI."""

    sumocfg = Path(sumocfg_file)
    replay_path = Path(replay_jsonl)
    records = _read_replay_records(replay_path)
    records_by_step: dict[int, list[dict[str, Any]]] = defaultdict(list)
    for record in records:
        records_by_step[int(record["step"])].append(record)

    paths = ensure_sumo_tools_on_path()
    traci = import_traci()
    started = False
    closed = False
    replayed_vehicle_ids: set[str] = set()
    focus_vehicle_id = track_vehicle_id or str(records[0]["vehicle_id"])

    try:
        traci.start(
            [
                paths.sumo_gui or paths.sumo,
                "-c",
                str(sumocfg),
                "--start",
                "--delay",
                str(delay_ms),
                "--no-step-log",
                "true",
                "--no-warnings",
                "true",
                "--quit-on-end",
                "false",
                "--end",
                str(DEFAULT_OBSERVATION_END_SECONDS),
            ]
        )
        started = True

        for step in sorted(records_by_step):
            for record in records_by_step[step]:
                _ensure_replay_vehicle(traci, record)
                _apply_replay_record(traci, record)
                vehicle_id = str(record["vehicle_id"])
                replayed_vehicle_ids.add(vehicle_id)
                if vehicle_id == focus_vehicle_id:
                    _focus_gui(traci, vehicle_id, _record_x(record), float(record["y"]))
            traci.simulationStep()

        for _ in range(max(0, int(post_roll_steps))):
            traci.simulationStep()

        if keep_open_after_replay:
            while True:
                try:
                    traci.simulationStep()
                except Exception:
                    break
                time.sleep(max(delay_ms, 1) / 1000.0)
        elif hold_seconds > 0:
            time.sleep(hold_seconds)
    finally:
        if started and not keep_open_after_replay:
            traci.close(True)
            closed = True

    return MvsGuiReplaySummary(
        status="ok",
        sumocfg_file=str(sumocfg),
        replay_jsonl=str(replay_path),
        sumo_gui_started=started,
        replayed_steps=len(records_by_step),
        replayed_vehicle_ids=tuple(sorted(replayed_vehicle_ids)),
        closed_on_finish=closed,
        track_vehicle_id=focus_vehicle_id,
        error=None,
    )


def _read_replay_records(path: Path) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    for line in path.read_text(encoding="utf-8").splitlines():
        if line.strip():
            records.append(json.loads(line))
    if not records:
        raise ValueError(f"No P17.1 replay records found in {path}")
    return records


def _ensure_replay_vehicle(traci: Any, record: dict[str, Any]) -> None:
    vehicle_id = str(record["vehicle_id"])
    if vehicle_id in set(traci.vehicle.getIDList()):
        return

    x_global = _record_x(record)
    y = float(record["y"])
    lane_role = _record_lane_role(record)
    road_role = _record_road_role(record, lane_role)
    edge_id, lane_index, depart_pos = _sumo_position_for_replay_record(record, x_global, lane_role, road_role)
    route_id = "route_ramp" if _uses_ramp_route(record, lane_role, road_role) else "route_main"
    traci.vehicle.add(
        vehicle_id,
        route_id,
        typeID="cormc_active",
        depart="now",
        departLane="best",
        departPos="0",
        departSpeed=f"{float(record['v']):.6f}",
    )
    traci.vehicle.setSpeedMode(vehicle_id, DEFAULT_SPEED_MODE_BITSET)
    traci.vehicle.setLaneChangeMode(vehicle_id, DEFAULT_LANE_CHANGE_MODE_BITSET)
    if hasattr(traci.vehicle, "setColor"):
        traci.vehicle.setColor(vehicle_id, _record_color(record))
    traci.vehicle.moveToXY(
        vehicle_id,
        edge_id,
        lane_index,
        x_global,
        y,
        90.0,
        keepRoute=DEFAULT_MOVE_TO_XY_KEEP_ROUTE,
    )


def _apply_replay_record(traci: Any, record: dict[str, Any]) -> None:
    vehicle_id = str(record["vehicle_id"])
    x_global = _record_x(record)
    y = float(record["y"])
    lane_role = _record_lane_role(record)
    road_role = _record_road_role(record, lane_role)
    edge_id, lane_index, _ = _sumo_position_for_replay_record(record, x_global, lane_role, road_role)
    speed = float(record["v"])
    traci.vehicle.setPreviousSpeed(vehicle_id, speed)
    traci.vehicle.setSpeed(vehicle_id, speed)
    if hasattr(traci.vehicle, "setColor"):
        traci.vehicle.setColor(vehicle_id, _record_color(record))
    traci.vehicle.moveToXY(
        vehicle_id,
        edge_id,
        lane_index,
        x_global,
        y,
        90.0,
        keepRoute=DEFAULT_MOVE_TO_XY_KEEP_ROUTE,
    )


def _record_x(record: dict[str, Any]) -> float:
    return float(record.get("x_global", record.get("command_x_global")))


def _record_lane_role(record: dict[str, Any]) -> str:
    lane = record.get("physical_lane")
    if lane:
        return str(lane)
    y = float(record["y"])
    if y <= -1.75:
        return "on_ramp"
    if y >= 1.75:
        return "lane_1"
    return "lane_2"


def _record_road_role(record: dict[str, Any], lane_role: str) -> str:
    road_role = record.get("road_role")
    if road_role:
        return str(road_role)
    return "on_ramp" if lane_role == "on_ramp" else "mainline"


def _uses_ramp_route(record: dict[str, Any], lane_role: str, road_role: str) -> bool:
    return lane_role == "on_ramp" or road_role in {"on_ramp", "on_ramp_mv"}


def _sumo_position_for_replay_record(
    record: dict[str, Any],
    x_global: float,
    lane_role: str,
    road_role: str,
) -> tuple[str, int, float]:
    try:
        return to_sumo_position(x_global, lane_role, road_role)
    except ValueError:
        hint = record.get("visual_replay_hint")
        if not isinstance(hint, dict) or hint.get("mode") != "allow_pre_control_on_ramp":
            raise
        edge_id = str(hint.get("edge_id", "ramp_pre"))
        lane_index = int(hint.get("lane_index", 0))
        return edge_id, lane_index, 0.0


def _record_color(record: dict[str, Any]) -> tuple[int, int, int]:
    value = record.get("color_rgb", (160, 160, 160))
    if isinstance(value, str):
        parts = value.split(",")
        return tuple(int(float(part.strip())) for part in parts[:3])  # type: ignore[return-value]
    if isinstance(value, (list, tuple)):
        return tuple(int(float(part)) for part in value[:3])  # type: ignore[return-value]
    return (160, 160, 160)


def _focus_gui(traci: Any, vehicle_id: str, x: float, y: float) -> None:
    try:
        view_id = traci.gui.DEFAULT_VIEW
        traci.gui.trackVehicle(view_id, vehicle_id)
        traci.gui.setZoom(view_id, 900.0)
        traci.gui.setOffset(view_id, x, y)
    except Exception:
        return


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Replay a P17.1 MVS artifact in SUMO-GUI.")
    parser.add_argument("--sumocfg", required=True)
    parser.add_argument("--replay", required=True)
    parser.add_argument("--track-vehicle-id")
    parser.add_argument("--delay-ms", type=int, default=DEFAULT_GUI_DELAY_MS)
    parser.add_argument("--hold-seconds", type=float, default=0.0)
    parser.add_argument("--post-roll-steps", type=int, default=5)
    parser.add_argument("--keep-open-after-replay", action="store_true")
    parser.add_argument("--status-output")
    args = parser.parse_args(argv)

    status_path = Path(args.status_output) if args.status_output else None
    try:
        summary = run_mvs_gui_replay(
            args.sumocfg,
            args.replay,
            track_vehicle_id=args.track_vehicle_id,
            delay_ms=args.delay_ms,
            hold_seconds=args.hold_seconds,
            post_roll_steps=args.post_roll_steps,
            keep_open_after_replay=args.keep_open_after_replay,
        )
    except Exception as exc:
        summary = MvsGuiReplaySummary(
            status="failed",
            sumocfg_file=str(args.sumocfg),
            replay_jsonl=str(args.replay),
            sumo_gui_started=False,
            replayed_steps=0,
            replayed_vehicle_ids=(),
            closed_on_finish=False,
            track_vehicle_id=args.track_vehicle_id,
            error=str(exc),
        )
        if status_path is not None:
            status_path.write_text(
                json.dumps(summary.to_dict(), ensure_ascii=False, indent=2, sort_keys=True),
                encoding="utf-8",
            )
        print(json.dumps(summary.to_dict(), ensure_ascii=False, indent=2, sort_keys=True))
        return 1

    if status_path is not None:
        status_path.write_text(
            json.dumps(summary.to_dict(), ensure_ascii=False, indent=2, sort_keys=True),
            encoding="utf-8",
        )
    print(json.dumps(summary.to_dict(), ensure_ascii=False, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
