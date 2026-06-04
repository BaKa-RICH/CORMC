from __future__ import annotations

import argparse
import json
import time
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from cormc.sumo.env import ensure_sumo_tools_on_path, import_traci
from cormc.sumo.executor import (
    DEFAULT_LANE_CHANGE_MODE_BITSET,
    DEFAULT_MOVE_TO_XY_KEEP_ROUTE,
    DEFAULT_SPEED_MODE_BITSET,
)
from cormc.sumo.mapping import to_sumo_position


DEFAULT_GUI_DELAY_MS = 150
DEFAULT_HOLD_SECONDS = 30
DEFAULT_POST_ROLL_STEPS = 5
DEFAULT_OBSERVATION_END_SECONDS = 3600


@dataclass(frozen=True)
class GuiReplaySummary:
    status: str
    sumocfg_file: str
    realization_jsonl: str
    replayed_steps: int
    replayed_vehicle_ids: tuple[str, ...]
    hold_seconds: float

    def to_dict(self) -> dict[str, Any]:
        return {
            "status": self.status,
            "sumocfg_file": self.sumocfg_file,
            "realization_jsonl": self.realization_jsonl,
            "replayed_steps": self.replayed_steps,
            "replayed_vehicle_ids": list(self.replayed_vehicle_ids),
            "hold_seconds": self.hold_seconds,
        }


def run_p17_gui_replay(
    sumocfg_file: str | Path,
    realization_jsonl: str | Path,
    *,
    delay_ms: int = DEFAULT_GUI_DELAY_MS,
    hold_seconds: float = DEFAULT_HOLD_SECONDS,
    post_roll_steps: int = DEFAULT_POST_ROLL_STEPS,
    wait_for_enter: bool = False,
    keep_open_after_replay: bool = False,
) -> GuiReplaySummary:
    """Open SUMO-GUI and replay the recorded P17 trajectory-authority trace."""

    sumocfg = Path(sumocfg_file)
    realization_path = Path(realization_jsonl)
    records = _read_realization_records(realization_path)
    records_by_step: dict[int, list[dict[str, Any]]] = defaultdict(list)
    for record in records:
        records_by_step[int(record["step"])].append(record)

    paths = ensure_sumo_tools_on_path()
    traci = import_traci()
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

    replayed_vehicle_ids: set[str] = set()
    try:
        _seed_background_vehicles(traci, realization_path)
        for step in sorted(records_by_step):
            for record in records_by_step[step]:
                _ensure_replay_vehicle(traci, record)
                _apply_replay_record(traci, record)
                replayed_vehicle_ids.add(str(record["vehicle_id"]))
            traci.simulationStep()

        for _ in range(max(0, post_roll_steps)):
            traci.simulationStep()

        if keep_open_after_replay:
            print("P17 SUMO-GUI replay finished. The GUI window is now in live observation mode.")
            print("The SUMO clock will keep advancing slowly; close the SUMO-GUI window when you are done.")
            while True:
                try:
                    traci.simulationStep()
                except Exception:
                    break
                time.sleep(max(delay_ms, 1) / 1000.0)
        elif wait_for_enter:
            input("P17 SUMO-GUI replay finished. Press Enter to close SUMO-GUI.")
        elif hold_seconds > 0:
            time.sleep(hold_seconds)
    finally:
        if not keep_open_after_replay:
            traci.close(True)

    return GuiReplaySummary(
        status="ok",
        sumocfg_file=str(sumocfg),
        realization_jsonl=str(realization_path),
        replayed_steps=len(records_by_step),
        replayed_vehicle_ids=tuple(sorted(replayed_vehicle_ids)),
        hold_seconds=hold_seconds,
    )


def _read_realization_records(path: Path) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    for line in path.read_text(encoding="utf-8").splitlines():
        if line.strip():
            records.append(json.loads(line))
    if not records:
        raise ValueError(f"No realization records found in {path}")
    return records


def _ensure_replay_vehicle(traci: Any, record: dict[str, Any]) -> None:
    vehicle_id = str(record["vehicle_id"])
    if vehicle_id in set(traci.vehicle.getIDList()):
        return

    x_global = float(record["command_x_global"])
    y = float(record["command_y"])
    lane_role = _lane_role_from_y(y)
    road_role = "on_ramp" if lane_role == "on_ramp" else "mainline"
    route_id = "route_ramp" if road_role == "on_ramp" else "route_main"
    edge_id, lane_index, depart_pos = to_sumo_position(x_global, lane_role, road_role)
    traci.vehicle.add(
        vehicle_id,
        route_id,
        typeID="cormc_active",
        depart="now",
        departLane=str(lane_index),
        departPos=f"{depart_pos:.6f}",
        departSpeed=f"{float(record['command_v']):.6f}",
    )
    traci.vehicle.setSpeedMode(vehicle_id, DEFAULT_SPEED_MODE_BITSET)
    traci.vehicle.setLaneChangeMode(vehicle_id, DEFAULT_LANE_CHANGE_MODE_BITSET)
    traci.vehicle.moveToXY(vehicle_id, edge_id, lane_index, x_global, y, 90.0, keepRoute=DEFAULT_MOVE_TO_XY_KEEP_ROUTE)


def _apply_replay_record(traci: Any, record: dict[str, Any]) -> None:
    vehicle_id = str(record["vehicle_id"])
    x_global = float(record["command_x_global"])
    y = float(record["command_y"])
    lane_role = _lane_role_from_y(y)
    road_role = "on_ramp" if lane_role == "on_ramp" else "mainline"
    edge_id, lane_index, _ = to_sumo_position(x_global, lane_role, road_role)
    speed = float(record["command_v"])
    traci.vehicle.setPreviousSpeed(vehicle_id, speed)
    traci.vehicle.setSpeed(vehicle_id, speed)
    traci.vehicle.moveToXY(vehicle_id, edge_id, lane_index, x_global, y, 90.0, keepRoute=DEFAULT_MOVE_TO_XY_KEEP_ROUTE)
    if vehicle_id == "MV_ACTIVE":
        _focus_gui(traci, vehicle_id, x_global, y)


def _lane_role_from_y(y: float) -> str:
    if y <= -1.75:
        return "on_ramp"
    if y >= 1.75:
        return "lane_1"
    return "lane_2"


def _seed_background_vehicles(traci: Any, realization_path: Path) -> None:
    manifest_path = realization_path.parent / "artifact_manifest.json"
    if not manifest_path.exists():
        return
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    seed = int(manifest.get("seed", 0))
    background_ids = list(manifest.get("background_vehicle_ids_sample", ()))
    for index, vehicle_id in enumerate(background_ids):
        if vehicle_id in set(traci.vehicle.getIDList()):
            continue
        lane_role = "lane_2" if index == 0 else "lane_1"
        y = 0.0 if lane_role == "lane_2" else 3.5
        x_global = 6890.0 + index * 42.0 + (seed % 7) * 0.1
        edge_id, lane_index, depart_pos = to_sumo_position(x_global, lane_role, "mainline")
        speed = 20.0 + index
        traci.vehicle.add(
            str(vehicle_id),
            "route_main",
            typeID="sumo_background",
            depart="now",
            departLane=str(lane_index),
            departPos=f"{depart_pos:.6f}",
            departSpeed=f"{speed:.6f}",
        )
        traci.vehicle.moveToXY(str(vehicle_id), edge_id, lane_index, x_global, y, 90.0, keepRoute=DEFAULT_MOVE_TO_XY_KEEP_ROUTE)


def _focus_gui(traci: Any, vehicle_id: str, x: float, y: float) -> None:
    try:
        view_id = traci.gui.DEFAULT_VIEW
        traci.gui.trackVehicle(view_id, vehicle_id)
        traci.gui.setZoom(view_id, 900.0)
        traci.gui.setOffset(view_id, x, y)
    except Exception:
        return


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Replay a P17 artifact in SUMO-GUI through TraCI.")
    parser.add_argument("--sumocfg", required=True)
    parser.add_argument("--realization", required=True)
    parser.add_argument("--delay-ms", type=int, default=DEFAULT_GUI_DELAY_MS)
    parser.add_argument("--hold-seconds", type=float, default=DEFAULT_HOLD_SECONDS)
    parser.add_argument("--post-roll-steps", type=int, default=DEFAULT_POST_ROLL_STEPS)
    parser.add_argument("--wait-for-enter", action="store_true")
    parser.add_argument("--keep-open-after-replay", action="store_true")
    args = parser.parse_args(argv)

    summary = run_p17_gui_replay(
        args.sumocfg,
        args.realization,
        delay_ms=args.delay_ms,
        hold_seconds=args.hold_seconds,
        post_roll_steps=args.post_roll_steps,
        wait_for_enter=args.wait_for_enter,
        keep_open_after_replay=args.keep_open_after_replay,
    )
    print(json.dumps(summary.to_dict(), ensure_ascii=False, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
