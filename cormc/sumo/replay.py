from __future__ import annotations

import json
import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from cormc.sumo.commands import ControlledTrajectoryCommand, P17SumoArtifactResult, SumoSimulationResult, SumoTrajectoryAuthorityConfig
from cormc.sumo.env import ensure_sumo_tools_on_path, get_sumo_version, import_traci
from cormc.sumo.executor import MoveToXYTrajectoryExecutor
from cormc.sumo.mapping import lane_index_for_role, to_sumo_position
from cormc.sumo.monitoring import (
    RealizationMonitor,
    collect_collision_events,
    collect_teleport_events,
    write_events_jsonl,
    write_realization_jsonl,
)
from cormc.sumo.network import P17SumoNetworkConfig, build_p17_sumo_network


REPLAY_MAIN_STRAIGHT = "REPLAY_MAIN_STRAIGHT"
REPLAY_CV_LANE_CHANGE = "REPLAY_CV_LANE_CHANGE"
REPLAY_MV_MERGE = "REPLAY_MV_MERGE"
REPLAY_IDS = (REPLAY_MAIN_STRAIGHT, REPLAY_CV_LANE_CHANGE, REPLAY_MV_MERGE)


@dataclass(frozen=True)
class ReplayTrajectory:
    run_id: str
    vehicle_id: str
    route_id: str
    vehicle_type: str
    commands: tuple[ControlledTrajectoryCommand, ...]


def build_replay_trajectory(run_id: str, *, step_length: float = 0.1) -> ReplayTrajectory:
    if run_id == REPLAY_MAIN_STRAIGHT:
        return _linear_replay(
            run_id=run_id,
            vehicle_id="MAIN_ACTIVE",
            route_id="route_main",
            physical_lane_start="lane_2",
            physical_lane_end="lane_2",
            road_role="mainline",
            x_start=100.0,
            x_end=160.0,
            y_start=0.0,
            y_end=0.0,
            v=30.0,
            step_length=step_length,
        )
    if run_id == REPLAY_CV_LANE_CHANGE:
        return _linear_replay(
            run_id=run_id,
            vehicle_id="CV_ACTIVE",
            route_id="route_main",
            physical_lane_start="lane_2",
            physical_lane_end="lane_1",
            road_role="mainline",
            x_start=6900.0,
            x_end=7040.0,
            y_start=0.0,
            y_end=3.5,
            v=30.0,
            step_length=step_length,
        )
    if run_id == REPLAY_MV_MERGE:
        return _linear_replay(
            run_id=run_id,
            vehicle_id="MV_ACTIVE",
            route_id="route_ramp",
            physical_lane_start="on_ramp",
            physical_lane_end="lane_2",
            road_role="on_ramp",
            x_start=6950.0,
            x_end=7250.0,
            y_start=-3.5,
            y_end=0.0,
            v=30.0,
            step_length=step_length,
        )
    raise ValueError(f"Unknown P17 replay id {run_id!r}; expected one of {REPLAY_IDS}")


def run_replay(
    run_id: str,
    output_dir: str | Path,
    *,
    config: SumoTrajectoryAuthorityConfig | None = None,
) -> P17SumoArtifactResult:
    cfg = config or SumoTrajectoryAuthorityConfig(executor_mode="move_to_xy_trajectory_authority")
    trajectory = build_replay_trajectory(run_id, step_length=cfg.step_length)
    out = Path(output_dir)
    sumo_dir = out / "sumo"
    network_end = trajectory.commands[-1].target_t + cfg.step_length * 3
    network_files = build_p17_sumo_network(
        sumo_dir,
        P17SumoNetworkConfig(
            step_length=cfg.step_length,
            lateral_resolution=cfg.lateral_resolution,
            collision_action=cfg.collision_action,
            end=network_end,
        ),
    )

    paths = ensure_sumo_tools_on_path()
    traci = import_traci()
    traci.start(
        [
            paths.sumo,
            "-c",
            network_files.sumocfg_file,
            "--no-step-log",
            "true",
            "--no-warnings",
            "true",
        ]
    )

    records = []
    collision_events: list[dict[str, Any]] = []
    teleport_events: list[dict[str, Any]] = []
    monitor = RealizationMonitor(cfg)
    executor = MoveToXYTrajectoryExecutor(cfg, traci_module=traci)

    try:
        first = trajectory.commands[0]
        edge_id, lane_index, depart_pos = to_sumo_position(first.x_global, first.physical_lane, first.road_role)
        traci.vehicle.add(
            trajectory.vehicle_id,
            trajectory.route_id,
            typeID=trajectory.vehicle_type,
            depart="now",
            departLane=str(lane_index),
            departPos=f"{depart_pos:.6f}",
            departSpeed=f"{first.v:.6f}",
        )
        traci.simulationStep()

        realized_speed = first.v
        active_ids = (trajectory.vehicle_id,)
        for command in trajectory.commands:
            executor.apply_command(command, realized_speed_at_t=realized_speed)
            traci.simulationStep()
            collisions = collect_collision_events(traci, step=command.step, t=command.target_t, active_vehicle_ids=active_ids)
            teleports = collect_teleport_events(traci, step=command.step, t=command.target_t, active_vehicle_ids=active_ids)
            collision_events.extend(collisions)
            teleport_events.extend(teleports)
            realized = executor.read_realized_vehicle(command.vehicle_id)
            record = monitor.classify_realization(
                command,
                realized,
                collision_vehicle_ids=_event_vehicle_ids(collisions),
                teleported_vehicle_ids=_event_vehicle_ids(teleports),
            )
            records.append(record)
            realized_speed = record.realized_v if record.realized_v is not None else command.v
    finally:
        traci.close(False)

    status = "passed" if all(record.result == "matched" for record in records) and not collision_events and not teleport_events else "failed"
    simulation_result = SumoSimulationResult(
        run_id=run_id,
        status=status,
        sumo_version=get_sumo_version(paths.sumo),
        net_file=network_files.net_file,
        route_file=network_files.routes_file,
        sumocfg_file=network_files.sumocfg_file,
        steps=len(records),
        realization_records=tuple(records),
        collision_events=tuple(collision_events),
        teleport_events=tuple(teleport_events),
    )

    realization_path = out / "realization.jsonl"
    events_path = out / "events.jsonl"
    manifest_path = out / "artifact_manifest.json"
    report_path = out / "run_report.md"
    write_realization_jsonl(realization_path, records)
    write_events_jsonl(events_path, [*collision_events, *teleport_events])
    _write_manifest(manifest_path, run_id, cfg, network_files.to_dict(), simulation_result, executor.metadata.to_dict())
    _write_report(report_path, run_id, cfg, paths.sumo_gui, network_files.sumocfg_file, simulation_result, executor.metadata.to_dict())

    return P17SumoArtifactResult(
        run_id=run_id,
        output_dir=str(out),
        network_files=network_files.to_dict(),
        simulation_result=simulation_result,
        manifest_path=str(manifest_path),
        report_path=str(report_path),
        extra_paths={
            "realization_jsonl": str(realization_path),
            "events_jsonl": str(events_path),
        },
    )


def run_all_replays(output_dir: str | Path, *, config: SumoTrajectoryAuthorityConfig | None = None) -> tuple[P17SumoArtifactResult, ...]:
    out = Path(output_dir)
    return tuple(run_replay(run_id, out / run_id, config=config) for run_id in REPLAY_IDS)


def _linear_replay(
    *,
    run_id: str,
    vehicle_id: str,
    route_id: str,
    physical_lane_start: str,
    physical_lane_end: str,
    road_role: str,
    x_start: float,
    x_end: float,
    y_start: float,
    y_end: float,
    v: float,
    step_length: float,
) -> ReplayTrajectory:
    duration = abs(x_end - x_start) / v
    steps = max(1, round(duration / step_length))
    commands: list[ControlledTrajectoryCommand] = []
    for index in range(steps + 1):
        ratio = index / steps
        x = x_start + (x_end - x_start) * ratio
        y = y_start + (y_end - y_start) * ratio
        physical_lane = physical_lane_start if ratio < 0.5 else physical_lane_end
        command_road_role = road_role
        if run_id == REPLAY_MV_MERGE and x >= 7250.0:
            command_road_role = "mainline"
        commands.append(
            ControlledTrajectoryCommand(
                vehicle_id=vehicle_id,
                step=index,
                t=index * step_length,
                target_t=(index + 1) * step_length,
                x_global=x,
                y=y,
                v=v,
                a=0.0,
                physical_lane=physical_lane,
                road_role=command_road_role,
                authority_mode="trajectory_authority",
                authority_reason=run_id,
            )
        )
    return ReplayTrajectory(run_id, vehicle_id, route_id, "cormc_active", tuple(commands))


def _event_vehicle_ids(events: list[dict[str, Any]]) -> tuple[str, ...]:
    ids: list[str] = []
    for event in events:
        if "vehicle_id" in event:
            ids.append(event["vehicle_id"])
        ids.extend(event.get("vehicle_ids", ()))
    return tuple(ids)


def _write_manifest(
    path: Path,
    run_id: str,
    config: SumoTrajectoryAuthorityConfig,
    network_files: dict[str, str],
    simulation_result: SumoSimulationResult,
    executor_metadata: dict[str, Any],
) -> None:
    payload = {
        "run_id": run_id,
        "config": config.to_dict(),
        "executor": executor_metadata,
        "network_files": network_files,
        "simulation_result": simulation_result.to_dict(),
    }
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True), encoding="utf-8")


def _write_report(
    path: Path,
    run_id: str,
    config: SumoTrajectoryAuthorityConfig,
    sumo_gui: str | None,
    sumocfg_file: str,
    simulation_result: SumoSimulationResult,
    executor_metadata: dict[str, Any],
) -> None:
    gui_binary = sumo_gui or "sumo-gui"
    matched = sum(1 for record in simulation_result.realization_records if record.result == "matched")
    total = len(simulation_result.realization_records)
    lines = [
        f"# P17 SUMO Replay Report: {run_id}",
        "",
        f"- status: {simulation_result.status}",
        f"- sumo_version: {simulation_result.sumo_version}",
        f"- matched_records: {matched}/{total}",
        f"- executor_mode: {executor_metadata['executor_mode']}",
        f"- speed_mode_bitset: {executor_metadata['speed_mode_bitset']}",
        f"- lane_change_mode_bitset: {executor_metadata['lane_change_mode_bitset']}",
        f"- move_to_xy_keep_route: {executor_metadata['move_to_xy_keep_route']}",
        f"- lateral_resolution: {config.lateral_resolution}",
        f"- collision_action: {config.collision_action}",
        "",
        "## GUI",
        "",
        "```powershell",
        f'& "{gui_binary}" -c "{sumocfg_file}"',
        "```",
        "",
    ]
    path.write_text("\n".join(lines), encoding="utf-8")
