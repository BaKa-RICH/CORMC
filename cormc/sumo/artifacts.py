from __future__ import annotations

import csv
import json
from pathlib import Path
from typing import Any, Iterable
from xml.etree import ElementTree as ET

from cormc.sumo.commands import P17SumoArtifactResult, SumoSimulationResult
from cormc.sumo.env import ensure_sumo_tools_on_path
from cormc.sumo.executor import (
    DEFAULT_LANE_CHANGE_MODE_BITSET,
    DEFAULT_MOVE_TO_XY_KEEP_ROUTE,
    DEFAULT_SPEED_MODE_BITSET,
    EXECUTOR_MODE,
)
from cormc.sumo.gui_replay import DEFAULT_GUI_DELAY_MS
from cormc.sumo.loop import SumoClosedLoopRunnerConfig, run_sumo_trajectory_authority_simulation
from cormc.sumo.mapping import to_sumo_position
from cormc.sumo.monitoring import write_events_jsonl, write_realization_jsonl


DEFAULT_RUN_ID = "p17_trajectory_authority_demo"
DEFAULT_OUTPUT_ROOT = "artifacts/sumo/p17_trajectory_authority"
SCENARIO_ID = "P17-SUMO-CLOSED-LOOP"
PROFILE_ID = "p16_internal_demo_v1"


def run_p17_sumo_artifact_bundle(
    run_id: str = DEFAULT_RUN_ID,
    output_root: str | Path = DEFAULT_OUTPUT_ROOT,
    seed: int = 16001,
    max_steps: int = 60,
    gui_config_only: bool = False,
) -> P17SumoArtifactResult:
    """Run the P17 SUMO closed loop and write the formal artifact bundle."""

    output_dir = Path(output_root) / run_id
    sumo_dir = output_dir / "sumo"
    output_dir.mkdir(parents=True, exist_ok=True)

    closed_loop = run_sumo_trajectory_authority_simulation(
        SumoClosedLoopRunnerConfig(
            run_id=run_id,
            output_dir=sumo_dir,
            seed=seed,
            max_steps=max_steps,
            scenario_id=SCENARIO_ID,
            gui_config_only=gui_config_only,
        )
    )
    paths = ensure_sumo_tools_on_path()
    config = SumoClosedLoopRunnerConfig(seed=seed, max_steps=max_steps)
    authority_config = config.authority_config

    simulation_result = SumoSimulationResult(
        run_id=run_id,
        status=closed_loop.status,
        sumo_version=closed_loop.sumo_version,
        net_file=closed_loop.net_file,
        route_file=closed_loop.route_file,
        sumocfg_file=closed_loop.sumocfg_file,
        steps=closed_loop.steps,
        realization_records=closed_loop.realization_records,
        collision_events=closed_loop.collision_events,
        teleport_events=closed_loop.teleport_events,
    )

    trajectory_path = output_dir / "trajectory.csv"
    events_path = output_dir / "events.jsonl"
    sanity_path = output_dir / "sanity.jsonl"
    realization_path = output_dir / "realization.jsonl"
    scenario_report_path = output_dir / "scenario_report.json"
    manifest_path = output_dir / "artifact_manifest.json"
    report_path = output_dir / "run_report.md"
    gui_replay_script_path = output_dir / "play_gui_replay.ps1"

    event_records = _event_records(closed_loop)
    sanity_records = _sanity_records(closed_loop)
    output_paths = {
        "trajectory_csv": str(trajectory_path),
        "events_jsonl": str(events_path),
        "sanity_jsonl": str(sanity_path),
        "realization_jsonl": str(realization_path),
        "artifact_manifest_json": str(manifest_path),
        "scenario_report_json": str(scenario_report_path),
        "run_report_md": str(report_path),
        "gui_replay_script_ps1": str(gui_replay_script_path),
    }
    sumo_paths = _sumo_paths(closed_loop)
    traci_sumocfg_path = sumo_dir / "p17.traci.sumocfg"
    preview_route_path = sumo_dir / "p17.preview.rou.xml"
    Path(sumo_paths["sumocfg_file"]).replace(traci_sumocfg_path)
    sumo_paths["traci_sumocfg_file"] = str(traci_sumocfg_path)
    sumo_paths["preview_route_file"] = str(preview_route_path)

    _write_trajectory_csv(trajectory_path, closed_loop.realization_records)
    write_events_jsonl(events_path, event_records)
    write_events_jsonl(sanity_path, sanity_records)
    write_realization_jsonl(realization_path, closed_loop.realization_records)
    _write_static_gui_preview(
        sumocfg_path=Path(sumo_paths["sumocfg_file"]),
        preview_route_path=preview_route_path,
        net_file_name=Path(sumo_paths["net_file"]).name,
        records=closed_loop.realization_records,
        seed=seed,
        background_ids=closed_loop.background_vehicle_ids_sample,
        step_length=authority_config.step_length,
    )
    _write_gui_replay_script(
        gui_replay_script_path,
        sumocfg_file=traci_sumocfg_path,
        realization_path=realization_path,
    )

    common_payload = _common_payload(
        closed_loop=closed_loop,
        run_id=run_id,
        seed=seed,
        max_steps=max_steps,
        gui_config_only=gui_config_only,
        sumo_home=paths.sumo_home,
        authority_config=authority_config,
        output_paths=output_paths,
        sumo_paths=sumo_paths,
    )
    scenario_report = {
        **common_payload,
        "scenario_purpose": "P17 SUMO TraCI trajectory-authority closed-loop smoke evidence",
        "simulation_result": simulation_result.to_dict(),
    }
    manifest = {
        **common_payload,
        "artifact_schema": "p17_sumo_artifact_bundle.v1",
        "files": {**output_paths, **sumo_paths},
        "simulation_result": simulation_result.to_dict(),
    }

    _write_json(scenario_report_path, scenario_report)
    _write_json(manifest_path, manifest)
    _write_run_report(report_path, manifest)

    return P17SumoArtifactResult(
        run_id=run_id,
        output_dir=str(output_dir),
        network_files=sumo_paths,
        simulation_result=simulation_result,
        manifest_path=str(manifest_path),
        report_path=str(report_path),
        extra_paths={
            **output_paths,
            "sumo_dir": str(sumo_dir),
        },
        scenario_id=SCENARIO_ID,
        status=closed_loop.status,
        sumo_config_path=sumo_paths["sumocfg_file"],
        artifact_manifest_path=str(manifest_path),
        run_report_path=str(report_path),
        realization_path=str(realization_path),
        trajectory_path=str(trajectory_path),
        events_path=str(events_path),
        sanity_path=str(sanity_path),
        generated_count=closed_loop.generated_count,
        blocked_spawn_count=closed_loop.blocked_spawn_count,
        collision_count=closed_loop.collision_count,
        teleport_count=closed_loop.teleport_count,
        realization_mismatch_count=closed_loop.mismatch_count,
        active_controlled_vehicle_ids=tuple(closed_loop.active_controlled_vehicle_ids),
    )


def _common_payload(
    *,
    closed_loop: Any,
    run_id: str,
    seed: int,
    max_steps: int,
    gui_config_only: bool,
    sumo_home: str | None,
    authority_config: Any,
    output_paths: dict[str, str],
    sumo_paths: dict[str, str],
) -> dict[str, Any]:
    return {
        "run_id": run_id,
        "scenario_id": SCENARIO_ID,
        "status": closed_loop.status,
        "sumo_version": closed_loop.sumo_version,
        "sumo_home": sumo_home,
        "executor_mode": EXECUTOR_MODE,
        "bitmasks": {
            "speed_mode": DEFAULT_SPEED_MODE_BITSET,
            "lane_change_mode": DEFAULT_LANE_CHANGE_MODE_BITSET,
        },
        "keepRoute": DEFAULT_MOVE_TO_XY_KEEP_ROUTE,
        "speed_mode_bitset": DEFAULT_SPEED_MODE_BITSET,
        "lane_change_mode_bitset": DEFAULT_LANE_CHANGE_MODE_BITSET,
        "move_to_xy_keep_route": DEFAULT_MOVE_TO_XY_KEEP_ROUTE,
        "step_length": authority_config.step_length,
        "lateral_resolution": authority_config.lateral_resolution,
        "collision_action": authority_config.collision_action,
        "seed": seed,
        "profile_id": PROFILE_ID,
        "max_steps": max_steps,
        "gui_config_only": gui_config_only,
        "active_ids": list(closed_loop.active_controlled_vehicle_ids),
        "background_sample": list(closed_loop.background_vehicle_ids_sample),
        "active_controlled_vehicle_ids": list(closed_loop.active_controlled_vehicle_ids),
        "background_vehicle_ids_sample": list(closed_loop.background_vehicle_ids_sample),
        "generated_count": closed_loop.generated_count,
        "blocked_count": closed_loop.blocked_spawn_count,
        "blocked_spawn_count": closed_loop.blocked_spawn_count,
        "collision_count": closed_loop.collision_count,
        "teleport_count": closed_loop.teleport_count,
        "mismatch_count": closed_loop.mismatch_count,
        "realization_mismatch_count": closed_loop.mismatch_count,
        "integration_failure_count": closed_loop.integration_failure_count,
        "command_count": closed_loop.command_count,
        "steps": closed_loop.steps,
        "generated_vehicle_ids": list(closed_loop.generated_vehicle_ids),
        "spawn_failures": list(closed_loop.spawn_failures),
        "output_paths": output_paths,
        "sumo_paths": sumo_paths,
        "gui_replay": {
            "entrypoint": output_paths["gui_replay_script_ps1"],
            "mode": "traci_recorded_trajectory_replay",
            "delay_ms": DEFAULT_GUI_DELAY_MS,
            "hold_seconds": 0,
            "wait_for_enter": False,
            "keep_open_after_replay": True,
            "realization_source": output_paths["realization_jsonl"],
            "sumo_config": sumo_paths["traci_sumocfg_file"],
        },
        "direct_sumo_preview": {
            "sumo_config": sumo_paths["sumocfg_file"],
            "route_file": sumo_paths["preview_route_file"],
            "mode": "static_preview_not_controller_exact",
            "end": 120.0,
        },
    }


def _sumo_paths(closed_loop: Any) -> dict[str, str]:
    sumocfg = Path(closed_loop.sumocfg_file)
    sumo_dir = sumocfg.parent
    return {
        "sumocfg_file": str(sumocfg),
        "net_file": str(closed_loop.net_file),
        "route_file": str(closed_loop.route_file),
        "nodes_file": str(sumo_dir / "p17.nod.xml"),
        "edges_file": str(sumo_dir / "p17.edg.xml"),
        "connections_file": str(sumo_dir / "p17.con.xml"),
    }


def _write_static_gui_preview(
    *,
    sumocfg_path: Path,
    preview_route_path: Path,
    net_file_name: str,
    records: Iterable[Any],
    seed: int,
    background_ids: Iterable[str],
    step_length: float,
) -> None:
    _write_xml(preview_route_path, _preview_routes_xml(records, seed=seed, background_ids=tuple(background_ids)))

    root = ET.Element("configuration")
    input_node = ET.SubElement(root, "input")
    ET.SubElement(input_node, "net-file", value=net_file_name)
    ET.SubElement(input_node, "route-files", value=preview_route_path.name)

    time_node = ET.SubElement(root, "time")
    ET.SubElement(time_node, "begin", value="0.0")
    ET.SubElement(time_node, "end", value="120.0")
    ET.SubElement(time_node, "step-length", value=f"{step_length:.3f}".rstrip("0").rstrip("."))

    processing_node = ET.SubElement(root, "processing")
    ET.SubElement(processing_node, "lateral-resolution", value="0.25")
    ET.SubElement(processing_node, "collision.action", value="warn")
    ET.SubElement(processing_node, "collision.check-junctions", value="true")
    _write_xml(sumocfg_path, root)


def _preview_routes_xml(records: Iterable[Any], *, seed: int, background_ids: tuple[str, ...]) -> ET.Element:
    routes = ET.Element("routes")
    ET.SubElement(
        routes,
        "vType",
        id="cormc_active",
        length="5.0",
        width="1.8",
        minGap="2.5",
        accel="2.6",
        decel="4.5",
        sigma="0.0",
        color="0,90,220",
        latAlignment="center",
    )
    ET.SubElement(
        routes,
        "vType",
        id="sumo_background",
        length="5.0",
        width="1.8",
        minGap="2.5",
        accel="2.0",
        decel="4.5",
        tau="1.0",
        sigma="0.5",
        carFollowModel="IDM",
        laneChangeModel="SL2015",
        color="160,160,160",
        latAlignment="center",
    )
    ET.SubElement(routes, "route", id="route_main", edges="main_pre merge_zone main_post")
    ET.SubElement(routes, "route", id="route_ramp", edges="ramp_upstream ramp_pre merge_zone main_post")

    vehicle_specs: list[dict[str, str]] = []
    first_record_by_vehicle: dict[str, Any] = {}
    for record in records:
        first_record_by_vehicle.setdefault(record.vehicle_id, record)
    for vehicle_id, record in sorted(first_record_by_vehicle.items()):
        y = float(record.command_y)
        lane_role = _lane_role_from_y(y)
        road_role = "on_ramp" if lane_role == "on_ramp" else "mainline"
        route_id = "route_ramp" if road_role == "on_ramp" else "route_main"
        _, lane_index, depart_pos = to_sumo_position(float(record.command_x_global), lane_role, road_role)
        vehicle_specs.append(
            {
                "id": f"preview_{vehicle_id}",
                "type": "cormc_active",
                "route": route_id,
                "depart": f"{float(record.t):.3f}".rstrip("0").rstrip("."),
                "departLane": str(lane_index),
                "departPos": f"{depart_pos:.3f}",
                "departSpeed": f"{float(record.command_v):.3f}",
                "color": "0,150,255",
            }
        )

    for index, vehicle_id in enumerate(background_ids):
        lane_role = "lane_2" if index == 0 else "lane_1"
        x_global = 6890.0 + index * 42.0 + (seed % 7) * 0.1
        _, lane_index, depart_pos = to_sumo_position(x_global, lane_role, "mainline")
        vehicle_specs.append(
            {
                "id": f"preview_{vehicle_id}",
                "type": "sumo_background",
                "route": "route_main",
                "depart": "0.0",
                "departLane": str(lane_index),
                "departPos": f"{depart_pos:.3f}",
                "departSpeed": f"{20.0 + index:.3f}",
                "color": "180,180,180",
            }
        )
    for spec in sorted(vehicle_specs, key=lambda item: (float(item["depart"]), item["id"])):
        ET.SubElement(routes, "vehicle", **spec)
    return routes


def _lane_role_from_y(y: float) -> str:
    if y <= -1.75:
        return "on_ramp"
    if y >= 1.75:
        return "lane_1"
    return "lane_2"


def _write_xml(path: Path, root: ET.Element) -> None:
    tree = ET.ElementTree(root)
    ET.indent(tree, space="  ")
    tree.write(path, encoding="utf-8", xml_declaration=True)


def _write_trajectory_csv(path: Path, records: Iterable[Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "vehicle_id",
        "step",
        "t",
        "command_x_global",
        "command_y",
        "command_v",
        "command_a",
        "realized_x_global",
        "realized_y",
        "realized_v",
        "result",
    ]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for record in records:
            row = record.to_dict()
            writer.writerow({key: row.get(key) for key in fieldnames})


def _event_records(closed_loop: Any) -> list[dict[str, Any]]:
    events = [
        {
            "event_type": "p17_closed_loop_summary",
            "run_id": closed_loop.run_id,
            "status": closed_loop.status,
            "steps": closed_loop.steps,
            "command_count": closed_loop.command_count,
            "generated_count": closed_loop.generated_count,
            "blocked_count": closed_loop.blocked_spawn_count,
            "active_ids": list(closed_loop.active_controlled_vehicle_ids),
            "background_sample": list(closed_loop.background_vehicle_ids_sample),
        }
    ]
    events.extend(dict(event) for event in closed_loop.collision_events)
    events.extend(dict(event) for event in closed_loop.teleport_events)
    for failure in closed_loop.spawn_failures:
        event = dict(failure)
        event.setdefault("event_type", "spawn_or_integration_failure")
        events.append(event)
    return events


def _sanity_records(closed_loop: Any) -> list[dict[str, Any]]:
    return [
        {
            "sanity_id": "p17_realization_records_present",
            "status": "passed" if bool(closed_loop.realization_records) else "failed",
            "observed": len(closed_loop.realization_records),
        },
        {
            "sanity_id": "p17_no_active_collision",
            "status": "passed" if closed_loop.collision_count == 0 else "failed",
            "observed": closed_loop.collision_count,
        },
        {
            "sanity_id": "p17_no_teleport",
            "status": "passed" if closed_loop.teleport_count == 0 else "failed",
            "observed": closed_loop.teleport_count,
        },
        {
            "sanity_id": "p17_realization_mismatch_count",
            "status": "passed" if closed_loop.mismatch_count == 0 else "failed",
            "observed": closed_loop.mismatch_count,
        },
        {
            "sanity_id": "p17_spawn_generation_observed",
            "status": "passed" if closed_loop.generated_count >= 1 else "failed",
            "observed": closed_loop.generated_count,
        },
    ]


def _write_json(path: Path, payload: dict[str, Any]) -> None:
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True), encoding="utf-8")


def _write_gui_replay_script(path: Path, *, sumocfg_file: Path, realization_path: Path) -> None:
    module = "cormc.sumo.gui_replay"
    repo_root = Path(__file__).resolve().parents[2]
    sumocfg_abs = sumocfg_file.resolve()
    realization_abs = realization_path.resolve()
    command = (
        f'python -m {module} --sumocfg "{sumocfg_abs}" --realization "{realization_abs}" '
        f"--delay-ms {DEFAULT_GUI_DELAY_MS} --hold-seconds 0 --keep-open-after-replay"
    )
    path.write_text(
        "\n".join(
            [
                "$ErrorActionPreference = 'Stop'",
                "$env:PYTHONIOENCODING = 'utf-8'",
                f'Set-Location -LiteralPath "{repo_root}"',
                command,
                "",
            ]
        ),
        encoding="utf-8",
    )


def _write_run_report(path: Path, manifest: dict[str, Any]) -> None:
    replay_script = manifest["gui_replay"]["entrypoint"]
    replay_command = f'& "{replay_script}"'
    preview_sumocfg_file = manifest["direct_sumo_preview"]["sumo_config"]
    traci_sumocfg_file = manifest["gui_replay"]["sumo_config"]

    lines = [
        f"# P17 SUMO Trajectory Authority Report: {manifest['run_id']}",
        "",
        f"- scenario_id: {manifest['scenario_id']}",
        f"- status: {manifest['status']}",
        f"- sumo_version: {manifest['sumo_version']}",
        f"- sumo_home: {manifest['sumo_home']}",
        f"- executor_mode: {manifest['executor_mode']}",
        f"- speed_mode_bitset: {manifest['speed_mode_bitset']}",
        f"- lane_change_mode_bitset: {manifest['lane_change_mode_bitset']}",
        f"- move_to_xy_keep_route: {manifest['move_to_xy_keep_route']}",
        f"- step_length: {manifest['step_length']}",
        f"- lateral_resolution: {manifest['lateral_resolution']}",
        f"- collision_action: {manifest['collision_action']}",
        f"- seed: {manifest['seed']}",
        f"- profile_id: {manifest['profile_id']}",
        f"- max_steps: {manifest['max_steps']}",
        f"- active_controlled_vehicle_count: {len(manifest['active_controlled_vehicle_ids'])}",
        f"- background_vehicle_sample_count: {len(manifest['background_vehicle_ids_sample'])}",
        f"- active_controlled_vehicle_ids: {', '.join(manifest['active_controlled_vehicle_ids'])}",
        f"- background_vehicle_ids_sample: {', '.join(manifest['background_vehicle_ids_sample'])}",
        f"- generated_count: {manifest['generated_count']}",
        f"- blocked_spawn_count: {manifest['blocked_spawn_count']}",
        f"- collision_count: {manifest['collision_count']}",
        f"- teleport_count: {manifest['teleport_count']}",
        f"- realization_mismatch_count: {manifest['realization_mismatch_count']}",
        "",
        "## Boundary",
        "",
        "P17 does not do the P18 paper grid; P18 remains the later dual-track experiment route.",
        "",
        "## GUI",
        "",
        "Use the replay script below for human visualization. It opens sumo-gui through TraCI and replays the recorded trajectory-authority trace.",
        "",
        "```powershell",
        replay_command,
        "```",
        "",
        f"Opening `{preview_sumocfg_file}` directly now shows a 120-second static SUMO preview with preview vehicles. It is useful for visual inspection, but it is not the exact TraCI-controlled replay.",
        f"The exact replay script uses `{traci_sumocfg_file}` plus `realization.jsonl` to drive the active vehicles through TraCI.",
        "",
        "## Outputs",
        "",
    ]
    for name, value in manifest["output_paths"].items():
        lines.append(f"- {name}: {value}")
    lines.extend(["", "## SUMO", ""])
    for name, value in manifest["sumo_paths"].items():
        lines.append(f"- {name}: {value}")
    lines.append("")
    path.write_text("\n".join(lines), encoding="utf-8")
