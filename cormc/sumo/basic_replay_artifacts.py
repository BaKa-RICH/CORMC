from __future__ import annotations

import argparse
import json
import subprocess
from dataclasses import asdict, dataclass
from datetime import UTC, datetime
from pathlib import Path
from typing import Any, Iterable, Mapping
from xml.etree import ElementTree as ET

from cormc.basic_runner import run_basic_numeric_scenario
from cormc.basic_scenarios import BASIC_SCENARIO_IDS, get_basic_expectation
from cormc.simulation_loop import SimulationLoopResult
from cormc.sumo.env import ensure_sumo_tools_on_path, get_sumo_version
from cormc.sumo.gui_replay import DEFAULT_GUI_DELAY_MS
from cormc.sumo.mvs_gui_replay import run_mvs_gui_replay
from cormc.sumo.mvs_replay_artifacts import ROLE_COLORS, verify_replay_fidelity
from cormc.sumo.network import P17SumoNetworkConfig, build_p17_sumo_network


DEFAULT_RUN_ID = "basic01_sumo_replay"
DEFAULT_OUTPUT_ROOT = "artifacts/sumo/basic_replay"
VISUAL_HINT_MODE = "allow_pre_control_on_ramp"

BASIC_ROLE_MAP: dict[str, dict[str, str]] = {
    "BASIC-01": {
        "B01_MV": "mv_on_ramp_active",
        "B01_CLV": "clv",
        "B01_CFV": "cfv_active_cooperative",
        "B01_TLV_CFV": "tlv",
    },
}


@dataclass(frozen=True)
class BasicSumoReplayScenarioResult:
    scenario_id: str
    run_id: str
    output_dir: str
    status: str
    numeric_gate_status: str
    replay_fidelity_status: str
    gui_smoke_status: str
    trajectory_path: str
    events_path: str
    sanity_path: str
    numeric_summary_path: str
    replay_trajectory_path: str | None
    scenario_report_path: str
    artifact_manifest_path: str
    gui_replay_script_path: str | None
    gui_smoke_status_path: str
    sumo_config_path: str | None

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class BasicSumoReplayRunResult:
    run_id: str
    output_root: str
    status: str
    report_path: str
    artifact_manifest_path: str
    scenario_results: tuple[BasicSumoReplayScenarioResult, ...]

    def to_dict(self) -> dict[str, Any]:
        payload = asdict(self)
        payload["scenario_results"] = [result.to_dict() for result in self.scenario_results]
        return payload


def run_basic_sumo_replay_artifacts(
    *,
    output_root: str | Path = DEFAULT_OUTPUT_ROOT,
    run_id: str = DEFAULT_RUN_ID,
    scenario: str = "BASIC-01",
    max_steps: int = 900,
    validate_gui_smoke: bool = False,
) -> BasicSumoReplayRunResult:
    output_dir = Path(output_root)
    output_dir.mkdir(parents=True, exist_ok=True)
    scenarios_dir = output_dir / "scenarios"
    scenarios_dir.mkdir(parents=True, exist_ok=True)

    scenario_ids = _select_scenarios(scenario)
    scenario_results = tuple(
        build_basic_sumo_replay_artifact(
            scenario_id=scenario_id,
            scenario_dir=scenarios_dir / scenario_id,
            run_id=run_id,
            max_steps=max_steps,
            validate_gui_smoke=validate_gui_smoke,
        )
        for scenario_id in scenario_ids
    )
    status = _root_status(scenario_results, validate_gui_smoke=validate_gui_smoke)
    manifest = _root_manifest(
        run_id=run_id,
        output_dir=output_dir,
        status=status,
        max_steps=max_steps,
        scenario_results=scenario_results,
        validate_gui_smoke=validate_gui_smoke,
    )
    manifest_path = output_dir / "artifact_manifest.json"
    report_path = output_dir / "report.md"
    _write_json(manifest_path, manifest)
    _write_root_report(report_path, manifest)
    return BasicSumoReplayRunResult(
        run_id=run_id,
        output_root=str(output_dir),
        status=status,
        report_path=str(report_path),
        artifact_manifest_path=str(manifest_path),
        scenario_results=scenario_results,
    )


def build_basic_sumo_replay_artifact(
    *,
    scenario_id: str = "BASIC-01",
    scenario_dir: str | Path,
    run_id: str = DEFAULT_RUN_ID,
    max_steps: int = 900,
    validate_gui_smoke: bool = False,
) -> BasicSumoReplayScenarioResult:
    if scenario_id not in BASIC_ROLE_MAP:
        known = ", ".join(sorted(BASIC_ROLE_MAP))
        raise ValueError(f"unsupported BASIC SUMO replay scenario {scenario_id!r}; expected one of: {known}")

    out = Path(scenario_dir)
    out.mkdir(parents=True, exist_ok=True)

    numeric = run_basic_numeric_scenario(
        scenario_id,
        output_dir=out.parent,
        run_id=run_id,
        max_steps=max_steps,
        render_png=False,
    )
    simulation = numeric.simulation_result
    numeric_summary = dict(numeric.numeric_summary)
    numeric_gate_status = str(numeric_summary["status"])

    replay_path: Path | None = None
    sumocfg_path: Path | None = None
    gui_script_path: Path | None = None
    replay_fidelity = {"status": "skipped", "reason": "numeric gate failed"}
    gui_smoke_status_path = out / "gui_smoke_status.json"
    gui_smoke_payload = _not_run_gui_smoke_status(scenario_id, out)
    _write_json(gui_smoke_status_path, gui_smoke_payload)

    if numeric_gate_status == "passed":
        replay_path = out / "replay_trajectory.jsonl"
        replay_records = write_basic_replay_trajectory_jsonl(
            replay_path,
            scenario_id=scenario_id,
            simulation=simulation,
        )
        replay_fidelity = verify_replay_fidelity(simulation, replay_records)
        _augment_pre_control_replay_checks(replay_fidelity, replay_records, scenario_id=scenario_id)

        sumo_files = build_basic_sumo_replay_files(out / "sumo", simulation=simulation)
        sumocfg_path = Path(sumo_files["sumocfg_file"])
        gui_script_path = out / "play_gui_replay.ps1"
        _write_gui_script(
            gui_script_path,
            sumocfg_file=sumocfg_path,
            replay_path=replay_path,
            status_path=gui_smoke_status_path,
            track_vehicle_id=get_basic_expectation(scenario_id).mv_id,
        )

        if validate_gui_smoke:
            gui_smoke_payload = validate_gui_smoke_for_basic_scenario(
                sumocfg_file=sumocfg_path,
                replay_path=replay_path,
                status_path=gui_smoke_status_path,
                track_vehicle_id=get_basic_expectation(scenario_id).mv_id,
            )

    scenario_manifest_path = out / "artifact_manifest.json"
    scenario_report_path = out / "scenario_report.md"
    scenario_status = _scenario_status(
        numeric_gate_status,
        str(replay_fidelity["status"]),
        str(gui_smoke_payload["status"]),
        validate_gui_smoke=validate_gui_smoke,
    )
    manifest = _scenario_manifest(
        run_id=run_id,
        scenario_id=scenario_id,
        scenario_dir=out,
        status=scenario_status,
        max_steps=max_steps,
        simulation=simulation,
        numeric_summary=numeric_summary,
        replay_fidelity=replay_fidelity,
        gui_smoke_status=gui_smoke_payload,
        paths={
            "trajectory_csv": Path(numeric.numeric_summary["artifact_paths"]["trajectory"]),
            "events_jsonl": Path(numeric.numeric_summary["artifact_paths"]["events"]),
            "sanity_jsonl": Path(numeric.numeric_summary["artifact_paths"]["sanity"]),
            "numeric_summary_json": Path(numeric.numeric_summary_path),
            "replay_trajectory_jsonl": replay_path,
            "scenario_report_md": scenario_report_path,
            "artifact_manifest_json": scenario_manifest_path,
            "play_gui_replay_ps1": gui_script_path,
            "gui_smoke_status_json": gui_smoke_status_path,
            "sumo_config": sumocfg_path,
        },
    )
    _write_json(scenario_manifest_path, manifest)
    _write_scenario_report(scenario_report_path, manifest)

    return BasicSumoReplayScenarioResult(
        scenario_id=scenario_id,
        run_id=run_id,
        output_dir=str(out),
        status=scenario_status,
        numeric_gate_status=numeric_gate_status,
        replay_fidelity_status=str(replay_fidelity["status"]),
        gui_smoke_status=str(gui_smoke_payload["status"]),
        trajectory_path=str(numeric.numeric_summary["artifact_paths"]["trajectory"]),
        events_path=str(numeric.numeric_summary["artifact_paths"]["events"]),
        sanity_path=str(numeric.numeric_summary["artifact_paths"]["sanity"]),
        numeric_summary_path=str(numeric.numeric_summary_path),
        replay_trajectory_path=str(replay_path) if replay_path is not None else None,
        scenario_report_path=str(scenario_report_path),
        artifact_manifest_path=str(scenario_manifest_path),
        gui_replay_script_path=str(gui_script_path.resolve()) if gui_script_path is not None else None,
        gui_smoke_status_path=str(gui_smoke_status_path),
        sumo_config_path=str(sumocfg_path) if sumocfg_path is not None else None,
    )


def write_basic_replay_trajectory_jsonl(
    path: str | Path,
    *,
    scenario_id: str,
    simulation: SimulationLoopResult,
) -> list[dict[str, Any]]:
    role_map = BASIC_ROLE_MAP[scenario_id]
    expectation = get_basic_expectation(scenario_id)
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    records: list[dict[str, Any]] = []
    with out.open("w", encoding="utf-8") as handle:
        for record in sorted(
            simulation.history.trajectory_records,
            key=lambda item: (int(item.step), str(item.vehicle_id)),
        ):
            vehicle_role = role_map.get(record.vehicle_id, "background")
            payload = {
                "step": record.step,
                "t": record.t,
                "vehicle_id": record.vehicle_id,
                "vehicle_role": vehicle_role,
                "source_scenario_id": scenario_id,
                "x_global": record.x_global,
                "y": record.y,
                "v": record.v,
                "a": record.a,
                "physical_lane": record.physical_lane,
                "road_role": record.road_role,
                "merge_state": record.merge_state,
                "lane_change_state": record.lane_change_state,
                "color_rgb": list(ROLE_COLORS.get(vehicle_role, ROLE_COLORS["background"])),
            }
            if _needs_visual_hint(payload, control_activation_x_global=expectation.control_activation_x_global):
                payload["visual_replay_hint"] = {
                    "mode": VISUAL_HINT_MODE,
                    "edge_id": "ramp_pre",
                    "lane_index": 0,
                    "reason": "pre-control on-ramp record is before current P17 ramp_pre edge start",
                }
            records.append(payload)
            handle.write(json.dumps(payload, ensure_ascii=False, sort_keys=True) + "\n")
    return records


def build_basic_sumo_replay_files(
    output_dir: str | Path,
    *,
    simulation: SimulationLoopResult,
) -> dict[str, str]:
    t_values = [float(record.t) for record in simulation.history.trajectory_records]
    end = (max(t_values) + 2.0) if t_values else 5.0
    network_files = build_p17_sumo_network(output_dir, P17SumoNetworkConfig(end=end))
    marker_path = Path(output_dir) / "basic_replay.markers.add.xml"
    _write_marker_file(marker_path)
    _patch_sumocfg_additional_file(Path(network_files.sumocfg_file), marker_path.name)
    payload = network_files.to_dict()
    payload["marker_file"] = str(marker_path)
    return payload


def validate_gui_smoke_for_basic_scenario(
    *,
    sumocfg_file: str | Path,
    replay_path: str | Path,
    status_path: str | Path,
    track_vehicle_id: str,
) -> dict[str, Any]:
    try:
        summary = run_mvs_gui_replay(
            sumocfg_file,
            replay_path,
            track_vehicle_id=track_vehicle_id,
            delay_ms=1,
            hold_seconds=0,
            post_roll_steps=0,
            keep_open_after_replay=False,
        )
    except Exception as exc:
        payload = {
            "status": "failed",
            "sumocfg_file": str(sumocfg_file),
            "replay_jsonl": str(replay_path),
            "sumo_gui_started": False,
            "replayed_steps": 0,
            "replayed_vehicle_ids": [],
            "closed_on_finish": False,
            "track_vehicle_id": track_vehicle_id,
            "error": str(exc),
        }
        _write_json(Path(status_path), payload)
        return payload

    payload = summary.to_dict()
    expected_ids = set(BASIC_ROLE_MAP["BASIC-01"])
    replayed_ids = set(payload["replayed_vehicle_ids"])
    if summary.status == "ok" and summary.replayed_steps > 0 and expected_ids <= replayed_ids:
        payload["status"] = "ok"
    else:
        payload["status"] = "failed"
        payload["error"] = (
            f"GUI smoke replayed_steps={summary.replayed_steps}, "
            f"replayed_vehicle_ids={sorted(replayed_ids)}, expected={sorted(expected_ids)}"
        )
    _write_json(Path(status_path), payload)
    return payload


def _select_scenarios(selector: str) -> tuple[str, ...]:
    if selector == "all":
        return tuple(BASIC_ROLE_MAP)
    if selector not in BASIC_SCENARIO_IDS:
        known = ", ".join(BASIC_SCENARIO_IDS)
        raise ValueError(f"unknown BASIC scenario {selector!r}; expected one of: {known}")
    if selector not in BASIC_ROLE_MAP:
        known = ", ".join(sorted(BASIC_ROLE_MAP))
        raise ValueError(f"unsupported BASIC SUMO replay scenario {selector!r}; expected one of: {known}")
    return (selector,)


def _needs_visual_hint(record: Mapping[str, Any], *, control_activation_x_global: float) -> bool:
    return (
        str(record.get("road_role")) in {"on_ramp", "on_ramp_mv"}
        and str(record.get("physical_lane")) == "on_ramp"
        and float(record["x_global"]) < control_activation_x_global
    )


def _augment_pre_control_replay_checks(
    replay_fidelity: dict[str, Any],
    replay_records: list[dict[str, Any]],
    *,
    scenario_id: str,
) -> None:
    expectation = get_basic_expectation(scenario_id)
    mv_id = expectation.mv_id
    first_step_records = [record for record in replay_records if int(record["step"]) == 0]
    mv_step0 = next((record for record in first_step_records if record["vehicle_id"] == mv_id), None)
    mv_records = [record for record in replay_records if record["vehicle_id"] == mv_id]
    pre_control_records = [
        record
        for record in mv_records
        if float(record["x_global"]) < expectation.control_activation_x_global
    ]
    replay_fidelity["basic_visual_checks"] = {
        "step0_vehicle_count": len(first_step_records),
        "step0_mv_present": mv_step0 is not None,
        "step0_mv_x_global": mv_step0["x_global"] if mv_step0 is not None else None,
        "step0_mv_y": mv_step0["y"] if mv_step0 is not None else None,
        "pre_control_mv_record_count": len(pre_control_records),
        "pre_control_hint_count": sum(1 for record in pre_control_records if "visual_replay_hint" in record),
        "mv_y_range": _range([float(record["y"]) for record in mv_records]),
        "final_mv_merge_state": mv_records[-1]["merge_state"] if mv_records else None,
        "final_mv_physical_lane": mv_records[-1]["physical_lane"] if mv_records else None,
    }


def _scenario_manifest(
    *,
    run_id: str,
    scenario_id: str,
    scenario_dir: Path,
    status: str,
    max_steps: int,
    simulation: SimulationLoopResult,
    numeric_summary: dict[str, Any],
    replay_fidelity: dict[str, Any],
    gui_smoke_status: dict[str, Any],
    paths: dict[str, Path | None],
) -> dict[str, Any]:
    expectation = get_basic_expectation(scenario_id)
    role_map = BASIC_ROLE_MAP[scenario_id]
    command = f'& "{(scenario_dir / "play_gui_replay.ps1").resolve()}"'
    t_values = [float(record.t) for record in simulation.history.trajectory_records]
    return {
        "artifact_schema": "basic_sumo_replay_scenario.v1",
        "generated_at": datetime.now(UTC).replace(microsecond=0).isoformat(),
        "git_commit": _git_commit(),
        "run_id": run_id,
        "scenario_id": scenario_id,
        "status": status,
        "max_steps": max_steps,
        "actual_steps": numeric_summary["actual_steps"],
        "t_range": [min(t_values), max(t_values)] if t_values else None,
        "track_vehicle_id": expectation.mv_id,
        "role_map": dict(role_map),
        "role_color_legend": _role_color_legend(role_map),
        "expectation": expectation.to_dict(),
        "numeric_summary": numeric_summary,
        "numeric_gate_status": numeric_summary["status"],
        "replay_fidelity": replay_fidelity,
        "gui_smoke_status": gui_smoke_status,
        "vehicle_ranges": _vehicle_ranges(simulation.history.trajectory_records),
        "manual_replay_command": command,
        "boundary_statement": {
            "kind": "SUMO-GUI replay",
            "not_sumo_native_simulation": True,
            "pre_control_segment": (
                "pre-control segment 6450 -> 6650 is numeric-simulation-only in the current P17 map. "
                "SUMO replay uses a visual edge/lane hint for those records and becomes visually authoritative "
                "after the MV enters ramp_pre at x >= 6650."
            ),
            "scope": "Internal trajectory replay; it does not replace P17 true closed-loop TraCI authority.",
        },
        "paths": {key: (str(value) if value is not None else None) for key, value in paths.items()},
    }


def _root_manifest(
    *,
    run_id: str,
    output_dir: Path,
    status: str,
    max_steps: int,
    scenario_results: tuple[BasicSumoReplayScenarioResult, ...],
    validate_gui_smoke: bool,
) -> dict[str, Any]:
    paths = ensure_sumo_tools_on_path()
    return {
        "artifact_schema": "basic_sumo_replay_root.v1",
        "generated_at": datetime.now(UTC).replace(microsecond=0).isoformat(),
        "git_commit": _git_commit(),
        "run_id": run_id,
        "status": status,
        "output_root": str(output_dir),
        "max_steps": max_steps,
        "sumo_binary_paths": {
            "sumo_home": paths.sumo_home,
            "sumo": paths.sumo,
            "sumo_gui": paths.sumo_gui,
            "netconvert": paths.netconvert,
            "version": _safe_sumo_version(paths.sumo),
        },
        "validate_gui_smoke": validate_gui_smoke,
        "boundary_statement": {
            "basic_sumo_replay": (
                "This is a SUMO-GUI replay of CORMC internal numeric trajectories, "
                "not a SUMO-native closed-loop simulation."
            ),
            "p17": "P17 true closed-loop TraCI trajectory authority remains the SUMO-native evidence path.",
        },
        "scenarios": [result.to_dict() for result in scenario_results],
    }


def _root_status(
    scenario_results: Iterable[BasicSumoReplayScenarioResult],
    *,
    validate_gui_smoke: bool,
) -> str:
    results = list(scenario_results)
    if any(result.numeric_gate_status != "passed" for result in results):
        return "failed"
    if any(result.replay_fidelity_status != "passed" for result in results):
        return "failed"
    if validate_gui_smoke and any(result.gui_smoke_status != "ok" for result in results):
        return "failed"
    return "passed"


def _scenario_status(
    numeric_status: str,
    replay_fidelity_status: str,
    gui_smoke_status: str,
    *,
    validate_gui_smoke: bool,
) -> str:
    if numeric_status != "passed" or replay_fidelity_status != "passed":
        return "failed"
    if validate_gui_smoke and gui_smoke_status != "ok":
        return "failed"
    return "passed"


def _write_root_report(path: Path, manifest: dict[str, Any]) -> None:
    command_lines = [
        result["gui_replay_script_path"]
        for result in manifest["scenarios"]
        if result.get("gui_replay_script_path")
    ]
    lines = [
        f"# BASIC SUMO Replay Report: {manifest['run_id']}",
        "",
        f"- status: `{manifest['status']}`",
        f"- generated_at: `{manifest['generated_at']}`",
        f"- git_commit: `{manifest['git_commit']}`",
        f"- sumo: `{manifest['sumo_binary_paths']['sumo']}`",
        f"- sumo_gui: `{manifest['sumo_binary_paths']['sumo_gui']}`",
        f"- sumo_version: `{manifest['sumo_binary_paths']['version']}`",
        "",
        "## Boundary",
        "",
        "- This is SUMO-GUI replay of internal CORMC trajectory records.",
        "- It is not SUMO-native closed-loop traffic behavior and does not replace P17 true closed-loop TraCI authority.",
        "- Do not use a bare `.sumocfg` launch as the replay entrypoint; use the scripts below.",
        "",
        "## Scenarios",
        "",
        "| scenario_id | numeric | replay fidelity | gui smoke |",
        "| --- | --- | --- | --- |",
    ]
    for result in manifest["scenarios"]:
        lines.append(
            f"| `{result['scenario_id']}` | `{result['numeric_gate_status']}` | "
            f"`{result['replay_fidelity_status']}` | `{result['gui_smoke_status']}` |"
        )
    lines.extend(["", "## Manual Replay Commands", "", "```powershell"])
    for script_path in command_lines:
        lines.append(f'& "{script_path}"')
    lines.extend(["```", ""])
    path.write_text("\n".join(lines), encoding="utf-8")


def _write_scenario_report(path: Path, manifest: dict[str, Any]) -> None:
    numeric = manifest["numeric_summary"]
    checks = manifest["replay_fidelity"].get("basic_visual_checks", {})
    command = manifest["manual_replay_command"]
    lines = [
        f"# BASIC SUMO Replay: {manifest['scenario_id']}",
        "",
        f"- scenario_id: `{manifest['scenario_id']}`",
        f"- status: `{manifest['status']}`",
        f"- actual_steps: `{manifest['actual_steps']}` / max `{manifest['max_steps']}`",
        f"- numeric_gate_status: `{manifest['numeric_gate_status']}`",
        f"- replay_fidelity_status: `{manifest['replay_fidelity']['status']}`",
        f"- gui_smoke_status: `{manifest['gui_smoke_status']['status']}`",
        f"- expected APS case: `{numeric['expected_aps_case']}`",
        f"- observed APS case: `{numeric.get('observed_aps_case')}`",
        f"- expected active CVs: `{', '.join(numeric.get('expected_active_cv_ids') or []) or 'none'}`",
        f"- active CVs: `{', '.join(numeric.get('active_cv_ids') or []) or 'none'}`",
        f"- expected Eq.10 consumers: `{', '.join(numeric.get('expected_eq10_consumer_ids') or []) or 'none'}`",
        f"- Eq.10 consumers: `{', '.join(numeric.get('eq10_consumers') or []) or 'none'}`",
        f"- merged past ramp: `{numeric['merged_and_past_ramp']}`",
        "",
        "## Replay Checks",
        "",
        f"- checked_records: `{manifest['replay_fidelity'].get('checked_records', 0)}`",
        f"- step0_vehicle_count: `{checks.get('step0_vehicle_count')}`",
        f"- step0_mv_present: `{checks.get('step0_mv_present')}`",
        f"- step0_mv_x_global: `{checks.get('step0_mv_x_global')}`",
        f"- step0_mv_y: `{checks.get('step0_mv_y')}`",
        f"- pre_control_mv_record_count: `{checks.get('pre_control_mv_record_count')}`",
        f"- pre_control_hint_count: `{checks.get('pre_control_hint_count')}`",
        f"- final_mv_merge_state: `{checks.get('final_mv_merge_state')}`",
        f"- final_mv_physical_lane: `{checks.get('final_mv_physical_lane')}`",
        "",
        "## Key Vehicle Ranges",
        "",
        "| vehicle_id | x_range | y_range | lane_change_states | merge_states |",
        "| --- | --- | --- | --- | --- |",
    ]
    for vehicle_id, ranges in sorted(manifest["vehicle_ranges"].items()):
        lines.append(
            f"| `{vehicle_id}` | `{_fmt_range(ranges['x_min'], ranges['x_max'])}` | "
            f"`{_fmt_range(ranges['y_min'], ranges['y_max'])}` | "
            f"`{', '.join(ranges['lane_change_states'])}` | `{', '.join(ranges['merge_states'])}` |"
        )
    lines.extend(
        [
            "",
            "## Role And Color Legend",
            "",
            "| role | color_rgb |",
            "| --- | --- |",
        ]
    )
    for role, color in manifest["role_color_legend"].items():
        lines.append(f"| `{role}` | `{','.join(str(part) for part in color)}` |")
    lines.extend(
        [
            "",
            "## Boundary",
            "",
            f"- {manifest['boundary_statement']['pre_control_segment']}",
            f"- {manifest['boundary_statement']['scope']}",
            "- Use the replay script below; opening the `.sumocfg` directly will not play the trajectory JSONL.",
            "",
            "## Manual Command",
            "",
            "```powershell",
            command,
            "```",
            "",
            "## Smoke Verification",
            "",
            f"- status: `{manifest['gui_smoke_status']['status']}`",
            f"- sumo_gui_started: `{manifest['gui_smoke_status']['sumo_gui_started']}`",
            f"- replayed_steps: `{manifest['gui_smoke_status']['replayed_steps']}`",
            f"- replayed_vehicle_ids: `{manifest['gui_smoke_status']['replayed_vehicle_ids']}`",
            f"- closed_on_finish: `{manifest['gui_smoke_status']['closed_on_finish']}`",
            f"- error: `{manifest['gui_smoke_status']['error']}`",
            "",
        ]
    )
    path.write_text("\n".join(lines), encoding="utf-8")


def _write_gui_script(
    path: Path,
    *,
    sumocfg_file: Path,
    replay_path: Path,
    status_path: Path,
    track_vehicle_id: str,
) -> None:
    repo_root = Path(__file__).resolve().parents[2]
    python_exe = repo_root / ".venv" / "Scripts" / "python.exe"
    python_command = str(python_exe if python_exe.exists() else "python")
    module = "cormc.sumo.mvs_gui_replay"
    base = (
        f'& "{python_command}" -m {module} '
        f'--sumocfg "{sumocfg_file.resolve()}" '
        f'--replay "{replay_path.resolve()}" '
        f'--track-vehicle-id "{track_vehicle_id}" '
    )
    script = "\n".join(
        [
            "param([switch]$Smoke)",
            "$ErrorActionPreference = 'Stop'",
            "$env:PYTHONIOENCODING = 'utf-8'",
            f'Set-Location -LiteralPath "{repo_root}"',
            "if ($Smoke) {",
            f'  {base}--delay-ms 1 --hold-seconds 0 --post-roll-steps 0 --status-output "{status_path.resolve()}"',
            "  exit $LASTEXITCODE",
            "}",
            f'{base}--delay-ms {DEFAULT_GUI_DELAY_MS} --hold-seconds 0 --post-roll-steps 5 --keep-open-after-replay --status-output "{status_path.resolve()}"',
            "exit $LASTEXITCODE",
            "",
        ]
    )
    path.write_text(script, encoding="utf-8")


def _not_run_gui_smoke_status(scenario_id: str, scenario_dir: Path) -> dict[str, Any]:
    return {
        "status": "not_run",
        "sumocfg_file": str(scenario_dir / "sumo" / "p17.sumocfg"),
        "replay_jsonl": str(scenario_dir / "replay_trajectory.jsonl"),
        "sumo_gui_started": False,
        "replayed_steps": 0,
        "replayed_vehicle_ids": [],
        "closed_on_finish": False,
        "track_vehicle_id": get_basic_expectation(scenario_id).mv_id,
        "error": None,
    }


def _write_marker_file(path: Path) -> None:
    root = ET.Element("additional")
    ET.SubElement(
        root,
        "poi",
        id="basic_merge_start",
        type="merge_marker",
        color="0,90,220",
        x="6950.000",
        y="-7.000",
        layer="10",
    )
    ET.SubElement(
        root,
        "poi",
        id="basic_merge_end",
        type="merge_marker",
        color="245,140,0",
        x="7250.000",
        y="-7.000",
        layer="10",
    )
    ET.SubElement(
        root,
        "poly",
        id="basic_pre_control_note",
        type="numeric_pre_control_segment",
        color="80,80,80",
        fill="false",
        layer="2",
        shape="6450.000,-8.000 6650.000,-8.000 6650.000,-1.000 6450.000,-1.000",
    )
    _write_xml(path, root)


def _patch_sumocfg_additional_file(sumocfg_file: Path, additional_file_name: str) -> None:
    tree = ET.parse(sumocfg_file)
    root = tree.getroot()
    input_node = root.find("input")
    if input_node is None:
        input_node = ET.SubElement(root, "input")
    additional_node = input_node.find("additional-files")
    if additional_node is None:
        ET.SubElement(input_node, "additional-files", value=additional_file_name)
    else:
        additional_node.attrib["value"] = additional_file_name
    ET.indent(tree, space="  ")
    tree.write(sumocfg_file, encoding="utf-8", xml_declaration=True)


def _vehicle_ranges(records: Iterable[Any]) -> dict[str, dict[str, Any]]:
    grouped: dict[str, list[Any]] = {}
    for record in records:
        grouped.setdefault(str(record.vehicle_id), []).append(record)
    ranges: dict[str, dict[str, Any]] = {}
    for vehicle_id, items in grouped.items():
        ranges[vehicle_id] = {
            "x_min": min(float(record.x_global) for record in items),
            "x_max": max(float(record.x_global) for record in items),
            "y_min": min(float(record.y) for record in items),
            "y_max": max(float(record.y) for record in items),
            "v_min": min(float(record.v) for record in items),
            "v_max": max(float(record.v) for record in items),
            "lane_change_states": sorted({str(record.lane_change_state) for record in items}),
            "merge_states": sorted({str(record.merge_state) for record in items}),
            "physical_lanes": sorted({str(record.physical_lane) for record in items}),
            "road_roles": sorted({str(record.road_role) for record in items}),
        }
    return ranges


def _role_color_legend(role_map: Mapping[str, str]) -> dict[str, list[int]]:
    roles = set(role_map.values()) | {"background"}
    return {role: list(ROLE_COLORS.get(role, ROLE_COLORS["background"])) for role in sorted(roles)}


def _range(values: list[float]) -> list[float] | None:
    if not values:
        return None
    return [min(values), max(values)]


def _fmt_range(low: float, high: float) -> str:
    return f"{low:.3f} -> {high:.3f}"


def _safe_sumo_version(sumo_binary: str | None) -> str | None:
    try:
        return get_sumo_version(sumo_binary)
    except Exception:
        return None


def _git_commit() -> str | None:
    try:
        completed = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            check=True,
            capture_output=True,
            encoding="utf-8",
        )
    except Exception:
        return None
    return completed.stdout.strip()


def _write_json(path: Path, payload: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(_to_plain(payload), ensure_ascii=False, indent=2, sort_keys=True), encoding="utf-8")


def _write_xml(path: Path, root: ET.Element) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tree = ET.ElementTree(root)
    ET.indent(tree, space="  ")
    tree.write(path, encoding="utf-8", xml_declaration=True)


def _to_plain(value: Any) -> Any:
    if hasattr(value, "to_dict"):
        return value.to_dict()
    if isinstance(value, Mapping):
        return {str(key): _to_plain(item) for key, item in value.items()}
    if isinstance(value, tuple):
        return [_to_plain(item) for item in value]
    if isinstance(value, list):
        return [_to_plain(item) for item in value]
    return value


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Build BASIC numeric and SUMO replay artifacts.")
    parser.add_argument("--output-root", default=DEFAULT_OUTPUT_ROOT)
    parser.add_argument("--run-id", default=DEFAULT_RUN_ID)
    parser.add_argument("--scenario", default="BASIC-01")
    parser.add_argument("--max-steps", type=int, default=900)
    parser.add_argument("--validate-gui-smoke", action="store_true")
    args = parser.parse_args(argv)

    result = run_basic_sumo_replay_artifacts(
        output_root=args.output_root,
        run_id=args.run_id,
        scenario=args.scenario,
        max_steps=args.max_steps,
        validate_gui_smoke=args.validate_gui_smoke,
    )
    print(json.dumps(result.to_dict(), ensure_ascii=False, indent=2, sort_keys=True))
    return 0 if result.status == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
