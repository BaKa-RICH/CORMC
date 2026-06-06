from __future__ import annotations

import argparse
import csv
import json
import subprocess
from dataclasses import asdict, dataclass
from datetime import UTC, datetime
from pathlib import Path
from typing import Any, Iterable, Mapping
from xml.etree import ElementTree as ET

from cormc.basic_scenarios import BASIC_SCENARIO_IDS, get_basic_expectation
from cormc.step9_11 import OutputHistory, TrajectoryRecord
from cormc.sumo.env import ensure_sumo_tools_on_path, get_sumo_version
from cormc.sumo.gui_replay import DEFAULT_GUI_DELAY_MS
from cormc.sumo.mapping import RAMP_UPSTREAM_START_X
from cormc.sumo.mvs_gui_replay import run_mvs_gui_replay
from cormc.sumo.mvs_replay_artifacts import ROLE_COLORS, verify_replay_fidelity
from cormc.sumo.network import P17SumoNetworkConfig, build_p17_sumo_network


DEFAULT_RUN_ID = "basic01_sumo_replay"
DEFAULT_OUTPUT_ROOT = "artifacts/sumo/basic_replay"
VISUAL_HINT_MODE = "allow_pre_control_on_ramp"

BASIC_ROLE_COLORS: dict[str, tuple[int, int, int]] = {
    **ROLE_COLORS,
    "clv_active_cooperative": ROLE_COLORS["clv"],
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


@dataclass(frozen=True)
class ImportedBasicNumericArtifact:
    scenario_id: str
    source_artifact_dir: str
    trajectory_path: Path
    events_path: Path | None
    sanity_path: Path | None
    numeric_summary_path: Path
    numeric_summary: dict[str, Any]
    history: OutputHistory
    lifecycle_summary: dict[str, Any]


@dataclass(frozen=True)
class ImportedBasicSimulation:
    history: OutputHistory
    scenario_id: str
    run_id: str
    status: str = "imported_from_artifact"


def run_basic_sumo_replay_artifacts(
    *,
    source_artifact_dir: str | Path,
    output_root: str | Path = DEFAULT_OUTPUT_ROOT,
    run_id: str = DEFAULT_RUN_ID,
    scenario: str = "BASIC-01",
    max_steps: int | None = None,
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
            source_artifact_dir=source_artifact_dir,
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
    source_artifact_dir: str | Path,
    scenario_dir: str | Path,
    run_id: str = DEFAULT_RUN_ID,
    max_steps: int | None = None,
    validate_gui_smoke: bool = False,
) -> BasicSumoReplayScenarioResult:
    out = Path(scenario_dir)
    out.mkdir(parents=True, exist_ok=True)

    source = import_basic_numeric_artifact(
        scenario_id=scenario_id,
        source_artifact_dir=source_artifact_dir,
    )
    simulation = ImportedBasicSimulation(
        history=source.history,
        scenario_id=source.scenario_id,
        run_id=run_id,
    )
    numeric_summary = dict(source.numeric_summary)
    numeric_gate_status = str(numeric_summary["status"])
    effective_max_steps = int(numeric_summary.get("max_steps") or max_steps or numeric_summary.get("actual_steps") or 0)

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
            source=source,
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
                expected_vehicle_ids=source_vehicle_ids(source),
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
        max_steps=effective_max_steps,
        source=source,
        simulation=simulation,
        numeric_summary=numeric_summary,
        replay_fidelity=replay_fidelity,
        gui_smoke_status=gui_smoke_payload,
        paths={
            "source_artifact_dir": Path(source.source_artifact_dir),
            "trajectory_csv": source.trajectory_path,
            "events_jsonl": source.events_path,
            "sanity_jsonl": source.sanity_path,
            "numeric_summary_json": source.numeric_summary_path,
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
        trajectory_path=str(source.trajectory_path),
        events_path=str(source.events_path) if source.events_path is not None else "",
        sanity_path=str(source.sanity_path) if source.sanity_path is not None else "",
        numeric_summary_path=str(source.numeric_summary_path),
        replay_trajectory_path=str(replay_path) if replay_path is not None else None,
        scenario_report_path=str(scenario_report_path),
        artifact_manifest_path=str(scenario_manifest_path),
        gui_replay_script_path=str(gui_script_path.resolve()) if gui_script_path is not None else None,
        gui_smoke_status_path=str(gui_smoke_status_path),
        sumo_config_path=str(sumocfg_path) if sumocfg_path is not None else None,
    )


def import_basic_numeric_artifact(
    *,
    scenario_id: str,
    source_artifact_dir: str | Path,
) -> ImportedBasicNumericArtifact:
    if scenario_id not in BASIC_SCENARIO_IDS:
        known = ", ".join(BASIC_SCENARIO_IDS)
        raise ValueError(f"unknown BASIC scenario {scenario_id!r}; expected one of: {known}")

    source_dir = Path(source_artifact_dir)
    if not source_dir.is_dir():
        raise FileNotFoundError(f"source artifact directory does not exist: {source_dir}")

    trajectory_path = source_dir / "trajectory.csv"
    numeric_summary_path = source_dir / "numeric_summary.json"
    events_path = source_dir / "events.jsonl"
    sanity_path = source_dir / "sanity.jsonl"
    for required in (trajectory_path, numeric_summary_path):
        if not required.exists():
            raise FileNotFoundError(f"source artifact directory is missing required file: {required}")

    numeric_summary = json.loads(numeric_summary_path.read_text(encoding="utf-8"))
    summary_scenario = str(numeric_summary.get("scenario_id", ""))
    if summary_scenario != scenario_id:
        raise ValueError(
            f"--scenario {scenario_id!r} does not match numeric_summary.json scenario_id {summary_scenario!r}"
        )
    if str(numeric_summary.get("status")) != "passed":
        raise ValueError(f"source numeric_summary.json status must be 'passed', got {numeric_summary.get('status')!r}")

    history = OutputHistory(trajectory_records=_read_trajectory_csv(trajectory_path, scenario_id=scenario_id))
    if not history.trajectory_records:
        raise ValueError(f"source trajectory.csv contains no records: {trajectory_path}")

    lifecycle_summary = _summarize_basic_lifecycle_events(
        events_path if events_path.exists() else None,
        numeric_summary=numeric_summary,
    )
    return ImportedBasicNumericArtifact(
        scenario_id=scenario_id,
        source_artifact_dir=str(source_dir),
        trajectory_path=trajectory_path,
        events_path=events_path if events_path.exists() else None,
        sanity_path=sanity_path if sanity_path.exists() else None,
        numeric_summary_path=numeric_summary_path,
        numeric_summary=numeric_summary,
        history=history,
        lifecycle_summary=lifecycle_summary,
    )


def source_vehicle_ids(source: ImportedBasicNumericArtifact) -> tuple[str, ...]:
    return tuple(sorted({str(record.vehicle_id) for record in source.history.trajectory_records}))


def build_basic_role_map(
    *,
    scenario_id: str,
    vehicle_ids: Iterable[str],
    numeric_summary: Mapping[str, Any],
) -> dict[str, str]:
    expectation = get_basic_expectation(scenario_id)
    active_cv_ids = {str(vehicle_id) for vehicle_id in numeric_summary.get("active_cv_ids", ())}
    role_map: dict[str, str] = {}
    for vehicle_id in vehicle_ids:
        base_role = _base_role_for_basic_vehicle(vehicle_id, mv_id=expectation.mv_id)
        if vehicle_id in active_cv_ids:
            if base_role == "clv":
                base_role = "clv_active_cooperative"
            elif base_role == "cfv":
                base_role = "cfv_active_cooperative"
            elif base_role == "background":
                base_role = "active_cooperative_cv"
        role_map[vehicle_id] = base_role
    return role_map


def _base_role_for_basic_vehicle(vehicle_id: str, *, mv_id: str) -> str:
    if vehicle_id == mv_id or vehicle_id.endswith("_MV"):
        return "mv_on_ramp_active"
    if "_TLV_" in vehicle_id or vehicle_id.endswith("_TLV"):
        return "tlv"
    if "_TFV_" in vehicle_id or vehicle_id.endswith("_TFV"):
        return "tfv"
    if vehicle_id.endswith("_CLV"):
        return "clv"
    if vehicle_id.endswith("_CFV"):
        return "cfv"
    return "background"


def _read_trajectory_csv(path: Path, *, scenario_id: str) -> list[TrajectoryRecord]:
    records: list[TrajectoryRecord] = []
    with path.open(encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        for row_number, row in enumerate(reader, start=2):
            row_scenario = str(row.get("scenario_id", ""))
            if row_scenario != scenario_id:
                raise ValueError(
                    f"--scenario {scenario_id!r} does not match trajectory.csv scenario_id "
                    f"{row_scenario!r} at row {row_number}"
                )
            records.append(
                TrajectoryRecord(
                    run_id=str(row.get("run_id", "")),
                    scenario_id=row_scenario,
                    step=int(row["step"]),
                    t=float(row["t"]),
                    vehicle_id=str(row["vehicle_id"]),
                    vehicle_type=str(row.get("vehicle_type") or ""),
                    compliance_state=str(row.get("compliance_state") or ""),
                    x_global=float(row["x_global"]),
                    y=float(row["y"]),
                    v=float(row["v"]),
                    a=float(row["a"]),
                    physical_lane=str(row.get("physical_lane") or ""),
                    road_role=str(row.get("road_role") or ""),
                    primary_leader_id=str(row.get("primary_leader_id") or "") or None,
                    lane_change_state=str(row.get("lane_change_state") or "normal"),
                    merge_state=str(row.get("merge_state") or "none"),
                    active_event_tags=_split_event_tags(row.get("active_event_tags")),
                )
            )
    return records


def _split_event_tags(value: str | None) -> tuple[str, ...]:
    if not value:
        return ()
    return tuple(part for part in str(value).split("|") if part)


def _summarize_basic_lifecycle_events(
    events_path: Path | None,
    *,
    numeric_summary: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    numeric_summary = numeric_summary or {}
    if events_path is None:
        return {
            "status": "not_available",
            **_numeric_assignment_lifecycle_fields(numeric_summary),
        }

    summary: dict[str, Any] = {
        "status": "available",
        "refresh_failed_retained_count": 0,
        "cooperative_request_vehicle_ids": [],
        "cuc_stay_lane_2_vehicle_ids": [],
        "cmc_recovery_front_only": False,
        "cmc_recovery_leader_id": None,
        "cmc_recovery_step": None,
        **_numeric_assignment_lifecycle_fields(numeric_summary),
    }
    cooperative_ids: set[str] = set()
    stay_lane_ids: set[str] = set()
    for line in events_path.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        event = json.loads(line)
        payload = event.get("payload", {}) or {}
        reason = str(event.get("reason", ""))
        event_type = str(event.get("event_type", ""))
        vehicle_id = event.get("vehicle_id")
        if payload.get("lifecycle_state") == "refresh_failed_retained":
            summary["refresh_failed_retained_count"] += 1
        if event_type == "cooperative_request" and vehicle_id:
            cooperative_ids.add(str(vehicle_id))
        if event_type == "CUC" and payload.get("final_choice") == "stay_lane_2" and vehicle_id:
            stay_lane_ids.add(str(vehicle_id))
        if reason == "cmc_recovery_current_gap" and payload.get("gap_type") == "front_only":
            summary["cmc_recovery_front_only"] = True
            summary["cmc_recovery_leader_id"] = payload.get("leader_id")
            summary["cmc_recovery_step"] = event.get("step")
    summary["cooperative_request_vehicle_ids"] = sorted(cooperative_ids)
    summary["cuc_stay_lane_2_vehicle_ids"] = sorted(stay_lane_ids)
    return summary


def _numeric_assignment_lifecycle_fields(numeric_summary: Mapping[str, Any]) -> dict[str, Any]:
    return {
        "bounded_assignment_merge_success": bool(
            numeric_summary.get("bounded_assignment_merge_success", False)
        ),
        "used_front_only_recovery_for_success": bool(
            numeric_summary.get("used_front_only_recovery_for_success", False)
        ),
        "merge_success_assignment_source": numeric_summary.get("merge_success_assignment_source"),
        "merge_success_gap_type": numeric_summary.get("merge_success_gap_type"),
        "merge_success_clv_id": numeric_summary.get("merge_success_clv_id"),
        "merge_success_cfv_id": numeric_summary.get("merge_success_cfv_id"),
    }


def write_basic_replay_trajectory_jsonl(
    path: str | Path,
    *,
    scenario_id: str,
    source: ImportedBasicNumericArtifact,
) -> list[dict[str, Any]]:
    role_map = build_basic_role_map(
        scenario_id=scenario_id,
        vehicle_ids=source_vehicle_ids(source),
        numeric_summary=source.numeric_summary,
    )
    expectation = get_basic_expectation(scenario_id)
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    records: list[dict[str, Any]] = []
    with out.open("w", encoding="utf-8") as handle:
        for record in sorted(
            source.history.trajectory_records,
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
                "color_rgb": list(BASIC_ROLE_COLORS.get(vehicle_role, ROLE_COLORS["background"])),
            }
            if _needs_visual_hint(payload, control_activation_x_global=expectation.control_activation_x_global):
                payload["visual_replay_hint"] = {
                    "mode": VISUAL_HINT_MODE,
                    "edge_id": "ramp_upstream",
                    "lane_index": 0,
                    "reason": "pre-control on-ramp record is before current P17 ramp_upstream edge start",
                }
            records.append(payload)
            handle.write(json.dumps(payload, ensure_ascii=False, sort_keys=True) + "\n")
    return records


def build_basic_sumo_replay_files(
    output_dir: str | Path,
    *,
    simulation: ImportedBasicSimulation,
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
    expected_vehicle_ids: Iterable[str],
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
    expected_ids = set(expected_vehicle_ids)
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
        raise ValueError("BASIC replay artifact import requires one --scenario and one --source-artifact-dir")
    if selector not in BASIC_SCENARIO_IDS:
        known = ", ".join(BASIC_SCENARIO_IDS)
        raise ValueError(f"unknown BASIC scenario {selector!r}; expected one of: {known}")
    return (selector,)


def _needs_visual_hint(record: Mapping[str, Any], *, control_activation_x_global: float) -> bool:
    _ = control_activation_x_global
    return (
        str(record.get("road_role")) in {"on_ramp", "on_ramp_mv"}
        and str(record.get("physical_lane")) == "on_ramp"
        and float(record["x_global"]) < RAMP_UPSTREAM_START_X
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
    source: ImportedBasicNumericArtifact,
    simulation: ImportedBasicSimulation,
    numeric_summary: dict[str, Any],
    replay_fidelity: dict[str, Any],
    gui_smoke_status: dict[str, Any],
    paths: dict[str, Path | None],
) -> dict[str, Any]:
    expectation = get_basic_expectation(scenario_id)
    role_map = build_basic_role_map(
        scenario_id=scenario_id,
        vehicle_ids=source_vehicle_ids(source),
        numeric_summary=numeric_summary,
    )
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
        "source_artifact_dir": source.source_artifact_dir,
        "track_vehicle_id": expectation.mv_id,
        "role_map": dict(role_map),
        "role_color_legend": _role_color_legend(role_map),
        "expectation": expectation.to_dict(),
        "numeric_summary": numeric_summary,
        "numeric_gate_status": numeric_summary["status"],
        "lifecycle_summary": source.lifecycle_summary,
        "replay_fidelity": replay_fidelity,
        "gui_smoke_status": gui_smoke_status,
        "vehicle_ranges": _vehicle_ranges(simulation.history.trajectory_records),
        "manual_replay_command": command,
        "boundary_statement": {
            "kind": "SUMO-GUI replay",
            "not_sumo_native_simulation": True,
            "pre_control_segment": (
                "pre-control segment 6450 -> 6650 is visible through the ramp_upstream SUMO edge; "
                "records below 6450 remain numeric-only and use a replay hint fallback."
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
    max_steps: int | None,
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
    lifecycle = manifest.get("lifecycle_summary", {})
    checks = manifest["replay_fidelity"].get("basic_visual_checks", {})
    command = manifest["manual_replay_command"]
    lines = [
        f"# BASIC SUMO Replay: {manifest['scenario_id']}",
        "",
        f"- scenario_id: `{manifest['scenario_id']}`",
        f"- source_artifact_dir: `{manifest['source_artifact_dir']}`",
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
        f"- refresh failed retained count: `{lifecycle.get('refresh_failed_retained_count', 0)}`",
        f"- cooperative request vehicles: `{', '.join(lifecycle.get('cooperative_request_vehicle_ids') or []) or 'none'}`",
        f"- CUC stay lane_2 vehicles: `{', '.join(lifecycle.get('cuc_stay_lane_2_vehicle_ids') or []) or 'none'}`",
        f"- bounded assignment merge success: `{lifecycle.get('bounded_assignment_merge_success')}`",
        f"- merge success gap type: `{lifecycle.get('merge_success_gap_type')}`",
        f"- merge success CLV/CFV: `{lifecycle.get('merge_success_clv_id')}` / `{lifecycle.get('merge_success_cfv_id')}`",
        f"- used front-only recovery for success: `{lifecycle.get('used_front_only_recovery_for_success')}`",
        f"- CMC recovery front-only: `{lifecycle.get('cmc_recovery_front_only')}`",
        f"- CMC recovery leader: `{lifecycle.get('cmc_recovery_leader_id')}`",
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
    parser = argparse.ArgumentParser(description="Import BASIC numeric artifacts and build SUMO replay artifacts.")
    parser.add_argument("--source-artifact-dir", required=True)
    parser.add_argument("--output-root", default=DEFAULT_OUTPUT_ROOT)
    parser.add_argument("--run-id", default=DEFAULT_RUN_ID)
    parser.add_argument("--scenario", default="BASIC-01")
    parser.add_argument("--max-steps", type=int)
    parser.add_argument("--validate-gui-smoke", action="store_true")
    args = parser.parse_args(argv)

    result = run_basic_sumo_replay_artifacts(
        source_artifact_dir=args.source_artifact_dir,
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
