from __future__ import annotations

import argparse
import json
import subprocess
from collections import Counter, defaultdict
from dataclasses import asdict, dataclass
from datetime import UTC, datetime
from pathlib import Path
from typing import Any, Iterable
from xml.etree import ElementTree as ET

from cormc.mvs.loader import load_builtin_scenario
from cormc.mvs.runner import DETERMINISTIC_SCENARIO_ROUTES
from cormc.p11_output import (
    export_event_history,
    export_sanity_history,
    export_trajectory_history,
    serialize_trajectory_record,
)
from cormc.simulation_loop import SimulationLoopConfig, SimulationLoopResult, run_deterministic_simulation
from cormc.sumo.env import ensure_sumo_tools_on_path
from cormc.sumo.gui_replay import DEFAULT_GUI_DELAY_MS
from cormc.sumo.mvs_gui_replay import MvsGuiReplaySummary, run_mvs_gui_replay
from cormc.sumo.mvs_replay_specs import (
    P171ReplaySpec,
    iter_p17_1_replay_specs,
)
from cormc.sumo.network import P17SumoNetworkConfig, build_p17_sumo_network


DEFAULT_RUN_ID = "p17_1_mvs_replay"
DEFAULT_OUTPUT_ROOT = "artifacts/sumo/p17_1_mvs_replay"
MARKER_FILE_NAME = "p17_1.markers.add.xml"


ROLE_COLORS: dict[str, tuple[int, int, int]] = {
    "mv_on_ramp_active": (0, 90, 220),
    "clv": (0, 160, 90),
    "cfv": (245, 140, 0),
    "cfv_active_cooperative": (245, 140, 0),
    "active_cooperative_cv": (245, 140, 0),
    "tlv": (120, 80, 220),
    "tfv": (0, 170, 180),
    "support": (120, 80, 220),
    "background": (160, 160, 160),
}

EXPECTED_LANE_CENTERLINES: dict[str, float] = {
    "ramp_pre_0": -3.5,
    "main_pre_0": 0.0,
    "main_pre_1": 3.5,
    "merge_zone_0": -3.5,
    "merge_zone_1": 0.0,
    "merge_zone_2": 3.5,
    "main_post_0": 0.0,
    "main_post_1": 3.5,
}


@dataclass(frozen=True)
class P171ScenarioArtifactResult:
    replay_id: str
    source_scenario_id: str
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
class P171ArtifactRunResult:
    run_id: str
    output_root: str
    status: str
    report_path: str
    artifact_manifest_path: str
    scenario_results: tuple[P171ScenarioArtifactResult, ...]

    def to_dict(self) -> dict[str, Any]:
        payload = asdict(self)
        payload["scenario_results"] = [result.to_dict() for result in self.scenario_results]
        return payload


def run_p17_1_mvs_replay_artifacts(
    *,
    output_root: str | Path = DEFAULT_OUTPUT_ROOT,
    run_id: str = DEFAULT_RUN_ID,
    scenario: str = "all",
    validate_gui_smoke: bool = False,
) -> P171ArtifactRunResult:
    output_dir = Path(output_root)
    output_dir.mkdir(parents=True, exist_ok=True)
    scenarios_dir = output_dir / "scenarios"
    scenarios_dir.mkdir(parents=True, exist_ok=True)

    scenario_results: list[P171ScenarioArtifactResult] = []
    for spec in iter_p17_1_replay_specs(scenario):
        scenario_results.append(
            build_p17_1_scenario_artifact(
                spec,
                scenarios_dir / spec.replay_id,
                run_id=run_id,
                validate_gui_smoke=validate_gui_smoke,
            )
        )

    status = _root_status(scenario_results, validate_gui_smoke=validate_gui_smoke)
    manifest_path = output_dir / "artifact_manifest.json"
    report_path = output_dir / "report.md"
    manifest = _root_manifest(
        run_id=run_id,
        output_dir=output_dir,
        status=status,
        scenario_results=tuple(scenario_results),
        validate_gui_smoke=validate_gui_smoke,
    )
    _write_json(manifest_path, manifest)
    _write_root_report(report_path, manifest)

    return P171ArtifactRunResult(
        run_id=run_id,
        output_root=str(output_dir),
        status=status,
        report_path=str(report_path),
        artifact_manifest_path=str(manifest_path),
        scenario_results=tuple(scenario_results),
    )


def build_p17_1_scenario_artifact(
    spec: P171ReplaySpec,
    scenario_dir: str | Path,
    *,
    run_id: str = DEFAULT_RUN_ID,
    validate_gui_smoke: bool = False,
) -> P171ScenarioArtifactResult:
    out = Path(scenario_dir)
    out.mkdir(parents=True, exist_ok=True)

    simulation = run_deterministic_simulation(
        SimulationLoopConfig(
            scenario_id=spec.source_scenario_id,
            run_id=spec.replay_id,
            max_steps=spec.replay_max_steps,
            output_dir=out,
            render_png=False,
        )
    )

    trajectory_path = export_trajectory_history(simulation.history, out / "trajectory.csv")
    events_path = export_event_history(simulation.history, out / "events.jsonl")
    sanity_path = export_sanity_history(simulation.history, out / "sanity.jsonl")

    numeric_summary = build_numeric_summary(spec, simulation)
    numeric_summary_path = out / "numeric_summary.json"
    _write_json(numeric_summary_path, numeric_summary)

    replay_path: Path | None = None
    sumocfg_path: Path | None = None
    gui_script_path: Path | None = None
    replay_fidelity = {"status": "skipped", "reason": "numeric gate failed"}
    lane_centerline_check = {"status": "skipped", "reason": "numeric gate failed"}

    gui_smoke_status_path = out / "gui_smoke_status.json"
    gui_smoke_payload = _not_run_gui_smoke_status(spec, out)
    _write_json(gui_smoke_status_path, gui_smoke_payload)

    if numeric_summary["numeric_gate_status"] == "passed":
        replay_path = out / "replay_trajectory.jsonl"
        replay_records = write_replay_trajectory_jsonl(
            replay_path,
            spec=spec,
            simulation=simulation,
        )
        replay_fidelity = verify_replay_fidelity(simulation, replay_records)

        sumo_files = build_p17_1_sumo_replay_files(
            out / "sumo",
            spec=spec,
            simulation=simulation,
        )
        sumocfg_path = Path(sumo_files["sumocfg_file"])
        lane_centerline_check = check_lane_centerlines(Path(sumo_files["net_file"]))
        gui_script_path = out / "play_gui_replay.ps1"
        _write_gui_script(
            gui_script_path,
            sumocfg_file=sumocfg_path,
            replay_path=replay_path,
            status_path=gui_smoke_status_path,
            track_vehicle_id=spec.track_vehicle_id,
        )

        if validate_gui_smoke:
            gui_smoke_payload = validate_gui_smoke_for_scenario(
                sumocfg_file=sumocfg_path,
                replay_path=replay_path,
                status_path=gui_smoke_status_path,
                track_vehicle_id=spec.track_vehicle_id,
            )

    scenario_manifest_path = out / "artifact_manifest.json"
    scenario_report_path = out / "scenario_report.md"
    scenario_manifest = _scenario_manifest(
        spec=spec,
        run_id=run_id,
        scenario_dir=out,
        simulation=simulation,
        numeric_summary=numeric_summary,
        replay_fidelity=replay_fidelity,
        lane_centerline_check=lane_centerline_check,
        gui_smoke_status=gui_smoke_payload,
        paths={
            "trajectory_csv": trajectory_path,
            "events_jsonl": events_path,
            "sanity_jsonl": sanity_path,
            "numeric_summary_json": numeric_summary_path,
            "replay_trajectory_jsonl": replay_path,
            "scenario_report_md": scenario_report_path,
            "artifact_manifest_json": scenario_manifest_path,
            "play_gui_replay_ps1": gui_script_path,
            "gui_smoke_status_json": gui_smoke_status_path,
            "sumo_config": sumocfg_path,
        },
    )
    _write_json(scenario_manifest_path, scenario_manifest)
    _write_scenario_report(scenario_report_path, scenario_manifest)

    scenario_status = _scenario_status(
        numeric_summary["numeric_gate_status"],
        replay_fidelity["status"],
        gui_smoke_payload["status"],
        validate_gui_smoke=validate_gui_smoke,
    )
    return P171ScenarioArtifactResult(
        replay_id=spec.replay_id,
        source_scenario_id=spec.source_scenario_id,
        output_dir=str(out),
        status=scenario_status,
        numeric_gate_status=str(numeric_summary["numeric_gate_status"]),
        replay_fidelity_status=str(replay_fidelity["status"]),
        gui_smoke_status=str(gui_smoke_payload["status"]),
        trajectory_path=str(trajectory_path),
        events_path=str(events_path),
        sanity_path=str(sanity_path),
        numeric_summary_path=str(numeric_summary_path),
        replay_trajectory_path=str(replay_path) if replay_path is not None else None,
        scenario_report_path=str(scenario_report_path),
        artifact_manifest_path=str(scenario_manifest_path),
        gui_replay_script_path=str(gui_script_path.resolve()) if gui_script_path is not None else None,
        gui_smoke_status_path=str(gui_smoke_status_path),
        sumo_config_path=str(sumocfg_path) if sumocfg_path is not None else None,
    )


def build_numeric_summary(spec: P171ReplaySpec, simulation: SimulationLoopResult) -> dict[str, Any]:
    trajectory_records = list(simulation.history.trajectory_records)
    event_dicts = simulation.history.event_dicts()
    sanity_dicts = simulation.history.sanity_dicts()
    vehicle_ranges = _vehicle_ranges(trajectory_records)
    event_type_counts = Counter(str(event["event_type"]) for event in event_dicts)
    failures: list[dict[str, Any]] = []
    hits = _run_numeric_checks(
        spec,
        vehicle_ranges=vehicle_ranges,
        events=event_dicts,
        sanity=sanity_dicts,
        trajectory_records=trajectory_records,
        failures=failures,
    )
    t_values = [float(record.t) for record in trajectory_records]
    steps = sorted({int(record.step) for record in trajectory_records})
    return {
        "replay_id": spec.replay_id,
        "source_scenario_id": spec.source_scenario_id,
        "extended": True,
        "replay_max_steps": spec.replay_max_steps,
        "actual_steps": len(steps),
        "step_range": [steps[0], steps[-1]] if steps else None,
        "t_range": [min(t_values), max(t_values)] if t_values else None,
        "vehicle_ranges": vehicle_ranges,
        "event_type_counts": dict(sorted(event_type_counts.items())),
        "event_hits": hits,
        "numeric_gate_status": "passed" if not failures else "failed",
        "numeric_gate_failures": failures,
    }


def write_replay_trajectory_jsonl(
    path: str | Path,
    *,
    spec: P171ReplaySpec,
    simulation: SimulationLoopResult,
) -> list[dict[str, Any]]:
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    records: list[dict[str, Any]] = []
    with out.open("w", encoding="utf-8") as handle:
        for record in sorted(
            simulation.history.trajectory_records,
            key=lambda item: (int(item.step), str(item.vehicle_id)),
        ):
            vehicle_role = spec.role_map.get(record.vehicle_id, "background")
            payload = {
                "step": record.step,
                "t": record.t,
                "vehicle_id": record.vehicle_id,
                "vehicle_role": vehicle_role,
                "source_scenario_id": spec.source_scenario_id,
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
            records.append(payload)
            handle.write(json.dumps(payload, ensure_ascii=False, sort_keys=True) + "\n")
    return records


def verify_replay_fidelity(
    simulation: SimulationLoopResult,
    replay_records: Iterable[dict[str, Any]],
) -> dict[str, Any]:
    source_records = {
        (int(record.step), str(record.vehicle_id)): serialize_trajectory_record(record)
        for record in simulation.history.trajectory_records
    }
    replay_map = {
        (int(record["step"]), str(record["vehicle_id"])): record
        for record in replay_records
    }
    mismatches: list[dict[str, Any]] = []
    if set(source_records) != set(replay_map):
        missing = sorted(set(source_records) - set(replay_map))
        extra = sorted(set(replay_map) - set(source_records))
        mismatches.append({"check": "key_set", "missing": missing[:10], "extra": extra[:10]})
    for key, source in source_records.items():
        replay = replay_map.get(key)
        if replay is None:
            continue
        for field in ("x_global", "y", "v", "a", "physical_lane", "road_role"):
            if replay[field] != source[field]:
                mismatches.append(
                    {
                        "check": "field_exact_match",
                        "key": key,
                        "field": field,
                        "expected": source[field],
                        "actual": replay[field],
                    }
                )
                break
    return {
        "status": "passed" if not mismatches else "failed",
        "checked_records": len(source_records),
        "mismatches": mismatches,
    }


def build_p17_1_sumo_replay_files(
    output_dir: str | Path,
    *,
    spec: P171ReplaySpec,
    simulation: SimulationLoopResult,
) -> dict[str, str]:
    t_values = [float(record.t) for record in simulation.history.trajectory_records]
    end = (max(t_values) + 2.0) if t_values else 5.0
    network_files = build_p17_sumo_network(
        output_dir,
        P17SumoNetworkConfig(end=end),
    )
    sumo_dir = Path(output_dir)
    marker_path = sumo_dir / MARKER_FILE_NAME
    _write_marker_file(marker_path, spec=spec)
    _patch_sumocfg_additional_file(Path(network_files.sumocfg_file), marker_path.name)
    payload = network_files.to_dict()
    payload["marker_file"] = str(marker_path)
    return payload


def validate_gui_smoke_for_scenario(
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
    else:
        payload = summary.to_dict()
    _write_json(Path(status_path), payload)
    return payload


def check_lane_centerlines(net_file: str | Path, *, tolerance_m: float = 0.05) -> dict[str, Any]:
    root = ET.parse(net_file).getroot()
    observed: dict[str, float] = {}
    failures: list[dict[str, Any]] = []
    for lane in root.findall(".//lane"):
        lane_id = lane.attrib.get("id")
        if lane_id not in EXPECTED_LANE_CENTERLINES:
            continue
        y_values = [_shape_y(point) for point in lane.attrib.get("shape", "").split() if "," in point]
        if not y_values:
            failures.append({"lane_id": lane_id, "reason": "missing shape"})
            continue
        y_avg = sum(y_values) / len(y_values)
        observed[lane_id] = y_avg
        expected = EXPECTED_LANE_CENTERLINES[lane_id]
        if abs(y_avg - expected) > tolerance_m:
            failures.append(
                {
                    "lane_id": lane_id,
                    "expected_y": expected,
                    "observed_y": y_avg,
                    "tolerance_m": tolerance_m,
                }
            )
    missing = sorted(set(EXPECTED_LANE_CENTERLINES) - set(observed))
    for lane_id in missing:
        failures.append({"lane_id": lane_id, "reason": "missing lane"})
    return {
        "status": "passed" if not failures else "failed",
        "tolerance_m": tolerance_m,
        "observed_centerlines": observed,
        "expected_centerlines": dict(EXPECTED_LANE_CENTERLINES),
        "failures": failures,
    }


def _run_numeric_checks(
    spec: P171ReplaySpec,
    *,
    vehicle_ranges: dict[str, dict[str, Any]],
    events: list[dict[str, Any]],
    sanity: list[dict[str, Any]],
    trajectory_records: list[Any],
    failures: list[dict[str, Any]],
) -> dict[str, dict[str, Any]]:
    hits: dict[str, dict[str, Any]] = {}

    def require(check_name: str, passed: bool, *, vehicle_id: str | None, message: str, detail: Any = None) -> None:
        hits[check_name] = {"passed": passed, "detail": detail}
        if not passed:
            failures.append(
                {
                    "replay_id": spec.replay_id,
                    "vehicle_id": vehicle_id,
                    "check_name": check_name,
                    "message": message,
                }
            )

    if spec.replay_id == "MVS-E2E-1-extended":
        require(
            "mv_demo_y_from_ramp_to_mainline",
            _vehicle_reaches_y(vehicle_ranges, "MV_DEMO", low=-3.49, high=-0.05),
            vehicle_id="MV_DEMO",
            message="MV_DEMO did not move from y=-3.5 to y=0.0",
            detail=vehicle_ranges.get("MV_DEMO"),
        )
        require(
            "eq53_pass",
            _has_event(events, vehicle_id="MV_DEMO", event_type="CMC", reason="eq53_gap", payload={"eq53_pass": True}),
            vehicle_id="MV_DEMO",
            message="MV_DEMO did not hit CMC Eq.53 pass",
        )
        require(
            "merge_start",
            _has_event(events, vehicle_id="MV_DEMO", event_type="CMC", reason="merge_start"),
            vehicle_id="MV_DEMO",
            message="MV_DEMO did not emit merge_start",
        )
        require(
            "lateral_trajectory_completed",
            _has_lateral_completed(events, "MV_DEMO") or _vehicle_has_state(vehicle_ranges, "MV_DEMO", "merge_states", "merged"),
            vehicle_id="MV_DEMO",
            message="MV_DEMO lateral trajectory did not complete",
        )
        require(
            "no_cooperative_request",
            not _has_event(events, event_type="cooperative_request"),
            vehicle_id=None,
            message="MVS-E2E-1-extended unexpectedly emitted cooperative_request",
        )

    elif spec.replay_id == "MVS-CMC-1-extended":
        require(
            "step0_eq53_pass",
            _has_event(events, step=0, vehicle_id="MV_CMC_1", event_type="CMC", reason="eq53_gap", payload={"eq53_pass": True}),
            vehicle_id="MV_CMC_1",
            message="MV_CMC_1 did not hit Eq.53 pass at step 0",
        )
        require(
            "step0_merge_start",
            _has_event(events, step=0, vehicle_id="MV_CMC_1", event_type="CMC", reason="merge_start"),
            vehicle_id="MV_CMC_1",
            message="MV_CMC_1 did not start merge at step 0",
        )
        require(
            "mv_cmc_1_y_from_ramp_to_mainline",
            _vehicle_reaches_y(vehicle_ranges, "MV_CMC_1", low=-3.49, high=-0.05),
            vehicle_id="MV_CMC_1",
            message="MV_CMC_1 did not complete y=-3.5 to y=0.0",
            detail=vehicle_ranges.get("MV_CMC_1"),
        )

    elif spec.replay_id == "MVS-CUC-1A-lanechange":
        require(
            "cfv_x_y_from_lane2_to_lane1",
            _vehicle_reaches_y(vehicle_ranges, "CFV_X", low=0.05, high=3.49, start_low=True),
            vehicle_id="CFV_X",
            message="CFV_X did not move from y=0.0 to y=3.5",
            detail=vehicle_ranges.get("CFV_X"),
        )
        require(
            "final_choice_change_to_lane_1",
            _has_event(events, vehicle_id="CFV_X", reason="final_choice_change_to_lane_1"),
            vehicle_id="CFV_X",
            message="CFV_X did not emit final_choice_change_to_lane_1",
        )
        require(
            "lane_change_command_created",
            _has_event(events, vehicle_id="CFV_X", reason="lane_change_command_created"),
            vehicle_id="CFV_X",
            message="CFV_X did not create lane-change command",
        )
        require(
            "same_step_overlay",
            _has_event(events, vehicle_id="CFV_X", reason="same_step_cuc_lane_change_relation_overlay")
            or _has_event(events, vehicle_id="CFV_X", event_type="lateral_trajectory", payload={"same_step_overlay_consumed": True}),
            vehicle_id="CFV_X",
            message="CFV_X did not consume same-step overlay",
        )
        require(
            "lateral_trajectory_completed",
            _has_lateral_completed(events, "CFV_X") or _vehicle_has_state(vehicle_ranges, "CFV_X", "lane_change_states", "normal"),
            vehicle_id="CFV_X",
            message="CFV_X lateral lane-change trajectory did not complete",
        )

    elif spec.replay_id == "MVS-CUC-2-eq10-window":
        cfv = vehicle_ranges.get("CFV_X", {})
        require(
            "cfv_x_stays_lane2",
            cfv.get("y_min", 999.0) >= -0.05 and cfv.get("y_max", -999.0) <= 0.05,
            vehicle_id="CFV_X",
            message="CFV_X y left lane_2 tolerance",
            detail=cfv,
        )
        require(
            "cfv_x_lane_change_state_normal",
            set(cfv.get("lane_change_states", ())) == {"normal"},
            vehicle_id="CFV_X",
            message="CFV_X lane_change_state was not normal for the whole Eq.10 window",
            detail=cfv.get("lane_change_states"),
        )
        require(
            "eq10_spacing_override_consumed",
            _has_event(events, vehicle_id="CFV_X", event_type="spacing_override_consumption", payload={"desired_spacing_source": "Eq10"}),
            vehicle_id="CFV_X",
            message="CFV_X did not consume Eq.10 spacing override",
        )
        require(
            "no_cfv_x_lateral_trajectory",
            not _has_event(events, vehicle_id="CFV_X", event_type="lateral_trajectory"),
            vehicle_id="CFV_X",
            message="CFV_X unexpectedly emitted lateral_trajectory",
        )

    elif spec.replay_id == "MVS-SAFE-1B-cap":
        mv = vehicle_ranges.get("MV_SAFE_EXEC", {})
        require(
            "mv_safe_exec_remains_executing",
            set(mv.get("merge_states", ())) == {"executing"},
            vehicle_id="MV_SAFE_EXEC",
            message="MV_SAFE_EXEC did not remain executing",
            detail=mv.get("merge_states"),
        )
        require(
            "speed_cap_binding",
            _has_event(events, vehicle_id="MV_SAFE_EXEC", reason="boundary_speed_cap", payload={"cap_binding": True})
            or _has_event(events, vehicle_id="MV_SAFE_EXEC", event_type="speed_cap", payload={"most_conservative_source": "boundary_speed_cap"}),
            vehicle_id="MV_SAFE_EXEC",
            message="MV_SAFE_EXEC did not hit binding boundary speed cap",
        )
        require(
            "lateral_trajectory_consumes_speed_cap",
            _has_event(events, vehicle_id="MV_SAFE_EXEC", event_type="lateral_trajectory", payload_contains={"p08_constraints_applied": "boundary_speed_cap"}),
            vehicle_id="MV_SAFE_EXEC",
            message="MV_SAFE_EXEC lateral trajectory did not consume boundary-cap-constrained speed",
        )
        require(
            "no_merge_complete",
            not _vehicle_has_state(vehicle_ranges, "MV_SAFE_EXEC", "merge_states", "merged")
            and not _has_lateral_completed(events, "MV_SAFE_EXEC"),
            vehicle_id="MV_SAFE_EXEC",
            message="MV_SAFE_EXEC incorrectly completed merge",
        )

    elif spec.replay_id == "MVS-COMMIT-1-full-extended":
        require(
            "cv_active_lc_y_to_lane1",
            _vehicle_reaches_y(vehicle_ranges, "CV_ACTIVE_LC", high=3.49),
            vehicle_id="CV_ACTIVE_LC",
            message="CV_ACTIVE_LC did not reach lane_1 y=3.5",
            detail=vehicle_ranges.get("CV_ACTIVE_LC"),
        )
        require(
            "mv_active_merge_y_to_mainline",
            _vehicle_reaches_y(vehicle_ranges, "MV_ACTIVE_MERGE", high=-0.05),
            vehicle_id="MV_ACTIVE_MERGE",
            message="MV_ACTIVE_MERGE did not reach y=0.0",
            detail=vehicle_ranges.get("MV_ACTIVE_MERGE"),
        )
        require(
            "mv_cache_y_to_mainline",
            _vehicle_reaches_y(vehicle_ranges, "MV_CACHE", low=-3.49, high=-0.05),
            vehicle_id="MV_CACHE",
            message="MV_CACHE did not reach y=0.0",
            detail=vehicle_ranges.get("MV_CACHE"),
        )
        require(
            "commit_sanity_all_pass",
            not _blocking_sanity_failures(sanity),
            vehicle_id=None,
            message="Commit sanity checks include failures",
        )
        require(
            "unique_commit_per_vehicle_step",
            _unique_commit_per_vehicle_step(events),
            vehicle_id=None,
            message="A vehicle had more than one commit event in a step",
        )

    else:  # pragma: no cover - protected by registry lookup
        require("known_replay_id", False, vehicle_id=None, message=f"Unhandled replay id {spec.replay_id}")

    return hits


def _vehicle_ranges(records: Iterable[Any]) -> dict[str, dict[str, Any]]:
    grouped: dict[str, list[Any]] = defaultdict(list)
    for record in records:
        grouped[str(record.vehicle_id)].append(record)
    ranges: dict[str, dict[str, Any]] = {}
    for vehicle_id, items in sorted(grouped.items()):
        x_values = [float(record.x_global) for record in items]
        y_values = [float(record.y) for record in items]
        v_values = [float(record.v) for record in items]
        ranges[vehicle_id] = {
            "x_min": min(x_values),
            "x_max": max(x_values),
            "y_min": min(y_values),
            "y_max": max(y_values),
            "v_min": min(v_values),
            "v_max": max(v_values),
            "first_step": min(int(record.step) for record in items),
            "last_step": max(int(record.step) for record in items),
            "lane_change_states": sorted({str(record.lane_change_state) for record in items}),
            "merge_states": sorted({str(record.merge_state) for record in items}),
        }
    return ranges


def _vehicle_reaches_y(
    ranges: dict[str, dict[str, Any]],
    vehicle_id: str,
    *,
    low: float | None = None,
    high: float | None = None,
    start_low: bool = False,
) -> bool:
    vehicle = ranges.get(vehicle_id)
    if not vehicle:
        return False
    if low is not None:
        if start_low:
            if float(vehicle["y_min"]) > low:
                return False
        elif float(vehicle["y_min"]) > low:
            return False
    if high is not None and float(vehicle["y_max"]) < high:
        return False
    return True


def _vehicle_has_state(ranges: dict[str, dict[str, Any]], vehicle_id: str, field: str, state: str) -> bool:
    return state in set(ranges.get(vehicle_id, {}).get(field, ()))


def _has_event(
    events: Iterable[dict[str, Any]],
    *,
    step: int | None = None,
    vehicle_id: str | None = None,
    event_type: str | None = None,
    result: str | None = None,
    reason: str | None = None,
    payload: dict[str, Any] | None = None,
    payload_contains: dict[str, Any] | None = None,
) -> bool:
    for event in events:
        if step is not None and int(event.get("step", -1)) != step:
            continue
        if vehicle_id is not None and event.get("vehicle_id") != vehicle_id:
            continue
        if event_type is not None and event.get("event_type") != event_type:
            continue
        if result is not None and event.get("result") != result:
            continue
        if reason is not None and event.get("reason") != reason:
            continue
        event_payload = event.get("payload", {}) or {}
        if payload is not None and not all(event_payload.get(key) == value for key, value in payload.items()):
            continue
        if payload_contains is not None:
            matched = True
            for key, value in payload_contains.items():
                actual = event_payload.get(key)
                if isinstance(actual, (list, tuple, set)):
                    if value not in actual:
                        matched = False
                        break
                elif actual != value:
                    matched = False
                    break
            if not matched:
                continue
        return True
    return False


def _has_lateral_completed(events: Iterable[dict[str, Any]], vehicle_id: str) -> bool:
    for event in events:
        if event.get("vehicle_id") != vehicle_id or event.get("event_type") != "lateral_trajectory":
            continue
        payload = event.get("payload", {}) or {}
        if payload.get("completed") is True or event.get("result") in {"merge_complete", "lane_change_complete"}:
            return True
    return False


def _blocking_sanity_failures(sanity: Iterable[dict[str, Any]]) -> list[dict[str, Any]]:
    return [
        record
        for record in sanity
        if record.get("result") == "fail"
        and record.get("severity") in {"error", "critical", "blocker"}
    ]


def _unique_commit_per_vehicle_step(events: Iterable[dict[str, Any]]) -> bool:
    counts = Counter(
        (int(event.get("step", -1)), str(event.get("vehicle_id")))
        for event in events
        if event.get("event_type") == "commit" and event.get("vehicle_id") is not None
    )
    return all(count == 1 for count in counts.values())


def _write_marker_file(path: Path, *, spec: P171ReplaySpec) -> None:
    root = ET.Element("additional")
    ET.SubElement(
        root,
        "poi",
        id="merge_start",
        type="merge_marker",
        color="0,90,220",
        x="6950.000",
        y="-7.000",
        layer="10",
    )
    ET.SubElement(
        root,
        "poi",
        id="merge_end",
        type="merge_marker",
        color="245,140,0",
        x="7250.000",
        y="-7.000",
        layer="10",
    )
    ET.SubElement(
        root,
        "poly",
        id=f"{spec.replay_id}_local_replay_window",
        type="local_replay_window",
        color="60,60,60",
        fill="false",
        layer="2",
        shape="6650.000,-8.000 7350.000,-8.000 7350.000,6.000 6650.000,6.000",
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


def _shape_y(point: str) -> float:
    _, y = point.split(",", 1)
    return float(y)


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
            f"  {base}--delay-ms 1 --hold-seconds 0 --post-roll-steps 0 --status-output \"{status_path.resolve()}\"",
            "  exit $LASTEXITCODE",
            "}",
            f"{base}--delay-ms {DEFAULT_GUI_DELAY_MS} --hold-seconds 0 --post-roll-steps 5 --keep-open-after-replay --status-output \"{status_path.resolve()}\"",
            "exit $LASTEXITCODE",
            "",
        ]
    )
    path.write_text(script, encoding="utf-8")


def _not_run_gui_smoke_status(spec: P171ReplaySpec, scenario_dir: Path) -> dict[str, Any]:
    return {
        "status": "not_run",
        "sumocfg_file": str(scenario_dir / "sumo" / "p17.sumocfg"),
        "replay_jsonl": str(scenario_dir / "replay_trajectory.jsonl"),
        "sumo_gui_started": False,
        "replayed_steps": 0,
        "replayed_vehicle_ids": [],
        "closed_on_finish": False,
        "track_vehicle_id": spec.track_vehicle_id,
        "error": None,
    }


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


def _root_status(
    scenario_results: Iterable[P171ScenarioArtifactResult],
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


def _scenario_manifest(
    *,
    spec: P171ReplaySpec,
    run_id: str,
    scenario_dir: Path,
    simulation: SimulationLoopResult,
    numeric_summary: dict[str, Any],
    replay_fidelity: dict[str, Any],
    lane_centerline_check: dict[str, Any],
    gui_smoke_status: dict[str, Any],
    paths: dict[str, Path | None],
) -> dict[str, Any]:
    command = f'& "{(scenario_dir / "play_gui_replay.ps1").resolve()}"'
    return {
        "artifact_schema": "p17_1_mvs_replay_scenario.v1",
        "run_id": run_id,
        "replay_id": spec.replay_id,
        "source_scenario_id": spec.source_scenario_id,
        "extended": True,
        "required_suite_default_steps": _required_suite_default_steps(spec.source_scenario_id),
        "p17_1_replay_max_steps": spec.replay_max_steps,
        "actual_steps": numeric_summary["actual_steps"],
        "t_range": numeric_summary["t_range"],
        "track_vehicle_id": spec.track_vehicle_id,
        "primary_vehicle_ids": list(spec.primary_vehicle_ids),
        "role_map": dict(spec.role_map),
        "role_color_legend": _role_color_legend(spec),
        "numeric_summary": numeric_summary,
        "numeric_gate_status": numeric_summary["numeric_gate_status"],
        "replay_fidelity": replay_fidelity,
        "lane_centerline_check": lane_centerline_check,
        "gui_smoke_status": gui_smoke_status,
        "manual_replay_command": command,
        "what_this_validates": _what_this_validates(spec.replay_id),
        "what_this_does_not_validate": _what_this_does_not_validate(spec.replay_id),
        "simulation_status": simulation.status,
        "paths": {key: (str(value) if value is not None else None) for key, value in paths.items()},
    }


def _root_manifest(
    *,
    run_id: str,
    output_dir: Path,
    status: str,
    scenario_results: tuple[P171ScenarioArtifactResult, ...],
    validate_gui_smoke: bool,
) -> dict[str, Any]:
    paths = ensure_sumo_tools_on_path()
    return {
        "artifact_schema": "p17_1_mvs_replay_root.v1",
        "generated_at": datetime.now(UTC).replace(microsecond=0).isoformat(),
        "git_commit": _git_commit(),
        "run_id": run_id,
        "status": status,
        "output_root": str(output_dir),
        "sumo_binary_paths": {
            "sumo_home": paths.sumo_home,
            "sumo": paths.sumo,
            "sumo_gui": paths.sumo_gui,
            "netconvert": paths.netconvert,
        },
        "validate_gui_smoke": validate_gui_smoke,
        "boundary_statement": {
            "p14_p15": "P14/P15 are algorithm numeric evidence.",
            "p16": "P16 is seeded random internal simulation evidence.",
            "p17": "P17 remains the true SUMO closed-loop smoke chain.",
            "p17_1": "P17.1 is MVS artifact replay evidence that SUMO can show internal algorithm trajectories.",
        },
        "regression_tests": _regression_test_commands(),
        "scenarios": [result.to_dict() for result in scenario_results],
    }


def _required_suite_default_steps(source_scenario_id: str) -> int | None:
    route = DETERMINISTIC_SCENARIO_ROUTES.get(source_scenario_id)
    if route is None:
        return None
    return int(route["max_steps"])


def _role_color_legend(spec: P171ReplaySpec) -> dict[str, list[int]]:
    roles = set(spec.role_map.values()) | {"background"}
    return {role: list(ROLE_COLORS.get(role, ROLE_COLORS["background"])) for role in sorted(roles)}


def _what_this_validates(replay_id: str) -> str:
    return {
        "MVS-E2E-1-extended": "APS case 1, no CUC request, CMC Eq.53 pass, merge start, merge completion, and commit replay.",
        "MVS-CMC-1-extended": "CMC Eq.53 pass and immediate merge start/completion from an existing helper gate.",
        "MVS-CUC-1A-lanechange": "CUC choice 1, lane-change command, same-step overlay, and mainline lateral intervention.",
        "MVS-CUC-2-eq10-window": "CUC longitudinal intervention through Eq.10 spacing override while CFV_X stays in lane_2.",
        "MVS-SAFE-1B-cap": "Boundary speed cap consumption by an executing lateral trajectory.",
        "MVS-COMMIT-1-full-extended": "Active trajectory continuation, non-APS cache behavior, and one commit per vehicle per step.",
    }[replay_id]


def _what_this_does_not_validate(replay_id: str) -> str:
    common = "It is not SUMO-native traffic behavior and does not replace P17 true closed-loop validation."
    specific = {
        "MVS-E2E-1-extended": "It has no mainline cooperative intervention.",
        "MVS-CMC-1-extended": "It does not promote MVS-CMC-1 into the deterministic required route matrix.",
        "MVS-CUC-1A-lanechange": "It does not show a full CUC-to-CMC merge completion chain.",
        "MVS-CUC-2-eq10-window": "It is an Eq.10 short window, not a complete merge showcase.",
        "MVS-SAFE-1B-cap": "It is a boundary-cap showcase, not a merge-complete showcase.",
        "MVS-COMMIT-1-full-extended": "It does not add a new APS case 1/2/3/4 long end-to-end strong scenario.",
    }[replay_id]
    return f"{specific} {common}"


def _regression_test_commands() -> list[dict[str, str]]:
    command = (
        '& "D:\\PycharmProjects\\CORMC\\.venv\\Scripts\\python.exe" -m pytest '
        "tests/test_p14_artifact_bundle.py tests/test_p15_engine_core.py tests/test_p15_recording.py "
        "tests/test_p16_seeded_runner.py tests/test_p17_sumo_artifact.py tests/test_p17_sumo_executor.py "
        "tests/test_p17_sumo_gui_replay.py tests/test_p17_sumo_replay.py "
        "tests/test_p17_1_mvs_replay_specs.py tests/test_p17_1_mvs_replay_numeric.py "
        "tests/test_p17_1_sumo_network.py tests/test_p17_1_gui_replay.py tests/test_p17_1_artifacts.py"
    )
    return [{"name": "P14-P17.1 targeted regression", "command": command, "result": "not_run_by_artifact_cli"}]


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


def _write_root_report(path: Path, manifest: dict[str, Any]) -> None:
    command_lines = [
        result["gui_replay_script_path"]
        for result in manifest["scenarios"]
        if result.get("gui_replay_script_path")
    ]
    lines = [
        f"# P17.1 MVS Replay Report: {manifest['run_id']}",
        "",
        f"- status: `{manifest['status']}`",
        f"- generated_at: `{manifest['generated_at']}`",
        f"- git_commit: `{manifest['git_commit']}`",
        f"- sumo: `{manifest['sumo_binary_paths']['sumo']}`",
        f"- sumo_gui: `{manifest['sumo_binary_paths']['sumo_gui']}`",
        "",
        "## Boundary",
        "",
        "- P14/P15 are algorithm numeric evidence.",
        "- P16 is seeded random internal simulation evidence.",
        "- P17.1 is replay evidence that internal algorithm trajectories can be seen in SUMO.",
        "- P17 true closed-loop TraCI trajectory-authority code is not replaced by P17.1 replay code.",
        "- `MVS-E2E-1-extended` has no mainline vehicle intervention.",
        "- `MVS-CUC-2-eq10-window` is an Eq.10 short window, not a complete merge showcase.",
        "- `MVS-SAFE-1B-cap` is a boundary-cap showcase, not a merge-complete showcase.",
        "- Do not use a bare `.sumocfg` launch as the replay entrypoint; use the scripts below.",
        "",
        "## Scenarios",
        "",
        "| replay_id | source | numeric | replay fidelity | gui smoke |",
        "| --- | --- | --- | --- | --- |",
    ]
    for result in manifest["scenarios"]:
        lines.append(
            f"| `{result['replay_id']}` | `{result['source_scenario_id']}` | "
            f"`{result['numeric_gate_status']}` | `{result['replay_fidelity_status']}` | "
            f"`{result['gui_smoke_status']}` |"
        )
    lines.extend(["", "## Manual Replay Commands", "", "```powershell"])
    for script_path in command_lines:
        lines.append(f'& "{script_path}"')
    lines.extend(["```", ""])
    path.write_text("\n".join(lines), encoding="utf-8")


def _write_scenario_report(path: Path, manifest: dict[str, Any]) -> None:
    numeric = manifest["numeric_summary"]
    command = manifest["manual_replay_command"]
    lines = [
        f"# P17.1 Scenario Replay: {manifest['replay_id']}",
        "",
        f"- replay_id: `{manifest['replay_id']}`",
        f"- source_scenario_id: `{manifest['source_scenario_id']}`",
        f"- extended: `{str(manifest['extended']).lower()}`",
        f"- required_suite_default_steps: `{manifest['required_suite_default_steps']}`",
        f"- p17_1_replay_max_steps: `{manifest['p17_1_replay_max_steps']}`",
        f"- actual_steps: `{manifest['actual_steps']}`",
        f"- t_range: `{manifest['t_range']}`",
        f"- track_vehicle_id: `{manifest['track_vehicle_id']}`",
        f"- numeric_gate_status: `{manifest['numeric_gate_status']}`",
        f"- replay_fidelity_status: `{manifest['replay_fidelity']['status']}`",
        f"- gui_smoke_status: `{manifest['gui_smoke_status']['status']}`",
        "",
        "## Key Vehicle Ranges",
        "",
        "| vehicle_id | x_range | y_range | lane_change_states | merge_states |",
        "| --- | --- | --- | --- | --- |",
    ]
    for vehicle_id, ranges in sorted(numeric["vehicle_ranges"].items()):
        lines.append(
            f"| `{vehicle_id}` | `{_fmt_range(ranges['x_min'], ranges['x_max'])}` | "
            f"`{_fmt_range(ranges['y_min'], ranges['y_max'])}` | "
            f"`{', '.join(ranges['lane_change_states'])}` | `{', '.join(ranges['merge_states'])}` |"
        )
    lines.extend(["", "## Key Event Hits", "", "| check | status |", "| --- | --- |"])
    for check, hit in numeric["event_hits"].items():
        lines.append(f"| `{check}` | `{'passed' if hit['passed'] else 'failed'}` |")
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
            "## Lane Centerline Check",
            "",
            f"- status: `{manifest['lane_centerline_check']['status']}`",
            "",
            "## Replay Fidelity Check",
            "",
            f"- status: `{manifest['replay_fidelity']['status']}`",
            f"- checked_records: `{manifest['replay_fidelity'].get('checked_records', 0)}`",
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
            f"- closed_on_finish: `{manifest['gui_smoke_status']['closed_on_finish']}`",
            f"- error: `{manifest['gui_smoke_status']['error']}`",
            "",
            "## Scope",
            "",
            f"- validates: {manifest['what_this_validates']}",
            f"- does_not_validate: {manifest['what_this_does_not_validate']}",
            "",
        ]
    )
    path.write_text("\n".join(lines), encoding="utf-8")


def _fmt_range(low: float, high: float) -> str:
    return f"{low:.3f} -> {high:.3f}"


def _write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True), encoding="utf-8")


def _write_xml(path: Path, root: ET.Element) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tree = ET.ElementTree(root)
    ET.indent(tree, space="  ")
    tree.write(path, encoding="utf-8", xml_declaration=True)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Build P17.1 MVS numeric and SUMO replay artifacts.")
    parser.add_argument("--output-root", default=DEFAULT_OUTPUT_ROOT)
    parser.add_argument("--run-id", default=DEFAULT_RUN_ID)
    parser.add_argument("--scenario", default="all")
    parser.add_argument("--validate-gui-smoke", action="store_true")
    args = parser.parse_args(argv)

    result = run_p17_1_mvs_replay_artifacts(
        output_root=args.output_root,
        run_id=args.run_id,
        scenario=args.scenario,
        validate_gui_smoke=args.validate_gui_smoke,
    )
    print(json.dumps(result.to_dict(), ensure_ascii=False, indent=2, sort_keys=True))
    return 0 if result.status == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
