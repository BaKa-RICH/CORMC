from __future__ import annotations

import json
from pathlib import Path
from xml.etree import ElementTree as ET

from cormc.sumo import P17SumoArtifactResult, run_p17_sumo_artifact_bundle
from cormc.sumo.env import ensure_sumo_available_or_skip


def test_p17_sumo_artifact_bundle_runs_real_closed_loop_and_writes_formal_files(tmp_path: Path) -> None:
    ensure_sumo_available_or_skip()

    result = run_p17_sumo_artifact_bundle(
        run_id="p17-artifact-smoke",
        output_root=tmp_path,
        seed=16001,
        max_steps=45,
    )

    assert isinstance(result, P17SumoArtifactResult)
    out = tmp_path / "p17-artifact-smoke"
    sumo_dir = out / "sumo"
    expected_paths = [
        out / "trajectory.csv",
        out / "events.jsonl",
        out / "sanity.jsonl",
        out / "realization.jsonl",
        out / "artifact_manifest.json",
        out / "scenario_report.json",
        out / "run_report.md",
        out / "play_gui_replay.ps1",
        sumo_dir / "p17.sumocfg",
        sumo_dir / "p17.traci.sumocfg",
        sumo_dir / "p17.preview.rou.xml",
        sumo_dir / "p17.net.xml",
        sumo_dir / "p17.rou.xml",
        sumo_dir / "p17.nod.xml",
        sumo_dir / "p17.edg.xml",
        sumo_dir / "p17.con.xml",
    ]

    for path in expected_paths:
        assert path.exists(), path

    assert (out / "realization.jsonl").read_text(encoding="utf-8").strip()
    assert not (tmp_path / "baseline").exists()
    assert not (tmp_path / "random" / "p16_seeded_demo").exists()


def test_p17_sumo_artifact_manifest_and_report_include_required_public_fields(tmp_path: Path) -> None:
    ensure_sumo_available_or_skip()

    result = run_p17_sumo_artifact_bundle(
        run_id="p17-artifact-manifest",
        output_root=tmp_path,
        seed=16001,
        max_steps=45,
    )

    manifest = json.loads(Path(result.manifest_path).read_text(encoding="utf-8"))
    for key in (
        "run_id",
        "scenario_id",
        "status",
        "sumo_version",
        "sumo_home",
        "executor_mode",
        "bitmasks",
        "keepRoute",
        "speed_mode_bitset",
        "lane_change_mode_bitset",
        "move_to_xy_keep_route",
        "step_length",
        "lateral_resolution",
        "collision_action",
        "seed",
        "profile_id",
        "max_steps",
        "active_ids",
        "background_sample",
        "active_controlled_vehicle_ids",
        "background_vehicle_ids_sample",
        "generated_count",
        "blocked_count",
        "blocked_spawn_count",
        "collision_count",
        "teleport_count",
        "mismatch_count",
        "realization_mismatch_count",
        "output_paths",
        "sumo_paths",
        "gui_replay",
    ):
        assert key in manifest

    assert manifest["run_id"] == "p17-artifact-manifest"
    assert manifest["scenario_id"] == "P17-SUMO-CLOSED-LOOP"
    assert manifest["executor_mode"] == "move_to_xy_trajectory_authority"
    assert manifest["bitmasks"]["speed_mode"] == 0
    assert manifest["bitmasks"]["lane_change_mode"] == 2560
    assert manifest["keepRoute"] == 3
    assert manifest["speed_mode_bitset"] == 0
    assert manifest["lane_change_mode_bitset"] == 2560
    assert manifest["move_to_xy_keep_route"] == 3
    assert manifest["generated_count"] >= 1
    assert manifest["blocked_spawn_count"] == manifest["blocked_count"]
    assert manifest["realization_mismatch_count"] == manifest["mismatch_count"]
    assert manifest["active_ids"]
    assert manifest["background_sample"]
    assert manifest["active_controlled_vehicle_ids"] == manifest["active_ids"]
    assert manifest["background_vehicle_ids_sample"] == manifest["background_sample"]
    assert manifest["output_paths"]["gui_replay_script_ps1"].endswith("play_gui_replay.ps1")
    assert manifest["gui_replay"]["entrypoint"] == manifest["output_paths"]["gui_replay_script_ps1"]
    assert manifest["gui_replay"]["mode"] == "traci_recorded_trajectory_replay"
    assert manifest["gui_replay"]["wait_for_enter"] is False
    assert manifest["gui_replay"]["keep_open_after_replay"] is True
    assert manifest["gui_replay"]["sumo_config"].endswith("p17.traci.sumocfg")
    assert manifest["direct_sumo_preview"]["sumo_config"].endswith("p17.sumocfg")
    assert manifest["direct_sumo_preview"]["route_file"].endswith("p17.preview.rou.xml")
    assert manifest["direct_sumo_preview"]["mode"] == "static_preview_not_controller_exact"

    preview_sumocfg = ET.parse(manifest["direct_sumo_preview"]["sumo_config"]).getroot()
    preview_values = {child.tag: child.attrib["value"] for section in preview_sumocfg for child in section}
    assert preview_values["route-files"] == "p17.preview.rou.xml"
    assert preview_values["end"] == "120.0"

    preview_routes = ET.parse(manifest["direct_sumo_preview"]["route_file"]).getroot()
    preview_vehicles = preview_routes.findall("vehicle")
    preview_vehicle_ids = {vehicle.attrib["id"] for vehicle in preview_vehicles}
    assert "preview_MV_ACTIVE" in preview_vehicle_ids
    assert "preview_BG_0" in preview_vehicle_ids
    preview_departs = [float(vehicle.attrib["depart"]) for vehicle in preview_vehicles]
    assert preview_departs == sorted(preview_departs)

    report = Path(result.report_path).read_text(encoding="utf-8")
    assert "sumo-gui" in report
    assert "-c" in report
    assert "play_gui_replay.ps1" in report
    assert "p17.sumocfg" in report
    assert "Opening `" in report
    assert "120-second static SUMO preview" in report
    assert "p17.traci.sumocfg" in report
    assert "P17 does not do the P18 paper grid" in report

    replay_script = Path(manifest["output_paths"]["gui_replay_script_ps1"]).read_text(encoding="utf-8")
    assert "python -m cormc.sumo.gui_replay" in replay_script
    assert "--keep-open-after-replay" in replay_script
    assert "--sumocfg" in replay_script
    assert "--realization" in replay_script
    assert "p17.traci.sumocfg" in replay_script


def test_p17_sumo_artifact_result_exposes_plan_level_public_fields(tmp_path: Path) -> None:
    ensure_sumo_available_or_skip()

    result = run_p17_sumo_artifact_bundle(
        run_id="p17-artifact-result-fields",
        output_root=tmp_path,
        seed=16001,
        max_steps=45,
    )

    assert result.scenario_id == "P17-SUMO-CLOSED-LOOP"
    assert result.status == result.simulation_result.status
    assert result.sumo_config_path and result.sumo_config_path.endswith("p17.sumocfg")
    assert result.artifact_manifest_path == result.manifest_path
    assert result.run_report_path == result.report_path
    assert result.realization_path and result.realization_path.endswith("realization.jsonl")
    assert result.trajectory_path and result.trajectory_path.endswith("trajectory.csv")
    assert result.events_path and result.events_path.endswith("events.jsonl")
    assert result.sanity_path and result.sanity_path.endswith("sanity.jsonl")
    assert result.generated_count >= 1
    assert result.blocked_spawn_count >= 0
    assert result.collision_count == 0
    assert result.teleport_count == 0
    assert result.realization_mismatch_count == 0
    assert result.active_controlled_vehicle_ids
