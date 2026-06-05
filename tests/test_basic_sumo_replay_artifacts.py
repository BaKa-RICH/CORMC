from __future__ import annotations

import json
from pathlib import Path

import pytest

from cormc.sumo import mvs_gui_replay
from cormc.sumo.basic_replay_artifacts import (
    main,
    run_basic_sumo_replay_artifacts,
)
from cormc.sumo.mapping import to_sumo_position


def test_basic_01_sumo_replay_artifact_keeps_full_visual_pre_control(tmp_path: Path) -> None:
    result = run_basic_sumo_replay_artifacts(
        output_root=tmp_path,
        run_id="basic01_test",
        scenario="BASIC-01",
        max_steps=900,
    )

    assert result.status == "passed"
    assert len(result.scenario_results) == 1
    scenario = result.scenario_results[0]
    assert scenario.status == "passed"
    assert scenario.numeric_gate_status == "passed"
    assert scenario.replay_fidelity_status == "passed"
    assert scenario.gui_smoke_status == "not_run"

    scenario_dir = Path(scenario.output_dir)
    for name in (
        "trajectory.csv",
        "events.jsonl",
        "sanity.jsonl",
        "numeric_summary.json",
        "replay_trajectory.jsonl",
        "scenario_report.md",
        "artifact_manifest.json",
        "play_gui_replay.ps1",
        "gui_smoke_status.json",
    ):
        assert (scenario_dir / name).exists(), name
    assert (scenario_dir / "sumo" / "p17.sumocfg").exists()
    assert (scenario_dir / "sumo" / "p17.net.xml").exists()
    assert (scenario_dir / "sumo" / "p17.rou.xml").exists()

    replay_records = [
        json.loads(line)
        for line in (scenario_dir / "replay_trajectory.jsonl").read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    step0 = [record for record in replay_records if int(record["step"]) == 0]
    assert {record["vehicle_id"] for record in step0} == {
        "B01_MV",
        "B01_CLV",
        "B01_CFV",
        "B01_TLV_CFV",
    }
    step0_mv = next(record for record in step0 if record["vehicle_id"] == "B01_MV")
    assert step0_mv["x_global"] < 6650
    assert step0_mv["y"] == pytest.approx(-3.5)
    assert step0_mv["visual_replay_hint"]["mode"] == "allow_pre_control_on_ramp"
    assert step0_mv["vehicle_role"] == "mv_on_ramp_active"

    mv_records = [record for record in replay_records if record["vehicle_id"] == "B01_MV"]
    assert min(float(record["y"]) for record in mv_records) == pytest.approx(-3.5)
    assert max(float(record["y"]) for record in mv_records) == pytest.approx(0.0)
    assert mv_records[-1]["merge_state"] == "merged" or mv_records[-1]["physical_lane"] == "lane_2"

    manifest = json.loads(Path(scenario.artifact_manifest_path).read_text(encoding="utf-8"))
    assert manifest["status"] == "passed"
    assert manifest["numeric_summary"]["observed_aps_case"] == "case_2"
    assert manifest["numeric_summary"]["active_cv_ids"] == ["B01_CFV"]
    assert manifest["numeric_summary"]["eq10_consumers"] == ["B01_CFV"]
    assert manifest["numeric_summary"]["merged_and_past_ramp"] is True
    assert manifest["replay_fidelity"]["status"] == "passed"
    assert manifest["replay_fidelity"]["basic_visual_checks"]["step0_mv_present"] is True
    assert manifest["gui_smoke_status"]["status"] == "not_run"
    for artifact_path in manifest["paths"].values():
        assert artifact_path is None or Path(artifact_path).exists(), artifact_path

    report = Path(scenario.scenario_report_path).read_text(encoding="utf-8")
    assert "BASIC-01" in report
    assert "case_2" in report
    assert "B01_CFV" in report
    assert "merged" in report
    assert "Use the replay script" in report

    root_report = Path(result.report_path).read_text(encoding="utf-8")
    assert "Do not use a bare `.sumocfg` launch as the replay entrypoint" in root_report
    assert scenario.gui_replay_script_path is not None
    assert f'& "{scenario.gui_replay_script_path}"' in root_report


def test_basic_01_cli_function_writes_artifact(tmp_path: Path) -> None:
    exit_code = main(
        [
            "--scenario",
            "BASIC-01",
            "--run-id",
            "basic01_cli_test",
            "--output-root",
            str(tmp_path),
        ]
    )

    assert exit_code == 0
    assert (tmp_path / "scenarios" / "BASIC-01" / "replay_trajectory.jsonl").exists()
    assert (tmp_path / "report.md").exists()


def test_basic_replay_visual_hint_does_not_weaken_strict_mapping() -> None:
    with pytest.raises(ValueError, match="outside ramp_pre range"):
        to_sumo_position(6642.04, "on_ramp", "on_ramp_mv")

    record = {
        "x_global": 6642.04,
        "physical_lane": "on_ramp",
        "road_role": "on_ramp_mv",
        "visual_replay_hint": {
            "mode": "allow_pre_control_on_ramp",
            "edge_id": "ramp_pre",
            "lane_index": 0,
        },
    }

    assert mvs_gui_replay._sumo_position_for_replay_record(
        record,
        6642.04,
        "on_ramp",
        "on_ramp_mv",
    ) == ("ramp_pre", 0, 0.0)
