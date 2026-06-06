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


BASIC01_SOURCE = Path("artifacts/basic/basic_01_06_900_bugcheck_20260605/scenarios/BASIC-01")
BASIC02_SOURCE = Path("artifacts/basic/basic02_case3_mv_clv_relation_900/BASIC-02")


def test_basic_01_sumo_replay_imports_existing_numeric_artifact(tmp_path: Path) -> None:
    result = run_basic_sumo_replay_artifacts(
        source_artifact_dir=BASIC01_SOURCE,
        output_root=tmp_path,
        run_id="basic01_import_test",
        scenario="BASIC-01",
    )

    assert result.status == "passed"
    assert len(result.scenario_results) == 1
    scenario = result.scenario_results[0]
    assert scenario.status == "passed"
    assert scenario.numeric_gate_status == "passed"
    assert scenario.replay_fidelity_status == "passed"
    assert scenario.gui_smoke_status == "not_run"
    assert Path(scenario.trajectory_path) == BASIC01_SOURCE / "trajectory.csv"

    scenario_dir = Path(scenario.output_dir)
    for name in (
        "replay_trajectory.jsonl",
        "scenario_report.md",
        "artifact_manifest.json",
        "play_gui_replay.ps1",
        "gui_smoke_status.json",
    ):
        assert (scenario_dir / name).exists(), name
    assert not (scenario_dir / "trajectory.csv").exists()
    assert not (scenario_dir / "numeric_summary.json").exists()
    assert (scenario_dir / "sumo" / "p17.sumocfg").exists()
    assert (scenario_dir / "sumo" / "p17.net.xml").exists()
    assert (scenario_dir / "sumo" / "p17.rou.xml").exists()

    replay_records = _read_replay_records(scenario_dir / "replay_trajectory.jsonl")
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
    assert "visual_replay_hint" not in step0_mv
    assert step0_mv["vehicle_role"] == "mv_on_ramp_active"

    roles = {record["vehicle_id"]: record["vehicle_role"] for record in step0}
    assert roles["B01_CFV"] == "cfv_active_cooperative"
    assert roles["B01_CLV"] == "clv"
    assert roles["B01_TLV_CFV"] == "tlv"

    manifest = json.loads(Path(scenario.artifact_manifest_path).read_text(encoding="utf-8"))
    assert manifest["status"] == "passed"
    assert manifest["source_artifact_dir"] == str(BASIC01_SOURCE)
    assert manifest["numeric_summary"]["observed_aps_case"] == "case_2"
    assert manifest["numeric_summary"]["active_cv_ids"] == ["B01_CFV"]
    assert manifest["numeric_summary"]["eq10_consumers"] == ["B01_CFV"]
    assert manifest["numeric_summary"]["merged_and_past_ramp"] is True
    assert manifest["role_map"]["B01_CFV"] == "cfv_active_cooperative"
    assert manifest["replay_fidelity"]["status"] == "passed"
    assert manifest["replay_fidelity"]["basic_visual_checks"]["step0_mv_present"] is True
    assert manifest["replay_fidelity"]["basic_visual_checks"]["pre_control_hint_count"] == 0
    assert manifest["gui_smoke_status"]["status"] == "not_run"
    for artifact_path in manifest["paths"].values():
        assert artifact_path is None or Path(artifact_path).exists(), artifact_path

    report = Path(scenario.scenario_report_path).read_text(encoding="utf-8")
    assert "BASIC-01" in report
    assert str(BASIC01_SOURCE) in report
    assert "case_2" in report
    assert "B01_CFV" in report
    assert "merged" in report
    assert "Use the replay script" in report

    root_report = Path(result.report_path).read_text(encoding="utf-8")
    assert "Do not use a bare `.sumocfg` launch as the replay entrypoint" in root_report
    assert scenario.gui_replay_script_path is not None
    assert f'& "{scenario.gui_replay_script_path}"' in root_report


def test_basic_02_sumo_replay_imports_assignment_lifecycle_artifact(tmp_path: Path) -> None:
    result = run_basic_sumo_replay_artifacts(
        source_artifact_dir=BASIC02_SOURCE,
        output_root=tmp_path,
        run_id="basic02_import_test",
        scenario="BASIC-02",
    )

    assert result.status == "passed"
    scenario = result.scenario_results[0]
    scenario_dir = Path(scenario.output_dir)
    replay_records = _read_replay_records(scenario_dir / "replay_trajectory.jsonl")
    step0 = [record for record in replay_records if int(record["step"]) == 0]
    assert {record["vehicle_id"] for record in step0} == {
        "B02_MV",
        "B02_CLV",
        "B02_CFV",
        "B02_TLV_CLV",
    }
    step0_mv = next(record for record in step0 if record["vehicle_id"] == "B02_MV")
    assert step0_mv["x_global"] < 6650
    assert step0_mv["y"] == pytest.approx(-3.5)
    assert "visual_replay_hint" not in step0_mv

    roles = {record["vehicle_id"]: record["vehicle_role"] for record in step0}
    assert roles["B02_MV"] == "mv_on_ramp_active"
    assert roles["B02_CLV"] == "clv_active_cooperative"
    assert roles["B02_CFV"] == "cfv"
    assert roles["B02_TLV_CLV"] == "tlv"

    manifest = json.loads(Path(scenario.artifact_manifest_path).read_text(encoding="utf-8"))
    assert manifest["numeric_summary"]["actual_steps"] == 221
    assert manifest["numeric_summary"]["observed_aps_case"] == "case_3"
    assert manifest["numeric_summary"]["active_cv_ids"] == ["B02_CLV"]
    assert manifest["numeric_summary"]["eq10_consumers"] == []
    assert manifest["numeric_summary"]["bounded_assignment_merge_success"] is True
    assert manifest["numeric_summary"]["used_front_only_recovery_for_success"] is False
    assert manifest["numeric_summary"]["merge_success_gap_type"] == "bounded"
    assert manifest["numeric_summary"]["merge_success_clv_id"] == "B02_CLV"
    assert manifest["numeric_summary"]["merge_success_cfv_id"] == "B02_CFV"
    assert manifest["numeric_summary"]["merged_and_past_ramp"] is True
    assert manifest["role_map"] == {
        "B02_CFV": "cfv",
        "B02_CLV": "clv_active_cooperative",
        "B02_MV": "mv_on_ramp_active",
        "B02_TLV_CLV": "tlv",
    }
    assert manifest["lifecycle_summary"]["refresh_failed_retained_count"] == 0
    assert manifest["lifecycle_summary"]["cooperative_request_vehicle_ids"] == ["B02_CLV"]
    assert manifest["lifecycle_summary"]["cuc_stay_lane_2_vehicle_ids"] == ["B02_CLV"]
    assert manifest["lifecycle_summary"]["bounded_assignment_merge_success"] is True
    assert manifest["lifecycle_summary"]["used_front_only_recovery_for_success"] is False
    assert manifest["lifecycle_summary"]["merge_success_gap_type"] == "bounded"
    assert manifest["lifecycle_summary"]["merge_success_clv_id"] == "B02_CLV"
    assert manifest["lifecycle_summary"]["merge_success_cfv_id"] == "B02_CFV"
    assert manifest["lifecycle_summary"]["cmc_recovery_front_only"] is False
    assert manifest["lifecycle_summary"]["cmc_recovery_leader_id"] is None
    assert manifest["replay_fidelity"]["status"] == "passed"
    assert manifest["replay_fidelity"]["basic_visual_checks"]["pre_control_hint_count"] == 0


def test_basic_replay_cli_requires_source_artifact_dir(tmp_path: Path) -> None:
    with pytest.raises(SystemExit) as excinfo:
        main(["--scenario", "BASIC-01", "--output-root", str(tmp_path)])

    assert excinfo.value.code == 2


def test_basic_01_cli_function_writes_imported_artifact(tmp_path: Path) -> None:
    exit_code = main(
        [
            "--scenario",
            "BASIC-01",
            "--source-artifact-dir",
            str(BASIC01_SOURCE),
            "--run-id",
            "basic01_cli_import_test",
            "--output-root",
            str(tmp_path),
        ]
    )

    assert exit_code == 0
    assert (tmp_path / "scenarios" / "BASIC-01" / "replay_trajectory.jsonl").exists()
    assert (tmp_path / "report.md").exists()


def test_basic_replay_rejects_mismatched_source_scenario(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="numeric_summary.json scenario_id"):
        run_basic_sumo_replay_artifacts(
            source_artifact_dir=BASIC02_SOURCE,
            output_root=tmp_path,
            run_id="mismatch",
            scenario="BASIC-01",
        )


def test_basic_replay_uses_ramp_upstream_mapping_and_keeps_hint_fallback() -> None:
    edge_id, lane_index, pos = to_sumo_position(6642.04, "on_ramp", "on_ramp_mv")
    assert (edge_id, lane_index) == ("ramp_upstream", 0)
    assert pos == pytest.approx(192.04)

    record = {
        "x_global": 6442.04,
        "physical_lane": "on_ramp",
        "road_role": "on_ramp_mv",
        "visual_replay_hint": {
            "mode": "allow_pre_control_on_ramp",
            "edge_id": "ramp_upstream",
            "lane_index": 0,
        },
    }

    assert mvs_gui_replay._sumo_position_for_replay_record(
        record,
        6442.04,
        "on_ramp",
        "on_ramp_mv",
    ) == ("ramp_upstream", 0, 0.0)


def _read_replay_records(path: Path) -> list[dict[str, object]]:
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]
