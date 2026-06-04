from __future__ import annotations

import json
from pathlib import Path

from cormc.sumo.mvs_replay_artifacts import run_p17_1_mvs_replay_artifacts


def test_p17_1_cli_artifacts_single_and_all(tmp_path: Path) -> None:
    single = run_p17_1_mvs_replay_artifacts(
        output_root=tmp_path / "single",
        run_id="p17_1_single",
        scenario="MVS-CMC-1-extended",
    )
    assert single.status == "passed"
    assert len(single.scenario_results) == 1
    assert Path(single.report_path).exists()

    all_result = run_p17_1_mvs_replay_artifacts(
        output_root=tmp_path / "all",
        run_id="p17_1_all",
        scenario="all",
    )
    assert all_result.status == "passed"
    assert len(all_result.scenario_results) == 6

    report = Path(all_result.report_path).read_text(encoding="utf-8")
    for result in all_result.scenario_results:
        scenario_dir = Path(result.output_dir)
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
            assert (scenario_dir / name).exists(), (result.replay_id, name)
        assert result.gui_replay_script_path is not None
        assert f'& "{result.gui_replay_script_path}"' in report
    assert "Do not use a bare `.sumocfg` launch as the replay entrypoint" in report
    assert "P17 true closed-loop TraCI trajectory-authority code is not replaced" in report


def test_p17_1_manifest_has_three_status_classes(tmp_path: Path) -> None:
    result = run_p17_1_mvs_replay_artifacts(output_root=tmp_path, run_id="p17_1_manifest")
    manifest = json.loads(Path(result.artifact_manifest_path).read_text(encoding="utf-8"))

    assert manifest["status"] == "passed"
    assert manifest["boundary_statement"]["p17"].startswith("P17 remains")
    for scenario in manifest["scenarios"]:
        assert scenario["numeric_gate_status"] == "passed"
        assert scenario["replay_fidelity_status"] == "passed"
        assert "gui_smoke_status" in scenario
