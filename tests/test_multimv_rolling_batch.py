from __future__ import annotations

import json
from pathlib import Path

from cormc.onestep.rolling.stage2_multimv_runner import (
    run_multimv_rolling_archive,
    run_one_multimv_archive,
)


def test_multimv_m2_s01_real_rolling_completes_and_exports(tmp_path: Path) -> None:
    row = run_one_multimv_archive("RM-M2-S01", tmp_path, "pytest_m2_s01")
    output_dir = Path(str(row["stage2_output_dir"]))

    assert row["status"] == "completed"
    assert (output_dir / "stage2_summary.json").exists()
    assert (output_dir / "stage2_report.md").exists()
    assert (output_dir / "trajectory.csv").exists()
    assert (output_dir / "gap_rows.json").exists()
    assert (output_dir / "process_x_t_local.png").exists()
    assert (output_dir / "stage8_artifact_manifest.json").exists()
    assert (output_dir / "stage8_sumo_replay").is_dir()
    assert row["observation_manifest"] == str(output_dir / "stage8_artifact_manifest.json")
    assert row["sumo_replay_dir"] == str(output_dir / "stage8_sumo_replay")


def test_multimv_m3_s05_batch_records_diagnostics_without_aborting(tmp_path: Path) -> None:
    result = run_multimv_rolling_archive(
        tmp_path,
        ["RM-M3-S05"],
        "pytest_m3_s05",
    )
    manifest_path = Path(str(result["manifest_path"]))
    summary_csv_path = Path(str(result["summary_csv_path"]))
    report_path = Path(str(result["report_path"]))

    assert manifest_path.exists()
    assert summary_csv_path.exists()
    assert report_path.exists()
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    rows = manifest["results"]
    assert [row["scenario_id"] for row in rows] == ["RM-M3-S05"]
    assert rows[0]["status"] in {
        "completed",
        "incomplete",
        "observation_failed",
        "incomplete_observation_failed",
        "exception",
    }
    assert "diagnostics" in rows[0]


def test_multimv_hdv_case_reaches_real_gap_diagnostics(tmp_path: Path) -> None:
    row = run_one_multimv_archive("RM-M2-S07", tmp_path, "pytest_m2_s07")

    assert row["status"] in {
        "completed",
        "incomplete",
        "observation_failed",
        "incomplete_observation_failed",
    }
    branch_names = {
        record.get("branch")
        for record in row["selected_gap_records"]
        if record.get("branch") is not None
    }
    summary_path = Path(str(row["stage2_output_dir"])) / "stage2_summary.json"
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    all_branches = {
        gap_row.get("branch")
        for mv_summary in summary["mv_summaries"].values()
        for gap_row in mv_summary.get("gap_rows", [])
    }
    assert branch_names | all_branches
    assert {
        "B_front_controllable_rear_uncontrollable",
        "C_rear_controllable_front_uncontrollable",
        "D_none_controllable",
    } & all_branches
