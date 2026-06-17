from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from cormc.onestep.rolling import run_onestep_stage2_analysis
from cormc.scenes import (
    RM_ONESTEP_S05_PLAN_STEP0_SCENARIO_ID,
    RM_ONESTEP_S07_2MV_REAR_MV_ID,
    RM_ONESTEP_S07_2MV_ROLLING_ENTRY_SCENARIO_ID,
    RM_ONESTEP_S07_MV_ID,
    RM_ONESTEP_S07_PLAN_STEP0_SCENARIO_ID,
)


@pytest.mark.parametrize(
    "scenario_id",
    [
        RM_ONESTEP_S05_PLAN_STEP0_SCENARIO_ID,
        RM_ONESTEP_S07_PLAN_STEP0_SCENARIO_ID,
    ],
)
def test_stage2_analysis_exports_summary_report_and_artifacts(
    tmp_path: Path,
    scenario_id: str,
) -> None:
    result = run_onestep_stage2_analysis(
        scenario_id,
        tmp_path,
        max_steps=120,
        run_id="stage2-analysis-test",
    )
    payload = json.loads(Path(result.summary_json_path).read_text(encoding="utf-8"))
    report = Path(result.report_path).read_text(encoding="utf-8")

    assert Path(result.summary_json_path).exists()
    assert Path(result.report_path).exists()
    assert set(payload) == {
        "scenario_summary",
        "round_summaries",
        "mv_summaries",
        "cross_mv_summary",
        "artifact_paths",
    }
    assert "Scenario Summary" in report
    assert "Round Summary" in report
    assert "MV Lifecycle" in report
    assert "Cross-MV Validation" in report
    assert "Artifacts" in report
    for artifact_path in payload["artifact_paths"].values():
        assert Path(artifact_path).exists()


def test_stage2_analysis_exports_2mv_formal_summary_report_and_gap_rows(
    tmp_path: Path,
) -> None:
    result = run_onestep_stage2_analysis(
        RM_ONESTEP_S07_2MV_ROLLING_ENTRY_SCENARIO_ID,
        tmp_path,
        max_steps=700,
        run_id="stage2-2mv-analysis-test",
    )
    payload = json.loads(Path(result.summary_json_path).read_text(encoding="utf-8"))
    report = Path(result.report_path).read_text(encoding="utf-8")
    gap_rows = json.loads(Path(result.gap_rows_json_path).read_text(encoding="utf-8"))

    assert set(payload) == {
        "scenario_summary",
        "round_summaries",
        "mv_summaries",
        "cross_mv_summary",
        "artifact_paths",
    }
    assert payload["scenario_summary"]["mv_ids"] == [
        RM_ONESTEP_S07_MV_ID,
        RM_ONESTEP_S07_2MV_REAR_MV_ID,
    ]
    assert gap_rows
    assert payload["artifact_paths"]["gap_rows_json"] == result.gap_rows_json_path
    assert RM_ONESTEP_S07_MV_ID in report
    assert RM_ONESTEP_S07_2MV_REAR_MV_ID in report
    assert "Cross-MV" in report
    assert "validation passed" in report
    for artifact_path in payload["artifact_paths"].values():
        assert Path(artifact_path).exists()
