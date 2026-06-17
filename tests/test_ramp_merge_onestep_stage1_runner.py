from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from cormc.scenes import (
    RM_ONESTEP_S05_GAP_INTERVALS_LOCAL,
    RM_ONESTEP_S05_MV_ID,
    RM_ONESTEP_S05_PLAN_STEP0_SCENARIO_ID,
    RM_ONESTEP_S05_ROLLING_ENTRY_SCENARIO_ID,
    RM_ONESTEP_S07_GAP_INTERVALS_LOCAL,
    RM_ONESTEP_S07_MV_ID,
    RM_ONESTEP_S07_PLAN_STEP0_SCENARIO_ID,
    RM_ONESTEP_S07_ROLLING_ENTRY_SCENARIO_ID,
)
from cormc.legacy import (
    build_initial_onestep_stage1_state,
    export_onestep_stage1_analysis,
    run_onestep_stage1_analysis,
    run_onestep_stage1_history,
    run_onestep_stage1_summary,
)


@pytest.mark.parametrize(
    ("plan_id", "rolling_id", "mv_id"),
    [
        (
            RM_ONESTEP_S05_PLAN_STEP0_SCENARIO_ID,
            RM_ONESTEP_S05_ROLLING_ENTRY_SCENARIO_ID,
            RM_ONESTEP_S05_MV_ID,
        ),
        (
            RM_ONESTEP_S07_PLAN_STEP0_SCENARIO_ID,
            RM_ONESTEP_S07_ROLLING_ENTRY_SCENARIO_ID,
            RM_ONESTEP_S07_MV_ID,
        ),
    ],
)
def test_build_initial_onestep_stage1_state_initializes_runtime_for_both_modes(
    plan_id: str,
    rolling_id: str,
    mv_id: str,
) -> None:
    plan_state, _ = build_initial_onestep_stage1_state(plan_id)
    rolling_state, _ = build_initial_onestep_stage1_state(rolling_id)

    assert plan_state.ramp_merge_runtime.mv_plan_states[mv_id].zone_state == "control_zone"
    assert rolling_state.ramp_merge_runtime.mv_plan_states[mv_id].zone_state == "outside_control_zone"


@pytest.mark.parametrize(
    ("scenario_id", "expected_gap_intervals", "expected_case_id"),
    [
        (
            RM_ONESTEP_S05_PLAN_STEP0_SCENARIO_ID,
            RM_ONESTEP_S05_GAP_INTERVALS_LOCAL,
            "S05",
        ),
        (
            RM_ONESTEP_S07_PLAN_STEP0_SCENARIO_ID,
            RM_ONESTEP_S07_GAP_INTERVALS_LOCAL,
            "S07",
        ),
    ],
)
def test_plan_step0_summary_extracts_step0_precheck_snapshot(
    scenario_id: str,
    expected_gap_intervals: tuple[tuple[float, float], ...],
    expected_case_id: str,
) -> None:
    summary = run_onestep_stage1_summary(
        scenario_id,
        max_steps=5,
        run_id="stage1-plan-test",
    )

    assert summary["scenario_id"] == scenario_id
    assert summary["case_spec"]["one_step_case_id"] == expected_case_id
    assert summary["first_check_step"] == 0
    assert summary["first_check_t"] == pytest.approx(0.0)
    assert summary["first_check_trigger_event"]["trigger_reason"] == "periodic"
    assert summary["first_check_gap_snapshot"]["step"] == 0
    assert summary["first_check_mv_state"]["x_global"] == pytest.approx(6650.0)
    assert summary["first_check_mv_state"]["zone_state"] == "control_zone"
    assert summary["first_check_mv_local_frame"]["origin_x_global"] == pytest.approx(6650.0)
    assert summary["first_check_mv_local_frame"]["x_m0_local"] == pytest.approx(0.0)
    assert summary["first_check_mv_local_frame"]["gap_intervals_local"] == [
        list(interval) for interval in expected_gap_intervals
    ]


@pytest.mark.parametrize(
    ("scenario_id", "mv_id", "expected_gap_intervals"),
    [
        (
            RM_ONESTEP_S05_ROLLING_ENTRY_SCENARIO_ID,
            RM_ONESTEP_S05_MV_ID,
            RM_ONESTEP_S05_GAP_INTERVALS_LOCAL,
        ),
        (
            RM_ONESTEP_S07_ROLLING_ENTRY_SCENARIO_ID,
            RM_ONESTEP_S07_MV_ID,
            RM_ONESTEP_S07_GAP_INTERVALS_LOCAL,
        ),
    ],
)
def test_rolling_entry_summary_keeps_first_check_fixed_at_step25(
    scenario_id: str,
    mv_id: str,
    expected_gap_intervals: tuple[tuple[float, float], ...],
) -> None:
    result = run_onestep_stage1_history(
        scenario_id,
        max_steps=35,
        run_id="stage1-rolling-test",
    )
    summary = dict(result.summary)

    assert summary["scenario_id"] == scenario_id
    assert summary["first_check_step"] == 25
    assert summary["first_check_t"] == pytest.approx(2.5)
    assert summary["first_check_trigger_event"]["trigger_reason"] == "MV_enter_control_zone"
    assert summary["first_check_gap_snapshot"]["step"] == 25
    assert summary["first_check_mv_state"]["x_global"] == pytest.approx(6650.0)
    assert summary["first_check_mv_state"]["zone_state"] == "control_zone"
    assert summary["first_check_mv_local_frame"]["origin_x_global"] == pytest.approx(6650.0)
    assert summary["first_check_mv_local_frame"]["x_m0_local"] == pytest.approx(0.0)
    assert summary["first_check_mv_local_frame"]["gap_intervals_local"] == [
        list(interval) for interval in expected_gap_intervals
    ]
    assert all(
        item["zone_state_by_mv"][mv_id] == "outside_control_zone"
        for item in summary["zone_state_timeline"][:25]
    )
    assert any(
        item["step"] == 25 and item["zone_state_by_mv"][mv_id] == "control_zone"
        for item in summary["zone_state_timeline"]
    )


def test_analysis_export_writes_json_and_markdown_from_summary(tmp_path: Path) -> None:
    summary = run_onestep_stage1_summary(
        RM_ONESTEP_S05_ROLLING_ENTRY_SCENARIO_ID,
        max_steps=35,
        run_id="stage1-analysis-export",
    )
    result = export_onestep_stage1_analysis(summary, tmp_path / "manual-export")

    assert Path(result.summary_json_path).exists()
    assert Path(result.report_path).exists()
    payload = json.loads(Path(result.summary_json_path).read_text(encoding="utf-8"))
    report = Path(result.report_path).read_text(encoding="utf-8")

    assert payload["scenario_id"] == RM_ONESTEP_S05_ROLLING_ENTRY_SCENARIO_ID
    assert payload["artifact_paths"]["summary_json"] == result.summary_json_path
    assert payload["artifact_paths"]["report_markdown"] == result.report_path
    assert "scenario loaded correctly" in report
    assert "MV reached the expected check frame" in report
    assert "lane 2 gap geometry aligns with S05" in report
    assert "gap_intervals_local" in report


@pytest.mark.parametrize(
    ("scenario_id", "max_steps"),
    [
        (RM_ONESTEP_S05_PLAN_STEP0_SCENARIO_ID, 5),
        (RM_ONESTEP_S05_ROLLING_ENTRY_SCENARIO_ID, 35),
        (RM_ONESTEP_S07_PLAN_STEP0_SCENARIO_ID, 5),
        (RM_ONESTEP_S07_ROLLING_ENTRY_SCENARIO_ID, 35),
    ],
)
def test_run_onestep_stage1_analysis_creates_stage1_plot_artifacts(
    tmp_path: Path,
    scenario_id: str,
    max_steps: int,
) -> None:
    result = run_onestep_stage1_analysis(
        scenario_id,
        tmp_path,
        max_steps=max_steps,
        run_id="stage1-analysis-run",
    )
    payload = json.loads(Path(result.summary_json_path).read_text(encoding="utf-8"))
    report = Path(result.report_path).read_text(encoding="utf-8")

    assert result.scenario_id == scenario_id
    assert Path(result.summary_json_path).exists()
    assert Path(result.report_path).exists()
    assert result.trajectory_csv_path is not None
    assert result.x_t_local_plot_path is not None
    assert result.v_t_plot_path is not None
    assert Path(result.trajectory_csv_path).exists()
    assert Path(result.x_t_local_plot_path).exists()
    assert Path(result.v_t_plot_path).exists()
    assert payload["artifact_paths"]["trajectory_csv"] == result.trajectory_csv_path
    assert payload["artifact_paths"]["x_t_local_plot"] == result.x_t_local_plot_path
    assert payload["artifact_paths"]["v_t_plot"] == result.v_t_plot_path
    assert "trajectory.csv" in report
    assert "x_t_local.png" in report
    assert "v_t.png" in report
    assert Path(result.x_t_local_plot_path).read_bytes().startswith(b"\x89PNG\r\n\x1a\n")
    assert Path(result.v_t_plot_path).read_bytes().startswith(b"\x89PNG\r\n\x1a\n")
