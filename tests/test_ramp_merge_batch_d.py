from __future__ import annotations

from copy import deepcopy
from pathlib import Path

from cormc.legacy import (
    AcceptanceReport,
    build_acceptance_report,
    detect_multi_mv_gap_conflicts,
    run_ramp_merge_basic_acceptance,
    run_ramp_merge_basic_smoke,
)


def test_batch_d_basic_04_acceptance_report_covers_new_algorithm_lifecycle() -> None:
    summary = run_ramp_merge_basic_smoke(
        "BASIC-04",
        max_steps=65,
        run_id="ramp-merge-batch-d-basic-04",
    )

    report = build_acceptance_report(summary)

    assert report.passed is True
    assert report.issues == ()
    assert "B04_MV" in report.vehicle_lifecycles

    lifecycle = report.vehicle_lifecycles["B04_MV"]
    assert lifecycle.entered_control_zone_step is not None
    assert lifecycle.selected_gap_steps
    assert lifecycle.locked_gap_step is not None
    assert lifecycle.entered_merging_step is not None
    assert lifecycle.merge_completed_step is not None
    assert lifecycle.final_physical_lane == "lane_2"
    assert lifecycle.final_road_role == "mainline"
    assert lifecycle.final_merge_state == "normal"

    assert report.integrity["old_assignment_record_count"] == 0
    assert report.integrity["old_active_maneuver_count"] == 0
    assert report.integrity["non_trigger_gap_event_count"] == 0
    assert report.integrity["multi_mv_gap_conflict"] is False
    assert summary["runtime_mv_ids"] == ()
    assert summary["runtime_mv_states"] == {}

    for expected_text in (
        "control_zone",
        "current_plan_gap",
        "locked_gap",
        "merging",
        "completed",
    ):
        assert expected_text in report.human_report


def test_batch_d_runner_acceptance_helper_returns_summary_and_report() -> None:
    result = run_ramp_merge_basic_acceptance(
        "BASIC-04",
        max_steps=65,
        run_id="ramp-merge-batch-d-helper",
    )

    assert set(result) == {"summary", "acceptance_report"}
    assert result["summary"]["scenario_id"] == "BASIC-04"
    assert isinstance(result["acceptance_report"], AcceptanceReport)
    assert result["acceptance_report"].passed is True


def test_batch_d_non_trigger_frames_may_advance_trajectory_without_gap_snapshot() -> None:
    summary = run_ramp_merge_basic_smoke(
        "BASIC-04",
        max_steps=65,
        run_id="ramp-merge-batch-d-non-trigger",
    )

    trigger_by_step = {
        event["step"]: event["trigger_plan"]
        for event in summary["trigger_events"]
    }
    gap_snapshot_steps = {
        event["step"]
        for event in summary["gap_snapshots"]
    }
    trajectory_steps = {
        event["step"]
        for event in summary["trajectory_events"]
        if event["trajectory_kind"] == "merge_execution"
    }

    assert all(
        trigger_by_step.get(step) is True
        for step in gap_snapshot_steps
    )
    assert any(
        trigger_by_step.get(step) is False
        for step in trajectory_steps
    )
    assert build_acceptance_report(summary).passed is True


def test_batch_d_detects_multi_mv_same_structured_gap_conflict() -> None:
    summary = run_ramp_merge_basic_smoke(
        "BASIC-04",
        max_steps=65,
        run_id="ramp-merge-batch-d-conflict-source",
    )
    clean_report = build_acceptance_report(summary)
    assert clean_report.passed is True
    assert clean_report.integrity["multi_mv_gap_conflict"] is False

    conflicted = deepcopy(summary)
    first_selection = deepcopy(conflicted["gap_selection_events"][0])
    second_selection = deepcopy(first_selection)
    second_selection["mv_id"] = "B04_MV_2"
    conflicted["gap_selection_events"] = [
        first_selection,
        second_selection,
        *conflicted["gap_selection_events"][1:],
    ]

    issues = detect_multi_mv_gap_conflicts(conflicted)
    assert [issue.check_id for issue in issues] == ["ACCEPT-012"]
    assert issues[0].severity == "error"
    assert set(issues[0].payload["mv_ids"]) == {"B04_MV", "B04_MV_2"}

    conflicted_report = build_acceptance_report(conflicted)
    assert conflicted_report.passed is False
    assert conflicted_report.integrity["multi_mv_gap_conflict"] is True


def test_batch_d_reports_missing_required_fields_and_old_state_pollution() -> None:
    summary = run_ramp_merge_basic_smoke(
        "BASIC-04",
        max_steps=65,
        run_id="ramp-merge-batch-d-negative",
    )

    missing = deepcopy(summary)
    missing.pop("gap_snapshots")
    missing_report = build_acceptance_report(missing)
    assert missing_report.passed is False
    assert "ACCEPT-001" in _issue_ids(missing_report)

    polluted_assignment = deepcopy(summary)
    polluted_assignment["old_assignment_record_count"] = 1
    polluted_assignment_report = build_acceptance_report(polluted_assignment)
    assert polluted_assignment_report.passed is False
    assert "ACCEPT-010" in _issue_ids(polluted_assignment_report)

    polluted_maneuver = deepcopy(summary)
    polluted_maneuver["old_active_maneuver_count"] = 1
    polluted_maneuver_report = build_acceptance_report(polluted_maneuver)
    assert polluted_maneuver_report.passed is False
    assert "ACCEPT-011" in _issue_ids(polluted_maneuver_report)


def test_batch_d_validation_stays_one_way_and_old_path_is_not_reintroduced() -> None:
    package_dir = Path(__file__).parents[1] / "cormc" / "onestep" / "rolling"
    legacy_runner = (
        Path(__file__).parents[1] / "cormc" / "legacy" / "ramp_merge_basic.py"
    )
    source_by_file = {
        path.name: path.read_text(encoding="utf-8")
        for path in package_dir.glob("*.py")
    }

    algorithm_modules = (
        "engine.py",
        "planner.py",
        "motion.py",
        "gaps.py",
        "safety.py",
    )
    for filename in algorithm_modules:
        source = source_by_file[filename]
        assert "validation" not in source
        for fragment in _old_algorithm_fragments():
            assert fragment not in source
        assert "assignment_records_by_mv" not in source
        assert "active_maneuvers" not in source

    legacy_source = legacy_runner.read_text(encoding="utf-8")
    assert "from cormc.onestep.rolling.validation import" in legacy_source
    assert "build_acceptance_report" in legacy_source
    assert "assignment_records_by_mv" in legacy_source
    assert "active_maneuvers" in legacy_source

    validation_source = source_by_file["validation.py"]
    for fragment in _old_algorithm_fragments():
        assert fragment not in validation_source
    for fragment in (
        "gap_id.split",
        ".startswith(",
        ".endswith(",
    ):
        assert fragment not in validation_source
        assert fragment not in source_by_file["planner.py"]
        assert fragment not in source_by_file["gaps.py"]


def _issue_ids(report: AcceptanceReport) -> set[str]:
    return {issue.check_id for issue in report.issues}


def _old_algorithm_fragments() -> tuple[str, ...]:
    return (
        "from cormc.simulation_core.aps",
        "from cormc.simulation_core.cmc",
        "from cormc.simulation_core.cooperative_request",
        "from cormc.simulation_core.cuc",
        "from cormc.simulation_core.engine",
        "import cormc.simulation_core.engine",
    )
