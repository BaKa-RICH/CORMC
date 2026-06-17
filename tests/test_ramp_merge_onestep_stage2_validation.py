from __future__ import annotations

from copy import deepcopy
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from cormc.onestep.rolling import (
    build_onestep_stage2_acceptance_report,
    run_onestep_stage2_summary,
)
from cormc.scenes import (
    RM_ONESTEP_S07_2MV_REAR_MV_ID,
    RM_ONESTEP_S07_2MV_ROLLING_ENTRY_SCENARIO_ID,
    RM_ONESTEP_S07_MV_ID,
)


@pytest.fixture(scope="module")
def s07_2mv_summary():
    return run_onestep_stage2_summary(
        RM_ONESTEP_S07_2MV_ROLLING_ENTRY_SCENARIO_ID,
        max_steps=700,
        run_id="stage2-validation-test",
    )


def test_stage2_acceptance_report_passes_real_2mv_summary(s07_2mv_summary) -> None:
    report = build_onestep_stage2_acceptance_report(s07_2mv_summary)

    assert report.passed is True
    assert report.issues == ()
    assert report.integrity["summary_schema_valid"] is True
    assert report.integrity["minimal_event_set_valid"] is True
    assert report.integrity["multi_mv_gap_conflict"] is False
    assert report.integrity["frontier_violation"] is False
    assert report.integrity["ownership_conflict"] is False
    assert report.integrity["runtime_leftover_count"] == 0


def test_stage2_acceptance_report_rejects_same_gap_conflict(s07_2mv_summary) -> None:
    broken = deepcopy(s07_2mv_summary)
    broken["cross_mv_summary"]["gap_conflicts"].append(
        {
            "round_id": "trigger_round:55",
            "gap_key": [55, 3, "S07_L2_05", "S07_L2_04"],
            "mv_ids": [RM_ONESTEP_S07_MV_ID, RM_ONESTEP_S07_2MV_REAR_MV_ID],
        }
    )

    report = build_onestep_stage2_acceptance_report(broken)

    assert report.passed is False
    assert any(issue.check_id == "S2-CROSS-001" for issue in report.issues)
    assert report.integrity["multi_mv_gap_conflict"] is True


def test_stage2_acceptance_report_rejects_missing_formal_event_kind(
    s07_2mv_summary,
) -> None:
    broken = deepcopy(s07_2mv_summary)
    broken["scenario_summary"]["formal_events"] = [
        event
        for event in broken["scenario_summary"]["formal_events"]
        if event["event_kind"] != "locked_gap_created"
    ]

    report = build_onestep_stage2_acceptance_report(broken)

    assert report.passed is False
    assert any(issue.check_id == "S2-EVENT-001" for issue in report.issues)
    assert report.integrity["minimal_event_set_valid"] is False


def test_stage2_acceptance_report_rejects_frontier_violation(s07_2mv_summary) -> None:
    broken = deepcopy(s07_2mv_summary)
    round_summary = next(
        item for item in broken["round_summaries"] if item["round_id"] == "trigger_round:55"
    )
    rear_plan = next(
        plan
        for plan in round_summary["plan_summaries"]
        if plan["mv_id"] == RM_ONESTEP_S07_2MV_REAR_MV_ID
    )
    rear_plan["gap_index"] = rear_plan["tail_frontier_gap_index_before"]
    broken["cross_mv_summary"]["frontier_violations"].append(
        {
            "round_id": "trigger_round:55",
            "mv_id": RM_ONESTEP_S07_2MV_REAR_MV_ID,
            "gap_index": rear_plan["gap_index"],
            "tail_frontier_gap_index_before": rear_plan[
                "tail_frontier_gap_index_before"
            ],
        }
    )

    report = build_onestep_stage2_acceptance_report(broken)

    assert report.passed is False
    assert any(issue.check_id == "S2-ROUND-002" for issue in report.issues)
    assert report.integrity["frontier_violation"] is True


def test_stage2_acceptance_report_rejects_ownership_conflict(s07_2mv_summary) -> None:
    broken = deepcopy(s07_2mv_summary)
    broken["cross_mv_summary"]["ownership_conflicts"].append(
        {
            "step": 55,
            "vehicle_owners": {"S07_L2_04": ["bundle-a", "bundle-b"]},
        }
    )

    report = build_onestep_stage2_acceptance_report(broken)

    assert report.passed is False
    assert any(issue.check_id == "S2-CROSS-002" for issue in report.issues)
    assert report.integrity["ownership_conflict"] is True


def test_stage2_acceptance_report_rejects_runtime_leftover(s07_2mv_summary) -> None:
    broken = deepcopy(s07_2mv_summary)
    leftovers = broken["cross_mv_summary"]["final_runtime_leftovers"]
    leftovers["active_bundle_ids"].append("stale-bundle")

    report = build_onestep_stage2_acceptance_report(broken)

    assert report.passed is False
    assert any(issue.check_id == "S2-RUNTIME-001" for issue in report.issues)
    assert report.integrity["runtime_leftover_count"] == 1
