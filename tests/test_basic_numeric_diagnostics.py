from __future__ import annotations

from pathlib import Path

from cormc.basic_runner import run_basic_numeric_scenario, run_basic_numeric_suite


def test_basic_04_numeric_summary_extracts_main_chain(tmp_path: Path) -> None:
    result = run_basic_numeric_scenario(
        "BASIC-04",
        output_dir=tmp_path,
        run_id="basic-test",
        max_steps=60,
        render_png=False,
    )
    summary = result.numeric_summary

    assert summary["scenario_id"] == "BASIC-04"
    assert summary["actual_steps"] == 60
    assert summary["first_control_zone_step"] == 0
    assert summary["first_merge_zone_step"] is not None
    assert summary["first_aps"]["step"] == 0
    assert summary["observed_aps_case"] == "case_2"
    assert summary["active_cv_ids"] == ["B04_CFV"]
    assert summary["eq10_consumers"] == ["B04_CFV"]
    assert summary["illegal_eq10_consumers"] == []
    assert summary["assignment_validity_timeline"]
    assert summary["aps_excluded_candidate_timeline"]
    first_excluded = summary["aps_excluded_candidate_timeline"][0]["payload"]["excluded_candidates"][0]
    assert first_excluded["vehicle_id"] == "B04_CFV"
    assert first_excluded["excluded_reason"] == "lane_change_executing"
    assert (
        summary["first_cached_boundary_invalidation"]["payload"][
            "old_assignment_marked_recovery_required"
        ]
        is True
    )
    assert summary["first_cached_boundary_invalidation"]["payload"]["invalid_boundary_id"] == "B04_CFV"
    assert summary["first_cached_boundary_invalidation"]["payload"]["invalid_reason"] == "lane_change_executing"
    assert Path(result.numeric_summary_path).exists()
    assert Path(result.scenario_report_path).exists()
    assert Path(result.artifact_manifest_path).exists()


def test_basic_01_pre_control_first_aps_after_control_zone(tmp_path: Path) -> None:
    result = run_basic_numeric_scenario(
        "BASIC-01",
        output_dir=tmp_path,
        run_id="basic-test",
        max_steps=20,
        render_png=False,
    )
    summary = result.numeric_summary

    assert summary["first_control_zone_step"] is not None
    assert summary["first_aps"]["step"] >= summary["first_control_zone_step"]
    assert summary["first_aps"]["payload"]["aps_case"] == "case_2"
    assert summary["illegal_pre_control_module_events"] == []
    assert summary["pre_control_suppressed_module_counts"] == {
        "APS": 0,
        "assignment_cache": 0,
        "cooperative_request": 0,
        "CUC": 0,
        "CMC": 0,
    }


def test_basic_01_and_02_use_assignment_relation_without_legacy_mv_cap(tmp_path: Path) -> None:
    basic_01 = run_basic_numeric_scenario(
        "BASIC-01",
        output_dir=tmp_path,
        run_id="basic-test",
        max_steps=70,
        render_png=False,
    ).numeric_summary
    basic_02 = run_basic_numeric_scenario(
        "BASIC-02",
        output_dir=tmp_path,
        run_id="basic-test",
        max_steps=90,
        render_png=False,
    ).numeric_summary

    assert basic_01["observed_aps_case"] == "case_2"
    assert basic_01["active_cv_ids"] == ["B01_CFV"]
    assert basic_01["eq10_consumers"] == ["B01_CFV"]
    assert ("aps_" + "gap_protection_timeline") not in basic_01
    assert basic_01["mv_longitudinal_relation_timeline"]
    assert all(
        item["longitudinal_mode"] in {"cav_gap_regulating", "cav_cruising"}
        for item in basic_01["mv_longitudinal_relation_timeline"]
    )

    assert basic_02["observed_aps_case"] == "case_3"
    assert basic_02["active_cv_ids"] == ["B02_CLV"]
    assert basic_02["eq10_consumers"] == []
    assert ("aps_" + "gap_protection_timeline") not in basic_02
    assert any(
        item["leader_id"] == "B02_CLV"
        and item["leader_relation_source"] == "aps_assignment_case_3_mv_clv_leader"
        for item in basic_02["mv_longitudinal_relation_timeline"]
    )


def test_basic_suite_writes_root_reports_and_all_scenario_artifacts(tmp_path: Path) -> None:
    suite = run_basic_numeric_suite(
        output_dir=tmp_path,
        run_id="suite-test",
        max_steps=20,
        render_png=False,
    )

    assert suite.suite_summary["scenario_count"] == 6
    assert suite.suite_summary["all_scenarios_have_artifacts"] is True
    assert len(suite.scenario_results) == 6
    assert Path(suite.suite_summary_path).exists()
    assert Path(suite.suite_report_path).exists()
    assert Path(suite.artifact_manifest_path).exists()
    for result in suite.scenario_results:
        artifact_paths = result.numeric_summary["artifact_paths"]
        assert Path(artifact_paths["trajectory"]).exists()
        assert Path(artifact_paths["events"]).exists()
        assert Path(artifact_paths["sanity"]).exists()
        assert Path(result.numeric_summary_path).exists()
        assert Path(result.scenario_report_path).exists()
