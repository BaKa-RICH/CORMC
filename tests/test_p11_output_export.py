from __future__ import annotations

import csv
import json
from dataclasses import fields

from cormc.legacy.artifact_reports import (
    P11_REQUIRED_MVS_IDS,
    ArtifactManifest,
    ArtifactManifestEntry,
    SmokeSuiteRegistry,
    aggregate_targeted_scenario_reports,
    build_p11_smoke_suite_registry,
    build_regression_report,
    build_scenario_artifact_bundle,
    derive_x_plot_for_renderer,
    export_event_history,
    export_sanity_history,
    export_trajectory_history,
    render_expected_png_features,
    render_time_space_png,
    run_full_required_mvs_smoke_suite,
    write_artifact_manifest,
    write_regression_report,
)
from cormc.simulation_core.commit import EventRecord, OutputHistory, SanityCheckRecord, TrajectoryRecord


def test_p11_required_mvs_registry_is_retired_by_default() -> None:
    registry = build_p11_smoke_suite_registry()

    assert registry.required_ids == ()
    assert registry.probe_ids == ()
    assert registry.deferred_ids == ()
    assert registry.extra_diagnostic_ids == ()
    assert run_full_required_mvs_smoke_suite().scenario_results == ()


def test_p11_exports_trajectory_history_for_committed_vehicles(tmp_path) -> None:
    history = _history()
    path = export_trajectory_history(history, tmp_path / "trajectory.csv")

    rows = list(csv.DictReader(path.read_text(encoding="utf-8").splitlines()))
    assert rows == [
        {
            "scenario_id": "MVS-E2E-1",
            "run_id": "p11-run",
            "step": "0",
            "t": "0.0",
            "vehicle_id": "MV_CMC_1",
            "vehicle_type": "cav",
            "compliance_state": "not_applicable",
            "x_global": "7002.0",
            "y": "-3.48",
            "v": "20.0",
            "a": "0.0",
            "physical_lane": "on_ramp",
            "road_role": "on_ramp",
            "primary_leader_id": "",
            "lane_change_state": "normal",
            "merge_state": "executing",
            "active_event_tags": "commit",
        }
    ]
    assert "x_plot" not in rows[0]
    assert "x_plot" not in {field.name for field in fields(history.trajectory_records[0])}


def test_p11_exports_event_history_with_source_reason_patch_payload(tmp_path) -> None:
    history = _history()
    path = export_event_history(history, tmp_path / "events.jsonl")

    event = json.loads(path.read_text(encoding="utf-8").splitlines()[0])
    assert event["event_type"] == "commit"
    assert event["source"] == "first_version_engineering_patch"
    assert event["reason"] == "commit_step_final_candidate"
    assert event["is_engineering_patch"] is True
    assert event["payload"]["source_longitudinal_candidate"] == "p08:0:MV_CMC_1:longitudinal"
    assert event["payload"]["source_lateral_candidate"] == "p09:0:MV_CMC_1:lateral"


def test_p11_exports_sanity_history_and_required_fail_blocks_report(tmp_path) -> None:
    history = _history(sanity_result="fail")
    path = export_sanity_history(history, tmp_path / "sanity.jsonl")

    sanity = json.loads(path.read_text(encoding="utf-8").splitlines()[0])
    assert sanity["check_type"] == "multiple_commit_for_one_vehicle"
    assert sanity["result"] == "fail"

    suite = aggregate_targeted_scenario_reports(
        [],
        required_histories={"MVS-COMMIT-1-full": history},
        registry=SmokeSuiteRegistry(required_ids=("MVS-COMMIT-1-full",)),
    )
    result = _scenario_result(suite.scenario_results, "MVS-COMMIT-1-full")
    assert result.classification == "required_failed"
    assert result.blocks_required_suite is True
    assert any("multiple_commit_for_one_vehicle" in reason for reason in result.failure_reasons)


def test_p11_formal_png_renderer_consumes_expected_features(tmp_path) -> None:
    history = _history()
    expected_features = [
        {
            "feature_type": "commit_marker",
            "required": True,
            "vehicle_ids": ["MV_CMC_1"],
            "expected_visibility": "visible",
        },
        {
            "feature_type": "trajectory_quicklook",
            "required": False,
            "vehicle_ids": ["MV_CMC_1"],
            "expected_visibility": "optional",
        },
        {
            "feature_type": "assignment_arrow",
            "required": True,
            "vehicle_ids": ["MV_CMC_1"],
            "expected_visibility": "not_visible",
        },
    ]

    render = render_time_space_png(
        history.trajectory_records,
        expected_features,
        tmp_path / "time_space.png",
        events=history.event_records,
    )

    assert (tmp_path / "time_space.png").read_bytes().startswith(b"\x89PNG\r\n\x1a\n")
    statuses = {item["feature_type"]: item for item in render.feature_statuses}
    assert statuses["commit_marker"]["renderer_status"] == "visible"
    assert statuses["trajectory_quicklook"]["renderer_status"] == "optional"
    assert statuses["assignment_arrow"]["renderer_status"] == "not_visible"
    assert all(item["evidence_source"] == "expected_png_features" for item in statuses.values())
    assert all("subjective" not in item for item in statuses.values())


def test_p11_x_plot_is_renderer_derived_only_and_no_state_mutation(tmp_path) -> None:
    history = _history()
    before = repr(history)

    assert derive_x_plot_for_renderer(7002.0) == 3002.0
    export_trajectory_history(history, tmp_path / "trajectory.csv")
    export_event_history(history, tmp_path / "events.jsonl")
    export_sanity_history(history, tmp_path / "sanity.jsonl")
    render_time_space_png(
        history.trajectory_records,
        [{"feature_type": "commit_marker", "vehicle_ids": ["MV_CMC_1"]}],
        tmp_path / "time_space.png",
    )

    assert repr(history) == before
    assert "x_plot" not in {field.name for field in fields(history.trajectory_records[0])}


def test_p11_artifact_manifest_records_each_scenario_bundle(tmp_path) -> None:
    history = _history()
    bundle = build_scenario_artifact_bundle(
        scenario_id="MVS-E2E-1",
        run_id="p11-run",
        output_dir=tmp_path,
        history=history,
        expected_png_features=[{"feature_type": "commit_marker", "vehicle_ids": ["MV_CMC_1"]}],
        status="required_blocked_until_runner_route_registered",
        input_config_ref="built-in-or-helper-reference",
        gaps=("built-in MVS-E2E-1 runner route not registered",),
    )
    entry = ArtifactManifestEntry(
        scenario_id=bundle.scenario_id,
        run_id=bundle.run_id,
        status=bundle.status,
        input_config_ref="built-in-or-helper-reference",
        exports=bundle.exports,
        png_paths=bundle.png_paths,
        scenario_report_path=bundle.scenario_report_path,
        regression_report_ref="artifacts/regression_report.json",
        gaps=bundle.gaps,
        png_feature_statuses=bundle.png_feature_statuses,
    )
    manifest_path = write_artifact_manifest(
        ArtifactManifest(run_id="p11-run", entries=(entry,)),
        tmp_path / "artifact_manifest.json",
    )

    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest_entry = manifest["entries"][0]
    assert manifest_entry["scenario_id"] == "MVS-E2E-1"
    assert manifest_entry["exports"]["trajectory"].endswith("trajectory.csv")
    assert manifest_entry["exports"]["events"].endswith("events.jsonl")
    assert manifest_entry["exports"]["sanity"].endswith("sanity.jsonl")
    assert manifest_entry["png_paths"][0].endswith("time_space.png")
    assert manifest_entry["gaps"] == ["built-in MVS-E2E-1 runner route not registered"]


def test_p11_regression_report_groups_required_probe_deferred_and_gaps(tmp_path) -> None:
    suite = run_full_required_mvs_smoke_suite()
    report = build_regression_report(
        suite,
        run_id="p11-run",
        schema_gaps=("OutputArtifactRecord authority missing in code",),
        artifact_paths=("artifacts/MVS-E2E-1/p11-run/time_space.png",),
    )
    path = write_regression_report(report, tmp_path / "regression_report.json")

    data = json.loads(path.read_text(encoding="utf-8"))
    assert data["suite_status"] == "passed"
    assert data["required_green"] == []
    assert data["required_blocked"] == []
    assert data["classification_blockers"] == []
    assert data["probe_observed"] == []
    assert data["deferred_skipped"] == []
    assert data["schema_gaps"] == ["OutputArtifactRecord authority missing in code"]
    assert data["runner_gaps"] == []


def test_p11_duplicate_commit_sanity_blocks_runner_continuation() -> None:
    history = _history(sanity_result="fail")

    suite = aggregate_targeted_scenario_reports(
        [],
        required_histories={"MVS-COMMIT-1-full": history},
        registry=SmokeSuiteRegistry(required_ids=("MVS-COMMIT-1-full",)),
    )

    result = _scenario_result(suite.scenario_results, "MVS-COMMIT-1-full")
    assert result.classification == "required_failed"
    assert result.blocks_required_suite is True
    assert result.passed is False
    assert "missing-vehicle next_state must not continue" in " ".join(result.failure_reasons)


def test_p11_does_not_enable_p12_random_or_paper_metrics() -> None:
    suite = run_full_required_mvs_smoke_suite()

    assert suite.p12_random_generation_enabled is False
    assert suite.paper_metrics_required is False
    assert "P12-paper-grid" not in P11_REQUIRED_MVS_IDS


def _history(*, sanity_result: str = "pass") -> OutputHistory:
    return OutputHistory(
        trajectory_records=[
            TrajectoryRecord(
                run_id="p11-run",
                scenario_id="MVS-E2E-1",
                step=0,
                t=0.0,
                vehicle_id="MV_CMC_1",
                vehicle_type="cav",
                compliance_state="not_applicable",
                x_global=7002.0,
                y=-3.48,
                v=20.0,
                a=0.0,
                physical_lane="on_ramp",
                road_role="on_ramp",
                lane_change_state="normal",
                merge_state="executing",
                active_event_tags=("commit",),
            )
        ],
        event_records=[
            EventRecord(
                event_id="p11-run:0:commit:MV_CMC_1",
                run_id="p11-run",
                scenario_id="MVS-E2E-1",
                step=0,
                t=0.0,
                module="commit",
                event_type="commit",
                vehicle_id="MV_CMC_1",
                related_vehicle_ids=("MV_CMC_1",),
                source_candidate_id="p10:0:MV_CMC_1:final",
                reason="commit_step_final_candidate",
                result="committed",
                is_engineering_patch=True,
                source="first_version_engineering_patch",
                payload={
                    "source_longitudinal_candidate": "p08:0:MV_CMC_1:longitudinal",
                    "source_lateral_candidate": "p09:0:MV_CMC_1:lateral",
                },
            )
        ],
        sanity_check_records=[
            SanityCheckRecord(
                check_id="p11-run:0:multiple_commit_for_one_vehicle",
                run_id="p11-run",
                scenario_id="MVS-E2E-1",
                step=0,
                t=0.0,
                check_type="multiple_commit_for_one_vehicle",
                severity="error" if sanity_result == "fail" else "info",
                result=sanity_result,
                vehicle_ids=("MV_CMC_1",),
                reason="duplicate_final_candidate" if sanity_result == "fail" else "one_final_candidate_per_vehicle",
            )
        ],
    )


def _scenario_result(results, scenario_id: str):
    for result in results:
        if result.scenario_id == scenario_id:
            return result
    raise AssertionError(f"missing scenario result {scenario_id}")
