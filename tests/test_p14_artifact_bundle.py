from __future__ import annotations

import json
from pathlib import Path

import pytest

from cormc.legacy.paper_artifacts import (
    P14_DETERMINISTIC_BASELINE_SCENARIOS,
    P14ArtifactRunConfig,
    run_p14_pre_p15_baseline,
    run_p14_scenario_artifact_bundle,
)
from cormc.legacy.artifact_reports import RegressionReport
from cormc.scenes import BASIC_SCENARIO_IDS


def test_p14_scenario_artifact_bundle_generates_formal_outputs(tmp_path: Path) -> None:
    scenario_id = BASIC_SCENARIO_IDS[0]
    result = run_p14_scenario_artifact_bundle(
        scenario_id,
        run_id="p14-single",
        output_root=tmp_path,
        max_steps=3,
    )
    scenario_dir = tmp_path / "p14-single" / "scenarios" / scenario_id

    assert Path(result.scenario_dir) == scenario_dir
    for filename in (
        "scenario_summary.md",
        "trajectory.csv",
        "events.jsonl",
        "sanity.jsonl",
        "time_space.png",
        "state_snapshot.json",
        "scenario_report.json",
    ):
        assert (scenario_dir / filename).exists()

    summary = (scenario_dir / "scenario_summary.md").read_text(encoding="utf-8")
    assert scenario_id in summary
    assert "status:" in summary
    assert "final step/t" in summary
    assert "time_space.png" in summary
    assert "Sanity Summary" in summary

    snapshot = json.loads((scenario_dir / "state_snapshot.json").read_text(encoding="utf-8"))
    assert snapshot["scenario_id"] == scenario_id
    assert snapshot["run_id"] == "p14-single"
    assert "initial_state" in snapshot
    assert "final_state" in snapshot
    assert snapshot["final_state"]["vehicle_states"]
    assert "active_maneuvers" in snapshot["final_state"]
    assert "controller_memory_by_vehicle" in snapshot["final_state"]

    png = (scenario_dir / "time_space.png").read_bytes()
    assert png.startswith(b"\x89PNG\r\n\x1a\n")
    assert len(png) > 500

    scenario_report = json.loads((scenario_dir / "scenario_report.json").read_text(encoding="utf-8"))
    assert scenario_report["exports"]["state_snapshot"].endswith("state_snapshot.json")
    assert scenario_report["png_paths"][0].endswith("time_space.png")
    assert scenario_report["png_feature_statuses"]
    assert "core_formula_status" in result.formula_status_summary
    assert "subformula_status" in result.formula_status_summary
    assert result.formula_status_summary["legacy_proxy_markers_present"] == []
    formula_status = scenario_report["formula_status_summary"]
    assert set(formula_status["core_formula_status"]) >= {"cuc_eq11_eq16", "cav_eq17_eq27"}
    assert set(formula_status["subformula_status"]) >= {"cav_cpid_eq21_eq27"}
    assert formula_status["legacy_proxy_markers_present"] == []
    assert "Formula Status" in summary
    assert "legacy proxy markers present: `none`" in summary


def test_p14_pre_p15_baseline_generates_run_reports_for_formal_static_baseline(tmp_path: Path) -> None:
    result = run_p14_pre_p15_baseline(
        P14ArtifactRunConfig(
            run_id="pre_p15",
            output_root=tmp_path,
            generated_label="stable-test-label",
        )
    )
    baseline_dir = tmp_path / "pre_p15"

    assert Path(result.output_dir) == baseline_dir
    assert [item.scenario_id for item in result.scenario_results] == list(P14_DETERMINISTIC_BASELINE_SCENARIOS)
    assert (baseline_dir / "run_report.md").exists()
    assert (baseline_dir / "artifact_manifest.json").exists()
    assert (baseline_dir / "regression_report.json").exists()
    assert (baseline_dir / "baseline_comparison_contract.json").exists()
    assert not (tmp_path / ".pre_p15.tmp").exists()

    run_report = (baseline_dir / "run_report.md").read_text(encoding="utf-8")
    assert "stable-test-label" in run_report
    assert "Suite Summary" in run_report
    assert "Formula Status" in run_report
    assert "P15 Baseline Note" in run_report
    for scenario_id in P14_DETERMINISTIC_BASELINE_SCENARIOS:
        assert scenario_id in run_report
        scenario_dir = baseline_dir / "scenarios" / scenario_id
        assert (scenario_dir / "scenario_summary.md").exists()
        assert (scenario_dir / "time_space.png").read_bytes().startswith(b"\x89PNG\r\n\x1a\n")
        scenario_report = json.loads((scenario_dir / "scenario_report.json").read_text(encoding="utf-8"))
        assert "formula_status_summary" in scenario_report
        assert scenario_report["formula_status_summary"]["legacy_proxy_markers_present"] == []
        scenario_summary = (scenario_dir / "scenario_summary.md").read_text(encoding="utf-8")
        assert "Formula Status" in scenario_summary
        assert "first_version_probe_not_eq11_eq12_locked" not in scenario_summary
        assert "cpid_memory_status=probe_schema_gap" not in scenario_summary

    manifest = json.loads((baseline_dir / "artifact_manifest.json").read_text(encoding="utf-8"))
    assert manifest["run_id"] == "pre_p15"
    assert len(manifest["entries"]) == len(P14_DETERMINISTIC_BASELINE_SCENARIOS)
    assert ".pre_p15.tmp" not in (baseline_dir / "artifact_manifest.json").read_text(encoding="utf-8")
    assert ".pre_p15.tmp" not in (baseline_dir / "regression_report.json").read_text(encoding="utf-8")
    assert ".pre_p15.tmp" not in run_report
    for entry in manifest["entries"]:
        assert entry["human_summary_path"].endswith("scenario_summary.md")
        assert ".pre_p15.tmp" not in entry["human_summary_path"]
        assert entry["exports"]["state_snapshot"].endswith("state_snapshot.json")
        assert entry["scenario_report_path"].endswith("scenario_report.json")

    regression = json.loads((baseline_dir / "regression_report.json").read_text(encoding="utf-8"))
    assert regression["suite_status"] == "passed"
    assert regression["required_green"] == []
    assert regression["required_failed"] == []
    assert regression["required_blocked"] == []
    assert regression["runner_gaps"] == []
    assert regression["classification_blockers"] == []
    assert regression["probe_observed"] == []
    assert regression["deferred_skipped"] == []

    contract = json.loads((baseline_dir / "baseline_comparison_contract.json").read_text(encoding="utf-8"))
    assert contract["numeric_tolerance_abs"] == 1e-6
    assert "final_state.step" in contract["strong_compare_fields"]
    assert "x_global" in contract["numeric_tolerance_compare_fields"]
    assert "png.header_valid" in contract["weak_compare_fields"]


def test_p14_failure_policy_does_not_update_final_baseline_on_suite_failure(tmp_path: Path) -> None:
    final_dir = tmp_path / "pre_p15"
    final_dir.mkdir()
    marker = final_dir / "existing.txt"
    marker.write_text("keep", encoding="utf-8")

    def failing_suite() -> RegressionReport:
        return RegressionReport(
            run_id="pre_p15",
            suite_status="failed_until_required_blockers_resolved",
            required_green=(),
            required_failed=({"scenario_id": "MVS-E2E-1", "failure_reasons": ["forced failure"]},),
        )

    with pytest.raises(RuntimeError, match="suite failed|required failures|formal static baseline"):
        run_p14_pre_p15_baseline(
            P14ArtifactRunConfig(
                run_id="pre_p15",
                output_root=tmp_path,
            ),
            suite_runner=failing_suite,
        )

    assert marker.read_text(encoding="utf-8") == "keep"
    assert not (tmp_path / ".pre_p15.tmp").exists()
