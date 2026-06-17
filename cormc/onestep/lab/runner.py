from __future__ import annotations

import json
from dataclasses import asdict, dataclass, is_dataclass
from datetime import UTC, datetime
from pathlib import Path
from typing import Any, Iterable, Mapping

from cormc.onestep.kernel import (
    GapEvaluationRow,
    OneStepEvaluationResult,
    OneStepScenarioArtifacts,
    evaluate_one_step_scenario,
)
from cormc.onestep.lab.artifacts import build_one_step_artifacts
from cormc.onestep.lab.experiments import (
    ONE_STEP_FIXED_SCENARIO_IDS,
    ONE_STEP_SWEEP_SCENARIO_IDS,
    OneStepExperimentScenario,
    OneStepFixedExpectation,
    apply_sweep_parameter,
    build_default_one_step_algorithm_config,
    build_one_step_scenario_config,
    build_one_step_trajectory_contract,
    load_one_step_experiment_scenario,
)
from cormc.onestep.lab.reference_case import get_reference_expected
from cormc.legacy.artifact_reports import ArtifactManifest, ArtifactManifestEntry, write_artifact_manifest


ONE_STEP_OUTPUT_ROOT = Path("artifacts/one_step_algorithm")


@dataclass(frozen=True)
class OneStepScenarioRunResult:
    scenario_id: str
    run_id: str
    status: str
    scenario_dir: str
    evaluation_summary: Mapping[str, Any]
    evaluation_summary_path: str
    scenario_report_path: str
    artifact_manifest_path: str
    artifact_paths: Mapping[str, str]

    def to_dict(self) -> dict[str, Any]:
        return _to_plain(self)


@dataclass(frozen=True)
class OneStepSweepPointResult:
    scenario_id: str
    run_id: str
    parameter_name: str
    parameter_value: float
    status: str
    evaluation_summary: Mapping[str, Any]
    artifact_paths: Mapping[str, str]

    def to_dict(self) -> dict[str, Any]:
        return _to_plain(self)


@dataclass(frozen=True)
class OneStepSweepRunResult:
    scenario_id: str
    run_id: str
    output_dir: str
    parameter_name: str
    point_results: tuple[OneStepSweepPointResult, ...]
    sweep_summary: Mapping[str, Any]
    sweep_summary_path: str
    sweep_report_path: str
    artifact_manifest_path: str

    def to_dict(self) -> dict[str, Any]:
        return _to_plain(self)


@dataclass(frozen=True)
class OneStepSuiteRunResult:
    run_id: str
    output_dir: str
    scenario_results: tuple[OneStepScenarioRunResult, ...]
    sweep_results: tuple[OneStepSweepRunResult, ...]
    suite_summary: Mapping[str, Any]
    suite_summary_path: str
    suite_report_path: str
    artifact_manifest_path: str

    def to_dict(self) -> dict[str, Any]:
        return _to_plain(self)


def run_one_step_fixed_scenario(
    scenario_id: str,
    output_dir: str | Path,
    *,
    run_id: str,
) -> OneStepScenarioRunResult:
    scenario = load_one_step_experiment_scenario(scenario_id)
    if scenario.fixed_expectation is None:
        raise ValueError(f"scenario {scenario_id} does not define fixed expectations")

    scenario_dir = Path(output_dir) / scenario_id
    evaluation = _evaluate_catalog_scenario(scenario)
    artifacts = _build_runner_artifacts(scenario, evaluation, scenario_dir, render_artifacts=True)
    validation_passed, validation_issues = _validate_fixed_evaluation(
        scenario.fixed_expectation,
        evaluation,
    )

    summary_path = scenario_dir / "evaluation_summary.json"
    report_path = scenario_dir / "scenario_report.md"
    manifest_path = scenario_dir / "artifact_manifest.json"
    artifact_paths = _collect_fixed_artifact_paths(
        artifacts,
        summary_path=summary_path,
        report_path=report_path,
        manifest_path=manifest_path,
    )
    summary = _build_fixed_summary(
        scenario=scenario,
        evaluation=evaluation,
        artifact_paths=artifact_paths,
        validation_passed=validation_passed,
        validation_issues=validation_issues,
    )
    _write_json(summary, summary_path)
    _write_fixed_scenario_report(
        report_path,
        summary=summary,
        validation_issues=validation_issues,
    )
    write_artifact_manifest(
        _build_fixed_manifest(
            run_id=run_id,
            scenario_id=scenario_id,
            status=evaluation.status,
            artifact_paths=artifact_paths,
            scenario_report_path=report_path,
        ),
        manifest_path,
    )
    return OneStepScenarioRunResult(
        scenario_id=scenario_id,
        run_id=run_id,
        status=evaluation.status,
        scenario_dir=str(scenario_dir),
        evaluation_summary=summary,
        evaluation_summary_path=str(summary_path),
        scenario_report_path=str(report_path),
        artifact_manifest_path=str(manifest_path),
        artifact_paths=artifact_paths,
    )


def run_one_step_fixed_suite(
    output_dir: str | Path = ONE_STEP_OUTPUT_ROOT,
    *,
    run_id: str | None = None,
    scenario_ids: Iterable[str] = ONE_STEP_FIXED_SCENARIO_IDS,
) -> OneStepSuiteRunResult:
    run_id = run_id or _default_run_id()
    suite_dir = Path(output_dir) / run_id
    scenario_results = tuple(
        run_one_step_fixed_scenario(
            scenario_id,
            suite_dir / "fixed",
            run_id=run_id,
        )
        for scenario_id in scenario_ids
    )
    return _finalize_suite_result(
        run_id=run_id,
        suite_dir=suite_dir,
        scenario_results=scenario_results,
        sweep_results=(),
    )


def run_one_step_sweep_scenario(
    scenario_id: str,
    output_dir: str | Path,
    *,
    run_id: str,
) -> OneStepSweepRunResult:
    scenario = load_one_step_experiment_scenario(scenario_id)
    if scenario.sweep is None:
        raise ValueError(f"scenario {scenario_id} does not define a parameter sweep")

    sweep = scenario.sweep
    sweep_dir = Path(output_dir) / scenario_id / sweep.parameter_name
    point_results: list[OneStepSweepPointResult] = []
    for value in sweep.values:
        point_dir = sweep_dir / _slugify_value(value)
        evaluation = _evaluate_catalog_scenario(
            scenario,
            parameter_name=sweep.parameter_name,
            parameter_value=value,
        )
        should_render_artifacts = value in sweep.representative_values_for_plot
        artifacts = _build_runner_artifacts(
            scenario,
            evaluation,
            point_dir,
            render_artifacts=should_render_artifacts,
        )
        summary_path = point_dir / "evaluation_summary.json"
        point_artifact_paths = _collect_fixed_artifact_paths(
            artifacts,
            summary_path=summary_path,
            report_path=None,
            manifest_path=None,
        )
        point_summary = _build_sweep_point_summary(
            scenario=scenario,
            parameter_name=sweep.parameter_name,
            parameter_value=value,
            evaluation=evaluation,
            artifact_paths=point_artifact_paths,
        )
        _write_json(point_summary, summary_path)
        point_results.append(
            OneStepSweepPointResult(
                scenario_id=scenario_id,
                run_id=run_id,
                parameter_name=sweep.parameter_name,
                parameter_value=value,
                status=evaluation.status,
                evaluation_summary=point_summary,
                artifact_paths=point_artifact_paths,
            )
        )

    sweep_summary_path = sweep_dir / "sweep_summary.json"
    sweep_report_path = sweep_dir / "sweep_report.md"
    manifest_path = sweep_dir / "artifact_manifest.json"
    sweep_summary = _build_sweep_summary(
        scenario=scenario,
        point_results=tuple(point_results),
    )
    _write_json(sweep_summary, sweep_summary_path)
    _write_sweep_report(
        sweep_report_path,
        sweep_summary=sweep_summary,
        point_results=tuple(point_results),
    )
    write_artifact_manifest(
        _build_sweep_manifest(
            run_id=run_id,
            scenario_id=scenario_id,
            parameter_name=sweep.parameter_name,
            point_results=tuple(point_results),
        ),
        manifest_path,
    )
    return OneStepSweepRunResult(
        scenario_id=scenario_id,
        run_id=run_id,
        output_dir=str(sweep_dir),
        parameter_name=sweep.parameter_name,
        point_results=tuple(point_results),
        sweep_summary=sweep_summary,
        sweep_summary_path=str(sweep_summary_path),
        sweep_report_path=str(sweep_report_path),
        artifact_manifest_path=str(manifest_path),
    )


def run_one_step_full_suite(
    output_dir: str | Path = ONE_STEP_OUTPUT_ROOT,
    *,
    run_id: str | None = None,
) -> OneStepSuiteRunResult:
    run_id = run_id or _default_run_id()
    suite_dir = Path(output_dir) / run_id
    scenario_results = tuple(
        run_one_step_fixed_scenario(
            scenario_id,
            suite_dir / "fixed",
            run_id=run_id,
        )
        for scenario_id in ONE_STEP_FIXED_SCENARIO_IDS
    )
    sweep_results = tuple(
        run_one_step_sweep_scenario(
            scenario_id,
            suite_dir / "sweeps",
            run_id=run_id,
        )
        for scenario_id in ONE_STEP_SWEEP_SCENARIO_IDS
    )
    return _finalize_suite_result(
        run_id=run_id,
        suite_dir=suite_dir,
        scenario_results=scenario_results,
        sweep_results=sweep_results,
    )


def _evaluate_catalog_scenario(
    scenario: OneStepExperimentScenario,
    *,
    parameter_name: str | None = None,
    parameter_value: float | None = None,
) -> OneStepEvaluationResult:
    scenario_config = build_one_step_scenario_config(scenario.x_targets)
    algorithm = build_default_one_step_algorithm_config()
    if parameter_name is not None and parameter_value is not None:
        algorithm = apply_sweep_parameter(algorithm, parameter_name, parameter_value)
    return evaluate_one_step_scenario(scenario_config, algorithm)


def _build_runner_artifacts(
    scenario: OneStepExperimentScenario,
    evaluation: OneStepEvaluationResult,
    output_dir: Path,
    *,
    render_artifacts: bool,
) -> OneStepScenarioArtifacts:
    if not render_artifacts:
        return OneStepScenarioArtifacts(
            evaluation=evaluation,
            bundle=None,
            trajectory_csv_path=None,
            xt_plot_path=None,
            vt_plot_path=None,
        )
    if evaluation.status == "no_solution":
        return OneStepScenarioArtifacts(
            evaluation=evaluation,
            bundle=None,
            trajectory_csv_path=None,
            xt_plot_path=None,
            vt_plot_path=None,
        )
    contract = (
        get_reference_expected().trajectory_contract
        if scenario.scenario_id == "S01"
        else build_one_step_trajectory_contract(evaluation)
    )
    return build_one_step_artifacts(evaluation, contract, output_dir)


def _collect_fixed_artifact_paths(
    artifacts: OneStepScenarioArtifacts,
    *,
    summary_path: Path,
    report_path: Path | None,
    manifest_path: Path | None,
) -> dict[str, str]:
    artifact_paths = {"evaluation_summary": str(summary_path)}
    if report_path is not None:
        artifact_paths["scenario_report"] = str(report_path)
    if manifest_path is not None:
        artifact_paths["artifact_manifest"] = str(manifest_path)
    if artifacts.trajectory_csv_path is not None:
        artifact_paths["trajectory_csv"] = artifacts.trajectory_csv_path
    if artifacts.xt_plot_path is not None:
        artifact_paths["x_t_plot"] = artifacts.xt_plot_path
    if artifacts.vt_plot_path is not None:
        artifact_paths["v_t_plot"] = artifacts.vt_plot_path
    return artifact_paths


def _validate_fixed_evaluation(
    expected: OneStepFixedExpectation,
    evaluation: OneStepEvaluationResult,
) -> tuple[bool, list[str]]:
    issues: list[str] = []
    if evaluation.status != expected.expected_status:
        issues.append(
            f"expected status {expected.expected_status}, got {evaluation.status}"
        )
    if evaluation.no_solution_reason != expected.expected_no_solution_reason:
        issues.append(
            f"expected no_solution_reason {expected.expected_no_solution_reason}, got {evaluation.no_solution_reason}"
        )

    if expected.expected_best_gap_id is not None:
        actual_gap_id = evaluation.best_gap.gap_id if evaluation.best_gap is not None else None
        if actual_gap_id != expected.expected_best_gap_id:
            issues.append(
                f"expected best_gap_id {expected.expected_best_gap_id}, got {actual_gap_id}"
            )
    if expected.expected_best_gap_interval is not None:
        actual_interval = (
            (evaluation.best_gap.x_rear, evaluation.best_gap.x_front)
            if evaluation.best_gap is not None
            else None
        )
        if actual_interval != expected.expected_best_gap_interval:
            issues.append(
                f"expected best_gap_interval {expected.expected_best_gap_interval}, got {actual_interval}"
            )

    best_score = evaluation.best_score
    if expected.expected_delta_f_star is not None:
        actual = None if best_score is None else best_score.delta_f_star
        if not _is_close(actual, expected.expected_delta_f_star):
            issues.append(
                f"expected delta_f_star {expected.expected_delta_f_star}, got {actual}"
            )
    if expected.expected_delta_r_star is not None:
        actual = None if best_score is None else best_score.delta_r_star
        if not _is_close(actual, expected.expected_delta_r_star):
            issues.append(
                f"expected delta_r_star {expected.expected_delta_r_star}, got {actual}"
            )
    if expected.expected_d_i is not None:
        actual = None if best_score is None else best_score.d_i
        if not _is_close(actual, expected.expected_d_i):
            issues.append(f"expected d_i {expected.expected_d_i}, got {actual}")

    gap_rows = {row.gap_id: row for row in evaluation.gap_rows}
    for gap_id in expected.must_exclude_gap_ids:
        if gap_rows[gap_id].included_in_scoring:
            issues.append(f"expected {gap_id} to be excluded from scoring")
    for gap_id in expected.must_have_reachable_gap_ids:
        if not gap_rows[gap_id].reachable:
            issues.append(f"expected {gap_id} to be reachable")
    for gap_id in expected.must_have_infeasible_gap_ids:
        row = gap_rows[gap_id]
        if row.coop_feasible is not False:
            issues.append(f"expected {gap_id} to be coop infeasible")

    return (not issues, issues)


def _build_fixed_summary(
    *,
    scenario: OneStepExperimentScenario,
    evaluation: OneStepEvaluationResult,
    artifact_paths: Mapping[str, str],
    validation_passed: bool,
    validation_issues: list[str],
) -> dict[str, Any]:
    best_gap = evaluation.best_gap
    best_score = evaluation.best_score
    return {
        "scenario_id": scenario.scenario_id,
        "description": scenario.description,
        "purpose": scenario.purpose,
        "status": evaluation.status,
        "no_solution_reason": evaluation.no_solution_reason,
        "scenario_inputs": _to_plain(evaluation.scenario),
        "algorithm_inputs": _to_plain(evaluation.algorithm),
        "best_gap_id": None if best_gap is None else best_gap.gap_id,
        "best_gap_interval": None if best_gap is None else [best_gap.x_rear, best_gap.x_front],
        "best_delta_f_star": None if best_score is None else best_score.delta_f_star,
        "best_delta_r_star": None if best_score is None else best_score.delta_r_star,
        "best_d_i": None if best_score is None else best_score.d_i,
        "best_t_m": None if best_score is None else best_score.t_m,
        "best_p_m": None if best_score is None else best_score.p_m,
        "best_J": None if best_score is None else best_score.J,
        "top2_margin": _compute_top2_margin(evaluation),
        "gap_rows": [_gap_row_to_dict(row) for row in evaluation.gap_rows],
        "artifact_paths": dict(artifact_paths),
        "validation_passed": validation_passed,
        "validation_issues": list(validation_issues),
        "plots_skipped": evaluation.status == "no_solution",
        "tags": list(scenario.tags),
    }


def _build_sweep_point_summary(
    *,
    scenario: OneStepExperimentScenario,
    parameter_name: str,
    parameter_value: float,
    evaluation: OneStepEvaluationResult,
    artifact_paths: Mapping[str, str],
) -> dict[str, Any]:
    best_gap = evaluation.best_gap
    best_score = evaluation.best_score
    return {
        "scenario_id": scenario.scenario_id,
        "description": scenario.description,
        "parameter_name": parameter_name,
        "parameter_value": parameter_value,
        "status": evaluation.status,
        "no_solution_reason": evaluation.no_solution_reason,
        "best_gap_id": None if best_gap is None else best_gap.gap_id,
        "best_gap_interval": None if best_gap is None else [best_gap.x_rear, best_gap.x_front],
        "best_delta_f_star": None if best_score is None else best_score.delta_f_star,
        "best_delta_r_star": None if best_score is None else best_score.delta_r_star,
        "best_d_i": None if best_score is None else best_score.d_i,
        "best_t_m": None if best_score is None else best_score.t_m,
        "best_p_m": None if best_score is None else best_score.p_m,
        "best_J": None if best_score is None else best_score.J,
        "best_C_coop": None if best_score is None else best_score.C_coop,
        "best_C_ego": None if best_score is None else best_score.C_ego,
        "gap_rows": [_gap_row_to_dict(row) for row in evaluation.gap_rows],
        "artifact_paths": dict(artifact_paths),
    }


def _build_sweep_summary(
    *,
    scenario: OneStepExperimentScenario,
    point_results: tuple[OneStepSweepPointResult, ...],
) -> dict[str, Any]:
    sweep = scenario.sweep
    if sweep is None:
        raise ValueError("sweep summary requires a sweep definition")
    return {
        "scenario_id": scenario.scenario_id,
        "parameter_name": sweep.parameter_name,
        "parameter_values": [result.parameter_value for result in point_results],
        "point_summaries": [dict(result.evaluation_summary) for result in point_results],
        "trend_observation": _observe_sweep_trend(scenario, point_results),
        "representative_plot_values": list(sweep.representative_values_for_plot),
        "expected_trend": sweep.expected_trend,
        "status_counts": _count_statuses(result.status for result in point_results),
    }


def _finalize_suite_result(
    *,
    run_id: str,
    suite_dir: Path,
    scenario_results: tuple[OneStepScenarioRunResult, ...],
    sweep_results: tuple[OneStepSweepRunResult, ...],
) -> OneStepSuiteRunResult:
    suite_summary_path = suite_dir / "suite_summary.json"
    suite_report_path = suite_dir / "suite_report.md"
    manifest_path = suite_dir / "artifact_manifest.json"
    suite_summary = _build_suite_summary(
        run_id=run_id,
        output_dir=suite_dir,
        scenario_results=scenario_results,
        sweep_results=sweep_results,
    )
    _write_json(suite_summary, suite_summary_path)
    _write_suite_report(
        suite_report_path,
        suite_summary=suite_summary,
        scenario_results=scenario_results,
        sweep_results=sweep_results,
    )
    write_artifact_manifest(
        _build_suite_manifest(
            run_id=run_id,
            scenario_results=scenario_results,
            sweep_results=sweep_results,
        ),
        manifest_path,
    )
    return OneStepSuiteRunResult(
        run_id=run_id,
        output_dir=str(suite_dir),
        scenario_results=scenario_results,
        sweep_results=sweep_results,
        suite_summary=suite_summary,
        suite_summary_path=str(suite_summary_path),
        suite_report_path=str(suite_report_path),
        artifact_manifest_path=str(manifest_path),
    )


def _build_suite_summary(
    *,
    run_id: str,
    output_dir: Path,
    scenario_results: tuple[OneStepScenarioRunResult, ...],
    sweep_results: tuple[OneStepSweepRunResult, ...],
) -> dict[str, Any]:
    return {
        "run_id": run_id,
        "output_dir": str(output_dir),
        "fixed_scenario_count": len(scenario_results),
        "sweep_scenario_count": len(sweep_results),
        "status_counts": _count_statuses(
            [result.status for result in scenario_results]
            + [point.status for sweep in sweep_results for point in sweep.point_results]
        ),
        "fixed_scenario_summaries": [
            {
                "scenario_id": result.scenario_id,
                "status": result.status,
                "best_gap_id": result.evaluation_summary["best_gap_id"],
                "best_t_m": result.evaluation_summary["best_t_m"],
                "validation_passed": result.evaluation_summary["validation_passed"],
            }
            for result in scenario_results
        ],
        "sweep_summaries": [
            {
                "scenario_id": result.scenario_id,
                "parameter_name": result.parameter_name,
                "point_count": len(result.point_results),
                "representative_plot_values": result.sweep_summary["representative_plot_values"],
                "trend_observation": result.sweep_summary["trend_observation"],
            }
            for result in sweep_results
        ],
        "all_scenarios_have_artifacts": all(
            Path(result.evaluation_summary_path).exists()
            and Path(result.scenario_report_path).exists()
            and Path(result.artifact_manifest_path).exists()
            for result in scenario_results
        ),
    }


def _build_fixed_manifest(
    *,
    run_id: str,
    scenario_id: str,
    status: str,
    artifact_paths: Mapping[str, str],
    scenario_report_path: Path,
) -> ArtifactManifest:
    return ArtifactManifest(
        run_id=run_id,
        entries=(
            ArtifactManifestEntry(
                scenario_id=scenario_id,
                run_id=run_id,
                status=status,
                input_config_ref=f"one_step_experiment:{scenario_id}",
                exports=dict(artifact_paths),
                scenario_report_path=str(scenario_report_path),
                human_summary_path=artifact_paths.get("evaluation_summary"),
            ),
        ),
    )


def _build_sweep_manifest(
    *,
    run_id: str,
    scenario_id: str,
    parameter_name: str,
    point_results: tuple[OneStepSweepPointResult, ...],
) -> ArtifactManifest:
    return ArtifactManifest(
        run_id=run_id,
        entries=tuple(
            ArtifactManifestEntry(
                scenario_id=f"{scenario_id}:{parameter_name}={_slugify_value(point.parameter_value)}",
                run_id=run_id,
                status=point.status,
                input_config_ref=f"one_step_experiment:{scenario_id}:{parameter_name}={point.parameter_value}",
                exports=dict(point.artifact_paths),
                human_summary_path=point.artifact_paths.get("evaluation_summary"),
            )
            for point in point_results
        ),
    )


def _build_suite_manifest(
    *,
    run_id: str,
    scenario_results: tuple[OneStepScenarioRunResult, ...],
    sweep_results: tuple[OneStepSweepRunResult, ...],
) -> ArtifactManifest:
    entries: list[ArtifactManifestEntry] = []
    entries.extend(
        ArtifactManifestEntry(
            scenario_id=result.scenario_id,
            run_id=run_id,
            status=result.status,
            input_config_ref=f"one_step_experiment:{result.scenario_id}",
            exports=dict(result.artifact_paths),
            scenario_report_path=result.scenario_report_path,
            human_summary_path=result.evaluation_summary_path,
        )
        for result in scenario_results
    )
    entries.extend(
        ArtifactManifestEntry(
            scenario_id=result.scenario_id,
            run_id=run_id,
            status="sweep",
            input_config_ref=f"one_step_experiment:{result.scenario_id}:{result.parameter_name}",
            exports={
                "sweep_summary": result.sweep_summary_path,
                "sweep_report": result.sweep_report_path,
                "artifact_manifest": result.artifact_manifest_path,
            },
            scenario_report_path=result.sweep_report_path,
            human_summary_path=result.sweep_summary_path,
        )
        for result in sweep_results
    )
    return ArtifactManifest(run_id=run_id, entries=tuple(entries))


def _write_fixed_scenario_report(
    path: Path,
    *,
    summary: Mapping[str, Any],
    validation_issues: list[str],
) -> Path:
    gap_rows = summary["gap_rows"]
    lines = [
        f"# {summary['scenario_id']} One-Step Scenario",
        "",
        f"- description: `{summary['description']}`",
        f"- purpose: `{summary['purpose']}`",
        f"- status: `{summary['status']}`",
        f"- no_solution_reason: `{summary['no_solution_reason']}`",
        f"- validation_passed: `{summary['validation_passed']}`",
        f"- plots_skipped: `{summary['plots_skipped']}`",
        "",
        "## Best Result",
        "",
        f"- best_gap_id: `{summary['best_gap_id']}`",
        f"- best_gap_interval: `{summary['best_gap_interval']}`",
        f"- best_delta_f_star: `{summary['best_delta_f_star']}`",
        f"- best_delta_r_star: `{summary['best_delta_r_star']}`",
        f"- best_d_i: `{summary['best_d_i']}`",
        f"- best_t_m: `{summary['best_t_m']}`",
        f"- best_p_m: `{summary['best_p_m']}`",
        f"- best_J: `{summary['best_J']}`",
        f"- top2_margin: `{summary['top2_margin']}`",
        "",
        "## Gap Rows",
        "",
        "| gap_id | reachable | coop_feasible | included_in_scoring | delta_f_star | delta_r_star | d_i | t_m | J |",
        "| --- | --- | --- | --- | ---: | ---: | ---: | ---: | ---: |",
    ]
    for row in gap_rows:
        lines.append(
            "| {gap_id} | {reachable} | {coop_feasible} | {included_in_scoring} | {delta_f_star} | {delta_r_star} | {d_i} | {t_m} | {J} |".format(
                **row
            )
        )
    lines.extend(["", "## Validation", ""])
    if not validation_issues:
        lines.append("- passed")
    else:
        lines.extend(f"- {issue}" for issue in validation_issues)
    lines.extend(["", "## Artifacts", ""])
    for name, artifact_path in summary["artifact_paths"].items():
        lines.append(f"- {name}: `{artifact_path}`")
    _ensure_parent(path).write_text("\n".join(lines) + "\n", encoding="utf-8")
    return path


def _write_sweep_report(
    path: Path,
    *,
    sweep_summary: Mapping[str, Any],
    point_results: tuple[OneStepSweepPointResult, ...],
) -> Path:
    lines = [
        f"# {sweep_summary['scenario_id']} Sweep",
        "",
        f"- parameter_name: `{sweep_summary['parameter_name']}`",
        f"- representative_plot_values: `{sweep_summary['representative_plot_values']}`",
        f"- expected_trend: `{sweep_summary['expected_trend']}`",
        f"- trend_observation: `{sweep_summary['trend_observation']}`",
        "",
        "| value | status | best_gap_id | best_t_m | best_J | best_C_coop | best_C_ego |",
        "| ---: | --- | --- | ---: | ---: | ---: | ---: |",
    ]
    for point in point_results:
        summary = point.evaluation_summary
        lines.append(
            "| {parameter_value} | {status} | {best_gap_id} | {best_t_m} | {best_J} | {best_C_coop} | {best_C_ego} |".format(
                parameter_value=point.parameter_value,
                status=point.status,
                best_gap_id=summary["best_gap_id"],
                best_t_m=summary["best_t_m"],
                best_J=summary["best_J"],
                best_C_coop=summary["best_C_coop"],
                best_C_ego=summary["best_C_ego"],
            )
        )
    lines.extend(["", "## Point Artifacts", ""])
    for point in point_results:
        lines.append(f"- {point.parameter_value}: `{point.artifact_paths}`")
    _ensure_parent(path).write_text("\n".join(lines) + "\n", encoding="utf-8")
    return path


def _write_suite_report(
    path: Path,
    *,
    suite_summary: Mapping[str, Any],
    scenario_results: tuple[OneStepScenarioRunResult, ...],
    sweep_results: tuple[OneStepSweepRunResult, ...],
) -> Path:
    lines = [
        "# One-Step Suite",
        "",
        f"- run_id: `{suite_summary['run_id']}`",
        f"- fixed_scenario_count: `{suite_summary['fixed_scenario_count']}`",
        f"- sweep_scenario_count: `{suite_summary['sweep_scenario_count']}`",
        f"- status_counts: `{suite_summary['status_counts']}`",
        "",
        "## Fixed Scenarios",
        "",
        "| scenario_id | status | best_gap_id | best_t_m | validation_passed |",
        "| --- | --- | --- | ---: | --- |",
    ]
    for result in scenario_results:
        lines.append(
            "| {scenario_id} | {status} | {best_gap_id} | {best_t_m} | {validation_passed} |".format(
                scenario_id=result.scenario_id,
                status=result.status,
                best_gap_id=result.evaluation_summary["best_gap_id"],
                best_t_m=result.evaluation_summary["best_t_m"],
                validation_passed=result.evaluation_summary["validation_passed"],
            )
        )
    lines.extend(["", "## Sweeps", ""])
    if not sweep_results:
        lines.append("- none")
    else:
        lines.extend(
            f"- {result.scenario_id} / {result.parameter_name}: `{result.sweep_summary['trend_observation']}`"
            for result in sweep_results
        )
    lines.extend(["", "## Artifact Index", ""])
    lines.extend(f"- {result.scenario_id}: `{result.scenario_report_path}`" for result in scenario_results)
    lines.extend(f"- {result.scenario_id}: `{result.sweep_report_path}`" for result in sweep_results)
    _ensure_parent(path).write_text("\n".join(lines) + "\n", encoding="utf-8")
    return path


def _observe_sweep_trend(
    scenario: OneStepExperimentScenario,
    point_results: tuple[OneStepSweepPointResult, ...],
) -> str:
    sweep = scenario.sweep
    if sweep is None:
        raise ValueError("trend observation requires a sweep definition")
    if sweep.parameter_name == "w_c":
        first = point_results[0].evaluation_summary
        last = point_results[-1].evaluation_summary
        return (
            f"best_gap 从 {first['best_gap_id']} 变化到 {last['best_gap_id']}，"
            f"best_C_coop 从 {first['best_C_coop']} 下降到 {last['best_C_coop']}。"
        )
    if sweep.parameter_name == "q":
        gap4_biases = [
            next(
                row["delta_f_star"] - row["delta_r_star"]
                for row in point.evaluation_summary["gap_rows"]
                if row["gap_id"] == "gap4"
            )
            for point in point_results
        ]
        return (
            f"gap4 的 (delta_f_star-delta_r_star) 从 {gap4_biases[0]} 变化到 {gap4_biases[-1]}，"
            f"q=6 时 best_gap={point_results[4].evaluation_summary['best_gap_id']}。"
        )
    if sweep.parameter_name == "w_t":
        first = point_results[0].evaluation_summary
        last = point_results[-1].evaluation_summary
        return (
            f"best_t_m 从 {first['best_t_m']} 下降到 {last['best_t_m']}，"
            f"best_gap 从 {first['best_gap_id']} 变化到 {last['best_gap_id']}。"
        )
    return sweep.expected_trend


def _gap_row_to_dict(row: GapEvaluationRow) -> dict[str, Any]:
    return _to_plain(row)


def _compute_top2_margin(evaluation: OneStepEvaluationResult) -> float | None:
    if len(evaluation.scores) < 2:
        return None
    ordered = sorted(evaluation.scores, key=lambda score: (score.J, score.gap_index))
    return ordered[1].J - ordered[0].J


def _count_statuses(statuses: Iterable[str]) -> dict[str, int]:
    counts: dict[str, int] = {}
    for status in statuses:
        counts[status] = counts.get(status, 0) + 1
    return counts


def _slugify_value(value: float) -> str:
    text = format(value, "g")
    return text.replace("-", "neg").replace(".", "p")


def _write_json(payload: Mapping[str, Any], path: Path) -> Path:
    output_path = _ensure_parent(path)
    output_path.write_text(
        json.dumps(_to_plain(payload), ensure_ascii=False, indent=2, sort_keys=True),
        encoding="utf-8",
    )
    return output_path


def _ensure_parent(path: str | Path) -> Path:
    output_path = Path(path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    return output_path


def _default_run_id() -> str:
    return datetime.now(UTC).strftime("one_step_%Y%m%d_%H%M%S")


def _is_close(left: float | None, right: float, *, tol: float = 1e-6) -> bool:
    if left is None:
        return False
    return abs(left - right) <= tol


def _to_plain(value: Any) -> Any:
    if hasattr(value, "to_dict"):
        return value.to_dict()
    if is_dataclass(value):
        return {field: _to_plain(item) for field, item in asdict(value).items()}
    if isinstance(value, Mapping):
        return {str(key): _to_plain(item) for key, item in value.items()}
    if isinstance(value, tuple):
        return [_to_plain(item) for item in value]
    if isinstance(value, list):
        return [_to_plain(item) for item in value]
    return value
