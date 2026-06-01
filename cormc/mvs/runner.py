from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from cormc.mvs.loader import classify_scenario_status, load_builtin_scenario, load_scenario_config
from cormc.mvs.matcher import (
    MatcherResult,
    match_expected_events,
    match_expected_png_features_v0,
    match_expected_sanity_checks,
    match_event_counts,
    match_forbidden_events,
)
from cormc.step4b_cmc import run_step4b_cmc_for_scenario
from cormc.step4a_aps import run_step4a_aps_for_scenario
from cormc.step9_11 import run_mvs_commit_1_lite


@dataclass(frozen=True)
class ScenarioRuntimeContext:
    config: dict[str, Any]

    @property
    def scenario_id(self) -> str:
        return self.config["scenario_id"]

    @property
    def status(self) -> str:
        return classify_scenario_status(self.config)


@dataclass(frozen=True)
class ScenarioRunResult:
    actual_events: list[dict[str, Any]] = field(default_factory=list)
    actual_sanity_checks: list[dict[str, Any]] = field(default_factory=list)
    actual_png_artifacts: list[dict[str, Any]] = field(default_factory=list)


@dataclass(frozen=True)
class ScenarioReport:
    scenario_id: str
    test_level: str
    status: str
    classification: str
    passed: bool
    blocks_required_suite: bool
    failure_reasons: list[str]
    matcher_results: list[MatcherResult]
    registered_png_features: list[dict[str, Any]]

    def to_dict(self) -> dict[str, Any]:
        return {
            "scenario_id": self.scenario_id,
            "test_level": self.test_level,
            "status": self.status,
            "classification": self.classification,
            "passed": self.passed,
            "blocks_required_suite": self.blocks_required_suite,
            "failure_reasons": list(self.failure_reasons),
            "matcher_results": [
                {
                    "name": result.name,
                    "passed": result.passed,
                    "issues": [
                        {
                            "code": issue.code,
                            "message": issue.message,
                            "expected": issue.expected,
                            "actual": issue.actual,
                            "required": issue.required,
                        }
                        for issue in result.issues
                    ],
                    "registered": result.registered,
                }
                for result in self.matcher_results
            ],
            "registered_png_features": list(self.registered_png_features),
        }


def run_targeted_scenario(
    scenario: str | dict[str, Any],
    *,
    actual_events: list[dict[str, Any]] | None = None,
    actual_sanity_checks: list[dict[str, Any]] | None = None,
) -> ScenarioReport:
    if isinstance(scenario, str):
        config = load_builtin_scenario(scenario)
    else:
        config = load_scenario_config(scenario)
    context = ScenarioRuntimeContext(config=config)
    if context.status == "deferred":
        return build_scenario_report(context, ScenarioRunResult(), skipped_deferred=True)

    if (
        actual_events is None
        and actual_sanity_checks is None
        and _is_p04_aps_scenario(context.scenario_id)
    ):
        aps_result = run_step4a_aps_for_scenario(config)
        return build_scenario_report(
            context,
            ScenarioRunResult(
                actual_events=aps_result.actual_events,
                actual_sanity_checks=aps_result.actual_sanity_checks,
            ),
        )

    if (
        actual_events is None
        and actual_sanity_checks is None
        and _is_p05_cmc_scenario(context.scenario_id)
    ):
        cmc_result = run_step4b_cmc_for_scenario(config)
        return build_scenario_report(
            context,
            ScenarioRunResult(
                actual_events=cmc_result.actual_events,
                actual_sanity_checks=cmc_result.actual_sanity_checks,
                actual_png_artifacts=cmc_result.expected_png_features,
            ),
        )

    if (
        actual_events is None
        and actual_sanity_checks is None
        and context.scenario_id == "MVS-COMMIT-1-lite"
    ):
        commit_result = run_mvs_commit_1_lite()
        return build_scenario_report(
            context,
            ScenarioRunResult(
                actual_events=commit_result.history.event_dicts(),
                actual_sanity_checks=commit_result.history.sanity_dicts(),
            ),
        )

    result = ScenarioRunResult(
        actual_events=list(actual_events or []),
        actual_sanity_checks=list(actual_sanity_checks or []),
    )
    return build_scenario_report(context, result)


def _is_p04_aps_scenario(scenario_id: str) -> bool:
    return scenario_id.startswith("MVS-APS") or scenario_id.startswith("P04-")


def _is_p05_cmc_scenario(scenario_id: str) -> bool:
    return (
        scenario_id.startswith("MVS-CMC")
        or scenario_id.startswith("MVS-ASSIGN")
        or scenario_id == "MVS-SAFE-1A_waiting_cap"
        or scenario_id.startswith("P05-")
    )


def build_scenario_report(
    context: ScenarioRuntimeContext,
    result: ScenarioRunResult,
    *,
    skipped_deferred: bool = False,
) -> ScenarioReport:
    status = context.status
    event_result = match_expected_events(
        context.config["expected_events"],
        result.actual_events,
        context.config["tolerances"],
    )
    forbidden_result = match_forbidden_events(
        context.config["forbidden_events"],
        result.actual_events,
    )
    event_count_result = match_event_counts(
        context.config["expected_event_counts"],
        result.actual_events,
    )
    sanity_result = match_expected_sanity_checks(
        context.config["expected_sanity_checks"],
        result.actual_sanity_checks,
    )
    png_result = match_expected_png_features_v0(context.config["expected_png_features"])
    matcher_results = [
        event_result,
        forbidden_result,
        event_count_result,
        sanity_result,
        png_result,
    ]

    required_failures = [
        issue.message
        for matcher_result in matcher_results
        for issue in matcher_result.issues
        if issue.required
    ]
    if skipped_deferred:
        classification = "skipped_deferred"
        passed = True
        blocks_required_suite = False
        failure_reasons: list[str] = []
    elif status == "required":
        classification = "required_failed" if required_failures else "required_passed"
        passed = not required_failures
        blocks_required_suite = bool(required_failures)
        failure_reasons = required_failures
    elif status == "probe":
        classification = "probe_failed" if required_failures else "probe_observation"
        passed = True
        blocks_required_suite = False
        failure_reasons = required_failures
    elif status == "deferred":
        classification = "loaded_deferred"
        passed = True
        blocks_required_suite = False
        failure_reasons = []
    else:
        classification = status
        passed = not required_failures
        blocks_required_suite = False
        failure_reasons = required_failures

    return ScenarioReport(
        scenario_id=context.scenario_id,
        test_level=str(context.config["test_level"]),
        status=status,
        classification=classification,
        passed=passed,
        blocks_required_suite=blocks_required_suite,
        failure_reasons=failure_reasons,
        matcher_results=matcher_results,
        registered_png_features=png_result.registered,
    )
