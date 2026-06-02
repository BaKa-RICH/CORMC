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
from cormc.step5_cooperative_request import (
    run_step5_cooperative_request_conflict_resolution_for_scenario,
)
from cormc.step9_11 import run_mvs_commit_1_lite


DETERMINISTIC_SCENARIO_ROUTES: dict[str, dict[str, Any]] = {
    "MVS-E2E-1": {"max_steps": 70},
    "MVS-CUC-1A_override_choice1": {"max_steps": 1},
    "MVS-CUC-2": {"max_steps": 1},
    "MVS-CUC-3": {"max_steps": 1},
    "MVS-SAFE-1A_waiting_cap": {"max_steps": 1},
    "MVS-SAFE-1B_executing_cap_lateral_consumption": {"max_steps": 1},
    "MVS-SAFE-2": {"max_steps": 1},
    "MVS-COMMIT-1-full": {"max_steps": 1},
}

MVS_SCENARIO_ROUTE_MATRIX: dict[str, str] = {
    "MVS-APS-FAIL-EMPTY": "aps_helper",
    "MVS-APS-FAIL-CACHE": "aps_helper",
    "MVS-APS-1": "aps_helper",
    "MVS-APS-2": "aps_helper",
    "MVS-APS-3": "aps_helper",
    "MVS-APS-4": "aps_helper",
    "MVS-E2E-1": "deterministic_loop",
    "MVS-COMMIT-1-lite": "commit_lite_helper",
    "MVS-CMC-1": "cmc_helper",
    "MVS-CMC-2": "cmc_helper",
    "MVS-CUC-1A_override_choice1": "deterministic_loop",
    "MVS-CUC-2": "deterministic_loop",
    "MVS-CUC-3": "deterministic_loop",
    "MVS-SAFE-1A_waiting_cap": "deterministic_loop",
    "MVS-SAFE-1B_executing_cap_lateral_consumption": "deterministic_loop",
    "MVS-SAFE-2": "deterministic_loop",
    "MVS-ASSIGN-1": "cmc_helper",
    "MVS-CONFLICT-1A": "cooperative_request_helper",
    "MVS-CONFLICT-1B": "cooperative_request_helper",
    "MVS-COMMIT-1-full": "deterministic_loop",
    "MVS-CUC-1B_real_utility_probe": "probe_registered_no_required_route",
    "MVS-CUC-1C_real_utility_choice1_locked": "deferred_skipped",
}


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
        and context.scenario_id in DETERMINISTIC_SCENARIO_ROUTES
    ):
        return _run_deterministic_route(
            context,
            max_steps=int(DETERMINISTIC_SCENARIO_ROUTES[context.scenario_id]["max_steps"]),
        )

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
        and _is_p06_cooperative_request_scenario(context.scenario_id)
    ):
        p06_result = run_step5_cooperative_request_conflict_resolution_for_scenario(config)
        return build_scenario_report(
            context,
            ScenarioRunResult(
                actual_events=p06_result.actual_events,
                actual_sanity_checks=p06_result.actual_sanity_checks,
                actual_png_artifacts=p06_result.expected_png_features,
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
        or scenario_id.startswith("P05-")
    )


def _is_p06_cooperative_request_scenario(scenario_id: str) -> bool:
    return scenario_id.startswith("MVS-CONFLICT") or scenario_id.startswith("P06-")


def _run_deterministic_route(
    context: ScenarioRuntimeContext,
    *,
    max_steps: int,
) -> ScenarioReport:
    from cormc.simulation_loop import SimulationLoopConfig, run_deterministic_simulation

    loop_result = run_deterministic_simulation(
        SimulationLoopConfig(
            scenario=context.config,
            run_id=context.scenario_id,
            max_steps=max_steps,
            render_png=False,
        )
    )
    return build_scenario_report(
        context,
        ScenarioRunResult(
            actual_events=loop_result.history.event_dicts(),
            actual_sanity_checks=loop_result.history.sanity_dicts(),
            actual_png_artifacts=list(loop_result.expected_png_features),
        ),
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
