"""Scenario config schema, matching, and reporting helpers."""

from cormc.scenario_schema.config import (
    ScenarioConfigError,
    classify_scenario_status,
    load_scenario_config,
)
from cormc.scenario_schema.matcher import (
    MatcherIssue,
    MatcherResult,
    compare_with_tolerance,
    match_expected_events,
    match_expected_png_features_v0,
    match_expected_sanity_checks,
    match_event_counts,
    match_forbidden_events,
    register_expected_png_features,
)
_REPORTING_EXPORTS = {
    "ScenarioReport",
    "ScenarioRunResult",
    "ScenarioRuntimeContext",
    "build_scenario_report",
    "run_targeted_scenario",
}


def __getattr__(name: str):
    if name in _REPORTING_EXPORTS:
        from cormc.scenario_schema import reporting

        value = getattr(reporting, name)
        globals()[name] = value
        return value
    raise AttributeError(f"module 'cormc.scenario_schema' has no attribute {name!r}")

__all__ = [
    "MatcherIssue",
    "MatcherResult",
    "ScenarioConfigError",
    "ScenarioReport",
    "ScenarioRunResult",
    "ScenarioRuntimeContext",
    "build_scenario_report",
    "classify_scenario_status",
    "compare_with_tolerance",
    "load_scenario_config",
    "match_expected_events",
    "match_expected_png_features_v0",
    "match_expected_sanity_checks",
    "match_event_counts",
    "match_forbidden_events",
    "register_expected_png_features",
    "run_targeted_scenario",
]
