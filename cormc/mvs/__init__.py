"""MVS runner v0 for P01 targeted scenario contracts."""

from cormc.mvs.loader import (
    BUILTIN_SCENARIOS,
    ScenarioConfigError,
    classify_scenario_status,
    load_builtin_scenario,
    load_scenario_config,
)
from cormc.mvs.matcher import (
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
_RUNNER_EXPORTS = {
    "ScenarioReport",
    "ScenarioRunResult",
    "ScenarioRuntimeContext",
    "build_scenario_report",
    "run_targeted_scenario",
}


def __getattr__(name: str):
    if name in _RUNNER_EXPORTS:
        from cormc.mvs import runner

        value = getattr(runner, name)
        globals()[name] = value
        return value
    raise AttributeError(f"module 'cormc.mvs' has no attribute {name!r}")

__all__ = [
    "BUILTIN_SCENARIOS",
    "MatcherIssue",
    "MatcherResult",
    "ScenarioConfigError",
    "ScenarioReport",
    "ScenarioRunResult",
    "ScenarioRuntimeContext",
    "build_scenario_report",
    "classify_scenario_status",
    "compare_with_tolerance",
    "load_builtin_scenario",
    "load_scenario_config",
    "match_expected_events",
    "match_expected_png_features_v0",
    "match_expected_sanity_checks",
    "match_event_counts",
    "match_forbidden_events",
    "register_expected_png_features",
    "run_targeted_scenario",
]
