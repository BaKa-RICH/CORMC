from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


@dataclass(frozen=True)
class MatcherIssue:
    code: str
    message: str
    expected: dict[str, Any] = field(default_factory=dict)
    actual: dict[str, Any] | None = None
    required: bool = True


@dataclass(frozen=True)
class MatcherResult:
    name: str
    passed: bool
    issues: list[MatcherIssue] = field(default_factory=list)
    registered: list[dict[str, Any]] = field(default_factory=list)


def compare_with_tolerance(
    expected: float,
    actual: float,
    tolerance: float,
) -> bool:
    return abs(float(actual) - float(expected)) <= float(tolerance)


def match_expected_events(
    expected_events: list[dict[str, Any]],
    actual_events: list[dict[str, Any]],
    tolerances: dict[str, float],
) -> MatcherResult:
    issues: list[MatcherIssue] = []
    for expected in expected_events:
        required = bool(expected.get("required", True))
        event_type = expected.get("event_type")
        candidates = [
            event
            for event in actual_events
            if event.get("event_type") == event_type or event.get("module") == event_type
        ]
        if not candidates:
            issues.append(
                MatcherIssue(
                    code="missing_event",
                    message=f"missing expected event: {event_type}",
                    expected=expected,
                    required=required,
                )
            )
            continue

        match = _first_matching_event(expected, candidates, tolerances)
        if match is not None:
            continue
        issues.append(
            MatcherIssue(
                code="event_mismatch",
                message=f"no actual event matched expected event: {event_type}",
                expected=expected,
                actual=candidates[0],
                required=required,
            )
        )

    return MatcherResult(
        name="expected_events",
        passed=not _has_required_issue(issues),
        issues=issues,
    )


def match_forbidden_events(
    forbidden_events: list[dict[str, Any]],
    actual_events: list[dict[str, Any]],
) -> MatcherResult:
    issues: list[MatcherIssue] = []
    for expected in forbidden_events:
        event_type = expected.get("event_type")
        candidates = [
            event
            for event in actual_events
            if event.get("event_type") == event_type or event.get("module") == event_type
        ]
        forbidden_matches = [
            candidate
            for candidate in candidates
            if _candidate_matches_expected(expected, candidate, {"derived_formula_abs": 0.0})
        ]
        if forbidden_matches:
            issues.append(
                MatcherIssue(
                    code="forbidden_event_present",
                    message=f"forbidden event present: {event_type}",
                    expected=expected,
                    actual=forbidden_matches[0],
                    required=True,
                )
            )
    return MatcherResult(
        name="forbidden_events",
        passed=not issues,
        issues=issues,
    )


def match_event_counts(
    event_counts: list[dict[str, Any]],
    actual_events: list[dict[str, Any]],
) -> MatcherResult:
    issues: list[MatcherIssue] = []
    for expected in event_counts:
        event_type = expected.get("event_type")
        candidates = [
            event
            for event in actual_events
            if event.get("event_type") == event_type or event.get("module") == event_type
        ]
        matching_count = sum(
            1
            for candidate in candidates
            if _candidate_matches_expected(expected, candidate, {"derived_formula_abs": 0.0})
        )
        expected_count = int(expected["expected_count"])
        comparison = expected.get("comparison", "exactly")
        if not _count_matches(matching_count, expected_count, comparison):
            issues.append(
                MatcherIssue(
                    code="event_count_mismatch",
                    message=(
                        f"event_count mismatch for {event_type}: "
                        f"comparison={comparison}, expected {expected_count}, "
                        f"actual {matching_count}"
                    ),
                    expected=expected,
                    actual={"event_count": matching_count},
                    required=True,
                )
            )
    return MatcherResult(
        name="expected_event_counts",
        passed=not issues,
        issues=issues,
    )


def match_expected_sanity_checks(
    expected_checks: list[dict[str, Any]],
    actual_checks: list[dict[str, Any]],
) -> MatcherResult:
    issues: list[MatcherIssue] = []
    for expected in expected_checks:
        required = bool(expected.get("required", True))
        check_type = expected.get("check_type")
        expected_status = expected.get("expected_status")
        candidates = [check for check in actual_checks if check.get("check_type") == check_type]
        if not candidates:
            issues.append(
                MatcherIssue(
                    code="missing_sanity_check",
                    message=f"missing expected sanity check: {check_type}",
                    expected=expected,
                    required=required,
                )
            )
            continue
        matched = False
        for check in candidates:
            if expected_status is not None and _actual_sanity_status(check) != expected_status:
                continue
            if not _vehicle_ids_match(expected, check):
                continue
            if not _time_window_match(expected, check):
                continue
            if expected.get("reason_code") is not None and check.get("reason") != expected["reason_code"]:
                continue
            matched = True
            break
        if not matched:
            actual_status = _actual_sanity_status(candidates[0])
            issues.append(
                MatcherIssue(
                    code="sanity_check_mismatch",
                    message=(
                        f"sanity check mismatch for {check_type}: "
                        f"expected_status={expected_status}, actual_status={actual_status}"
                    ),
                    expected=expected,
                    actual=candidates[0],
                    required=required,
                )
            )
    return MatcherResult(
        name="expected_sanity_checks",
        passed=not _has_required_issue(issues),
        issues=issues,
    )


def register_expected_png_features(
    expected_features: list[dict[str, Any]],
) -> list[dict[str, Any]]:
    registered = []
    for feature in expected_features:
        registered.append(
            {
                **feature,
                "registration_status": "feature_registered",
                "renderer_status": "renderer_deferred",
            }
        )
    return registered


def match_expected_png_features_v0(
    expected_features: list[dict[str, Any]],
) -> MatcherResult:
    return MatcherResult(
        name="expected_png_features",
        passed=True,
        issues=[],
        registered=register_expected_png_features(expected_features),
    )


def _first_matching_event(
    expected: dict[str, Any],
    candidates: list[dict[str, Any]],
    tolerances: dict[str, float],
) -> dict[str, Any] | None:
    for candidate in candidates:
        if _candidate_matches_expected(expected, candidate, tolerances):
            return candidate
    return None


def _candidate_matches_expected(
    expected: dict[str, Any],
    candidate: dict[str, Any],
    tolerances: dict[str, float],
) -> bool:
    if not _vehicle_ids_match(expected, candidate):
        return False
    if not _time_window_match(expected, candidate):
        return False
    if expected.get("reason_code") is not None and candidate.get("reason") != expected["reason_code"]:
        return False
    if not _source_matches(expected, candidate):
        return False
    if not _match_payload(expected.get("match", {}), candidate):
        return False
    if not _match_numeric(expected.get("numeric_expectations", {}), candidate, tolerances):
        return False
    return True


def _vehicle_ids_match(expected: dict[str, Any], actual: dict[str, Any]) -> bool:
    expected_ids = expected.get("vehicle_ids")
    if expected_ids in (None, []):
        return True
    actual_ids = set(actual.get("vehicle_ids") or actual.get("related_vehicle_ids") or [])
    vehicle_id = actual.get("vehicle_id")
    if vehicle_id is not None:
        actual_ids.add(vehicle_id)
    return set(expected_ids).issubset(actual_ids)


def _match_payload(expected_match: dict[str, Any], actual: dict[str, Any]) -> bool:
    payload = actual.get("payload") or {}
    for key, expected_value in expected_match.items():
        actual_value = payload.get(key, actual.get(key))
        if actual_value != expected_value:
            return False
    return True


def _source_matches(expected: dict[str, Any], actual: dict[str, Any]) -> bool:
    expected_source = expected.get("source")
    if expected_source is None:
        return True
    if actual.get("source") == expected_source:
        return True
    return (
        expected_source == "first_version_engineering_patch"
        and actual.get("is_engineering_patch") is True
    )


def _match_numeric(
    numeric_expectations: dict[str, Any],
    actual: dict[str, Any],
    tolerances: dict[str, float],
) -> bool:
    payload = actual.get("payload") or {}
    for key, spec in numeric_expectations.items():
        actual_value = payload.get(key, actual.get(key))
        if actual_value is None:
            return False
        if isinstance(spec, dict):
            expected_value = spec.get("value")
            tolerance_key = spec.get("tolerance", "derived_formula_abs")
        else:
            expected_value = spec
            tolerance_key = "derived_formula_abs"
        tolerance = tolerances[tolerance_key]
        if not compare_with_tolerance(expected_value, actual_value, tolerance):
            return False
    return True


def _time_window_match(expected: dict[str, Any], actual: dict[str, Any]) -> bool:
    window = expected.get("time_window")
    if window in (None, []):
        return True
    step = actual.get("step")
    t = actual.get("t")
    if isinstance(window, dict):
        if "step" in window and step != window["step"]:
            return False
        if "step_min" in window and (step is None or step < window["step_min"]):
            return False
        if "step_max" in window and (step is None or step > window["step_max"]):
            return False
        if "t" in window and t != window["t"]:
            return False
        if "t_min" in window and (t is None or t < window["t_min"]):
            return False
        if "t_max" in window and (t is None or t > window["t_max"]):
            return False
        return True
    if isinstance(window, (list, tuple)) and len(window) == 2:
        if step is not None:
            return window[0] <= step <= window[1]
        if t is not None:
            return window[0] <= t <= window[1]
        return False
    return step == window or t == window


def _count_matches(actual_count: int, expected_count: int, comparison: str) -> bool:
    if comparison == "exactly":
        return actual_count == expected_count
    if comparison == "at_least":
        return actual_count >= expected_count
    if comparison == "at_most":
        return actual_count <= expected_count
    return False


def _actual_sanity_status(check: dict[str, Any]) -> str | None:
    result = check.get("result")
    if result == "false":
        return "pass"
    if result == "true":
        return "fail"
    return result


def _has_required_issue(issues: list[MatcherIssue]) -> bool:
    return any(issue.required for issue in issues)
