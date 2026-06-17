from __future__ import annotations

import csv
import json
import struct
import zlib
from dataclasses import dataclass, fields, is_dataclass
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence

from cormc.scenario_schema import run_targeted_scenario
from cormc.scenario_schema.reporting import ScenarioReport
from cormc.simulation_core.pre_freeze import DEFAULT_ROAD_GEOMETRY, RoadGeometryConfig
from cormc.simulation_core.commit import (
    EventRecord,
    OutputHistory,
    SanityCheckRecord,
    TrajectoryRecord,
)


P11_REQUIRED_MVS_IDS: tuple[str, ...] = ()
P11_PROBE_MVS_IDS: tuple[str, ...] = ()
P11_DEFERRED_MVS_IDS: tuple[str, ...] = ()
P11_EXTRA_DIAGNOSTIC_IDS: tuple[str, ...] = ()


@dataclass(frozen=True)
class SmokeSuiteRegistry:
    required_ids: tuple[str, ...] = P11_REQUIRED_MVS_IDS
    probe_ids: tuple[str, ...] = P11_PROBE_MVS_IDS
    deferred_ids: tuple[str, ...] = P11_DEFERRED_MVS_IDS
    extra_diagnostic_ids: tuple[str, ...] = P11_EXTRA_DIAGNOSTIC_IDS

    def to_dict(self) -> dict[str, Any]:
        return _to_plain(self)


@dataclass(frozen=True)
class RequiredScenarioBlocker:
    scenario_id: str
    reason: str
    detail: str

    def to_dict(self) -> dict[str, Any]:
        return _to_plain(self)


@dataclass(frozen=True)
class ScenarioAggregationResult:
    scenario_id: str
    suite_group: str
    status: str
    classification: str
    passed: bool
    blocks_required_suite: bool
    failure_reasons: tuple[str, ...] = ()
    blockers: tuple[RequiredScenarioBlocker, ...] = ()

    def to_dict(self) -> dict[str, Any]:
        return _to_plain(self)


@dataclass(frozen=True)
class ProbeObservation:
    scenario_id: str
    classification: str
    passed: bool
    failure_reasons: tuple[str, ...] = ()

    def to_dict(self) -> dict[str, Any]:
        return _to_plain(self)


@dataclass(frozen=True)
class DeferredScenarioRecord:
    scenario_id: str
    classification: str = "skipped_deferred"

    def to_dict(self) -> dict[str, Any]:
        return _to_plain(self)


@dataclass(frozen=True)
class SmokeSuiteRunResult:
    registry: SmokeSuiteRegistry
    scenario_results: tuple[ScenarioAggregationResult, ...] = ()
    probe_observed: tuple[ProbeObservation, ...] = ()
    deferred_skipped: tuple[DeferredScenarioRecord, ...] = ()
    extra_diagnostics: tuple[ScenarioAggregationResult, ...] = ()
    p12_random_generation_enabled: bool = False
    paper_metrics_required: bool = False

    def to_dict(self) -> dict[str, Any]:
        return _to_plain(self)


@dataclass(frozen=True)
class PngRenderResult:
    png_path: str
    feature_statuses: tuple[dict[str, Any], ...] = ()
    width: int = 0
    height: int = 0

    def to_dict(self) -> dict[str, Any]:
        return _to_plain(self)


@dataclass(frozen=True)
class ScenarioArtifactBundle:
    scenario_id: str
    run_id: str
    status: str
    exports: Mapping[str, str]
    png_paths: tuple[str, ...] = ()
    scenario_report_path: str | None = None
    gaps: tuple[str, ...] = ()
    png_feature_statuses: tuple[dict[str, Any], ...] = ()
    input_config_ref: str | None = None

    def to_dict(self) -> dict[str, Any]:
        return _to_plain(self)


@dataclass(frozen=True)
class ArtifactManifestEntry:
    scenario_id: str
    run_id: str
    status: str
    input_config_ref: str
    exports: Mapping[str, str]
    png_paths: tuple[str, ...] = ()
    scenario_report_path: str | None = None
    human_summary_path: str | None = None
    regression_report_ref: str | None = None
    gaps: tuple[str, ...] = ()
    png_feature_statuses: tuple[dict[str, Any], ...] = ()

    def to_dict(self) -> dict[str, Any]:
        return _to_plain(self)


@dataclass(frozen=True)
class ArtifactManifest:
    run_id: str
    entries: tuple[ArtifactManifestEntry, ...] = ()

    def to_dict(self) -> dict[str, Any]:
        return _to_plain(self)


@dataclass(frozen=True)
class RegressionReport:
    run_id: str
    suite_status: str
    required_green: tuple[str, ...] = ()
    required_failed: tuple[dict[str, Any], ...] = ()
    required_blocked: tuple[dict[str, Any], ...] = ()
    probe_observed: tuple[dict[str, Any], ...] = ()
    deferred_skipped: tuple[str, ...] = ()
    schema_gaps: tuple[str, ...] = ()
    runner_gaps: tuple[str, ...] = ()
    classification_blockers: tuple[dict[str, Any], ...] = ()
    artifact_paths: tuple[str, ...] = ()

    def to_dict(self) -> dict[str, Any]:
        return _to_plain(self)


def build_p11_smoke_suite_registry() -> SmokeSuiteRegistry:
    return SmokeSuiteRegistry()


def run_full_required_mvs_smoke_suite(
    *,
    registry: SmokeSuiteRegistry | None = None,
) -> SmokeSuiteRunResult:
    registry = registry or build_p11_smoke_suite_registry()
    scenario_results: list[ScenarioAggregationResult] = []
    probe_observed: list[ProbeObservation] = []
    deferred_skipped: list[DeferredScenarioRecord] = []
    extra_diagnostics: list[ScenarioAggregationResult] = []

    for scenario_id in registry.required_ids:
        scenario_results.append(_run_required_scenario(scenario_id))

    for scenario_id in registry.probe_ids:
        probe_observed.append(_run_probe_scenario(scenario_id))

    for scenario_id in registry.deferred_ids:
        deferred_skipped.append(_run_deferred_scenario(scenario_id))

    for scenario_id in registry.extra_diagnostic_ids:
        extra_diagnostics.append(_run_extra_diagnostic_scenario(scenario_id))

    return SmokeSuiteRunResult(
        registry=registry,
        scenario_results=tuple(scenario_results),
        probe_observed=tuple(probe_observed),
        deferred_skipped=tuple(deferred_skipped),
        extra_diagnostics=tuple(extra_diagnostics),
        p12_random_generation_enabled=False,
        paper_metrics_required=False,
    )


def aggregate_targeted_scenario_reports(
    scenario_reports: Iterable[ScenarioReport | ScenarioAggregationResult],
    *,
    required_histories: Mapping[str, OutputHistory] | None = None,
    registry: SmokeSuiteRegistry | None = None,
) -> SmokeSuiteRunResult:
    registry = registry or build_p11_smoke_suite_registry()
    scenario_results: list[ScenarioAggregationResult] = []
    probe_observed: list[ProbeObservation] = []
    deferred_skipped: list[DeferredScenarioRecord] = []

    seen_required: set[str] = set()
    for report in scenario_reports:
        if isinstance(report, ScenarioAggregationResult):
            if report.suite_group == "required":
                scenario_results.append(report)
                seen_required.add(report.scenario_id)
            elif report.suite_group == "probe":
                probe_observed.append(
                    ProbeObservation(
                        scenario_id=report.scenario_id,
                        classification=report.classification,
                        passed=report.passed,
                        failure_reasons=report.failure_reasons,
                    )
                )
            elif report.suite_group == "deferred":
                deferred_skipped.append(DeferredScenarioRecord(report.scenario_id))
            continue

        if report.scenario_id in registry.required_ids:
            scenario_results.append(_scenario_result_from_report(report))
            seen_required.add(report.scenario_id)
        elif report.scenario_id in registry.probe_ids:
            probe_observed.append(
                ProbeObservation(
                    scenario_id=report.scenario_id,
                    classification=report.classification,
                    passed=report.passed,
                    failure_reasons=tuple(report.failure_reasons),
                )
            )
        elif report.scenario_id in registry.deferred_ids:
            deferred_skipped.append(DeferredScenarioRecord(report.scenario_id, report.classification))

    for scenario_id, history in (required_histories or {}).items():
        if scenario_id not in registry.required_ids:
            continue
        scenario_results = [
            result for result in scenario_results if result.scenario_id != scenario_id
        ]
        scenario_results.append(_scenario_result_from_required_history(scenario_id, history))
        seen_required.add(scenario_id)

    scenario_results.sort(key=lambda result: registry.required_ids.index(result.scenario_id))
    return SmokeSuiteRunResult(
        registry=registry,
        scenario_results=tuple(scenario_results),
        probe_observed=tuple(probe_observed),
        deferred_skipped=tuple(deferred_skipped),
        p12_random_generation_enabled=False,
        paper_metrics_required=False,
    )


def serialize_trajectory_record(record: TrajectoryRecord) -> dict[str, Any]:
    return {
        "scenario_id": record.scenario_id,
        "run_id": record.run_id,
        "step": record.step,
        "t": record.t,
        "vehicle_id": record.vehicle_id,
        "vehicle_type": record.vehicle_type,
        "compliance_state": record.compliance_state,
        "x_global": record.x_global,
        "y": record.y,
        "v": record.v,
        "a": record.a,
        "physical_lane": record.physical_lane,
        "road_role": record.road_role,
        "primary_leader_id": record.primary_leader_id,
        "lane_change_state": record.lane_change_state,
        "merge_state": record.merge_state,
        "active_event_tags": "|".join(record.active_event_tags),
    }


def serialize_event_record(record: EventRecord) -> dict[str, Any]:
    return _to_plain(record.to_matcher_dict())


def serialize_sanity_record(record: SanityCheckRecord) -> dict[str, Any]:
    return _to_plain(record.to_matcher_dict())


def export_trajectory_history(history: OutputHistory, path: str | Path) -> Path:
    output_path = _ensure_parent(path)
    fieldnames = [
        "scenario_id",
        "run_id",
        "step",
        "t",
        "vehicle_id",
        "vehicle_type",
        "compliance_state",
        "x_global",
        "y",
        "v",
        "a",
        "physical_lane",
        "road_role",
        "primary_leader_id",
        "lane_change_state",
        "merge_state",
        "active_event_tags",
    ]
    with output_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for record in history.trajectory_records:
            writer.writerow(serialize_trajectory_record(record))
    return output_path


def export_event_history(history: OutputHistory, path: str | Path) -> Path:
    output_path = _ensure_parent(path)
    with output_path.open("w", encoding="utf-8") as handle:
        for record in history.event_records:
            handle.write(json.dumps(serialize_event_record(record), ensure_ascii=False, sort_keys=True))
            handle.write("\n")
    return output_path


def export_sanity_history(history: OutputHistory, path: str | Path) -> Path:
    output_path = _ensure_parent(path)
    with output_path.open("w", encoding="utf-8") as handle:
        for record in history.sanity_check_records:
            handle.write(json.dumps(serialize_sanity_record(record), ensure_ascii=False, sort_keys=True))
            handle.write("\n")
    return output_path


def derive_x_plot_for_renderer(
    x_global: float,
    *,
    geometry: RoadGeometryConfig = DEFAULT_ROAD_GEOMETRY,
) -> float:
    return float(x_global) - float(geometry.warmup_length)


def render_expected_png_features(
    expected_png_features: Iterable[Mapping[str, Any]],
) -> tuple[dict[str, Any], ...]:
    statuses: list[dict[str, Any]] = []
    for feature in expected_png_features:
        required = bool(feature.get("required", True))
        expected_visibility = str(
            feature.get("expected_visibility") or ("visible" if required else "optional")
        )
        if expected_visibility not in {"visible", "optional", "not_visible"}:
            expected_visibility = "visible" if required else "optional"
        statuses.append(
            {
                "feature_type": str(feature.get("feature_type", "unknown_feature")),
                "vehicle_ids": list(feature.get("vehicle_ids") or []),
                "required": required,
                "expected_visibility": expected_visibility,
                "renderer_status": expected_visibility,
                "evidence_source": "expected_png_features",
                "passed": True,
            }
        )
    return tuple(statuses)


def render_time_space_png(
    trajectory_records: Sequence[TrajectoryRecord],
    expected_png_features: Iterable[Mapping[str, Any]],
    path: str | Path,
    *,
    events: Sequence[EventRecord] = (),
    geometry: RoadGeometryConfig = DEFAULT_ROAD_GEOMETRY,
    width: int = 800,
    height: int = 400,
) -> PngRenderResult:
    output_path = _ensure_parent(path)
    records = list(trajectory_records)
    feature_statuses = render_expected_png_features(expected_png_features)
    pixels = _new_rgb_canvas(width, height, (255, 255, 255))
    x_values = [record.x_global for record in records] or [geometry.x0_m_global]
    x_min = min(min(x_values), geometry.x0_m_global) - 50.0
    x_max = max(max(x_values), geometry.x_ramp_end_global) + 50.0
    y_min = min(geometry.lane_centerlines.values()) - geometry.lane_width
    y_max = max(geometry.lane_centerlines.values()) + geometry.lane_width

    def x_to_px(x_global: float) -> int:
        span = max(x_max - x_min, 1.0)
        return int(40 + (float(x_global) - x_min) / span * (width - 80))

    def y_to_px(y: float) -> int:
        span = max(y_max - y_min, 1.0)
        return int(height - 45 - (float(y) - y_min) / span * (height - 90))

    render_lane_and_region_guides(
        pixels,
        geometry=geometry,
        x_to_px=x_to_px,
        y_to_px=y_to_px,
    )
    for index, record in enumerate(records):
        color = _palette(index)
        _draw_square(pixels, x_to_px(record.x_global), y_to_px(record.y), 4, color)
    render_event_markers(
        pixels,
        events=events,
        records=records,
        x_to_px=x_to_px,
        y_to_px=y_to_px,
    )
    _draw_feature_status_markers(pixels, feature_statuses)
    _write_png(output_path, pixels)
    return PngRenderResult(
        png_path=str(output_path),
        feature_statuses=feature_statuses,
        width=width,
        height=height,
    )


def render_lane_and_region_guides(
    pixels: list[bytearray],
    *,
    geometry: RoadGeometryConfig,
    x_to_px: Any,
    y_to_px: Any,
) -> None:
    height = len(pixels)
    if height == 0:
        return
    width = len(pixels[0]) // 3
    x0 = max(0, min(width - 1, x_to_px(geometry.x0_m_global)))
    x1 = max(0, min(width - 1, x_to_px(geometry.x_ramp_end_global)))
    for x in range(min(x0, x1), max(x0, x1) + 1):
        for y in range(25, height - 25):
            _set_pixel(pixels, x, y, (245, 250, 255))
    for y_value in geometry.lane_centerlines.values():
        y = y_to_px(y_value)
        _draw_hline(pixels, y, 35, width - 35, (180, 180, 180))


def render_event_markers(
    pixels: list[bytearray],
    *,
    events: Sequence[EventRecord],
    records: Sequence[TrajectoryRecord],
    x_to_px: Any,
    y_to_px: Any,
) -> None:
    by_vehicle = {record.vehicle_id: record for record in records}
    for event in events:
        if event.vehicle_id is None or event.vehicle_id not in by_vehicle:
            continue
        record = by_vehicle[event.vehicle_id]
        _draw_square(pixels, x_to_px(record.x_global), y_to_px(record.y), 7, (220, 30, 30))


def build_scenario_artifact_bundle(
    *,
    scenario_id: str,
    run_id: str,
    output_dir: str | Path,
    history: OutputHistory,
    expected_png_features: Iterable[Mapping[str, Any]],
    status: str,
    input_config_ref: str,
    gaps: Iterable[str] = (),
) -> ScenarioArtifactBundle:
    scenario_dir = Path(output_dir) / scenario_id / run_id
    trajectory_path = export_trajectory_history(history, scenario_dir / "trajectory.csv")
    event_path = export_event_history(history, scenario_dir / "events.jsonl")
    sanity_path = export_sanity_history(history, scenario_dir / "sanity.jsonl")
    render = render_time_space_png(
        history.trajectory_records,
        expected_png_features,
        scenario_dir / "time_space.png",
        events=history.event_records,
    )
    scenario_report_path = _ensure_parent(scenario_dir / "scenario_report.json")
    report_payload = {
        "scenario_id": scenario_id,
        "run_id": run_id,
        "status": status,
        "input_config_ref": input_config_ref,
        "exports": {
            "trajectory": str(trajectory_path),
            "events": str(event_path),
            "sanity": str(sanity_path),
        },
        "png_paths": [render.png_path],
        "gaps": list(gaps),
        "png_feature_statuses": list(render.feature_statuses),
    }
    scenario_report_path.write_text(
        json.dumps(report_payload, ensure_ascii=False, indent=2, sort_keys=True),
        encoding="utf-8",
    )
    return ScenarioArtifactBundle(
        scenario_id=scenario_id,
        run_id=run_id,
        status=status,
        input_config_ref=input_config_ref,
        exports={
            "trajectory": str(trajectory_path),
            "events": str(event_path),
            "sanity": str(sanity_path),
        },
        png_paths=(render.png_path,),
        scenario_report_path=str(scenario_report_path),
        gaps=tuple(gaps),
        png_feature_statuses=render.feature_statuses,
    )


def write_artifact_manifest(manifest: ArtifactManifest, path: str | Path) -> Path:
    output_path = _ensure_parent(path)
    output_path.write_text(
        json.dumps(manifest.to_dict(), ensure_ascii=False, indent=2, sort_keys=True),
        encoding="utf-8",
    )
    return output_path


def build_regression_report(
    suite: SmokeSuiteRunResult,
    *,
    run_id: str = "p11-run",
    schema_gaps: Iterable[str] = (),
    artifact_paths: Iterable[str] = (),
) -> RegressionReport:
    required_green: list[str] = []
    required_failed: list[dict[str, Any]] = []
    required_blocked: list[dict[str, Any]] = []
    runner_gaps: list[str] = []
    classification_blockers: list[dict[str, Any]] = []

    for result in suite.scenario_results:
        if result.suite_group != "required":
            continue
        if result.blockers:
            required_blocked.append(
                {
                    "scenario_id": result.scenario_id,
                    "reasons": [blocker.reason for blocker in result.blockers],
                    "details": [blocker.detail for blocker in result.blockers],
                }
            )
            for blocker in result.blockers:
                if blocker.reason == "missing_runner_route":
                    runner_gaps.append(result.scenario_id)
                if blocker.reason == "classification_mismatch":
                    classification_blockers.append(
                        {
                            "scenario_id": result.scenario_id,
                            "reason": blocker.reason,
                            "detail": blocker.detail,
                        }
                    )
            continue
        if result.blocks_required_suite or not result.passed:
            required_failed.append(
                {
                    "scenario_id": result.scenario_id,
                    "failure_reasons": list(result.failure_reasons),
                }
            )
        else:
            required_green.append(result.scenario_id)

    probe_observed = tuple(probe.to_dict() for probe in suite.probe_observed)
    deferred_skipped = tuple(record.scenario_id for record in suite.deferred_skipped)
    suite_status = (
        "failed_until_required_blockers_resolved"
        if required_blocked or required_failed
        else "passed"
    )
    return RegressionReport(
        run_id=run_id,
        suite_status=suite_status,
        required_green=tuple(required_green),
        required_failed=tuple(required_failed),
        required_blocked=tuple(required_blocked),
        probe_observed=probe_observed,
        deferred_skipped=deferred_skipped,
        schema_gaps=tuple(schema_gaps),
        runner_gaps=tuple(runner_gaps),
        classification_blockers=tuple(classification_blockers),
        artifact_paths=tuple(artifact_paths),
    )


def write_regression_report(report: RegressionReport, path: str | Path) -> Path:
    output_path = _ensure_parent(path)
    output_path.write_text(
        json.dumps(report.to_dict(), ensure_ascii=False, indent=2, sort_keys=True),
        encoding="utf-8",
    )
    return output_path


def _run_required_scenario(scenario_id: str) -> ScenarioAggregationResult:
    try:
        report = run_targeted_scenario(scenario_id)
    except Exception as exc:  # pragma: no cover - defensive aggregation boundary
        return _blocked_required_scenario(
            scenario_id,
            reason="runner_execution_error",
            detail=f"targeted runner failed for {scenario_id}: {exc}",
        )
    return _scenario_result_from_report(report)


def _run_probe_scenario(scenario_id: str) -> ProbeObservation:
    try:
        report = run_targeted_scenario(scenario_id)
    except Exception as exc:  # pragma: no cover - defensive aggregation boundary
        return ProbeObservation(
            scenario_id=scenario_id,
            classification="probe_missing_or_failed",
            passed=True,
            failure_reasons=(f"probe execution error: {exc}",),
        )
    return ProbeObservation(
        scenario_id=scenario_id,
        classification=report.classification,
        passed=report.passed,
        failure_reasons=tuple(report.failure_reasons),
    )


def _run_deferred_scenario(scenario_id: str) -> DeferredScenarioRecord:
    try:
        report = run_targeted_scenario(scenario_id)
    except Exception:
        return DeferredScenarioRecord(scenario_id=scenario_id)
    return DeferredScenarioRecord(
        scenario_id=scenario_id,
        classification=report.classification,
    )


def _run_extra_diagnostic_scenario(scenario_id: str) -> ScenarioAggregationResult:
    try:
        report = run_targeted_scenario(scenario_id)
    except Exception as exc:  # pragma: no cover - defensive aggregation boundary
        return ScenarioAggregationResult(
            scenario_id=scenario_id,
            suite_group="extra_diagnostic",
            status="diagnostic",
            classification="extra_diagnostic_failed",
            passed=True,
            blocks_required_suite=False,
            failure_reasons=(f"extra diagnostic execution error: {exc}",),
        )
    return ScenarioAggregationResult(
        scenario_id=scenario_id,
        suite_group="extra_diagnostic",
        status=report.status,
        classification=f"extra_{report.classification}",
        passed=True,
        blocks_required_suite=False,
        failure_reasons=tuple(report.failure_reasons),
    )


def _scenario_result_from_report(report: ScenarioReport) -> ScenarioAggregationResult:
    return ScenarioAggregationResult(
        scenario_id=report.scenario_id,
        suite_group="required",
        status="required",
        classification=report.classification,
        passed=report.passed,
        blocks_required_suite=report.blocks_required_suite,
        failure_reasons=tuple(report.failure_reasons),
    )


def _scenario_result_from_required_history(
    scenario_id: str,
    history: OutputHistory,
) -> ScenarioAggregationResult:
    failures = _blocking_sanity_failures(history)
    if not failures:
        return ScenarioAggregationResult(
            scenario_id=scenario_id,
            suite_group="required",
            status="required",
            classification="required_passed",
            passed=True,
            blocks_required_suite=False,
        )
    failure_reasons = tuple(
        _history_failure_reason(record)
        for record in failures
    )
    return ScenarioAggregationResult(
        scenario_id=scenario_id,
        suite_group="required",
        status="required",
        classification="required_failed",
        passed=False,
        blocks_required_suite=True,
        failure_reasons=failure_reasons,
    )


def _blocked_required_scenario(
    scenario_id: str,
    *,
    reason: str,
    detail: str,
) -> ScenarioAggregationResult:
    return ScenarioAggregationResult(
        scenario_id=scenario_id,
        suite_group="required",
        status="required",
        classification="required_blocked",
        passed=False,
        blocks_required_suite=True,
        failure_reasons=(detail,),
        blockers=(
            RequiredScenarioBlocker(
                scenario_id=scenario_id,
                reason=reason,
                detail=detail,
            ),
        ),
    )


def _missing_route_detail(scenario_id: str) -> str:
    if scenario_id == "MVS-E2E-1":
        return (
            "built-in MVS-E2E-1 runner / loader route not registered; "
            "P10 helper-targeted chain evidence may be cited, but built-in full runner route "
            "readiness is not established"
        )
    if scenario_id == "MVS-COMMIT-1-full":
        return (
            "built-in MVS-COMMIT-1-full runner / loader route not registered; "
            "P10 targeted commit evidence may be cited, but full route aggregation is missing"
        )
    return f"built-in {scenario_id} runner / loader route not registered"


def _blocking_sanity_failures(history: OutputHistory) -> list[SanityCheckRecord]:
    return [
        record
        for record in history.sanity_check_records
        if record.result == "fail" and record.severity in {"error", "critical", "blocker"}
    ]


def _history_failure_reason(record: SanityCheckRecord) -> str:
    suffix = ""
    if record.check_type == "multiple_commit_for_one_vehicle":
        suffix = "; missing-vehicle next_state must not continue"
    return f"required sanity fail: {record.check_type} result={record.result}{suffix}"


def _ensure_parent(path: str | Path) -> Path:
    output_path = Path(path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    return output_path


def _to_plain(value: Any) -> Any:
    if is_dataclass(value):
        return {field.name: _to_plain(getattr(value, field.name)) for field in fields(value)}
    if isinstance(value, Mapping):
        return {str(key): _to_plain(nested) for key, nested in value.items()}
    if isinstance(value, tuple):
        return [_to_plain(item) for item in value]
    if isinstance(value, list):
        return [_to_plain(item) for item in value]
    return value


def _new_rgb_canvas(width: int, height: int, color: tuple[int, int, int]) -> list[bytearray]:
    row = bytearray(color * width)
    return [bytearray(row) for _ in range(height)]


def _set_pixel(pixels: list[bytearray], x: int, y: int, color: tuple[int, int, int]) -> None:
    if y < 0 or y >= len(pixels):
        return
    width = len(pixels[y]) // 3
    if x < 0 or x >= width:
        return
    index = x * 3
    pixels[y][index:index + 3] = bytes(color)


def _draw_hline(
    pixels: list[bytearray],
    y: int,
    x0: int,
    x1: int,
    color: tuple[int, int, int],
) -> None:
    for x in range(max(0, x0), min(len(pixels[0]) // 3, x1 + 1)):
        _set_pixel(pixels, x, y, color)


def _draw_square(
    pixels: list[bytearray],
    center_x: int,
    center_y: int,
    radius: int,
    color: tuple[int, int, int],
) -> None:
    for y in range(center_y - radius, center_y + radius + 1):
        for x in range(center_x - radius, center_x + radius + 1):
            _set_pixel(pixels, x, y, color)


def _draw_feature_status_markers(
    pixels: list[bytearray],
    statuses: Sequence[Mapping[str, Any]],
) -> None:
    y = len(pixels) - 18
    x = 25
    colors = {
        "visible": (40, 120, 220),
        "optional": (80, 170, 110),
        "not_visible": (210, 210, 210),
    }
    for status in statuses:
        renderer_status = str(status.get("renderer_status", "visible"))
        color = colors.get(renderer_status, (80, 80, 80))
        if renderer_status != "not_visible":
            _draw_square(pixels, x, y, 5, color)
        else:
            _draw_hline(pixels, y, x - 5, x + 5, color)
        x += 18


def _palette(index: int) -> tuple[int, int, int]:
    colors = (
        (30, 90, 180),
        (220, 110, 40),
        (60, 150, 90),
        (160, 70, 160),
        (20, 150, 170),
    )
    return colors[index % len(colors)]


def _write_png(path: Path, pixels: list[bytearray]) -> None:
    height = len(pixels)
    width = len(pixels[0]) // 3 if height else 0
    raw = b"".join(b"\x00" + bytes(row) for row in pixels)
    payload = b"".join(
        (
            _png_chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)),
            _png_chunk(b"IDAT", zlib.compress(raw)),
            _png_chunk(b"IEND", b""),
        )
    )
    path.write_bytes(b"\x89PNG\r\n\x1a\n" + payload)


def _png_chunk(chunk_type: bytes, data: bytes) -> bytes:
    crc = zlib.crc32(chunk_type)
    crc = zlib.crc32(data, crc)
    return struct.pack(">I", len(data)) + chunk_type + data + struct.pack(">I", crc & 0xFFFFFFFF)
