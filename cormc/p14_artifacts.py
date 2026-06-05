from __future__ import annotations

import json
import shutil
from dataclasses import dataclass, fields, is_dataclass
from datetime import UTC, datetime
from pathlib import Path
from typing import Any, Callable, Iterable, Mapping

from cormc.mvs.runner import DETERMINISTIC_SCENARIO_ROUTES
from cormc.p145_parameters import LOCKED_FORMULA_STATUS
from cormc.p11_output import (
    ArtifactManifest,
    ArtifactManifestEntry,
    RegressionReport,
    ScenarioArtifactBundle,
    build_regression_report,
    build_scenario_artifact_bundle,
    run_full_required_mvs_smoke_suite,
    write_artifact_manifest,
    write_regression_report,
)
from cormc.random_generation import (
    DEFAULT_P16_MAX_STEPS,
    DEFAULT_P16_SEED,
    P16_DEMO_SCENARIO_ID,
    default_p16_seeded_random_profile,
)
from cormc.simulation_loop import (
    SimulationLoopConfig,
    SimulationLoopResult,
    run_deterministic_simulation,
    run_seeded_random_simulation,
)
from cormc.step0_3 import ManeuverTrajectoryState, SimulationState, VehicleState


P14_DETERMINISTIC_BASELINE_SCENARIOS: tuple[str, ...] = (
    "MVS-E2E-1",
    "MVS-CUC-1A_override_choice1",
    "MVS-CUC-2",
    "MVS-CUC-3",
    "MVS-SAFE-1A_waiting_cap",
    "MVS-SAFE-1B_executing_cap_lateral_consumption",
    "MVS-SAFE-2",
    "MVS-COMMIT-1-full",
)

P14_REQUIRED_EVENT_TYPES: tuple[str, ...] = (
    "APS",
    "CMC",
    "CUC",
    "longitudinal_model",
    "lateral_trajectory",
    "commit",
    "time_advance",
)

P14_NUMERIC_TOLERANCE_ABS = 1e-6

P145_CORE_FORMULA_STATUS_KEYS: tuple[str, ...] = (
    "cuc_eq11_eq16",
    "cav_eq17_eq27",
    "chv_eq28_eq29",
    "front_collision_eq42_eq46",
)

P145_SUBFORMULA_STATUS_KEYS: tuple[str, ...] = (
    "cuc_eq14_eq15",
    "cav_cruising_eq20",
    "cav_cpid_eq21_eq27",
    "chv_idm_eq28_eq29",
    "front_collision_speed_constraint",
)

P145_FORMULA_STATUS_FIELDS: tuple[str, ...] = (
    "formula_status",
    "utility_formula_status",
    "longitudinal_formula_status",
    "idm_formula_status",
    "front_collision_formula_status",
)


@dataclass(frozen=True)
class P14ArtifactRunConfig:
    run_id: str = "pre_p15"
    output_root: str | Path = "artifacts/baseline"
    generated_label: str | None = None
    scenario_ids: tuple[str, ...] = P14_DETERMINISTIC_BASELINE_SCENARIOS


@dataclass(frozen=True)
class P14ScenarioArtifactResult:
    scenario_id: str
    run_id: str
    status: str
    scenario_dir: str
    bundle: ScenarioArtifactBundle
    simulation_result: SimulationLoopResult
    state_snapshot_path: str
    human_summary_path: str
    scenario_report_path: str
    sanity_summary: Mapping[str, Any]
    key_event_summary: Mapping[str, bool]
    formula_status_summary: Mapping[str, Any]

    def to_dict(self) -> dict[str, Any]:
        return _to_plain(self)


@dataclass(frozen=True)
class P14ArtifactRunResult:
    run_id: str
    output_dir: str
    scenario_results: tuple[P14ScenarioArtifactResult, ...]
    manifest_path: str
    regression_report_path: str
    run_report_path: str
    baseline_comparison_contract_path: str
    regression_report: RegressionReport

    def to_dict(self) -> dict[str, Any]:
        return _to_plain(self)


@dataclass(frozen=True)
class P16SeededRandomArtifactResult:
    run_id: str
    scenario_id: str
    status: str
    output_dir: str
    bundle: ScenarioArtifactBundle
    simulation_result: SimulationLoopResult
    artifact_manifest_path: str
    run_report_path: str
    seed: int
    profile_id: str
    generated_count: int
    blocked_spawn_count: int

    def to_dict(self) -> dict[str, Any]:
        return _to_plain(self)


def run_p14_scenario_artifact_bundle(
    scenario_id: str,
    *,
    run_id: str = "pre_p15",
    output_root: str | Path = "artifacts/baseline",
    _run_output_dir: str | Path | None = None,
) -> P14ScenarioArtifactResult:
    if scenario_id not in DETERMINISTIC_SCENARIO_ROUTES:
        raise ValueError(f"P14 requires deterministic scenario route: {scenario_id}")

    run_output_dir = Path(_run_output_dir) if _run_output_dir is not None else Path(output_root) / run_id
    scenario_dir = run_output_dir / "scenarios" / scenario_id
    max_steps = int(DETERMINISTIC_SCENARIO_ROUTES[scenario_id]["max_steps"])
    simulation = run_deterministic_simulation(
        SimulationLoopConfig(
            scenario_id=scenario_id,
            run_id=run_id,
            max_steps=max_steps,
            render_png=False,
        )
    )
    p11_bundle = build_scenario_artifact_bundle(
        scenario_id=scenario_id,
        run_id=run_id,
        output_dir=scenario_dir.parent,
        history=simulation.history,
        expected_png_features=simulation.expected_png_features,
        status=simulation.status,
        input_config_ref=f"builtin:{scenario_id}",
    )
    bundle = _normalize_p11_bundle_to_p14_layout(p11_bundle, scenario_dir)

    state_snapshot_path = _write_state_snapshot(
        simulation,
        scenario_dir / "state_snapshot.json",
    )
    sanity_summary = _summarize_sanity(simulation)
    key_event_summary = _summarize_key_events(simulation)
    formula_status_summary = _summarize_formula_status(simulation)
    scenario_report_path = _write_scenario_report(
        scenario_id=scenario_id,
        run_id=run_id,
        status=simulation.status,
        scenario_dir=scenario_dir,
        bundle=bundle,
        state_snapshot_path=state_snapshot_path,
        sanity_summary=sanity_summary,
        key_event_summary=key_event_summary,
        formula_status_summary=formula_status_summary,
    )
    human_summary_path = _write_scenario_summary(
        scenario_id=scenario_id,
        run_id=run_id,
        status=simulation.status,
        simulation=simulation,
        scenario_dir=scenario_dir,
        bundle=bundle,
        state_snapshot_path=state_snapshot_path,
        scenario_report_path=scenario_report_path,
        sanity_summary=sanity_summary,
        key_event_summary=key_event_summary,
        formula_status_summary=formula_status_summary,
    )
    _assert_scenario_bundle_complete(
        scenario_id=scenario_id,
        bundle=bundle,
        state_snapshot_path=state_snapshot_path,
        human_summary_path=human_summary_path,
        scenario_report_path=scenario_report_path,
    )
    return P14ScenarioArtifactResult(
        scenario_id=scenario_id,
        run_id=run_id,
        status=simulation.status,
        scenario_dir=str(scenario_dir),
        bundle=bundle,
        simulation_result=simulation,
        state_snapshot_path=str(state_snapshot_path),
        human_summary_path=str(human_summary_path),
        scenario_report_path=str(scenario_report_path),
        sanity_summary=sanity_summary,
        key_event_summary=key_event_summary,
        formula_status_summary=formula_status_summary,
    )


def run_p16_seeded_random_artifact_bundle(
    *,
    run_id: str = "p16_seeded_demo",
    output_root: str | Path = "artifacts/random/p16_seeded_demo",
    seed: int = DEFAULT_P16_SEED,
    max_steps: int = DEFAULT_P16_MAX_STEPS,
) -> P16SeededRandomArtifactResult:
    output_dir = Path(output_root) / run_id
    profile = default_p16_seeded_random_profile(seed=seed)
    simulation = run_seeded_random_simulation(
        SimulationLoopConfig(
            scenario_id=P16_DEMO_SCENARIO_ID,
            run_id=run_id,
            max_steps=max_steps,
            output_dir=output_dir,
            render_png=False,
            random_enabled=True,
            seed=seed,
            seeded_random_profile=profile,
        )
    )
    bundle = build_scenario_artifact_bundle(
        scenario_id=P16_DEMO_SCENARIO_ID,
        run_id=run_id,
        output_dir=output_dir.parent,
        history=simulation.history,
        expected_png_features=simulation.expected_png_features,
        status=simulation.status,
        input_config_ref=f"p16_seeded_random_profile:{profile.profile_id}",
    )
    scenario_dir = output_dir
    bundle = _normalize_p11_bundle_to_p16_layout(bundle, scenario_dir)
    generated_ids = _p16_generated_vehicle_ids(simulation)
    blocked_ids = _p16_blocked_vehicle_ids(simulation)
    manifest_path = _write_p16_artifact_manifest(
        scenario_dir / "artifact_manifest.json",
        run_id=run_id,
        scenario_id=P16_DEMO_SCENARIO_ID,
        bundle=bundle,
        seed=seed,
        profile_id=profile.profile_id,
        max_steps=max_steps,
        generated_count=len(generated_ids),
        blocked_spawn_count=len(blocked_ids),
    )
    scenario_report_path = _write_p16_scenario_report(
        Path(bundle.scenario_report_path or scenario_dir / "scenario_report.json"),
        bundle=bundle,
        simulation=simulation,
        seed=seed,
        profile_id=profile.profile_id,
        max_steps=max_steps,
        generated_count=len(generated_ids),
        blocked_spawn_count=len(blocked_ids),
    )
    run_report_path = _write_p16_run_report(
        scenario_dir / "run_report.md",
        run_id=run_id,
        scenario_id=P16_DEMO_SCENARIO_ID,
        bundle=bundle,
        simulation=simulation,
        manifest_path=manifest_path,
        scenario_report_path=scenario_report_path,
        seed=seed,
        profile_id=profile.profile_id,
        generated_count=len(generated_ids),
        blocked_spawn_count=len(blocked_ids),
    )
    _assert_p16_bundle_complete(
        bundle=bundle,
        manifest_path=manifest_path,
        run_report_path=run_report_path,
        baseline_root=Path("artifacts/baseline"),
    )
    return P16SeededRandomArtifactResult(
        run_id=run_id,
        scenario_id=P16_DEMO_SCENARIO_ID,
        status=simulation.status,
        output_dir=str(scenario_dir),
        bundle=bundle,
        simulation_result=simulation,
        artifact_manifest_path=str(manifest_path),
        run_report_path=str(run_report_path),
        seed=seed,
        profile_id=profile.profile_id,
        generated_count=len(generated_ids),
        blocked_spawn_count=len(blocked_ids),
    )


def run_p14_pre_p15_baseline(
    config: P14ArtifactRunConfig | None = None,
    *,
    scenario_runner: Callable[[str, str, Path], P14ScenarioArtifactResult] | None = None,
    suite_runner: Callable[[], Any] | None = None,
) -> P14ArtifactRunResult:
    config = config or P14ArtifactRunConfig()
    final_dir = Path(config.output_root) / config.run_id
    tmp_dir = final_dir.parent / f".{config.run_id}.tmp"
    if tmp_dir.exists():
        shutil.rmtree(tmp_dir)
        tmp_dir.mkdir(parents=True, exist_ok=True)

    try:
        if tuple(config.scenario_ids) != P14_DETERMINISTIC_BASELINE_SCENARIOS:
            raise RuntimeError("P14 baseline requires the exact 8 deterministic baseline scenarios")
        run_scenario = scenario_runner or _run_scenario_for_baseline
        scenario_results = tuple(
            run_scenario(scenario_id, config.run_id, tmp_dir)
            for scenario_id in config.scenario_ids
        )
        if len(scenario_results) != len(config.scenario_ids):
            raise RuntimeError("P14 baseline scenario result count mismatch")
        _assert_no_duplicate_scenarios(scenario_results)

        artifact_paths = _artifact_paths_for_regression(scenario_results)
        suite_or_report = (suite_runner or run_full_required_mvs_smoke_suite)()
        if isinstance(suite_or_report, RegressionReport):
            regression = suite_or_report
        else:
            regression = build_regression_report(
                suite_or_report,
                run_id=config.run_id,
                artifact_paths=artifact_paths,
            )
        _assert_regression_report_green(regression)
        regression_report_path = write_regression_report(
            regression,
            tmp_dir / "regression_report.json",
        )

        manifest = _build_manifest(
            run_id=config.run_id,
            scenario_results=scenario_results,
            regression_report_path=regression_report_path,
        )
        manifest_path = write_artifact_manifest(
            manifest,
            tmp_dir / "artifact_manifest.json",
        )
        comparison_contract_path = _write_baseline_comparison_contract(
            tmp_dir / "baseline_comparison_contract.json"
        )
        run_report_path = _write_run_report(
            config=config,
            output_dir=tmp_dir,
            scenario_results=scenario_results,
            regression=regression,
            manifest_path=manifest_path,
            regression_report_path=regression_report_path,
            comparison_contract_path=comparison_contract_path,
        )

        _assert_run_level_outputs_complete(
            scenario_results=scenario_results,
            manifest_path=manifest_path,
            regression_report_path=regression_report_path,
            run_report_path=run_report_path,
            comparison_contract_path=comparison_contract_path,
        )
        _rewrite_text_references(tmp_dir, final_dir)
        regression = _with_final_regression_paths(regression, tmp_dir, final_dir)
        if final_dir.exists():
            shutil.rmtree(final_dir)
        tmp_dir.replace(final_dir)
        return P14ArtifactRunResult(
            run_id=config.run_id,
            output_dir=str(final_dir),
            scenario_results=_with_final_paths(scenario_results, tmp_dir, final_dir),
            manifest_path=str(final_dir / "artifact_manifest.json"),
            regression_report_path=str(final_dir / "regression_report.json"),
            run_report_path=str(final_dir / "run_report.md"),
            baseline_comparison_contract_path=str(final_dir / "baseline_comparison_contract.json"),
            regression_report=regression,
        )
    except Exception:
        if tmp_dir.exists():
            shutil.rmtree(tmp_dir)
        raise


def _run_scenario_for_baseline(
    scenario_id: str,
    run_id: str,
    output_dir: Path,
) -> P14ScenarioArtifactResult:
    return run_p14_scenario_artifact_bundle(
        scenario_id,
        run_id=run_id,
        output_root=output_dir.parent,
        _run_output_dir=output_dir,
    )


def _normalize_p11_bundle_to_p14_layout(
    bundle: ScenarioArtifactBundle,
    scenario_dir: Path,
) -> ScenarioArtifactBundle:
    p11_dir = Path(bundle.exports["trajectory"]).parent
    if p11_dir != scenario_dir:
        scenario_dir.mkdir(parents=True, exist_ok=True)
        for path in p11_dir.iterdir():
            target = scenario_dir / path.name
            if target.exists():
                target.unlink()
            path.replace(target)
        _remove_empty_parents_until(p11_dir, scenario_dir.parent)
    exports = {
        "trajectory": str(scenario_dir / "trajectory.csv"),
        "events": str(scenario_dir / "events.jsonl"),
        "sanity": str(scenario_dir / "sanity.jsonl"),
    }
    return ScenarioArtifactBundle(
        scenario_id=bundle.scenario_id,
        run_id=bundle.run_id,
        status=bundle.status,
        exports=exports,
        png_paths=(str(scenario_dir / "time_space.png"),),
        scenario_report_path=str(scenario_dir / "scenario_report.json"),
        gaps=bundle.gaps,
        png_feature_statuses=bundle.png_feature_statuses,
        input_config_ref=bundle.input_config_ref,
    )


def _normalize_p11_bundle_to_p16_layout(
    bundle: ScenarioArtifactBundle,
    scenario_dir: Path,
) -> ScenarioArtifactBundle:
    source_dir = Path(bundle.exports["trajectory"]).parent
    scenario_dir.mkdir(parents=True, exist_ok=True)
    exports = {
        "trajectory": str(scenario_dir / "trajectory.csv"),
        "events": str(scenario_dir / "events.jsonl"),
        "sanity": str(scenario_dir / "sanity.jsonl"),
    }
    for source_name, target in (
        ("trajectory.csv", exports["trajectory"]),
        ("events.jsonl", exports["events"]),
        ("sanity.jsonl", exports["sanity"]),
        ("time_space.png", scenario_dir / "time_space.png"),
        ("scenario_report.json", scenario_dir / "scenario_report.json"),
    ):
        source = source_dir / source_name
        target_path = Path(target)
        if source.resolve() != target_path.resolve():
            shutil.copy2(source, target_path)
    return ScenarioArtifactBundle(
        scenario_id=bundle.scenario_id,
        run_id=bundle.run_id,
        status=bundle.status,
        input_config_ref=bundle.input_config_ref,
        exports=exports,
        png_paths=(str(scenario_dir / "time_space.png"),),
        scenario_report_path=str(scenario_dir / "scenario_report.json"),
        gaps=bundle.gaps,
        png_feature_statuses=bundle.png_feature_statuses,
    )


def _write_p16_artifact_manifest(
    path: Path,
    *,
    run_id: str,
    scenario_id: str,
    bundle: ScenarioArtifactBundle,
    seed: int,
    profile_id: str,
    max_steps: int,
    generated_count: int,
    blocked_spawn_count: int,
) -> Path:
    payload = {
        "run_id": run_id,
        "scenario_id": scenario_id,
        "random_enabled": True,
        "seed": seed,
        "profile_id": profile_id,
        "max_steps": max_steps,
        "generated_count": generated_count,
        "blocked_spawn_count": blocked_spawn_count,
        "output_paths": {
            **dict(bundle.exports),
            "time_space_png": bundle.png_paths[0],
            "scenario_report": bundle.scenario_report_path,
        },
    }
    return _write_json(payload, path)


def _write_p16_scenario_report(
    path: Path,
    *,
    bundle: ScenarioArtifactBundle,
    simulation: SimulationLoopResult,
    seed: int,
    profile_id: str,
    max_steps: int,
    generated_count: int,
    blocked_spawn_count: int,
) -> Path:
    payload = json.loads(path.read_text(encoding="utf-8")) if path.exists() else {}
    payload.update(
        {
            "scenario_id": simulation.scenario_id,
            "run_id": simulation.run_id,
            "status": simulation.status,
            "random_enabled": True,
            "seed": seed,
            "profile_id": profile_id,
            "max_steps": max_steps,
            "generated_count": generated_count,
            "blocked_spawn_count": blocked_spawn_count,
            "final_step": simulation.final_state.step,
            "final_t": simulation.final_state.t,
            "final_active_vehicle_count": len(simulation.final_state.active_vehicle_ids),
            "exports": dict(bundle.exports),
            "png_paths": list(bundle.png_paths),
            "png_feature_statuses": list(bundle.png_feature_statuses),
        }
    )
    return _write_json(payload, path)


def _write_p16_run_report(
    path: Path,
    *,
    run_id: str,
    scenario_id: str,
    bundle: ScenarioArtifactBundle,
    simulation: SimulationLoopResult,
    manifest_path: Path,
    scenario_report_path: Path,
    seed: int,
    profile_id: str,
    generated_count: int,
    blocked_spawn_count: int,
) -> Path:
    observed_event_types = sorted({record.event_type for record in simulation.history.event_records})
    lines = [
        "# P16 Seeded Random Internal Simulation",
        "",
        f"- run_id: `{run_id}`",
        f"- scenario_id: `{scenario_id}`",
        "- random_enabled: `true`",
        f"- seed: `{seed}`",
        f"- profile_id: `{profile_id}`",
        f"- status: `{simulation.status}`",
        f"- final step/t: `{simulation.final_state.step}` / `{_format_float(simulation.final_state.t)}`",
        f"- generated vehicle count: `{generated_count}`",
        f"- blocked spawn count: `{blocked_spawn_count}`",
        "",
        "## Observed Events",
        "",
        f"- event types: `{', '.join(observed_event_types)}`",
        "",
        "## Evidence Files",
        "",
        f"- trajectory.csv: `{_rel_to(path.parent, bundle.exports['trajectory'])}`",
        f"- events.jsonl: `{_rel_to(path.parent, bundle.exports['events'])}`",
        f"- sanity.jsonl: `{_rel_to(path.parent, bundle.exports['sanity'])}`",
        f"- time_space.png: `{_rel_to(path.parent, bundle.png_paths[0])}`",
        f"- artifact_manifest.json: `{_rel_to(path.parent, manifest_path)}`",
        f"- scenario_report.json: `{_rel_to(path.parent, scenario_report_path)}`",
        "",
        "## Boundary",
        "",
        "Random vehicle generation is applied only through Step 1 pre-freeze boundary decisions. The algorithm steps consume frozen `SimulationState` and ordinary `VehicleSpec` data.",
        "",
    ]
    path.write_text("\n".join(lines), encoding="utf-8")
    return path


def _p16_generated_vehicle_ids(simulation: SimulationLoopResult) -> tuple[str, ...]:
    generated: list[str] = []
    for event in simulation.history.event_dicts():
        if event.get("event_type") != "boundary_generation":
            continue
        for vehicle_id in event.get("payload", {}).get("generated_vehicle_ids") or ():
            _append_unique(generated, str(vehicle_id))
    return tuple(generated)


def _p16_blocked_vehicle_ids(simulation: SimulationLoopResult) -> tuple[str, ...]:
    blocked: list[str] = []
    for event in simulation.history.event_dicts():
        if event.get("event_type") != "boundary_generation":
            continue
        for vehicle_id in event.get("payload", {}).get("blocked_spawn_vehicle_ids") or ():
            _append_unique(blocked, str(vehicle_id))
    return tuple(blocked)


def _assert_p16_bundle_complete(
    *,
    bundle: ScenarioArtifactBundle,
    manifest_path: Path,
    run_report_path: Path,
    baseline_root: Path,
) -> None:
    required = [
        Path(bundle.exports["trajectory"]),
        Path(bundle.exports["events"]),
        Path(bundle.exports["sanity"]),
        Path(bundle.png_paths[0]),
        Path(bundle.scenario_report_path or ""),
        manifest_path,
        run_report_path,
    ]
    missing = [str(path) for path in required if not path.exists() or path.stat().st_size == 0]
    if missing:
        raise RuntimeError(f"P16 seeded artifact bundle missing or empty files: {missing}")
    if not Path(bundle.png_paths[0]).read_bytes().startswith(b"\x89PNG\r\n\x1a\n"):
        raise RuntimeError(f"P16 seeded artifact bundle invalid PNG: {bundle.png_paths[0]}")
    for output_path in required:
        try:
            output_path.relative_to(baseline_root)
        except ValueError:
            continue
        raise RuntimeError(f"P16 seeded artifact must not write baseline path: {output_path}")


def _remove_empty_parents_until(path: Path, stop: Path) -> None:
    current = path
    while current != stop and current.exists():
        try:
            current.rmdir()
        except OSError:
            return
        current = current.parent


def _write_state_snapshot(
    simulation: SimulationLoopResult,
    path: Path,
) -> Path:
    payload = {
        "scenario_id": simulation.scenario_id,
        "run_id": simulation.run_id,
        "initial_state": _serialize_simulation_state(simulation.initial_state),
        "final_state": _serialize_simulation_state(simulation.final_state),
    }
    return _write_json(payload, path)


def _serialize_simulation_state(state: SimulationState) -> dict[str, Any]:
    return {
        "step": state.step,
        "t": state.t,
        "dt": state.dt,
        "active_vehicle_ids": list(state.active_vehicle_ids),
        "vehicle_states": [
            _serialize_vehicle_state(state.vehicle_states[vehicle_id])
            for vehicle_id in sorted(state.vehicle_states)
        ],
        "assignment_records_by_mv": _to_plain(state.assignment_records_by_mv),
        "controller_memory_by_vehicle": _to_plain(state.controller_memory_by_vehicle),
        "active_maneuvers": [
            _serialize_active_maneuver(maneuver)
            for _, maneuver in sorted(state.active_maneuvers.items())
        ],
        "road_config_ref": state.road_config_ref,
        "parameter_config_ref": state.parameter_config_ref,
        "scenario_config_ref": state.scenario_config_ref,
        "output_config_ref": state.output_config_ref,
    }


def _serialize_vehicle_state(state: VehicleState) -> dict[str, Any]:
    return {
        "vehicle_id": state.vehicle_id,
        "x_global": state.x_global,
        "y": state.y,
        "v": state.v,
        "a": state.a,
        "physical_lane": state.physical_lane,
        "road_role": state.road_role,
        "lane_change_state": state.lane_change_state,
        "merge_state": state.merge_state,
        "is_active": state.is_active,
    }


def _serialize_active_maneuver(maneuver: ManeuverTrajectoryState) -> dict[str, Any]:
    return {
        "vehicle_id": maneuver.vehicle_id,
        "maneuver_type": maneuver.maneuver_type,
        "start_step": maneuver.start_step,
        "start_t": maneuver.start_t,
        "start_x_global": maneuver.start_x_global,
        "start_y": maneuver.start_y,
        "target_lane": maneuver.target_lane,
        "target_y": maneuver.target_y,
        "source_command_id": maneuver.source_command_id,
        "source_event_id": maneuver.source_event_id,
        "planned_length": maneuver.planned_length,
        "progress": maneuver.progress,
        "last_planning_speed": maneuver.last_planning_speed,
        "assigned_clv_id": maneuver.assigned_clv_id,
        "assigned_cfv_id": maneuver.assigned_cfv_id,
    }


def _write_scenario_report(
    *,
    scenario_id: str,
    run_id: str,
    status: str,
    scenario_dir: Path,
    bundle: ScenarioArtifactBundle,
    state_snapshot_path: Path,
    sanity_summary: Mapping[str, Any],
    key_event_summary: Mapping[str, bool],
    formula_status_summary: Mapping[str, Any],
) -> Path:
    path = scenario_dir / "scenario_report.json"
    payload = {
        "scenario_id": scenario_id,
        "run_id": run_id,
        "status": status,
        "input_config_ref": f"builtin:{scenario_id}",
        "exports": {
            **dict(bundle.exports),
            "state_snapshot": str(state_snapshot_path),
        },
        "png_paths": list(bundle.png_paths),
        "state_snapshot_path": str(state_snapshot_path),
        "sanity_summary": dict(sanity_summary),
        "key_event_summary": dict(key_event_summary),
        "formula_status_summary": _to_plain(formula_status_summary),
        "png_feature_statuses": list(bundle.png_feature_statuses),
        "gaps": list(bundle.gaps),
    }
    return _write_json(payload, path)


def _write_scenario_summary(
    *,
    scenario_id: str,
    run_id: str,
    status: str,
    simulation: SimulationLoopResult,
    scenario_dir: Path,
    bundle: ScenarioArtifactBundle,
    state_snapshot_path: Path,
    scenario_report_path: Path,
    sanity_summary: Mapping[str, Any],
    key_event_summary: Mapping[str, bool],
    formula_status_summary: Mapping[str, Any],
) -> Path:
    lines = [
        f"# Scenario Summary: {scenario_id}",
        "",
        f"- run_id: `{run_id}`",
        f"- status: `{status}`",
        f"- initial step/t: `{simulation.initial_state.step}` / `{simulation.initial_state.t}`",
        f"- final step/t: `{simulation.final_state.step}` / `{simulation.final_state.t}`",
        f"- final active vehicle count: `{len(simulation.final_state.active_vehicle_ids)}`",
        "",
        "## Final Vehicles",
        "",
        "| vehicle_id | x_global | y | v | physical_lane | road_role | lane_change_state | merge_state |",
        "|---|---:|---:|---:|---|---|---|---|",
    ]
    for vehicle_id in sorted(simulation.final_state.active_vehicle_ids):
        vehicle = simulation.final_state.vehicle_states[vehicle_id]
        lines.append(
            "| "
            + " | ".join(
                [
                    vehicle.vehicle_id,
                    _format_float(vehicle.x_global),
                    _format_float(vehicle.y),
                    _format_float(vehicle.v),
                    vehicle.physical_lane,
                    vehicle.road_role,
                    vehicle.lane_change_state,
                    vehicle.merge_state,
                ]
            )
            + " |"
        )

    lines.extend(
        [
            "",
            "## Active Maneuvers",
            "",
            "| vehicle_id | maneuver_type | progress | last_planning_speed |",
            "|---|---|---:|---:|",
        ]
    )
    if simulation.final_state.active_maneuvers:
        for vehicle_id, maneuver in sorted(simulation.final_state.active_maneuvers.items()):
            lines.append(
                "| "
                + " | ".join(
                    [
                        vehicle_id,
                        maneuver.maneuver_type,
                        _format_float(maneuver.progress),
                        _format_optional_float(maneuver.last_planning_speed),
                    ]
                )
                + " |"
            )
    else:
        lines.append("| none | none | 0 |  |")

    failed_ids = ", ".join(sanity_summary["failed_check_ids"]) or "none"
    lines.extend(
        [
            "",
            "## Sanity Summary",
            "",
            f"- pass count: `{sanity_summary['pass_count']}`",
            f"- fail count: `{sanity_summary['fail_count']}`",
            f"- failed check ids: `{failed_ids}`",
            "",
            "## Key Event Existence",
            "",
        ]
    )
    for key in P14_REQUIRED_EVENT_TYPES:
        lines.append(f"- {key}: `{key_event_summary[key]}`")

    lines.extend(
        [
            "",
            "## Formula Status",
            "",
        ]
    )
    for key, value in formula_status_summary["core_formula_status"].items():
        lines.append(f"- {key}: `{value}`")
    legacy_markers = ", ".join(formula_status_summary["legacy_proxy_markers_present"]) or "none"
    lines.append(f"- legacy proxy markers present: `{legacy_markers}`")

    png_features = sorted(
        {
            str(status_item.get("feature_type"))
            for status_item in bundle.png_feature_statuses
        }
    )
    lines.extend(
        [
            "",
            "## PNG",
            "",
            f"- time_space.png: `{_rel_to(scenario_dir, bundle.png_paths[0])}`",
            f"- registered PNG feature types: `{', '.join(png_features)}`",
            "",
            "## Evidence Files",
            "",
            f"- trajectory.csv: `{_rel_to(scenario_dir, bundle.exports['trajectory'])}`",
            f"- events.jsonl: `{_rel_to(scenario_dir, bundle.exports['events'])}`",
            f"- sanity.jsonl: `{_rel_to(scenario_dir, bundle.exports['sanity'])}`",
            f"- state_snapshot.json: `{_rel_to(scenario_dir, state_snapshot_path)}`",
            f"- scenario_report.json: `{_rel_to(scenario_dir, scenario_report_path)}`",
            "",
        ]
    )
    path = scenario_dir / "scenario_summary.md"
    path.write_text("\n".join(lines), encoding="utf-8")
    return path


def _write_run_report(
    *,
    config: P14ArtifactRunConfig,
    output_dir: Path,
    scenario_results: tuple[P14ScenarioArtifactResult, ...],
    regression: RegressionReport,
    manifest_path: Path,
    regression_report_path: Path,
    comparison_contract_path: Path,
) -> Path:
    generated_label = config.generated_label or datetime.now(UTC).replace(microsecond=0).isoformat()
    lines = [
        "# P14 pre-P15 Baseline Run Report",
        "",
        f"- run_id: `{config.run_id}`",
        f"- output root: `{output_dir}`",
        f"- generated: `{generated_label}`",
        "",
        "## Deterministic Baseline Scenarios",
        "",
    ]
    lines.extend(f"- `{scenario_id}`" for scenario_id in config.scenario_ids)
    lines.extend(
        [
            "",
            "## Suite Summary",
            "",
            f"- suite_status: `{regression.suite_status}`",
            f"- required_green count: `{len(regression.required_green)}`",
            f"- required_failed: `{len(regression.required_failed)}`",
            f"- required_blocked: `{len(regression.required_blocked)}`",
            f"- runner_gaps: `{len(regression.runner_gaps)}`",
            f"- probe list: `{', '.join(item['scenario_id'] for item in regression.probe_observed)}`",
            f"- deferred list: `{', '.join(regression.deferred_skipped)}`",
            "",
            "## Scenario Summaries",
            "",
            "| scenario_id | status | final step | final t | active vehicles | sanity pass/fail | PNG | summary |",
            "|---|---|---:|---:|---:|---|---|---|",
        ]
    )
    for result in scenario_results:
        final_state = result.simulation_result.final_state
        lines.append(
            "| "
            + " | ".join(
                [
                    result.scenario_id,
                    result.status,
                    str(final_state.step),
                    _format_float(final_state.t),
                    str(len(final_state.active_vehicle_ids)),
                    f"{result.sanity_summary['pass_count']}/{result.sanity_summary['fail_count']}",
                    _rel_to(output_dir, result.bundle.png_paths[0]),
                    _rel_to(output_dir, result.human_summary_path),
                ]
            )
            + " |"
        )

    formula_status = _aggregate_formula_status(scenario_results)
    legacy_markers = ", ".join(formula_status["legacy_proxy_markers_present"]) or "none"
    lines.extend(
        [
            "",
            "## Formula Status",
            "",
            f"- legacy proxy markers present: `{legacy_markers}`",
            "",
            "| formula area | aggregate status | scenario statuses |",
            "|---|---|---|",
        ]
    )
    for key in P145_CORE_FORMULA_STATUS_KEYS:
        scenario_statuses = ", ".join(
            f"{scenario_id}: {status}"
            for scenario_id, status in formula_status["scenario_core_status"][key].items()
        )
        lines.append(
            "| "
            + " | ".join(
                [
                    key,
                    formula_status["core_formula_status"][key],
                    scenario_statuses,
                ]
            )
            + " |"
        )
    lines.extend(
        [
            "",
            "| subformula area | aggregate status | evidence count |",
            "|---|---|---:|",
        ]
    )
    for key in P145_SUBFORMULA_STATUS_KEYS:
        lines.append(
            "| "
            + " | ".join(
                [
                    key,
                    formula_status["subformula_status"][key],
                    str(formula_status["evidence_event_count_by_key"][key]),
                ]
            )
            + " |"
        )

    lines.extend(
        [
            "",
            "## Run-Level Files",
            "",
            f"- artifact manifest: `{_rel_to(output_dir, manifest_path)}`",
            f"- regression report: `{_rel_to(output_dir, regression_report_path)}`",
            f"- baseline comparison contract: `{_rel_to(output_dir, comparison_contract_path)}`",
            "",
            "## P15 Baseline Note",
            "",
            "This directory is the pre-P15 baseline. P15 may read these JSON and CSV files for regression comparison.",
            "For human review, open this run_report.md first, then inspect each scenario_summary.md and time_space.png.",
            "The artifact outputs are evidence only: they do not feed back into vehicle motion, and x_plot remains renderer-derived.",
            "",
        ]
    )
    path = output_dir / "run_report.md"
    path.write_text("\n".join(lines), encoding="utf-8")
    return path


def _write_baseline_comparison_contract(path: Path) -> Path:
    payload = {
        "contract_id": "p14_pre_p15_baseline_comparison_contract",
        "numeric_tolerance_abs": P14_NUMERIC_TOLERANCE_ABS,
        "strong_compare_fields": [
            "final_state.step",
            "final_state.t",
            "vehicle.physical_lane",
            "vehicle.road_role",
            "vehicle.lane_change_state",
            "vehicle.merge_state",
            "active_vehicle_set",
            "active_maneuver_existence_cleanup",
            "key_event_existence",
            "sanity_status",
            "required_suite_classification",
        ],
        "numeric_tolerance_compare_fields": [
            "x_global",
            "y",
            "v",
            "active_maneuver.progress",
            "active_maneuver.last_planning_speed",
            "event.payload.planning_speed",
        ],
        "weak_compare_fields": [
            "png.file_exists",
            "png.file_nonempty",
            "png.header_valid",
            "png.feature_registered",
            "required_marker_status_exists",
        ],
        "non_goals": [
            "No P15 comparator implementation",
            "No P16 random generation",
            "No P17 paper experiment grid",
        ],
    }
    return _write_json(payload, path)


def _build_manifest(
    *,
    run_id: str,
    scenario_results: tuple[P14ScenarioArtifactResult, ...],
    regression_report_path: Path,
) -> ArtifactManifest:
    entries = []
    for result in scenario_results:
        exports = {
            **dict(result.bundle.exports),
            "state_snapshot": result.state_snapshot_path,
        }
        entries.append(
            ArtifactManifestEntry(
                scenario_id=result.scenario_id,
                run_id=run_id,
                status=result.status,
                input_config_ref=f"builtin:{result.scenario_id}",
                exports=exports,
                png_paths=result.bundle.png_paths,
                scenario_report_path=result.scenario_report_path,
                human_summary_path=result.human_summary_path,
                regression_report_ref=str(regression_report_path),
                gaps=result.bundle.gaps,
                png_feature_statuses=result.bundle.png_feature_statuses,
            )
        )
    return ArtifactManifest(run_id=run_id, entries=tuple(entries))


def _summarize_sanity(simulation: SimulationLoopResult) -> dict[str, Any]:
    failed = [
        record.check_id
        for record in simulation.history.sanity_check_records
        if record.result == "fail"
    ]
    pass_count = sum(
        1
        for record in simulation.history.sanity_check_records
        if record.result == "pass"
    )
    return {
        "pass_count": pass_count,
        "fail_count": len(failed),
        "failed_check_ids": failed,
        "total_count": len(simulation.history.sanity_check_records),
    }


def _summarize_key_events(simulation: SimulationLoopResult) -> dict[str, bool]:
    events = simulation.history.event_dicts()
    return {
        event_type: any(
            event.get("event_type") == event_type or event.get("module") == event_type
            for event in events
        )
        for event_type in P14_REQUIRED_EVENT_TYPES
    }


def _summarize_formula_status(simulation: SimulationLoopResult) -> dict[str, Any]:
    events = simulation.history.event_dicts()
    core_status = {key: "not_observed" for key in P145_CORE_FORMULA_STATUS_KEYS}
    subformula_status = {key: "not_observed" for key in P145_SUBFORMULA_STATUS_KEYS}
    evidence_events = {
        key: []
        for key in (*P145_CORE_FORMULA_STATUS_KEYS, *P145_SUBFORMULA_STATUS_KEYS)
    }
    status_values_seen_by_field: dict[str, list[str]] = {}
    legacy_proxy_markers: list[str] = []

    for event in events:
        payload = event.get("payload") or {}
        if not isinstance(payload, Mapping):
            continue
        _collect_formula_status_values(payload, status_values_seen_by_field)
        _collect_legacy_proxy_markers(payload, legacy_proxy_markers)

        if _is_locked_cuc_eq11_eq16(event, payload):
            _mark_locked(core_status, evidence_events, "cuc_eq11_eq16", event)
        if _is_locked_cuc_eq14_eq15(event, payload):
            _mark_locked(subformula_status, evidence_events, "cuc_eq14_eq15", event)
        if _is_locked_cav_longitudinal(event, payload):
            _mark_locked(core_status, evidence_events, "cav_eq17_eq27", event)
            if payload.get("eq20_locked") is True:
                _mark_locked(subformula_status, evidence_events, "cav_cruising_eq20", event)
            if (
                payload.get("eq21_eq27_locked") is True
                and payload.get("cpid_mode") == "minimal_formula_mode"
            ):
                _mark_locked(subformula_status, evidence_events, "cav_cpid_eq21_eq27", event)
        if _is_locked_chv_idm(event, payload):
            _mark_locked(core_status, evidence_events, "chv_eq28_eq29", event)
            _mark_locked(subformula_status, evidence_events, "chv_idm_eq28_eq29", event)
        if _is_locked_front_collision(event, payload):
            _mark_locked(core_status, evidence_events, "front_collision_eq42_eq46", event)
            _mark_locked(
                subformula_status,
                evidence_events,
                "front_collision_speed_constraint",
                event,
            )

    return {
        "core_formula_status": core_status,
        "subformula_status": subformula_status,
        "legacy_proxy_markers_present": sorted(legacy_proxy_markers),
        "legacy_proxy_marker_count": len(legacy_proxy_markers),
        "status_values_seen_by_field": {
            key: sorted(values)
            for key, values in sorted(status_values_seen_by_field.items())
        },
        "evidence_events_by_key": evidence_events,
        "evidence_event_count_by_key": {
            key: len(value) for key, value in evidence_events.items()
        },
    }


def _aggregate_formula_status(
    scenario_results: tuple[P14ScenarioArtifactResult, ...],
) -> dict[str, Any]:
    scenario_core_status = {
        key: {
            result.scenario_id: str(result.formula_status_summary["core_formula_status"][key])
            for result in scenario_results
        }
        for key in P145_CORE_FORMULA_STATUS_KEYS
    }
    scenario_subformula_status = {
        key: {
            result.scenario_id: str(result.formula_status_summary["subformula_status"][key])
            for result in scenario_results
        }
        for key in P145_SUBFORMULA_STATUS_KEYS
    }
    legacy_proxy_markers = sorted(
        {
            str(marker)
            for result in scenario_results
            for marker in result.formula_status_summary["legacy_proxy_markers_present"]
        }
    )
    status_values_seen_by_field: dict[str, list[str]] = {}
    for result in scenario_results:
        values_by_field = result.formula_status_summary["status_values_seen_by_field"]
        for field_name, values in values_by_field.items():
            target = status_values_seen_by_field.setdefault(str(field_name), [])
            for value in values:
                _append_unique(target, str(value))

    evidence_counts: dict[str, int] = {}
    for key in (*P145_CORE_FORMULA_STATUS_KEYS, *P145_SUBFORMULA_STATUS_KEYS):
        evidence_counts[key] = sum(
            int(result.formula_status_summary["evidence_event_count_by_key"][key])
            for result in scenario_results
        )

    return {
        "core_formula_status": {
            key: _aggregate_status(values.values())
            for key, values in scenario_core_status.items()
        },
        "subformula_status": {
            key: _aggregate_status(values.values())
            for key, values in scenario_subformula_status.items()
        },
        "scenario_core_status": scenario_core_status,
        "scenario_subformula_status": scenario_subformula_status,
        "legacy_proxy_markers_present": legacy_proxy_markers,
        "legacy_proxy_marker_count": len(legacy_proxy_markers),
        "status_values_seen_by_field": {
            key: sorted(values) for key, values in sorted(status_values_seen_by_field.items())
        },
        "evidence_event_count_by_key": evidence_counts,
    }


def _is_locked_cuc_eq11_eq16(event: Mapping[str, Any], payload: Mapping[str, Any]) -> bool:
    return (
        event.get("event_type") == "CUC"
        and payload.get("formula_status") == LOCKED_FORMULA_STATUS
        and payload.get("utility_formula_status") == LOCKED_FORMULA_STATUS
        and payload.get("eq11_eq12_locked") is True
        and payload.get("eq13_locked") is True
        and payload.get("eq14_eq15_locked") is True
        and payload.get("eq16_locked") is True
    )


def _is_locked_cuc_eq14_eq15(event: Mapping[str, Any], payload: Mapping[str, Any]) -> bool:
    return (
        event.get("event_type") == "CUC"
        and payload.get("formula_status") == LOCKED_FORMULA_STATUS
        and payload.get("eq14_eq15_locked") is True
    )


def _is_locked_cav_longitudinal(event: Mapping[str, Any], payload: Mapping[str, Any]) -> bool:
    return (
        event.get("event_type") == "longitudinal_model"
        and str(payload.get("vehicle_type") or "").lower() == "cav"
        and payload.get("longitudinal_formula_status") == LOCKED_FORMULA_STATUS
        and payload.get("eq17_eq19_locked") is True
        and (payload.get("eq20_locked") is True or payload.get("eq21_eq27_locked") is True)
    )


def _is_locked_chv_idm(event: Mapping[str, Any], payload: Mapping[str, Any]) -> bool:
    return (
        event.get("event_type") == "longitudinal_model"
        and str(payload.get("vehicle_type") or "").lower() == "chv"
        and payload.get("idm_formula_status") == LOCKED_FORMULA_STATUS
        and payload.get("eq28_eq29_locked") is True
    )


def _is_locked_front_collision(event: Mapping[str, Any], payload: Mapping[str, Any]) -> bool:
    return (
        event.get("event_type") in {"longitudinal_model", "speed_cap"}
        and payload.get("front_collision_formula_status") == LOCKED_FORMULA_STATUS
        and payload.get("eq42_eq46_locked") is True
        and payload.get("front_collision_status") != "not_applicable"
    )


def _collect_formula_status_values(
    payload: Mapping[str, Any],
    status_values_seen_by_field: dict[str, list[str]],
) -> None:
    for field_name in P145_FORMULA_STATUS_FIELDS:
        if field_name not in payload:
            continue
        _append_unique(
            status_values_seen_by_field.setdefault(field_name, []),
            str(payload[field_name]),
        )


def _collect_legacy_proxy_markers(
    payload: Mapping[str, Any],
    legacy_proxy_markers: list[str],
) -> None:
    if payload.get("eq14_eq15_locked") is False:
        _append_unique(legacy_proxy_markers, "eq14_eq15_locked=False")
    if payload.get("cpid_memory_status") == "probe_schema_gap":
        _append_unique(legacy_proxy_markers, "cpid_memory_status=probe_schema_gap")
    for value in _flatten_values(payload):
        if value == "first_version_probe_not_eq11_eq12_locked":
            _append_unique(legacy_proxy_markers, "first_version_probe_not_eq11_eq12_locked")


def _mark_locked(
    status_by_key: dict[str, str],
    evidence_events: dict[str, list[dict[str, Any]]],
    key: str,
    event: Mapping[str, Any],
) -> None:
    status_by_key[key] = LOCKED_FORMULA_STATUS
    evidence_events[key].append(_formula_event_ref(event))


def _aggregate_status(statuses: Iterable[str]) -> str:
    observed = tuple(statuses)
    if LOCKED_FORMULA_STATUS in observed:
        return LOCKED_FORMULA_STATUS
    if "not_applicable" in observed:
        return "not_applicable"
    return "not_observed"


def _formula_event_ref(event: Mapping[str, Any]) -> dict[str, Any]:
    return {
        "event_id": event.get("event_id"),
        "step": event.get("step"),
        "module": event.get("module"),
        "event_type": event.get("event_type"),
        "vehicle_id": event.get("vehicle_id"),
        "reason": event.get("reason"),
    }


def _flatten_values(value: Any) -> Iterable[Any]:
    if isinstance(value, Mapping):
        for nested in value.values():
            yield from _flatten_values(nested)
    elif isinstance(value, (list, tuple)):
        for nested in value:
            yield from _flatten_values(nested)
    else:
        yield value


def _append_unique(values: list[str], value: str) -> None:
    if value not in values:
        values.append(value)


def _artifact_paths_for_regression(
    scenario_results: Iterable[P14ScenarioArtifactResult],
) -> tuple[str, ...]:
    paths: list[str] = []
    for result in scenario_results:
        paths.extend(result.bundle.exports.values())
        paths.extend(result.bundle.png_paths)
        paths.extend(
            [
                result.state_snapshot_path,
                result.scenario_report_path,
                result.human_summary_path,
            ]
        )
    return tuple(paths)


def _assert_scenario_bundle_complete(
    *,
    scenario_id: str,
    bundle: ScenarioArtifactBundle,
    state_snapshot_path: Path,
    human_summary_path: Path,
    scenario_report_path: Path,
) -> None:
    required = [
        Path(bundle.exports["trajectory"]),
        Path(bundle.exports["events"]),
        Path(bundle.exports["sanity"]),
        Path(bundle.png_paths[0]),
        state_snapshot_path,
        human_summary_path,
        scenario_report_path,
    ]
    missing = [str(path) for path in required if not path.exists()]
    if missing:
        raise RuntimeError(f"P14 scenario bundle missing files for {scenario_id}: {missing}")
    png = Path(bundle.png_paths[0])
    data = png.read_bytes()
    if not data or not data.startswith(b"\x89PNG\r\n\x1a\n"):
        raise RuntimeError(f"P14 scenario bundle invalid PNG for {scenario_id}: {png}")
    if not bundle.png_feature_statuses:
        raise RuntimeError(f"P14 scenario bundle has no registered PNG features for {scenario_id}")


def _assert_regression_report_green(report: RegressionReport) -> None:
    if report.suite_status != "passed":
        raise RuntimeError(f"P14 suite failed: {report.suite_status}")
    if len(report.required_green) != 20:
        raise RuntimeError(f"P14 requires 20 required green scenarios, got {len(report.required_green)}")
    if report.required_failed or report.required_blocked or report.runner_gaps:
        raise RuntimeError("P14 regression report has required failures, blockers, or runner gaps")
    if report.classification_blockers:
        raise RuntimeError("P14 regression report has classification blockers")


def _assert_run_level_outputs_complete(
    *,
    scenario_results: tuple[P14ScenarioArtifactResult, ...],
    manifest_path: Path,
    regression_report_path: Path,
    run_report_path: Path,
    comparison_contract_path: Path,
) -> None:
    required = [
        manifest_path,
        regression_report_path,
        run_report_path,
        comparison_contract_path,
    ]
    for result in scenario_results:
        required.extend(
            [
                Path(result.human_summary_path),
                Path(result.scenario_report_path),
                Path(result.state_snapshot_path),
                Path(result.bundle.png_paths[0]),
            ]
        )
    missing = [str(path) for path in required if not path.exists()]
    if missing:
        raise RuntimeError(f"P14 baseline incomplete: {missing}")


def _assert_no_duplicate_scenarios(
    scenario_results: tuple[P14ScenarioArtifactResult, ...],
) -> None:
    scenario_ids = [result.scenario_id for result in scenario_results]
    if len(scenario_ids) != len(set(scenario_ids)):
        raise RuntimeError("P14 baseline scenario ids must be unique")


def _with_final_paths(
    scenario_results: tuple[P14ScenarioArtifactResult, ...],
    tmp_dir: Path,
    final_dir: Path,
) -> tuple[P14ScenarioArtifactResult, ...]:
    updated = []
    for result in scenario_results:
        mapping = result.to_dict()
        mapping["scenario_dir"] = _replace_prefix(mapping["scenario_dir"], tmp_dir, final_dir)
        mapping["state_snapshot_path"] = _replace_prefix(mapping["state_snapshot_path"], tmp_dir, final_dir)
        mapping["human_summary_path"] = _replace_prefix(mapping["human_summary_path"], tmp_dir, final_dir)
        mapping["scenario_report_path"] = _replace_prefix(mapping["scenario_report_path"], tmp_dir, final_dir)
        updated_bundle = ScenarioArtifactBundle(
            scenario_id=result.bundle.scenario_id,
            run_id=result.bundle.run_id,
            status=result.bundle.status,
            input_config_ref=result.bundle.input_config_ref,
            exports={
                key: _replace_prefix(path, tmp_dir, final_dir)
                for key, path in result.bundle.exports.items()
            },
            png_paths=tuple(_replace_prefix(path, tmp_dir, final_dir) for path in result.bundle.png_paths),
            scenario_report_path=_replace_prefix(result.scenario_report_path, tmp_dir, final_dir),
            gaps=result.bundle.gaps,
            png_feature_statuses=result.bundle.png_feature_statuses,
        )
        updated.append(
            P14ScenarioArtifactResult(
                scenario_id=result.scenario_id,
                run_id=result.run_id,
                status=result.status,
                scenario_dir=mapping["scenario_dir"],
                bundle=updated_bundle,
                simulation_result=result.simulation_result,
                state_snapshot_path=mapping["state_snapshot_path"],
                human_summary_path=mapping["human_summary_path"],
                scenario_report_path=mapping["scenario_report_path"],
                sanity_summary=result.sanity_summary,
                key_event_summary=result.key_event_summary,
                formula_status_summary=result.formula_status_summary,
            )
        )
    return tuple(updated)


def _with_final_regression_paths(
    regression: RegressionReport,
    tmp_dir: Path,
    final_dir: Path,
) -> RegressionReport:
    return RegressionReport(
        run_id=regression.run_id,
        suite_status=regression.suite_status,
        required_green=regression.required_green,
        required_failed=regression.required_failed,
        required_blocked=regression.required_blocked,
        probe_observed=regression.probe_observed,
        deferred_skipped=regression.deferred_skipped,
        schema_gaps=regression.schema_gaps,
        runner_gaps=regression.runner_gaps,
        classification_blockers=regression.classification_blockers,
        artifact_paths=tuple(
            _replace_prefix(path, tmp_dir, final_dir) or ""
            for path in regression.artifact_paths
        ),
    )


def _rewrite_text_references(tmp_dir: Path, final_dir: Path) -> None:
    raw_replacements = (
        (str(tmp_dir), str(final_dir)),
        (tmp_dir.as_posix(), final_dir.as_posix()),
        (str(tmp_dir.resolve()), str(final_dir.resolve())),
        (tmp_dir.resolve().as_posix(), final_dir.resolve().as_posix()),
    )
    replacements = list(raw_replacements)
    replacements.extend(
        (json.dumps(old)[1:-1], json.dumps(new)[1:-1])
        for old, new in raw_replacements
    )
    for path in tmp_dir.rglob("*"):
        if path.suffix.lower() not in {".json", ".md"}:
            continue
        text = path.read_text(encoding="utf-8")
        updated = text
        for old, new in replacements:
            updated = updated.replace(old, new)
        if updated != text:
            path.write_text(updated, encoding="utf-8")


def _replace_prefix(path: str | Path | None, old: Path, new: Path) -> str | None:
    if path is None:
        return None
    path_obj = Path(path)
    try:
        rel = path_obj.relative_to(old)
    except ValueError:
        return str(path_obj)
    return str(new / rel)


def _write_json(payload: Mapping[str, Any], path: Path) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(_to_plain(payload), ensure_ascii=False, indent=2, sort_keys=True),
        encoding="utf-8",
    )
    return path


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


def _format_float(value: float) -> str:
    return f"{float(value):.6g}"


def _format_optional_float(value: float | None) -> str:
    if value is None:
        return ""
    return _format_float(value)


def _rel_to(base: Path, target: str | Path) -> str:
    try:
        return str(Path(target).relative_to(base))
    except ValueError:
        return str(target)
