from copy import deepcopy

from cormc.onestep.rolling import (
    build_onestep_stage2_acceptance_report,
    build_onestep_stage2_random_acceptance_report,
    run_onestep_stage2_history,
)
from cormc.onestep.rolling.stage2_random_runner import (
    run_onestep_stage2_random_history,
)
from cormc.scenes import (
    FlowValidationSpec,
    RM_ONESTEP_S07_2MV_ROLLING_ENTRY_SCENARIO_ID,
    build_onestep_random_lane2_ramp_scene,
)


def test_random_stage2_acceptance_allows_open_horizon_mvs() -> None:
    spec = build_onestep_random_lane2_ramp_scene(
        density="high",
        horizon_s=30.0,
        max_steps=120,
        safe_spawn_gap_m=5.0,
    )
    spec = replace_validation(spec, min_completed_mv_count=0)
    run = run_onestep_stage2_random_history(
        spec,
        max_steps=120,
        run_id="random-validation-open",
    )

    report = build_onestep_stage2_random_acceptance_report(run.summary)

    assert report.passed
    assert report.integrity["generated_vehicle_count"] > 0
    assert report.integrity["gap_row_count"] > 0
    assert report.integrity["multi_mv_gap_conflict"] is False


def test_random_stage2_acceptance_fails_completed_mv_out_of_order() -> None:
    run = run_onestep_stage2_random_history(
        build_onestep_random_lane2_ramp_scene(
            density="high",
            horizon_s=30.0,
            max_steps=200,
            safe_spawn_gap_m=5.0,
        ),
        max_steps=200,
        run_id="random-validation-bad-order",
    )
    broken = deepcopy(run.summary)
    completed_id = broken["scenario_summary"]["completed_mv_ids"][0]
    lifecycle = broken["mv_summaries"][completed_id]["lifecycle"]
    lifecycle["locked_gap_step"] = 10
    lifecycle["lateral_start_step"] = 9

    report = build_onestep_stage2_random_acceptance_report(broken)

    assert not report.passed
    assert any(issue.check_id == "S2R-LIFE-001" for issue in report.issues)


def test_random_stage2_acceptance_fails_open_mv_without_final_status() -> None:
    spec = build_onestep_random_lane2_ramp_scene(
        density="high",
        horizon_s=30.0,
        max_steps=120,
        safe_spawn_gap_m=5.0,
    )
    spec = replace_validation(spec, min_completed_mv_count=0)
    run = run_onestep_stage2_random_history(
        spec,
        max_steps=120,
        run_id="random-validation-open-status",
    )
    broken = deepcopy(run.summary)
    open_ids = broken["scenario_summary"]["open_mv_ids_at_horizon"]
    assert open_ids
    broken["mv_summaries"][open_ids[0]]["lifecycle"]["final_status"] = {}

    report = build_onestep_stage2_random_acceptance_report(broken)

    assert not report.passed
    assert any(issue.check_id == "S2R-LIFE-005" for issue in report.issues)


def test_static_stage2_acceptance_still_requires_closed_lifecycle() -> None:
    summary = run_onestep_stage2_history(
        RM_ONESTEP_S07_2MV_ROLLING_ENTRY_SCENARIO_ID,
        max_steps=700,
        run_id="static-validation-regression",
    ).summary

    report = build_onestep_stage2_acceptance_report(summary)

    assert report.passed


def replace_validation(spec, *, min_completed_mv_count: int):
    return type(spec)(
        scenario_id=spec.scenario_id,
        scenario_name=spec.scenario_name,
        purpose=spec.purpose,
        initial_vehicles=spec.initial_vehicles,
        boundary_flow_source=spec.boundary_flow_source,
        safe_spawn_gap_m=spec.safe_spawn_gap_m,
        stop_condition=spec.stop_condition,
        validation=FlowValidationSpec(
            min_generated_lane2_count=spec.validation.min_generated_lane2_count,
            min_generated_on_ramp_mv_count=spec.validation.min_generated_on_ramp_mv_count,
            min_completed_mv_count=min_completed_mv_count,
            allow_open_mvs_at_horizon=spec.validation.allow_open_mvs_at_horizon,
        ),
        module_overrides=spec.module_overrides,
        notes=spec.notes,
        test_level=spec.test_level,
        status=spec.status,
        derivation_ref=spec.derivation_ref,
        initial_time=spec.initial_time,
    )
