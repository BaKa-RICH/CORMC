from __future__ import annotations

from dataclasses import replace
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from cormc.onestep.rolling import (
    BUNDLE_SHAPE_MV_FRONT,
    BUNDLE_SHAPE_MV_ONLY,
    BUNDLE_SHAPE_MV_REAR,
    build_motion_outputs,
    build_initial_onestep_stage2_state,
    decide_trigger_plan,
    identify_and_number_gaps,
    plan_stage2_for_trigger,
    run_onestep_stage2_history,
)
from cormc.scenes import (
    RM_ONESTEP_S05_MV_ID,
    RM_ONESTEP_S05_PLAN_STEP0_SCENARIO_ID,
    RM_ONESTEP_S05_ROLLING_ENTRY_SCENARIO_ID,
    RM_ONESTEP_S07_MV_ID,
    RM_ONESTEP_S07_PLAN_STEP0_SCENARIO_ID,
    RM_ONESTEP_S07_ROLLING_ENTRY_SCENARIO_ID,
)


def test_stage2_motion_completes_longitudinal_bundle_and_cleans_runtime_for_s07() -> None:
    result = run_onestep_stage2_history(
        RM_ONESTEP_S07_PLAN_STEP0_SCENARIO_ID,
        max_steps=420,
        run_id="stage2-motion-longrun-test",
    )
    summary = dict(result.summary)
    final_runtime = result.history

    lateral_start = _first_formal_payload(summary, "lateral_started")
    assert lateral_start is not None
    assert summary["scenario_summary"]["final_vehicle_states"][RM_ONESTEP_S07_MV_ID]["x_global"] > 7000.0
    assert final_runtime.trajectory_records
    assert any(
        event.get("event_type") == "ramp_merge_onestep_longitudinal_completion"
        for event in result.actual_events
    )
    released = _last_formal_payload(summary, "bundle_released")
    assert released is not None
    assert released["bundle_id"] == lateral_start["bundle_id"]
    assert released["bundle_action"] == "bundle_released"
    assert released["reason"] == "onestep_stage2_lateral_start_release"
    assert released["bundle_id"] not in summary["cross_mv_summary"]["final_runtime_leftovers"]["active_bundle_ids"]
    assert not any(
        controlled["bundle_id"] == released["bundle_id"]
        for controlled in summary["cross_mv_summary"]["final_runtime_leftovers"]["controlled_vehicle_states"].values()
    )
    lifecycle = summary["mv_summaries"][RM_ONESTEP_S07_MV_ID]["lifecycle"]
    assert lifecycle["lateral_start_step"] is not None
    assert lifecycle["lateral_completed_step"] is not None
    assert lifecycle["mainline_conversion_step"] is not None
    assert lifecycle["final_status"]["runtime_present"] is False
    assert lifecycle["final_status"]["physical_lane"] == "lane_2"
    assert lifecycle["final_status"]["road_role"] == "mainline"
    assert lifecycle["final_status"]["merge_state"] == "normal"


@pytest.mark.parametrize(
    (
        "hdv_vehicle_ids",
        "position_updates",
        "expected_shape",
        "default_boundary_vehicle_ids",
    ),
    [
        (
            ("S07_L2_04",),
            {},
            BUNDLE_SHAPE_MV_FRONT,
            ("S07_L2_04",),
        ),
        (
            ("S05_L2_02",),
            {},
            BUNDLE_SHAPE_MV_REAR,
            ("S05_L2_02",),
        ),
        (
            ("S07_L2_04", "S07_L2_05"),
            {"S07_L2_04": 6660.0, "S07_L2_05": 6790.0},
            BUNDLE_SHAPE_MV_ONLY,
            ("S07_L2_04", "S07_L2_05"),
        ),
    ],
)
def test_stage2_motion_leaves_non_controlled_boundaries_on_default_20mps(
    hdv_vehicle_ids: tuple[str, ...],
    position_updates: dict[str, float],
    expected_shape: str,
    default_boundary_vehicle_ids: tuple[str, ...],
) -> None:
    scenario_id = (
        RM_ONESTEP_S05_PLAN_STEP0_SCENARIO_ID
        if expected_shape == BUNDLE_SHAPE_MV_REAR
        else RM_ONESTEP_S07_PLAN_STEP0_SCENARIO_ID
    )
    state, _ = build_initial_onestep_stage2_state(scenario_id)
    state = _with_vehicle_x_updates(state, position_updates)
    state = _with_hdv_specs(state, hdv_vehicle_ids)
    planned = _plan_once(state)
    runtime = planned.runtime
    bundle = next(iter(runtime.onestep_plan_bundles.values()))
    assert bundle.bundle_shape == expected_shape

    outputs = build_motion_outputs(
        state,
        runtime,
        algorithm_variant="onestep_stage2",
    )

    for vehicle_id in default_boundary_vehicle_ids:
        candidate = outputs.candidate_kinematics[vehicle_id][0]
        current = state.vehicle_states[vehicle_id]
        assert vehicle_id not in bundle.controlled_vehicle_ids
        assert candidate.v == pytest.approx(20.0)
        assert candidate.a == pytest.approx(0.0)
        assert candidate.x_global == pytest.approx(current.x_global + 20.0 * state.dt)
    controlled_candidate = outputs.candidate_kinematics[bundle.mv_id][0]
    assert abs(controlled_candidate.v - 20.0) > 1e-3


@pytest.mark.parametrize(
    ("scenario_id", "mv_id"),
    [
        (RM_ONESTEP_S05_ROLLING_ENTRY_SCENARIO_ID, RM_ONESTEP_S05_MV_ID),
        (RM_ONESTEP_S07_ROLLING_ENTRY_SCENARIO_ID, RM_ONESTEP_S07_MV_ID),
    ],
)
def test_stage2_motion_keeps_velocity_continuity_across_periodic_replans(
    scenario_id: str,
    mv_id: str,
) -> None:
    result = run_onestep_stage2_history(
        scenario_id,
        max_steps=120,
        run_id="stage2-motion-velocity-continuity-test",
    )
    mv_records = [
        record
        for record in result.history.trajectory_records
        if record.vehicle_id == mv_id
    ]
    v_by_step = {int(record.step): float(record.v) for record in mv_records}
    first_trigger_step = int(result.summary["round_summaries"][0]["step"])
    periodic_steps = [
        int(event["step"])
        for event in result.summary["scenario_summary"]["formal_events"]
        if event["event_kind"] == "trigger_round"
        and event["payload"].get("trigger_reason") == "periodic"
        and int(event["step"]) > first_trigger_step
    ]
    assert periodic_steps
    for step in periodic_steps[:3]:
        assert step - 1 in v_by_step
        assert step in v_by_step
        assert abs(v_by_step[step] - v_by_step[step - 1]) < 0.5
        assert abs(v_by_step[step] - 20.0) > 0.5


def _plan_once(state):
    runtime = state.ramp_merge_runtime
    trigger = decide_trigger_plan(
        state,
        runtime.planner_state,
        entry_plan_trigger=False,
        safety_alert=False,
        entry_vehicle_ids=(),
    )
    snapshot = identify_and_number_gaps(state, ())
    return plan_stage2_for_trigger(state, runtime, snapshot, trigger)


def _with_hdv_specs(state, vehicle_ids: tuple[str, ...]):
    vehicle_specs = dict(state.vehicle_specs)
    for vehicle_id in vehicle_ids:
        vehicle_specs[vehicle_id] = replace(
            vehicle_specs[vehicle_id],
            vehicle_type="HDV",
            compliance_state="not_applicable",
        )
    return replace(state, vehicle_specs=vehicle_specs)


def _with_vehicle_x_updates(state, updates: dict[str, float]):
    if not updates:
        return state
    vehicle_states = dict(state.vehicle_states)
    for vehicle_id, x_global in updates.items():
        vehicle_states[vehicle_id] = replace(
            vehicle_states[vehicle_id],
            x_global=x_global,
        )
    return replace(state, vehicle_states=vehicle_states)


def _first_formal_payload(summary, kind):
    events = [
        event
        for event in summary["scenario_summary"]["formal_events"]
        if event["event_kind"] == kind
    ]
    return events[0]["payload"] if events else None


def _last_formal_payload(summary, kind):
    events = [
        event
        for event in summary["scenario_summary"]["formal_events"]
        if event["event_kind"] == kind
    ]
    return events[-1]["payload"] if events else None
