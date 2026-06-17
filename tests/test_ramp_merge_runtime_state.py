from __future__ import annotations

from dataclasses import replace
import sys
from pathlib import Path
from types import MappingProxyType

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from cormc.onestep.rolling import (
    GapPlan,
    LateralTrajectoryRef,
    RampMergeRuntimeState,
    build_initial_onestep_stage2_state,
    refresh_runtime_state,
)
from cormc.scenes import (
    RM_ONESTEP_S07_2MV_MV_IDS,
    RM_ONESTEP_S07_2MV_REAR_MV_ID,
    RM_ONESTEP_S07_2MV_ROLLING_ENTRY_SCENARIO_ID,
    RM_ONESTEP_S07_MV_ID,
)


def test_runtime_state_initializes_two_mvs_with_empty_plan_registries() -> None:
    state, _ = build_initial_onestep_stage2_state(
        RM_ONESTEP_S07_2MV_ROLLING_ENTRY_SCENARIO_ID
    )
    runtime = state.ramp_merge_runtime

    assert isinstance(runtime, RampMergeRuntimeState)
    assert tuple(runtime.mv_plan_states) == RM_ONESTEP_S07_2MV_MV_IDS
    assert dict(runtime.gap_plans) == {}
    assert dict(runtime.lateral_trajectories) == {}
    for mv_id in RM_ONESTEP_S07_2MV_MV_IDS:
        mv_state = runtime.mv_plan_states[mv_id]
        assert mv_state.current_plan_id is None
        assert mv_state.locked_plan_id is None
        assert mv_state.active_lateral_trajectory_id is None


def test_refresh_runtime_state_filters_gap_plan_and_lateral_registries_by_active_mv() -> None:
    state, _ = build_initial_onestep_stage2_state(
        RM_ONESTEP_S07_2MV_ROLLING_ENTRY_SCENARIO_ID
    )
    runtime = state.ramp_merge_runtime
    active_plan = _gap_plan("plan-active", RM_ONESTEP_S07_MV_ID)
    stale_plan = _gap_plan("plan-stale", RM_ONESTEP_S07_2MV_REAR_MV_ID)
    active_lateral = LateralTrajectoryRef(
        trajectory_id="lat-active",
        owner_mv_id=RM_ONESTEP_S07_MV_ID,
        source_plan_id=active_plan.plan_id,
        start_step=state.step,
        start_t=state.t,
        start_y=-3.5,
        target_y=0.0,
        duration_steps=10,
    )
    stale_lateral = LateralTrajectoryRef(
        trajectory_id="lat-stale",
        owner_mv_id=RM_ONESTEP_S07_2MV_REAR_MV_ID,
        source_plan_id=stale_plan.plan_id,
        start_step=state.step,
        start_t=state.t,
        start_y=-3.5,
        target_y=0.0,
        duration_steps=10,
    )
    runtime = replace(
        runtime,
        gap_plans=MappingProxyType(
            {
                active_plan.plan_id: active_plan,
                stale_plan.plan_id: stale_plan,
            }
        ),
        lateral_trajectories=MappingProxyType(
            {
                active_lateral.trajectory_id: active_lateral,
                stale_lateral.trajectory_id: stale_lateral,
            }
        ),
    )
    without_rear_mv = _without_vehicle(state, RM_ONESTEP_S07_2MV_REAR_MV_ID)

    refreshed = refresh_runtime_state(runtime, without_rear_mv)

    assert tuple(refreshed.gap_plans) == ("plan-active",)
    assert tuple(refreshed.lateral_trajectories) == ("lat-active",)


def test_refresh_runtime_state_clears_missing_plan_and_lateral_ids_from_mv_state() -> None:
    state, _ = build_initial_onestep_stage2_state(
        RM_ONESTEP_S07_2MV_ROLLING_ENTRY_SCENARIO_ID
    )
    runtime = state.ramp_merge_runtime
    mv_state = runtime.mv_plan_states[RM_ONESTEP_S07_MV_ID]
    runtime = replace(
        runtime,
        mv_plan_states=MappingProxyType(
            {
                **dict(runtime.mv_plan_states),
                RM_ONESTEP_S07_MV_ID: replace(
                    mv_state,
                    current_plan_id="missing-current-plan",
                    locked_plan_id="missing-locked-plan",
                    active_lateral_trajectory_id="missing-lateral",
                ),
            }
        ),
    )

    refreshed = refresh_runtime_state(runtime, state)
    refreshed_mv = refreshed.mv_plan_states[RM_ONESTEP_S07_MV_ID]

    assert refreshed_mv.current_plan_id is None
    assert refreshed_mv.locked_plan_id is None
    assert refreshed_mv.active_lateral_trajectory_id is None


def _gap_plan(plan_id: str, mv_id: str) -> GapPlan:
    return GapPlan(
        plan_id=plan_id,
        mv_id=mv_id,
        gap_id="gap:0:1",
        gap_index=1,
        front_vehicle_id="S07_L2_02",
        rear_vehicle_id="S07_L2_01",
        snapshot_step=0,
        snapshot_t=0.0,
    )


def _without_vehicle(state, vehicle_id: str):
    return replace(
        state,
        active_vehicle_ids=tuple(
            active_id for active_id in state.active_vehicle_ids if active_id != vehicle_id
        ),
        vehicle_states={
            active_id: vehicle_state
            for active_id, vehicle_state in state.vehicle_states.items()
            if active_id != vehicle_id
        },
        vehicle_specs={
            active_id: vehicle_spec
            for active_id, vehicle_spec in state.vehicle_specs.items()
            if active_id != vehicle_id
        },
    )
