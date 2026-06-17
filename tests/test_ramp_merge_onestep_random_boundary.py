from dataclasses import replace

from cormc.traffic_flow.generation import ArrivalStream, SeededRandomProfile, generate_boundary_queue
from cormc.onestep.rolling import RampMergeEngine
from cormc.scenes import (
    RM_ONESTEP_RANDOM_S07_LANE2_RAMP_100S_SCENARIO_ID,
    load_traffic_flow_scene_config,
)
from cormc.simulation_core.pre_freeze import (
    ON_RAMP,
    ON_RAMP_MV_ROLE,
    build_prefreeze_workspace_from_scenario,
    freeze_simulation_state,
)
from cormc.onestep.rolling.state import initialize_runtime_state


def test_onestep_engine_generates_random_on_ramp_mv_before_freeze() -> None:
    config = load_traffic_flow_scene_config(
        RM_ONESTEP_RANDOM_S07_LANE2_RAMP_100S_SCENARIO_ID
    )
    queue = generate_boundary_queue(_single_on_ramp_profile(), max_t=0.2)
    state = freeze_simulation_state(build_prefreeze_workspace_from_scenario(config)[0])
    state = replace(state, t=0.2)
    state = replace(state, ramp_merge_runtime=initialize_runtime_state(state))
    engine = RampMergeEngine(
        config,
        algorithm_variant="onestep_stage2",
        random_queue=queue,
        safe_spawn_gap_m=1.0,
    )

    result = engine.advance_one_step(state)

    boundary_index = next(
        i for i, event in enumerate(result.actual_events) if event["event_type"] == "boundary_generation"
    )
    freeze_index = next(
        i for i, event in enumerate(result.actual_events) if event["event_type"] == "freeze"
    )
    generated_id = queue[0].vehicle_id
    generated_state = result.frozen_state.vehicle_states[generated_id]

    assert boundary_index < freeze_index
    assert generated_id in result.frozen_state.active_vehicle_ids
    assert generated_state.road_role == ON_RAMP_MV_ROLE
    assert generated_state.merge_state == "not_started"
    assert generated_id in result.ramp_merge_runtime.mv_plan_states


def test_onestep_engine_records_blocked_random_spawn_without_inserting() -> None:
    config = load_traffic_flow_scene_config(
        RM_ONESTEP_RANDOM_S07_LANE2_RAMP_100S_SCENARIO_ID
    )
    queue = generate_boundary_queue(_single_on_ramp_profile(), max_t=0.2)
    state = freeze_simulation_state(build_prefreeze_workspace_from_scenario(config)[0])
    state = replace(state, t=0.2)
    state = replace(state, ramp_merge_runtime=initialize_runtime_state(state))
    engine = RampMergeEngine(
        config,
        algorithm_variant="onestep_stage2",
        random_queue=queue,
        safe_spawn_gap_m=500.0,
    )

    result = engine.advance_one_step(state)
    boundary = next(
        event for event in result.actual_events if event["event_type"] == "boundary_generation"
    )
    payload = boundary["payload"]

    assert queue[0].vehicle_id in payload["blocked_spawn_vehicle_ids"]
    assert payload["blocked_reason"][queue[0].vehicle_id] == "safe_spawn_gap_not_met"
    assert queue[0].vehicle_id not in result.frozen_state.active_vehicle_ids


def _single_on_ramp_profile() -> SeededRandomProfile:
    return SeededRandomProfile(
        seed=123,
        profile_id="test_on_ramp_one_item",
        arrival_streams=(
            ArrivalStream(
                lane_id=ON_RAMP,
                road_role=ON_RAMP_MV_ROLE,
                merge_state="not_started",
                spawn_x=6450.0,
                spawn_y=-3.5,
                initial_speed=20.0,
                shifted_headway=0.1,
                mean_headway=1.0,
                vehicle_id_prefix="test_random",
                vehicle_id_lane_label=ON_RAMP,
            ),
        ),
    )
