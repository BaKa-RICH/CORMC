from cormc.onestep.rolling.stage2_random_runner import (
    run_onestep_stage2_random_history,
)
from cormc.scenes import (
    RM_ONESTEP_RANDOM_S07_LANE2_RAMP_100S_SCENARIO_ID,
    build_onestep_random_lane2_ramp_scene,
)


def test_random_stage2_runner_produces_boundary_flow_summary() -> None:
    run = run_onestep_stage2_random_history(
        RM_ONESTEP_RANDOM_S07_LANE2_RAMP_100S_SCENARIO_ID,
        max_steps=60,
        run_id="test-random-stage2-runner",
    )
    summary = run.summary
    scenario = summary["scenario_summary"]

    assert set(summary) == {
        "scenario_summary",
        "round_summaries",
        "mv_summaries",
        "cross_mv_summary",
        "artifact_paths",
    }
    assert any(
        event["event_type"] == "boundary_generation"
        and event["payload"]["generated_vehicle_ids"]
        for event in run.actual_events
    )
    assert scenario["traffic_mode"] == "boundary_flow"
    assert scenario["boundary_flow_source"]["source_type"] == "seeded_random"
    assert scenario["generated_vehicle_count"] > 0
    assert any(count > 0 for count in scenario["generated_by_lane"].values())
    if scenario["generated_on_ramp_mv_count"] > 0:
        generated = set(scenario["generated_on_ramp_mv_ids"])
        accounted = set(summary["mv_summaries"]) | set(scenario["open_mv_ids_at_horizon"])
        assert generated <= accounted


def test_random_stage2_runner_accepts_custom_spec_parameters() -> None:
    first = run_onestep_stage2_random_history(
        build_onestep_random_lane2_ramp_scene(seed=645001, horizon_s=30.0, max_steps=20),
        max_steps=20,
        run_id="same-seed",
    )
    second = run_onestep_stage2_random_history(
        build_onestep_random_lane2_ramp_scene(seed=645001, horizon_s=30.0, max_steps=20),
        max_steps=20,
        run_id="same-seed-again",
    )
    different_seed = run_onestep_stage2_random_history(
        build_onestep_random_lane2_ramp_scene(seed=645002, horizon_s=30.0, max_steps=20),
        max_steps=20,
        run_id="different-seed",
    )
    high = run_onestep_stage2_random_history(
        build_onestep_random_lane2_ramp_scene(
            seed=645001,
            density="high",
            horizon_s=30.0,
            max_steps=20,
        ),
        max_steps=20,
        run_id="high-density",
    )
    faster = build_onestep_random_lane2_ramp_scene(
        seed=645001,
        horizon_s=30.0,
        max_steps=20,
        lane2_initial_speed=25.0,
    ).boundary_flow_source.build_queue(30.0)

    assert first.summary["scenario_summary"]["boundary_queue_size"] == second.summary[
        "scenario_summary"
    ]["boundary_queue_size"]
    assert first.summary["scenario_summary"]["boundary_queue_fingerprint"] != different_seed.summary[
        "scenario_summary"
    ]["boundary_queue_fingerprint"]
    assert high.summary["scenario_summary"]["boundary_queue_size"] > first.summary[
        "scenario_summary"
    ]["boundary_queue_size"]
    assert any(item.initial_state.v == 25.0 for item in faster if item.lane_id == "lane_2")
