from __future__ import annotations

from cormc.legacy.artifact_reports import run_full_required_mvs_smoke_suite
from cormc.traffic_flow.generation import P16_DEMO_SCENARIO_ID
from cormc.scenes import BASIC_SCENARIO_IDS
from cormc.simulation_core.loop import (
    SimulationLoopConfig,
    run_deterministic_simulation,
    run_seeded_random_simulation,
)


def test_p16_same_seed_same_config_replays_trajectory_events_and_sanity() -> None:
    left = _run(seed=16001, run_id="same-seed")
    right = _run(seed=16001, run_id="same-seed")

    assert _trajectory_signature(left) == _trajectory_signature(right)
    assert _event_signature(left) == _event_signature(right)
    assert _sanity_signature(left) == _sanity_signature(right)


def test_p16_different_seed_changes_generated_vehicle_stream() -> None:
    left = _run(seed=16001, run_id="seed-left")
    right = _run(seed=16002, run_id="seed-right")

    assert _generated_vehicle_ids(left) != _generated_vehicle_ids(right)


def test_p16_random_disabled_delegates_to_deterministic_runner() -> None:
    config = SimulationLoopConfig(
        scenario_id=BASIC_SCENARIO_IDS[0],
        run_id="p16-disabled",
        max_steps=1,
        render_png=False,
        random_enabled=False,
    )

    disabled = run_seeded_random_simulation(config)
    deterministic = run_deterministic_simulation(config)

    assert disabled.final_state == deterministic.final_state
    assert _trajectory_signature(disabled) == _trajectory_signature(deterministic)
    assert _event_signature(disabled) == _event_signature(deterministic)


def test_p16_seeded_runner_preserves_commit_and_time_advance_invariants() -> None:
    result = _run(seed=16001, run_id="invariants", max_steps=20)

    assert result.final_state.step == 20
    assert result.final_state.t > result.initial_state.t
    assert result.step_traces
    for trace in result.step_traces:
        assert trace.commit_result.previous_state == trace.step0_3_result.state
        assert trace.time_advance_result.previous_state == trace.commit_result.next_state
        assert trace.time_advance_result.advanced_state.step == trace.step + 1
        duplicate_checks = [
            check
            for check in trace.actual_sanity_checks
            if check.get("check_type") == "multiple_commit_for_one_vehicle"
        ]
        assert all(check.get("result") in {"pass", "not_applicable"} for check in duplicate_checks)
    assert not [
        check
        for check in result.history.sanity_dicts()
        if check["check_type"] == "no_write_before_commit" and check["result"] == "fail"
    ]


def test_p16_random_disabled_retired_mvs_suite_is_empty_and_nonblocking() -> None:
    suite = run_full_required_mvs_smoke_suite()
    assert suite.scenario_results == ()
    assert not [item for item in suite.scenario_results if item.blocks_required_suite]


def _run(seed: int, run_id: str, max_steps: int = 30):
    return run_seeded_random_simulation(
        SimulationLoopConfig(
            scenario_id=P16_DEMO_SCENARIO_ID,
            run_id=run_id,
            max_steps=max_steps,
            render_png=False,
            random_enabled=True,
            seed=seed,
        )
    )


def _trajectory_signature(result) -> tuple[tuple, ...]:
    return tuple(
        (
            record.step,
            round(record.t, 6),
            record.vehicle_id,
            record.vehicle_type,
            record.compliance_state,
            round(record.x_global, 6),
            round(record.y, 6),
            round(record.v, 6),
            round(record.a, 6),
            record.physical_lane,
            record.road_role,
            record.lane_change_state,
            record.merge_state,
        )
        for record in result.history.trajectory_records
    )


def _event_signature(result) -> tuple[tuple, ...]:
    return tuple(
        (
            record.step,
            round(record.t, 6),
            record.module,
            record.event_type,
            record.vehicle_id,
            record.related_vehicle_ids,
            record.reason,
            _plain(record.payload),
        )
        for record in result.history.event_records
    )


def _sanity_signature(result) -> tuple[tuple, ...]:
    return tuple(
        (
            record.step,
            round(record.t, 6),
            record.check_type,
            record.result,
            record.vehicle_ids,
            record.reason,
            _plain(record.payload),
        )
        for record in result.history.sanity_check_records
    )


def _generated_vehicle_ids(result) -> tuple[str, ...]:
    ids: list[str] = []
    for event in result.history.event_dicts():
        if event["event_type"] == "boundary_generation":
            ids.extend(event["payload"].get("generated_vehicle_ids") or [])
    return tuple(ids)


def _plain(value):
    if isinstance(value, dict):
        return tuple(sorted((str(key), _plain(item)) for key, item in value.items()))
    if isinstance(value, list):
        return tuple(_plain(item) for item in value)
    if isinstance(value, tuple):
        return tuple(_plain(item) for item in value)
    if isinstance(value, float):
        return round(value, 6)
    return value
