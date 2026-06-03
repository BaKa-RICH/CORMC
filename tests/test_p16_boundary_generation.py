from __future__ import annotations

import ast
from pathlib import Path

from cormc.random_generation import (
    ArrivalStream,
    SeededRandomProfile,
    compute_spawn_decisions,
    generate_boundary_queue,
)
from cormc.step0_3 import (
    VehicleSpec,
    VehicleState,
    build_prefreeze_workspace_from_scenario,
    freeze_simulation_state,
    run_step0_to_step3,
)


def test_p16_boundary_generation_event_precedes_freeze_and_generated_vehicle_is_frozen() -> None:
    profile = _single_lane_profile(spawn_x=0.0, mean_headway=0.2, safe_spawn_gap_m=20.0)
    queue = generate_boundary_queue(profile, max_t=0.5)
    scenario = _scenario([], boundary_enabled=True, t=0.5)
    state = freeze_simulation_state(build_prefreeze_workspace_from_scenario(scenario)[0])
    decisions = compute_spawn_decisions(queue, state, safe_spawn_gap_m=profile.safe_spawn_gap_m)

    result = run_step0_to_step3(scenario, boundary_spawn_decisions=decisions)
    events = result.actual_events
    boundary_index = next(i for i, event in enumerate(events) if event["event_type"] == "boundary_generation")
    freeze_index = next(i for i, event in enumerate(events) if event["event_type"] == "freeze")
    generated_id = decisions[0].queue_item.vehicle_id

    assert boundary_index < freeze_index
    assert generated_id in result.state.active_vehicle_ids
    assert result.state.vehicle_states[generated_id].physical_lane == "lane_1"
    assert result.state.vehicle_states[generated_id].road_role == "mainline"
    assert result.state.vehicle_states[generated_id].merge_state == "none"
    assert result.state.vehicle_specs[generated_id].source_lane_at_generation == "lane_1"
    assert events[boundary_index]["payload"]["freeze_phase"] == "pre_freeze"
    assert events[boundary_index]["payload"]["seed"] == 99
    assert events[boundary_index]["payload"]["profile_id"] == "boundary-unit"
    assert events[boundary_index]["payload"]["lane_id"][generated_id] == "lane_1"
    assert generated_id in events[boundary_index]["payload"]["generated_vehicle_ids"]
    assert events[boundary_index]["payload"]["random_generation_complete_mechanism"] == (
        "p16_seeded_boundary_queue_spawn_decisions"
    )


def test_p16_blocked_spawn_records_event_without_inserting_vehicle() -> None:
    profile = _single_lane_profile(spawn_x=0.0, mean_headway=0.2, safe_spawn_gap_m=20.0)
    queue = generate_boundary_queue(profile, max_t=0.5)
    scenario = _scenario([_vehicle("EXISTING", "lane_1", 5.0, 3.5)], boundary_enabled=True, t=0.5)
    state = freeze_simulation_state(build_prefreeze_workspace_from_scenario(scenario)[0])
    decisions = compute_spawn_decisions(queue, state, safe_spawn_gap_m=profile.safe_spawn_gap_m)

    result = run_step0_to_step3(scenario, boundary_spawn_decisions=decisions)
    boundary = next(event for event in result.actual_events if event["event_type"] == "boundary_generation")
    blocked_id = decisions[0].queue_item.vehicle_id

    assert decisions[0].generated is False
    assert blocked_id not in result.state.active_vehicle_ids
    assert "EXISTING" in result.state.active_vehicle_ids
    assert boundary["reason"] == "blocked_safe_spawn_gap"
    assert blocked_id in boundary["payload"]["blocked_spawn_vehicle_ids"]
    assert boundary["payload"]["blocked_reason"][blocked_id] == "safe_spawn_gap_not_met"


def test_p16_boundary_generation_does_not_modify_existing_vehicle_kinematics() -> None:
    scenario = _scenario([_vehicle("EXISTING", "lane_1", 50.0, 3.5)], boundary_enabled=True, t=0.5)
    before = freeze_simulation_state(build_prefreeze_workspace_from_scenario(scenario)[0])
    profile = _single_lane_profile(spawn_x=0.0, mean_headway=0.2, safe_spawn_gap_m=20.0)
    decisions = compute_spawn_decisions(
        generate_boundary_queue(profile, max_t=0.5),
        before,
        safe_spawn_gap_m=profile.safe_spawn_gap_m,
    )

    result = run_step0_to_step3(scenario, boundary_spawn_decisions=decisions)
    after = result.state.vehicle_states["EXISTING"]

    assert (after.x_global, after.y, after.v, after.a) == (
        before.vehicle_states["EXISTING"].x_global,
        before.vehicle_states["EXISTING"].y,
        before.vehicle_states["EXISTING"].v,
        before.vehicle_states["EXISTING"].a,
    )


def test_p16_explicit_candidates_still_supported_for_p02_compatibility() -> None:
    scenario = _scenario([], boundary_enabled=True, t=0.0)
    candidate = VehicleState(
        vehicle_id="EXPLICIT",
        x_global=0.0,
        y=0.0,
        v=20.0,
        a=0.0,
        physical_lane="lane_2",
        road_role="mainline",
    )
    spec = VehicleSpec(
        vehicle_id="EXPLICIT",
        vehicle_type="cav",
        compliance_state="not_applicable",
    )

    result = run_step0_to_step3(scenario, boundary_vehicle_candidates=[(candidate, spec)])

    assert "EXPLICIT" in result.state.active_vehicle_ids
    boundary = next(event for event in result.actual_events if event["event_type"] == "boundary_generation")
    assert boundary["payload"]["random_generation_complete_mechanism"] == (
        "p16_seeded_boundary_queue_spawn_decisions"
    )


def test_p16_algorithm_modules_do_not_depend_on_seed_or_rng_calls() -> None:
    for path in [
        Path("cormc/step4a_aps.py"),
        Path("cormc/step4b_cmc.py"),
        Path("cormc/step6_cuc.py"),
    ]:
        tree = ast.parse(path.read_text(encoding="utf-8"))
        imported_modules = {
            node.module
            for node in ast.walk(tree)
            if isinstance(node, ast.ImportFrom) and node.module is not None
        }
        imported_names = {
            alias.name
            for node in ast.walk(tree)
            if isinstance(node, ast.Import)
            for alias in node.names
        }
        called_attrs = {
            node.func.attr
            for node in ast.walk(tree)
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
        }
        source = path.read_text(encoding="utf-8")

        assert "random" not in imported_modules
        assert "random" not in imported_names
        assert "seed" not in source
        assert {"random", "uniform", "gauss", "expovariate"}.isdisjoint(called_attrs)


def _single_lane_profile(
    *,
    spawn_x: float,
    mean_headway: float,
    safe_spawn_gap_m: float,
) -> SeededRandomProfile:
    return SeededRandomProfile(
        seed=99,
        profile_id="boundary-unit",
        arrival_streams=(
            ArrivalStream(
                lane_id="lane_1",
                shifted_headway=0.1,
                initial_speed=30.0,
                spawn_x=spawn_x,
                spawn_y=3.5,
                mean_headway=mean_headway,
            ),
        ),
        cav_penetration_rate=1.0,
        chv_compliance_rate=0.75,
        safe_spawn_gap_m=safe_spawn_gap_m,
    )


def _scenario(
    vehicles: list[dict],
    *,
    boundary_enabled: bool,
    t: float,
) -> dict:
    return {
        "scenario_id": "P16-BOUNDARY-UNIT",
        "scenario_name": "P16-BOUNDARY-UNIT",
        "purpose": "P16 boundary generation unit test",
        "test_level": "unit",
        "status": "required",
        "initial_time": {"t": t, "step": int(round(t * 10)), "dt": 0.1},
        "initial_vehicles": vehicles,
        "module_overrides": {
            "boundary_generation_enabled": boundary_enabled,
            "random_arrival_enabled": boundary_enabled,
            "random_vehicle_attributes_enabled": boundary_enabled,
            "ordinary_mainline_lane_change_enabled": False,
            "platoon_cmc_enabled": False,
            "mpc_lateral_tracking_enabled": False,
        },
        "preloaded_assignments": [],
        "preloaded_state_machine_states": [],
        "preloaded_maneuver_trajectory_states": [],
        "expected_events": [],
        "forbidden_events": [],
        "expected_event_counts": [],
        "expected_sanity_checks": [],
        "expected_png_features": [],
    }


def _vehicle(vehicle_id: str, lane_id: str, x_global: float, y: float) -> dict:
    return {
        "vehicle_id": vehicle_id,
        "vehicle_type": "CAV",
        "compliance_state": "not_applicable",
        "initial_x_global": x_global,
        "initial_y": y,
        "initial_v": 20.0,
        "initial_a": 0.0,
        "physical_lane": lane_id,
        "road_role": "mainline",
        "lane_change_state": "normal",
        "merge_state": "none",
        "spec_overrides": {},
    }
