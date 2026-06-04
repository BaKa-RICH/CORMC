from __future__ import annotations

from pathlib import Path

from cormc.random_generation import ArrivalStream, SeededRandomProfile, compute_spawn_decisions, generate_boundary_queue
from cormc.step0_3 import build_prefreeze_workspace_from_scenario, freeze_simulation_state
from cormc.sumo.env import ensure_sumo_available_or_skip, import_traci
from cormc.sumo.network import P17SumoNetworkConfig, build_p17_sumo_network
from cormc.sumo.spawn import SpawnRegistry, SumoSpawnAdapter, spawn_command_from_decision


def test_spawn_real_sumo_adds_generated_once_and_duplicate_does_not_add(tmp_path: Path) -> None:
    paths = ensure_sumo_available_or_skip()
    files = build_p17_sumo_network(tmp_path, P17SumoNetworkConfig(end=1.0))
    traci = import_traci()
    traci.start([paths.sumo, "-c", files.sumocfg_file, "--no-step-log", "true", "--no-warnings", "true"])
    try:
        decision = _decision(lane_id="lane_1", spawn_x=0.0, t=0.5, generated=True)
        adapter = SumoSpawnAdapter(traci_module=traci)

        first = adapter.realize_decision(decision)
        second = adapter.realize_decision(decision)
        traci.simulationStep()

        vehicle_id = decision.queue_item.vehicle_id
        assert first.result == "generated"
        assert second.result == "duplicate"
        assert vehicle_id in set(traci.vehicle.getIDList())
        assert first.command is not None
        assert first.command.route_id == "route_main"
        assert first.command.vehicle_type == "cormc_active"
        assert first.vehicle_type == "cormc_active"
        assert first.compliance_state == "not_applicable"
    finally:
        traci.close(False)


def test_spawn_real_sumo_blocked_decision_does_not_add(tmp_path: Path) -> None:
    paths = ensure_sumo_available_or_skip()
    files = build_p17_sumo_network(tmp_path, P17SumoNetworkConfig(end=1.0))
    traci = import_traci()
    traci.start([paths.sumo, "-c", files.sumocfg_file, "--no-step-log", "true", "--no-warnings", "true"])
    try:
        decision = _decision(lane_id="lane_1", spawn_x=0.0, t=0.5, generated=False)
        result = SumoSpawnAdapter(traci_module=traci).realize_decision(decision)
        traci.simulationStep()

        assert result.result == "blocked"
        assert decision.queue_item.vehicle_id not in set(traci.vehicle.getIDList())
    finally:
        traci.close(False)


def test_spawn_command_maps_ramp_to_route_ramp_and_records_background_type() -> None:
    decision = _decision(lane_id="on_ramp", spawn_x=6850.0, t=0.5, generated=True, cav=False)

    command = spawn_command_from_decision(decision)

    assert command.route_id == "route_ramp"
    assert command.edge_id == "ramp_pre"
    assert command.lane_index == 0
    assert command.vehicle_type == "sumo_background"
    assert command.compliance_state == "compliant"
    assert command.source_queue_seed == 77
    assert command.source_profile_id == "spawn-unit"


def test_spawn_integration_failure_is_classified() -> None:
    decision = _decision(lane_id="lane_2", spawn_x=0.0, t=0.5, generated=True)
    registry = SpawnRegistry()

    result = SumoSpawnAdapter(traci_module=_FailingTraci(), registry=registry).realize_decision(decision)

    assert result.result == "integration_failure"
    assert result.reason == "traci_add_or_move_failed"
    assert decision.queue_item.vehicle_id in registry.failures


def _decision(*, lane_id: str, spawn_x: float, t: float, generated: bool, cav: bool = True):
    profile = SeededRandomProfile(
        seed=77,
        profile_id="spawn-unit",
        arrival_streams=(
            ArrivalStream(
                lane_id=lane_id,
                shifted_headway=0.1,
                initial_speed=16.0 if lane_id == "on_ramp" else 30.0,
                spawn_x=spawn_x,
                spawn_y=-3.5 if lane_id == "on_ramp" else (3.5 if lane_id == "lane_1" else 0.0),
                mean_headway=0.2,
            ),
        ),
        cav_penetration_rate=1.0 if cav else 0.0,
        chv_compliance_rate=1.0,
    )
    queue = generate_boundary_queue(profile, max_t=t)
    scenario = {
        "scenario_id": "P17-SPAWN",
        "scenario_name": "P17-SPAWN",
        "purpose": "P17 spawn adapter unit fixture",
        "test_level": "unit",
        "status": "required",
        "initial_time": {"t": t, "step": int(t * 10), "dt": 0.1},
        "initial_vehicles": [],
        "module_overrides": {},
        "preloaded_assignments": [],
        "preloaded_maneuver_trajectory_states": [],
        "expected_events": [],
        "forbidden_events": [],
        "expected_event_counts": [],
        "expected_sanity_checks": [],
        "expected_png_features": [],
    }
    state = freeze_simulation_state(build_prefreeze_workspace_from_scenario(scenario)[0])
    decision = compute_spawn_decisions(queue, state, safe_spawn_gap_m=20.0)[0]
    if generated:
        return decision
    return type(decision)(
        queue_item=decision.queue_item,
        generated=False,
        reason="blocked_safe_spawn_gap",
        blocked_reason="safe_spawn_gap_not_met",
        nearest_vehicle_id="EXISTING",
        nearest_gap_m=5.0,
    )


class _FailingVehicleApi:
    def add(self, *args, **kwargs):
        raise RuntimeError("forced add failure")


class _FailingTraci:
    vehicle = _FailingVehicleApi()
