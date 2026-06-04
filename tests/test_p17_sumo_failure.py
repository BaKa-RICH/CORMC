from __future__ import annotations

from pathlib import Path

from cormc.sumo.env import ensure_sumo_available_or_skip
from cormc.sumo.loop import (
    CORMC_FAILURE,
    INTEGRATION_FAILURE,
    SumoClosedLoopRunnerConfig,
    run_sumo_trajectory_authority_simulation,
)


def test_active_collision_real_sumo_classifies_cormc_failure(tmp_path: Path) -> None:
    ensure_sumo_available_or_skip()

    result = run_sumo_trajectory_authority_simulation(
        SumoClosedLoopRunnerConfig(
            run_id="p17-active-collision",
            output_dir=tmp_path / "collision",
            seed=16001,
            max_steps=12,
            force_active_collision=True,
        )
    )

    assert result.status == CORMC_FAILURE
    assert result.collision_count > 0
    assert any("MV_ACTIVE" in event.get("vehicle_ids", ()) for event in result.collision_events)


def test_spawn_add_failure_classifies_integration_failure(monkeypatch, tmp_path: Path) -> None:
    ensure_sumo_available_or_skip()

    from cormc.sumo import spawn

    original = spawn.SumoSpawnAdapter.realize_decision

    def fail_generated(self, decision, *, traci_module=None):
        if getattr(decision, "generated", False):
            return spawn.SpawnRealization(
                vehicle_id=decision.queue_item.vehicle_id,
                result=INTEGRATION_FAILURE,
                reason="traci_add_or_move_failed",
                error="forced add failure",
            )
        return original(self, decision, traci_module=traci_module)

    monkeypatch.setattr(spawn.SumoSpawnAdapter, "realize_decision", fail_generated)

    result = run_sumo_trajectory_authority_simulation(
        SumoClosedLoopRunnerConfig(
            run_id="p17-spawn-failure",
            output_dir=tmp_path / "spawn-failure",
            seed=16001,
            max_steps=45,
        )
    )

    assert result.status == INTEGRATION_FAILURE
    assert result.integration_failure_count >= 1
    assert result.spawn_failures


def test_mapping_failure_classifies_integration_failure(monkeypatch, tmp_path: Path) -> None:
    ensure_sumo_available_or_skip()

    import cormc.sumo.loop as loop

    def fail_add_vehicle(*args, **kwargs):
        raise ValueError("forced mapping failure")

    monkeypatch.setattr(loop, "_add_vehicle", fail_add_vehicle)

    result = run_sumo_trajectory_authority_simulation(
        SumoClosedLoopRunnerConfig(
            run_id="p17-mapping-failure",
            output_dir=tmp_path / "mapping-failure",
            seed=16001,
            max_steps=5,
        )
    )

    assert result.status == INTEGRATION_FAILURE
    assert result.integration_failure_count >= 1
    assert result.spawn_failures[0]["reason"] == "runner_exception"
