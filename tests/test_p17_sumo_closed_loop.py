from __future__ import annotations

from pathlib import Path

from cormc.sumo.env import ensure_sumo_available_or_skip
from cormc.sumo.loop import (
    OK,
    SumoClosedLoopRunnerConfig,
    run_sumo_trajectory_authority_simulation,
)


def test_real_sumo_closed_loop_generates_active_commands_and_p16_spawn(tmp_path: Path) -> None:
    ensure_sumo_available_or_skip()

    result = run_sumo_trajectory_authority_simulation(
        SumoClosedLoopRunnerConfig(
            run_id="p17-closed-loop-smoke",
            output_dir=tmp_path / "smoke",
            seed=16001,
            max_steps=45,
        )
    )

    assert result.status == OK
    assert result.steps >= 30
    assert result.generated_count >= 1
    assert result.generated_vehicle_ids
    assert any(vehicle_id.startswith("p16_16001_on_ramp") for vehicle_id in result.generated_vehicle_ids)
    assert "MV_ACTIVE" in result.active_controlled_vehicle_ids
    assert result.command_count > 0
    assert result.realization_records
    assert result.sumo_config_path.endswith(".sumocfg")


def test_closed_loop_commands_are_only_for_authority_active_not_background(tmp_path: Path) -> None:
    ensure_sumo_available_or_skip()

    result = run_sumo_trajectory_authority_simulation(
        SumoClosedLoopRunnerConfig(
            run_id="p17-active-only",
            output_dir=tmp_path / "active-only",
            seed=16001,
            max_steps=35,
        )
    )

    commanded_ids = {
        vehicle_id
        for _, vehicle_ids in result.command_vehicle_ids_by_step
        for vehicle_id in vehicle_ids
    }

    assert len(result.background_vehicle_ids_sample) >= 2
    assert commanded_ids
    assert commanded_ids.isdisjoint(result.background_vehicle_ids_sample)
    assert set(result.active_controlled_vehicle_ids) == commanded_ids


def test_closed_loop_same_seed_same_signature_reproducible(tmp_path: Path) -> None:
    ensure_sumo_available_or_skip()

    left = run_sumo_trajectory_authority_simulation(
        SumoClosedLoopRunnerConfig(
            run_id="p17-same-left",
            output_dir=tmp_path / "same-left",
            seed=16001,
            max_steps=35,
        )
    )
    right = run_sumo_trajectory_authority_simulation(
        SumoClosedLoopRunnerConfig(
            run_id="p17-same-right",
            output_dir=tmp_path / "same-right",
            seed=16001,
            max_steps=35,
        )
    )

    assert _signature(left) == _signature(right)


def test_closed_loop_different_seed_changes_signature(tmp_path: Path) -> None:
    ensure_sumo_available_or_skip()

    left = run_sumo_trajectory_authority_simulation(
        SumoClosedLoopRunnerConfig(
            run_id="p17-seed-left",
            output_dir=tmp_path / "seed-left",
            seed=16001,
            max_steps=45,
        )
    )
    right = run_sumo_trajectory_authority_simulation(
        SumoClosedLoopRunnerConfig(
            run_id="p17-seed-right",
            output_dir=tmp_path / "seed-right",
            seed=16002,
            max_steps=45,
        )
    )

    assert _signature(left) != _signature(right)


def _signature(result) -> tuple[object, ...]:
    return (
        result.status,
        result.generated_vehicle_ids,
        result.active_controlled_vehicle_ids,
        tuple(
            (
                record.vehicle_id,
                record.step,
                round(record.command_x_global, 3),
                round(record.command_y, 3),
                round(record.command_v, 3),
                record.result,
            )
            for record in result.realization_records[:12]
        ),
        result.command_vehicle_ids_by_step[:12],
    )
