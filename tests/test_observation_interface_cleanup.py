from __future__ import annotations

import importlib
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


@pytest.mark.parametrize(
    "module_name",
    [
        "cormc.random_6450_runner",
        "cormc.sumo.basic_replay_artifacts",
        "cormc.sumo.rolling_basic_replay_artifacts",
        "cormc.sumo.random_6450_replay_artifacts",
        "cormc.sumo.ramp_merge_replay_artifacts",
        "cormc.sumo.mvs_replay_artifacts",
        "cormc.sumo.mvs_replay_specs",
    ],
)
def test_old_public_replay_modules_are_removed(module_name: str) -> None:
    with pytest.raises(ModuleNotFoundError):
        importlib.import_module(module_name)


@pytest.mark.parametrize(
    "module_name",
    [
        "cormc.observation.dataset",
        "cormc.observation.stage2_artifacts",
        "cormc.observation.plotting",
        "cormc.observation.sumo_replay",
        "cormc.observation.artifacts",
        "cormc.sumo.trajectory_gui_replay",
        "cormc.sumo.env",
        "cormc.sumo.network",
        "cormc.sumo.mapping",
    ],
)
def test_new_observation_and_sumo_foundation_modules_import(module_name: str) -> None:
    assert importlib.import_module(module_name)


def test_random_6450_runner_is_not_public_validation_path() -> None:
    import cormc
    import cormc.onestep

    assert not hasattr(cormc, "run_random_6450_numeric_scenario")
    assert not hasattr(cormc, "run_onestep_stage2_random_analysis")
    assert hasattr(cormc.onestep, "run_onestep_stage2_random_analysis")
