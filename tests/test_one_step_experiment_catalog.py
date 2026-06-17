from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from cormc.onestep.lab.experiments import (
    ONE_STEP_FIXED_SCENARIO_IDS,
    ONE_STEP_SCENARIO_IDS,
    ONE_STEP_SWEEP_SCENARIO_IDS,
    build_default_one_step_algorithm_config,
    build_one_step_scenario_config,
    load_one_step_experiment_scenario,
    load_one_step_experiment_scenarios,
)


def test_one_step_catalog_contains_s01_to_s14() -> None:
    scenarios = load_one_step_experiment_scenarios()

    assert ONE_STEP_SCENARIO_IDS == (
        "S01",
        "S02",
        "S03",
        "S04",
        "S05",
        "S06",
        "S07",
        "S08",
        "S09",
        "S10",
        "S11",
        "S12",
        "S13",
        "S14",
    )
    assert tuple(scenarios) == ONE_STEP_SCENARIO_IDS


def test_fixed_scenarios_define_fixed_expectations_and_sweeps_exist_where_expected() -> None:
    scenarios = load_one_step_experiment_scenarios()

    assert all(scenarios[scenario_id].fixed_expectation is not None for scenario_id in ONE_STEP_FIXED_SCENARIO_IDS)
    assert scenarios["S11"].sweep is not None
    assert scenarios["S13"].sweep is not None
    assert scenarios["S14"].sweep is not None
    assert ONE_STEP_SWEEP_SCENARIO_IDS == ("S11", "S13", "S14")


def test_scenario_and_algorithm_builders_match_s01_strict_defaults() -> None:
    scenario = load_one_step_experiment_scenario("S01")
    config = build_one_step_scenario_config(scenario.x_targets)
    algorithm = build_default_one_step_algorithm_config()

    assert config.x_targets == (-180.0, -90.0, -25.0, 30.0, 110.0, 190.0, 250.0)
    assert config.x_m0 == 0.0
    assert config.v_ref == 20.0
    assert config.v_max == 30.0
    assert config.v_min == 0.0
    assert config.a_max == 3.0
    assert config.a_min == -4.0
    assert config.T == 20.0

    assert algorithm.D_h == 40.0
    assert algorithm.l_m == 5.0
    assert algorithm.w_c == 0.2
    assert algorithm.w_e == 1.0
    assert algorithm.w_t == 10.0
    assert algorithm.delta_ref == 35.0
    assert algorithm.q == 6.0
    assert algorithm.epsilon_delta == 0.05
    assert algorithm.K == 120.0 / 7.0
    assert algorithm.boundary_adjustment == 100.0
