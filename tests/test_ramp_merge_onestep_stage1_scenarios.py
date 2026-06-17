from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from cormc.simulation_core.pre_freeze import build_prefreeze_workspace_from_scenario, freeze_simulation_state
from cormc.scenes import (
    RM_ONESTEP_S05_MAINLINE_X_GLOBAL,
    RM_ONESTEP_S05_MAINLINE_X_LOCAL,
    RM_ONESTEP_S05_MV_ID,
    RM_ONESTEP_S05_PLAN_STEP0_SCENARIO_ID,
    RM_ONESTEP_S05_ROLLING_ENTRY_SCENARIO_ID,
    RM_ONESTEP_S05_SCENARIO_IDS,
    RM_ONESTEP_S07_MAINLINE_X_GLOBAL,
    RM_ONESTEP_S07_MAINLINE_X_LOCAL,
    RM_ONESTEP_S07_MV_ID,
    RM_ONESTEP_S07_2MV_ROLLING_ENTRY_SCENARIO_ID,
    RM_ONESTEP_S07_PLAN_STEP0_SCENARIO_ID,
    RM_ONESTEP_S07_ROLLING_ENTRY_SCENARIO_ID,
    RM_ONESTEP_S07_SCENARIO_IDS,
    RM_ONESTEP_SCENARIO_IDS,
    get_ramp_merge_onestep_case_spec,
    get_ramp_merge_onestep_stage1_expectation,
    load_scene_config,
)
from cormc.onestep.rolling.state import initialize_runtime_state


def test_onestep_stage1_catalog_exposes_s05_and_s07_scene_ids() -> None:
    scenarios = {scenario_id: load_scene_config(scenario_id) for scenario_id in RM_ONESTEP_SCENARIO_IDS}

    assert RM_ONESTEP_S05_SCENARIO_IDS == (
        RM_ONESTEP_S05_PLAN_STEP0_SCENARIO_ID,
        RM_ONESTEP_S05_ROLLING_ENTRY_SCENARIO_ID,
    )
    assert RM_ONESTEP_S07_SCENARIO_IDS == (
        RM_ONESTEP_S07_PLAN_STEP0_SCENARIO_ID,
        RM_ONESTEP_S07_ROLLING_ENTRY_SCENARIO_ID,
        RM_ONESTEP_S07_2MV_ROLLING_ENTRY_SCENARIO_ID,
    )
    assert tuple(scenarios) == RM_ONESTEP_SCENARIO_IDS


@pytest.mark.parametrize(
    ("scenario_id", "mv_id", "expected_mv_x_global", "expected_positions", "expected_offsets"),
    [
        (
            RM_ONESTEP_S05_PLAN_STEP0_SCENARIO_ID,
            RM_ONESTEP_S05_MV_ID,
            6650.0,
            RM_ONESTEP_S05_MAINLINE_X_GLOBAL,
            RM_ONESTEP_S05_MAINLINE_X_LOCAL,
        ),
        (
            RM_ONESTEP_S07_PLAN_STEP0_SCENARIO_ID,
            RM_ONESTEP_S07_MV_ID,
            6650.0,
            RM_ONESTEP_S07_MAINLINE_X_GLOBAL,
            RM_ONESTEP_S07_MAINLINE_X_LOCAL,
        ),
    ],
)
def test_plan_step0_scene_globalizes_mainline_layout_and_places_mv_in_control_zone(
    scenario_id: str,
    mv_id: str,
    expected_mv_x_global: float,
    expected_positions: tuple[float, ...],
    expected_offsets: tuple[float, ...],
) -> None:
    config = load_scene_config(scenario_id)
    workspace, _ = build_prefreeze_workspace_from_scenario(config)
    state = freeze_simulation_state(workspace)
    runtime = initialize_runtime_state(state)

    mv = state.vehicle_states[mv_id]
    lane_2_positions = tuple(
        state.vehicle_states[vehicle_id].x_global
        for vehicle_id in sorted(
            (
                vehicle_id
                for vehicle_id in state.active_vehicle_ids
                if state.vehicle_states[vehicle_id].physical_lane == "lane_2"
            ),
            key=lambda vehicle_id: state.vehicle_states[vehicle_id].x_global,
        )
    )

    assert mv.x_global == pytest.approx(expected_mv_x_global)
    assert lane_2_positions == pytest.approx(expected_positions)
    assert tuple(position - mv.x_global for position in lane_2_positions) == pytest.approx(
        expected_offsets
    )
    assert runtime.mv_plan_states[mv_id].zone_state == "control_zone"


@pytest.mark.parametrize(
    ("plan_id", "rolling_id", "mv_id"),
    [
        (
            RM_ONESTEP_S05_PLAN_STEP0_SCENARIO_ID,
            RM_ONESTEP_S05_ROLLING_ENTRY_SCENARIO_ID,
            RM_ONESTEP_S05_MV_ID,
        ),
        (
            RM_ONESTEP_S07_PLAN_STEP0_SCENARIO_ID,
            RM_ONESTEP_S07_ROLLING_ENTRY_SCENARIO_ID,
            RM_ONESTEP_S07_MV_ID,
        ),
    ],
)
def test_rolling_entry_scene_shifts_whole_layout_upstream_and_starts_outside_control_zone(
    plan_id: str,
    rolling_id: str,
    mv_id: str,
) -> None:
    plan = load_scene_config(plan_id)
    rolling = load_scene_config(rolling_id)
    workspace, _ = build_prefreeze_workspace_from_scenario(rolling)
    state = freeze_simulation_state(workspace)
    runtime = initialize_runtime_state(state)

    plan_vehicle_map = {
        item["vehicle_id"]: item["initial_x_global"]
        for item in plan["initial_vehicles"]
    }
    rolling_vehicle_map = {
        item["vehicle_id"]: item["initial_x_global"]
        for item in rolling["initial_vehicles"]
    }

    assert rolling_vehicle_map[mv_id] == pytest.approx(6600.0)
    assert plan_vehicle_map[mv_id] == pytest.approx(6650.0)
    for vehicle_id, x_global in rolling_vehicle_map.items():
        assert x_global == pytest.approx(plan_vehicle_map[vehicle_id] - 50.0)
    assert state.dt == pytest.approx(0.1)
    assert runtime.mv_plan_states[mv_id].zone_state == "outside_control_zone"


@pytest.mark.parametrize(
    ("scenario_id", "expected_case_id", "expected_mv_id", "expected_x_targets"),
    [
        (
            RM_ONESTEP_S05_PLAN_STEP0_SCENARIO_ID,
            "S05",
            RM_ONESTEP_S05_MV_ID,
            RM_ONESTEP_S05_MAINLINE_X_LOCAL,
        ),
        (
            RM_ONESTEP_S07_PLAN_STEP0_SCENARIO_ID,
            "S07",
            RM_ONESTEP_S07_MV_ID,
            RM_ONESTEP_S07_MAINLINE_X_LOCAL,
        ),
    ],
)
def test_case_spec_exposes_expected_bridge_metadata(
    scenario_id: str,
    expected_case_id: str,
    expected_mv_id: str,
    expected_x_targets: tuple[float, ...],
) -> None:
    spec = get_ramp_merge_onestep_case_spec(scenario_id)

    assert spec.one_step_case_id == expected_case_id
    assert spec.mv_id == expected_mv_id
    assert spec.mainline_x_local == expected_x_targets


@pytest.mark.parametrize(
    ("plan_id", "rolling_id"),
    [
        (RM_ONESTEP_S05_PLAN_STEP0_SCENARIO_ID, RM_ONESTEP_S05_ROLLING_ENTRY_SCENARIO_ID),
        (RM_ONESTEP_S07_PLAN_STEP0_SCENARIO_ID, RM_ONESTEP_S07_ROLLING_ENTRY_SCENARIO_ID),
    ],
)
def test_onestep_stage1_expectations_are_fixed_constants(
    plan_id: str,
    rolling_id: str,
) -> None:
    plan = get_ramp_merge_onestep_stage1_expectation(plan_id)
    rolling = get_ramp_merge_onestep_stage1_expectation(rolling_id)

    assert plan.mode == "plan_step0"
    assert plan.expected_first_check_step == 0
    assert plan.expected_first_check_t == pytest.approx(0.0)
    assert plan.expected_initial_zone_state == "control_zone"
    assert plan.expected_first_trigger_reason == "periodic"
    assert plan.expected_mv_x_global_at_check == pytest.approx(6650.0)
    assert plan.expected_mv_x_local_at_check == pytest.approx(0.0)

    assert rolling.mode == "rolling_entry"
    assert rolling.expected_first_check_step == 25
    assert rolling.expected_first_check_t == pytest.approx(2.5)
    assert rolling.expected_initial_zone_state == "outside_control_zone"
    assert rolling.expected_first_trigger_reason == "MV_enter_control_zone"
    assert rolling.expected_mv_x_global_at_check == pytest.approx(6650.0)
    assert rolling.expected_mv_x_local_at_check == pytest.approx(0.0)
