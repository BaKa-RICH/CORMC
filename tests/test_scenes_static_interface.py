from __future__ import annotations

import pytest

from cormc.onestep.rolling.state import initialize_runtime_state
from cormc.scenes import (
    BASIC_SCENARIO_IDS,
    RM_MULTIMV_SCENARIO_IDS,
    RM_ONESTEP_S07_2MV_MV_IDS,
    RM_ONESTEP_S07_2MV_REAR_MV_ID,
    RM_ONESTEP_S07_2MV_ROLLING_ENTRY_SCENARIO_ID,
    ROLLING_BASIC_SCENARIO_ID,
    STATIC_SCENE_IDS,
    StaticSceneSpec,
    compile_static_scene,
    lane2,
    load_scene_config,
    mv,
)
from cormc.simulation_core.pre_freeze import build_prefreeze_workspace_from_scenario, freeze_simulation_state


def test_static_scene_interface_compiles_multi_mv_multi_mainline_scene():
    config = compile_static_scene(
        StaticSceneSpec(
            scenario_id="TEST-STATIC-2MV-3MAIN",
            scenario_name="static interface fixture",
            purpose="verify static scene interface",
            vehicles=(
                mv("MV_FRONT", 6600.0, speed=20.0),
                mv("MV_REAR", 6540.0, speed=19.0),
                lane2("L2_REAR", 6500.0, speed=18.0),
                lane2("L2_MID", 6605.0, speed=20.0),
                lane2("L2_FRONT", 6700.0, speed=21.0),
            ),
        )
    )

    workspace, loaded = build_prefreeze_workspace_from_scenario(config)
    state = freeze_simulation_state(workspace)
    runtime = initialize_runtime_state(state)

    assert loaded["scenario_id"] == "TEST-STATIC-2MV-3MAIN"
    assert tuple(runtime.mv_plan_states) == ("MV_FRONT", "MV_REAR")
    assert state.vehicle_states["MV_REAR"].x_global == pytest.approx(6540.0)
    assert state.vehicle_states["L2_FRONT"].v == pytest.approx(21.0)
    assert [
        vehicle_id
        for vehicle_id in state.active_vehicle_ids
        if state.vehicle_states[vehicle_id].physical_lane == "lane_2"
    ] == ["L2_REAR", "L2_MID", "L2_FRONT"]


def test_static_registry_contains_exact_official_scene_ids():
    assert STATIC_SCENE_IDS == (
        *BASIC_SCENARIO_IDS,
        ROLLING_BASIC_SCENARIO_ID,
        "RM-ONESTEP-S05-PLAN-STEP0",
        "RM-ONESTEP-S05-ROLLING-ENTRY",
        "RM-ONESTEP-S07-PLAN-STEP0",
        "RM-ONESTEP-S07-ROLLING-ENTRY",
        "RM-ONESTEP-S07-2MV-ROLLING-ENTRY",
        *RM_MULTIMV_SCENARIO_IDS,
    )
    assert len(STATIC_SCENE_IDS) == 33


def test_s07_2mv_scene_loads_through_unified_registry():
    config = load_scene_config(RM_ONESTEP_S07_2MV_ROLLING_ENTRY_SCENARIO_ID)
    vehicles = {item["vehicle_id"]: item for item in config["initial_vehicles"]}

    assert tuple(
        item["vehicle_id"]
        for item in config["initial_vehicles"]
        if item["road_role"] == "on_ramp_mv"
    ) == RM_ONESTEP_S07_2MV_MV_IDS
    assert vehicles[RM_ONESTEP_S07_2MV_REAR_MV_ID]["initial_x_global"] == pytest.approx(
        6540.0
    )
