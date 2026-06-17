from __future__ import annotations

import pytest

from cormc.onestep.rolling import (
    build_initial_onestep_stage2_state,
    build_stage2_local_frame,
    evaluate_stage2_one_step_from_snapshot,
    identify_and_number_gaps,
    effective_controllable,
)
from cormc.scenes import (
    RM_MULTIMV_M2_SCENARIO_IDS,
    RM_MULTIMV_M3_SCENARIO_IDS,
    RM_MULTIMV_M4_SCENARIO_IDS,
    RM_MULTIMV_ORIGIN_X_GLOBAL,
    RM_MULTIMV_SCENARIO_IDS,
    RM_MULTIMV_SCENE_SPECS,
    compile_static_scene,
    get_multimv_case_spec,
    load_scene_config,
)


def test_multimv_scene_ids_are_public_and_grouped() -> None:
    assert len(RM_MULTIMV_SCENARIO_IDS) == 21
    assert len(RM_MULTIMV_M2_SCENARIO_IDS) == 8
    assert len(RM_MULTIMV_M3_SCENARIO_IDS) == 7
    assert len(RM_MULTIMV_M4_SCENARIO_IDS) == 6
    assert RM_MULTIMV_SCENARIO_IDS[0] == "RM-M2-S01"
    assert RM_MULTIMV_SCENARIO_IDS[-1] == "RM-M4-S06"


def test_all_multimv_scenes_load_through_public_registry() -> None:
    for scenario_id in RM_MULTIMV_SCENARIO_IDS:
        config = load_scene_config(scenario_id)
        assert config["scenario_id"] == scenario_id
        assert compile_static_scene(RM_MULTIMV_SCENE_SPECS[scenario_id])["scenario_id"] == scenario_id


def test_multimv_scene_coordinates_follow_origin_policy() -> None:
    for scenario_id in RM_MULTIMV_SCENARIO_IDS:
        case = get_multimv_case_spec(scenario_id)
        config = load_scene_config(scenario_id)
        vehicles = {vehicle["vehicle_id"]: vehicle for vehicle in config["initial_vehicles"]}

        for index, x_m in enumerate(case.x_m_list, start=1):
            vehicle_id = f"{case.id.replace('-', '_')}_MV_{index:02d}"
            vehicle = vehicles[vehicle_id]
            assert vehicle["initial_x_global"] == pytest.approx(
                RM_MULTIMV_ORIGIN_X_GLOBAL + x_m
            )
            assert vehicle["physical_lane"] == "on_ramp"
            assert vehicle["road_role"] == "on_ramp_mv"
            assert vehicle["initial_y"] == pytest.approx(-3.5)
            assert vehicle["merge_state"] == "not_started"

        for index, x_target in enumerate(case.x_targets, start=1):
            vehicle_id = f"{case.id.replace('-', '_')}_L2_{index:02d}"
            vehicle = vehicles[vehicle_id]
            assert vehicle["initial_x_global"] == pytest.approx(
                RM_MULTIMV_ORIGIN_X_GLOBAL + x_target
            )
            assert vehicle["physical_lane"] == "lane_2"
            assert vehicle["road_role"] == "mainline"
            assert vehicle["initial_y"] == pytest.approx(0.0)


def test_multimv_hdv_cases_compile_as_uncontrollable_hdv() -> None:
    expected_hdv = {
        "RM-M2-S07": ("M2_S07_L2_02",),
        "RM-M4-S06": ("M4_S06_L2_02", "M4_S06_L2_03"),
    }
    for scenario_id, vehicle_ids in expected_hdv.items():
        config = load_scene_config(scenario_id)
        vehicles = {vehicle["vehicle_id"]: vehicle for vehicle in config["initial_vehicles"]}
        state, _ = build_initial_onestep_stage2_state(scenario_id)
        for vehicle_id in vehicle_ids:
            assert vehicles[vehicle_id]["vehicle_type"] == "HDV"
            assert effective_controllable(vehicle_id, state, ()) is False


def test_multimv_hdv_cases_reach_expected_controllability_rows() -> None:
    state, _ = build_initial_onestep_stage2_state("RM-M2-S07")
    snapshot = identify_and_number_gaps(state, ())
    local_frame = build_stage2_local_frame(state, "M2_S07_MV_01")
    evaluation = evaluate_stage2_one_step_from_snapshot(
        state,
        "M2_S07_MV_01",
        snapshot,
    ).evaluation
    rows_by_runtime_index = {
        local_frame["runtime_gap_index_by_kernel_index"][row.gap_index]: row
        for row in evaluation.gap_rows
    }
    assert rows_by_runtime_index[4].rear_vehicle_id == "M2_S07_L2_02"
    assert rows_by_runtime_index[4].rear_controllable is False
    assert rows_by_runtime_index[4].front_controllable is True

    state, _ = build_initial_onestep_stage2_state("RM-M4-S06")
    snapshot = identify_and_number_gaps(state, ())
    local_frame = build_stage2_local_frame(state, "M4_S06_MV_01")
    evaluation = evaluate_stage2_one_step_from_snapshot(
        state,
        "M4_S06_MV_01",
        snapshot,
    ).evaluation
    rows_by_runtime_index = {
        local_frame["runtime_gap_index_by_kernel_index"][row.gap_index]: row
        for row in evaluation.gap_rows
    }
    assert rows_by_runtime_index[5].rear_vehicle_id == "M4_S06_L2_03"
    assert rows_by_runtime_index[5].front_vehicle_id == "M4_S06_L2_04"
    assert rows_by_runtime_index[5].rear_controllable is False
    assert rows_by_runtime_index[5].front_controllable is True
    assert rows_by_runtime_index[6].rear_vehicle_id == "M4_S06_L2_02"
    assert rows_by_runtime_index[6].front_vehicle_id == "M4_S06_L2_03"
    assert rows_by_runtime_index[6].rear_controllable is False
    assert rows_by_runtime_index[6].front_controllable is False
