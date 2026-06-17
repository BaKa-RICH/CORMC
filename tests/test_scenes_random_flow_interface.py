from cormc.scenes import (
    RM_ONESTEP_RANDOM_S07_LANE2_RAMP_100S_SCENARIO_ID,
    RM_ONESTEP_SCENARIO_IDS,
    RM_ONESTEP_S07_2MV_MV_IDS,
    get_traffic_flow_scene_spec,
    load_scene_config,
    load_traffic_flow_scene_config,
)


def test_random_flow_scene_compiles_to_valid_scenario_config() -> None:
    config = load_traffic_flow_scene_config(
        RM_ONESTEP_RANDOM_S07_LANE2_RAMP_100S_SCENARIO_ID
    )

    assert config["scenario_id"] == RM_ONESTEP_RANDOM_S07_LANE2_RAMP_100S_SCENARIO_ID
    vehicles = {item["vehicle_id"]: item for item in config["initial_vehicles"]}
    for mv_id in RM_ONESTEP_S07_2MV_MV_IDS:
        assert mv_id in vehicles
        assert vehicles[mv_id]["road_role"] == "on_ramp_mv"
    assert sum(1 for item in vehicles.values() if item["physical_lane"] == "lane_2") == 7


def test_random_flow_scene_module_overrides_target_boundary_generation_only() -> None:
    config = load_traffic_flow_scene_config(
        RM_ONESTEP_RANDOM_S07_LANE2_RAMP_100S_SCENARIO_ID
    )
    overrides = config["module_overrides"]

    assert overrides["boundary_generation_enabled"] is True
    assert overrides["random_arrival_enabled"] is True
    assert overrides["random_vehicle_attributes_enabled"] is True
    assert overrides["ordinary_mainline_lane_change_enabled"] is False
    assert overrides["platoon_cmc_enabled"] is False
    assert overrides["mpc_lateral_tracking_enabled"] is False


def test_random_flow_scene_source_builds_lane2_and_on_ramp_queue() -> None:
    spec = get_traffic_flow_scene_spec(
        RM_ONESTEP_RANDOM_S07_LANE2_RAMP_100S_SCENARIO_ID
    )

    queue = spec.boundary_flow_source.build_queue(100.0)
    lanes = {item.lane_id for item in queue}

    assert {"lane_2", "on_ramp"} <= lanes


def test_static_onestep_scene_ids_still_use_static_loader() -> None:
    assert (
        RM_ONESTEP_RANDOM_S07_LANE2_RAMP_100S_SCENARIO_ID
        not in RM_ONESTEP_SCENARIO_IDS
    )
    for scenario_id in RM_ONESTEP_SCENARIO_IDS:
        config = load_scene_config(scenario_id)
        assert config["scenario_id"] == scenario_id
