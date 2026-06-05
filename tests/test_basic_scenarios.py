from __future__ import annotations

from cormc.basic_scenarios import (
    BASIC_SCENARIO_IDS,
    get_basic_expectation,
    load_basic_scenario,
)
from cormc.mvs.runner import DETERMINISTIC_SCENARIO_ROUTES


def test_basic_scenario_ids_are_independent_from_required_mvs_routes() -> None:
    assert BASIC_SCENARIO_IDS == (
        "BASIC-01",
        "BASIC-02",
        "BASIC-03",
        "BASIC-04",
        "BASIC-05",
        "BASIC-06",
    )
    assert not set(BASIC_SCENARIO_IDS).intersection(DETERMINISTIC_SCENARIO_ROUTES)


def test_basic_scenario_vehicle_tables_match_document() -> None:
    expected = {
        "BASIC-01": [
            ("B01_MV", "on_ramp", "on_ramp_mv", 6640.0, -3.5, 20.0),
            ("B01_CLV", "lane_2", "mainline", 6674.0, 0.0, 20.0),
            ("B01_CFV", "lane_2", "mainline", 6634.0, 0.0, 20.0),
            ("B01_TLV_CFV", "lane_1", "mainline", 6643.0, 3.5, 15.0),
        ],
        "BASIC-02": [
            ("B02_MV", "on_ramp", "on_ramp_mv", 6640.0, -3.5, 20.0),
            ("B02_CLV", "lane_2", "mainline", 6654.0, 0.0, 20.0),
            ("B02_CFV", "lane_2", "mainline", 6614.0, 0.0, 20.0),
            ("B02_TLV_CLV", "lane_1", "mainline", 6663.0, 3.5, 15.0),
        ],
        "BASIC-03": [
            ("B03_MV", "on_ramp", "on_ramp_mv", 6640.0, -3.5, 20.0),
            ("B03_CLV", "lane_2", "mainline", 6654.0, 0.0, 20.0),
            ("B03_CFV", "lane_2", "mainline", 6634.0, 0.0, 20.0),
            ("B03_TLV_CFV", "lane_1", "mainline", 6643.0, 3.5, 15.0),
            ("B03_TLV_CLV", "lane_1", "mainline", 6663.0, 3.5, 15.0),
        ],
        "BASIC-04": [
            ("B04_MV", "on_ramp", "on_ramp_mv", 6850.0, -3.5, 20.0),
            ("B04_CLV", "lane_2", "mainline", 6884.0, 0.0, 20.0),
            ("B04_CFV", "lane_2", "mainline", 6844.0, 0.0, 20.0),
            ("B04_TLV_CFV", "lane_1", "mainline", 6853.0, 3.5, 15.0),
        ],
        "BASIC-05": [
            ("B05_MV", "on_ramp", "on_ramp_mv", 6850.0, -3.5, 20.0),
            ("B05_CLV", "lane_2", "mainline", 6864.0, 0.0, 20.0),
            ("B05_CFV", "lane_2", "mainline", 6824.0, 0.0, 20.0),
            ("B05_TLV_CLV", "lane_1", "mainline", 6873.0, 3.5, 15.0),
        ],
        "BASIC-06": [
            ("B06_MV", "on_ramp", "on_ramp_mv", 6850.0, -3.5, 20.0),
            ("B06_CLV", "lane_2", "mainline", 6864.0, 0.0, 20.0),
            ("B06_CFV", "lane_2", "mainline", 6844.0, 0.0, 20.0),
            ("B06_TLV_CFV", "lane_1", "mainline", 6853.0, 3.5, 15.0),
            ("B06_TLV_CLV", "lane_1", "mainline", 6873.0, 3.5, 15.0),
        ],
    }

    for scenario_id in BASIC_SCENARIO_IDS:
        config = load_basic_scenario(scenario_id)
        actual = [
            (
                vehicle["vehicle_id"],
                vehicle["physical_lane"],
                vehicle["road_role"],
                vehicle["initial_x_global"],
                vehicle["initial_y"],
                vehicle["initial_v"],
            )
            for vehicle in config["initial_vehicles"]
        ]
        assert actual == expected[scenario_id]
        assert all(vehicle["vehicle_type"] == "CAV" for vehicle in config["initial_vehicles"])
        assert all(
            vehicle["compliance_state"] == "not_applicable"
            for vehicle in config["initial_vehicles"]
        )


def test_basic_expectations_match_case_and_eq10_matrix() -> None:
    matrix = {
        "BASIC-01": ("case_2", ("B01_CFV",), ("B01_CFV",)),
        "BASIC-02": ("case_3", ("B02_CLV",), ()),
        "BASIC-03": ("case_4", ("B03_CLV", "B03_CFV"), ("B03_CFV",)),
        "BASIC-04": ("case_2", ("B04_CFV",), ("B04_CFV",)),
        "BASIC-05": ("case_3", ("B05_CLV",), ()),
        "BASIC-06": ("case_4", ("B06_CLV", "B06_CFV"), ("B06_CFV",)),
    }

    for scenario_id, expected in matrix.items():
        expectation = get_basic_expectation(scenario_id)
        assert (
            expectation.expected_aps_case,
            expectation.expected_active_cv_ids,
            expectation.expected_eq10_consumer_ids,
        ) == expected
