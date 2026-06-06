from __future__ import annotations

import pytest

from cormc.sumo.mapping import EDGE_METADATA, LANE_ROLE_MAP, from_sumo_position, to_sumo_position, to_sumo_xy


def test_p17_edge_metadata_and_lane_role_map_are_fixed() -> None:
    assert EDGE_METADATA["main_pre"].start_x == 0
    assert EDGE_METADATA["main_pre"].end_x == 6950
    assert EDGE_METADATA["main_pre"].lane_count == 2
    assert EDGE_METADATA["merge_zone"].lane_count == 3
    assert EDGE_METADATA["main_post"].end_x == 10000
    assert EDGE_METADATA["ramp_upstream"].start_x == 6450
    assert EDGE_METADATA["ramp_upstream"].end_x == 6650
    assert EDGE_METADATA["ramp_pre"].start_x == 6650

    assert [(lane.lane_index, lane.role, lane.y) for lane in LANE_ROLE_MAP["merge_zone"]] == [
        (0, "on_ramp", -3.5),
        (1, "lane_2", 0.0),
        (2, "lane_1", 3.5),
    ]


@pytest.mark.parametrize(
    ("x_global", "physical_lane", "road_role", "expected"),
    [
        (0.0, "lane_2", "mainline", ("main_pre", 0, 0.0)),
        (6949.9, "lane_1", "mainline", ("main_pre", 1, 6949.9)),
        (6950.0, "lane_2", "mainline", ("merge_zone", 1, 0.0)),
        (7249.9, "on_ramp", "on_ramp", ("merge_zone", 0, 299.9)),
        (6950.0, "on_ramp", "on_ramp_mv", ("merge_zone", 0, 0.0)),
        (7250.0, "lane_1", "mainline", ("main_post", 1, 0.0)),
        (10000.0, "lane_2", "mainline", ("main_post", 0, 2750.0)),
        (6450.0, "on_ramp", "on_ramp", ("ramp_upstream", 0, 0.0)),
        (6642.04, "on_ramp", "on_ramp_mv", ("ramp_upstream", 0, 192.04)),
        (6650.0, "on_ramp", "on_ramp", ("ramp_pre", 0, 0.0)),
        (6949.9, "on_ramp", "on_ramp", ("ramp_pre", 0, 299.9)),
    ],
)
def test_p17_to_sumo_position_boundaries(
    x_global: float, physical_lane: str, road_role: str, expected: tuple[str, int, float]
) -> None:
    edge_id, lane_index, pos = to_sumo_position(x_global, physical_lane, road_role)

    assert (edge_id, lane_index) == expected[:2]
    assert pos == pytest.approx(expected[2])


def test_p17_from_sumo_position_round_trips_x_global() -> None:
    for x_global, physical_lane, road_role in (
        (10.0, "lane_2", "mainline"),
        (6642.04, "on_ramp", "on_ramp"),
        (7000.0, "lane_1", "mainline"),
        (6900.0, "on_ramp", "on_ramp"),
        (7300.0, "lane_2", "mainline"),
    ):
        edge_id, _, pos = to_sumo_position(x_global, physical_lane, road_role)
        assert from_sumo_position(edge_id, pos) == pytest.approx(x_global)


def test_p17_to_sumo_xy_uses_cormc_coordinates_directly() -> None:
    assert to_sumo_xy(7000.0, -1.25, "on_ramp") == (7000.0, -1.25)


def test_p17_mapping_errors_are_clear() -> None:
    with pytest.raises(ValueError, match="outside supported P17 range"):
        to_sumo_position(10000.1, "lane_2", "mainline")
    with pytest.raises(ValueError, match="outside ramp range"):
        to_sumo_position(6449.9, "on_ramp", "on_ramp")
    with pytest.raises(ValueError, match="invalid on edge"):
        to_sumo_position(10.0, "on_ramp", "mainline")
    with pytest.raises(ValueError, match="Unknown SUMO edge_id"):
        from_sumo_position("missing", 0.0)
