from __future__ import annotations

from pathlib import Path
from xml.etree import ElementTree as ET

import pytest

from cormc.sumo.env import ensure_sumo_available_or_skip
from cormc.sumo.mapping import to_sumo_position
from cormc.sumo.mvs_replay_artifacts import EXPECTED_LANE_CENTERLINES, check_lane_centerlines
from cormc.sumo.network import build_p17_sumo_network


def test_p17_1_network_lane_centerlines_match_cormc_geometry(tmp_path: Path) -> None:
    ensure_sumo_available_or_skip()
    files = build_p17_sumo_network(tmp_path)

    check = check_lane_centerlines(files.net_file)

    assert check["status"] == "passed"
    for lane_id, expected_y in EXPECTED_LANE_CENTERLINES.items():
        assert check["observed_centerlines"][lane_id] == pytest.approx(expected_y, abs=0.05)

    edges = {edge.attrib["id"]: edge.attrib for edge in ET.parse(files.edges_file).getroot()}
    assert edges["ramp_upstream"]["shape"].startswith("6450.000,-3.500")
    assert edges["ramp_upstream"]["shape"].endswith("6650.000,-3.500")
    assert edges["ramp_upstream"]["spreadType"] == "center"
    assert edges["ramp_pre"]["shape"].startswith("6650.000,-3.500")
    assert edges["ramp_pre"]["shape"].endswith("6950.000,-3.500")
    assert edges["ramp_pre"]["spreadType"] == "center"


def test_p17_1_sumo_position_mapping_boundaries() -> None:
    edge_id, lane_index, pos = to_sumo_position(6642.04, "on_ramp", "on_ramp_mv")
    assert (edge_id, lane_index) == ("ramp_upstream", 0)
    assert pos == pytest.approx(192.04)
    assert to_sumo_position(6650.0, "on_ramp", "on_ramp") == ("ramp_pre", 0, 0.0)
    assert to_sumo_position(6950.0, "on_ramp", "on_ramp") == ("merge_zone", 0, 0.0)
    assert to_sumo_position(7250.0, "lane_2", "mainline") == ("main_post", 0, 0.0)
