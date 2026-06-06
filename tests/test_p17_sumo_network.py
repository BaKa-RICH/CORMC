from __future__ import annotations

import subprocess
from pathlib import Path
from xml.etree import ElementTree as ET

from cormc.sumo.env import ensure_sumo_available_or_skip
from cormc.sumo.network import build_p17_sumo_network


def test_p17_builds_plainxml_net_routes_and_sumocfg(tmp_path: Path) -> None:
    ensure_sumo_available_or_skip()

    files = build_p17_sumo_network(tmp_path)

    for path in (
        files.nodes_file,
        files.edges_file,
        files.connections_file,
        files.routes_file,
        files.net_file,
        files.sumocfg_file,
    ):
        assert Path(path).exists()
        assert Path(path).stat().st_size > 0
    assert Path(files.nodes_file).name == "p17.nod.xml"
    assert Path(files.edges_file).name == "p17.edg.xml"
    assert Path(files.connections_file).name == "p17.con.xml"
    assert Path(files.routes_file).name == "p17.rou.xml"

    edges = {edge.attrib["id"]: edge.attrib for edge in ET.parse(files.edges_file).getroot()}
    assert edges["main_pre"]["numLanes"] == "2"
    assert edges["merge_zone"]["numLanes"] == "3"
    assert edges["main_post"]["numLanes"] == "2"
    assert edges["ramp_upstream"]["numLanes"] == "1"
    assert edges["ramp_pre"]["numLanes"] == "1"


def test_p17_routes_and_sumocfg_have_required_defaults(tmp_path: Path) -> None:
    ensure_sumo_available_or_skip()

    files = build_p17_sumo_network(tmp_path)
    routes = ET.parse(files.routes_file).getroot()
    sumocfg = ET.parse(files.sumocfg_file).getroot()

    route_by_id = {route.attrib["id"]: route.attrib["edges"] for route in routes.findall("route")}
    assert route_by_id == {
        "route_main": "main_pre merge_zone main_post",
        "route_ramp": "ramp_upstream ramp_pre merge_zone main_post",
    }

    vtypes = {vtype.attrib["id"]: vtype.attrib for vtype in routes.findall("vType")}
    assert vtypes["cormc_active"]["latAlignment"] == "center"
    assert vtypes["sumo_background"]["latAlignment"] == "center"
    assert vtypes["sumo_background"]["carFollowModel"] == "IDM"

    values = {child.tag: child.attrib["value"] for section in sumocfg for child in section}
    assert values["step-length"] == "0.1"
    assert values["lateral-resolution"] == "0.25"
    assert values["collision.action"] == "warn"


def test_p17_sumo_config_smoke_runs_real_sumo(tmp_path: Path) -> None:
    paths = ensure_sumo_available_or_skip()
    files = build_p17_sumo_network(tmp_path)

    completed = subprocess.run(
        [paths.sumo, "-c", files.sumocfg_file],
        cwd=tmp_path,
        check=True,
        capture_output=True,
        encoding="utf-8",
    )

    assert completed.returncode == 0
