from __future__ import annotations

import subprocess
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any
from xml.etree import ElementTree as ET

from cormc.sumo.env import ensure_sumo_tools_on_path


@dataclass(frozen=True)
class SumoNetworkConfig:
    step_length: float = 0.1
    lateral_resolution: float = 0.25
    collision_action: str = "warn"
    speed_limit_mps: float = 33.33
    lane_width: float = 3.5
    begin: float = 0.0
    end: float = 1.0

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class SumoNetworkFiles:
    output_dir: str
    nodes_file: str
    edges_file: str
    connections_file: str
    routes_file: str
    net_file: str
    sumocfg_file: str
    netconvert_log: str

    def to_dict(self) -> dict[str, str]:
        return asdict(self)


def build_sumo_network(output_dir: str | Path, config: SumoNetworkConfig | None = None) -> SumoNetworkFiles:
    cfg = config or SumoNetworkConfig()
    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)

    nodes_file = out / "p17.nod.xml"
    edges_file = out / "p17.edg.xml"
    connections_file = out / "p17.con.xml"
    routes_file = out / "p17.rou.xml"
    net_file = out / "p17.net.xml"
    sumocfg_file = out / "p17.sumocfg"

    _write_xml(nodes_file, _nodes_xml())
    _write_xml(edges_file, _edges_xml(cfg))
    _write_xml(connections_file, _connections_xml())

    paths = ensure_sumo_tools_on_path()
    completed = subprocess.run(
        [
            paths.netconvert,
            "--node-files",
            str(nodes_file),
            "--edge-files",
            str(edges_file),
            "--connection-files",
            str(connections_file),
            "--output-file",
            str(net_file),
            "--no-turnarounds",
            "true",
            "--offset.disable-normalization",
            "true",
        ],
        check=True,
        capture_output=True,
        encoding="utf-8",
    )

    _write_xml(routes_file, _routes_xml())
    _write_xml(sumocfg_file, _sumocfg_xml(cfg, net_file.name, routes_file.name))

    return SumoNetworkFiles(
        output_dir=str(out),
        nodes_file=str(nodes_file),
        edges_file=str(edges_file),
        connections_file=str(connections_file),
        routes_file=str(routes_file),
        net_file=str(net_file),
        sumocfg_file=str(sumocfg_file),
        netconvert_log=(completed.stdout + completed.stderr).strip(),
    )


def _nodes_xml() -> ET.Element:
    nodes = ET.Element("nodes")
    for node_id, x, y in (
        ("main_start", 0.0, 0.0),
        ("merge_start", 6950.0, 0.0),
        ("merge_end", 7250.0, 0.0),
        ("main_end", 10000.0, 0.0),
        ("ramp_upstream_start", 6450.0, -3.5),
        ("ramp_start", 6650.0, -3.5),
    ):
        ET.SubElement(nodes, "node", id=node_id, x=f"{x:.3f}", y=f"{y:.3f}", type="priority")
    return nodes


def _edges_xml(config: SumoNetworkConfig) -> ET.Element:
    edges = ET.Element("edges")
    edge_specs = (
        ("main_pre", "main_start", "merge_start", 2, "0.000,1.750 6950.000,1.750"),
        ("merge_zone", "merge_start", "merge_end", 3, "6950.000,0.000 7250.000,0.000"),
        ("main_post", "merge_end", "main_end", 2, "7250.000,1.750 10000.000,1.750"),
        ("ramp_upstream", "ramp_upstream_start", "ramp_start", 1, "6450.000,-3.500 6650.000,-3.500"),
        ("ramp_pre", "ramp_start", "merge_start", 1, "6650.000,-3.500 6950.000,-3.500"),
    )
    for edge_id, from_node, to_node, lanes, shape in edge_specs:
        ET.SubElement(
            edges,
            "edge",
            id=edge_id,
            **{"from": from_node, "to": to_node},
            numLanes=str(lanes),
            speed=f"{config.speed_limit_mps:.2f}",
            width=f"{config.lane_width:.2f}",
            shape=shape,
            spreadType="center",
        )
    return edges


def _connections_xml() -> ET.Element:
    connections = ET.Element("connections")
    connection_specs = (
        ("main_pre", "merge_zone", 0, 1),
        ("main_pre", "merge_zone", 1, 2),
        ("ramp_upstream", "ramp_pre", 0, 0),
        ("ramp_pre", "merge_zone", 0, 0),
        ("merge_zone", "main_post", 1, 0),
        ("merge_zone", "main_post", 2, 1),
        ("merge_zone", "main_post", 0, 0),
    )
    for from_edge, to_edge, from_lane, to_lane in connection_specs:
        ET.SubElement(
            connections,
            "connection",
            **{"from": from_edge, "to": to_edge},
            fromLane=str(from_lane),
            toLane=str(to_lane),
        )
    return connections


def _routes_xml() -> ET.Element:
    routes = ET.Element("routes")
    ET.SubElement(
        routes,
        "vType",
        id="cormc_active",
        length="5.0",
        width="1.8",
        minGap="2.5",
        accel="2.6",
        decel="4.5",
        sigma="0.0",
        color="0,90,220",
        latAlignment="center",
    )
    ET.SubElement(
        routes,
        "vType",
        id="sumo_background",
        length="5.0",
        width="1.8",
        minGap="2.5",
        accel="2.0",
        decel="4.5",
        tau="1.0",
        sigma="0.5",
        carFollowModel="IDM",
        laneChangeModel="SL2015",
        color="160,160,160",
        latAlignment="center",
    )
    ET.SubElement(routes, "route", id="route_main", edges="main_pre merge_zone main_post")
    ET.SubElement(routes, "route", id="route_ramp", edges="ramp_upstream ramp_pre merge_zone main_post")
    return routes


def _sumocfg_xml(config: SumoNetworkConfig, net_file: str, routes_file: str) -> ET.Element:
    root = ET.Element("configuration")
    input_node = ET.SubElement(root, "input")
    ET.SubElement(input_node, "net-file", value=net_file)
    ET.SubElement(input_node, "route-files", value=routes_file)

    time_node = ET.SubElement(root, "time")
    ET.SubElement(time_node, "begin", value=f"{config.begin:.1f}")
    ET.SubElement(time_node, "end", value=f"{config.end:.1f}")
    ET.SubElement(time_node, "step-length", value=f"{config.step_length:.3f}".rstrip("0").rstrip("."))

    processing_node = ET.SubElement(root, "processing")
    ET.SubElement(processing_node, "lateral-resolution", value=f"{config.lateral_resolution:.3f}".rstrip("0").rstrip("."))
    ET.SubElement(processing_node, "collision.action", value=config.collision_action)
    ET.SubElement(processing_node, "collision.check-junctions", value="true")
    return root


def _write_xml(path: Path, root: ET.Element) -> None:
    tree = ET.ElementTree(root)
    ET.indent(tree, space="  ")
    tree.write(path, encoding="utf-8", xml_declaration=True)
