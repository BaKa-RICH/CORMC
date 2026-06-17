from __future__ import annotations

from dataclasses import dataclass


MAINLINE_START_X = 0.0
MERGE_START_X = 6950.0
MERGE_END_X = 7250.0
MAINLINE_END_X = 10000.0
RAMP_UPSTREAM_START_X = 6450.0
RAMP_START_X = 6650.0
LANE_WIDTH = 3.5


@dataclass(frozen=True)
class EdgeMetadata:
    edge_id: str
    start_x: float
    end_x: float
    lane_count: int
    road_roles: tuple[str, ...]


@dataclass(frozen=True)
class LaneRole:
    edge_id: str
    lane_index: int
    role: str
    y: float

    @property
    def lane_id(self) -> str:
        return f"{self.edge_id}_{self.lane_index}"


EDGE_METADATA: dict[str, EdgeMetadata] = {
    "main_pre": EdgeMetadata("main_pre", MAINLINE_START_X, MERGE_START_X, 2, ("mainline",)),
    "merge_zone": EdgeMetadata("merge_zone", MERGE_START_X, MERGE_END_X, 3, ("mainline", "on_ramp")),
    "main_post": EdgeMetadata("main_post", MERGE_END_X, MAINLINE_END_X, 2, ("mainline",)),
    "ramp_upstream": EdgeMetadata("ramp_upstream", RAMP_UPSTREAM_START_X, RAMP_START_X, 1, ("on_ramp",)),
    "ramp_pre": EdgeMetadata("ramp_pre", RAMP_START_X, MERGE_START_X, 1, ("on_ramp",)),
}

LANE_ROLE_MAP: dict[str, tuple[LaneRole, ...]] = {
    "main_pre": (
        LaneRole("main_pre", 0, "lane_2", 0.0),
        LaneRole("main_pre", 1, "lane_1", LANE_WIDTH),
    ),
    "merge_zone": (
        LaneRole("merge_zone", 0, "on_ramp", -LANE_WIDTH),
        LaneRole("merge_zone", 1, "lane_2", 0.0),
        LaneRole("merge_zone", 2, "lane_1", LANE_WIDTH),
    ),
    "main_post": (
        LaneRole("main_post", 0, "lane_2", 0.0),
        LaneRole("main_post", 1, "lane_1", LANE_WIDTH),
    ),
    "ramp_upstream": (LaneRole("ramp_upstream", 0, "on_ramp", -LANE_WIDTH),),
    "ramp_pre": (LaneRole("ramp_pre", 0, "on_ramp", -LANE_WIDTH),),
}


def to_sumo_position(x_global: float, physical_lane: str, road_role: str = "mainline") -> tuple[str, int, float]:
    edge_id = _edge_for_x_and_role(x_global, _normalized_road_role(road_role))
    lane_index = lane_index_for_role(edge_id, physical_lane)
    edge = EDGE_METADATA[edge_id]
    pos = x_global - edge.start_x
    if pos < -1e-9 or pos > edge.end_x - edge.start_x + 1e-9:
        raise ValueError(
            f"x_global={x_global} maps outside edge {edge_id} range "
            f"[{edge.start_x}, {edge.end_x}] for road_role={road_role!r}"
        )
    return edge_id, lane_index, pos


def from_sumo_position(edge_id: str, lane_position: float) -> float:
    edge = EDGE_METADATA.get(edge_id)
    if edge is None:
        raise ValueError(f"Unknown SUMO edge_id={edge_id!r}; expected one of {sorted(EDGE_METADATA)}")
    length = edge.end_x - edge.start_x
    if lane_position < -1e-9 or lane_position > length + 1e-9:
        raise ValueError(f"lane_position={lane_position} is outside edge {edge_id!r} length [0, {length}]")
    return edge.start_x + lane_position


def to_sumo_xy(x_global: float, y: float, road_role: str = "mainline") -> tuple[float, float]:
    _edge_for_x_and_role(x_global, _normalized_road_role(road_role))
    return x_global, y


def lane_index_for_role(edge_id: str, physical_lane: str) -> int:
    for lane in LANE_ROLE_MAP.get(edge_id, ()):
        if lane.role == physical_lane:
            return lane.lane_index
    roles = [lane.role for lane in LANE_ROLE_MAP.get(edge_id, ())]
    raise ValueError(f"Lane role {physical_lane!r} is invalid on edge {edge_id!r}; expected one of {roles}")


def _edge_for_x_and_role(x_global: float, road_role: str) -> str:
    if road_role == "on_ramp" and x_global < MERGE_START_X:
        if RAMP_UPSTREAM_START_X <= x_global < RAMP_START_X:
            return "ramp_upstream"
        if RAMP_START_X <= x_global < MERGE_START_X:
            return "ramp_pre"
        raise ValueError(
            f"on_ramp x_global={x_global} is outside ramp range "
            f"[{RAMP_UPSTREAM_START_X}, {MERGE_START_X})"
        )
    if road_role == "on_ramp" and MERGE_START_X <= x_global < MERGE_END_X:
        return "merge_zone"
    if road_role not in {"mainline", "on_ramp"}:
        raise ValueError("road_role must be 'mainline' or 'on_ramp'")
    if MAINLINE_START_X <= x_global < MERGE_START_X:
        return "main_pre"
    if MERGE_START_X <= x_global < MERGE_END_X:
        return "merge_zone"
    if MERGE_END_X <= x_global <= MAINLINE_END_X:
        return "main_post"
    raise ValueError(f"x_global={x_global} is outside supported P17 range [{MAINLINE_START_X}, {MAINLINE_END_X}]")


def _normalized_road_role(road_role: str) -> str:
    if road_role == "on_ramp_mv":
        return "on_ramp"
    return road_role
