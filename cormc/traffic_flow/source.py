from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, Protocol

if TYPE_CHECKING:
    from cormc.traffic_flow.generation import BoundaryQueueItem, SeededRandomProfile


class BoundaryFlowSource(Protocol):
    source_id: str
    source_type: str

    def build_queue(self, horizon_s: float) -> tuple[BoundaryQueueItem, ...]:
        ...

    def to_summary(self) -> dict[str, Any]:
        ...


@dataclass(frozen=True)
class SeededRandomBoundaryFlowSource:
    source_id: str
    profile: SeededRandomProfile
    source_type: str = "seeded_random"

    def build_queue(self, horizon_s: float) -> tuple[BoundaryQueueItem, ...]:
        from cormc.traffic_flow.generation import generate_boundary_queue

        return generate_boundary_queue(
            self.profile,
            max_t=float(horizon_s),
            start_t=0.0,
            start_step=0,
        )

    def to_summary(self) -> dict[str, Any]:
        from cormc.traffic_flow.generation import profile_to_dict

        profile = profile_to_dict(self.profile)
        return {
            "source_id": self.source_id,
            "source_type": self.source_type,
            "seed": self.profile.seed,
            "profile_id": self.profile.profile_id,
            "arrival_streams": [
                _arrival_stream_summary(stream)
                for stream in self.profile.arrival_streams
            ],
            "safe_spawn_gap_m": self.profile.safe_spawn_gap_m,
            "cav_penetration_rate": self.profile.cav_penetration_rate,
            "chv_compliance_rate": self.profile.chv_compliance_rate,
            "desired_speed_mean": self.profile.desired_speed_mean,
            "desired_speed_std": self.profile.desired_speed_std,
            "profile": profile,
        }


def _arrival_stream_summary(stream: Any) -> dict[str, Any]:
    return {
        "lane_id": stream.lane_id,
        "road_role": stream.road_role,
        "merge_state": stream.merge_state,
        "spawn_x": stream.spawn_x,
        "spawn_y": stream.spawn_y,
        "initial_speed": stream.initial_speed,
        "shifted_headway": stream.shifted_headway,
        "mean_headway": stream.mean_headway,
        "flow_policy": stream.flow_policy,
        "vehicle_id_prefix": stream.vehicle_id_prefix,
        "vehicle_id_lane_label": stream.vehicle_id_lane_label,
    }
